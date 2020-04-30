#include <linux/module.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/anon_inodes.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/file.h>
#include <linux/kref.h>
#include <linux/interrupt.h>

#include "uharddoom.h"
#include "udoomdev.h"
#include "udoomfw.h"

#define UHARDDOOM_MAX_DEVICES 256

#define BATCH_BUF_SIZE 0x100

MODULE_AUTHOR("Marcelina Kościelnicka");
MODULE_LICENSE("GPL");

#if 0
struct uharddoom_fence {
	struct list_head lh;
	wait_queue_head_t wq;
	uint32_t counter;
	int active;
};

struct uharddoom_device {
	struct pci_dev *pdev;
	struct cdev cdev;
	int idx;
	struct device *dev;
	void __iomem *bar;
	spinlock_t slock;
	struct mutex submit_lock;
	struct mutex last_lock;
	struct uharddoom_buffer *last_bufs[7];
#ifdef USE_CMD_FETCH
	struct uharddoom_buffer *cmd_buf;
	int cmd_write_idx;
#endif
	int cmd_dirty;
	int cmds_to_ping;
	uint32_t fence_wait;
	uint32_t last_recv_fence;
	uint32_t last_sent_fence;
	int free_count;
	wait_queue_head_t free_wq;
	int wait_for_free;
	struct list_head fence_list;
	uint32_t cmd_pending[8];
};

struct uharddoom_context {
	struct mutex lock;
	struct uharddoom_device *dev;
	struct uharddoom_buffer *bufs[7];
};

struct uharddoom_buffer {
	struct uharddoom_device *dev;
	/* width == 0 means it's not a surface.  */
	uint32_t size;
	uint16_t width;
	uint16_t height;
	__le32 *pt_cpu;
	dma_addr_t pt_dma;
	struct uharddoom_page *pages;
	uint32_t pnum;
	struct mutex lock;
	struct uharddoom_fence read_fence;
	struct uharddoom_fence free_fence;
	int dirty;
	struct kref kref;
};

struct uharddoom_page {
	void *page_cpu;
	dma_addr_t page_dma;
};

static dev_t uharddoom_devno;
static struct uharddoom_device *uharddoom_devices[UHARDDOOM_MAX_DEVICES];
static DEFINE_MUTEX(uharddoom_devices_lock);
static struct class uharddoom_class = {
	.name = "uharddoom",
	.owner = THIS_MODULE,
};

/* Hardware handling. */

static inline void uharddoom_iow(struct uharddoom_device *dev, uint32_t reg, uint32_t val)
{
	iowrite32(val, dev->bar + reg);
	//printk(KERN_ALERT "uharddoom %03x <- %08x\n", reg, val);
}

static inline uint32_t uharddoom_ior(struct uharddoom_device *dev, uint32_t reg)
{
	uint32_t res = ioread32(dev->bar + reg);
	//printk(KERN_ALERT "uharddoom %03x -> %08x\n", reg, res);
	return res;
}

static void uharddoom_process_fences(struct uharddoom_device *dev)
{
	struct list_head *lh, *tmp;
	uint32_t last_fence = dev->last_recv_fence;
	uint32_t cur_fence = uharddoom_ior(dev, UHARDDOOM_FENCE_COUNTER);
	if (cur_fence == last_fence)
		return;
	list_for_each_safe(lh, tmp, &dev->fence_list) {
		struct uharddoom_fence *fence = list_entry(lh, struct uharddoom_fence, lh);
		if (last_fence < cur_fence) {
			if (fence->counter <= last_fence || fence->counter > cur_fence)
				break;
		} else {
			if (fence->counter <= last_fence && fence->counter > cur_fence)
				break;
		}
		fence->active = 0;
		list_del(lh);
		wake_up(&fence->wq);
	}
	dev->last_recv_fence = cur_fence;
}

static void uharddoom_submit_raw(struct uharddoom_device *dev, uint32_t *cmd)
{
	int i;
#ifdef USE_CMD_FETCH
	int pidx = dev->cmd_write_idx / CMDS_PER_PAGE;
	int cidx = dev->cmd_write_idx % CMDS_PER_PAGE;
	struct uharddoom_page *page = &dev->cmd_buf->pages[pidx];
	uint32_t *data = page->page_cpu;
	for (i = 0; i < UHARDDOOM_CMD_SEND_SIZE; i++) {
		data[cidx * 8 + i] =  __cpu_to_le32(cmd[i]);
	}
	dev->cmd_write_idx++;
	if (dev->cmd_write_idx == COMMAND_BUF_SIZE)
		dev->cmd_write_idx = 0;
#else
	for (i = 0; i < UHARDDOOM_CMD_SEND_SIZE; i++) {
		uharddoom_iow(dev, UHARDDOOM_CMD_SEND(i), cmds[i]);
	}
#endif
	dev->free_count--;
}

static void uharddoom_fence_cmd(struct uharddoom_device *dev)
{
	if (!dev->cmd_dirty)
		return;
	dev->last_sent_fence++;
	dev->cmd_pending[0] |= UHARDDOOM_CMD_FLAG_FENCE;
	uharddoom_submit_raw(dev, dev->cmd_pending);
#ifdef USE_CMD_FETCH
	uharddoom_iow(dev, UHARDDOOM_CMD_WRITE_IDX, dev->cmd_write_idx);
#endif
	dev->cmd_dirty = 0;
}

static void uharddoom_fence_init(struct uharddoom_fence *fence)
{
	fence->active = 0;
	init_waitqueue_head(&fence->wq);
}

static void uharddoom_fence_set_waiting(struct uharddoom_device *dev, struct uharddoom_fence *fence)
{
	uint32_t cur, mine;
	cur = dev->fence_wait - dev->last_recv_fence - 1;
	mine = fence->counter - dev->last_recv_fence - 1;
	if (mine < cur) {
		dev->fence_wait = fence->counter;
		uharddoom_iow(dev, UHARDDOOM_FENCE_WAIT, fence->counter);
		uharddoom_process_fences(dev);
	}
}

static void uharddoom_fence_wait(struct uharddoom_device *dev, struct uharddoom_fence *fence)
{
	unsigned long flags;
	spin_lock_irqsave(&dev->slock, flags);
	while (fence->active) {
		uharddoom_fence_set_waiting(dev, fence);
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_event(fence->wq, !fence->active);
		spin_lock_irqsave(&dev->slock, flags);
	}
	spin_unlock_irqrestore(&dev->slock, flags);
}

static int uharddoom_fence_wait_interruptible(struct uharddoom_device *dev, struct uharddoom_fence *fence)
{
	int res;
	unsigned long flags;
	spin_lock_irqsave(&dev->slock, flags);
	while (fence->active) {
		uharddoom_fence_set_waiting(dev, fence);
		spin_unlock_irqrestore(&dev->slock, flags);
		if ((res = wait_event_interruptible(fence->wq, !fence->active)))
			return res;
		spin_lock_irqsave(&dev->slock, flags);
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	return 0;
}

static void uharddoom_fence_set(struct uharddoom_device *dev, struct uharddoom_fence *fence)
{
	unsigned long flags;
	uharddoom_fence_cmd(dev);
	spin_lock_irqsave(&dev->slock, flags);
	if (fence->active)
		list_del(&fence->lh);
	fence->counter = dev->last_sent_fence;
	fence->active = 1;
	list_add_tail(&fence->lh, &dev->fence_list);
	spin_unlock_irqrestore(&dev->slock, flags);
}

static void uharddoom_update_free_count(struct uharddoom_device *dev)
{
#ifdef USE_CMD_FETCH
	int read_idx = uharddoom_ior(dev, UHARDDOOM_CMD_READ_IDX);
	if (read_idx <= dev->cmd_write_idx)
		dev->free_count = COMMAND_BUF_SIZE + read_idx - dev->cmd_write_idx - 1;
	else
		dev->free_count = read_idx - dev->cmd_write_idx - 1;
#else
	dev->free_count = uharddoom_ior(dev, UHARDDOOM_FIFO_FREE);
#endif
}

static int uharddoom_wait_free(struct uharddoom_device *dev, int num)
{
	int res;
	unsigned long flags;
	if (num <= dev->free_count)
		return 0;
	spin_lock_irqsave(&dev->slock, flags);
	uharddoom_process_fences(dev);
	spin_unlock_irqrestore(&dev->slock, flags);
	uharddoom_update_free_count(dev);
	if (num <= dev->free_count)
		return 0;
	while (1) {
		spin_lock_irqsave(&dev->slock, flags);
		dev->wait_for_free = 1;
		uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, UHARDDOOM_INTR_MASK);
		spin_unlock_irqrestore(&dev->slock, flags);
		uharddoom_update_free_count(dev);
		if (num <= dev->free_count)
			return 0;
		if ((res = wait_event_interruptible(dev->free_wq, dev->wait_for_free == 0)))
			return res;
	}
}

static int uharddoom_submit(struct uharddoom_device *dev, uint32_t *cmd)
{
	int res;
	if ((res = uharddoom_wait_free(dev, 2)))
		return res;
	if (dev->cmd_dirty)
		uharddoom_submit_raw(dev, dev->cmd_pending);
	memcpy(dev->cmd_pending, cmd, 32);
	dev->cmd_dirty = 1;
	dev->cmds_to_ping--;
	if (!dev->cmds_to_ping) {
		dev->cmds_to_ping = UHARDDOOM_PING_INTERVAL;
		dev->cmd_pending[0] |= UHARDDOOM_CMD_FLAG_PING_ASYNC;
#ifdef USE_CMD_FETCH
		uharddoom_iow(dev, UHARDDOOM_CMD_WRITE_IDX, dev->cmd_write_idx);
#endif
	}
	return 0;
}

static irqreturn_t uharddoom_isr(int irq, void *opaque)
{
	struct uharddoom_device *dev = opaque;
	unsigned long flags;
	uint32_t istatus;
	spin_lock_irqsave(&dev->slock, flags);
	istatus = uharddoom_ior(dev, UHARDDOOM_INTR) & uharddoom_ior(dev, UHARDDOOM_INTR_ENABLE);
	if (istatus) {
		uharddoom_iow(dev, UHARDDOOM_INTR, istatus);
		if (istatus & UHARDDOOM_INTR_FENCE) {
			/* NOTIFY handling -- wake up someone waiting for the device.  */
			uharddoom_process_fences(dev);
		}
		if (istatus & UHARDDOOM_INTR_PONG_ASYNC) {
			dev->wait_for_free = 0;
			wake_up(&dev->free_wq);
			uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, UHARDDOOM_INTR_MASK & ~UHARDDOOM_INTR_PONG_ASYNC);
		}
		/* All other interrupts are nonrecoverable errors that should never happen.
		 * Just log them and leave the device in hung state.  */
		if (istatus & UHARDDOOM_INTR_FE_ERROR)
			printk(KERN_ALERT "uharddoom: got FE_ERROR error %03x\n",
					uharddoom_ior(dev, UHARDDOOM_FE_ERROR_CODE));
		if (istatus & UHARDDOOM_INTR_CMD_OVERFLOW)
			printk(KERN_ALERT "uharddoom: got CMD_OVERFLOW error\n");
		if (istatus & UHARDDOOM_INTR_SURF_SRC_OVERFLOW)
			printk(KERN_ALERT "uharddoom: got SURF_SRC_OVERFLOW error\n");
		if (istatus & UHARDDOOM_INTR_SURF_DST_OVERFLOW)
			printk(KERN_ALERT "uharddoom: got SURF_DST_OVERFLOW error\n");
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_CMD)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_CMD error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_CMD));
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_SURF_DST)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_SURF_DST error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_SURF_DST));
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_SURF_SRC)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_SURF_SRC error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_SURF_SRC));
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_TEXTURE)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_TEXTURE error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_TEXTURE));
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_FLAT)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_FLAT error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_FLAT));
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_TRANSLATION)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_TRANSLATION error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_TRANSLATION));
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_COLORMAP)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_COLORMAP error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_COLORMAP));
		if (istatus & UHARDDOOM_INTR_PAGE_FAULT_TRANMAP)
			printk(KERN_ALERT "uharddoom: got PAGE_FAULT_TRANMAP error [%08x]\n",
					uharddoom_ior(dev, UHARDDOOM_TLB_ENTRY_TRANMAP));
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	return IRQ_RETVAL(istatus);
}

/* Paged memory handling.  */

static struct uharddoom_buffer *uharddoom_buffer_create(struct uharddoom_device *dev, uint32_t size, uint16_t width, uint16_t height)
{
	int pti;
	struct uharddoom_buffer *buffer = kzalloc(sizeof *buffer, 1);
	if (!buffer)
		return 0;
	kref_init(&buffer->kref);
	buffer->dev = dev;
	buffer->size = size;
	buffer->width = width;
	buffer->height = height;
	buffer->pnum = (size + UHARDDOOM_PAGE_SIZE - 1) >> UHARDDOOM_PAGE_SHIFT;
	mutex_init(&buffer->lock);
	uharddoom_fence_init(&buffer->free_fence);
	uharddoom_fence_init(&buffer->read_fence);
	buffer->pages = kzalloc(sizeof *buffer->pages * buffer->pnum, GFP_KERNEL);
	if (!buffer->pages)
		goto out_pages;
	// XXX alignment?
	if (!(buffer->pt_cpu = dma_alloc_coherent(&dev->pdev->dev,
			buffer->pnum * 4,
			&buffer->pt_dma, GFP_KERNEL)))
		goto out_pt;
	for (pti = 0; pti < buffer->pnum; pti++) {
		if (!(buffer->pages[pti].page_cpu = dma_alloc_coherent(&dev->pdev->dev,
				UHARDDOOM_PAGE_SIZE,
				&buffer->pages[pti].page_dma, GFP_KERNEL | __GFP_ZERO)))
			goto out_page;
		buffer->pt_cpu[pti] = __cpu_to_le32(buffer->pages[pti].page_dma >> 8 | UHARDDOOM_PTE_VALID | UHARDDOOM_PTE_WRITABLE);
	}
	return buffer;

out_page:
	for (; pti >= 0; pti--) {
		dma_free_coherent(&dev->pdev->dev, UHARDDOOM_PAGE_SIZE, buffer->pages[pti].page_cpu, buffer->pages[pti].page_dma);
	}
	dma_free_coherent(&dev->pdev->dev, buffer->pnum * 4, buffer->pt_cpu, buffer->pt_dma);
out_pt:
	kfree(buffer->pages);
out_pages:
	mutex_destroy(&buffer->lock);
	kfree(buffer);
	return 0;
}

static void uharddoom_buffer_destroy(struct uharddoom_buffer *buffer)
{
	int pti;
	int i;
	struct uharddoom_device *dev = buffer->dev;
	uharddoom_fence_wait(dev, &buffer->free_fence);
	mutex_lock(&dev->last_lock);
	for (i = 0; i < 7; i++)
		if (buffer == dev->last_bufs[i])
			dev->last_bufs[i] = 0;
	mutex_unlock(&dev->last_lock);
	for (pti = 0; pti < buffer->pnum; pti++) {
		dma_free_coherent(&dev->pdev->dev, UHARDDOOM_PAGE_SIZE, buffer->pages[pti].page_cpu, buffer->pages[pti].page_dma);
	}
	dma_free_coherent(&dev->pdev->dev, buffer->pnum * 4, buffer->pt_cpu, buffer->pt_dma);
	mutex_destroy(&buffer->lock);
	kfree(buffer->pages);
	kfree(buffer);
}

static void uharddoom_buffer_destroy_kref(struct kref *kref) {
	uharddoom_buffer_destroy(container_of(kref, struct uharddoom_buffer, kref));
}

static void uharddoom_buffer_put(struct uharddoom_buffer *buffer) {
	if (buffer)
		kref_put(&buffer->kref, uharddoom_buffer_destroy_kref);
}

static long uharddoom_buffer_read(struct uharddoom_buffer *buffer, void __user *buf, 
		size_t len, loff_t off)
{
	long total = 0;
	if (off >= buffer->size || off < 0)
		return 0;
	if (len >= buffer->size - off)
		len = buffer->size - off;
	while (len) {
		int pti = off >> UHARDDOOM_PAGE_SHIFT;
		int po = (off & (UHARDDOOM_PAGE_SIZE - 1));
		int chunk = UHARDDOOM_PAGE_SIZE - po;
		if (len < chunk)
			chunk = len;
		if (copy_to_user(buf, buffer->pages[pti].page_cpu + po, chunk))
			return -EFAULT;
		total += chunk;
		off += chunk;
		buf += chunk;
		len -= chunk;
	}
	return total;
}

static long uharddoom_buffer_write(struct uharddoom_buffer *buffer, const void __user *buf, 
		size_t len, loff_t off)
{
	long total = 0;
	if (off >= buffer->size || off < 0)
		return -ENOSPC;
	if (len >= buffer->size - off)
		len = buffer->size - off;
	while (len) {
		int pti = off >> UHARDDOOM_PAGE_SHIFT;
		int po = (off & (UHARDDOOM_PAGE_SIZE - 1));
		int chunk = UHARDDOOM_PAGE_SIZE - po;
		if (len < chunk)
			chunk = len;
		if (copy_from_user(buffer->pages[pti].page_cpu + po, buf, chunk))
			return -EFAULT;
		total += chunk;
		off += chunk;
		buf += chunk;
		len -= chunk;
	}
	return total;
}

static int uharddoom_setup(struct uharddoom_device *dev, struct uharddoom_buffer **bufs) {
	int res;
	uint32_t cmd[8] = {0};
	uint32_t flags = 0;
	uint16_t sdw = 0;
	uint16_t ssw = 0;
	int i;
	for (i = 0; i < 7; i++)
		if (bufs[i] && bufs[i] != dev->last_bufs[i]) {
			if (i == 0)
				sdw = bufs[i]->width;
			if (i == 1)
				ssw = bufs[i]->width;
			cmd[1+i] = bufs[i]->pt_dma >> 8;
			flags |= 1 << (9 + i);
		}
	if (!flags)
		return 0;
	cmd[0] = UHARDDOOM_CMD_W0_SETUP(UHARDDOOM_CMD_TYPE_SETUP, flags, sdw, ssw);
	if ((res = uharddoom_submit(dev, cmd)))
		return res;
	for (i = 0; i < 7; i++)
		if (bufs[i])
			dev->last_bufs[i] = bufs[i];
	return 0;
}


/* Buffer node handling.  */

static int uharddoom_buffer_release(struct inode *inode, struct file *file)
{
	uharddoom_buffer_put(file->private_data);
	return 0;
}

static ssize_t uharddoom_buffer_file_read(struct file *file, char __user *buf,
		size_t len, loff_t *off)
{
	struct uharddoom_buffer *surf = file->private_data;
	int res;
	if ((res = mutex_lock_interruptible(&surf->lock)))
		return res;
	if ((res = uharddoom_fence_wait_interruptible(surf->dev, &surf->read_fence))) {
		mutex_unlock(&surf->lock);
		return res;
	}
	res = uharddoom_buffer_read(surf, buf, len, *off);
	if (res > 0)
		*off += res;
	mutex_unlock(&surf->lock);
	return res;
}

static ssize_t uharddoom_buffer_file_write(struct file *file, const char __user *buf,
		size_t len, loff_t *off)
{
	struct uharddoom_buffer *surf = file->private_data;
	int res;
	if ((res = mutex_lock_interruptible(&surf->lock)))
		return res;
	if ((res = uharddoom_fence_wait_interruptible(surf->dev, &surf->free_fence))) {
		mutex_unlock(&surf->lock);
		return res;
	}
	res = uharddoom_buffer_write(surf, buf, len, *off);
	if (res > 0)
		*off += res;
	mutex_unlock(&surf->lock);
	return res;
}

static const struct file_operations uharddoom_buffer_file_ops = {
	.owner = THIS_MODULE,
	.release = uharddoom_buffer_release,
	.read = uharddoom_buffer_file_read,
	.write = uharddoom_buffer_file_write,
};

static ssize_t uharddoom_write(struct file *file, const char __user *buf,
		size_t len, loff_t *off)
{
	int res;
	int i;
	size_t processed = 0;
	int used[7] = {1, 0, 0, 0, 0, 0, 0};
	struct uharddoom_context *ctx = file->private_data;
	if (len % sizeof (struct doomdev2_cmd))
		return -EINVAL;
	if (!len)
		return 0;
	if ((res = mutex_lock_interruptible(&ctx->lock)))
		goto out_lock_ctx;
	if ((res = mutex_lock_interruptible(&ctx->dev->submit_lock)))
		goto out_lock_submit;
	mutex_lock(&ctx->dev->last_lock);
	if ((res = uharddoom_setup(ctx->dev, ctx->bufs))) {
		mutex_unlock(&ctx->dev->last_lock);
		goto out_setup;
	}
	mutex_unlock(&ctx->dev->last_lock);
	if (!ctx->bufs[0]) {
		res = -EINVAL;
		goto out_setup;
	}
	while (processed < len) {
		struct doomdev2_cmd ucmd;
		uint32_t cmd[8] = {0};
		if (copy_from_user(&ucmd, buf, sizeof ucmd)) {
			res = -EFAULT;
			break;
		}
		switch (ucmd.type) {
			case DOOMDEV2_CMD_TYPE_FILL_RECT:
				if (ucmd.fill_rect.pos_x + ucmd.fill_rect.width > ctx->bufs[0]->width) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.fill_rect.pos_y + ucmd.fill_rect.height > ctx->bufs[0]->height) {
					res = -EINVAL;
					goto end;
				}
				cmd[0] = UHARDDOOM_CMD_TYPE_FILL_RECT;
				cmd[2] = UHARDDOOM_CMD_W2(ucmd.fill_rect.pos_x, ucmd.fill_rect.pos_y, 0);
				cmd[6] = UHARDDOOM_CMD_W6_A(ucmd.fill_rect.width, ucmd.fill_rect.height, ucmd.fill_rect.fill_color);
				break;
			case DOOMDEV2_CMD_TYPE_COPY_RECT:
				if (!ctx->bufs[1]) {
					res = -EINVAL;
					goto end;
				}
				used[1] = 1;
				if (ucmd.copy_rect.pos_dst_x + ucmd.copy_rect.width > ctx->bufs[0]->width) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.copy_rect.pos_dst_y + ucmd.copy_rect.height > ctx->bufs[0]->height) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.copy_rect.pos_src_x + ucmd.copy_rect.width > ctx->bufs[1]->width) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.copy_rect.pos_src_y + ucmd.copy_rect.height > ctx->bufs[1]->height) {
					res = -EINVAL;
					goto end;
				}
				cmd[0] = UHARDDOOM_CMD_TYPE_COPY_RECT;
				if (ctx->bufs[1]->dirty) {
					ctx->bufs[1]->dirty = 0;
					cmd[0] |= UHARDDOOM_CMD_FLAG_INTERLOCK;
				}
				cmd[2] = UHARDDOOM_CMD_W2(ucmd.copy_rect.pos_dst_x, ucmd.copy_rect.pos_dst_y, 0);
				cmd[3] = UHARDDOOM_CMD_W3(ucmd.copy_rect.pos_src_x, ucmd.copy_rect.pos_src_y);
				cmd[6] = UHARDDOOM_CMD_W6_A(ucmd.copy_rect.width, ucmd.copy_rect.height, 0);
				break;
			case DOOMDEV2_CMD_TYPE_DRAW_LINE:
				if (ucmd.draw_line.pos_a_x >= ctx->bufs[0]->width || ucmd.draw_line.pos_a_y >= ctx->bufs[0]->height) {
					res = -EINVAL;
					break;
				}
				if (ucmd.draw_line.pos_b_x >= ctx->bufs[0]->width || ucmd.draw_line.pos_b_y >= ctx->bufs[0]->height) {
					res = -EINVAL;
					break;
				}
				cmd[0] = UHARDDOOM_CMD_TYPE_DRAW_LINE;
				cmd[2] = UHARDDOOM_CMD_W2(ucmd.draw_line.pos_a_x, ucmd.draw_line.pos_a_y, 0);
				cmd[3] = UHARDDOOM_CMD_W3(ucmd.draw_line.pos_b_x, ucmd.draw_line.pos_b_y);
				cmd[6] = UHARDDOOM_CMD_W6_A(0, 0, ucmd.draw_line.fill_color);
				break;
			case DOOMDEV2_CMD_TYPE_DRAW_BACKGROUND:
				if (!ctx->bufs[3]) {
					res = -EINVAL;
					goto end;
				}
				used[3] = 1;
				if (ucmd.draw_background.pos_x + ucmd.draw_background.width > ctx->bufs[0]->width) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.draw_background.pos_y + ucmd.draw_background.height > ctx->bufs[0]->height) {
					res = -EINVAL;
					goto end;
				}
				if ((ucmd.draw_background.flat_idx + 1) * 0x1000 > ctx->bufs[3]->size) {
					res = -EINVAL;
					goto end;
				}
				cmd[0] = UHARDDOOM_CMD_TYPE_DRAW_BACKGROUND;
				cmd[2] = UHARDDOOM_CMD_W2(ucmd.draw_background.pos_x, ucmd.draw_background.pos_y, ucmd.draw_background.flat_idx);
				cmd[6] = UHARDDOOM_CMD_W6_A(ucmd.draw_background.width, ucmd.draw_background.height, 0);
				break;
			case DOOMDEV2_CMD_TYPE_DRAW_COLUMN: {
				int tridx = 0;
				int cmidx = 0;
				cmd[0] = UHARDDOOM_CMD_TYPE_DRAW_COLUMN;
				if (!ctx->bufs[2]) {
					res = -EINVAL;
					goto end;
				}
				used[2] = 1;
				if (ucmd.draw_column.flags & DOOMDEV2_CMD_FLAGS_TRANSLATE) {
					if (!ctx->bufs[4] || ctx->bufs[4]->size < (ucmd.draw_column.translation_idx + 1) * 0x100) {
						res = -EINVAL;
						goto end;
					}
					used[4] = 1;
					cmd[0] |= UHARDDOOM_CMD_FLAG_TRANSLATION;
					tridx = ucmd.draw_column.translation_idx;
				}
				if (ucmd.draw_column.flags & DOOMDEV2_CMD_FLAGS_COLORMAP) {
					if (!ctx->bufs[5] || ctx->bufs[5]->size < (ucmd.draw_column.colormap_idx + 1) * 0x100) {
						res = -EINVAL;
						goto end;
					}
					used[5] = 1;
					cmd[0] |= UHARDDOOM_CMD_FLAG_COLORMAP;
					cmidx = ucmd.draw_column.colormap_idx;
				}
				if (ucmd.draw_column.flags & DOOMDEV2_CMD_FLAGS_TRANMAP) {
					if (!ctx->bufs[6] || ctx->bufs[6]->size < 0x10000) {
						res = -EINVAL;
						goto end;
					}
					used[6] = 1;
					cmd[0] |= UHARDDOOM_CMD_FLAG_TRANMAP;
				}
				cmd[1] = UHARDDOOM_CMD_W1(tridx, cmidx);
				if (ucmd.draw_column.pos_x >= ctx->bufs[0]->width) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.draw_column.pos_b_y >= ctx->bufs[0]->height || ucmd.draw_column.pos_b_y < ucmd.draw_column.pos_a_y) {
					res = -EINVAL;
					goto end;
				}
				cmd[2] = UHARDDOOM_CMD_W2(ucmd.draw_column.pos_x, ucmd.draw_column.pos_a_y, 0);
				cmd[3] = UHARDDOOM_CMD_W3(ucmd.draw_column.pos_x, ucmd.draw_column.pos_b_y);
				cmd[4] = ucmd.draw_column.ustart;
				cmd[5] = ucmd.draw_column.ustep;
				cmd[6] = ucmd.draw_column.texture_offset;
				if (ucmd.draw_column.texture_offset & ~0x3fffff) {
					res = -EINVAL;
					goto end;
				}
				cmd[7] = UHARDDOOM_CMD_W7_B((ctx->bufs[2]->size - 1) >> 6, ucmd.draw_column.texture_height);
				break;
			}
			case DOOMDEV2_CMD_TYPE_DRAW_FUZZ: {
				cmd[0] = UHARDDOOM_CMD_TYPE_DRAW_FUZZ;
				if (!ctx->bufs[2]) {
					res = -EINVAL;
					goto end;
				}
				used[2] = 1;
				if (!ctx->bufs[5] || ctx->bufs[5]->size < (ucmd.draw_fuzz.colormap_idx + 1) * 0x100) {
					res = -EINVAL;
					goto end;
				}
				used[5] = 1;
				cmd[1] = UHARDDOOM_CMD_W1(0, ucmd.draw_fuzz.colormap_idx);
				if (ucmd.draw_fuzz.pos_x >= ctx->bufs[0]->width) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.draw_fuzz.fuzz_end >= ctx->bufs[0]->height ||
						ucmd.draw_fuzz.pos_b_y > ucmd.draw_fuzz.fuzz_end ||
						ucmd.draw_fuzz.pos_a_y > ucmd.draw_fuzz.pos_b_y ||
						ucmd.draw_fuzz.fuzz_start > ucmd.draw_fuzz.pos_a_y ||
						ucmd.draw_fuzz.fuzz_pos >= 50) {
					res = -EINVAL;
					goto end;
				}
				cmd[2] = UHARDDOOM_CMD_W2(ucmd.draw_fuzz.pos_x, ucmd.draw_fuzz.pos_a_y, 0);
				cmd[3] = UHARDDOOM_CMD_W3(ucmd.draw_fuzz.pos_x, ucmd.draw_fuzz.pos_b_y);
				cmd[6] = UHARDDOOM_CMD_W6_C(ucmd.draw_fuzz.fuzz_start, ucmd.draw_fuzz.fuzz_end, ucmd.draw_fuzz.fuzz_pos);
				break;
			}
			case DOOMDEV2_CMD_TYPE_DRAW_SPAN: {
				int tridx = 0;
				int cmidx = 0;
				cmd[0] = UHARDDOOM_CMD_TYPE_DRAW_SPAN;
				if (!ctx->bufs[3] || ctx->bufs[3]->size < (ucmd.draw_span.flat_idx + 1) * 0x1000) {
					res = -EINVAL;
					goto end;
				}
				used[3] = 1;
				if (ucmd.draw_span.flags & DOOMDEV2_CMD_FLAGS_TRANSLATE) {
					if (!ctx->bufs[4] || ctx->bufs[4]->size < (ucmd.draw_span.translation_idx + 1) * 0x100) {
						res = -EINVAL;
						goto end;
					}
					used[4] = 1;
					cmd[0] |= UHARDDOOM_CMD_FLAG_TRANSLATION;
					tridx = ucmd.draw_span.translation_idx;
				}
				if (ucmd.draw_span.flags & DOOMDEV2_CMD_FLAGS_COLORMAP) {
					if (!ctx->bufs[5] || ctx->bufs[5]->size < (ucmd.draw_span.colormap_idx + 1) * 0x100) {
						res = -EINVAL;
						goto end;
					}
					used[5] = 1;
					cmd[0] |= UHARDDOOM_CMD_FLAG_COLORMAP;
					cmidx = ucmd.draw_span.colormap_idx;
				}
				if (ucmd.draw_span.flags & DOOMDEV2_CMD_FLAGS_TRANMAP) {
					if (!ctx->bufs[6] || ctx->bufs[6]->size < 0x10000) {
						res = -EINVAL;
						goto end;
					}
					used[6] = 1;
					cmd[0] |= UHARDDOOM_CMD_FLAG_TRANMAP;
				}
				cmd[1] = UHARDDOOM_CMD_W1(tridx, cmidx);
				if (ucmd.draw_span.pos_y >= ctx->bufs[0]->height) {
					res = -EINVAL;
					goto end;
				}
				if (ucmd.draw_span.pos_b_x >= ctx->bufs[0]->width || ucmd.draw_span.pos_b_x < ucmd.draw_span.pos_a_x) {
					res = -EINVAL;
					goto end;
				}
				cmd[2] = UHARDDOOM_CMD_W2(ucmd.draw_span.pos_a_x, ucmd.draw_span.pos_y, ucmd.draw_span.flat_idx);
				cmd[3] = UHARDDOOM_CMD_W3(ucmd.draw_span.pos_b_x, ucmd.draw_span.pos_y);
				cmd[4] = ucmd.draw_span.ustart;
				cmd[5] = ucmd.draw_span.ustep;
				cmd[6] = ucmd.draw_span.vstart;
				cmd[7] = ucmd.draw_span.vstep;
				break;
			}
			default:
				res = -EINVAL;
				goto end;
		}
		if ((res = uharddoom_submit(ctx->dev, cmd)))
			break;
		processed += sizeof ucmd;
		buf += sizeof ucmd;
	}
end:
	mutex_lock(&ctx->bufs[0]->lock);
	ctx->bufs[0]->dirty = 1;
	uharddoom_fence_set(ctx->dev, &ctx->bufs[0]->read_fence);
	mutex_unlock(&ctx->bufs[0]->lock);
	for (i = 0; i < 7; i++)
		if (used[i]) {
			mutex_lock(&ctx->bufs[i]->lock);
			uharddoom_fence_set(ctx->dev, &ctx->bufs[i]->free_fence);
			mutex_unlock(&ctx->bufs[i]->lock);
		}

out_setup:
	mutex_unlock(&ctx->dev->submit_lock);
out_lock_submit:
	mutex_unlock(&ctx->lock);
out_lock_ctx:
	return processed ? processed : res;
}

/* Main device node handling.  */

static int uharddoom_open(struct inode *inode, struct file *file)
{
	struct uharddoom_device *dev = container_of(inode->i_cdev, struct uharddoom_device, cdev);
	struct uharddoom_context *ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;
	mutex_init(&ctx->lock);
	file->private_data = ctx;
	return nonseekable_open(inode, file);
}

static int uharddoom_release(struct inode *inode, struct file *file)
{
	struct uharddoom_context *ctx = file->private_data;
	int i;
	for (i = 0; i < 7; i++)
		uharddoom_buffer_put(ctx->bufs[i]);
	kfree(ctx);
	return 0;
}

int uharddoom_make_fd(const char *name, const struct file_operations *fops,
		     struct uharddoom_buffer *priv)
{
	int error, fd;
	struct file *file;

	error = get_unused_fd_flags(O_RDWR | O_CLOEXEC);
	if (error < 0)
		return error;
	fd = error;

	file = anon_inode_getfile(name, fops, priv, O_RDWR | O_CLOEXEC);
	if (IS_ERR(file)) {
		error = PTR_ERR(file);
		goto err_put_unused_fd;
	}
	file->f_mode |= FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE;
	fd_install(fd, file);

	return fd;

err_put_unused_fd:
	put_unused_fd(fd);
	return error;
}

static struct uharddoom_buffer *uharddoom_get_buffer(struct uharddoom_device *dev, int fd) {
	struct fd bfd;
	struct uharddoom_buffer *res;
	if (fd == -1)
		return 0;
	bfd = fdget(fd);
	if (!bfd.file)
		return ERR_PTR(-EBADF);
	if (bfd.file->f_op != &uharddoom_buffer_file_ops) {
		fdput(bfd);
		return ERR_PTR(-EINVAL);
	}
	res = bfd.file->private_data;
	if (res->dev != dev) {
		fdput(bfd);
		return ERR_PTR(-EXDEV);
	}
	kref_get(&res->kref);
	fdput(bfd);
	return res;
}

static long uharddoom_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct uharddoom_context *ctx = file->private_data;
	struct uharddoom_device *dev = ctx->dev;
	int res;
	void __user *parg = (void __user *)arg;
	switch (cmd) {
	case DOOMDEV2_IOCTL_CREATE_SURFACE: {
		struct doomdev2_ioctl_create_surface param;
		struct uharddoom_buffer *surf;
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		if (param.width > 2048 || !param.width || param.width % 64)
			return -EINVAL;
		if (param.height > 2048 || !param.height)
			return -EINVAL;
		surf = uharddoom_buffer_create(dev, param.width * param.height, param.width, param.height);
		if (!surf)
			return -ENOMEM;
		res = uharddoom_make_fd("uharddoom_surf", &uharddoom_buffer_file_ops, surf);
		if (res < 0)
			uharddoom_buffer_destroy(surf);
		return res;
	}
	case DOOMDEV2_IOCTL_CREATE_BUFFER: {
		struct doomdev2_ioctl_create_buffer param;
		struct uharddoom_buffer *buffer;
		uint32_t size;
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		if (param.size > 0x400000 || param.size == 0)
			return -EINVAL;
		size = param.size;
		if (size & 0x3f) {
			size |= 0x3f;
			size++;
		}
		buffer = uharddoom_buffer_create(dev, size, 0, 0);
		if (!buffer)
			return -ENOMEM;
		if ((res = uharddoom_make_fd("uharddoom_texture", &uharddoom_buffer_file_ops,
				buffer)) < 0)
			goto tex_out_anon;
		return res;
tex_out_anon:
		uharddoom_buffer_destroy(buffer);
		return res;
	}
	case DOOMDEV2_IOCTL_SETUP: {
		int i;
		struct doomdev2_ioctl_setup param;
		struct uharddoom_buffer *bufs[7];
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		if (mutex_lock_interruptible(&ctx->lock))
			return -ERESTARTSYS;
		bufs[0] = uharddoom_get_buffer(dev, param.surf_dst_fd);
		if (IS_ERR(bufs[0])) {
			res = PTR_ERR(bufs[0]);
			goto out_surf_dst;
		}
		if (bufs[0] && !bufs[0]->width) {
			res = -EINVAL;
			goto out_surf_src;
		}
		bufs[1] = uharddoom_get_buffer(dev, param.surf_src_fd);
		if (IS_ERR(bufs[1])) {
			res = PTR_ERR(bufs[1]);
			goto out_surf_src;
		}
		if (bufs[1] && !bufs[1]->width) {
			res = -EINVAL;
			goto out_texture;
		}
		bufs[2] = uharddoom_get_buffer(dev, param.texture_fd);
		if (IS_ERR(bufs[2])) {
			res = PTR_ERR(bufs[2]);
			goto out_texture;
		}
		bufs[3] =  uharddoom_get_buffer(dev, param.flat_fd);
		if (IS_ERR(bufs[3])) {
			res = PTR_ERR(bufs[3]);
			goto out_flat;
		}
		bufs[4] = uharddoom_get_buffer(dev, param.translation_fd);
		if (IS_ERR(bufs[4])) {
			res = PTR_ERR(bufs[4]);
			goto out_translation;
		}
		bufs[5] = uharddoom_get_buffer(dev, param.colormap_fd);
		if (IS_ERR(bufs[5])) {
			res = PTR_ERR(bufs[5]);
			goto out_colormap;
		}
		bufs[6] = uharddoom_get_buffer(dev, param.tranmap_fd);
		if (IS_ERR(bufs[6])) {
			res = PTR_ERR(bufs[6]);
			goto out_tranmap;
		}
		for (i = 0; i < 7; i++) {
			uharddoom_buffer_put(ctx->bufs[i]);
			ctx->bufs[i] = bufs[i];
		}
		mutex_unlock(&ctx->lock);
		return 0;

out_tranmap:
		uharddoom_buffer_put(bufs[5]);
out_colormap:
		uharddoom_buffer_put(bufs[4]);
out_translation:
		uharddoom_buffer_put(bufs[3]);
out_flat:
		uharddoom_buffer_put(bufs[2]);
out_texture:
		uharddoom_buffer_put(bufs[1]);
out_surf_src:
		uharddoom_buffer_put(bufs[0]);
out_surf_dst:
		mutex_unlock(&ctx->lock);
		return res;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations uharddoom_file_ops = {
	.owner = THIS_MODULE,
	.open = uharddoom_open,
	.release = uharddoom_release,
	.unlocked_ioctl = uharddoom_ioctl,
	.compat_ioctl = uharddoom_ioctl,
	.write = uharddoom_write,
};

/* PCI driver.  */

static int uharddoom_probe(struct pci_dev *pdev,
	const struct pci_device_id *pci_id)
{
	int err, i;

	/* Allocate our structure.  */
	struct uharddoom_device *dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto out_alloc;
	}
	pci_set_drvdata(pdev, dev);
	dev->pdev = pdev;

	/* Locks etc.  */
	spin_lock_init(&dev->slock);
	mutex_init(&dev->submit_lock);
	init_waitqueue_head(&dev->free_wq);
	INIT_LIST_HEAD(&dev->fence_list);

	/* Command bookkeeping.  */
	dev->cmds_to_ping = UHARDDOOM_PING_INTERVAL;

	/* Allocate a free index.  */
	mutex_lock(&uharddoom_devices_lock);
	for (i = 0; i < UHARDDOOM_MAX_DEVICES; i++)
		if (!uharddoom_devices[i])
			break;
	if (i == UHARDDOOM_MAX_DEVICES) {
		err = -ENOSPC; // XXX right?
		mutex_unlock(&uharddoom_devices_lock);
		goto out_slot;
	}
	uharddoom_devices[i] = dev;
	dev->idx = i;
	mutex_unlock(&uharddoom_devices_lock);

	/* Enable hardware resources.  */
	if ((err = pci_enable_device(pdev)))
		goto out_enable;

	if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32))))
		goto out_mask;
	if ((err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32))))
		goto out_mask;
	pci_set_master(pdev);

	if ((err = pci_request_regions(pdev, "uharddoom")))
		goto out_regions;

	/* Map the BAR.  */
	if (!(dev->bar = pci_iomap(pdev, 0, 0))) {
		err = -ENOMEM;
		goto out_bar;
	}

	/* Connect the IRQ line.  */
	if ((err = request_irq(pdev->irq, uharddoom_isr, IRQF_SHARED, "uharddoom", dev)))
		goto out_irq;

	/* Reset things that need resetting.  */
	uharddoom_iow(dev, UHARDDOOM_INTR, UHARDDOOM_INTR_MASK);
	uharddoom_iow(dev, UHARDDOOM_RESET, UHARDDOOM_RESET_ALL);
	uharddoom_iow(dev, UHARDDOOM_FENCE_COUNTER, 0);
	uharddoom_iow(dev, UHARDDOOM_FENCE_WAIT, 0);
	uharddoom_iow(dev, UHARDDOOM_FE_CODE_ADDR, 0);
	for (i = 0; i < ARRAY_SIZE(doomcode2); i++)
		uharddoom_iow(dev, UHARDDOOM_FE_CODE_WINDOW, doomcode2[i]);

#ifdef USE_CMD_FETCH
	/* Set up the command buffer.  */
	dev->cmd_buf = uharddoom_buffer_create(dev, COMMAND_BUF_SIZE * 32, 0, 0);
	if (!dev->cmd_buf) {
		err = -ENOMEM;
		goto out_cmd;
	}
	uharddoom_iow(dev, UHARDDOOM_CMD_PT, dev->cmd_buf->pt_dma >> 8);
	uharddoom_iow(dev, UHARDDOOM_CMD_SIZE, COMMAND_BUF_SIZE);
	uharddoom_iow(dev, UHARDDOOM_CMD_READ_IDX, 0);
	uharddoom_iow(dev, UHARDDOOM_CMD_WRITE_IDX, 0);
#endif
	
	/* Now bring up the device.  */
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, UHARDDOOM_INTR_MASK & ~UHARDDOOM_INTR_PONG_ASYNC);
#ifdef USE_CMD_FETCH
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL);
#else
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL & ~UHARDDOOM_ENABLE_CMD_FETCH);
#endif

	/* We're live.  Let's export the cdev.  */
	cdev_init(&dev->cdev, &uharddoom_file_ops);
	if ((err = cdev_add(&dev->cdev, uharddoom_devno + dev->idx, 1)))
		goto out_cdev;

	/* And register it in sysfs.  */
	dev->dev = device_create(&uharddoom_class,
			&dev->pdev->dev, uharddoom_devno + dev->idx, dev,
			"doom%d", dev->idx);
	if (IS_ERR(dev->dev)) {
		printk(KERN_ERR "uharddoom: failed to register subdevice\n");
		/* too bad. */
		dev->dev = 0;
	}

	return 0;

out_cdev:
#ifdef USE_CMD_FETCH
	uharddoom_buffer_destroy(dev->cmd_buf);
out_cmd:
#endif
	free_irq(pdev->irq, dev);
out_irq:
	pci_iounmap(pdev, dev->bar);
out_bar:
	pci_release_regions(pdev);
out_regions:
out_mask:
	pci_disable_device(pdev);
out_enable:
	mutex_lock(&uharddoom_devices_lock);
	uharddoom_devices[dev->idx] = 0;
	mutex_unlock(&uharddoom_devices_lock);
out_slot:
	kfree(dev);
out_alloc:
	return err;
}

static void uharddoom_remove(struct pci_dev *pdev)
{
	struct uharddoom_device *dev = pci_get_drvdata(pdev);
	if (dev->dev) {
		device_destroy(&uharddoom_class, uharddoom_devno + dev->idx);
	}
	cdev_del(&dev->cdev);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, 0);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, 0);
	uharddoom_ior(dev, UHARDDOOM_INTR);
#ifdef USE_CMD_FETCH
	uharddoom_buffer_destroy(dev->cmd_buf);
#endif
	free_irq(pdev->irq, dev);
	pci_iounmap(pdev, dev->bar);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	mutex_lock(&uharddoom_devices_lock);
	uharddoom_devices[dev->idx] = 0;
	mutex_unlock(&uharddoom_devices_lock);
	// XXX kref me instead?
	kfree(dev);
}

static struct pci_device_id uharddoom_pciids[] = {
	{ PCI_DEVICE(UHARDDOOM_VENDOR_ID, UHARDDOOM_DEVICE_ID) },
	{ 0 }
};

static struct pci_driver uharddoom_pci_driver = {
	.name = "uharddoom",
	.id_table = uharddoom_pciids,
	.probe = uharddoom_probe,
	.remove = uharddoom_remove,
	// XXX suspend
};

/* Init & exit.  */

static int uharddoom_init(void)
{
	int err;
	if ((err = alloc_chrdev_region(&uharddoom_devno, 0, UHARDDOOM_MAX_DEVICES, "uharddoom")))
		goto err_chrdev;
	if ((err = class_register(&uharddoom_class)))
		goto err_class;
	if ((err = pci_register_driver(&uharddoom_pci_driver)))
		goto err_pci;
	return 0;

err_pci:
	class_unregister(&uharddoom_class);
err_class:
	unregister_chrdev_region(uharddoom_devno, UHARDDOOM_MAX_DEVICES);
err_chrdev:
	return err;
}

static void uharddoom_exit(void)
{
	pci_unregister_driver(&uharddoom_pci_driver);
	class_unregister(&uharddoom_class);
	unregister_chrdev_region(uharddoom_devno, UHARDDOOM_MAX_DEVICES);
}

module_init(uharddoom_init);
module_exit(uharddoom_exit);
#endif
