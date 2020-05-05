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

#define USE_BATCH
#define BATCH_BUF_SIZE 0x100

MODULE_AUTHOR("Marcelina Kościelnicka");
MODULE_LICENSE("GPL");

struct uharddoom_page {
	void *page_cpu;
	dma_addr_t page_dma;
};

struct uharddoom_buffer {
	struct uharddoom_device *dev;
	uint32_t size;
	struct uharddoom_page *pages;
	uint32_t pnum;
	struct kref kref;
};

struct uharddoom_vmap {
	struct list_head lh;
	struct uharddoom_buffer *buf;
	uint32_t va;
};

struct uharddoom_vspace {
	struct mutex lock;
	struct uharddoom_device *dev;
	__le32 *pd_cpu;
	dma_addr_t pd_dma;
	__le32 **pts;
	struct list_head maps;
};

#ifdef USE_BATCH
struct uharddoom_job {
	struct list_head lh;
	struct list_head ctx_lh;
	struct uharddoom_context *ctx;
	int batch_pos;
};

struct uharddoom_wait {
	struct list_head lh;
	struct completion completion;
	int batch_pos;
	int done;
};
#else
struct uharddoom_job {
	struct list_head lh;
	struct uharddoom_context *ctx;
	uint32_t cmd_ptr;
	uint32_t cmd_size;
};
#endif

struct uharddoom_device {
	struct pci_dev *pdev;
	struct cdev cdev;
	int idx;
	struct device *dev;
	void __iomem *bar;
	spinlock_t slock;
#ifdef USE_BATCH
	struct uharddoom_vspace *batch_vs;
	struct uharddoom_buffer *batch_buf;
	uint32_t batch_addr;
	int batch_put_idx;
	int batch_get_idx;
	struct list_head waits;
#else	
	wait_queue_head_t idle_wq;
#endif
	struct list_head pending_jobs;
};

struct uharddoom_context {
	struct mutex lock;
	struct uharddoom_device *dev;
	struct uharddoom_vspace *vs;
#ifdef USE_BATCH
	struct list_head pending_jobs;
#else
	int pending_jobs;
	wait_queue_head_t wq;
#endif
	int error;
};

static dev_t uharddoom_devno;
static struct uharddoom_device *uharddoom_devices[UHARDDOOM_MAX_DEVICES];
static DEFINE_MUTEX(uharddoom_devices_lock);
static struct class uharddoom_class = {
	.name = "uharddoom",
	.owner = THIS_MODULE,
};

/* Hardware access. */

static inline void uharddoom_iow(struct uharddoom_device *dev, uint32_t reg, uint32_t val)
{
	iowrite32(val, dev->bar + reg);
	printk(KERN_ALERT "uharddoom %03x <- %08x\n", reg, val);
}

static inline uint32_t uharddoom_ior(struct uharddoom_device *dev, uint32_t reg)
{
	uint32_t res = ioread32(dev->bar + reg);
	printk(KERN_ALERT "uharddoom %03x -> %08x\n", reg, res);
	return res;
}

/* Buffers & vspaces.  */

static struct uharddoom_buffer *uharddoom_buffer_create(struct uharddoom_device *dev, uint32_t size) {
	int pti;
	struct uharddoom_buffer *buffer = kzalloc(sizeof *buffer, 1);
	if (!buffer)
		return 0;
	size = round_up(size, PAGE_SIZE);
	size = round_up(size, UHARDDOOM_PAGE_SIZE);
	kref_init(&buffer->kref);
	buffer->dev = dev;
	buffer->size = size;
	buffer->pnum = (size + UHARDDOOM_PAGE_SIZE - 1) >> UHARDDOOM_PAGE_SHIFT;
	buffer->pages = kzalloc(sizeof *buffer->pages * buffer->pnum, GFP_KERNEL);
	if (!buffer->pages)
		goto out_pages;
	for (pti = 0; pti < buffer->pnum; pti++)
		if (!(buffer->pages[pti].page_cpu = dma_alloc_coherent(&dev->pdev->dev,
				UHARDDOOM_PAGE_SIZE,
				&buffer->pages[pti].page_dma, GFP_KERNEL | __GFP_ZERO)))
			goto out_page;
	printk(KERN_ALERT "CREATE BUF %p %u\n", buffer, kref_read(&buffer->kref));
	return buffer;

out_page:
	for (; pti >= 0; pti--)
		dma_free_coherent(&dev->pdev->dev, UHARDDOOM_PAGE_SIZE, buffer->pages[pti].page_cpu, buffer->pages[pti].page_dma);
	kfree(buffer->pages);
out_pages:
	kfree(buffer);
	return 0;
}

static void uharddoom_buffer_destroy(struct kref *kref) {
	struct uharddoom_buffer *buffer = container_of(kref, struct uharddoom_buffer, kref);
	struct uharddoom_device *dev = buffer->dev;
	int pti;
	printk(KERN_ALERT "KILL BUF %p %u\n", buffer, kref_read(&buffer->kref));
	for (pti = buffer->pnum - 1; pti >= 0; pti--)
		dma_free_coherent(&dev->pdev->dev, UHARDDOOM_PAGE_SIZE, buffer->pages[pti].page_cpu, buffer->pages[pti].page_dma);
	kfree(buffer->pages);
	kfree(buffer);
}

static inline struct uharddoom_buffer *uharddoom_buffer_get(struct uharddoom_buffer *buf) {
	printk(KERN_ALERT "GET BUF %p %u\n", buf, kref_read(&buf->kref));
	kref_get(&buf->kref);
	printk(KERN_ALERT "... %p %u\n", buf, kref_read(&buf->kref));
	return buf;
}

static void uharddoom_buffer_put(struct uharddoom_buffer *buf) {
	printk(KERN_ALERT "PUT BUF %p %u\n", buf, kref_read(&buf->kref));
	kref_put(&buf->kref, uharddoom_buffer_destroy);
}

static struct uharddoom_vspace *uharddoom_vspace_create(struct uharddoom_device *dev) {
	struct uharddoom_vspace *vs = kzalloc(sizeof *vs, GFP_KERNEL);
	if (!vs)
		goto out_vs;
	vs->dev = dev;
	INIT_LIST_HEAD(&vs->maps);
	mutex_init(&vs->lock);
	if (!(vs->pd_cpu = dma_alloc_coherent(&dev->pdev->dev,
			UHARDDOOM_PAGE_SIZE,
			&vs->pd_dma, GFP_KERNEL | __GFP_ZERO)))
		goto out_pd;
	if (!(vs->pts = kzalloc(sizeof *vs->pts * 0x400, GFP_KERNEL)))
		goto out_pts;
	return vs;

out_pts:
	dma_free_coherent(&dev->pdev->dev, UHARDDOOM_PAGE_SIZE, vs->pd_cpu, vs->pd_dma);
out_pd:
	mutex_destroy(&vs->lock);
	kfree(vs);
out_vs:
	return 0;
}

static void uharddoom_vspace_destroy(struct uharddoom_vspace *vs) {
	struct uharddoom_device *dev = vs->dev;
	struct list_head *lh, *tmp;
	int i;
	list_for_each_safe(lh, tmp, &vs->maps) {
		struct uharddoom_vmap *map = list_entry(lh, struct uharddoom_vmap, lh);
		uharddoom_buffer_put(map->buf);
		kfree(map);
	}
	for (i = 0; i < 0x400; i++)
		if (vs->pts[i]) {
			uint32_t pde = __le32_to_cpu(vs->pd_cpu[i]);
			dma_addr_t pt_dma = (uint64_t)(pde & UHARDDOOM_PDE_PA_MASK) << UHARDDOOM_PDE_PA_SHIFT;
			dma_free_coherent(&dev->pdev->dev, UHARDDOOM_PAGE_SIZE, vs->pts[i], pt_dma);
		}
	kfree(vs->pts);
	dma_free_coherent(&dev->pdev->dev, UHARDDOOM_PAGE_SIZE, vs->pd_cpu, vs->pd_dma);
	mutex_destroy(&vs->lock);
	kfree(vs);
}

static int uharddoom_vspace_map(struct uharddoom_vspace *vs, struct uharddoom_buffer *buf, int map_rdonly, uint32_t *res) {
	struct uharddoom_device *dev = vs->dev;
	uint32_t addr = 0, ea;
	struct list_head *lh, *target;
	struct uharddoom_vmap *map;
	int i;
	if (dev != buf->dev)
		return -EXDEV;
	mutex_lock(&vs->lock);
	target = &vs->maps;
	ea = addr + buf->pnum * UHARDDOOM_PAGE_SIZE + UHARDDOOM_PAGE_SIZE;
	list_for_each(lh, &vs->maps) {
		map = list_entry(lh, struct uharddoom_vmap, lh);
		if (ea < addr) {
			mutex_unlock(&vs->lock);
			return -ENOMEM;
		}
		if (map->va >= ea)
			break;
		target = lh;
		addr = map->va + map->buf->pnum * UHARDDOOM_PAGE_SIZE + UHARDDOOM_PAGE_SIZE;
		ea = addr + buf->pnum * UHARDDOOM_PAGE_SIZE + UHARDDOOM_PAGE_SIZE;
	}
	if (ea < addr) {
		mutex_unlock(&vs->lock);
		return -ENOMEM;
	}
	for (i = addr >> 22; i <= ((ea - 1) >> 22); i++) {
		dma_addr_t pt_dma;
		uint32_t pde;
		if (vs->pts[i])
			continue;
		if (!(vs->pts[i] = dma_alloc_coherent(&dev->pdev->dev,
				UHARDDOOM_PAGE_SIZE,
				&pt_dma, GFP_KERNEL | __GFP_ZERO))) {

			mutex_unlock(&vs->lock);
			return -ENOMEM;
		}
		pde = pt_dma >> UHARDDOOM_PDE_PA_SHIFT | UHARDDOOM_PDE_PRESENT;
		vs->pd_cpu[i] = __cpu_to_le32(pde);
	}
	map = kzalloc(sizeof *map, GFP_KERNEL);
	if (!map) {
		mutex_unlock(&vs->lock);
		return -ENOMEM;
	}
	map->buf = uharddoom_buffer_get(buf);
	map->va = addr;
	list_add(&map->lh, target);
	for (i = 0; i < buf->pnum; i++) {
		int pti = (addr >> UHARDDOOM_PAGE_SHIFT) + i;
		int pdi = pti >> 10;
		uint32_t pte = buf->pages[i].page_dma >> UHARDDOOM_PTE_PA_SHIFT | UHARDDOOM_PTE_PRESENT;
		if (!map_rdonly)
			pte |= UHARDDOOM_PTE_WRITABLE;
		pti &= 0x3ff;
		vs->pts[pdi][pti] = __cpu_to_le32(pte);
	}
	mutex_unlock(&vs->lock);
	*res = addr;
	return 0;
}

static int uharddoom_vspace_unmap(struct uharddoom_vspace *vs, uint32_t addr) {
	struct list_head *lh;
	int i;
	mutex_lock(&vs->lock);
	list_for_each(lh, &vs->maps) {
		struct uharddoom_vmap *map = list_entry(lh, struct uharddoom_vmap, lh);
		if (map->va == addr) {
			for (i = 0; i < map->buf->pnum; i++) {
				int pti = (addr >> UHARDDOOM_PAGE_SHIFT) + i;
				int pdi = pti >> 10;
				pti &= 0x3ff;
				vs->pts[pdi][pti] = 0;
			}
			uharddoom_iow(vs->dev, UHARDDOOM_RESET, UHARDDOOM_RESET_TLB_USER);
			uharddoom_ior(vs->dev, UHARDDOOM_STATUS);
			uharddoom_buffer_put(map->buf);
			list_del(&map->lh);
			kfree(map);
			mutex_unlock(&vs->lock);
			return 0;
		}
	}
	mutex_unlock(&vs->lock);
	return -ENOENT;
}

/* Context handling.  */

#ifdef USE_BATCH
static int uharddoom_batch_lt(struct uharddoom_device *dev, int a, int b) {
	if (a <= dev->batch_put_idx)
		a += BATCH_BUF_SIZE;
	if (b <= dev->batch_put_idx)
		b += BATCH_BUF_SIZE;
	return a < b;
}

static void uharddoom_update_wait(struct uharddoom_device *dev) {
	int new_get;
	struct list_head *lh, *tmp;
	int changed;
redo:
	if (list_empty(&dev->waits)) {
		uharddoom_iow(dev, UHARDDOOM_BATCH_WAIT, ~0xfu);
	} else {
		int idx = list_first_entry(&dev->waits, struct uharddoom_wait, lh)->batch_pos;
		idx += 1;
		idx %= BATCH_BUF_SIZE;
		uharddoom_iow(dev, UHARDDOOM_BATCH_WAIT, dev->batch_addr + idx * UHARDDOOM_BATCH_JOB_SIZE);
	}
	new_get = (uharddoom_ior(dev, UHARDDOOM_BATCH_GET) - dev->batch_addr) / UHARDDOOM_BATCH_JOB_SIZE;
	changed = 0;
	list_for_each_safe(lh, tmp, &dev->waits) {
		struct uharddoom_wait *wait = list_entry(lh, struct uharddoom_wait, lh);
		if (uharddoom_batch_lt(dev, wait->batch_pos, new_get)) {
			complete(&wait->completion);
			wait->done = 1;
			list_del(&wait->lh);
			changed = 1;
		} else
			break;
	}
	list_for_each_safe(lh, tmp, &dev->pending_jobs) {
		struct uharddoom_job *job = list_entry(lh, struct uharddoom_job, lh);
		if (uharddoom_batch_lt(dev, job->batch_pos, new_get)) {
			list_del(&job->lh);
			list_del(&job->ctx_lh);
			kfree(job);
		} else
			break;
	}
	dev->batch_get_idx = new_get;
	if (changed)
		goto redo;
}

static void uharddoom_start_wait(struct uharddoom_device *dev, struct uharddoom_wait *wait, struct uharddoom_job *job) {
	struct list_head *lh;
	wait->batch_pos = job->batch_pos;
	init_completion(&wait->completion);
	wait->done = 0;
	list_for_each(lh, &dev->waits) {
		if (uharddoom_batch_lt(dev, wait->batch_pos, list_entry(lh, struct uharddoom_wait, lh)->batch_pos)) {
			list_add_tail(&wait->lh, lh);
			uharddoom_update_wait(dev);
			return;
		}
	}
	list_add_tail(&wait->lh, &dev->waits);
	uharddoom_update_wait(dev);
}

static void uharddoom_abort_wait(struct uharddoom_device *dev, struct uharddoom_wait *wait) {
	if (!wait->done) {
		list_del(&wait->lh);
		uharddoom_update_wait(dev);
	}
}

static int uharddoom_batch_avail(struct uharddoom_device *dev) {
	return (dev->batch_put_idx + 1) % BATCH_BUF_SIZE != dev->batch_get_idx;
}
#endif

static struct uharddoom_context *uharddoom_context_create(struct uharddoom_device *dev) {
	struct uharddoom_context *ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return 0;
	ctx->dev = dev;
	mutex_init(&ctx->lock);
	ctx->vs = uharddoom_vspace_create(dev);
	if (!ctx->vs) {
		kfree(ctx);
		return 0;
	}
#ifdef USE_BATCH
	INIT_LIST_HEAD(&ctx->pending_jobs);
#else
	ctx->pending_jobs = 0;
	init_waitqueue_head(&ctx->wq);
#endif
	return ctx;
}

static void uharddoom_context_destroy(struct uharddoom_context *ctx) {
#ifdef USE_BATCH
	unsigned long flags;
	struct uharddoom_device *dev = ctx->dev;
	spin_lock_irqsave(&dev->slock, flags);
	if (!list_empty(&ctx->pending_jobs)) {
		struct uharddoom_job *job = list_last_entry(&ctx->pending_jobs, struct uharddoom_job, ctx_lh);
		struct uharddoom_wait wait;
		uharddoom_start_wait(ctx->dev, &wait, job);
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_for_completion(&wait.completion);
		spin_lock_irqsave(&dev->slock, flags);
	}
	spin_unlock_irqrestore(&dev->slock, flags);
#else
	wait_event(ctx->wq, !ctx->pending_jobs);
#endif
	uharddoom_vspace_destroy(ctx->vs);
	kfree(ctx);
}

static int uharddoom_context_run(struct uharddoom_context *ctx, uint32_t addr, uint32_t size) {
	int res = 0;
	unsigned long flags;
	struct uharddoom_device *dev = ctx->dev;
	struct uharddoom_job *job;
#ifdef USE_BATCH
	uint32_t off;
	void *page;
	__le32 *bjob;
#endif
	if (addr & 3 || size & 3)
		return -EINVAL;
	if (mutex_lock_interruptible(&ctx->lock))
		return -ERESTARTSYS;
	if (ctx->error) {
		res = -EIO;
		goto out;
	}
	job = kzalloc(sizeof *job, GFP_KERNEL);
	if (!job) {
		res = -ENOMEM;
		goto out;
	}
	job->ctx = ctx;
#ifdef USE_BATCH
	spin_lock_irqsave(&dev->slock, flags);
	if (!uharddoom_batch_avail(dev)) {
		uharddoom_update_wait(dev);
		while (!uharddoom_batch_avail(dev)) {
			struct uharddoom_job *fjob = list_first_entry(&dev->pending_jobs, struct uharddoom_job, lh);
			struct uharddoom_wait wait;
			uharddoom_start_wait(ctx->dev, &wait, fjob);
			spin_unlock_irqrestore(&dev->slock, flags);
			if (wait_for_completion_interruptible(&wait.completion)) {
				spin_lock_irqsave(&dev->slock, flags);
				uharddoom_abort_wait(dev, &wait);
				spin_unlock_irqrestore(&dev->slock, flags);
				kfree(job);
				res = -ERESTARTSYS;
				goto out;
			}
			spin_lock_irqsave(&dev->slock, flags);
		}
	}
	if (ctx->error) {
		spin_unlock_irqrestore(&dev->slock, flags);
		kfree(job);
		res = -EIO;
		goto out;
	}
	off = dev->batch_put_idx * UHARDDOOM_BATCH_JOB_SIZE;
	page = dev->batch_buf->pages[off >> UHARDDOOM_PAGE_SHIFT].page_cpu;
	bjob = page + (off & 0xfff);
	bjob[0] = cpu_to_le32(ctx->vs->pd_dma >> UHARDDOOM_PDP_SHIFT);
	bjob[1] = cpu_to_le32(addr);
	bjob[2] = cpu_to_le32(size);
	job->batch_pos = dev->batch_put_idx++;
	dev->batch_put_idx %= BATCH_BUF_SIZE;;
	uharddoom_iow(dev, UHARDDOOM_BATCH_PUT, dev->batch_addr + dev->batch_put_idx * UHARDDOOM_BATCH_JOB_SIZE);
	list_add_tail(&job->lh, &dev->pending_jobs);
	list_add_tail(&job->ctx_lh, &ctx->pending_jobs);
	spin_unlock_irqrestore(&dev->slock, flags);
#else
	job->cmd_ptr = addr;
	job->cmd_size = size;
	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&dev->pending_jobs)) {
		uharddoom_iow(dev, UHARDDOOM_JOB_PDP, ctx->vs->pd_dma >> UHARDDOOM_PDP_SHIFT);
		uharddoom_iow(dev, UHARDDOOM_JOB_CMD_PTR, addr);
		uharddoom_iow(dev, UHARDDOOM_JOB_CMD_SIZE, size);
		uharddoom_iow(dev, UHARDDOOM_JOB_TRIGGER, 0);
	}
	list_add_tail(&job->lh, &dev->pending_jobs);
	spin_unlock_irqrestore(&dev->slock, flags);
	ctx->pending_jobs++;
#endif
out:
	mutex_unlock(&ctx->lock);
	return res;
}

static int uharddoom_context_wait(struct uharddoom_context *ctx, int num_back) {
	int res = 0;
#ifdef USE_BATCH
	unsigned long flags;
	struct list_head *lh;
	struct uharddoom_job *job;
	struct uharddoom_wait wait;
	struct uharddoom_device *dev = ctx->dev;
#endif
	if (mutex_lock_interruptible(&ctx->lock))
		return -ERESTARTSYS;
#ifdef USE_BATCH
	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&ctx->pending_jobs)) {
		spin_unlock_irqrestore(&dev->slock, flags);
		res = ctx->error ? -EIO : 0;
		goto out;
	}
	lh = ctx->pending_jobs.next;
	while (num_back--) {
		lh = lh->next;
		if (lh == &ctx->pending_jobs) {
			spin_unlock_irqrestore(&dev->slock, flags);
			res = ctx->error ? -EIO : 0;
			goto out;
		}
	}
	job = list_entry(lh, struct uharddoom_job, ctx_lh);
	uharddoom_start_wait(ctx->dev, &wait, job);
	spin_unlock_irqrestore(&dev->slock, flags);
	if (wait_for_completion_interruptible(&wait.completion)) {
		spin_lock_irqsave(&dev->slock, flags);
		uharddoom_abort_wait(dev, &wait);
		spin_unlock_irqrestore(&dev->slock, flags);
		res = -ERESTARTSYS;
		goto out;
	}
	spin_lock_irqsave(&dev->slock, flags);
	if (ctx->error)
		res = -EIO;
	spin_unlock_irqrestore(&dev->slock, flags);
#else
	if (wait_event_interruptible(ctx->wq, ctx->pending_jobs <= num_back)) {
		res = -ERESTARTSYS;
		goto out;
	}
	if (ctx->error) {
		res = -EIO;
		goto out;
	}
#endif
out:
	mutex_unlock(&ctx->lock);
	return res;
}

static irqreturn_t uharddoom_isr(int irq, void *opaque)
{
	struct uharddoom_device *dev = opaque;
	unsigned long flags;
	uint32_t istatus;
	int i;
	spin_lock_irqsave(&dev->slock, flags);
	istatus = uharddoom_ior(dev, UHARDDOOM_INTR) & uharddoom_ior(dev, UHARDDOOM_INTR_ENABLE);
	if (istatus) {
		struct uharddoom_job *job;
		int error = 0;
		uharddoom_iow(dev, UHARDDOOM_INTR, istatus);
		if (istatus & (UHARDDOOM_INTR_FE_ERROR)) {
			printk(KERN_ALERT "uharddoom: FE_ERROR %03x %08x %08x\n",
					uharddoom_ior(dev, UHARDDOOM_FE_ERROR_CODE),
					uharddoom_ior(dev, UHARDDOOM_FE_ERROR_DATA_A),
					uharddoom_ior(dev, UHARDDOOM_FE_ERROR_DATA_B)
			);
			error = 1;
		}
		if (istatus & (UHARDDOOM_INTR_CMD_ERROR)) {
			printk(KERN_ALERT "uharddoom: CMD_ERROR\n");
			error = 1;
		}
		for (i = 0; i < 8; i++)
			if (istatus & UHARDDOOM_INTR_PAGE_FAULT(i)) {
				printk(KERN_ALERT "uharddoom: PAGE_FAULT %d %08x\n", i,
						uharddoom_ior(dev, UHARDDOOM_TLB_CLIENT_VA(i)));
				error = 1;
			}
#ifdef USE_BATCH
		uharddoom_update_wait(dev);
		if (error) {
			struct list_head *lh;
			job = list_first_entry(&dev->pending_jobs, struct uharddoom_job, lh);
			printk(KERN_ALERT "FAIL JOB %08x %08x\n", job->batch_pos * UHARDDOOM_BATCH_JOB_SIZE, dev->batch_get_idx * UHARDDOOM_BATCH_JOB_SIZE);
			job->ctx->error = 1;
			list_for_each(lh, &job->ctx->pending_jobs) {
				struct uharddoom_job *ojob = list_entry(lh, struct uharddoom_job, ctx_lh);
				uint32_t off = ojob->batch_pos * UHARDDOOM_BATCH_JOB_SIZE;
				void *page = dev->batch_buf->pages[off >> UHARDDOOM_PAGE_SHIFT].page_cpu;
				__le32 *bjob = page + (off & 0xfff);
				printk(KERN_ALERT "KILL JOB %08x\n", off);
				bjob[2] = 0;
			}
			uharddoom_iow(dev, UHARDDOOM_RESET, UHARDDOOM_RESET_ALL);
			uharddoom_iow(dev, UHARDDOOM_INTR, UHARDDOOM_INTR_MASK);
			uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL);
			uharddoom_update_wait(dev);
		}
#else
		BUG_ON(list_empty(&dev->pending_jobs));
		job = list_first_entry(&dev->pending_jobs, struct uharddoom_job, lh);
		if (error) {
			job->ctx->error = 1;
			uharddoom_iow(dev, UHARDDOOM_RESET, UHARDDOOM_RESET_ALL);
			uharddoom_iow(dev, UHARDDOOM_INTR, UHARDDOOM_INTR_MASK);
			uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL & ~UHARDDOOM_ENABLE_BATCH);
		}
		list_del(&job->lh);
		job->ctx->pending_jobs--;
		wake_up(&job->ctx->wq);
		kfree(job);
retry:
		if (!list_empty(&dev->pending_jobs)) {
			job = list_first_entry(&dev->pending_jobs, struct uharddoom_job, lh);
			if (job->ctx->error) {
				list_del(&job->lh);
				job->ctx->pending_jobs--;
				wake_up(&job->ctx->wq);
				kfree(job);
				goto retry;
			}
			uharddoom_iow(dev, UHARDDOOM_JOB_PDP, job->ctx->vs->pd_dma >> UHARDDOOM_PDP_SHIFT);
			uharddoom_iow(dev, UHARDDOOM_JOB_CMD_PTR, job->cmd_ptr);
			uharddoom_iow(dev, UHARDDOOM_JOB_CMD_SIZE, job->cmd_size);
			uharddoom_iow(dev, UHARDDOOM_JOB_TRIGGER, 0);
		} else {
			wake_up(&dev->idle_wq);
		}
#endif
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	return IRQ_RETVAL(istatus);
}

/* Buffer node handling.  */

static vm_fault_t uharddoom_buffer_fault(struct vm_fault *vmf)
{
	struct uharddoom_buffer *buf = vmf->vma->vm_file->private_data;
	unsigned long offset = vmf->pgoff << PAGE_SHIFT;
	if (offset >= buf->size)
		return VM_FAULT_SIGBUS;
	vmf->page = virt_to_page(buf->pages[offset >> UHARDDOOM_PAGE_SHIFT].page_cpu);
	get_page(vmf->page);
	return 0;
}

static const struct vm_operations_struct uharddoom_buffer_vm_ops = {
	.fault = uharddoom_buffer_fault,
};

static int uharddoom_buffer_release(struct inode *inode, struct file *file)
{
	uharddoom_buffer_put(file->private_data);
	return 0;
}

static int uharddoom_buffer_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_ops = &uharddoom_buffer_vm_ops;
	return 0;
}

static const struct file_operations uharddoom_buffer_file_ops = {
	.owner = THIS_MODULE,
	.release = uharddoom_buffer_release,
	.mmap = uharddoom_buffer_mmap,
};

/* Main device node handling.  */

static int uharddoom_open(struct inode *inode, struct file *file)
{
	struct uharddoom_device *dev = container_of(inode->i_cdev, struct uharddoom_device, cdev);
	struct uharddoom_context *ctx = uharddoom_context_create(dev);
	if (!ctx)
		return -ENOMEM;
	file->private_data = ctx;
	return nonseekable_open(inode, file);
}

static int uharddoom_release(struct inode *inode, struct file *file)
{
	struct uharddoom_context *ctx = file->private_data;
	uharddoom_context_destroy(ctx);
	return 0;
}

int uharddoom_make_fd(struct uharddoom_buffer *priv)
{
	int error, fd;
	struct file *file;

	error = get_unused_fd_flags(O_RDWR | O_CLOEXEC);
	if (error < 0)
		return error;
	fd = error;

	file = anon_inode_getfile("uharddoom_buffer", &uharddoom_buffer_file_ops, priv, O_RDWR | O_CLOEXEC);
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
	uharddoom_buffer_get(res);
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
	case UDOOMDEV_IOCTL_CREATE_BUFFER: {
		struct udoomdev_ioctl_create_buffer param;
		struct uharddoom_buffer *buffer;
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		if (param.size > 0x400000 || param.size == 0)
			return -EINVAL;
		buffer = uharddoom_buffer_create(dev, param.size);
		if (!buffer)
			return -ENOMEM;
		if ((res = uharddoom_make_fd(buffer)) < 0)
			goto buf_out_anon;
		return res;
buf_out_anon:
		uharddoom_buffer_put(buffer);
		return res;
	}
	case UDOOMDEV_IOCTL_MAP_BUFFER: {
		struct udoomdev_ioctl_map_buffer param;
		struct uharddoom_buffer *buf;
		uint32_t addr;
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		if (param.map_rdonly > 1)
			return -EINVAL;
		buf = uharddoom_get_buffer(dev, param.buf_fd);
		if (IS_ERR(buf))
			return PTR_ERR(buf);
		res = uharddoom_vspace_map(ctx->vs, buf, param.map_rdonly, &addr);
		uharddoom_buffer_put(buf);
		if (res < 0)
			return res;
		return addr;
	}
	case UDOOMDEV_IOCTL_UNMAP_BUFFER: {
		struct udoomdev_ioctl_unmap_buffer param;
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		return uharddoom_vspace_unmap(ctx->vs, param.addr);
	}
	case UDOOMDEV_IOCTL_RUN: {
		struct udoomdev_ioctl_run param;
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		return uharddoom_context_run(ctx, param.addr, param.size);
	}
	case UDOOMDEV_IOCTL_WAIT: {
		struct udoomdev_ioctl_wait param;
		if (copy_from_user(&param, parg, sizeof param))
			return -EFAULT;
		return uharddoom_context_wait(ctx, param.num_back);
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
#ifdef USE_BATCH
	INIT_LIST_HEAD(&dev->waits);
#else
	init_waitqueue_head(&dev->idle_wq);
#endif
	INIT_LIST_HEAD(&dev->pending_jobs);

	/* Allocate a free index.  */
	mutex_lock(&uharddoom_devices_lock);
	for (i = 0; i < UHARDDOOM_MAX_DEVICES; i++)
		if (!uharddoom_devices[i])
			break;
	if (i == UHARDDOOM_MAX_DEVICES) {
		err = -ENOSPC;
		mutex_unlock(&uharddoom_devices_lock);
		goto out_slot;
	}
	uharddoom_devices[i] = dev;
	dev->idx = i;
	mutex_unlock(&uharddoom_devices_lock);

	/* Enable hardware resources.  */
	if ((err = pci_enable_device(pdev)))
		goto out_enable;

	if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(40))))
		goto out_mask;
	if ((err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(40))))
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
	uharddoom_iow(dev, UHARDDOOM_FE_CODE_ADDR, 0);
	for (i = 0; i < ARRAY_SIZE(udoomfw); i++)
		uharddoom_iow(dev, UHARDDOOM_FE_CODE_WINDOW, udoomfw[i]);
	uharddoom_iow(dev, UHARDDOOM_INTR, UHARDDOOM_INTR_MASK);
	uharddoom_iow(dev, UHARDDOOM_RESET, UHARDDOOM_RESET_ALL);

#ifdef USE_BATCH
	/* Set up the kernel vspace.  */
	dev->batch_vs = uharddoom_vspace_create(dev);
	if (!dev->batch_vs) {
		err = -ENOMEM;
		goto out_vs;
	}
	/* Set up the batch buffer.  */
	dev->batch_buf = uharddoom_buffer_create(dev, BATCH_BUF_SIZE * UHARDDOOM_BATCH_JOB_SIZE);
	if (!dev->batch_buf) {
		err = -ENOMEM;
		goto out_batch;
	}
	if ((err = uharddoom_vspace_map(dev->batch_vs, dev->batch_buf, 1, &dev->batch_addr)))
		goto out_bmap;
	dev->batch_put_idx = dev->batch_get_idx = 0;
	uharddoom_iow(dev, UHARDDOOM_BATCH_PDP, dev->batch_vs->pd_dma >> UHARDDOOM_PDP_SHIFT);
	uharddoom_iow(dev, UHARDDOOM_BATCH_WRAP_TO, dev->batch_addr);
	uharddoom_iow(dev, UHARDDOOM_BATCH_WRAP_FROM, dev->batch_addr + BATCH_BUF_SIZE * UHARDDOOM_BATCH_JOB_SIZE);
	uharddoom_iow(dev, UHARDDOOM_BATCH_GET, dev->batch_addr);
	uharddoom_iow(dev, UHARDDOOM_BATCH_PUT, dev->batch_addr);
	uharddoom_iow(dev, UHARDDOOM_BATCH_WAIT, ~0xfu);
#endif
	
	/* Now bring up the device.  */
#ifdef USE_BATCH
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, UHARDDOOM_INTR_MASK & ~UHARDDOOM_INTR_JOB_DONE);
#else
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL & ~UHARDDOOM_ENABLE_BATCH);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, UHARDDOOM_INTR_MASK);
#endif

	/* We're live.  Let's export the cdev.  */
	cdev_init(&dev->cdev, &uharddoom_file_ops);
	if ((err = cdev_add(&dev->cdev, uharddoom_devno + dev->idx, 1)))
		goto out_cdev;

	/* And register it in sysfs.  */
	dev->dev = device_create(&uharddoom_class,
			&dev->pdev->dev, uharddoom_devno + dev->idx, dev,
			"udoom%d", dev->idx);
	if (IS_ERR(dev->dev)) {
		printk(KERN_ERR "uharddoom: failed to register subdevice\n");
		/* too bad. */
		dev->dev = 0;
	}

	return 0;

out_cdev:
#ifdef USE_BATCH
out_bmap:
	uharddoom_buffer_put(dev->batch_buf);
out_batch:
	uharddoom_vspace_destroy(dev->batch_vs);
out_vs:
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
#ifdef USE_BATCH
	uharddoom_buffer_put(dev->batch_buf);
	uharddoom_vspace_destroy(dev->batch_vs);
#endif
	free_irq(pdev->irq, dev);
	pci_iounmap(pdev, dev->bar);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	mutex_lock(&uharddoom_devices_lock);
	uharddoom_devices[dev->idx] = 0;
	mutex_unlock(&uharddoom_devices_lock);
	kfree(dev);
}

static int uharddoom_suspend(struct pci_dev *pdev, pm_message_t state)
{
	unsigned long flags;
	struct uharddoom_device *dev = pci_get_drvdata(pdev);
	spin_lock_irqsave(&dev->slock, flags);
#ifdef USE_BATCH
	if (!list_empty(&dev->pending_jobs)) {
		struct uharddoom_job *job = list_last_entry(&dev->pending_jobs, struct uharddoom_job, lh);
		struct uharddoom_wait wait;
		uharddoom_start_wait(dev, &wait, job);
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_for_completion(&wait.completion);
		spin_lock_irqsave(&dev->slock, flags);
	}
#else
	while (!list_empty(&dev->pending_jobs)) {
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_event(dev->idle_wq, list_empty(&dev->pending_jobs));
		spin_lock_irqsave(&dev->slock, flags);
	}
#endif
	spin_unlock_irqrestore(&dev->slock, flags);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, 0);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, 0);
	uharddoom_ior(dev, UHARDDOOM_INTR);
	return 0;
}

static int uharddoom_resume(struct pci_dev *pdev)
{
	struct uharddoom_device *dev = pci_get_drvdata(pdev);
	int i;
	uharddoom_iow(dev, UHARDDOOM_FE_CODE_ADDR, 0);
	for (i = 0; i < ARRAY_SIZE(udoomfw); i++)
		uharddoom_iow(dev, UHARDDOOM_FE_CODE_WINDOW, udoomfw[i]);
	uharddoom_iow(dev, UHARDDOOM_INTR, UHARDDOOM_INTR_MASK);
	uharddoom_iow(dev, UHARDDOOM_RESET, UHARDDOOM_RESET_ALL);
#ifdef USE_BATCH
	dev->batch_put_idx = dev->batch_get_idx = 0;
	uharddoom_iow(dev, UHARDDOOM_BATCH_PDP, dev->batch_vs->pd_dma >> UHARDDOOM_PDP_SHIFT);
	uharddoom_iow(dev, UHARDDOOM_BATCH_WRAP_TO, dev->batch_addr);
	uharddoom_iow(dev, UHARDDOOM_BATCH_WRAP_FROM, dev->batch_addr + BATCH_BUF_SIZE * UHARDDOOM_BATCH_JOB_SIZE);
	uharddoom_iow(dev, UHARDDOOM_BATCH_GET, dev->batch_addr);
	uharddoom_iow(dev, UHARDDOOM_BATCH_PUT, dev->batch_addr);
	uharddoom_iow(dev, UHARDDOOM_BATCH_WAIT, ~0xfu);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, UHARDDOOM_INTR_MASK & ~UHARDDOOM_INTR_JOB_DONE);
#else
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL & ~UHARDDOOM_ENABLE_BATCH);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, UHARDDOOM_INTR_MASK);
#endif
	return 0;
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
	.suspend = uharddoom_suspend,
	.resume = uharddoom_resume,
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
