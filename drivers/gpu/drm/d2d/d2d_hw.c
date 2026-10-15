// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 * Author: Yogender Kumar Arya <yogender.kumar@alifsemi.com>
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-chain.h>
#include <linux/dma-mapping.h>
#include <linux/dma-resv.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <drm/drm_syncobj.h>

#include "alif_drm_driver.h"
#include "regs_d2d.h"

#define D2D_MAX_QUEUED_JOBS	128
#define D2D_JOB_TIMEOUT_MS	500

struct d2d_fence {
	struct dma_fence base;
	/* Protects dma_fence signaling. */
	spinlock_t lock;
};

static const char *d2d_fence_get_driver_name(struct dma_fence *fence)
{
	return "alif-d2d";
}

static const char *d2d_fence_get_timeline_name(struct dma_fence *fence)
{
	return "dlist";
}

static const struct dma_fence_ops d2d_fence_ops = {
	.use_64bit_seqno = true,
	.get_driver_name = d2d_fence_get_driver_name,
	.get_timeline_name = d2d_fence_get_timeline_name,
};

static void d2d_signal_jobs(struct list_head *jobs)
{
	struct d2d_job *job;

	list_for_each_entry(job, jobs, node) {
		if (job->error)
			dma_fence_set_error(job->fence, job->error);
		dma_fence_signal(job->fence);
		dma_fence_put(job->fence);
		job->fence = NULL;
	}
}

static int d2d_check_timeline_point(struct drm_syncobj *syncobj, u64 point)
{
	struct dma_fence *prev;
	int ret = 0;

	prev = drm_syncobj_fence_get(syncobj);
	if (!prev)
		return 0;

	/*
	 * Timeline points are the chain seqnos. A binary syncobj, or a
	 * point that does not move the timeline forward, cannot record
	 * this job on its own.
	 */
	if (!dma_fence_is_chain(prev) || prev->seqno >= point)
		ret = -EINVAL;
	dma_fence_put(prev);
	return ret;
}

static void d2d_queue_cleanup(struct d2d_device *d2d, struct list_head *retired)
{
	unsigned long flags;

	if (list_empty(retired))
		return;

	d2d_signal_jobs(retired);

	spin_lock_irqsave(&d2d->job_lock, flags);
	list_splice_tail_init(retired, &d2d->done);
	spin_unlock_irqrestore(&d2d->job_lock, flags);
	schedule_work(&d2d->job_cleanup);
}

static void d2d_kick_locked(struct d2d_device *d2d)
{
	struct d2d_job *job;

	if (d2d->running || d2d->wedged)
		return;

	job = list_first_entry_or_null(&d2d->pending, struct d2d_job, node);
	if (!job)
		return;

	list_del_init(&job->node);
	d2d->running = job;
	/*
	 * Publishing the start address starts the DLR immediately
	 * (TRM 4.4.1.7.4). BOs are write-combined, so the barrier is what
	 * makes CPU writes to the list and its buffers visible to the
	 * master bus.
	 */
	wmb();
	D2D_REG_WRITE(d2d, D2_DLISTSTART, job->paddr);
	dev_dbg(&d2d->pdev->dev, "kick dlist=0x%x status=0x%x dliststart=0x%x\n",
		job->paddr, D2D_REG_READ(d2d, D2_STATUS),
		D2D_REG_READ(d2d, D2_DLISTSTART));
	mod_delayed_work(system_wq, &d2d->job_watchdog,
			 msecs_to_jiffies(D2D_JOB_TIMEOUT_MS));
}

/*
 * The GPU caches are shared by every BO and D2_CACHECTL must not be
 * written while the DLR is busy, so CPU_PREP/CPU_FINI cannot maintain
 * them. Instead the job's fence signals only after this list has run,
 * which leaves both caches clean and empty between jobs.
 */
static void d2d_start_flush_locked(struct d2d_device *d2d)
{
	d2d->running->flushing = true;
	D2D_REG_WRITE(d2d, D2_DLISTSTART,
		      lower_32_bits(d2d->flush_dlist_dma));
}

static void d2d_retire_job_locked(struct d2d_job *job, int err,
				  struct list_head *retired)
{
	job->error = err;
	list_add_tail(&job->node, retired);
}

static void d2d_complete_running_locked(struct d2d_device *d2d, int err,
					struct list_head *retired)
{
	struct d2d_job *job = d2d->running;

	if (!job)
		return;

	d2d->running = NULL;
	d2d_retire_job_locked(job, err, retired);
}

static void d2d_fail_pending_locked(struct d2d_device *d2d, int err,
				    struct list_head *retired)
{
	struct d2d_job *job, *tmp;

	list_for_each_entry_safe(job, tmp, &d2d->pending, node) {
		list_del_init(&job->node);
		d2d_retire_job_locked(job, err, retired);
	}
}

static void d2d_job_cleanup(struct work_struct *work)
{
	struct d2d_device *d2d =
		container_of(work, struct d2d_device, job_cleanup);
	struct d2d_job *job, *tmp;
	LIST_HEAD(local);
	unsigned long flags;
	u32 i;

	spin_lock_irqsave(&d2d->job_lock, flags);
	list_splice_init(&d2d->done, &local);
	spin_unlock_irqrestore(&d2d->job_lock, flags);

	list_for_each_entry_safe(job, tmp, &local, node) {
		list_del(&job->node);
		for (i = 0; i < job->nr_bos; i++)
			drm_gem_object_put(job->bos[i]);
		kfree(job->bos);
		kfree(job);
	}
}

static unsigned int d2d_queued_jobs(struct d2d_device *d2d)
{
	unsigned int n = 0;
	struct d2d_job *job;

	if (d2d->running)
		n++;
	list_for_each_entry(job, &d2d->pending, node)
		n++;
	return n;
}

static void d2d_wedge_locked(struct d2d_device *d2d, const char *why)
{
	if (d2d->wedged)
		return;

	d2d->wedged = true;
	/*
	 * The IRQ may stay asserted if the reader is still active and the
	 * status bits cannot be cleared. Drop the line without waiting for
	 * this handler to finish.
	 */
	disable_irq_nosync(d2d->irq);
	dev_err(&d2d->pdev->dev, "%s; GPU needs a reset\n", why);
}

static void d2d_fail_active_reader(struct d2d_device *d2d, const char *why)
{
	LIST_HEAD(retired);
	unsigned long flags;

	spin_lock_irqsave(&d2d->job_lock, flags);
	cancel_delayed_work(&d2d->job_watchdog);
	d2d_wedge_locked(d2d, why);
	d2d_complete_running_locked(d2d, -EIO, &retired);
	d2d_fail_pending_locked(d2d, -EIO, &retired);
	spin_unlock_irqrestore(&d2d->job_lock, flags);
	d2d_queue_cleanup(d2d, &retired);
}

/* Reader is idle. Clear the interrupt and retire the display-list job. */
static void d2d_ack_irq(struct d2d_device *d2d, u32 status)
{
	u32 temp = D2IRQCTL_ENABLE_FINISH_DLIST | D2IRQCTL_ENABLE_BUS_ERROR;
	bool bus_error = status & D2C_IRQ_BUS_ERROR;
	unsigned long flags;
	LIST_HEAD(retired);

	if (status & D2C_IRQ_DLIST)
		temp |= D2IRQCTL_CLR_FINISH_DLIST;
	if (bus_error)
		temp |= D2IRQCTL_CLR_BUS_ERROR;
	/*
	 * Clear and re-enable the interrupts before starting another list;
	 * clearing afterwards could drop that list's finish interrupt.
	 */
	D2D_REG_WRITE(d2d, D2_IRQCTL, temp);

	if (!(status & (D2C_IRQ_DLIST | D2C_IRQ_BUS_ERROR)))
		return;

	spin_lock_irqsave(&d2d->job_lock, flags);
	if (d2d->wedged) {
		spin_unlock_irqrestore(&d2d->job_lock, flags);
		return;
	}
	if (bus_error) {
		/*
		 * TRM 6.1: a bus error stops the transfer and the core can
		 * only be recovered by reset. Do not start another list.
		 */
		cancel_delayed_work(&d2d->job_watchdog);
		d2d_wedge_locked(d2d, "AMBA bus error");
		d2d_complete_running_locked(d2d, -EIO, &retired);
		d2d_fail_pending_locked(d2d, -EIO, &retired);
	} else if (d2d->running && !d2d->running->flushing &&
		   (d2d->hwrevision & D2_HWREVISION_FEATURES_FB_CACHE)) {
		/* User list finished; write the FB cache back to DRAM. */
		d2d_start_flush_locked(d2d);
	} else {
		cancel_delayed_work(&d2d->job_watchdog);
		d2d_complete_running_locked(d2d, 0, &retired);
		d2d_kick_locked(d2d);
	}
	spin_unlock_irqrestore(&d2d->job_lock, flags);
	d2d_queue_cleanup(d2d, &retired);
}

/*
 * Finish was raised while the end command was still executing. The hard
 * IRQ has masked the level line. Wait until the reader goes idle, then
 * acknowledge. A slave write of IRQCTL before that can hang the core.
 */
static void d2d_irq_bh(struct work_struct *work)
{
	struct d2d_device *d2d = container_of(work, struct d2d_device, irq_bh);
	u32 status;
	int ret;

	ret = read_poll_timeout(D2D_REG_READ, status,
				!(status & D2C_DLISTACTIVE), 100,
				D2D_JOB_TIMEOUT_MS * 1000, false,
				d2d, D2_STATUS);
	if (ret) {
		d2d_fail_active_reader(d2d, status & D2C_IRQ_BUS_ERROR ?
				       "AMBA bus error with the display-list reader still active" :
				       "display-list reader did not go idle");
		return;
	}

	d2d_ack_irq(d2d, status);
	if (!d2d->wedged)
		enable_irq(d2d->irq);
}

/*
 * D2D device IRQ handler. The line is level triggered. A reader that is
 * still executing is handed to d2d_irq_bh; this function does not wait.
 */
static irqreturn_t d2d_irq_handler(int irq, void *arg)
{
	struct d2d_device *d2d = arg;
	u32 status;

	status = D2D_REG_READ(d2d, D2_STATUS);
	if (!(status & (D2C_IRQ_DLIST | D2C_IRQ_BUS_ERROR)))
		return IRQ_NONE;

	if (status & D2C_DLISTACTIVE) {
		if (status & D2C_IRQ_BUS_ERROR) {
			d2d_fail_active_reader(d2d,
					       "AMBA bus error with the display-list reader still active");
			return IRQ_HANDLED;
		}

		/*
		 * Mask the level line. The bottom half unmasks it after
		 * the reader goes idle and IRQCTL has been written.
		 * The kick watchdog must not run during that wait: finish
		 * has arrived, and the reader is still executing the end
		 * command.
		 */
		disable_irq_nosync(d2d->irq);
		cancel_delayed_work(&d2d->job_watchdog);
		schedule_work(&d2d->irq_bh);
		return IRQ_HANDLED;
	}

	d2d_ack_irq(d2d, status);
	return IRQ_HANDLED;
}

static void d2d_job_watchdog(struct work_struct *work)
{
	struct d2d_device *d2d =
		container_of(work, struct d2d_device, job_watchdog.work);
	LIST_HEAD(retired);
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&d2d->job_lock, flags);
	if (!d2d->running) {
		spin_unlock_irqrestore(&d2d->job_lock, flags);
		return;
	}

	status = D2D_REG_READ(d2d, D2_STATUS);
	/*
	 * Finish is already latched. The reader may still be executing
	 * the end command while the bottom half waits. Failing the job
	 * here wedges the GPU and leaves the line masked.
	 */
	if ((status & D2C_IRQ_DLIST) && !(status & D2C_IRQ_BUS_ERROR)) {
		spin_unlock_irqrestore(&d2d->job_lock, flags);
		return;
	}

	dev_err(&d2d->pdev->dev,
		"display list timed out, status=0x%x flushing=%d job=0x%x\n",
		status, d2d->running->flushing, d2d->running->paddr);
	if (status & (D2C_DLISTACTIVE | D2C_IRQ_BUS_ERROR)) {
		/*
		 * CACHECTL and DLISTSTART are slave writes. TRM 4.4.1.7.4
		 * says a slave write during an active list can drop commands
		 * or hang the core. TRM 6.1: only a reset recovers a bus error.
		 */
		d2d_wedge_locked(d2d, "display list timed out with the reader still active");
	} else if (status & D2C_CACHE_DIRTY) {
		D2D_REG_WRITE(d2d, D2_CACHECTL,
			      D2C_CACHECTL_ENABLE_FB | D2C_CACHECTL_FLUSH_FB |
			      D2C_CACHECTL_ENABLE_TX | D2C_CACHECTL_FLUSH_TX);
		/* job_lock is held, so poll atomically for up to 1 ms. */
		read_poll_timeout_atomic(D2D_REG_READ, status,
					 !(status & D2C_CACHE_DIRTY), 10, 1000,
					 false, d2d, D2_STATUS);
	}
	d2d_complete_running_locked(d2d, -ETIMEDOUT, &retired);
	d2d_fail_pending_locked(d2d, -ETIMEDOUT, &retired);
	spin_unlock_irqrestore(&d2d->job_lock, flags);
	d2d_queue_cleanup(d2d, &retired);
}

static int d2d_init_irq(struct d2d_device *d2d)
{
	struct device *dev = &d2d->pdev->dev;
	int rc;

	rc = devm_request_irq(dev, d2d->irq, d2d_irq_handler, 0,
			      dev_name(dev), d2d);
	if (rc < 0) {
		dev_dbg(dev, "Failed to register IRQ.\n");
		return rc;
	}
	/*
	 * Display-list finish and bus error cover SUBMIT_DLIST.
	 * Enumeration-done stays disabled; userspace polls that bit.
	 */
	D2D_REG_WRITE(d2d, D2_IRQCTL,
		      D2IRQCTL_ENABLE_FINISH_DLIST |
		      D2IRQCTL_ENABLE_BUS_ERROR |
		      D2IRQCTL_CLR_FINISH_ENUM |
		      D2IRQCTL_CLR_FINISH_DLIST |
		      D2IRQCTL_CLR_BUS_ERROR);

	return 0;
}

static void d2d_deinit_irq(struct d2d_device *d2d)
{
	struct platform_device *pdev = d2d->pdev;

	D2D_REG_WRITE(d2d, D2_IRQCTL, 0);
	disable_irq(d2d->irq);
	cancel_work_sync(&d2d->irq_bh);
	devm_free_irq(&pdev->dev, d2d->irq, d2d);
}

int d2d_read_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args)
{
	struct platform_device *pdev = d2d->pdev;

	if (args->offset > D2_MAXREGISTER) {
		dev_dbg(&pdev->dev, "Invalid register offset!\n");
		return -EINVAL;
	}

	args->value = D2D_REG_READ(d2d, args->offset);
	return 0;
}

int d2d_write_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args)
{
	struct platform_device *pdev = d2d->pdev;
	unsigned long flags;
	u32 status;

	if (args->offset > D2_MAXREGISTER) {
		dev_dbg(&pdev->dev, "Invalid register offset!\n");
		return -EINVAL;
	}

	/*
	 * TRM 4.4.1.7.4: writing D2_DLISTSTART starts the DLR. Kicks must
	 * go through SUBMIT_DLIST so the driver can sync the BO and wait
	 * for the dlist-finished IRQ. IRQCTL is owned by the driver.
	 * Other registers, including D2_ORIGIN, stay writable so register
	 * mode can run. Userspace polls enumeration-done itself.
	 */
	if (args->offset == D2_DLISTSTART || args->offset == D2_IRQCTL)
		return -EINVAL;

	spin_lock_irqsave(&d2d->job_lock, flags);
	status = D2D_REG_READ(d2d, D2_STATUS);
	if (d2d->running || !list_empty(&d2d->pending) ||
	    (status & D2C_DLISTACTIVE)) {
		spin_unlock_irqrestore(&d2d->job_lock, flags);
		return -EBUSY;
	}

	D2D_REG_WRITE(d2d, args->offset, args->value);
	spin_unlock_irqrestore(&d2d->job_lock, flags);
	return 0;
}

bool d2d_has_dlr(struct d2d_device *d2d)
{
	return !!(d2d->hwrevision & D2_HWREVISION_FEATURES_DLR);
}

static bool d2d_ptr_in_bos(struct drm_gem_object **bos, u32 nr, u32 addr)
{
	u32 i;

	for (i = 0; i < nr; i++) {
		struct drm_gem_dma_object *dma = to_drm_gem_dma_obj(bos[i]);
		u64 start = dma->dma_addr;
		u64 end = start + bos[i]->size;

		if ((u64)addr >= start && (u64)addr < end)
			return true;
	}
	return false;
}

/*
 * The reader walks memory until the end command (TRM 4.4.2.3). Require
 * that command inside the display-list BO, and require framebuffer and
 * texture bases to fall inside a BO named for this job.
 */
int d2d_validate_dlist(struct drm_gem_object **bos, u32 nr_bos, u32 offset)
{
	struct drm_gem_dma_object *dma = to_drm_gem_dma_obj(bos[0]);
	u32 *words = dma->vaddr;
	u32 nwords, i;

	if (!words || offset >= bos[0]->size || (offset & 3))
		return -EINVAL;

	nwords = bos[0]->size / sizeof(*words);
	i = offset / sizeof(*words);

	while (i < nwords) {
		u32 addr = words[i++];
		u8 idx[4];
		int slot;

		idx[0] = addr;
		idx[1] = addr >> 8;
		idx[2] = addr >> 16;
		idx[3] = addr >> 24;

		if (idx[0] == D2_DLIST_SPECIAL) {
			if (idx[1] & D2_DLIST_END)
				return 0;
			continue;
		}

		for (slot = 0; slot < 4; slot++) {
			u32 value;

			if (idx[slot] >= D2_DLIST_PAD)
				continue;
			if (idx[slot] == D2_DLISTSTART || idx[slot] == D2_IRQCTL)
				return -EINVAL;
			if (i >= nwords)
				return -EINVAL;
			value = words[i++];
			if ((idx[slot] == D2_ORIGIN ||
			     idx[slot] == D2_TEXORIGIN) &&
			    !d2d_ptr_in_bos(bos, nr_bos, value))
				return -EINVAL;
		}
	}

	/* The reader would continue past this BO. */
	return -EINVAL;
}

/*
 * On success the job takes ownership of @bos and of the reference held by
 * each entry. On failure both stay with the caller.
 */
int d2d_submit_dlist(struct d2d_device *d2d, struct drm_gem_object **bos,
		     const unsigned long *write, u32 nr_bos, u32 paddr,
		     struct drm_syncobj *syncobj, u64 point)
{
	struct d2d_job *job = NULL;
	struct d2d_fence *dfence = NULL;
	struct dma_fence_chain *chain = NULL;
	struct ww_acquire_ctx ctx;
	unsigned long flags;
	u32 i;
	int rc;

	if (!d2d_has_dlr(d2d))
		return -ENODEV;
	if (!syncobj || !point || !nr_bos)
		return -EINVAL;
	rc = d2d_check_timeline_point(syncobj, point);
	if (rc)
		return rc;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	dfence = kzalloc(sizeof(*dfence), GFP_KERNEL);
	chain = dma_fence_chain_alloc();
	if (!job || !dfence || !chain) {
		rc = -ENOMEM;
		goto err_free;
	}

	spin_lock_init(&dfence->lock);
	INIT_LIST_HEAD(&job->node);
	job->bos = bos;
	job->nr_bos = nr_bos;
	job->paddr = paddr;

	rc = drm_gem_lock_reservations(bos, nr_bos, &ctx);
	if (rc)
		goto err_free;

	for (i = 0; i < nr_bos; i++) {
		rc = dma_resv_reserve_fences(bos[i]->resv, 1);
		if (rc)
			goto err_unlock;
	}

	spin_lock_irqsave(&d2d->job_lock, flags);
	if (d2d->wedged) {
		spin_unlock_irqrestore(&d2d->job_lock, flags);
		rc = -EIO;
		goto err_unlock;
	}
	if (d2d_queued_jobs(d2d) >= D2D_MAX_QUEUED_JOBS) {
		spin_unlock_irqrestore(&d2d->job_lock, flags);
		rc = -ENOSPC;
		goto err_unlock;
	}

	/*
	 * The extra reference keeps the fence alive if the IRQ signals
	 * this job before drm_syncobj_add_point() publishes it. The job
	 * keeps the init reference until dma_fence_signal().
	 */
	dma_fence_init(&dfence->base, &d2d_fence_ops, &dfence->lock,
		       d2d->fence_context, ++d2d->fence_seqno);
	job->fence = dma_fence_get(&dfence->base);

	/*
	 * The fence must be in every reservation before the DLR starts;
	 * otherwise CPU_PREP could hand a BO the GPU is using to the CPU.
	 */
	for (i = 0; i < nr_bos; i++)
		dma_resv_add_fence(bos[i]->resv, &dfence->base,
				   test_bit(i, write) ? DMA_RESV_USAGE_WRITE :
							DMA_RESV_USAGE_READ);

	list_add_tail(&job->node, &d2d->pending);
	d2d_kick_locked(d2d);
	spin_unlock_irqrestore(&d2d->job_lock, flags);
	drm_gem_unlock_reservations(bos, nr_bos, &ctx);

	drm_syncobj_add_point(syncobj, chain, &dfence->base, point);
	dma_fence_put(&dfence->base);
	return 0;

err_unlock:
	drm_gem_unlock_reservations(bos, nr_bos, &ctx);
err_free:
	dma_fence_chain_free(chain);
	kfree(dfence);
	kfree(job);
	return rc;
}

void d2d_job_fini(struct d2d_device *d2d)
{
	LIST_HEAD(retired);
	unsigned long flags;

	spin_lock_irqsave(&d2d->job_lock, flags);
	d2d_complete_running_locked(d2d, -ENODEV, &retired);
	d2d_fail_pending_locked(d2d, -ENODEV, &retired);
	spin_unlock_irqrestore(&d2d->job_lock, flags);
	d2d_queue_cleanup(d2d, &retired);
	cancel_delayed_work_sync(&d2d->job_watchdog);
	cancel_work_sync(&d2d->job_cleanup);
	d2d_job_cleanup(&d2d->job_cleanup);
}

static int d2d_init_device_config_regs(struct d2d_device *d2d)
{
	struct device *dev = &d2d->pdev->dev;
	u32 temp_reg = 0;
	u32 d2d_type;

	d2d->hwrevision = D2D_REG_READ(d2d, D2_HWREVISION);
	dev_info(dev, "HW revision number - 0x%x%s", d2d->hwrevision,
		 d2d_has_dlr(d2d) ? " (DLR)" : " (no DLR)");

	/*
	 * Setup the Perf Counters.
	 */
	d2d_type = (d2d->hwrevision & D2_HWREVISION_HWTYPE_MASK) >>
		D2_HWREVISION_HWTYPE_SHIFT;
	if (d2d_type != D2_HWREVISION_HWTYPE_STANDARD &&
	    d2d_type != D2_HWREVISION_HWTYPE_LIGHT) {
		dev_info(dev, "Unknown D2D device detected. Aborting probe!\n");
		return -ENODEV;
	}

	if (d2d->hwrevision & D2_HWREVISION_FEATURES_PERFCOUNT) {
		/*
		 * Setup the perf counters only when the version of D/AVE HW is
		 * Standard and the perf counters are available in the h/w caps.
		 */
		temp_reg = ((D2PC_NONE << D2_PERFTRIGGER_1_SHIFT) &
				D2_PERFTRIGGER_1_MASK) |
			   ((D2PC_NONE << D2_PERFTRIGGER_2_SHIFT) &
				D2_PERFTRIGGER_2_MASK);
		dev_info(dev, "Setting PERFTRIGGER reg - 0x%x\n", temp_reg);
		D2D_REG_WRITE(d2d, D2_PERFTRIGGER, temp_reg);
	}

	temp_reg = 0;
	if (d2d->hwrevision & D2_HWREVISION_FEATURES_FB_CACHE)
		temp_reg |= D2C_CACHECTL_ENABLE_FB | D2C_CACHECTL_FLUSH_FB;
	if (d2d->hwrevision & D2_HWREVISION_FEATURES_TX_CACHE)
		temp_reg |= D2C_CACHECTL_ENABLE_TX | D2C_CACHECTL_FLUSH_TX;
	dev_info(dev, "CACHE-CTL reg - 0x%x\n", temp_reg);
	D2D_REG_WRITE(d2d, D2_CACHECTL, temp_reg);

	return 0;
}

u32 d2d_get_clk_rate(struct d2d_device *d2d)
{
	return d2d->clk_freq;
}

static int d2d_device_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct d2d_device *d2d;
	int rc;

	/*
	 * Allocating driver private for D2D.
	 */
	d2d = devm_kzalloc(dev, sizeof(*d2d), GFP_KERNEL);
	if (!d2d)
		return -ENOMEM;

	dev_set_drvdata(dev, d2d);
	d2d->pdev = pdev;
	spin_lock_init(&d2d->job_lock);
	INIT_LIST_HEAD(&d2d->pending);
	INIT_LIST_HEAD(&d2d->done);
	INIT_WORK(&d2d->job_cleanup, d2d_job_cleanup);
	INIT_WORK(&d2d->irq_bh, d2d_irq_bh);
	INIT_DELAYED_WORK(&d2d->job_watchdog, d2d_job_watchdog);
	d2d->fence_context = dma_fence_context_alloc(1);

	d2d->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(d2d->clk))
		return dev_err_probe(dev, PTR_ERR(d2d->clk),
				     "failed to enable GPU clock\n");
	d2d->clk_freq = clk_get_rate(d2d->clk);
	dev_info(dev, "GPU 2D clock %u Hz (enabled)\n", d2d->clk_freq);

	/*
	 * Get the IRQ for the device.
	 */
	d2d->irq = platform_get_irq(pdev, 0);
	if (d2d->irq < 0) {
		dev_dbg(dev, "Parsing IRQ from DT failed!\n");
		return d2d->irq;
	}
	dev_info(dev, "IRQ number - %d\n", d2d->irq);

	/*
	 * Get the physical I/O range of the device and map it to a virtual
	 * base pointer.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(dev, "Failed to get MMIO for D/AVE IP!\n");
		return -ENODEV;
	}
	d2d->phy_base = res->start;
	d2d->size = resource_size(res);

	d2d->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(d2d->regs)) {
		dev_dbg(dev, "Failed to map MMIO regs! - %ld\n",
			PTR_ERR(d2d->regs));
		return PTR_ERR(d2d->regs);
	}
	dev_info(dev, "Mapped IO range %pa (size - 0x%x) to %p\n",
		 &d2d->phy_base, d2d->size, d2d->regs);

	/*
	 * Init the configuration registers for the D2D device.
	 */
	rc = d2d_init_device_config_regs(d2d);
	if (rc != 0) {
		dev_dbg(dev, "Device regs config failed!\n");
		return rc;
	}

	d2d->flush_dlist = dmam_alloc_coherent(dev, 2 * sizeof(u32),
					       &d2d->flush_dlist_dma,
					       GFP_KERNEL);
	if (!d2d->flush_dlist)
		return -ENOMEM;
	/* Separate words, so the flush completes before end-of-list. */
	d2d->flush_dlist[0] = DRM_D2D_DLIST_ADDR_END(DRM_D2D_DLIST_FLUSH);
	d2d->flush_dlist[1] = DRM_D2D_DLIST_ADDR_END(DRM_D2D_DLIST_END);

	rc = d2d_init_irq(d2d);
	if (rc)
		return rc;

	dev_info(dev, "Init DRM core interface.\n");
	rc = drm_init(&pdev->dev);
	if (rc != 0) {
		dev_dbg(dev, "DRM Init failed. rc - %d", rc);
		return rc;
	}

	return 0;
}

static void d2d_device_remove(struct platform_device *pdev)
{
	struct d2d_device *d2d = platform_get_drvdata(pdev);

	drm_free(d2d->alif);
	d2d_deinit_irq(d2d);
	d2d_job_fini(d2d);
}

static const struct of_device_id of_d2d_match[] = {
	{ .compatible = D2D_DEVICE_COMPATIBLE },
	{ }
};
MODULE_DEVICE_TABLE(of, of_d2d_match);

static struct platform_driver d2d_platform_driver = {
	.driver = {
		.name = D2D_DRIVER_NAME,
		.of_match_table = of_d2d_match,
	},
	.probe = d2d_device_probe,
	.remove = d2d_device_remove,
};

module_platform_driver(d2d_platform_driver);

MODULE_DESCRIPTION("TES ALIF 2D GPU");
MODULE_AUTHOR("Yogender Kumar Arya <yogender.kumar@alifsemi.com>");
MODULE_LICENSE("GPL");
