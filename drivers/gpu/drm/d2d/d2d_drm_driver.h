/* SPDX-License-Identifier: GPL-2.0-only */
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

#ifndef _D2D_DRM_DRIVER_H_
#define _D2D_DRM_DRIVER_H_

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_ioctl.h>

#include <uapi/drm/alif_drm.h>

#define D2D_DEVICE_COMPATIBLE	"tes,d2d-1.0"
#define D2D_DRIVER_NAME		"alif-d2d"

#define D2D_REG_READ(d2d, offset) \
	readl((d2d)->regs + ((offset) << 2))
#define D2D_REG_WRITE(d2d, offset, value) \
	writel((value), (d2d)->regs + ((offset) << 2))

struct alif_dev;
struct dma_fence;
struct drm_syncobj;

struct d2d_job {
	struct list_head node;
	/* bos[0] is the display-list BO. Each entry holds a reference. */
	struct drm_gem_object **bos;
	u32 nr_bos;
	struct dma_fence *fence;
	u32 paddr;
	int error;
	/* The user list finished; the DLR now runs flush_dlist for it. */
	bool flushing;
};

/*
 * Structure to hold the D/AVE-2D data.
 */
struct d2d_device {
	struct alif_dev *alif;
	struct platform_device *pdev;
	/* HW revision Information of the D2D device. */
	u32 hwrevision;
	/* GPU AXI/APB gate (ENSEMBLE_GPU_CLK). */
	struct clk *clk;
	/* Cached value of clock frequency. */
	u32 clk_freq;
	/* Virtual address of the D2D device after mapping. */
	void __iomem *regs;
	/* Physical address of D2D device on bus. */
	resource_size_t phy_base;
	/* Size of MMIO region for D2D device. */
	u32 size;
	/* IRQ number of the device. */
	int irq;
	/* Protects the D-List job queue and fence_seqno. */
	spinlock_t job_lock;
	/* Jobs waiting for the DLR. */
	struct list_head pending;
	/* Completed jobs waiting for GEM put (process context). */
	struct list_head done;
	/* Job currently executing on the DLR, or NULL. */
	struct d2d_job *running;
	/*
	 * Bus error or a reader that did not go idle. New display lists are
	 * refused: TRM 6.1 says only a reset recovers the core, and this SoC
	 * has no GPU reset control for the driver to toggle.
	 */
	bool wedged;
	/* dma_fence context shared by every display-list job. */
	u64 fence_context;
	/* Next dma_fence seqno. Protected by job_lock. */
	u64 fence_seqno;
	/* Drops GEM refs and frees retired jobs. */
	struct work_struct job_cleanup;
	/* Retires a job whose dlist-finished IRQ never arrived. */
	struct delayed_work job_watchdog;
	/*
	 * Retires the job after the hard IRQ has cleared the interrupt.
	 * Does not write IRQCTL.
	 */
	struct work_struct irq_bh;
	/* Latched IRQ status for irq_bh. Protected by job_lock. */
	u32 irq_status;
	/*
	 * Kernel-owned display list run after every job: writes back the
	 * FB cache and invalidates the FB and texture caches.
	 */
	u32 *flush_dlist;
	dma_addr_t flush_dlist_dma;
};

struct alif_dev {
	struct drm_device *drm;

	/* D2D HW config structure. */
	struct d2d_device *d2d;
};

static inline struct alif_dev *to_alif_dev(struct drm_device *drm)
{
	return drm->dev_private;
}

static inline struct device *to_device(struct alif_dev *alif)
{
	struct platform_device *pdev = alif->d2d->pdev;

	return &pdev->dev;
}

int drm_init(struct device *dev);
void drm_free(struct alif_dev *alif);

/*
 * IOCTLs for Buffer Object creation and manipulation.
 */
int drm_d2d_gem_create_bo_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv);
int drm_d2d_gem_mmap_bo_ioctl(struct drm_device *dev, void *data,
			      struct drm_file *file_priv);

/*
 * IOCTLs for register read/write and generic parameters retrieval from
 * driver (clk, perf, ...).
 */
int drm_d2d_read_reg_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv);
int drm_d2d_write_reg_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file_priv);
int drm_d2d_get_params_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file_priv);
int drm_d2d_submit_dlist_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv);
int drm_d2d_gem_cpu_prep_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv);
int drm_d2d_gem_cpu_fini_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv);

int d2d_read_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args);
int d2d_write_reg_validate(struct d2d_device *d2d, struct drm_d2d_reg *args);
u32 d2d_get_clk_rate(struct d2d_device *d2d);
bool d2d_has_dlr(struct d2d_device *d2d);
int d2d_submit_dlist(struct d2d_device *d2d, struct drm_gem_object **bos,
		     const unsigned long *write, u32 nr_bos, u32 paddr,
		     struct drm_syncobj *syncobj, u64 point);
int d2d_validate_dlist(struct drm_gem_object **bos, u32 nr_bos, u32 offset);
void d2d_job_fini(struct d2d_device *d2d);

#endif /* _D2D_DRM_DRIVER_H_ */
