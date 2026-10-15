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

#include <linux/bitmap.h>
#include <linux/dma-resv.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wordpart.h>

#include <drm/drm_syncobj.h>

#include "alif_drm_driver.h"

int drm_d2d_gem_create_bo_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct drm_d2d_create_bo *params = data;
	struct drm_gem_dma_object *dma_obj;
	int ret;

	dma_obj = drm_gem_dma_create(dev, params->size);
	if (IS_ERR(dma_obj))
		return PTR_ERR(dma_obj);

	params->paddr = lower_32_bits(dma_obj->dma_addr);
	ret = drm_gem_handle_create(file_priv, &dma_obj->base, &params->handle);
	drm_gem_object_put(&dma_obj->base);
	return ret;
}

int drm_d2d_gem_mmap_bo_ioctl(struct drm_device *dev, void *data,
			      struct drm_file *file_priv)
{
	struct drm_d2d_mmap_bo *params = data;
	struct drm_gem_object *gem_obj;

	gem_obj = drm_gem_object_lookup(file_priv, params->handle);
	if (!gem_obj)
		return -EINVAL;

	/* The mmap offset was set up at BO allocation time. */
	params->offset = drm_vma_node_offset_addr(&gem_obj->vma_node);
	drm_gem_object_put(gem_obj);
	return 0;
}

int drm_d2d_read_reg_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct alif_dev *alif = to_alif_dev(dev);
	struct drm_d2d_reg *args = data;

	return d2d_read_reg_validate(alif->d2d, args);
}

int drm_d2d_write_reg_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct alif_dev *alif = to_alif_dev(dev);
	struct drm_d2d_reg *args = data;

	return d2d_write_reg_validate(alif->d2d, args);
}

int drm_d2d_get_params_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file_priv)
{
	struct alif_dev *alif = to_alif_dev(dev);
	struct drm_d2d_get_params *params = data;
	int rc = 0;

	switch (params->param_id) {
	case DRM_D2D_GET_PARAMS_CLK:
		params->value = d2d_get_clk_rate(alif->d2d);
		break;
	case DRM_D2D_GET_PARAMS_HWREVISION:
		params->value = alif->d2d->hwrevision;
		break;
	case DRM_D2D_GET_PARAMS_HAS_DLR:
		params->value = d2d_has_dlr(alif->d2d);
		break;
	default:
		pr_err("Invalid case!\n");
		rc = -EINVAL;
	}
	return rc;
}

int drm_d2d_submit_dlist_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct alif_dev *alif = to_alif_dev(dev);
	struct drm_d2d_submit_dlist *args = data;
	DECLARE_BITMAP(write, DRM_D2D_SUBMIT_MAX_BOS + 1);
	struct drm_d2d_submit_bo *ubos = NULL;
	struct drm_gem_object **bos;
	struct drm_gem_object *obj;
	struct drm_syncobj *syncobj;
	u32 nr = 0, i, j;
	u32 paddr;
	int ret;

	if (args->flags || args->pad || !args->point || (args->offset & 3) ||
	    args->nr_bos > DRM_D2D_SUBMIT_MAX_BOS)
		return -EINVAL;

	bos = kcalloc(args->nr_bos + 1, sizeof(*bos), GFP_KERNEL);
	if (!bos)
		return -ENOMEM;
	bitmap_zero(write, DRM_D2D_SUBMIT_MAX_BOS + 1);

	if (args->nr_bos) {
		ubos = memdup_array_user(u64_to_user_ptr(args->bos),
					 args->nr_bos, sizeof(*ubos));
		if (IS_ERR(ubos)) {
			ret = PTR_ERR(ubos);
			ubos = NULL;
			goto out_free;
		}
	}

	bos[nr] = drm_gem_object_lookup(file_priv, args->handle);
	if (!bos[nr]) {
		ret = -ENOENT;
		goto out_put;
	}
	nr++;

	if (args->offset >= bos[0]->size) {
		ret = -EINVAL;
		goto out_put;
	}

	/*
	 * A BO may appear more than once (or be the display list itself);
	 * its reservation can only be locked once, so merge the flags.
	 */
	for (i = 0; i < args->nr_bos; i++) {
		u32 bo_flags = ubos[i].flags;

		if (!bo_flags || (bo_flags & ~(DRM_D2D_SUBMIT_BO_READ |
					       DRM_D2D_SUBMIT_BO_WRITE))) {
			ret = -EINVAL;
			goto out_put;
		}

		obj = drm_gem_object_lookup(file_priv, ubos[i].handle);
		if (!obj) {
			ret = -ENOENT;
			goto out_put;
		}

		for (j = 0; j < nr; j++)
			if (bos[j] == obj)
				break;
		if (j < nr) {
			drm_gem_object_put(obj);
		} else {
			bos[nr] = obj;
			j = nr++;
		}
		if (bo_flags & DRM_D2D_SUBMIT_BO_WRITE)
			__set_bit(j, write);
	}

	syncobj = drm_syncobj_find(file_priv, args->syncobj);
	if (!syncobj) {
		ret = -ENOENT;
		goto out_put;
	}

	paddr = lower_32_bits(to_drm_gem_dma_obj(bos[0])->dma_addr +
			      args->offset);
	ret = d2d_validate_dlist(bos, nr, args->offset);
	if (ret)
		goto out_syncobj;
	ret = d2d_submit_dlist(alif->d2d, bos, write, nr, paddr, syncobj,
			       args->point);
	drm_syncobj_put(syncobj);
	if (!ret) {
		kfree(ubos);
		return 0;
	}
	goto out_put;

out_syncobj:
	drm_syncobj_put(syncobj);
out_put:
	for (i = 0; i < nr; i++)
		drm_gem_object_put(bos[i]);
out_free:
	kfree(ubos);
	kfree(bos);
	return ret;
}

int drm_d2d_gem_cpu_prep_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct drm_d2d_gem_cpu_prep *args = data;
	struct drm_gem_object *obj;
	enum dma_resv_usage usage;
	unsigned long timeout;
	long ret;

	if (!args->flags || (args->flags & ~(DRM_D2D_CPU_PREP_READ |
					     DRM_D2D_CPU_PREP_WRITE)))
		return -EINVAL;

	/* A CPU write must also wait for GPU reads of the old contents. */
	usage = (args->flags & DRM_D2D_CPU_PREP_WRITE) ?
		DMA_RESV_USAGE_READ : DMA_RESV_USAGE_WRITE;

	obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	if (!args->timeout_ns) {
		ret = dma_resv_test_signaled(obj->resv, usage) ? 0 : -EBUSY;
	} else {
		timeout = min_t(u64, nsecs_to_jiffies64(args->timeout_ns),
				MAX_SCHEDULE_TIMEOUT);
		ret = dma_resv_wait_timeout(obj->resv, usage, true, timeout);
		if (ret == 0)
			ret = -ETIMEDOUT;
		else if (ret > 0)
			ret = 0;
	}

	drm_gem_object_put(obj);
	return ret;
}

int drm_d2d_gem_cpu_fini_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct drm_d2d_gem_cpu_fini *args = data;
	struct drm_gem_object *obj;

	if (args->flags)
		return -EINVAL;

	obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	/*
	 * Nothing to maintain here. BOs come from dma_alloc_wc(), so there
	 * is no CPU cache; the wmb() before D2_DLISTSTART orders CPU writes.
	 * The GPU caches are already empty: every job's fence signals only
	 * after the driver's flush list has run.
	 */
	drm_gem_object_put(obj);
	return 0;
}

static const struct drm_ioctl_desc drm_d2d_ioctls[] = {
	DRM_IOCTL_DEF_DRV(D2D_GEM_CREATE_BO, drm_d2d_gem_create_bo_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_GEM_MMAP_BO, drm_d2d_gem_mmap_bo_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_READ_REG, drm_d2d_read_reg_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_WRITE_REG, drm_d2d_write_reg_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_GET_PARAMS, drm_d2d_get_params_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_SUBMIT_DLIST, drm_d2d_submit_dlist_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_GEM_CPU_PREP, drm_d2d_gem_cpu_prep_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(D2D_GEM_CPU_FINI, drm_d2d_gem_cpu_fini_ioctl,
			  DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_DMA_FOPS(d2d_drm_fops);

static struct drm_driver alif_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_RENDER |
				  DRIVER_SYNCOBJ | DRIVER_SYNCOBJ_TIMELINE,

	.ioctls			= drm_d2d_ioctls,
	.num_ioctls		= ARRAY_SIZE(drm_d2d_ioctls),

	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.fops			= &d2d_drm_fops,
	.name			= D2D_DRIVER_NAME,
	.desc			= "TES ALIF 2D-GPU",
	.major			= 1,
	.minor			= 0,
};

int drm_init(struct device *dev)
{
	struct drm_device *drm;
	struct alif_dev *alif;
	struct d2d_device *d2d = dev_get_drvdata(dev);
	int rc;

	alif = devm_kzalloc(dev, sizeof(*alif), GFP_KERNEL);
	if (!alif)
		return -ENOMEM;

	drm = drm_dev_alloc(&alif_drm_driver, dev);
	if (IS_ERR(drm)) {
		dev_dbg(dev, "Failed to allocate the DRM Device structure.\n");
		return PTR_ERR(drm);
	}
	alif->drm = drm;
	drm->dev_private = alif;

	alif->d2d = d2d;
	d2d->alif = alif;

	rc = drm_dev_register(alif->drm, 0);
	if (rc) {
		dev_dbg(dev, "DRM Device registration failed! rc - %d", rc);
		drm_dev_put(drm);
		return rc;
	}

	return 0;
}

void drm_free(struct alif_dev *alif)
{
	struct drm_device *drm = alif->drm;

	drm_dev_unregister(drm);

	drm->dev_private = NULL;
	alif->drm = NULL;

	drm_dev_put(drm);
}
