/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
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

#ifndef _UAPI_ALIF_DRM_H_
#define _UAPI_ALIF_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

struct drm_d2d_create_bo {
	__u32 size;
	__u32 flags;
	/** Returned GEM handle for the BO. */
	__u32 handle;
	/** Physical address of contiguous buffer. */
	__u32 paddr;
};

struct drm_d2d_mmap_bo {
	/** Handle for the object being mapped. */
	__u32 handle;
	__u32 flags;
	/** offset into the drm node to use for subsequent mmap call. */
	__u64 offset;
};

struct drm_d2d_reg {
	/** Offset of register w.r.t the base. */
	__u32 offset;
	/** Value of the register. */
	__u32 value;
};

enum {
	DRM_D2D_GET_PARAMS_CLK = 0,
	DRM_D2D_GET_PARAMS_HWREVISION,
	DRM_D2D_GET_PARAMS_HAS_DLR,
};

struct drm_d2d_get_params {
	__u32 param_id;
	__u32 value;
};

/*
 * Packed display-list format (D/AVE-2D TRM 4.4.2).
 *
 * The stream is 32-bit words: an address word (up to four register
 * indices, LSB first) followed by one data word per real index.
 * Unused index slots are DRM_D2D_DLIST_PAD (0x80). Indices >= 0x80
 * are special; 0xff in the first slot makes the second slot a command:
 *  bit 0 = end of list (raises dlist-finished IRQ)
 *  bit 1 = wait FB writeback, flush caches, wait FB complete
 *  bit 2 = wait pipeline write to FB cache
 */
#define DRM_D2D_DLIST_PAD		0x80
#define DRM_D2D_DLIST_SPECIAL		0xff
#define DRM_D2D_DLIST_END		(1u << 0)
#define DRM_D2D_DLIST_FLUSH		(1u << 1)
#define DRM_D2D_DLIST_WAIT_FB		(1u << 2)

#define DRM_D2D_DLIST_ADDR_WORD(i0, i1, i2, i3) \
	(((__u32)(i0)) | ((__u32)(i1) << 8) | \
	 ((__u32)(i2) << 16) | ((__u32)(i3) << 24))

#define DRM_D2D_DLIST_ADDR_END(flags) \
	DRM_D2D_DLIST_ADDR_WORD(DRM_D2D_DLIST_SPECIAL, (flags), \
				DRM_D2D_DLIST_PAD, DRM_D2D_DLIST_PAD)

#define DRM_D2D_SUBMIT_BO_READ		(1u << 0)
#define DRM_D2D_SUBMIT_BO_WRITE		(1u << 1)
#define DRM_D2D_SUBMIT_MAX_BOS		64

struct drm_d2d_submit_bo {
	/** GEM handle of a BO the display list reads or writes. */
	__u32 handle;
	/** DRM_D2D_SUBMIT_BO_* access flags. At least one must be set. */
	__u32 flags;
};

struct drm_d2d_submit_dlist {
	/**
	 * GEM handle of the BO that holds the packed display list.
	 * It is tracked as a read access; it need not appear in @bos.
	 * The driver holds a reference to every BO until this job's
	 * dma_fence signals.
	 */
	__u32 handle;
	/** Byte offset of the first address word. Must be 4-byte aligned. */
	__u32 offset;
	/**
	 * Syncobj handle. Create it with DRM_IOCTL_SYNCOBJ_CREATE and flags 0.
	 * The job's dma_fence is installed at @point; wait with
	 * DRM_IOCTL_SYNCOBJ_TIMELINE_WAIT. Each fence carries its own errno.
	 * It signals only after the driver has written back the FB cache
	 * and invalidated the FB and texture caches, so the list need not
	 * end with DRM_D2D_DLIST_FLUSH.
	 */
	__u32 syncobj;
	/** Must be zero. */
	__u32 flags;
	/**
	 * Timeline point signaled when this job retires. Must be non-zero
	 * and greater than every point already added to @syncobj.
	 */
	__u64 point;
	/**
	 * User pointer to an array of struct drm_d2d_submit_bo: every
	 * render buffer the display list touches. The job's fence is added
	 * to each BO's reservation, so DRM_IOCTL_D2D_GEM_CPU_PREP waits for
	 * it.
	 */
	__u64 bos;
	/** Number of entries in @bos. At most DRM_D2D_SUBMIT_MAX_BOS. */
	__u32 nr_bos;
	/** Must be zero. */
	__u32 pad;
};

#define DRM_D2D_CPU_PREP_READ		(1u << 0)
#define DRM_D2D_CPU_PREP_WRITE		(1u << 1)

/*
 * Moves a BO to the CPU domain. Waits until submitted display lists no
 * longer write the BO (READ), or no longer access it at all (WRITE).
 */
struct drm_d2d_gem_cpu_prep {
	__u32 handle;
	/** DRM_D2D_CPU_PREP_* flags. At least one must be set. */
	__u32 flags;
	/**
	 * Relative timeout in ns. 0 only polls and returns -EBUSY if the
	 * GPU still owns the BO. A timed-out wait returns -ETIMEDOUT.
	 */
	__u64 timeout_ns;
};

/*
 * Ends CPU access started with DRM_IOCTL_D2D_GEM_CPU_PREP. Call it before
 * the BO is named in another submit.
 */
struct drm_d2d_gem_cpu_fini {
	__u32 handle;
	/** Must be zero. */
	__u32 flags;
};

/*
 * IOCTL types
 */
enum {
	DRM_D2D_GEM_CREATE_BO = 0,
	DRM_D2D_GEM_MMAP_BO,
	DRM_D2D_READ_REG,
	DRM_D2D_WRITE_REG,
	DRM_D2D_GET_PARAMS,
	DRM_D2D_SUBMIT_DLIST,
	DRM_D2D_GEM_CPU_PREP,
	DRM_D2D_GEM_CPU_FINI,
};

#define DRM_IOCTL_D2D_GEM_CREATE_BO \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_GEM_CREATE_BO, \
		struct drm_d2d_create_bo)
#define DRM_IOCTL_D2D_GEM_MMAP_BO \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_GEM_MMAP_BO, \
		struct drm_d2d_mmap_bo)
#define DRM_IOCTL_D2D_READ_REG	\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_READ_REG, struct drm_d2d_reg)
#define DRM_IOCTL_D2D_WRITE_REG	\
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_WRITE_REG, struct drm_d2d_reg)
#define DRM_IOCTL_D2D_GET_PARAMS \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_D2D_GET_PARAMS, \
		struct drm_d2d_get_params)
#define DRM_IOCTL_D2D_SUBMIT_DLIST \
	DRM_IOW(DRM_COMMAND_BASE + DRM_D2D_SUBMIT_DLIST, \
		struct drm_d2d_submit_dlist)
#define DRM_IOCTL_D2D_GEM_CPU_PREP \
	DRM_IOW(DRM_COMMAND_BASE + DRM_D2D_GEM_CPU_PREP, \
		struct drm_d2d_gem_cpu_prep)
#define DRM_IOCTL_D2D_GEM_CPU_FINI \
	DRM_IOW(DRM_COMMAND_BASE + DRM_D2D_GEM_CPU_FINI, \
		struct drm_d2d_gem_cpu_fini)

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_ALIF_DRM_H_ */
