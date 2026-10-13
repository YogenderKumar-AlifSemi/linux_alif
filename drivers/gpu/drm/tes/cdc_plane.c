// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * cdc_plane.c  --  CDC Display Controller Plane
 *
 * Copyright (C) 2017 TES Electronic Solutions GmbH
 * Author: Christian Thaler <christian.thaler@tes-dst.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane.h>

#include "cdc_regs.h"
#include "cdc_drv.h"
#include "cdc_kms.h"
#include "cdc_plane.h"
#include "cdc_hw_helpers.h"

/* Constant alpha is 8-bit. This is fully opaque. */
#define CDC_ALPHA_OPAQUE 255

static struct cdc_plane *to_cdc_plane(struct drm_plane *p)
{
	return container_of(p, struct cdc_plane, plane);
}

void cdc_plane_setup_fb(struct cdc_plane *plane)
{
	struct cdc_device *cdc = plane->cdc;
	unsigned int layer = plane->hw_idx;
	struct drm_framebuffer *fb = plane->plane.state->fb;
	struct drm_gem_dma_object *gem;
	unsigned int byte_offset;
	dma_addr_t fb_addr;

	byte_offset = (plane->plane.state->src_y >> 16) * fb->pitches[0]
		+ (plane->plane.state->src_x >> 16) * fb->format->cpp[0];
	gem = drm_fb_dma_get_gem_obj(fb, 0);
	if (WARN_ON(!gem))
		return;

	fb_addr = gem->dma_addr + fb->offsets[0] + byte_offset;
	cdc_hw_set_cb_address(cdc, layer, fb_addr);
	if (cdc->dswz)
		dswz_set_fb_addr(cdc->dswz, fb_addr);
}

void cdc_plane_setup_window(struct drm_plane *plane)
{
	struct cdc_plane *cplane = to_cdc_plane(plane);
	struct cdc_device *cdc = cplane->cdc;
	unsigned int layer = cplane->hw_idx;
	const struct drm_display_mode *mode =
		&plane->state->crtc->state->adjusted_mode;
	s32 x = plane->state->crtc_x;
	s32 y = plane->state->crtc_y;
	s32 w = plane->state->crtc_w;
	s32 h = plane->state->crtc_h;

	/* Do clipping. CDC requires windows that lie inside of the screen. */
	if (x >= mode->hdisplay)
		x = mode->hdisplay - 1;
	if (y >= mode->vdisplay)
		y = mode->vdisplay - 1;
	if ((x + w) > mode->hdisplay)
		w -= x + w - mode->hdisplay;
	if ((y + h) > mode->vdisplay)
		h -= y + h - mode->vdisplay;

	dev_dbg(cdc->dev, "%s for layer %d (crtc id %d)\n",
		__func__, layer, plane->state->crtc->base.id);
	dev_dbg(cdc->dev, "setWindow(%d,%d:%dx%d)@%dx%d\n", x, y, w, h, mode->hdisplay,
		mode->vdisplay);

	cdc->planes[layer].window_width = w;
	cdc->planes[layer].window_height = h;
	cdc->planes[layer].window_x = x;
	cdc->planes[layer].window_y = y;

	cdc_hw_set_window(cdc, layer, x, y, w, h, plane->state->fb->pitches[0]);
}

int cdc_plane_disable(struct drm_plane *plane)
{
	struct cdc_plane *cplane = to_cdc_plane(plane);
	struct cdc_device *cdc = cplane->cdc;
	int layer;

	dev_dbg(cdc->dev, "%s (plane: %d)\n", __func__, cplane->hw_idx);

	if (!cplane->enabled)
		return 0;

	layer = cplane->hw_idx;
	cdc_hw_layer_set_enabled(cdc, layer, false);

	return 0;
}

static void cdc_plane_atomic_update(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *old_state;
	struct cdc_plane *cplane = to_cdc_plane(plane);
	struct cdc_device *cdc = cplane->cdc;
	struct drm_plane_state *new_state = plane->state;
	struct cdc_plane_state *old_cstate;
	struct cdc_plane_state *new_cstate;
	const struct cdc_format *fmt;
	int layer = cplane->hw_idx;

	old_state = drm_atomic_get_old_plane_state(state, plane);
	old_cstate = to_cdc_plane_state(old_state);
	new_cstate = to_cdc_plane_state(plane->state);

	if (old_cstate->alpha != new_cstate->alpha) {
		dev_dbg(cdc->dev, "Plane %d: setting alpha to %u\n", layer,
			new_cstate->alpha);
		cdc_hw_layer_set_constant_alpha(cdc, layer, new_cstate->alpha);
	}

	if (!new_state->crtc || !new_state->fb) {
		WARN_ON(new_state->crtc && !new_state->fb);
		if (old_state->crtc || new_state->crtc)
			cdc_hw_layer_set_enabled(cdc, layer, false);
		return;
	}

	fmt = cdc_format_info(new_state->fb->format->format);
	if (WARN_ON(!fmt))
		return;

	cdc_hw_set_pixel_format(cdc, layer, fmt->cdc_hw_format);

	/* CDC default config only supports CONS_ALPHA(_INV) and
	 * ALPHA_X_CONST_ALPHA(_INV).
	 */
	if (layer != 0 &&
	    new_state->fb->format->format != DRM_FORMAT_XRGB8888) {
		/* Enable pixel alpha for overlay layers only */
		cdc_hw_set_blend_mode(cdc, layer,
				      CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA,
				      CDC_BLEND_PIXEL_ALPHA_X_CONST_ALPHA_INV);
	} else {
		/* No blending for the primary layer or XRGB8888. */
		cdc_hw_set_blend_mode(cdc, layer, CDC_BLEND_CONST_ALPHA,
				      CDC_BLEND_CONST_ALPHA_INV);
	}

	cdc_plane_setup_fb(cplane);
	cdc_plane_setup_window(plane);
	cdc_hw_layer_set_enabled(cdc, layer, true);
}

static int cdc_plane_atomic_set_property(struct drm_plane *plane,
					 struct drm_plane_state *state,
					 struct drm_property *property,
					 u64 val)
{
	struct cdc_plane_state *cstate = to_cdc_plane_state(state);
	struct cdc_plane *cplane = to_cdc_plane(plane);
	struct cdc_device *cdc = cplane->cdc;

	if (property == cdc->alpha)
		cstate->alpha = val;
	else
		return -EINVAL;

	return 0;
}

static int cdc_plane_atomic_get_property(struct drm_plane *plane,
					 const struct drm_plane_state *state,
					 struct drm_property *property,
					 u64 *val)
{
	const struct cdc_plane_state *cstate =
		container_of(state, const struct cdc_plane_state, state);
	struct cdc_plane *cplane = to_cdc_plane(plane);
	struct cdc_device *cdc = cplane->cdc;

	if (property == cdc->alpha)
		*val = cstate->alpha;
	else
		return -EINVAL;

	return 0;
}

static void cdc_plane_atomic_destroy_state(struct drm_plane *plane,
					   struct drm_plane_state *state);

static void cdc_plane_reset(struct drm_plane *plane)
{
	struct cdc_plane_state *state;

	if (plane->state)
		cdc_plane_atomic_destroy_state(plane, plane->state);
	plane->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return;

	__drm_atomic_helper_plane_reset(plane, &state->state);
	state->alpha = CDC_ALPHA_OPAQUE;
}

static struct drm_plane_state *
cdc_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct cdc_plane_state *copy;

	if (WARN_ON(!plane->state))
		return NULL;

	copy = kzalloc(sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->state);
	copy->alpha = to_cdc_plane_state(plane->state)->alpha;
	return &copy->state;
}

static void cdc_plane_atomic_destroy_state(struct drm_plane *plane,
					   struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_cdc_plane_state(state));
}

static const struct drm_plane_helper_funcs cdc_plane_helper_funcs = {
	.atomic_update = cdc_plane_atomic_update,
};

static const struct drm_plane_funcs cdc_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.atomic_set_property = cdc_plane_atomic_set_property,
	.atomic_get_property = cdc_plane_atomic_get_property,
	.reset = cdc_plane_reset,
	.atomic_duplicate_state = cdc_plane_atomic_duplicate_state,
	.atomic_destroy_state = cdc_plane_atomic_destroy_state,
};

static const u32 cdc_supported_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ARGB1555,
};

int cdc_planes_init(struct cdc_device *cdc)
{
	int ret;
	int i;

	cdc->alpha = drm_property_create_range(cdc->ddev, 0, "alpha", 0,
					       CDC_ALPHA_OPAQUE);
	if (!cdc->alpha)
		return -ENOMEM;

	for (i = 0; i < cdc->hw.layer_count; ++i) {
		enum drm_plane_type type;
		struct cdc_plane *plane = &cdc->planes[i];

		if (i == 0)
			type = DRM_PLANE_TYPE_PRIMARY;
		else
			type = DRM_PLANE_TYPE_OVERLAY;

		dev_dbg(cdc->dev, "Initializing plane %d as %d type...\n", i, type);
		ret = drm_universal_plane_init(cdc->ddev, &plane->plane, 1,
					       &cdc_plane_funcs, cdc_supported_formats,
			ARRAY_SIZE(cdc_supported_formats), NULL,
			type, NULL);
		if (ret < 0) {
			dev_err(cdc->dev, "could not initialize plane %d...\n", i);
			return ret;
		}

		drm_plane_helper_add(&plane->plane, &cdc_plane_helper_funcs);

		if (type != DRM_PLANE_TYPE_OVERLAY)
			continue;

		dev_dbg(cdc->dev, "Adding alpha property to plane %d...\n", i);
		drm_object_attach_property(&plane->plane.base, cdc->alpha,
					   CDC_ALPHA_OPAQUE);
	}

	return 0;
}
