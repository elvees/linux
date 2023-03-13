// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017-2023 RnD Center "ELVEES", JSC
 *
 * Based on tilcdc:
 * Copyright (C) 2015 Texas Instruments
 * Author: Jyri Sarha <jsarha@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_plane_helper.h>

#include "vpout-drm-drv.h"

static const uint32_t vpout_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static struct drm_plane_funcs vpout_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= drm_plane_cleanup,
	.reset		= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int vpout_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct drm_crtc_state *crtc_state;
	//bool can_position = (plane->type == DRM_PLANE_TYPE_OVERLAY);

	if (!state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state->state, state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/*
	 * - no scaling
	 * - Final coordinates must match crtc size
	 */
	return drm_atomic_helper_check_plane_state(state, crtc_state,
						DRM_PLANE_HELPER_NO_SCALING,
						DRM_PLANE_HELPER_NO_SCALING,
						false, true);
}

static void vpout_plane_atomic_update(struct drm_plane *plane,
				struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;

	if (!state->crtc)
		return;

	if (WARN_ON(!state->fb || !state->crtc->state))
		return;

	vpout_drm_crtc_update_fb(state->crtc,
			state->fb,
			state->crtc->state->event);
}

static const struct drm_plane_helper_funcs vpout_plane_helper_funcs = {
	.atomic_check = vpout_plane_atomic_check,
	.atomic_update = vpout_plane_atomic_update,
};

int vpout_plane_primary_init(struct drm_device *drm,
	struct drm_plane *plane)
{
	int ret;
	// Allow all crct's
	const uint32_t possible_crtcs = 0xff;

	ret = drm_universal_plane_init(drm, plane, possible_crtcs,
				&vpout_plane_funcs,
				vpout_formats, ARRAY_SIZE(vpout_formats),
				NULL, DRM_PLANE_TYPE_PRIMARY, NULL);

	if (ret) {
		dev_err(drm->dev, "Failed to initialize plane: %d\n", ret);
		return ret;
	}

	drm_plane_helper_add(plane, &vpout_plane_helper_funcs);

	return 0;
}

