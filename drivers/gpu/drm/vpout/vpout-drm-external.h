/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017 RnD Center "ELVEES", JSC
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

#ifndef __VPOUT_DRM_EXTERNAL_H__
#define __VPOUT_DRM_EXTERNAL_H__

int vpout_drm_add_external_encoders(struct drm_device *dev);

void vpout_drm_remove_external_encoders(struct drm_device *dev);

int vpout_drm_get_external_components(struct device *dev,
				      struct component_match **match);

int fixup_connectors_names(struct drm_device *drm_dev);

int vpout_drm_has_preferred_connectors(struct drm_device *drm_dev);

#endif /* __VPOUT_DRM_EXTERNAL_H__ */
