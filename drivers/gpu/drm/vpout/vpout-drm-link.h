/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017-2018 RnD Center "ELVEES", JSC
 * Author: Mikhail Rasputin <mrasputin@elvees.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef DRIVERS_GPU_DRM_VPOUT_VPOUT_BIND_MANAGER_H_
#define DRIVERS_GPU_DRM_VPOUT_VPOUT_BIND_MANAGER_H_

#include <linux/device.h>
#include <drm/drmP.h>

struct vpout_drm_info {
	uint32_t bpp;
	bool invert_pxl_clk;
	bool dsi;
	const char *label;
};

void vpout_drm_link_endpoint(struct device *dev);
int vpout_drm_link_encoders(struct drm_device *drm_dev);

void vpout_drm_unlink_all(void);

struct vpout_drm_info*
vpout_drm_get_encoder_info(struct drm_encoder *encoder);

struct vpout_drm_info*
vpout_drm_get_connector_info(struct drm_connector *connector);

int vpout_drm_link_init(int capacity);
void vpout_drm_link_release(void);

#endif /* DRIVERS_GPU_DRM_VPOUT_VPOUT_DRM_LINK_H_ */
