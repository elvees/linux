/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017-2018 RnD Center "ELVEES", JSC
 *
 * Based on tilcdc:
 * Copyright (C) 2015 Texas Instruments
 * Author: Jyri Sarha <jsarha@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/component.h>
#include <linux/of_graph.h>

#include "vpout-drm-drv.h"
#include "vpout-drm-external.h"

#include "vpout-drm-link.h"

static int vpout_drm_external_mode_valid(struct drm_connector *connector,
					 struct drm_display_mode *mode)
{
	struct vpout_drm_private *priv = connector->dev->dev_private;
	int ret, i;

	ret = vpout_drm_crtc_mode_valid(priv->crtc, mode);
	if (ret != MODE_OK)
		return ret;

	/* forward call to appropriate connector */
	for (i = 0; i < priv->num_slaves; i++)
		if (priv->slaves[i].connector == connector)
			break;

	if (priv->slaves[i].funcs->mode_valid)
		return priv->slaves[i].funcs->mode_valid(connector, mode);

	return MODE_OK;
}

static void discard_all_preferred_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	list_for_each_entry(mode, &connector->probed_modes, head) {
		mode->type &= ~DRM_MODE_TYPE_PREFERRED;
	}
}

static int vpout_drm_external_get_modes(struct drm_connector *connector)
{
	struct vpout_drm_private *priv = connector->dev->dev_private;
	int count = 0;
	int i;

	/* forward call to appropriate connector */
	for (i = 0; i < priv->num_slaves; i++)
		if (priv->slaves[i].connector == connector)
			break;

	if (priv->slaves[i].funcs->get_modes)
		count = priv->slaves[i].funcs->get_modes(connector);

	/* Without discarding, a collision between connectors can occur. */
	discard_all_preferred_modes(connector);

	if (connector->cmdline_mode.specified)
		drm_set_preferred_mode(connector,
				       connector->cmdline_mode.xres,
				       connector->cmdline_mode.yres);
	return count;
}

static int
insert_proxy(struct drm_device *drm_dev, struct drm_connector *connector)
{
	struct drm_connector_helper_funcs *proxy_funcs;

	const struct drm_connector_helper_funcs *conn_funcs;
	struct vpout_drm_private *priv = drm_dev->dev_private;
	struct device *dev = drm_dev->dev;
	int idx = priv->num_slaves;

	conn_funcs = connector->helper_private;

	proxy_funcs = devm_kzalloc(dev, sizeof(*proxy_funcs), GFP_KERNEL);
	if (!proxy_funcs)
		return -ENOMEM;

	/* save original object for restore in further */
	priv->slaves[idx].funcs = conn_funcs;
	priv->slaves[idx].connector = connector;

	*proxy_funcs = *conn_funcs;

	/*
	 * insert our proxy function.
	 * This allow check drm pipeline.
	 * In latest kernel release this is integrated into the kernel.
	 */
	proxy_funcs->mode_valid = vpout_drm_external_mode_valid;

	/* This allow to mark cmdline mode as preferred */
	proxy_funcs->get_modes = vpout_drm_external_get_modes;

	drm_connector_helper_add(connector, proxy_funcs);
	priv->num_slaves++;

	dev_dbg(dev, "External encoder '%s' connected\n",
		connector->encoder->name);

	return 0;
}

int vpout_drm_add_external_encoders(struct drm_device *drm_dev)
{
	struct drm_connector *connector;
	int ret;

	mutex_lock(&drm_dev->mode_config.mutex);
	drm_for_each_connector(connector, drm_dev) {
		ret = insert_proxy(drm_dev, connector);
		if (ret) {
			mutex_unlock(&drm_dev->mode_config.mutex);
			return ret;
		}
	}
	mutex_unlock(&drm_dev->mode_config.mutex);

	/* link encoders with corresponding remote endpoints */
	return vpout_drm_link_encoders(drm_dev);
}

void vpout_drm_remove_external_encoders(struct drm_device *drm_dev)
{
	struct vpout_drm_private *priv = drm_dev->dev_private;
	int i;

	/* restore original objects */
	for (i = 0; i < priv->num_slaves; i++)
		drm_connector_helper_add(priv->slaves[i].connector,
					 priv->slaves[i].funcs);

	/* destroy links between encoders and endpoints */
	vpout_drm_unlink_all();
}

static int dev_match_of(struct device *dev, void *data)
{
	bool matched = dev->of_node == data;

	if (matched)
		vpout_drm_link_endpoint(dev);

	return matched;
}

int vpout_drm_get_external_components(struct device *dev,
				      struct component_match **match)
{
	struct device_node *ep = NULL;
	int count = 0;

	while ((ep = of_graph_get_next_endpoint(dev->of_node, ep))) {
		struct device_node *node;

		node = of_graph_get_remote_port_parent(ep);
		if (!node || !of_device_is_available(node)) {
			of_node_put(node);
			continue;
		}

		dev_dbg(dev, "Subdevice node '%s' found\n", node->name);
		if (match)
			component_match_add(dev, match, dev_match_of, node);
		of_node_put(node);
		count++;
	}

	return count;
}

static void store_options(struct drm_connector *connector, const char *option)
{
	bool ok;
	struct drm_cmdline_mode *mode = &connector->cmdline_mode;

	ok = drm_mode_parse_command_line_for_connector(option, connector, mode);
	if (!ok)
		return;
	connector->force = mode->force;
}

static int lookup_label(struct drm_connector *connector)
{
	const char *label = NULL;
	char *new_name = NULL;
	char *option = NULL;

	label = vpout_drm_get_connector_info(connector)->label;

	/* if label isn't assigned then connector name will not be changed */
	if (!label)
		return 0;

	/*
	 * Create a new connector name.
	 * New name will consist of an old name and a specified label
	 * from DTS. The <old_name>-<label> schema provides to link
	 * temporary DRM connector name and physical device which owns this
	 * connector.
	 *
	 * In some cases we want to set kernel command line options for
	 * specified device connector. Because between reloads temporary DRM
	 * connector name can change, we use constant <label>
	 * for it. So kernel command line option will look like:
	 *
	 *  video=<label>:<params>
	 *
	 * Previous name schema also available if label isn't used
	 */
	new_name = kasprintf(GFP_KERNEL, "%s-%s", connector->name, label);
	if (unlikely(!new_name))
		return -ENOMEM;

	kfree(connector->name);
	connector->name = new_name;

	/* check again if we have parameters in a kernel command line */
	if (fb_get_options(label, &option) == 0)
		store_options(connector, option);

	return 0;
}

int fixup_connectors_names(struct drm_device *drm_dev)
{
	struct drm_connector *connector;
	int ret = 0;

	mutex_lock(&drm_dev->mode_config.mutex);
	drm_for_each_connector(connector, drm_dev) {
		/* unregister connector with old name */
		drm_connector_unregister(connector);

		ret = lookup_label(connector);
		if (ret)
			break;

		/* register connector again */
		ret = drm_connector_register(connector);
		if (ret)
			break;
	}
	mutex_unlock(&drm_dev->mode_config.mutex);

	return ret;
}

int vpout_drm_has_preferred_connectors(struct drm_device *drm_dev)
{
	struct drm_connector *connector;
	int prefs = 0;

	mutex_lock(&drm_dev->mode_config.mutex);
	drm_for_each_connector(connector, drm_dev) {
		prefs += connector->cmdline_mode.specified;
	}
	mutex_unlock(&drm_dev->mode_config.mutex);

	return prefs;
}
