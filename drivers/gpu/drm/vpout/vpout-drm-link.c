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
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/of_graph.h>
#include <linux/bug.h>
#include <linux/types.h>
#include <linux/mempool.h>

#include "vpout-drm-link.h"

enum linkage_state {
	unguarded = 0,
	guarded
};

/*
 * linkage connects encoder, device of encoder and additional info of device
 * together
 */
struct linkage {
	struct list_head head;
	struct device *device;
	enum linkage_state state;
	struct drm_encoder *encoder;
	struct vpout_drm_info *info;
};


struct linkage_manager {
	struct list_head nodes;
	int num_nodes;
	struct mutex locker;
	mempool_t *pool;
};

static struct linkage_manager manager;

int vpout_drm_link_init(int capacity)
{
	manager.pool = mempool_create_kmalloc_pool(capacity,
						   sizeof(struct linkage));
	if (manager.pool == NULL)
		return -ENOMEM;

	manager.num_nodes = 0;
	INIT_LIST_HEAD(&manager.nodes);
	mutex_init(&manager.locker);

	return 0;
}

void vpout_drm_link_release(void)
{
	WARN_ON(manager.num_nodes != 0);
	if (unlikely(manager.num_nodes))
		vpout_drm_unlink_all();

	mempool_destroy(manager.pool);
}

static struct linkage *take_node(void)
{
	return mempool_alloc(manager.pool, GFP_KERNEL);
}

static void put_node(struct linkage *node)
{
	mempool_free(node, manager.pool);
}

struct vpout_drm_info*
vpout_drm_get_connector_info(struct drm_connector *connector)
{
	return vpout_drm_get_encoder_info(connector->encoder);
}

struct vpout_drm_info *vpout_drm_get_encoder_info(struct drm_encoder *encoder)
{
	struct linkage *node;

	mutex_lock(&manager.locker);

	list_for_each_entry(node, &manager.nodes, head) {
		if (node->encoder == encoder)
			break;
	}

	mutex_unlock(&manager.locker);

	return node->info;
}

void vpout_drm_link_endpoint(struct device *dev)
{
	struct linkage *node;
	struct linkage *avail;

	mutex_lock(&manager.locker);

	/* check that this device has not been added previously */
	list_for_each_entry(avail, &manager.nodes, head) {
		if (avail->device == dev) {
			mutex_unlock(&manager.locker);
			return;
		}
	}

	node = take_node();
	node->device = dev;

	list_add_tail(&node->head, &manager.nodes);
	manager.num_nodes++;

	mutex_unlock(&manager.locker);
}

static bool try_protect_node_device(struct linkage *node)
{
	if (!try_module_get(node->device->driver->owner))
		return 0;

	node->state = guarded;
	return 1;
}

static void unprotect_node_device(struct linkage *node)
{
	if (node->state == guarded)
		module_put(node->device->driver->owner);

	node->state = unguarded;
}

static struct device_node *match_device_endpoint(struct device *internal_dev,
				struct device *external_dev)
{
	struct device_node *ep = NULL;
	struct device_node *external_dev_node;

	for_each_endpoint_of_node(internal_dev->of_node, ep) {
		external_dev_node = of_graph_get_remote_port_parent(ep);
		of_node_put(external_dev_node);

		if (external_dev_node == external_dev->of_node)
			return ep;
	}

	return NULL;
}

static struct vpout_drm_info *get_endpoint_info(struct device_node *endpoint)
{
	struct device_node *info_np;
	struct vpout_drm_info *info;
	int ret = 0;

	if (!endpoint) {
		pr_err("%s: no device node given\n", __func__);
		return NULL;
	}

	info_np = of_get_child_by_name(endpoint, "info");
	if (!info_np) {
		pr_err("%s: could not find endpoint info node\n", __func__);
		return NULL;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		of_node_put(info_np);
		return NULL;
	}

	ret = of_property_read_u32(info_np, "bpp", &info->bpp);
	if (ret) {
		pr_err("%s: error reading endpoint info properties\n",
		       __func__);
		kfree(info);
		of_node_put(info_np);
		return NULL;
	}

	info->invert_pxl_clk = of_property_read_bool(info_np,
						     "invert-pxl-clk");
	info->label = of_get_property(info_np, "label", NULL);
	if (strcmp(info->label, "DSI") == 0)
		info->dsi = true;


	of_node_put(info_np);

	return info;
}

static inline struct drm_mode_config *get_drm_mode(struct drm_device *dev)
{
	return &dev->mode_config;
}

static void vpout_drm_unlink_safety(void)
{
	struct linkage *node;
	struct linkage *saver;

	list_for_each_entry_safe(node, saver, &manager.nodes, head) {
		unprotect_node_device(node);
		kfree(node->info);
		list_del(&node->head);
		put_node(node);
	}
}

int vpout_drm_link_encoders(struct drm_device *drm_dev)
{
	struct device_node *endpoint;
	struct linkage *node;
	struct drm_mode_config *mode;
	struct drm_encoder *encoder;
	int ret = -ENXIO;

	mode = get_drm_mode(drm_dev);

	mutex_lock(&mode->mutex);

	mutex_lock(&manager.locker);

	/* check that lists length are equal */
	if (manager.num_nodes != mode->num_encoder)
		goto out;

	/*
	 * first node in the nodes correspond to first encoder
	 * in the drm device.
	 */
	node = list_first_entry(&manager.nodes, typeof(*node), head);

	drm_for_each_encoder(encoder, drm_dev) {
		/* get the endpoint corresponding to the remote device */
		endpoint = match_device_endpoint(drm_dev->dev, node->device);
		if (!endpoint)
			goto out;

		node->info = get_endpoint_info(endpoint), of_node_put(endpoint);
		if (!node->info)
			goto out;

		node->encoder = encoder;

		if (!try_protect_node_device(node))
			goto out;

		node = list_next_entry(node, head);
	}

	ret = 0;

out:
	if (ret)
		vpout_drm_unlink_safety();

	mutex_unlock(&manager.locker);
	mutex_unlock(&mode->mutex);
	return ret;
}

void vpout_drm_unlink_all(void)
{
	mutex_lock(&manager.locker);
	vpout_drm_unlink_safety();
	manager.num_nodes = 0;
	mutex_unlock(&manager.locker);
}
