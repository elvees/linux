// SPDX-License-Identifier: GPL-2.0
/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017-2018 RnD Center "ELVEES", JSC
 *
 * Based on tilcdc:
 * Copyright (C) 2012 Texas Instruments
 * Author: Rob Clark <robdclark@gmail.com>
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

#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/types.h>

#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>

#include "vpout-drm-drv.h"
#include "vpout-drm-external.h"
#include "vpout-drm-link.h"

static struct drm_framebuffer *
vpout_drm_fb_create(struct drm_device *drm_dev, struct drm_file *file_priv,
		    const struct drm_mode_fb_cmd2 *mode_cmd)
{
	return drm_gem_fb_create(drm_dev, file_priv, mode_cmd);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = vpout_drm_fb_create,
};

static int
of_get_child_count_by_name(struct device_node *parent, const char *name)
{
	struct device_node *child;
	int count = 0;

	for_each_child_of_node(parent, child)
		if (child->name && (of_node_cmp(child->name, name) == 0))
			count++;

	return count;
}

static struct device_node *get_crtc_port(struct device *dev)
{
	struct device_node *parent, *ports, *port;
	int count;

	parent = dev->of_node;

	ports = of_get_child_by_name(parent, "ports");
	if (ports)
		parent = ports;

	count = of_get_child_count_by_name(parent, "port");

	if (count > 1) {
		dev_err(dev,
			"The CRTC supports only one port for all endpoints\n");
		of_node_put(ports);
		return NULL;
	}

	port = of_get_child_by_name(parent, "port");

	of_node_put(ports);

	return port;
}

uint vpout_drm_get_preferred_bpp(struct drm_device *drm_dev)
{
	uint bpp = 32;
	struct drm_connector *con;
	struct drm_connector_list_iter iter;

	drm_connector_list_iter_begin(drm_dev, &iter);
	drm_for_each_connector_iter(con, &iter) {
		if (con->cmdline_mode.specified) {
			/* get bpp for preferred connector */
			bpp = vpout_drm_get_connector_info(con)->bpp;
			break;
		}
	}
	drm_connector_list_iter_end(&iter);

	return bpp;
}

/*
 * DRM operations:
 */

static int vpout_init(struct drm_driver *drv, struct device *dev)
{
	struct drm_device *drm_dev;
	struct platform_device *plat_dev = to_platform_device(dev);
	struct vpout_drm_private *priv;
	struct device_node *port;
	struct resource *res;
	uint bpp;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm_dev = drm_dev_alloc(drv, dev);
	if (IS_ERR(drm_dev))
		return PTR_ERR(drm_dev);

	drm_dev->dev_private = priv;

	priv->wq = alloc_ordered_workqueue("elvees-vpout-drm", 0);
	if (!priv->wq) {
		ret = -ENOMEM;
		goto fail_free_drmdev;
	}

	res = platform_get_resource_byname(plat_dev, IORESOURCE_MEM, "lcd");

	priv->mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->mmio)) {
		dev_err(dev, "failed to map LCD registers\n");
		ret = PTR_ERR(priv->mmio);
		goto fail_free_wq;
	}

	res = platform_get_resource_byname(plat_dev, IORESOURCE_MEM, "dsi");

	priv->dsi = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->dsi)) {
		dev_err(dev, "failed to map DSI registers\n");
		ret = PTR_ERR(priv->dsi);
		goto fail_free_wq;
	}

	priv->smctr = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "elvees,smctr");
	if (IS_ERR(priv->smctr)) {
		dev_err(dev, "failed to get SMCTR regmap\n");
		ret = PTR_ERR(priv->smctr);
		goto fail_free_wq;
	}

	priv->clk = devm_clk_get(dev, 0);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "failed to get clock\n");
		ret = PTR_ERR(priv->clk);
		goto fail_free_wq;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable clock\n");
		goto fail_free_wq;
	}

	drm_mode_config_init(drm_dev);

	priv->crtc = vpout_drm_crtc_create(drm_dev);

	if (!priv->crtc) {
		ret = -ENXIO;
		goto fail_mode_config_cleanup;
	}

	port = get_crtc_port(dev);
	if (!port) {
		ret = -ENXIO;
		goto fail_mode_config_cleanup;
	}

	priv->crtc->port = port;
	of_node_put(port);

	drm_dev->mode_config.min_width = 0;
	drm_dev->mode_config.min_height = 0;
	drm_dev->mode_config.max_width = 2048;
	drm_dev->mode_config.max_height = 2048;
	drm_dev->mode_config.funcs = &mode_config_funcs;

	platform_set_drvdata(plat_dev, drm_dev);

	/*
	 * Now we enum all slave device. In this time
	 * they all present in system and probed.
	 */
	ret = component_bind_all(dev, drm_dev);
	if (ret < 0)
		goto fail_mode_config_cleanup;

	ret = vpout_drm_add_external_encoders(drm_dev);
	if (ret < 0)
		goto fail_component_cleanup;

	if (priv->num_slaves == 0) {
		dev_err(dev, "no encoders/connectors found\n");
		ret = -ENXIO;
		goto fail_external_cleanup;
	}

	/* fixup names and lookup cmdline options if connectors have labels */
	ret = fixup_connectors_names(drm_dev);
	if (ret) {
		dev_err(dev, "failed to fixup connectors names\n");
		goto fail_external_cleanup;
	}

	/*
	 * Query preferred connectors.
	 * It can be made after lookup cmdline options.
	 * Preferred connector should be assigned through 'video' parameter.
	 * In any time should be only one preferred connector.
	 */
	if (vpout_drm_has_preferred_connectors(drm_dev) != 1) {
		dev_err(dev, "failed to select preferred connector\n");
		goto fail_external_cleanup;
	}

	ret = drm_vblank_init(drm_dev, 1);
	if (ret < 0) {
		dev_err(dev, "failed to initialize vblank\n");
		goto fail_external_cleanup;
	}

	ret = drm_irq_install(drm_dev, platform_get_irq(plat_dev, 0));
	if (ret < 0) {
		dev_err(dev, "failed to install IRQ handler\n");
		goto fail_external_cleanup;
	}

	ret = drm_dev_register(drm_dev, 0);
	if (ret)
		goto fail_irq_uninstall;

	drm_kms_helper_poll_init(drm_dev);

	bpp = vpout_drm_get_preferred_bpp(drm_dev);
	drm_fbdev_generic_setup(drm_dev, bpp);

	return 0;

fail_irq_uninstall:
	drm_irq_uninstall(drm_dev);

fail_external_cleanup:
	vpout_drm_remove_external_encoders(drm_dev);

fail_component_cleanup:
	component_unbind_all(dev, drm_dev);

fail_mode_config_cleanup:
	drm_mode_config_cleanup(drm_dev);
	clk_disable_unprepare(priv->clk);

fail_free_wq:
	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

fail_free_drmdev:
	drm_dev_put(drm_dev);
	drm_dev->dev_private = NULL;

	return ret;
}

static void vpout_fini(struct drm_device *drm_dev)
{
	struct vpout_drm_private *priv = drm_dev->dev_private;

	vpout_drm_remove_external_encoders(drm_dev);

	drm_dev_unregister(drm_dev);
	drm_kms_helper_poll_fini(drm_dev);
	drm_mode_config_cleanup(drm_dev);
	component_unbind_all(drm_dev->dev, drm_dev);

	drm_irq_uninstall(drm_dev);

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	clk_disable_unprepare(priv->clk);
	drm_dev->dev_private = NULL;
	drm_dev_put(drm_dev);
}

static irqreturn_t vpout_drm_irq(int irq, void *arg)
{
	struct drm_device *drm_dev = arg;
	struct vpout_drm_private *priv = drm_dev->dev_private;

	return vpout_drm_crtc_irq(priv->crtc);
}

DEFINE_DRM_GEM_CMA_FOPS(fops);

static struct drm_driver vpout_drm_driver = {
	.driver_features    = DRIVER_HAVE_IRQ | DRIVER_GEM | DRIVER_MODESET,
	.irq_handler        = vpout_drm_irq,
	.gem_free_object    = drm_gem_cma_free_object,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import   = drm_gem_prime_import,
	.gem_prime_export   = drm_gem_prime_export,
	.gem_prime_get_sg_table    = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap     = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap   = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap     = drm_gem_cma_prime_mmap,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.dumb_create        = drm_gem_cma_dumb_create,
	.fops               = &fops,
	.name               = "vpout-drm",
	.desc               = "ELVEES VPOUT Controller DRM",
	.date               = "20170825",
	.major              = 1,
	.minor              = 0,
};

/*
 * Platform driver:
 */

static int vpout_drm_bind(struct device *dev)
{
	return vpout_init(&vpout_drm_driver, dev);
}

static void vpout_drm_unbind(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	/* Check if a subcomponent has already triggered the unloading. */
	if (!drm_dev->dev_private)
		return;

	vpout_fini(drm_dev);
}

static const struct component_master_ops vpout_drm_comp_ops = {
	.bind = vpout_drm_bind,
	.unbind = vpout_drm_unbind,
};

static int vpout_drm_probe(struct platform_device *plat_dev)
{
	struct component_match *match = NULL;
	int count;
	int ret;

	if (!plat_dev->dev.of_node) {
		dev_err(&plat_dev->dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	count = vpout_drm_get_external_components(&plat_dev->dev, &match);
	if (count < 0)
		return count;

	ret = vpout_drm_link_init(count);
	if (ret)
		return ret;

	return component_master_add_with_match(&plat_dev->dev,
					       &vpout_drm_comp_ops, match);
}

static int vpout_drm_remove(struct platform_device *plat_dev)
{
	component_master_del(&plat_dev->dev, &vpout_drm_comp_ops);

	vpout_drm_link_release();

	return 0;
}

static const struct of_device_id vpout_drm_of_match[] = {
	{ .compatible = "elvees,mcom02-vpout", },
	{ },
};
MODULE_DEVICE_TABLE(of, vpout_drm_of_match);

static struct platform_driver vpout_drm_platform_driver = {
	.probe = vpout_drm_probe,
	.remove = vpout_drm_remove,
	.driver = {
		.name = "vpout-drm",
		.of_match_table = vpout_drm_of_match,
	},
};

static int __init vpout_drm_init(void)
{
	return platform_driver_register(&vpout_drm_platform_driver);
}

static void __exit vpout_drm_fini(void)
{
	platform_driver_unregister(&vpout_drm_platform_driver);
}

module_init(vpout_drm_init);
module_exit(vpout_drm_fini);

MODULE_AUTHOR("Alexey Kiselev <akiselev@elvees.com");
MODULE_DESCRIPTION("ELVEES VPOUT Controller DRM Driver");
MODULE_LICENSE("GPL");
