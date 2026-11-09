// SPDX-License-Identifier: GPL-2.0
// Copyright 2026 RnD Center "ELVEES", JSC

#include <linux/component.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <drm/drm_panel.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>

static int mcom03_drm_bind(struct device *dev, struct device *master,
			   void *data)
{
	return 0;
}

static void mcom03_drm_unbind(struct device *dev, struct device *master,
			      void *data)
{

}

static int mcom03_find_panel_or_bridge(const struct device_node *np,
				       struct drm_panel **panel,
				       struct drm_bridge **bridge)
{
	int ret = -EPROBE_DEFER;

	if (!panel && !bridge)
		return -EINVAL;
	if (panel)
		*panel = NULL;

	if (!of_graph_is_present(np))
		return -ENODEV;

	if (panel) {
		*panel = of_drm_find_panel(np);
		if (!IS_ERR(*panel))
			ret = 0;
		else
			*panel = NULL;
	}

	if (bridge) {
		if (ret) {
			/* No panel found yet, check for a bridge next. */
			*bridge = of_drm_find_bridge(np);
			if (*bridge)
				ret = 0;
		} else {
			*bridge = NULL;
		}

	}

	return ret;
}

static const struct component_ops mcom03_drm_ops = {
	.bind	= mcom03_drm_bind,
	.unbind	= mcom03_drm_unbind,
};

static int mcom03_drm_check_bridge_chain(struct device_node *np)
{
	struct device_node *cur_remote = of_node_get(np);

	while (cur_remote) {
		struct device_node *next_remote = NULL;
		struct drm_panel *panel;
		struct drm_bridge *bridge;
		int ret;

		ret = mcom03_find_panel_or_bridge(cur_remote, &panel, &bridge);
		if (ret) {
			of_node_put(cur_remote);
			return ret;
		}

		if (panel) {
			of_node_put(cur_remote);
			break;
		}

		next_remote = of_graph_get_remote_node(cur_remote, 1, 0);
		if (!next_remote)
			next_remote = of_graph_get_remote_node(cur_remote, 2, 0);

		of_node_put(cur_remote);
		cur_remote = next_remote;
	}

	return 0;
}

static int mcom03_drm_probe(struct platform_device *pdev)
{
	struct device_node *dp_np, *ep;
	int ret = 0;

	dp_np = of_graph_get_remote_node(pdev->dev.of_node, 0, 0);
	if (!dp_np)
		return -ENODEV;

	for_each_endpoint_of_node(dp_np, ep) {
		struct device_node *remote = of_graph_get_remote_port_parent(ep);

		if (!remote || remote == pdev->dev.of_node || !of_device_is_available(remote)) {
			dev_dbg(&pdev->dev, "Skip node %s", remote->full_name);
			of_node_put(remote);
			continue;
		}

		ret = mcom03_drm_check_bridge_chain(remote);
		of_node_put(remote);

		if (ret) {
			of_node_put(ep);
			break;
		}
	}

	of_node_put(dp_np);

	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to check bridge chain\n");

	ret = component_add(&pdev->dev, &mcom03_drm_ops);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register our component\n");
		return ret;
	}

	return 0;
}

static void mcom03_drm_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mcom03_drm_ops);
}

static const struct of_device_id mcom03_drm_dt_ids[] = {
	{ .compatible = "elvees,mcom03-drm", },
	{ },
};
MODULE_DEVICE_TABLE(of, mcom03_drm_dt_ids);

struct platform_driver mcom03_drm_driver = {
	.probe  = mcom03_drm_probe,
	.remove = mcom03_drm_remove,
	.driver = {
		.name = "mcom03-drm",
		.of_match_table = mcom03_drm_dt_ids
	},
};

module_platform_driver(mcom03_drm_driver);
