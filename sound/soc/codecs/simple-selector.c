// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 * Author: Ilya Popov <ipopov@evlees.com>
 *
 * Should be bidirectional channel selector at least for 1 channel.
 *
 *            -------------     ----------
 * -------    | Channel 1 |<--->| codec1 |
 * | cpu |<-->|           |     |--------|
 * -------    | Channel 2 |<--->| codec2 |
 *            -------------     ----------
 *
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>

struct simple_selector {
	struct gpio_desc *gpiod_select;
	unsigned int select;
};

static const char * const simple_selector_texts[] = {
	"Channel 1", "Channel 2"
};

static SOC_ENUM_SINGLE_EXT_DECL(simple_selector_enum, simple_selector_texts);

static int simple_selector_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *c = snd_soc_dapm_to_component(dapm);
	struct simple_selector *priv = snd_soc_component_get_drvdata(c);

	ucontrol->value.enumerated.item[0] = priv->select;

	return 0;
}

static int simple_selector_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_component *c = snd_soc_dapm_to_component(dapm);
	struct simple_selector *priv = snd_soc_component_get_drvdata(c);

	if (ucontrol->value.enumerated.item[0] > e->items)
		return -EINVAL;

	if (priv->select == ucontrol->value.enumerated.item[0])
		return 0;

	priv->select = ucontrol->value.enumerated.item[0];

	gpiod_set_value_cansleep(priv->gpiod_select, priv->select);

	return snd_soc_dapm_mux_update_power(dapm, kcontrol,
					     ucontrol->value.enumerated.item[0],
					     e, NULL);
}

static unsigned int simple_selector_read(struct snd_soc_component *component,
		unsigned int reg)
{
	struct simple_selector *priv = snd_soc_component_get_drvdata(component);

	return priv->select;
}

static const struct snd_kcontrol_new simple_selector_ctrl =
	SOC_DAPM_ENUM_EXT("Selector", simple_selector_enum, simple_selector_control_get,
		simple_selector_control_put);

static const struct snd_soc_dapm_widget simple_selector_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Channel1 OUT"),
	SND_SOC_DAPM_OUTPUT("Channel2 OUT"),
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_DEMUX("Sel demux", SND_SOC_NOPM, 0, 0, &simple_selector_ctrl),

	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_INPUT("Channel1 IN"),
	SND_SOC_DAPM_INPUT("Channel2 IN"),
	SND_SOC_DAPM_MUX("Sel mux", SND_SOC_NOPM, 0, 0, &simple_selector_ctrl),
};

static const struct snd_soc_dapm_route simple_selector_dapm_routes[] = {
	{ "Sel demux", NULL, "IN" },
	{ "Channel1 OUT", "Channel 1", "Sel demux" },
	{ "Channel2 OUT", "Channel 2", "Sel demux" },

	{ "OUT", NULL, "Sel mux" },
	{ "Sel mux", "Channel 1", "Channel1 IN"},
	{ "Sel mux", "Channel 2", "Channel2 IN"},
};

static const struct snd_soc_component_driver simple_selector_component_driver = {
	.dapm_widgets		= simple_selector_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(simple_selector_dapm_widgets),
	.dapm_routes		= simple_selector_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(simple_selector_dapm_routes),
	.read			= simple_selector_read,
};

static int simple_selector_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simple_selector *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	priv->gpiod_select = devm_gpiod_get(dev, "select", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_select))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_select),
				"Failed to get 'select' gpio");

	return devm_snd_soc_register_component(dev, &simple_selector_component_driver, NULL, 0);
}

#ifdef CONFIG_OF
static const struct of_device_id simple_selector_ids[] = {
	{ .compatible = "simple-audio-selector", },
	{ }
};
MODULE_DEVICE_TABLE(of, simple_selector_ids);
#endif

static struct platform_driver simple_selector_driver = {
	.driver = {
		.name = "simple-selector",
		.of_match_table = of_match_ptr(simple_selector_ids),
	},
	.probe = simple_selector_probe,
};

module_platform_driver(simple_selector_driver);

MODULE_DESCRIPTION("ASoC Simple Audio selector driver");
MODULE_AUTHOR("Ilya Popov <ipopov@elvees.com>");
MODULE_LICENSE("GPL");
