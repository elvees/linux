// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 */

#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/input-event-codes.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/arizona.h"
#include "../codecs/wm8998.h"

#define MCLK1_FREQ 12288000

#define MODEM_TDM_TX_MASK 1
#define MODEM_TDM_RX_MASK 1
#define MODEM_TDM_SLOTS 1
#define MODEM_TDM_SLOT_WIDTH 128

struct trustphone_wm8998_priv {
	struct device *dev;
};

static const struct snd_kcontrol_new trustphone_wm8998_controls[] = {
	SOC_DAPM_PIN_SWITCH("Modem In"),
	SOC_DAPM_PIN_SWITCH("Modem Out"),
};

static const struct snd_soc_dapm_widget trustphone_wm8998_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Modem In", NULL),
	SND_SOC_DAPM_LINE("Modem Out", NULL),
};

static int trustphone_wm8998_modem_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->codec_dai;
	struct snd_soc_component *component = dai->component;
	int ret;

	ret = snd_soc_component_set_sysclk(component, ARIZONA_CLK_SYSCLK,
					   ARIZONA_CLK_SRC_MCLK1, MCLK1_FREQ,
					   SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP) {
		dev_err(dai->dev, "Failed to set sysclk source\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(dai, ARIZONA_CLK_SYSCLK, MCLK1_FREQ,
				     SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP) {
		dev_err(dai->dev, "Failed to set sysclk for DAI %d\n",
			dai->id);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(dai, MODEM_TDM_TX_MASK,
				       MODEM_TDM_RX_MASK,
				       MODEM_TDM_SLOTS,
				       MODEM_TDM_SLOT_WIDTH);
	if (ret && ret != -ENOTSUPP) {
		dev_err(dai->dev, "Failed to set TDM slots\n");
		return ret;
	}

	return 0;
}

static const struct snd_soc_pcm_stream trustphone_wm8998_modem_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
};

static const struct snd_soc_dapm_widget trustphone_wm8998_modem_widgets[] = {
	SND_SOC_DAPM_INPUT("Modem RX"),
	SND_SOC_DAPM_OUTPUT("Modem TX"),
};

static const struct snd_soc_dapm_route trustphone_wm8998_modem_routes[] = {
	{ "Modem Capture", NULL, "Modem RX" },
	{ "Modem TX", NULL, "Modem Playback" },
};

static const struct snd_soc_component_driver trustphone_wm8998_component = {
	.name			= "trustphone-wm8998",
	.dapm_widgets		= trustphone_wm8998_modem_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(trustphone_wm8998_modem_widgets),
	.dapm_routes		= trustphone_wm8998_modem_routes,
	.num_dapm_routes	= ARRAY_SIZE(trustphone_wm8998_modem_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static struct snd_soc_dai_driver trustphone_wm8998_modem_dai[] = {
	{
		.name = "Voice call",
		.playback = {
			.stream_name = "Modem Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rate_min = 8000,
			.rate_max = 8000,
			.rates = SNDRV_PCM_RATE_8000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "Modem Capture",
			.channels_min = 1,
			.channels_max = 1,
			.rate_min = 8000,
			.rate_max = 8000,
			.rates = SNDRV_PCM_RATE_8000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

static struct snd_soc_dai_link trustphone_wm8998_dai[] = {
	{
		.name = "WM8998 AIF2",
		.stream_name = "Modem",
		.cpu_dai_name = "Voice call",
		.codec_dai_name = "wm8998-aif2",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBM_CFM |
			   SND_SOC_DAIFMT_IB_NF,
		.init = &trustphone_wm8998_modem_init,
		.params = &trustphone_wm8998_modem_params,
		.ignore_suspend = 1,
	},
};

static struct snd_soc_card trustphone_wm8998_card = {
	.name = "Trustphone WM8998",
	.owner = THIS_MODULE,
	.dai_link = trustphone_wm8998_dai,
	.num_links = ARRAY_SIZE(trustphone_wm8998_dai),
	.controls = trustphone_wm8998_controls,
	.num_controls = ARRAY_SIZE(trustphone_wm8998_controls),
	.dapm_widgets = trustphone_wm8998_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(trustphone_wm8998_dapm_widgets),
};

static int trustphone_wm8998_probe(struct platform_device *pdev)
{
	struct trustphone_wm8998_priv *priv;
	struct snd_soc_card *card = &trustphone_wm8998_card;
	struct device_node *codec;
	struct snd_soc_dai_link *dai_link;
	struct device *dev = &pdev->dev;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	snd_soc_card_set_drvdata(card, priv);
	card->dev = dev;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret < 0) {
		dev_err(dev, "Failed to get card name\n");
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "elvees,audio-routing");
	if (ret < 0) {
		dev_err(dev, "Failed to get audio routing\n");
		return ret;
	}

	codec = of_get_child_by_name(dev->of_node, "codec");
	if (!codec)
		return -EINVAL;

	for (i = 0; i < card->num_links; i++) {
		dai_link = &card->dai_link[i];
		dai_link->codec_of_node = of_parse_phandle(codec,
							   "sound-dai", 0);
		if (!dai_link->codec_of_node) {
			ret = -EINVAL;
			goto out;
		}
	}

	ret = devm_snd_soc_register_component(dev,
					      &trustphone_wm8998_component,
					      trustphone_wm8998_modem_dai,
					      ARRAY_SIZE(trustphone_wm8998_modem_dai));
	if (ret < 0) {
		dev_err(dev, "Failed to register component: %d\n", ret);
		goto out;
	}

	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		dev_err(dev, "Failed to register card %d\n", ret);

out:
	of_node_put(codec);

	return ret;
}

static const struct of_device_id trustphone_wm8998_of_match[] = {
	{ .compatible = "elvees,trustphone-wm8998" },
	{ },
};
MODULE_DEVICE_TABLE(of, trustphone_wm8998_of_match);

static struct platform_driver trustphone_wm8998_driver = {
	.driver = {
		.name		= "trustphone-wm8998",
		.pm		= &snd_soc_pm_ops,
		.of_match_table	= trustphone_wm8998_of_match,
	},
	.probe	= trustphone_wm8998_probe,
};
module_platform_driver(trustphone_wm8998_driver);

MODULE_DESCRIPTION("ELVEES Trustphone WM8998 soundcard");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: arizona-i2c");
