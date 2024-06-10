// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * ALSA SoC ELVEES MFBSP I2S Interface driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mfbsp.h"

static void mfbsp_i2s_start(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct mfbsp_data *mfbsp = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		mfbsp_writel(mfbsp->base, MFBSP_I2S_TSTART, 1);
	else {
		if (mfbsp->bclk_provider || mfbsp->frame_provider)
			mfbsp_writel(mfbsp->base, MFBSP_I2S_TSTART, 1);
		mfbsp_writel(mfbsp->base, MFBSP_I2S_RSTART, 1);
	};
}

static void mfbsp_i2s_stop(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct mfbsp_data *mfbsp = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*
		 * MFBSP I2S slave mode: unconditionally turn off MFBSP I2S transmitter.
		 * MFBSP I2S master mode: turn off the transmitter only if CAPTURE is
		 * not active, because during CAPTURE, MFBSP I2S receiver is clocked from
		 * transmitter clock generator, which will be turned off if the transmitter
		 * is turned off.
		*/
		if (!mfbsp->active || !(mfbsp->bclk_provider || mfbsp->frame_provider))
			mfbsp_writel(mfbsp->base, MFBSP_I2S_TSTART, 0);
	} else {
		mfbsp_writel(mfbsp->base, MFBSP_I2S_RSTART, 0);
		/*
		 * MFBSP I2S slave mode: don't touch transmitter registers.
		 * MFBSP I2S master mode: turn off the transmitter if PLAYBACK is not active.
		 */
		if (!mfbsp->active && (mfbsp->bclk_provider || mfbsp->frame_provider))
			mfbsp_writel(mfbsp->base, MFBSP_I2S_TSTART, 0);
	}
}

static int mfbsp_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct mfbsp_data *mfbsp = snd_soc_dai_get_drvdata(dai);
	struct device *dev = dai->dev;
	u32 dir_reg = mfbsp_readl(mfbsp->base, MFBSP_I2S_DIR);
	u32 tctr_reg = mfbsp_readl(mfbsp->base, MFBSP_I2S_TCTR);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		mfbsp->bclk_provider = false;
		mfbsp->frame_provider = false;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		dir_reg |= MFBSP_I2S_DIR_TCLK | MFBSP_I2S_DIR_TCS;
		mfbsp->bclk_provider = true;
		mfbsp->frame_provider = true;
		break;
	default:
		return -EINVAL;
	}

	mfbsp_writel(mfbsp->base, MFBSP_I2S_DIR, dir_reg);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	/*
	 * If SND_SOC_DAIFMT_GATED is enabled in master mode, transmitter clock
	 * generator generates TCLK only if there is data in the transmit buffer.
	 * TCLK is also used to receive data (driver supports only schemes,
	 * described in Figure 37.12, 37.18 of the MCom-03 User Manual). As a
	 * result, during CAPTURE, transmitter buffer is always empty and MFBSP
	 * does not generate TCLK, required for receiving data. For this reason,
	 * SND_SOC_DAIFMT_GATED mode is not supported.
	 * SND_SOC_DAIFMT_CONT is supported since in this mode MFBSP generates TCLK
	 * if the transmitter is enabled (MFBSP_I2S_TSTART = 1).
	*/
	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SND_SOC_DAIFMT_CONT:
		mfbsp_writel(mfbsp->base, MFBSP_I2S_TCTR, tctr_reg | MFBSP_I2S_TCTR_CS_CONT |
			     MFBSP_I2S_TCTR_CLK_CONT);
		break;
	case SND_SOC_DAIFMT_GATED:
		if (mfbsp->bclk_provider || mfbsp->frame_provider) {
			dev_err(dev, "Gated clocks are not supported in master mode\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mfbsp_i2s_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct mfbsp_data *mfbsp = snd_soc_dai_get_drvdata(dai);
	struct device *dev = dai->dev;
	int ret = -EINVAL;
	u32 sysclk, clk_div, cs_div;
	u32 tctr_reg = mfbsp_readl(mfbsp->base, MFBSP_I2S_TCTR);
	snd_pcm_format_t format = params_format(params);

	if (snd_pcm_format_width(format) < 0)
		return -EINVAL;

	/*
	 * EN bit is changed by hardware when we write in TSTART register for
	 * playback stream. Because TCTR register is changed for both
	 * playback and capture, we need to save EN bit unchanged.
	 * Continuous clocks configuration should be unchanged.
	 */
	tctr_reg &= MFBSP_I2S_TCTR_EN | MFBSP_I2S_TCTR_CS_CONT | MFBSP_I2S_TCTR_CLK_CONT;
	tctr_reg |= MFBSP_I2S_TCTR_MBF | MFBSP_I2S_TCTR_CSNEG |
		    MFBSP_I2S_TCTR_DEL | MFBSP_I2S_TCTR_NEG;

	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_U16_LE:
		tctr_reg |= MFBSP_I2S_TCTR_SWAP | MFBSP_I2S_TCTR_PACK;
		tctr_reg |= MFBSP_I2S_TCTR_WORDLEN(15) |
			    MFBSP_I2S_TCTR_WORDCNT(0);
		break;
	default:
		return -EINVAL;
	}

	/*
	 * On Salute boards, TWS and TCLK are used both for transmit and
	 * receive of data. Therefore transmitter should be properly
	 * configured before receiving data.
	 *
	 * TODO: Configure TCTR only when it's required for receiver.
	 */
	mfbsp_writel(mfbsp->base, MFBSP_I2S_TCTR, tctr_reg);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		u32 rctr_reg = MFBSP_I2S_RCTR_MBF | MFBSP_I2S_RCTR_CSNEG |
			       MFBSP_I2S_RCTR_DEL | MFBSP_I2S_RCTR_NEG |
			       MFBSP_I2S_RCTR_CS_CP | MFBSP_I2S_RCTR_CLK_CP;

		switch (format) {
		case SNDRV_PCM_FORMAT_S16_LE:
		case SNDRV_PCM_FORMAT_U16_LE:
			rctr_reg |= MFBSP_I2S_RCTR_SWAP | MFBSP_I2S_RCTR_PACK;
			rctr_reg |= MFBSP_I2S_RCTR_WORDLEN(15) |
				    MFBSP_I2S_RCTR_WORDCNT(0);
			break;
		default:
			return -EINVAL;
		}

		mfbsp_writel(mfbsp->base, MFBSP_I2S_RCTR, rctr_reg);
	}

	sysclk = clk_get_rate(mfbsp->clk);

	if (mfbsp->bclk_provider) {
		clk_div = sysclk / (2 * snd_soc_params_to_bclk(params)) - 1;
		ret = snd_soc_dai_set_clkdiv(dai, MFBSP_TCLK_RATE_DIV, clk_div);
		if (ret) {
			dev_err(dev, "Failed to set TCLK_RATE divider, rc=%d\n", ret);
			return ret;
		}
	}

	if (mfbsp->frame_provider) {
		u32 tctr_reg = mfbsp_readl(mfbsp->base, MFBSP_I2S_TCTR);
		u32 wordlen = FIELD_GET(MFBSP_I2S_CTR_WORDLEN_MASK, tctr_reg);
		u32 wordcnt = FIELD_GET(MFBSP_I2S_CTR_WORDCNT_MASK, tctr_reg);

		cs_div = snd_pcm_format_width(format) - 1;

		if (cs_div < ((wordlen + 1)*(wordcnt + 1) - 1)) {
			dev_err(dev, "Invalid CS divider for: WORDLEN=0x%x WORDCNT=0x%x\n",
				wordlen, wordcnt);
			return -EINVAL;
		}

		ret = snd_soc_dai_set_clkdiv(dai, MFBSP_TCS_RATE_DIV, cs_div);
		if (ret) {
			dev_err(dev, "Failed to set TCS_RATE divider, rc=%d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int mfbsp_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct mfbsp_data *mfbsp = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		mfbsp->active--;
		mfbsp_i2s_stop(substream, dai);
		break;
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mfbsp->active++;
		mfbsp_i2s_start(substream, dai);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mfbsp_i2s_set_clkdiv(struct snd_soc_dai *dai,
				int div_id, int div)
{
	struct mfbsp_data *mfbsp = snd_soc_dai_get_drvdata(dai);
	u32 val;

	switch (div_id) {
	case MFBSP_RCLK_RATE_DIV:
		val = mfbsp_readl(mfbsp->base, MFBSP_I2S_RCTR_RATE) &
				  ~MFBSP_I2S_CLK_RATE_MASK;
		mfbsp_writel(mfbsp->base, MFBSP_I2S_RCTR_RATE, val |
			     FIELD_PREP(MFBSP_I2S_CLK_RATE_MASK, div));
		break;
	case MFBSP_RCS_RATE_DIV:
		val = mfbsp_readl(mfbsp->base, MFBSP_I2S_RCTR_RATE) &
				  ~MFBSP_I2S_CS_RATE_MASK;
		mfbsp_writel(mfbsp->base, MFBSP_I2S_RCTR_RATE, val |
			     FIELD_PREP(MFBSP_I2S_CS_RATE_MASK, div));
		break;
	case MFBSP_TCLK_RATE_DIV:
		val = mfbsp_readl(mfbsp->base, MFBSP_I2S_TCTR_RATE) &
				  ~MFBSP_I2S_CLK_RATE_MASK;
		mfbsp_writel(mfbsp->base, MFBSP_I2S_TCTR_RATE, val |
			     FIELD_PREP(MFBSP_I2S_CLK_RATE_MASK, div));
		break;
	case MFBSP_TCS_RATE_DIV:
		val = mfbsp_readl(mfbsp->base, MFBSP_I2S_TCTR_RATE) &
				  ~MFBSP_I2S_CS_RATE_MASK;
		mfbsp_writel(mfbsp->base, MFBSP_I2S_TCTR_RATE, val |
			     FIELD_PREP(MFBSP_I2S_CS_RATE_MASK, div));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops mfbsp_i2s_dai_ops = {
	.set_fmt	= mfbsp_i2s_set_fmt,
	.hw_params	= mfbsp_i2s_hw_params,
	.trigger	= mfbsp_i2s_trigger,
	.set_clkdiv	= mfbsp_i2s_set_clkdiv,
};

static struct snd_soc_dai_driver mfbsp_i2s_dai_driver = {
	.ops			= &mfbsp_i2s_dai_ops,
	.capture		= {
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_U16_LE,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.rate_min	= 8000,
		.rate_max	= 48000,
		.channels_min	= 2,
		.channels_max	= 2,
	},
	.playback		= {
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_U16_LE,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.rate_min	= 8000,
		.rate_max	= 48000,
		.channels_min	= 2,
		.channels_max	= 2,
	},
};

static const struct snd_soc_component_driver mfbsp_i2s_component_driver = {
	.name = "mfbsp-i2s",
};

static int mfbsp_i2s_probe(struct platform_device *pdev)
{
	struct mfbsp_data *mfbsp;
	struct resource *res;
	struct clk *clk;
	void __iomem *base;
	int ret;

	mfbsp = devm_kzalloc(&pdev->dev, sizeof(*mfbsp), GFP_KERNEL);
	if (!mfbsp)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mfbsp");
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	mfbsp->base = base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "dma-playback");
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	mfbsp->playback_dma.base = base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "dma-capture");
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	mfbsp->capture_dma.base = base;

	ret = platform_get_irq_byname(pdev, "dma-playback");
	if (ret < 0)
		return ret;
	mfbsp->playback_dma.irq = ret;

	ret = platform_get_irq_byname(pdev, "dma-capture");
	if (ret < 0)
		return ret;
	mfbsp->capture_dma.irq = ret;

	clk = devm_clk_get(&pdev->dev, 0);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	mfbsp->clk = clk;

	ret = clk_prepare_enable(clk);
	if (ret != 0)
		return ret;

	/* i2s lines configuration:
	bit : val : description
	 5  :  1  :  TD out
	 3  :  0  :  TWS in, receive word select from i2s master
	 1  :  0  :  TCLK in, receive i2s clock from i2s master
	*/
	mfbsp_writel(mfbsp->base, MFBSP_I2S_DIR, MFBSP_I2S_DIR_TD);
	mfbsp_writel(mfbsp->base, MFBSP_I2S_CSR, MFBSP_I2S_CSR_EN);

	dev_set_drvdata(&pdev->dev, mfbsp);

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &mfbsp_i2s_component_driver,
					      &mfbsp_i2s_dai_driver, 1);
	if (ret != 0) {
		clk_disable_unprepare(clk);
		return ret;
	}

	ret = mfbsp_register_platform(pdev);
	if (ret != 0) {
		clk_disable_unprepare(clk);
		return ret;
	}

	return 0;
}

static int mfbsp_i2s_remove(struct platform_device *pdev)
{
	struct mfbsp_data *mfbsp = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(mfbsp->clk);

	return 0;
}

static const struct of_device_id mfbsp_i2s_match_table[] = {
	{ .compatible = "elvees,mfbsp-i2s" },
	{ },
};
MODULE_DEVICE_TABLE(of, mfbsp_i2s_match_table);

static struct platform_driver mfbsp_i2s_platform_driver = {
	.probe			= mfbsp_i2s_probe,
	.remove			= mfbsp_i2s_remove,
	.driver			= {
		.name		= "mfbsp-i2s",
		.of_match_table	= of_match_ptr(mfbsp_i2s_match_table),
	},
};

module_platform_driver(mfbsp_i2s_platform_driver);

MODULE_ALIAS("platform:mfbsp-i2s");
MODULE_AUTHOR("Alexey Kiselev <akiselev@elvees.com>");
MODULE_DESCRIPTION("ELVEES MFBSP I2S SoC Interface");
MODULE_LICENSE("GPL");
