// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * ALSA SoC ELVEES MFBSP DMA PCM driver
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

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mfbsp.h"

#define MFBSP_PCM_PERIODS	8
#define MFBSP_PCM_PERIOD_BYTES	4096
#define MFBSP_PCM_BUFFER_BYTES	(MFBSP_PCM_PERIODS * MFBSP_PCM_PERIOD_BYTES)

static const struct snd_pcm_hardware mfbsp_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_U16_LE,
	.rates			= SNDRV_PCM_RATE_8000_48000,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= MFBSP_PCM_BUFFER_BYTES,
	.period_bytes_min	= MFBSP_PCM_PERIOD_BYTES,
	.period_bytes_max	= MFBSP_PCM_PERIOD_BYTES,
	.periods_min		= MFBSP_PCM_PERIODS,
	.periods_max		= MFBSP_PCM_PERIODS,
};

static void start_dma(struct mfbsp_dma_data *dma)
{
	struct mfbsp_dma_desc *dl = &dma->desc_list[0];

	// Seems it is MCom-03 errata: this will not start DMA opertaion
	// as documentation states:
	// mfbsp_writeq(dma->base, MFBSP_DMA_CP, dma->desc_addr | BIT(0));

	// Instead we need to prepare IR, CP of next DMA descriptor, and start with CSR.RUN.
	mfbsp_writeq(dma->base, MFBSP_DMA_IR, dl->ir);
	mfbsp_writeq(dma->base, MFBSP_DMA_CP, dl->cp);
	mfbsp_writel(dma->base, MFBSP_DMA_CSR, dl->csr | MFBSP_DMA_CSR_RUN);
}

static irqreturn_t mfbsp_pcm_irq_handler(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mfbsp_dma_data *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai,
							      substream);

	mfbsp_readl(dma->base, MFBSP_DMA_CSR);

	snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}

static int mfbsp_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mfbsp_dma_data *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai,
							      substream);

	snd_soc_set_runtime_hwparams(substream, &mfbsp_pcm_hardware);

	return request_irq(dma->irq, mfbsp_pcm_irq_handler, 0,
			   dev_name(rtd->cpu_dai->dev), substream);
}

static int mfbsp_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mfbsp_dma_data *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai,
							      substream);

	free_irq(dma->irq, substream);

	return 0;
}

static int mfbsp_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int mfbsp_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mfbsp_dma_data *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai,
							      substream);
	dma_addr_t buffer_addr = substream->dma_buffer.addr;
	dma_addr_t desc_addr = dma->desc_addr;
	struct mfbsp_dma_desc *desc_list = dma->desc_list;
	int period;

	for (period = 0; period < MFBSP_PCM_PERIODS; ++period) {
		desc_addr += sizeof(*desc_list);

		desc_list[period].ir = buffer_addr;
		desc_list[period].cp = desc_addr;
		desc_list[period].csr =
			MFBSP_DMA_CSR_WCX((MFBSP_PCM_PERIOD_BYTES >> 3) - 1) |
			MFBSP_DMA_CSR_IM |
			MFBSP_DMA_CSR_CHEN |
			MFBSP_DMA_CSR_WN(0) |
			MFBSP_DMA_CSR_RUN;

		buffer_addr += MFBSP_PCM_PERIOD_BYTES;
	}

	desc_list[MFBSP_PCM_PERIODS - 1].cp = dma->desc_addr;

	return 0;
}

static int mfbsp_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mfbsp_dma_data *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai,
							      substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		mfbsp_writel(dma->base, MFBSP_DMA_RUN, 0);
		while (mfbsp_readl(dma->base, MFBSP_DMA_RUN) & BIT(0))
			;
		break;
	case SNDRV_PCM_TRIGGER_START:
		start_dma(dma);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mfbsp_writel(dma->base, MFBSP_DMA_RUN, BIT(0));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t mfbsp_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mfbsp_dma_data *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai,
							      substream);
	dma_addr_t buffer_addr = substream->dma_buffer.addr;
	dma_addr_t buffer_pos = mfbsp_readq(dma->base, MFBSP_DMA_IR);

	return bytes_to_frames(substream->runtime, buffer_pos - buffer_addr);
}

static const struct snd_pcm_ops mfbsp_pcm_ops = {
	.open		= mfbsp_pcm_open,
	.close		= mfbsp_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mfbsp_pcm_hw_params,
	.hw_free	= snd_pcm_lib_free_pages,
	.prepare	= mfbsp_pcm_prepare,
	.trigger	= mfbsp_pcm_trigger,
	.pointer	= mfbsp_pcm_pointer,
};

static int mfbsp_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct mfbsp_data *mfbsp = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	struct device *dev = rtd->cpu_dai->dev;
	struct dma_pool *desc_pool;
	dma_addr_t desc_addr;
	struct mfbsp_dma_desc *desc_list;
	size_t desc_list_size = MFBSP_PCM_PERIODS * sizeof(*desc_list);

	/* By default dev->dma_coherent_mask is 32 bit,
	*  but for mcom03 Kernel RAM region addresses is above 33 bit,
	*  so usage of GFP_KERNEL in dma_poll_alloc requires changind this mask
	*/
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64))) {
		dev_warn(dev, "Failed to set DMA mask\n");
		/* will fail on next dma_pool_alloc on 64-bit mode on mcom03,
		*  not an error if 32-bit mode
		*/
	}

	desc_pool = dmam_pool_create(dev_name(dev), dev, desc_list_size, 8, 0);
	if (!desc_pool)
		return -ENOMEM;
	mfbsp->dma_desc_pool = desc_pool;

	desc_list = dma_pool_alloc(desc_pool, GFP_KERNEL, &desc_addr);
	if (!desc_list)
		return -ENOMEM;
	mfbsp->playback_dma.desc_addr = desc_addr;
	mfbsp->playback_dma.desc_list = desc_list;

	desc_list = dma_pool_alloc(desc_pool, GFP_KERNEL, &desc_addr);
	if (!desc_list)
		return -ENOMEM;
	mfbsp->capture_dma.desc_addr = desc_addr;
	mfbsp->capture_dma.desc_list = desc_list;

	return snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
						     SNDRV_DMA_TYPE_DEV, dev,
						     MFBSP_PCM_BUFFER_BYTES,
						     MFBSP_PCM_BUFFER_BYTES);
}

static void mfbsp_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static const struct snd_soc_component_driver mfbsp_component_driver = {
	.pcm_new	= mfbsp_pcm_new,
	.pcm_free	= mfbsp_pcm_free,
	.ops		= &mfbsp_pcm_ops,
};

int mfbsp_register_platform(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
						&mfbsp_component_driver,
						NULL, 0);
}
