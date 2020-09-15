/*
 * ELVEES Avico (a.k.a. VELcore-01) driver
 *
 * Copyright 2015 ELVEES NeoTek JSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Authors: Anton Leontiev <aleontiev@elvees.com>
 *          Vasiliy Zasukhin <vzasukhin@elvees.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include "avico.h"
#include "avico-bitstream.h"
#include "avico-debug.h"

#define MODULE_NAME "avico"

enum avico_flags {
	AVICO_ST_NONE,
	AVICO_ST_ENCODING,
	AVICO_ST_SAVING,	/* HW thread context saving */
	AVICO_ST_RESTORING,	/* HW thread context restoring */
};

/* Per queue */
#define AVICO_DEF_NUM_BUFS VIDEO_MAX_FRAME

/* \todo Get value from device tree */
#define XYRAM_BASE 0x3A400000
#define BOUNCE_BUF_BASE XYRAM_BASE
/* Size of macroblock in bytes */
#define MB_SIZE (16 * 16 * 3 / 2)
/* Length of data block VDMA transfers. Should be multiple of 16 bytes. */
#define DMA_CBS_LEN (40 * 8 * 16)
/* Size of a buffer for encoded data in VRAM. Should be multiple of 16 bytes. */
#define SIZE_CBS (DMA_CBS_LEN * 2)
/*
 * We need 4 bounce buffers BOUNCE_BUF_SIZE each (2 buffers for reconstructed
 * frame and 2 buffers for datastream).
 * Bounce buffer for reconstructed frame should be able to store up to one row
 * of macroblocks.
 * Bounce buffer for datastream should be able to store encoded data whose size
 * is multiple of DMA_CBS_LEN so BOUNCE_BUF_SIZE is rounded up if necessary.
 */
#define BOUNCE_BUF_SIZE roundup(AVICO_WMAX / 16 * MB_SIZE, DMA_CBS_LEN)

static const char *clknames[NCLKS] = { "pclk", "aclk", "sclk", "dsp_aclk" };

static inline void avico_write(u32 const value,
			       struct avico_ctx const *const ctx,
			       off_t const reg)
{
	iowrite32(value, ctx->dev->regs + reg);
}

static inline u32 avico_read(struct avico_ctx const *const ctx, off_t const reg)
{
	return ioread32(ctx->dev->regs + reg);
}

static inline void avico_dma_write(u32 const value,
				   struct avico_ctx const *const ctx,
				   unsigned const channel,
				   off_t const reg)
{
	avico_write(value, ctx, AVICO_VDMA_BASE +
		    AVICO_VDMA_CHANNEL_BASE(channel) + reg);
}

static inline u32 avico_dma_read(struct avico_ctx const *const ctx,
				 unsigned const channel, off_t const reg)
{
	return avico_read(ctx, AVICO_VDMA_BASE +
			  AVICO_VDMA_CHANNEL_BASE(channel) + reg);
}

/*
 * avico_ready() - check whether an instance is ready to be scheduled to run
 */
static int avico_ready(void *priv)
{
	struct avico_ctx *ctx = priv;

	if (ctx->aborting)
		return 0;

	return 1;
}

static void avico_abort(void *priv)
{
	struct avico_ctx *ctx = priv;

	ctx->aborting = 1;
}

void avico_thread_configure(struct avico_ctx *ctx)
{
	union frmn frmn = {
		.frmn = ctx->par.frame,
		.gop = ctx->par.gop,
		.idr = ctx->par.idr,
		.ftype = ctx->par.frame_type != VE_FR_I
	};

	union cfg cfg = {
		.slice_type = ctx->par.frame_type,
		.num_ref = 0,
		.auto_trailing = 1,
		.auto_flush = 1,
		.offset_qpskip = 0,
		.offset_a = 0,
		.offset_b = 0,
		.offset_qp = 0
		/* md_accur_shift = 0 */
	};

	avico_write(frmn.val, ctx,
		    AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_FRMN);
	avico_write(cfg.val, ctx,
		    AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_CFG);
}

void avico_dma_configure_reference(struct avico_ctx *ctx,
				   unsigned const channel)
{
	union adr adr;
	union vdma_acnt acnt;
	union vdma_bccnt bccnt;
	union vdma_hvecnt hvecnt;
	union vdma_hvicnt hvicnt;
	union vdma_cfg cfg = { .val = 0 };

	bool const swap_lines = (ctx->par.frame % 2) && (ctx->mby % 2);

	adr.val = avico_read(ctx, AVICO_THREAD_BASE(ctx->id) +
			     AVICO_THREAD_ADR);

	/* Reference frame: RAM to VRAM */

	avico_dma_write(ctx->dmaref, ctx, channel, AVICO_VDMA_CHANNEL_A0E);
	avico_dma_write(ctx->dmaref + 16 * 24 * ctx->mbx + 16, ctx, channel,
			AVICO_VDMA_CHANNEL_AECUR); /* + one MB line + one MB */
	avico_dma_write(ctx->dev->vram + adr.aref * 0x0400, ctx, channel,
			AVICO_VDMA_CHANNEL_A0I);

	if (!swap_lines) { /* => + one MB */
		avico_dma_write(ctx->dev->vram + adr.aref * 0x0400 +
				MB_REF_SIZE, ctx, channel,
				AVICO_VDMA_CHANNEL_AICUR);
	} else { /* swap_lines => + one MB line + one MB */
		avico_dma_write(ctx->dev->vram + adr.aref * 0x0400 +
				MB_REF_SIZE * (ctx->mbx + 1), ctx, channel,
				AVICO_VDMA_CHANNEL_AICUR);
	}

	avico_dma_write((ctx->mbx - 1) * 16, ctx, channel,
			AVICO_VDMA_CHANNEL_BEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CEIDX);
	avico_dma_write(-16 * 23 * ctx->mbx, ctx, channel,
			AVICO_VDMA_CHANNEL_HEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_VEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_BIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CIIDX);
	avico_dma_write(MB_REF_SIZE - 16 * 24, ctx, channel,
			AVICO_VDMA_CHANNEL_HIIDX);
	avico_dma_write(MB_REF_SIZE - 16 * 24, ctx, channel,
			AVICO_VDMA_CHANNEL_VIIDX);

	acnt.arld = acnt.acnt = (16 >> ctx->vdma_trans_size_m1) - 1;
	avico_dma_write(acnt.val, ctx, channel, AVICO_VDMA_CHANNEL_ACNT);

	bccnt.bcnt = bccnt.brld = 24 - 1;
	bccnt.ccnt = bccnt.crld = 1 - 1;
	avico_dma_write(bccnt.val, ctx, channel, AVICO_VDMA_CHANNEL_BCCNT);

	hvecnt.hecnt = ctx->mbx - 1 - 1;
	hvecnt.herld = ctx->mbx - 1;
	hvecnt.vecnt = ctx->mby - 1 - 1;
	hvecnt.verld = 2 - 1;
	avico_dma_write(hvecnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVECNT);

	hvicnt.hicnt = ctx->mbx - 1 - 1;
	hvicnt.hirld = ctx->mbx - 1;
	hvicnt.vicnt = (swap_lines ? 1 : 2) - 1;
	hvicnt.virld = 2 - 1;
	avico_dma_write(hvicnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVICNT);

	avico_dma_write(1, ctx, channel, AVICO_VDMA_CHANNEL_RUN);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_DONE);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_IMRDY);

	cfg.dir = 0;
	cfg.dim = 1;
	cfg.cycle = 1;
	cfg.prt = channel;
	cfg.brst_ae = 1;
	cfg.brst_ai = 1;
	cfg.size = ctx->vdma_trans_size_m1;
	avico_dma_write(cfg.val, ctx, channel, AVICO_VDMA_CHANNEL_CFG);
}

void avico_dma_configure_input_last_row(struct avico_ctx *ctx,
					unsigned const channel)
{
	union vdma_bccnt bccnt;
	uint8_t mb_lines = ctx->out_q.height % 16 * 3 / 2;

	if (mb_lines == 0)
		mb_lines = 24;

	avico_dma_write(-16 * (mb_lines - 1) * ctx->mbx, ctx, channel,
			AVICO_VDMA_CHANNEL_HEIDX);
	avico_dma_write(MB_CUR_SIZE - 16 * mb_lines, ctx, channel,
			AVICO_VDMA_CHANNEL_HIIDX);
	avico_dma_write(MB_CUR_SIZE - 16 * mb_lines, ctx, channel,
			AVICO_VDMA_CHANNEL_VIIDX);

	bccnt.bcnt = bccnt.brld = mb_lines - 1;
	bccnt.ccnt = bccnt.crld = 1 - 1;
	avico_dma_write(bccnt.val, ctx, channel, AVICO_VDMA_CHANNEL_BCCNT);
}

void avico_dma_configure_input(struct avico_ctx *ctx, unsigned const channel)
{
	union adr adr;
	union vdma_acnt acnt;
	union vdma_bccnt bccnt;
	union vdma_hvecnt hvecnt;
	union vdma_hvicnt hvicnt;
	union vdma_cfg cfg = { .val = 0 };

	bool const swap_lines = (ctx->par.frame % 2) && (ctx->mby % 2);

	adr.val = avico_read(ctx, AVICO_THREAD_BASE(ctx->id) +
			     AVICO_THREAD_ADR);

	/* Input frame: RAM to VRAM */

	avico_dma_write(ctx->dmainp, ctx, channel, AVICO_VDMA_CHANNEL_A0E);
	avico_dma_write(ctx->dmainp, ctx, channel, AVICO_VDMA_CHANNEL_AECUR);
	avico_dma_write(ctx->dev->vram + adr.acur * 0x0400, ctx, channel,
			AVICO_VDMA_CHANNEL_A0I);

	if (!swap_lines) { /* => as is */
		avico_dma_write(ctx->dev->vram + adr.acur * 0x0400, ctx,
				channel, AVICO_VDMA_CHANNEL_AICUR);
	} else { /* swap_lines => + one MB line */
		avico_dma_write(ctx->dev->vram + adr.acur * 0x0400 +
				MB_CUR_SIZE * ctx->mbx, ctx, channel,
				AVICO_VDMA_CHANNEL_AICUR);
	}

	avico_dma_write((ctx->mbx - 1) * 16, ctx, channel,
			AVICO_VDMA_CHANNEL_BEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CEIDX);
	avico_dma_write(-16 * 23 * ctx->mbx, ctx, channel,
			AVICO_VDMA_CHANNEL_HEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_VEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_BIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CIIDX);
	avico_dma_write(MB_CUR_SIZE - 16 * 24, ctx, channel,
			AVICO_VDMA_CHANNEL_HIIDX);
	avico_dma_write(MB_CUR_SIZE - 16 * 24, ctx, channel,
			AVICO_VDMA_CHANNEL_VIIDX);

	acnt.arld = acnt.acnt = (16 >> ctx->vdma_trans_size_m1) - 1;
	avico_dma_write(acnt.val, ctx, channel, AVICO_VDMA_CHANNEL_ACNT);

	bccnt.bcnt = bccnt.brld = 24 - 1;
	bccnt.ccnt = bccnt.crld = 1 - 1;
	avico_dma_write(bccnt.val, ctx, channel, AVICO_VDMA_CHANNEL_BCCNT);

	hvecnt.hecnt = hvecnt.herld = ctx->mbx - 1;
	hvecnt.vecnt = hvecnt.verld = ctx->mby - 1;
	avico_dma_write(hvecnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVECNT);

	hvicnt.hicnt = hvicnt.hirld = ctx->mbx - 1;
	hvicnt.vicnt = (swap_lines ? 1 : 2) - 1;
	hvicnt.virld = 2 - 1;
	avico_dma_write(hvicnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVICNT);

	avico_dma_write(1, ctx, channel, AVICO_VDMA_CHANNEL_RUN);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_DONE);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_IMRDY);

	cfg.dir = 0;
	cfg.dim = 1;
	cfg.cycle = 1;
	cfg.prt = channel;
	cfg.brst_ae = 1;
	cfg.brst_ai = 1;
	cfg.size = ctx->vdma_trans_size_m1;
	avico_dma_write(cfg.val, ctx, channel, AVICO_VDMA_CHANNEL_CFG);
}

void avico_dma_configure_reconstructured(struct avico_ctx *ctx,
					 unsigned const channel)
{
	union adr adr;
	union vdma_acnt acnt;
	union vdma_bccnt bccnt;
	union vdma_hvecnt hvecnt;
	union vdma_hvicnt hvicnt;
	union vdma_cfg cfg = { .val = 0 };

	bool const swap_lines = (ctx->par.frame % 2) && (ctx->mby % 2);

	adr.val = avico_read(ctx, AVICO_THREAD_BASE(ctx->id) +
			     AVICO_THREAD_ADR);

	/* Reconstructed frame: VRAM to bounce buffer */

	avico_dma_write(ctx->bounceref[ctx->bounce_active], ctx, channel,
			AVICO_VDMA_CHANNEL_A0E);
	avico_dma_write(ctx->bounceref[ctx->bounce_active], ctx, channel,
			AVICO_VDMA_CHANNEL_AECUR);
	avico_dma_write(ctx->dev->vram + adr.acur * 0x0400, ctx, channel,
			AVICO_VDMA_CHANNEL_A0I);

	if (!swap_lines) { /* => - one MB line */
		/* \bug + MB_CUR_SIZE or - MB_CUR_SIZE */
		avico_dma_write(ctx->dev->vram + adr.acur * 0x0400 +
				MB_CUR_SIZE * ctx->mbx, ctx, channel,
				AVICO_VDMA_CHANNEL_AICUR);
	} else { /* swap_lines */
		avico_dma_write(ctx->dev->vram + adr.acur * 0x0400, ctx,
				channel, AVICO_VDMA_CHANNEL_AICUR);
	}

	avico_dma_write((ctx->mbx - 1) * 16, ctx, channel,
			AVICO_VDMA_CHANNEL_BEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CEIDX);
	avico_dma_write(-16 * 23 * ctx->mbx, ctx, channel,
			AVICO_VDMA_CHANNEL_HEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_VEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_BIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CIIDX);
	avico_dma_write(MB_CUR_SIZE - 16 * 24, ctx, channel,
			AVICO_VDMA_CHANNEL_HIIDX);
	avico_dma_write(MB_CUR_SIZE - 16 * 24, ctx, channel,
			AVICO_VDMA_CHANNEL_VIIDX);

	acnt.arld = acnt.acnt = (16 >> ctx->vdma_trans_size_m1) - 1;
	avico_dma_write(acnt.val, ctx, channel, AVICO_VDMA_CHANNEL_ACNT);

	bccnt.bcnt = bccnt.brld = 24 - 1;
	bccnt.ccnt = bccnt.crld = 1 - 1;
	avico_dma_write(bccnt.val, ctx, channel, AVICO_VDMA_CHANNEL_BCCNT);

	hvecnt.hecnt = hvecnt.herld = ctx->mbx - 1;
	hvecnt.vecnt = 1 - 1;
	hvecnt.verld = ctx->mby - 1 - 1;
	avico_dma_write(hvecnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVECNT);

	hvicnt.hicnt = hvicnt.hirld = ctx->mbx - 1;
	hvicnt.vicnt = (swap_lines ? 2 : 1) - 1;
	hvicnt.virld = 2 - 1;
	avico_dma_write(hvicnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVICNT);

	avico_dma_write(1, ctx, channel, AVICO_VDMA_CHANNEL_RUN);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_DONE);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_IMRDY);

	cfg.dir = 1;
	cfg.dim = 1;
	cfg.cycle = 1;
	cfg.prt = channel;
	cfg.brst_ae = 1;
	cfg.brst_ai = 1;
	cfg.size = ctx->vdma_trans_size_m1;
	avico_dma_write(cfg.val, ctx, channel, AVICO_VDMA_CHANNEL_CFG);
}

void avico_dma_configure_bounceout(struct avico_ctx *ctx,
				   unsigned const channel)
{
	union vdma_acnt acnt;
	union vdma_hvecnt hvecnt = { .val = 0 };

	avico_dma_write(ctx->bounceout[ctx->bounce_active], ctx, channel,
			AVICO_VDMA_CHANNEL_A0E);

	acnt.arld = acnt.acnt = (DMA_CBS_LEN >> ctx->vdma_trans_size_m1) - 1;
	avico_dma_write(acnt.val, ctx, channel, AVICO_VDMA_CHANNEL_ACNT);

	hvecnt.hecnt = hvecnt.herld = (BOUNCE_BUF_SIZE / DMA_CBS_LEN) - 1;
	avico_dma_write(hvecnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVECNT);
}

/*
 * VDMA transfers data blocks from CBS buffer to bounce buffer, where each
 * data block has size DMA_CBS_LEN.
 */
void avico_dma_configure_output(struct avico_ctx *ctx, unsigned const channel)
{
	union adr adr;
	union vdma_hvicnt hvicnt = { .val = 0 };
	union vdma_cfg cfg = { .val = 0 };

	adr.val = avico_read(ctx, AVICO_THREAD_BASE(ctx->id) +
			     AVICO_THREAD_ADR);

	avico_dma_write(ctx->dev->vram + (adr.ares + 1) * 0x0800, ctx,
			channel, AVICO_VDMA_CHANNEL_A0I);

	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_BEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_HEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_VEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_BIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_HIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_VIIDX);

	hvicnt.hicnt = hvicnt.hirld = 2 - 1;
	avico_dma_write(hvicnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVICNT);

	avico_dma_configure_bounceout(ctx, channel);

	avico_dma_write(1, ctx, channel, AVICO_VDMA_CHANNEL_RUN);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_DONE);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_IMRDY);

	cfg.dir = 1;
	cfg.dim = 0;
	cfg.cycle = 1;
	cfg.prt = channel;
	cfg.brst_ae = 1;
	cfg.brst_ai = 1;
	cfg.size = ctx->vdma_trans_size_m1;
	avico_dma_write(cfg.val, ctx, channel, AVICO_VDMA_CHANNEL_CFG);
}

void avico_dma_configure(struct avico_ctx *ctx)
{
	uint8_t channel = ctx->id * 4;

	avico_dma_configure_reference(ctx, channel++);
	avico_dma_configure_input(ctx, channel++);
	avico_dma_configure_reconstructured(ctx, channel++);
	avico_dma_configure_output(ctx, channel);
}

static void avico_ec_configure(struct avico_ctx *const ctx)
{
	int i, ecd_stuff_pos;
	uint32_t data[2];
	unsigned int bits[2];
	union ecd_task task = { .val = 0 };

	avico_bitstream_get64(ctx, data, bits);

	ecd_stuff_pos = avico_bitstream_ecd_stuff_pos(ctx);
	avico_bitstream_cut64(ctx);

	task.val = avico_read(ctx, AVICO_EC_BASE(ctx->id) + AVICO_EC_TASKCTRC +
				   AVICO_TASKCTRC_TASK);
	task.id = ECD_TASK_H264_ENC_SLICE_RESET;
	task.repn = 0;
	task.rep = 1;
	task.run = 1;
	avico_write(task.val, ctx, AVICO_EC_BASE(ctx->id) + AVICO_EC_TASKCTRC +
				   AVICO_TASKCTRC_TASK);

	do {
		task.val = avico_read(ctx, AVICO_EC_BASE(ctx->id) +
					   AVICO_EC_TASKCTRC +
					   AVICO_TASKCTRC_TASK);
	} while (task.ready == 0);

	avico_write(0, ctx, AVICO_EC_BASE(ctx->id) + AVICO_EC_PACKER +
			    AVICO_PACKER_CBS_STUFF_MODE);
	for (i = 0; i < 2 && bits[i] != 0; i++) {
		avico_write(data[i], ctx, AVICO_EC_BASE(ctx->id) +
					  AVICO_EC_PACKER +
					  AVICO_PACKER_CBS_EXTBITS);
		avico_write(bits[i], ctx, AVICO_EC_BASE(ctx->id) +
					  AVICO_EC_PACKER +
					  AVICO_PACKER_CBS_EXTBITS_LEN);

		task.val = avico_read(ctx, AVICO_EC_BASE(ctx->id) +
					   AVICO_EC_TASKCTRC +
					   AVICO_TASKCTRC_TASK);
		task.id = ECD_TASK_ADDEXBYTES;
		task.repn = 0;
		task.rep = 1;
		task.run = 1;
		avico_write(task.val, ctx, AVICO_EC_BASE(ctx->id) +
					   AVICO_EC_TASKCTRC +
					   AVICO_TASKCTRC_TASK);

		do {
			task.val = avico_read(ctx, AVICO_EC_BASE(ctx->id) +
						   AVICO_EC_TASKCTRC +
						   AVICO_TASKCTRC_TASK);
		} while (task.ready == 0);
	}

	/* Configure ECD stuffing */
	avico_write(ecd_stuff_pos, ctx, AVICO_EC_BASE(ctx->id) +
					AVICO_EC_PACKER +
					AVICO_PACKER_CBS_STUFF_POS);
	avico_write(1, ctx, AVICO_EC_BASE(ctx->id) + AVICO_EC_PACKER +
			    AVICO_PACKER_CBS_STUFF_MODE);
}

void avico_codec_run(struct avico_ctx *ctx)
{
	union task task;

	/* Writing 1 to ON register is not working, due to HW error. */
	task.val = avico_read(ctx,
			      AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_TASK);
	task.run = 1;
	avico_write(task.val, ctx,
		    AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_TASK);

	avico_dma_write(1, ctx, ctx->id * 4, AVICO_VDMA_CHANNEL_IMRDY);
	avico_dma_write(1, ctx, ctx->id * 4 + 1, AVICO_VDMA_CHANNEL_IMRDY);
}

/* Enable or disable m6pos field in TASKx register */
void m6pos_enable(struct avico_ctx *const ctx, const bool enable)
{
	union task task;

	task.val = avico_read(ctx, AVICO_THREAD_BASE(ctx->id) +
				   AVICO_THREAD_TASK);
	task.m6pos = enable;
	task.m6eof = !enable; /* To use m6pos we need to disable m6eof */
	avico_write(task.val, ctx, AVICO_THREAD_BASE(ctx->id) +
				   AVICO_THREAD_TASK);
}

static void avico_thread_init(struct avico_ctx *ctx)
{
	struct avico_frame_params *par = &ctx->par;
	union mbpos mbpos = {
		.nx = DIV_ROUND_UP(ctx->out_q.width, 16),
		.ny = DIV_ROUND_UP(ctx->out_q.height, 16)
	};

	union adr adr = {
		.acur = ctx->id * 0x0100,
		.aref = ctx->id * 0x0100 + mbpos.nx,
		.ares = ctx->id * 0x0080 + mbpos.nx
	};

	uint8_t qp = par->frame_type == VE_FR_I ? par->qp_i : par->qp_p;
	union task task = {
		.std = VE_STD_H264,
		.qpy = qp,
		.qpc = clamp(qp + par->qpc_offset, 0, 51),
		.dbf = par->dbf,
		.m6eof = 1,
		.m7eof = 1
	};

	union frmn frmn = {
		.gop = par->gop
	};

	avico_write(mbpos.val, ctx, AVICO_THREAD_BASE(ctx->id) +
		    AVICO_THREAD_MBPOS);
	avico_write(adr.val, ctx, AVICO_THREAD_BASE(ctx->id) +
		    AVICO_THREAD_ADR);
	avico_write(task.val, ctx, AVICO_THREAD_BASE(ctx->id) +
		    AVICO_THREAD_TASK);
	avico_write(frmn.val, ctx, AVICO_THREAD_BASE(ctx->id) +
		    AVICO_THREAD_FRMN);
}

static void avico_ec_init(struct avico_ctx *ctx)
{
	union ecd_task ecd_task;

	avico_write(SIZE_CBS / 16, ctx, AVICO_EC_BASE(ctx->id) +
			AVICO_EC_VRAMCTRC + AVICO_VRAMCTRC_SIZE_CBS);

	avico_write(DMA_CBS_LEN / 16, ctx, AVICO_EC_BASE(ctx->id) +
			AVICO_EC_TASKCTRC + AVICO_TASKCTRC_DMALEN);

	avico_write(ECD_CS_CAVLC, ctx, AVICO_EC_BASE(ctx->id) +
			AVICO_EC_TASKCTRC + AVICO_TASKCTRC_CS);

	ecd_task.val = avico_read(ctx, AVICO_EC_BASE(ctx->id) +
				  AVICO_EC_TASKCTRC + AVICO_TASKCTRC_TASK);
	ecd_task.id = ECD_TASK_H264_ENC_RESET;
	ecd_task.rep = 1;
	ecd_task.run = 1;
	ecd_task.repn = 0;
	avico_write(ecd_task.val, ctx, AVICO_EC_BASE(ctx->id) +
		    AVICO_EC_TASKCTRC + AVICO_TASKCTRC_TASK);

	do {
		ecd_task.val = avico_read(ctx, AVICO_EC_BASE(ctx->id) +
				AVICO_EC_TASKCTRC + AVICO_TASKCTRC_TASK);
	} while (ecd_task.ready == 0);
}

void avico_dma_configure_save_restore(struct avico_ctx *ctx,
				      unsigned int const channel,
				      unsigned int const dir,
				      dma_addr_t const a0e,
				      dma_addr_t const a0i,
				      uint32_t const alen,
				      uint32_t const blen)
{
	union vdma_acnt acnt;
	union vdma_bccnt bccnt = { .val = 0 };
	union vdma_hvecnt const hvecnt = { .val = 0 };
	union vdma_hvicnt const hvicnt = { .val = 0 };
	union vdma_cfg cfg = { .val = 0 };

	avico_dma_write(a0e, ctx, channel, AVICO_VDMA_CHANNEL_A0E);
	avico_dma_write(a0i, ctx, channel, AVICO_VDMA_CHANNEL_A0I);

	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_BEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_HEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_VEIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_BIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_CIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_HIIDX);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_VIIDX);

	acnt.arld = acnt.acnt = (alen >> ctx->vdma_trans_size_m1) - 1;
	avico_dma_write(acnt.val, ctx, channel, AVICO_VDMA_CHANNEL_ACNT);

	bccnt.bcnt = bccnt.brld = blen - 1;
	avico_dma_write(bccnt.val, ctx, channel, AVICO_VDMA_CHANNEL_BCCNT);

	avico_dma_write(hvecnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVECNT);
	avico_dma_write(hvicnt.val, ctx, channel, AVICO_VDMA_CHANNEL_HVICNT);

	avico_dma_write(1, ctx, channel, AVICO_VDMA_CHANNEL_RUN);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_DONE);
	avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_IMRDY);

	cfg.dir = dir;
	cfg.dim = 0;
	cfg.cycle = 0;
	cfg.prt = channel;
	cfg.brst_ae = 1;
	cfg.brst_ai = 1;
	cfg.size = ctx->vdma_trans_size_m1;
	avico_dma_write(cfg.val, ctx, channel, AVICO_VDMA_CHANNEL_CFG);
}

static void avico_restore(struct avico_ctx *ctx)
{
	union adr const adr = {
		.acur = ctx->id * 0x0100,
		.aref = ctx->id * 0x0100 + ctx->mbx
	};

	avico_dma_configure_save_restore(ctx, 0, 0, ctx->dmambref,
					 ctx->dev->vram + adr.aref * 0x0400,
					 MB_REF_SIZE, ctx->mbx * 2);
	avico_dma_configure_save_restore(ctx, 1, 0, ctx->dmambcur,
					 ctx->dev->vram + adr.acur * 0x0400,
					 MB_CUR_SIZE, ctx->mbx * 2);

	/* @bug Following will not work for several threads */
	avico_write(EV_1 << (ctx->id * 8), ctx,
		    AVICO_CTRL_BASE + AVICO_CTRL_MSK_INT);

	ctx->state = AVICO_ST_RESTORING;

	avico_dma_write(1, ctx, ctx->id * 4, AVICO_VDMA_CHANNEL_IMRDY);
	avico_dma_write(1, ctx, ctx->id * 4 + 1, AVICO_VDMA_CHANNEL_IMRDY);
}

static void avico_save(struct avico_ctx *ctx)
{
	union adr const adr = {
		.acur = ctx->id * 0x0100,
		.aref = ctx->id * 0x0100 + ctx->mbx
	};

	/*
	 * We need to switch off thread because additional events are generated
	 * otherwise.
	 */
	avico_write(1, ctx, AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_OFF);

	avico_dma_configure_save_restore(ctx, 0, 1, ctx->dmambref,
					 ctx->dev->vram + adr.aref * 0x0400,
					 MB_REF_SIZE, ctx->mbx * 2);
	avico_dma_configure_save_restore(ctx, 1, 1, ctx->dmambcur,
					 ctx->dev->vram + adr.acur * 0x0400,
					 MB_CUR_SIZE, ctx->mbx * 2);

	/* @bug Following will not work for several threads */
	avico_write(EV_1 << (ctx->id * 8), ctx,
		    AVICO_CTRL_BASE + AVICO_CTRL_MSK_INT);

	ctx->state = AVICO_ST_SAVING;

	avico_dma_write(1, ctx, ctx->id * 4, AVICO_VDMA_CHANNEL_IMRDY);
	avico_dma_write(1, ctx, ctx->id * 4 + 1, AVICO_VDMA_CHANNEL_IMRDY);
}

static void avico_start(struct avico_ctx *ctx)
{
	struct vb2_buffer *src, *dst;
	uint8_t *out;
	/* Line of half-MBs reserve above fr_ref */
	/* \todo Rename fr_ref_reserve_offset and fr_size_ram */
	uint32_t const fr_ref_reserve_offset = (ctx->mbx * 16) * 12;
	uint32_t const fr_size_ram = (ctx->mbx * 16) * (ctx->mby * 16) * 3 / 2;
	union smbpos smbpos;
	bool write_pps = false;
	int8_t qpc_offset;
	struct avico_frame_params *par = &ctx->par;

	ctx->bounce_count = 0;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	out = vb2_plane_vaddr(dst, 0);
	WARN_ON(out == NULL);

	ctx->dmainp = vb2_dma_contig_plane_dma_addr(src, 0);
	ctx->dmaout = vb2_dma_contig_plane_dma_addr(dst, 0);

	ctx->error = false;

	par->idr = (par->frame % par->gop) == 0;

	/*
	 * We can't take ctx->ctrl_handler.lock in this function because
	 * it can be called in atomic context so we don't support atomic
	 * reading of controls.
	 */
	par->qp_i = ctx->ctrl_qp_i->cur.val;
	par->qp_p = ctx->ctrl_qp_p->cur.val;
	qpc_offset = ctx->ctrl_qpc_off->cur.val;
	if (ctx->force_key) {
		ctx->force_key = false;
		par->idr = true;
	}

	if (par->idr) {
		par->gop = ctx->ctrl_gop->cur.val;
		par->log2_max_frame = max(4, order_base_2(par->gop));
		par->frame_type = VE_FR_I;
		par->idr_id++;
		par->frame = 0;
		write_pps = true;
	} else if (par->i_period > 0 && par->frame % par->i_period == 0) {
		par->frame_type = VE_FR_I;
	} else {
		par->frame_type  = VE_FR_P;
	}

	/*
	 * Write PPS if chroma_qp_index_offset has been changed. qpc_offset is
	 * used instead of ctx->ctrl_qpc_off->cur.val to read control
	 * atomically.
	 */
	if (par->qpc_offset != qpc_offset) {
		write_pps = true;
		par->qpc_offset = qpc_offset;
	}

	/*
	 * TODO: Optimization: don't call if the same SW context is run on the
	 * same HW context
	 */
	avico_thread_init(ctx);

	/* \todo Configure MD */

	avico_ec_init(ctx);

	/* Enable stop by SMBPOS */
	if (ctx->mby > 1) {
		m6pos_enable(ctx, true);

		/* Stop at the beginning of the second row of MB */
		smbpos.val = 0;
		smbpos.y6 = 1;
		avico_write(smbpos.val, ctx, AVICO_THREAD_BASE(ctx->id) +
					     AVICO_THREAD_SMBPOS);
	}

	avico_bitstream_init(ctx, out, vb2_plane_size(dst, 0));

	/*if (vb2_plane_size(src, 0) > vb2_plane_size(dst, 0)) {
		v4l2_err(&dev->v4l2_dev, "Output buffer is too small\n");
		return;
	}*/

	if (par->idr)
		avico_bitstream_write_sps(ctx);

	if (write_pps)
		avico_bitstream_write_pps(ctx);

	avico_bitstream_write_slice_header(ctx);

	avico_write(0, ctx, AVICO_EC_BASE(ctx->id) + AVICO_EC_PACKER +
			    AVICO_PACKER_CBS_TOTAL_LEN);

	/* Configure sequencer for slice data encoding */
	avico_thread_configure(ctx);
	avico_ec_configure(ctx);

	ctx->dmaout += ctx->bs.p - ctx->bs.start;
	ctx->bitstream_size = ctx->bs.end - ctx->bs.p;

	ctx->bounce_active = 0;

	/* At begin will receive last row of previous frame. Half of this row
	 * must be placed at end of frame, but second half must be placed at
	 * begin of frame. We set ref_ptr_off to end of frame minus
	 * half of row */
	ctx->ref_ptr_off = fr_ref_reserve_offset + fr_size_ram -
			   16 * 24 * ctx->mbx;
	ctx->out_ptr_off = 0;

	avico_dma_configure(ctx);

	/* @bug Following will not work for several threads */
	avico_write(0xc0 << (ctx->id * 8), ctx,
		    AVICO_CTRL_BASE + AVICO_CTRL_MSK_INT);

	/* Need for M6POS */
	/* \todo Add reference to the section in manual. */
	avico_write(1, ctx, AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_OFF);

	ctx->state = AVICO_ST_ENCODING;

	/* Run slice data encoding */
	avico_codec_run(ctx);
}

/*
 * avico_run() - prepares and starts VPU
 */
static void avico_run(void *priv)
{
	struct avico_ctx *ctx = priv;

	avico_restore(ctx);
}

/*
 * This function will be called by SDMA after frame's last encoded and
 * reconstructed data was copied to DDR
 */
static void avico_eof_sdma_callback(void *data)
{
	struct avico_ctx *ctx = (struct avico_ctx *)data;
	struct avico_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned long flags;
	uint32_t encoded;
	bool error = ctx->error;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	encoded = avico_read(ctx, AVICO_EC_BASE(ctx->id) + AVICO_EC_PACKER +
				  AVICO_PACKER_CBS_TOTAL_LEN);
	WARN_ON(encoded % 8 != 0);
	ctx->bs.p += encoded / 8;
	vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->bs.p - ctx->bs.start);

	/*
	 * If offset in output buffer calculated from bounce-to-DDR DMA
	 * transfers differs from size of encoded data from VPU (rounded to
	 * minimal VRAM-to-bounce DMA transfer size), that means that
	 * bounce buffer was overflowed at least once and data is corrupted.
	 */
	if (ctx->out_ptr_off != roundup(encoded / 8, DMA_CBS_LEN)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "frame %u: insufficient data in capture buffer\n",
			 ctx->capseq);
		error = true;
	}

	dst->sequence = ctx->capseq++;
	src->sequence = ctx->outseq++;

	memcpy(&dst->timestamp, &src->timestamp, sizeof(struct timeval));
	if (src->flags & V4L2_BUF_FLAG_TIMECODE) {
		memcpy(&dst->timecode, &src->timecode,
		       sizeof(struct v4l2_timecode));
	}

	dst->field = src->field;
	dst->flags = src->flags;

	avico_save(ctx);

	/* \todo Do not understand why we need irqlock here */
	spin_lock_irqsave(&dev->irqlock, flags);
	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, error ? VB2_BUF_STATE_ERROR :
			       VB2_BUF_STATE_DONE);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	if (++ctx->par.frame >= ctx->par.gop)
		ctx->par.frame = 0;
}

static void avico_vdma_next_bounce(struct avico_ctx *ctx)
{
	union smbpos smbpos;
	u32 msk_int;

	/* Swap active buffers */
	ctx->bounce_active ^= 1;
	avico_dma_write(ctx->bounceref[ctx->bounce_active], ctx, 2,
			AVICO_VDMA_CHANNEL_A0E);
	avico_dma_configure_bounceout(ctx, 3);

	smbpos.val = avico_read(ctx, AVICO_THREAD_BASE(ctx->id) +
				     AVICO_THREAD_SMBPOS);
	smbpos.y6++;  /* Set next stop position */

	/* If next row is last then M6POS will not stop and we
	 * need to enable M6EOF bit */
	if (smbpos.y6 >= ctx->mby) {
		msk_int = 0x80;
		m6pos_enable(ctx, false);
		avico_dma_configure_input_last_row(ctx, 1);
	} else {
		msk_int = 0x40;
		avico_write(smbpos.val, ctx,
			    AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_SMBPOS);
	}

	/* In M6POS mode before continue we need to OFF and ON thread */
	/* \todo Add reference to the section in manual. */
	avico_write(1, ctx, AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_OFF);
	avico_write(1, ctx, AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_ON);

	avico_write(0x40 << (ctx->id * 8), ctx,
		    AVICO_CTRL_BASE + AVICO_CTRL_MSK_EV);
	avico_write(0, ctx, AVICO_CTRL_BASE + AVICO_CTRL_EVENTS);

	/* BUG: We do not know how correctly clean events flag before
	 * exit from IRQ handler. If we exit immediately after cleaning
	 * events flag then IRQ handler can be called again. To
	 * prevent this we run DMA channels after cleaning event and
	 * use memory barrier. */
	wmb();

	/* If next row isn't last then enable interrupt by EV6 */
	avico_write(msk_int, ctx, AVICO_CTRL_BASE + AVICO_CTRL_MSK_INT);

	avico_dma_write(1, ctx, ctx->id * 4 + 1, AVICO_VDMA_CHANNEL_RUN);
	avico_dma_write(1, ctx, ctx->id * 4 + 2, AVICO_VDMA_CHANNEL_RUN);
	avico_dma_write(1, ctx, ctx->id * 4 + 3, AVICO_VDMA_CHANNEL_RUN);
}

/*
 * This function will be called by SDMA after intermediate encoded and
 * reconstructed data was copied to DDR
 */
static void avico_sdma_callback(void *data)
{
	struct avico_ctx *ctx = (struct avico_ctx *)data;
	unsigned long flags;

	spin_lock_irqsave(&ctx->bounce_lock, flags);
	if (ctx->bounce_count-- >= 2)
		avico_vdma_next_bounce(ctx);
	spin_unlock_irqrestore(&ctx->bounce_lock, flags);
}

/* Setup SDMA to copy encoded data from bounce buffer to DDR */
static uint32_t avico_setup_bounceout(struct avico_ctx *ctx)
{
	uint32_t out_size = avico_dma_read(ctx, 3, AVICO_VDMA_CHANNEL_AECUR) -
			    ctx->bounceout[ctx->bounce_active];
	struct dma_async_tx_descriptor *tx;

	/* If out_size is zero and DONE is 1 then AECUR is wrapped to A0E */
	if (out_size == 0 && avico_dma_read(ctx, 3, AVICO_VDMA_CHANNEL_DONE))
		out_size = BOUNCE_BUF_SIZE;
	avico_dma_write(0, ctx, 3, AVICO_VDMA_CHANNEL_DONE);

	/*
	 * Bitstream could not to be ready for some rows (for the last row it is
	 * only possible due to error) so we can't run SDMA when out_size is
	 * zero.
	 */
	if (out_size) {
		tx = ctx->dev->dma_ch->device->device_prep_dma_memcpy(
			ctx->dev->dma_ch, ctx->dmaout + ctx->out_ptr_off,
			ctx->bounceout[ctx->bounce_active], out_size, 0);
		dmaengine_submit(tx);

		ctx->out_ptr_off += out_size;
	}

	return out_size;
}

/* Setup SDMA to copy reconstructed data from bounce buffer to DDR */
static void avico_setup_bounceref(struct avico_ctx *ctx,
				  dma_async_tx_callback callback)
{
	uint32_t frame_size = ctx->mbx * ctx->mby * MB_SIZE;
	uint32_t ref_size = ctx->mbx * MB_SIZE; /* We work by row of MB */
	struct dma_async_tx_descriptor *tx;

	if (ctx->ref_ptr_off + ref_size > frame_size) {
		/* First half of last row should be written to end of the frame.
		 * Second half should be written to the beginning of the
		 * frame */
		uint32_t end_part_size = frame_size - ctx->ref_ptr_off;

		tx = ctx->dev->dma_ch->device->device_prep_dma_memcpy(
			ctx->dev->dma_ch, ctx->dmaref + ctx->ref_ptr_off,
			ctx->bounceref[ctx->bounce_active], end_part_size, 0);
		dmaengine_submit(tx);
		tx = ctx->dev->dma_ch->device->device_prep_dma_memcpy(
			ctx->dev->dma_ch, ctx->dmaref,
			ctx->bounceref[ctx->bounce_active] + end_part_size,
			ref_size - end_part_size, 0);
		tx->callback_param = ctx;
		tx->callback = callback;
		dmaengine_submit(tx);

		ctx->ref_ptr_off = ref_size - end_part_size;
	} else {
		tx = ctx->dev->dma_ch->device->device_prep_dma_memcpy(
			ctx->dev->dma_ch, ctx->dmaref + ctx->ref_ptr_off,
			ctx->bounceref[ctx->bounce_active], ref_size, 0);
		tx->callback_param = ctx;
		tx->callback = callback;
		dmaengine_submit(tx);

		ctx->ref_ptr_off += ref_size;
	}
}

/* Setup SDMA to copy data from bounce buffer to DDR */
static int avico_copy_bounce(struct avico_ctx *ctx, bool eof)
{
	uint32_t out_size = avico_setup_bounceout(ctx);

	if (!eof)
		avico_setup_bounceref(ctx, avico_sdma_callback);
	else
		avico_setup_bounceref(ctx, avico_eof_sdma_callback);

	dma_async_issue_pending(ctx->dev->dma_ch);

	/*
	 * We expect out_size isn't zero for last row in frame so if it is zero
	 * we return error. We don't terminate SDMA activity in that case
	 * because we wait for the end of copying of reconstructed frame.
	 */
	if (eof && out_size == 0)
		return -EFAULT;

	return 0;
}

static void avico_clear_disable_events(struct avico_ctx *ctx)
{
	unsigned int channel;

	/*
	 * Clear RUN registers to ensure VDMA channels are switched off. EVENTS
	 * register depends on DONE registers so we clear them before clearing
	 * of EVENTS.
	 */
	for (channel = ctx->id * 4; channel < ctx->id * 4 + 4; channel++) {
		avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_RUN);
		avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_DONE);
	}

	/* \bug Will not work for several threads
	 * We should gaurantee, that others will not overwrite MSK_EV. */
	avico_write(0xff << (ctx->id * 8), ctx,
		    AVICO_CTRL_BASE + AVICO_CTRL_MSK_EV);
	avico_write(0, ctx, AVICO_CTRL_BASE + AVICO_CTRL_EVENTS);

	/* BUG: We do not know how correctly clean events flag before
	 * exit from IRQ handler. If we exit immediately after cleaning
	 * events flag then IRQ handler can be called again. To
	 * prevent this we run DMA channels after cleaning event and
	 * use memory barrier. */
	wmb();

	/* \todo Move to context allocation */
	/* \bug Will not work for several threads */
	avico_write(0, ctx, AVICO_CTRL_BASE + AVICO_CTRL_MSK_INT);
}

static void avico_wait_for_mb_flush(struct avico_ctx *ctx)
{
	int i;
	u32 events = avico_read(ctx, AVICO_CTRL_EVENTS);

	/* Wait for EV_0, EV_1 */
	while ((events & EV_0) == 0 || (events & EV_1) == 0)
		events = avico_read(ctx, AVICO_CTRL_EVENTS);

	/* \bug Need delay: rf#2003 */
	for (i = 0; i < 80; i++)
		events = avico_read(ctx, AVICO_CTRL_EVENTS);

	/* Wait for flush of previous MB data */
	while (events != (EV_0 | EV_1 | EV_6))
		events = avico_read(ctx, AVICO_CTRL_EVENTS);
}

static irqreturn_t avico_irq(int irq, void *data)
{
	struct avico_dev *dev = (struct avico_dev *)data;
	struct avico_ctx *ctx;
	u32 events;
	bool eof;

	/* \todo How does this will work for several HW threads? */
	ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (ctx == NULL) {
		pr_err("Instance released before the end of transaction\n");

		return IRQ_HANDLED;
	}

	events = avico_read(ctx, AVICO_CTRL_EVENTS);
	eof = events & EV_7;

	switch (ctx->state) {
	case AVICO_ST_ENCODING:
		if (!eof) {
			avico_wait_for_mb_flush(ctx);
			avico_dma_write(0, ctx, ctx->id * 4 + 1,
					AVICO_VDMA_CHANNEL_RUN);
			avico_dma_write(0, ctx, ctx->id * 4 + 2,
					AVICO_VDMA_CHANNEL_RUN);
			avico_dma_write(0, ctx, ctx->id * 4 + 3,
					AVICO_VDMA_CHANNEL_RUN);
		}

		if (ctx->aborting || avico_copy_bounce(ctx, eof) < 0)
			goto err;

		/* If it isn't last row then run next row */
		if (!eof) {
			spin_lock(&ctx->bounce_lock);
			/* Run VDMA for the next buffer if there is no SDMA */
			if (++ctx->bounce_count < 2)
				avico_vdma_next_bounce(ctx);
			else
				avico_write(0, ctx, AVICO_CTRL_BASE +
						    AVICO_CTRL_MSK_INT);
			spin_unlock(&ctx->bounce_lock);
		} else {
			avico_clear_disable_events(ctx);
		}

		break;
	case AVICO_ST_RESTORING:
	case AVICO_ST_SAVING:
		/*
		 * Wait for the end of saving/restoring of mb_cur and mb_ref
		 * structures.
		 */
		WARN_ON((events & EV_1) == 0);
		while ((events & EV_0) == 0)
			events = avico_read(ctx, AVICO_CTRL_EVENTS);

		avico_clear_disable_events(ctx);

		if (ctx->state == AVICO_ST_RESTORING) {
			avico_start(ctx);
		} else {
			ctx->state = AVICO_ST_NONE;
			v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
		}
		break;
	default:
		v4l2_err(&dev->v4l2_dev, "Invalid state\n");
		break;
	}

	return IRQ_HANDLED;

err:
	ctx->error = true;
	avico_clear_disable_events(ctx);
	if (ctx->aborting) {
		dmaengine_terminate_all(ctx->dev->dma_ch);
		avico_eof_sdma_callback(ctx);
	}

	return IRQ_HANDLED;
}

static const struct v4l2_m2m_ops avico_m2m_ops = {
	.device_run = avico_run,
	.job_ready  = avico_ready,
	.job_abort  = avico_abort,
};

static int avico_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strlcpy(cap->driver, MODULE_NAME, sizeof(cap->driver));
	strlcpy(cap->card, MODULE_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 MODULE_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}


static inline bool avico_compressed(struct v4l2_fmtdesc *f)
{
	if (f->flags & V4L2_FMT_FLAG_COMPRESSED)
		return true;
	return false;
}

static struct v4l2_fmtdesc avico_formats[] = {
	{
		.description = "M420",
		.pixelformat = V4L2_PIX_FMT_M420
	},
	{
		.flags = V4L2_FMT_FLAG_COMPRESSED,
		.description = "H264 Encoded Stream",
		.pixelformat = V4L2_PIX_FMT_H264
	},
};

static struct v4l2_fmtdesc *avico_find_format(struct avico_ctx *ctx,
					      u32 pixelformat, u32 type)
{
	unsigned int i;
	bool compressed = ctx->thread_type == AVICO_ENCODER ?
					      !V4L2_TYPE_IS_OUTPUT(type) :
					      V4L2_TYPE_IS_OUTPUT(type);

	for (i = 0; i < ARRAY_SIZE(avico_formats); ++i) {
		struct v4l2_fmtdesc *f = &avico_formats[i];

		if (f->pixelformat == pixelformat &&
		    avico_compressed(f) == compressed)
			return f;
	}

	return NULL;
}

static inline struct avico_q_data *get_q_data(struct avico_ctx *ctx,
					      enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->out_q;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->cap_q;
	default:
		return NULL;
	}
}

static int avico_enum_fmt(struct v4l2_fmtdesc *formats, int nformats,
			  struct v4l2_fmtdesc *f, bool compressed)
{
	int i, index = 0;

	for (i = 0; i < nformats; ++i)
		if (avico_compressed(&formats[i]) == compressed &&
		    index++ == f->index)
			break;

	/* Format not found */
	if (i >= nformats)
		return -EINVAL;

	strlcpy(f->description, formats[i].description, sizeof(f->description));
	f->pixelformat = formats[i].pixelformat;
	f->flags = formats[i].flags;

	return 0;
}

static int avico_enum_fmt_output(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);

	return avico_enum_fmt(avico_formats, ARRAY_SIZE(avico_formats), f,
			      ctx->thread_type == AVICO_DECODER);
}

static int avico_enum_fmt_capture(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);

	return avico_enum_fmt(avico_formats, ARRAY_SIZE(avico_formats), f,
			      ctx->thread_type == AVICO_ENCODER);
}

static int avico_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);
	struct avico_q_data *q_data = get_q_data(ctx, f->type);

	if (!q_data)
		return -EINVAL;

	f->fmt.pix.width = q_data->width;
	f->fmt.pix.height = q_data->height;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat = q_data->f->pixelformat;
	f->fmt.pix.bytesperline = q_data->bytesperline;
	f->fmt.pix.sizeimage = q_data->sizeimage;
	f->fmt.pix.colorspace = ctx->colorspace;
	f->fmt.pix.ycbcr_enc = ctx->matrix;
	f->fmt.pix.quantization = ctx->range;
	f->fmt.pix.xfer_func = ctx->transfer;

	return 0;
}

/* SD streams likely use SMPTE170M and HD streams REC709 */
static enum v4l2_colorspace avico_def_colorspace(unsigned int width,
						 unsigned int height)
{
	if (width <= 720 && height <= 576)
		return V4L2_COLORSPACE_SMPTE170M;

	return V4L2_COLORSPACE_REC709;
}

static void avico_try_colorspace(struct v4l2_format *f)
{
	switch (f->fmt.pix.colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
	case V4L2_COLORSPACE_REC709:
	case V4L2_COLORSPACE_SRGB:
	case V4L2_COLORSPACE_BT2020:
	case V4L2_COLORSPACE_SMPTE240M:
	case V4L2_COLORSPACE_470_SYSTEM_M:
	case V4L2_COLORSPACE_470_SYSTEM_BG:
	case V4L2_COLORSPACE_RAW:
		break;
	default:
		f->fmt.pix.colorspace = avico_def_colorspace(f->fmt.pix.width,
							     f->fmt.pix.height);
		break;
	}

	switch (f->fmt.pix.ycbcr_enc) {
	case V4L2_YCBCR_ENC_XV601:
	case V4L2_YCBCR_ENC_SYCC:
	case V4L2_YCBCR_ENC_601:
	case V4L2_YCBCR_ENC_XV709:
	case V4L2_YCBCR_ENC_709:
	case V4L2_YCBCR_ENC_BT2020_CONST_LUM:
	case V4L2_YCBCR_ENC_BT2020:
	case V4L2_YCBCR_ENC_SMPTE240M:
	case V4L2_YCBCR_ENC_DEFAULT:
		break;
	default:
		f->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		break;
	}

	switch (f->fmt.pix.xfer_func) {
	case V4L2_XFER_FUNC_709:
	case V4L2_XFER_FUNC_SRGB:
	case V4L2_XFER_FUNC_SMPTE240M:
	case V4L2_XFER_FUNC_NONE:
	case V4L2_XFER_FUNC_DEFAULT:
		break;
	default:
		f->fmt.pix.xfer_func = V4L2_YCBCR_ENC_DEFAULT;
		break;
	}
}

static int avico_try_fmt_common(struct v4l2_format *f)
{
	enum v4l2_field field = f->fmt.pix.field;

	if (field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (field != V4L2_FIELD_NONE)
		return -EINVAL;

	f->fmt.pix.field = field;
	f->fmt.pix.flags = 0;

	v4l_bound_align_image(&f->fmt.pix.width, AVICO_WMIN,
			      AVICO_WMAX, AVICO_WALIGN,
			      &f->fmt.pix.height, AVICO_HMIN,
			      AVICO_HMAX, AVICO_HALIGN, 0);

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_M420:
		f->fmt.pix.bytesperline = round_up(f->fmt.pix.width,
						   16);
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline *
				       f->fmt.pix.height / 2 * 3;
		break;
	case V4L2_PIX_FMT_H264:
		f->fmt.pix.bytesperline = 0;
		/* TODO: Think how to specify sizeimage more intellegently */
		f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height * 2;
		break;
	default:
		/* We don't support other pixel formats */
		__WARN();
	}

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		avico_try_colorspace(f);

	return 0;
}

static int avico_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);
	struct v4l2_fmtdesc *fmt = avico_find_format(ctx,
						     f->fmt.pix.pixelformat,
						     f->type);

	if (!fmt) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Image format (0x%08x) is invalid.\n",
			 f->fmt.pix.pixelformat);

		return -EINVAL;
	}

	return avico_try_fmt_common(f);
}


static int avico_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);
	struct avico_q_data *q_data = get_q_data(ctx, f->type);
	int ret = avico_try_fmt(file, priv, f);

	if (ret)
		return ret;

	if (!q_data)
		return -EINVAL;

	q_data->width = f->fmt.pix.width;
	q_data->height = f->fmt.pix.height;
	q_data->bytesperline = f->fmt.pix.bytesperline;
	q_data->sizeimage = f->fmt.pix.sizeimage;
	q_data->f = avico_find_format(ctx, f->fmt.pix.pixelformat, f->type);

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->colorspace = f->fmt.pix.colorspace;
		ctx->matrix = f->fmt.pix.ycbcr_enc;
		ctx->range = f->fmt.pix.quantization;
		ctx->transfer = f->fmt.pix.xfer_func;
	}

	return avico_g_fmt(file, priv, f);
}

static bool validate_timeperframe(struct v4l2_fract const fract)
{
	if (fract.numerator != 0 && fract.denominator != 0)
		return true;
	else
		return false;
}

/* Simplify a fraction using a simple continued fraction decomposition. The
 * idea here is to convert fractions such as 333333/10000000 to 1/30 using
 * 32 bit arithmetic only. The algorithm is not perfect and relies upon two
 * arbitrary parameters to remove non-significative terms from the simple
 * continued fraction decomposition. Using 8 and 333 for n_terms and threshold
 * respectively seems to give nice results.
 * Adapted from uvc_driver.c.
 */
static struct v4l2_fract simplify_fraction(struct v4l2_fract const fract,
		unsigned int const n_terms, unsigned int const threshold)
{
	uint32_t *an;
	uint32_t x, y, r;
	unsigned int i, n;

	an = kmalloc_array(n_terms, sizeof(*an), GFP_KERNEL);
	if (an == NULL)
		return fract;

	/* Convert the fraction to a simple continued fraction. See
	 * http://mathforum.org/dr.math/faq/faq.fractions.html
	 * Stop if the current term is bigger than or equal to the given
	 * threshold.
	 */
	x = fract.numerator;
	y = fract.denominator;

	for (n = 0; n < n_terms && y != 0; ++n) {
		an[n] = x / y;
		if (an[n] >= threshold) {
			if (n < 2)
				n++;
			break;
		}

		r = x - an[n] * y;
		x = y;
		y = r;
	}

	/* Expand the simple continued fraction back to an integer fraction. */
	x = 0;
	y = 1;

	for (i = n; i > 0; --i) {
		r = y;
		y = an[i-1] * y + x;
		x = r;
	}

	kfree(an);

	return (struct v4l2_fract) { .numerator = y, .denominator = x };
}

static int avico_g_parm(struct file *file, void *fh,
			struct v4l2_streamparm *parm)
{
	struct avico_ctx *ctx = container_of(fh, struct avico_ctx, fh);

	switch (parm->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		parm->parm.output = (struct v4l2_outputparm) {
			.capability = V4L2_CAP_TIMEPERFRAME,
			.timeperframe = ctx->timeperframe
		};
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		parm->parm.capture = (struct v4l2_captureparm) {
			.capability = V4L2_CAP_TIMEPERFRAME,
			.timeperframe = ctx->timeperframe
		};
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int avico_s_parm(struct file *file, void *fh,
			struct v4l2_streamparm *parm)
{
	struct avico_ctx *ctx = container_of(fh, struct avico_ctx, fh);
	struct v4l2_fract *tpf;

	switch (parm->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		tpf = &parm->parm.output.timeperframe;
		if (validate_timeperframe(*tpf))
			ctx->timeperframe = simplify_fraction(*tpf, 8, 333);
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Ignore requested value */
		break;
	default:
		return -EINVAL;
	}

	return avico_g_parm(file, fh, parm);
}

static const struct v4l2_ioctl_ops avico_ioctl_ops = {
	.vidioc_querycap = avico_querycap,

	.vidioc_enum_fmt_vid_out = avico_enum_fmt_output,
	.vidioc_g_fmt_vid_out    = avico_g_fmt,
	.vidioc_s_fmt_vid_out    = avico_s_fmt,
	.vidioc_try_fmt_vid_out  = avico_try_fmt,

	.vidioc_enum_fmt_vid_cap = avico_enum_fmt_capture,
	.vidioc_g_fmt_vid_cap    = avico_g_fmt,
	.vidioc_s_fmt_vid_cap    = avico_s_fmt,
	.vidioc_try_fmt_vid_cap  = avico_try_fmt,

	.vidioc_g_parm    = avico_g_parm,
	.vidioc_s_parm    = avico_s_parm,

	.vidioc_reqbufs   = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf  = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf      = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf     = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf    = v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon  = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe
};

/*
 * Queue operations
 */

static int avico_queue_setup(struct vb2_queue *vq,
			     const void *parg,
			     unsigned int *nbuffers, unsigned int *nplanes,
			     unsigned int sizes[], void *alloc_ctxs[])
{
	struct avico_ctx *ctx = vb2_get_drv_priv(vq);
	struct avico_q_data *q_data = get_q_data(ctx, vq->type);

	if (!q_data)
		return -EINVAL;

	sizes[0] = q_data->sizeimage;
	*nplanes = 1;

	alloc_ctxs[0] = ctx->dev->alloc_ctx;

	pr_devel("Request %d buffer(s) of size %d each.\n", *nbuffers,
		 sizes[0]);

	return 0;
}

static int avico_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct avico_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct avico_q_data *q_data = get_q_data(ctx, vb->type);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	if (vbuf->field != V4L2_FIELD_NONE)
		return -EINVAL;

	if (!q_data)
		return -EINVAL;

	if (vb2_plane_size(vb, 0) < q_data->sizeimage) {
		v4l2_printk(KERN_NOTICE, &ctx->dev->v4l2_dev,
			    "Data will not fit into plane (%lu < %u)\n",
			    vb2_plane_size(vb, 0), q_data->sizeimage);
		return -EINVAL;
	}

	if (ctx->thread_type == AVICO_ENCODER)
		vb2_set_plane_payload(vb, 0, q_data->sizeimage);

	return 0;
}

static void avico_buf_queue(struct vb2_buffer *vb)
{
	struct avico_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int avico_init_streaming(struct avico_ctx *ctx)
{
	struct avico_frame_params *par = &ctx->par;
	int ret = 0;

	if (ctx->out_q.width != ctx->cap_q.width ||
	    ctx->out_q.height != ctx->cap_q.height) {
		v4l2_err(&ctx->dev->v4l2_dev, "can't convert %dx%d to %dx%d\n",
			 ctx->out_q.width, ctx->out_q.height,
			 ctx->cap_q.width, ctx->cap_q.height);
		ret = -EINVAL;
		goto err_ret;
	}

	ctx->outseq = ctx->capseq = 0;

	/* \todo Assert that width and height < 4096 */
	ctx->mbx = DIV_ROUND_UP(ctx->out_q.width, 16);
	ctx->mby = DIV_ROUND_UP(ctx->out_q.height, 16);
	par->dbf = 0;

	par->crop.left = 0;
	par->crop.top = 0;
	par->crop.right = (ctx->mbx * 16 - ctx->out_q.width) / 2;
	par->crop.bottom = (ctx->mby * 16 - ctx->out_q.height) / 2;

	par->frame = 0;
	par->gop = ctx->ctrl_gop->cur.val;
	par->i_period = 0;
	par->poc_type = 2;
	ctx->vdma_trans_size_m1 = 3;

	if (ctx->thread_type == AVICO_ENCODER) {
		ctx->refsize = ctx->mbx * ctx->mby * (256 + 128);
	} else {
		unsigned int refreserve = ctx->mbx * 16 * 12;

		/* + reserve line of half-MBs above mb_ref */
		ctx->refsize = ctx->mbx * ctx->mby * (256 + 128) + refreserve;
	}

	ctx->vref = dma_alloc_coherent(ctx->dev->v4l2_dev.dev, ctx->refsize,
				       &ctx->dmaref, GFP_KERNEL);
	if (!ctx->vref) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Can not allocate memory for reference frame\n");

		return -ENOMEM;
	}

	if (ctx->thread_type == AVICO_ENCODER) {
		ctx->mbrefsize = MB_REF_SIZE * ctx->mbx * 2;
		ctx->vmbref = dma_alloc_coherent(ctx->dev->v4l2_dev.dev,
						 ctx->mbrefsize, &ctx->dmambref,
						 GFP_KERNEL);
		if (!ctx->vmbref) {
			ret = -ENOMEM;
			goto free_ref;
		}

		ctx->mbcursize = MB_CUR_SIZE * ctx->mbx * 2;
		ctx->vmbcur = dma_alloc_coherent(ctx->dev->v4l2_dev.dev,
						 ctx->mbcursize, &ctx->dmambcur,
						 GFP_KERNEL);
		if (!ctx->vmbcur) {
			ret = -ENOMEM;
			goto free_mbref;
		}
	}

	return 0;

free_mbref:
	dma_free_coherent(ctx->dev->v4l2_dev.dev, ctx->mbrefsize, ctx->vmbref,
			  ctx->dmambref);
free_ref:
	dma_free_coherent(ctx->dev->v4l2_dev.dev, ctx->refsize, ctx->vref,
			  ctx->dmaref);
err_ret:
	return ret;
}

static int avico_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct avico_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *buf;
	int ret = 0;
	struct vb2_queue *other_vq;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		other_vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
					 V4L2_BUF_TYPE_VIDEO_CAPTURE);
	else
		other_vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
					 V4L2_BUF_TYPE_VIDEO_OUTPUT);

	/* TODO: Move to avico_open() */
	ret = pm_runtime_get_sync(ctx->dev->dev);
	if (ret < 0) {
		v4l2_err(&ctx->dev->v4l2_dev, "Failed to set runtime PM\n");
		goto err_ret_bufs;
	}

	if (vb2_is_streaming(other_vq)) {
		ret = avico_init_streaming(ctx);
		if (ret)
			goto pm_put;
	}

	return 0;

pm_put:
	pm_runtime_put(ctx->dev->dev);
err_ret_bufs:
	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	}

	return ret;
}

static void avico_stop_streaming(struct vb2_queue *q)
{
	struct avico_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;
	unsigned long flags;
	struct vb2_queue *other_vq;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		other_vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
					 V4L2_BUF_TYPE_VIDEO_CAPTURE);
		while ((vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx))) {
			/*
			 * stop_streaming() could be called asynchronously to
			 * avico_dma_out_callback() so spinlock ensures buffer
			 * isn't added to the done buffers list twice.
			 */
			spin_lock_irqsave(&ctx->dev->irqlock, flags);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
			spin_unlock_irqrestore(&ctx->dev->irqlock, flags);
		}
	} else {
		other_vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
					 V4L2_BUF_TYPE_VIDEO_OUTPUT);
		while ((vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx))) {
			spin_lock_irqsave(&ctx->dev->irqlock, flags);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
			spin_unlock_irqrestore(&ctx->dev->irqlock, flags);
		}
	}

	if (vb2_is_streaming(other_vq)) {
		dma_free_coherent(ctx->dev->v4l2_dev.dev, ctx->refsize,
				  ctx->vref, ctx->dmaref);

		if (ctx->thread_type == AVICO_ENCODER) {
			dma_free_coherent(ctx->dev->v4l2_dev.dev,
					  ctx->mbrefsize, ctx->vmbref,
					  ctx->dmambref);

			dma_free_coherent(ctx->dev->v4l2_dev.dev,
					  ctx->mbcursize, ctx->vmbcur,
					  ctx->dmambcur);
		}
		ctx->aborting = 0;
	}

	pm_runtime_put(ctx->dev->dev);
}

static struct vb2_ops avico_qops = {
	.queue_setup     = avico_queue_setup,
	.buf_prepare     = avico_buf_prepare,
	.buf_queue       = avico_buf_queue,
	.start_streaming = avico_start_streaming,
	.stop_streaming  = avico_stop_streaming,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
};

static int queue_init(void *priv, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct avico_ctx *ctx = priv;
	int rc;

	src->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src->io_modes = VB2_MMAP | VB2_DMABUF;
	src->drv_priv = ctx;
	src->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src->ops = &avico_qops;
	src->mem_ops = &vb2_dma_contig_memops;
	src->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src->lock = &ctx->dev->mutex; /* \todo Do we really need this? */

	rc = vb2_queue_init(src);
	if (rc)
		return rc;

	dst->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst->io_modes = VB2_MMAP | VB2_DMABUF;
	dst->drv_priv = ctx;
	dst->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst->ops = &avico_qops;
	dst->mem_ops = &vb2_dma_contig_memops;
	dst->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst->lock = &ctx->dev->mutex; /* \todo Do we really need this? */

	return vb2_queue_init(dst);
}

static int avico_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avico_ctx *ctx = container_of(ctrl->handler, struct avico_ctx,
					     ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->force_key = true;
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops avico_ctrl_ops = {
	.s_ctrl		= avico_s_ctrl,
};

static int avico_ctrls_create(struct avico_ctx *ctx)
{
	struct avico_dev *dev = ctx->dev;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 4);

	ctx->ctrl_qp_i = v4l2_ctrl_new_std(&ctx->ctrl_handler, &avico_ctrl_ops,
					   V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
					   0, 51, 1, 28);
	ctx->ctrl_qp_p = v4l2_ctrl_new_std(&ctx->ctrl_handler, &avico_ctrl_ops,
					   V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
					   0, 51, 1, 28);
	ctx->ctrl_qpc_off = v4l2_ctrl_new_std(&ctx->ctrl_handler,
					      &avico_ctrl_ops,
				V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET,
					      -12, 12, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &avico_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0, 0, 0, 0);
	ctx->ctrl_gop = v4l2_ctrl_new_std(&ctx->ctrl_handler, &avico_ctrl_ops,
					  V4L2_CID_MPEG_VIDEO_GOP_SIZE,
					  1, 1 << 16, 1, 60);

	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &avico_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
		~BIT(V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE),
		V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE);

	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &avico_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			       V4L2_MPEG_VIDEO_H264_LEVEL_4_0,
			       ~BIT(V4L2_MPEG_VIDEO_H264_LEVEL_4_0),
			       V4L2_MPEG_VIDEO_H264_LEVEL_4_0);

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;

		v4l2_err(&dev->v4l2_dev, "Failed to create controls\n");
		return err;
	}

	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);

	return 0;
}

static void avico_ctrls_delete(struct avico_ctx *ctx)
{
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
}

static int avico_open(struct file *file)
{
	/* \todo This function should
	 * 1. Allocate hardware instance (! no-no-no you have to do it only
	 *    when streamon or similar)
	 * 2. Allocate and initialize device-specific context and m2m_ctx
	 * 3. Setup controls */

	struct avico_dev *dev = video_drvdata(file);
	struct avico_ctx *ctx;
	int rc = 0;
	int i;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		rc = -ENOMEM;
		goto open_unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;

	ctx->thread_type = AVICO_ENCODER;
	/* \todo Should be allocated dynamically */
	ctx->id = 0;

	ctx->timeperframe = (struct v4l2_fract) {
		.numerator = 1,
		.denominator = 25
	};
	ctx->outseq = ctx->capseq = 0;

	ctx->out_q.width = ctx->cap_q.width = 1280;
	ctx->out_q.height = ctx->cap_q.height = 720;

	ctx->colorspace = avico_def_colorspace(ctx->out_q.width,
					       ctx->out_q.height);
	ctx->transfer = V4L2_XFER_FUNC_DEFAULT;
	ctx->matrix = V4L2_YCBCR_ENC_DEFAULT;
	ctx->range = V4L2_QUANTIZATION_DEFAULT;

	ctx->out_q.bytesperline = round_up(ctx->out_q.width, 16);
	ctx->out_q.sizeimage = ctx->out_q.bytesperline *
			       round_up(ctx->out_q.height, 16) * 3 / 2;
	ctx->cap_q.bytesperline = 0;
	ctx->cap_q.sizeimage = ctx->cap_q.width * ctx->cap_q.height * 2;

	ctx->out_q.f = avico_find_format(ctx, V4L2_PIX_FMT_M420,
					 V4L2_BUF_TYPE_VIDEO_OUTPUT);
	ctx->cap_q.f = avico_find_format(ctx, V4L2_PIX_FMT_H264,
					 V4L2_BUF_TYPE_VIDEO_CAPTURE);

	for (i = 0; i < 2; i++) {
		/* \bug Will not work for several threads
		 * \todo Should be allocated dynamically */
		ctx->bounceref[i] = BOUNCE_BUF_BASE + BOUNCE_BUF_SIZE * i;
		ctx->bounceout[i] = BOUNCE_BUF_BASE + BOUNCE_BUF_SIZE * (2 + i);
	}
	spin_lock_init(&ctx->bounce_lock);

	pr_devel("Initializing M2M context...\n");
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);

	if (IS_ERR(ctx->fh.m2m_ctx)) {
		rc = PTR_ERR(ctx->fh.m2m_ctx);

		pr_devel("Can not initizlize M2M context!\n");
		goto error_ctx_free;
	}

	rc = avico_ctrls_create(ctx);
	if (rc)
		goto error_ctrls_del;

	/* Use separate control handler per file handle/context */
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	v4l2_fh_add(&ctx->fh);

	pr_devel("Created instance: %p, m2m_ctx: %p\n", ctx, ctx->fh.m2m_ctx);

	mutex_unlock(&dev->mutex);
	return 0;

error_ctrls_del:
	avico_ctrls_delete(ctx);
error_ctx_free:
	kfree(ctx);
open_unlock:
	mutex_unlock(&dev->mutex);
	return rc;
}

static int avico_release(struct file *file)
{
	struct avico_dev *dev = video_drvdata(file);
	struct avico_ctx *ctx = container_of(file->private_data,
					     struct avico_ctx, fh);

	mutex_lock(&dev->mutex);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dev->mutex);
	avico_ctrls_delete(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations avico_fops = {
	.owner   = THIS_MODULE,
	.open    = avico_open,
	.release = avico_release,
	.poll    = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap    = v4l2_m2m_fop_mmap
};

static struct video_device const avico_video_device = {
	.name      = MODULE_NAME,
	.vfl_dir   = VFL_DIR_M2M,
	.fops      = &avico_fops,
	.ioctl_ops = &avico_ioctl_ops,
	.minor     = -1,
	.release = video_device_release_empty
};

static int avico_clk_get(struct avico_dev *dev)
{
	int i;

	for (i = 0; i < NCLKS; i++) {
		dev->clk[i] = devm_clk_get(dev->dev, clknames[i]);
		if (IS_ERR(dev->clk[i])) {
			int rc = PTR_ERR(dev->clk[i]);

			dev_err(dev->dev, "Failed to get clock %s (%u)\n",
				clknames[i], rc);

			return rc;
		}
	}

	return 0;
}

static int avico_probe(struct platform_device *pdev)
{
	struct avico_dev *dev;
	struct resource *res;
	int ret, irq;
	dma_cap_mask_t mask;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENXIO;
	}

	/* Try to get clocks */
	ret = avico_clk_get(dev);
	if (ret)
		return ret;

	dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!dev->regs) {
		dev_err(&pdev->dev, "Can not map memory region\n");
		return -ENXIO;
	}

	/* VRAM address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get VRAM resource\n");
		return -ENXIO;
	}

	dev->vram = res->start;

	ret = of_reserved_mem_device_init(dev->dev);
	if (ret && ret != -ENODEV)
		dev_info(dev->dev, "Filed to init reserved memory\n");

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	dev->dma_ch = dma_request_channel(mask, NULL, NULL);
	if (!dev->dma_ch) {
		dev_err(&pdev->dev, "Failed to request DMA channel\n");
		ret = -ENXIO;
		goto res_release;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ resource\n");
		ret = irq; /* -ENXIO */
		goto dma_release;
	}

	ret = devm_request_irq(&pdev->dev, irq, avico_irq, 0, pdev->name, dev);
	if (ret)
		goto dma_release;

	/* \todo Request and enable clock */

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto dma_release;

	spin_lock_init(&dev->irqlock);
	mutex_init(&dev->mutex);

	dev->vfd = avico_video_device;
	dev->vfd.lock = &dev->mutex;
	dev->vfd.v4l2_dev = &dev->v4l2_dev;

	dev->m2m_dev = v4l2_m2m_init(&avico_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		v4l2_err(&dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(dev->m2m_dev);
		goto err_m2m;
	}

	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		v4l2_err(&dev->v4l2_dev, "Failed to alloc vb2 context\n");
		ret = PTR_ERR(dev->alloc_ctx);
		goto err_alloc;
	}

	pm_runtime_enable(&pdev->dev);

	ret = video_register_device(&dev->vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto unreg_dev;
	}

	video_set_drvdata(&dev->vfd, dev);
	v4l2_info(&dev->v4l2_dev, "Device registered as /dev/video%d\n",
		dev->vfd.num);
	platform_set_drvdata(pdev, dev);

	return 0;

unreg_dev:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	pm_runtime_disable(&pdev->dev);
err_alloc:
	v4l2_m2m_release(dev->m2m_dev);
err_m2m:
	v4l2_device_unregister(&dev->v4l2_dev);
dma_release:
	dma_release_channel(dev->dma_ch);
res_release:
	of_reserved_mem_device_release(dev->dev);

	return ret;
}

static int avico_remove(struct platform_device *pdev)
{
	struct avico_dev *dev = (struct avico_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " MODULE_NAME "\n");
	video_unregister_device(&dev->vfd);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	v4l2_m2m_release(dev->m2m_dev);
	pm_runtime_disable(&pdev->dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	dma_release_channel(dev->dma_ch);

	of_reserved_mem_device_release(dev->dev);

	return 0;
}

static int __maybe_unused avico_runtime_suspend(struct device *dev)
{
	int i;
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct avico_dev *adev = container_of(v4l2_dev, struct avico_dev,
					      v4l2_dev);

	/* Clocks must be disabled in reverse order by common sense */
	for (i = NCLKS - 1; i >= 0; i--)
		clk_disable_unprepare(adev->clk[i]);

	return 0;
}

static int __maybe_unused avico_runtime_resume(struct device *dev)
{
	int i, rc;
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct avico_dev *adev = container_of(v4l2_dev, struct avico_dev,
					      v4l2_dev);

	for (i = 0; i < NCLKS; i++) {
		rc = clk_prepare_enable(adev->clk[i]);
		if (rc) {
			dev_err(dev, "Failed to enable clock %s (%u)\n",
				clknames[i], rc);

			while (--i >= 0)
				clk_disable_unprepare(adev->clk[i]);

			return rc;
		}
	}

	return 0;
}

/* BUG: System Sleep callbacks must not be executed when the driver is in use.
 * Otherwise the proccess will get stuck and the driver will become unavailable
 * after resuming. */
static int __maybe_unused avico_suspend(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return avico_runtime_suspend(dev);
}

static int __maybe_unused avico_resume(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return avico_runtime_resume(dev);
}

static const struct dev_pm_ops avico_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(avico_suspend, avico_resume)
	SET_RUNTIME_PM_OPS(avico_runtime_suspend, avico_runtime_resume, NULL)
};

static struct platform_device_id avico_platform_ids[] = {
	{ .name = "avico", .driver_data = 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, avico_platform_ids);

#ifdef CONFIG_OF
static const struct of_device_id avico_dt_ids[] = {
	{ .compatible = "elvees,avico", .data = NULL },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, avico_dt_ids);
#endif

static struct platform_driver avico_driver = {
	.probe = avico_probe,
	.remove = avico_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &avico_pm_ops,
		.of_match_table = of_match_ptr(avico_dt_ids)
	},
	.id_table = avico_platform_ids
};

module_platform_driver(avico_driver);

MODULE_AUTHOR("Anton Leontiev <aleontiev@elvees.com>");
MODULE_DESCRIPTION("Avico (a.k.a. VELcore-01) driver");
MODULE_LICENSE("GPL");
