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
#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/clk.h>

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

/* Per queue */
#define AVICO_DEF_NUM_BUFS VIDEO_MAX_FRAME
/* In bytes, per queue */
#define AVICO_QUEUE_MEM_LIMIT (16 * 1024 * 1024)

/* \todo Get value from device tree */
#define XYRAM_BASE 0x3A400000
#define BOUNCE_BUF_BASE XYRAM_BASE
/* Size of macroblock in bytes */
#define MB_SIZE (16 * 16 * 3 / 2)
/* Length of data block VDMA transfers. Should be multiple of 16 bytes. */
#define DMA_CBS_LEN (40 * 8 * 16)
/* Size of a buffer for encoded data in VRAM. Should be multiple of 16 bytes. */
#define SIZE_CBS (80 * 8 * 16)
/*
 * We need 4 bounce buffers BOUNCE_BUF_SIZE each (2 buffers for reconstructed
 * frame and 2 buffers for datastream).
 * Bounce buffer for reconstructed frame should be able to store up to one row
 * of macroblocks.
 * Bounce buffer for datastream should be able to store encoded data whose size
 * is multiple of DMA_CBS_LEN so BOUNCE_BUF_SIZE is rounded up if necessary.
 */
#define BOUNCE_BUF_SIZE roundup(1920 / 16 * MB_SIZE, DMA_CBS_LEN)

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
	return 1;
}

static void avico_abort(void *priv)
{
	/* Required function */
}

void avico_thread_configure(struct avico_ctx *ctx)
{
	union frmn frmn = {
		.frmn = ctx->frame,
		.gop = ctx->gop,
		.idr = ctx->idr,
		.ftype = ctx->frame_type != VE_FR_I
	};

	union cfg cfg = {
		.slice_type = ctx->frame_type,
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

	bool const swap_lines = (ctx->frame % 2) && (ctx->mby % 2);

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

void avico_dma_configure_input(struct avico_ctx *ctx, unsigned const channel)
{
	union adr adr;
	union vdma_acnt acnt;
	union vdma_bccnt bccnt;
	union vdma_hvecnt hvecnt;
	union vdma_hvicnt hvicnt;
	union vdma_cfg cfg = { .val = 0 };

	bool const swap_lines = (ctx->frame % 2) && (ctx->mby % 2);

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
				MB_REF_SIZE * ctx->mbx, ctx, channel,
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

	bool const swap_lines = (ctx->frame % 2) && (ctx->mby % 2);

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

/*
 * avico_run() - prepares and starts VPU
 */
static void avico_run(void *priv)
{
	struct avico_ctx *ctx = priv;
	struct avico_dev *dev = ctx->dev;
	struct vb2_buffer *src, *dst;
	uint8_t *out;
	/* Line of half-MBs reserve above fr_ref */
	/* \todo Rename fr_ref_reserve_offset and fr_size_ram */
	uint32_t const fr_ref_reserve_offset = (ctx->mbx * 16) * 12;
	uint32_t const fr_size_ram = (ctx->mbx * 16) * (ctx->mby * 16) * 3 / 2;
	union smbpos smbpos;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	out = vb2_plane_vaddr(dst, 0);
	WARN_ON(out == NULL);

	ctx->dmainp = vb2_dma_contig_plane_dma_addr(src, 0);
	ctx->dmaout = vb2_dma_contig_plane_dma_addr(dst, 0);

	WARN_ON(ctx->dmainp == 0);
	WARN_ON(ctx->dmaout == 0);

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

	if (!ctx->dmainp || !ctx->dmaout) {
		v4l2_err(&dev->v4l2_dev,
			 "Acquiring kernel pointers to buffers failed\n");
		return;
	}

	/*if (vb2_plane_size(src, 0) > vb2_plane_size(dst, 0)) {
		v4l2_err(&dev->v4l2_dev, "Output buffer is too small\n");
		return;
	}*/

	if (ctx->frame % ctx->gop == 0) {
		ctx->frame_type = VE_FR_I;
		ctx->idr = true;
	} else if (ctx->i_period > 0 && ctx->frame % ctx->i_period == 0) {
		ctx->frame_type = VE_FR_I;
		ctx->idr = false;
	} else {
		ctx->frame_type  = VE_FR_P;
		ctx->idr = false;
	}

	if (ctx->frame_type == VE_FR_I && ctx->idr) {
		ctx->frame = 0;
		avico_bitstream_write_sps_pps(ctx);
	}

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

	/* @bug Will not work for several threads
	 * We should gaurantee, that others will not overwrite MSK_EV. */
	avico_write(0xc0 << (ctx->id * 8), ctx,
		    AVICO_CTRL_BASE + AVICO_CTRL_MSK_EV);
	avico_write(0, ctx, AVICO_CTRL_BASE + AVICO_CTRL_EVENTS);
	/* @bug Following will not work for several threads */
	avico_write(0xc0 << (ctx->id * 8), ctx,
		    AVICO_CTRL_BASE + AVICO_CTRL_MSK_INT);

	/* Need for M6POS */
	/* \todo Add reference to the section in manual. */
	avico_write(1, ctx, AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_OFF);

	/* Run slice data encoding */
	avico_codec_run(ctx);
}

/* This function will call by DMA after coping last output data in frame */
static void avico_dma_out_callback(void *data)
{
	struct avico_ctx *ctx = (struct avico_ctx *)data;
	struct avico_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned long flags;
	uint32_t encoded;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	encoded = avico_read(ctx, AVICO_EC_BASE(ctx->id) + AVICO_EC_PACKER +
				  AVICO_PACKER_CBS_TOTAL_LEN);
	WARN_ON(encoded % 8 != 0);
	ctx->bs.p += encoded / 8;
	vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->bs.p - ctx->bs.start);

	dst->sequence = ctx->capseq++;
	src->sequence = ctx->outseq++;

	memcpy(&dst->timestamp, &src->timestamp,
	       sizeof(struct timeval));
	if (src->flags & V4L2_BUF_FLAG_TIMECODE) {
		memcpy(&dst->timecode, &src->timecode,
		       sizeof(struct v4l2_timecode));
	}

	dst->field = src->field;
	dst->flags = src->flags;

	/* \todo Do not understand why we need irqlock here */
	spin_lock_irqsave(&dev->irqlock, flags);
	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	if (++ctx->frame >= ctx->maxframe)
		ctx->frame = 0;

	v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
}

/* Setup and start DMA for copy data from bounce buffer to DDR */
static void avico_copy_bounce(struct avico_ctx *ctx,
			      dma_async_tx_callback callback)
{
	uint32_t frame_size;
	uint32_t ref_size, out_size;
	struct dma_async_tx_descriptor *tx;

	ref_size = ctx->mbx * MB_SIZE; /* We are working by row of MB */
	out_size = avico_dma_read(ctx, 3, AVICO_VDMA_CHANNEL_AECUR) -
					  ctx->bounceout[ctx->bounce_active];

	/* Bitstream redy not for all rows. We can not run DMA without
	 * buffer length */
	if (out_size) {
		tx = ctx->dev->dma_ch->device->device_prep_dma_memcpy(
			ctx->dev->dma_ch, ctx->dmaout + ctx->out_ptr_off,
			ctx->bounceout[ctx->bounce_active], out_size, 0);
		tx->callback_param = ctx;
		tx->callback = callback;
		dmaengine_submit(tx);
		ctx->out_ptr_off += out_size;
	}

	frame_size = ctx->mbx * ctx->mby * MB_SIZE;
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
		dmaengine_submit(tx);

		ctx->ref_ptr_off = ref_size - end_part_size;
	} else {
		tx = ctx->dev->dma_ch->device->device_prep_dma_memcpy(
			ctx->dev->dma_ch, ctx->dmaref + ctx->ref_ptr_off,
			ctx->bounceref[ctx->bounce_active], ref_size, 0);
		dmaengine_submit(tx);

		ctx->ref_ptr_off += ref_size;
	}
	dma_async_issue_pending(ctx->dev->dma_ch);
}

static irqreturn_t avico_irq(int irq, void *data)
{
	struct avico_dev *dev = (struct avico_dev *)data;
	struct avico_ctx *ctx;
	u32 events;
	bool eof;
	int i;

	/* \todo How does this will work for several HW threads? */
	ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);

	if (ctx == NULL) {
		pr_err("Instance released before the end of transaction\n");
		return IRQ_HANDLED;
	}

	events = avico_read(ctx, AVICO_CTRL_EVENTS);
	eof = events & EV_7;

	if (!eof) {
		/* Wait for EV_0, EV_1 */
		while ((events & EV_0) == 0 || (events & EV_1) == 0)
			events = avico_read(ctx, AVICO_CTRL_EVENTS);

		/* \bug Need delay: rf#2003 */
		for (i = 0; i < 80; i++)
			events = avico_read(ctx, AVICO_CTRL_EVENTS);

		/* Wait for flush of previous MB data */
		while (events != (EV_0 | EV_1 | EV_6))
			events = avico_read(ctx, AVICO_CTRL_EVENTS);

		avico_dma_write(0, ctx, ctx->id * 4 + 2,
				AVICO_VDMA_CHANNEL_RUN);
		avico_dma_write(0, ctx, ctx->id * 4 + 3,
				AVICO_VDMA_CHANNEL_RUN);
	}

	avico_copy_bounce(ctx, eof ? avico_dma_out_callback : NULL);

	/* Swap active buffers */
	ctx->bounce_active ^= 1;
	avico_dma_write(ctx->bounceref[ctx->bounce_active], ctx, 2,
			AVICO_VDMA_CHANNEL_A0E);
	avico_dma_configure_bounceout(ctx, 3);

	/* If not last row then run next row */
	if (!eof) {
		union smbpos smbpos;

		smbpos.val = avico_read(ctx, AVICO_THREAD_BASE(ctx->id) +
					     AVICO_THREAD_SMBPOS);
		smbpos.y6++;  /* Set next stop position */

		/* If next row is last then M6POS will not stop and we
		 * need to enable M6EOF bit */
		if (smbpos.y6 >= ctx->mby) {
			avico_write(0x80,
				    ctx, AVICO_CTRL_BASE + AVICO_CTRL_MSK_INT);
			m6pos_enable(ctx, false);
		} else
			avico_write(smbpos.val, ctx,
				    AVICO_THREAD_BASE(ctx->id) +
				    AVICO_THREAD_SMBPOS);

		/* In M6POS mode before continue we need to OFF and ON thread */
		/* \todo Add reference to the section in manual. */
		avico_write(1, ctx,
			    AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_OFF);
		avico_write(1, ctx,
			    AVICO_THREAD_BASE(ctx->id) + AVICO_THREAD_ON);

		avico_write(0x40 << (ctx->id * 8), ctx,
			    AVICO_CTRL_BASE + AVICO_CTRL_MSK_EV);
		avico_write(0, ctx, AVICO_CTRL_BASE + AVICO_CTRL_EVENTS);
		/* BUG: We do not know how correctly clean events flag before
		 * exit from IRQ handler. If we exit immediately after cleaning
		 * events flag then IRQ handler can be called again. To
		 * prevent this we run DMA channels after cleaning event and
		 * use memory barrier. */
		wmb();

		avico_dma_write(1, ctx, ctx->id * 4 + 2,
				AVICO_VDMA_CHANNEL_RUN);
		avico_dma_write(1, ctx, ctx->id * 4 + 3,
				AVICO_VDMA_CHANNEL_RUN);

		return IRQ_HANDLED;
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

static struct v4l2_fmtdesc output_formats[] = {
	{
		.index = 0,
		.type  = V4L2_BUF_TYPE_VIDEO_OUTPUT,
		.description = "M420",
		.pixelformat = V4L2_PIX_FMT_M420
	}
};

static struct v4l2_fmtdesc capture_formats[] = {
	{
		.index = 0,
		.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
		.description = "H264 Encoded Stream",
		.pixelformat = V4L2_PIX_FMT_H264
	}
};

static int avico_enum_fmt_output(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	if (f->index > ARRAY_SIZE(output_formats))
		return -EINVAL;
	*f = output_formats[f->index];

	return 0;
}

static int avico_enum_fmt_capture(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(capture_formats))
		return -EINVAL;
	*f = capture_formats[f->index];

	return 0;
}

static int avico_g_fmt_output(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);

	f->fmt.pix.width = ctx->width;
	f->fmt.pix.height = ctx->height;
	f->fmt.pix.pixelformat = output_formats[ctx->outfmt].pixelformat;
	f->fmt.pix.field = V4L2_FIELD_NONE;

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_M420:
		f->fmt.pix.bytesperline = ctx->width;
		f->fmt.pix.sizeimage = ctx->outsize;
		break;
	default:
		/* We don't support other pixel formats */
		__WARN();
	}
	f->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
	f->fmt.pix.flags = output_formats[ctx->outfmt].flags;

	return 0;
}

static int avico_g_fmt_capture(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);

	f->fmt.pix.width = ctx->width;
	f->fmt.pix.height = ctx->height;
	f->fmt.pix.pixelformat = capture_formats[ctx->capfmt].pixelformat;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = 0;
	/* TODO: Think how to specify sizeimage more intellegently */
	f->fmt.pix.sizeimage = ctx->capsize;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
	f->fmt.pix.flags = capture_formats[ctx->capfmt].flags;

	return 0;
}

static int avico_s_fmt_output(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);
	unsigned int fmt = 0;

	ctx->width = round_up(f->fmt.pix.width, 16);
	ctx->height = round_up(f->fmt.pix.height, 16);
	ctx->outsize = ctx->width * ctx->height / 2 * 3;
	ctx->capsize = ctx->width * ctx->height * 2;

	while (fmt < ARRAY_SIZE(output_formats) &&
	       f->fmt.pix.pixelformat != output_formats[fmt].pixelformat)
		fmt++;

	if (fmt >= ARRAY_SIZE(output_formats))
		fmt = 0;

	ctx->outfmt = fmt;

	return avico_g_fmt_output(file, priv, f);
}

static int avico_s_fmt_capture(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct avico_ctx *ctx = container_of(priv, struct avico_ctx, fh);
	unsigned int fmt = 0;

	/* Ignore width & height. They are only set for output end. */

	while (fmt < ARRAY_SIZE(capture_formats) &&
	       f->fmt.pix.pixelformat != capture_formats[fmt].pixelformat)
		fmt++;

	if (fmt >= ARRAY_SIZE(capture_formats))
		fmt = 0;

	ctx->capfmt = fmt;

	return avico_g_fmt_capture(file, priv, f);
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
	.vidioc_g_fmt_vid_out    = avico_g_fmt_output,
	.vidioc_s_fmt_vid_out    = avico_s_fmt_output,

	.vidioc_enum_fmt_vid_cap = avico_enum_fmt_capture,
	.vidioc_g_fmt_vid_cap    = avico_g_fmt_capture,
	.vidioc_s_fmt_vid_cap    = avico_s_fmt_capture,

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

	switch (vq->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		sizes[0] = ctx->outsize;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		sizes[0] = ctx->capsize;
		break;
	default:
		return -EINVAL;
	}

	*nbuffers = min(*nbuffers, AVICO_QUEUE_MEM_LIMIT / sizes[0]);
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
	unsigned int size;

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	if (vbuf->field != V4L2_FIELD_NONE)
		return -EINVAL;

	switch (vb->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		size = ctx->outsize;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		size = ctx->capsize;
		break;
	default:
		return -EINVAL;
	}

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_printk(KERN_NOTICE, &ctx->dev->v4l2_dev,
			    "Data will not fit into plane (%lu < %u)\n",
			    vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void avico_buf_queue(struct vb2_buffer *vb)
{
	struct avico_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static void avico_thread_init(struct avico_ctx *ctx)
{
	union mbpos mbpos = {
		.nx = DIV_ROUND_UP(ctx->width, 16),
		.ny = DIV_ROUND_UP(ctx->height, 16)
	};

	union adr adr = {
		.acur = ctx->id * 0x0100,
		.aref = ctx->id * 0x0100 + mbpos.nx,
		.ares = ctx->id * 0x0080 + mbpos.nx
	};

	union task task = {
		.std = VE_STD_H264,
		.qpy = ctx->qpy,
		.qpc = ctx->qpc,
		.m6eof = 1,
		.m7eof = 1
	};

	union frmn frmn = {
		.gop = ctx->gop
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

static void avico_grab_controls(struct avico_ctx *ctx, bool grab)
{
	v4l2_ctrl_grab(ctx->ctrl_qp, grab);
}

static int avico_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct avico_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned int reserve;
	unsigned int channel;

	switch (vq->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		ctx->outseq = 0;
		ctx->outon = 1;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		ctx->capseq = 0;
		ctx->capon = 1;
		break;
	default:
		return -EINVAL;
	}

	if (ctx->outon != 1 || ctx->capon != 1)
		return 0;

	/* \todo Assert that width and height < 4096 */
	ctx->mbx = DIV_ROUND_UP(ctx->width, 16);
	ctx->mby = DIV_ROUND_UP(ctx->height, 16);
	ctx->dbf = 0;

	ctx->frame = 0;
	/* \todo Make configurable.
	 * This should correlate with log2_max_frame_num_minus4 */
	ctx->maxframe = 16;
	ctx->gop = 250;
	ctx->i_period = 0;
	ctx->poc_type = 2;
	ctx->vdma_trans_size_m1 = 3;

	ctx->thread = ctx->dev->regs + AVICO_THREAD_BASE(ctx->id);

	avico_write(0UL, ctx, AVICO_CTRL_BASE + AVICO_CTRL_EVENTS);

	avico_grab_controls(ctx, true);
	avico_thread_init(ctx);

	/* \todo Maybe be do not need padding */
	reserve = ctx->mbx * 16 * 12; /* From Rolschikov's code */
	ctx->refsize = ctx->mbx * ctx->mby * (256 + 128) + reserve;

	ctx->vref = dma_alloc_coherent(ctx->dev->v4l2_dev.dev, ctx->refsize,
				       &ctx->dmaref, GFP_KERNEL);
	if (!ctx->vref) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Can not allocate memory for reference frame");
	}

	for (channel = ctx->id * 4; channel <= ctx->id * 4 + 4; channel++) {
		avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_RUN);
		avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_DONE);
		avico_dma_write(0, ctx, channel, AVICO_VDMA_CHANNEL_IMRDY);
	}

	/* \todo Configure MD */

	avico_ec_init(ctx);

	return 0;
}

static void avico_stop_streaming(struct vb2_queue *q)
{
	struct avico_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;
	unsigned long flags;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (vbuf == NULL)
			break;
		spin_lock_irqsave(&ctx->dev->irqlock, flags);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		spin_unlock_irqrestore(&ctx->dev->irqlock, flags);
	}

	if (ctx->vref) {
		/* \todo Check return value */
		dma_free_coherent(ctx->dev->v4l2_dev.dev, ctx->refsize,
				  ctx->vref, ctx->dmaref);
		ctx->vref = NULL;
		/* \bug This code is not thread safe. To test it one should
		 * write test that simultaneously stops capture and output
		 * streams. */
	}

	/* \bug Only one should be zeroed */
	ctx->capon = ctx->outon = 0;
	avico_grab_controls(ctx, false);
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
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctx->qpy = ctx->qpc = ctrl->val;
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

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 1);

	ctx->ctrl_qp = v4l2_ctrl_new_std(&ctx->ctrl_handler, &avico_ctrl_ops,
					 V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
					 0, 51, 1, 28);

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

	/* \todo Should be allocated dynamically */
	ctx->id = 0;

	ctx->timeperframe = (struct v4l2_fract) {
		.numerator = 1,
		.denominator = 25
	};

	ctx->width = 1280;
	ctx->height = 720;

	ctx->outsize = ctx->width * ctx->height * 3 / 2;
	/* \todo Following is surplus */
	ctx->capsize = round_up(ctx->width * ctx->height * 2, PAGE_SIZE);

	ctx->outseq = ctx->capseq = 0;
	ctx->colorspace = V4L2_COLORSPACE_REC709;

	for (i = 0; i < 2; i++) {
		/* \bug Will not work for several threads
		 * \todo Should be allocated dynamically */
		ctx->bounceref[i] = BOUNCE_BUF_BASE + BOUNCE_BUF_SIZE * i;
		ctx->bounceout[i] = BOUNCE_BUF_BASE + BOUNCE_BUF_SIZE * (2 + i);
	}

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

	avico_ctrls_delete(ctx);
	mutex_lock(&dev->mutex);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dev->mutex);
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

static int avico_clk_init(struct avico_dev *dev, struct platform_device *pdev)
{
	int err;

	dev->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(dev->pclk)) {
		err = PTR_ERR(dev->pclk);
		dev_err(&pdev->dev,
			"failed to get pclk (%u)\n", err);
		return err;
	}

	dev->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(dev->aclk)) {
		err = PTR_ERR(dev->aclk);
		dev_err(&pdev->dev,
			"failed to get aclk (%u)\n", err);
		return err;
	}

	dev->sclk = devm_clk_get(&pdev->dev, "sclk");
	if (IS_ERR(dev->sclk)) {
		err = PTR_ERR(dev->sclk);
		dev_err(&pdev->dev,
			"failed to get sclk (%u)\n", err);
		return err;
	}

	err = clk_prepare_enable(dev->pclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable pclk (%u)\n", err);
		return err;
	}

	err = clk_prepare_enable(dev->aclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable aclk (%u)\n", err);
		goto disable_pclk;
	}

	err = clk_prepare_enable(dev->sclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable sclk (%u)\n", err);
		goto disable_aclk;
	}

	return 0;

disable_aclk:
	clk_disable_unprepare(dev->aclk);
disable_pclk:
	clk_disable_unprepare(dev->pclk);

	return err;
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

	/* Try to get and enable clocks */
	ret = avico_clk_init(dev, pdev);
	if (ret)
		return ret;

	dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!dev->regs) {
		dev_err(&pdev->dev, "Can not map memory region\n");
		ret = -ENXIO;
		goto disable_clks;
	}

	/* VRAM address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get VRAM resource\n");
		ret = -ENXIO;
		goto disable_clks;
	}

	dev->vram = res->start;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	dev->dma_ch = dma_request_channel(mask, NULL, NULL);
	if (!dev->dma_ch) {
		dev_err(&pdev->dev, "Failed to request DMA channel\n");
		ret = -ENXIO;
		goto disable_clks;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ resource\n");
		ret = irq; /* -ENXIO */
		goto disable_clks;
	}

	ret = devm_request_irq(&pdev->dev, irq, avico_irq, 0, pdev->name, dev);
	if (ret)
		goto disable_clks;

	/* \todo Request and enable clock */

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto disable_clks;

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
err_alloc:
	v4l2_m2m_release(dev->m2m_dev);
err_m2m:
	v4l2_device_unregister(&dev->v4l2_dev);
disable_clks:
	if (dev->dma_ch)
		dma_release_channel(dev->dma_ch);
	clk_disable_unprepare(dev->pclk);
	clk_disable_unprepare(dev->aclk);
	clk_disable_unprepare(dev->sclk);

	return ret;
}

static int avico_remove(struct platform_device *pdev)
{
	struct avico_dev *dev = (struct avico_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " MODULE_NAME "\n");
	video_unregister_device(&dev->vfd);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	v4l2_m2m_release(dev->m2m_dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	dma_release_channel(dev->dma_ch);

	clk_disable_unprepare(dev->pclk);
	clk_disable_unprepare(dev->aclk);
	clk_disable_unprepare(dev->sclk);

	return 0;
}

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
		.of_match_table = of_match_ptr(avico_dt_ids)
	},
	.id_table = avico_platform_ids
};

module_platform_driver(avico_driver);

MODULE_AUTHOR("Anton Leontiev <aleontiev@elvees.com>");
MODULE_DESCRIPTION("Avico (a.k.a. VELcore-01) driver");
MODULE_LICENSE("GPL");
