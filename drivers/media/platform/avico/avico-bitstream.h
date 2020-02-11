/*
 * ELVEES Avico (a.k.a. VELcore-01) driver - Bitstream functions
 *
 * Copyright 2015 ELVEES NeoTek JSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Author: Anton Leontiev <aleontiev@elvees.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef AVICO_BITSTREAM_H
#define AVICO_BITSTREAM_H

#include "avico.h"

enum nalu_type_t {
	NALU_SLICE = 1,
	NALU_DPA,
	NALU_DPB,
	NALU_DPC,
	NALU_IDR,
	NALU_SEI,
	NALU_SPS,
	NALU_PPS,
	NALU_AUD,
	NALU_EOSEQ,
	NALU_EOSTREAM,
	NALU_FILL,
	NALU_NONE
};

enum nalu_priority_t {
	NALU_PRIOR_DISPOSABLE,
	NALU_PRIOR_LOW,
	NALU_PRIOR_HIGH,
	NALU_PRIOR_HIGHEST,
	NALU_PRIOR_NONE
};

struct bitstream {
	uint8_t *start;
	uint8_t *end;
	uint8_t *p;
	uint8_t cb;
	uint8_t bitsleft;
	int8_t nulls;
};

struct avico_ctx;

void avico_bitstream_init(struct avico_ctx *ctx, void *ptr, unsigned int size);
void avico_bitstream_write_sps(struct avico_ctx *ctx);
void avico_bitstream_write_pps(struct avico_ctx *ctx);
void avico_bitstream_write_slice_header(struct avico_ctx *ctx);
void avico_bitstream_get64(struct avico_ctx *ctx, uint32_t data[2],
			   unsigned int bits[2]);
int avico_bitstream_ecd_stuff_pos(struct avico_ctx *ctx);
void avico_bitstream_cut64(struct avico_ctx *ctx);
void avico_bitstream_dump(struct bitstream *bs);

#endif /* AVICO_BITSTREAM_H */
