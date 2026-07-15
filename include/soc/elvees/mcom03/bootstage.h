/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2025 RnD Center "ELVEES", JSC
 *
 */

#ifndef __SOC_BOOTSTAGE_H
#define __SOC_BOOTSTAGE_H

/*
 * A list of boot stages that we know about. Each of these indicates the
 * state that we are at, and the action that we are about to perform.
 */
enum bootstage_id {
	BOOTSTAGE_ID_RESERVED = 0,
	BOOTSTAGE_ID_SBL_S1_START,
	BOOTSTAGE_ID_DDRINIT_START,
	BOOTSTAGE_ID_SBL_S2_START,
	BOOTSTAGE_ID_SBL_S2_LOAD_START,
	BOOTSTAGE_ID_SBL_S2_LOAD_COMPLETE,
	BOOTSTAGE_ID_SBL_S3_START,
	BOOTSTAGE_ID_TF_A_START,

	// The IDs below has the same value as in the U-Boot in order to reused it
	BOOTSTAGE_ID_RUN_OS = 15,
	BOOTSTAGE_ID_START_UBOOT_F = 178,
	BOOTSTAGE_ID_START_TFTF = BOOTSTAGE_ID_START_UBOOT_F,
};

#endif
