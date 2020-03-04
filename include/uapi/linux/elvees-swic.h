/*
 * Copyright 2018-2019 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef ELVEES_SWIC_H
#define ELVEES_SWIC_H

#include <linux/types.h>

#define ELVEES_SWIC_MAX_PACKET_SIZE (1024*1024)

enum swic_link_state {
	LINK_ERROR_RESET,
	LINK_ERROR_WAIT,
	LINK_READY,
	LINK_STARTED,
	LINK_CONNECTING,
	LINK_RUN
};

enum swic_tx_speed {
	TX_SPEED_2P4 = 255,
	TX_SPEED_4P8 = 0,
	TX_SPEED_72 = 1,
	TX_SPEED_120 = 2,
	TX_SPEED_168 = 3,
	TX_SPEED_216 = 4,
	TX_SPEED_264 = 5,
	TX_SPEED_312 = 6,
	TX_SPEED_360 = 7,
	TX_SPEED_408 = 8
};

struct elvees_swic_speed {
	unsigned int rx;
	unsigned int tx;
};

struct elvees_swic_stats {
	__u64 tx_data_bytes;
	__u64 rx_data_bytes;
	__u32 tx_packets;
	__u32 rx_eop_packets;
	__u32 rx_eep_packets;
	__u32 dc_err;
	__u32 parity_err;
	__u32 escape_err;
	__u32 credit_err;
};

struct elvees_swic_lvds_test {
	unsigned int iters;
	unsigned int s_lvds_0;
	unsigned int s_lvds_1;
	unsigned int d_lvds_0;
	unsigned int d_lvds_1;
};

#define SWICIOC_MAGIC 'w'

#define SWICIOC_SET_LINK \
	_IOW(SWICIOC_MAGIC, 1, unsigned int)

#define SWICIOC_GET_LINK_STATE \
	_IOR(SWICIOC_MAGIC, 2, enum swic_link_state *)

#define SWICIOC_SET_TX_SPEED \
	_IOW(SWICIOC_MAGIC, 3, enum swic_tx_speed)

#define SWICIOC_GET_SPEED \
	_IOR(SWICIOC_MAGIC, 4, struct elvees_swic_speed *)

#define SWICIOC_SET_MTU \
	_IOW(SWICIOC_MAGIC, 5, unsigned long)

#define SWICIOC_GET_MTU \
	_IOR(SWICIOC_MAGIC, 6, unsigned long)

#define SWICIOC_FLUSH \
	_IO(SWICIOC_MAGIC, 7)

#define SWICIOC_GET_STATS \
	_IOR(SWICIOC_MAGIC, 8, struct elvees_swic_stats *)

#define SWICIOC_RESET_STATS \
	_IO(SWICIOC_MAGIC, 9)

#define SWICIOC_LVDS_TEST \
	_IOWR(SWICIOC_MAGIC, 10, struct elvees_swic_lvds_test *)

#endif
