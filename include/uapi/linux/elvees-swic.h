/*
 * Copyright 2018 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef ELVEES_SWIC_H
#define ELVEES_SWIC_H

enum swic_link_state {
	LINK_ERROR_RESET,
	LINK_ERROR_WAIT,
	LINK_READY,
	LINK_STARTED,
	LINK_CONNECTING,
	LINK_RUN
};

#define SWICIOC_MAGIC 'w'

#define SWICIOC_SET_LINK \
	_IO(SWICIOC_MAGIC, 1)

#define SWICIOC_GET_LINK_STATE \
	_IOR(SWICIOC_MAGIC, 2, enum swic_link_state *)

#endif
