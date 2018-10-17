/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *  Copyright 2018-2019 RnD Center "ELVEES", JSC
 */

#ifndef _LINUX_DELCORE30M_H
#define _LINUX_DELCORE30M_H

#include <stddef.h>
#include <linux/types.h>

#define MAX_CORES 2

#define MAX_SDMA_CHANNELS 8

#define MAX_INPUTS 15
#define MAX_OUTPUTS 15

enum elcore30m_core {
	ELCORE30M_CORE_0 = 0x1,
	ELCORE30M_CORE_1 = 0x2,
	ELCORE30M_CORE_ALL = 0x3,
};

#define BANK_SIZE 0x8000
#define STACK_SIZE 4096

struct delcore30m_firmware {
	unsigned long cores;
	size_t size;
	__u8 *data;
};

enum delcore30m_job_status {
	DELCORE30M_JOB_IDLE,
	DELCORE30M_JOB_ENQUEUED,
	DELCORE30M_JOB_RUNNING,
};

enum delcore30m_job_rc {
	DELCORE30M_JOB_ERROR = -2,
	DELCORE30M_JOB_CANCELLED = -1,
	DELCORE30M_JOB_SUCCESS = 0,
};

enum delcore30m_memory_type {
	DELCORE30M_MEMORY_XYRAM,
	DELCORE30M_MEMORY_SYSTEM,
};

struct delcore30m_buffer {
	int fd;
	enum delcore30m_memory_type type;
	int core_num;
	size_t size;
};

struct delcore30m_job {
	int fd;

	unsigned int inum; /* actual number of input arguments */
	unsigned int onum; /* actual number of output arguments */

	int input[MAX_INPUTS];
	int output[MAX_OUTPUTS];

	int cores_fd;
	int sdmas_fd;

	enum delcore30m_job_status status;
	enum delcore30m_job_rc rc;
};

enum delcore30m_resource_type {
	DELCORE30M_CORE,
	DELCORE30M_SDMA,
};

struct delcore30m_resource {
	int fd;
	enum delcore30m_resource_type type;
	unsigned int num;
	unsigned long mask;
};

struct delcore30m_hardware {
	int ncores;
	size_t xyram_size;
	size_t core_pram_size;
};

enum sdma_channel_type {
	SDMA_CHANNEL_INPUT,
	SDMA_CHANNEL_OUTPUT,
};

struct sdma_channel {
	enum sdma_channel_type type;
	unsigned int num;
};

/*
 * @delcore30m_dmachain: SDMA descriptor chain data
 * @codebuf - descriptor of DDR buffer for SDMA prorgam.
 * TODO: Make @codebuf private.
 * @core - DSP that will receive interrupts from SDMA
 * @external - descriptor of DDR buffer
 * @internal - array of two XYRAM buffers descriptors.
 * @chain - descriptor of buffer with SDMA descriptors.
 * @job - descriptor of job associated with this chain.
 * @channel - SDMA channel info.
 */
struct delcore30m_dmachain {
	int codebuf;
	int core;
	int external;
	int internal[2];
	int chain;
	int job;
	struct sdma_channel channel;
};

struct tile_info {
	__u32 x, y;
	__u32 width, height;
	__u32 stride[2];
};

/*
 * @sdma_descriptor_type: type of sdma descriptor.
 * @SDMA_DESCRIPTOR_E1I1 - descriptor waits event and sends interrupt.
 * @SDMA_DESCRIPTOR_E1I0 - descriptor waits event and doesn't send interrupt.
 * @SDMA_DESCRIPTOR_E0I0 - descriptor doesn't wait event and doesn't send
 *                         interrupt.
 * @SDMA_DESCRIPTOR_E0I1 - descriptor doesn't wait event and sends interrupt.
 */
enum sdma_descriptor_type {
	SDMA_DESCRIPTOR_E1I1,
	SDMA_DESCRIPTOR_E1I0,
	SDMA_DESCRIPTOR_E0I0,
	SDMA_DESCRIPTOR_E0I1,
};

/*
 * @sdma_descriptor: sdma descriptor data.
 * @a0e - offset (in bytes) of external buffer.
 * @a0i - reserved by driver.
 * @asize - offset (in bytes) to the next string of buffer (in case of 2D).
 * @bcnt - string count (in case of 1D buffer bcnt equals 1).
 * @ccr - SDMA configuration register.
 * @acnt - string size.
 * @type - sdma descriptor type.
 * @a_init - offset (in bytes) to the next sdma descriptor. For the last
 *           descriptor this field equals zero.
 */
struct sdma_descriptor {
	__u32 a0e;
	__u32 a0i;
	__u32 astride;
	__u32 bcnt;
	__u32 ccr;
	__u32 asize;
	enum sdma_descriptor_type type;
	__u32 a_init;
};

#define ELCIOC_MAGIC 'e'

#define ELCIOC_JOB_CREATE \
	_IOWR(ELCIOC_MAGIC, 1, struct delcore30m_job *)
#define ELCIOC_JOB_ENQUEUE \
	_IOW(ELCIOC_MAGIC, 2, struct delcore30m_job *)
#define ELCIOC_JOB_STATUS \
	_IOWR(ELCIOC_MAGIC, 3, struct delcore30m_job *)
#define ELCIOC_JOB_CANCEL \
	_IOWR(ELCIOC_MAGIC, 4, struct delcore30m_job *)
#define ELCIOC_BUF_ALLOC \
	_IOW(ELCIOC_MAGIC, 5, struct delcore30m_buffer *)
#define ELCIOC_RESOURCE_REQUEST \
	_IOWR(ELCIOC_MAGIC, 6, struct delcore30m_resource *)
#define ELCIOC_SYS_INFO \
	_IOW(ELCIOC_MAGIC, 8, struct delcore30m_hardware *)
#define ELCIOC_DMACHAIN_SETUP \
	_IOWR(ELCIOC_MAGIC, 9, struct delcore30m_dmachain *)

#endif
