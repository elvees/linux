/*
 *  Copyright 2017 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/linkage.h>
#include <linux/init.h>

ENTRY(mcom02_secondary_startup)
	ldr	r0, =0x38095000		@ PMCTR base address
	mov	r1, #0
	str	r1, [r0, #0x10]		@ disable interrupt
	mov	r1, #0x01
	str	r1, [r0, #0x1c]		@ clear interrupt
	ldr	r0, =0x38096000		@ SMCTR base address
	mov	r1, #0
	str	r1, [r0, #4]		@ remap BOOT to NORMPORT
	mrc 	p15, 4, r0, c15, c0, 0	@ get SCU base address
	mrc     p15, 0, r1, c0, c0, 5	@ read MPIDR
	and	r1, r1, #3		@ mask out cpu ID
	lsl	r1, r1, #3		@ we will shift by cpu_id * 8 bits
	ldr	r2, [r0, #8]		@ SCU Power Status Register
	mov	r3, #3
	lsl	r3, r3, r1
	bic	r2, r2, r3		@ Clear bits of our CPU (Run Mode)
	str	r2, [r0, #8]		@ write back

	bl	v7_invalidate_l1
	b	secondary_startup
ENDPROC(mcom02_secondary_startup)

ENTRY(mcom02_secondary_trampoline)
	ldr	pc, 1f
ENDPROC(mcom02_secondary_trampoline)
	.globl	mcom02_secondary_boot_vector
mcom02_secondary_boot_vector:
1:	.space	4

ENTRY(mcom02_secondary_trampoline_end)
