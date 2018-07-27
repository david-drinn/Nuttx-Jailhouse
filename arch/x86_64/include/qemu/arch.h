/****************************************************************************
 * arch/x86_64/include/qemu/arch.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_X86_64_INCLUDE_QEMU_ARCH_H
#define __ARCH_X86_64_INCLUDE_QEMU_ARCH_H

#include <stdint.h>

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define COMM_REGION_BASE 0x100000

#define COMM_REGION_GENERIC_HEADER					\
	/** Message code sent from hypervisor to cell. */		\
	volatile uint32_t msg_to_cell;					\
	/** Reply code sent from cell to hypervisor. */			\
	volatile uint32_t reply_from_cell;					\
	/** Cell state, initialized by hypervisor, updated by cell. */	\
	volatile uint32_t cell_state;					\
	/** \privatesection */						\
	volatile uint32_t padding;						\
	/** \publicsection */

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
struct jailhouse_comm_region {
	COMM_REGION_GENERIC_HEADER;

	/** Base address of PCI memory mapped config (x86-specific). */
	uint64_t pci_mmconfig_base;
	/** I/O port address of the PM timer (x86-specific). */
	uint16_t pm_timer_address;
	/** Number of CPUs available to the cell (x86-specific). */
	uint16_t num_cpus;
	/** Calibrated TSC frequency in kHz (x86-specific). */
	uint32_t tsc_khz;
	/** Calibrated APIC timer frequency in kHz or 0 if TSC deadline timer
	 * is available (x86-specific). */
	uint32_t apic_khz;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#define comm_region     ((struct jailhouse_comm_region *)COMM_REGION_BASE)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_X86_64_INCLUDE_QEMU_ARCH_H */

