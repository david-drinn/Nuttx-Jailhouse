/****************************************************************************
 * arch/x86_64/include/intel64/arch.h
 *
 *   Copyright (C) 2011, 2015, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_X86_64_INCLUDE_INTEL64_ARCH_H
#define __ARCH_X86_64_INCLUDE_INTEL64_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RFLAGS bits */

#define X86_64_RFLAGS_CF             (1 << 0)  /* Bit 0:  Carry Flag */
                                              /* Bit 1:  Reserved */
#define X86_64_RFLAGS_PF             (1 << 2)  /* Bit 2:  Parity Flag */
                                              /* Bit 3:  Reserved */
#define X86_64_RFLAGS_AF             (1 << 4)  /* Bit 4:  Auxillary carry Flag */
                                              /* Bit 5:  Reserved */
#define X86_64_RFLAGS_ZF             (1 << 6)  /* Bit 6:  Zero Flag */
#define X86_64_RFLAGS_SF             (1 << 7)  /* Bit 7:  Sign Flag */
#define X86_64_RFLAGS_TF             (1 << 8)  /* Bit 8:  Trap Flag */
#define X86_64_RFLAGS_IF             (1 << 9)  /* Bit 9:  Interrupt Flag */
#define X86_64_RFLAGS_DF             (1 << 10) /* Bit 10: Direction Flag */
#define X86_64_RFLAGS_OF             (1 << 11) /* Bit 11: Overflow Flag */
#define X86_64_RFLAGS_IOPL_SHIFT     (12)      /* Bits 12-13: IOPL mask (286+ only)*/
#define X86_64_RFLAGS_IOPL_MASK      (3 << X86_64_RFLAGS_IOPL_SHIFT)
#define X86_64_RFLAGS_NT             (1 << 14) /* Bit 14: Nested Task */
                                              /* Bit 15: Reserved */
#define X86_64_RFLAGS_RF            (1 << 16) /* Bit 16: Resume Flag (386+ only) */
#define X86_64_RFLAGS_VM            (1 << 17) /* Bit 17: Virtual Mode (386+ only) */
#define X86_64_RFLAGS_AC            (1 << 18) /* Bit 18: Alignment Check (486SX+ only) */
#define X86_64_RFLAGS_VIF           (1 << 19) /* Bit 19: Virtual Interrupt Flag (Pentium+) */
#define X86_64_RFLAGS_VIP           (1 << 20) /* Bit 20: Virtual Interrupt Pending (Pentium+) */
#define X86_64_RFLAGS_ID            (1 << 21) /* Bit 21: CPUID detection flag (Pentium+) */


/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IDT data structures ******************************************************
 *
 * The Interrupt Descriptor Table (IDT) is a data structure used by the x86
 * architecture to implement an interrupt vector table. The IDT is used by the
 * processor to determine the correct response to interrupts and exceptions.
 */

begin_packed_struct struct idt_entry_s
{
  uint16_t lobase;           /* Lower 16-bits of vector address for interrupt */
  uint16_t sel;              /* Kernel segment selector */
  uint8_t  ist;              /* 0..2 bits is Interrupt Stack Table offset, reset is zero */
  uint8_t  flags;            /* (See documentation) */
  uint16_t hibase;           /* Upper 16-bits of vector address for interrupt */
  uint32_t xhibase;          /* Top   32-bits of vector address for interrupt */
  uint32_t zero;             /* reserved */
} end_packed_struct;

/* A struct describing a pointer to an array of interrupt handlers.  This is
 * in a format suitable for giving to 'lidt'.
 */

begin_packed_struct struct idt_ptr_s
{
  uint16_t limit;
  uint64_t base;             /* The address of the first GDT entry */
} end_packed_struct;

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

static inline uint64_t rdtsc(void)
{
	uint32_t lo, hi;

	asm volatile("rdtscp" : "=a" (lo), "=d" (hi)::"memory");
	return (uint64_t)lo | (((uint64_t)hi) << 32);
}

static inline uint64_t _rdtsc(void)
{
	uint32_t lo, hi;

	asm volatile("rdtsc" : "=a" (lo), "=d" (hi)::"memory");
	return (uint64_t)lo | (((uint64_t)hi) << 32);
}


static inline unsigned long read_msr(unsigned int msr)
{
	uint32_t low, high;

	asm volatile("rdmsr" : "=a" (low), "=d" (high) : "c" (msr));
	return low | ((unsigned long)high << 32);
}

static inline void write_msr(unsigned int msr, unsigned long val)
{
	asm volatile("wrmsr"
		: /* no output */
		: "c" (msr), "a" (val), "d" (val >> 32)
		: "memory");
}

/* Return stack pointer */

static inline uint64_t up_getrsp()
{
  uint64_t regval;

  asm volatile(
    "\tmovq %%rsp, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

/* Get segment registers */

static inline uint32_t up_getds()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%ds, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getcs()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%cs, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getss()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%ss, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getes()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%es, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getfs()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%fs, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getgs()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%gs, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}
/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint64_t* pd;

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

void gdt_flush(uint32_t gdt_addr);
void idt_flush(uint32_t idt_addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_INCLUDE_INTEL64_ARCH_H */
