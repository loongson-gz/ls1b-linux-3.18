/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#ifndef _LOONGSON1_MACHTYPE_H
#define _LOONGSON1_MACHTYPE_H

#include <asm/mips_machine.h>

enum loongson1_mach_type {
	LOONGSON1_MACH_GENERIC_OF = -1,	/* Device tree board */
	LOONGSON1_MACH_GENERIC = 0,
	LOONGSON1_MACH_LSGZ_1B_DEV,		/* guangzhou loongson 1b dev v2.0 board */
	LOONGSON1_MACH_1B_CORE,				/* guangzhou loongson 1b core board */
	LOONGSON1_MACH_1A,					/* guangzhou loongson 1a reference board */
	LOONGSON1_MACH_1C,					/* guangzhou loongson 1c reference board */
};

#if defined(CONFIG_LS1B_DEV_BOARD)
#define LOONGSON1_MACHTYPE		LOONGSON1_MACH_LSGZ_1B_DEV
#elif defined(CONFIG_LS1B_CORE_BOARD)
#define LOONGSON1_MACHTYPE		LOONGSON1_MACH_1B_CORE
#elif defined(CONFIG_LS1C_CBII_V0A_BOARD)
#define LOONGSON1_MACHTYPE		LOONGSON1_MACH_1C
#else
#define LOONGSON1_MACHTYPE		LOONGSON1_MACH_GENERIC
#endif

#endif /* _LOONGSON1_MACHTYPE_H */
