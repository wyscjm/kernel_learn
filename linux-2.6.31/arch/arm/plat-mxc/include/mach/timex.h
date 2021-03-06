/*
 *  Copyright (C) 1999 ARM Limited
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_MXC_TIMEX_H__
#define __ASM_ARCH_MXC_TIMEX_H__

#if defined CONFIG_ARCH_MX1
#define CLOCK_TICK_RATE		16000000
#elif defined CONFIG_ARCH_MX2 || defined CONFIG_ARCH_MX25
#define CLOCK_TICK_RATE		13300000
#elif defined CONFIG_ARCH_MX3 || defined CONFIG_ARCH_MX35
#define CLOCK_TICK_RATE		16625000
#elif defined CONFIG_ARCH_MX37
#define CLOCK_TICK_RATE		8000000
#elif defined CONFIG_ARCH_MX51
#define CLOCK_TICK_RATE		8000000
#endif

#endif				/* __ASM_ARCH_MXC_TIMEX_H__ */
