/*
 * STMP LCDIF Register Definitions
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * This file is created by xml file. Don't Edit it.
 */

#ifndef __ARCH_ARM___LCDIF_H
#define __ARCH_ARM___LCDIF_H  1

#define REGS_LCDIF_BASE (STMP3XXX_REGS_BASE + 0x30000)
#define REGS_LCDIF_PHYS (0x80030000)
#define REGS_LCDIF_SIZE 0x00002000

#define HW_LCDIF_CTRL	(0x00000000)
#define HW_LCDIF_CTRL_SET	(0x00000004)
#define HW_LCDIF_CTRL_CLR	(0x00000008)
#define HW_LCDIF_CTRL_TOG	(0x0000000c)
#define HW_LCDIF_CTRL_ADDR  \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL)
#define HW_LCDIF_CTRL_SET_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET)
#define HW_LCDIF_CTRL_CLR_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR)
#define HW_LCDIF_CTRL_TOG_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL_TOG)

#define BM_LCDIF_CTRL_SFTRST	0x80000000
#define BM_LCDIF_CTRL_CLKGATE	0x40000000
#define BM_LCDIF_CTRL_YCBCR422_INPUT	0x20000000
#define BM_LCDIF_CTRL_RSRVD0	0x10000000
#define BM_LCDIF_CTRL_WAIT_FOR_VSYNC_EDGE	0x08000000
#define BM_LCDIF_CTRL_DATA_SHIFT_DIR	0x04000000
#define BV_LCDIF_CTRL_DATA_SHIFT_DIR__TXDATA_SHIFT_LEFT  0x0
#define BV_LCDIF_CTRL_DATA_SHIFT_DIR__TXDATA_SHIFT_RIGHT 0x1
#define BP_LCDIF_CTRL_SHIFT_NUM_BITS	21
#define BM_LCDIF_CTRL_SHIFT_NUM_BITS	0x03E00000
#define BF_LCDIF_CTRL_SHIFT_NUM_BITS(v)  \
		(((v) << 21) & BM_LCDIF_CTRL_SHIFT_NUM_BITS)
#define BM_LCDIF_CTRL_DVI_MODE	0x00100000
#define BM_LCDIF_CTRL_BYPASS_COUNT	0x00080000
#define BM_LCDIF_CTRL_VSYNC_MODE	0x00040000
#define BM_LCDIF_CTRL_DOTCLK_MODE	0x00020000
#define BM_LCDIF_CTRL_DATA_SELECT	0x00010000
#define BV_LCDIF_CTRL_DATA_SELECT__CMD_MODE  0x0
#define BV_LCDIF_CTRL_DATA_SELECT__DATA_MODE 0x1
#define BP_LCDIF_CTRL_INPUT_DATA_SWIZZLE	14
#define BM_LCDIF_CTRL_INPUT_DATA_SWIZZLE	0x0000C000
#define BF_LCDIF_CTRL_INPUT_DATA_SWIZZLE(v)  \
		(((v) << 14) & BM_LCDIF_CTRL_INPUT_DATA_SWIZZLE)
#define BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__NO_SWAP         0x0
#define BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__LITTLE_ENDIAN   0x0
#define BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__BIG_ENDIAN_SWAP 0x1
#define BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__SWAP_ALL_BYTES  0x1
#define BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__HWD_SWAP        0x2
#define BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__HWD_BYTE_SWAP   0x3
#define BP_LCDIF_CTRL_CSC_DATA_SWIZZLE	12
#define BM_LCDIF_CTRL_CSC_DATA_SWIZZLE	0x00003000
#define BF_LCDIF_CTRL_CSC_DATA_SWIZZLE(v)  \
		(((v) << 12) & BM_LCDIF_CTRL_CSC_DATA_SWIZZLE)
#define BV_LCDIF_CTRL_CSC_DATA_SWIZZLE__NO_SWAP         0x0
#define BV_LCDIF_CTRL_CSC_DATA_SWIZZLE__LITTLE_ENDIAN   0x0
#define BV_LCDIF_CTRL_CSC_DATA_SWIZZLE__BIG_ENDIAN_SWAP 0x1
#define BV_LCDIF_CTRL_CSC_DATA_SWIZZLE__SWAP_ALL_BYTES  0x1
#define BV_LCDIF_CTRL_CSC_DATA_SWIZZLE__HWD_SWAP        0x2
#define BV_LCDIF_CTRL_CSC_DATA_SWIZZLE__HWD_BYTE_SWAP   0x3
#define BP_LCDIF_CTRL_LCD_DATABUS_WIDTH	10
#define BM_LCDIF_CTRL_LCD_DATABUS_WIDTH	0x00000C00
#define BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(v)  \
		(((v) << 10) & BM_LCDIF_CTRL_LCD_DATABUS_WIDTH)
#define BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__16_BIT 0x0
#define BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__8_BIT  0x1
#define BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__18_BIT 0x2
#define BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__24_BIT 0x3
#define BP_LCDIF_CTRL_WORD_LENGTH	8
#define BM_LCDIF_CTRL_WORD_LENGTH	0x00000300
#define BF_LCDIF_CTRL_WORD_LENGTH(v)  \
		(((v) << 8) & BM_LCDIF_CTRL_WORD_LENGTH)
#define BV_LCDIF_CTRL_WORD_LENGTH__16_BIT 0x0
#define BV_LCDIF_CTRL_WORD_LENGTH__8_BIT  0x1
#define BV_LCDIF_CTRL_WORD_LENGTH__18_BIT 0x2
#define BV_LCDIF_CTRL_WORD_LENGTH__24_BIT 0x3
#define BM_LCDIF_CTRL_RGB_TO_YCBCR422_CSC	0x00000080
#define BM_LCDIF_CTRL_ENABLE_PXP_HANDSHAKE	0x00000040
#define BM_LCDIF_CTRL_LCDIF_MASTER	0x00000020
#define BM_LCDIF_CTRL_DMA_BURST_LENGTH	0x00000010
#define BM_LCDIF_CTRL_DATA_FORMAT_16_BIT	0x00000008
#define BM_LCDIF_CTRL_DATA_FORMAT_18_BIT	0x00000004
#define BV_LCDIF_CTRL_DATA_FORMAT_18_BIT__LOWER_18_BITS_VALID 0x0
#define BV_LCDIF_CTRL_DATA_FORMAT_18_BIT__UPPER_18_BITS_VALID 0x1
#define BM_LCDIF_CTRL_DATA_FORMAT_24_BIT	0x00000002
#define BV_LCDIF_CTRL_DATA_FORMAT_24_BIT__ALL_24_BITS_VALID          0x0
#define BV_LCDIF_CTRL_DATA_FORMAT_24_BIT__DROP_UPPER_2_BITS_PER_BYTE 0x1
#define BM_LCDIF_CTRL_RUN	0x00000001

#define HW_LCDIF_CTRL1	(0x00000010)
#define HW_LCDIF_CTRL1_SET	(0x00000014)
#define HW_LCDIF_CTRL1_CLR	(0x00000018)
#define HW_LCDIF_CTRL1_TOG	(0x0000001c)
#define HW_LCDIF_CTRL1_ADDR  \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL1)
#define HW_LCDIF_CTRL1_SET_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET)
#define HW_LCDIF_CTRL1_CLR_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR)
#define HW_LCDIF_CTRL1_TOG_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CTRL1_TOG)

#define BP_LCDIF_CTRL1_RSRVD1	27
#define BM_LCDIF_CTRL1_RSRVD1	0xF8000000
#define BF_LCDIF_CTRL1_RSRVD1(v) \
		(((v) << 27) & BM_LCDIF_CTRL1_RSRVD1)
#define BM_LCDIF_CTRL1_BM_ERROR_IRQ_EN	0x04000000
#define BM_LCDIF_CTRL1_BM_ERROR_IRQ	0x02000000
#define BV_LCDIF_CTRL1_BM_ERROR_IRQ__NO_REQUEST 0x0
#define BV_LCDIF_CTRL1_BM_ERROR_IRQ__REQUEST    0x1
#define BM_LCDIF_CTRL1_RECOVER_ON_UNDERFLOW	0x01000000
#define BM_LCDIF_CTRL1_INTERLACE_FIELDS	0x00800000
#define BM_LCDIF_CTRL1_START_INTERLACE_FROM_SECOND_FIELD	0x00400000
#define BM_LCDIF_CTRL1_FIFO_CLEAR	0x00200000
#define BM_LCDIF_CTRL1_IRQ_ON_ALTERNATE_FIELDS	0x00100000
#define BP_LCDIF_CTRL1_BYTE_PACKING_FORMAT	16
#define BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT	0x000F0000
#define BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(v)  \
		(((v) << 16) & BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT)
#define BM_LCDIF_CTRL1_OVERFLOW_IRQ_EN	0x00008000
#define BM_LCDIF_CTRL1_UNDERFLOW_IRQ_EN	0x00004000
#define BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN	0x00002000
#define BM_LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN	0x00001000
#define BM_LCDIF_CTRL1_OVERFLOW_IRQ	0x00000800
#define BV_LCDIF_CTRL1_OVERFLOW_IRQ__NO_REQUEST 0x0
#define BV_LCDIF_CTRL1_OVERFLOW_IRQ__REQUEST    0x1
#define BM_LCDIF_CTRL1_UNDERFLOW_IRQ	0x00000400
#define BV_LCDIF_CTRL1_UNDERFLOW_IRQ__NO_REQUEST 0x0
#define BV_LCDIF_CTRL1_UNDERFLOW_IRQ__REQUEST    0x1
#define BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ	0x00000200
#define BV_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ__NO_REQUEST 0x0
#define BV_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ__REQUEST    0x1
#define BM_LCDIF_CTRL1_VSYNC_EDGE_IRQ	0x00000100
#define BV_LCDIF_CTRL1_VSYNC_EDGE_IRQ__NO_REQUEST 0x0
#define BV_LCDIF_CTRL1_VSYNC_EDGE_IRQ__REQUEST    0x1
#define BM_LCDIF_CTRL1_RSRVD0	0x00000080
#define BM_LCDIF_CTRL1_PAUSE_TRANSFER	0x00000040
#define BM_LCDIF_CTRL1_PAUSE_TRANSFER_IRQ_EN	0x00000020
#define BM_LCDIF_CTRL1_PAUSE_TRANSFER_IRQ	0x00000010
#define BV_LCDIF_CTRL1_PAUSE_TRANSFER_IRQ__NO_REQUEST 0x0
#define BV_LCDIF_CTRL1_PAUSE_TRANSFER_IRQ__REQUEST    0x1
#define BM_LCDIF_CTRL1_LCD_CS_CTRL	0x00000008
#define BM_LCDIF_CTRL1_BUSY_ENABLE	0x00000004
#define BV_LCDIF_CTRL1_BUSY_ENABLE__BUSY_DISABLED 0x0
#define BV_LCDIF_CTRL1_BUSY_ENABLE__BUSY_ENABLED  0x1
#define BM_LCDIF_CTRL1_MODE86	0x00000002
#define BV_LCDIF_CTRL1_MODE86__8080_MODE 0x0
#define BV_LCDIF_CTRL1_MODE86__6800_MODE 0x1
#define BM_LCDIF_CTRL1_RESET	0x00000001
#define BV_LCDIF_CTRL1_RESET__LCDRESET_LOW  0x0
#define BV_LCDIF_CTRL1_RESET__LCDRESET_HIGH 0x1

#define HW_LCDIF_TRANSFER_COUNT	(0x00000020)
#define HW_LCDIF_TRANSFER_COUNT_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT)

#define BP_LCDIF_TRANSFER_COUNT_V_COUNT	16
#define BM_LCDIF_TRANSFER_COUNT_V_COUNT	0xFFFF0000
#define BF_LCDIF_TRANSFER_COUNT_V_COUNT(v) \
		(((v) << 16) & BM_LCDIF_TRANSFER_COUNT_V_COUNT)
#define BP_LCDIF_TRANSFER_COUNT_H_COUNT	0
#define BM_LCDIF_TRANSFER_COUNT_H_COUNT	0x0000FFFF
#define BF_LCDIF_TRANSFER_COUNT_H_COUNT(v)  \
		(((v) << 0) & BM_LCDIF_TRANSFER_COUNT_H_COUNT)

#define HW_LCDIF_CUR_BUF	(0x00000030)
#define HW_LCDIF_CUR_BUF_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CUR_BUF)

#define BP_LCDIF_CUR_BUF_ADDR	0
#define BM_LCDIF_CUR_BUF_ADDR	0xFFFFFFFF
#define BF_LCDIF_CUR_BUF_ADDR(v)	(v)

#define HW_LCDIF_NEXT_BUF	(0x00000040)
#define HW_LCDIF_NEXT_BUF_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF)

#define BP_LCDIF_NEXT_BUF_ADDR	0
#define BM_LCDIF_NEXT_BUF_ADDR	0xFFFFFFFF
#define BF_LCDIF_NEXT_BUF_ADDR(v)	(v)

#define HW_LCDIF_PAGETABLE	(0x00000050)
#define HW_LCDIF_PAGETABLE_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_PAGETABLE)

#define BP_LCDIF_PAGETABLE_BASE	14
#define BM_LCDIF_PAGETABLE_BASE	0xFFFFC000
#define BF_LCDIF_PAGETABLE_BASE(v) \
		(((v) << 14) & BM_LCDIF_PAGETABLE_BASE)
#define BP_LCDIF_PAGETABLE_RSVD1	2
#define BM_LCDIF_PAGETABLE_RSVD1	0x00003FFC
#define BF_LCDIF_PAGETABLE_RSVD1(v)  \
		(((v) << 2) & BM_LCDIF_PAGETABLE_RSVD1)
#define BM_LCDIF_PAGETABLE_FLUSH	0x00000002
#define BM_LCDIF_PAGETABLE_ENABLE	0x00000001

#define HW_LCDIF_TIMING	(0x00000060)
#define HW_LCDIF_TIMING_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_TIMING)

#define BP_LCDIF_TIMING_CMD_HOLD	24
#define BM_LCDIF_TIMING_CMD_HOLD	0xFF000000
#define BF_LCDIF_TIMING_CMD_HOLD(v) \
		(((v) << 24) & BM_LCDIF_TIMING_CMD_HOLD)
#define BP_LCDIF_TIMING_CMD_SETUP	16
#define BM_LCDIF_TIMING_CMD_SETUP	0x00FF0000
#define BF_LCDIF_TIMING_CMD_SETUP(v)  \
		(((v) << 16) & BM_LCDIF_TIMING_CMD_SETUP)
#define BP_LCDIF_TIMING_DATA_HOLD	8
#define BM_LCDIF_TIMING_DATA_HOLD	0x0000FF00
#define BF_LCDIF_TIMING_DATA_HOLD(v)  \
		(((v) << 8) & BM_LCDIF_TIMING_DATA_HOLD)
#define BP_LCDIF_TIMING_DATA_SETUP	0
#define BM_LCDIF_TIMING_DATA_SETUP	0x000000FF
#define BF_LCDIF_TIMING_DATA_SETUP(v)  \
		(((v) << 0) & BM_LCDIF_TIMING_DATA_SETUP)

#define HW_LCDIF_VDCTRL0	(0x00000070)
#define HW_LCDIF_VDCTRL0_SET	(0x00000074)
#define HW_LCDIF_VDCTRL0_CLR	(0x00000078)
#define HW_LCDIF_VDCTRL0_TOG	(0x0000007c)
#define HW_LCDIF_VDCTRL0_ADDR  \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0)
#define HW_LCDIF_VDCTRL0_SET_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0_SET)
#define HW_LCDIF_VDCTRL0_CLR_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0_CLR)
#define HW_LCDIF_VDCTRL0_TOG_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0_TOG)

#define BP_LCDIF_VDCTRL0_RSRVD2	30
#define BM_LCDIF_VDCTRL0_RSRVD2	0xC0000000
#define BF_LCDIF_VDCTRL0_RSRVD2(v) \
		(((v) << 30) & BM_LCDIF_VDCTRL0_RSRVD2)
#define BM_LCDIF_VDCTRL0_VSYNC_OEB	0x20000000
#define BV_LCDIF_VDCTRL0_VSYNC_OEB__VSYNC_OUTPUT 0x0
#define BV_LCDIF_VDCTRL0_VSYNC_OEB__VSYNC_INPUT  0x1
#define BM_LCDIF_VDCTRL0_ENABLE_PRESENT	0x10000000
#define BM_LCDIF_VDCTRL0_VSYNC_POL	0x08000000
#define BM_LCDIF_VDCTRL0_HSYNC_POL	0x04000000
#define BM_LCDIF_VDCTRL0_DOTCLK_POL	0x02000000
#define BM_LCDIF_VDCTRL0_ENABLE_POL	0x01000000
#define BP_LCDIF_VDCTRL0_RSRVD1	22
#define BM_LCDIF_VDCTRL0_RSRVD1	0x00C00000
#define BF_LCDIF_VDCTRL0_RSRVD1(v)  \
		(((v) << 22) & BM_LCDIF_VDCTRL0_RSRVD1)
#define BM_LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT	0x00200000
#define BM_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT	0x00100000
#define BM_LCDIF_VDCTRL0_HALF_LINE	0x00080000
#define BM_LCDIF_VDCTRL0_HALF_LINE_MODE	0x00040000
#define BP_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH	0
#define BM_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH	0x0003FFFF
#define BF_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH(v)  \
		(((v) << 0) & BM_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH)

#define HW_LCDIF_VDCTRL1	(0x00000080)
#define HW_LCDIF_VDCTRL1_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL1)

#define BP_LCDIF_VDCTRL1_VSYNC_PERIOD	0
#define BM_LCDIF_VDCTRL1_VSYNC_PERIOD	0xFFFFFFFF
#define BF_LCDIF_VDCTRL1_VSYNC_PERIOD(v)	(v)

#define HW_LCDIF_VDCTRL2	(0x00000090)
#define HW_LCDIF_VDCTRL2_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL2)

#define BP_LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH	24
#define BM_LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH	0xFF000000
#define BF_LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH(v) \
		(((v) << 24) & BM_LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH)
#define BP_LCDIF_VDCTRL2_RSRVD0	18
#define BM_LCDIF_VDCTRL2_RSRVD0	0x00FC0000
#define BF_LCDIF_VDCTRL2_RSRVD0(v)  \
		(((v) << 18) & BM_LCDIF_VDCTRL2_RSRVD0)
#define BP_LCDIF_VDCTRL2_HSYNC_PERIOD	0
#define BM_LCDIF_VDCTRL2_HSYNC_PERIOD	0x0003FFFF
#define BF_LCDIF_VDCTRL2_HSYNC_PERIOD(v)  \
		(((v) << 0) & BM_LCDIF_VDCTRL2_HSYNC_PERIOD)

#define HW_LCDIF_VDCTRL3	(0x000000a0)
#define HW_LCDIF_VDCTRL3_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL3)

#define BP_LCDIF_VDCTRL3_RSRVD0	30
#define BM_LCDIF_VDCTRL3_RSRVD0	0xC0000000
#define BF_LCDIF_VDCTRL3_RSRVD0(v) \
		(((v) << 30) & BM_LCDIF_VDCTRL3_RSRVD0)
#define BM_LCDIF_VDCTRL3_MUX_SYNC_SIGNALS	0x20000000
#define BM_LCDIF_VDCTRL3_VSYNC_ONLY	0x10000000
#define BP_LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT	16
#define BM_LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT	0x0FFF0000
#define BF_LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT(v)  \
		(((v) << 16) & BM_LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT)
#define BP_LCDIF_VDCTRL3_VERTICAL_WAIT_CNT	0
#define BM_LCDIF_VDCTRL3_VERTICAL_WAIT_CNT	0x0000FFFF
#define BF_LCDIF_VDCTRL3_VERTICAL_WAIT_CNT(v)  \
		(((v) << 0) & BM_LCDIF_VDCTRL3_VERTICAL_WAIT_CNT)

#define HW_LCDIF_VDCTRL4	(0x000000b0)
#define HW_LCDIF_VDCTRL4_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4)

#define BP_LCDIF_VDCTRL4_RSRVD0	19
#define BM_LCDIF_VDCTRL4_RSRVD0	0xFFF80000
#define BF_LCDIF_VDCTRL4_RSRVD0(v) \
		(((v) << 19) & BM_LCDIF_VDCTRL4_RSRVD0)
#define BM_LCDIF_VDCTRL4_SYNC_SIGNALS_ON	0x00040000
#define BP_LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT	0
#define BM_LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT	0x0003FFFF
#define BF_LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT(v)  \
		(((v) << 0) & BM_LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT)

#define HW_LCDIF_DVICTRL0	(0x000000c0)
#define HW_LCDIF_DVICTRL0_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL0)

#define BM_LCDIF_DVICTRL0_START_TRS	0x80000000
#define BP_LCDIF_DVICTRL0_H_ACTIVE_CNT	20
#define BM_LCDIF_DVICTRL0_H_ACTIVE_CNT	0x7FF00000
#define BF_LCDIF_DVICTRL0_H_ACTIVE_CNT(v)  \
		(((v) << 20) & BM_LCDIF_DVICTRL0_H_ACTIVE_CNT)
#define BP_LCDIF_DVICTRL0_H_BLANKING_CNT	10
#define BM_LCDIF_DVICTRL0_H_BLANKING_CNT	0x000FFC00
#define BF_LCDIF_DVICTRL0_H_BLANKING_CNT(v)  \
		(((v) << 10) & BM_LCDIF_DVICTRL0_H_BLANKING_CNT)
#define BP_LCDIF_DVICTRL0_V_LINES_CNT	0
#define BM_LCDIF_DVICTRL0_V_LINES_CNT	0x000003FF
#define BF_LCDIF_DVICTRL0_V_LINES_CNT(v)  \
		(((v) << 0) & BM_LCDIF_DVICTRL0_V_LINES_CNT)

#define HW_LCDIF_DVICTRL1	(0x000000d0)
#define HW_LCDIF_DVICTRL1_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL1)

#define BP_LCDIF_DVICTRL1_RSRVD0	30
#define BM_LCDIF_DVICTRL1_RSRVD0	0xC0000000
#define BF_LCDIF_DVICTRL1_RSRVD0(v) \
		(((v) << 30) & BM_LCDIF_DVICTRL1_RSRVD0)
#define BP_LCDIF_DVICTRL1_F1_START_LINE	20
#define BM_LCDIF_DVICTRL1_F1_START_LINE	0x3FF00000
#define BF_LCDIF_DVICTRL1_F1_START_LINE(v)  \
		(((v) << 20) & BM_LCDIF_DVICTRL1_F1_START_LINE)
#define BP_LCDIF_DVICTRL1_F1_END_LINE	10
#define BM_LCDIF_DVICTRL1_F1_END_LINE	0x000FFC00
#define BF_LCDIF_DVICTRL1_F1_END_LINE(v)  \
		(((v) << 10) & BM_LCDIF_DVICTRL1_F1_END_LINE)
#define BP_LCDIF_DVICTRL1_F2_START_LINE	0
#define BM_LCDIF_DVICTRL1_F2_START_LINE	0x000003FF
#define BF_LCDIF_DVICTRL1_F2_START_LINE(v)  \
		(((v) << 0) & BM_LCDIF_DVICTRL1_F2_START_LINE)

#define HW_LCDIF_DVICTRL2	(0x000000e0)
#define HW_LCDIF_DVICTRL2_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL2)

#define BP_LCDIF_DVICTRL2_RSRVD0	30
#define BM_LCDIF_DVICTRL2_RSRVD0	0xC0000000
#define BF_LCDIF_DVICTRL2_RSRVD0(v) \
		(((v) << 30) & BM_LCDIF_DVICTRL2_RSRVD0)
#define BP_LCDIF_DVICTRL2_F2_END_LINE	20
#define BM_LCDIF_DVICTRL2_F2_END_LINE	0x3FF00000
#define BF_LCDIF_DVICTRL2_F2_END_LINE(v)  \
		(((v) << 20) & BM_LCDIF_DVICTRL2_F2_END_LINE)
#define BP_LCDIF_DVICTRL2_V1_BLANK_START_LINE	10
#define BM_LCDIF_DVICTRL2_V1_BLANK_START_LINE	0x000FFC00
#define BF_LCDIF_DVICTRL2_V1_BLANK_START_LINE(v)  \
		(((v) << 10) & BM_LCDIF_DVICTRL2_V1_BLANK_START_LINE)
#define BP_LCDIF_DVICTRL2_V1_BLANK_END_LINE	0
#define BM_LCDIF_DVICTRL2_V1_BLANK_END_LINE	0x000003FF
#define BF_LCDIF_DVICTRL2_V1_BLANK_END_LINE(v)  \
		(((v) << 0) & BM_LCDIF_DVICTRL2_V1_BLANK_END_LINE)

#define HW_LCDIF_DVICTRL3	(0x000000f0)
#define HW_LCDIF_DVICTRL3_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL3)

#define BP_LCDIF_DVICTRL3_RSRVD1	26
#define BM_LCDIF_DVICTRL3_RSRVD1	0xFC000000
#define BF_LCDIF_DVICTRL3_RSRVD1(v) \
		(((v) << 26) & BM_LCDIF_DVICTRL3_RSRVD1)
#define BP_LCDIF_DVICTRL3_V2_BLANK_START_LINE	16
#define BM_LCDIF_DVICTRL3_V2_BLANK_START_LINE	0x03FF0000
#define BF_LCDIF_DVICTRL3_V2_BLANK_START_LINE(v)  \
		(((v) << 16) & BM_LCDIF_DVICTRL3_V2_BLANK_START_LINE)
#define BP_LCDIF_DVICTRL3_RSRVD0	10
#define BM_LCDIF_DVICTRL3_RSRVD0	0x0000FC00
#define BF_LCDIF_DVICTRL3_RSRVD0(v)  \
		(((v) << 10) & BM_LCDIF_DVICTRL3_RSRVD0)
#define BP_LCDIF_DVICTRL3_V2_BLANK_END_LINE	0
#define BM_LCDIF_DVICTRL3_V2_BLANK_END_LINE	0x000003FF
#define BF_LCDIF_DVICTRL3_V2_BLANK_END_LINE(v)  \
		(((v) << 0) & BM_LCDIF_DVICTRL3_V2_BLANK_END_LINE)

#define HW_LCDIF_DVICTRL4	(0x00000100)
#define HW_LCDIF_DVICTRL4_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL4)

#define BP_LCDIF_DVICTRL4_Y_FILL_VALUE	24
#define BM_LCDIF_DVICTRL4_Y_FILL_VALUE	0xFF000000
#define BF_LCDIF_DVICTRL4_Y_FILL_VALUE(v) \
		(((v) << 24) & BM_LCDIF_DVICTRL4_Y_FILL_VALUE)
#define BP_LCDIF_DVICTRL4_CB_FILL_VALUE	16
#define BM_LCDIF_DVICTRL4_CB_FILL_VALUE	0x00FF0000
#define BF_LCDIF_DVICTRL4_CB_FILL_VALUE(v)  \
		(((v) << 16) & BM_LCDIF_DVICTRL4_CB_FILL_VALUE)
#define BP_LCDIF_DVICTRL4_CR_FILL_VALUE	8
#define BM_LCDIF_DVICTRL4_CR_FILL_VALUE	0x0000FF00
#define BF_LCDIF_DVICTRL4_CR_FILL_VALUE(v)  \
		(((v) << 8) & BM_LCDIF_DVICTRL4_CR_FILL_VALUE)
#define BP_LCDIF_DVICTRL4_H_FILL_CNT	0
#define BM_LCDIF_DVICTRL4_H_FILL_CNT	0x000000FF
#define BF_LCDIF_DVICTRL4_H_FILL_CNT(v)  \
		(((v) << 0) & BM_LCDIF_DVICTRL4_H_FILL_CNT)

#define HW_LCDIF_CSC_COEFF0	(0x00000110)
#define HW_LCDIF_CSC_COEFF0_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF0)

#define BP_LCDIF_CSC_COEFF0_RSRVD1	26
#define BM_LCDIF_CSC_COEFF0_RSRVD1	0xFC000000
#define BF_LCDIF_CSC_COEFF0_RSRVD1(v) \
		(((v) << 26) & BM_LCDIF_CSC_COEFF0_RSRVD1)
#define BP_LCDIF_CSC_COEFF0_C0	16
#define BM_LCDIF_CSC_COEFF0_C0	0x03FF0000
#define BF_LCDIF_CSC_COEFF0_C0(v)  \
		(((v) << 16) & BM_LCDIF_CSC_COEFF0_C0)
#define BP_LCDIF_CSC_COEFF0_RSRVD0	2
#define BM_LCDIF_CSC_COEFF0_RSRVD0	0x0000FFFC
#define BF_LCDIF_CSC_COEFF0_RSRVD0(v)  \
		(((v) << 2) & BM_LCDIF_CSC_COEFF0_RSRVD0)
#define BP_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER	0
#define BM_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER	0x00000003
#define BF_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER(v)  \
		(((v) << 0) & BM_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER)
#define BV_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER__SAMPLE_AND_HOLD 0x0
#define BV_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER__RSRVD           0x1
#define BV_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER__INTERSTITIAL    0x2
#define BV_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER__COSITED         0x3

#define HW_LCDIF_CSC_COEFF1	(0x00000120)
#define HW_LCDIF_CSC_COEFF1_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF1)

#define BP_LCDIF_CSC_COEFF1_RSRVD1	26
#define BM_LCDIF_CSC_COEFF1_RSRVD1	0xFC000000
#define BF_LCDIF_CSC_COEFF1_RSRVD1(v) \
		(((v) << 26) & BM_LCDIF_CSC_COEFF1_RSRVD1)
#define BP_LCDIF_CSC_COEFF1_C2	16
#define BM_LCDIF_CSC_COEFF1_C2	0x03FF0000
#define BF_LCDIF_CSC_COEFF1_C2(v)  \
		(((v) << 16) & BM_LCDIF_CSC_COEFF1_C2)
#define BP_LCDIF_CSC_COEFF1_RSRVD0	10
#define BM_LCDIF_CSC_COEFF1_RSRVD0	0x0000FC00
#define BF_LCDIF_CSC_COEFF1_RSRVD0(v)  \
		(((v) << 10) & BM_LCDIF_CSC_COEFF1_RSRVD0)
#define BP_LCDIF_CSC_COEFF1_C1	0
#define BM_LCDIF_CSC_COEFF1_C1	0x000003FF
#define BF_LCDIF_CSC_COEFF1_C1(v)  \
		(((v) << 0) & BM_LCDIF_CSC_COEFF1_C1)

#define HW_LCDIF_CSC_COEFF2	(0x00000130)
#define HW_LCDIF_CSC_COEFF2_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF2)

#define BP_LCDIF_CSC_COEFF2_RSRVD1	26
#define BM_LCDIF_CSC_COEFF2_RSRVD1	0xFC000000
#define BF_LCDIF_CSC_COEFF2_RSRVD1(v) \
		(((v) << 26) & BM_LCDIF_CSC_COEFF2_RSRVD1)
#define BP_LCDIF_CSC_COEFF2_C4	16
#define BM_LCDIF_CSC_COEFF2_C4	0x03FF0000
#define BF_LCDIF_CSC_COEFF2_C4(v)  \
		(((v) << 16) & BM_LCDIF_CSC_COEFF2_C4)
#define BP_LCDIF_CSC_COEFF2_RSRVD0	10
#define BM_LCDIF_CSC_COEFF2_RSRVD0	0x0000FC00
#define BF_LCDIF_CSC_COEFF2_RSRVD0(v)  \
		(((v) << 10) & BM_LCDIF_CSC_COEFF2_RSRVD0)
#define BP_LCDIF_CSC_COEFF2_C3	0
#define BM_LCDIF_CSC_COEFF2_C3	0x000003FF
#define BF_LCDIF_CSC_COEFF2_C3(v)  \
		(((v) << 0) & BM_LCDIF_CSC_COEFF2_C3)

#define HW_LCDIF_CSC_COEFF3	(0x00000140)
#define HW_LCDIF_CSC_COEFF3_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF3)

#define BP_LCDIF_CSC_COEFF3_RSRVD1	26
#define BM_LCDIF_CSC_COEFF3_RSRVD1	0xFC000000
#define BF_LCDIF_CSC_COEFF3_RSRVD1(v) \
		(((v) << 26) & BM_LCDIF_CSC_COEFF3_RSRVD1)
#define BP_LCDIF_CSC_COEFF3_C6	16
#define BM_LCDIF_CSC_COEFF3_C6	0x03FF0000
#define BF_LCDIF_CSC_COEFF3_C6(v)  \
		(((v) << 16) & BM_LCDIF_CSC_COEFF3_C6)
#define BP_LCDIF_CSC_COEFF3_RSRVD0	10
#define BM_LCDIF_CSC_COEFF3_RSRVD0	0x0000FC00
#define BF_LCDIF_CSC_COEFF3_RSRVD0(v)  \
		(((v) << 10) & BM_LCDIF_CSC_COEFF3_RSRVD0)
#define BP_LCDIF_CSC_COEFF3_C5	0
#define BM_LCDIF_CSC_COEFF3_C5	0x000003FF
#define BF_LCDIF_CSC_COEFF3_C5(v)  \
		(((v) << 0) & BM_LCDIF_CSC_COEFF3_C5)

#define HW_LCDIF_CSC_COEFF4	(0x00000150)
#define HW_LCDIF_CSC_COEFF4_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF4)

#define BP_LCDIF_CSC_COEFF4_RSRVD1	26
#define BM_LCDIF_CSC_COEFF4_RSRVD1	0xFC000000
#define BF_LCDIF_CSC_COEFF4_RSRVD1(v) \
		(((v) << 26) & BM_LCDIF_CSC_COEFF4_RSRVD1)
#define BP_LCDIF_CSC_COEFF4_C8	16
#define BM_LCDIF_CSC_COEFF4_C8	0x03FF0000
#define BF_LCDIF_CSC_COEFF4_C8(v)  \
		(((v) << 16) & BM_LCDIF_CSC_COEFF4_C8)
#define BP_LCDIF_CSC_COEFF4_RSRVD0	10
#define BM_LCDIF_CSC_COEFF4_RSRVD0	0x0000FC00
#define BF_LCDIF_CSC_COEFF4_RSRVD0(v)  \
		(((v) << 10) & BM_LCDIF_CSC_COEFF4_RSRVD0)
#define BP_LCDIF_CSC_COEFF4_C7	0
#define BM_LCDIF_CSC_COEFF4_C7	0x000003FF
#define BF_LCDIF_CSC_COEFF4_C7(v)  \
		(((v) << 0) & BM_LCDIF_CSC_COEFF4_C7)

#define HW_LCDIF_CSC_OFFSET	(0x00000160)
#define HW_LCDIF_CSC_OFFSET_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CSC_OFFSET)

#define BP_LCDIF_CSC_OFFSET_RSRVD1	25
#define BM_LCDIF_CSC_OFFSET_RSRVD1	0xFE000000
#define BF_LCDIF_CSC_OFFSET_RSRVD1(v) \
		(((v) << 25) & BM_LCDIF_CSC_OFFSET_RSRVD1)
#define BP_LCDIF_CSC_OFFSET_CBCR_OFFSET	16
#define BM_LCDIF_CSC_OFFSET_CBCR_OFFSET	0x01FF0000
#define BF_LCDIF_CSC_OFFSET_CBCR_OFFSET(v)  \
		(((v) << 16) & BM_LCDIF_CSC_OFFSET_CBCR_OFFSET)
#define BP_LCDIF_CSC_OFFSET_RSRVD0	9
#define BM_LCDIF_CSC_OFFSET_RSRVD0	0x0000FE00
#define BF_LCDIF_CSC_OFFSET_RSRVD0(v)  \
		(((v) << 9) & BM_LCDIF_CSC_OFFSET_RSRVD0)
#define BP_LCDIF_CSC_OFFSET_Y_OFFSET	0
#define BM_LCDIF_CSC_OFFSET_Y_OFFSET	0x000001FF
#define BF_LCDIF_CSC_OFFSET_Y_OFFSET(v)  \
		(((v) << 0) & BM_LCDIF_CSC_OFFSET_Y_OFFSET)

#define HW_LCDIF_CSC_LIMIT	(0x00000170)
#define HW_LCDIF_CSC_LIMIT_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_CSC_LIMIT)

#define BP_LCDIF_CSC_LIMIT_CBCR_MIN	24
#define BM_LCDIF_CSC_LIMIT_CBCR_MIN	0xFF000000
#define BF_LCDIF_CSC_LIMIT_CBCR_MIN(v) \
		(((v) << 24) & BM_LCDIF_CSC_LIMIT_CBCR_MIN)
#define BP_LCDIF_CSC_LIMIT_CBCR_MAX	16
#define BM_LCDIF_CSC_LIMIT_CBCR_MAX	0x00FF0000
#define BF_LCDIF_CSC_LIMIT_CBCR_MAX(v)  \
		(((v) << 16) & BM_LCDIF_CSC_LIMIT_CBCR_MAX)
#define BP_LCDIF_CSC_LIMIT_Y_MIN	8
#define BM_LCDIF_CSC_LIMIT_Y_MIN	0x0000FF00
#define BF_LCDIF_CSC_LIMIT_Y_MIN(v)  \
		(((v) << 8) & BM_LCDIF_CSC_LIMIT_Y_MIN)
#define BP_LCDIF_CSC_LIMIT_Y_MAX	0
#define BM_LCDIF_CSC_LIMIT_Y_MAX	0x000000FF
#define BF_LCDIF_CSC_LIMIT_Y_MAX(v)  \
		(((v) << 0) & BM_LCDIF_CSC_LIMIT_Y_MAX)

#define HW_LCDIF_PIN_SHARING_CTRL0	(0x00000180)
#define HW_LCDIF_PIN_SHARING_CTRL0_SET	(0x00000184)
#define HW_LCDIF_PIN_SHARING_CTRL0_CLR	(0x00000188)
#define HW_LCDIF_PIN_SHARING_CTRL0_TOG	(0x0000018c)
#define HW_LCDIF_PIN_SHARING_CTRL0_ADDR  \
		(REGS_LCDIF_BASE + HW_LCDIF_PIN_SHARING_CTRL0)
#define HW_LCDIF_PIN_SHARING_CTRL0_SET_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_PIN_SHARING_CTRL0_SET)
#define HW_LCDIF_PIN_SHARING_CTRL0_CLR_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_PIN_SHARING_CTRL0_CLR)
#define HW_LCDIF_PIN_SHARING_CTRL0_TOG_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_PIN_SHARING_CTRL0_TOG)

#define BP_LCDIF_PIN_SHARING_CTRL0_RSRVD1	6
#define BM_LCDIF_PIN_SHARING_CTRL0_RSRVD1	0xFFFFFFC0
#define BF_LCDIF_PIN_SHARING_CTRL0_RSRVD1(v) \
		(((v) << 6) & BM_LCDIF_PIN_SHARING_CTRL0_RSRVD1)
#define BP_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE	4
#define BM_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE	0x00000030
#define BF_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE(v)  \
		(((v) << 4) & BM_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE)
#define BV_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE__NO_OVERRIDE 0x0
#define BV_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE__RSRVD       0x1
#define BV_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE__LCDIF_SEL   0x2
#define BV_LCDIF_PIN_SHARING_CTRL0_MUX_OVERRIDE__GPMI_SEL    0x3
#define BM_LCDIF_PIN_SHARING_CTRL0_RSRVD0	0x00000008
#define BM_LCDIF_PIN_SHARING_CTRL0_PIN_SHARING_IRQ_EN	0x00000004
#define BM_LCDIF_PIN_SHARING_CTRL0_PIN_SHARING_IRQ	0x00000002
#define BV_LCDIF_PIN_SHARING_CTRL0_PIN_SHARING_IRQ__NO_REQUEST 0x0
#define BV_LCDIF_PIN_SHARING_CTRL0_PIN_SHARING_IRQ__REQUEST    0x1
#define BM_LCDIF_PIN_SHARING_CTRL0_PIN_SHARING_ENABLE	0x00000001

#define HW_LCDIF_PIN_SHARING_CTRL1	(0x00000190)
#define HW_LCDIF_PIN_SHARING_CTRL1_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_PIN_SHARING_CTRL1)

#define BP_LCDIF_PIN_SHARING_CTRL1_THRESHOLD1	0
#define BM_LCDIF_PIN_SHARING_CTRL1_THRESHOLD1	0xFFFFFFFF
#define BF_LCDIF_PIN_SHARING_CTRL1_THRESHOLD1(v)	(v)

#define HW_LCDIF_PIN_SHARING_CTRL2	(0x000001a0)
#define HW_LCDIF_PIN_SHARING_CTRL2_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_PIN_SHARING_CTRL2)

#define BP_LCDIF_PIN_SHARING_CTRL2_THRESHOLD2	0
#define BM_LCDIF_PIN_SHARING_CTRL2_THRESHOLD2	0xFFFFFFFF
#define BF_LCDIF_PIN_SHARING_CTRL2_THRESHOLD2(v)	(v)

#define HW_LCDIF_DATA	(0x000001b0)
#define HW_LCDIF_DATA_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DATA)

#define BP_LCDIF_DATA_DATA_THREE	24
#define BM_LCDIF_DATA_DATA_THREE	0xFF000000
#define BF_LCDIF_DATA_DATA_THREE(v) \
		(((v) << 24) & BM_LCDIF_DATA_DATA_THREE)
#define BP_LCDIF_DATA_DATA_TWO	16
#define BM_LCDIF_DATA_DATA_TWO	0x00FF0000
#define BF_LCDIF_DATA_DATA_TWO(v)  \
		(((v) << 16) & BM_LCDIF_DATA_DATA_TWO)
#define BP_LCDIF_DATA_DATA_ONE	8
#define BM_LCDIF_DATA_DATA_ONE	0x0000FF00
#define BF_LCDIF_DATA_DATA_ONE(v)  \
		(((v) << 8) & BM_LCDIF_DATA_DATA_ONE)
#define BP_LCDIF_DATA_DATA_ZERO	0
#define BM_LCDIF_DATA_DATA_ZERO	0x000000FF
#define BF_LCDIF_DATA_DATA_ZERO(v)  \
		(((v) << 0) & BM_LCDIF_DATA_DATA_ZERO)

#define HW_LCDIF_BM_ERROR_STAT	(0x000001c0)
#define HW_LCDIF_BM_ERROR_STAT_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_BM_ERROR_STAT)

#define BP_LCDIF_BM_ERROR_STAT_ADDR	0
#define BM_LCDIF_BM_ERROR_STAT_ADDR	0xFFFFFFFF
#define BF_LCDIF_BM_ERROR_STAT_ADDR(v)	(v)

#define HW_LCDIF_STAT	(0x000001d0)
#define HW_LCDIF_STAT_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_STAT)

#define BM_LCDIF_STAT_PRESENT	0x80000000
#define BM_LCDIF_STAT_DMA_REQ	0x40000000
#define BM_LCDIF_STAT_LFIFO_FULL	0x20000000
#define BM_LCDIF_STAT_LFIFO_EMPTY	0x10000000
#define BM_LCDIF_STAT_TXFIFO_FULL	0x08000000
#define BM_LCDIF_STAT_TXFIFO_EMPTY	0x04000000
#define BM_LCDIF_STAT_BUSY	0x02000000
#define BM_LCDIF_STAT_DVI_CURRENT_FIELD	0x01000000
#define BP_LCDIF_STAT_RSRVD0	0
#define BM_LCDIF_STAT_RSRVD0	0x00FFFFFF
#define BF_LCDIF_STAT_RSRVD0(v)  \
		(((v) << 0) & BM_LCDIF_STAT_RSRVD0)

#define HW_LCDIF_VERSION	(0x000001e0)
#define HW_LCDIF_VERSION_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_VERSION)

#define BP_LCDIF_VERSION_MAJOR	24
#define BM_LCDIF_VERSION_MAJOR	0xFF000000
#define BF_LCDIF_VERSION_MAJOR(v) \
		(((v) << 24) & BM_LCDIF_VERSION_MAJOR)
#define BP_LCDIF_VERSION_MINOR	16
#define BM_LCDIF_VERSION_MINOR	0x00FF0000
#define BF_LCDIF_VERSION_MINOR(v)  \
		(((v) << 16) & BM_LCDIF_VERSION_MINOR)
#define BP_LCDIF_VERSION_STEP	0
#define BM_LCDIF_VERSION_STEP	0x0000FFFF
#define BF_LCDIF_VERSION_STEP(v)  \
		(((v) << 0) & BM_LCDIF_VERSION_STEP)

#define HW_LCDIF_DEBUG0	(0x000001f0)
#define HW_LCDIF_DEBUG0_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DEBUG0)

#define BM_LCDIF_DEBUG0_STREAMING_END_DETECTED	0x80000000
#define BM_LCDIF_DEBUG0_WAIT_FOR_VSYNC_EDGE_OUT	0x40000000
#define BM_LCDIF_DEBUG0_SYNC_SIGNALS_ON_REG	0x20000000
#define BM_LCDIF_DEBUG0_DMACMDKICK	0x10000000
#define BM_LCDIF_DEBUG0_ENABLE	0x08000000
#define BM_LCDIF_DEBUG0_HSYNC	0x04000000
#define BM_LCDIF_DEBUG0_VSYNC	0x02000000
#define BM_LCDIF_DEBUG0_CUR_FRAME_TX	0x01000000
#define BM_LCDIF_DEBUG0_EMPTY_WORD	0x00800000
#define BP_LCDIF_DEBUG0_CUR_STATE	16
#define BM_LCDIF_DEBUG0_CUR_STATE	0x007F0000
#define BF_LCDIF_DEBUG0_CUR_STATE(v)  \
		(((v) << 16) & BM_LCDIF_DEBUG0_CUR_STATE)
#define BM_LCDIF_DEBUG0_PXP_LCDIF_B0_READY	0x00008000
#define BM_LCDIF_DEBUG0_LCDIF_PXP_B0_DONE	0x00004000
#define BM_LCDIF_DEBUG0_PXP_LCDIF_B1_READY	0x00002000
#define BM_LCDIF_DEBUG0_LCDIF_PXP_B1_DONE	0x00001000
#define BM_LCDIF_DEBUG0_GPMI_LCDIF_REQ	0x00000800
#define BM_LCDIF_DEBUG0_LCDIF_GPMI_GRANT	0x00000400
#define BP_LCDIF_DEBUG0_RSRVD0	0
#define BM_LCDIF_DEBUG0_RSRVD0	0x000003FF
#define BF_LCDIF_DEBUG0_RSRVD0(v)  \
		(((v) << 0) & BM_LCDIF_DEBUG0_RSRVD0)

#define HW_LCDIF_DEBUG1	(0x00000200)
#define HW_LCDIF_DEBUG1_ADDR \
		(REGS_LCDIF_BASE + HW_LCDIF_DEBUG1)

#define BP_LCDIF_DEBUG1_H_DATA_COUNT	16
#define BM_LCDIF_DEBUG1_H_DATA_COUNT	0xFFFF0000
#define BF_LCDIF_DEBUG1_H_DATA_COUNT(v) \
		(((v) << 16) & BM_LCDIF_DEBUG1_H_DATA_COUNT)
#define BP_LCDIF_DEBUG1_V_DATA_COUNT	0
#define BM_LCDIF_DEBUG1_V_DATA_COUNT	0x0000FFFF
#define BF_LCDIF_DEBUG1_V_DATA_COUNT(v)  \
		(((v) << 0) & BM_LCDIF_DEBUG1_V_DATA_COUNT)
#endif /* __ARCH_ARM___LCDIF_H */
