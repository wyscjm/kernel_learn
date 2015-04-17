/*
 * STMP APBH Register Definitions
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

#ifndef __ARCH_ARM___APBH_H
#define __ARCH_ARM___APBH_H  1

#define REGS_APBH_BASE (STMP3XXX_REGS_BASE + 0x4000)
#define REGS_APBH_PHYS (0x80004000)
#define REGS_APBH_SIZE 0x00002000

#define HW_APBH_CTRL0	(0x00000000)
#define HW_APBH_CTRL0_SET	(0x00000004)
#define HW_APBH_CTRL0_CLR	(0x00000008)
#define HW_APBH_CTRL0_TOG	(0x0000000c)
#define HW_APBH_CTRL0_ADDR  \
		(REGS_APBH_BASE + HW_APBH_CTRL0)
#define HW_APBH_CTRL0_SET_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL0_SET)
#define HW_APBH_CTRL0_CLR_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL0_CLR)
#define HW_APBH_CTRL0_TOG_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL0_TOG)

#define BM_APBH_CTRL0_SFTRST	0x80000000
#define BM_APBH_CTRL0_CLKGATE	0x40000000
#define BM_APBH_CTRL0_AHB_BURST8_EN	0x20000000
#define BM_APBH_CTRL0_APB_BURST4_EN	0x10000000
#define BP_APBH_CTRL0_RSVD0	24
#define BM_APBH_CTRL0_RSVD0	0x0F000000
#define BF_APBH_CTRL0_RSVD0(v)  \
		(((v) << 24) & BM_APBH_CTRL0_RSVD0)
#define BP_APBH_CTRL0_RESET_CHANNEL	16
#define BM_APBH_CTRL0_RESET_CHANNEL	0x00FF0000
#define BF_APBH_CTRL0_RESET_CHANNEL(v)  \
		(((v) << 16) & BM_APBH_CTRL0_RESET_CHANNEL)
#define BV_APBH_CTRL0_RESET_CHANNEL__SSP1  0x02
#define BV_APBH_CTRL0_RESET_CHANNEL__SSP2  0x04
#define BV_APBH_CTRL0_RESET_CHANNEL__ATA   0x10
#define BV_APBH_CTRL0_RESET_CHANNEL__NAND0 0x10
#define BV_APBH_CTRL0_RESET_CHANNEL__NAND1 0x20
#define BV_APBH_CTRL0_RESET_CHANNEL__NAND2 0x40
#define BV_APBH_CTRL0_RESET_CHANNEL__NAND3 0x80
#define BP_APBH_CTRL0_CLKGATE_CHANNEL	8
#define BM_APBH_CTRL0_CLKGATE_CHANNEL	0x0000FF00
#define BF_APBH_CTRL0_CLKGATE_CHANNEL(v)  \
		(((v) << 8) & BM_APBH_CTRL0_CLKGATE_CHANNEL)
#define BV_APBH_CTRL0_CLKGATE_CHANNEL__SSP1  0x02
#define BV_APBH_CTRL0_CLKGATE_CHANNEL__SSP2  0x04
#define BV_APBH_CTRL0_CLKGATE_CHANNEL__ATA   0x10
#define BV_APBH_CTRL0_CLKGATE_CHANNEL__NAND0 0x10
#define BV_APBH_CTRL0_CLKGATE_CHANNEL__NAND1 0x20
#define BV_APBH_CTRL0_CLKGATE_CHANNEL__NAND2 0x40
#define BV_APBH_CTRL0_CLKGATE_CHANNEL__NAND3 0x80
#define BP_APBH_CTRL0_FREEZE_CHANNEL	0
#define BM_APBH_CTRL0_FREEZE_CHANNEL	0x000000FF
#define BF_APBH_CTRL0_FREEZE_CHANNEL(v)  \
		(((v) << 0) & BM_APBH_CTRL0_FREEZE_CHANNEL)
#define BV_APBH_CTRL0_FREEZE_CHANNEL__SSP1  0x02
#define BV_APBH_CTRL0_FREEZE_CHANNEL__SSP2  0x04
#define BV_APBH_CTRL0_FREEZE_CHANNEL__ATA   0x10
#define BV_APBH_CTRL0_FREEZE_CHANNEL__NAND0 0x10
#define BV_APBH_CTRL0_FREEZE_CHANNEL__NAND1 0x20
#define BV_APBH_CTRL0_FREEZE_CHANNEL__NAND2 0x40
#define BV_APBH_CTRL0_FREEZE_CHANNEL__NAND3 0x80

#define HW_APBH_CTRL1	(0x00000010)
#define HW_APBH_CTRL1_SET	(0x00000014)
#define HW_APBH_CTRL1_CLR	(0x00000018)
#define HW_APBH_CTRL1_TOG	(0x0000001c)
#define HW_APBH_CTRL1_ADDR  \
		(REGS_APBH_BASE + HW_APBH_CTRL1)
#define HW_APBH_CTRL1_SET_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL1_SET)
#define HW_APBH_CTRL1_CLR_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL1_CLR)
#define HW_APBH_CTRL1_TOG_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL1_TOG)

#define BP_APBH_CTRL1_RSVD1	24
#define BM_APBH_CTRL1_RSVD1	0xFF000000
#define BF_APBH_CTRL1_RSVD1(v) \
		(((v) << 24) & BM_APBH_CTRL1_RSVD1)
#define BM_APBH_CTRL1_CH7_CMDCMPLT_IRQ_EN	0x00800000
#define BM_APBH_CTRL1_CH6_CMDCMPLT_IRQ_EN	0x00400000
#define BM_APBH_CTRL1_CH5_CMDCMPLT_IRQ_EN	0x00200000
#define BM_APBH_CTRL1_CH4_CMDCMPLT_IRQ_EN	0x00100000
#define BM_APBH_CTRL1_CH3_CMDCMPLT_IRQ_EN	0x00080000
#define BM_APBH_CTRL1_CH2_CMDCMPLT_IRQ_EN	0x00040000
#define BM_APBH_CTRL1_CH1_CMDCMPLT_IRQ_EN	0x00020000
#define BM_APBH_CTRL1_CH0_CMDCMPLT_IRQ_EN	0x00010000
#define BP_APBH_CTRL1_RSVD0	8
#define BM_APBH_CTRL1_RSVD0	0x0000FF00
#define BF_APBH_CTRL1_RSVD0(v)  \
		(((v) << 8) & BM_APBH_CTRL1_RSVD0)
#define BM_APBH_CTRL1_CH7_CMDCMPLT_IRQ	0x00000080
#define BM_APBH_CTRL1_CH6_CMDCMPLT_IRQ	0x00000040
#define BM_APBH_CTRL1_CH5_CMDCMPLT_IRQ	0x00000020
#define BM_APBH_CTRL1_CH4_CMDCMPLT_IRQ	0x00000010
#define BM_APBH_CTRL1_CH3_CMDCMPLT_IRQ	0x00000008
#define BM_APBH_CTRL1_CH2_CMDCMPLT_IRQ	0x00000004
#define BM_APBH_CTRL1_CH1_CMDCMPLT_IRQ	0x00000002
#define BM_APBH_CTRL1_CH0_CMDCMPLT_IRQ	0x00000001

#define HW_APBH_CTRL2	(0x00000020)
#define HW_APBH_CTRL2_SET	(0x00000024)
#define HW_APBH_CTRL2_CLR	(0x00000028)
#define HW_APBH_CTRL2_TOG	(0x0000002c)
#define HW_APBH_CTRL2_ADDR  \
		(REGS_APBH_BASE + HW_APBH_CTRL2)
#define HW_APBH_CTRL2_SET_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL2_SET)
#define HW_APBH_CTRL2_CLR_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL2_CLR)
#define HW_APBH_CTRL2_TOG_ADDR \
		(REGS_APBH_BASE + HW_APBH_CTRL2_TOG)

#define BP_APBH_CTRL2_RSVD1	24
#define BM_APBH_CTRL2_RSVD1	0xFF000000
#define BF_APBH_CTRL2_RSVD1(v) \
		(((v) << 24) & BM_APBH_CTRL2_RSVD1)
#define BM_APBH_CTRL2_CH7_ERROR_STATUS	0x00800000
#define BV_APBH_CTRL2_CH7_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH7_ERROR_STATUS__BUS_ERROR   0x1
#define BM_APBH_CTRL2_CH6_ERROR_STATUS	0x00400000
#define BV_APBH_CTRL2_CH6_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH6_ERROR_STATUS__BUS_ERROR   0x1
#define BM_APBH_CTRL2_CH5_ERROR_STATUS	0x00200000
#define BV_APBH_CTRL2_CH5_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH5_ERROR_STATUS__BUS_ERROR   0x1
#define BM_APBH_CTRL2_CH4_ERROR_STATUS	0x00100000
#define BV_APBH_CTRL2_CH4_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH4_ERROR_STATUS__BUS_ERROR   0x1
#define BM_APBH_CTRL2_CH3_ERROR_STATUS	0x00080000
#define BV_APBH_CTRL2_CH3_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH3_ERROR_STATUS__BUS_ERROR   0x1
#define BM_APBH_CTRL2_CH2_ERROR_STATUS	0x00040000
#define BV_APBH_CTRL2_CH2_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH2_ERROR_STATUS__BUS_ERROR   0x1
#define BM_APBH_CTRL2_CH1_ERROR_STATUS	0x00020000
#define BV_APBH_CTRL2_CH1_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH1_ERROR_STATUS__BUS_ERROR   0x1
#define BM_APBH_CTRL2_CH0_ERROR_STATUS	0x00010000
#define BV_APBH_CTRL2_CH0_ERROR_STATUS__TERMINATION 0x0
#define BV_APBH_CTRL2_CH0_ERROR_STATUS__BUS_ERROR   0x1
#define BP_APBH_CTRL2_RSVD0	8
#define BM_APBH_CTRL2_RSVD0	0x0000FF00
#define BF_APBH_CTRL2_RSVD0(v)  \
		(((v) << 8) & BM_APBH_CTRL2_RSVD0)
#define BM_APBH_CTRL2_CH7_ERROR_IRQ	0x00000080
#define BM_APBH_CTRL2_CH6_ERROR_IRQ	0x00000040
#define BM_APBH_CTRL2_CH5_ERROR_IRQ	0x00000020
#define BM_APBH_CTRL2_CH4_ERROR_IRQ	0x00000010
#define BM_APBH_CTRL2_CH3_ERROR_IRQ	0x00000008
#define BM_APBH_CTRL2_CH2_ERROR_IRQ	0x00000004
#define BM_APBH_CTRL2_CH1_ERROR_IRQ	0x00000002
#define BM_APBH_CTRL2_CH0_ERROR_IRQ	0x00000001

#define HW_APBH_DEVSEL	(0x00000030)
#define HW_APBH_DEVSEL_ADDR \
		(REGS_APBH_BASE + HW_APBH_DEVSEL)

#define BP_APBH_DEVSEL_CH7	28
#define BM_APBH_DEVSEL_CH7	0xF0000000
#define BF_APBH_DEVSEL_CH7(v) \
		(((v) << 28) & BM_APBH_DEVSEL_CH7)
#define BP_APBH_DEVSEL_CH6	24
#define BM_APBH_DEVSEL_CH6	0x0F000000
#define BF_APBH_DEVSEL_CH6(v)  \
		(((v) << 24) & BM_APBH_DEVSEL_CH6)
#define BP_APBH_DEVSEL_CH5	20
#define BM_APBH_DEVSEL_CH5	0x00F00000
#define BF_APBH_DEVSEL_CH5(v)  \
		(((v) << 20) & BM_APBH_DEVSEL_CH5)
#define BP_APBH_DEVSEL_CH4	16
#define BM_APBH_DEVSEL_CH4	0x000F0000
#define BF_APBH_DEVSEL_CH4(v)  \
		(((v) << 16) & BM_APBH_DEVSEL_CH4)
#define BP_APBH_DEVSEL_CH3	12
#define BM_APBH_DEVSEL_CH3	0x0000F000
#define BF_APBH_DEVSEL_CH3(v)  \
		(((v) << 12) & BM_APBH_DEVSEL_CH3)
#define BP_APBH_DEVSEL_CH2	8
#define BM_APBH_DEVSEL_CH2	0x00000F00
#define BF_APBH_DEVSEL_CH2(v)  \
		(((v) << 8) & BM_APBH_DEVSEL_CH2)
#define BP_APBH_DEVSEL_CH1	4
#define BM_APBH_DEVSEL_CH1	0x000000F0
#define BF_APBH_DEVSEL_CH1(v)  \
		(((v) << 4) & BM_APBH_DEVSEL_CH1)
#define BP_APBH_DEVSEL_CH0	0
#define BM_APBH_DEVSEL_CH0	0x0000000F
#define BF_APBH_DEVSEL_CH0(v)  \
		(((v) << 0) & BM_APBH_DEVSEL_CH0)

/*
 *  multi-register-define name HW_APBH_CHn_CURCMDAR
 *              base 0x00000040
 *              count 8
 *              offset 0x70
 */
#define HW_APBH_CHn_CURCMDAR(n)	(0x00000040 + (n) * 0x70)
#define HW_APBH_CHn_CURCMDAR_ADDR(n) \
		(REGS_APBH_BASE + HW_APBH_CHn_CURCMDAR(n))
#define BP_APBH_CHn_CURCMDAR_CMD_ADDR	0
#define BM_APBH_CHn_CURCMDAR_CMD_ADDR	0xFFFFFFFF
#define BF_APBH_CHn_CURCMDAR_CMD_ADDR(v)	(v)

/*
 *  multi-register-define name HW_APBH_CHn_NXTCMDAR
 *              base 0x00000050
 *              count 8
 *              offset 0x70
 */
#define HW_APBH_CHn_NXTCMDAR(n)	(0x00000050 + (n) * 0x70)
#define HW_APBH_CHn_NXTCMDAR_ADDR(n) \
		(REGS_APBH_BASE + HW_APBH_CHn_NXTCMDAR(n))
#define BP_APBH_CHn_NXTCMDAR_CMD_ADDR	0
#define BM_APBH_CHn_NXTCMDAR_CMD_ADDR	0xFFFFFFFF
#define BF_APBH_CHn_NXTCMDAR_CMD_ADDR(v)	(v)

/*
 *  multi-register-define name HW_APBH_CHn_CMD
 *              base 0x00000060
 *              count 8
 *              offset 0x70
 */
#define HW_APBH_CHn_CMD(n)	(0x00000060 + (n) * 0x70)
#define HW_APBH_CHn_CMD_ADDR(n) \
		(REGS_APBH_BASE + HW_APBH_CHn_CMD(n))
#define BP_APBH_CHn_CMD_XFER_COUNT	16
#define BM_APBH_CHn_CMD_XFER_COUNT	0xFFFF0000
#define BF_APBH_CHn_CMD_XFER_COUNT(v) \
		(((v) << 16) & BM_APBH_CHn_CMD_XFER_COUNT)
#define BP_APBH_CHn_CMD_CMDWORDS	12
#define BM_APBH_CHn_CMD_CMDWORDS	0x0000F000
#define BF_APBH_CHn_CMD_CMDWORDS(v)  \
		(((v) << 12) & BM_APBH_CHn_CMD_CMDWORDS)
#define BP_APBH_CHn_CMD_RSVD1	9
#define BM_APBH_CHn_CMD_RSVD1	0x00000E00
#define BF_APBH_CHn_CMD_RSVD1(v)  \
		(((v) << 9) & BM_APBH_CHn_CMD_RSVD1)
#define BM_APBH_CHn_CMD_HALTONTERMINATE	0x00000100
#define BM_APBH_CHn_CMD_WAIT4ENDCMD	0x00000080
#define BM_APBH_CHn_CMD_SEMAPHORE	0x00000040
#define BM_APBH_CHn_CMD_NANDWAIT4READY	0x00000020
#define BM_APBH_CHn_CMD_NANDLOCK	0x00000010
#define BM_APBH_CHn_CMD_IRQONCMPLT	0x00000008
#define BM_APBH_CHn_CMD_CHAIN	0x00000004
#define BP_APBH_CHn_CMD_COMMAND	0
#define BM_APBH_CHn_CMD_COMMAND	0x00000003
#define BF_APBH_CHn_CMD_COMMAND(v)  \
		(((v) << 0) & BM_APBH_CHn_CMD_COMMAND)
#define BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER 0x0
#define BV_APBH_CHn_CMD_COMMAND__DMA_WRITE   0x1
#define BV_APBH_CHn_CMD_COMMAND__DMA_READ    0x2
#define BV_APBH_CHn_CMD_COMMAND__DMA_SENSE   0x3

/*
 *  multi-register-define name HW_APBH_CHn_BAR
 *              base 0x00000070
 *              count 8
 *              offset 0x70
 */
#define HW_APBH_CHn_BAR(n)	(0x00000070 + (n) * 0x70)
#define HW_APBH_CHn_BAR_ADDR(n) \
		(REGS_APBH_BASE + HW_APBH_CHn_BAR(n))
#define BP_APBH_CHn_BAR_ADDRESS	0
#define BM_APBH_CHn_BAR_ADDRESS	0xFFFFFFFF
#define BF_APBH_CHn_BAR_ADDRESS(v)	(v)

/*
 *  multi-register-define name HW_APBH_CHn_SEMA
 *              base 0x00000080
 *              count 8
 *              offset 0x70
 */
#define HW_APBH_CHn_SEMA(n)	(0x00000080 + (n) * 0x70)
#define HW_APBH_CHn_SEMA_ADDR(n) \
		(REGS_APBH_BASE + HW_APBH_CHn_SEMA(n))
#define BP_APBH_CHn_SEMA_RSVD2	24
#define BM_APBH_CHn_SEMA_RSVD2	0xFF000000
#define BF_APBH_CHn_SEMA_RSVD2(v) \
		(((v) << 24) & BM_APBH_CHn_SEMA_RSVD2)
#define BP_APBH_CHn_SEMA_PHORE	16
#define BM_APBH_CHn_SEMA_PHORE	0x00FF0000
#define BF_APBH_CHn_SEMA_PHORE(v)  \
		(((v) << 16) & BM_APBH_CHn_SEMA_PHORE)
#define BP_APBH_CHn_SEMA_RSVD1	8
#define BM_APBH_CHn_SEMA_RSVD1	0x0000FF00
#define BF_APBH_CHn_SEMA_RSVD1(v)  \
		(((v) << 8) & BM_APBH_CHn_SEMA_RSVD1)
#define BP_APBH_CHn_SEMA_INCREMENT_SEMA	0
#define BM_APBH_CHn_SEMA_INCREMENT_SEMA	0x000000FF
#define BF_APBH_CHn_SEMA_INCREMENT_SEMA(v)  \
		(((v) << 0) & BM_APBH_CHn_SEMA_INCREMENT_SEMA)

/*
 *  multi-register-define name HW_APBH_CHn_DEBUG1
 *              base 0x00000090
 *              count 8
 *              offset 0x70
 */
#define HW_APBH_CHn_DEBUG1(n)	(0x00000090 + (n) * 0x70)
#define HW_APBH_CHn_DEBUG1_ADDR(n) \
		(REGS_APBH_BASE + HW_APBH_CHn_DEBUG1(n))
#define BM_APBH_CHn_DEBUG1_REQ	0x80000000
#define BM_APBH_CHn_DEBUG1_BURST	0x40000000
#define BM_APBH_CHn_DEBUG1_KICK	0x20000000
#define BM_APBH_CHn_DEBUG1_END	0x10000000
#define BM_APBH_CHn_DEBUG1_SENSE	0x08000000
#define BM_APBH_CHn_DEBUG1_READY	0x04000000
#define BM_APBH_CHn_DEBUG1_LOCK	0x02000000
#define BM_APBH_CHn_DEBUG1_NEXTCMDADDRVALID	0x01000000
#define BM_APBH_CHn_DEBUG1_RD_FIFO_EMPTY	0x00800000
#define BM_APBH_CHn_DEBUG1_RD_FIFO_FULL	0x00400000
#define BM_APBH_CHn_DEBUG1_WR_FIFO_EMPTY	0x00200000
#define BM_APBH_CHn_DEBUG1_WR_FIFO_FULL	0x00100000
#define BP_APBH_CHn_DEBUG1_RSVD1	5
#define BM_APBH_CHn_DEBUG1_RSVD1	0x000FFFE0
#define BF_APBH_CHn_DEBUG1_RSVD1(v)  \
		(((v) << 5) & BM_APBH_CHn_DEBUG1_RSVD1)
#define BP_APBH_CHn_DEBUG1_STATEMACHINE	0
#define BM_APBH_CHn_DEBUG1_STATEMACHINE	0x0000001F
#define BF_APBH_CHn_DEBUG1_STATEMACHINE(v)  \
		(((v) << 0) & BM_APBH_CHn_DEBUG1_STATEMACHINE)
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__IDLE            0x00
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__REQ_CMD1        0x01
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__REQ_CMD3        0x02
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__REQ_CMD2        0x03
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__XFER_DECODE     0x04
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__REQ_WAIT        0x05
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__REQ_CMD4        0x06
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__PIO_REQ         0x07
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__READ_FLUSH      0x08
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__READ_WAIT       0x09
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__WRITE           0x0C
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__READ_REQ        0x0D
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__CHECK_CHAIN     0x0E
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__XFER_COMPLETE   0x0F
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__TERMINATE       0x14
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__WAIT_END        0x15
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__WRITE_WAIT      0x1C
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__HALT_AFTER_TERM 0x1D
#define BV_APBH_CHn_DEBUG1_STATEMACHINE__CHECK_WAIT      0x1E

/*
 *  multi-register-define name HW_APBH_CHn_DEBUG2
 *              base 0x000000A0
 *              count 8
 *              offset 0x70
 */
#define HW_APBH_CHn_DEBUG2(n)	(0x000000a0 + (n) * 0x70)
#define HW_APBH_CHn_DEBUG2_ADDR(n) \
		(REGS_APBH_BASE + HW_APBH_CHn_DEBUG2(n))
#define BP_APBH_CHn_DEBUG2_APB_BYTES	16
#define BM_APBH_CHn_DEBUG2_APB_BYTES	0xFFFF0000
#define BF_APBH_CHn_DEBUG2_APB_BYTES(v) \
		(((v) << 16) & BM_APBH_CHn_DEBUG2_APB_BYTES)
#define BP_APBH_CHn_DEBUG2_AHB_BYTES	0
#define BM_APBH_CHn_DEBUG2_AHB_BYTES	0x0000FFFF
#define BF_APBH_CHn_DEBUG2_AHB_BYTES(v)  \
		(((v) << 0) & BM_APBH_CHn_DEBUG2_AHB_BYTES)

#define HW_APBH_VERSION	(0x000003f0)
#define HW_APBH_VERSION_ADDR \
		(REGS_APBH_BASE + HW_APBH_VERSION)

#define BP_APBH_VERSION_MAJOR	24
#define BM_APBH_VERSION_MAJOR	0xFF000000
#define BF_APBH_VERSION_MAJOR(v) \
		(((v) << 24) & BM_APBH_VERSION_MAJOR)
#define BP_APBH_VERSION_MINOR	16
#define BM_APBH_VERSION_MINOR	0x00FF0000
#define BF_APBH_VERSION_MINOR(v)  \
		(((v) << 16) & BM_APBH_VERSION_MINOR)
#define BP_APBH_VERSION_STEP	0
#define BM_APBH_VERSION_STEP	0x0000FFFF
#define BF_APBH_VERSION_STEP(v)  \
		(((v) << 0) & BM_APBH_VERSION_STEP)
#endif /* __ARCH_ARM___APBH_H */
