
/*!
 * Define this option to specify we are using the newer SDMA module.
 */
#define MXC_SDMA_V2

/*
 * IRAM
 */
#define MX35_IRAM_BASE_ADDR		0x10000000	/* internal ram */
#define IRAM_BASE_ADDR			MX35_IRAM_BASE_ADDR
#define IRAM_BASE_ADDR_VIRT		0xFC600000
#define MX35_IRAM_SIZE		SZ_128K

#ifndef CONFIG_SDMA_IRAM
#define CONFIG_SDMA_IRAM_SIZE 0
#endif
#ifdef CONFIG_SND_MXC_SOC_IRAM
#define SND_RAM_SIZE 0x10000
#else
#define SND_RAM_SIZE 0
#endif

#define SND_RAM_BASE_ADDR	(MX35_IRAM_BASE_ADDR + CONFIG_SDMA_IRAM_SIZE)
#define MLB_IRAM_ADDR_OFFSET	(CONFIG_SDMA_IRAM_SIZE + SND_RAM_SIZE)

#define MXC_FEC_BASE_ADDR	0x50038000
#define MX35_NFC_BASE_ADDR	0xBB000000
#define NFC_BASE_ADDR		MX35_NFC_BASE_ADDR
#define NFC_BASE_ADDR_VIRT    	0xFC700000
#define NFC_SIZE		SZ_1M
#define NFC_IO_ADDRESS(x)	(((x) - NFC_BASE_ADDR) + NFC_BASE_ADDR_VIRT)


#define MMC_SDHC1_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B4000)
#define MMC_SDHC2_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B8000)
#define MMC_SDHC3_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000BC000)
#define CAN1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000E4000)
#define CAN2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000E8000)
#define IIM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000F0000)
#define OTG_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000F4000)
#define MLB_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000F8000)
#define SPDIF_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00028000)
#define ATA_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00020000)
#define ASRC_BASE_ADDR		(SPBA0_BASE_ADDR + 0x0002C000)
#define ESAI_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00034000)
#define RNGC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000B0000)

#define SPBA_MSHC	0x24
#define SPBA_SPDIR	0x28
#define SPBA_ASRC	0x2C
#define SPBA_ESAI	0x34
#define SPBA_FEC	0x38

/*
 * Interrupt numbers
 */
#define MXC_INT_OWIRE		2
#define MX35_INT_MMC_SDHC1	7
#define MXC_INT_MMC_SDHC2	8
#define MXC_INT_MMC_SDHC3	9
#define MX35_INT_SSI1		11
#define MX35_INT_SSI2		12
#define MXC_INT_SSI1		MX35_INT_SSI1
#define MXC_INT_SSI2		MX35_INT_SSI2
#define MXC_INT_GPU2D		16
#define MXC_INT_ASRC		17
#define MXC_INT_USBHS		35
#define MXC_INT_USBOTG		37
#define MXC_INT_ESAI		40
#define MXC_INT_CAN1		43
#define MXC_INT_CAN2		44
#define MXC_INT_MLB		46
#define MXC_INT_SPDIF		47
#define MXC_INT_FEC		57
#define MXC_INT_RNGC		MXC_INT_RNGA

#define MXC_INT_FORCE		MXC_INT_RESV1

/*!
 * Defines for modules using static and dynamic DMA channels
 */
#define MXC_DMA_CHANNEL_IRAM         30
#define MXC_DMA_CHANNEL_UART1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC1  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC2  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC3  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#ifdef CONFIG_SDMA_IRAM
#define MXC_DMA_CHANNEL_SSI1_TX  (MXC_DMA_CHANNEL_IRAM + 1)
#else
#define MXC_DMA_CHANNEL_SSI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#endif
#define MXC_DMA_CHANNEL_SSI2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MEMORY  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SPDIF_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SPDIF_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCC_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCC_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ESAI_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ESAI_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_ESAI MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_ESAI MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCC_ESAI MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI1_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI1_TX1 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI2_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI2_TX1 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI1_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI1_TX1 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI2_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI2_TX1 MXC_DMA_DYNAMIC_CHANNEL

#define DMA_REQ_ASRC_DMA6  41
#define DMA_REQ_ASRC_DMA5  40
#define DMA_REQ_ASRC_DMA4  39
#define DMA_REQ_ASRC_DMA3  38
#define DMA_REQ_ASRC_DMA2  37
#define DMA_REQ_ASRC_DMA1  36
#define DMA_REQ_RSVD3      35
#define DMA_REQ_RSVD2      34
#define DMA_REQ_ESAI_TX    33
#define DMA_REQ_ESAI_RX    32
#define DMA_REQ_IPU        21
#define DMA_REQ_RSVD1      20
#define DMA_REQ_SPDIF_TX   13
#define DMA_REQ_SPDIF_RX   12
#define DMA_REQ_UART3_TX   11
#define DMA_REQ_UART3_RX   10
#define DMA_REQ_MSHC       5
#define DMA_REQ_DPTC	   1
#define DMA_REQ_DVFS  	   1

/*!
 * NFMS bit in RCSR register for pagesize of nandflash
 */
#define NFMS		(*((volatile u32 *)IO_ADDRESS(CCM_BASE_ADDR+0x18)))
#define NFMS_BIT		8
#define NFMS_NF_DWIDTH		14
#define NFMS_NF_PG_SZ		8
