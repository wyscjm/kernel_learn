/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <mach/mmc.h>
#include <mach/spba.h>
#include <mach/sdma.h>

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>

#include "iomux.h"
#include "sdma_script_code.h"
#include "board-mx25_3stack.h"

void mxc_sdma_get_script_info(sdma_script_start_addrs * sdma_script_addr)
{
	sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR;
	sdma_script_addr->mxc_sdma_ap_2_bp_addr = -1;
	sdma_script_addr->mxc_sdma_bp_2_ap_addr = -1;
	sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;

	sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
	sdma_script_addr->mxc_sdma_firi_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_firi_addr = -1;

	sdma_script_addr->mxc_sdma_uart_2_per_addr = uart_2_per_ADDR;
	sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_per_2_app_addr = per_2_app_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR;

	sdma_script_addr->mxc_sdma_per_2_per_addr = -1;

	sdma_script_addr->mxc_sdma_uartsh_2_per_addr = uartsh_2_per_ADDR;
	sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr = uartsh_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_per_2_shp_addr = per_2_shp_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR;

	sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR;

	sdma_script_addr->mxc_sdma_app_2_per_addr = app_2_per_ADDR;
	sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_shp_2_per_addr = shp_2_per_ADDR;
	sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR;

	sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = -1;

	sdma_script_addr->mxc_sdma_spdif_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_spdif_addr = -1;

	sdma_script_addr->mxc_sdma_asrc_2_mcu_addr = -1;

	sdma_script_addr->mxc_sdma_dptc_dvfs_addr = -1;
	sdma_script_addr->mxc_sdma_ext_mem_2_ipu_addr = ext_mem__ipu_ram_ADDR;
	sdma_script_addr->mxc_sdma_descrambler_addr = -1;

	sdma_script_addr->mxc_sdma_start_addr = (unsigned short *)sdma_code;
	sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
	sdma_script_addr->mxc_sdma_ram_code_start_addr = RAM_CODE_START_ADDR;
}

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_RTC_DRV_IMXDI) || defined(CONFIG_RTC_DRV_IMXDI_MODULE)
static struct resource rtc_resources[] = {
	{
	 .start = SRTC_BASE_ADDR,
	 .end = SRTC_BASE_ADDR + 0x40,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MXC_INT_DRYICE_NORM,
	 .flags = IORESOURCE_IRQ,
	 },
};
static struct platform_device imxdi_rtc_device = {
	.name = "imxdi_rtc",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};
static void mxc_init_rtc(void)
{
	(void)platform_device_register(&imxdi_rtc_device);
}
#else
static inline void mxc_init_rtc(void)
{
}
#endif

#if defined(CONFIG_MXC_WATCHDOG) || defined(CONFIG_MXC_WATCHDOG_MODULE)
static struct resource wdt_resources[] = {
	{
	 .start = WDOG1_BASE_ADDR,
	 .end = WDOG1_BASE_ADDR + 0x30,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device mxc_wdt_device = {
	.name = "mxc_wdt",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource = wdt_resources,
};

static void mxc_init_wdt(void)
{
	(void)platform_device_register(&mxc_wdt_device);
}
#else
static inline void mxc_init_wdt(void)
{
}
#endif

/* SPI controller and device data */
#if defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)

#ifdef CONFIG_SPI_MXC_SELECT1
/*!
 * Resource definition for the CSPI1
 */
static struct resource mxcspi1_resources[] = {
	[0] = {
	       .start = CSPI1_BASE_ADDR,
	       .end = CSPI1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_CSPI1,
	       .end = MXC_INT_CSPI1,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI1 */
static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 7,
};

/*! Device Definition for MXC CSPI1 */
static struct platform_device mxcspi1_device = {
	.name = "mxc_spi",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi1_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi1_resources),
	.resource = mxcspi1_resources,
};

#endif				/* CONFIG_SPI_MXC_SELECT1 */

#ifdef CONFIG_SPI_MXC_SELECT2
/*!
 * Resource definition for the CSPI2
 */
static struct resource mxcspi2_resources[] = {
	[0] = {
	       .start = CSPI2_BASE_ADDR,
	       .end = CSPI2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_CSPI2,
	       .end = MXC_INT_CSPI2,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI2 */
static struct mxc_spi_master mxcspi2_data = {
	.maxchipselect = 4,
	.spi_version = 7,
};

/*! Device Definition for MXC CSPI2 */
static struct platform_device mxcspi2_device = {
	.name = "mxc_spi",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi2_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi2_resources),
	.resource = mxcspi2_resources,
};
#endif				/* CONFIG_SPI_MXC_SELECT2 */

#ifdef CONFIG_SPI_MXC_SELECT3
/*!
 * Resource definition for the CSPI3
 */
static struct resource mxcspi3_resources[] = {
	[0] = {
	       .start = CSPI3_BASE_ADDR,
	       .end = CSPI3_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_CSPI3,
	       .end = MXC_INT_CSPI3,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI3 */
static struct mxc_spi_master mxcspi3_data = {
	.maxchipselect = 4,
	.spi_version = 7,
};

/*! Device Definition for MXC CSPI3 */
static struct platform_device mxcspi3_device = {
	.name = "mxc_spi",
	.id = 2,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi3_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi3_resources),
	.resource = mxcspi3_resources,
};
#endif				/* CONFIG_SPI_MXC_SELECT3 */

static inline void mxc_init_spi(void)
{
	spba_take_ownership(SPBA_CSPI2, SPBA_MASTER_A);
	spba_take_ownership(SPBA_CSPI3, SPBA_MASTER_A);

#ifdef CONFIG_SPI_MXC_SELECT1
	if (platform_device_register(&mxcspi1_device) < 0)
		printk(KERN_ERR "Error: Registering the SPI Controller_1\n");
#endif				/* CONFIG_SPI_MXC_SELECT1 */
#ifdef CONFIG_SPI_MXC_SELECT2
	if (platform_device_register(&mxcspi2_device) < 0)
		printk(KERN_ERR "Error: Registering the SPI Controller_2\n");
#endif				/* CONFIG_SPI_MXC_SELECT2 */
#ifdef CONFIG_SPI_MXC_SELECT3
	if (platform_device_register(&mxcspi3_device) < 0)
		printk(KERN_ERR "Error: Registering the SPI Controller_3\n");
#endif				/* CONFIG_SPI_MXC_SELECT3 */
  	struct spi_board_info info[] = {
        	{
                	.modalias = "spidev",
                	.max_speed_hz = 3000000,
                	.bus_num = 1,
                	.chip_select = 0,
        	},
        	{
                	.modalias = "spidev",
                	.max_speed_hz = 3000000,
                	.bus_num = 1,
                	.chip_select = 1,
        	},
    	};

        return spi_register_board_info(&info, 3);
}
#else
static inline void mxc_init_spi(void)
{
}
#endif

/* I2C controller and device data */
#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
/*!
 * Resource definition for the I2C1
 */
static struct resource mxci2c1_resources[] = {
	[0] = {
	       .start = I2C_BASE_ADDR,
	       .end = I2C_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_I2C,
	       .end = MXC_INT_I2C,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC I2C */
static struct mxc_i2c_platform_data mxci2c1_data = {
	.i2c_clk = 100000,
};
#endif

#ifdef CONFIG_I2C_MXC_SELECT2
/*!
 * Resource definition for the I2C2
 */
static struct resource mxci2c2_resources[] = {
	[0] = {
	       .start = I2C2_BASE_ADDR,
	       .end = I2C2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_I2C2,
	       .end = MXC_INT_I2C2,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC I2C */
static struct mxc_i2c_platform_data mxci2c2_data = {
	.i2c_clk = 100000,
};
#endif

#ifdef CONFIG_I2C_MXC_SELECT3
/*!
 * Resource definition for the I2C3
 */
static struct resource mxci2c3_resources[] = {
	[0] = {
	       .start = I2C3_BASE_ADDR,
	       .end = I2C3_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_I2C3,
	       .end = MXC_INT_I2C3,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC I2C */
static struct mxc_i2c_platform_data mxci2c3_data = {
	.i2c_clk = 100000,
};
#endif

#if 0
#define RTC_SDA_IOMUX_PIN 	MX25_PIN_VSTBY_REQ
#define RTC_SCL_IOMUX_PIN	MX25_PIN_UPLL_BYPCLK
#else 
#define RTC_SCL_IOMUX_PIN 	MX25_PIN_VSTBY_REQ
#define RTC_SDA_IOMUX_PIN	MX25_PIN_UPLL_BYPCLK
#endif
static struct i2c_gpio_platform_data i2cgpio_1_pdata = {
        .sda_pin                = IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN),
        .sda_is_open_drain      = 0,
        .scl_pin                = IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN),
        .scl_is_open_drain      = 0,
        .scl_is_output_only     = 1,
        .udelay                 = 2,            /* ~100 kHz */
};

static struct i2c_gpio_platform_data i2cgpio_2_pdata = {
        //.sda_pin                = IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL),
        .scl_pin                = IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL),
        .sda_is_open_drain      = 0,
        //.scl_pin                = IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK),
        .sda_pin                = IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK),
        .scl_is_open_drain      = 0,
        .scl_is_output_only     = 1,
        .udelay                 = 2,            /* ~100 kHz */
};


/*! Device Definition for MXC I2C1 */
static struct platform_device mxci2c_devices[] = {
#ifdef CONFIG_I2C_MXC_SELECT1
	{
	 .name = "mxc_i2c",
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &mxci2c1_data,
		 },
	 .num_resources = ARRAY_SIZE(mxci2c1_resources),
	 .resource = mxci2c1_resources,},
#endif
#ifdef CONFIG_I2C_MXC_SELECT2
	{
	 .name = "mxc_i2c",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &mxci2c2_data,
		 },
	 .num_resources = ARRAY_SIZE(mxci2c2_resources),
	 .resource = mxci2c2_resources,},
#endif
#ifdef CONFIG_I2C_MXC_SELECT3
	{
	 .name = "mxc_i2c",
	 .id = 2,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &mxci2c3_data,
		 },
	 .num_resources = ARRAY_SIZE(mxci2c3_resources),
	 .resource = mxci2c3_resources,},
#endif

	{
        .name                   = "i2c-gpio",
        .id                     = 2,
        .dev.platform_data      = &i2cgpio_1_pdata,
	},

	{
        .name                   = "i2c-gpio",
        .id                     = 1,
        .dev.platform_data      = &i2cgpio_2_pdata,
	},
	
};

static struct i2c_board_info __initdata GPIO_i2c1_board_info [] = {
        {
                //I2C_BOARD_INFO("rtc-pcf8563", 0x51),
                I2C_BOARD_INFO("pcf8563", 0x51),
        },
};
#include <mach/mxc.h>
#include <linux/delay.h>
#include <mach/mx25.h>
#define IOMUXGPR                (IO_ADDRESS(IOMUXC_BASE_ADDR))
static inline void mxc_init_i2c(void)
{
	int i, status = 0;

	for (i = 0; i < ARRAY_SIZE(mxci2c_devices); i++) {
		if (platform_device_register(&mxci2c_devices[i]) < 0)
			dev_err(&mxci2c_devices[i].dev,
				"Unable to register I2C device\n");
	}

	//printk("rtc sda: %d \n", IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN));
	//printk("rtc scl: %d \n", IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN));
	// 初始化 gpio模拟i2c引脚

//	printk("pin SDA :0x%x \n",readl(IOMUXGPR + PIN_TO_IOMUX_MUX(RTC_SDA_IOMUX_PIN)));
	if  (mxc_request_iomux(RTC_SDA_IOMUX_PIN, 	MUX_CONFIG_ALT5)) {  	/* i2c1 SDA GPIO3[17]		*/
		printk("RTC_SDA_IOMUX_PIN reques pin faile \n");
	}
	if (mxc_request_iomux(RTC_SCL_IOMUX_PIN, 	MUX_CONFIG_ALT5)) {	/* i2c1 SCL GPIO3[16]		*/
		printk("RTC_SCL_IOMUX_PIN reques pin faile \n");
	}

	mxc_request_iomux(MX25_PIN_POWER_FAIL,  MUX_CONFIG_ALT5);	/* i2c2 SDA GPIO3[19]		*/
	mxc_request_iomux(MX25_PIN_VSTBY_ACK,  	MUX_CONFIG_ALT5);	/* i2c2 SCL GPIO3[18]		*/

#if 0
        mxc_iomux_set_pad(RTC_SDA_IOMUX_PIN, 	PAD_CTL_SRE_FAST | PAD_CTL_DRV_NORMAL | PAD_CTL_ODE_CMOS | 
						PAD_CTL_22K_PU | PAD_CTL_PUE_PULL |PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_3_3V);
#else 
        mxc_iomux_set_pad(RTC_SCL_IOMUX_PIN, 	PAD_CTL_SRE_FAST | PAD_CTL_DRV_NORMAL | PAD_CTL_ODE_CMOS | 
						PAD_CTL_22K_PU | PAD_CTL_PUE_PULL |PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_3_3V);
#endif
        //mxc_iomux_set_pad(RTC_SCL_IOMUX_PIN, PAD_CTL_22K_PU | PAD_CTL_DRV_MAX);
        mxc_iomux_set_pad(MX25_PIN_POWER_FAIL, PAD_CTL_22K_PU);
        mxc_iomux_set_pad(MX25_PIN_VSTBY_ACK,  PAD_CTL_22K_PU);

	gpio_free(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN));
	gpio_free(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN));

	gpio_free(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL));
	gpio_free(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK));
#if 0	
	status = gpio_request(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN), "rtc_sda");
	if (status) {
		printk("reques RTC_SDA_IOMUX_PIN gpio faile:%d \n", status);
	}
	status = gpio_request(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN), "rtc_clk");
	if (status) {
		printk("reques RTC_SCL_IOMUX_PIN gpio faile:%d \n", status);
	}

	status = gpio_direction_input(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN));
	if (status){
		printk("set RTC_SDA_IOMUX_PIN input faile:%d  \n", status);
	}
	status = gpio_direction_input(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN));
	if (status){
		printk("set RTC_SCL_IOMUX_PIN input faile:%d \n", status);
	}
	
	printk("SDA input: %x \n", gpio_get_value(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN)));
	printk("CLK input: %x \n", gpio_get_value(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN)));

	gpio_direction_output(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN), 1);
	gpio_direction_output(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN), 1);

	gpio_set_value(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN), 0);  
	gpio_set_value(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN), 0);   
        udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN), 1);  
        gpio_set_value(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN), 1);   
        udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN), 0);  
        gpio_set_value(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN), 0);   
	udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN), 1);  
        gpio_set_value(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN), 1);   
        udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(RTC_SDA_IOMUX_PIN), 0);  
        gpio_set_value(IOMUX_TO_GPIO(RTC_SCL_IOMUX_PIN), 0);   


	status = gpio_request(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL), "rtc_sda");
	if (status) {
		printk("reques MX25_PIN_POWER_FAIL gpio faile:%d \n", status);
	}
	status = gpio_request(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), "rtc_clk");
	if (status) {
		printk("reques MX25_PIN_VSTBY_ACK gpio faile:%d \n", status);
	}

	status = gpio_direction_input(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL));
	if (status){
		printk("set RTC_SDA_IOMUX_PIN input faile:%d  \n", status);
	}
	status = gpio_direction_input(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK));
	if (status){
		printk("set RTC_SCL_IOMUX_PIN input faile:%d \n", status);
	}
	
	printk("SDA input: %x \n", gpio_get_value(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL)));
	printk("CLK input: %x \n", gpio_get_value(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK)));

	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL), 1);
	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), 1);

	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL), 0);  
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), 0);   
        udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL), 1);  
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), 1);   
        udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL), 0);  
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), 0);   
	udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL), 1);  
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), 1);   
        udelay(2);
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_POWER_FAIL), 0);  
        gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), 0);  
#endif



	i2c_register_board_info(1, GPIO_i2c1_board_info,         /* modefy by zhuguojun          */
                               ARRAY_SIZE(GPIO_i2c1_board_info));
}
#else
static inline void mxc_init_i2c(void)
{
}
#endif

struct mxc_gpio_port mxc_gpio_ports[] = {
	[0] = {
	 .chip.label = "gpio-0",
	 .base = IO_ADDRESS(GPIO1_BASE_ADDR),
	 .irq = MXC_INT_GPIO1,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START
	 },
	[1] = {
	 .chip.label = "gpio-1",
	 .base = IO_ADDRESS(GPIO2_BASE_ADDR),
	 .irq = MXC_INT_GPIO2,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START + 32
	 },
	[2] = {
	 .chip.label = "gpio-2",
	 .base = IO_ADDRESS(GPIO3_BASE_ADDR),
	 .irq = MXC_INT_GPIO3,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 2
	 },
	[3] = {
	 .chip.label = "gpio-3",
	 .base = IO_ADDRESS(GPIO4_BASE_ADDR),
	 .irq = MXC_INT_GPIO4,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 3
	 }
};

int __init mxc_register_gpios(void)
{
	return mxc_gpio_init(mxc_gpio_ports, ARRAY_SIZE(mxc_gpio_ports));
}

static inline void mxc_init_ssi(void)
{
	/* SPBA configuration for SSI - SDMA and MCU are set */
	spba_take_ownership(SPBA_SSI1, SPBA_MASTER_A | SPBA_MASTER_C);
	spba_take_ownership(SPBA_SSI2, SPBA_MASTER_A | SPBA_MASTER_C);
}

static struct platform_device mxc_dma_device = {
	.name = "mxc_dma",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};

static inline void mxc_init_dma(void)
{
	(void)platform_device_register(&mxc_dma_device);
}

/* imx adc driver */
#if defined(CONFIG_IMX_ADC) || defined(CONFIG_IMX_ADC_MODULE)

static struct resource imx_adc_resources[] = {
	[0] = {
	       .start = MXC_INT_TSC,
	       .end = MXC_INT_TSC,
	       .flags = IORESOURCE_IRQ,
	       },
	[1] = {
	       .start = TSC_BASE_ADDR,
	       .end = TSC_BASE_ADDR + PAGE_SIZE,
	       .flags = IORESOURCE_MEM,
	       }
};

static struct platform_device imx_adc_device = {
	.name = "imx_adc",
	.id = 0,
	.num_resources = ARRAY_SIZE(imx_adc_resources),
	.resource = imx_adc_resources,
	.dev = {
		.release = NULL,
		},
};
static void imx_init_adc(void)
{
	(void)platform_device_register(&imx_adc_device);
}
#else
static void imx_init_adc(void)
{
}
#endif

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)

static struct resource flexcan1_resources[] = {
	{
	 .start = CAN1_BASE_ADDR,
	 .end = CAN1_BASE_ADDR + 0x97F,
	 .flags = IORESOURCE_MEM,},
	{
	 .start = MXC_INT_CAN1,
	 .end = MXC_INT_CAN1,
	 .flags = IORESOURCE_IRQ,}
};
static struct resource flexcan2_resources[] = {
	{
	 .start = CAN3_BASE_ADDR,
	 .end = CAN3_BASE_ADDR + 0x97F,
	 .flags = IORESOURCE_MEM,},
	{
	 .start = MXC_INT_CAN2,
	 .end = MXC_INT_CAN2,
	 .flags = IORESOURCE_IRQ,}
};

static struct platform_device flexcan_devices[] = {
	{
	 .name = "FlexCAN",
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &flexcan_data[0],
		 },
	 .num_resources = ARRAY_SIZE(flexcan1_resources),
	 .resource = flexcan1_resources,},
	{
	 .name = "FlexCAN",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &flexcan_data[1],
		 },
	 .num_resources = ARRAY_SIZE(flexcan2_resources),
	 .resource = flexcan2_resources,},
};

static inline void mxc_init_flexcan(void)
{
#ifdef CONFIG_FLEXCAN_MXC_SELECT1
	/* MX25 3stack doesn't use CAN1 */
	platform_device_register(&flexcan_devices[0]);
#endif
	platform_device_register(&flexcan_devices[1]);
}
#else
static inline void mxc_init_flexcan(void)
{
}
#endif

static struct platform_device mxc_alsa_surround_device = {
	.name = "imx-3stack-wm8580",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};

static void mxc_init_surround_audio(void)
{
	platform_device_register(&mxc_alsa_surround_device);
}

#if defined(CONFIG_MXC_IIM) || defined(CONFIG_MXC_IIM_MODULE)
static struct resource mxc_iim_resources[] = {
	{
	 .start = IIM_BASE_ADDR,
	 .end = IIM_BASE_ADDR + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device mxc_iim_device = {
	.name = "mxc_iim",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(mxc_iim_resources),
	.resource = mxc_iim_resources
};

static inline void mxc_init_iim(void)
{
	if (platform_device_register(&mxc_iim_device) < 0)
		dev_err(&mxc_iim_device.dev,
			"Unable to register mxc iim device\n");
}
#else
static inline void mxc_init_iim(void)
{
}
#endif


static int __init mxc_init_devices(void)
{
	mxc_init_wdt();
	mxc_init_spi();
	mxc_init_i2c();
	mxc_init_dma();
	mxc_init_ssi();
	mxc_init_surround_audio();
	mxc_init_rtc();
	imx_init_adc();
	mxc_init_flexcan();
	mxc_init_iim();

	return 0;
}

arch_initcall(mxc_init_devices);
