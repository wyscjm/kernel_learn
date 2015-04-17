#include <linux/tty.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-pnx.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/leds.h>

#include <asm/arch/hardware.h>
#include <asm/arch/platform.h>
#include <asm/arch/lpc32xx_gpio.h>

#define GPIO_IOBASE io_p2v(GPIO_BASE)


static void daq1000_beep_set(struct led_classdev *led_cdev,
                enum led_brightness value)
{
        if (value) {
		__raw_writel(_BIT(5), GPIO_P3_OUTP_CLR(GPIO_IOBASE));	
        } else {
		__raw_writel(_BIT(5), GPIO_P3_OUTP_SET(GPIO_IOBASE));
        }
}

static struct led_classdev daq1000_beep = {
        .name           = "daq1000-beep",
        .brightness_set = daq1000_beep_set,
};

static int daq1000_beep_probe(struct platform_device *pdev)
{
        int err = 0;

    err = led_classdev_register(&pdev->dev, &daq1000_beep);
    if (err < 0)
        goto out;

out:
        return err;
}
static int daq1000_beep_remove(struct platform_device *pdev)
{
    led_classdev_unregister(&daq1000_beep);
        return 0;
}
#ifndef CONFIG_PM
static int daq1000_beep_suspend(struct platfrom_device *pdev)
{
        led_classdev_suspend(&daq1000_beep);
        return 0;
}
static int daq1000_beep_resume(struct platfrom_device *pdev)
{
        led_classdev_resume(&daq1000_beep);
        return 0;
}
#else
#define daq1000_beep_suspend NULL
#define daq1000_beep_resume NULL
#endif

static struct platform_driver daq1000_beep_driver = {
        .probe          = daq1000_beep_probe,
        .remove         = daq1000_beep_remove,
        .suspend        = daq1000_beep_suspend,
        .resume         = daq1000_beep_resume,
        .driver         = {
                .name   = "daq1000_beep",
                .owner  = THIS_MODULE,
        },
};

static struct platform_device *daq1000_beep_devs;

static int __init daq1000_beep_init(void)
{
	int ret;

	printk("daq-1000 beep init \n");

	 daq1000_beep_devs = platform_device_alloc("daq1000_beep",
                                                    NULL);
        if (!daq1000_beep_devs) {
                ret = -ENOMEM;
		return ret;
        }

        ret = platform_device_add(daq1000_beep_devs);
        if (ret)
		return ret;

        return platform_driver_register(&daq1000_beep_driver);
}

static void __exit daq1000_beep_exit(void)
{
        return platform_driver_unregister(&daq1000_beep_driver);
}

module_init(daq1000_beep_init);
module_exit(daq1000_beep_exit);

MODULE_AUTHOR("zhuguojun <www.zlgmcu.com>");
MODULE_DESCRIPTION("leds to beep driver");
MODULE_LICENSE("GPL");

