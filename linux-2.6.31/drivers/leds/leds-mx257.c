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

#include <linux/gpio.h>

extern void M257_err_led_ctl(int value);
extern void M257_run_led_ctl(int value);
extern void M257_beep_ctl(int value);

static void M257_led1_set(struct led_classdev *led_cdev,
                enum led_brightness value)
{
	M257_err_led_ctl(value);
}

static struct led_classdev M257_led1 = {
        .name           = "user-led",
        .brightness_set = M257_led1_set,
//	.default_trigger= "ide-disk",
};

static void M257_led2_set(struct led_classdev *led_cdev,
                enum led_brightness value)
{
	M257_run_led_ctl(value);
}

static struct led_classdev M257_led2 = {
        //.name           = "error-led",
        .name           = "running-led",
        .brightness_set = M257_led2_set,
	.default_trigger= "heartbeat",
};

static void M257_led3_set(struct led_classdev *led_cdev,
                enum led_brightness value)
{
	M257_beep_ctl(value);
}

static struct led_classdev M257_led3 = {
        //.name           = "error-led",
        .name           = "beep",
        .brightness_set = M257_led3_set,
};

static int M257_led_probe(struct platform_device *pdev)
{
        int err = 0;

    	err = led_classdev_register(&pdev->dev, &M257_led1);
    	if (err < 0) {
		printk("init LED1 faile \n");
        	goto out;
	}

	err = led_classdev_register(&pdev->dev, &M257_led2);
    	if (err < 0) {
		printk("init LED2 faile \n");
        	goto out;
	}

	err = led_classdev_register(&pdev->dev, &M257_led3);
    	if (err < 0) {
		printk("init LED3 faile \n");
        	goto out;
	}
	
out:
        return err;
}
static int M257_led_remove(struct platform_device *pdev)
{
    	led_classdev_unregister(&M257_led1);
    	led_classdev_unregister(&M257_led2);
    	led_classdev_unregister(&M257_led3);

        return 0;
}
#ifndef CONFIG_PM
static int M257_led_suspend(struct platfrom_device *pdev)
{
        led_classdev_suspend(&M257_led1);
        led_classdev_suspend(&M257_led2);

        return 0;
}
static int M257_led_resume(struct platfrom_device *pdev)
{
        led_classdev_resume(&M257_led1);
        led_classdev_resume(&M257_led2);

        return 0;
}
#else
#define M257_led_suspend NULL
#define M257_led_resume NULL
#endif

static struct platform_driver M257_led_driver = {
        .probe          = M257_led_probe,
        .remove         = M257_led_remove,
        .suspend        = M257_led_suspend,
        .resume         = M257_led_resume,
        .driver         = {
                .name   = "M257_led",
                .owner  = THIS_MODULE,
        },
};

static struct platform_device *M257_led_devs;

static int __init M257_led_init(void)
{
	int ret;

	M257_led_devs = platform_device_alloc("M257_led",
                                                    NULL);
        if (!M257_led_devs) {
                ret = -ENOMEM;
                return ret;
        }

        ret = platform_device_add(M257_led_devs);
        if (ret)
                return ret;

        return platform_driver_register(&M257_led_driver);
}

static void __exit M257_led_exit(void)
{
        return platform_driver_unregister(&M257_led_driver);
}

module_init(M257_led_init);
module_exit(M257_led_exit);

MODULE_AUTHOR("Zhou Shaochuan <tenmachow@gmail.com>");
MODULE_DESCRIPTION("DEMO LED driver");
MODULE_LICENSE("GPL");


