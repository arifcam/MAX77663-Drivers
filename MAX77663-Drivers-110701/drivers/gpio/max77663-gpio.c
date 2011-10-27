/* 
 * max77663-gpio.c
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */
 
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/max77663-core.h>
#include <linux/max77663-gpio.h>
#include <linux/seq_file.h>

#define MAX77663_DBNC_M             0xC0
#define MAX77663_DBNC_SHIFT         6
#define MAX77663_NODBNC_V           0x00
#define MAX77663_DBNC_8MS           0x40
#define MAX77663_DBNC_16MS          0x80
#define MAX77663_DBNC_32MS          0xC0

#define MAX77663_REFE_IRQ_M         0x30
#define MAX77663_REFE_IRQ_SHIFT     4
#define MAX77663_MASK_INT           0x00
#define MAX77663_FALLING_EDGE_INT   0x10
#define MAX77663_RISING_EDGE_INT    0x20
#define MAX77663_BOTH_EDGE_INT      0x30

#define MAX77663_GPIO_DOUT_M		0x08
#define MAX77663_GPIO_DOUT_SHIFT	3
#define MAX77663_GPIO_DOUT_HI_V		0x08
#define MAX77663_GPIO_DOUT_LO_V		0x00

#define MAX77663_GPIO_DIN_M		    0x04
#define MAX77663_GPIO_DIN_SHIFT		2
#define MAX77663_GPIO_DIN_HI_V		0x04
#define MAX77663_GPIO_DIN_LO_V		0x00

#define MAX77663_GPIO_DIR_M		    0x02
#define MAX77663_GPIO_DIR_SHIFT		1
#define MAX77663_GPIO_DIR_INPUT_V	0x02
#define MAX77663_GPIO_DIR_OUTPUT_V	0x00

#define MAX77663_PPDRV_M            0x01
#define MAX77663_PPDRV_SHIFT        0
#define MAX77663_OPEN_DRAIN         0x00
#define MAX77663_PUSH_PULL          0x01

#define MAX77663_GPIO_REG_BASE		0x36
#define MAX77663_GPIO_REG_ADDR(ofs)	MAX77663_GPIO_REG_BASE + ofs

#define MAX77663_PUE_GPIO   0x3E
#define MAX77663_PDE_GPIO   0x3F
#define MAX77663_AME_GPIO   0x40  

enum {
    MAX77663_GPIO_OUTPUT = 0,
    MAX77663_GPIO_INPUT,
    MAX77663_GPIO_AME,
};

#define MAX77663_GPIO_AME_ENABLE    1

struct max77663_gpio_chip {
	struct gpio_chip	    chip;
	struct max77663_chip    *max_chip;
	unsigned char ctrl[MAX77663_GPIOS];
};

static struct max77663_chip *max_gpio_chip;

static int max77663_gpio_write(struct max77663_chip *chip, u16 addr, u8 val,
        u8 mask, u8 *bak)
{
	u8 reg = (*bak & ~mask) | (val & mask);

	int ret = max77663_write(chip, addr, &reg, 1);

	if (!ret)
		*bak = reg;

	return ret;
}

static int max77663_gpio_get_pin(struct max77663_chip *chip, int offset)
{
	u8 addr = MAX77663_GPIO_REG_ADDR(offset);
	u8 val;

	max77663_read(chip, addr, &val, 1);

	pr_err("GPIO GET PIN %x %x %d\n", chip->dev->addr, addr, val);

	return (val & MAX77663_GPIO_DIN_M) >> MAX77663_GPIO_DIN_SHIFT;
}

static int max77663_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct max77663_gpio_platform_data *pdata;
	pdata = chip->dev->platform_data;
	return pdata->irq_base + offset;
}

static int max77663_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct max77663_gpio_chip *io_chip = dev_get_drvdata(chip->dev);
	int ret;

	if ((io_chip->ctrl[offset] & MAX77663_GPIO_DIR_M) ==
		MAX77663_GPIO_DIR_OUTPUT_V)
		ret = (io_chip->ctrl[offset] & MAX77663_GPIO_DIN_M) >>
			MAX77663_GPIO_DIN_SHIFT;
	else
		ret = max77663_gpio_get_pin(io_chip->max_chip, offset);

	printk(KERN_INFO "Max77663> %s: ret=%d offset=%d\n", __func__, 
		ret, offset);
        
	printk(KERN_INFO "Max77663> I2C Addr: 0x%x  %x\n", 
		max_gpio_chip->dev->addr, io_chip->max_chip->dev->addr); 
        
	pr_err("Max77663> %s: max77663_gpio_get(): ret=%d\n", __func__, ret);

	return ret;
}

static void max77663_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct max77663_gpio_chip *io_chip = dev_get_drvdata(chip->dev);
	u8 reg = val ? MAX77663_GPIO_DOUT_HI_V : MAX77663_GPIO_DOUT_LO_V;
	int ret;

	ret = max77663_gpio_write(io_chip->max_chip, 
		MAX77663_GPIO_REG_ADDR(offset), 
		reg, MAX77663_GPIO_DOUT_M,
		&io_chip->ctrl[offset]);

	pr_err("Max77663> %s: rc=%d offset=%d val=%d\n", __func__, 
		ret, offset, val);

	pr_err("Max77663> I2C Addr: 0x%x  %x \n", 
		max_gpio_chip->dev->addr, io_chip->max_chip->dev->addr);

	pr_err("Max77663> %s: max77663_gpio_write(): rc=%d\n", __func__, ret);
}

static int max77663_gpio_dir_input(struct gpio_chip *chip, unsigned offset)
{
	struct max77663_gpio_chip *io_chip = dev_get_drvdata(chip->dev);
	int rc = max77663_gpio_write(io_chip->max_chip,
			MAX77663_GPIO_REG_ADDR(offset),
			MAX77663_GPIO_DIR_INPUT_V,
			MAX77663_GPIO_DIR_M, &io_chip->ctrl[offset]);
	
	pr_info("Max77663> %s: rc=%d offset=%d ctl=%x`\n", __func__, 
		rc, offset, io_chip->ctrl[offset]);

	pr_err("Max77663> %s: max77663_gpio_write(): rc=%d\n", __func__, rc);

	return rc;
}

static int max77663_gpio_dir_output(struct gpio_chip *chip,
	unsigned offset, int val)
{
	struct max77663_gpio_chip *io_chip = dev_get_drvdata(chip->dev);
	
	int ret = max77663_gpio_write(io_chip->max_chip,
			MAX77663_GPIO_REG_ADDR(offset),
			MAX77663_GPIO_DIR_OUTPUT_V,
			MAX77663_GPIO_DIR_M, &io_chip->ctrl[offset]);
		
	printk(KERN_INFO "Max77663> %s: rc=%d val=%d ctl=%x\n", __func__, 
		ret, val, io_chip->ctrl[offset]);

	pr_err("Max77663> %s: max77663_gpio_write(): rc=%d\n", __func__, ret);

	return ret;
}

static void max77663_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
        static const char *ctype[] = { "d_in", "d_out", "bi_dir", "a_in",
                "a_out" };
        struct max77663_gpio_chip *io_chip = dev_get_drvdata(chip->dev);
        u8 type, state;
        const char *label;
        int i;

        for (i = 0; i < MAX77663_GPIOS; i++) {
                label = gpiochip_is_requested(chip, i);
                type = (io_chip->ctrl[i] & MAX77663_GPIO_DIR_M) >>
                        MAX77663_GPIO_DIR_SHIFT;
                state = max77663_gpio_get_pin(max_gpio_chip, i);
                seq_printf(s, "gpio-%-3d (%-12.12s) %-10.10s"
                                " %s 0x%02x\n",
                                chip->base + i,
                                label ? label : "--",
                                ctype[type],
                                state ? "hi" : "lo",
                                io_chip->ctrl[i]);
        }
}

static struct max77663_gpio_chip max77663_gpio_chip = {
	.chip = {
		.label			= "max77663-gpio",
		.to_irq			= max77663_gpio_to_irq,
		.get			= max77663_gpio_get,
		.set			= max77663_gpio_set,
		.direction_input	= max77663_gpio_dir_input,
		.direction_output	= max77663_gpio_dir_output,
		.dbg_show		= max77663_gpio_dbg_show,
		.ngpio			= MAX77663_GPIOS,
	},
};

int max77663_gpio_config(unsigned gpio, unsigned type, unsigned level)
{
	u8	config, mask;
	int	ret;

	if (gpio >= MAX77663_GPIOS)
		return -EINVAL;

	if (type == MAX77663_GPIO_OUTPUT)
	{
		mask = MAX77663_GPIO_DIR_M | MAX77663_GPIO_DOUT_M; 
		config = (type << MAX77663_GPIO_DIR_SHIFT) & MAX77663_GPIO_DIR_M;
		config |= (level << MAX77663_GPIO_DOUT_SHIFT) &	MAX77663_GPIO_DOUT_M;

        ret = max77663_gpio_write(max77663_gpio_chip.max_chip, 
                                  MAX77663_GPIO_REG_ADDR(gpio),
                                  config, mask, &max77663_gpio_chip.ctrl[gpio]);
        if (ret) 
            pr_err("Max77663> %s: max77663_gpio_write(): ret=%d\n", __func__, ret);
	}
	else if (type == MAX77663_GPIO_INPUT)
	{
		mask = MAX77663_GPIO_DIR_M; 
		config = (type << MAX77663_GPIO_DIR_SHIFT) & MAX77663_GPIO_DIR_M;
        ret = max77663_gpio_write(max77663_gpio_chip.max_chip, 
                                  MAX77663_GPIO_REG_ADDR(gpio),
                                  config, mask, &max77663_gpio_chip.ctrl[gpio]);
        if (ret) 
            pr_err("Max77663> %s: max77663_gpio_write(): ret=%d\n", __func__, ret);

        mask = 1<<gpio;
        if (level == 1) {
            // pull-up
            ret = max77663_set_bits(max77663_gpio_chip.max_chip, MAX77663_PUE_GPIO, mask, mask);   // no pull-up
            if (ret) 
                pr_err("Max77663> %s: max77663_gpio_write(): ret=%d\n", __func__, ret);
        }
        else
        {
            // pull-down
            ret = max77663_set_bits(max77663_gpio_chip.max_chip, MAX77663_PUE_GPIO, mask, mask);   // no pull-up
            if (ret) 
                pr_err("Max77663> %s: max77663_gpio_write(): ret=%d\n", __func__, ret);
        }
	}
    else    // MAX77663_GPIO_AME
    {
        mask = MAX77663_GPIO_AME_ENABLE<<gpio;
        ret = max77663_set_bits(max77663_gpio_chip.max_chip, MAX77663_AME_GPIO, mask, mask);
        if (ret) 
            pr_err("Max77663> %s: max77663_gpio_write(): ret=%d\n", __func__, ret);

        if (gpio == 7) {
            ret = max77663_set_bits(max77663_gpio_chip.max_chip, MAX77663_PUE_GPIO, mask, 0);   // no pull-up
            if (ret) 
                pr_err("Max77663> %s: max77663_gpio_write(): ret=%d\n", __func__, ret);
                                                                                             
            ret = max77663_set_bits(max77663_gpio_chip.max_chip, MAX77663_PDE_GPIO, mask, 0);   // no pull-down
            if (ret) 
                pr_err("Max77663> %s: max77663_gpio_write(): ret=%d\n", __func__, ret);
        }
    }

	return ret;
}
EXPORT_SYMBOL(max77663_gpio_config);

static int __devinit max77663_gpio_probe(struct platform_device *pdev)
{
	int ret, i;
	struct max77663_gpio_platform_data *pdata = pdev->dev.platform_data;

	pr_info("Max77663> gpio probe\n");

	max77663_gpio_chip.max_chip = platform_get_drvdata(pdev);
	max_gpio_chip = max77663_gpio_chip.max_chip;
	for (i = 0; i < MAX77663_GPIOS; i++) {
        max77663_gpio_chip.ctrl[i] = ( pdata->gpios[i].dbnc << MAX77663_DBNC_SHIFT |
                                       pdata->gpios[i].refe_irq << MAX77663_REFE_IRQ_SHIFT |
                                       pdata->gpios[i].output_value << MAX77663_GPIO_DOUT_SHIFT |
                                       pdata->gpios[i].direction << MAX77663_GPIO_DIR_SHIFT |
                                       pdata->gpios[i].ppdrv << MAX77663_PPDRV_SHIFT );
        ret = max77663_write(max77663_gpio_chip.max_chip, 
                             MAX77663_GPIO_REG_ADDR(i), 
                             &max77663_gpio_chip.ctrl[i], 
                             1);
       if (ret)
			goto bail;
	}

    ret = max77663_write(max77663_gpio_chip.max_chip, MAX77663_AME_GPIO, &pdata->alternate_mode, 1);
    if (ret)
         goto bail;

    ret = max77663_write(max77663_gpio_chip.max_chip, MAX77663_PUE_GPIO, &pdata->pull_up, 1);
    if (ret)
         goto bail;

    ret = max77663_write(max77663_gpio_chip.max_chip, MAX77663_PDE_GPIO, &pdata->pull_down, 1);
    if (ret)
         goto bail;

	platform_set_drvdata(pdev, &max77663_gpio_chip);
	max77663_gpio_chip.chip.dev = &pdev->dev;
	max77663_gpio_chip.chip.base = pdata->gpio_base;
	ret = gpiochip_add(&max77663_gpio_chip.chip);

	pr_info("Max77663> %s: gpiochip_add(): ret=%d\n", __func__, ret);

bail:
	return ret;
}

static int __devexit max77663_gpio_remove(struct platform_device *pdev)
{
	return gpiochip_remove(&max77663_gpio_chip.chip);
}

static struct platform_driver max77663_gpio_driver = {
	.probe		= max77663_gpio_probe,
	.remove		= __devexit_p(max77663_gpio_remove),
	.driver		= {
		.name = "max77663-gpio",
		.owner = THIS_MODULE,
	},
};

static int __init max77663_gpio_init(void)
{
	pr_info("Max77663> gpio init\n");

	return platform_driver_register(&max77663_gpio_driver);
}

static void __exit max77663_gpio_exit(void)
{
	platform_driver_unregister(&max77663_gpio_driver);
}

subsys_initcall(max77663_gpio_init);
module_exit(max77663_gpio_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77663 GPIO driver");
MODULE_VERSION("1.0");


