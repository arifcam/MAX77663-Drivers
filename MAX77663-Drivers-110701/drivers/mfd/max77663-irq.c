/*
 * Maxim MAX77663 IRQ driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77663-core.h>


#define MAX77663_IRQTOP_REG     0x05
#define MAX77663_IRQTOPM_REG    0x0D
#define MAX77663_IRQ_GLBL_MASK  0x80
#define MAX77663_IRQ_SD_MASK    0x40
#define MAX77663_IRQ_LDO_MASK   0x20
#define MAX77663_IRQ_GPIO_MASK  0x10
#define MAX77663_IRQ_RTC_MASK   0x08
#define MAX77663_IRQ_32K_MASK   0x04
#define MAX77663_IRQ_ONOFF_MASK 0x02
#define MAX77663_IRQ_NVER_MASK  0x01

#define MAX77663_INTLBT_REG             0x06
#define MAX77663_INTLBTM_REG            0x0E
#define MAX77663_INTLBT_LB_MASK         0x08
#define MAX77663_INTLBT_TJALRM1_MASK    0x04
#define MAX77663_INTLBT_TJALRM2_MASK    0x02

struct max77663_irq_data {
	int	reg;
	int	mask_reg;
	int	enable;		/* enable interrupt */
    int mask_bit;   /* bit in mask registe*/
    int irq_bit;    /* bit in interrupt register*/
};

static struct max77663_irq_data max77663_irqtop[] = {
	[MAX77663_IRQTOP_NVER] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit    = MAX77663_IRQ_NVER_MASK,
		.mask_bit	= MAX77663_IRQ_NVER_MASK,
        .enable     = MAX77663_IRQ_NVER_MASK,
	},
	[MAX77663_IRQTOP_ONOFF] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit	= MAX77663_IRQ_ONOFF_MASK,
        .mask_bit	= MAX77663_IRQ_ONOFF_MASK,
        .enable     = MAX77663_IRQ_ONOFF_MASK,
	},
	[MAX77663_IRQTOP_32K] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit	= MAX77663_IRQ_32K_MASK,
        .mask_bit	= MAX77663_IRQ_32K_MASK,
        .enable     = 0x00,
	},
	[MAX77663_IRQTOP_RTC] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit	= MAX77663_IRQ_RTC_MASK,
        .mask_bit	= MAX77663_IRQ_RTC_MASK,
        .enable     = MAX77663_IRQ_RTC_MASK,
	},
	[MAX77663_IRQTOP_GPIO] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit	= MAX77663_IRQ_GPIO_MASK,
        .mask_bit	= MAX77663_IRQ_GPIO_MASK,
        .enable     = 0x00,
	},
	[MAX77663_IRQTOP_LDO] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit	= MAX77663_IRQ_LDO_MASK,
        .mask_bit	= MAX77663_IRQ_LDO_MASK,
        .enable     = 0x00,
	},
	[MAX77663_IRQTOP_SD] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit	= MAX77663_IRQ_SD_MASK,
        .mask_bit	= MAX77663_IRQ_SD_MASK,
        .enable     = 0x00,
	},
	[MAX77663_IRQTOP_GLBL] = {
		.reg		= MAX77663_IRQTOP_REG,
		.mask_reg	= MAX77663_IRQTOPM_REG,
        .irq_bit	= MAX77663_IRQ_GLBL_MASK,
        .mask_bit	= MAX77663_IRQ_GLBL_MASK,
        .enable     = MAX77663_IRQ_GLBL_MASK,
	},
};

static irqreturn_t max77663_irqtop_isr(int irq, void *data)
{
	struct max77663_chip *chip = data;
	struct max77663_irq_data *irq_data;
	u8 irqtop= 0;
	int i = 0, ret = 0, handled = 0;
    u16 irqs_to_handle[MAX77663_IRQTOP_NR_INTS];

    printk(KERN_ERR "Top level IRQ ISR enter\n");

    // read first level interrupt in group A
    ret = max77663_read(chip, MAX77663_IRQTOP_REG, &irqtop, 1);
    if (ret == -EINVAL)
    {
        printk(KERN_ERR "Top level IRQ read error\n");
    }
    else {
        printk(KERN_ERR "Top level IRQ read =0x%x\n", irqtop);
    }

    for (i = MAX77663_IRQTOP_START; i <= MAX77663_IRQTOP_END; i++) {
        irq_data = &max77663_irqtop[i];
        if ((irqtop & irq_data->irq_bit) && (irq_data->enable != 0)) {
            // top level interrupt happened
            /* Found one */
			irqs_to_handle[handled] = i + chip->pdata.irq_base;
			handled++;
        }
    }


	for (i = 0; i < handled; i++)
    {
        printk(KERN_ERR "Top level IRQ nested =0x%x\n", irqs_to_handle[i]);
		handle_nested_irq(irqs_to_handle[i]);
    }

    printk(KERN_ERR "Top level IRQ ISR exit\n");

    return IRQ_HANDLED;
}

/* Internal functions */
static void max77663_irqtop_disable(unsigned int irq)
{
    // disable IRQ
    struct	max77663_chip *chip = get_irq_data(irq);

    irq -= chip->pdata.irq_base;

    max77663_irqtop[irq].enable = 0;
}

static void max77663_irqtop_enable(unsigned int irq)
{
    // enable IRQ
    struct	max77663_chip *chip = get_irq_data(irq);

    irq -= chip->pdata.irq_base;

    max77663_irqtop[irq].enable = max77663_irqtop[irq].mask_bit;
}

static void max77663_irqtop_lock(unsigned int irq)
{
	struct max77663_chip *chip = get_irq_data(irq);
	mutex_lock(&chip->irq_lock);
}

static void max77663_irqtop_sync_unlock(unsigned int irq)
{
    struct max77663_irq_data *irq_data;

    struct	max77663_chip *chip = get_irq_data(irq);

    u8 config = 0;
    static u8 cache_mask = 0x74;    // default mask value

    irq -= chip->pdata.irq_base;
    irq_data = &max77663_irqtop[irq];
    // 0 - Enable, 1 - Disable
    if (irq_data->enable == 0) 
    {
        // Disable IRQ : mask bit - 1
        config = cache_mask | irq_data->mask_bit;
    }
    else
    {
        // Enable IRQ : mask bit - 0
        config = cache_mask & ~irq_data->mask_bit;
    }

    if (cache_mask != config) {
        max77663_write(chip, irq_data->mask_reg, &config, 1);
        cache_mask = config;
        printk(KERN_ERR "TOPIRQM write irq_num=%d\n", irq);
    }
    mutex_unlock(&chip->irq_lock);
}

static struct irq_chip max77663_irqtop_chip = {
	.name		= "max77663-irqtop",
	.bus_lock	= max77663_irqtop_lock,
	.bus_sync_unlock = max77663_irqtop_sync_unlock,
    .enable     = max77663_irqtop_enable,
    .disable    = max77663_irqtop_disable,
};

static int max77663_irq_init(struct max77663_chip *chip, int irq,
			    struct max77663_platform_data *pdata)
{
	int i, ret = 0;
    unsigned long flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_DISABLED;

	mutex_init(&chip->irq_lock);
	chip->irq_base = pdata->irq_base;

    /* register with genirq */
    for (i = pdata->irq_base; i < (pdata->irq_base + MAX77663_IRQTOP_NR_INTS); i++) {
        set_irq_chip_data(i, chip);
        set_irq_chip_and_handler(i, &max77663_irqtop_chip,
                     handle_edge_irq);
        set_irq_flags(i, IRQF_VALID);
        set_irq_nested_thread(i, 1);
    }

    ret = request_threaded_irq(irq, NULL, max77663_irqtop_isr, flags,
				   "max8957-irqtop", chip);
	if (ret) {
		dev_err(&chip->dev->dev, "Failed to request IRQ: %d\n", irq);
	}

    return ret;
}

int __devinit max77663_device_init(struct max77663_chip *chip,
				  struct max77663_platform_data *pdata)
{
	int ret;

	ret = max77663_irq_init(chip, chip->dev->irq, pdata);

    // initialize some device or register setting here

    return ret;
}

void __devexit max77663_device_exit(struct max77663_chip *chip)
{
	if (chip->dev->irq)
		free_irq(chip->dev->irq, chip);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77663 IRQ driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:max77663-irq");
