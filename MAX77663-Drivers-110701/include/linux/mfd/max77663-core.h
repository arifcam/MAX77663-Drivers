/* 
 * linux/mfd/max77663-core.h
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */
 
#ifndef __LINUX_MAX77663_H__
#define __LINUX_MAX77663_H__

#include <linux/irq.h>
#include <linux/mfd/core.h>

/* Top level Interrupt */
enum {
    MAX77663_IRQTOP_START,
	MAX77663_IRQTOP_NVER = MAX77663_IRQTOP_START,
	MAX77663_IRQTOP_ONOFF,
	MAX77663_IRQTOP_32K,
	MAX77663_IRQTOP_RTC,
	MAX77663_IRQTOP_GPIO,
	MAX77663_IRQTOP_LDO,
	MAX77663_IRQTOP_SD,
    MAX77663_IRQTOP_GLBL,
	MAX77663_IRQTOP_END = MAX77663_IRQTOP_GLBL,

    MAX77663_ONOFFIRQ_START,
    MAX77663_ONOFFIRQ_ACOK_R = MAX77663_ONOFFIRQ_START,
    MAX77663_ONOFFIRQ_ACOK_F,
    MAX77663_ONOFFIRQ_LID_R,
    MAX77663_ONOFFIRQ_LID_F,
    MAX77663_ONOFFIRQ_EN0_R,
    MAX77663_ONOFFIRQ_EN0_F,
    MAX77663_ONOFFIRQ_EN0_1SEC,
    MAX77663_ONOFFIRQ_HRDPOWRN,
    MAX77663_ONOFFIRQ_END = MAX77663_ONOFFIRQ_HRDPOWRN,

    MAX77663_GPIOIRQ_START,
    MAX77663_GPIOIRQ_EDGE0 = MAX77663_GPIOIRQ_START,
    MAX77663_GPIOIRQ_EDGE1,
    MAX77663_GPIOIRQ_EDGE2,
    MAX77663_GPIOIRQ_EDGE3,
    MAX77663_GPIOIRQ_EDGE4,
    MAX77663_GPIOIRQ_EDGE5,
    MAX77663_GPIOIRQ_EDGE6,
    MAX77663_GPIOIRQ_EDGE7,
    MAX77663_GPIOIRQ_END = MAX77663_GPIOIRQ_EDGE7,

    MAX77663_INTLBT_START,
    MAX77663_INTLBT_TJALRM2_R = MAX77663_INTLBT_START,    
    MAX77663_INTLBT_TJALRM1_R,
    MAX77663_INTLBT_MBATTLOW_R,
    MAX77663_INTLBT_END = MAX77663_INTLBT_MBATTLOW_R,
};

#define MAX77663_IRQTOP_NR_INTS (MAX77663_IRQTOP_END - MAX77663_IRQTOP_START + 1)
#define MAX77663_ONOFFIRQ_NR_INTS (MAX77663_ONOFFIRQ_END - MAX77663_ONOFFIRQ_START + 1)
#define MAX77663_GPIOIRQ_NR_INTS (MAX77663_GPIOIRQ_END - MAX77663_GPIOIRQ_START + 1)
#define MAX77663_INTLBT_NR_INTS (MAX77663_INTLBT_END - MAX77663_INTLBT_START + 1)

struct max77663_platform_data {
	int     		irq_base;
	int			    num_subdevs;
	struct mfd_cell *sub_devices;
};

struct max77663_chip {
	struct max77663_platform_data	pdata;
	struct i2c_client	*dev;
	struct mutex		io_lock;
    struct mutex        irq_lock;

    int irq_base;
};

int max77663_read(struct max77663_chip *chip, u8 addr, u8 *values, unsigned int len);
int max77663_write(struct max77663_chip *chip, u8 addr, u8 *values, unsigned int len);
int max77663_set_bits(struct max77663_chip *chip, u8 addr, u8 mask, u8 value);
extern int max77663_device_init(struct max77663_chip *chip,
				  struct max77663_platform_data *pdata);
extern void max77663_device_exit(struct max77663_chip *chip);
#endif /* __LINUX_MAX77663_H__ */

