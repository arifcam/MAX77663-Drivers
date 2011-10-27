/* 
 * linux/max77663-gpio.h
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */
 
#ifndef __MAX77663_GPIO_H__
#define __MAX77663_GPIO_H__

#include <linux/irq.h>
#include <linux/mfd/core.h>

#define MAX77663_GPIOS	 		8

struct max77663_gpio_config {
    const char *name;   // gpio name
    u8  id;             // gpio number
    u8  dbnc;           // debounce
	u8	refe_irq;       // irq mask
    u8  output_value;   // output level
    u8  input_value;    // input level
	u8  direction;      // direction
    u8  ppdrv;          // open-drain/push-pull
	u8	pull_up;         
	u8	pull_down;
	u8	alternate_mode;
};

struct max77663_gpio_platform_data {
	int	gpio_base;
	int	irq_base;
    struct max77663_gpio_config *gpios;
    u8 alternate_mode;
    u8 pull_up;
    u8 pull_down;
};

#endif /* __MAX77663_GPIO_H__ */

