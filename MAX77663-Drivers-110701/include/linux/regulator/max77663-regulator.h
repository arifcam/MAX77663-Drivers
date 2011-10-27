/* 
 * max77663-regulator.h 
 * Maxim LDO and Buck regulators driver
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __MAX77663_REGULATOR_H__
#define __MAX77663_REGULATOR_H__

#include <linux/regulator/machine.h>

/* Step-down
 */
#define MAX77663_VREG_ID_B0			0
#define MAX77663_VREG_ID_DVSB0		1
#define MAX77663_VREG_ID_B1			2
#define MAX77663_VREG_ID_DVSB1		3
#define MAX77663_VREG_ID_B2			4
#define MAX77663_VREG_ID_B3			5
#define MAX77663_VREG_ID_B4			6

/* LDO 
 */
#define MAX77663_VREG_ID_L0			7
#define MAX77663_VREG_ID_L1			8
#define MAX77663_VREG_ID_L2			9
#define MAX77663_VREG_ID_L3			10
#define MAX77663_VREG_ID_L4			11
#define MAX77663_VREG_ID_L5			12
#define MAX77663_VREG_ID_L6			13
#define MAX77663_VREG_ID_L7			14
#define MAX77663_VREG_ID_L8			15

#define MAX77663_VREG_MAX			16

/* B0 and B1 pin ctrl 
 */
enum max77663_vreg_pin_ctrl {
	MAX77663_VREG_PIN_CTRL_ENABLE = 0,
	MAX77663_VREG_PIN_CTRL_MODE,
};

struct max77663_vreg_pdata {
	struct regulator_init_data		init_data;
	unsigned int				    pin;
	enum max77663_vreg_pin_ctrl		pin_ctrl;
};

#endif /* __MAX77663_REGULATOR_H__ */

