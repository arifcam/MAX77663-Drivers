/* 
 * max77663-regulator.c 
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

#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max77663-regulator.h>


#define MAX77663_VREG_SD0		    0x16
#define MAX77663_VREG_DVSSD0	    0x1B
#define MAX77663_VREG_SD0_CFG		0x1D
#define MAX77663_VREG_DVSSD0_CFG    0x1D

#define MAX77663_VREG_SD1	    	0x17
#define MAX77663_VREG_DVSSD1		0x1C
#define MAX77663_VREG_SD1_CFG		0x1E
#define MAX77663_VREG_DVSSD1_CFG	0x1E

#define MAX77663_VREG_SD2		    0x18
#define MAX77663_VREG_SD2_CFG		0x1F

#define MAX77663_VREG_SD3		    0x19
#define MAX77663_VREG_SD4		    0x1A
#define MAX77663_VREG_SD3_CFG		0x20
#define MAX77663_VREG_SD4_CFG		0x21

#define MAX77663_VREG_LDO0		    0x23
#define MAX77663_VREG_LDO0_CFG2		0x24
#define MAX77663_VREG_LDO1		    0x25
#define MAX77663_VREG_LDO1_CFG2		0x26
#define MAX77663_VREG_LDO2		    0x27
#define MAX77663_VREG_LDO2_CFG2		0x28
#define MAX77663_VREG_LDO3		    0x29
#define MAX77663_VREG_LDO3_CFG2		0x2A
#define MAX77663_VREG_LDO4		    0x2B
#define MAX77663_VREG_LDO4_CFG2		0x2C
#define MAX77663_VREG_LDO5		    0x2D
#define MAX77663_VREG_LDO5_CFG2		0x2E
#define MAX77663_VREG_LDO6		    0x2F
#define MAX77663_VREG_LDO6_CFG2		0x30
#define MAX77663_VREG_LDO7		    0x31
#define MAX77663_VREG_LDO7_CFG2		0x32
#define MAX77663_VREG_LDO8		    0x33
#define MAX77663_VREG_LDO8_CFG2		0x34

// When FPSSRC_Lx[1:0]=0b11 in FPS_x register
//  - not configured as part of a flexible power sequence.
#define MAX77663_MODE_NORMAL		0x11
#define MAX77663_MODE_LPM           0x10
#define MAX77663_MODE_GLPM          0x01
#define MAX77663_MODE_DISABLE		0x00
// When FPSSRC_Lx[1:0]!=0b11 in FPS_x register
#define MAX77663_MODE_FPS_NORMAL    0x11
#define MAX77663_MODE_FPS_LPM       0x10
#define MAX77663_MODE_FPS_GLPM      0x01


#define MAX77663_VREG_TYPE_SD    	0X00
#define MAX77663_VREG_TYPE_LDO   	0X01

#define MAX77663_SD_MODE_M		    0x30
#define MAX77663_SD_MODE_SHIFT   	4
#define MAX77663_LDO_MODE_M      	0xC0
#define MAX77663_LDO_MODE_SHIFT  	6

#define MAX77663_LDO_VOLT_M      	0x3F

struct max77663_vreg{
	struct max77663_vreg_pdata 	*pdata;
	struct regulator_dev		*rdev;
	u8 id;
    u8 type;
    u8 volt_reg;
    u8 cfg_reg;
    u32 min_uV;
    u32 max_uV;
    u32 step_uV;
	u8 volt_shadow;
	u8 cfg_shadow;
    u8 power_mode;
};

static int max77663_regulator_set_voltage(struct regulator_dev *dev, int min_uV, int max_uV);
static int max77663_regulator_get_voltage(struct regulator_dev *dev);
static int max77663_regulator_enable(struct regulator_dev *dev);
static int max77663_regulator_disable(struct regulator_dev *dev);
static int max77663_regulator_is_enabled(struct regulator_dev *dev);
static int max77663_regulator_set_mode(struct regulator_dev *dev, unsigned int power_mode);
static unsigned int max77663_regulator_get_mode(struct regulator_dev *dev);

static struct regulator_ops max77663_ldo_ops = {
    .set_voltage = max77663_regulator_set_voltage,
    .get_voltage = max77663_regulator_get_voltage,
    .enable = max77663_regulator_enable,
    .disable = max77663_regulator_disable,
    .is_enabled = max77663_regulator_is_enabled,
    .set_mode = max77663_regulator_set_mode,
    .get_mode = max77663_regulator_get_mode,
};

#define VREG(_id, _volt_reg, _cfg_reg, _type, _min, _max, _step) \
    [_id] = { \
	   .id = _id, \
        .min_uV = _min, \
        .max_uV = _max, \
        .step_uV = _step, \
        .volt_reg = _volt_reg, \
        .cfg_reg = _cfg_reg, \
        .power_mode = MAX77663_MODE_NORMAL, \
	.type = _type, \
    }

static struct max77663_vreg max77663_regulators[MAX77663_VREG_MAX] = {
    VREG(MAX77663_VREG_ID_B0, MAX77663_VREG_SD0, MAX77663_VREG_SD0_CFG, MAX77663_VREG_TYPE_SD, 600000, 3387500, 12500),
    VREG(MAX77663_VREG_ID_DVSB0, MAX77663_VREG_DVSSD0, MAX77663_VREG_DVSSD0_CFG, MAX77663_VREG_TYPE_SD, 600000, 3387500, 12500),
    VREG(MAX77663_VREG_ID_B1, MAX77663_VREG_SD1, MAX77663_VREG_SD1_CFG, MAX77663_VREG_TYPE_SD, 800000, 1587500, 12500),
    VREG(MAX77663_VREG_ID_DVSB1, MAX77663_VREG_DVSSD1, MAX77663_VREG_DVSSD1_CFG, MAX77663_VREG_TYPE_SD, 800000, 1587500, 12500),
    VREG(MAX77663_VREG_ID_B2, MAX77663_VREG_SD2, MAX77663_VREG_SD2_CFG, MAX77663_VREG_TYPE_SD, 600000, 3387500, 12500),
    VREG(MAX77663_VREG_ID_B3, MAX77663_VREG_SD3, MAX77663_VREG_SD3_CFG, MAX77663_VREG_TYPE_SD, 600000, 3387500, 12500),
    VREG(MAX77663_VREG_ID_B4, MAX77663_VREG_SD4, MAX77663_VREG_SD4_CFG, MAX77663_VREG_TYPE_SD, 600000, 3387500, 12500),
    VREG(MAX77663_VREG_ID_L0, MAX77663_VREG_LDO0, MAX77663_VREG_LDO0_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 2350000, 25000),
    VREG(MAX77663_VREG_ID_L1, MAX77663_VREG_LDO1, MAX77663_VREG_LDO1_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 2350000, 25000),
    VREG(MAX77663_VREG_ID_L2, MAX77663_VREG_LDO2, MAX77663_VREG_LDO2_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 3950000, 50000),
    VREG(MAX77663_VREG_ID_L3, MAX77663_VREG_LDO3, MAX77663_VREG_LDO3_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 3950000, 50000),
    VREG(MAX77663_VREG_ID_L4, MAX77663_VREG_LDO4, MAX77663_VREG_LDO4_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 1587500, 12500),
    VREG(MAX77663_VREG_ID_L5, MAX77663_VREG_LDO5, MAX77663_VREG_LDO5_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 3950000, 50000),
    VREG(MAX77663_VREG_ID_L6, MAX77663_VREG_LDO6, MAX77663_VREG_LDO6_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 3950000, 50000),
    VREG(MAX77663_VREG_ID_L7, MAX77663_VREG_LDO7, MAX77663_VREG_LDO7_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 3950000, 50000),
    VREG(MAX77663_VREG_ID_L8, MAX77663_VREG_LDO8, MAX77663_VREG_LDO8_CFG2, MAX77663_VREG_TYPE_LDO, 800000, 3950000, 50000),
};

#define VREG_DESC(_id, _name, _ops) \
	[_id] = { \
		.name = _name, \
		.id = _id, \
		.ops = _ops, \
		.type = REGULATOR_VOLTAGE, \
		.owner = THIS_MODULE, \
	}

static struct regulator_desc max77663_regulator_desc[MAX77663_VREG_MAX] = {
    VREG_DESC(MAX77663_VREG_ID_B0, "max77663_sd0", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_DVSB0, "max77663_dvssd0", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_B1, "max77663_sd1", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_DVSB1, "max77663_dvssd1", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_B2, "max77663_sd2", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_B3, "max77663_sd3", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_B4, "max77663_sd4", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L0, "max77663_ldo0", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L1, "max77663_ldo1", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L2, "max77663_ldo2", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L3, "max77663_ldo3", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L4, "max77663_ldo4", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L5, "max77663_ldo5", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L6, "max77663_ldo6", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L7, "max77663_ldo7", &max77663_ldo_ops),
    VREG_DESC(MAX77663_VREG_ID_L8, "max77663_ldo8", &max77663_ldo_ops),
};

static int max77663_vreg_write(struct max77663_chip *chip, u16 addr, u8 val,
        u8 mask, u8 *bak)
{
	u8 reg = (*bak & ~mask) | (val & mask);

	int ret = max77663_write(chip, addr, &reg, 1);

	if (!ret)
		*bak = reg;

	return ret;
}

static int max77663_regulator_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV) 
{
    struct max77663_vreg *vreg= rdev_get_drvdata(rdev);
    struct max77663_chip *chip = dev_get_drvdata(rdev->dev.parent);
    u8 val;
    int rc;
    
    pr_info("Max77663> %s: Addr=%x RequV=%d, MinuV=%d, StpuV=%d\n", __func__, chip->dev->addr, 
	min_uV, vreg->min_uV, vreg->step_uV);

    if (min_uV < vreg->min_uV || max_uV > vreg->max_uV)
        return -EDOM;
            
    val = (min_uV - vreg->min_uV) / vreg->step_uV;

    pr_info("Max77663>  set volt reg val = %d\n", val);

    if (vreg->type == MAX77663_VREG_TYPE_SD)
    {
        rc = max77663_write(chip, vreg->volt_reg, &val, 1);
	   if (rc == 0)
             vreg->volt_shadow = val;
    }
    else
    {
        rc = max77663_vreg_write(chip, vreg->volt_reg, val, MAX77663_LDO_VOLT_M,
            &(vreg->volt_shadow));
    }

    return rc;
}

static int max77663_regulator_get_voltage(struct regulator_dev *rdev)
{
    struct max77663_vreg *vreg= rdev_get_drvdata(rdev);
    struct max77663_chip *chip = dev_get_drvdata(rdev->dev.parent);
    u8 val;
    int volt;
    int rc;
    
    rc = max77663_read(chip, vreg->volt_reg, &val, 1);
    if (rc == 0)
        vreg->volt_shadow = val;

    if (vreg->type == MAX77663_VREG_TYPE_SD)
    {
        volt = val * vreg->step_uV + vreg->min_uV;
    }
    else
    {
        volt = (val & MAX77663_LDO_VOLT_M) * vreg->step_uV + vreg->min_uV;
    }
    
    pr_err("Max77663> %s: val=%d volt=%d\n", __func__, val, volt);
    
    return volt;
}

static int max77663_regulator_enable(struct regulator_dev *rdev)
{
    int rc = -EDOM;

    pr_err("Max77663> %s: \n", __func__);

    rc = max77663_regulator_set_mode(rdev, MAX77663_MODE_NORMAL);

    return rc;
}

static int max77663_regulator_disable(struct regulator_dev *rdev)
{
    int rc = -EDOM;
    
    pr_err("Max77663> %s: \n", __func__);

    rc = max77663_regulator_set_mode(rdev, MAX77663_MODE_DISABLE);

    return rc;
}

static int max77663_regulator_is_enabled(struct regulator_dev *rdev)
{
    struct max77663_vreg *vreg= rdev_get_drvdata(rdev);
    
    pr_err("Max77663> %s: id=%d\n", __func__, vreg->id);

    return (max77663_regulator_get_mode(rdev) != 0);
}

static int max77663_regulator_set_mode(struct regulator_dev *rdev, unsigned int power_mode)
{
    struct max77663_vreg *vreg= rdev_get_drvdata(rdev);
    struct max77663_chip *chip = dev_get_drvdata(rdev->dev.parent);
    int rc;

    pr_err("Max77663> %s: id=%d, power_mode=%d\n", __func__, vreg->id, power_mode);
            
    vreg->power_mode = power_mode;

    if (vreg->type == MAX77663_VREG_TYPE_SD)
    {
        vreg->cfg_shadow = (vreg->cfg_shadow & ~MAX77663_SD_MODE_M) | 
            vreg->power_mode;

        rc = max77663_write(chip, vreg->cfg_reg, &vreg->cfg_shadow, 1);
    }
    else
    {
        vreg->volt_shadow = (vreg->volt_shadow & ~MAX77663_LDO_MODE_M) | 
            vreg->power_mode;

        rc = max77663_write(chip, vreg->volt_reg, &vreg->volt_shadow, 1);
    }

    return 0;
}

static unsigned int max77663_regulator_get_mode(struct regulator_dev *rdev)
{
    struct max77663_vreg *vreg= rdev_get_drvdata(rdev);
    struct max77663_chip *chip = dev_get_drvdata(rdev->dev.parent);
    int rc;
    u8 val = -1;;

    if (vreg->type == MAX77663_VREG_TYPE_SD)
    {
        rc = max77663_read(chip, vreg->cfg_reg, &val, 1);
        if (rc == 0)
        {
            vreg->cfg_shadow = val;
            vreg->power_mode = (vreg->cfg_shadow & MAX77663_SD_MODE_M) >> \
		MAX77663_SD_MODE_SHIFT;
        }
    }
    else
    {
        rc = max77663_read(chip, vreg->volt_reg, &val, 1);
        if (rc == 0)
	{
            vreg->volt_shadow = val;
            vreg->power_mode = (vreg->volt_shadow & MAX77663_LDO_MODE_M) >> \
		MAX77663_LDO_MODE_SHIFT;
        }
    }
    
    pr_err("Max77663> %s: rc=%d, id=%d, val=%d, mode=%d\n", __func__, rc, vreg->id, 
	val, vreg->power_mode);

    return (unsigned int)vreg->power_mode;
}

static int max77663_init_regulator(struct max77663_chip *chip,
        struct max77663_vreg *vreg)
{
    int rc = 0;

    rc = max77663_read(chip, vreg->volt_reg, &vreg->volt_shadow, 1);
    rc = max77663_read(chip, vreg->cfg_reg, &vreg->cfg_shadow, 1);

    pr_err("Max77663> max77663 init.\n");

    return rc;
}

static int max77663_regulator_probe(struct platform_device *pdev)
{
    struct regulator_desc *rdesc;
    struct max77663_chip *chip;
    struct max77663_vreg *vreg;
    const char *reg_name = NULL;
    int rc = 0;

	pr_info("Max77663> regulator probe dev_id=%d\n", pdev->id);

    if (pdev == NULL)
        return -EINVAL;

    if (pdev->id >= 0 && pdev->id < MAX77663_VREG_MAX) 
	{
        chip = platform_get_drvdata(pdev);
		rdesc = &max77663_regulator_desc[pdev->id];
        vreg = &max77663_regulators[pdev->id];
		vreg->pdata = pdev->dev.platform_data;
		reg_name = max77663_regulator_desc[pdev->id].name;

        rc = max77663_init_regulator(chip, vreg);
        if (rc)
            goto error;

        vreg->rdev = regulator_register(rdesc, &pdev->dev,
                	&vreg->pdata->init_data, vreg);

        if (IS_ERR(vreg->rdev))
		{
			pr_err("Max77663> regulator register err.");
            rc = PTR_ERR(vreg->rdev);
		}
    } 
	else 
	{
        	rc = -ENODEV;
    }

error:
    if (rc) 
    {
        pr_info("Max77663> %s: id=%d, name=%s, rc=%d\n", __func__, pdev->id, 
                reg_name, rc);
    }

    return rc;
}

static int max77663_regulator_remove(struct platform_device *pdev)
{
   	struct regulator_dev *rdev = platform_get_drvdata(pdev);

    regulator_unregister(rdev);
    return 0;
}

static struct platform_driver max77663_regulator_driver =
{
	.probe = max77663_regulator_probe,
	.remove = __devexit_p(max77663_regulator_remove),
	.driver = {
		.name = "max77663-regulator",
		.owner = THIS_MODULE,
	},
};

static int __init max77663_regulator_init(void)
{
	pr_info("Max77663> regulator init\n");

	return platform_driver_register(&max77663_regulator_driver);
}
subsys_initcall(max77663_regulator_init);

static void __exit max77663_reg_exit(void)
{
    platform_driver_unregister(&max77663_regulator_driver);
}
module_exit(max77663_reg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("max77663 regulator driver");
MODULE_VERSION("1.0");

