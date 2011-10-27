/*
 * Max77663 mfd driver (I2C bus access)
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#include <linux/mfd/max77663-core.h>

#define MAX77663_DBG_MSG_ERR	0
#define MAX77663_DBG_MSG_HI	1
#define MAX77663_DBG_MSG_MED	2
#define MAX77663_DBG_MSG_LO	3

//static int dbg_msg_lvl = MAX77663_DBG_MSG_ERR;

#define MSG_LVL_HI  (dbg_msg_lvl >= MAX77663_DBG_MSG_HI) 
#define MSG_LVL_MED (dbg_msg_lvl >= MAX77663_DBG_MSG_MED) 
#define MSG_LVL_LO  (dbg_msg_lvl >= MAX77663_DBG_MSG_LO) 

static int max77663_i2c_write(struct i2c_client *i2c, unsigned char addr, 
	void *src, unsigned int bytes)
{
	unsigned char buf[bytes + 1];
	int ret;

	buf[0] = addr;
	memcpy(&buf[1], src, bytes);

	ret = i2c_master_send(i2c, buf, bytes + 1);
	if (ret < 0)
		return ret;
	return 0;
}

/* This is a temporary function for placeholder. 
 * i2c issues are being investigated.
 */
static int max77663_i2c_read(struct i2c_client *i2c, unsigned char addr, 
	unsigned char *dest, unsigned int bytes)
{
	int ret;

	if (bytes > 1)
	{
		ret = i2c_smbus_read_i2c_block_data(i2c, addr, bytes, dest);
	}
	else 
	{
		ret = i2c_smbus_read_byte_data(i2c, addr);

		if (ret < 0)
			return ret;

		*dest = (unsigned char)ret;
	}

	return ret;
}

int max77663_read(struct max77663_chip *chip, u8 addr, u8 *values,
        unsigned int len)
{
	int ret;

	if (chip == NULL)
		return -EINVAL;

	mutex_lock(&chip->io_lock);

	ret = max77663_i2c_read(chip->dev, addr, values, len);

	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(max77663_read);

int max77663_write(struct max77663_chip *chip, u8 addr, u8 *values,
	unsigned int len)
{
	int ret;

	if (chip == NULL)
		return -EINVAL;

	mutex_lock(&chip->io_lock);

	ret = max77663_i2c_write(chip->dev, addr, values, len);

	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(max77663_write);

int max77663_set_bits(struct max77663_chip *chip, u8 addr, u8 mask, u8 value)
{
	u8 tmp;
	int ret;
    
	if (chip == NULL)
		return -EINVAL;

	mutex_lock(&chip->io_lock);
    
	ret = max77663_i2c_read(chip->dev, addr, &tmp, 1);
	if (ret == 0)
	{
		value = (tmp & ~mask) | (value & mask);
		ret = max77663_i2c_write(chip->dev, addr, &value, 1);
	}
    
	mutex_unlock(&chip->io_lock);
    
	return ret;
}
EXPORT_SYMBOL(max77663_set_bits);

static int max77663_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int i, rc;
	struct  max77663_platform_data *pdata = client->dev.platform_data;
	struct  max77663_chip *chip;

	pr_err("Max77663> %s: ID=%s, Addr=%x\n", __func__, id->name, client->addr); 

	if (pdata == NULL || pdata->num_subdevs == 0) 
	{
		pr_err("Max77663> %s: Invalid platform_data.\n", __func__);
		return -ENODEV;
	}

	chip = kzalloc(sizeof(struct max77663_chip), GFP_KERNEL);
	if (chip == NULL) 
	{
		pr_err("Max77663> %s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	chip->dev = client;

	(void) memcpy((void *)&chip->pdata, (const void *)pdata,
		sizeof(chip->pdata));

	i2c_set_clientdata(client, chip);

	mutex_init(&chip->io_lock);

    max77663_device_init(chip, pdata);

	for (i = 0; i < pdata->num_subdevs; i++) 
	{
		pdata->sub_devices[i].driver_data = chip;
	}
	
	rc = mfd_add_devices(&chip->dev->dev, 0, pdata->sub_devices,
		pdata->num_subdevs, NULL, 0);

	pr_err("Max77663> %s: -\n", __func__);

	return 0;
}

static int __devexit max77663_remove(struct i2c_client *client)
{
	struct  max77663_chip *chip;

	chip = i2c_get_clientdata(client);
    max77663_device_exit(chip);
	mfd_remove_devices(&chip->dev->dev);
	if (chip) 
	{
		mutex_destroy(&chip->io_lock);
		chip->dev = NULL;

		kfree(chip);
	}

	return 0;
}

#ifdef CONFIG_PM
static int max77663_suspend(struct device *dev)
{
	struct i2c_client *client;
	struct  max77663_chip *chip;

	client = to_i2c_client(dev);
	chip = i2c_get_clientdata(client);

	return 0;
}

static int max77663_resume(struct device *dev)
{
	struct i2c_client *client;
	struct  max77663_chip *chip;

	client = to_i2c_client(dev);
	chip = i2c_get_clientdata(client);

	return 0;
}
#else
#define max77663_suspend      NULL
#define max77663_resume       NULL
#endif

static const struct i2c_device_id max77663_id[] = {
	{ "max77663", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max77663_id);

static struct dev_pm_ops max77663_pm = {
	.suspend = max77663_suspend,
	.resume = max77663_resume,
};

static struct i2c_driver max77663_driver = {
	.driver = {
		.name = "max77663",
		.owner = THIS_MODULE,
		.pm      = &max77663_pm,
	},
	.probe      	= max77663_probe,
	.remove     	= __devexit_p(max77663_remove),
	.id_table   	= max77663_id,
};

static int __init max77663_init(void)
{
	int rc = i2c_add_driver(&max77663_driver);
	pr_info("Max77663> %s: i2c add driver: rc = %d\n", __func__, rc);

	return rc;
}

static void __exit max77663_exit(void)
{
	i2c_del_driver(&max77663_driver);
}

arch_initcall(max77663_init);
module_exit(max77663_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77663 Multi Function Device Core Driver");
MODULE_VERSION("1.0");


