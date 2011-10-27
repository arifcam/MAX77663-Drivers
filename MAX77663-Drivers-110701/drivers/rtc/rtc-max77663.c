/* 
 * rtc-max77663.c 
 * max77663 RTC driver
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/rtc/rtc-max77663.h>
#include <linux/mfd/max77663-core.h>

enum {
    MAX77663_RTCINT_RTC60S = 0,
    MAX77663_RTCINT_RTCA1,
    MAX77663_RTCINT_RTCA2,
    MAX77663_RTCINT_SMPL,
    MAX77663_RTCINT_RTC1S,
};

static unsigned char reg_update0_shadow = MAX77663_FCUR | MAX77663_RTCWAKE;

#define MAX77663_RTC_RETRY_LIMIT	20
#define MAX77663_YEAR_BASE  100

static int max77663_rtc_i2c_write(struct i2c_client *i2c, unsigned char addr, 
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

static int max77663_rtc_i2c_read(struct i2c_client *i2c, unsigned char addr, 
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

static int max77663_rtc_i2c_set_bits(struct i2c_client *i2c, u8 addr, u8 mask, u8 value)
{
	u8 tmp;
	int ret;

	ret = max77663_rtc_i2c_read(i2c, addr, &tmp, 1);
	if (ret == 0)
	{
		value = (tmp & ~mask) | (value & mask);
		ret = max77663_rtc_i2c_write(i2c, addr, &value, 1);
	}
    
	return ret;
}

static int max77663_rtc_read(struct max77663_rtc_chip *chip, u8 addr, u8 *values, unsigned int len)
{
	int ret;

	if (chip == NULL)
		return -EINVAL;

	mutex_lock(&chip->io_lock);

	ret = max77663_rtc_i2c_read(chip->client, addr, values, len);

	mutex_unlock(&chip->io_lock);

	return ret;
}

static int max77663_rtc_write(struct max77663_rtc_chip *chip, u8 addr, u8 *values, unsigned int len)
{
	int ret;

	if (chip == NULL)
		return -EINVAL;

	mutex_lock(&chip->io_lock);

	ret = max77663_rtc_i2c_write(chip->client, addr, values, len);

	mutex_unlock(&chip->io_lock);

	return ret;
}

static int max77663_rtc_set_bits(struct max77663_rtc_chip *chip, u8 addr, u8 mask, u8 value)
{
    int ret;
    
    if (chip == NULL)
        return -EINVAL;

    mutex_lock(&chip->io_lock);

    ret = max77663_rtc_i2c_set_bits(chip->client, addr, mask, value);
    
    mutex_unlock(&chip->io_lock);
    
    return ret;
}

static int max77663_rtc_format_register2time(unsigned char *reg, 
	struct rtc_time *tm)
{
	unsigned char wkday = reg[RTC_WEEKDAY] & 0x7F; 	/* 6:0 */

	tm->tm_year = (int)(reg[RTC_YEAR]) + MAX77663_YEAR_BASE; 	/* 7:0 */
	tm->tm_mon = (int)(reg[RTC_MONTH] & 0x1F) - 1; 	/* 4:0 */
	tm->tm_mday = (int)(reg[RTC_DATE] & 0x3F);	/* 5:0 */
	tm->tm_hour = (int)(reg[RTC_HOUR] & 0x3F);	/* 5:0 */
	tm->tm_min = (int)(reg[RTC_MIN] & 0x7F);	/* 6:0 */
	tm->tm_sec = (int)(reg[RTC_SEC] & 0x7F);	/* 6:0 */

	if (wkday == 0) {
		pr_err("Max77663> %s: RTC_WEEKDAY error.\n", __func__);
		return -EINVAL;
	}
	else {
		tm->tm_wday = 0;
		while ((wkday & 0x01) != 0x01) {
			tm->tm_wday++;
			wkday >>= 1;
		}
	}

	return 0;
}

static int max77663_rtc_format_time2register(struct rtc_time *tm, 
	unsigned char *reg, int alarm)
{
	unsigned char alarm_bit = alarm ? 0x80 : 0x00;

	if (tm->tm_year < MAX77663_YEAR_BASE || tm->tm_year > MAX77663_YEAR_BASE+99) {
		pr_err("%s: invalid year(%d)\n", __func__, tm->tm_year);
		return -EINVAL;
	}

    reg[RTC_YEAR] = (unsigned char)(tm->tm_year - MAX77663_YEAR_BASE) | alarm_bit;
	reg[RTC_MONTH] = (unsigned char)(tm->tm_mon + 1) | alarm_bit;
	reg[RTC_DATE] = (unsigned char)tm->tm_mday | alarm_bit;
	reg[RTC_HOUR] = (unsigned char)tm->tm_hour | alarm_bit;
	reg[RTC_MIN] = (unsigned char)tm->tm_min | alarm_bit;
	reg[RTC_SEC] = (unsigned char)tm->tm_sec | alarm_bit;
	reg[RTC_WEEKDAY] = (unsigned char)(1 << tm->tm_wday) | alarm_bit;

	return 0;
}

static int max77663_rtc_sync_read_buffer(struct max77663_rtc_chip *rtc_chip)
{
	unsigned char tmp;
	int ret;
	int retry = 0;

	tmp = reg_update0_shadow | MAX77663_RBUDR;
	ret = max77663_rtc_write(rtc_chip, MAX77663_RTCUPDATE0, &tmp, 1);
	if (ret != 0) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		goto out;
	}

	/* There is an issue waiting for RBUDF=1. 
         * It is being investigated.
         */
	while (1)
	{
		ret = max77663_rtc_read(rtc_chip, MAX77663_RTCUPDATE1, &tmp, 1);
		if (ret) {
			pr_err("Max77663> %s: Err: %d\n", __func__, ret);
			goto out;
		}

		if (tmp & MAX77663_RBUDF)
			break;

		if(retry++ > MAX77663_RTC_RETRY_LIMIT) {
			pr_err("%s: retry failed!\n", __func__);
			goto out;
		}

		mdelay(1);
	}

out:
	return ret;
}

static int max77663_rtc_commit_write_buffer(struct max77663_rtc_chip *rtc_chip)
{
	unsigned char tmp;
	int ret;
	int retry = 0;

	tmp = reg_update0_shadow | MAX77663_UDR;
	ret = max77663_rtc_write(rtc_chip, MAX77663_RTCUPDATE0, &tmp, 1);
	if (ret != 0) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		goto out;
	}

	/* There is an issue waiting for UDF=1. 
         * It is being investigated.
         */
	while (1)
	{
		ret = max77663_rtc_read(rtc_chip, MAX77663_RTCUPDATE1, &tmp, 1);
		if (ret) {
			pr_err("Max77663> %s: Err: %d\n", __func__, ret);
			goto out;
		}

		if (tmp & MAX77663_UDF)
			break;

		if(retry++ > MAX77663_RTC_RETRY_LIMIT) {
			pr_err("%s: retry failed!\n", __func__);
			goto out;
		}

		mdelay(1);
	}

out:
	return ret;
}

static int max77663_rtc_read_reg(struct max77663_rtc_chip *rtc_chip, 
	unsigned char reg, unsigned char count, unsigned char *buf)
{
	int ret = 0;

	pr_err("Max77663> %s: Entry.\n", __func__);

	ret = max77663_rtc_sync_read_buffer(rtc_chip);

	ret = max77663_rtc_read(rtc_chip, reg, buf, count);

	pr_err("Max77663> %s: Exit.\n", __func__);

	return ret; 
}

static int max77663_rtc_write_reg(struct max77663_rtc_chip *rtc_chip, 
	unsigned char reg, unsigned char count, unsigned char *buf)
{	
	int ret = 0;

	pr_err("Max77663> %s: Entry.\n", __func__);

	ret = max77663_rtc_write(rtc_chip, reg, buf, count);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		goto out;
	}

	ret = max77663_rtc_commit_write_buffer(rtc_chip);

out:
	pr_err("Max77663> %s: Exit.\n", __func__);

	return ret;	
}

static irqreturn_t max77663_rtc_update_handler(int irq, void *drvdata)
{
	unsigned char irq_status;	
    int ret;
    struct max77663_rtc_chip *rtc_chip = (struct max77663_rtc_chip *)drvdata;

	ret = max77663_rtc_read(rtc_chip, MAX77663_RTCINT, &irq_status, 1);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
	    return ret;
	}

    if (irq_status & MAX77663_RTCA1)
        rtc_update_irq(rtc_chip->rtc, 1, RTC_IRQF | RTC_AF);

    if (irq_status & MAX77663_RTC1S)
        rtc_update_irq(rtc_chip->rtc, 1, RTC_IRQF | RTC_UF);

	return IRQ_HANDLED;
}


static int max77663_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct max77663_rtc_chip *rtc_chip = dev_get_drvdata(dev);
	unsigned char irq_bit = MAX77663_RTCA1M;
	unsigned char irq_state = enabled ? 0 : MAX77663_RTCA1M;

	return max77663_rtc_set_bits(rtc_chip, MAX77663_RTCINTM, irq_bit, irq_state);
}

static int max77663_rtc_update_irq_enable(struct device *dev, unsigned int enabled)
{
	struct max77663_rtc_chip *rtc_chip = dev_get_drvdata(dev);
	unsigned char irq_bit = MAX77663_RTC1SM;
	unsigned char irq_state = enabled ? 0 : MAX77663_RTC1SM;

	return max77663_rtc_set_bits(rtc_chip, MAX77663_RTCINTM, irq_bit, irq_state);
}

static int max77663_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct max77663_rtc_chip *rtc_chip = dev_get_drvdata(dev);
	unsigned char buf[RTC_BYTE_CNT];
	int ret;

	ret = max77663_rtc_read_reg(rtc_chip, MAX77663_RTCSEC, RTC_BYTE_CNT, buf);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
	    	return ret;
	}

	return max77663_rtc_format_register2time(buf, tm);
}

static int max77663_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct max77663_rtc_chip *rtc_chip = dev_get_drvdata(dev);
	unsigned char buf[RTC_BYTE_CNT];
	int ret;

	ret = max77663_rtc_format_time2register(tm, buf, 0);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		return ret;
	}

	return max77663_rtc_write_reg(rtc_chip, MAX77663_RTCSEC, RTC_BYTE_CNT, buf);
}

static int max77663_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct max77663_rtc_chip *rtc_chip = dev_get_drvdata(dev);
	unsigned char buf[RTC_BYTE_CNT];
	int ret;

	ret = max77663_rtc_read_reg(rtc_chip, MAX77663_RTCSECA1, RTC_BYTE_CNT, buf);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		return ret;
	}

	ret = max77663_rtc_format_register2time(buf, &alrm->time);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		return ret;
	}

    ret = max77663_rtc_read(rtc_chip, MAX77663_RTCINTM, buf, 1);

	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		return ret;
	}

	alrm->enabled = (buf[0] & MAX77663_RTCA1M) ? 0 : 1;

	return ret;
}

static int max77663_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct max77663_rtc_chip *rtc_chip = dev_get_drvdata(dev);
	unsigned char buf[RTC_BYTE_CNT];
	int ret;

	ret = max77663_rtc_format_time2register(&alrm->time, buf, 1);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		return ret;
	}

	ret = max77663_rtc_write_reg(rtc_chip, MAX77663_RTCSECA1, RTC_BYTE_CNT, buf);
	if (ret) {
		pr_err("Max77663> %s: Err: %d\n", __func__, ret);
		return ret;
	}

	return max77663_rtc_alarm_irq_enable(dev, alrm->enabled ? 1 : 0);
}

static int max77663_rtc_device_init(struct max77663_rtc_chip *rtc_chip)
{
	int ret = 0;
	unsigned char buf;

	/* BCD=binary, HRMODE=24hour mode */
	buf = MAX77663_BCD_BINARY | MAX77663_HRMODE_24H;
	ret = max77663_rtc_write(rtc_chip, MAX77663_RTCCNTL, &buf, 1);
	if (ret) {
		pr_err("%s: failed\n", __func__);
		goto out_init;
	}

    /* All interrupts are masked */
	buf = MAX77663_RTC60SM | MAX77663_RTCA1M | MAX77663_RTCA2M
		| MAX77663_SMPLM | MAX77663_RTC1SM;
    ret = max77663_rtc_write(rtc_chip, MAX77663_RTCINTM, &buf, 1);
	pr_err("Max77663> %s: Mask all interrupt. %x\n", __func__, rtc_chip->client->addr);
	if (ret) {
		pr_err("Max77663> %s: Init failed 2.\n", __func__);
		goto out_init;
	}

out_init:
	return ret;
}

static const struct rtc_class_ops max77663_rtc_ops = {
	.read_time	= max77663_rtc_read_time,
	.set_time	= max77663_rtc_set_time,
	.read_alarm	= max77663_rtc_read_alarm,
	.set_alarm	= max77663_rtc_set_alarm,
	.alarm_irq_enable = max77663_rtc_alarm_irq_enable,
	.update_irq_enable = max77663_rtc_update_irq_enable,
};
static int __devinit max77663_rtc_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
//static int __devinit max77663_rtc_probe(struct platform_device *pdev)
{
    struct max77663_rtc_platform_data *pdata = client->dev.platform_data;
    static struct max77663_rtc_chip *rtc_chip;
	int ret;
	int irq;

	pr_err("Max77663> %s: probe entry.\n", __func__);

    if (pdata == NULL) {
        pr_err("%s: No platform_data.\n", __func__);
        return -ENODEV;
    }

    if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
        pr_err("%s: i2c_check_functionality failed.\n", __func__);
        return -ENODEV;
    }

    rtc_chip = kzalloc(sizeof(struct max77663_rtc_chip), GFP_KERNEL);
    if (rtc_chip == NULL) {
        pr_err("%s: kzalloc() failed.\n", __func__);
        return -ENOMEM;
    }

	(void) memcpy((void *)&rtc_chip->pdata, (const void *)pdata,
		sizeof(rtc_chip->pdata));

    rtc_chip->client = client;
    rtc_chip->dev = &client->dev;
    rtc_chip->irq_base = pdata->irq_base;
	i2c_set_clientdata(client, rtc_chip);

    mutex_init(&rtc_chip->io_lock);

	max77663_rtc_device_init(rtc_chip);

	irq = rtc_chip->irq_base + MAX77663_IRQTOP_RTC;

	ret = request_threaded_irq(irq, NULL, max77663_rtc_update_handler,
                               IRQF_ONESHOT, "rtc-max77663-alarm", rtc_chip);

	if (ret < 0) {
		dev_err(&client->dev, "Max77663> Failed to request IRQ: #%d: Err: %d\n", 
                irq, ret);
		goto out_irq;
	}

	rtc_chip->rtc = rtc_device_register("max77663-rtc", &client->dev,
                                  &max77663_rtc_ops, THIS_MODULE);
	ret = PTR_ERR(rtc_chip->rtc);
	if (IS_ERR(rtc_chip->rtc)) {
		dev_err(&client->dev, "Max77663> ailed to register RTC device: %d\n", ret);
		goto out_rtc;
	}

	return 0;

out_rtc:
	free_irq(irq, rtc_chip->rtc);
out_irq:
    kfree(rtc_chip->rtc);
	return ret;
}

static int __devexit max77663_rtc_remove(struct i2c_client *client)
{
    struct max77663_rtc_chip *rtc_chip;

    rtc_chip = i2c_get_clientdata(client);
    if (rtc_chip) {
		free_irq(rtc_chip->irq_base + MAX77663_IRQTOP_RTC, rtc_chip);
        rtc_device_unregister(rtc_chip->rtc);
        kfree(rtc_chip);
    }
	
	return 0;
}

static const struct i2c_device_id max77663_rtc_ids[] = {
    { "max77663rtc", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, max77663_rtc_ids);

static struct i2c_driver max77663_rtc_driver = {
	.driver		= {
		.name	= "max77663-rtc",
		.owner	= THIS_MODULE,
	},
	.probe	= max77663_rtc_probe,
	.remove	= __devexit_p(max77663_rtc_remove),
    .id_table = max77663_rtc_ids,
};

static int __init max77663_rtc_init(void)
{
    int ret;

    ret = i2c_add_driver(&max77663_rtc_driver);
	if (ret != 0)
		pr_err("Failed to register MAX77663 RTC I2C driver: %d\n", ret);
	return ret;    
}
module_init(max77663_rtc_init);

static void __exit max77663_rtc_exit(void)
{
	i2c_del_driver(&max77663_rtc_driver);
}
module_exit(max77663_rtc_exit);

MODULE_DESCRIPTION("max77663 RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");

