/* 
 * rtc-max.h
 * Maxim RTC common definition shared by multiple IC products
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __RTC_MAX77663_H__
#define __RTC_MAX77663_H__

enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_MONTH,
	RTC_YEAR,
	RTC_DATE,
	RTC_BYTE_CNT
};

/* Registers */
#define MAX77663_RTCINT		0x00
#define MAX77663_RTCINTM	0x01
#define MAX77663_RTCCNTLM	0x02
#define MAX77663_RTCCNTL	0x03
#define MAX77663_RTCUPDATE0	0x04
#define MAX77663_RTCUPDATE1	0x05
#define MAX77663_RTCSMPL	0x06
#define MAX77663_RTCSEC		0x07
#define MAX77663_RTCMIN		0x08
#define MAX77663_RTCHOUR	0x09
#define MAX77663_RTCDOW		0x0A
#define MAX77663_RTCMONTH	0x0B
#define MAX77663_RTCYEAR	0x0C
#define MAX77663_RTCDOM		0x0D
#define MAX77663_RTCSECA1	0x0E
#define MAX77663_RTCMINA1	0x0F
#define MAX77663_RTCHOURA1	0x10
#define MAX77663_RTCDOWA1	0x11
#define MAX77663_RTCMONTHA1	0x12
#define MAX77663_RTCYEARA1	0x13
#define MAX77663_RTCDOMA1	0x14
#define MAX77663_RTCSECA2	0x15
#define MAX77663_RTCMINA2	0x16
#define MAX77663_RTCHOURA2	0x17
#define MAX77663_RTCDOWA2	0x18
#define MAX77663_RTCMONTHA2	0x19
#define MAX77663_RTCYEARA2	0x1A
#define MAX77663_RTCDOMA2	0x1B

/* RTCINT */
#define MAX77663_RTC60S		(1 << 0)
#define MAX77663_RTCA1		(1 << 1)
#define MAX77663_RTCA2		(1 << 2)
#define MAX77663_SMPL		(1 << 3)
#define MAX77663_RTC1S		(1 << 4)

/* RTCINTM */
#define MAX77663_RTC60SM	(1 << 0)
#define MAX77663_RTCA1M		(1 << 1)
#define MAX77663_RTCA2M		(1 << 2)
#define MAX77663_SMPLM		(1 << 3)
#define MAX77663_RTC1SM		(1 << 4)

/* RTCCNTLM */
#define MAX77663_BCDM		(1 << 0)
#define MAX77663_HRMODEM	(1 << 1)

/* RTCCNTL */
#define MAX77663_BCD		(1 << 0)
#define MAX77663_BCD_BINARY 0
#define MAX77663_BCD_BCD    MAX77663_BCD
#define MAX77663_HRMODE		(1 << 1)
#define MAX77663_HRMODE_12H 0
#define MAX77663_HRMODE_24H MAX77663_HRMODE

/* RTCUPDATE0 */
#define MAX77663_UDR		(1 << 0)
#define MAX77663_FCUR		(1 << 1)
#define MAX77663_FREEZE_SEC	(1 << 2)
#define MAX77663_RTCWAKE	(1 << 3)
#define MAX77663_RBUDR		(1 << 4)

/* RTCUPDATE1 */
#define MAX77663_UDF		(1 << 0)
#define MAX77663_RBUDF		(1 << 1)


struct max77663_rtc_platform_data {
	int     		irq_base;
};

struct max77663_rtc_chip {
	struct max77663_rtc_platform_data	*pdata;
	struct i2c_client	*client;
	struct rtc_device	*rtc;
    struct device       *dev;
	struct mutex		io_lock;
    int irq_base;
};

#endif /* __RTC_MAX77663_H__ */

