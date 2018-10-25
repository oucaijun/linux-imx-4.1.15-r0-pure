/*
 * RTC client/driver for the Maxim/Dallas DS3231 Real-Time Clock over I2C
 *
 * Copyright (C) 2009-2011 Freescale Semiconductor.
 * Author: Jack Lan <jack.lan@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
/*
 * It would be more efficient to use i2c msgs/i2c_transfer directly but, as
 * recommened in .../Documentation/i2c/writing-clients section
 * "Sending and receiving", using SMBus level communication is preferred.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/rtc/ds3231.h>

/*
 * We can't determine type by probing, but if we expect pre-Linux code
 * to have set the chip up as a clock (turning on the oscillator and
 * setting the date and time), Linux can ignore the non-clock features.
 * That's a natural job for a factory or repair bench.
 */
enum ds_type {
	ds_3231,
	last_ds_type /* always last */
	/* rs5c372 too?  different address... */
};

/* RTC registers don't differ much, except for the century flag */
#define DS3231_REG_SECS		0x00	/* 00-59 */
#	define DS3231_BIT_CH		0x80
#define DS3231_REG_MIN		0x01	/* 00-59 */
#define DS3231_REG_HOUR		0x02	/* 00-23, or 1-12{am,pm} */
#	define DS3231_BIT_12HR		0x40	/* in REG_HOUR */
#	define DS3231_BIT_PM		0x20	/* in REG_HOUR */
#define DS3231_REG_WDAY		0x03	/* 01-07 */
#define DS3231_REG_MDAY		0x04	/* 01-31 */
#define DS3231_REG_MONTH	0x05	/* 01-12 */
#	define DS3231_BIT_CENTURY	0x80	/* in REG_MONTH */
#define DS3231_REG_YEAR		0x06	/* 00-99 */

/*
 * Other registers (control, status, alarms, trickle charge, NVRAM, etc)
 * start at 7, and they differ a LOT. Only control and status matter for
 * basic RTC date and time functionality; be careful using them.
 */
#	define DS3231_BIT_RS0		0x01
#define DS3231_REG_CONTROL	0x0e
#	define DS3231_BIT_nEOSC		0x80
#	define DS3231_BIT_BBSQW		0x40 /* same as BBSQI */
#	define DS3231_BIT_RS2		0x10
#	define DS3231_BIT_RS1		0x08
#	define DS3231_BIT_INTCN		0x04
#	define DS3231_BIT_A2IE		0x02
#	define DS3231_BIT_A1IE		0x01
#define DS3231_REG_STATUS	0x0f
#	define DS3231_BIT_OSF		0x80
#	define DS3231_BIT_A2I		0x02
#	define DS3231_BIT_A1I		0x01
#define DS3231_REG_ALARM1_SECS	0x07
#define DS3231_TRICKLE_CHARGER_MAGIC	0xa0

struct ds3231 {
	u8			offset; /* register's offset */
	u8			regs[11];
	u16			nvram_offset;
	struct bin_attribute	*nvram;
	enum ds_type		type;
	unsigned long		flags;
#define HAS_NVRAM	0		/* bit 0 == sysfs file active */
#define HAS_ALARM	1		/* bit 1 == irq claimed */
	struct i2c_client	*client;
	struct rtc_device	*rtc;
	struct work_struct	work;
	s32 (*read_block_data)(const struct i2c_client *client, u8 command,
			       u8 length, u8 *values);
	s32 (*write_block_data)(const struct i2c_client *client, u8 command,
				u8 length, const u8 *values);
};

struct chip_desc {
	unsigned		alarm:1;
	u16			nvram_offset;
	u16			nvram_size;
	u16			trickle_charger_reg;
};

static const struct chip_desc chips[last_ds_type] = {
	[ds_3231] = {
		.alarm		= 1,
	},
};

static const struct i2c_device_id ds3231_id[] = {
	{ "ds3231", ds_3231 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds3231_id);

/*----------------------------------------------------------------------*/

#define BLOCK_DATA_MAX_TRIES 10

static s32 ds3231_read_block_data_once(const struct i2c_client *client,
				       u8 command, u8 length, u8 *values)
{
	s32 i, data;

	for (i = 0; i < length; i++) {
		data = i2c_smbus_read_byte_data(client, command + i);
		if (data < 0)
			return data;
		values[i] = data;
	}
	return i;
}

static s32 ds3231_read_block_data(const struct i2c_client *client, u8 command,
				  u8 length, u8 *values)
{
	u8 oldvalues[255];
	s32 ret;
	int tries = 0;

	dev_dbg(&client->dev, "ds3231_read_block_data (length=%d)\n", length);
	ret = ds3231_read_block_data_once(client, command, length, values);
	if (ret < 0)
		return ret;
	do {
		if (++tries > BLOCK_DATA_MAX_TRIES) {
			dev_err(&client->dev,
				"ds3231_read_block_data failed\n");
			return -EIO;
		}
		memcpy(oldvalues, values, length);
		ret = ds3231_read_block_data_once(client, command, length,
						  values);
		if (ret < 0)
			return ret;
	} while (memcmp(oldvalues, values, length));
	return length;
}

static s32 ds3231_write_block_data(const struct i2c_client *client, u8 command,
				   u8 length, const u8 *values)
{
	u8 currvalues[255];
	int tries = 0;

	dev_dbg(&client->dev, "ds3231_write_block_data (length=%d)\n", length);
	do {
		s32 i, ret;

		if (++tries > BLOCK_DATA_MAX_TRIES) {
			dev_err(&client->dev,
				"ds3231_write_block_data failed\n");
			return -EIO;
		}
		for (i = 0; i < length; i++) {
			ret = i2c_smbus_write_byte_data(client, command + i,
							values[i]);
			if (ret < 0)
				return ret;
		}
		ret = ds3231_read_block_data_once(client, command, length,
						  currvalues);
		if (ret < 0)
			return ret;
	} while (memcmp(currvalues, values, length));
	return length;
}

/*----------------------------------------------------------------------*/

/* These RTC devices are not designed to be connected to a SMbus adapter.
   SMbus limits block operations length to 32 bytes, whereas it's not
   limited on I2C buses. As a result, accesses may exceed 32 bytes;
   in that case, split them into smaller blocks */

static s32 ds3231_native_smbus_write_block_data(const struct i2c_client *client,
				u8 command, u8 length, const u8 *values)
{
	u8 suboffset = 0;

	if (length <= I2C_SMBUS_BLOCK_MAX)
		return i2c_smbus_write_i2c_block_data(client,
					command, length, values);

	while (suboffset < length) {
		s32 retval = i2c_smbus_write_i2c_block_data(client,
				command + suboffset,
				min(I2C_SMBUS_BLOCK_MAX, length - suboffset),
				values + suboffset);
		if (retval < 0)
			return retval;

		suboffset += I2C_SMBUS_BLOCK_MAX;
	}
	return length;
}

static s32 ds3231_native_smbus_read_block_data(const struct i2c_client *client,
				u8 command, u8 length, u8 *values)
{
	u8 suboffset = 0;

	if (length <= I2C_SMBUS_BLOCK_MAX)
		return i2c_smbus_read_i2c_block_data(client,
					command, length, values);

	while (suboffset < length) {
		s32 retval = i2c_smbus_read_i2c_block_data(client,
				command + suboffset,
				min(I2C_SMBUS_BLOCK_MAX, length - suboffset),
				values + suboffset);
		if (retval < 0)
			return retval;

		suboffset += I2C_SMBUS_BLOCK_MAX;
	}
	return length;
}

/*----------------------------------------------------------------------*/

/*
 * The IRQ logic includes a "real" handler running in IRQ context just
 * long enough to schedule this workqueue entry.   We need a task context
 * to talk to the RTC, since I2C I/O calls require that; and disable the
 * IRQ until we clear its status on the chip, so that this handler can
 * work with any type of triggering (not just falling edge).
 *
 * The ds1337 and ds1339 both have two alarms, but we only use the first
 * one (with a "seconds" field).  For ds1337 we expect nINTA is our alarm
 * signal; ds1339 chips have only one alarm signal.
 */
static void ds3231_work(struct work_struct *work)
{
	struct ds3231		*ds3231;
	struct i2c_client	*client;
	struct mutex		*lock;
	int			stat, control;

	ds3231 = container_of(work, struct ds3231, work);
	client = ds3231->client;
	lock = &ds3231->rtc->ops_lock;

	mutex_lock(lock);
	stat = i2c_smbus_read_byte_data(client, DS3231_REG_STATUS);
	if (stat < 0)
		goto out;

	if (stat & DS3231_BIT_A1I) {
		stat &= ~DS3231_BIT_A1I;
		i2c_smbus_write_byte_data(client, DS3231_REG_STATUS, stat);

		control = i2c_smbus_read_byte_data(client, DS3231_REG_CONTROL);
		if (control < 0)
			goto out;

		control &= ~DS3231_BIT_A1IE;
		i2c_smbus_write_byte_data(client, DS3231_REG_CONTROL, control);

		rtc_update_irq(ds3231->rtc, 1, RTC_AF | RTC_IRQF);
	}

out:
	if (test_bit(HAS_ALARM, &ds3231->flags))
		enable_irq(client->irq);
	mutex_unlock(lock);
}

static irqreturn_t ds3231_irq(int irq, void *dev_id)
{
	struct i2c_client	*client = dev_id;
	struct ds3231		*ds3231 = i2c_get_clientdata(client);

	disable_irq_nosync(irq);
	schedule_work(&ds3231->work);
	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------*/

static int ds3231_get_time(struct device *dev, struct rtc_time *t)
{
	struct ds3231	*ds3231 = dev_get_drvdata(dev);
	int		tmp;

	/* read the RTC date and time registers all at once */
	tmp = ds3231->read_block_data(ds3231->client,
		ds3231->offset, 7, ds3231->regs);
	if (tmp != 7) {
		dev_err(dev, "%s error %d\n", "read", tmp);
		return -EIO;
	}

	dev_dbg(dev, "%s: %7ph\n", "read", ds3231->regs);

	t->tm_sec = bcd2bin(ds3231->regs[DS3231_REG_SECS] & 0x7f);
	t->tm_min = bcd2bin(ds3231->regs[DS3231_REG_MIN] & 0x7f);
	tmp = ds3231->regs[DS3231_REG_HOUR] & 0x3f;
	t->tm_hour = bcd2bin(tmp);
	t->tm_wday = bcd2bin(ds3231->regs[DS3231_REG_WDAY] & 0x07) - 1;
	t->tm_mday = bcd2bin(ds3231->regs[DS3231_REG_MDAY] & 0x3f);
	tmp = ds3231->regs[DS3231_REG_MONTH] & 0x1f;
	t->tm_mon = bcd2bin(tmp) - 1;

	/* assume 20YY not 19YY, and ignore DS3231_BIT_CENTURY */
	t->tm_year = bcd2bin(ds3231->regs[DS3231_REG_YEAR]) + 100;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"read", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	/* initial clock setting can be undefined */
	return rtc_valid_tm(t);
}

static int ds3231_set_time(struct device *dev, struct rtc_time *t)
{
	struct ds3231	*ds3231 = dev_get_drvdata(dev);
	int		result;
	int		tmp;
	u8		*buf = ds3231->regs;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"write", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	buf[DS3231_REG_SECS] = bin2bcd(t->tm_sec);
	buf[DS3231_REG_MIN] = bin2bcd(t->tm_min);
	buf[DS3231_REG_HOUR] = bin2bcd(t->tm_hour);
	buf[DS3231_REG_WDAY] = bin2bcd(t->tm_wday + 1);
	buf[DS3231_REG_MDAY] = bin2bcd(t->tm_mday);
	buf[DS3231_REG_MONTH] = bin2bcd(t->tm_mon + 1);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	buf[DS3231_REG_YEAR] = bin2bcd(tmp);

	switch (ds3231->type) {
	case ds_3231:
		buf[DS3231_REG_MONTH] |= DS3231_BIT_CENTURY;
		break;
	default:
		break;
	}

	dev_dbg(dev, "%s: %7ph\n", "write", buf);

	result = ds3231->write_block_data(ds3231->client,
		ds3231->offset, 7, buf);
	if (result < 0) {
		dev_err(dev, "%s error %d\n", "write", result);
		return result;
	}
	return 0;
}

static int ds1337_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client       *client = to_i2c_client(dev);
	struct ds3231		*ds3231 = i2c_get_clientdata(client);
	int			ret;

	if (!test_bit(HAS_ALARM, &ds3231->flags))
		return -EINVAL;

	/* read all ALARM1, ALARM2, and status registers at once */
	ret = ds3231->read_block_data(client,
			DS3231_REG_ALARM1_SECS, 9, ds3231->regs);
	if (ret != 9) {
		dev_err(dev, "%s error %d\n", "alarm read", ret);
		return -EIO;
	}

	dev_dbg(dev, "%s: %02x %02x %02x %02x, %02x %02x %02x, %02x %02x\n",
			"alarm read",
			ds3231->regs[0], ds3231->regs[1],
			ds3231->regs[2], ds3231->regs[3],
			ds3231->regs[4], ds3231->regs[5],
			ds3231->regs[6], ds3231->regs[7],
			ds3231->regs[8]);

	/*
	 * report alarm time (ALARM1); assume 24 hour and day-of-month modes,
	 * and that all four fields are checked matches
	 */
	t->time.tm_sec = bcd2bin(ds3231->regs[0] & 0x7f);
	t->time.tm_min = bcd2bin(ds3231->regs[1] & 0x7f);
	t->time.tm_hour = bcd2bin(ds3231->regs[2] & 0x3f);
	t->time.tm_mday = bcd2bin(ds3231->regs[3] & 0x3f);
	t->time.tm_mon = -1;
	t->time.tm_year = -1;
	t->time.tm_wday = -1;
	t->time.tm_yday = -1;
	t->time.tm_isdst = -1;

	/* ... and status */
	t->enabled = !!(ds3231->regs[7] & DS3231_BIT_A1IE);
	t->pending = !!(ds3231->regs[8] & DS3231_BIT_A1I);

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, enabled=%d, pending=%d\n",
		"alarm read", t->time.tm_sec, t->time.tm_min,
		t->time.tm_hour, t->time.tm_mday,
		t->enabled, t->pending);

	return 0;
}

static int ds1337_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client	*client = to_i2c_client(dev);
	struct ds3231		*ds3231 = i2c_get_clientdata(client);
	unsigned char		*buf = ds3231->regs;
	u8			control, status;
	int			ret;

	if (!test_bit(HAS_ALARM, &ds3231->flags))
		return -EINVAL;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, enabled=%d, pending=%d\n",
		"alarm set", t->time.tm_sec, t->time.tm_min,
		t->time.tm_hour, t->time.tm_mday,
		t->enabled, t->pending);

	/* read current status of both alarms and the chip */
	ret = ds3231->read_block_data(client,
			DS3231_REG_ALARM1_SECS, 9, buf);
	if (ret != 9) {
		dev_err(dev, "%s error %d\n", "alarm write", ret);
		return -EIO;
	}
	control = ds3231->regs[7];
	status = ds3231->regs[8];

	dev_dbg(dev, "%s: %02x %02x %02x %02x, %02x %02x %02x, %02x %02x\n",
			"alarm set (old status)",
			ds3231->regs[0], ds3231->regs[1],
			ds3231->regs[2], ds3231->regs[3],
			ds3231->regs[4], ds3231->regs[5],
			ds3231->regs[6], control, status);

	/* set ALARM1, using 24 hour and day-of-month modes */
	buf[0] = bin2bcd(t->time.tm_sec);
	buf[1] = bin2bcd(t->time.tm_min);
	buf[2] = bin2bcd(t->time.tm_hour);
	buf[3] = bin2bcd(t->time.tm_mday);

	/* set ALARM2 to non-garbage */
	buf[4] = 0;
	buf[5] = 0;
	buf[6] = 0;

	/* optionally enable ALARM1 */
	buf[7] = control & ~(DS3231_BIT_A1IE | DS3231_BIT_A2IE);
	if (t->enabled) {
		dev_dbg(dev, "alarm IRQ armed\n");
		buf[7] |= DS3231_BIT_A1IE;	/* only ALARM1 is used */
	}
	buf[8] = status & ~(DS3231_BIT_A1I | DS3231_BIT_A2I);

	ret = ds3231->write_block_data(client,
			DS3231_REG_ALARM1_SECS, 9, buf);
	if (ret < 0) {
		dev_err(dev, "can't set alarm time\n");
		return ret;
	}

	return 0;
}

static int ds3231_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client	*client = to_i2c_client(dev);
	struct ds3231		*ds3231 = i2c_get_clientdata(client);
	int			ret;

	if (!test_bit(HAS_ALARM, &ds3231->flags))
		return -ENOTTY;

	ret = i2c_smbus_read_byte_data(client, DS3231_REG_CONTROL);
	if (ret < 0)
		return ret;

	if (enabled)
		ret |= DS3231_BIT_A1IE;
	else
		ret &= ~DS3231_BIT_A1IE;

	ret = i2c_smbus_write_byte_data(client, DS3231_REG_CONTROL, ret);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct rtc_class_ops ds13xx_rtc_ops = {
	.read_time	= ds3231_get_time,
	.set_time	= ds3231_set_time,
	.read_alarm	= ds1337_read_alarm,
	.set_alarm	= ds1337_set_alarm,
	.alarm_irq_enable = ds3231_alarm_irq_enable,
};

/*----------------------------------------------------------------------*/

static ssize_t ds3231_nvram_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct i2c_client	*client;
	struct ds3231		*ds3231;
	int			result;

	client = kobj_to_i2c_client(kobj);
	ds3231 = i2c_get_clientdata(client);

	if (unlikely(off >= ds3231->nvram->size))
		return 0;
	if ((off + count) > ds3231->nvram->size)
		count = ds3231->nvram->size - off;
	if (unlikely(!count))
		return count;

	result = ds3231->read_block_data(client, ds3231->nvram_offset + off,
								count, buf);
	if (result < 0)
		dev_err(&client->dev, "%s error %d\n", "nvram read", result);
	return result;
}

static ssize_t ds3231_nvram_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct i2c_client	*client;
	struct ds3231		*ds3231;
	int			result;

	client = kobj_to_i2c_client(kobj);
	ds3231 = i2c_get_clientdata(client);

	if (unlikely(off >= ds3231->nvram->size))
		return -EFBIG;
	if ((off + count) > ds3231->nvram->size)
		count = ds3231->nvram->size - off;
	if (unlikely(!count))
		return count;

	result = ds3231->write_block_data(client, ds3231->nvram_offset + off,
								count, buf);
	if (result < 0) {
		dev_err(&client->dev, "%s error %d\n", "nvram write", result);
		return result;
	}
	return count;
}

/*----------------------------------------------------------------------*/

static int ds3231_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ds3231		*ds3231;
	int			err = -ENODEV;
	int			tmp;
	const struct chip_desc	*chip = &chips[id->driver_data];
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	bool			want_irq = false;
	unsigned char		*buf;
	struct ds3231_platform_data *pdata = dev_get_platdata(&client->dev);
	static const int	bbsqi_bitpos[] = {
		[ds_3231] = DS3231_BIT_BBSQW,
	};

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)
	    && !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	ds3231 = devm_kzalloc(&client->dev, sizeof(struct ds3231), GFP_KERNEL);
	if (!ds3231)
		return -ENOMEM;

	i2c_set_clientdata(client, ds3231);

	ds3231->client	= client;
	ds3231->type	= id->driver_data;

	if (pdata && pdata->trickle_charger_setup && chip->trickle_charger_reg)
		i2c_smbus_write_byte_data(client, chip->trickle_charger_reg,
			DS3231_TRICKLE_CHARGER_MAGIC | pdata->trickle_charger_setup);

	buf = ds3231->regs;
	if (i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		ds3231->read_block_data = ds3231_native_smbus_read_block_data;
		ds3231->write_block_data = ds3231_native_smbus_write_block_data;
	} else {
		ds3231->read_block_data = ds3231_read_block_data;
		ds3231->write_block_data = ds3231_write_block_data;
	}

	switch (ds3231->type) {
	case ds_3231:
		/* get registers that the "rtc" read below won't read... */
		tmp = ds3231->read_block_data(ds3231->client,
				DS3231_REG_CONTROL, 2, buf);
		if (tmp != 2) {
			dev_dbg(&client->dev, "read error %d\n", tmp);
			err = -EIO;
			goto exit;
		}

		/* oscillator off?  turn it on, so clock can tick. */
		if (ds3231->regs[0] & DS3231_BIT_nEOSC)
			ds3231->regs[0] &= ~DS3231_BIT_nEOSC;

		/*
		 * Using IRQ?  Disable the square wave and both alarms.
		 * For some variants, be sure alarms can trigger when we're
		 * running on Vbackup (BBSQI/BBSQW)
		 */
		if (ds3231->client->irq > 0 && chip->alarm) {
			INIT_WORK(&ds3231->work, ds3231_work);

			ds3231->regs[0] |= DS3231_BIT_INTCN
					| bbsqi_bitpos[ds3231->type];
			ds3231->regs[0] &= ~(DS3231_BIT_A2IE | DS3231_BIT_A1IE);

			want_irq = true;
		}

		i2c_smbus_write_byte_data(client, DS3231_REG_CONTROL,
							ds3231->regs[0]);

		/* oscillator fault?  clear flag, and warn */
		if (ds3231->regs[1] & DS3231_BIT_OSF) {
			i2c_smbus_write_byte_data(client, DS3231_REG_STATUS,
				ds3231->regs[1] & ~DS3231_BIT_OSF);
			dev_warn(&client->dev, "SET TIME!\n");
		}
		break;
	default:
		break;
	}

	/* read RTC registers */
	tmp = ds3231->read_block_data(ds3231->client, ds3231->offset, 8, buf);
	if (tmp != 8) {
		dev_dbg(&client->dev, "read error %d\n", tmp);
		err = -EIO;
		goto exit;
	}

	tmp = ds3231->regs[DS3231_REG_HOUR];
	switch (ds3231->type) {
	default:
		if (!(tmp & DS3231_BIT_12HR))
			break;

		/*
		 * Be sure we're in 24 hour mode.  Multi-master systems
		 * take note...
		 */
		tmp = bcd2bin(tmp & 0x1f);
		if (tmp == 12)
			tmp = 0;
		if (ds3231->regs[DS3231_REG_HOUR] & DS3231_BIT_PM)
			tmp += 12;
		i2c_smbus_write_byte_data(client,
				ds3231->offset + DS3231_REG_HOUR,
				bin2bcd(tmp));
	}

	ds3231->rtc = devm_rtc_device_register(&client->dev, client->name,
				&ds13xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(ds3231->rtc)) {
		err = PTR_ERR(ds3231->rtc);
		dev_err(&client->dev,
			"unable to register the class device\n");
		goto exit;
	}

	if (want_irq) {
		err = request_irq(client->irq, ds3231_irq, IRQF_SHARED,
			  ds3231->rtc->name, client);
		if (err) {
			dev_err(&client->dev,
				"unable to request IRQ!\n");
			goto exit;
		}

		device_set_wakeup_capable(&client->dev, 1);
		set_bit(HAS_ALARM, &ds3231->flags);
		dev_dbg(&client->dev, "got IRQ %d\n", client->irq);
	}

	if (chip->nvram_size) {
		ds3231->nvram = devm_kzalloc(&client->dev,
					sizeof(struct bin_attribute),
					GFP_KERNEL);
		if (!ds3231->nvram) {
			err = -ENOMEM;
			goto err_irq;
		}
		ds3231->nvram->attr.name = "nvram";
		ds3231->nvram->attr.mode = S_IRUGO | S_IWUSR;
		sysfs_bin_attr_init(ds3231->nvram);
		ds3231->nvram->read = ds3231_nvram_read;
		ds3231->nvram->write = ds3231_nvram_write;
		ds3231->nvram->size = chip->nvram_size;
		ds3231->nvram_offset = chip->nvram_offset;
		err = sysfs_create_bin_file(&client->dev.kobj, ds3231->nvram);
		if (err)
			goto err_irq;
		set_bit(HAS_NVRAM, &ds3231->flags);
		dev_info(&client->dev, "%zu bytes nvram\n", ds3231->nvram->size);
	}

	return 0;

err_irq:
	free_irq(client->irq, client);
exit:
	return err;
}

static int ds3231_remove(struct i2c_client *client)
{
	struct ds3231 *ds3231 = i2c_get_clientdata(client);

	if (test_and_clear_bit(HAS_ALARM, &ds3231->flags)) {
		free_irq(client->irq, client);
		cancel_work_sync(&ds3231->work);
	}

	if (test_and_clear_bit(HAS_NVRAM, &ds3231->flags))
		sysfs_remove_bin_file(&client->dev.kobj, ds3231->nvram);

	return 0;
}

static struct i2c_driver ds3231_driver = {
	.driver = {
		.name	= "rtc-ds3231",
		.owner	= THIS_MODULE,
	},
	.probe		= ds3231_probe,
	.remove		= ds3231_remove,
	.id_table	= ds3231_id,
};

module_i2c_driver(ds3231_driver);

MODULE_DESCRIPTION("RTC driver for DS3231 and similar chips");
MODULE_LICENSE("GPL");

