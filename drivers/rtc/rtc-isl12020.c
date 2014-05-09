/*
 * Intersil ISL12020 rtc class driver
 *
 * Copyright 2005,2006 Hebert Valerio Riedel <hvr@gnu.org>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>

#define DRV_VERSION "0.4"

/* Register map */
/* rtc section */
#define ISL12020_REG_SC  0x00
#define ISL12020_REG_MN  0x01
#define ISL12020_REG_HR  0x02
#define ISL12020_REG_HR_MIL     (1<<7)	/* 24h/12h mode */
#define ISL12020_REG_HR_PM      (1<<5)	/* PM/AM bit in 12h mode */
#define ISL12020_REG_DT  0x03
#define ISL12020_REG_MO  0x04
#define ISL12020_REG_YR  0x05
#define ISL12020_REG_DW  0x06
#define ISL12020_RTC_SECTION_LEN 7

/* control/status section */
#define ISL12020_REG_SR  0x07
#define ISL12020_REG_SR_BUSY    (1<<7)  /* temperature sensing in progress */
#define ISL12020_REG_SR_OSCF    (1<<6)  /* crystal oscillator stopped */
#define ISL12020_REG_SR_DSTADJ  (1<<5)  /* daylight saving time change */
#define ISL12020_REG_SR_ALM     (1<<4)  /* alarm */
#define ISL12020_REG_SR_LVDD    (1<<3)  /* low VDD */
#define ISL12020_REG_SR_LBAT85  (1<<2)  /* battery 85% */
#define ISL12020_REG_SR_LBAT75  (1<<1)  /* battery 75% */
#define ISL12020_REG_SR_RTCF    (1<<0)  /* rtc fail */

#define ISL12020_REG_INT 0x08
#define ISL12020_REG_INT_ARST (1 << 7)
#define ISL12020_REG_INT_WRTC (1 << 6)
#define ISL12020_REG_INT_IM (1 << 5)
#define ISL12020_REG_INT_FOBATB (1 << 4)
#define ISL12020_REG_INT_FO3 (1 << 3)
#define ISL12020_REG_INT_FO2 (1 << 2)
#define ISL12020_REG_INT_FO1 (1 << 1)
#define ISL12020_REG_INT_FO0 (1 << 0)

#define ISL12020_REG_PWR_VDD 0x09
#define ISL12020_REG_PWR_VBAT 0x0a
#define ISL12020_REG_PWR_ITRO 0x0b
#define ISL12020_REG_ALPHA 0x0c

#define ISL12020_REG_BETA 0x0d
#define ISL12020_REG_BETA_TSE (1 << 7) /* temp sense enabled */
#define ISL12020_REG_BETA_BTSE (1 << 6) /* temp sense on battery */
#define ISL12020_REG_BETA_BTSR (1 << 5) /* temp sense freq */

#define ISL12020_REG_FATR 0x0e
#define ISL12020_REG_FDTR 0x0f

/* alarm section */
#define ISL12020_REG_SCA 0x10
#define ISL12020_REG_MNA 0x11
#define ISL12020_REG_HRA 0x12
#define ISL12020_REG_DTA 0x13
#define ISL12020_REG_MOA 0x14
#define ISL12020_REG_DWA 0x15
#define ISL12020_ALARM_SECTION_LEN 6

/* temperature section */
#define ISL12020_REG_TK0L 0x28
#define ISL12020_REG_TK0M 0x29
#define ISL12020_TEMP_SECTION_LEN 2

static struct i2c_driver isl12020_driver;

/* block read */
static int
isl12020_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[],
		      unsigned len)
{
	u8 reg_addr[1] = { reg };
	struct i2c_msg msgs[2] = {
		{client->addr, 0, sizeof(reg_addr), reg_addr}
		,
		{client->addr, I2C_M_RD, len, buf}
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret > 0)
		ret = 0;
	return ret;
}

/* block write */
static int
isl12020_i2c_set_regs(struct i2c_client *client, u8 const buf[])
{
	u8 i2c_buf[ISL12020_RTC_SECTION_LEN + 2];
	struct i2c_msg msgs[1] = {
		{client->addr, 0, ISL12020_RTC_SECTION_LEN + 1, i2c_buf}
	};
	int ret;

	i2c_buf[0] = ISL12020_REG_SC;
	memcpy(&i2c_buf[1], &buf[0], ISL12020_RTC_SECTION_LEN);

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	return ret;
}

/* block read */
static int
isl12020_i2c_read_alarm_regs(struct i2c_client *client,
		u8 buf[ISL12020_ALARM_SECTION_LEN])
{
	u8 reg_addr[1] = { ISL12020_REG_SCA };
	struct i2c_msg msgs[2] = {
		{client->addr, 0, sizeof(reg_addr), reg_addr}
		,
		{client->addr, I2C_M_RD, ISL12020_ALARM_SECTION_LEN, buf}
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret > 0)
		ret = 0;
	return ret;
}

/* block write */
static int
isl12020_i2c_set_alarm_regs(struct i2c_client *client, u8 const buf[ISL12020_ALARM_SECTION_LEN])
{
	u8 i2c_buf[ISL12020_ALARM_SECTION_LEN + 2];
	struct i2c_msg msgs[1] = {
		{client->addr, 0, ISL12020_ALARM_SECTION_LEN + 1, i2c_buf}
	};
	int ret;

	i2c_buf[0] = ISL12020_REG_SCA;
	memcpy(&i2c_buf[1], &buf[0], ISL12020_ALARM_SECTION_LEN);

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	return ret;
}

/* simple check to see wether we have a isl12020 */
static int
isl12020_i2c_validate_client(struct i2c_client *client)
{
	u8 regs[ISL12020_RTC_SECTION_LEN] = { 0, };
	u8 zero_mask[ISL12020_RTC_SECTION_LEN] = {
		0x80, 0x80, 0x40, 0xc0, 0xe0, 0x00, 0xf8
	};
	int i;
	int ret;

	ret = isl12020_i2c_read_regs(client, 0, regs, ISL12020_RTC_SECTION_LEN);
	if (ret < 0)
		return ret;

	for (i = 0; i < ISL12020_RTC_SECTION_LEN; ++i) {
		if (regs[i] & zero_mask[i])	/* check if bits are cleared */
			return -ENODEV;
	}

	return 0;
}

static int
isl12020_i2c_get_sr(struct i2c_client *client)
{
	int sr = i2c_smbus_read_byte_data(client, ISL12020_REG_SR);
	if (sr < 0)
		return -EIO;

	return sr;
}

static int
isl12020_i2c_get_inter(struct i2c_client *client)
{
	int inter = i2c_smbus_read_byte_data(client, ISL12020_REG_INT);
	if (inter < 0)
		return -EIO;

	return inter;
}

static int
isl12020_i2c_get_atr(struct i2c_client *client)
{
	int atr = i2c_smbus_read_byte_data(client, ISL12020_REG_FATR);
	if (atr < 0)
		return atr;

	/* The 6bit value in the ATR register controls the load
	 * capacitance C_load * in steps of 0.25pF
	 *
	 * bit (1<<5) of the ATR register is inverted
	 *
	 * C_load(ATR=0x20) =  4.50pF
	 * C_load(ATR=0x00) = 12.50pF
	 * C_load(ATR=0x1f) = 20.25pF
	 *
	 */

	atr &= 0x3f;		/* mask out lsb */
	atr ^= 1 << 5;		/* invert 6th bit */
	atr += 2 * 9;		/* add offset of 4.5pF; unit[atr] = 0.25pF */

	return atr;
}

static int
isl12020_i2c_get_dtr(struct i2c_client *client)
{
	int dtr = i2c_smbus_read_byte_data(client, ISL12020_REG_FDTR);
	if (dtr < 0)
		return -EIO;

	/* dtr encodes adjustments of {-60,-40,-20,0,20,40,60} ppm */
	dtr = ((dtr & 0x3) * 20) * (dtr & (1 << 2) ? -1 : 1);

	return dtr;
}

static int
isl12020_i2c_get_temp(struct i2c_client *client, int *temp)
{
	u8 buf[ISL12020_TEMP_SECTION_LEN] = { 0, };
	int ret;
	int beta;

	beta = i2c_smbus_read_byte_data(client, ISL12020_REG_BETA);
	if (beta >= 0) {
		beta |= ISL12020_REG_BETA_TSE;
		ret = i2c_smbus_write_byte_data(client, ISL12020_REG_BETA,
				beta);
		if (ret < 0)
			return ret;
	}

	mdelay(30);

	ret = isl12020_i2c_read_regs(client, ISL12020_REG_TK0L, buf,
				    ISL12020_TEMP_SECTION_LEN);
	if (ret < 0)
		return ret;

	*temp = ((buf[1] << 8) | buf[0]) / 2 - 273;
	return 0;
}

static int
isl12020_i2c_read_time(struct i2c_client *client, struct rtc_time *tm)
{
	int sr, rc;
	u8 regs[ISL12020_RTC_SECTION_LEN] = { 0, };

	sr = isl12020_i2c_get_sr(client);
	if (sr < 0) {
		dev_err(&client->dev, "%s: reading SR failed\n", __func__);
		return -EIO;
	}

	rc = isl12020_i2c_read_regs(client, 0, regs, ISL12020_RTC_SECTION_LEN);
	if (rc < 0) {
		dev_err(&client->dev, "%s: reading RTC section failed\n",
			__func__);
		return rc;
	}

	tm->tm_sec = bcd2bin(regs[ISL12020_REG_SC]);
	tm->tm_min = bcd2bin(regs[ISL12020_REG_MN]);

	/* HR field has a more complex interpretation */
	{
		const u8 _hr = regs[ISL12020_REG_HR];
		if (_hr & ISL12020_REG_HR_MIL)	/* 24h format */
			tm->tm_hour = bcd2bin(_hr & 0x3f);
		else {
			/* 12h format */
			tm->tm_hour = bcd2bin(_hr & 0x1f);
			if (_hr & ISL12020_REG_HR_PM)	/* PM flag set */
				tm->tm_hour += 12;
		}
	}

	tm->tm_mday = bcd2bin(regs[ISL12020_REG_DT]);
	tm->tm_mon = bcd2bin(regs[ISL12020_REG_MO]) - 1;	/* rtc starts at 1 */
	tm->tm_year = bcd2bin(regs[ISL12020_REG_YR]) + 100;
	tm->tm_wday = bcd2bin(regs[ISL12020_REG_DW]);

	return 0;
}

static int
isl12020_i2c_read_alarm(struct i2c_client *client, struct rtc_wkalrm *alarm)
{
	struct rtc_time *const tm = &alarm->time;
	u8 regs[ISL12020_ALARM_SECTION_LEN] = { 0, };
	const u8 *r;
	int sr, rc;

	sr = isl12020_i2c_get_sr(client);
	if (sr < 0) {
		dev_err(&client->dev, "%s: reading SR failed\n", __func__);
		return sr;
	}

	rc = isl12020_i2c_read_regs(client, ISL12020_REG_SCA, regs,
				   ISL12020_ALARM_SECTION_LEN);
	if (rc < 0) {
		dev_err(&client->dev, "%s: reading alarm section failed\n",
			__func__);
		return rc;
	}

	/* MSB of each alarm register is an enable bit */
	r = regs;

	tm->tm_sec  = *r & 0x80 ? bcd2bin(*r & 0x7f)     : -1; r ++;
	tm->tm_min  = *r & 0x80 ? bcd2bin(*r & 0x7f)     : -1; r ++;
	tm->tm_hour = *r & 0x80 ? bcd2bin(*r & 0x7f)     : -1; r ++;
	tm->tm_mday = *r & 0x80 ? bcd2bin(*r & 0x7f)     : -1; r ++;
	tm->tm_mon  = *r & 0x80 ? bcd2bin(*r & 0x7f) - 1 : -1; r ++;
	tm->tm_wday = *r & 0x80 ? bcd2bin(*r & 0x7f)     : -1; r ++;
	tm->tm_year = -1;

	return 0;
}

static int
isl12020_i2c_control_alarm(struct i2c_client *client, int enable)
{
	u8 regs[ISL12020_ALARM_SECTION_LEN] = { 0, };
	int rc;

	printk(KERN_INFO "control_alarm %d\n", enable);
	if (enable) return 0;

	rc = isl12020_i2c_read_alarm_regs(client, regs);
	if (rc < 0) {
		dev_err(&client->dev, "%s: reading alarm section failed\n",
			__func__);
		return rc;
	}

	memset(regs, 0x80, sizeof(regs));

	rc = isl12020_i2c_set_alarm_regs(client, regs);
	if (rc < 0) {
		dev_err(&client->dev, "%s: writing alarm section failed\n",
			__func__);
		return rc;
	}

	return 0;
}

static int isl12020_i2c_clear_pending_alarm(struct i2c_client *client)
{
	int sr, rc;

	dev_info(&client->dev, "clearing pending alarm\n");

	rc = isl12020_i2c_control_alarm(client, 0);
	if (rc < 0) return rc;

	sr = isl12020_i2c_get_sr(client);
	if (sr < 0) {
		dev_err(&client->dev, "%s: reading SR failed\n", __func__);
		return sr;
	}
	sr &= ~ISL12020_REG_SR_ALM;
	rc = i2c_smbus_write_byte_data(client, ISL12020_REG_SR, sr);
	if (rc < 0) {
		dev_err(&client->dev, "%s: writing SR failed\n", __func__);
		return rc;
	}

	return 0;
}

static int
isl12020_i2c_set_alarm(struct i2c_client *client, struct rtc_wkalrm *alarm)
{
	struct rtc_time *const tm = &alarm->time;
	struct rtc_time rtc_now;
	u8 regs[ISL12020_ALARM_SECTION_LEN] = { 0, };
	int rc;
	unsigned long later, now;

	/* Get alarm time in seconds */
	rc = rtc_tm_to_time(tm, &later);
	if (rc < 0)
		return rc;

	/* Get current time in seconds */
	rc = isl12020_i2c_read_time(client, &rtc_now);
	if (rc < 0)
		return rc;
	rc = rtc_tm_to_time(&rtc_now, &now);
	if (rc < 0)
		return rc;

	if (later <= now)
		return -EINVAL;
	if ((later - now) > 365 * 86400)
		return -EDOM;

	regs[ISL12020_REG_SCA - ISL12020_REG_SCA] =
		0x80 | bin2bcd(tm->tm_sec);
	regs[ISL12020_REG_MNA - ISL12020_REG_SCA] =
		0x80 | bin2bcd(tm->tm_min);
	regs[ISL12020_REG_HRA - ISL12020_REG_SCA] =
		0x80 | bin2bcd(tm->tm_hour);
	regs[ISL12020_REG_DTA - ISL12020_REG_SCA] =
		0x80 | bin2bcd(tm->tm_mday);
	regs[ISL12020_REG_MOA - ISL12020_REG_SCA] =
		0x80 | bin2bcd(tm->tm_mon + 1);
	regs[ISL12020_REG_DWA - ISL12020_REG_SCA] =
		0;

	rc = isl12020_i2c_set_alarm_regs(client, regs);
	if (rc < 0) {
		dev_err(&client->dev, "%s: setting alarm section failed\n",
			__func__);
		return rc;
	}

	return 0;
}

static int
isl12020_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return isl12020_i2c_read_time(to_i2c_client(dev), tm);
}

static int
isl12020_i2c_set_time(struct i2c_client *client, struct rtc_time const *tm)
{
	int inter, rc;
	u8 regs[ISL12020_RTC_SECTION_LEN] = { 0, };

	/* The clock has an 8 bit wide bcd-coded register (they never learn)
	 * for the year. tm_year is an offset from 1900 and we are interested
	 * in the 2000-2099 range, so any value less than 100 is invalid.
	 */
	if (tm->tm_year < 100)
		return -EINVAL;

	regs[ISL12020_REG_SC] = bin2bcd(tm->tm_sec);
	regs[ISL12020_REG_MN] = bin2bcd(tm->tm_min);
	regs[ISL12020_REG_HR] = bin2bcd(tm->tm_hour) | ISL12020_REG_HR_MIL;

	regs[ISL12020_REG_DT] = bin2bcd(tm->tm_mday);
	regs[ISL12020_REG_MO] = bin2bcd(tm->tm_mon + 1);
	regs[ISL12020_REG_YR] = bin2bcd(tm->tm_year - 100);

	regs[ISL12020_REG_DW] = bin2bcd(tm->tm_wday & 7);

	inter = isl12020_i2c_get_inter(client);
	if (inter < 0) {
		dev_err(&client->dev, "%s: reading INT failed\n", __func__);
		return inter;
	}

	/* set WRTC */
	inter |= ISL12020_REG_INT_WRTC;
	rc = i2c_smbus_write_byte_data(client, ISL12020_REG_INT, inter);
	if (rc < 0) {
		dev_err(&client->dev, "%s: writing INT failed\n", __func__);
		return rc;
	}

	/* write RTC registers */
	rc = isl12020_i2c_set_regs(client, regs);
	if (rc < 0) {
		dev_err(&client->dev, "%s: writing RTC section failed\n",
			__func__);
		return rc;
	}

	/* clear WRTC again */
	rc = i2c_smbus_write_byte_data(client, ISL12020_REG_INT,
				       inter & ~ISL12020_REG_INT_WRTC);
	if (rc < 0) {
		dev_err(&client->dev, "%s: writing INT failed\n", __func__);
		return rc;
	}

	return 0;
}


static int
isl12020_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return isl12020_i2c_set_time(to_i2c_client(dev), tm);
}

static int
isl12020_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	return isl12020_i2c_read_alarm(to_i2c_client(dev), alarm);
}

static int
isl12020_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	return isl12020_i2c_set_alarm(to_i2c_client(dev), alarm);
}

static int isl12020_rtc_alarm_irq_enable(struct device *dev,
		unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);

	return isl12020_i2c_control_alarm(client, enabled);
}

static int
isl12020_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct i2c_client *const client = to_i2c_client(dev);
	int sr, inter, dtr, atr, temp;

	sr = isl12020_i2c_get_sr(client);
	if (sr < 0) {
		dev_err(&client->dev, "%s: reading SR failed\n", __func__);
		return sr;
	}

	seq_printf(seq, "status_reg\t:%s%s%s%s%s%s%s%s (0x%.2x)\n",
		   (sr & ISL12020_REG_SR_BUSY) ? " BUSY" : "",
		   (sr & ISL12020_REG_SR_OSCF) ? " OSCF" : "",
		   (sr & ISL12020_REG_SR_DSTADJ) ? " DSTADJ" : "",
		   (sr & ISL12020_REG_SR_ALM) ? " ALM" : "",
		   (sr & ISL12020_REG_SR_LVDD) ? " LVDD" : "",
		   (sr & ISL12020_REG_SR_LBAT85) ? " LBAT85" : "",
		   (sr & ISL12020_REG_SR_LBAT75) ? " LBAT75" : "",
		   (sr & ISL12020_REG_SR_RTCF) ? " RTCF" : "", sr);

	if (sr & ISL12020_REG_SR_ALM)
		isl12020_i2c_clear_pending_alarm(client);

	inter = isl12020_i2c_get_inter(client);
	if (inter < 0) {
		dev_err(&client->dev, "%s: reading INT failed\n", __func__);
		return inter;
	}

	seq_printf(seq, "int_reg\t\t:%s%s%s%s%s%s%s%s (0x%.2x)\n",
		   (inter & ISL12020_REG_INT_ARST) ? " ARST" : "",
		   (inter & ISL12020_REG_INT_WRTC) ? " WRTC" : "",
		   (inter & ISL12020_REG_INT_IM) ? " IM" : "",
		   (inter & ISL12020_REG_INT_FOBATB) ? " FOBATB" : "",
		   (inter & ISL12020_REG_INT_FO3) ? " FO3" : "",
		   (inter & ISL12020_REG_INT_FO2) ? " FO2" : "",
		   (inter & ISL12020_REG_INT_FO1) ? " FO1" : "",
		   (inter & ISL12020_REG_INT_FO0) ? " FO0" : "", inter);

	seq_printf(seq, "batt_status\t: %s\n",
		   (sr & ISL12020_REG_SR_LBAT75) ? "<75%" :
		   	(sr & ISL12020_REG_SR_LBAT85) ? "<85%" :
			"okay");

	dtr = isl12020_i2c_get_dtr(client);
	if (dtr >= 0 - 1)
		seq_printf(seq, "digital_trim\t: %d ppm\n", dtr);

	atr = isl12020_i2c_get_atr(client);
	if (atr >= 0)
		seq_printf(seq, "analog_trim\t: %d.%.2d pF\n",
			   atr >> 2, (atr & 0x3) * 25);

	if (isl12020_i2c_get_temp(client, &temp) == 0)
		seq_printf(seq, "temp\t\t: %d C\n", temp);

	return 0;
}

static const struct rtc_class_ops isl12020_rtc_ops = {
	.proc = isl12020_rtc_proc,
	.read_time = isl12020_rtc_read_time,
	.set_time = isl12020_rtc_set_time,
	.read_alarm = isl12020_rtc_read_alarm,
	.set_alarm = isl12020_rtc_set_alarm,
	.alarm_irq_enable = isl12020_rtc_alarm_irq_enable,
};

/* sysfs interface */

static ssize_t
isl12020_sysfs_show_atrim(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int atr = isl12020_i2c_get_atr(to_i2c_client(dev));
	if (atr < 0)
		return atr;

	return sprintf(buf, "%d.%.2d pF\n", atr >> 2, (atr & 0x3) * 25);
}

static DEVICE_ATTR(atrim, S_IRUGO, isl12020_sysfs_show_atrim, NULL);

static ssize_t
isl12020_sysfs_show_dtrim(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int dtr = isl12020_i2c_get_dtr(to_i2c_client(dev));
	if (dtr < 0)
		return dtr;

	return sprintf(buf, "%d ppm\n", dtr);
}

static DEVICE_ATTR(dtrim, S_IRUGO, isl12020_sysfs_show_dtrim, NULL);

static ssize_t
isl12020_sysfs_show_temp(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	int temp;
	int rc = isl12020_i2c_get_temp(to_i2c_client(dev), &temp);

	if (rc < 0)
		return rc;

	return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR(temp, S_IRUGO, isl12020_sysfs_show_temp,
		   NULL);

static int
isl12020_sysfs_register(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dev_attr_atrim);
	if (err)
		return err;

	err = device_create_file(dev, &dev_attr_dtrim);
	if (err) {
		device_remove_file(dev, &dev_attr_atrim);
		return err;
	}

	err = device_create_file(dev, &dev_attr_temp);
	if (err) {
		device_remove_file(dev, &dev_attr_atrim);
		device_remove_file(dev, &dev_attr_dtrim);
	}

	return 0;
}

static int
isl12020_sysfs_unregister(struct device *dev)
{
	device_remove_file(dev, &dev_attr_dtrim);
	device_remove_file(dev, &dev_attr_atrim);
	device_remove_file(dev, &dev_attr_temp);

	return 0;
}

static int
isl12020_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int sr, inter, rc = 0;
	struct rtc_device *rtc;
	int beta;

	/**
         * This needs to be disabled to work with xilinx_iic
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;
        */

	if (isl12020_i2c_validate_client(client) < 0)
		return -ENODEV;

	dev_info(&client->dev,
		 "chip found, driver version " DRV_VERSION "\n");

	rtc = rtc_device_register(isl12020_driver.driver.name,
				  &client->dev, &isl12020_rtc_ops,
				  THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	i2c_set_clientdata(client, rtc);

	sr = isl12020_i2c_get_sr(client);
	if (sr < 0) {
		dev_err(&client->dev, "reading status failed\n");
		goto exit_unregister;
	}

	if (sr & ISL12020_REG_SR_RTCF)
		dev_warn(&client->dev, "rtc power failure detected, "
			 "please set clock.\n");

	if (sr & ISL12020_REG_SR_ALM)
		isl12020_i2c_clear_pending_alarm(client);

	/* enable temperature sensing */
	beta = i2c_smbus_read_byte_data(client, ISL12020_REG_BETA);
	if (beta >= 0) {
		beta |= ISL12020_REG_BETA_BTSR;
		beta &= ~ISL12020_REG_BETA_BTSE;
		rc = i2c_smbus_write_byte_data(client, ISL12020_REG_BETA, beta);
		if (rc < 0)
			dev_warn(&client->dev, "can't write beta reg");
	} else dev_warn(&client->dev, "can't read beta reg");

	inter = isl12020_i2c_get_inter(client);
	if (inter < 0) {
		dev_warn(&client->dev, "%s: reading INT failed\n", __func__);
		goto exit_unregister;
	}

	/* Clear IM and set FOBATB */
	inter &= ~(ISL12020_REG_INT_IM | ISL12020_REG_INT_FOBATB |
			ISL12020_REG_INT_FO3 |
			ISL12020_REG_INT_FO2 |
			ISL12020_REG_INT_FO1 |
			ISL12020_REG_INT_FO0);
	rc = i2c_smbus_write_byte_data(client, ISL12020_REG_INT, inter);
	if (rc < 0) {
		dev_err(&client->dev, "%s: writing INT failed\n", __func__);
		goto exit_unregister;
	}

	/* Register with sysfs */
	rc = isl12020_sysfs_register(&client->dev);
	if (rc)
		goto exit_unregister;

	return 0;

exit_unregister:
	rtc_device_unregister(rtc);

	return rc;
}

static int
isl12020_remove(struct i2c_client *client)
{
	struct rtc_device *rtc = i2c_get_clientdata(client);

	isl12020_sysfs_unregister(&client->dev);
	rtc_device_unregister(rtc);

	return 0;
}

static const struct i2c_device_id isl12020_id[] = {
	{ "isl12020", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl12020_id);

static struct i2c_driver isl12020_driver = {
	.driver = {
		   .name = "rtc-isl12020",
		   },
	.probe = isl12020_probe,
	.remove = isl12020_remove,
	.id_table = isl12020_id,
};

static int __init
isl12020_init(void)
{
	return i2c_add_driver(&isl12020_driver);
}

static void __exit
isl12020_exit(void)
{
	i2c_del_driver(&isl12020_driver);
}

MODULE_AUTHOR("Herbert Valerio Riedel <hvr@gnu.org>");
MODULE_DESCRIPTION("Intersil ISL12020 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(isl12020_init);
module_exit(isl12020_exit);
