/*
 * drivers/input/misc/elan_epl2182.c - light and proxmity sensors driver
 * Copyright (C) 2011-2014 ELAN Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h> /* Required for DEVICE_ATTR macros */
#include "linux/elan_interface.h"

/*********************************************************
 * configuration
*********************************************************/
/* 0 is polling mode, 1 is interrupt mode*/
#define NO_P_SENSOR
#ifndef NO_P_SENSOR
#define PS_INTERRUPT_MODE		1
#else
#define PS_INTERRUPT_MODE		0
#endif

#define PS_POLLING_RATE			500     /* msec */
#define ALS_POLLING_RATE		1000	/* msec */
#define LUX_PER_COUNT			440

//#define DEBUG

#define TXBYTES				2
#define RXBYTES				2

#define HS_INTT_CENTER			4
static int HS_INTT 			= HS_INTT_CENTER;

#define PS_DELAY			55
#define ALS_DELAY			100
#define HS_DELAY 			30

#define PS_DRIVE			EPL_DRIVE_120MA

#define I2C_RETRY_COUNT			3 /* Kernel I2C core handles retries, this is a fallback */
#define P_INTT				1

#define PS_INTT				4
#define ALS_INTT			12

#define PS_H_THRESHOLD			2000
#define PS_L_THRESHOLD			1000

#ifndef NO_P_SENSOR
#define PS_AUTO_ENABLE	1
#else
#define PS_AUTO_ENABLE	0
#endif
#if PS_AUTO_ENABLE
#define DYN_L_OFFSET	500
#define DYN_H_OFFSET	700
#define DYN_CONDITION	30000
#endif

enum cmc_mode {
	CMC_MODE_ALS = 0x00,
	CMC_MODE_PS = 0x10,
};

/* primitive raw data from I2C */
struct epl_raw_data {
	u8 raw_bytes[8]; /* Max read size */
	u16 renvo;
	u16 ps_int_state;
	u16 ps_ch1_raw;
	u16 als_ch1_raw;
	u16 ps_state;
	u16 ps_sta;
	u16 hs_data[200];
#if PS_AUTO_ENABLE
	u16 ps_min_raw;
	u16 ps_condition;
	u16 ps_cal_h;
	u16 ps_cal_l;
#endif
};

struct elan_epl_data {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vio;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
	struct workqueue_struct *epl_wq;
#if PS_INTERRUPT_MODE
	struct work_struct irq_work;
#endif
	struct delayed_work report_polling_work;
	struct delayed_work polling_work;
	struct mutex data_mutex;
	struct epl_raw_data raw_data;
	struct wake_lock ps_wlock;

	unsigned int als_poll_delay;
	unsigned int ps_poll_delay;
	int ps_opened;
	int als_opened;
	unsigned int ps_th_l;
	unsigned int ps_th_h;
	int enable_pflag;
	int enable_lflag;
	int enable_hflag;
	int l_suspend;
	int read_flag;
	int irq_gpio;
	unsigned int irq_gpio_flags;
	int dual_count;
};

/* A single instance is used to bridge miscdevice fops to the driver data */
static struct elan_epl_data *epl_data_ptr;

#if PS_INTERRUPT_MODE
static bool change_int_time = false;
static void epl_sensor_irq_do_work(struct work_struct *work);
#endif
static int hs_count;
static int hs_idx;
static int show_hs_raws_flag;
static int hs_als_flag;

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "elan",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "30000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#ifndef NO_P_SENSOR
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "elan",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

static int set_psensor_intr_threshold(struct elan_epl_data *epld,
				      u16 low_thd, u16 high_thd);

static void report_polling_do_work(struct work_struct *work);
static void polling_do_work(struct work_struct *work);

static int elan_sensor_I2C_Write(struct i2c_client *client, u8 regaddr,
				 u8 bytecount, u8 txbyte, u8 data)
{
	u8 buffer[2];
	int ret;

	buffer[0] = (regaddr << 3) | bytecount;
	buffer[1] = data;

	ret = i2c_master_send(client, buffer, txbyte);
	if (ret != txbyte) {
		dev_err(&client->dev, "i2c_master_send failed (ret=%d)\n", ret);
		return (ret < 0) ? ret : -EIO;
	}
	return 0;
}

static int elan_sensor_I2C_Read(struct i2c_client *client, u8 *buf, int len)
{
	int ret;

	ret = i2c_master_recv(client, buf, len);
	if (ret != len) {
		dev_err(&client->dev, "i2c_master_recv failed (ret=%d)\n", ret);
		return (ret < 0) ? ret : -EIO;
	}
	return 0;
}

#if PS_INTERRUPT_MODE
static void epl2182_read_hs(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;
	int max_frame = ARRAY_SIZE(epld->raw_data.hs_data);
	int idx = hs_idx + hs_count;
	u16 data;
	int ret;

	mutex_lock(&epld->data_mutex);
	elan_sensor_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	ret = elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 2);
	if (ret)
		goto unlock;

	data = (epld->raw_data.raw_bytes[1] << 8) | epld->raw_data.raw_bytes[0];

	if (data > 60800 && HS_INTT > HS_INTT_CENTER - 5) {
		HS_INTT--;
		change_int_time = true;
	} else if (data > 6400 && data < 25600 && HS_INTT < HS_INTT_CENTER + 5) {
		HS_INTT++;
		change_int_time = true;
	} else {
		change_int_time = false;
		if (idx >= max_frame)
			idx -= max_frame;

		epld->raw_data.hs_data[idx] = data;

		if (hs_count >= max_frame) {
			hs_idx++;
			if (hs_idx >= max_frame)
				hs_idx = 0;
		}

		hs_count++;
		if (hs_count >= max_frame)
			hs_count = max_frame;
	}
unlock:
	mutex_unlock(&epld->data_mutex);
}
#endif

static void elan_sensor_restart_work(struct elan_epl_data *epld)
{
	cancel_delayed_work_sync(&epld->polling_work);
	cancel_delayed_work_sync(&epld->report_polling_work);
	queue_delayed_work(epld->epl_wq, &epld->polling_work,
			   msecs_to_jiffies(10));
}

#if PS_AUTO_ENABLE
static void dyn_ps_cal(struct elan_epl_data *epld)
{
	if ((epld->raw_data.ps_ch1_raw < epld->raw_data.ps_min_raw) &&
	    (epld->raw_data.ps_sta != 1) &&
	    (epld->raw_data.ps_condition <= DYN_CONDITION)) {
		epld->raw_data.ps_min_raw = epld->raw_data.ps_ch1_raw;
		epld->ps_th_l = epld->raw_data.ps_ch1_raw + DYN_L_OFFSET;
		epld->ps_th_h = epld->raw_data.ps_ch1_raw + DYN_H_OFFSET;
		set_psensor_intr_threshold(epld, epld->ps_th_l, epld->ps_th_h);
		dev_dbg(&epld->client->dev,
			"dyn ps raw=%d, min=%d, h_th=%d, l_th=%d\n",
			epld->raw_data.ps_ch1_raw, epld->raw_data.ps_min_raw,
			epld->ps_th_h, epld->ps_th_l);
	}
}
#endif

static void epl2182_hs_enable(struct elan_epl_data *epld, bool interrupt,
			      bool full_enable)
{
	struct i2c_client *client = epld->client;
	u8 regdata = 0;

	if (full_enable) {
		regdata = PS_DRIVE |
			  (interrupt ? EPL_INT_FRAME_ENABLE : EPL_INT_DISABLE);
		elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				      regdata);

		regdata = EPL_SENSING_1_TIME | EPL_PS_MODE | EPL_L_GAIN |
			  EPL_S_SENSING_MODE;
		elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02,
				      regdata);

		regdata = (HS_INTT << 4) | EPL_PST_1_TIME | EPL_12BIT_ADC;
		elan_sensor_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02,
				      regdata);

		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02,
				      EPL_C_RESET);
	}

	elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_C_START_RUN);
}

static int elan_sensor_psensor_enable(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;
	u8 regdata = 0;
	int ret;

	dev_dbg(&client->dev, "Proximity sensor Enable\n");

	elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
			      EPL_INT_DISABLE);

	regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN;
	regdata |= (PS_INTERRUPT_MODE ? EPL_C_SENSING_MODE :
				      EPL_S_SENSING_MODE);
	elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

	regdata = (PS_INTT << 4) | EPL_PST_1_TIME | EPL_14BIT_ADC;
	elan_sensor_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

	set_psensor_intr_threshold(epld, epld->ps_th_l, epld->ps_th_h);

	elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02, EPL_C_RESET);
	ret = elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				    EPL_C_START_RUN);
	msleep(PS_DELAY);

#if PS_INTERRUPT_MODE
	if (epld->enable_pflag) {
		elan_sensor_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
		if (!elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 1)) {
			epld->raw_data.ps_state =
				!((epld->raw_data.raw_bytes[0] & 0x04) >> 2);
			epld->raw_data.ps_sta =
				((epld->raw_data.raw_bytes[0] & 0x02) >> 1);
		}
#if PS_AUTO_ENABLE
		elan_sensor_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
		if (!elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 2)) {
			epld->raw_data.ps_ch1_raw =
				(epld->raw_data.raw_bytes[1] << 8) |
				epld->raw_data.raw_bytes[0];
		}

		elan_sensor_I2C_Write(client, REG_14, R_TWO_BYTE, 0x01, 0x00);
		if (!elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 2)) {
			epld->raw_data.ps_condition =
				(epld->raw_data.raw_bytes[1] << 8) |
				epld->raw_data.raw_bytes[0];
		}
		dyn_ps_cal(epld);
#endif

		if (epld->raw_data.ps_state != epld->raw_data.ps_int_state) {
			elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
					      EPL_INT_FRAME_ENABLE);
		} else {
			elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
					      EPL_INT_ACTIVE_LOW);
		}
	} else {
		elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				      EPL_INT_ACTIVE_LOW);
	}
#endif
	return ret;
}

static int elan_sensor_lsensor_enable(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;
	u8 regdata = 0;

	dev_dbg(&client->dev, "ALS sensor Enable\n");

	elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
			      EPL_INT_DISABLE);

	regdata = EPL_S_SENSING_MODE | EPL_SENSING_8_TIME | EPL_ALS_MODE |
		  EPL_AUTO_GAIN;
	elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

	regdata = (ALS_INTT << 4) | EPL_PST_1_TIME | EPL_10BIT_ADC;
	elan_sensor_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

	elan_sensor_I2C_Write(client, REG_10, W_SINGLE_BYTE, 0x02, EPL_GO_MID);
	elan_sensor_I2C_Write(client, REG_11, W_SINGLE_BYTE, 0x02, EPL_GO_LOW);
	elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02, EPL_C_RESET);

	return elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				     EPL_C_START_RUN);
}

static void elan_epl_ps_poll_rawdata(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;
	int ret;

	mutex_lock(&epld->data_mutex);

	elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_LOCK);

	elan_sensor_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
	ret = elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 1);
	if (!ret)
		epld->raw_data.ps_state =
			!((epld->raw_data.raw_bytes[0] & 0x04) >> 2);

	elan_sensor_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	ret = elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 2);
	if (!ret)
		epld->raw_data.ps_ch1_raw =
			(epld->raw_data.raw_bytes[1] << 8) |
			epld->raw_data.raw_bytes[0];

	elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_DATA_UNLOCK);

	dev_dbg(&client->dev, "ps_ch1_raw_data (%d), value(%d)\n",
		epld->raw_data.ps_ch1_raw, epld->raw_data.ps_state);

	input_report_abs(epld->ps_input_dev, ABS_DISTANCE,
			 epld->raw_data.ps_state);
	input_sync(epld->ps_input_dev);

	mutex_unlock(&epld->data_mutex);
}

static void elan_epl_als_rawdata(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;
	u32 lux;
	ktime_t ts;
	int ret;

	mutex_lock(&epld->data_mutex);

	ts = ktime_get_boottime();
	elan_sensor_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	ret = elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 2);
	if (ret)
		goto unlock;

	epld->raw_data.als_ch1_raw = (epld->raw_data.raw_bytes[1] << 8) |
				   epld->raw_data.raw_bytes[0];

	lux = (epld->raw_data.als_ch1_raw * LUX_PER_COUNT) / 1000 * 15 / 100;
	if (lux > 20000)
		lux = 20000;

	dev_dbg(&client->dev, "ALS raw = %d, lux = %d\n",
		epld->raw_data.als_ch1_raw, lux);

	input_report_abs(epld->als_input_dev, ABS_MISC, lux);
	input_event(epld->als_input_dev, EV_SYN, SYN_TIME_SEC,
		    ktime_to_timespec(ts).tv_sec);
	input_event(epld->als_input_dev, EV_SYN, SYN_TIME_NSEC,
		    ktime_to_timespec(ts).tv_nsec);
	input_sync(epld->als_input_dev);

unlock:
	mutex_unlock(&epld->data_mutex);
}

static int set_psensor_intr_threshold(struct elan_epl_data *epld, u16 low_thd,
				      u16 high_thd)
{
	struct i2c_client *client = epld->client;
	u8 high_msb, high_lsb, low_msb, low_lsb;

	high_msb = (u8)(high_thd >> 8);
	high_lsb = (u8)(high_thd & 0x00ff);
	low_msb = (u8)(low_thd >> 8);
	low_lsb = (u8)(low_thd & 0x00ff);

	elan_sensor_I2C_Write(client, REG_2, W_SINGLE_BYTE, 0x02, high_lsb);
	elan_sensor_I2C_Write(client, REG_3, W_SINGLE_BYTE, 0x02, high_msb);
	elan_sensor_I2C_Write(client, REG_4, W_SINGLE_BYTE, 0x02, low_lsb);
	elan_sensor_I2C_Write(client, REG_5, W_SINGLE_BYTE, 0x02, low_msb);

	return 0;
}

#if PS_INTERRUPT_MODE
static void epl_sensor_irq_do_work(struct work_struct *work)
{
	struct elan_epl_data *epld =
		container_of(work, struct elan_epl_data, irq_work);
	struct i2c_client *client = epld->client;
	int mode = 0;
	int ret;

	if (epld->enable_hflag == 1) {
		epl2182_read_hs(epld);
		epl2182_hs_enable(epld, true, change_int_time);
	}

	if (epld->enable_pflag == 1) {
		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_DATA_LOCK);
		elan_sensor_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);

		ret = elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 1);
		if (!ret) {
			mode = epld->raw_data.raw_bytes[0] & (3 << 4);

			if (mode == CMC_MODE_PS) {
				epld->raw_data.ps_int_state =
				  !((epld->raw_data.raw_bytes[0] & 0x04) >> 2);
				elan_epl_ps_poll_rawdata(epld);
			} else {
				dev_warn(&client->dev, "interrupt in wrong mode\n");
			}
		}

		elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				      EPL_INT_ACTIVE_LOW);

		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_DATA_UNLOCK);
	}
	enable_irq(client->irq);
}

static irqreturn_t elan_sensor_irq_handler(int irqNo, void *handle)
{
	struct i2c_client *client = (struct i2c_client *)handle;
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	disable_irq_nosync(client->irq);
	queue_work(epld->epl_wq, &epld->irq_work);

	return IRQ_HANDLED;
}
#endif

static void report_polling_do_work(struct work_struct *work)
{
	struct elan_epl_data *epld = container_of(to_delayed_work(work),
						  struct elan_epl_data,
						  report_polling_work);
	if (epld->dual_count == CMC_MODE_PS)
		elan_epl_ps_poll_rawdata(epld);
	else if (epld->dual_count == CMC_MODE_ALS)
		elan_epl_als_rawdata(epld);

	if (epld->enable_pflag) {
		elan_sensor_psensor_enable(epld);

		if (PS_INTERRUPT_MODE == 0) {
			epld->dual_count = CMC_MODE_PS;
			queue_delayed_work(epld->epl_wq,
					   &epld->report_polling_work,
					   msecs_to_jiffies(epld->ps_poll_delay));
		}
	}
}

static void polling_do_work(struct work_struct *work)
{
	struct elan_epl_data *epld = container_of(to_delayed_work(work),
						  struct elan_epl_data,
						  polling_work);
	struct i2c_client *client = epld->client;

	bool is_interleaving = epld->enable_pflag && epld->enable_lflag &&
			       !epld->l_suspend;
	bool is_als_only = !epld->enable_pflag && epld->enable_lflag &&
			   !epld->l_suspend;
	bool is_ps_only = epld->enable_pflag &&
			  (!epld->enable_lflag || epld->l_suspend);

	cancel_delayed_work(&epld->polling_work);
	cancel_delayed_work(&epld->report_polling_work);

	dev_dbg(&client->dev, "polling work: pflag=%d, lflag=%d\n",
		epld->enable_pflag, epld->enable_lflag);

	if (is_als_only || is_interleaving) {
		elan_sensor_lsensor_enable(epld);
		epld->dual_count = CMC_MODE_ALS;
		queue_delayed_work(epld->epl_wq, &epld->report_polling_work,
				   msecs_to_jiffies(ALS_DELAY));
		queue_delayed_work(epld->epl_wq, &epld->polling_work,
				   msecs_to_jiffies(epld->als_poll_delay));
	} else if (is_ps_only || PS_AUTO_ENABLE) {
		elan_sensor_psensor_enable(epld);
		epld->dual_count = CMC_MODE_PS;

		if (!PS_INTERRUPT_MODE) {
			queue_delayed_work(epld->epl_wq, &epld->report_polling_work,
					   msecs_to_jiffies(PS_DELAY));
			queue_delayed_work(epld->epl_wq, &epld->polling_work,
					   msecs_to_jiffies(epld->ps_poll_delay));
		}
	} else {
		elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				      EPL_INT_DISABLE);
		elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02,
				      EPL_S_SENSING_MODE);
	}
}

static ssize_t elan_ls_operationmode_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct elan_epl_data *epld = dev_get_drvdata(dev);
	u16 mode;

	if (kstrtou16(buf, 0, &mode))
		return -EINVAL;

	dev_dbg(&epld->client->dev, "operation mode set to %u\n", mode);

	switch (mode) {
	case 0:
		epld->enable_lflag = 0;
		epld->enable_pflag = 0;
		break;
	case 1:
		epld->enable_lflag = 1;
		epld->enable_pflag = 0;
		break;
#ifndef NO_P_SENSOR
	case 2:
		epld->enable_lflag = 0;
		epld->enable_pflag = 1;
		break;
	case 3:
		epld->enable_lflag = 1;
		epld->enable_pflag = 1;
		break;
#endif
	default:
		dev_warn(&epld->client->dev, "invalid mode %u\n", mode);
		return -EINVAL;
	}

	elan_sensor_restart_work(epld);
	return count;
}

static ssize_t elan_ls_operationmode_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct elan_epl_data *epld = dev_get_drvdata(dev);
	u16 mode = 0;

	if (epld->enable_pflag == 0 && epld->enable_lflag == 0)
		mode = 0;
	else if (epld->enable_pflag == 0 && epld->enable_lflag == 1)
		mode = 1;
	else if (epld->enable_pflag == 1 && epld->enable_lflag == 0)
		mode = 2;
	else if (epld->enable_pflag == 1 && epld->enable_lflag == 1)
		mode = 3;

	return scnprintf(buf, PAGE_SIZE, "%u\n", mode);
}

static ssize_t elan_ls_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct elan_epl_data *epld = dev_get_drvdata(dev);
	u16 ch1 = 0;

	mutex_lock(&epld->data_mutex);
	elan_sensor_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_DATA_LOCK);

	elan_sensor_I2C_Write(epld->client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	if (!elan_sensor_I2C_Read(epld->client, epld->raw_data.raw_bytes, 2))
		ch1 = (epld->raw_data.raw_bytes[1] << 8) |
		      epld->raw_data.raw_bytes[0];

	elan_sensor_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_DATA_UNLOCK);
	mutex_unlock(&epld->data_mutex);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ch1);
}

static ssize_t epl2182_show_renvo(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct elan_epl_data *epld = dev_get_drvdata(dev);
	u16 renvo;

	mutex_lock(&epld->data_mutex);
	renvo = epld->raw_data.renvo;
	mutex_unlock(&epld->data_mutex);

	return scnprintf(buf, PAGE_SIZE, "%x\n", renvo);
}

static ssize_t epl2182_store_hs_enable(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct elan_epl_data *epld = dev_get_drvdata(dev);
	bool enable_als = epld->enable_lflag && !epld->l_suspend;
	u16 mode;

	if (kstrtou16(buf, 0, &mode))
		return -EINVAL;

	if (mode) {
		if (enable_als) {
			epld->l_suspend = 1;
			hs_als_flag = 1;
			msleep(ALS_DELAY);
		}
		epld->enable_hflag = 1;
		hs_idx = 0;
		hs_count = 0;
		epl2182_hs_enable(epld, true, true);
	} else {
		epld->enable_hflag = 0;
		if (hs_als_flag == 1) {
			epld->l_suspend = 0;
			hs_als_flag = 0;
		}
		elan_sensor_lsensor_enable(epld);
	}

	return count;
}

static ssize_t epl2182_show_hs_raws(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct elan_epl_data *epld = dev_get_drvdata(dev);
	int i, start, max_frame;
	ssize_t len = 0;
	u16 count;

	max_frame = ARRAY_SIZE(epld->raw_data.hs_data);

	mutex_lock(&epld->data_mutex);

	count = hs_count;
	start = hs_idx;
	len += scnprintf(buf + len, PAGE_SIZE - len, "%u", count);

	if (count == 0 && show_hs_raws_flag) {
		len += scnprintf(buf + len, PAGE_SIZE - len, " 0");
		show_hs_raws_flag = 0;
	} else if (count > 0) {
		for (i = 0; i < count; i++) {
			int current_idx = (start + i) % max_frame;
			len += scnprintf(buf + len, PAGE_SIZE - len, " %u",
					epld->raw_data.hs_data[current_idx]);
		}
	} else if (count == 0) {
		len += scnprintf(buf + len, PAGE_SIZE - len, " 0");
		show_hs_raws_flag = 1;
	}

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

	hs_count = 0;
	hs_idx = 0;

	mutex_unlock(&epld->data_mutex);

	return len;
}

static DEVICE_ATTR(elan_ls_operationmode, S_IROTH | S_IWOTH,
		   elan_ls_operationmode_show, elan_ls_operationmode_store);
static DEVICE_ATTR(elan_ls_status, S_IROTH | S_IWOTH, elan_ls_status_show,
		   NULL);
static DEVICE_ATTR(elan_renvo, S_IROTH | S_IWOTH, epl2182_show_renvo,
		   NULL);
static DEVICE_ATTR(hs_enable, S_IROTH | S_IWOTH, NULL,
		   epl2182_store_hs_enable);
static DEVICE_ATTR(hs_raws, S_IROTH | S_IWOTH, epl2182_show_hs_raws,
		   NULL);

static struct attribute *ets_attributes[] = {
	&dev_attr_elan_ls_operationmode.attr,
	&dev_attr_elan_ls_status.attr,
	&dev_attr_elan_renvo.attr,
	&dev_attr_hs_enable.attr,
	&dev_attr_hs_raws.attr,
	NULL,
};

static const struct attribute_group ets_attr_group = {
	.attrs = ets_attributes,
};

static int elan_als_open(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data_ptr;

	if (!epld)
		return -ENODEV;

	if (epld->als_opened)
		return -EBUSY;

	epld->als_opened = 1;
	return 0;
}

static int elan_als_release(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data_ptr;

	if (!epld)
		return -ENODEV;

	epld->als_opened = 0;
	return 0;
}

static long elan_als_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct elan_epl_data *epld = epl_data_ptr;
	int flag;
	unsigned long buf;

	void __user *argp = (void __user *)arg;

	if (!epld)
		return -ENODEV;

	switch (cmd) {
	case ELAN_EPL6800_IOCTL_GET_LFLAG:
		flag = epld->enable_lflag;
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case ELAN_EPL6800_IOCTL_ENABLE_LFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		epld->enable_lflag = flag;
		elan_sensor_restart_work(epld);
		break;
	case ELAN_EPL6800_IOCTL_GETDATA:
		mutex_lock(&epld->data_mutex);
		buf = (unsigned long)epld->raw_data.als_ch1_raw;
		mutex_unlock(&epld->data_mutex);
		if (copy_to_user(argp, &buf, sizeof(buf)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations elan_als_fops = {
	.owner = THIS_MODULE,
	.open = elan_als_open,
	.release = elan_als_release,
	.unlocked_ioctl = elan_als_ioctl,
};

static struct miscdevice elan_als_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "elan_als",
	.fops = &elan_als_fops,
};

#ifndef NO_P_SENSOR
static int elan_ps_open(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data_ptr;

	if (!epld)
		return -ENODEV;

	if (epld->ps_opened)
		return -EBUSY;
	epld->ps_opened = 1;
	return 0;
}

static int elan_ps_release(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data_ptr;

	if (!epld)
		return -ENODEV;

	epld->ps_opened = 0;
	return 0;
}

static long elan_ps_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct elan_epl_data *epld = epl_data_ptr;
	int value;
	int flag;
	void __user *argp = (void __user *)arg;

	if (!epld)
		return -ENODEV;

	switch (cmd) {
	case ELAN_EPL6800_IOCTL_GET_PFLAG:
		flag = epld->enable_pflag;
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case ELAN_EPL6800_IOCTL_ENABLE_PFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		epld->enable_pflag = flag;
		elan_sensor_restart_work(epld);
		break;
	case ELAN_EPL6800_IOCTL_GETDATA:
		mutex_lock(&epld->data_mutex);
		value = epld->raw_data.ps_ch1_raw;
		mutex_unlock(&epld->data_mutex);
		if (copy_to_user(argp, &value, sizeof(value)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations elan_ps_fops = {
	.owner = THIS_MODULE,
	.open = elan_ps_open,
	.release = elan_ps_release,
	.unlocked_ioctl = elan_ps_ioctl
};

static struct miscdevice elan_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "elan_ps",
	.fops = &elan_ps_fops
};
#endif

static int initial_sensor(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;
	int ret;

	ret = elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 1);
	if (ret < 0)
		return -EIO;

	elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02,
			      EPL_S_SENSING_MODE);
	elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
			      EPL_INT_DISABLE);
	set_psensor_intr_threshold(epld, epld->ps_th_l, epld->ps_th_h);

	msleep(20);

	epld->enable_lflag = 0;
	epld->enable_pflag = 0;

	return 0;
}

static int elan_power_on(struct elan_epl_data *epld, bool on)
{
	int rc;
	struct device *dev = &epld->client->dev;

	if (!on) {
		regulator_disable(epld->vio);
		regulator_disable(epld->vdd);
		return 0;
	}

	rc = regulator_enable(epld->vdd);
	if (rc) {
		dev_err(dev, "Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(epld->vio);
	if (rc) {
		dev_err(dev, "Regulator vio enable failed rc=%d\n", rc);
		regulator_disable(epld->vdd);
	}
	return rc;
}

static int elan_power_init(struct elan_epl_data *epld, bool on)
{
	int rc;
	struct device *dev = &epld->client->dev;

	if (!on) {
		regulator_put(epld->vio);
		regulator_put(epld->vdd);
		return 0;
	}

	epld->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(epld->vdd)) {
		rc = PTR_ERR(epld->vdd);
		dev_err(dev, "Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	epld->vio = regulator_get(dev, "vio");
	if (IS_ERR(epld->vio)) {
		rc = PTR_ERR(epld->vio);
		dev_err(dev, "Regulator get failed vio rc=%d\n", rc);
		regulator_put(epld->vdd);
		return rc;
	}
	return 0;
}

static int lightsensor_setup(struct elan_epl_data *epld)
{
	int err;

	epld->als_input_dev = input_allocate_device();
	if (!epld->als_input_dev)
		return -ENOMEM;

	epld->als_input_dev->name = "light";
	set_bit(EV_ABS, epld->als_input_dev->evbit);
	input_set_abs_params(epld->als_input_dev, ABS_MISC, 0, 9, 0, 0);

	err = input_register_device(epld->als_input_dev);
	if (err) {
		input_free_device(epld->als_input_dev);
		return err;
	}

	err = misc_register(&elan_als_device);
	if (err)
		input_unregister_device(epld->als_input_dev);

	return err;
}

#ifndef NO_P_SENSOR
static int psensor_setup(struct elan_epl_data *epld)
{
	int err;

	epld->ps_input_dev = input_allocate_device();
	if (!epld->ps_input_dev)
		return -ENOMEM;

	epld->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, epld->ps_input_dev->evbit);
	input_set_abs_params(epld->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	err = input_register_device(epld->ps_input_dev);
	if (err) {
		input_free_device(epld->ps_input_dev);
		return err;
	}

	err = misc_register(&elan_ps_device);
	if (err)
		input_unregister_device(epld->ps_input_dev);

	return err;
}
#endif

#if PS_INTERRUPT_MODE
static int setup_interrupt(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;
	int err;

	if (gpio_is_valid(epld->irq_gpio)) {
		err = gpio_request(epld->irq_gpio, "elan_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed\n");
			return err;
		}
		err = gpio_direction_input(epld->irq_gpio);
		if (err) {
			dev_err(&client->dev, "set_direction for irq gpio failed\n");
			gpio_free(epld->irq_gpio);
			return err;
		}
	}

	err = request_irq(client->irq, elan_sensor_irq_handler,
			  IRQF_TRIGGER_FALLING, ELAN_LS_2182, client);
	if (err) {
		dev_err(&client->dev, "request_irq failed\n");
		if (gpio_is_valid(epld->irq_gpio))
			gpio_free(epld->irq_gpio);
		return err;
	}
	irq_set_irq_wake(client->irq, 1);
	return 0;
}
#endif

static int elan_enable_als_sensor(struct i2c_client *client, int val)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	if (val < 0 || val > 1)
		return -EINVAL;

	if (epld->enable_lflag != val) {
		epld->enable_lflag = val;
		elan_sensor_restart_work(epld);
	}
	return 0;
}

#ifndef NO_P_SENSOR
static int elan_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	if (val < 0 || val > 1)
		return -EINVAL;

	if (epld->enable_pflag != val) {
		epld->enable_pflag = val;
		elan_sensor_restart_work(epld);
	}
	return 0;
}
#endif

static int elan_als_set_enable(struct sensors_classdev *sensors_cdev,
			       unsigned int enable)
{
	struct elan_epl_data *epld =
		container_of(sensors_cdev, struct elan_epl_data, als_cdev);
	return elan_enable_als_sensor(epld->client, enable);
}

#ifndef NO_P_SENSOR
static int elan_ps_set_enable(struct sensors_classdev *sensors_cdev,
			      unsigned int enable)
{
	struct elan_epl_data *epld =
		container_of(sensors_cdev, struct elan_epl_data, ps_cdev);

#if PS_AUTO_ENABLE
	if (enable)
		epld->raw_data.ps_min_raw = 0xffff;
#endif

	return elan_enable_ps_sensor(epld->client, enable);
}
#endif

static ssize_t elan_als_poll_delay(struct sensors_classdev *sensors_cdev,
				   unsigned int delay_msec)
{
	struct elan_epl_data *epld =
		container_of(sensors_cdev, struct elan_epl_data, als_cdev);

	if (delay_msec < ALS_POLLING_RATE / 2)
		delay_msec = ALS_POLLING_RATE / 2;

	epld->als_poll_delay = delay_msec;

	if (epld->enable_lflag)
		elan_sensor_restart_work(epld);

	return 0;
}

#ifndef NO_P_SENSOR
static ssize_t elan_ps_poll_delay(struct sensors_classdev *sensors_cdev,
				  unsigned int delay_msec)
{
	struct elan_epl_data *epld =
		container_of(sensors_cdev, struct elan_epl_data, ps_cdev);

	if (delay_msec < PS_POLLING_RATE)
		delay_msec = PS_POLLING_RATE;

	epld->ps_poll_delay = delay_msec;

	if (epld->enable_pflag)
		elan_sensor_restart_work(epld);

	return 0;
}
#endif

static int elan_sensor_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	if (!epld->enable_pflag) {
		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_C_P_DOWN);
		cancel_delayed_work_sync(&epld->polling_work);
	}
	return 0;
}

static int elan_sensor_resume(struct i2c_client *client)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	if (epld->enable_pflag || epld->enable_lflag)
		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_C_P_UP);

	if (epld->enable_pflag)
		elan_sensor_restart_work(epld);

	return 0;
}

static int sensor_parse_dt(struct device *dev, struct elan_epl_data *epld)
{
	struct device_node *np = dev->of_node;
#ifndef NO_P_SENSOR
	int rc;
	u32 tmp;
#endif

	epld->irq_gpio = of_get_named_gpio_flags(np, "epl2182,irq-gpio", 0,
						 &epld->irq_gpio_flags);
	if (epld->irq_gpio < 0)
		return epld->irq_gpio;
#ifndef NO_P_SENSOR
	rc = of_property_read_u32(np, "epl2182,prox_th_min", &tmp);
	if (rc) {
		dev_warn(dev, "Unable to read prox_th_min, using default\n");
		epld->ps_th_l = PS_L_THRESHOLD;
	} else {
		epld->ps_th_l = tmp;
	}

	rc = of_property_read_u32(np, "epl2182,prox_th_max", &tmp);
	if (rc) {
		dev_warn(dev, "Unable to read prox_th_max, using default\n");
		epld->ps_th_h = PS_H_THRESHOLD;
	} else {
		epld->ps_th_h = tmp;
	}
#endif
	return 0;
}

static int elan_sensor_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct elan_epl_data *epld;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check failed\n");
		return -EOPNOTSUPP;
	}

	epld = devm_kzalloc(&client->dev, sizeof(*epld), GFP_KERNEL);
	if (!epld)
		return -ENOMEM;

	epld->client = client;
	i2c_set_clientdata(client, epld);

	err = sensor_parse_dt(&client->dev, epld);
	if (err)
		return err;

	err = elan_power_init(epld, true);
	if (err)
		return err;

	err = elan_power_on(epld, true);
	if (err)
		goto err_power_deinit;

	mutex_init(&epld->data_mutex);
	elan_sensor_I2C_Write(client, REG_19, R_TWO_BYTE, 0x01, 0x00);
	if (!elan_sensor_I2C_Read(client, epld->raw_data.raw_bytes, 2))
		epld->raw_data.renvo = (epld->raw_data.raw_bytes[1] << 8) |
				       epld->raw_data.raw_bytes[0];

	epld->epl_wq = create_singlethread_workqueue("elan_sensor_wq");
	if (!epld->epl_wq) {
		err = -ENOMEM;
		goto err_power_off;
	}

#if PS_INTERRUPT_MODE
	INIT_WORK(&epld->irq_work, epl_sensor_irq_do_work);
#endif
	INIT_DELAYED_WORK(&epld->report_polling_work, report_polling_do_work);
	INIT_DELAYED_WORK(&epld->polling_work, polling_do_work);

	epld->als_poll_delay = ALS_POLLING_RATE;
#ifndef NO_P_SENSOR
	epld->ps_poll_delay = PS_POLLING_RATE;
#endif
	epld->l_suspend = 0;

	err = lightsensor_setup(epld);
	if (err)
		goto err_destroy_wq;

#ifndef NO_P_SENSOR
	err = psensor_setup(epld);
	if (err)
		goto err_misc_als_deregister;
#endif

	err = initial_sensor(epld);
	if (err)
		goto err_misc_ps_deregister;

#if PS_INTERRUPT_MODE
	err = setup_interrupt(epld);
	if (err)
		goto err_misc_ps_deregister;
#endif

	wake_lock_init(&epld->ps_wlock, WAKE_LOCK_SUSPEND, "ps_wakelock");

	err = sysfs_create_group(&client->dev.kobj, &ets_attr_group);
	if (err)
		goto err_free_irq;

	epld->als_cdev = sensors_light_cdev;
	epld->als_cdev.sensors_enable = elan_als_set_enable;
	epld->als_cdev.sensors_poll_delay = elan_als_poll_delay;

	err = sensors_classdev_register(&client->dev, &epld->als_cdev);
	if (err)
		goto err_remove_sysfs;

#ifndef NO_P_SENSOR
	epld->ps_cdev = sensors_proximity_cdev;
	epld->ps_cdev.sensors_enable = elan_ps_set_enable;
	epld->ps_cdev.sensors_poll_delay = elan_ps_poll_delay;

	err = sensors_classdev_register(&client->dev, &epld->ps_cdev);
	if (err)
		goto err_unregister_als_class;
#endif

	epl_data_ptr = epld;
	dev_info(&client->dev, "sensor probe success.\n");
	return 0;

#ifndef NO_P_SENSOR
err_unregister_als_class:
	sensors_classdev_unregister(&epld->als_cdev);
#endif
err_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &ets_attr_group);
err_free_irq:
#if PS_INTERRUPT_MODE
	free_irq(client->irq, client);
	if (gpio_is_valid(epld->irq_gpio))
		gpio_free(epld->irq_gpio);
#endif
err_misc_ps_deregister:
#ifndef NO_P_SENSOR
	misc_deregister(&elan_ps_device);
	input_unregister_device(epld->ps_input_dev);
#endif
#ifndef NO_P_SENSOR
err_misc_als_deregister:
#endif
	misc_deregister(&elan_als_device);
	input_unregister_device(epld->als_input_dev);
err_destroy_wq:
	destroy_workqueue(epld->epl_wq);
err_power_off:
	elan_power_on(epld, false);
err_power_deinit:
	elan_power_init(epld, false);
	return err;
}

static int elan_sensor_remove(struct i2c_client *client)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	epl_data_ptr = NULL;
	sensors_classdev_unregister(&epld->als_cdev);
#ifndef NO_P_SENSOR
	sensors_classdev_unregister(&epld->ps_cdev);
#endif
	sysfs_remove_group(&client->dev.kobj, &ets_attr_group);
	wake_lock_destroy(&epld->ps_wlock);
#if PS_INTERRUPT_MODE
	free_irq(client->irq, client);
	if (gpio_is_valid(epld->irq_gpio))
		gpio_free(epld->irq_gpio);
#endif
	destroy_workqueue(epld->epl_wq);
	misc_deregister(&elan_als_device);
#ifndef NO_P_SENSOR
	misc_deregister(&elan_ps_device);
#endif
	input_unregister_device(epld->als_input_dev);
#ifndef NO_P_SENSOR
	input_unregister_device(epld->ps_input_dev);
#endif
	elan_power_on(epld, false);
	elan_power_init(epld, false);

	return 0;
}

static const struct i2c_device_id elan_sensor_id[] = {
	{ ELAN_LS_2182, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, elan_sensor_id);

static const struct of_device_id elan_match_table[] = {
	{ .compatible = "elan,epl2182" },
	{},
};
MODULE_DEVICE_TABLE(of, elan_match_table);

static struct i2c_driver elan_sensor_driver = {
	.driver = {
		.name = ELAN_LS_2182,
		.of_match_table = of_match_ptr(elan_match_table),
	},
	.probe = elan_sensor_probe,
	.remove = elan_sensor_remove,
	.suspend = elan_sensor_suspend,
	.resume = elan_sensor_resume,
	.id_table = elan_sensor_id,
};

module_i2c_driver(elan_sensor_driver);

MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl2182 driver");
MODULE_LICENSE("GPL v2");
