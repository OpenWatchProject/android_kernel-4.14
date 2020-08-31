#ifndef _BMA4XY_DRIVER_H_
#define _BMA4XY_DRIVER_H_
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
//#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/firmware.h>

#include <cust_acc.h>
#include <accel.h>

#include "bma4_defs.h"
#include "bma4.h"
#include "bma423.h"

#define SENSOR_NAME "bma4xy_acc"

//#define BMA4XY_ENABLE_INT
#define BMA4XY_INT_PIN 63
#define DEBUG

struct bma4xy_data {
	struct bma4_dev device;
	struct i2c_client *client;
	struct acc_hw hw;
	/* misc */
	atomic_t trace;
	atomic_t suspend;
	atomic_t filter;
	struct hwmsen_convert cvt;
	/* data */
	struct input_dev *acc_input;
	struct input_dev *uc_input;
	uint8_t acc_pm;
	uint8_t acc_odr;
	int irq;
	struct work_struct irq_work;
	uint16_t fw_version;
	uint8_t config_stream_choose;
	char *config_stream_name;
	unsigned long config_stream_size;
	int reg_sel;
	int reg_len;
	//struct wake_lock wakelock;
	struct delayed_work delay_work_sig;
	atomic_t in_suspend;
	uint8_t tap_type;
	uint8_t selftest;
	uint8_t sigmotion_enable;
	uint8_t stepdet_enable;
	uint8_t stepcounter_enable;
	uint8_t tilt_enable;
	uint8_t pickup_enable;
	uint8_t glance_enable;
	uint8_t wakeup_enable;
	uint8_t anymotion_enable;
	uint8_t nomotion_enable;
	uint8_t orientation_enable;
	uint8_t flat_enable;
	uint8_t tap_enable;
	uint8_t highg_enable;
	uint8_t lowg_enable;
	uint8_t activity_enable;
	uint8_t err_int_trigger_num;
	uint32_t step_counter_val;
	uint32_t step_counter_temp;
	uint8_t any_motion_axis;
	uint8_t no_motion_axis;
	uint8_t foc;
	uint8_t change_step_par;
	uint8_t	wrist_wear;
	uint8_t single_tap;
	uint8_t double_tap;
};

#endif
