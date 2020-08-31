#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>

#include <cust_acc.h>
#include <accel.h>

#include "bma4xy_driver.h"
#include "bma4xy_i2c.h"

//#include <step_counter.h>
//#include <wake_gesture.h>
//#include <tilt_detector.h>

static struct bma4xy_data *bma4xy_data;

//#define BMA4_STEP_COUNTER
//#define BMA4_WAKEUP
//#define BMA4_TILT

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bma4xy_i2c_remove(struct i2c_client *client);
#ifdef CONFIG_PM_SLEEP
static int bma4xy_i2c_suspend(struct device *dev);
static int bma4xy_i2c_resume(struct device *dev);
#endif
static int bma4xy_acc_init(void);
static int bma4xy_acc_uninit(void);
static int gsensor_set_delay(uint64_t ns);

#define G_RANGE 4
#define SENSITIVITY ((1 << 12) / 2)

#define BMA4XY_TAG "[BMA4xy] "
#ifdef DEBUG
#define BMA4XY_FUN() pr_err(BMA4XY_TAG "%s\n", __func__)
#define BMA4XY_ERR(fmt, args...) pr_err(BMA4XY_TAG "%s %d: " fmt, __func__, __LINE__, ##args)
#define BMA4XY_LOG(fmt, args...) pr_err(BMA4XY_TAG fmt, ##args)
#define BMA4XY_DBG(fmt, args...) pr_err(BMA4XY_TAG fmt, ##args)
#else
#define BMA4XY_FUN() pr_debug(BMA4XY_TAG "%s\n", __func__)
#define BMA4XY_ERR(fmt, args...) pr_err(BMA4XY_TAG "%s %d: " fmt, __func__, __LINE__, ##args)
#define BMA4XY_LOG(fmt, args...) pr_info(BMA4XY_TAG fmt, ##args)
#define BMA4XY_DBG(fmt, args...) pr_debug(BMA4XY_TAG fmt, ##args)
#endif

//static int remap_dir = 0;
static bool enable_status;
static bool sensor_power = true;
static int sensor_suspend;
static DEFINE_MUTEX(gsensor_mutex);

static void bma4xy_i2c_delay_us(uint32_t period_us, void *intf_ptr)
{
	usleep_range(period_us, period_us + 1000);
}

static BMA4_INTF_RET_TYPE bma4xy_i2c_read_wrapper(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
	return bma4xy_i2c_read(bma4xy_data->client, reg_addr, data, len);
}

static BMA4_INTF_RET_TYPE bma4xy_i2c_write_wrapper(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
	return bma4xy_i2c_write(bma4xy_data->client, reg_addr, data, len);
}

static int bma4xy_set_power_mode(bool enable)
{
	int err = 0;

	if (enable == 0 &&
	    bma4xy_data->sigmotion_enable == 0 &&
	    bma4xy_data->stepdet_enable == 0 &&
	    bma4xy_data->stepcounter_enable == 0 &&
	    bma4xy_data->tilt_enable == 0 &&
	    bma4xy_data->wrist_wear == 0 &&
	    bma4xy_data->pickup_enable == 0 &&
	    bma4xy_data->glance_enable == 0 &&
	    bma4xy_data->wakeup_enable == 0) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &bma4xy_data->device);
		if (err) {
			BMA4XY_ERR("bma4_set_accel_enable failed: %d\n", err);
			return err;
		}

		BMA4XY_LOG("bma4xy disabled!\n");
	} else if (enable == 1) {
		err = bma4_set_accel_enable(BMA4_ENABLE, &bma4xy_data->device);
		if (err) {
			BMA4XY_ERR("bma4_set_accel_enable failed: %d\n", err);
			return err;
		}
		BMA4XY_LOG("bma4xy enabled!\n");
	}

	sensor_power = enable;
	bma4xy_data->acc_pm = enable;
	bma4xy_i2c_delay_us(5, NULL);

	BMA4XY_LOG("bma4xy_set_power_mode finished! sensor_power = %d\n", sensor_power);

	return err;
}

static int bma4xy_set_tilt_remap()
{
	int err;
	struct bma423_axes_remap remap_data;
	struct hwmsen_convert *cvt = &bma4xy_data->cvt;

	remap_data.x_axis = cvt->map[0];
	remap_data.x_axis_sign = cvt->sign[0] < 0 ? 1 : 0;
	remap_data.y_axis = cvt->map[1];
	remap_data.y_axis_sign = cvt->sign[1] < 0 ? 1 : 0;
	remap_data.z_axis = cvt->map[2];
	remap_data.z_axis_sign = cvt->sign[2] < 0 ? 1 : 0;

	err = bma423_set_remap_axes(&remap_data, &bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma423_set_remap_axes failed: %d\n", err);
		return err;
	}

	return 0;
}

#ifdef BMA4XY_ENABLE_INT
static void bma4xy_uc_function_handle(uint8_t status)
{
#ifdef BMA4_STEP_COUNTER
	if (status & SIG_MOTION_OUT == 0x01)
		step_notify(TYPE_SIGNIFICANT);
	if (status & STEP_DET_OUT == 0x02)
		step_notify(TYPE_STEP_DETECTOR);
#endif
#ifdef BMA4_TILT
	if (status & TILT_OUT == 0x08) {
		tilt_notify();
	}
#endif
#ifdef BMA4_WAKEUP
	if ((status & WAKEUP_OUT) == 0x20)
		wag_notify();
#endif
}

static void bma4xy_irq_work_func(struct work_struct *work)
{
	unsigned char int_status[2] = {0, 0};
	int err = 0;
	int in_suspend_copy;

	//in_suspend_copy = atomic_read(&bma4xy_data->in_suspend);
	//tilt_notify();
	/*read the interrut status two register*/
	err = bma4xy_data->device.bus_read(BMA4_INT_STAT_0_ADDR, int_status, 2, NULL);
	if (err) {
		BMA4XY_ERR("[%s] bus_read BMA4xy stat reg fail\n", __func__);
		return;
	}
	BMA4XY_LOG("int_status0 = 0x%x int_status1 =0x%x",
	           int_status[0], int_status[1]);

	return ;   //LSQ add
	if (in_suspend_copy &&
	    ((int_status[0] & STEP_DET_OUT) == 0x02)) {
		return;
	}
	if (int_status[0])
		bma4xy_uc_function_handle(bma4xy_data, (uint8_t)int_status[0]);

}

static void bma4xy_delay_sigmo_work_func(struct work_struct *work)
{
	unsigned char int_status[2] = {0, 0};
	int err = 0;
	/*read the interrut status two register*/
	err = bma4xy_data->device.bus_read(bma4xy_data->device.dev_addr,
	                                   BMA4_INT_STAT_0_ADDR, int_status, 2);
	if (err)
		return;
	BMA4XY_LOG("int_status0 = %x int_status1 =%x",
	           int_status[0], int_status[1]);
#ifdef BMA4_STEP_COUNTER
	if ((int_status[0] & SIG_MOTION_OUT) == 0x01)
		step_notify(TYPE_SIGNIFICANT);
#endif
}

static irqreturn_t bma4xy_irq_handle(int irq, void *handle)
{
#if 0  //LSQ mask these code
	int in_suspend_copy;
	BMA4XY_LOG("[%s] and tilt_enable:[%d]   wakeup_enable:[%d]\n", __func__, bma4xy_data->tilt_enable, bma4xy_data->wakeup_enable);
	in_suspend_copy = atomic_read(&bma4xy_data->in_suspend);
	BMA4XY_LOG("[%s] and is_suspend_copy:[%d]\n", __func__, in_suspend_copy);

	/*this only deal with SIG_motion CTS test*/
	if ((in_suspend_copy == 1) &&
	    ((bma4xy_data->sigmotion_enable == 1) &&
	     (bma4xy_data->tilt_enable != 1) &&
	     (bma4xy_data->pickup_enable != 1) &&
	     (bma4xy_data->glance_enable != 1) &&
	     (bma4xy_data->wakeup_enable != 1))) {
		wake_lock_timeout(&bma4xy_data->wakelock, HZ);
		schedule_delayed_work(&bma4xy_data->delay_work_sig,
		                      msecs_to_jiffies(50));
	} else if ((in_suspend_copy == 1) &&
	           ((bma4xy_data->sigmotion_enable == 1) ||
	            (bma4xy_data->tilt_enable == 1) ||
	            (bma4xy_data->pickup_enable == 1) ||
	            (bma4xy_data->glance_enable == 1) ||
	            (bma4xy_data->wakeup_enable == 1))) {
		wake_lock_timeout(&bma4xy_data->wakelock, HZ);
		schedule_work(&bma4xy_data->irq_work);
	} else
		schedule_work(&bma4xy_data->irq_work);
#else
	schedule_work(&bma4xy_data->irq_work);
#endif
	return IRQ_HANDLED;
}

static int bma4xy_request_irq()
{
	int err = 0;	

#if 1
	err = gpio_request_one(BMA4XY_INT_PIN, GPIOF_IN, "bma4xy_interrupt");
	if (err < 0)
		return err;

	err = gpio_direction_input(BMA4XY_INT_PIN);
	if (err < 0)
		return err;

	bma4xy_data->irq = gpio_to_irq(BMA4XY_INT_PIN);

	err = request_irq(bma4xy_data->irq, bma4xy_irq_handle, IRQF_TRIGGER_RISING, SENSOR_NAME, bma4xy_data);
	if (err < 0)
		return err;
	INIT_WORK(&bma4xy_data->irq_work, bma4xy_irq_work_func);

	return err;
#else
	struct device_node *node = NULL;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	int ret = 0;

	accelPltFmDev = get_accel_platformdev();
	if (NULL != accelPltFmDev) {
		pinctrl = devm_pinctrl_get(&accelPltFmDev->dev);
		if (IS_ERR(pinctrl)) {
			ret = PTR_ERR(pinctrl);
			printk("Cannot find accel pinctrl!\n");
		}
		pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
		if (IS_ERR(pins_default)) {
			ret = PTR_ERR(pins_default);
			printk("Cannot find accel pinctrl default!\n");

		}

		pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
		if (IS_ERR(pins_cfg)) {
			ret = PTR_ERR(pins_cfg);
			printk("Cannot find accel pinctrl pin_cfg!\n");
		} else {
			printk("find out accel pinctrl pin_cfg!\n");
		}

		pinctrl_select_state(pinctrl, pins_cfg);
	} else {
		printk("Cannot find accel platform dev!\n");
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,gse_1-eint");
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		bma4xy_data->irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(bma4xy_data->irq, bma4xy_irq_handle,
		                  IRQF_TRIGGER_RISING ,
		                  SENSOR_NAME, bma4xy_data);
		if (ret > 0)
			printk(KERN_ERR " [bma4xy_request_irq]  bma4xy request_irq failed\n");
		else
			printk(KERN_ERR " [bma4xy_request_irq]  bma4xy request_irq sucess\n");
	} else {
		printk(KERN_INFO "[%s] can not find!", __func__);
	}
	printk(KERN_INFO "[bma4xy_request_irq] irq_num= %d IRQ_num=%d \n",
	       bma4xy_data->gpio_pin, bma4xy_data->irq);
	INIT_WORK(&bma4xy_data->irq_work, bma4xy_irq_work_func);
	INIT_DELAYED_WORK(&bma4xy_data->delay_work_sig,
	                  bma4xy_delay_sigmo_work_func);
#endif
	return err;
}
#endif

static int bma4xy_init_client()
{
	int err = 0;
	struct bma4_accel_config accel_conf;

	BMA4XY_LOG("bma4xy_init_client started!\n");

	err = bma423_init(&bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma4xy_init failed: %d\n", err);
		return err;
	}

	err = bma423_write_config_file(&bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma4xy_write_config_file failed: %d\n", err);
		return err;
	}

	err = bma4xy_set_power_mode(0);
	if (err) {
		BMA4XY_ERR("bma4xy_set_power_mode failed: %d\n", err);
		return err;
	}

	accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
#if (G_RANGE == 2)
	accel_conf.range = BMA4_ACCEL_RANGE_2G;
#elif (G_RANGE == 4)
	accel_conf.range = BMA4_ACCEL_RANGE_4G;
#elif (G_RANGE == 8)
	accel_conf.range = BMA4_ACCEL_RANGE_8G;
#elif (G_RANGE == 16)
	accel_conf.range = BMA4_ACCEL_RANGE_16G;
#endif
	accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
	accel_conf.perf_mode = BMA4_CONTINUOUS_MODE;

	err = bma4_set_accel_config(&accel_conf, &bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma4_set_accel_config failed: %d\n", err);
		return err;
	}

	err = bma4xy_set_tilt_remap();
	if (err)
		BMA4XY_ERR("bma4xy_set_tilt_remap failed: %d\n", err);

	BMA4XY_LOG("bma4xy_init_client finished!\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bma4xy_i2c_suspend(struct device *dev)
{
	BMA4XY_FUN();

	enable_irq_wake(bma4xy_data->irq);
	atomic_set(&bma4xy_data->in_suspend, 1);

	return 0;
}

static int bma4xy_i2c_resume(struct device *dev)
{
	BMA4XY_FUN();

	disable_irq_wake(bma4xy_data->irq);
	atomic_set(&bma4xy_data->in_suspend, 0);

	return 0;
}

static const struct dev_pm_ops bma4xy_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bma4xy_i2c_suspend, bma4xy_i2c_resume)
};
#endif

static int gsensor_open_report_data(int open)
{
	return 0;
}

static int gsensor_enable_nodata(int en)
{
	int err = 0;

	if ((en == 0 && sensor_power == false) ||
	    (en == 1 && sensor_power == true)) {
		enable_status = sensor_power;
	} else {
		enable_status = !sensor_power;

		if (atomic_read(&bma4xy_data->suspend) == 0) {
			err = bma4xy_set_power_mode(enable_status);
			if (err) {
				BMA4XY_ERR("gsensor_enable_nodata failed!\n");
				return err;
			}

			BMA4XY_LOG("bma4xy not in suspend. enable_status = %d\n", enable_status);
		} else {
			BMA4XY_LOG("bma4xy in suspend, can't enable/disable! enable_status = %d\n", enable_status);
		}
	}

	BMA4XY_LOG("gsensor_enable_nodata finished!\n");

	return 0;
}

static int gsensor_set_delay(uint64_t ns)
{
	int err = 0;
	int value;
	struct bma4_accel_config accel_conf;

	BMA4XY_LOG("gsensor_set_delay start!\n");

	err = bma4_get_accel_config(&accel_conf, &bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma4_get_accel_config failed: %d\n", err);
		return err;
	}

	value = ns / 1000 / 1000;
	if (value >= 80) {
		accel_conf.odr = BMA4_OUTPUT_DATA_RATE_12_5HZ;
	} else if (value >= 40) {
		accel_conf.odr = BMA4_OUTPUT_DATA_RATE_25HZ;
	} else if (value >= 20) {
		accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;
	} else if (value >= 10) {
		accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
	} else {
		accel_conf.odr = BMA4_OUTPUT_DATA_RATE_200HZ;
	}

	err = bma4_set_accel_config(&accel_conf, &bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma4_set_accel_config failed: %d\n", err);
		return err;
	}

	BMA4XY_LOG("gsensor_set_delay finished!\n");

	return 0;
}

static int gsensor_acc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return gsensor_set_delay(samplingPeriodNs);
}

static int gsensor_acc_flush(void)
{
	return acc_flush_report();
}

static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
	int err;
	struct bma4_accel accel;
	int raw_data[3] = {0};
	struct hwmsen_convert *cvt = &bma4xy_data->cvt;

	if (sensor_suspend == 1)
		return 0;

	mutex_lock(&gsensor_mutex);
	err = bma4_read_accel_xyz(&accel, &bma4xy_data->device);
	mutex_unlock(&gsensor_mutex);
	if (err) {
		BMA4XY_ERR("bma4_read_accel_xyz failed: %d\n", err);
		return err;
	}

	raw_data[0] = GRAVITY_EARTH_1000 * accel.x * G_RANGE / SENSITIVITY;
	raw_data[1] = GRAVITY_EARTH_1000 * accel.y * G_RANGE / SENSITIVITY;
	raw_data[2] = GRAVITY_EARTH_1000 * accel.z * G_RANGE / SENSITIVITY;

	*x = raw_data[cvt->map[0]] * cvt->sign[0];
	*y = raw_data[cvt->map[1]] * cvt->sign[1];
	*z = raw_data[cvt->map[2]] * cvt->sign[2];
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static struct acc_init_info bma4xy_init_info = {
	.name = SENSOR_NAME,
	.init = bma4xy_acc_init,
	.uninit = bma4xy_acc_uninit,
};

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct acc_control_path control_path = {0};
	struct acc_data_path data_path = {0};

	BMA4XY_LOG("bma4xy_i2c_probe started!\n");

	// Allocate memory for the bma4xy_data struct
	bma4xy_data = kzalloc(sizeof(struct bma4xy_data), GFP_KERNEL);
	if (bma4xy_data == NULL) {
		BMA4XY_ERR("kzalloc for bma4xy_data failed!");
		err = -ENOMEM;
		goto exit;
	}

	// Get acc configuration from DTS
	err = get_accel_dts_func(client->dev.of_node, &bma4xy_data->hw);
	if (err) {
		BMA4XY_ERR("get_accel_dts_func failed: %d\n", err);
		goto exit_err_clean;
	}

	err = hwmsen_get_convert(bma4xy_data->hw.direction, &bma4xy_data->cvt);
	if (err) {
		BMA4XY_ERR("Invalid direction: %d\n", bma4xy_data->hw.direction);
		goto exit_err_clean;
	}

	// Set I2C address (should be 0x18)
	client->addr = BMA4_I2C_ADDR_PRIMARY;
	i2c_set_clientdata(client, bma4xy_data);
	bma4xy_data->client = client;

	atomic_set(&bma4xy_data->trace, 0);
	atomic_set(&bma4xy_data->suspend, 0);

	// Set functions required by the Bosch API
	bma4xy_data->device.intf = BMA4_I2C_INTF;
	bma4xy_data->device.bus_read = bma4xy_i2c_read_wrapper;
	bma4xy_data->device.bus_write = bma4xy_i2c_write_wrapper;
	bma4xy_data->device.variant = BMA42X_VARIANT;
	bma4xy_data->device.intf_ptr = &client->addr;
	bma4xy_data->device.delay_us = bma4xy_i2c_delay_us;
	bma4xy_data->device.read_write_len = 8;

	//wake_lock_init(&bma4xy_data->wakelock, WAKE_LOCK_SUSPEND, "bma4xy");
	err = bma4xy_init_client();
	if (err)
		BMA4XY_ERR("bma4xy_init_client failed: %d\n", err);

	control_path.open_report_data = gsensor_open_report_data;
	control_path.enable_nodata = gsensor_enable_nodata;
	control_path.set_delay = gsensor_set_delay;
	control_path.is_report_input_direct = false;
	control_path.is_support_batch = false;
	control_path.batch = gsensor_acc_batch;
	control_path.flush = gsensor_acc_flush;
	err = acc_register_control_path(&control_path);
	if (err) {
		BMA4XY_ERR("acc_register_control_path failed: %d\n", err);
		goto exit_err_clean;
	}

	data_path.get_data = gsensor_get_data;
	data_path.vender_div = 1000;
	err = acc_register_data_path(&data_path);
	if (err) {
		BMA4XY_ERR("acc_register_data_path failed: %d\n", err);
		goto exit_err_clean;
	}

#ifdef BMA4XY_ENABLE_INT
	err = bma4xy_request_irq();
	if (err < 0)
		BMA4XY_ERR("bma4xy_request_irq failed: %d\n", err);
#endif

	BMA4XY_LOG("bma4xy_i2c_probe finished!\n");

exit:
	return err;

exit_err_clean:
	kfree(bma4xy_data);
	bma4xy_data = NULL;
	return err;
}

static int bma4xy_i2c_remove(struct i2c_client *client)
{
	kfree(bma4xy_data);
	bma4xy_data = NULL;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id acc_of_match[] = {
	{.compatible = "mediatek,bma4xy"},
	{.compatible = "mediatek,gsensor"},
	{}
};
MODULE_DEVICE_TABLE(i2c, acc_of_match);
#endif

static struct i2c_driver bma4xy_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = acc_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
		.pm = &bma4xy_pm_ops,
#endif
	},
	.probe = bma4xy_i2c_probe,
	.remove = bma4xy_i2c_remove,
};

static int bma4xy_acc_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&bma4xy_i2c_driver);
	if (ret) {
		BMA4XY_ERR("Error while adding I2C driver: %d\n", ret);
		goto exit;
	}

exit:
	return ret;
}

static int bma4xy_acc_uninit(void)
{
	i2c_del_driver(&bma4xy_i2c_driver);
	return 0;
}

static int __init bma4xy_init(void)
{
	int err = 0;

	// Register the accelerometer driver
	err = acc_driver_add(&bma4xy_init_info);
	if (err) {
		BMA4XY_ERR("acc_driver_add failed: %d\n", err);
		goto exit;
	}

#ifdef BMA4_STEP_COUNTER
	// Register the step counter driver
	err = step_c_driver_add(&bma4xy_stc_init_info);
	if (err) {
		BMA4XY_ERR("step_c_driver_add failed: %d\n", err);
		goto exit;
	}
#endif

#ifdef BMA4_WAKEUP
	// Register the wakeup sensor driver
	err = wag_driver_add(&bma4xy_wakeup_init_info);
	if (err) {
		BMA4XY_ERR("wag_driver_add failed: %d\n", err);
		goto exit;
	}
#endif

#ifdef BMA4_TILT
	// Register the tilt sensor driver
	err = tilt_driver_add(&bma4xy_tilt_init_info);
	if (err) {
		BMA4XY_ERR("tilt_driver_add failed: %d\n", err);
		goto exit;
	}
#endif

exit:
	return err;
}

static void __exit bma4xy_exit(void)
{
	// Nothing to do on exit
}

module_init(bma4xy_init);
module_exit(bma4xy_exit);

MODULE_AUTHOR("Iscle @ OpenWatch Project");
MODULE_DESCRIPTION("BMA4xy Acceleration Sensor Driver");
