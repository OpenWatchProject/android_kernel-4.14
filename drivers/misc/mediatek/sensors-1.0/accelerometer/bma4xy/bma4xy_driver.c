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

#ifdef BMA4_STEP_COUNTER
#include <step_counter.h>
#endif
#ifdef BMA4_WAKEUP
#include <wake_gesture.h>
#endif
#ifdef BMA4_TILT
#include <tilt_detector.h>
#endif

static int bma4xy_acc_init(void);
static int bma4xy_acc_uninit(void);

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

static struct bma4xy_data *bma4xy_data;
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

static int bma4xy_enable(uint8_t enable)
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

	BMA4XY_LOG("bma4xy_enable finished! enable = %d\n", enable);

	return err;
}

static int bma4xy_set_feature_remap()
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
static void bma4xy_uc_function_handle(uint8_t int_status_0)
{
#ifdef BMA4_STEP_COUNTER
	if (int_status_0 & BMA423_STEP_CNTR_INT)
		step_notify(TYPE_STEP_DETECTOR);
#endif

#ifdef BMA4_TILT
	if (int_status_0 & BMA423_WRIST_WEAR_INT)
		tilt_notify();
#endif

#ifdef BMA4_WAKEUP
	if (int_status_0 & BMA423_ANY_MOT_INT)
		wag_notify();
#endif
}

static void bma4xy_irq_work_func(struct work_struct *work)
{
	uint8_t int_status[2] = {0};
	int err = 0;

	err = bma4_read_regs(BMA4_INT_STAT_0_ADDR, int_status, 2, &bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma4_read_regs failed: %d\n", err);
		return;
	}

	BMA4XY_LOG("int_status_0 = 0x%X, int_status_1 = 0x%X\n", int_status[0], int_status[1]);

	bma4xy_uc_function_handle(int_status[0]);
}

static irqreturn_t bma4xy_irq_handler(int irq, void *handle)
{
	schedule_work(&bma4xy_data->irq_work);
	return IRQ_HANDLED;
}

static int bma4xy_request_irq()
{
	int err;
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,gse_1");
	if (node == NULL) {
		BMA4XY_ERR("of_find_compatible_node failed!\n");
		return -EINVAL;
	}

	BMA4XY_LOG("Found IRQ node!\n");

	bma4xy_data->irq = irq_of_parse_and_map(node, 0);
	if (!bma4xy_data->irq) {
		BMA4XY_ERR("irq_of_parse_and_map failed!\n");
		return -EINVAL;
	}

	err = request_irq(bma4xy_data->irq, bma4xy_irq_handler, IRQF_TRIGGER_RISING, "mediatek,gse_1", NULL);
	if (err) {
		BMA4XY_LOG("request_irq failed: %d\n", err);
		return err;
	}

	INIT_WORK(&bma4xy_data->irq_work, bma4xy_irq_work_func);

	BMA4XY_LOG("Request IRQ OK!\n");

	return 0;
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

	err = bma4xy_enable(0);
	if (err) {
		BMA4XY_ERR("bma4xy_enable failed: %d\n", err);
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

	err = bma4xy_set_feature_remap();
	if (err)
		BMA4XY_ERR("bma4xy_set_feature_remap failed: %d\n", err);

	BMA4XY_LOG("bma4xy_init_client finished!\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bma4xy_i2c_suspend(struct device * dev)
{
	BMA4XY_FUN();

	enable_irq_wake(bma4xy_data->irq);
	atomic_set(&bma4xy_data->in_suspend, 1);

	return 0;
}

static int bma4xy_i2c_resume(struct device * dev)
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

static int gsensor_enable_nodata(int en)
{
	int err = 0;

	if (atomic_read(&bma4xy_data->suspend) == 0) {
		err = bma4xy_enable(en);
		if (err) {
			BMA4XY_ERR("bma4xy_enable failed: %d\n", err);
			return err;
		}

		BMA4XY_LOG("bma4xy not suspended. en = %d\n", en);
	} else {
		BMA4XY_LOG("bma4xy is suspended, can't enable or disable it! en = %d\n", en);
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

static int bma4xy_i2c_probe(struct i2c_client * client, const struct i2c_device_id * id)
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

	// TODO: Remove temporary hack (tested on LEM12)
	bma4xy_data->hw.direction = 1;

	err = hwmsen_get_convert(bma4xy_data->hw.direction, &bma4xy_data->cvt);
	if (err) {
		BMA4XY_ERR("Invalid direction: %d\n", bma4xy_data->hw.direction);
		goto exit_err_clean;
	}

	// Set I2C address (should be 0x18)
	client->addr = BMA4_I2C_ADDR_PRIMARY;
	i2c_set_clientdata(client, bma4xy_data);
	bma4xy_data->client = client;

	atomic_set(&bma4xy_data->suspend, 0);

	// Set functions required by the Bosch API
	bma4xy_data->device.intf = BMA4_I2C_INTF;
	bma4xy_data->device.bus_read = bma4xy_i2c_read_wrapper;
	bma4xy_data->device.bus_write = bma4xy_i2c_write_wrapper;
	bma4xy_data->device.variant = BMA42X_VARIANT;
	bma4xy_data->device.intf_ptr = &client->addr;
	bma4xy_data->device.delay_us = bma4xy_i2c_delay_us;
	bma4xy_data->device.read_write_len = 8;

	err = bma4xy_init_client();
	if (err)
		BMA4XY_ERR("bma4xy_init_client failed: %d\n", err);

	control_path.enable_nodata = gsensor_enable_nodata;
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

static int bma4xy_i2c_remove(struct i2c_client * client)
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

#ifdef BMA4_STEP_COUNTER
static int bma4xy_step_c_open_report_data(int open)
{
	return 0;
}

static int bma4xy_step_c_set_delay(uint64_t delay)
{
	return 0;
}

static int bma4xy_setp_d_set_selay(uint64_t delay)
{
	return 0;
}

static int bma4xy_step_c_enable_nodata(int en)
{
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	BMA4XY_LOG("bma4xy_step_c_enable_nodata en = %d\n", en);

	if (en == 1) {
		err = bma4_set_advance_power_save(0, &client_data->device);
		bma4xy_i2c_delay(10);
		err += bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		bma4xy_i2c_delay(10);
	}
	if (err)
		BMA4XY_ERR("set acc_op_mode failed\n\n");

	if (bma423_feature_enable(
	        BMA423_STEP_CNTR, en, &client_data->device) < 0) {
		BMA4XY_ERR("set bma421 virtual error\n\n");
		return -EINVAL;
	}

	if ((en == 0) && (bma4xy_data->sigmotion_enable == 0) &&
	    (bma4xy_data->stepdet_enable == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		bma4xy_i2c_delay(10);
	}
	if (err)
		BMA4XY_ERR("set acc_op_mode failed\n\n");
	bma4xy_data->stepcounter_enable = en;
	return err;
}

static int bma4xy_step_c_enable_significant(int en)
{
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	BMA4XY_LOG("bma4xy_step_c_enable_significant en = %d", en);

	if (en == 1) {
		err = bma4_set_advance_power_save(0, &client_data->device);
		bma4xy_i2c_delay(10);
		err += bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		bma4xy_i2c_delay(10);
	}

	if (err)
		BMA4XY_ERR("set acc_op_mode failed\n");

	if ((en == 0) && (bma4xy_data->stepcounter_enable == 0) &&
	    (bma4xy_data->stepdet_enable == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		bma4xy_i2c_delay(10);
	}
	if (err)
		BMA4XY_ERR("set acc_op_mode failed\n");

	bma4xy_i2c_delay(10);

	bma4xy_data->sigmotion_enable = en;

	return err;

}

static int bma4xy_step_c_enable_step_detect(int enable)
{
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	BMA4XY_LOG("bma4xy_step_c_enable_step_detect en = %d", enable);
	if (enable == 1) {
		err = bma4_set_advance_power_save(0, &client_data->device);
		bma4xy_i2c_delay(10);
		err += bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		bma4xy_i2c_delay(10);
	}
	if (err)
		BMA4XY_ERR("set acc_op_mode failed\n");

	bma4xy_i2c_delay(10);

	if (bma423_step_detector_enable(enable, &client_data->device) < 0) {
		BMA4XY_ERR("set bma421 virtual error\n");
		return -EINVAL;
	}

	if ((enable == 0) && (bma4xy_data->sigmotion_enable == 0) &&
	    (bma4xy_data->stepcounter_enable == 0) &&
	    (bma4xy_data->wakeup_enable == 0) &&
	    (bma4xy_data->tilt_enable == 0) &&
	    (bma4xy_data->wrist_wear == 0) &&
	    (bma4xy_data->single_tap == 0) &&
	    (bma4xy_data->double_tap == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		bma4xy_i2c_delay(10);
	}
	if (err)
		BMA4XY_ERR("set acc_op_mode failed\n");
	bma4xy_data->stepdet_enable = enable;
	return err;
}

static int bma4xy_step_c_get_data(uint32_t *value, int *status)
{
	int err = 0;
	uint32_t step_counter_val = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma423_step_counter_output(
	          &step_counter_val, &client_data->device);

	if (err) {
		BMA4XY_ERR("read failed");
		return err;
	}
	*value = step_counter_val;
	*status = 1;
	BMA4XY_LOG("step_c_get_data = %d\n", (int)(*value));
	return err;
}

static int bma4xy_stc_get_data_significant(uint32_t *value, int *status)
{
	return 0;
}

static int bma4xy_stc_get_data_step_d(uint32_t *value, int *status)
{
	return 0;
}

static int bma4xy_floor_set_delay(uint64_t ns)
{
	return 0;
}

static int bma4xy_floor_c_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int bma4xy_floor_c_flush(void)
{
	return 0;
}

static int bma4xy_floor_enable(int en)
{
	return 0;
}
static int bma4xy_get_data_floor(uint32_t *value, int *status)
{
	return 0;
}

static int bma4xy_step_c_probe(void)
{
	int err = 0;
	struct step_c_control_path control_path = {0};
	struct step_c_data_path data_path = {0};

	BMA4XY_FUN();

	control_path.open_report_data = bma4xy_step_c_open_report_data;
	control_path.enable_nodata = bma4xy_step_c_enable_nodata;
	control_path.enable_step_detect = bma4xy_step_c_enable_step_detect;
	control_path.enable_significant = bma4xy_step_c_enable_significant;
	control_path.step_c_set_delay = bma4xy_step_c_set_delay;
	control_path.step_d_set_delay = bma4xy_setp_d_set_selay;
	control_path.is_report_input_direct = false;
	control_path.is_counter_support_batch = false;
	control_path.is_detector_support_batch = false;
	control_path.is_smd_support_batch = false;
	control_path.is_report_input_direct = false;
	control_path.enable_floor_c = bma4xy_floor_enable;
	control_path.floor_c_set_delay = bma4xy_floor_set_delay;
	control_path.floor_c_batch = bma4xy_floor_c_batch;
	control_path.floor_c_flush = bma4xy_floor_c_flush;
	err = step_c_register_control_path(&control_path);
	if (err) {
		BMA4XY_ERR("step_c_register_control_path fail = %d\n", err);
		goto exit;
	}

	data_path.get_data = bma4xy_step_c_get_data;
	data_path.vender_div = 1000;
	data_path.get_data_significant = bma4xy_stc_get_data_significant;
	data_path.get_data_step_d = bma4xy_stc_get_data_step_d;
	data_path.get_data_floor_c = bma4xy_get_data_floor;
	err = step_c_register_data_path(&data_path);
	if (err) {
		BMA4XY_ERR("step_c_register_data_path fail = %d\n", err);
		goto exit;
	}

	BMA4XY_LOG("%s: OK\n", __func__);

	return 0;
exit:
	BMA4XY_ERR("err = %d\n", err);
	return err;
}

static int bma4xy_step_c_uninit(void)
{
	BMA4XY_FUN();
	return 0;
}

static int bma4xy_step_c_init(void)
{
	int err = 0;
	BMA4XY_LOG("bma4xy_step_c_init\n");

	err = bma4xy_step_c_probe();
	if (err) {
		BMA4XY_ERR("bma4xy_step_c_probe failed: %d\n", err);
		return err;
	}

	return 0;
}

static struct step_c_init_info bma4xy_step_c_init_info = {
	.name = "bma4xy_step_c",
	.init = bma4xy_step_c_init,
	.uninit = bma4xy_step_c_uninit,
};
#endif

#ifdef BMA4_TILT
static int bma4xy_tilt_enable(int en)
{
	int err = 0;

	BMA4XY_LOG("bma4xy_tilt_enable enable = %d\n", en);

	if (en == 1) {
		err = bma4_set_advance_power_save(BMA4_DISABLE, &bma4xy_data->device);
		if (err)
			BMA4XY_ERR("bma4_set_advance_power_save failed: %d\n", err);

		bma4xy_i2c_delay_us(10, NULL);

		err = bma4_set_accel_enable(BMA4_ENABLE, &bma4xy_data->device);
		if (err)
			BMA4XY_ERR("bma4_set_accel_enable failed: %d\n", err);

		bma4xy_i2c_delay_us(10, NULL);
	}

	err = bma423_feature_enable(BMA423_WRIST_WEAR, en, &bma4xy_data->device);
	if (err) {
		BMA4XY_ERR("bma423_feature_enable failed: %d\n", err);
		return err;
	}

	bma4xy_i2c_delay_us(10, NULL);

	if (en == 0 &&
	    bma4xy_data->sigmotion_enable == 0 &&
	    bma4xy_data->stepdet_enable == 0 &&
	    bma4xy_data->stepcounter_enable == 0 &&
	    bma4xy_data->wakeup_enable == 0 &&
	    bma4xy_data->single_tap == 0 &&
	    bma4xy_data->double_tap == 0) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &bma4xy_data->device);
		if (err)
			BMA4XY_ERR("bma4_set_accel_enable failed: %d\n", err);
	}

	bma4xy_i2c_delay_us(10, NULL);
	bma4xy_data->wrist_wear = en;
	bma4xy_data->tilt_enable = en;

	return 0;
}

static int bma4xy_tilt_get_data(int *value, int *status)
{
	return 0;
}

static int bma4xy_tilt_probe(void)
{
	int err = 0;
	struct tilt_control_path control_path = {0};
	struct tilt_data_path data_path = {0};

	BMA4XY_FUN();

	control_path.open_report_data = bma4xy_tilt_enable;
	err = tilt_register_control_path(&control_path);
	if (err) {
		BMA4XY_ERR("tilt_register_control_path failed: %d\n", err);
		return err;
	}

	data_path.get_data = bma4xy_tilt_get_data;
	err = tilt_register_data_path(&data_path);
	if (err) {
		BMA4XY_ERR("tilt_register_data_path failed: %d\n", err);
		return err;
	}

	return 0;
}

static int bma4xy_tilt_uninit(void)
{
	BMA4XY_FUN();
	return 0;
}

static int bma4xy_tilt_init(void)
{
	int err;

	BMA4XY_FUN();

	err = bma4xy_tilt_probe();
	if (err) {
		BMA4XY_ERR("bma4xy_tilt_probe failed: %d\n", err);
		return err;
	}

	return 0;
}

static struct tilt_init_info bma4xy_tilt_init_info = {
	.name = "bma4xy_tilt",
	.init = bma4xy_tilt_init,
	.uninit = bma4xy_tilt_uninit,
};
#endif

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
	err = step_c_driver_add(&bma4xy_step_c_init_info);
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
