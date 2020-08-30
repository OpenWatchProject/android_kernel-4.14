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
//static struct platform_device *accelPltFmDev;

#if !defined(BMA420) && !defined(BMA456)
//#define BMA4_STEP_COUNTER
#endif
#if defined(BMA422) || defined(BMA455) || defined(BMA424) || defined(BMA423)
//#define BMA4_WAKEUP
#endif
#if defined(BMA423)
//#define BMA4_TILT
#endif

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bma4xy_i2c_remove(struct i2c_client *client);
#ifdef CONFIG_PM_SLEEP
static int bma4xy_i2c_suspend(struct device *dev);
static int bma4xy_i2c_resume(struct device *dev);
#endif
static int bma4xy_acc_init(void);
static int bma4xy_acc_uninit(void);
static int gsensor_set_delay(uint64_t ns);

#if defined(BMA420) || defined(BMA421) || defined(BMA421L) || defined(BMA422) || defined(BMA423)
static struct data_resolution bma4xy_acc_data_resolution[] = {
	{{1, 95}, 512},	/* +/-4g  in 12-bit resolution: 1.95 mg/LSB */
};
static struct data_resolution bma4xy_acc_offset_resolution = {{1, 95}, 512};
#elif defined(BMA455) || defined(BMA424) || defined(BMA424SC) || defined(BMA455N)
static struct data_resolution bma4xy_acc_data_resolution[] = {
	{{0, 12}, 8192}, /* +/-4g  in 16-bit resolution: 0.12 mg/LSB */
};
static struct data_resolution bma4xy_acc_offset_resolution = {{0, 12}, 8192};
#endif

#define GSE_TAG "[BMA4xy] "
#ifdef DEBUG
#define GSE_FUN() pr_err(GSE_TAG "%s\n", __func__)
#define GSE_ERR(fmt, args...) pr_err(GSE_TAG "%s %d: " fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...) pr_err(GSE_TAG fmt, ##args)
#define GSE_DBG(fmt, args...) pr_err(GSE_TAG fmt, ##args)
#else
#define GSE_FUN() pr_debug(GSE_TAG "%s\n", __func__)
#define GSE_ERR(fmt, args...) pr_err(GSE_TAG "%s %d: " fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...) pr_info(GSE_TAG fmt, ##args)
#define GSE_DBG(fmt, args...) pr_debug(GSE_TAG fmt, ##args)
#endif

static int remap_dir = 0;
static bool enable_status;
static bool sensor_power = true;
static int sensor_suspend;
//static struct GSENSOR_VECTOR3D gsensor_gain;
static DEFINE_MUTEX(gsensor_mutex);

enum BMA4XY_SENSOR_INT_MAP {
	BMA4XY_FFULL_INT = 8,
	BMA4XY_FWM_INT = 9,
	BMA4XY_DRDY_INT = 10,
};

enum BMA4XY_CONFIG_FUN {
	BMA4XY_SIG_MOTION_SENSOR = 0,
	BMA4XY_STEP_DETECTOR_SENSOR = 1,
	BMA4XY_STEP_COUNTER_SENSOR = 2,
	BMA4XY_TILT_SENSOR = 3,
	BMA4XY_PICKUP_SENSOR = 4,
	BMA4XY_GLANCE_DETECTOR_SENSOR = 5,
	BMA4XY_WAKEUP_SENSOR = 6,
	BMA4XY_ANY_MOTION_SENSOR = 7,
	BMA4XY_ORIENTATION_SENSOR = 8,
	BMA4XY_FLAT_SENSOR = 9,
	BMA4XY_TAP_SENSOR = 10,
	BMA4XY_HIGH_G_SENSOR = 11,
	BMA4XY_LOW_G_SENSOR = 12,
	BMA4XY_ACTIVITY_SENSOR = 13,
	BMA4XY_NO_MOTION_SENSOR = 14,
};

enum BMA4XY_INT_STATUS0 {
	SIG_MOTION_OUT = 0x01,
	STEP_DET_OUT = 0x02,
#if defined (BMA423)
	TILT_OUT = 0x08,
#else
	TILT_OUT = 0x04,
#endif
	PICKUP_OUT = 0x08,
	GLANCE_OUT = 0x10,
	WAKEUP_OUT = 0x20,
	ANY_NO_MOTION_OUT = 0x40,
	ERROR_INT_OUT = 0x80,
};

enum BMA4XY_INT_STATUS1 {
	FIFOFULL_OUT = 0x01,
	FIFOWATERMARK_OUT = 0x02,
	MAG_DRDY_OUT = 0x20,
	ACC_DRDY_OUT = 0x80,
};

/*bma4 fifo analyse return err status*/
enum BMA4_FIFO_ANALYSE_RETURN_T {
	FIFO_OVER_READ_RETURN = -10,
	FIFO_SENSORTIME_RETURN = -9,
	FIFO_SKIP_OVER_LEN = -8,
	FIFO_M_A_OVER_LEN = -5,
	FIFO_M_OVER_LEN = -3,
	FIFO_A_OVER_LEN = -1
};

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

static int bma4xy_set_data_resolution()
{
	bma4xy_data->reso = bma4xy_acc_data_resolution;
	return 0;
}

static int bma4xy_read_data(int16_t data[BMA4XY_ACC_AXIS_NUM])
{
	int err = 0;
	struct bma4_accel raw_data;

	err = bma4_read_accel_xyz(&raw_data, &bma4xy_data->device);
	if (err < 0)
		return err;

	data[BMA4XY_ACC_AXIS_X] = raw_data.x;
	data[BMA4XY_ACC_AXIS_Y] = raw_data.y;
	data[BMA4XY_ACC_AXIS_Z] = raw_data.z;

	return err;
}

static int bma4xy_read_sensor_data(char *buf, int bufsize)
{
	int acc[BMA4XY_ACC_AXIS_NUM];
	int res = 0;

	if (buf == NULL)
		return -1;

	if (sensor_suspend == 1)
		return 0;

	res = bma4xy_read_data(bma4xy_data->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}

	bma4xy_data->data[BMA4XY_ACC_AXIS_X] += bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X];
	bma4xy_data->data[BMA4XY_ACC_AXIS_Y] += bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y];
	bma4xy_data->data[BMA4XY_ACC_AXIS_Z] += bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z];

	acc[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_X]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_X] * bma4xy_data->data[BMA4XY_ACC_AXIS_X];
	acc[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Y]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Y] * bma4xy_data->data[BMA4XY_ACC_AXIS_Y];
	acc[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Z]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Z] * bma4xy_data->data[BMA4XY_ACC_AXIS_Z];

	acc[BMA4XY_ACC_AXIS_X] =
	    acc[BMA4XY_ACC_AXIS_X] * GRAVITY_EARTH_1000 / bma4xy_data->reso->sensitivity;
	acc[BMA4XY_ACC_AXIS_Y] =
	    acc[BMA4XY_ACC_AXIS_Y] * GRAVITY_EARTH_1000 / bma4xy_data->reso->sensitivity;
	acc[BMA4XY_ACC_AXIS_Z] =
	    acc[BMA4XY_ACC_AXIS_Z] * GRAVITY_EARTH_1000 / bma4xy_data->reso->sensitivity;

	snprintf(buf, bufsize, "%04x %04x %04x",
	         acc[BMA4XY_ACC_AXIS_X],
	         acc[BMA4XY_ACC_AXIS_Y],
	         acc[BMA4XY_ACC_AXIS_Z]);

	if (atomic_read(&bma4xy_data->trace) & ADX_TRC_IOCTL)
		GSE_LOG("gsensor data: %s!\n", buf);

	return 0;
}

static int bma4xy_read_raw_data(char *buf)
{
	int res = 0;

	if (buf == NULL)
		return -EINVAL;

	res = bma4xy_read_data(bma4xy_data->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}

	snprintf(buf, PAGE_SIZE, "bma4xy_read_raw_data %04x %04x %04x",
	         bma4xy_data->data[BMA4XY_ACC_AXIS_X],
	         bma4xy_data->data[BMA4XY_ACC_AXIS_Y],
	         bma4xy_data->data[BMA4XY_ACC_AXIS_Z]);

	return 0;
}

static int bma4xy_read_offset(int8_t ofs[BMA4XY_ACC_AXIS_NUM])
{
	int err = 0;
#ifdef SW_CALIBRATION
	ofs[0] = ofs[1] = ofs[2] = 0x0;
#else
	err = bma4xy_i2c_read(bma4xy_data->client,
	                      BMA4_OFFSET_0_ADDR, ofs, BMA4XY_ACC_AXIS_NUM);
	if (err)
		GSE_ERR("error: %d\n", err);
#endif
	GSE_LOG("offesx=%x, y=%x, z=%x", ofs[0], ofs[1], ofs[2]);

	return err;
}

static int bma4xy_reset_calibration()
{
	int err = 0;

#ifdef SW_CALIBRATION
#else
	uint8_t ofs[3] = {0, 0, 0};

	err = bma4xy_i2c_write(bma4xy_data->client, BMA4_OFFSET_0_ADDR, ofs, 3);
	if (err)
		GSE_ERR("error: %d\n", err);
#endif

	memset(bma4xy_data->cali_sw, 0x00, sizeof(bma4xy_data->cali_sw));
	memset(bma4xy_data->offset, 0x00, sizeof(bma4xy_data->offset));
	return err;
}

static int bma4xy_read_calibration(int dat[BMA4XY_ACC_AXIS_NUM])
{
	int err = 0;
	int mul;

	GSE_FUN();
#ifdef SW_CALIBRATION
	mul = 0;/*only SW Calibration, disable HW Calibration*/
#else
	err = bma4xy_read_offset(bma4xy_data->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = priv->reso->sensitivity / bma4xy_acc_offset_resolution.sensitivity;
#endif

	dat[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_X]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_X] * (bma4xy_data->offset[BMA4XY_ACC_AXIS_X] * mul +
	            bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X]);
	dat[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Y]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Y] * (bma4xy_data->offset[BMA4XY_ACC_AXIS_Y] * mul +
	            bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y]);
	dat[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Z]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Z] * (bma4xy_data->offset[BMA4XY_ACC_AXIS_Z] * mul +
	            bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z]);

	return err;
}

static int bma4xy_read_calibrationEx(int act[BMA4XY_ACC_AXIS_NUM],
                                     int raw[BMA4XY_ACC_AXIS_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data */
	int err;
	int mul;

	err = 0;
#ifdef SW_CALIBRATION
	mul = 0;/* only SW Calibration, disable HW Calibration */
#else
	err = bma4xy_read_offset(bma4xy_data->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = bma4xy_data->reso->sensitivity / bma4xy_acc_offset_resolution.sensitivity;
#endif

	raw[BMA4XY_ACC_AXIS_X] =
	    bma4xy_data->offset[BMA4XY_ACC_AXIS_X] * mul + bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X];
	raw[BMA4XY_ACC_AXIS_Y] =
	    bma4xy_data->offset[BMA4XY_ACC_AXIS_Y] * mul + bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y];
	raw[BMA4XY_ACC_AXIS_Z] =
	    bma4xy_data->offset[BMA4XY_ACC_AXIS_Z] * mul + bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z];
	act[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_X]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_X] * raw[BMA4XY_ACC_AXIS_X];
	act[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Y]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Y] * raw[BMA4XY_ACC_AXIS_Y];
	act[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Z]] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Z] * raw[BMA4XY_ACC_AXIS_Z];
	return err;
}

static int bma4xy_write_calibration(int dat[BMA4XY_ACC_AXIS_NUM])
{
	int err = 0;
	int cali[BMA4XY_ACC_AXIS_NUM], raw[BMA4XY_ACC_AXIS_NUM];

	err = bma4xy_read_calibrationEx(cali, raw);
	if (0 != err) {/*offset will be updated in bma4xy_data->offset */
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	        raw[BMA4XY_ACC_AXIS_X], raw[BMA4XY_ACC_AXIS_Y], raw[BMA4XY_ACC_AXIS_Z],
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_X],
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_Y],
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_Z],
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X],
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y],
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z]);

	/*calculate the real offset expected by caller */
	cali[BMA4XY_ACC_AXIS_X] += dat[BMA4XY_ACC_AXIS_X];
	cali[BMA4XY_ACC_AXIS_Y] += dat[BMA4XY_ACC_AXIS_Y];
	cali[BMA4XY_ACC_AXIS_Z] += dat[BMA4XY_ACC_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n",
	        dat[BMA4XY_ACC_AXIS_X],
	        dat[BMA4XY_ACC_AXIS_Y],
	        dat[BMA4XY_ACC_AXIS_Z]);

#ifdef SW_CALIBRATION
	bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_X] *
	    (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_X]]);
	bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Y] *
	    (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Y]]);
	bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Z] *
	    (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Z]]);
#else
	int divisor = bma4xy_data->reso->sensitivity / lsb;/* modified */

	bma4xy_data->offset[BMA4XY_ACC_AXIS_X] =
	    (int8_t) (bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_X] *
	              (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_X]]) / (divisor));
	bma4xy_data->offset[BMA4XY_ACC_AXIS_Y] =
	    (int8_t) (bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Y] *
	              (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Y]]) / (divisor));
	bma4xy_data->offset[BMA4XY_ACC_AXIS_Z] =
	    (int8_t) (bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Z] *
	              (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Z]]) / (divisor));

	/*convert software calibration using standard calibration */
	bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_X] *
	    (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_X]]) % (divisor);
	bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Y] *
	    (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Y]]) % (divisor);
	bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z] =
	    bma4xy_data->cvt.sign[BMA4XY_ACC_AXIS_Z] *
	    (cali[bma4xy_data->cvt.map[BMA4XY_ACC_AXIS_Z]]) % (divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_X] * divisor +
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X],
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_Y] * divisor +
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y],
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_Z] * divisor +
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z],
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_X], bma4xy_data->offset[BMA4XY_ACC_AXIS_Y],
	        bma4xy_data->offset[BMA4XY_ACC_AXIS_Z],
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X], bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y],
	        bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z]);

	err = hwmsen_write_block(bma4xy_data->client,
	                         BMA4_OFFSET_0_ADDR, bma4xy_data->offset, BMA4XY_ACC_AXIS_NUM);
	if (err) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif
	bma4xy_i2c_delay_us(1, NULL);
	return err;
}

static int bma4xy_set_power_mode(bool enable)
{
	int err = 0;

	if (enable == 0 &&
	    (bma4xy_data->sigmotion_enable == 0) &&
	    (bma4xy_data->stepdet_enable == 0) &&
	    (bma4xy_data->stepcounter_enable == 0) &&
	    (bma4xy_data->tilt_enable == 0) &&
	    (bma4xy_data->wrist_wear == 0) &&
	    (bma4xy_data->pickup_enable == 0) &&
	    (bma4xy_data->glance_enable == 0) &&
	    (bma4xy_data->wakeup_enable == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &bma4xy_data->device);
		GSE_LOG("acc_op_mode 0\n");
	} else if (enable == 1) {
		err = bma4_set_accel_enable(BMA4_ENABLE, &bma4xy_data->device);
		GSE_LOG("acc_op_mode 1\n");
	}

	if (err) {
		GSE_ERR("failed\n");
		return err;
	}

	sensor_power = enable;
	bma4xy_data->acc_pm = enable;
	bma4xy_i2c_delay_us(5, NULL);
	GSE_LOG("leave Sensor power status is sensor_power = %d\n",
	        sensor_power);
	return err;
}

static int bma4xy_set_data_format(uint8_t dataformat)
{
	int err = 0;
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &bma4xy_data->device);
	acc_config.range = (uint8_t)(BMA4_ACCEL_RANGE_4G);
	err += bma4_set_accel_config(&acc_config, &bma4xy_data->device);
	if (err) {
		GSE_ERR("faliled");
		return -EIO;
	}
	bma4xy_i2c_delay_us(5, NULL);
	return bma4xy_set_data_resolution();
}

static int bma4xy_set_bw_rate(uint8_t bwrate)
{
	int ret = 0;
	uint8_t data = 0;

	if (bwrate == 4)
		data = 0x74;
	else
		data = bwrate | 0xA0;

	ret = bma4xy_i2c_write_wrapper(0x40, &data, 1, NULL);
	if (ret) {
		GSE_ERR("failed");
		return ret;
	}

	bma4xy_i2c_delay_us(5, NULL);
	GSE_LOG("acc_odr =%d", data);
	return ret;
}

static ssize_t bma4xy_show_chip_id(struct device_driver *ddri, char *buf)
{
	uint8_t chip_id = 0;
	int err = 0;

	err = bma4xy_i2c_read_wrapper(BMA4_CHIP_ID_ADDR, &chip_id, 1, NULL);
	if (err) {
		GSE_ERR("falied");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "chip_id=%x\n", chip_id);
}

static ssize_t bma4xy_show_acc_op_mode(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char acc_op_mode;

	err = bma4_get_accel_enable(&acc_op_mode, &bma4xy_data->device);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "1 mean enable now is %d\n", acc_op_mode);
}
static ssize_t bma4xy_store_acc_op_mode(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;
	if (op_mode == 2 &&
	    (bma4xy_data->sigmotion_enable == 0) &&
	    (bma4xy_data->stepdet_enable == 0) &&
	    (bma4xy_data->stepcounter_enable == 0) &&
	    (bma4xy_data->tilt_enable == 0) &&
	    (bma4xy_data->pickup_enable == 0) &&
	    (bma4xy_data->glance_enable == 0) &&
	    (bma4xy_data->wakeup_enable == 0) &&
	    (bma4xy_data->anymotion_enable == 0) &&
	    (bma4xy_data->nomotion_enable == 0) &&
	    (bma4xy_data->orientation_enable == 0)) {
		err = bma4_set_accel_enable(
		          BMA4_DISABLE, &bma4xy_data->device);
		GSE_LOG("acc_op_mode %ld", op_mode);
	} else if (op_mode == 0) {
		err = bma4_set_accel_enable(BMA4_ENABLE, &bma4xy_data->device);
		GSE_LOG("acc_op_mode %ld", op_mode);
	}
	if (err) {
		GSE_ERR("failed");
		return err;
	}
	bma4xy_data->acc_pm = op_mode;
	bma4xy_i2c_delay_us(5, NULL);
	return count;
}

static ssize_t bma4xy_show_acc_value(struct device_driver *ddri, char *buf)
{
	struct bma4_accel data;
	int err;

	err = bma4_read_accel_xyz(&data, &bma4xy_data->device);
	if (err < 0)
		return err;
	return snprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
	                data.x, data.y, data.z);
}

static ssize_t bma4xy_show_acc_range(struct device_driver *ddri, char *buf)
{
	int err;
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &bma4xy_data->device);
	if (err) {
		GSE_ERR("failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", acc_config.range);
}

static ssize_t bma4xy_store_acc_range(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long acc_range;
	struct bma4_accel_config acc_config;

	err = kstrtoul(buf, 10, &acc_range);
	if (err)
		return err;
	err = bma4_get_accel_config(&acc_config, &bma4xy_data->device);
	acc_config.range = (uint8_t)(acc_range);
	err += bma4_set_accel_config(&acc_config, &bma4xy_data->device);
	bma4xy_i2c_delay_us(5, NULL);
	if (err) {
		GSE_ERR("faliled");
		return -EIO;
	}
	return count;
}

static ssize_t bma4xy_show_acc_odr(struct device_driver *ddri, char *buf)
{
	int err;
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &bma4xy_data->device);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	bma4xy_data->acc_odr = acc_config.odr;
	return snprintf(buf, PAGE_SIZE, "%d\n", bma4xy_data->acc_odr);

}

static ssize_t bma4xy_store_acc_odr(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long acc_odr;
	uint8_t data = 0;

	err = kstrtoul(buf, 10, &acc_odr);
	if (err)
		return err;
	data = (uint8_t)acc_odr;
	if (acc_odr == 4)
		data = 0x74;
	else
		data |= 0xA0;
	err = bma4xy_data->device.bus_write(0x40, &data, 1, NULL);
	if (err) {
		GSE_ERR("faliled");
		return -EIO;
	}
	GSE_ERR("acc_odr =%ld", acc_odr);
	bma4xy_i2c_delay_us(5, NULL);
	bma4xy_data->acc_odr = acc_odr;
	return count;
}

static ssize_t bma4xy_show_selftest(struct device_driver *ddri, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", bma4xy_data->selftest);
}
static ssize_t bma4xy_store_selftest(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	uint8_t result = 0;

	err = bma4_perform_accel_selftest(
	          &result, &bma4xy_data->device);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	if (result == 0) {
		GSE_LOG("Selftest successsful");
		bma4xy_data->selftest = 1;
	} else
		bma4xy_data->selftest = 0;
	return count;
}

static ssize_t bma4xy_show_foc(struct device_driver *ddri, char *buf)
{

	if (bma4xy_data == NULL) {
		GSE_ERR("Invalid bma4xy_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, 64,
	                "Use echo g_sign aixs > foc to begin foc\n");
}

static ssize_t bma4xy_store_foc(struct device_driver *ddri, const char *buf, size_t count)
{
	int g_value[3] = {0};
	struct bma4_accel_foc_g_value data = {0};
	int err = 0;

	err = sscanf(buf, "%11d %11d %11d",
	             &g_value[0], &g_value[1], &g_value[2]);

	GSE_LOG("g_value0=%d, g_value1=%d, g_value2=%d",
	        g_value[0], g_value[1], g_value[2]);

	if (err != 3) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}

	data.x = g_value[0];
	data.y = g_value[1];
	data.z = g_value[2];
	err = bma4_perform_accel_foc(&data, &bma4xy_data->device);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}

	GSE_LOG("FOC successsfully");

	return count;
}

static ssize_t bma4xy_show_config_function(struct device_driver *ddri, char *buf)
{

	if (bma4xy_data == NULL) {
		GSE_ERR("Invalid bma4xy_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, PAGE_SIZE,
	                "sig_motion0=%d step_detector1=%d step_counter2=%d\n"
	                "tilt3=%d pickup4=%d glance_detector5=%d wakeup6=%d\n"
	                "any_motion7=%d nomotion8=%d\n"
	                "orientation9=%d flat10=%d\n"
	                "high_g11=%d low_g12=%d activity13=%d nomotion14=%d\n",
	                bma4xy_data->sigmotion_enable, bma4xy_data->stepdet_enable,
	                bma4xy_data->stepcounter_enable, bma4xy_data->tilt_enable,
	                bma4xy_data->pickup_enable, bma4xy_data->glance_enable,
	                bma4xy_data->wakeup_enable, bma4xy_data->anymotion_enable,
	                bma4xy_data->nomotion_enable, bma4xy_data->orientation_enable,
	                bma4xy_data->flat_enable, bma4xy_data->highg_enable,
	                bma4xy_data->lowg_enable, bma4xy_data->activity_enable,
	                bma4xy_data->nomotion_enable);
}

static ssize_t bma4xy_store_config_function(struct device_driver *ddri, const char *buf, size_t count)
{
	int ret;
	int config_func = 0;
	int enable = 0;
	uint8_t feature;
	feature = 0;

	ret = sscanf(buf, "%11d %11d", &config_func, &enable);
	GSE_LOG("config_func = %d, enable=%d", config_func, enable);
	if (ret != 2) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	if (config_func < 0 || config_func > 14)
		return -EINVAL;
	switch (config_func) {
	case BMA4XY_SIG_MOTION_SENSOR:
#if defined(BMA422)
		feature = BMA422_SIG_MOTION;
#endif
#if defined(BMA422N)
		feature = BMA422N_SIG_MOTION;
#endif
#if defined(BMA455N)
		feature = BMA455N_SIG_MOTION;
#endif
#if defined(BMA424)
		feature = BMA424_SIG_MOTION;
#endif
#if defined(BMA455)
		feature = BMA455_SIG_MOTION;
#endif
		bma4xy_data->sigmotion_enable = enable;
		break;
	case BMA4XY_STEP_DETECTOR_SENSOR:
#if defined(BMA421)
		if (bma421_step_detector_enable(
		        enable, &bma4xy_data->device) < 0) {
			GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
#endif
#if defined(BMA423)
		if (bma423_step_detector_enable(
		        enable, &bma4xy_data->device) < 0) {
			GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
#endif
#if defined(BMA424SC)
		if (bma424sc_step_detector_enable(
		        enable, &bma4xy_data->device) < 0) {
			GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
#endif
#if defined(BMA421L)
		if (bma421l_step_detector_enable(
		        enable, &bma4xy_data->device) < 0) {
			GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
#endif
#if defined(BMA422)
		if (bma422_step_detector_enable(
		        enable, &bma4xy_data->device) < 0) {
			GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
#endif
#if defined(BMA424)
		if (bma424_step_detector_enable(
		        enable, &bma4xy_data->device) < 0) {
			GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
#endif
#if defined(BMA455)
		if (bma455_step_detector_enable(
		        enable, &bma4xy_data->device) < 0) {
			GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
#endif
#if defined(BMA422N)
		feature = BMA422N_STEP_DETR;
#endif
#if defined(BMA455N)
		feature = BMA455N_STEP_DETR;
#endif
		bma4xy_data->stepdet_enable = enable;
		break;
	case BMA4XY_STEP_COUNTER_SENSOR:
#if defined(BMA421)
		feature = BMA421_STEP_CNTR;
#endif
#if defined(BMA423)
		feature = BMA423_STEP_CNTR;
#endif
#if defined(BMA424SC)
		feature = BMA424SC_STEP_CNTR;
#endif
#if defined(BMA421L)
		feature = BMA421L_STEP_CNTR;
#endif
#if defined(BMA422)
		feature = BMA422_STEP_CNTR;
#endif
#if defined(BMA422N)
		feature = BMA422N_STEP_CNTR;
#endif
#if defined(BMA455N)
		feature = BMA455N_STEP_CNTR;
#endif
#if defined(BMA424)
		feature = BMA424_STEP_CNTR;
#endif
#if defined(BMA455)
		feature = BMA455_STEP_CNTR;
#endif
		bma4xy_data->stepcounter_enable = enable;
		break;
	case BMA4XY_TILT_SENSOR:
#if defined(BMA422)
		feature = BMA422_TILT;
#endif
#if defined(BMA424)
		feature = BMA424_TILT;
#endif
#if defined(BMA455)
		feature = BMA455_TILT;
#endif
		bma4xy_data->tilt_enable = enable;
		break;
	case BMA4XY_PICKUP_SENSOR:
#if defined(BMA422)
		feature = BMA422_PICKUP;
#endif
#if defined(BMA424)
		feature = BMA424_PICKUP;
#endif
#if defined(BMA455)
		feature = BMA455_PICKUP;
#endif
		bma4xy_data->pickup_enable = enable;
		break;
	case BMA4XY_GLANCE_DETECTOR_SENSOR:
#if defined(BMA422)
		feature = BMA422_GLANCE;
#endif
#if defined(BMA424)
		feature = BMA424_GLANCE;
#endif
#if defined(BMA455)
		feature = BMA455_GLANCE;
#endif
		bma4xy_data->glance_enable = enable;
		break;
	case BMA4XY_WAKEUP_SENSOR:
#if defined(BMA422)
		feature = BMA422_WAKEUP;
#endif
#if defined(BMA424)
		feature = BMA424_WAKEUP;
#endif
#if defined(BMA455)
		feature = BMA455_WAKEUP;
#endif
		bma4xy_data->wakeup_enable = enable;
		break;
	case BMA4XY_ANY_MOTION_SENSOR:
#if defined(BMA420)
		feature = BMA420_ANY_MOTION;
#endif
#if defined(BMA421)
		feature = BMA421_ANY_MOTION;
#endif
#if defined(BMA424SC)
		feature = BMA424SC_ANY_MOTION;
#endif
#if defined(BMA421L)
		feature = BMA421L_ANY_MOTION;
#endif
#if defined(BMA422)
		feature = BMA422_ANY_MOTION;
#endif
#if defined(BMA422N)
		if (enable == 1) {
			if (bma422n_anymotion_enable_axis(
			        bma4xy_data->any_motion_axis, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		} else if (enable == 0) {
			if (bma422n_anymotion_enable_axis(
			        0, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		}
#endif
#if defined(BMA455N)
		if (enable == 1) {
			if (bma455n_anymotion_enable_axis(
			        bma4xy_data->any_motion_axis, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		} else if (enable == 0) {
			if (bma455n_anymotion_enable_axis(
			        0, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		}
#endif
#if defined(BMA424)
		feature = BMA424_ANY_MOTION;
#endif
#if defined(BMA455)
		feature = BMA455_ANY_MOTION;
#endif
		bma4xy_data->anymotion_enable = enable;
		break;
	case BMA4XY_ORIENTATION_SENSOR:
#if defined(BMA420)
		feature = BMA420_ORIENTATION;
#endif
#if defined(BMA422N)
		feature = BMA422N_ORIENTATION;
#endif
#if defined(BMA455N)
		feature = BMA455N_ORIENTATION;
#endif
#if defined(BMA456)
		feature = BMA420_ORIENTATION;
#endif
		bma4xy_data->orientation_enable = enable;
		break;
	case BMA4XY_FLAT_SENSOR:
#if defined(BMA420)
		feature = BMA420_FLAT;
#endif
#if defined(BMA421L)
		feature = BMA421L_FLAT;
#endif
#if defined(BMA456)
		feature = BMA456_FLAT;
#endif
		bma4xy_data->flat_enable = enable;
		break;
	case BMA4XY_TAP_SENSOR:
#if defined(BMA420)
		feature = BMA420_TAP;
#endif
		bma4xy_data->tap_enable = enable;
		break;
	case BMA4XY_HIGH_G_SENSOR:
#if defined(BMA420)
		feature = BMA420_HIGH_G;
#endif
		bma4xy_data->highg_enable = enable;
		break;
	case BMA4XY_LOW_G_SENSOR:
#if defined(BMA420)
		feature = BMA420_LOW_G;
#endif
		bma4xy_data->lowg_enable = enable;
		break;
	case BMA4XY_ACTIVITY_SENSOR:
#if defined(BMA421)
		feature = BMA421_ACTIVITY;
#endif
#if defined(BMA422N)
		feature = BMA422N_ACTIVITY;
#endif
#if defined(BMA455N)
		feature = BMA455N_ACTIVITY;
#endif
#if defined(BMA424SC)
		feature = BMA424SC_ACTIVITY;
#endif
		bma4xy_data->activity_enable = enable;
		break;
	case BMA4XY_NO_MOTION_SENSOR:
#if defined(BMA422N)
		if (enable == 1) {
			if (bma422n_no_motion_enable_axis(
			        bma4xy_data->no_motion_axis, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		} else if (enable == 0) {
			if (bma422n_no_motion_enable_axis(
			        0, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		}
#endif
#if defined(BMA455N)
		if (enable == 1) {
			if (bma455n_no_motion_enable_axis(
			        bma4xy_data->no_motion_axis, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		} else if (enable == 0) {
			if (bma455n_no_motion_enable_axis(
			        0, &bma4xy_data->device) < 0) {
				GSE_ERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
				return -EINVAL;
			}
		}
#endif
		bma4xy_data->nomotion_enable = enable;
		break;
	default:
		GSE_ERR("Invalid sensor handle: %d", config_func);
		return -EINVAL;
	}
#if defined(BMA420)
	if (bma420_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma420 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA421)
	if (bma421_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma421 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA423)
	if (bma423_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma421 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA424SC)
	if (bma424sc_feature_enable(
	        feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma421 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA421L)
	if (bma421l_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma421 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA422)
	if (bma422_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma422 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA422N)
	if (bma422n_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma422 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA455N)
	if (bma455n_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma422 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA424)
	if (bma424_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma422 virtual error");
		return -EINVAL;
	}
#endif
#if defined(BMA455)
	if (bma455_feature_enable(feature, enable, &bma4xy_data->device) < 0) {
		GSE_ERR("set bma455 virtual error");
		return -EINVAL;
	}
#endif
	return count;
}


static ssize_t bma4xy_dump_regs_function(struct device_driver *ddri, char *buf)
{
#if defined(BMA423)
	//return bma423_dump_reg(buf,&bma4xy_data->device);
	return 0;
#else
	return 0;
#endif
}

static ssize_t bma4xy_show_load_config_stream(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "config stream %s\n", bma4xy_data->config_stream_name);
}

#if 1

#if defined(BMA420)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma420_axes_remap * axis_remap_data)
#elif defined(BMA421)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma421_axes_remap * axis_remap_data)
#elif defined(BMA423)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma423_axes_remap * axis_remap_data)
#elif defined(BMA424SC)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma424sc_axes_remap * axis_remap_data)
#elif defined(BMA421L)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma421l_axes_remap * axis_remap_data)
#elif defined(BMA422)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma422_axes_remap * axis_remap_data)
#elif defined(BMA422N)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma422n_axes_remap * axis_remap_data)
#elif defined(BMA455N)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma455n_axes_remap * axis_remap_data)
#elif defined(BMA424)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma424_axes_remap * axis_remap_data)
#elif defined(BMA455)
static void bma4xy_set_tilt_remap(struct bma4xy_data *bma4xy_data, struct bma455_axes_remap * axis_remap_data)
#endif
{
	switch (/*bma4xy_data->hw.*/remap_dir) {
	case 1:
		axis_remap_data->x_axis = 0;
		axis_remap_data->x_axis_sign = 0;
		axis_remap_data->y_axis = 1;
		axis_remap_data->y_axis_sign = 0;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 0;
		break;
	case 2:
		axis_remap_data->x_axis = 0;
		axis_remap_data->x_axis_sign = 1;
		axis_remap_data->y_axis = 1;
		axis_remap_data->y_axis_sign = 1;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 0;
		break;
	case 3:
		axis_remap_data->x_axis = 1;
		axis_remap_data->x_axis_sign = 1;
		axis_remap_data->y_axis = 0;
		axis_remap_data->y_axis_sign = 0;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 0;
		break;
	case 4:
		axis_remap_data->x_axis = 1;
		axis_remap_data->x_axis_sign = 0;
		axis_remap_data->y_axis = 0;
		axis_remap_data->y_axis_sign = 1;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 0;
		break;
	case 5:
		axis_remap_data->x_axis = 1;
		axis_remap_data->x_axis_sign = 0;
		axis_remap_data->y_axis = 0;
		axis_remap_data->y_axis_sign = 0;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 1;
		break;
	case 6:
		axis_remap_data->x_axis = 1;
		axis_remap_data->x_axis_sign = 1;
		axis_remap_data->y_axis = 0;
		axis_remap_data->y_axis_sign = 1;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 1;
		break;
	case 7:
		axis_remap_data->x_axis = 0;
		axis_remap_data->x_axis_sign = 1;
		axis_remap_data->y_axis = 1;
		axis_remap_data->y_axis_sign = 0;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 1;
		break;
	case 8:
		axis_remap_data->x_axis = 0;
		axis_remap_data->x_axis_sign = 0;
		axis_remap_data->y_axis = 1;
		axis_remap_data->y_axis_sign = 1;
		axis_remap_data->z_axis = 2;
		axis_remap_data->z_axis_sign = 1;
		break;
	default:
		break;
	}
}

#endif

int bma4xy_init_after_config_stream_load()
{
	int err = 0;
	uint8_t int_enable = 0x0B; //0x0A //LSQ change from 0x0A
	uint8_t latch_enable = 0x00; //0x01
	uint8_t int1_map = 0x08; //0xFF

#if defined(BMA420)
	struct bma420_axes_remap axis_remap_data;
#elif defined(BMA421)
	struct bma421_axes_remap axis_remap_data;
#elif defined(BMA423)
	struct bma423_axes_remap axis_remap_data;
#elif defined(BMA424SC)
	struct bma424sc_axes_remap axis_remap_data;
#elif defined(BMA421L)
	struct bma421l_axes_remap axis_remap_data;
#elif defined(BMA422)
	struct bma422_axes_remap axis_remap_data;
#elif defined(BMA422N)
	struct bma422n_axes_remap axis_remap_data;
#elif defined(BMA455N)
	struct bma455n_axes_remap axis_remap_data;
#elif defined(BMA424)
	struct bma424_axes_remap axis_remap_data;
#elif defined(BMA455)
	struct bma455_axes_remap axis_remap_data;
#endif

	GSE_FUN();

	err = bma4_write_regs(
	          BMA4_INT_MAP_1_ADDR, &int1_map, 1, &bma4xy_data->device);
	bma4xy_i2c_delay_us(10, NULL);
	err += bma4_write_regs(
	           BMA4_INT1_IO_CTRL_ADDR, &int_enable, 1, &bma4xy_data->device);
	bma4xy_i2c_delay_us(1, NULL);
	err += bma4_write_regs(
	           BMA4_INTR_LATCH_ADDR, &latch_enable, 1, &bma4xy_data->device);
	bma4xy_i2c_delay_us(1, NULL);
	if (err)
		GSE_ERR("map and enable interrupr1 failed err=%d", err);
	memset(&axis_remap_data, 0, sizeof(axis_remap_data));
	//LSQ set BMA remap dir
#if 0
	axis_remap_data.x_axis = 1;
	axis_remap_data.x_axis_sign = 1;
	axis_remap_data.y_axis = 0;
	axis_remap_data.y_axis_sign = 0;
	axis_remap_data.z_axis = 2;
	axis_remap_data.z_axis_sign = 0;
#else
	bma4xy_set_tilt_remap(bma4xy_data, &axis_remap_data);
#endif


#if defined(BMA420)
	err = bma420_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
#endif
#if defined(BMA421)
	err = bma421_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
	err = bma421_select_platform(BMA421_PHONE_CONFIG, &bma4xy_data->device);
	if (err)
		GSE_ERR("set bma421 step_count select platform error");
#endif
#if defined(BMA423)
	err = bma423_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
	//err = bma423_select_platform(BMA423_PHONE_CONFIG, &bma4xy_data->device);
	if (err)
		GSE_ERR("set bma421 step_count select platform error");
#endif
#if defined(BMA424SC)
	err = bma424sc_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
	err = bma424sc_select_platform(BMA424SC_PHONE_CONFIG, &bma4xy_data->device);
	if (err)
		GSE_ERR("set bma421 step_count select platform error");
#endif
#if defined(BMA421L)
	err = bma421l_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
#endif
#if defined(BMA422)
	err = bma422_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
	bma4xy_i2c_delay_us(5, NULL);
	err += bma422n_select_platform(
	           BMA422N_PHONE_CONFIG, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
	bma4xy_i2c_delay_us(5, NULL);
	err += bma455n_select_platform(
	           BMA455N_PHONE_CONFIG, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_set_remap_axes(&axis_remap_data, &bma4xy_data->device);
#endif
	bma4xy_i2c_delay_us(1, NULL);
	if (err) {
		GSE_ERR("write axis_remap failed");
		return err;
	}
	return err;
}

int bma4xy_init_fifo_config(
    struct bma4xy_data *bma4xy_data)
{
	int err = 0;
	err = bma4_set_fifo_config(
	          BMA4_FIFO_HEADER, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("enable fifo header failed err=%d", err);
	bma4xy_i2c_delay_us(1, NULL);
	err = bma4_set_fifo_config(
	          BMA4_FIFO_TIME, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("enable fifo timer failed err=%d", err);
	bma4xy_i2c_delay_us(1, NULL);
	return err;
}

int bma4xy_update_config_stream(int choose)
{
	char *name;
	int err = 0;
	uint8_t crc_check = 0;

	switch (choose) {
	case 1:
		name = "android.tbin";
		break;
	case 2:
		name = "legacy.tbin";
		break;
	default:
		GSE_ERR("no choose fw = %d,use dafault ", choose);
		name = "bma4xy_config_stream";
		break;
	}
	GSE_LOG("choose the config_stream %s", name);
	bma4xy_data->config_stream_name = name;
	if ((choose == 1) || (choose == 2)) {
		GSE_ERR("no choose fw = %d,use dafault ", choose);
		GSE_LOG("choose the config_stream %s", name);
	} else if (choose == 3) {
#if defined(BMA420)
		err = bma420_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA421)
		err = bma421_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA423)
		err = bma423_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA424SC)
		err = bma424sc_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA421L)
		err = bma421l_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA422)
		err = bma422_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA422N)
		err = bma422n_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA455N)
		err = bma455n_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA424)
		err = bma424_write_config_file(&bma4xy_data->device);
#endif
#if defined(BMA455)
		err = bma455_write_config_file(&bma4xy_data->device);
#endif
		if (err)
			GSE_ERR("download config stream failer");
		bma4xy_i2c_delay_us(200, NULL);
		err = bma4_read_regs(BMA4_INTERNAL_STAT,
		                     &crc_check, 1, &bma4xy_data->device);
		if (err)
			GSE_ERR("reading CRC failer");
		if (crc_check != BMA4_ASIC_INITIALIZED)
			GSE_ERR("crc check error %x", crc_check);
	}
	return err;
}

static ssize_t bma4xy_store_load_config_stream(
    struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long choose = 0;
	int err = 0;

	err = kstrtoul(buf, 10, &choose);
	if (err)
		return err;
	GSE_LOG("config_stream_choose %ld", choose);
	err = bma4xy_update_config_stream(choose);
	if (err) {
		GSE_ERR("config_stream load error");
		return count;
	}
	err = bma4xy_init_after_config_stream_load();
	if (err) {
		GSE_ERR("bma4xy_init_after_config_stream_load error");
		return count;
	}
	return count;
}

static int bma4xy_load_config_stream()
{
	int err = 0;

	err = bma4xy_update_config_stream(3);
	if (err) {
		GSE_ERR("config_stream load error");
		return err;
	}

	err = bma4xy_init_after_config_stream_load();
	if (err) {
		GSE_ERR("bma4xy_init_after_config_stream_load error");
		return err;
	}

	return err;
}

static ssize_t bma4xy_show_reg_sel(struct device_driver *ddri, char *buf)
{

	if (bma4xy_data == NULL) {
		GSE_ERR("Invalid bma4xy_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, 64, "reg=0X%02X, len=%d\n",
	                bma4xy_data->reg_sel, bma4xy_data->reg_len);
}

static ssize_t bma4xy_store_reg_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	ssize_t ret;

	if (bma4xy_data == NULL) {
		GSE_ERR("Invalid bma4xy_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11X %11d",
	             &bma4xy_data->reg_sel, &bma4xy_data->reg_len);
	if (ret != 2) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	return count;
}

static ssize_t bma4xy_show_reg_val(struct device_driver *ddri, char *buf)
{

	ssize_t ret;
	uint8_t reg_data[128], i;
	int pos;

	if (bma4xy_data->reg_sel == BMA4_FEATURE_CONFIG_ADDR)
		ret = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR,
		                     reg_data, bma4xy_data->reg_len, &bma4xy_data->device);
	else
		ret = bma4xy_data->device.bus_read(bma4xy_data->reg_sel, reg_data, bma4xy_data->reg_len, NULL);
	if (ret < 0) {
		GSE_ERR("Reg op failed");
		return ret;
	}
	pos = 0;
	for (i = 0; i < bma4xy_data->reg_len; ++i) {
		pos += snprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';
	return pos;
}

static ssize_t bma4xy_store_reg_val(struct device_driver *ddri, const char *buf, size_t count)
{
	ssize_t ret;
	uint8_t reg_data[128];
	int i, j, status, digit;

	if (bma4xy_data == NULL) {
		GSE_ERR("Invalid bma4xy_data pointer");
		return -ENODEV;
	}
	status = 0;
	for (i = j = 0; i < count && j < bma4xy_data->reg_len; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
		    buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		GSE_LOG("digit is %d", digit);
		switch (status) {
		case 2:
			++j; /* Fall thru */
		case 0:
			reg_data[j] = digit;
			status = 1;
			break;
		case 1:
			reg_data[j] = reg_data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j > bma4xy_data->reg_len)
		j = bma4xy_data->reg_len;
	else if (j < bma4xy_data->reg_len) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	GSE_LOG("Reg data read as");
	for (i = 0; i < j; ++i)
		GSE_LOG("%d", reg_data[i]);
	if (bma4xy_data->reg_sel == BMA4_FEATURE_CONFIG_ADDR)
		ret = bma4_write_regs(BMA4_FEATURE_CONFIG_ADDR,
		                      reg_data, bma4xy_data->reg_len, &bma4xy_data->device);
	else
		ret = bma4xy_data->device.bus_write(bma4xy_data->reg_sel, reg_data, bma4xy_data->reg_len, NULL);
	if (ret < 0) {
		GSE_ERR("Reg op failed");
		return ret;
	}
	return count;
}

static ssize_t bma4xy_show_config_file_version(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint16_t version = 0;

#if defined(BMA420)
	err = bma420_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA421)
	err = bma421_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA423)
	err = bma423_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA424SC)
	err = bma424sc_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA421L)
	err = bma421l_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA422)
	err = bma422_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA422N)
	err = bma422n_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA455N)
	err = bma455n_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA424)
	err = bma424_get_config_id(&version, &bma4xy_data->device);
#elif defined(BMA455)
	err = bma455_get_config_id(&version, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 128, "Config_stream version :0x%x\n", version);
}

static ssize_t bma4xy_show_avail_sensor(struct device_driver *ddri, char *buf)
{
	uint16_t avail_sensor = 0;
#if defined(BMA420)
	avail_sensor = 420;
#elif defined(BMA421)
	avail_sensor = 421;
#elif defined(BMA424SC)
	avail_sensor = 421;
#elif defined(BMA421L)
	avail_sensor = 421;
#elif defined(BMA422)
	avail_sensor = 422;
#elif defined(BMA422N)
	avail_sensor = 4227;
#elif defined(BMA455N)
	avail_sensor = 4557;
#elif defined(BMA424)
	avail_sensor = 422;
#elif defined(BMA455)
	avail_sensor = 455;
#elif defined(BMA456)
	avail_sensor = 456;
#endif
	return snprintf(buf, 32, "%d\n", avail_sensor);
}

#if defined(BMA422) || defined(BMA455) || defined(BMA424) || defined(BMA422N) || defined(BMA455N)
static ssize_t bma4xy_show_sig_motion_config(struct device_driver *ddri, char *buf)
{
	int err;

#if defined(BMA422)
	struct bma422_sig_motion_config sig_m_config;
#elif defined(BMA455)
	struct bma455_sig_motion_config sig_m_config;
#elif defined(BMA424)
	struct bma424_sig_motion_config sig_m_config;
#elif defined(BMA422N)
	struct bma422n_sig_motion_config sig_m_config;
#elif defined(BMA455N)
	struct bma455n_sig_motion_config sig_m_config;
#endif

#if defined(BMA422)
	err = bma422_get_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_get_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_get_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_get_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_get_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
#if !defined(BMA422N) && !defined(BMA455N)
	return snprintf(buf, PAGE_SIZE,
	                "threshold =0x%x skiptime= 0x%x prooftime = 0x%x\n",
	                sig_m_config.threshold, sig_m_config.skiptime, sig_m_config.prooftime);
#else
	return snprintf(buf, PAGE_SIZE,
	                "block_size =0x%x p2p_min_intvl= 0x%x p2p_max_intvl = 0x%x mcr_min= 0x%x mcr_max = 0x%x\n",
	                sig_m_config.block_size, sig_m_config.p2p_min_intvl, sig_m_config.p2p_max_intvl,
	                sig_m_config.mcr_min, sig_m_config.mcr_max);
#endif
}

static ssize_t bma4xy_store_sig_motion_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
#if !defined(BMA422N) && !defined(BMA455N)
	unsigned int data[3] = {0};
#else
	unsigned int data[5] = {0};
#endif

#if defined(BMA422)
	struct bma422_sig_motion_config sig_m_config;
#elif defined(BMA455)
	struct bma455_sig_motion_config sig_m_config;
#elif defined(BMA424)
	struct bma424_sig_motion_config sig_m_config;
#elif defined(BMA422N)
	struct bma422n_sig_motion_config sig_m_config;
#elif defined(BMA455N)
	struct bma455n_sig_motion_config sig_m_config;
#endif

#if !defined(BMA422N) && !defined(BMA455N)
	err = sscanf(buf, "%11x %11x %11x", &data[0], &data[1], &data[2]);
	if (err != 3) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	memset(&sig_m_config, 0, sizeof(sig_m_config));
	sig_m_config.threshold = (uint16_t)data[0];
	sig_m_config.skiptime = (uint16_t)data[1];
	sig_m_config.prooftime = (uint8_t)data[2];
#else
	err = sscanf(buf, "%11x %11x %11x %11x %11x", &data[0], &data[1], &data[2], &data[3], &data[4]);
	if (err != 3) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	memset(&sig_m_config, 0, sizeof(sig_m_config));
	sig_m_config.block_size = (uint16_t)data[0];
	sig_m_config.p2p_min_intvl = (uint16_t)data[1];
	sig_m_config.p2p_max_intvl = (uint16_t)data[2];
	sig_m_config.mcr_min = (uint16_t)data[3];
	sig_m_config.mcr_max = (uint16_t)data[4];
#endif
#if defined(BMA422)
	err = bma422_set_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_set_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_set_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_set_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_set_sig_motion_config(&sig_m_config, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif

#if defined(BMA422) || defined(BMA455) || defined(BMA424)
static ssize_t bma4xy_show_tilt_threshold(struct device_driver *ddri, char *buf)
{
	int err;
	uint8_t tilt_threshold;

#if defined(BMA422)
	err = bma422_tilt_get_threshold(&tilt_threshold, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_tilt_get_threshold(&tilt_threshold, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_tilt_get_threshold(&tilt_threshold, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", tilt_threshold);
}

static ssize_t bma4xy_store_tilt_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long tilt_threshold;

	err = kstrtoul(buf, 10, &tilt_threshold);
	if (err)
		return err;
	GSE_LOG("tilt_threshold %ld", tilt_threshold);
#if defined(BMA422)
	err = bma422_tilt_set_threshold(tilt_threshold, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_tilt_set_threshold(tilt_threshold, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_tilt_set_threshold(tilt_threshold, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if !defined(BMA420) && !defined(BMA456)
static ssize_t bma4xy_show_step_counter_val(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint32_t step_counter_val = 0;

#if defined(BMA421)
	err = bma421_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA423)
	err = bma423_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA424SC)
	err = bma424sc_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA421L)
	err = bma421l_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA422)
	err = bma422_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_step_counter_output(
	          &step_counter_val, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	GSE_LOG("val %u", step_counter_val);
	if (bma4xy_data->err_int_trigger_num == 0) {
		bma4xy_data->step_counter_val = step_counter_val;
		GSE_LOG("report val %u", bma4xy_data->step_counter_val);
		err = snprintf(buf, 96, "%u\n", bma4xy_data->step_counter_val);
		bma4xy_data->step_counter_temp = bma4xy_data->step_counter_val;
	} else {
		GSE_LOG("after err report val %u",
		        bma4xy_data->step_counter_val + step_counter_val);
		err = snprintf(buf, 96, "%u\n",
		               bma4xy_data->step_counter_val + step_counter_val);
		bma4xy_data->step_counter_temp =
		    bma4xy_data->step_counter_val + step_counter_val;
	}
	return err;
}
static ssize_t bma4xy_show_step_counter_watermark(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint16_t watermark;

#if defined(BMA421)
	err = bma421_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA423)
	err = bma423_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA424SC)
	err = bma424sc_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA421L)
	err = bma421l_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA422)
	err = bma422_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_step_counter_get_watermark(
	          &watermark, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", watermark);
}
static ssize_t bma4xy_store_step_counter_watermark(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long step_watermark;

	err = kstrtoul(buf, 10, &step_watermark);
	if (err)
		return err;
	GSE_LOG("watermark step_counter %ld", step_watermark);
#if defined(BMA421)
	err = bma421_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA423)
	err = bma423_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA424SC)
	err = bma424sc_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA421L)
	err = bma421l_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA422)
	err = bma422_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_step_counter_set_watermark(
	          step_watermark, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}

static ssize_t bma4xy_show_step_counter_parameter(struct device_driver *ddri, char *buf)
{
	int err = 0;

#if defined(BMA421)
	struct bma421_stepcounter_settings setting;
#elif defined(BMA424SC)
	struct bma424sc_stepcounter_settings setting;
#elif defined(BMA423)
	struct bma423_stepcounter_settings setting;
#elif defined(BMA421L)
	struct bma421l_stepcounter_settings setting;
#elif defined(BMA422)
	struct bma422_stepcounter_settings setting;
#elif defined(BMA422N)
	struct bma422n_stepcounter_settings setting;
#elif defined(BMA455N)
	struct bma455n_stepcounter_settings setting;
#elif defined(BMA424)
	struct bma424_stepcounter_settings setting;
#elif defined(BMA455)
	struct bma455_stepcounter_settings setting;
#endif
#if defined(BMA421)
	err = bma421_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA423)
	err = bma423_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA424SC)
	err = bma424sc_stepcounter_get_parameter(
	          &setting, &bma4xy_data->device);
#endif
#if defined(BMA421L)
	err = bma421l_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA422)
	err = bma422_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_stepcounter_get_parameter(&setting, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("read failed");
		return err;
	}

#if defined(BMA421L) || defined(BMA422) || defined(BMA455) || defined(BMA424)
	return snprintf(buf, PAGE_SIZE,
	                "parameter1 =0x%x parameter2= 0x%x\n"
	                "parameter3 = 0x%x parameter4 = 0x%x\n"
	                "parameter5 = 0x%x parameter6 = 0x%x\n"
	                "parameter7 = 0x%x\n",
	                setting.param1, setting.param2, setting.param3, setting.param4,
	                setting.param5, setting.param6, setting.param7);
#elif defined(BMA421) || defined(BMA424SC) || defined(BMA422N) || defined(BMA455N) || defined(BMA423)
	return snprintf(buf, PAGE_SIZE,
	                "parameter1 =0x%x parameter2= 0x%x\n"
	                "parameter3 = 0x%x parameter4 = 0x%x\n"
	                "parameter5 = 0x%x parameter6 = 0x%x\n"
	                "parameter7 = 0x%x parameter8 = 0x%x\n"
	                "parameter9 = 0x%x parameter10 = 0x%x\n"
	                "parameter11 = 0x%x parameter12 = 0x%x\n"
	                "parameter13 = 0x%x parameter14 = 0x%x\n"
	                "parameter15 = 0x%x parameter16 = 0x%x\n"
	                "parameter17 = 0x%x parameter18 = 0x%x\n"
	                "parameter19 = 0x%x parameter20 = 0x%x\n"
	                "parameter21 = 0x%x parameter22 = 0x%x\n"
	                "parameter23 = 0x%x parameter24 = 0x%x\n"
	                "parameter25 = 0x%x\n",
	                setting.param1, setting.param2, setting.param3, setting.param4,
	                setting.param5, setting.param6, setting.param7, setting.param8,
	                setting.param9, setting.param10, setting.param11, setting.param12,
	                setting.param13, setting.param14, setting.param15, setting.param16,
	                setting.param17, setting.param18, setting.param19, setting.param20,
	                setting.param21, setting.param22, setting.param23, setting.param24,
	                setting.param25);
#endif
}
static ssize_t bma4xy_store_step_counter_parameter(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
#if defined(BMA421)
	unsigned int data[25] = {0};
	struct bma421_stepcounter_settings setting;
#elif defined(BMA424SC)
	unsigned int data[25] = {0};
	struct bma424sc_stepcounter_settings setting;
#elif defined(BMA423)
	unsigned int data[25] = {0};
	struct bma423_stepcounter_settings setting;
#elif defined(BMA422N)
	unsigned int data[25] = {0};
	struct bma422n_stepcounter_settings setting;
#elif defined(BMA455N)
	unsigned int data[25] = {0};
	struct bma455n_stepcounter_settings setting;
#elif defined(BMA421L)
	unsigned int data[7] = {0};
	struct bma421l_stepcounter_settings setting;
#elif defined(BMA422)
	unsigned int data[7] = {0};
	struct bma422_stepcounter_settings setting;
#elif defined(BMA424)
	unsigned int data[7] = {0};
	struct bma424_stepcounter_settings setting;
#elif defined(BMA455)
	unsigned int data[7] = {0};
	struct bma455_stepcounter_settings setting;
#endif

#if defined(BMA421L) || defined(BMA422) || defined(BMA455) || defined(BMA424)
	err = sscanf(buf, "%11x %11x %11x %11x %11x %11x %11x",
	             &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]);
	if (err != 7) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	setting.param1 = (uint16_t)data[0];
	setting.param2 = (uint16_t)data[1];
	setting.param3 = (uint16_t)data[2];
	setting.param4 = (uint16_t)data[3];
	setting.param5 = (uint16_t)data[4];
	setting.param6 = (uint16_t)data[5];
	setting.param7 = (uint16_t)data[6];
#elif defined(BMA421) || defined(BMA424SC) || defined(BMA422N) || defined(BMA455N) || defined(BMA423)
	err = sscanf(buf,
	             "%11x %11x %11x %11x %11x %11x %11x %11x\n"
	             "%11x %11x %11x %11x %11x %11x %11x %11x\n"
	             "%11x %11x %11x %11x %11x %11x %11x %11x\n"
	             "%11x\n",
	             &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6],
	             &data[7], &data[8], &data[9], &data[10], &data[11], &data[12],
	             &data[13],
	             &data[14], &data[15], &data[16], &data[17], &data[18], &data[19],
	             &data[20],
	             &data[21], &data[22], &data[23], &data[24]);
	if (err != 25) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	setting.param1 = (uint16_t)data[0];
	setting.param2 = (uint16_t)data[1];
	setting.param3 = (uint16_t)data[2];
	setting.param4 = (uint16_t)data[3];
	setting.param5 = (uint16_t)data[4];
	setting.param6 = (uint16_t)data[5];
	setting.param7 = (uint16_t)data[6];
	setting.param8 = (uint16_t)data[7];
	setting.param9 = (uint16_t)data[8];
	setting.param10 = (uint16_t)data[9];
	setting.param11 = (uint16_t)data[10];
	setting.param12 = (uint16_t)data[11];
	setting.param13 = (uint16_t)data[12];
	setting.param14 = (uint16_t)data[13];
	setting.param15 = (uint16_t)data[14];
	setting.param16 = (uint16_t)data[15];
	setting.param17 = (uint16_t)data[16];
	setting.param18 = (uint16_t)data[17];
	setting.param19 = (uint16_t)data[18];
	setting.param20 = (uint16_t)data[19];
	setting.param21 = (uint16_t)data[20];
	setting.param22 = (uint16_t)data[21];
	setting.param23 = (uint16_t)data[22];
	setting.param24 = (uint16_t)data[23];
	setting.param25 = (uint16_t)data[24];
#endif
#if defined(BMA421)
	err = bma421_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA423)
	err = bma423_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA424SC)
	err = bma424sc_stepcounter_set_parameter(
	          &setting, &bma4xy_data->device);
#endif
#if defined(BMA421L)
	err = bma421l_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA422)
	err = bma422_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_stepcounter_set_parameter(&setting, &bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}

static ssize_t bma4xy_store_step_counter_reset(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long reset_counter;

	err = kstrtoul(buf, 10, &reset_counter);
	if (err)
		return err;
	GSE_LOG("reset_counter %ld", reset_counter);
#if defined(BMA421)
	err = bma421_reset_step_counter(&bma4xy_data->device);
#endif
#if defined(BMA423)
	err = bma423_reset_step_counter(&bma4xy_data->device);
#endif
#if defined(BMA424SC)
	err = bma424sc_reset_step_counter(&bma4xy_data->device);
#endif
#if defined(BMA422)
	err = bma422_reset_step_counter(&bma4xy_data->device);
#endif
#if defined(BMA422N)
	err = bma422n_reset_step_counter(&bma4xy_data->device);
#endif
#if defined(BMA455N)
	err = bma455n_reset_step_counter(&bma4xy_data->device);
#endif
#if defined(BMA424)
	err = bma424_reset_step_counter(&bma4xy_data->device);
#endif
#if defined(BMA455)
	err = bma455_reset_step_counter(&bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	bma4xy_data->step_counter_val = 0;
	bma4xy_data->step_counter_temp = 0;
	return count;
}
#endif

#if defined(BMA420)
int bma420_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		feature = feature | BMA420_ANY_MOTION;
	if (bma4xy_data->orientation_enable == BMA4_ENABLE)
		feature = feature | BMA420_ORIENTATION;
	if (bma4xy_data->flat_enable == BMA4_ENABLE)
		feature = feature | BMA420_FLAT;
	if (bma4xy_data->tap_enable == BMA4_ENABLE)
		feature = feature | BMA420_TAP;
	if (bma4xy_data->highg_enable == BMA4_ENABLE)
		feature = feature | BMA420_HIGH_G;
	if (bma4xy_data->lowg_enable == BMA4_ENABLE)
		feature = feature | BMA420_LOW_G;
	err = bma420_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;

}
#endif
#if defined(BMA421)
int bma421_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE) {
		if (bma421_step_detector_enable(
		        BMA4_ENABLE, &bma4xy_data->device) < 0)
			GSE_ERR("set BMA421_STEP_DECTOR error");
	}
	bma4xy_i2c_delay_us(2, NULL);
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		feature = feature | BMA421_ANY_MOTION;
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA421_STEP_CNTR;
	if (bma4xy_data->activity_enable == BMA4_ENABLE)
		feature = feature | BMA421_ACTIVITY;
	err = bma421_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif
#if defined(BMA423)
int bma423_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE) {
		if (bma423_step_detector_enable(
		        BMA4_ENABLE, &bma4xy_data->device) < 0)
			GSE_ERR("set BMA421_STEP_DECTOR error");
	}
	bma4xy_i2c_delay_us(2, NULL);

	if (bma4xy_data->wrist_wear == BMA4_ENABLE)
		feature = feature | BMA423_WRIST_WEAR;
	if (bma4xy_data->single_tap == BMA4_ENABLE)
		feature = feature | BMA423_SINGLE_TAP;
	if (bma4xy_data->double_tap == BMA4_ENABLE)
		feature = feature | BMA423_DOUBLE_TAP;
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA423_STEP_CNTR;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE)
		feature = feature | BMA423_STEP_ACT;

	err = bma423_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif

#if defined(BMA424SC)
int bma424sc_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE) {
		if (bma424sc_step_detector_enable(
		        BMA4_ENABLE, &bma4xy_data->device) < 0)
			GSE_ERR("set BMA424SC_STEP_DECTOR error");
	}
	bma4xy_i2c_delay_us(2, NULL);
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		feature = feature | BMA424SC_ANY_MOTION;
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA424SC_STEP_CNTR;
	if (bma4xy_data->activity_enable == BMA4_ENABLE)
		feature = feature | BMA424SC_ACTIVITY;
	err = bma424sc_feature_enable(
	          feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif

#if defined(BMA421L)
int bma421l_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE) {
		if (bma421l_step_detector_enable(
		        BMA4_ENABLE, &bma4xy_data->device) < 0)
			GSE_ERR("set BMA421L_STEP_DECTOR error");
	}
	bma4xy_i2c_delay_us(2, NULL);
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		feature = feature | BMA421L_ANY_MOTION;
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA421L_STEP_CNTR;
	if (bma4xy_data->flat_enable == BMA4_ENABLE)
		feature = feature | BMA421L_FLAT;
	err = bma421l_feature_enable(
	          feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif

#if defined(BMA422)
int bma422_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->sigmotion_enable == BMA4_ENABLE)
		feature = feature | BMA422_SIG_MOTION;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE) {
		if (bma422_step_detector_enable(
		        BMA4_ENABLE, &bma4xy_data->device) < 0)
			GSE_ERR("set BMA422_STEP_DECTOR error");
	}
	bma4xy_i2c_delay_us(2, NULL);
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA422_STEP_CNTR;
	if (bma4xy_data->tilt_enable == BMA4_ENABLE)
		feature = feature | BMA422_TILT;
	if (bma4xy_data->pickup_enable == BMA4_ENABLE)
		feature = feature | BMA422_PICKUP;
	if (bma4xy_data->glance_enable == BMA4_ENABLE)
		feature = feature | BMA422_GLANCE;
	if (bma4xy_data->wakeup_enable == BMA4_ENABLE)
		feature = feature | BMA422_WAKEUP;
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		feature = feature | BMA422_ANY_MOTION;
	err = bma422_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif
#if defined(BMA422N)
int bma422n_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->sigmotion_enable == BMA4_ENABLE)
		feature = feature | BMA422N_SIG_MOTION;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE)
		feature = feature | BMA422N_STEP_DETR;
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA422N_STEP_CNTR;
	if (bma4xy_data->orientation_enable == BMA4_ENABLE)
		feature = feature | BMA422N_ORIENTATION;
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		err = bma422n_anymotion_enable_axis(bma4xy_data->any_motion_axis, &bma4xy_data->device);
	err = bma422n_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif
#if defined(BMA455N)
int bma455n_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->sigmotion_enable == BMA4_ENABLE)
		feature = feature | BMA455N_SIG_MOTION;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE)
		feature = feature | BMA455N_STEP_DETR;
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA455N_STEP_CNTR;
	if (bma4xy_data->orientation_enable == BMA4_ENABLE)
		feature = feature | BMA455N_ORIENTATION;
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		err = bma455n_anymotion_enable_axis(bma4xy_data->any_motion_axis, &bma4xy_data->device);
	err = bma455n_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif

#if defined(BMA424)
int bma424_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->sigmotion_enable == BMA4_ENABLE)
		feature = feature | BMA424_SIG_MOTION;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE) {
		if (bma424_step_detector_enable(
		        BMA4_ENABLE, &bma4xy_data->device) < 0)
			GSE_ERR("set BMA422_STEP_DECTOR error");
	}
	bma4xy_i2c_delay_us(2, NULL);
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA424_STEP_CNTR;
	if (bma4xy_data->tilt_enable == BMA4_ENABLE)
		feature = feature | BMA424_TILT;
	if (bma4xy_data->pickup_enable == BMA4_ENABLE)
		feature = feature | BMA424_PICKUP;
	if (bma4xy_data->glance_enable == BMA4_ENABLE)
		feature = feature | BMA424_GLANCE;
	if (bma4xy_data->wakeup_enable == BMA4_ENABLE)
		feature = feature | BMA424_WAKEUP;
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		feature = feature | BMA424_ANY_MOTION;
	err = bma424_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif

#if defined(BMA455)
int bma455_config_feature()
{
	int err = 0;
	uint8_t feature = 0;
	if (bma4xy_data->sigmotion_enable == BMA4_ENABLE)
		feature = feature | BMA455_SIG_MOTION;
	if (bma4xy_data->stepdet_enable == BMA4_ENABLE) {
		if (bma455_step_detector_enable(
		        BMA4_ENABLE, &bma4xy_data->device) < 0)
			GSE_ERR("set BMA455_STEP_DECTOR error");
	}
	bma4xy_i2c_delay_us(2, NULL);
	if (bma4xy_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA455_STEP_CNTR;
	if (bma4xy_data->tilt_enable == BMA4_ENABLE)
		feature = feature | BMA455_TILT;
	if (bma4xy_data->pickup_enable == BMA4_ENABLE)
		feature = feature | BMA455_PICKUP;
	if (bma4xy_data->glance_enable == BMA4_ENABLE)
		feature = feature | BMA455_GLANCE;
	if (bma4xy_data->wakeup_enable == BMA4_ENABLE)
		feature = feature | BMA455_WAKEUP;
	if (bma4xy_data->anymotion_enable == BMA4_ENABLE)
		feature = feature | BMA455_ANY_MOTION;
	err = bma455_feature_enable(feature, BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set feature err");
	return err;
}
#endif

int bma4xy_reinit_after_error_interrupt()
{
	int err = 0;
	uint8_t data = 0;
	uint8_t crc_check = 0;

	bma4xy_data->err_int_trigger_num += 1;
	bma4xy_data->step_counter_val = bma4xy_data->step_counter_temp;
	/*reset the bma4xy*/
	err = bma4_set_command_register(0xB6, &bma4xy_data->device);
	if (!err)
		GSE_LOG("reset chip");
	/*reinit the fifo config*/
	err = bma4xy_init_fifo_config(bma4xy_data);
	if (err)
		GSE_ERR("fifo init failed");
	/*reload the config_stream*/
	err = bma4_write_config_file(&bma4xy_data->device);
	if (err)
		GSE_ERR("download config stream failer");
	bma4xy_i2c_delay_us(200, NULL);
	err = bma4_read_regs(BMA4_INTERNAL_STAT,
	                     &crc_check, 1, &bma4xy_data->device);
	if (err)
		GSE_ERR("reading CRC failer");
	if (crc_check != BMA4_ASIC_INITIALIZED)
		GSE_ERR("crc check error %x", crc_check);
	/*reconfig interrupt and remap*/
	err = bma4xy_init_after_config_stream_load();
	if (err)
		GSE_ERR("reconfig interrupt and remap error");
	/*reinit the feature*/
#if defined(BMA420)
	err = bma420_config_feature();
#endif
#if defined(BMA423)
	err = bma423_config_feature();
#endif
#if defined(BMA421)
	err = bma421_config_feature();
#endif
#if defined(BMA424SC)
	err = bma424sc_config_feature();
#endif
#if defined(BMA421L)
	err = bma421l_config_feature();
#endif
#if defined(BMA422)
	err = bma422_config_feature();
#endif
#if defined(BMA422N)
	err = bma422n_config_feature();
#endif
#if defined(BMA455N)
	err = bma455n_config_feature();
#endif
#if defined(BMA424)
	err = bma424_config_feature();
#endif
#if defined(BMA455)
	err = bma455_config_feature();
#endif
	if (err)
		GSE_ERR("reinit the virtual sensor error");
	/*reinit acc*/
	if (bma4xy_data->acc_odr != 0) {
		data = bma4xy_data->acc_odr;
		if (data == 4)
			data = 0x74;
		else
			data |= 0xA0;
		err = bma4xy_data->device.bus_write(0x40, &data, 1, NULL);
		if (err)
			GSE_ERR("set acc_odr faliled");
		bma4xy_i2c_delay_us(2, NULL);
	}
	if (bma4xy_data->acc_pm == 0)
		err = bma4_set_accel_enable(BMA4_ENABLE, &bma4xy_data->device);
	if (err)
		GSE_ERR("set acc_op_mode failed");
	bma4xy_i2c_delay_us(2, NULL);
	err = bma4_set_fifo_config(BMA4_FIFO_ACCEL,
	                           bma4xy_data->fifo_acc_enable, &bma4xy_data->device);
	if (err)
		GSE_ERR("set acc_fifo_enable faliled");
	bma4xy_i2c_delay_us(5, NULL);
	return 0;
}


#if defined(BMA4XY_ENABLE_INT)
static void bma4xy_uc_function_handle(uint8_t status)
{
	int err = 0;
#if defined(BMA420) || defined(BMA456)
	unsigned char uc_gpio[2] = {0};
#endif
	if (status & ERROR_INT_OUT) {
		err = bma4xy_reinit_after_error_interrupt(bma4xy_data);
		if (err)
			GSE_ERR("reinit failed");
	}
#if defined(BMA422) || defined(BMA421) || defined(BMA421L) || defined(BMA455) \
	|| defined(BMA424) || defined(BMA424SC) || defined(BMA422N) || defined(BMA455N) || defined(BMA423)
#if defined(BMA4_STEP_COUNTER)
	if ((status & STEP_DET_OUT) == 0x02)
		step_notify(TYPE_STEP_DETECTOR);
	if ((status & SIG_MOTION_OUT) == 0x01)
		step_notify(TYPE_SIGNIFICANT);
#endif
#if defined(BMA4_WAKEUP)
	if ((status & WAKEUP_OUT) == 0x20)
		//wag_notify();
#endif
#if defined(BMA4_TILT)
		if ((status & TILT_OUT) == 0x08) {
			//tilt_notify();
		}
#endif
#endif
#if defined(BMA420) || defined(BMA456)
	if (bma4xy_data->orientation_enable)
		err = bma4_read_regs(BMA4_STEP_CNT_OUT_0_ADDR,
		                     &uc_gpio[0], 1, &bma4xy_data->device);
	if (bma4xy_data->highg_enable)
		err += bma4_read_regs(BMA4_HIGH_G_OUT_ADDR,
		                      &uc_gpio[1], 1, &bma4xy_data->device);
	if (err) {
		GSE_ERR("read uc_gpio failed");
		return;
	}
	GSE_LOG("%d %d %d", uc_gpio[0], uc_gpio[1], uc_gpio[2]);
#endif
}

static void bma4xy_irq_work_func(struct work_struct *work)
{
	unsigned char int_status[2] = {0, 0};
	int err = 0;
	int in_suspend_copy;
	GSE_LOG("[%s]\n", __func__);

	//in_suspend_copy = atomic_read(&bma4xy_data->in_suspend);
	//tilt_notify();
	/*read the interrut status two register*/
	err = bma4xy_data->device.bus_read(bma4xy_data->device.dev_addr,
	                                   BMA4_INT_STAT_0_ADDR, int_status, 2);
	if (err) {
		GSE_ERR("[%s] bus_read BMA4xy stat reg fail\n", __func__);
		return;
	}
	GSE_LOG("int_status0 = 0x%x int_status1 =0x%x",
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
	GSE_LOG("int_status0 = %x int_status1 =%x",
	        int_status[0], int_status[1]);
#if defined(BMA4_STEP_COUNTER)
	if ((int_status[0] & SIG_MOTION_OUT) == 0x01)
		step_notify(TYPE_SIGNIFICANT);
#endif
}

static irqreturn_t bma4xy_irq_handle(int irq, void *handle)
{
#if 0  //LSQ mask these code
	int in_suspend_copy;
	GSE_LOG("[%s] and tilt_enable:[%d]   wakeup_enable:[%d]\n", __func__, bma4xy_data->tilt_enable, bma4xy_data->wakeup_enable);
	in_suspend_copy = atomic_read(&bma4xy_data->in_suspend);
	GSE_LOG("[%s] and is_suspend_copy:[%d]\n", __func__, in_suspend_copy);

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
	bma4xy_data->gpio_pin = 63;
	err = gpio_request_one(bma4xy_data->gpio_pin,
	                       GPIOF_IN, "bma4xy_interrupt");
	if (err < 0)
		return err;
	err = gpio_direction_input(bma4xy_data->gpio_pin);
	if (err < 0)
		return err;
	bma4xy_data->irq = gpio_to_irq(bma4xy_data->gpio_pin);
	err = request_irq(bma4xy_data->irq, bma4xy_irq_handle,
	                  IRQF_TRIGGER_RISING,
	                  SENSOR_NAME, bma4xy_data);

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

static int bma4xy_init_client(uint8_t reset_cali)
{
	int res = 0;

	GSE_FUN();

	res = bma4xy_set_data_format(BMA4_ACCEL_RANGE_4G);
	if (res) {
		GSE_ERR("SetDataFormat failed");
		return res;
	}

	//gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = bma4xy_data->reso->sensitivity;
	res = bma4xy_set_power_mode(0);
	if (res) {
		GSE_ERR("SetPowerMode failed");
		return res;
	}

	if (reset_cali) {
		/*reset calibration only in power on */
		res = bma4xy_reset_calibration();
		if (res) {
			GSE_ERR("reset_calibration failed");
			return res;
		}
	}

	res = bma4xy_load_config_stream();
	if (res)
		GSE_ERR("init failed after load config_stream");
	GSE_LOG("BMA4XY_init_client OK!\n");
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bma4xy_i2c_suspend(struct device *dev)
{
	GSE_FUN();

	enable_irq_wake(bma4xy_data->irq);
	atomic_set(&bma4xy_data->in_suspend, 1);

	return 0;
}

static int bma4xy_i2c_resume(struct device *dev)
{
	GSE_FUN();

	disable_irq_wake(bma4xy_data->irq);
	atomic_set(&bma4xy_data->in_suspend, 0);

	return 0;
}

static const struct dev_pm_ops bma4xy_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bma4xy_i2c_suspend, bma4xy_i2c_resume)
};
#endif

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "BMA4xy\n");
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BMA4XY_BUFSIZE];

	bma4xy_read_sensor_data(strbuf, BMA4XY_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	int err, len = 0, mul;
	int tmp[BMA4XY_ACC_AXIS_NUM];

	err = bma4xy_read_offset(bma4xy_data->offset);
	if (0 != err)
		return -EINVAL;

	err = bma4xy_read_calibration(tmp);
	if (0 != err)
		return -EINVAL;

	mul = bma4xy_data->reso->sensitivity / bma4xy_acc_offset_resolution.sensitivity;
	len +=
	    snprintf(buf + len, PAGE_SIZE - len,
	             "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_X],
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_Y],
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_Z],
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_X],
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_Y],
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_Z]);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len,
	             "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
	             bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X],
	             bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y],
	             bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z]);

	len +=
	    snprintf(buf + len, PAGE_SIZE - len,
	             "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_X] * mul + bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_X],
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_Y] * mul + bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Y],
	             bma4xy_data->offset[BMA4XY_ACC_AXIS_Z] * mul + bma4xy_data->cali_sw[BMA4XY_ACC_AXIS_Z],
	             tmp[BMA4XY_ACC_AXIS_X], tmp[BMA4XY_ACC_AXIS_Y], tmp[BMA4XY_ACC_AXIS_Z]);
	return len;
}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int err, x, y, z;
	int dat[BMA4XY_ACC_AXIS_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = bma4xy_reset_calibration();
		if (0 != err)
			GSE_ERR("reset offset err = %d\n", err);
	} else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[BMA4XY_ACC_AXIS_X] = x;
		dat[BMA4XY_ACC_AXIS_Y] = y;
		dat[BMA4XY_ACC_AXIS_Z] = z;

		err = bma4xy_write_calibration(dat);
		if (0 != err)
			GSE_ERR("write calibration err = %d\n", err);
	} else {
		GSE_ERR("invalid format\n");
	}
	return count;
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not support\n");
}

static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&bma4xy_data->trace));
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (!kstrtoint(buf, 16, &trace))
		atomic_set(&bma4xy_data->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %d\n",
		        buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
	                "CUST: %d %d (%d %d)\n",
	                bma4xy_data->hw.i2c_num, bma4xy_data->hw.direction, bma4xy_data->hw.power_id,
	                bma4xy_data->hw.power_vol);
	return len;
}

static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	unsigned char acc_op_mode;
	int err = 0;

	err = bma4_get_accel_enable(&acc_op_mode, &bma4xy_data->device);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	if (sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n",
		        sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n",
		        sensor_power);
	return snprintf(buf, PAGE_SIZE, "%x\n", acc_op_mode);
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *pbBuf)
{
	ssize_t _tLength = 0;

	GSE_LOG("[%s] default direction: %d\n", __func__, bma4xy_data->hw.direction);
	_tLength = snprintf(pbBuf, PAGE_SIZE,
	                    "default direction = %d\n", bma4xy_data->hw.direction);
	return _tLength;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, const char *pbBuf, size_t tCount)
{
	int _nDirection = 0;

	if (!kstrtoint(pbBuf, 10, &_nDirection)) {
		if (hwmsen_get_convert(_nDirection, &bma4xy_data->cvt))
			GSE_ERR("ERR: fail to set direction\n");
	}

	GSE_LOG("[%s] set direction: %d\n", __func__, _nDirection);

	return tCount;
}

static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(chip_id, S_IRUGO, bma4xy_show_chip_id, NULL);
static DRIVER_ATTR(acc_value, S_IRUGO, bma4xy_show_acc_value, NULL);
static DRIVER_ATTR(acc_range, S_IWUSR | S_IRUGO, bma4xy_show_acc_range, bma4xy_store_acc_range);
static DRIVER_ATTR(acc_odr, S_IWUSR | S_IRUGO, bma4xy_show_acc_odr, bma4xy_store_acc_odr);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, bma4xy_show_selftest, bma4xy_store_selftest);
static DRIVER_ATTR(avail_sensor, S_IRUGO, bma4xy_show_avail_sensor, NULL);
static DRIVER_ATTR(load_fw, S_IWUSR | S_IRUGO, bma4xy_show_load_config_stream, bma4xy_store_load_config_stream);
static DRIVER_ATTR(reg_val, S_IWUSR | S_IRUGO, bma4xy_show_reg_val, bma4xy_store_reg_val);
static DRIVER_ATTR(foc, S_IWUSR | S_IRUGO, bma4xy_show_foc, bma4xy_store_foc);
static DRIVER_ATTR(acc_op_mode, S_IWUSR | S_IRUGO, bma4xy_show_acc_op_mode, bma4xy_store_acc_op_mode);
static DRIVER_ATTR(reg_sel, S_IWUSR | S_IRUGO, bma4xy_show_reg_sel, bma4xy_store_reg_sel);
static DRIVER_ATTR(config_file_version, S_IRUGO, bma4xy_show_config_file_version, NULL);
static DRIVER_ATTR(config_function, S_IWUSR | S_IRUGO, bma4xy_show_config_function, bma4xy_store_config_function);
static DRIVER_ATTR(dump_regs, S_IRUGO, bma4xy_dump_regs_function, NULL);
#if defined(BMA422) || defined(BMA455) || defined(BMA424) || defined(BMA422N) || defined(BMA455N)
static DRIVER_ATTR(sig_config, S_IWUSR | S_IRUGO, bma4xy_show_sig_motion_config, bma4xy_store_sig_motion_config);
#endif
#if !defined(BMA420) && !defined(BMA456)
static DRIVER_ATTR(step_counter_val, S_IRUGO, bma4xy_show_step_counter_val, NULL);
static DRIVER_ATTR(step_counter_watermark, S_IWUSR | S_IRUGO, bma4xy_show_step_counter_watermark, bma4xy_store_step_counter_watermark);
static DRIVER_ATTR(step_counter_parameter, S_IWUSR | S_IRUGO, bma4xy_show_step_counter_parameter, bma4xy_store_step_counter_parameter);
static DRIVER_ATTR(step_counter_reset, S_IWUSR | S_IRUGO, NULL, bma4xy_store_step_counter_reset);
#endif
#if defined(BMA422) || defined(BMA455) || defined(BMA424)
static DRIVER_ATTR(tilt_threshold, S_IWUSR | S_IRUGO, bma4xy_show_tilt_threshold, bma4xy_store_tilt_threshold);
#endif
#if defined(BMA420) || defined(BMA456)
static DRIVER_ATTR(tap_type, S_IWUSR | S_IRUGO, bma4xy_show_tap_type, bma4xy_store_tap_type);
#endif

static struct driver_attribute *bma4xy_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_cali,
	&driver_attr_firlen,
	&driver_attr_trace,
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_orientation,
	&driver_attr_chip_id,
	&driver_attr_acc_op_mode,
	&driver_attr_acc_value,
	&driver_attr_acc_range,
	&driver_attr_acc_odr,
	&driver_attr_selftest,
	&driver_attr_avail_sensor,
	&driver_attr_foc,
	&driver_attr_load_fw,
	&driver_attr_reg_sel,
	&driver_attr_reg_val,
	&driver_attr_config_function,
	&driver_attr_dump_regs,
	&driver_attr_config_file_version,
#if defined(BMA422) || defined(BMA455) || defined(BMA424) || defined(BMA422N) || defined(BMA455N)
	&driver_attr_sig_config,
#endif
#if !defined(BMA420) && !defined(BMA456)
	&driver_attr_step_counter_val,
	&driver_attr_step_counter_watermark,
	&driver_attr_step_counter_parameter,
	&driver_attr_step_counter_reset,
#endif
#if defined(BMA422) || defined(BMA455) || defined(BMA424)
	&driver_attr_tilt_threshold,
#endif
#if defined(BMA420) || defined(BMA456)
	&driver_attr_tap_type,
#endif
#if defined(BMA422N) || defined(BMA455N)
	&driver_attr_nomotion_config_enable_axis,
	&driver_attr_nomotion_config,
#endif
#if defined(BMA420) || defined(BMA456) || defined(BMA422N) || defined(BMA455N)
	&driver_attr_orientation_config,
#endif
#if defined(BMA420) || defined(BMA456)
	&driver_attr_flat_config,
#endif
};

static int bma4xy_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bma4xy_attr_list) / sizeof(bma4xy_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;
	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bma4xy_attr_list[idx]);
		if (0 != err) {
			GSE_ERR("driver_create_file (%s) = %d\n",
			        bma4xy_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int bma4xy_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bma4xy_attr_list) / sizeof(bma4xy_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;
	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, bma4xy_attr_list[idx]);
	return err;
}

/* if use  this typ of enable ,
Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int gsensor_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*if use  this typ of enable , Gsensor only
enabled but not report inputEvent to HAL */
static int gsensor_enable_nodata(int en)
{
	int err = 0;

	if (((en == 0) && (sensor_power == false)) ||
	    ((en == 1) && (sensor_power == true))) {
		enable_status = sensor_power;
		GSE_LOG("Gsensor device have updated!\n");
	} else {
		enable_status = !sensor_power;
		if (atomic_read(&bma4xy_data->suspend) == 0) {
			err = bma4xy_set_power_mode(enable_status);
			GSE_LOG("Gsensor not in suspend  enable_status = %d\n",
			        enable_status);
		} else {
			GSE_LOG("G in suspend can not enable disable!s = %d\n",
			        enable_status);
		}
	}
	if (err) {
		GSE_ERR("gsensor_enable_nodata fail!\n");
		return -1;
	}
	GSE_LOG("gsensor_enable_nodata OK!\n");
	return 0;
}

static int gsensor_set_delay(uint64_t ns)
{
	int err = 0;
	int value;
	int data_rate;

	value = ns / 1000 / 1000;
	if (value >= 80) {
		data_rate = BMA4_OUTPUT_DATA_RATE_12_5HZ;
	} else if (value >= 40) {
		data_rate = BMA4_OUTPUT_DATA_RATE_25HZ;
	} else if (value >= 20) {
		data_rate = BMA4_OUTPUT_DATA_RATE_50HZ;
	} else if (value >= 10) {
		data_rate = BMA4_OUTPUT_DATA_RATE_100HZ;
	} else {
		data_rate = BMA4_OUTPUT_DATA_RATE_200HZ;
	}

	err = bma4xy_set_bw_rate(data_rate);
	if (err) {
		GSE_ERR("Set delay parameter error!\n");
		return -1;
	}

	if (value >= 50)
		atomic_set(&bma4xy_data->filter, 0);
	GSE_LOG("gsensor_set_delay (%d)\n", value);
	return 0;
}

static int gsensor_acc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;

	GSE_LOG("bma acc set delay = (%d) ok.\n", value);
	return gsensor_set_delay(samplingPeriodNs);
}

static int gsensor_acc_flush(void)
{
	return acc_flush_report();
}

static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
	char buff[BMA4XY_BUFSIZE];
	int ret;

	mutex_lock(&gsensor_mutex);
	bma4xy_read_sensor_data(buff, BMA4XY_BUFSIZE);
	mutex_unlock(&gsensor_mutex);
	ret = sscanf(buff, "%x %x %x", x, y, z);
	if (ret != 3) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

static struct acc_init_info bma4xy_init_info = {
	.name = SENSOR_NAME,
	.init = bma4xy_acc_init,
	.uninit = bma4xy_acc_uninit,
};

static int bma4xy_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = bma4xy_set_power_mode(1);
	if (err) {
		GSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = gsensor_acc_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		GSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}

static int bma4xy_factory_get_data(int32_t data[3], int *status)
{
	return gsensor_get_data(&data[0], &data[1], &data[2], status);
}

static int bma4xy_factory_get_raw_data(int32_t data[3])
{
	char strbuf[BMA4XY_BUFSIZE] = {0};

	bma4xy_read_raw_data(strbuf);
	data[0] = strbuf[0];
	data[1] = strbuf[1];
	data[2] = strbuf[2];

	return 0;
}

static int bma4xy_factory_enable_calibration(void)
{
	return 0;
}
static int bma4xy_factory_clear_cali(void)
{
	int err = 0;
	err = bma4xy_reset_calibration();
	return 0;
}

static int bma4xy_factory_set_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };
	cali[BMA4XY_ACC_AXIS_X] = data[0]
	                          * bma4xy_data->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[BMA4XY_ACC_AXIS_Y] = data[1]
	                          * bma4xy_data->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[BMA4XY_ACC_AXIS_Z] = data[2]
	                          * bma4xy_data->reso->sensitivity / GRAVITY_EARTH_1000;
	err = bma4xy_write_calibration(cali);
	if (err) {
		GSE_ERR("bma_WriteCalibration failed!\n");
		return -1;
	}

	return 0;
}

static int bma4xy_factory_get_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };
	err = bma4xy_read_calibration(cali);
	if (err) {
		GSE_ERR("bmi160_ReadCalibration failed!\n");
		return -1;
	}
	data[0] = cali[BMA4XY_ACC_AXIS_X]
	          * GRAVITY_EARTH_1000 / bma4xy_data->reso->sensitivity;
	data[1] = cali[BMA4XY_ACC_AXIS_X]
	          * GRAVITY_EARTH_1000 / bma4xy_data->reso->sensitivity;
	data[2] = cali[BMA4XY_ACC_AXIS_X]
	          * GRAVITY_EARTH_1000 / bma4xy_data->reso->sensitivity;
	return 0;
}

static int bma4xy_factory_do_self_test(void)
{
	return 0;
}

static struct accel_factory_fops bma4xy_factory_fops = {
	.enable_sensor = bma4xy_factory_enable_sensor,
	.get_data = bma4xy_factory_get_data,
	.get_raw_data = bma4xy_factory_get_raw_data,
	.enable_calibration = bma4xy_factory_enable_calibration,
	.clear_cali = bma4xy_factory_clear_cali,
	.set_cali = bma4xy_factory_set_cali,
	.get_cali = bma4xy_factory_get_cali,
	.do_self_test = bma4xy_factory_do_self_test,
};

static struct accel_factory_public bma4xy_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &bma4xy_factory_fops,
};

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};

	GSE_FUN();

	// Allocate memory for the bma4xy_data struct
	bma4xy_data = kzalloc(sizeof(struct bma4xy_data), GFP_KERNEL);
	if (bma4xy_data == NULL) {
		GSE_ERR("no memory available");
		err = -ENOMEM;
		goto exit;
	}

	// Get acc configuration from DTS
	err = get_accel_dts_func(client->dev.of_node, &bma4xy_data->hw);
	if (err) {
		GSE_ERR("get dts info fail\n");
		goto exit_err_clean;
	}

	err = hwmsen_get_convert(bma4xy_data->hw.direction, &bma4xy_data->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", bma4xy_data->hw.direction);
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

#if defined(BMA420)
	err = bma420_init(&bma4xy_data->device);
#elif defined(BMA421)
	err = bma421_init(&bma4xy_data->device);
#elif defined(BMA423)
	err = bma423_init(&bma4xy_data->device);
#elif defined(BMA424SC)
	err = bma424sc_init(&bma4xy_data->device);
#elif defined(BMA421L)
	err = bma421l_init(&bma4xy_data->device);
#elif defined(BMA422)
	err = bma422_init(&bma4xy_data->device);
#elif defined(BMA422N)
	err = bma422n_init(&bma4xy_data->device);
	bma4xy_data->any_motion_axis = 7;
	bma4xy_data->no_motion_axis = 7;
#elif defined(BMA455N)
	err = bma455n_init(&bma4xy_data->device);
	bma4xy_data->any_motion_axis = 7;
	bma4xy_data->no_motion_axis = 7;
#elif defined(BMA424)
	err = bma424_init(&bma4xy_data->device);
#elif defined(BMA455)
	err = bma455_init(&bma4xy_data->device);
#endif
	if (err) {
		GSE_ERR("init failed\n");
		goto exit_err_clean;
	}

#if defined(BMA4XY_ENABLE_INT)
	err = bma423_map_interrupt(0, (0x0001 << 3), 1, &bma4xy_data->device); // LSQ change // | 0x0001 <<6
	if (err < 0)
		GSE_ERR("BMA423 map irq failed\n");
#endif

	//wake_lock_init(&bma4xy_data->wakelock, WAKE_LOCK_SUSPEND, "bma4xy");
	err = bma4xy_init_client(1);
	if (err)
		GSE_ERR("bma4xy_device init cilent fail time\n");

	err = accel_factory_device_register(&bma4xy_factory_device);
	if (err) {
		GSE_ERR("bma4xy_device register failed\n");
		goto exit_err_clean;
	}

	err = bma4xy_create_attr(&bma4xy_init_info.platform_diver_addr->driver);
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_err_clean;
	}

	ctl.open_report_data = gsensor_open_report_data;
	ctl.enable_nodata = gsensor_enable_nodata;
	ctl.set_delay = gsensor_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;
	ctl.batch = gsensor_acc_batch;
	ctl.flush = gsensor_acc_flush;
	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_err_clean;
	}

	data.get_data = gsensor_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err\n");
		goto exit_err_clean;
	}

#if defined(BMA4XY_ENABLE_INT)
	err = bma4xy_request_irq();
	if (err < 0)
		GSE_ERR("Request irq failed\n");
#endif

	GSE_LOG("Init ok\n");

exit:
	return err;

exit_err_clean:
	kfree(bma4xy_data);
	bma4xy_data = NULL;
	return err;
}

static int bma4xy_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = bma4xy_delete_attr(&bma4xy_init_info.platform_diver_addr->driver);
	if (err)
		GSE_ERR("bma4xy_delete_attr fail: %d\n", err);

	err = accel_factory_device_deregister(&bma4xy_factory_device);
	if (err)
		GSE_ERR("acc_deregister fail: %d\n", err);

	kfree(bma4xy_data);
	bma4xy_data = NULL;
	bma4xy_data->client = NULL;

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
		GSE_ERR("add driver error\n");
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
		// TODO: Log error
		goto exit;
	}

#if defined(BMA4_STEP_COUNTER)
	// Register the step counter driver
	err = step_c_driver_add(&bma4xy_stc_init_info);
	if (err) {
		// TODO: Log error
		goto exit;
	}
#endif

#if defined(BMA4_WAKEUP)
	// Register the wakeup sensor driver
	err = wag_driver_add(&bma4xy_wakeup_init_info);
	if (err) {
		// TODO: Log error
		goto exit;
	}
#endif

#if defined(BMA4_TILT)
	// Register the tilt sensor driver
	err = tilt_driver_add(&bma4xy_tilt_init_info);
	if (err) {
		// TODO: Log error
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
