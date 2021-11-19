// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <asm/unaligned.h>

#if 1
#undef dev_dbg
#define dev_dbg dev_err
#endif

#define HX_CMD_TSSLPIN 0x80
#define HX_CMD_TSSLPOUT 0x81
#define HX_CMD_TSSOFF 0x82
#define HX_CMD_TSSON 0x83
#define HX_CMD_ROE 0x85
#define HX_CMD_RAE 0x86
#define HX_CMD_RLE 0x87
#define HX_CMD_CLRES 0x88

#define HX_VER_FW_CFG 0x39

struct himax_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;

	struct gpio_desc *gpiod_irq;
	struct gpio_desc *gpiod_rst;
};

/**
 * himax_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @cmd: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
int himax_i2c_read(struct i2c_client *client, u8 cmd, u8 *buf, u16 len)
{
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &cmd;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret >= 0)
		ret = (ret == ARRAY_SIZE(msgs) ? 0 : -EIO);

	if (ret)
		dev_err(&client->dev, "Error reading %d bytes from 0x%02x: %d\n",
		        len, cmd, ret);
	return ret;
}

/**
 * himax_i2c_write_buf - write data to a register of the i2c slave device.
 *
 * @client: i2c device.
 * @cmd: the register to write to.
 * @buf: raw data buffer to write.
 * @len: length of the buffer to write
 */
int himax_i2c_write(struct i2c_client *client, u8 *buf, u16 len)
{
	struct i2c_msg msg;
	int ret;

	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = buf;
	msg.len = len;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		ret = (ret == 1 ? 0 : -EIO);

	if (ret)
		dev_err(&client->dev, "Error writing %d bytes: %d\n", len, ret);
	return ret;
}

int himax_i2c_write_buf(struct i2c_client *client, u8 cmd, u8 *buf, u16 len)
{
	u8 *addr_buf;
	int ret;

	addr_buf = kmalloc(len + 1, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = cmd;
	memcpy(&addr_buf[1], buf, len);

	ret = himax_i2c_write(client, addr_buf, len + 1);

	kfree(addr_buf);

	return ret;
}

int himax_i2c_write_buf_single(struct i2c_client *client, u8 cmd, u8 value)
{
	return himax_i2c_write_buf(client, cmd, &value, sizeof(value));
}

int himax_i2c_write_buf_cmd(struct i2c_client *client, u8 cmd)
{
	return himax_i2c_write_buf(client, cmd, NULL, 0);
}

static int himax_read_event_stack(struct i2c_client *client, u8 *buf, u16 len)
{
	int error;

//	if (length > 56)
//		length = 128;

	error = himax_i2c_read(client, HX_CMD_RAE, buf, len);
	if (error)
		dev_err(&client->dev, "Failed to read event stack: %d", error);

	return error;
}

static int himax_read_touch_info(struct himax_ts_data *ts)
{
	int error;
	u8 buf[12];

	buf[0] = 0x14;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d", error);
	msleep(20);

	buf[0] = 0x00;
	buf[1] = 0x70;
	error = himax_i2c_write_buf(ts->client, 0x8B, buf, 2);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8B: %d", error);
	msleep(20);

	error = himax_i2c_read(ts->client, 0x5A, buf, 12);
	if (error) {
		dev_err(&ts->client->dev, "Failed to read 0x5A: %d", error);
	} else {
		//ic_data->HX_RX_NUM = buf[0]; // HX_RX_NUM
		//ic_data->HX_TX_NUM = buf[1]; // HX_TX_NUM
		//ic_data->HX_MAX_PT = buf[2] >> 4; // HX_MAX_PT
		//if (buf[4] & 0x04) { // HX_XY_REVERSE
		//	ic_data->HX_XY_REVERSE = true;
		//	ic_data->HX_Y_RES = buf[6] << 8 | buf[7]; // HX_Y_RES
		//	ic_data->HX_X_RES = buf[8] << 8 | buf[9]; // HX_X_RES
		//} else {
		//	ic_data->HX_XY_REVERSE = false;
		//	ic_data->HX_X_RES = buf[6] << 8 | buf[7]; // HX_X_RES
		//	ic_data->HX_Y_RES = buf[8] << 8 | buf[9]; // HX_Y_RES
		//}
	}

	buf[0] = 0x00;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d", error);
	msleep(20);

#ifdef HX_TP_PROC_2T2R
	buf[0] = 0x8C;
	buf[1] = 0x14;
	i2c_himax_master_write(client, &buf[0], 2, DEFAULT_RETRY_CNT);
	msleep(20);

	buf[0] = 0x8B;
	buf[1] = 0x00;
	buf[2] = HX_2T2R_Addr;
	i2c_himax_master_write(client, &buf[0], 3, DEFAULT_RETRY_CNT);
	msleep(20);

	i2c_himax_read(client, 0x5A, buf, 10, DEFAULT_RETRY_CNT);

	ic_data->HX_RX_NUM_2 = buf[0];
	ic_data->HX_TX_NUM_2 = buf[1];

	I("%s:Touch Panel Type=%d\n", __func__, buf[2]);
	if ((buf[2] & 0x02) == HX_2T2R_en_setting) /* 2T2R type panel */
		Is_2T2R = true;
	else
		Is_2T2R = false;

	buf[0] = 0x8C;
	buf[1] = 0x00;
	i2c_himax_master_write(client, &buf[0], 2, DEFAULT_RETRY_CNT);
	msleep(20);
#endif

	buf[0] = 0x14;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d", error);
	msleep(20);

	buf[0] = 0x00;
	buf[1] = 0x02;
	error = himax_i2c_write_buf(ts->client, 0x8B, buf, 2);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8B: %d", error);
	msleep(20);

	error = himax_i2c_read(ts->client, 0x5A, buf, 10);
	if (error) {
		dev_err(&ts->client->dev, "Failed to read 0x5A: %d", error);
	} else {
		if (buf[1] & 0x01) { // HX_INT_IS_EDGE
			//ic_data->HX_INT_IS_EDGE = true;
		} else {
			//ic_data->HX_INT_IS_EDGE = false;
		}
	}

	buf[0] = 0x00;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d", error);
	msleep(20);

	error = himax_i2c_read(ts->client, HX_VER_FW_CFG, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to read 0x5A: %d", error);
	else
		dev_dbg(&ts->client->dev, "HX_VER_FW_CFG = 0x%02X", buf[0]);

	return error;
}

/**
 * himax_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our himax_ts_data pointer
 *
 * Must be called during probe
 */
static int himax_request_input_dev(struct himax_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->name = "Himax Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
		        "Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

/**
 * himax_configure_dev - Finish device initialization
 *
 * @ts: our himax_ts_data pointer
 *
 * Must be called from probe to finish initialization of the device.
 * Contains the common initialization code for both devices that
 * declare gpio pins and devices that do not. It is either called
 * directly from probe or from request_firmware_wait callback.
 */
static int himax_configure_dev(struct himax_ts_data *ts)
{
	int error;

	error = himax_request_input_dev(ts);
	if (error)
		return error;

	return 0;
}

static int himax_get_gpio_config(struct himax_ts_data *ts)
{
	int error;
	struct gpio_desc *gpiod;

	/* Get the interrupt GPIO pin number */
	gpiod = devm_gpiod_get_optional(&ts->client->dev, "irq", GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		if (error != -EPROBE_DEFER)
			dev_dbg(&ts->client->dev, "Failed to get irq GPIO: %d\n", error);
		return error;
	}

	ts->gpiod_irq = gpiod;

	/* Get the reset GPIO pin number */
	gpiod = devm_gpiod_get_optional(&ts->client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		if (error != -EPROBE_DEFER)
			dev_dbg(&ts->client->dev, "Failed to get reset GPIO: %d\n", error);
		return error;
	}

	ts->gpiod_rst = gpiod;

	return 0;
}

static int himax_ts_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
	struct himax_ts_data *ts;
	int error;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	error = himax_get_gpio_config(ts);
	if (error)
		dev_err(&client->dev, "Failed to get GPIO config: %d", error);

	error = himax_read_touch_info(ts);
	if (error)
		dev_err(&client->dev, "Failed to read touch info: %d", error);

	error = himax_configure_dev(ts);
	if (error)
		return error;

	return 0;
}

static int himax_ts_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	return 0;
}

static int __maybe_unused himax_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int error;

	error = himax_i2c_write_buf_cmd(client, HX_CMD_TSSOFF);
	if (error)
		dev_err(&client->dev, "Failed to send TSSOFF command: %d\n", error);
	msleep(5);

	error = himax_i2c_write_buf_cmd(client, HX_CMD_TSSLPIN);
	if (error)
		dev_err(&client->dev, "Failed to send TSSLPIN command: %d\n", error);
	msleep(5);

	return 0;
}

static int __maybe_unused himax_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int error;

	error = himax_i2c_write_buf_cmd(client, HX_CMD_TSSLPOUT);
	if (error)
		dev_err(&client->dev, "Failed to send TSSLPOUT command: %d\n", error);
	msleep(5);

	error = himax_i2c_write_buf_cmd(client, HX_CMD_TSSON);
	if (error)
		dev_err(&client->dev, "Failed to send TSSON command: %d\n", error);
	msleep(5);

	return 0;
}

static SIMPLE_DEV_PM_OPS(himax_pm_ops, himax_suspend, himax_resume);

static const struct i2c_device_id himax_ts_id[] = {
	{ "hx852x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, himax_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id himax_of_match[] = {
	{ .compatible = "himax,hx852x" },
	{ }
};
MODULE_DEVICE_TABLE(of, himax_of_match);
#endif

static struct i2c_driver himax_ts_driver = {
	.probe = himax_ts_probe,
	.remove = himax_ts_remove,
	.id_table = himax_ts_id,
	.driver = {
		.name = "himax_ts",
		.of_match_table = of_match_ptr(himax_of_match),
		.pm = &himax_pm_ops,
	},
};
module_i2c_driver(himax_ts_driver);

MODULE_AUTHOR("Iscle Gil <albertiscle9@gmail.com>");
MODULE_DESCRIPTION("Himax touchscreen driver");
MODULE_LICENSE("GPL");
