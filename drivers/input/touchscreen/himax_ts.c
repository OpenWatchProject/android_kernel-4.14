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

struct himax_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
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
 * himax_i2c_write - write data to a register of the i2c slave device.
 *
 * @client: i2c device.
 * @cmd: the register to write to.
 * @buf: raw data buffer to write.
 * @len: length of the buffer to write
 */
int himax_i2c_write(struct i2c_client *client, u8 cmd, const u8 *buf, u16 len)
{
	u8 *addr_buf;
	struct i2c_msg msg;
	int ret;

	addr_buf = kmalloc(len + 1, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = cmd;
	memcpy(&addr_buf[1], buf, len);

	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = addr_buf;
	msg.len = len + 1;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		ret = (ret == 1 ? 0 : -EIO);

	kfree(addr_buf);

	if (ret)
		dev_err(&client->dev, "Error writing %d bytes to 0x%02x: %d\n",
			len, cmd, ret);
	return ret;
}

int himax_i2c_write_u8(struct i2c_client *client, u8 cmd, u8 value)
{
	return himax_i2c_write(client, cmd, &value, sizeof(value));
}

int himax_i2c_write_cmd(struct i2c_client *client, u8 cmd)
{
	return himax_i2c_write(client, cmd, NULL, 0);
}

static int himax_read_event_stack(struct i2c_client *client, u8 *buf, u16 len)
{
	int error;

	error = himax_i2c_read(client, HX_CMD_RAE, buf, len);
	if (error) 
		dev_err(&client->dev, "Failed to read event stack: %d", error);

	return 0;
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

	error = himax_i2c_write_cmd(client, HX_CMD_TSSOFF);
	if (error)
		dev_err(&client->dev, "Failed to send TSSOFF command: %d\n", error);
	msleep(5);

	error = himax_i2c_write_cmd(client, HX_CMD_TSSLPIN);
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

	error = himax_i2c_write_cmd(client, HX_CMD_TSSLPOUT);
	if (error)
		dev_err(&client->dev, "Failed to send TSSLPOUT command: %d\n", error);
	msleep(5);

	error = himax_i2c_write_cmd(client, HX_CMD_TSSON);
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
