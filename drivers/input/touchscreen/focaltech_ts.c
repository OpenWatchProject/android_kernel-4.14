// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#if defined(CONFIG_FB)
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

//#define dev_dbg dev_err

#define FT_POINT_SIZE 6
#define FT_REG_TD_STATUS 0x02
#define FT_REG_CHIP_ID2 0x9F
#define FT_REG_CHIP_ID 0xA3
#define FT_REG_POWER_MODE 0xA5

struct focaltech_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;

	// Regulators
	struct regulator *avdd28;
	struct regulator *vddio;

	// GPIOs
	struct gpio_desc *gpiod_irq;
	struct gpio_desc *gpiod_rst;

	// Configuration
	uint8_t max_points;
	uint16_t x_res;
	uint16_t y_res;
	struct touchscreen_properties prop;

	// Data
	uint8_t *touch_events;
	uint16_t touch_events_len;
	uint8_t suspended;
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
};

static int focaltech_i2c_read_raw(struct i2c_client *client, uint8_t *cmd, uint16_t cmd_len, uint8_t *buf, uint16_t buf_len)
{
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr = client->addr;
	msgs[0].buf = cmd;
	msgs[0].len = cmd_len;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].buf = buf;
	msgs[1].len = buf_len;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret >= 0)
		ret = (ret == ARRAY_SIZE(msgs) ? 0 : -EIO);

	if (ret)
		dev_err(&client->dev, "Error reading %d bytes from 0x%02x: %d\n",
		        buf_len, cmd, ret);
	return ret;
}

static int focaltech_i2c_read(struct i2c_client *client, uint8_t cmd, uint8_t *buf, uint16_t len)
{
	return focaltech_i2c_read_raw(client, &cmd, 1, buf, len);
}

static int focaltech_i2c_write_raw(struct i2c_client *client, uint8_t *buf, uint16_t len)
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

static int focaltech_i2c_write(struct i2c_client *client, uint8_t cmd, uint8_t *buf, uint16_t len)
{
	uint8_t *addr_buf;
	int ret;

	addr_buf = kmalloc(len + 1, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = cmd;
	memcpy(&addr_buf[1], buf, len);

	ret = focaltech_i2c_write_raw(client, addr_buf, len + 1);

	kfree(addr_buf);

	return ret;
}

static int focaltech_i2c_write_reg(struct i2c_client *client, uint8_t cmd, uint8_t data)
{
	return focaltech_i2c_write(client, cmd, &data, 1);
}

static int focaltech_read_events(struct focaltech_ts_data *ts)
{
	int error;

	error = focaltech_i2c_read(ts->client, FT_REG_TD_STATUS, ts->touch_events, ts->touch_events_len);
	if (error)
		dev_err(&ts->client->dev, "Failed to read events: %d\n", error);

	return error;
}

static void focaltech_report_points(struct focaltech_ts_data *ts)
{
	uint8_t points;
	int i;
	uint16_t base;
	uint8_t flag;
	uint16_t x;
	uint8_t id;
	uint16_t y;

	points = ts->touch_events[0] & 0x0F;

	if (points) {
		for (i = 0; i < ts->max_points; i++) {
			base = 1 + i * FT_POINT_SIZE;
			flag = (ts->touch_events[base] >> 6) & 0x03;
			x = (ts->touch_events[base] << 8 | ts->touch_events[base + 1]) & 0x0FFF;
			id = (ts->touch_events[base + 2] >> 4) & 0x0F;
			y = (ts->touch_events[base + 2] << 8 | ts->touch_events[base + 3]) & 0x0FFF;

			if (x < ts->x_res && y < ts->y_res) {
				touchscreen_report_pos(ts->input_dev, &ts->prop, x, y, true);
				input_mt_sync(ts->input_dev);
			}
		}
	} else {
		input_mt_sync(ts->input_dev);
	}

	input_report_key(ts->input_dev, BTN_TOUCH, points > 0);
	input_sync(ts->input_dev);
}

static void focaltech_process_events(struct focaltech_ts_data *ts)
{
	int error;

	error = focaltech_read_events(ts);
	if (error)
		return;

	focaltech_report_points(ts);
}

static irqreturn_t focaltech_irq_handler(int irq, void *dev_id)
{
	struct focaltech_ts_data *ts = dev_id;

	focaltech_process_events(ts);

	return IRQ_HANDLED;
}

static int focaltech_request_irq(struct focaltech_ts_data *ts)
{
	return devm_request_threaded_irq(&ts->client->dev, ts->client->irq,
	                                 NULL, focaltech_irq_handler,
	                                 IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ts->client->name, ts);
}

static void focaltech_free_irq(struct focaltech_ts_data *ts)
{
	devm_free_irq(&ts->client->dev, ts->client->irq, ts);
}

static int focaltech_configure_dev(struct focaltech_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->name = "FocalTech capacitive touchscreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;

	input_set_capability(ts->input_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->x_res, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->y_res, 0, 0);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	touchscreen_parse_properties(ts->input_dev, true, &ts->prop);

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
		        "Failed to register input device: %d", error);
		return error;
	}

	error = focaltech_request_irq(ts);
	if (error) {
		dev_err(&ts->client->dev, "Failed to request IRQ: %d\n", error);
		return error;
	}

	return 0;
}

static int focaltech_sleep_enable(struct focaltech_ts_data *ts)
{
	int error;

	error = focaltech_i2c_write_reg(ts->client, FT_REG_POWER_MODE, 0x03);
	if (error) {
		dev_err(&ts->client->dev, "Failed to write to FT_REG_POWER_MODE register: %d\n", error);
		return error;
	}

	return 0;
}

static int focaltech_sleep_disable(struct focaltech_ts_data *ts)
{
	int error;

	error = focaltech_i2c_write_reg(ts->client, FT_REG_POWER_MODE, 0x00);
	if (error) {
		dev_err(&ts->client->dev, "Failed to write to FT_REG_POWER_MODE register: %d\n", error);
		return error;
	}

	return 0;
}

static void focaltech_parse_dtb(struct focaltech_ts_data *ts)
{
	int error;

	ts->avdd28 = devm_regulator_get(&ts->client->dev, "AVDD28");
	if (IS_ERR(ts->avdd28)) {
		error = PTR_ERR(ts->avdd28);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get AVDD28 regulator: %d\n", error);
		ts->avdd28 = NULL;
	}

	ts->vddio = devm_regulator_get(&ts->client->dev, "VDDIO");
	if (IS_ERR(ts->vddio)) {
		error = PTR_ERR(ts->vddio);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get VDDIO regulator: %d\n", error);
		ts->vddio = NULL;
	}

	ts->gpiod_irq = devm_gpiod_get_optional(&ts->client->dev, "irq", GPIOD_IN);
	if (IS_ERR(ts->gpiod_irq)) {
		error = PTR_ERR(ts->gpiod_irq);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get irq GPIO: %d\n", error);
		ts->gpiod_irq = NULL;
	}

	ts->gpiod_rst = devm_gpiod_get_optional(&ts->client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->gpiod_rst)) {
		error = PTR_ERR(ts->gpiod_rst);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get reset GPIO: %d\n", error);
		ts->gpiod_rst = NULL;
	}
}

static void focaltech_enable_regulators(struct focaltech_ts_data *ts)
{
	int error;

	if (ts->avdd28) {
		error = regulator_enable(ts->avdd28);
		if (error)
			dev_err(&ts->client->dev, "Failed to enable AVDD28 regulator: %d\n", error);
	}

	if (ts->vddio) {
		error = regulator_enable(ts->vddio);
		if (error)
			dev_err(&ts->client->dev, "Failed to enable VDDIO regulator: %d\n", error);
	}
}

static void focaltech_reset(struct focaltech_ts_data *ts)
{
	int error;

	if (ts->gpiod_rst) {
		error = gpiod_direction_output(ts->gpiod_rst, 1);
		if (error)
			dev_err(&ts->client->dev, "Failed to set gpiod_rst: %d\n", error);
		msleep(20);

		error = gpiod_direction_output(ts->gpiod_rst, 0);
		if (error)
			dev_err(&ts->client->dev, "Failed to clear gpiod_rst: %d\n", error);
		msleep(20);

		error = gpiod_direction_output(ts->gpiod_rst, 1);
		if (error)
			dev_err(&ts->client->dev, "Failed to set gpiod_rst: %d\n", error);
		msleep(200); // TODO: 20
	} else {
		dev_err(&ts->client->dev, "Failed to reset, gpiod_rst is NULL!\n");
	}
}

static int focaltech_check_ic(struct focaltech_ts_data *ts)
{
	int error;
	uint8_t buf[2];
	int i;
	uint8_t chip_ids[][2] = {
		{0x58, 0x22},
		{0x54, 0x22},
		{0x64, 0x26},
		{0x33, 0x67},
		{0x87, 0x16},
		{0x87, 0x36},
		{0x80, 0x06},
		{0x86, 0x07},
		{0xF0, 0x06},
		{0x86, 0x13},
		{0x87, 0x19},
		{0x87, 0x39},
		{0x86, 0x15},
		{0x82, 0x01},
		{0x86, 0x22},
		{0x72, 0x51},
		{0x72, 0x52},
		{0xF6, 0x13},
		{0x54, 0x52},
		{0x54, 0x22},
		{0x54, 0x56},
		{0x62, 0x16},
	};

	for (i = 0; i < 3; i++) {
		error = focaltech_i2c_read(ts->client, FT_REG_CHIP_ID, &buf[0], 1);
		if (error) {
			dev_err(&ts->client->dev, "Failed to read FT_REG_CHIP_ID: %d\n", error);
			return error;
		}

		error = focaltech_i2c_read(ts->client, FT_REG_CHIP_ID2, &buf[1], 1);
		if (error) {
			dev_err(&ts->client->dev, "Failed to read FT_REG_CHIP_ID2: %d\n", error);
			return error;
		}

		if (buf[0] != 0x00 && buf[1] != 0x00)
			break;
	}

	if (i == 3) {
		dev_err(&ts->client->dev, "Failed to read chip id\n");
		return -EIO;
	}

	dev_dbg(&ts->client->dev, "FT_REG_CHIP_ID: 0x%02X, FT_REG_CHIP_ID2: 0x%02X\n", buf[0], buf[1]);

	for (i = 0; i < ARRAY_SIZE(chip_ids); i++) {
		if (chip_ids[i][0] == buf[0] && chip_ids[i][1] == buf[1])
			break;
	}

	if (i == ARRAY_SIZE(chip_ids))
		return -ENODEV;

	return 0;
}

static int focaltech_read_touch_info(struct focaltech_ts_data *ts)
{
	// TODO
	ts->max_points = 2;
	ts->x_res = 400;
	ts->y_res = 400;

	return 0;
}

static int focaltech_init_touch_event_data(struct focaltech_ts_data *ts)
{
	ts->touch_events_len = 1 + (ts->max_points * FT_POINT_SIZE);
	ts->touch_events = devm_kzalloc(&ts->client->dev,
	                                ts->touch_events_len, GFP_KERNEL);
	if (!ts->touch_events)
		return -ENOMEM;

	return 0;
}

static void focaltech_common_suspend(struct focaltech_ts_data *ts)
{
	if (ts->suspended) {
		dev_dbg(&ts->client->dev, "Already suspended!\n");
		return;
	}

	ts->suspended = 1;

	focaltech_free_irq(ts);

	focaltech_sleep_enable(ts);
}

static void focaltech_common_resume(struct focaltech_ts_data *ts)
{
	int error;

	dev_dbg(&ts->client->dev, "Resuming...\n");

	if (!ts->suspended) {
		dev_dbg(&ts->client->dev, "Not suspended!\n");
		return;
	}

	ts->suspended = 0;

	focaltech_sleep_disable(ts);

	error = focaltech_request_irq(ts);
	if (error)
		dev_err(&ts->client->dev, "Failed to request IRQ: %d\n", error);
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *nb,
                                unsigned long action, void *data)
{
	struct fb_event *evdata = data;
	struct focaltech_ts_data *ts = container_of(nb, struct focaltech_ts_data, fb_notif);
	int fb_blank = *((int *)evdata->data);

	if (action != FB_EVENT_BLANK)
		return 0;

	switch (fb_blank) {
	case FB_BLANK_UNBLANK:
		focaltech_common_resume(ts);
		break;
	case FB_BLANK_POWERDOWN:
		focaltech_common_suspend(ts);
		break;
	}

	return 0;
}
#endif

static int focaltech_ts_probe(struct i2c_client *client,
                              const struct i2c_device_id *id)
{
	struct focaltech_ts_data *ts;
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

	focaltech_parse_dtb(ts);

	focaltech_enable_regulators(ts);

	focaltech_reset(ts);

	error = focaltech_check_ic(ts);
	if (error) {
		dev_err(&client->dev, "Compatible FocalTech IC not found: %d\n", error);
		return error;
	}

	error = focaltech_read_touch_info(ts);
	if (error) {
		dev_err(&client->dev, "Failed to read touch info: %d\n", error);
		return error;
	}

	error = focaltech_init_touch_event_data(ts);
	if (error) {
		dev_err(&client->dev, "Failed to initialize touch event data: %d\n", error);
		return error;
	}

	error = focaltech_configure_dev(ts);
	if (error) {
		dev_err(&client->dev, "Failed to configure device: %d\n", error);
		return error;
	}

#ifdef CONFIG_FB
	ts->fb_notif.notifier_call = fb_notifier_callback;
	error = fb_register_client(&ts->fb_notif);
	if (error)
		dev_err(&client->dev, "Failed to register framebuffer client: %d\n", error);
#endif

	return 0;
}

#ifndef CONFIG_FB
static int __maybe_unused focaltech_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct focaltech_ts_data *ts = i2c_get_clientdata(client);

	focaltech_common_suspend(ts);

	return 0;
}

static int __maybe_unused focaltech_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct focaltech_ts_data *ts = i2c_get_clientdata(client);

	focaltech_common_resume(ts);

	return 0;
}

static SIMPLE_DEV_PM_OPS(focaltech_pm_ops, focaltech_suspend, focaltech_resume);
#endif

static const struct i2c_device_id focaltech_ts_id[] = {
	{"ft3267", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, focaltech_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id focaltech_of_match[] = {
	{.compatible = "focaltech,ft3267"},
	{}
};
MODULE_DEVICE_TABLE(of, focaltech_of_match);
#endif

static struct i2c_driver focaltech_ts_driver = {
	.probe = focaltech_ts_probe,
	.id_table = focaltech_ts_id,
	.driver = {
		.name = "focaltech_ts",
		.of_match_table = of_match_ptr(focaltech_of_match),
#ifndef CONFIG_FB
		.pm = &focaltech_pm_ops,
#endif
	},
};
module_i2c_driver(focaltech_ts_driver);

MODULE_AUTHOR("Iscle Gil <albertiscle9@gmail.com>");
MODULE_DESCRIPTION("FocalTech capacitive touchscreen driver");
MODULE_LICENSE("GPL");
