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

#define HX_VER_FW_MAJ 0x33
#define HX_VER_FW_MIN 0x32
#define HX_VER_FW_CFG 0x39
#define HX_CMD_TSSLPIN 0x80
#define HX_CMD_TSSLPOUT 0x81
#define HX_CMD_TSSOFF 0x82
#define HX_CMD_TSSON 0x83
#define HX_CMD_ROE 0x85
#define HX_CMD_RAE 0x86
#define HX_CMD_RLE 0x87
#define HX_CMD_CLRES 0x88
#define HX_VER_IC 0xD1

struct himax_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;

	// Regulators
	struct regulator *avdd28;
	struct regulator *vddio;

	// GPIOs
	struct gpio_desc *gpiod_irq;
	struct gpio_desc *gpiod_rst;

	// Configuration
	uint8_t HX_RX_NUM;
	uint8_t HX_TX_NUM;
	uint8_t HX_MAX_PT;
	uint8_t HX_XY_REVERSE;
	uint16_t HX_X_RES;
	uint16_t HX_Y_RES;
	uint8_t HX_INT_IS_EDGE;
	struct touchscreen_properties prop;

	// Data
	uint8_t is_reset; // A device reset triggers an invalid IRQ
	uint8_t *touch_events;
	uint16_t touch_events_len;
	uint16_t touch_events_point_cnt_offset;
};

static int himax_i2c_read(struct i2c_client *client, uint8_t cmd, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].buf   = &cmd;
	msgs[0].len   = 1;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].buf   = buf;
	msgs[1].len   = len;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret >= 0)
		ret = (ret == ARRAY_SIZE(msgs) ? 0 : -EIO);

	if (ret)
		dev_err(&client->dev, "Error reading %d bytes from 0x%02x: %d\n",
		        len, cmd, ret);
	return ret;
}

static int himax_i2c_write(struct i2c_client *client, uint8_t *buf, uint16_t len)
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

static int himax_i2c_write_buf(struct i2c_client *client, uint8_t cmd, uint8_t *buf, uint16_t len)
{
	uint8_t *addr_buf;
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

static int himax_i2c_write_buf_cmd(struct i2c_client *client, uint8_t cmd)
{
	return himax_i2c_write_buf(client, cmd, NULL, 0);
}

static int himax_read_all_events(struct himax_ts_data *ts)
{
	int error;

	error = himax_i2c_read(ts->client, HX_CMD_RAE, ts->touch_events, ts->touch_events_len);
	if (error)
		dev_err(&ts->client->dev, "Failed to read event stack: %d\n", error);

	return error;
}

static int himax_calculate_checksum(struct himax_ts_data *ts)
{
	uint16_t i;
	uint16_t checksum;

	if (ts->is_reset) {
		ts->is_reset = 0;
		dev_dbg(&ts->client->dev, "Ignoring first IRQ after reset\n");
		return -1;
	}

	checksum = 0;
	for (i = 0; i < ts->touch_events_len; i++) {
		checksum += ts->touch_events[i];
	}

	if (checksum % 0x100) {
		dev_err(&ts->client->dev, "Invalid checksum: 0x%04X\n", checksum);
		return -2;
	}

	return 0;
}

static void himax_report_points(struct himax_ts_data *ts)
{
	uint8_t points;
	uint8_t i;
	uint16_t base;
	uint16_t x;
	uint16_t y;
	uint8_t w;

	points = ts->touch_events[ts->touch_events_point_cnt_offset] & 0x0F;

	if (points) {
		for (i = 0; i < ts->HX_MAX_PT; i++) {
			base = i * 4;
			x = ts->touch_events[base] << 8 | ts->touch_events[base + 1];
			y = ts->touch_events[base + 2] << 8 | ts->touch_events[base + 3];
			w = ts->touch_events[ts->HX_MAX_PT * 4 + i];

			if (x < ts->HX_X_RES && y < ts->HX_Y_RES) {
				touchscreen_report_pos(ts->input_dev, &ts->prop, x, y, true);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
				input_mt_sync(ts->input_dev);
			}
		}
	} else {
		input_mt_sync(ts->input_dev);
	}

	input_report_key(ts->input_dev, BTN_TOUCH, points > 0);
	input_sync(ts->input_dev);
}

static void himax_process_events(struct himax_ts_data *ts)
{
	int error;

	error = himax_read_all_events(ts);
	if (error)
		return;

	error = himax_calculate_checksum(ts);
	if (error)
		return;

	himax_report_points(ts);
}

static irqreturn_t himax_irq_handler(int irq, void *dev_id)
{
	struct himax_ts_data *ts = dev_id;

	himax_process_events(ts);

	return IRQ_HANDLED;
}

static int himax_request_irq(struct himax_ts_data *ts)
{
	long irq_flags;

	if (ts->HX_INT_IS_EDGE)
		irq_flags = IRQF_TRIGGER_FALLING;
	else
		irq_flags = IRQF_TRIGGER_LOW;

	return devm_request_threaded_irq(&ts->client->dev, ts->client->irq,
	                                  NULL, himax_irq_handler,
	                                  irq_flags | IRQF_ONESHOT, ts->client->name, ts);
}

static void himax_free_irq(struct himax_ts_data *ts)
{
	devm_free_irq(&ts->client->dev, ts->client->irq, ts);
}

static int himax_configure_dev(struct himax_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->name = "Himax capacitive touchscreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;

	input_set_capability(ts->input_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->HX_X_RES, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->HX_Y_RES, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	touchscreen_parse_properties(ts->input_dev, true, &ts->prop);

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
		        "Failed to register input device: %d", error);
		return error;
	}

	error = himax_request_irq(ts);
	if (error) {
		dev_err(&ts->client->dev, "Failed to request IRQ: %d\n", error);
		return error;
	}

	return 0;
}

static int himax_sleep_enable(struct himax_ts_data *ts)
{
	int error;

	error = himax_i2c_write_buf_cmd(ts->client, HX_CMD_TSSOFF);
	if (error)
		dev_err(&ts->client->dev, "Failed to send TSSOFF command: %d\n", error);
	msleep(50);

	error = himax_i2c_write_buf_cmd(ts->client, HX_CMD_TSSLPIN);
	if (error)
		dev_err(&ts->client->dev, "Failed to send TSSLPIN command: %d\n", error);
	msleep(50);

	return 0;
}

static int himax_sleep_disable(struct himax_ts_data *ts)
{
	int error;

	error = himax_i2c_write_buf_cmd(ts->client, HX_CMD_TSSLPOUT);
	if (error)
		dev_err(&ts->client->dev, "Failed to send TSSLPOUT command: %d\n", error);
	msleep(50);

	error = himax_i2c_write_buf_cmd(ts->client, HX_CMD_TSSON);
	if (error)
		dev_err(&ts->client->dev, "Failed to send TSSON command: %d\n", error);
	msleep(30);

	return 0;
}

static void himax_get_gpio_config(struct himax_ts_data *ts)
{
	int error;

	ts->avdd28 = devm_regulator_get(&ts->client->dev, "AVDD28");
	if (IS_ERR(ts->avdd28)) {
		error = PTR_ERR(ts->avdd28);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get AVDD28 regulator: %d\n", error);
	}

	ts->vddio = devm_regulator_get(&ts->client->dev, "VDDIO");
	if (IS_ERR(ts->vddio)) {
		error = PTR_ERR(ts->vddio);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get VDDIO regulator: %d\n", error);
	}

	ts->gpiod_irq = devm_gpiod_get_optional(&ts->client->dev, "irq", GPIOD_IN);
	if (IS_ERR(ts->gpiod_irq)) {
		error = PTR_ERR(ts->gpiod_irq);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get irq GPIO: %d\n", error);
	}

	ts->gpiod_rst = devm_gpiod_get_optional(&ts->client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->gpiod_rst)) {
		error = PTR_ERR(ts->gpiod_rst);
		if (error != -EPROBE_DEFER)
			dev_err(&ts->client->dev, "Failed to get reset GPIO: %d\n", error);
	}
}

static void himax_enable_regulators(struct himax_ts_data *ts)
{
	int error;

	error = regulator_enable(ts->avdd28);
	if (error)
		dev_err(&ts->client->dev, "Failed to enable AVDD28 regulator: %d\n", error);

	error = regulator_enable(ts->vddio);
	if (error)
		dev_err(&ts->client->dev, "Failed to enable VDDIO regulator: %d\n", error);
}

static void himax_reset(struct himax_ts_data *ts)
{
	int error;

	ts->is_reset = 1;

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
		msleep(20);
	} else {
		dev_err(&ts->client->dev, "Failed to reset, gpiod_rst is NULL!\n");
	}
}

static int himax_check_ic(struct himax_ts_data *ts)
{
	int error;
	uint8_t buf[3];

	error = himax_i2c_read(ts->client, HX_VER_IC, buf, 3);
	if (error) {
		dev_err(&ts->client->dev, "Failed to read HX_VER_IC: %d\n", error);
		return error;
	}

	dev_dbg(&ts->client->dev, "HX_VER_IC: 0x%06X\n", buf[0] << 16 | buf[1] << 8 | buf[2]);

	if (buf[0] == 0x05 && buf[1] == 0x85
	    && (buf[2] == 0x25 || buf[2] == 0x26
	        || buf[2] == 0x27 || buf[2] == 0x28))
		return 0;

	return -1;
}

static int himax_get_fw_ver(struct himax_ts_data *ts)
{
	int error;
	uint8_t buf[2];

	error = himax_i2c_read(ts->client, HX_VER_FW_MAJ, &buf[0], 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to read HX_VER_FW_MAJ: %d\n", error);

	error = himax_i2c_read(ts->client, HX_VER_FW_MIN, &buf[1], 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to read HX_VER_FW_MIN: %d\n", error);

	dev_dbg(&ts->client->dev, "HX_VER_FW: 0x%04X\n", buf[0] << 8 | buf[1]);

	error = himax_i2c_read(ts->client, HX_VER_FW_CFG, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to read HX_VER_FW_CFG: %d\n", error);

	dev_dbg(&ts->client->dev, "HX_VER_FW_CFG: 0x%02X\n", buf[0]);

	return 0;
}

static int himax_get_sensor_id(struct himax_ts_data *ts)
{
	int error;
	uint8_t buf[3];
	uint8_t val_high;
	uint8_t val_low;
	uint8_t id0;
	uint8_t id1;
	uint8_t sensor_id;

	buf[0] = 0x02;
	buf[1] = 0x02; /* ID pin pull high */
	error = himax_i2c_write_buf(ts->client, 0x56, buf, 2);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x56: %d\n", error);
	msleep(20);

	error = himax_i2c_read(ts->client, 0x57, &val_high, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to read 0x57: %d\n", error);

	buf[0] = 0x01;
	buf[1] = 0x01; /* ID pin pull low */
	error = himax_i2c_write_buf(ts->client, 0x56, buf, 2);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x56: %d\n", error);
	msleep(20);

	error = himax_i2c_read(ts->client, 0x57, &val_low, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to read 0x57: %d\n", error);

	if ((val_high & 0x01) != 0x01)
		id0 = 0x02; /* GND */
	else if ((val_low & 0x01) != 0x01)
		id0 = 0x01; /* Floating */
	else
		id0 = 0x04; /* VCC */

	if ((val_high & 0x02) != 0x02)
		id1 = 0x02; /* GND */
	else if ((val_low & 0x02) != 0x02)
		id1 = 0x01; /* Floating */
	else
		id1 = 0x04; /* VCC */

	if (id0 == 0x04 && id1 != 0x04) {
		buf[0] = 0x02;
		buf[1] = 0x01; /* ID pin pull high, low */
		error = himax_i2c_write_buf(ts->client, 0x56, buf, 2);
		if (error)
			dev_err(&ts->client->dev, "Failed to write 0x56: %d\n", error);
		msleep(20);
	} else if (id0 != 0x04 && id1 == 0x04) {
		buf[0] = 0x01;
		buf[1] = 0x02; /* ID pin pull low, high */
		error = himax_i2c_write_buf(ts->client, 0x56, buf, 2);
		if (error)
			dev_err(&ts->client->dev, "Failed to write 0x56: %d\n", error);
		msleep(20);
	} else if (id0 == 0x04 && id1 == 0x04) {
		buf[0] = 0x02;
		buf[1] = 0x02; /* ID pin pull high, high */
		error = himax_i2c_write_buf(ts->client, 0x56, buf, 2);
		if (error)
			dev_err(&ts->client->dev, "Failed to write 0x56: %d\n", error);
		msleep(20);
	}

	sensor_id = id1 << 4 | id0;

	dev_dbg(&ts->client->dev, "Sensor ID: 0x%02X\n", sensor_id);

	buf[0] = sensor_id;
	error = himax_i2c_write_buf(ts->client, 0xE4, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0xE4: %d\n", error);
	msleep(20);

	return 0;
}

static int himax_read_touch_info(struct himax_ts_data *ts)
{
	int error;
	uint8_t buf[12];

	error = himax_sleep_disable(ts);
	if (error)
		return error;

	error = himax_sleep_enable(ts);
	if (error)
		return error;

	buf[0] = 0x14;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d\n", error);
	msleep(20);

	buf[0] = 0x00;
	buf[1] = 0x70;
	error = himax_i2c_write_buf(ts->client, 0x8B, buf, 2);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8B: %d\n", error);
	msleep(20);

	error = himax_i2c_read(ts->client, 0x5A, buf, 12);
	if (error) {
		dev_err(&ts->client->dev, "Failed to read 0x5A: %d\n", error);
	} else {
		ts->HX_RX_NUM = buf[0];
		ts->HX_TX_NUM = buf[1];
		ts->HX_MAX_PT = buf[2] >> 4;
		ts->HX_XY_REVERSE = (buf[4] & 0x04) == 0x04;
		ts->HX_X_RES = buf[6] << 8 | buf[7];
		ts->HX_Y_RES = buf[8] << 8 | buf[9];

		dev_dbg(&ts->client->dev, "HX_RX_NUM: %u\n", ts->HX_RX_NUM);
		dev_dbg(&ts->client->dev, "HX_TX_NUM: %u\n", ts->HX_TX_NUM);
		dev_dbg(&ts->client->dev, "HX_MAX_PT: %u\n", ts->HX_MAX_PT);
		dev_dbg(&ts->client->dev, "HX_XY_REVERSE: %u\n", ts->HX_XY_REVERSE);
		dev_dbg(&ts->client->dev, "HX_X_RES: %u\n", ts->HX_X_RES);
		dev_dbg(&ts->client->dev, "HX_Y_RES: %u\n", ts->HX_Y_RES);
	}

	buf[0] = 0x00;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d\n", error);
	msleep(20);

	buf[0] = 0x14;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d\n", error);
	msleep(20);

	buf[0] = 0x00;
	buf[1] = 0x02;
	error = himax_i2c_write_buf(ts->client, 0x8B, buf, 2);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8B: %d\n", error);
	msleep(20);

	error = himax_i2c_read(ts->client, 0x5A, buf, 10);
	if (error) {
		dev_err(&ts->client->dev, "Failed to read 0x5A: %d\n", error);
	} else {
		ts->HX_INT_IS_EDGE = (buf[1] & 0x01) == 0x01;

		dev_dbg(&ts->client->dev, "HX_INT_IS_EDGE: %d\n", ts->HX_INT_IS_EDGE);
	}

	buf[0] = 0x00;
	error = himax_i2c_write_buf(ts->client, 0x8C, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to write 0x8C: %d\n", error);
	msleep(20);

	error = himax_i2c_read(ts->client, HX_VER_FW_CFG, buf, 1);
	if (error)
		dev_err(&ts->client->dev, "Failed to read HX_VER_FW_CFG: %d\n", error);
	else
		dev_dbg(&ts->client->dev, "HX_VER_FW_CFG: 0x%02X\n", buf[0]);

	error = himax_sleep_disable(ts);
	if (error)
		return error;

	return 0;
}

static int himax_init_touch_event_data(struct himax_ts_data *ts)
{
	int max;
	int rmd;

	max = ts->HX_MAX_PT / 4;
	rmd = ts->HX_MAX_PT % 4;

	if (rmd) {
		ts->touch_events_len = (ts->HX_MAX_PT + max + 2) * 4;
		ts->touch_events_point_cnt_offset = (ts->HX_MAX_PT + max + 1) * 4;
	} else {
		ts->touch_events_len = (ts->HX_MAX_PT + max + 1) * 4;
		ts->touch_events_point_cnt_offset = (ts->HX_MAX_PT + max) * 4;
	}

	ts->touch_events = devm_kzalloc(&ts->client->dev,
	                                ts->touch_events_len, GFP_KERNEL);
	if (!ts->touch_events)
		return -ENOMEM;

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

	himax_get_gpio_config(ts);

	himax_enable_regulators(ts);

	himax_reset(ts);

	error = himax_check_ic(ts);
	if (error) {
		dev_err(&client->dev, "Compatible Himax IC not found: %d\n", error);
		return error;
	}

	error = himax_get_fw_ver(ts);
	if (error)
		dev_err(&client->dev, "Failed to get firmware version: %d\n", error);

	error = himax_get_sensor_id(ts);
	if (error)
		dev_err(&client->dev, "Failed to get sensor ID: %d\n", error);

	error = himax_read_touch_info(ts);
	if (error) {
		dev_err(&client->dev, "Failed to read touch info: %d\n", error);
		return error;
	}

	error = himax_init_touch_event_data(ts);
	if (error) {
		dev_err(&client->dev, "Failed to initialize touch event data: %d\n", error);
		return error;
	}

	error = himax_configure_dev(ts);
	if (error) {
		dev_err(&client->dev, "Failed to configure device: %d\n", error);
		return error;
	}

	return 0;
}

static int __maybe_unused himax_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int error;

	himax_free_irq(ts);

	error = himax_sleep_enable(ts);
	if (error)
		dev_err(&ts->client->dev, "Failed to enter sleep: %d\n", error);

	return 0;
}

static int __maybe_unused himax_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int error;

	error = himax_sleep_disable(ts);
	if (error)
		dev_err(&ts->client->dev, "Failed to exit sleep: %d\n", error);

	error = himax_request_irq(ts);
	if (error)
		dev_err(&ts->client->dev, "Failed to request IRQ: %d\n", error);

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
	.id_table = himax_ts_id,
	.driver = {
		.name = "himax_ts",
		.of_match_table = of_match_ptr(himax_of_match),
		.pm = &himax_pm_ops,
	},
};
module_i2c_driver(himax_ts_driver);

MODULE_AUTHOR("Iscle Gil <albertiscle9@gmail.com>");
MODULE_DESCRIPTION("Himax capacitive touchscreen driver");
MODULE_LICENSE("GPL");
