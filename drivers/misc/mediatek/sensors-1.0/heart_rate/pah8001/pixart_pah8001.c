#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include "hwmsensor.h"
#include "sensor_attr.h"
#include "sensor_event.h"
#include "pixart_pah8001.h"
#include "gpio_driver.h"

#define DEV_HEART_RATE "m_hrs_misc"
#define PAH8001_NAME "pixart_pah8001"
#define PAH8001_ADDR 0x33

struct sensor_attr_t pixart_device;
static pah8001_data_t pah8001data;

static int pah8001_i2c_write(char reg, char *data, int len) {
	char buf[20];
	int rc = 0, i = 0, ret = 0;
	buf[0] = reg;

	for (i = 0; i < len; i++) {
		buf[i + 1] = data[i];
	}

	rc = i2c_master_send(pah8001data.client, buf, len + 1);
	if (rc != len + 1) {
		printk("%s: FAILED: writing to reg 0x%X\n", __func__, reg);
		ret = -1;
	}

	return ret;
}

static int pah8001_i2c_read(char reg, char *data) {
	char buf[20];
	int rc = 0;
	buf[0] = reg;

	rc = i2c_master_send(pah8001data.client, buf, 1);
	if (rc != 1) {
		printk("%s: FAILED: writing to address 0x%X\n", __func__, reg);
		return -1;
	}

	rc = i2c_master_recv(pah8001data.client, buf, 1);
	if (rc != 1) {
		printk("%s: FAILED: reading data\n", __func__);
		return -1;
	}

	*data = buf[0];
	return 0;
}

static int pah8001_i2c_burst_read(char reg, char *data, char len) {
	char buf[256];
	int rc = 0;
	buf[0] = reg;

	rc = i2c_master_send(pah8001data.client, buf, 1);
	if (rc != 1) {
		printk("%s: FAILED: writing to address 0x%X\n", __func__, reg);
		return -1;
	}

	rc = i2c_master_recv(pah8001data.client, data, len);
	if (rc != len) {
		printk("%s: FAILED: reading data %d\n", __func__, rc);
		return -1;
	}

	return 0;
}

static unsigned char pah8001_write_reg(unsigned char addr, unsigned char data) {
	return pah8001_i2c_write(addr, &data, 1);
}

static unsigned char pah8001_read_reg(unsigned char addr, unsigned char *data) {
	return pah8001_i2c_read(addr, data);
}

static unsigned char pah8001_burst_read_reg(unsigned char addr, unsigned char *data, unsigned int length) {
	return pah8001_i2c_burst_read(addr, data, length);
}

static void led_ctrl(uint8_t touch) {
	uint8_t data;
	uint16_t EP_L, EP_H, Exposure_Line;

	if ((touch == 0x80) || (_led_step < 31)) {
		pah8001_write_reg(0x7F, 0x00);
		pah8001_write_reg(0x05, 0x98);

		pah8001_read_reg(0x33, &data);
		EP_H = data & 0x03;
		pah8001_read_reg(0x32, &data);
		EP_L = data;

		Exposure_Line = (EP_H << 8) + EP_L;
		pah8001_write_reg(0x7F, 0x01);

		if (_sleepflag == 1) {
			pah8001_write_reg(0x38, (0xE0 | DEFAULT_LED_STEP));
			_led_step = DEFAULT_LED_STEP;
			_sleepflag = 0;
		}

		if (_state_count <= STATE_COUNT_TH) {
			_state_count++;
			_led_change_flag = 0;
		} else {
			_state_count = 0;
			if (_state == 0) {
				if ((Exposure_Line >= LED_CTRL_EXPO_TIME_HI_BOUND) ||
						(Exposure_Line <= LED_CTRL_EXPO_TIME_LOW_BOUND)) {
					pah8001_read_reg(0x38, &data);
					_led_step = data & 0x1F;

					if ((Exposure_Line >= LED_CTRL_EXPO_TIME_HI_BOUND) &&
							(_led_step < LED_CURRENT_HI)) {
						_state = 1;
						_led_step = _led_step + LED_INC_DEC_STEP;

						if (_led_step > LED_CURRENT_HI) {
							_led_step = LED_CURRENT_HI;
						}

						pah8001_write_reg(0x38, (_led_step | 0xE0));
						_led_change_flag = 1;
					} else if ((Exposure_Line <= LED_CTRL_EXPO_TIME_LOW_BOUND) &&
							(_led_step > LED_CURRENT_LOW)) {
						_state = 2;

						if (_led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP)) {
							_led_step = LED_CURRENT_LOW;
						} else {
							_led_step = _led_step - LED_INC_DEC_STEP;
						}

						pah8001_write_reg(0x38, (_led_step | 0xE0));
						_led_change_flag = 1;
					} else {
						_state = 0;
						_led_change_flag = 0;
					}
				} else {
					_led_change_flag = 0;
				}
			} else if (_state == 1) {
				if (Exposure_Line > LED_CTRL_EXPO_TIME_HI) {
					_state = 1;
					_led_step = _led_step + LED_INC_DEC_STEP;

					if (_led_step >= LED_CURRENT_HI) {
						_state = 0;
						_led_step = LED_CURRENT_HI;
					}

					pah8001_write_reg(0x38, (_led_step | 0xE0));
					_led_change_flag = 1;
				} else {
					_state = 0;
					_led_change_flag = 0;
				}
			} else {
				if (Exposure_Line < LED_CTRL_EXPO_TIME_LOW) {
					_state = 2;

					if (_led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP)) {
						_state = 0;
						_led_step = LED_CURRENT_LOW;
					} else {
						_led_step = _led_step - LED_INC_DEC_STEP;
					}

					pah8001_write_reg(0x38, (_led_step | 0xE0));
					_led_change_flag = 1;
				} else {
					_state = 0;
					_led_change_flag = 0;
				}
			}
		}
	} else {
		pah8001_write_reg(0x7F, 0x00);
		pah8001_write_reg(0x05, 0xB8);
		pah8001_write_reg(0x7F, 0x01);
		pah8001_write_reg(0x38, 0xFF);
		_sleepflag = 1;
		_led_change_flag = 0;
	}
}

static void pah8001_power_down(char value) {
	if (value) {
		pah8001_write_reg(0x7F, 0x00);
		pah8001_write_reg(0x06, 0x0A);
		gpio_set(pin10_gpio_out1);
		gpio_set(pin8_gpio_out1);
	} else {
		gpio_set(pin10_gpio_out0);
		pah8001_write_reg(0x7F, 0x00);
		pah8001_write_reg(0x06, 0x02);
	}
}

static int pah8001_chip_id(void) {
	char data0 = 0, data1 = 0;

	pah8001_write_reg(0x7F, 0x00);
	pah8001_read_reg(0x00, &data0);
	pah8001_read_reg(0x01, &data1);

	printk("%s: ADDR0: 0x%X, ADDR1: 0x%X\n", __func__, data0, data1);

	if ((data0 != 0x30) || ((data1 & 0xF0) != 0xD0)) {
		return 1;
	}

	return 0;
}

static int pah8001_init_reg(void) {
	int i = 0, bank = 0;
	char data = 0;

	for (i = 0; i < ARRAY_SIZE(init_ppg_register_array); i++) {
		if (init_ppg_register_array[i][0] == 0x7F) {
			bank = init_ppg_register_array[i][1];
		}

		if ((bank == 0) && (init_ppg_register_array[i][0] == 0x17)) {
			pah8001_read_reg(0x17, &data);
			data |= 0x80;
			pah8001_write_reg(0x17, data);
		} else {
			pah8001_write_reg(init_ppg_register_array[i][0], init_ppg_register_array[i][1]);
		}
	}

	return 0;
}

static void pah8001_ppg(void) {
	struct sensor_event event;
	static unsigned long volatile start_jiffies = 0, end_jiffies = 0;
	static char Frame_Count = 0;
	char touch_flag = 0, data = 0;

	if (pah8001data.run_ppg) {
		pah8001_write_reg(0x7F, 0x00);
		pah8001_read_reg(0x59, &touch_flag);
		touch_flag &= 0x80;
		led_ctrl(touch_flag);

		pah8001_write_reg(0x7F, 0x01);
		pah8001_read_reg(0x68, &data);
		pah8001data.ppg_mems_data.HRD_Data[0] = data & 0x0F;

		if (pah8001data.ppg_mems_data.HRD_Data[0] == 0) {
			pah8001_write_reg(0x7F, 0x00);
			msleep(10);
		} else {
			pah8001_burst_read_reg(0x64, &(pah8001data.ppg_mems_data.HRD_Data[1]), 4);
			pah8001_burst_read_reg(0x1A, &(pah8001data.ppg_mems_data.HRD_Data[5]), 3);

			pah8001data.ppg_mems_data.HRD_Data[8] = Frame_Count++;
			end_jiffies = jiffies;
			pah8001data.ppg_mems_data.HRD_Data[9] = jiffies_to_msecs(end_jiffies - start_jiffies);
			start_jiffies = end_jiffies;

			pah8001data.ppg_mems_data.HRD_Data[10] = _led_change_flag;
			pah8001data.ppg_mems_data.HRD_Data[11] = touch_flag;
			pah8001data.ppg_mems_data.HRD_Data[12] = pah8001data.ppg_mems_data.HRD_Data[6];

			event.flush_action = DATA_ACTION;
			event.handle = ID_HEART_RATE;

			event.word[0] = *(uint32_t *)(pah8001data.ppg_mems_data.HRD_Data);
			event.word[1] = *(uint32_t *)(pah8001data.ppg_mems_data.HRD_Data + 4);
			event.word[2] = *(uint32_t *)(pah8001data.ppg_mems_data.HRD_Data + 8);
			event.word[5] = 0x00; // Report HeartRate Data
			sensor_input_event(pixart_device.minor, &event);
		}
	}
}

static void pah8001_x_work_func(struct work_struct *work) {
	pah8001_power_down(0);

	while (pah8001data.run_ppg) {
		pah8001_ppg();
	}

	pah8001_power_down(1);
}

static int pah8001_open(struct inode *inode, struct file *file) {
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t pah8001_read(struct file *file, char *buf, size_t count, loff_t *pos) {
	return sensor_event_read(pixart_device.minor, file, buf, count, pos);
}

static unsigned int pah8001_poll(struct file *file, poll_table *wait) {
	return sensor_event_poll(pixart_device.minor, file, wait);
}

static struct file_operations pah8001_fops = {
	.owner = THIS_MODULE,
	.open = pah8001_open,
	.read = pah8001_read,
	.poll = pah8001_poll,
};

struct sensor_attr_t pixart_device = {
	.minor = ID_HEART_RATE,
	.name = DEV_HEART_RATE,
	.fops = &pah8001_fops,
};

static ssize_t pah8001_active_store(struct device* dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0;
	sscanf(buf, "%d", &enable);

	if (enable) {
		printk("%s: enable\n", __func__);
		pah8001data.run_ppg = true;
		schedule_delayed_work(&pah8001data.x_work, msecs_to_jiffies(100));
	} else {
		printk("%s: disable\n", __func__);
		pah8001data.run_ppg = false;
	}

	return count;
}

static ssize_t pah8001_active_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", pah8001data.run_ppg);
}

static ssize_t pah8001_no_support_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported");
}

static DEVICE_ATTR(hrs_active, 0644, pah8001_active_show, pah8001_active_store);
static DEVICE_ATTR(hrs_batch, 0444, pah8001_no_support_show, NULL);
static DEVICE_ATTR(hrs_flush, 0444, pah8001_no_support_show, NULL);

static struct attribute *pah8001_attr_list[] = {
	&dev_attr_hrs_active.attr,
	&dev_attr_hrs_batch.attr,
	&dev_attr_hrs_flush.attr,
	NULL,
};

static struct attribute_group pah8001_attribute_group = {
	.attrs = pah8001_attr_list
};

static int pah8001_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	printk("%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		return -1;
	}

	client->addr = PAH8001_ADDR;
	pah8001data.client = client;

	gpio_set(pin10_gpio_out1);
	gpio_set(pin8_gpio_out1);
	gpio_set(pin11_gpio_in_eint);
	mdelay(50);
	gpio_set(pin9_ldo_out1);
	mdelay(50);

	gpio_set(pin8_gpio_out0);
	mdelay(50);
	gpio_set(pin8_gpio_out1);
	mdelay(50);

	pah8001_power_down(0);

	if (sensor_attr_register(&pixart_device)) {
		printk("%s: sensor_attr_register failed!\n", __func__);
		goto error;
	}

	if (sysfs_create_group(&pixart_device.this_device->kobj, &pah8001_attribute_group)) {
		printk("%s: sysfs_create_group failed!\n", __func__);
		goto error;
	}

	if (pah8001_chip_id()) {
		printk("%s: pah8001_chip_id error!\n", __func__);
		goto error;
	}

	pah8001_init_reg();
	pah8001_power_down(1);

	INIT_DELAYED_WORK(&pah8001data.x_work, pah8001_x_work_func);
	pah8001data.run_ppg = false;

	return 0;

error:
	gpio_set(pin9_ldo_out0);
	gpio_set(pin8_gpio_out0);
	gpio_set(pin10_gpio_out0);

	return -1;
}

static int pah8001_i2c_remove(struct i2c_client *client) {
	return 0;
}

static const struct i2c_device_id pah8001_device_id[] = {
	{"pixart_pah8001", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pah8001_device_id);

static struct of_device_id pah8001_i2c_match_table[] = {
	{.compatible = "mediatek,heartrate",},
	{},
};

static struct i2c_driver pah8001_i2c_driver = {
	.driver = {
		.name = PAH8001_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pah8001_i2c_match_table,
	},
	.probe = pah8001_i2c_probe,
	.remove = pah8001_i2c_remove,
	.id_table = pah8001_device_id,
};

static int __init pah8001_init(void) {
	if (i2c_add_driver(&pah8001_i2c_driver) != 0) {
		printk("%s: i2c_add_driver failed\n", __func__);
	}

	return 0;
}

static void __exit pah8001_exit(void) {
	sensor_attr_deregister(&pixart_device);
	sysfs_remove_group(&pixart_device.this_device->kobj, &pah8001_attribute_group);
	i2c_del_driver(&pah8001_i2c_driver);
}

module_init(pah8001_init);
module_exit(pah8001_exit);
