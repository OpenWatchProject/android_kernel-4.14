#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include "pah8001_driver.h"
#include "pah8001_led_ctrl.h"

// This is temporary
#include "hwmsensor.h"
#include "sensor_attr.h"
#include "sensor_event.h"
#define DEV_HEART_RATE "m_hrs_misc"

static struct of_device_id pah8001_match_table[] = {
	{.compatible = "mediatek,pah8001"},
	{.compatible = "mediatek,hrsensor"},
	{},
};
MODULE_DEVICE_TABLE(of, pah8001_match_table);

#define PAH_TAG "[pah8001] "
#define PAH_ERR(fmt, args...) printk(PAH_TAG "%s, %d: " fmt, __func__, __LINE__, ##args)
#define PAH_DBG(fmt, args...) printk(PAH_TAG "%s, %d: " fmt, __func__, __LINE__, ##args)
#define PAH_LOG(fmt, args...) printk(PAH_TAG "%s, %d: " fmt, __func__, __LINE__, ##args)
#define PAH_FUN() printk(PAH_TAG "%s, %d\n", __func__, __LINE__)

static pah8001_data_t pah8001_data = {0};
static struct pinctrl *pctrl;
static struct pinctrl_state *ldo_pin_state0;
static struct pinctrl_state *ldo_pin_state1;
static struct pinctrl_state *rst_pin_state0;
static struct pinctrl_state *rst_pin_state1;
static struct pinctrl_state *pdn_pin_state0;
static struct pinctrl_state *pdn_pin_state1;
static struct sensor_attr_t pah8001_attr;

static int pah8001_i2c_write(uint8_t reg, uint8_t *data, int len)
{
	int ret = 0;
	struct i2c_msg msg[] = {
		{
			.addr = pah8001_data.client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = pah8001_data.client->addr,
			.flags = I2C_M_NOSTART,
			.len = len,
			.buf = data,
		}
	};

	ret = i2c_transfer(pah8001_data.client->adapter, msg, 2);
	if (ret != 2) {
		PAH_ERR("i2c_transfer failed: %d\n", ret);
		if (ret >= 0) {
			ret = -EIO;
		}
	} else {
		ret = 0;
	}

	return ret;
}

static int pah8001_i2c_read(uint8_t reg, uint8_t *data, int len)
{
	int ret = 0;
	struct i2c_msg msg[] = {
		{
			.addr = pah8001_data.client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = pah8001_data.client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		}
	};

	ret = i2c_transfer(pah8001_data.client->adapter, msg, 2);
	if (ret != 2) {
		PAH_ERR("i2c_transfer failed: %d\n", ret);
		if (ret >= 0) {
			ret = -EIO;
		}
	} else {
		ret = 0;
	}

	return ret;
}

int pah8001_write_reg(uint8_t reg, uint8_t data)
{
	return pah8001_i2c_write(reg, &data, 1);
}

int pah8001_read_reg(uint8_t reg, uint8_t *data)
{
	return pah8001_i2c_read(reg, data, 1);
}

static int pah8001_enable_regulator(uint8_t enable)
{
	int err = 0;

	if (enable) {
		err = pinctrl_select_state(pctrl, ldo_pin_state1);
	} else {
		err = pinctrl_select_state(pctrl, ldo_pin_state0);
	}

	if (err)
		PAH_ERR("pinctrl_select_state failed: %d\n", err);

	return err;
}

static int pah8001_power_down(uint8_t power_down)
{
	int err = 0;

	pah8001_write_reg(0x7F, 0x00); // Switch bank
	if (power_down) {
		pah8001_write_reg(0x06, 0x0A);
		err = pinctrl_select_state(pctrl, pdn_pin_state1);
	} else {
		pah8001_write_reg(0x06, 0x02);
		err = pinctrl_select_state(pctrl, pdn_pin_state0);
		pah8001_data.start_jiffies = get_jiffies_64();
	}
	if (err)
		PAH_ERR("pinctrl_select_state failed: %d\n", err);

	return err;
}

static void pah8001_led_ctrl(uint8_t touch)
{
	led_ctrl(touch);
}

static int pah8001_init_ppg_reg(void)
{
	uint8_t data = 0;

	pah8001_write_reg(0x7F, 0x00); // Switch to bank 0
	pah8001_write_reg(0x06, 0x82); // Reset the chip
	mdelay(20);
	pah8001_write_reg(0x09, 0x5A); // Set registers to RW
	pah8001_write_reg(0x05, 0xBC); // Enter sleep 2 mode
	//pah8001_write_reg(0x0D, 0xFA); // Unknown (Added from old code)
	pah8001_read_reg(0x17, &data);
	pah8001_write_reg(0x17, data | 0x80);
	pah8001_write_reg(0x27, 0xFF); // Unknown
	pah8001_write_reg(0x28, 0xFA); // Unknown
	pah8001_write_reg(0x29, 0x0A); // Unknown
	pah8001_write_reg(0x2A, 0xC8); // Unknown
	pah8001_write_reg(0x2B, 0xA0); // Unknown
	pah8001_write_reg(0x2C, 0x8C); // Unknown
	pah8001_write_reg(0x2D, 0x64); // Unknown
	pah8001_write_reg(0x42, 0x20); // Unknown
	pah8001_write_reg(0x48, 0x00); // Unknown
	pah8001_write_reg(0x4D, 0x1A); // Set resolution: 0x1A -> wrist application; 0x18 -> Finger tip
	pah8001_write_reg(0x7A, 0xB5); // Unknown
	pah8001_write_reg(0x7F, 0x01); // Switch bank
	pah8001_write_reg(0x07, 0x48); // Enable touch interrupt
	pah8001_write_reg(0x23, 0x3C); // Set touch threshold
	pah8001_write_reg(0x26, 0x0F); // Set thuch threshold
	pah8001_write_reg(0x2E, 0x48); // Unknown
	pah8001_write_reg(0x38, 0xEA); // Set LED step (10mA)
	pah8001_write_reg(0x42, 0xA4); // Unknown
	pah8001_write_reg(0x43, 0x41); // Unknown
	pah8001_write_reg(0x44, 0x41); // Unknown
	pah8001_write_reg(0x45, 0x24); // Unknown
	pah8001_write_reg(0x46, 0xC0); // Unknown
	pah8001_write_reg(0x52, 0x32); // Unknown
	pah8001_write_reg(0x53, 0x28); // Unknown
	pah8001_write_reg(0x56, 0x60); // Unknown
	pah8001_write_reg(0x57, 0x28); // Unknown
	pah8001_write_reg(0x6D, 0x02); // Unknown
	pah8001_write_reg(0x0F, 0xC8); // Unknown
	pah8001_write_reg(0x7F, 0x00); // Switch bank
	pah8001_write_reg(0x5D, 0x81); // Start PPG

	return 0;
}

static void pah8001_read_ppg(void)
{
	static uint8_t frame_count = 0;
	static ppg_mems_data_t ppg_mems_data;
	uint64_t end_jiffies = 0;
	uint8_t touch_flag = 0;
	struct sensor_event event;

	pah8001_write_reg(0x7F, 0x00); // Switch bank
	pah8001_read_reg(0x59, &touch_flag); // Read touch flag
	pah8001_led_ctrl(touch_flag & 0x80);

	if (touch_flag & 0x80) {
		memset(&ppg_mems_data, 0, sizeof(ppg_mems_data));
		pah8001_write_reg(0x7F, 0x01); // Switch bank
		pah8001_read_reg(0x68, &ppg_mems_data.HR_Data[0]); // Get data status
		ppg_mems_data.HR_Data[0] &= 0x0F; // We're only interested in the lower 4 bits
		if (ppg_mems_data.HR_Data[0] == 1) {
			pah8001_i2c_read(0x64, &ppg_mems_data.HR_Data[1], 4); // Read HR_Data registers
			pah8001_i2c_read(0x1A, &ppg_mems_data.HR_Data[5], 3); // Read HR_Data_Algo registers
			ppg_mems_data.HR_Data[8] = frame_count++;
			end_jiffies = get_jiffies_64();
			ppg_mems_data.HR_Data[9] = jiffies_to_msecs(end_jiffies - pah8001_data.start_jiffies);
			pah8001_data.start_jiffies = end_jiffies;
			ppg_mems_data.HR_Data[10] = get_led_current_change_flag();
			ppg_mems_data.HR_Data[11] = touch_flag;
			ppg_mems_data.HR_Data[12] = ppg_mems_data.HR_Data[6];

			event.flush_action = DATA_ACTION;
			event.handle = ID_HEART_RATE;
			memcpy(event.word, ppg_mems_data.HR_Data, sizeof(ppg_mems_data.HR_Data));
			event.word[5] = 0x00; // Report HeartRate Data
			sensor_input_event(pah8001_attr.minor, &event);
		} else {
			// Data is not ready
			msleep(10);
		}
	}
}

static void pah8001_work_func(struct work_struct *work)
{
	pah8001_power_down(0);

	while (pah8001_data.run_ppg)
		pah8001_read_ppg();

	pah8001_power_down(1);
}

static int pah8001_set_hrs_enable(uint8_t enable)
{
	if (enable) {
		pah8001_data.run_ppg = 1;
		schedule_work(&pah8001_data.work);
	} else {
		pah8001_data.run_ppg = 0;
	}

	return 0;
}

static int pah8001_check_chip_id(void)
{
	int err = 0;
	uint8_t id[2];

	err = pah8001_write_reg(0x7F, 0x00); // Switch bank
	if (err) {
		PAH_ERR("Failed to write switch bank command: %d\n", err);
		return -1;
	}

	err = pah8001_i2c_read(0x00, id, 2); // Read ID registers
	if (err) {
		PAH_ERR("pah8001_i2c_read failed:: %d\n", err);
		return -1;
	}

	PAH_DBG("Returned chip ID1: 0x%02X, ID2: 0x%02X\n", id[0], id[1]);

	if (id[0] != 0x30 || (id[1] & 0xF0) != 0xD0) {
		return -1;
	}

	return 0;
}

static ssize_t pah8001_active_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count > 0)
		pah8001_set_hrs_enable(buf[0] != 0 && buf[0] != '0');

	return count;
}

static ssize_t pah8001_active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d", pah8001_data.run_ppg);
}

static DEVICE_ATTR(hrsactive, 0644, pah8001_active_show, pah8001_active_store);

static struct attribute *pah8001_attr_list[] = {
	&dev_attr_hrsactive.attr,
	NULL,
};

static struct attribute_group pah8001_attribute_group = {
	.attrs = pah8001_attr_list,
};

static ssize_t pah8001_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	return sensor_event_read(pah8001_attr.minor, filp, buf, count, pos);
}

static unsigned int pah8001_poll(struct file *filp, poll_table *ptable)
{
	return sensor_event_poll(pah8001_attr.minor, filp, ptable);
}

static struct file_operations pah8001_fops = {
	.owner = THIS_MODULE,
	.open = nonseekable_open,
	.read = pah8001_read,
	.poll = pah8001_poll,
};

static struct sensor_attr_t pah8001_attr = {
	.minor = ID_HEART_RATE,
	.name = DEV_HEART_RATE,
	.fops = &pah8001_fops,
};

static void pah8001_reset(void)
{
	int err = 0;

	err = pinctrl_select_state(pctrl, rst_pin_state0);
	if (err)
		PAH_ERR("pinctrl_select_state failed: %d\n", err);
	mdelay(50);

	err = pinctrl_select_state(pctrl, rst_pin_state1);
	if (err)
		PAH_ERR("pinctrl_select_state failed: %d\n", err);
	mdelay(50);
}

static int pah8001_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	PAH_FUN();

	PAH_DBG("started...\n");

	if (client->addr != PIXART_I2C_ADDRESS) {
		PAH_DBG("Changing I2C address from 0x%02X to 0x%02X\n", client->addr, PIXART_I2C_ADDRESS);
		client->addr = PIXART_I2C_ADDRESS;
	}

	pah8001_data.client = client;

	pctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pctrl)) {
		err = PTR_ERR(pctrl);
		PAH_ERR("devm_pinctrl_get failed: %d\n", err);
		return err;
	}

	ldo_pin_state0 = pinctrl_lookup_state(pctrl, "ldo_pin0");
	if (IS_ERR(ldo_pin_state0)) {
		err = PTR_ERR(ldo_pin_state0);
		PAH_ERR("pinctrl_lookup_state failed for ldo_pin0: %d\n", err);
		return err;
	}

	ldo_pin_state1 = pinctrl_lookup_state(pctrl, "ldo_pin1");
	if (IS_ERR(ldo_pin_state1)) {
		err = PTR_ERR(ldo_pin_state1);
		PAH_ERR("pinctrl_lookup_state failed for ldo_pin1: %d\n", err);
		return err;
	}

	rst_pin_state0 = pinctrl_lookup_state(pctrl, "rst_pin0");
	if (IS_ERR(rst_pin_state0)) {
		err = PTR_ERR(rst_pin_state0);
		PAH_ERR("pinctrl_lookup_state failed for rst_pin0: %d\n", err);
		return err;
	}

	rst_pin_state1 = pinctrl_lookup_state(pctrl, "rst_pin1");
	if (IS_ERR(rst_pin_state1)) {
		err = PTR_ERR(rst_pin_state1);
		PAH_ERR("pinctrl_lookup_state failed for rst_pin1: %d\n", err);
		return err;
	}

	pdn_pin_state0 = pinctrl_lookup_state(pctrl, "pdn_pin0");
	if (IS_ERR(pdn_pin_state0)) {
		err = PTR_ERR(pdn_pin_state0);
		PAH_ERR("pinctrl_lookup_state failed for pdn_pin0: %d\n", err);
		return err;
	}

	pdn_pin_state1 = pinctrl_lookup_state(pctrl, "pdn_pin1");
	if (IS_ERR(pdn_pin_state1)) {
		err = PTR_ERR(pdn_pin_state1);
		PAH_ERR("pinctrl_lookup_state failed for pdn_pin1: %d\n", err);
		return err;
	}

	pah8001_enable_regulator(1);
	mdelay(50);
	pah8001_reset();

	pah8001_power_down(0);

	err = pah8001_check_chip_id();
	if (err) {
		PAH_ERR("pah8001_check_chip_id failed: %d\n", err);
		goto exit_err;
	}

	err = pah8001_init_ppg_reg();
	if (err) {
		PAH_ERR("pah8001_init_ppg_reg failed: %d\n", err);
		goto exit_err;
	}

	pah8001_power_down(1);

	err = sensor_attr_register(&pah8001_attr);
	if (err) {
		PAH_ERR("sensor_attr_register failed: %d\n", err);
		goto exit_err;
	}

	err = sysfs_create_group(&pah8001_attr.this_device->kobj, &pah8001_attribute_group);
	if (err) {
		PAH_ERR("sysfs_create_group failed: %d\n", err);
		goto exit_err;
	}

	INIT_WORK(&pah8001_data.work, pah8001_work_func);
	pah8001_data.run_ppg = 0;

	PAH_DBG("finished!\n");

	return err;

exit_err:
	pah8001_enable_regulator(0);
	pinctrl_select_state(pctrl, rst_pin_state0);
	pinctrl_select_state(pctrl, pdn_pin_state0);

	PAH_DBG("finished with error!\n");

	return err;
}

static int pah8001_i2c_remove(struct i2c_client *client)
{
	sysfs_remove_group(&pah8001_attr.this_device->kobj, &pah8001_attribute_group);
	sensor_attr_deregister(&pah8001_attr);
	return 0;
}

static struct i2c_driver pah8001_i2c_driver = {
	.driver = {
		.name = PIXART_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pah8001_match_table,
	},
	.probe = pah8001_i2c_probe,
	.remove = pah8001_i2c_remove,
};

static int __init pah8001_init(void)
{
	int err = 0;

	PAH_FUN();

	PAH_DBG("started!\n");

	err = i2c_add_driver(&pah8001_i2c_driver);
	if (err)
		PAH_ERR("i2c_add_driver failed: %d\n", err);

	PAH_DBG("finished!\n");

	return err;
}
module_init(pah8001_init);

static void __exit pah8001_exit(void)
{
	i2c_del_driver(&pah8001_i2c_driver);
}
module_exit(pah8001_exit);

MODULE_AUTHOR("Iscle <albertiscle9@gmail.com>");
MODULE_DESCRIPTION("PixArt PAH8001 Driver");
MODULE_LICENSE("GPL");
