#include "bma4xy_i2c.h"
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define BMA4XY_MAX_RETRY_I2C_XFER	3
#define BMA4XY_I2C_RETRY_DELAY		1000

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

int bma4xy_i2c_read(struct i2c_client *client, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;

		usleep_range(BMA4XY_I2C_RETRY_DELAY, BMA4XY_I2C_RETRY_DELAY + 1000);
	}

	if (retry == BMA4XY_MAX_RETRY_I2C_XFER) {
		GSE_ERR("I2C xfer error\n");
		return -EIO;
	}

	return 0;

}

int bma4xy_i2c_write(struct i2c_client *client, uint8_t reg_addr, const uint8_t *data, uint16_t len)
{
	uint8_t retry;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = NULL,
	};

	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (msg.buf == NULL) {
		GSE_ERR("Allocate mem failed\n");
		return -ENOMEM;
	}

	msg.buf[0] = reg_addr;
	memcpy(&msg.buf[1], data, len);

	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, &msg, 1) > 0)
			break;

		usleep_range(BMA4XY_I2C_RETRY_DELAY, BMA4XY_I2C_RETRY_DELAY + 1000);
	}

	kfree(msg.buf);

	if (retry == BMA4XY_MAX_RETRY_I2C_XFER) {
		GSE_ERR("I2C xfer error\n");
		return -EIO;
	}

	return 0;
}
