#ifndef _BMA4XY_I2C_H_
#define _BMA4XY_I2C_H_

#include <linux/i2c.h>

int bma4xy_i2c_read(struct i2c_client *client, uint8_t reg_addr, uint8_t *data, uint16_t len);
int bma4xy_i2c_write(struct i2c_client *client, uint8_t reg_addr, const uint8_t *data, uint16_t len);

#endif