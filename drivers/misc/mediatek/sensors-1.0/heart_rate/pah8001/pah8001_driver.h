#ifndef _PAH8001_DRIVER_H_
#define _PAH8001_DRIVER_H_

#define PIXART_DEV_NAME "pah8001"
#define PIXART_I2C_ADDRESS 0x33

typedef struct {
	uint8_t HR_Data[13];
	float MEMS_Data[3];
} ppg_mems_data_t;

typedef struct {
	struct i2c_client *client;
	struct work_struct work;
	uint8_t run_ppg;
	uint64_t start_jiffies;
} pah8001_data_t;

#endif