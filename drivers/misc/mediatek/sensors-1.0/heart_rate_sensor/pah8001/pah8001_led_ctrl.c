#include "pah8001_led_ctrl.h"

extern uint8_t pah8001_write_reg(uint8_t addr, uint8_t data);
extern uint8_t pah8001_read_reg(uint8_t addr, uint8_t *data);

static uint8_t led_step = DEFAULT_LED_STEP;
static uint8_t state = 0, state_count = 0;
static uint8_t led_current_change_flag = 0;
static uint8_t sleepflag = 1;
static uint8_t light_count = 0;
static uint8_t light_flag = 0;

uint8_t get_led_current_change_flag(void)
{
	return led_current_change_flag;
}

void led_ctrl(uint8_t touch)
{
	uint8_t data;

	pah8001_write_reg(0x7F, 0x01);
	pah8001_read_reg(0x1A, &data);

	if ((touch & 0x80) != 0x80 && data > 100 && led_step < 31) {
		if (light_flag == 1) {
			light_count++;
			if (light_count > 10) {
				led_step = 31;
				pah8001_write_reg(0x38, (led_step | 0xE0));
			}
		}
		light_flag = 1;
	} else if ((touch & 0x80) == 0x80 || led_step < 31) {
		uint8_t data;
		uint16_t exposure_time_l, exposure_time_h, exposure_time;

		pah8001_write_reg(0x7F, 0x00);
		//pah8001_write_reg(0x05, 0x98); // 0x98 is an invalid value according to the datasheet

		pah8001_read_reg(0x33, &data);
		exposure_time_h = data & 0x03;
		pah8001_read_reg(0x32, &data);
		exposure_time_l = data;
		exposure_time = (exposure_time_h << 8) + exposure_time_l;

		pah8001_write_reg(0x7F, 0x01);
		if (sleepflag == 1) {
			pah8001_write_reg(0x38, (0xE0 | DEFAULT_LED_STEP));
			led_step = DEFAULT_LED_STEP;
			sleepflag = 0;
		}

		if (state_count <= STATE_COUNT_TH) {
			state_count++;
			led_current_change_flag = 0;
		} else {
			state_count = 0;

			if (state == 0) {
				if ((exposure_time >= LED_CTRL_EXPO_TIME_HI_BOUND) ||
				    (exposure_time <= LED_CTRL_EXPO_TIME_LOW_BOUND)) {
					pah8001_read_reg(0x38, &data);
					led_step = data & 0x1F;

					if ((exposure_time >= LED_CTRL_EXPO_TIME_HI_BOUND)
					    && (led_step < LED_CURRENT_HI)) {
						state = 1;
						led_step = led_step + LED_INC_DEC_STEP;
						if (led_step > LED_CURRENT_HI)
							led_step = LED_CURRENT_HI;
						pah8001_write_reg(0x38, (led_step | 0xE0));
						led_current_change_flag = 1;
					} else if ((exposure_time <= LED_CTRL_EXPO_TIME_LOW_BOUND)
					           && (led_step > LED_CURRENT_LOW)) {
						state = 2;
						if (led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP))
							led_step = LED_CURRENT_LOW;
						else
							led_step = led_step - LED_INC_DEC_STEP;
						pah8001_write_reg(0x38, (led_step | 0xE0));
						led_current_change_flag = 1;
					} else {
						state = 0;
						led_current_change_flag = 0;
					}
				} else {
					led_current_change_flag = 0;
				}
			} else if (state == 1) {
				if (exposure_time > LED_CTRL_EXPO_TIME_HI) {
					state = 1;
					led_step = led_step + LED_INC_DEC_STEP;

					if (led_step >= LED_CURRENT_HI) {
						state = 0;
						led_step = LED_CURRENT_HI;
					}
					pah8001_write_reg(0x38, (led_step | 0xE0));
					led_current_change_flag = 1;
				} else {
					state = 0;
					led_current_change_flag = 0;
				}
			} else {
				if (exposure_time < LED_CTRL_EXPO_TIME_LOW) {
					state = 2;
					if (led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP)) {
						state = 0;
						led_step = LED_CURRENT_LOW;
					} else
						led_step = led_step - LED_INC_DEC_STEP;
					pah8001_write_reg(0x38, (led_step | 0xE0));
					led_current_change_flag = 1;
				} else {
					state = 0;
					led_current_change_flag = 0;
				}
			}
		}
	} else {
		pah8001_write_reg(0x7F, 0x00);
		pah8001_write_reg(0x05, 0xB8);
		pah8001_write_reg(0x7F, 0x01);
		//led_step = DEFAULT_LED_STEP;
		//pah8001_write_reg(0x38, (0xE0 | DEFAULT_LED_STEP));	//for Asian person only
		pah8001_write_reg(0x38, 0xFF);
		sleepflag = 1;

		led_current_change_flag = 0;
	}
}
