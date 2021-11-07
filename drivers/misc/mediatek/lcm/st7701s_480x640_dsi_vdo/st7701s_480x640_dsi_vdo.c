/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "lcm_drv.h"
#include <linux/string.h>

#define FRAME_HEIGHT 640
#define FRAME_WIDTH 480

#define REGFLAG_DELAY 0x0A
#define REGFLAG_END_OF_TABLE 0x0B

static struct LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v) lcm_util.set_reset_pin(v)

#define UDELAY(n) lcm_util.udelay(n)
#define MDELAY(n) lcm_util.mdelay(n)

#define dsi_set_cmdq_V2(cmd, count, para_list, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, para_list, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define dsi_write_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define dsi_write_regs(addr, para, nums) lcm_util.dsi_write_regs(addr, para, nums)
#define dsi_dcs_read_lcm_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[32];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x13}},
	{0xEF, 1, {0x08}},
	{0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x00}},
	{0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC0, 2, {0x63, 0x00}},
	{0xC1, 2, {0x09, 0x02}},
	{0xC2, 2, {0x01, 0x02}},
	{0xB0, 16, {0xC0, 0x07, 0x90, 0x0E, 0x10, 0x04, 0x00, 0x0B, 0x06, 0x1A, 0x03, 0x51, 0x0E, 0x65, 0x2D, 0x1F}},
	{0xB1, 16, {0xC0, 0x94, 0xD9, 0x0E, 0x11, 0x08, 0x03, 0x05, 0x09, 0x20, 0x06, 0x54, 0x12, 0xE7, 0xED, 0x0F}},
	{0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x11}},
	{0xB0, 1, {0x58}},
	{0xB1, 1, {0x7B}},
	{0xB2, 1, {0x81}},
	{0xB3, 1, {0x80}},
	{0xB5, 1, {0x42}},
	{0xB7, 1, {0x8A}},
	{0xB8, 1, {0x21}},
	{0xC1, 1, {0x78}},
	{0xC2, 1, {0x78}},
	{0xD0, 1, {0x88}},
	{REGFLAG_DELAY, 100, {}},
	{0xE0, 3, {0x00, 0x00, 0x02}},
	{0xE1, 11, {0x01, 0xA0, 0x03, 0xA0, 0x02, 0xA0, 0x04, 0xA0, 0x00, 0x44, 0x44}},
	{0xE2, 12, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0xE3, 4, {0x00, 0x00, 0x33, 0x33}},
	{0xE4, 3, {0x44, 0x44}},
	{0xE5, 16, {0x01, 0x26, 0xA0, 0xA0, 0x03, 0x28, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0}},
	{0xE6, 3, {0x00, 0x00, 0x33, 0x33}},
	{0xE7, 2, {0x44, 0x44}},
	{0xE8, 16, {0x02, 0x26, 0xA0, 0xA0, 0x04, 0x28, 0xA0, 0xA0, 0x06, 0x2A, 0xA0, 0xA0, 0x08, 0x2C, 0xA0, 0xA0}},
	{0xEB, 7, {0x00, 0x00, 0xE4, 0xE4, 0x44, 0x00, 0x40}},
	{0xED, 16, {0xFF, 0xF7, 0x65, 0x4F, 0x0B, 0xA1, 0xCF, 0xFF, 0xFF, 0xFC, 0x1A, 0xB0, 0xF4, 0x56, 0x7F, 0xFF}},
	{0xEF, 6, {0x08, 0x08, 0x08, 0x45, 0x3F, 0x54}},
	{0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x13}},
	{0xE6, 2, {0x14, 0x7C}},
	{0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x00}},
	{0x21, 1, {0x00}},
	{0x11, 0, {}},
	{REGFLAG_DELAY, 5, {}},
	{0x29, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {}},
	{REGFLAG_DELAY, 5, {}},

	// Display On
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display Off
	{0x28, 0, {}},

	// Sleep In
	{0x10, 0, {}},
	{REGFLAG_DELAY, 5, {}},
	{REGFLAG_END_OF_TABLE, 0, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for (i = 0; i < count; i++) {
		struct LCM_setting_table *t = &table[i];

		switch (t->cmd) {
		case REGFLAG_DELAY:
			MDELAY(t->count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(t->cmd, t->count, t->para_list, force_update);
			break;
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.LANE_NUM = LCM_ONE_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.packet_size = 256;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = 640 * 3;
	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 20;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 30;
	params->dsi.horizontal_frontporch = 30;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.PLL_CLOCK = 350;
	params->dsi.ssc_disable = 1;
}

static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(120);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
	return 1;

#if 0
	unsigned int array[4];
	unsigned short device_id;
	unsigned char buffer[2];

	SET_RESET_PIN(0);
	MDELAY(120);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);
	dsi_dcs_read_lcm_reg_v2(0xA1, buffer, 2);

	device_id = buffer[0] << 8 | buffer[1];

	return (0x8802 == device_id) ? 1 : 0;
#endif
}

struct LCM_DRIVER st7701s_480x640_dsi_vdo_lcm_drv = {
	.name = "st7701s_480x640_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
};
