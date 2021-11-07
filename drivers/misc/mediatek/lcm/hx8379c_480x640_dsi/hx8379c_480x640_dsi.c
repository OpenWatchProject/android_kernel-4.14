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
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
    {0xB9, 3, {0xFF, 0x83, 0x79}},
    {0xB1, 16, {0x44, 0x18, 0x18, 0x31, 0x31, 0x50, 0xD0, 0xEE, 0x58, 0x80, 0x38, 0x38, 0xF8, 0x33, 0x32, 0x22}},
    {0xB2, 9, {0x80, 0x28, 0x0C, 0x03, 0x20, 0x50, 0x11, 0x42, 0x1D}},
    {0xB4, 10, {0x02, 0x9D, 0x02, 0x9D, 0x02, 0x9D, 0x22, 0xB5, 0x23, 0xB5}},
    {0xC7, 4, {0x00, 0x00, 0x00, 0xC0}},
    {0xCC, 1, {0x02}},
    {0xD2, 1, {0x33}},
    {0xD3, 37, {0x00, 0x07, 0x00, 0x00, 0x00, 0x08, 0x08, 0x32, 0x10, 0x01, 0x00, 0x01, 0x03, 0x72, 0x03, 0x72, 0x00, 0x08, 0x00, 0x08, 0x33, 0x33, 0x05, 0x05, 0x37, 0x05, 0x05, 0x37, 0x0A, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x01, 0x01, 0x0F}},
    {0xD5, 34, {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x18, 0x18, 0x21, 0x20, 0x18, 0x18, 0x19, 0x19, 0x23, 0x22, 0x38, 0x38, 0x78, 0x78, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00}},
    {0xD6, 32, {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x18, 0x18, 0x22, 0x23, 0x19, 0x19, 0x18, 0x18, 0x20, 0x21, 0x38, 0x38, 0x38, 0x38, 0x18, 0x18, 0x18, 0x18}},
    {0xE0, 42, {0x00, 0x04, 0x09, 0x2C, 0x34, 0x3F, 0x15, 0x35, 0x06, 0x0A, 0x0C, 0x17, 0x0D, 0x11, 0x13, 0x11, 0x13, 0x07, 0x12, 0x13, 0x17, 0x00, 0x04, 0x08, 0x2C, 0x34, 0x3F, 0x14, 0x35, 0x07, 0x0B, 0x0D, 0x18, 0x0E, 0x11, 0x14, 0x11, 0x13, 0x07, 0x12, 0x13, 0x17}},
    {0xB6, 2, {0xB0, 0xB0}},
    {0x3A, 1, {0x77}},
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 255, {}},
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 255, {}},
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
    params->dsi.mode = SYNC_PULSE_VDO_MODE;
    params->dsi.LANE_NUM = LCM_ONE_LANE;
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
    params->dsi.packet_size = 256;
    params->dsi.intermediat_buffer_num = 0;
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 8;
    params->dsi.vertical_frontporch = 5;
    params->dsi.vertical_active_line = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active = 60;
    params->dsi.horizontal_backporch = 60;
    params->dsi.horizontal_frontporch = 100;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.PLL_CLOCK = 342;
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
}

struct LCM_DRIVER hx8379c_480x640_dsi_lcm_drv = {
    .name = "hx8379c_480x640_dsi",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .compare_id = lcm_compare_id,
};
