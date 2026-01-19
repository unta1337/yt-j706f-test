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

#ifndef _ktz8864a_SW_H_
#define _ktz8864a_SW_H_

#define GPIO_OUT_ONE               1
#define GPIO_OUT_ZERO              0

static unsigned int GPIO_LCD_BL_EN;
static unsigned int GPIO_LCD_BIAS_ENP;
static unsigned int GPIO_LCD_BIAS_ENN;
static struct regulator *lcm_vtp;
extern unsigned int islcmconnected;

#ifndef BUILD_LK
struct ktz8864a_setting_table {
	unsigned char cmd;
	unsigned char data;
};

extern int ktz8864a_read_byte(unsigned char cmd, unsigned char *returnData);
extern int ktz8864a_write_byte(unsigned char cmd, unsigned char writeData);

extern int lcm_get_vtp_supply(struct device *dev);
extern int lcm_vtp_supply_enable(void);
extern int lcm_vtp_supply_disable(void);
extern void lcm_request_gpio_control(struct device *dev);

#endif

#endif
