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
#include "../OCP2138.h"
#include "../SGM37604A.h"
#include "../../video/mt6785/dispsys/ddp_hal.h"
#include "../../video/mt6785/videox/mtkfb_console.h"

#ifdef BUILD_LK
#  include <platform/upmu_common.h>
#  include <platform/mt_gpio.h>
#  include <platform/mt_i2c.h>
#  include <platform/mt_pmic.h>
#  include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/atomic.h>
#include <linux/of.h>
#endif

#define LOG_TAG "LCM"
#define FRAME_WIDTH				(1200)
#define FRAME_HEIGHT			(2000)
#define GPIO_OUT_ONE				1
#define GPIO_OUT_ZERO				0

/* physical size in um */
#define LCM_PHYSICAL_WIDTH		(143100)
#define LCM_PHYSICAL_HEIGHT		(238500)
#define LCM_DENSITY				(240)

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

#define LCM_ID_HX83102P			0x83

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef BUILD_LK
#  define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#  define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#  define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#  define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int GPIO_LCD_BL_EN;
static unsigned int GPIO_LCD_BIAS_ENP;
static unsigned int GPIO_LCD_BIAS_ENN;
static struct regulator *lcm_vtp;
extern unsigned int islcmconnected;
extern int p522_gesture_flag;
extern int do_esd_recover;
extern int esd_backlight_level;
int bl_flag1 = 0;

static int hx83102p_vtp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	LCM_LOGI("[hx83102p] %s() enter\n", __func__);
	if (lcm_vtp == NULL)
		return 0;

	ret = regulator_is_enabled(lcm_vtp);
	if (ret > 0){
		LCM_LOGI("[hx83102p] %s vtp is shared by others!!!\n", __func__);
		return ret;
	}

	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vtp, 1800000, 1800000);
	if (ret != 0) {
		LCM_LOGI("[hx83102p] %s lcm failed to set lcm_vtp voltage: %d\n", __func__, ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vtp);
	if (volt != 1800000)
		LCM_LOGI("[hx83102p] %s check regulator voltage=1800000 fail! (voltage: %d)\n", __func__, volt);

	ret = regulator_enable(lcm_vtp);
	if (ret != 0) {
		LCM_LOGI("[hx83102p] %s Failed to enable lcm_vtp: %d\n", __func__, ret);
		return ret;
	}

	LCM_LOGI("[hx83102p] %s() end\n", __func__);
	return ret;
}


static int hx83102p_lcm_vtp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	LCM_LOGI("[hx83102p] %s() enter\n", __func__);
	if (lcm_vtp == NULL)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vtp);
	LCM_LOGI("[hx83102p] %s lcm query regulator enable status[%d]\n", __func__, isenable);

	if (isenable > 0) {
		ret = regulator_disable(lcm_vtp);
		if (ret != 0) {
			LCM_LOGI("[hx83102p] %s lcm failed to disable lcm_vtp: %d\n", __func__, ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vtp);
		if (isenable > 0) {
			LCM_LOGI("[hx83102p] %s lcm regulator disable failed, vtp is shared by others\n", __func__);
		}
	}

	LCM_LOGI("[hx83102p] %s() end\n", __func__);
	return ret;
}


#ifndef BUILD_LK
static int hx83102p_lcm_driver_probe(struct device *dev, void const *data)
{
	int ret;
	struct regulator *lcm_vtp_ldo;

	LCM_LOGI("[hx83102p] %s() enter\n", __func__);

	/* get VTP LDO supply */
	lcm_vtp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vtp_ldo)) {
		ret = PTR_ERR(lcm_vtp_ldo);
		LCM_LOGI("[hx83102p] %s failed to get reg-lcm LDO, %d\n",__func__, ret);
		return ret;
	}

	ret = regulator_enable(lcm_vtp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vtp_ldo);
	LCM_LOGI("[hx83102p] %s get VTP-LDO2 voltage = %d\n",__func__, ret);
	lcm_vtp = lcm_vtp_ldo;

	/* config gpios */
	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en", 0);
	gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	GPIO_LCD_BIAS_ENP = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enp", 0);
	gpio_request(GPIO_LCD_BIAS_ENP, "GPIO_LCD_BIAS_ENP");
	GPIO_LCD_BIAS_ENN = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enn", 0);
	gpio_request(GPIO_LCD_BIAS_ENN, "GPIO_LCD_BIAS_ENN");

	LCM_LOGI("[hx83102p] %s() successful\n", __func__);
	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "hx,hx83102p",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return hx83102p_lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		.name = "hx83102p_fhdp_dsi_vdo_boe_p522",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init hx83102p_lcm_drv_init(void)
{
	LCM_LOGI("[hx83102p] %s enter, islcmconnected:%d\n", __func__, islcmconnected);

	if (0 == islcmconnected) {
		LCM_LOGI("[hx83102p] %s lcm is not connect return", __func__);
	}

	if (platform_driver_register(&lcm_driver)) {
		LCM_LOGI("[hx83102p] %s failed to register disp driver\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static void __exit hx83102p_lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	LCM_LOGI("[hx83102p] %s Unregister lcm driver done\n", __func__);
}

late_initcall(hx83102p_lcm_drv_init);
module_exit(hx83102p_lcm_drv_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
#endif

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
#endif
}

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static void lcm_suspend_deepsleep_registers(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500;
	dsi_set_cmdq(data_array,1, 1);
	MDELAY(30);
	data_array[0]=0x00100500;
	dsi_set_cmdq(data_array,1, 1);
	MDELAY(120);
	if (p522_gesture_flag != 1) {
		data_array[0]=0x00043902;
		data_array[1]=0x2E1083B9;
		dsi_set_cmdq(data_array,2, 1);

		data_array[0]=0x00023902;
		data_array[1]=0x000021B1;
		dsi_set_cmdq(data_array,2, 1);
	}
}

static void lcm_initial_registers(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00043902;
	data_array[1]=0x2E1083B9;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x0000CDE9;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000001BB;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000000E9;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00053902;
	data_array[1]=0xFF0C67D1;
	data_array[2]=0x00000005;
	dsi_set_cmdq(data_array,3, 1);

	data_array[0]=0x00123902;
	data_array[1]=0xAFFA10B1;
	data_array[2]=0xB22B2BAF;
	data_array[3]=0x36364D57;
	data_array[4]=0x21223636;
	data_array[5]=0x00000015;
	dsi_set_cmdq(data_array,6, 1);

	data_array[0]=0x00113902;
	data_array[1]=0x47B000B2;
	data_array[2]=0x502C00D0;
	data_array[3]=0x0000002C;
	data_array[4]=0xD7201500;
	data_array[5]=0x00000000;
	dsi_set_cmdq(data_array,6, 1);

	data_array[0]=0x00113902;
	data_array[1]=0x016070B4;
	data_array[2]=0x00678001;
	data_array[3]=0x019B0100;
	data_array[4]=0x00FF0058;
	data_array[5]=0x000000FF;
	dsi_set_cmdq(data_array,6, 1);

	data_array[0]=0x00043902;
	data_array[1]=0x8085FCBF;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x002B2BD2;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x002C3902;
	data_array[1]=0x000000D3;
	data_array[2]=0x00047800;
	data_array[3]=0x00270004;
	data_array[4]=0x2D2D4F64;
	data_array[5]=0x10320000;
	data_array[6]=0x32270027;
	data_array[7]=0x23002310;
	data_array[8]=0x08031832;
	data_array[9]=0x20000003;
	data_array[10]=0x21550130;
	data_array[11]=0x0F55012E;
	dsi_set_cmdq(data_array,12, 1);

	data_array[0]=0x002F3902;
	data_array[1]=0x110600E0;
	data_array[2]=0x533B2119;
	data_array[3]=0x745B605A;
	data_array[4]=0x878A7D78;
	data_array[5]=0xAAAA988F;
	data_array[6]=0x70665B54;
	data_array[7]=0x19110600;
	data_array[8]=0x5A533B21;
	data_array[9]=0x78745B60;
	data_array[10]=0x8F878A7D;
	data_array[11]=0x54AAAA98;
	data_array[12]=0x0070665B;
	dsi_set_cmdq(data_array,13, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000001BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00053902;
	data_array[1]=0x019B01B1;
	data_array[2]=0x00000031;
	dsi_set_cmdq(data_array,3, 1);

	data_array[0]=0x000B3902;
	data_array[1]=0x1236F4CB;
	data_array[2]=0x6C28C016;
	data_array[3]=0x00043F85;
	dsi_set_cmdq(data_array,4, 1);

	data_array[0]=0x000C3902;
	data_array[1]=0xBC0001D3;
	data_array[2]=0x10110000;
	data_array[3]=0x01000E00;
	dsi_set_cmdq(data_array,4, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000002BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00073902;
	data_array[1]=0x33004EB4;
	data_array[2]=0x00883311;
	dsi_set_cmdq(data_array,3, 1);

	data_array[0]=0x00043902;
	data_array[1]=0x0200F2BF;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000000BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x000F3902;
	data_array[1]=0x222323C0;
	data_array[2]=0x0017A211;
	data_array[3]=0x08000080;
	data_array[4]=0x00636300;
	dsi_set_cmdq(data_array,5, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x0000F9C6;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000030C7;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00093902;
	data_array[1]=0x040400C8;
	data_array[2]=0x43850000;
	data_array[3]=0x000000FF;
	dsi_set_cmdq(data_array,4, 1);

	data_array[0]=0x00043902;
	data_array[1]=0x050407D0;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x002D3902;
	data_array[1]=0x181818D5;
	data_array[2]=0x1A1A1A18;
	data_array[3]=0x1B1B1B1A;
	data_array[4]=0x2424241B;
	data_array[5]=0x07060724;
	data_array[6]=0x05040506;
	data_array[7]=0x03020304;
	data_array[8]=0x01000102;
	data_array[9]=0x21202100;
	data_array[10]=0x18181820;
	data_array[11]=0x18181818;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array,13, 1);

	data_array[0]=0x00183902;
	data_array[1]=0x021312E7;
	data_array[2]=0x0E494902;
	data_array[3]=0x1D1A0F0E;
	data_array[4]=0x01742874;
	data_array[5]=0x00000007;
	data_array[6]=0x68001700;
	dsi_set_cmdq(data_array,7, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000001BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00083902;
	data_array[1]=0x013802E7;
	data_array[2]=0x0EDA0D93;
	dsi_set_cmdq(data_array,3, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000002BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x001D3902;
	data_array[1]=0xFF01FFE7;
	data_array[2]=0x22000001;
	data_array[3]=0x00000000;
	data_array[4]=0x00000000;
	data_array[5]=0x00000000;
	data_array[6]=0x00000000;
	data_array[7]=0x02008100;
	data_array[8]=0x00000040;
	dsi_set_cmdq(data_array,9, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000000BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00093902;
	data_array[1]=0xA80370BA;
	data_array[2]=0xC080F283;
	data_array[3]=0x0000000D;
	dsi_set_cmdq(data_array,4, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000002BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x000D3902;
	data_array[1]=0xFFFFFFD8;
	data_array[2]=0xFF00F0FF;
	data_array[3]=0xF0FFFFFF;
	data_array[4]=0x00000000;
	dsi_set_cmdq(data_array,5, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000003BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00193902;
	data_array[1]=0xAAAAAAD8;
	data_array[2]=0xAA00A0AA;
	data_array[3]=0xA0AAAAAA;
	data_array[4]=0x55555500;
	data_array[5]=0x55005055;
	data_array[6]=0x50555555;
	data_array[7]=0x00000000;
	dsi_set_cmdq(data_array,8, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000000BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x000601E1;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000002CC;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000003BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000080B2;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000000BD;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x00000051;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x00002453;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00053902;
	data_array[1]=0xF00D04C9;
	data_array[2]=0x00000000;
	dsi_set_cmdq(data_array,3, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x00000035;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x0020005E;
	dsi_set_cmdq(data_array,2, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(60);

	data_array[0]=0x00113902;
	data_array[1]=0x47B000B2;
	data_array[2]=0x502C00D0;
	data_array[3]=0x0000002C;
	data_array[4]=0xD7201500;
	data_array[5]=0x00000000;
	dsi_set_cmdq(data_array,6, 1);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(25);
}

static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x00, 0x00} },
	{0x53, 1, {0x2C} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table bl_level1[] = {
	{0x51, 2, {0x00, 0x00} },
	{0x53, 1, {0x24} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
				unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;
		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
					 table[i].para_list, force_update);
			break;
		}
	}
}

static void hx83102p_lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void hx83102p_lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 38;
	params->dsi.vertical_frontporch = 80;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 28;
	params->dsi.horizontal_frontporch = 36;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 522;
	params->dsi.PLL_CK_CMD = 480;
	params->dsi.CLK_HS_POST = 50;
	params->dsi.cont_clock = 0;
	params->dsi.clk_lp_per_line_enable = 0;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0;
	params->dsi.lcm_esd_check_table[0].count = 0;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0;
}


/* turn on gate ic & control voltage to 5.5V */
static void hx83102p_lcm_init_power(void)
{
	LCM_LOGI("[hx83102p] %s enter, islcmconnected %d\n", __func__, islcmconnected);
	if (1 == islcmconnected) {
		hx83102p_vtp_supply_enable();
		MDELAY(3);

		ocp2138_write_byte(0x00, 0x12);		// set avdd 5.8v
		ocp2138_write_byte(0x01, 0x12);		// set avee 5.8v
		sgm37604a_write_byte(0x1A, 0x00);	// set bl 21.5mA
		sgm37604a_write_byte(0x19, 0xD7);	// set bl 21.5mA

		// enable avdd
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ONE);
		MDELAY(3);
		// enable  avee
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ONE);
		MDELAY(3);
	}
	LCM_LOGI("[hx83102p] %s exit\n", __func__);
}

static void hx83102p_lcm_suspend_power(void)
{
	LCM_LOGI("[hx83102p] %s p522_gesture_flag=%d, islcmconnected=%d enter\n", __func__,
		    p522_gesture_flag, islcmconnected);
	if (do_esd_recover) {
		LCM_LOGI("[nt36523w] %s do_esd_recover\n", __func__);
		SET_RESET_PIN(0);
		MDELAY(2);
		//disable avee
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ZERO);
		MDELAY(3);
		// disable avdd
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ZERO);
		MDELAY(3);
		// disable vddi
		hx83102p_lcm_vtp_supply_disable();
	}
	if ((p522_gesture_flag != 1) && (1 == islcmconnected)) {
		LCM_LOGI("[nt36523w] %s normal suspend\n", __func__);
		//disable avee
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ZERO);
		MDELAY(3);
		// disable avdd
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ZERO);
		MDELAY(3);
	}
	LCM_LOGI("[hx83102p] %s exit\n", __func__);
}

/* turn on gate ic & control voltage to 5.5V */
static void hx83102p_lcm_resume_power(void)
{
	LCM_LOGI("[hx83102p] %s enter,islcmconnected:%d\n", __func__,islcmconnected);
	if ((p522_gesture_flag != 1) && (1 == islcmconnected)) {
		// hx83102p_vtp_supply_enable();
		// MDELAY(3);
		// enable avdd & avee
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ONE);
		MDELAY(3);
		// enable avdd & avee
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ONE);
		MDELAY(3);
	}
	LCM_LOGI("[hx83102p] %s exit\n", __func__);
}

static void hx83102p_lcm_init(void)
{
	LCM_LOGI("[hx83102p] %s enter, islcmconnected:%d\n", __func__, islcmconnected);
	if (1 == islcmconnected) {
		SET_RESET_PIN(1);
		MDELAY(2);
		SET_RESET_PIN(0);
		MDELAY(18);
		SET_RESET_PIN(1);
		MDELAY(8);

		lcm_initial_registers();
		MDELAY(50);
		// lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE); // Enable BL
		sgm37604a_write_byte(0x1A, 0x00);	// set bl 21.5mA
		sgm37604a_write_byte(0x19, 0xD7);	// set bl 21.5mA
		sgm37604a_write_byte(0x10, 0x1F);	// Enable BL
	}
	LCM_LOGI("[hx83102p] %s exit\n", __func__);
}

static void hx83102p_lcm_suspend(void)
{
	LCM_LOGI("[hx83102p] %s enter islcmconnected:%d\n", __func__, islcmconnected);
	if (1 == islcmconnected) {
		// lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO); // Disable BL
		sgm37604a_write_byte(0x10, 0x00);	// Disable BL
		sgm37604a_write_byte(0x1A, 0x00);
		sgm37604a_write_byte(0x19, 0x00);
		MDELAY(1);
		lcm_suspend_deepsleep_registers();
		MDELAY(1);
	}
	LCM_LOGI("[hx83102p] %s exit\n", __func__);
}

static void hx83102p_lcm_resume(void)
{
	hx83102p_lcm_init();
}

static unsigned int hx83102p_lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int id[3] = {0x83, 0x11, 0x2B};
	unsigned int data_array[3];
	unsigned char read_buf[3];

	data_array[0] = 0x00033700; /* set max return size = 3 */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, read_buf, 3); /* read lcm id */

	LCM_LOGI("ATA read = 0x%x, 0x%x, 0x%x\n", read_buf[0], read_buf[1], read_buf[2]);

	if ((read_buf[0] == id[0]) &&(read_buf[1] == id[1]) && (read_buf[2] == id[2]))
		ret = 1;
	else
		ret = 0;

	return ret;
#else
	return 0;
#endif
}

static void hx83102p_lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	unsigned int dbt = 0;

	if (level == 1 || level == 2)
		level = 3;

	LCM_LOGI("[hx83102p] %s, level = %d, bl_flag1 = %d\n", __func__, level, bl_flag1);

	dbt = ((level & 0xFF) << 4) | (level >> 4);

	if ((bl_flag1 == 0) && (level < 15)) {
		bl_flag1 = 1;
		bl_level1[0].para_list[0] = ((dbt >> 8) & 0xF);
		bl_level1[0].para_list[1] = (dbt & 0xFF);
		push_table(handle, bl_level1, ARRAY_SIZE(bl_level1), 1);
	} else {
		bl_level[0].para_list[0] = ((dbt >> 8) & 0xF);
		bl_level[0].para_list[1] = (dbt & 0xFF);
		push_table(handle, bl_level, ARRAY_SIZE(bl_level), 1);
	}

	if (level == 0)
		bl_flag1 = 0;
	else
		bl_flag1 = 1;
}

static unsigned int hx83102p_lcm_esd_recover(void) {
	LCM_LOGI("[hx83102p] %s enter,\n", __func__);
	hx83102p_lcm_init_power();
	hx83102p_lcm_init();
	hx83102p_lcm_setbacklight_cmdq(NULL, esd_backlight_level);
	return TRUE;
}

static void hx83102p_lcm_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

#ifdef LCM_SET_DISPLAY_ON_DELAY
	lcm_set_display_on();
#endif

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int hx83102p_lcm_compare_id(void)
{
	return 0;
}

struct LCM_DRIVER hx83102p_fhdp_dsi_vdo_boe_p522_lcm_drv = {
	.name = "hx83102p_fhdp_dsi_vdo_boe_p522_drv",
	.set_util_funcs = hx83102p_lcm_set_util_funcs,
	.get_params = hx83102p_lcm_get_params,
	.init = hx83102p_lcm_init,
	.suspend = hx83102p_lcm_suspend,
	.resume = hx83102p_lcm_resume,
	.init_power = hx83102p_lcm_init_power,
	.resume_power = hx83102p_lcm_resume_power,
	.suspend_power = hx83102p_lcm_suspend_power,
	.compare_id = hx83102p_lcm_compare_id,
	.set_backlight_cmdq = hx83102p_lcm_setbacklight_cmdq,
	.esd_recover = hx83102p_lcm_esd_recover,
	.ata_check = hx83102p_lcm_ata_check,
	.update = hx83102p_lcm_update,
};

