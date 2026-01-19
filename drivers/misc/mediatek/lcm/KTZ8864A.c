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

#ifndef BUILD_LK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "KTZ8864A.h"

/* I2C Slave Setting */
#define ktz8864a_SLAVE_ADDR_WRITE	0x11
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/LCM]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/LCM]"fmt, ##args)

static struct i2c_client *new_client;
static const struct i2c_device_id ktz8864a_i2c_id[] = { {"ktz8864a", 0}, {} };

static int ktz8864a_driver_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int ktz8864a_driver_remove(struct i2c_client *client);

#ifdef CONFIG_OF
static const struct of_device_id ktz8864a_id[] = {
	{.compatible = "ktz8864a"},
	{},
};

MODULE_DEVICE_TABLE(of, ktz8864a_id);
#endif

static struct i2c_driver ktz8864a_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ktz8864a",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ktz8864a_id),
#endif
	},
	.probe = ktz8864a_driver_probe,
	.remove = ktz8864a_driver_remove,
	.id_table = ktz8864a_i2c_id,
};

static DEFINE_MUTEX(ktz8864a_i2c_access);

/* I2C Function For Read/Write */
int ktz8864a_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[2] = { 0x00, 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&ktz8864a_i2c_access);

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], 1);
	ret = i2c_master_recv(new_client, &cmd_buf[1], 1);
	if (ret < 0) {
		mutex_unlock(&ktz8864a_i2c_access);
		return 0;
	}

	readData = cmd_buf[1];
	*returnData = readData;

	mutex_unlock(&ktz8864a_i2c_access);

	return 1;
}

int ktz8864a_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	LCM_LOGI("[KTZ8864A] cmd: %02X, data: %02X,%s\n", cmd, writeData, __func__);

	mutex_lock(&ktz8864a_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		mutex_unlock(&ktz8864a_i2c_access);
		LCM_LOGI("[ktz8864a] I2C write fail!!!\n");

		return 0;
	}

	mutex_unlock(&ktz8864a_i2c_access);

	return 1;
}

static int ktz8864a_driver_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;

	LCM_LOGI("[KTZ8864A] name=%s addr=0x%X\n",
		client->name, client->addr);
	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}

	memset(new_client, 0, sizeof(struct i2c_client));
	new_client = client;

	return 0;

 exit:
	return err;

}

static int ktz8864a_driver_remove(struct i2c_client *client)
{
	LCM_LOGI("[KTZ8864A] %s\n", __func__);

	new_client = NULL;
	i2c_unregister_device(client);

	return 0;
}

#define ktz8864a_BUSNUM 5

static int __init ktz8864a_init(void)
{
	LCM_LOGI("[KTZ8864A] %s\n", __func__);

	if (i2c_add_driver(&ktz8864a_driver) != 0)
		LCM_LOGI("[KTZ8864A] failed to register ktz8864a i2c driver.\n");
	else
		LCM_LOGI("[KTZ8864A] Success to register ktz8864a i2c driver.\n");

	return 0;
}

static void __exit ktz8864a_exit(void)
{
	i2c_del_driver(&ktz8864a_driver);
}

module_init(ktz8864a_init);
module_exit(ktz8864a_exit);

/* get VTP LDO supply */
int lcm_get_vtp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vtp_ldo;

	LCM_LOGI("%s() enter\n", __func__);

	lcm_vtp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vtp_ldo)) {
		ret = PTR_ERR(lcm_vtp_ldo);
		LCM_LOGI("failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	LCM_LOGI("[Kernel/LCM]: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vtp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vtp_ldo);
	LCM_LOGI("%s get VTP-LDO2 voltage = %d\n",__func__, ret);

	lcm_vtp = lcm_vtp_ldo;

	return ret;
}

int lcm_vtp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	LCM_LOGI("%s() enter\n", __func__);

	if (lcm_vtp == NULL)
		return 0;

	ret = regulator_is_enabled(lcm_vtp);
	if (ret) {
		LCM_LOGI("%s vtp is shared by others!!!\n", __func__);
		return 0;
	}

	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vtp, 1800000, 1800000);
	if (ret != 0) {
		LCM_LOGI("failed to set lcm_vtp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vtp);
	if (volt == 1800000)
		LCM_LOGI("check regulator voltage=1.8V pass!\n");
	else
		LCM_LOGI("check regulator voltage=1.8V fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vtp);
	if (ret != 0) {
		LCM_LOGI("Failed to enable lcm_vtp: %d\n", ret);
		return ret;
	}

	return ret;
}

int lcm_vtp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable = 1;

	if (lcm_vtp == NULL)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vtp);
	LCM_LOGI("query regulator enable status[%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vtp);
		if (ret != 0) {
			LCM_LOGI("failed to disable lcm_vtp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vtp);
		if (isenable) {
			lcm_vtp_supply_disable();
		}
		else
			LCM_LOGI("regulator disable pass\n");
	}

	return ret;
}

void lcm_request_gpio_control(struct device *dev)
{
	LCM_LOGI("%s enter\n", __func__);

	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en", 0);
	gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");

	GPIO_LCD_BIAS_ENP = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enp", 0);
	gpio_request(GPIO_LCD_BIAS_ENP, "GPIO_LCD_BIAS_ENP");
	GPIO_LCD_BIAS_ENN = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enn", 0);
	gpio_request(GPIO_LCD_BIAS_ENN, "GPIO_LCD_BIAS_ENN");
}


#endif

