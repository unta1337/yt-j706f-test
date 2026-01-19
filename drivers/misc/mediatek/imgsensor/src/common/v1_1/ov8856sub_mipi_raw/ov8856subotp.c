/*
 * Copyright (C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
/*
 * NOTE:
 * The modification is appended to initialization of image sensor.
 * After sensor initialization, use the function
 * bool otp_update_wb(unsigned char golden_rg, unsigned char golden_bg)
 * and
 * bool otp_update_lenc(void)
 * and then the calibration of AWB & LSC & BLC will be applied.
 * After finishing the OTP written, we will provide you the typical
 * value of golden sample.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <linux/slab.h>

#ifndef OV8856SUB_OTP
//#include "kd_camera_hw.h"
/*Caohua.Lin@Camera.Drv, 20180126 remove to adapt with mt6771*/
#endif
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "ov8856submipiraw_Sensor.h"
#include "ov8856subotp.h"

#define PFX "OV8856SUB_OTP"
//static struct otp_struct *otp_ptr;
#define LOG_INF(fmt, args...)   pr_info(PFX "[%s] " fmt, __func__, ##args)
#define LOG_DBG(fmt, args...)   pr_debug(PFX "[%s] " fmt, __func__, ##args)
#define LOG_ERR(fmt, args...)   pr_err(PFX "[%s] " fmt, __func__, ##args)
#define I2C_ID             0x20


//#define RG_TYPICAL_ov8856 0x011b // 0x0137
//#define BG_TYPICAL_ov8856 0x0163 // 0x0139
#define R_TYPICAL_ov8856sub 0x0000
#define G_TYPICAL_ov8856sub 0x0000
#define B_TYPICAL_ov8856sub 0x0000
#define LSC_PARAM_QTY 240
#define AWB_PARAM_QTY 24
//extern int addr; 
//extern int temp;
//extern int i;
//extern int otp_flag;

struct otp_info_t {
    uint16_t flag;
    uint16_t module_id;
    uint16_t sensor_id;
	uint16_t lens_id;
    uint16_t year;
    uint16_t month;
    uint16_t day;
    uint16_t rg_ratio_current;
    uint16_t bg_ratio_current;
    uint16_t rg_ratio_typical;
    uint16_t bg_ratio_typical;
    uint16_t r_current;
    uint16_t g_current;
    uint16_t b_current;
    uint16_t r_typical;
    uint16_t g_typical;
    uint16_t b_typical;
    uint16_t awb_param[AWB_PARAM_QTY];
    uint16_t lsc_param[LSC_PARAM_QTY];
	uint16_t checksum;
};
static struct otp_info_t ov8856sub_otp_info;

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, I2C_ID);
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, I2C_ID);
}

bool ov8856sub_read_otp_info(void) {
	uint32_t rtn = 1;
	int addr, i, awb_lsb_current, otp_flag, awb_lsb_typical;
	int temp1;
	int checksum_awb = 0;
	int checksum_lsc = 0;
    struct otp_info_t *otp_info = &ov8856sub_otp_info; //(struct otp_info_t *)param_ptr;
	
    otp_info->r_typical = R_TYPICAL_ov8856sub;
    otp_info->g_typical = G_TYPICAL_ov8856sub;
    otp_info->b_typical = B_TYPICAL_ov8856sub;

    /*TODO*/
    write_cmos_sensor(0x0100, 0x01);
    temp1 = read_cmos_sensor(0x5001);
    write_cmos_sensor(0x5001,(0x00 & 0x08) | (temp1 & (~0x08)));
    // read OTP into buffer
    write_cmos_sensor(0x3d84, 0xC0);
    write_cmos_sensor(0x3d88, 0x70); // OTP start address
    write_cmos_sensor(0x3d89, 0x10);
    write_cmos_sensor(0x3d8A, 0x72); // OTP end address
    write_cmos_sensor(0x3d8B, 0x0D);
    write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
    mdelay(10);

    // OTP base information and WB calibration data
    otp_flag = read_cmos_sensor(0x7010);
    addr = 0;
    if ((otp_flag & 0xc0) == 0x40) {
        addr = 0x7011; // base address of info group 1
    } else if ((otp_flag & 0x30) == 0x10) {
        addr = 0x701E; // base address of info group 2
    }

    if (addr != 0) {
        otp_info->flag = 0xC0; // valid info and AWB in OTP
        otp_info->module_id = read_cmos_sensor(addr);
        otp_info->sensor_id = read_cmos_sensor(addr + 1);
		otp_info->lens_id = read_cmos_sensor(addr + 2);
        otp_info->year = read_cmos_sensor(addr + 3);
        otp_info->month = read_cmos_sensor(addr + 4);

		/* read awb current */
		awb_lsb_current = read_cmos_sensor(addr + 7);
        otp_info->rg_ratio_current =
            (read_cmos_sensor(addr + 5) << 2) + ((awb_lsb_current >> 6) & 0x03);
        otp_info->bg_ratio_current =
            (read_cmos_sensor(addr + 6) << 2) + ((awb_lsb_current >> 4) & 0x03);

		/* read awb typical */
		awb_lsb_typical = read_cmos_sensor(addr + 10);
        otp_info->rg_ratio_typical =
            (read_cmos_sensor(addr + 8) << 2) + ((awb_lsb_typical >> 6) & 0x03);
	    otp_info->bg_ratio_typical =
            (read_cmos_sensor(addr + 9) << 2) + ((awb_lsb_typical >> 4) & 0x03);

		if (otp_info->rg_ratio_typical == 0) {
            otp_info->rg_ratio_typical = otp_info->rg_ratio_current;
            LOG_INF("rg_ratio_typical read 0, adjust to rg_ratio_current");
        }
        if (otp_info->bg_ratio_typical == 0) {
            otp_info->bg_ratio_typical = otp_info->bg_ratio_current;
            LOG_INF("bg_ratio_typical read 0, adjust to bg_ratio_current");
        }
		
		for (i = 0; i < 11; i++) {
			otp_info->awb_param[i] = read_cmos_sensor(addr + i);
			checksum_awb += otp_info->awb_param[i];
		}
		
		checksum_awb = (checksum_awb) % 255 + 1;
		
		LOG_ERR("checksum_awb = 0x%x, 0x%x\n", checksum_awb , read_cmos_sensor(addr + 11));
		if (read_cmos_sensor((addr + 11)) == checksum_awb) {
            otp_info->flag |= 0x10;
        }	
    } else {
        otp_info->flag = 0x00; // not info and AWB in OTP
        otp_info->module_id = 0;
        otp_info->sensor_id = 0;
		otp_info->lens_id = 0;
        otp_info->year = 0;
        otp_info->month = 0;
        otp_info->rg_ratio_current = 0;
        otp_info->bg_ratio_current = 0;
		otp_info->rg_ratio_typical = 0;
        otp_info->bg_ratio_typical = 0;
    }

    // OTP Lens Calibration
    otp_flag = read_cmos_sensor(0x702a);
    addr = 0;

    if ((otp_flag & 0xc0) == 0x40) {
        addr = 0x702B; // base address of Lenc Calibration group 1,
    } else if ((otp_flag & 0x30) == 0x10) {
        addr = 0x711D; // base address of Lenc Calibration group 2,
    }
    if (addr != 0) {
        for (i = 0; i < 240; i++) {
            otp_info->lsc_param[i] =
                read_cmos_sensor(addr + i);
            checksum_lsc += otp_info->lsc_param[i];
        }
        checksum_lsc = (checksum_lsc) % 255 + 1;

        LOG_INF("checksum_lsc=0x%x, 0x%x", checksum_lsc, read_cmos_sensor(addr + 240));
        if (read_cmos_sensor((addr + 240)) == checksum_lsc) {
            otp_info->flag |= 0x10;
        }
    } else {
        for (i = 0; i < 240; i++) {
            otp_info->lsc_param[i] = 0;
        }
    }

    /*clear OTP buffer, recommended use continuous write to accelarate,0x720a*/
    for (i = 0x7010; i <= 0x720D; i++) {
        write_cmos_sensor(i, 0);
    }

    // set 0x5001[3] to "1"
    temp1 = read_cmos_sensor(0x5001);
    write_cmos_sensor(0x5001, (0x08 & 0x08) | (temp1 & (~0x08)));

    write_cmos_sensor(0x0100, 0x00);

    /*print otp information*/
    // SENSOR_LOGI("___read otp info_________start\n");
    LOG_ERR("flag=0x%x", otp_flag);
    LOG_ERR("module_id=0x%x", otp_info->module_id);
    LOG_ERR("sensor_id=0x%x", otp_info->sensor_id);
    LOG_ERR("lens_id=0x%x", otp_info->lens_id);
    LOG_INF("data=%d-%d", otp_info->year, otp_info->month);
    LOG_ERR("rg_ratio_current=0x%x", otp_info->rg_ratio_current);
    LOG_ERR("bg_ratio_current=0x%x", otp_info->bg_ratio_current);
    LOG_ERR("rg_ratio_typical=0x%x", otp_info->rg_ratio_typical);
    LOG_ERR("bg_ratio_typical=0x%x", otp_info->bg_ratio_typical);
    return rtn;
}

bool ov8856sub_update_awb(void) {
    uint32_t rtn = 1;
    struct otp_info_t *otp_info = &ov8856sub_otp_info; //(struct otp_info_t *)param_ptr;

    /*TODO*/
    int rg, bg, R_gain, G_gain, B_gain, Base_gain;

    // apply OTP WB Calibration
    if (otp_info->flag & 0x40) {
        rg = otp_info->rg_ratio_current;
        bg = otp_info->bg_ratio_current;

        // calculate G gain
        R_gain = (otp_info->rg_ratio_typical * 1000) / rg;
        B_gain = (otp_info->bg_ratio_typical * 1000) / bg;
        G_gain = 1000;

        if (R_gain < 1000 || B_gain < 1000) {
            if (R_gain < B_gain)
                Base_gain = R_gain;
            else
                Base_gain = B_gain;
        } else {
            Base_gain = G_gain;
        }
        R_gain = 0x400 * R_gain / (Base_gain);
        B_gain = 0x400 * B_gain / (Base_gain);
        G_gain = 0x400 * G_gain / (Base_gain);

        LOG_ERR("r_Gain=0x%x\n", R_gain);
        LOG_ERR("g_Gain=0x%x\n", G_gain);
        LOG_ERR("b_Gain=0x%x\n", B_gain);

        // update sensor WB gain
        if (R_gain > 0x400) {
            write_cmos_sensor(0x5019, R_gain >> 8);
            write_cmos_sensor(0x501a, R_gain & 0x00ff);
        }
        if (G_gain > 0x400) {
            write_cmos_sensor(0x501b, G_gain >> 8);
            write_cmos_sensor(0x501c, G_gain & 0x00ff);
        }
        if (B_gain > 0x400) {
            write_cmos_sensor(0x501d, B_gain >> 8);
            write_cmos_sensor(0x501e, B_gain & 0x00ff);
        }
    }

    return rtn;
}

bool ov8856sub_update_lsc(void) {
    uint32_t rtn = 1;
    struct otp_info_t *otp_info = &ov8856sub_otp_info; //(struct otp_info_t *)param_ptr;

    /*TODO*/
    int i = 0, temp = 0;
    if (otp_info->flag & 0x10) {
        LOG_ERR("apply otp lsc \n");
        temp = read_cmos_sensor(0x5000);
        temp = 0x20 | temp;
        write_cmos_sensor(0x5000, temp);
        for (i = 0; i < 240; i++) {
            write_cmos_sensor(0x5900 + i, otp_info->lsc_param[i]);
        }
        write_cmos_sensor(0x59f4, 0x32);
    }

    return rtn;
}

bool ov8856sub_update_otp(void) {
    uint32_t rtn = 1;
	if(ov8856sub_read_otp_info() == -1){
		LOG_ERR("OTP read data fail  empty!!!\n");
		rtn &= 0;
	}
	else {
		if(ov8856sub_update_awb() == 0 || ov8856sub_update_lsc() == 0) {
			LOG_ERR("update otp fail!!!\n");
			rtn &= 0;
		}
		else {
			LOG_ERR("update otp ok\n");
		}
	}
    return rtn;
}
