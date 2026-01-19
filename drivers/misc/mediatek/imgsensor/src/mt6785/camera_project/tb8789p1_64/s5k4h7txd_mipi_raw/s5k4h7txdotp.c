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

#ifndef VENDOR_EDIT
//#include "kd_camera_hw.h"
/*Caohua.Lin@Camera.Drv, 20180126 remove to adapt with mt6771*/
#endif
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "s5k4h7txdmipiraw_Sensor.h"
#include "s5k4h7txdotp.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define I2C_ID             0x20

typedef struct {
	unsigned short	awb_infoflag;
	unsigned short	lsc_infoflag;
	unsigned short	module_integrator_id;
	int		lsc_offset;
	int		lsc_group;
	int     awbc[18];
	int 	awb_page;
	unsigned short	frgcur;
	unsigned short	fbgcur;
	unsigned int	nr_gain;
	unsigned int	ng_gain;
	unsigned int	nb_gain;
	unsigned int	ngrcur;
	unsigned int	ngbcur;
	unsigned int	ngcur;
	unsigned int	nrcur;
	unsigned int	nbcur;
	unsigned int	nggolden;
	unsigned int	nrgolden;
	unsigned int	nbgolden;
	unsigned int	ngrgolden;
	unsigned int	ngbgolden;
	unsigned int	frggolden;
	unsigned int	fbggolden;
	unsigned int	awb_checksum;
	unsigned int	lsc_checksum;
} OTP;

OTP s5k4h7txd_otp_info = {0};

static kal_uint16 read_cmos_sensor_8(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, I2C_ID);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {
	(char)(addr >> 8),
	(char)(addr & 0xFF),
	 (char)(para & 0xFF)};

    iWriteRegI2C(pusendcmd, 3, I2C_ID);
}

/**********************************************************
 * s5k4h7txd_get_page_data
 * get page data
 * return true or false
 * ***********************************************************/
void s5k4h7txd_get_page_data(int pageidx, unsigned char *pdata)
{
	unsigned short get_byte = 0;
	unsigned int addr = 0x0A04;
	int i = 0;

	write_cmos_sensor_8(0x0A02, pageidx);
	write_cmos_sensor_8(0x0A00, 0x01);

	do {
		mdelay(1);
		get_byte = read_cmos_sensor_8(0x0A01);
	} while ((get_byte & 0x01) != 1);

	for (i = 0; i < 64; i++) {
		pdata[i] = read_cmos_sensor_8(addr);
		addr++;
	}

	write_cmos_sensor_8(0x0A00, 0x00);
}

unsigned short s5k4h7txd_selective_read_region_8(int pageidx, unsigned int addr)
{
	unsigned short get_byte = 0;

	write_cmos_sensor_8(0x0A02, pageidx);
	write_cmos_sensor_8(0x0A00, 0x01);
	do {
		mdelay(1);
		get_byte = read_cmos_sensor_8(0x0A01);
	} while ((get_byte & 0x01) != 1);

	get_byte = read_cmos_sensor_8(addr);
	write_cmos_sensor_8(0x0A00, 0x00);

	return get_byte;
}

/*********************************************************
 *s5k4h7txd_apply_otp_lsc
 * ******************************************************/
void s5k4h7txd_apply_otp_lsc(void)
{
	printk("OTP enable lsc\n");
	write_cmos_sensor_8(0x0B00, 0x01);
}

/*********************************************************
 * s5k4h7txd_otp_group_info
 * *****************************************************/
int s5k4h7txd_otp_group_info(void)
{
	memset(&s5k4h7txd_otp_info, 0, sizeof(OTP));

	s5k4h7txd_otp_info.lsc_infoflag = s5k4h7txd_selective_read_region_8(21, 0x0A2A);
	if (((s5k4h7txd_otp_info.lsc_infoflag >> 6)&0x3) == 0x01) {
		s5k4h7txd_otp_info.lsc_offset = 0;
		s5k4h7txd_otp_info.lsc_group = 1;
		s5k4h7txd_otp_info.lsc_checksum = s5k4h7txd_selective_read_region_8(21, 0x0A2B);
	} else {
		s5k4h7txd_otp_info.lsc_infoflag = s5k4h7txd_selective_read_region_8(22, 0x0A2A);
		if(((s5k4h7txd_otp_info.lsc_infoflag >> 6)&0x3) == 0x01) {
			s5k4h7txd_otp_info.lsc_offset = 1;
			s5k4h7txd_otp_info.lsc_group = 2;
			s5k4h7txd_otp_info.lsc_checksum = s5k4h7txd_selective_read_region_8(22, 0x0A2B);
		} else {
			printk("S5K4H7TXD OTP read data fail lsc empty!!!\n");
			goto error;
		}
	}

	s5k4h7txd_otp_info.awb_infoflag = s5k4h7txd_selective_read_region_8(21, 0x0A04);
	if (((s5k4h7txd_otp_info.awb_infoflag >> 6)&0x3) == 0x01) {
		s5k4h7txd_otp_info.awb_page = 21;
	} else {
		s5k4h7txd_otp_info.awb_infoflag = s5k4h7txd_selective_read_region_8(22, 0x0A04);
		if (((s5k4h7txd_otp_info.awb_infoflag >> 6)&0x3) == 0x01)
			s5k4h7txd_otp_info.awb_page = 22;
		else {
			printk("S5K4H7TXD OTP read data fail awb empty!!!\n");
			goto error;
		}
	}

	s5k4h7txd_otp_info.module_integrator_id = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A19);
	s5k4h7txd_otp_info.awb_checksum = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A17);

	s5k4h7_tyd_awb_r = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A0D);
	s5k4h7_tyd_awb_gr = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A0E);
	s5k4h7_tyd_awb_gb = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A0F);
	s5k4h7_tyd_awb_b = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A10);
	s5k4h7_tyd_awb_lsb = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A11);

	s5k4h7_tyd_awb_golden_r = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A12);
	s5k4h7_tyd_awb_golden_gr = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A13);
	s5k4h7_tyd_awb_golden_gb = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A14);
	s5k4h7_tyd_awb_golden_b = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A15);
	s5k4h7_tyd_awb_golden_lsb = s5k4h7txd_selective_read_region_8(s5k4h7txd_otp_info.awb_page, 0x0A16);

	printk("lfl_camera,s5k4h7_tyd_awb_r = %d,s5k4h7_tyd_awb_gr = %d,s5k4h7_tyd_awb_gb = %d,s5k4h7_tyd_awb_b = %d,s5k4h7_tyd_awb_lsb = %d,s5k4h7_tyd_awb_golden_r = %d,s5k4h7_tyd_awb_golden_gr = %d,s5k4h7_tyd_awb_golden_gb = %d,s5k4h7_tyd_awb_golden_b = %d,s5k4h7_tyd_awb_golden_lsb = %d\n",s5k4h7_tyd_awb_r,s5k4h7_tyd_awb_gr,s5k4h7_tyd_awb_gb,s5k4h7_tyd_awb_b,s5k4h7_tyd_awb_lsb,s5k4h7_tyd_awb_golden_r,s5k4h7_tyd_awb_golden_gr,s5k4h7_tyd_awb_golden_gb,s5k4h7_tyd_awb_golden_b,s5k4h7_tyd_awb_golden_lsb);

	return  0;
error:
	return  -1;
}

/*********************************************************
 * s5k4h7txd_read_page
 * s5k4h7txd_read_page1~Page21 of data
 * return true or false
 ********************************************************/
bool s5k4h7txd_read_page(int page_start, int page_end, unsigned char *pdata)
{
	bool bresult = true;
	int st_page_start = page_start;

	if (page_start <= 0 || page_end > 21) {
		bresult = false;
		printk(" OTP page_end is large!");
		return bresult;
	}
	for (; st_page_start <= page_end; st_page_start++) {
		s5k4h7txd_get_page_data(st_page_start, pdata);
	}
	return bresult;
}

/**********
s5k4h7txd_checksum_lsc_flag
*********/
unsigned int s5k4h7txd_checksum_lsc_flag(unsigned int sum_start, unsigned int sum_end, unsigned char *pdata)
{
	int i = 0;
	unsigned int start;
	unsigned int re_sum = 0;

	for (start = 0x0A04; i < 64; i++, start++) {
		if ((start >= sum_start) && (start <= sum_end)) {
			re_sum += pdata[i];
		}
	}
	return  re_sum;
}

/*******
S5K4H7TXD_checksum_awb
********/
bool S5K4H7TXD_checksum_awb(void)
{
	int i;
	bool bresult = true;
	unsigned int  checksum_awb = 0;

	write_cmos_sensor_8(0x0A02, s5k4h7txd_otp_info.awb_page);
	write_cmos_sensor_8(0x0A00, 0x01);

	for (i = 0; i <= 17; i++) {
		s5k4h7txd_otp_info.awbc[i] = read_cmos_sensor_8(0x0A05 + i);
		checksum_awb += s5k4h7txd_otp_info.awbc[i];
	}

	checksum_awb = (checksum_awb) % 255 + 1;
	printk("checksum_awb = %d,s5k4h7txd_otp_info.checksum_awb = %d",checksum_awb,s5k4h7txd_otp_info.awb_checksum);

	if (checksum_awb != s5k4h7txd_otp_info.awb_checksum) {
		s5k4h7_tyd_awb_r = 0;
		s5k4h7_tyd_awb_gr = 0;
		s5k4h7_tyd_awb_gb = 0;
		s5k4h7_tyd_awb_b = 0;
		s5k4h7_tyd_awb_lsb = 0;

		s5k4h7_tyd_awb_golden_r = 0;
		s5k4h7_tyd_awb_golden_gr = 0;
		s5k4h7_tyd_awb_golden_gb = 0;
		s5k4h7_tyd_awb_golden_b = 0;
		s5k4h7_tyd_awb_golden_lsb = 0;
		printk("S5K4H7TXD OTP checksum awb flag sum fail!!!");
		bresult &= 0;
	}
	return  bresult;
}

/**********
S5K4H7TXD_checksum_info
*********/
bool S5K4H7TXD_checksum_info(void)
{
	int i;
	UINT8 group_info;
	unsigned int checksum_info = 0;
	unsigned int checksum_info_data;
	group_info = s5k4h7txd_selective_read_region_8(21, 0x0A18);
	if (((group_info >> 6)&0x3) == 0x01) {
		write_cmos_sensor_8(0x0A02, 21);
		write_cmos_sensor_8(0x0A00, 0x01);		
		for (i = 0; i <= 15; i++) {
			checksum_info += read_cmos_sensor_8(0x0A19 + i);
		}
		checksum_info = (checksum_info) % 255 + 1;
		checksum_info_data = s5k4h7txd_selective_read_region_8(21, 0x0A29);
	} else {
		group_info = s5k4h7txd_selective_read_region_8(22, 0x0A18);
		if (((group_info >> 6)&0x3) == 0x01) {
			write_cmos_sensor_8(0x0A02, 22);
			write_cmos_sensor_8(0x0A00, 0x01);		
			for (i = 0; i <= 15; i++) {
				checksum_info += read_cmos_sensor_8(0x0A19 + i);
			}
			checksum_info = (checksum_info) % 255 + 1;
			checksum_info_data = s5k4h7txd_selective_read_region_8(22, 0x0A29);
		} else {
				printk("read otp info failed!\n");
				return false;	
			}
	}
	if (checksum_info == checksum_info_data) {
		printk("otp info checksum success\n");
		return true;
	}
	printk("otp info checksum fail,checksum_info = %u,checksum_info_data = %d\n",checksum_info,checksum_info_data);
	return false;
}

/**********
S5K4H7TXD_checksum_lsc
*********/
bool S5K4H7TXD_checksum_lsc(void)
{
	int page_start = 21, page_end = 21;
	unsigned char data_p[21][64] = {};
	bool bresult = true;
	unsigned int  checksum_lsc = 0;

	if (s5k4h7txd_otp_info.lsc_group == 1) {
		for (page_start = 1, page_end = 6; page_start <= page_end; page_start++) {
			bresult &= s5k4h7txd_read_page(page_start, page_start, data_p[page_start-1]);
			if (page_start == 6) {
				checksum_lsc += s5k4h7txd_checksum_lsc_flag(0x0A04, 0x0A2B, data_p[page_start-1]);
				continue;
			}
			checksum_lsc += s5k4h7txd_checksum_lsc_flag(0x0A04, 0x0A43, data_p[page_start-1]);
		}
	} else if (s5k4h7txd_otp_info.lsc_group == 2) {
		for (page_start = 6, page_end = 12; page_start <= page_end; page_start++) {
			bresult &= s5k4h7txd_read_page(page_start, page_start, data_p[page_start-1]);
			if (page_start == 6) {
				checksum_lsc += s5k4h7txd_checksum_lsc_flag(0x0A2C, 0x0A43, data_p[page_start-1]);
				continue;
			} else if (page_start < 12) {
				checksum_lsc += s5k4h7txd_checksum_lsc_flag(0x0A04, 0x0A43, data_p[page_start-1]);
			} else {
				checksum_lsc += s5k4h7txd_checksum_lsc_flag(0x0A04, 0x0A13, data_p[page_start-1]);
			}
		}
	}
	checksum_lsc = (checksum_lsc) % 255 + 1;
	printk("checksum_lsc = %d,s5k4h7txd_otp_info.checksum_lsc = %d",checksum_lsc,s5k4h7txd_otp_info.lsc_checksum);

	if (checksum_lsc == s5k4h7txd_otp_info.lsc_checksum) {
		s5k4h7txd_apply_otp_lsc();
	} else {
		printk("S5K4H7TXD OTP checksum lsc sum fail!!!");
		bresult &= 0;
	}

	return  bresult;
}

/********
S5K4H7TXD_otp_update
********/
bool S5K4H7TXD_otp_update(void)
{
	int result = 1;
	if(s5k4h7txd_otp_group_info() == -1){
		printk("OTP read data fail  empty!!!\n");
		result &= 0;
	}
	else {
		if(S5K4H7TXD_checksum_awb() == 0 || S5K4H7TXD_checksum_lsc() == 0 || S5K4H7TXD_checksum_info() == 0){
			printk("S5K4H7TXD OTP checksum sum fail!!!\n");
			result &= 0;
		}
		else {
			printk("S5K4H7TXD OTP checksumsum ok\n");
		}
	}
	return  result;
}

UINT8 get_txd_module_id(void)
{
	int page;
	UINT8 group_info, module_integrator_id;
	
	for (page = 21;page <= 22 ;page++){
		group_info = s5k4h7txd_selective_read_region_8(page, 0x0A18);
		if ((group_info>>6 & 0x03 )== 0x01){
			module_integrator_id = s5k4h7txd_selective_read_region_8(page, 0x0A19);			
			return module_integrator_id;
		}
	}
	return 0;
}
