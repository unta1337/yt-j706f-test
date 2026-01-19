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

#ifndef __S5K4H7_OTP_H__
#define __S5K4H7_OTP_H__

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
		       u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);

extern int s5k4h7_ofilm_awb_r_H;
extern int s5k4h7_ofilm_awb_r_L;
extern int s5k4h7_ofilm_awb_b_H;
extern int s5k4h7_ofilm_awb_b_L;
extern int s5k4h7_ofilm_awb_gr_H;
extern int s5k4h7_ofilm_awb_gr_L;
extern int s5k4h7_ofilm_awb_gb_H;
extern int s5k4h7_ofilm_awb_gb_L;

extern bool S5K4H7YXSUB_otp_update(void);
extern bool S5K4H7YXSUB_checksum_awb(void);
extern bool S5K4H7YXSUB_checksum_lsc(void);
extern UINT8 get_ofilm_module_id(void);
extern void s5k4h7yxsub_apply_otp_lsc(void);

#endif
