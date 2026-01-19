/*! \file sx933x.c
 * \brief  sx933x Driver
 *
 * Driver for the sx933x
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
//#define DEBUG
#define DRIVER_NAME "sx933x"

#define MAX_WRITE_ARRAY_SIZE 32

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
//#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <situation.h>
#include <hwmsensor.h>
#include <sensor_list.h>
#include <sensor_attr.h>
#include "sx933x.h" 	/* main struct, interrupt,init,pointers */

#define SX933x_I2C_M_WR                 0 /* for i2c Write */
#define SX933x_I2C_M_RD                 1 /* for i2c Read */

#define IDLE			    0
#define ACTIVE			  1

#define MAIN_SENSOR		1 //CS1

/* Failer Index */
#define SX933x_ID_ERROR 	1
#define SX933x_NIRQ_ERROR	2
#define SX933x_CONN_ERROR	3
#define SX933x_I2C_ERROR	4

#ifdef SAR_USE_VFE28_LDO
struct regulator *vfe28_ldo = NULL;
bool vfe28_ldo_en = false;
#endif
psx93XX_t semtech_sar_ptr;
static struct sensor_attr_t semtech_mdev;
/*! \struct sx933x
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx933x
{
    pbuttonInformation_t pbuttonInformation;
    psx933x_platform_data_t hw;		/* specific platform data settings */
} sx933x_t, *psx933x_t;

static int irq_gpio_num;

/*! \fn static int sx933x_i2c_write_16bit(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */

static int sx933x_i2c_write_16bit(psx93XX_t this, u16 reg_addr, u32 buf)
{
    int ret =  -ENOMEM;
    struct i2c_client *i2c = 0;
    struct i2c_msg msg;
    unsigned char w_buf[6];

    if (this && this->bus)
    {
        i2c = this->bus;
        w_buf[0] = (u8)(reg_addr>>8);
        w_buf[1] = (u8)(reg_addr);
        w_buf[2] = (u8)(buf>>24);
        w_buf[3] = (u8)(buf>>16);
        w_buf[4] = (u8)(buf>>8);
        w_buf[5] = (u8)(buf);

        msg.addr = i2c->addr;
        msg.flags = SX933x_I2C_M_WR;
        msg.len = 6; //2bytes regaddr + 4bytes data
        msg.buf = (u8 *)w_buf;

        ret = i2c_transfer(i2c->adapter, &msg, 1);
        if (ret < 0)
            pr_err("[SX933X]: %s - i2c write error %d\n", __func__, ret);

    }
    return ret;
}



/*! \fn static int sx933x_i2c_read_16bit(psx93XX_t this, u8 address, u8 *value)
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int sx933x_i2c_read_16bit(psx93XX_t this, u16 reg_addr, u32 *data32)
{
    int ret =  -ENOMEM;
    struct i2c_client *i2c = 0;
    struct i2c_msg msg[2];
    u8 w_buf[2];
    u8 buf[4];

    if (this && this->bus)
    {
        i2c = this->bus;

        w_buf[0] = (u8)(reg_addr>>8);
        w_buf[1] = (u8)(reg_addr);

        msg[0].addr = i2c->addr;
        msg[0].flags = SX933x_I2C_M_WR;
        msg[0].len = 2;
        msg[0].buf = (u8 *)w_buf;

        msg[1].addr = i2c->addr;;
        msg[1].flags = SX933x_I2C_M_RD;
        msg[1].len = 4;
        msg[1].buf = (u8 *)buf;

        ret = i2c_transfer(i2c->adapter, msg, 2);
        if (ret < 0)
            pr_err("[SX933x]: %s - i2c read error %d\n", __func__, ret);

        data32[0] = ((u32)buf[0]<<24) | ((u32)buf[1]<<16) | ((u32)buf[2]<<8) | ((u32)buf[3]);

    }
    return ret;
}


/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
    u32 data = 0;
    if (this)
    {
        if (sx933x_i2c_read_16bit(this,SX933X_HOSTIRQSRC_REG,&data) > 0)
            return (data & 0x00FF);
    }
    return 0;
}

static int sx933x_Hardware_Check(psx93XX_t this)
{
    //int ret;
    //u32 failcode;
    u8 loop = 0;
    this->failStatusCode = 0;

    //Check th IRQ Status
    while(this->get_nirq_low && this->get_nirq_low())
    {
        read_regStat(this);
        msleep(100);
        if(++loop >10)
        {
            this->failStatusCode = SX933x_NIRQ_ERROR;
            break;
        }
    }

#if 0
    //Check I2C Connection
    ret = sx933x_i2c_read_16bit(this, SX933X_INFO_REG, &failcode);
    if(ret < 0)
    {
        this->failStatusCode = SX933x_I2C_ERROR;
    }
#endif

    dev_info(this->pdev, "[SX933x]: sx933x failcode = 0x%x\n",this->failStatusCode);
    return (int)this->failStatusCode;
}

static int sx933x_global_variable_init(psx93XX_t this)
{
    this->irq_disabled = 0;
    this->failStatusCode = 0;
    this->reg_in_dts = true;
    return 0;
}

/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx93XX_t this)
{
    int ret = 0;
    ret = sx933x_i2c_write_16bit(this, SX933X_CMD_REG, I2C_REGCMD_COMPEN);
    return ret;

}

/****************************************************************************/
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u32 reg_value = 0;
    psx93XX_t this = semtech_sar_ptr;

    dev_info(this->pdev, "[SX933x]: Reading IRQSTAT_REG\n");
    sx933x_i2c_read_16bit(this,SX933X_HOSTIRQSRC_REG,&reg_value);
    return sprintf(buf, "%d\n", reg_value);
}


static ssize_t manual_offset_calibration_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    psx93XX_t this = semtech_sar_ptr;

    if (kstrtoul(buf, 10, &val))                //(strict_strtoul(buf, 10, &val)) {
    {
        pr_err("[SX933X]: %s - Invalid Argument\n", __func__);
        return -EINVAL;
    }

    dev_info(this->pdev, "[SX933x]: manual offset calibration val=%d\n", val);
    if (val)
        manual_offset_calibration(this);

    return count;
}

/****************************************************************************/
static ssize_t sx933x_register_write_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    u32 reg_address = 0, val = 0;
    psx93XX_t this = semtech_sar_ptr;

    if (sscanf(buf, "%x,%x", &reg_address, &val) != 2)
    {
        pr_err("[SX933x]: %s - The number of data are wrong\n",__func__);
        return -EINVAL;
    }

    sx933x_i2c_write_16bit(this, reg_address, val);
    pr_info("[SX933x]: %s - Register(0x%x) data(0x%x)\n",__func__, reg_address, val);

    return count;
}

//read registers not include the advanced one
static ssize_t sx933x_register_read_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    u32 val=0;
    int regist = 0;
    psx93XX_t this = semtech_sar_ptr;

    dev_info(this->pdev, "Reading register\n");

    if (sscanf(buf, "%x", &regist) != 1)
    {
        pr_err("[SX933x]: %s - The number of data are wrong\n",__func__);
        return -EINVAL;
    }

    sx933x_i2c_read_16bit(this, regist, &val);
    pr_info("[SX933x]: %s - Register(0x%2x) data(0x%4x)\n",__func__, regist, val);

    return count;
}

static void read_dbg_raw(psx93XX_t this)
{
	int ph, state;
	u32 uData, ph_sel;
	s32 ant_use, ant_raw;
	s32 avg, diff;
	u16 off;
	s32 adc_min, adc_max, use_flt_dlt_var;
	s32 ref_a_use=0, ref_b_use=0;	
	int ref_ph_a, ref_ph_b;
	
	psx933x_t pDevice = NULL;
	psx933x_platform_data_t pdata = NULL;
	
	pDevice = this->pDevice;
	pdata = pDevice->hw;
	ref_ph_a = pdata->ref_phase_a;
	ref_ph_b = pdata->ref_phase_b;
	//dev_info(this->pdev, "[SX933x] ref_ph_a= %d ref_ph_b= %d\n", ref_ph_a, ref_ph_b);
	
	sx933x_i2c_read_16bit(this, SX933X_STAT0_REG, &uData);
	dev_info(this->pdev, "SX933X_STAT0_REG= 0x%X\n", uData);
	
	if(ref_ph_a != 0xFF)
	{
		sx933x_i2c_read_16bit(this, SX933X_USEPH0_REG + ref_ph_a*4, &uData);
		ref_a_use = (s32)uData >> 10;
	}
	if(ref_ph_b != 0xFF)
	{
		sx933x_i2c_read_16bit(this, SX933X_USEPH0_REG + ref_ph_b*4, &uData);
		ref_b_use = (s32)uData >> 10;
	}		

	sx933x_i2c_read_16bit(this, SX933X_REG_DBG_PHASE_SEL, &ph_sel);

	sx933x_i2c_read_16bit(this, SX933X_REG_PROX_ADC_MIN, &uData);
	adc_min = (s32)uData>>10;
	sx933x_i2c_read_16bit(this, SX933X_REG_PROX_ADC_MAX, &uData);
	adc_max = (s32)uData>>10;
	sx933x_i2c_read_16bit(this, SX933X_REG_PROX_RAW, &uData);
	ant_raw = (s32)uData>>10;
	sx933x_i2c_read_16bit(this, SX933X_REG_DLT_VAR, &uData);
	use_flt_dlt_var = (s32)uData>>3;

	if (((ph_sel >> 3) & 0x7) == 0)
	{
		sx933x_i2c_read_16bit(this, SX933X_USEPH0_REG, &uData);
		ant_use = (s32)uData>>10;
		ph = 0;
	}
	else if (((ph_sel >> 3) & 0x7) == 1)
	{
		sx933x_i2c_read_16bit(this, SX933X_USEPH1_REG, &uData);
		ant_use = (s32)uData>>10;
		ph = 1;
	}
	else if (((ph_sel >> 3) & 0x7) == 2)
	{
		sx933x_i2c_read_16bit(this, SX933X_USEPH2_REG, &uData);
		ant_use = (s32)uData>>10;
		ph = 2;
	}
	else if (((ph_sel >> 3) & 0x7) == 3)
	{
		sx933x_i2c_read_16bit(this, SX933X_USEPH3_REG, &uData);
		ant_use = (s32)uData>>10;
		ph = 3;
	}
	else if (((ph_sel >> 3) & 0x7) == 4)
	{
		sx933x_i2c_read_16bit(this, SX933X_USEPH4_REG, &uData);
		ant_use = (s32)uData>>10;
		ph = 4;
	}
	else
	{
		dev_info(this->pdev, "read_dbg_raw(): invalid reg_val= 0x%X\n", ph_sel);
		ph = -1;
	}

	if(ph != -1)
	{
		sx933x_i2c_read_16bit(this, SX933X_AVGPH0_REG + ph*4, &uData);
		avg = (s32)uData>>10;
		sx933x_i2c_read_16bit(this, SX933X_DIFFPH0_REG + ph*4, &uData);
		diff = (s32)uData>>10;
		sx933x_i2c_read_16bit(this, SX933X_OFFSETPH0_REG + ph*4*2, &uData);
		off = (u16)(uData & 0x7FFF);
		state = psmtcButtons[ph].state;
#if 0 //remove log
		if(ref_ph_a != 0xFF && ref_ph_b != 0xFF)
		{
			dev_info(this->pdev, 
			"SMTC_DBG PH= %d USE= %d RAW= %d PH%d_USE= %d PH%d_USE= %d STATE= %d AVG= %d DIFF= %d OFF= %d ADC_MIN= %d ADC_MAX= %d DLT= %d SMTC_END\n",
			ph,    ant_use, ant_raw, ref_ph_a, ref_a_use,  ref_ph_b, ref_b_use,    state,    avg,    diff,    off,    adc_min,   adc_max,    use_flt_dlt_var);
		}
		else if(ref_ph_a != 0xFF)
		{
			dev_info(this->pdev, 
			"SMTC_DBG PH= %d USE= %d RAW= %d PH%d_USE= %d STATE= %d AVG= %d DIFF= %d OFF= %d ADC_MIN= %d ADC_MAX= %d DLT= %d SMTC_END\n",
			ph,    ant_use, ant_raw, ref_ph_a, ref_a_use,  state,    avg,    diff,    off,    adc_min,   adc_max,    use_flt_dlt_var);
		}
		else if(ref_ph_b != 0xFF)
		{
			dev_info(this->pdev, 
			"SMTC_DBG PH= %d USE= %d RAW= %d PH%d_USE= %d STATE= %d AVG= %d DIFF= %d OFF= %d ADC_MIN= %d ADC_MAX= %d DLT= %d SMTC_END\n",
			ph,    ant_use, ant_raw, ref_ph_b, ref_b_use,  state,    avg,    diff,    off,    adc_min,   adc_max,    use_flt_dlt_var);
		}
		else
		{
			dev_info(this->pdev, 
			"SMTC_DBG PH= %d USE= %d RAW= %d STATE= %d AVG= %d DIFF= %d OFF= %d ADC_MIN= %d ADC_MAX= %d DLT= %d SMTC_END\n",
			ph,    ant_use, ant_raw, state,    avg,    diff,    off,    adc_min,   adc_max,    use_flt_dlt_var);
		}
#endif
	}
}

struct raw_data_t {
	s32 useful;
	s32 average;
	s32 diff;
	u16 offset;
};

struct raw_data_t raw_data_phase[5];

static void read_rawData(psx93XX_t this)
{
	u8 csx, index;
	//s32 useful, average, diff;
	s32 ref_a_use=0, ref_b_use=0;
	u32 uData;
	//u16 offset;
	int state;
	psx933x_t pDevice = NULL;
	psx933x_platform_data_t pdata = NULL;
	int ref_ph_a, ref_ph_b;
	int32_t value[3] = {0};

	if(this)
	{		
		pDevice = this->pDevice;
		pdata = pDevice->hw;
		ref_ph_a = pdata->ref_phase_a;
		ref_ph_b = pdata->ref_phase_b;
		dev_info(this->pdev, "[SX933x] ref_ph_a= %d ref_ph_b= %d\n", ref_ph_a, ref_ph_b);

		sx933x_i2c_read_16bit(this, SX933X_STAT0_REG, &uData);
		//dev_info(this->pdev, "SX933X_STAT0_REG= 0x%X\n", uData);
#ifdef CONFIG_TARGET_PROJECT_P522
		value[0] = (uData >> 24) & 0xF;
#else
		value[0] = (uData >> 24) & 0x7;
#endif

		if(ref_ph_a != 0xFF)
		{
			sx933x_i2c_read_16bit(this, SX933X_USEPH0_REG + ref_ph_a*4, &uData);
			ref_a_use = (s32)uData >> 10;
		}
		if(ref_ph_b != 0xFF)
		{
			sx933x_i2c_read_16bit(this, SX933X_USEPH0_REG + ref_ph_b*4, &uData);
			ref_b_use = (s32)uData >> 10;
		}		

		for(csx =0; csx<5; csx++)
		{
			index = csx*4;
			sx933x_i2c_read_16bit(this, SX933X_USEPH0_REG + index, &uData);
			raw_data_phase[csx].useful = (s32)uData>>10;
			sx933x_i2c_read_16bit(this, SX933X_AVGPH0_REG + index, &uData);
			raw_data_phase[csx].average = (s32)uData>>10;
			sx933x_i2c_read_16bit(this, SX933X_DIFFPH0_REG + index, &uData);
			raw_data_phase[csx].diff = (s32)uData>>10;
			sx933x_i2c_read_16bit(this, SX933X_OFFSETPH0_REG + index*2, &uData);
			raw_data_phase[csx].offset = (u16)(uData & 0x7FFF);

			state = psmtcButtons[csx].state;
#if 0 //remove log
			if(ref_ph_a != 0xFF && ref_ph_b != 0xFF)
			{
				dev_info(this->pdev, 
				"SMTC_DAT PH= %d DIFF= %d USE= %d PH%d_USE= %d PH%d_USE= %d STATE= %d OFF= %d AVG= %d SMTC_END\n",
				csx, raw_data_phase[csx].diff, raw_data_phase[csx].useful,
				ref_ph_a, ref_a_use, ref_ph_b, ref_b_use, state, 
				raw_data_phase[csx].offset, raw_data_phase[csx].average);
			}
			else if(ref_ph_a != 0xFF)
			{				
				dev_info(this->pdev, 
				"SMTC_DAT PH= %d DIFF= %d USE= %d PH%d_USE= %d STATE= %d OFF= %d AVG= %d SMTC_END\n",
				csx, raw_data_phase[csx].diff, raw_data_phase[csx].useful,
				ref_ph_a, ref_a_use, state, raw_data_phase[csx].offset, raw_data_phase[csx].average);
			}
			else if(ref_ph_b != 0xFF)
			{				
				dev_info(this->pdev, 
				"SMTC_DAT PH= %d DIFF= %d USE= %d PH%d_USE= %d STATE= %d OFF= %d AVG= %d SMTC_END\n",
				csx, raw_data_phase[csx].diff, raw_data_phase[csx].useful,
				ref_ph_b, ref_b_use, state, raw_data_phase[csx].offset, raw_data_phase[csx].average);
			}
			else
			{
				dev_info(this->pdev, 
				"SMTC_DAT PH= %d DIFF= %d USE= %d STATE= %d OFF= %d AVG= %d SMTC_END\n",
				csx, raw_data_phase[csx].diff, raw_data_phase[csx].useful,
				state, raw_data_phase[csx].offset, raw_data_phase[csx].average);
			}
#endif
		}

		if ((value[0] & 0x1) == 0x1) {
			value[1] = raw_data_phase[0].diff;
			value[2] = raw_data_phase[0].offset;
#ifdef CONFIG_TARGET_PROJECT_P522
		} else if ((value[0] & 0x4) == 0x4) {
#else
		} else if ((value[0] & 0x2) == 0x2) {
#endif
			value[1] = raw_data_phase[1].diff;
			value[2] = raw_data_phase[1].offset;
#ifdef CONFIG_TARGET_PROJECT_P522
		} else if ((value[0] & 0x8) == 0x8) {
#else
		} else if ((value[0] & 0x4) == 0x4) {
#endif
			value[1] = raw_data_phase[2].diff;
			value[2] = raw_data_phase[2].offset;
		}
		//dev_info(this->pdev, "value0= 0x%X\n", value[0]);
		sar_data_report(value);
		read_dbg_raw(this);
	}	
}

static ssize_t sx933x_raw_data_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    psx93XX_t this = semtech_sar_ptr;
    read_rawData(this);
    return 0;
}

static ssize_t sar_info_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	u32 reg_value = 0;
	psx93XX_t this = semtech_sar_ptr;
	dev_info(this->pdev, "Reading SAR ID FOR ATA TEST\n");
	sx933x_i2c_read_16bit(this,SX933X_INFO_REG,&reg_value);
	return sprintf(buf, "0x%x\n", reg_value);
}

static ssize_t sar_show_proxstatus_cs0(struct device *dev,
								struct device_attribute *attr, char *buf)
{  
	u32 reg_value = 0;
	int state =0;
	uint8_t status;
	psx93XX_t this = semtech_sar_ptr;
	dev_info(this->pdev, "Reading SAR state\n");
	sx933x_i2c_read_16bit(this,SX933X_STAT0_REG,&reg_value);
	status = (uint8_t)((reg_value&0x3f000000) >> 24);
	if(((status>>0) & 0x01)==1)
	{
		state=1;//near
		pr_info("[CS1]Reading sx SAR state is:%d\n",state);
		//read_rawData(this);
	}
	else
	{
		state=2;//realse
		pr_info("[CS0]Reading sx9328 SAR state is:%d\n",state);
		//read_rawData(this);
	}
	return	sprintf(buf, "%d\n", state);
}

#ifdef CONFIG_TARGET_PROJECT_P530
static ssize_t sar_show_proxstatus_cs1(struct device *dev,
								struct device_attribute *attr, char *buf)
{  
	u32 reg_value = 0;
	int state =0;
	uint8_t status;
	psx93XX_t this = semtech_sar_ptr;
	dev_info(this->pdev, "Reading SAR state\n");
	sx933x_i2c_read_16bit(this,SX933X_STAT0_REG,&reg_value);
	status = (uint8_t)((reg_value&0x3f000000) >> 24);
	if(((status>>1) & 0x01)==1)
	{
		state=1;//near
		pr_info("[CS1]Reading sx SAR state is:%d\n",state);
		//read_rawData(this);
	}
	else
	{
		state=2;//realse
		pr_info("[CS1]Reading sx9328 SAR state is:%d\n",state);
		//read_rawData(this);
	}

	return	sprintf(buf, "%d\n", state);
}
#endif

static ssize_t sar_show_proxstatus_cs2(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	u32 reg_value = 0;
	int state =0;
	uint8_t status;
	psx93XX_t this = semtech_sar_ptr;
	dev_info(this->pdev, "Reading SAR state\n");
	sx933x_i2c_read_16bit(this,SX933X_STAT0_REG,&reg_value);
	status = (uint8_t)((reg_value&0x3f000000) >> 24);
	if (((status>>2) & 0x01)==1)
	{
		state=1;//near
		pr_info("[CS2]Reading sx SAR state is:%d\n",state);
		//read_rawData(this);
	}
	else
	{
		state=2;//realse
		pr_info("[CS2]Reading sx9328 SAR state is:%d\n",state);
		//read_rawData(this);
	}
	return	sprintf(buf, "%d\n", state);
}

#ifdef CONFIG_TARGET_PROJECT_P522
static ssize_t sar_show_proxstatus_cs3(struct device *dev,
								struct device_attribute *attr, char *buf)
{  
	u32 reg_value = 0;
	int state =0;
	uint8_t status;
	psx93XX_t this = semtech_sar_ptr;
	dev_info(this->pdev, "Reading SAR state\n");
	sx933x_i2c_read_16bit(this,SX933X_STAT0_REG,&reg_value);
	status = (uint8_t)((reg_value&0x3f000000) >> 24);
	if(((status>>3) & 0x01)==1)
	{
		state=1;//near
		pr_info("[CS1]Reading sx SAR state is:%d\n",state);
		//read_rawData(this);
	}
	else
	{
		state=2;//realse
		pr_info("[CS1]Reading sx9328 SAR state is:%d\n",state);
		//read_rawData(this);
	}

	return	sprintf(buf, "%d\n", state);
}
#endif

static DEVICE_ATTR(calibrate, 0664, manual_offset_calibration_show,manual_offset_calibration_store);
static DEVICE_ATTR(register_write,  0664, NULL,sx933x_register_write_store);
static DEVICE_ATTR(register_read,0664, NULL,sx933x_register_read_store);
static DEVICE_ATTR(raw_data,0664,sx933x_raw_data_show,NULL);
static DEVICE_ATTR(sar_info,0444,sar_info_show,NULL);
static DEVICE_ATTR(sar_proxstatus_cs0,0444,sar_show_proxstatus_cs0,NULL);
#ifdef CONFIG_TARGET_PROJECT_P530
static DEVICE_ATTR(sar_proxstatus_cs1,0444,sar_show_proxstatus_cs1,NULL);
#endif
static DEVICE_ATTR(sar_proxstatus_cs2,0444,sar_show_proxstatus_cs2,NULL);
#ifdef CONFIG_TARGET_PROJECT_P522
static DEVICE_ATTR(sar_proxstatus_cs3,0444,sar_show_proxstatus_cs3,NULL);
#endif

static struct attribute *sx933x_attributes[] =
{
    &dev_attr_calibrate.attr,
    &dev_attr_register_write.attr,
    &dev_attr_register_read.attr,
    &dev_attr_raw_data.attr,
    &dev_attr_sar_info.attr,
    &dev_attr_sar_proxstatus_cs0.attr,
#ifdef CONFIG_TARGET_PROJECT_P530
    &dev_attr_sar_proxstatus_cs1.attr,
#endif
    &dev_attr_sar_proxstatus_cs2.attr,
#ifdef CONFIG_TARGET_PROJECT_P522
    &dev_attr_sar_proxstatus_cs3.attr,
#endif
    NULL,
};
static struct attribute_group sx933x_attr_group =
{
    .attrs = sx933x_attributes,
};

/**************************************/

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct
 */
static void sx933x_reg_init(psx93XX_t this)
{
    psx933x_t pDevice = 0;
    psx933x_platform_data_t pdata = 0;
    int i = 0;
    //uint32_t tmpvalue;
    /* configure device */
    dev_info(this->pdev, "[SX933x]:Going to Setup I2C Registers\n");
    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
#if 0		
        /*******************************************************************************/
        // try to initialize from device tree!
        /*******************************************************************************/
        while ( i < ARRAY_SIZE(sx933x_i2c_reg_setup))
        {
            /* Write all registers/values contained in i2c_reg */
            dev_info(this->pdev, "[SX933x]:Going to Write Reg: 0x%x Value: 0x%x\n",
                     sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
            tmpvalue = sx933x_i2c_reg_setup[i].val;
            if (sx933x_i2c_reg_setup[i].reg == SX933X_GNRLCTRL2_REG)
            {
                if((sx933x_i2c_reg_setup[i].val & 0x3F) == 0)
                {
                    tmpvalue = (sx933x_i2c_reg_setup[i].val|0x3F);
                }
            }
            //sx933x_i2c_write_16bit(this, sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
            sx933x_i2c_write_16bit(this, sx933x_i2c_reg_setup[i].reg, tmpvalue);
            i++;
        }
#else
        if (this->reg_in_dts == true)
        {
			i = 0;
            while ( i < pdata->i2c_reg_num)
            {
                /* Write all registers/values contained in i2c_reg */
                dev_info(this->pdev, "[SX933x]: Going to Write Reg from dts: 0x%x Value: 0x%x\n",
                         pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
                sx933x_i2c_write_16bit(this, (u16)pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
                i++;
            }
        }
        else     // use static ones!!
        {
            while ( i < ARRAY_SIZE(sx933x_i2c_reg_setup))
            {
                /* Write all registers/values contained in i2c_reg */
                dev_info(this->pdev, "[SX933x]:Going to Write Reg: 0x%x Value: 0x%x\n",
                         sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
                sx933x_i2c_write_16bit(this, sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
                i++;
            }
        }
#endif
        /*******************************************************************************/
        sx933x_i2c_write_16bit(this, SX933X_CMD_REG,SX933X_PHASE_CONTROL);  //enable phase control
    }
    else
    {
        dev_err(this->pdev, "[SX933x]: ERROR! platform data 0x%p\n",pDevice->hw);
    }

}


/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx93XX_t this)
{
    int ret;
    if (this)
    {
        pr_info("[SX933x]: SX933x income initialize\n");
        /* prepare reset by disabling any irq handling */
        this->irq_disabled = 1;
        disable_irq(this->irq);
        /* perform a reset */
        sx933x_i2c_write_16bit(this, SX933X_RESET_REG, I2C_SOFTRESET_VALUE);
        /* wait until the reset has finished by monitoring NIRQ */
        dev_info(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
        /* just sleep for awhile instead of using a loop with reading irq status */
        msleep(100);
        ret = sx933x_global_variable_init(this);
        sx933x_reg_init(this);
        msleep(100); /* make sure everything is running */
        manual_offset_calibration(this);

        /* re-enable interrupt handling */
        enable_irq(this->irq);

        /* make sure no interrupts are pending since enabling irq will only
        * work on next falling edge */
        read_regStat(this);
        return 0;
    }
    return -ENOMEM;
}

/*!
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct
 */
static void touchProcess(psx93XX_t this)
{
    int counter = 0;
    u32 i = 0;
    int numberOfButtons = 0;
    psx933x_t pDevice = NULL;
    struct _buttonInfo *buttons = NULL;
    struct input_dev *input = NULL;

    struct _buttonInfo *pCurrentButton  = NULL;
    if (this && (pDevice = this->pDevice))
    {
        sx933x_i2c_read_16bit(this, SX933X_STAT0_REG, &i);

        buttons = pDevice->pbuttonInformation->buttons;
        input = pDevice->pbuttonInformation->input;
        numberOfButtons = pDevice->pbuttonInformation->buttonSize;

        if (unlikely( (buttons==NULL) || (input==NULL) ))
        {
            dev_err(this->pdev, "[SX933x]:ERROR!! buttons or input NULL!!!\n");
            return;
        }

        for (counter = 0; counter < numberOfButtons; counter++)
        {
            pCurrentButton = &buttons[counter];
            if (pCurrentButton==NULL)
            {
                dev_err(this->pdev,"[SX933x]:ERROR!! current button at index: %d NULL!!!\n", counter);
                return; // ERRORR!!!!
            }
            switch (pCurrentButton->state)
            {
            case IDLE: /* Button is not being touched! */
                if (((i & pCurrentButton->mask) == pCurrentButton->mask))
                {
                    /* User pressed button */
                    dev_info(this->pdev, "[SX933x]:Button %d touched\n", counter);
                    input_report_key(input, pCurrentButton->keycode, 1);
                    input_sync(input);
                    input_report_key(input, pCurrentButton->keycode, 0);
                    input_sync(input);
                    pCurrentButton->state = ACTIVE;
                }
                else
                {
                    dev_info(this->pdev, "[SX933x]:Button %d already released.\n",counter);
                }
                break;
            case ACTIVE: /* Button is being touched! */
                if (((i & pCurrentButton->mask) != pCurrentButton->mask))
                {
                    /* User released button */
                    dev_info(this->pdev, "[SX933x]:Button %d released\n",counter);
                    input_report_key(input, pCurrentButton->keycode_release, 1);
                    input_sync(input);
                    input_report_key(input, pCurrentButton->keycode_release, 0);
                    input_sync(input);
                    pCurrentButton->state = IDLE;
                }
                else
                {
                    dev_info(this->pdev, "[SX933x]:Button %d still touched.\n",counter);
                }
                break;
            default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                break;
            };
        }
        input_sync(input);

        //dev_info(this->pdev, "Leaving touchProcess()\n");
    }
}

static int sx933x_parse_dt(struct sx933x_platform_data *pdata, struct device *dev)
{
    struct device_node *dNode = dev->of_node;
    enum of_gpio_flags flags;
//	u8 ref_phases;
    //int ret;

    if (dNode == NULL)
        return -ENODEV;

    pdata->irq_gpio= of_get_named_gpio_flags(dNode,
                     "Semtech,nirq-gpio", 0, &flags);
    irq_gpio_num = pdata->irq_gpio;
    if (pdata->irq_gpio < 0)
    {
        pr_err("[SX933x]: %s - get irq_gpio error\n", __func__);
        return -ENODEV;
    }

	pdata->ref_phase_a = -1;
	pdata->ref_phase_b = -1;
	if ( of_property_read_u32(dNode,"Semtech,ref-phases-a",&pdata->ref_phase_a) )
	{
		pr_err("[SX933x]: %s - get ref-phases error\n", __func__);
        return -ENODEV;
	}
	if ( of_property_read_u32(dNode,"Semtech,ref-phases-b",&pdata->ref_phase_b) )
	{
		pr_err("[SX933x]: %s - get ref-phases-b error\n", __func__);
        return -ENODEV;
	}
	pr_info("[SX933x]: %s ref_phase_a= %d ref_phase_b= %d\n", 
		__func__, pdata->ref_phase_a, pdata->ref_phase_b);

    /***********************************************************************/
    // load in registers from device tree
    of_property_read_u32(dNode,"Semtech,reg-num",&pdata->i2c_reg_num);
    // layout is register, value, register, value....
    // if an extra item is after just ignore it. reading the array in will cause it to fail anyway
    pr_info("[sx933x]:%s -  size of elements %d\n", __func__,pdata->i2c_reg_num);
    if (pdata->i2c_reg_num > 0)
    {
        // initialize platform reg data array
        pdata->pi2c_reg = devm_kzalloc(dev,sizeof(struct smtc_reg_data)*pdata->i2c_reg_num, GFP_KERNEL);
        if (unlikely(pdata->pi2c_reg == NULL))
        {
            return -ENOMEM;
        }

        // initialize the array
        if (of_property_read_u32_array(dNode,"Semtech,reg-init",(u32*)&(pdata->pi2c_reg[0]),sizeof(struct smtc_reg_data)*pdata->i2c_reg_num/sizeof(u32)))
            return -ENOMEM;
    }

    /***********************************************************************/

    pr_info("[SX933x]: %s -[%d] parse_dt complete\n", __func__,pdata->irq_gpio);
    return 0;
}

/* get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
static int sx933x_init_platform_hw(struct i2c_client *client)
{
    psx93XX_t this = i2c_get_clientdata(client);
    struct sx933x *pDevice = NULL;
    struct sx933x_platform_data *pdata = NULL;

    int rc;

    pr_info("[SX933x] : %s init_platform_hw start!",__func__);

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
        if (gpio_is_valid(pdata->irq_gpio))
        {
            rc = gpio_request(pdata->irq_gpio, "sx933x_irq_gpio");
            if (rc < 0)
            {
                dev_err(this->pdev, "SX933x Request gpio. Fail![%d]\n", rc);
                return rc;
            }
            rc = gpio_direction_input(pdata->irq_gpio);
            if (rc < 0)
            {
                dev_err(this->pdev, "SX933x Set gpio direction. Fail![%d]\n", rc);
                return rc;
            }
            this->irq = client->irq = gpio_to_irq(pdata->irq_gpio);
        }
        else
        {
            dev_err(this->pdev, "SX933x Invalid irq gpio num.(init)\n");
        }
    }
    else
    {
        pr_err("[SX933x] : %s - Do not init platform HW", __func__);
    }

    pr_err("[SX933x]: %s - sx933x_irq_debug\n",__func__);
    return rc;
}

static void sx933x_exit_platform_hw(struct i2c_client *client)
{
    psx93XX_t this = i2c_get_clientdata(client);
    struct sx933x *pDevice = NULL;
    struct sx933x_platform_data *pdata = NULL;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
        if (gpio_is_valid(pdata->irq_gpio))
        {
            gpio_free(pdata->irq_gpio);
        }
        else
        {
            dev_err(this->pdev, "Invalid irq gpio num.(exit)\n");
        }
    }
    return;
}

static int sx933x_get_nirq_state(void)
{
    return  !gpio_get_value(irq_gpio_num);
}

static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
    unsigned long flags;
    if (this)
    {
        dev_info(this->pdev, "sx93XX_schedule_work()\n");
        spin_lock_irqsave(&this->lock,flags);
        /* Stop any pending penup queues */
        cancel_delayed_work(&this->dworker);
        //after waiting for a delay, this put the job in the kernel-global workqueue. so no need to create new thread in work queue.
        schedule_delayed_work(&this->dworker,delay);
        spin_unlock_irqrestore(&this->lock,flags);
    }
    else
        printk(KERN_ERR "sx93XX_schedule_work, NULL psx93XX_t\n");
}

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
    psx93XX_t this = 0;
    if (pvoid)
    {
        this = (psx93XX_t)pvoid;
        if ((!this->get_nirq_low) || this->get_nirq_low())
        {
            sx93XX_schedule_work(this,0);
        }
        else
        {
            dev_err(this->pdev, "sx93XX_irq - nirq read high\n");
        }
    }
    else
    {
        printk(KERN_ERR "sx93XX_irq, NULL pvoid\n");
    }
    return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
    psx93XX_t this = 0;
    int status = 0;
    int counter = 0;
    u8 nirqLow = 0;
    if (work)
    {
        this = container_of(work,sx93XX_t,dworker.work);

        if (!this)
        {
            printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
            return;
        }
        if (unlikely(this->useIrqTimer))
        {
            if ((!this->get_nirq_low) || this->get_nirq_low())
            {
                nirqLow = 1;
            }
        }
        /* since we are not in an interrupt don't need to disable irq. */
        status = this->refreshStatus(this);
        counter = -1;
        dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);

#if 0
        while((++counter) < MAX_NUM_STATUS_BITS)   /* counter start from MSB */
        {
            if (((status>>counter) & 0x01) && (this->statusFunc[counter]))
            {
                dev_info(this->pdev, "SX933x Function Pointer Found. Calling\n");
                this->statusFunc[counter](this);
            }
        }
#endif
        this->statusFunc[3](this);

        while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
                if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
                //dev_info(this->pdev, "SX932x Function Pointer Found. Calling,counter = %d\n",counter);
                this->statusFunc[counter](this);
                }
        }

        if (unlikely(this->useIrqTimer && nirqLow))
        {
            /* Early models and if RATE=0 for newer models require a penup timer */
            /* Queue up the function again for checking on penup */
            sx93XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
        }
    }
    else
    {
        printk(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
    }
}

int sx93XX_IRQ_init(psx93XX_t this)
{
    int err = 0;
    if (this && this->pDevice)
    {
        /* initialize spin lock */
        spin_lock_init(&this->lock);
        /* initialize worker function */
        INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
        /* initailize interrupt reporting */
        this->irq_disabled = 0;
        err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
                          this->pdev->driver->name, this);
        if (err)
        {
            dev_err(this->pdev, "irq %d busy?\n", this->irq);
            return err;
        }
        dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
    }
    return -ENOMEM;
}

static int sx933x_open_report_data(int open)
{
	pr_info("%s open = %d\n", __func__, open);
	if (open == 1) {
		psx93XX_t this = semtech_sar_ptr;
		read_rawData(this);
	}
	return 0;
}

static int sx933x_batch(int flag,
	int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int sx933x_flush(void)
{
	pr_info("%s\n", __func__);
	return situation_flush_report(ID_SAR);
}

static int sx933x_get_data(int *probability, int *status)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int sx933x_fill_situation_func(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;

	ctl.open_report_data = sx933x_open_report_data;
	ctl.batch = sx933x_batch;
	ctl.flush = sx933x_flush;
	ctl.is_support_wake_lock = true;
	ctl.is_support_batch = false;
	err = situation_register_control_path(&ctl, ID_SAR);
	if (err) {
		pr_err("register stationary control path err\n");
		return -1;
	}

	data.get_data = sx933x_get_data;
	err = situation_register_data_path(&data, ID_SAR);
	if (err) {
		pr_err("register stationary data path err\n");
		return -1;
	}

	return 0;
}

static const struct file_operations semtech_sar_fops = {
	.owner = THIS_MODULE,
};

static int semtech_misc_init()
{
	int err;

	semtech_mdev.minor = ID_SAR;
	semtech_mdev.name = "m_sar_misc";
	semtech_mdev.fops = &semtech_sar_fops;
	err = sensor_attr_register(&semtech_mdev);
	if (err)
		pr_err("%s: unable to register sar misc device\n", __func__);

	return err;
}

/*! \fn static int sx933x_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx933x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0;
    int err = 0;

    psx93XX_t this = 0;
    psx933x_t pDevice = 0;
    psx933x_platform_data_t pplatData = 0;
    struct totalButtonInformation *pButtonInformationData = NULL;
    struct input_dev *input = NULL;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct sensorInfo_NonHub_t sar_devinfo;
	u32 chip_id = 0;

    dev_info(&client->dev, "[SX933x]:sx933x_probe()\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
    {
        dev_err(&client->dev, "[SX933x]:Check i2c functionality.Fail!\n");
        err = -EIO;
        return err;
    }

    this = devm_kzalloc(&client->dev,sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
    dev_info(&client->dev, "[SX933x]:\t Initialized Main Memory: 0x%p\n",this);

    pButtonInformationData = devm_kzalloc(&client->dev , sizeof(struct totalButtonInformation), GFP_KERNEL);
    if (!pButtonInformationData)
    {
        dev_err(&client->dev, "[SX933x]:Failed to allocate memory(totalButtonInformation)\n");
        err = -ENOMEM;
        return err;
    }

    pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons);
    pButtonInformationData->buttons =  psmtcButtons;
    pplatData = devm_kzalloc(&client->dev,sizeof(struct sx933x_platform_data), GFP_KERNEL);
    if (!pplatData)
    {
        dev_err(&client->dev, "[SX933x]:platform data is required!\n");
        return -EINVAL;
    }
    pplatData->get_is_nirq_low = sx933x_get_nirq_state;
    pplatData->pbuttonInformation = pButtonInformationData;

#ifdef SAR_USE_VFE28_LDO
    if (vfe28_ldo == NULL) {
        dev_err(&client->dev, "[SX933x]: vfe28_ldo is NULL, get it\n");
    	vfe28_ldo = regulator_get(&client->dev, "vfe28");
    }

    if (IS_ERR(vfe28_ldo)) {
            if (PTR_ERR(vfe28_ldo) == -EPROBE_DEFER) {
                    err = PTR_ERR(vfe28_ldo);
            }
            dev_err(&client->dev, "%s: Failed to get regulator\n", __func__);
    } else {
            if (!regulator_is_enabled(vfe28_ldo)) {
                dev_err(&client->dev, "%s: vfe28_ldo is disabled, enable it now\n", __func__);
            	err = regulator_enable(vfe28_ldo);
            	if (err) {
                    dev_err(&client->dev, "%s: Error %d enable regulator\n", __func__, err);
            	} else {
            		vfe28_ldo_en = true;
		}
	    }
            dev_err(&client->dev, "cap_vdd regulator is %s\n",
                 regulator_is_enabled(vfe28_ldo) ? "on" : "off");
    }
#endif

    client->dev.platform_data = pplatData;
    err = sx933x_parse_dt(pplatData, &client->dev);
    if (err)
    {
        dev_err(&client->dev, "[SX933x]:could not setup pin\n");
        return ENODEV;
    }

    pplatData->init_platform_hw = sx933x_init_platform_hw;
    dev_err(&client->dev, "[SX933x]:SX933x init_platform_hw done!\n");

    if (this)
    {
        dev_info(&client->dev, "[SX933x]:SX933x initialize start!!");
        /* In case we need to reinitialize data
        * (e.q. if suspend reset device) */
        this->init = initialize;
        /* shortcut to read status of interrupt */
        this->refreshStatus = read_regStat;
        /* pointer to function from platform data to get pendown
        * (1->NIRQ=0, 0->NIRQ=1) */
        this->get_nirq_low = pplatData->get_is_nirq_low;
        /* save irq in case we need to reference it */
        this->irq = client->irq;
        /* do we need to create an irq timer after interrupt ? */
        this->useIrqTimer = 0;

        /* Setup function to call on corresponding reg irq source bit */
        if (MAX_NUM_STATUS_BITS>= 8)
        {
            this->statusFunc[0] = 0; /* TXEN_STAT */
            this->statusFunc[1] = 0; /* UNUSED */
            this->statusFunc[2] = 0; /* UNUSED */
            this->statusFunc[3] = read_rawData; /* CONV_STAT */
            this->statusFunc[4] = touchProcess; /* COMP_STAT */
            this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
            this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
            this->statusFunc[7] = 0; /* RESET_STAT */
        }

        /* setup i2c communication */
        this->bus = client;
        i2c_set_clientdata(client, this);

        /* record device struct */
        this->pdev = &client->dev;

	//Check chip id
	err = sx933x_i2c_read_16bit(this, SX933X_INFO_REG, &chip_id);
	if(err < 0)
	{
		dev_info(this->pdev, "[SX933x]: sx933x chip_id = 0x%x\n", chip_id);
		return -EINVAL;
	}
	dev_info(this->pdev, "[SX933x]: check chipid success = 0x%x\n", chip_id);

       /* create memory for device specific struct */
        this->pDevice = pDevice = devm_kzalloc(&client->dev,sizeof(sx933x_t), GFP_KERNEL);
        dev_info(&client->dev, "[SX933x]:\t Initialized Device Specific Memory: 0x%p\n",pDevice);

        if (pDevice)
        {

	    semtech_sar_ptr = this;
	    err = semtech_misc_init();
	    if (err < 0) {
		pr_err("%s: unable to register sar device\n", __func__);
		return -EINVAL;
	    }
            /* for accessing items in user data (e.g. calibrate) */
            err = sysfs_create_group(&semtech_mdev.this_device->kobj, &sx933x_attr_group);
	    if (err < 0) {
		pr_err("%s: error creating sysfs attr files\n", __func__);
		return -EINVAL;
	    }
            /* Add Pointer to main platform data struct */
            pDevice->hw = pplatData;

            /* Check if we hava a platform initialization function to call*/
            if (pplatData->init_platform_hw)
                pplatData->init_platform_hw(client);

            /* Initialize the button information initialized with keycodes */
            pDevice->pbuttonInformation = pplatData->pbuttonInformation;
#if 1
            /* Create the input device */
            input = input_allocate_device();
            if (!input)
            {
                return -ENOMEM;
            }
            /* Set all the keycodes */
            __set_bit(EV_KEY, input->evbit);
#if 1
            for (i = 0; i < pButtonInformationData->buttonSize; i++)
            {
                __set_bit(pButtonInformationData->buttons[i].keycode,input->keybit);
                __set_bit(pButtonInformationData->buttons[i].keycode_release,input->keybit);
                pButtonInformationData->buttons[i].state = IDLE;
            }
#endif
            /* save the input pointer and finish initialization */
            pButtonInformationData->input = input;
            input->name = "SX933x Cap Touch";
            input->id.bustype = BUS_I2C;
            if(input_register_device(input))
            {
                return -ENOMEM;
            }
#endif
        }


        sx93XX_IRQ_init(this);
        /* call init function pointer (this should initialize all registers */
        if (this->init)
        {
            this->init(this);
        }
        else
        {
            dev_err(this->pdev,"[SX933x]:No init function!!!!\n");
            return -ENOMEM;
        }
    }
    else
    {
        return -1;
    }

    sx933x_Hardware_Check(this);

    pplatData->exit_platform_hw = sx933x_exit_platform_hw;
	sx933x_fill_situation_func();

	/*
	 * Since sar sensor driver in AP,
	 * it should register information to sensorlist.
	 */
	strncpy(sar_devinfo.name, DRIVER_NAME, sizeof(DRIVER_NAME));
	sensorlist_register_deviceinfo(ID_SAR, &sar_devinfo);

    dev_info(&client->dev, "[SX933x]:sx933x_probe() Done\n");

    return 0;
}

/*! \fn static int sx933x_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx93XX_remove()
 */
//static int __devexit sx933x_remove(struct i2c_client *client)
static int sx933x_remove(struct i2c_client *client)
{
    psx933x_platform_data_t pplatData =0;
    psx933x_t pDevice = 0;
    psx93XX_t this = i2c_get_clientdata(client);
    if (this && (pDevice = this->pDevice))
    {
        input_unregister_device(pDevice->pbuttonInformation->input);

        sysfs_remove_group(&client->dev.kobj, &sx933x_attr_group);
        pplatData = client->dev.platform_data;
        if (pplatData && pplatData->exit_platform_hw)
            pplatData->exit_platform_hw(client);
#ifdef SAR_USE_VFE28_LDO
	if (vfe28_ldo_en) {
            regulator_disable(vfe28_ldo);
            regulator_put(vfe28_ldo);
	    vfe28_ldo = NULL;
            vfe28_ldo_en = false;
        }
#endif
        kfree(this->pDevice);

		cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
        /*destroy_workqueue(this->workq); */
        free_irq(this->irq, this);
        kfree(this);
        return 0;
    }

	return -ENOMEM;
}
#if 0//def CONFIG_PM
/*====================================================*/
/***** Kernel Suspend *****/
static int sx933x_suspend(struct device *dev)
{
    psx93XX_t this = dev_get_drvdata(dev);
    //sx93XX_suspend(this);
    if (this)
        disable_irq(this->irq);
    return 0;
}
/***** Kernel Resume *****/
static int sx933x_resume(struct device *dev)
{
    psx93XX_t this = dev_get_drvdata(dev);
    //sx93XX_resume(this);
    if (this)
    {
        enable_irq(this->irq);
    }
    return 0;
}
/*====================================================*/
#else
#define sx933x_suspend		NULL
#define sx933x_resume		NULL
#endif /* CONFIG_PM */
static struct i2c_device_id sx933x_idtable[] =
{
    { DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sx933x_idtable);
#ifdef CONFIG_OF
static struct of_device_id sx933x_match_table[] =
{
    { .compatible = "Semtech,sx933x",},
    { },
};
#else
#define sx933x_match_table NULL
#endif
static const struct dev_pm_ops sx933x_pm_ops =
{
    .suspend = sx933x_suspend,
    .resume = sx933x_resume,
};

static struct i2c_driver sx933x_driver =
{
    .driver = {
        .owner			= THIS_MODULE,
        .name			= DRIVER_NAME,
        .of_match_table	= sx933x_match_table,
        .pm			= &sx933x_pm_ops,
    },
    .id_table			= sx933x_idtable,
    .probe			= sx933x_probe,
    .remove			= sx933x_remove,
};

static int sx933x_local_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&sx933x_driver);
}

static int sx933x_local_uninit(void)
{
	i2c_del_driver(&sx933x_driver);
	return 0;
}

static struct situation_init_info sx933x_init_info = {
	.name = "semtch_sar",
	.init = sx933x_local_init,
	.uninit = sx933x_local_uninit,
};

static int __init sx933x_init(void)
{
#ifndef CONFIG_MTK_AP_SAR
	pr_info("%s is not ap sar\n", __func__);
	return 0;
#endif
	pr_info("%s\n", __func__);
	situation_driver_add(&sx933x_init_info, ID_SAR);
	return 0;
}
static void __exit sx933x_exit(void)
{
	pr_info("%s\n", __func__);
}

module_init(sx933x_init);
module_exit(sx933x_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX933x Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
