/*
 * aw881xx_cali.c cali_module
 *
 * Version: v0.2.0
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include "aw881xx.h"
#include "aw881xx_reg.h"
#include "aw881xx_cali.h"
#include "aw881xx_monitor.h"

/******************************************************
 *
 * aw881xx cali store
 *
 ******************************************************/

/*write cali to persist file example */
static LIST_HEAD(g_dev_list);
static DEFINE_MUTEX(g_dev_lock);

static bool is_single_cali = false;

static unsigned int g_cali_re_time = AW_CALI_RE_TIME;

#ifndef ABS_VALUE
#define ABS_VALUE(a) (((a) >=0) ? (a) : (-(a)))
#endif
#ifndef ML_THRESHOLD
#define ML_THRESHOLD (168)
#endif
#ifndef LE_COE
#define LE_COE (10)
#endif
#ifndef NS_ITERATION_THRESHOLD
#define NS_ITERATION_THRESHOLD (20)
#endif
#ifndef UNIVERSAL_COE
#define UNIVERSAL_COE (2)
#endif

enum {
	DATA_ACCURACY_24BIT = 16777216,
	DATA_ACCURACY_12BIT = 4096,
	PI_24BIT = 52707178,
	PI2_24BIT = 105414357,
	LN10_24BIT = 38630967,
	ACCURACY_CONST_MAX
};

enum {
	AC_COE_1 = 181,
	AC_COE_2 = 9664,
	AC_COE_3 = 1245892,
	AC_COE_4 = 3558689,
	AC_COE_5 = 26352456,
	AC_COE_MAX
};

enum {
	MECF_CALCU_COE_1 = 4194304,
	MECF_CALCU_COE_2 = 2670176,
	MECF_CALCU_COE_3 = 1000,
	MECF_CALCU_COE_4 = 8,
	MECF_CALCU_COE_MAX
};

typedef struct {
	int64_t data1;
	int64_t data2;
} data_str;

static int aw_ml(uint64_t x, unsigned int data_accuracy, int64_t *log_res)
{
	unsigned int th = ML_THRESHOLD;
	int func_status = 0;
	uint64_t data_accuracy2 = (uint64_t)data_accuracy * data_accuracy;
	int64_t x1 = x - data_accuracy2;
	int64_t x2 = x + data_accuracy2;
	int64_t median_value_1 = data_accuracy2 * x1 / x2;
	int64_t median_value_2 = median_value_1;
	int64_t median_value_3 = 0;
	int64_t s2 = 0;
	unsigned long i = 0;

	median_value_3 = ABS_VALUE(median_value_2 * UNIVERSAL_COE);
	while (median_value_3 > th) {
		s2 = median_value_2 / ((i << 1) + 1) + s2;
		median_value_2 = ((((median_value_1 * median_value_1) / data_accuracy2) * median_value_2) / data_accuracy2);
		i = i + 1;
		median_value_3 = ABS_VALUE(median_value_2 * UNIVERSAL_COE) / ((i << 1) + 1);
	}

	*log_res = s2 * UNIVERSAL_COE;

	return func_status;
}

static int aw_le_v2(uint64_t x, unsigned int data_accuracy, int64_t *log_res)
{
	unsigned int coef = 0;
	int func_status = 0;
	int64_t res = 0;
	uint64_t conditon_1 = (uint64_t)LE_COE * DATA_ACCURACY_12BIT * data_accuracy;
	uint64_t conditon_2 = (uint64_t)DATA_ACCURACY_12BIT * data_accuracy;

	if(x > conditon_1) {
		while (x > conditon_1) {
			coef++;
			x = x / LE_COE;
		}
		aw_ml(x, data_accuracy, &res);
		res += coef * LN10_24BIT;
	} else if (x < conditon_2) {
		while (x < conditon_2) {
			coef++;
			x = x * LE_COE;
		}
		aw_ml(x, data_accuracy, &res);
		res -= coef * LN10_24BIT;
	} else {
		aw_ml(x, data_accuracy, &res);
	}
	*log_res = res;

	return func_status;
}

static int aw_ns_v2(uint64_t x, unsigned int data_accuracy, int64_t *sqrt_res)
{
	int func_status = 0;
	uint64_t median_value_1 = 0;
	uint64_t median_value_2 = x;
	unsigned int i = 0;

	if (x == 0) {
		*sqrt_res = 0;
		return func_status;
	}
	median_value_1 = x;
	for (i = 0; i < NS_ITERATION_THRESHOLD; i++) {
		median_value_2 = (uint64_t)((median_value_2 + median_value_1 * data_accuracy / median_value_2) >> 1);
	}
	*sqrt_res = median_value_2;

	return func_status;
}

static int aw_ac_v2(int64_t x, int64_t *arc_res)
{
	unsigned short x_corros0 = 0;
	int func_status = 0;
	int64_t res = 0;
	int64_t median_value_1 = 0;
	uint64_t median_value_2 = 0;
	int64_t median_value_3 = 0;
	int64_t median_value_4 = 0;

	if (x > DATA_ACCURACY_24BIT) {
		res = x + DATA_ACCURACY_24BIT;
		median_value_1 = x - DATA_ACCURACY_24BIT;
		res = res * median_value_1 / DATA_ACCURACY_24BIT;
		aw_ns_v2(res, DATA_ACCURACY_24BIT, &res);
		res = x - res;
		aw_le_v2(res, DATA_ACCURACY_12BIT, &median_value_4);
		res = -median_value_4;
	} else if (x < -DATA_ACCURACY_24BIT) {
		res = x + DATA_ACCURACY_24BIT;
		median_value_1 = x - DATA_ACCURACY_24BIT;
		res = res * median_value_1 / DATA_ACCURACY_24BIT;
		aw_ns_v2(res, DATA_ACCURACY_24BIT, &res);
		res = x - res;
		res = ABS_VALUE(res);
		aw_le_v2(res, DATA_ACCURACY_12BIT, &median_value_4);
		res = median_value_4;
		median_value_2 = (uint64_t)PI_24BIT * PI_24BIT / DATA_ACCURACY_24BIT;
		res = res * res / DATA_ACCURACY_24BIT + median_value_2;
		aw_ns_v2(res, DATA_ACCURACY_24BIT, &res);
	} else {
		if (x < 0) {
			x_corros0 = 1;
		}
		median_value_2 = ABS_VALUE(x);
		res = - (AC_COE_1 * median_value_2 / AC_COE_2 );
		res = res + AC_COE_3;
		res = res * median_value_2 / DATA_ACCURACY_24BIT - AC_COE_4;
		res = median_value_2 * res;
		res = res / DATA_ACCURACY_24BIT;
		res = res + AC_COE_5;
		aw_ns_v2((DATA_ACCURACY_24BIT - median_value_2), DATA_ACCURACY_24BIT, &median_value_3);
		res = res * median_value_3 / DATA_ACCURACY_24BIT;
		res = res - (x_corros0 << 1) * res;
		res = res + x_corros0 * PI_24BIT;
	}
	*arc_res = res;

	return func_status;
}

int aw_mec_f0_calculate(unsigned int fs, data_str *input, data_str *output)
{
	int func_status = 0;
	int64_t a1 = (input->data1) * DATA_ACCURACY_12BIT;
	int64_t a2 = (input->data2) * DATA_ACCURACY_12BIT;
	int64_t median_value1 = 0;
	int64_t median_value2 = 0;
	int64_t median_value3 = 0;
	int64_t median_value4 = 0;
	int64_t median_value5 = 0;
	int64_t res_1 = 0;
	int64_t res_2 = 0;

	if (a2 <= 0) {
		func_status = -1;
		return func_status;
	}
	aw_ns_v2(a2, DATA_ACCURACY_24BIT, &median_value1);
	median_value1 = median_value1 * UNIVERSAL_COE * (-1);
	median_value1 = ((int64_t)a1) * DATA_ACCURACY_24BIT / median_value1;
	aw_ac_v2(median_value1, &median_value2);
	aw_le_v2(a2, DATA_ACCURACY_12BIT, &median_value3);
	median_value4 = median_value2 * median_value2 / DATA_ACCURACY_24BIT;
	if (median_value1 > DATA_ACCURACY_24BIT) {
		median_value4 = -median_value4;
	}
	median_value5 = median_value4 + median_value3 * median_value3 / DATA_ACCURACY_24BIT * MECF_CALCU_COE_1 / DATA_ACCURACY_24BIT;
	aw_ns_v2(ABS_VALUE(median_value5), DATA_ACCURACY_24BIT, &median_value5);
	res_1 = median_value5 * fs * MECF_CALCU_COE_2 / DATA_ACCURACY_24BIT;
	if (median_value3 == 0) {
		res_2 = 0;
	} else {
		res_2 = -res_1 / median_value3 * PI2_24BIT;
		res_2 = res_2 / fs;
	}
	output->data2 = res_2 * MECF_CALCU_COE_3 / DATA_ACCURACY_24BIT;
	output->data1 = (res_1 / DATA_ACCURACY_24BIT) / MECF_CALCU_COE_4;

	return func_status;
}

#ifdef AW_CALI_STORE_EXAMPLE

#define AWINIC_CALI_FILE  "/mnt/vendor/nvcfg/audio/aw881xx/aw_cali.bin"

static int aw881xx_dev_get_list_head(struct list_head **head)
{
	if (list_empty(&g_dev_list))
		return -EINVAL;

	*head = &g_dev_list;

	return 0;
}

/******************************************************
 *
 * aw881xx cali
 *
 ******************************************************/
static int aw881xx_write_cali_re_to_file(uint32_t cali_re, uint8_t channel)
{
	struct file *fp = NULL;
	char buf[50] = { 0 };
	loff_t pos = 0;
	mm_segment_t fs;

	pos = channel * AW_INT_DEC_DIGIT;

	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			__func__, channel, AWINIC_CALI_FILE);
		return -EINVAL;
	}
	cali_re = ((cali_re * 1000) >> 12);
	snprintf(buf, sizeof(buf), "%10u", cali_re);

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, strlen(buf), &pos);

	set_fs(fs);

	pr_info("%s: channel:%d  buf:%s cali_re:%d\n",
		__func__, channel, buf, cali_re);

	filp_close(fp, NULL);
	return 0;
}

static int aw881xx_get_cali_re_from_file(uint32_t *cali_re, uint8_t channel)
{
	struct file *fp = NULL;
	int f_size;
	char *buf = NULL;
	int32_t int_cali_re = 0;
	loff_t pos = 0;
	mm_segment_t fs;

	pos = channel * AW_INT_DEC_DIGIT;

	fp = filp_open(AWINIC_CALI_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			__func__, channel, AWINIC_CALI_FILE);
		return -EINVAL;
	}

	f_size = AW_INT_DEC_DIGIT;

	buf = kzalloc(f_size + 1, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s: channel:%d malloc mem %d failed!\n",
			__func__, channel, f_size);
		filp_close(fp, NULL);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read(fp, buf, f_size, &pos);

	set_fs(fs);

	if (sscanf(buf, "%d", &int_cali_re) == 1)
		*cali_re = ((int_cali_re << 12) / 1000);
	else
		*cali_re = AW_ERRO_CALI_VALUE;

	pr_info("%s: channel:%d buf:%s int_cali_re: %d\n",
		__func__, channel, buf, int_cali_re);

	kfree(buf);
	buf = NULL;
	filp_close(fp, NULL);

	return 0;

}
#endif


static int aw881xx_get_cali_re_from_phone(struct aw881xx *aw881xx)
{
	/* customer add, get re from nv or persist or cali file */
#ifdef AW_CALI_STORE_EXAMPLE
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;

	return aw881xx_get_cali_re_from_file(&cali_attr->cali_re,
					aw881xx->channel);
#else
	return -EBUSY;
#endif
}

void aw881xx_get_cali_re(struct aw881xx_cali_attr *cali_attr)
{
	int ret;
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);

	ret = aw881xx_get_cali_re_from_phone(aw881xx);
	if (ret < 0) {
		cali_attr->cali_re = AW_ERRO_CALI_VALUE;
		aw_dev_err(aw881xx->dev, "%s: get re failed, use default\n", __func__);
	}
}

static int aw881xx_set_cali_re_to_phone(struct aw881xx *aw881xx)
{
	/* customer add, set re to nv or persist or cali file */
#ifdef AW_CALI_STORE_EXAMPLE
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;

	return aw881xx_write_cali_re_to_file(cali_attr->cali_re,
					aw881xx->channel);
#else
	return -EBUSY;
#endif
}

static int aw881xx_get_ra(struct aw881xx *aw881xx,
				uint16_t *dsp_ra)
{
	int ret;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RA, dsp_ra);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:dsp read dsp ra error\n",
			__func__);
		return ret;
	}

	aw_dev_dbg(aw881xx->dev,"%s: ra=0x%x\n", __func__, *dsp_ra);

	return 0;
}

void aw881xx_set_cali_re_to_dsp(struct aw881xx_cali_attr *cali_attr)
{
	uint16_t cali_re = 0;
	uint16_t dsp_ra = 0;
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);

	aw881xx_get_ra(aw881xx, &dsp_ra);

	cali_re = cali_attr->cali_re + dsp_ra;

	/* set cali re to aw881xx */
	aw881xx_dsp_write(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RE,
			cali_re);

}

static int aw881xx_read_cali_re_from_dsp(struct aw881xx *aw881xx, uint32_t *read_re)
{
	uint16_t dsp_re = 0;
	uint16_t dsp_ra = 0;
	int ret;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RA, &dsp_ra);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:read ra fail\n", __func__);
		return ret;
	}

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RE, &dsp_re);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:read re fail\n", __func__);
		return ret;
	}

	*read_re = dsp_re - dsp_ra;

	return 0;
}

static int aw881xx_set_cali_re(struct aw881xx_cali_attr *cali_attr)
{
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);
	int ret;

	ret = aw881xx_set_cali_re_to_phone(aw881xx);
	if (ret < 0)
		return ret;

	/* set cali re to aw881xx */
	aw881xx_set_cali_re_to_dsp(cali_attr);

	return 0;
}

static int aw881xx_get_ste_re_addr(struct aw881xx *aw881xx, uint16_t *dsp_addr)
{
	int ret = 0;

	switch (aw881xx->pid) {
	case AW881XX_PID_01:
		*dsp_addr = AW88194_DSP_REG_ST_STE_RE;
		break;
	case AW881XX_PID_03:
		*dsp_addr = AW88195_DSP_REG_ST_STE_RE;
		break;
	default:
		ret = -1;
		*dsp_addr = AW88195_DSP_REG_ST_STE_RE;
		break;
	}

	return ret;
}

#if 0
static int aw881xx_get_ste_re(struct aw881xx *aw881xx, uint16_t *dsp_addr, uint16_t *ste_re)
{
	int ret;

	ret = aw881xx_get_ste_re_addr(aw881xx, dsp_addr);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev,
			"%s: get ste_re addr failed\n", __func__);
		return ret;
	}

	ret = aw881xx_dsp_read(aw881xx, *dsp_addr, ste_re);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev,
			"%s: dsp read reg0x%04x error\n", __func__, *dsp_addr);
		return ret;
	}

	return ret;
}
#endif

static ssize_t aw881xx_cali_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->cali_re = databuf[0] * (1 << AW881XX_DSP_RE_SHIFT) / 1000;
	ret = aw881xx_set_cali_re(cali_attr);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t aw881xx_cali_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cali_re=%umohm\n",
			(uint32_t) (cali_attr->cali_re * 1000) /
			(1 << AW881XX_DSP_RE_SHIFT));

	return len;
}

static ssize_t aw881xx_re_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->re = databuf[0];

	return count;
}

static ssize_t aw881xx_re_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"re=%umohm\n", cali_attr->re);

	return len;
}

static ssize_t aw881xx_f0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->f0 = databuf[0];

	return count;
}

static ssize_t aw881xx_f0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "f0=%uHz\n", cali_attr->f0);

	return len;
}

static ssize_t aw881xx_q_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->q = databuf[0];

	return count;
}

static ssize_t aw881xx_q_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "q=%u\n", cali_attr->q);

	return len;
}
/******************************************************
 *
 * aw881xx cali re
 *
 ******************************************************/
int aw881xx_cali_init_check(struct aw881xx *aw881xx)
{
	int ret = -1;

	ret = aw881xx_sysst_check(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:syst_check failed\n",
				__func__);
		return ret;
	}

	ret = aw881xx_get_dsp_status(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:dsp status error\n",
				__func__);
		return ret;
	}

	ret = aw881xx_get_hmute(aw881xx);
	if (ret == 1) {
		aw_dev_err(aw881xx->dev, "%s: pa mute status\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int aw881xx_dev_cali_init(struct aw881xx *aw881xx)
{
	int ret = -1;

	ret = aw881xx_cali_init_check(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev,"%s: cali check fail, error = %d\n",
			__func__, ret);
		return ret;
	}

	ret = aw881xx_read_dsp_pid(aw881xx);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev,"%s: read pid fail, error = %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static void aw_cali_set_cali_status(struct aw881xx *aw881xx, int status)
{
	aw881xx->cali_attr.status = status;

	aw_dev_info(aw881xx->dev, "cali %s",
		(status == 0) ? ("disable") : ("enable"));
}

int aw881xx_run_dsp_mute(struct aw881xx *aw881xx, bool cali_flag, bool mute_en)
{
	int ret;
	uint16_t reg_val = 0;
	uint16_t temp = 0;

	if (!mute_en)
		return 0;
	if (cali_flag) {
		ret = aw881xx_reg_read(aw881xx, AW881XX_REG_DSPCFG, &reg_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:dsp read reg0x%02x error\n",
					__func__, AW881XX_REG_DSPCFG);
			return ret;
		}
		temp = reg_val & (~AW881XX_DSP_VOL_MASK);
		aw881xx->dsp_vol = temp;
		reg_val = ((reg_val & AW881XX_DSP_VOL_MASK) | (~AW881XX_DSP_VOL_MASK));
		ret = aw881xx_reg_write(aw881xx, AW881XX_REG_DSPCFG, reg_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s:dsp read reg0x%02x error\n",
				__func__, AW881XX_REG_DSPCFG);
			return ret;
		}
	} else {
		ret = aw881xx_reg_read(aw881xx, AW881XX_REG_DSPCFG, &reg_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%02x error\n",
				__func__, AW881XX_REG_DSPCFG);
			return ret;
		}
		temp = reg_val & AW881XX_DSP_VOL_MASK;
		reg_val = aw881xx->dsp_vol | temp;
		ret = aw881xx_reg_write(aw881xx, AW881XX_REG_DSPCFG, reg_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%02x error\n",
				__func__, AW881XX_REG_DSPCFG);
			return ret;
		}
	}

	return 0;
}

static int aw881xx_cali_init_cfg(struct aw881xx *aw881xx, bool flag)
{
	int ret = -1;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;

	/* cfg mbmec actampth = 0*/
	reg_addr = AW881XX_DSP_REG_CFG_MBMEC_ACTAMPTH;
	ret = aw881xx_dsp_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%02x error\n",
			__func__, reg_addr);
		return ret;
	}

	if (flag) {
		aw881xx->dft_mbmec_actampth = reg_val;
		reg_val = 0;
	} else {
		reg_val = aw881xx->dft_mbmec_actampth;
	}
	ret = aw881xx_dsp_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp write reg0x%02x error\n",
			__func__, reg_addr);
		return ret;
	}

	/* cfg mbmec noiseampth = 0*/
	reg_addr = AW881XX_DSP_REG_CFG_MBMEC_NOISEAMPTH;
	ret = aw881xx_dsp_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%02x error\n",
			__func__, reg_addr);
		return ret;
	}
	if (flag) {
		aw881xx->dft_mbmec_noiseampth = reg_val;
		reg_val = 0;
	} else {
		reg_val = aw881xx->dft_mbmec_noiseampth;
	}
	ret = aw881xx_dsp_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp write reg0x%02x error\n",
			__func__, reg_addr);
		return ret;
	}

	/* cfg adpz ustepn = 1*/
	reg_addr = AW881XX_DSP_REG_CFG_ADPZ_USTEPN;
	ret = aw881xx_dsp_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%02x error\n",
			__func__, reg_addr);
		return ret;
	}
	if (flag) {
		aw881xx->dft_adpz_ustepn = reg_val;
		reg_val = (uint16_t)(-1);
	} else {
		reg_val = aw881xx->dft_adpz_ustepn;
	}
	ret = aw881xx_dsp_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp write reg0x%02x error\n",
			__func__, reg_addr);
		return ret;
	}

	/* cfg g_dft_re_alphan = 1*/
	reg_addr = AW881XX_DSP_REG_CFG_RE_ALPHAN;
	ret = aw881xx_dsp_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:dsp read reg0x%02x error\n", __func__, reg_addr);
		return ret;
	}
	if (flag) {
		aw881xx->dft_re_alphan = reg_val;
		reg_val = (uint16_t)(1);
	} else {
		reg_val = aw881xx->dft_re_alphan;
	}
	ret = aw881xx_dsp_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp write reg0x%02x error\n", __func__, reg_addr);
		return ret;
	}

	return 0;
}

static int aw881xx_cali_mode_en(struct aw881xx *aw881xx, bool cali_flag, bool mute_en)
{
	int ret = -1;

	if (cali_flag) {
		ret = aw881xx_dev_cali_init(aw881xx);
		if (ret < 0)
			return ret;

		aw_cali_set_cali_status(aw881xx, true);

		ret = aw881xx_run_dsp_mute(aw881xx, true, mute_en);
		if (ret < 0)
			goto run_dsp_mute_failed;

		ret = aw881xx_cali_init_cfg(aw881xx, true);
		if (ret)
			goto cali_init_cfg_failed;
	} else {
		aw881xx_cali_init_cfg(aw881xx, false);
		aw881xx_run_dsp_mute(aw881xx, false, mute_en);
		aw_cali_set_cali_status(aw881xx, false);
	}

	return 0;

cali_init_cfg_failed:
	aw881xx_run_dsp_mute(aw881xx, false, mute_en);
run_dsp_mute_failed:
	aw_cali_set_cali_status(aw881xx, false);

	return ret;
}

static void aw881xx_bubble_sort(struct aw881xx *aw881xx, uint32_t *data, int data_size)
{
	int loop_num = data_size - 1;
	uint32_t temp_store = 0;
	int i;
	int j;

	if (data == NULL) {
		aw_dev_err(aw881xx->dev, "%s: data is NULL\n", __func__);
		return;
	}

	for (i = 0; i < loop_num; i++) {
		for (j = 0; j < loop_num - i; j++) {
			if (data[j] > data[j + 1]) {
				temp_store = data[j];
				data[j] = data[j + 1];
				data[j + 1] = temp_store;
			}
		}
	}
}

static int aw881xx_cali_get_iv(struct aw881xx *aw881xx, uint8_t *iv_flag)
{
	int ret = -1;
	uint16_t dsp_addr = 0;
	uint16_t dsp_val = 0;
	uint32_t iabs_val = 0;
	uint32_t iabs_temp[CALI_READ_CNT_MAX] = { 0 };
	uint32_t iabs_sum = 0;
	uint16_t iabs_average;
	uint8_t cnt = 0;

	if (aw881xx->pid != AW881XX_PID_03) {
		*iv_flag = 1;
		return 0;
	}

	for (cnt = 0; cnt < CALI_READ_CNT_MAX; cnt++) {
		dsp_addr = AW88195_DSP_REG_ST_STE_IABS;
		ret = aw881xx_dsp_read(aw881xx, dsp_addr, &dsp_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%04x error\n",
				__func__, dsp_addr);
			return ret;
		}
		iabs_val = dsp_val;
		iabs_temp[cnt] = iabs_val;
		msleep(30);	/* delay 30 ms*/
	}

	/*sort read re value*/
	aw881xx_bubble_sort(aw881xx, iabs_temp, CALI_READ_CNT_MAX);

	/*delete two min value and two max value,compute mid value*/
	for (cnt = 1; cnt < CALI_READ_CNT_MAX - 1; cnt++)
		iabs_sum += iabs_temp[cnt];

	iabs_average = (uint16_t)(iabs_sum / (CALI_READ_CNT_MAX - CALI_DATA_SUM_RM));
	if (iabs_average < 32)
		*iv_flag = 0;
	else
		*iv_flag = 1;

	aw_dev_dbg(aw881xx->dev,"%s: iv_flag=%d\n", __func__, *iv_flag);

	return 0;
}

static int aw881xx_cali_get_re(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint16_t dsp_addr = 0;
	uint16_t dsp_val = 0;
	uint32_t re_val = 0;
	uint32_t re_temp[CALI_READ_CNT_MAX] = { 0 };
	uint32_t re_sum = 0;
	uint16_t re_average = 0;
	uint16_t ra_val = 0;
	uint8_t iv_flag = 0;
	uint8_t cnt = 0;

	for (cnt = 0; cnt < CALI_READ_CNT_MAX; cnt++) {
		ret = aw881xx_get_ste_re_addr(aw881xx, &dsp_addr);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: get ste re addr error\n", __func__);
			return ret;
		}
		ret = aw881xx_dsp_read(aw881xx, dsp_addr, &dsp_val);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%04x error\n", __func__, dsp_addr);
			return ret;
		}
		re_val = dsp_val;
		re_temp[cnt] = re_val;

		msleep(30);
	}

	/*sort read re value*/
	aw881xx_bubble_sort(aw881xx, re_temp, CALI_READ_CNT_MAX);

	/*delete two min value and two max value,compute mid value*/
	for (cnt = 1; cnt < CALI_READ_CNT_MAX - 1; cnt++)
		re_sum += re_temp[cnt];

	re_average = (uint16_t)(re_sum / (CALI_READ_CNT_MAX - CALI_DATA_SUM_RM));

	aw_dev_dbg(aw881xx->dev,"%s: re_average = 0x%x\n", __func__, re_average);

	ret = aw881xx_get_ra(aw881xx, &ra_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp read ra error\n",
			__func__);
		return ret;
	}

	aw881xx->cali_attr.re = AW_DSP_RE_TO_SHOW_RE(re_average - ra_val);

	aw881xx_cali_get_iv(aw881xx, &iv_flag);
	if (!iv_flag)
		aw881xx->cali_attr.re = 0x7fff;

	aw_dev_info(aw881xx->dev, "%s: re = %d mohm",
			__func__, aw881xx->cali_attr.re);

	return 0;
}

static int aw881xx_cali_single_dev_re(struct aw881xx *aw881xx)
{
	int ret = -1;
	bool mute_en = true;

	ret = aw881xx_cali_mode_en(aw881xx, true, mute_en);
	if (ret < 0)
		return ret;

	msleep(g_cali_re_time);

	aw881xx_cali_get_re(aw881xx);

	ret = aw881xx_cali_mode_en(aw881xx, false, mute_en);

	return ret;

}

static int aw881xx_cali_mult_dev_re(struct aw881xx *aw881xx)
{
	struct list_head *dev_list = NULL;
	struct list_head *pos = NULL;
	struct aw881xx *local_aw881xx = NULL;
	int ret = -1;
	bool mute_en = true;

	ret = aw881xx_dev_get_list_head(&dev_list);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "get dev list failed");
		return ret;
	}

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		ret = aw881xx_cali_mode_en(local_aw881xx, true, mute_en);
		if (ret < 0)
			goto error;
	}

	msleep(g_cali_re_time);

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		aw881xx_cali_get_re(local_aw881xx);
		ret = aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
		if (ret < 0)
			aw_dev_err(aw881xx->dev, "%s: back cali mode failed", __func__);
	}

	return ret;

error:
	if (pos->prev == dev_list)
		return ret;

	for(pos = pos->prev; pos != dev_list; pos= pos->prev) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
	}

	return ret;
}

static int aw881xx_dev_cali_re(struct aw881xx *aw881xx, bool is_single)
{
	if (is_single)
		return aw881xx_cali_single_dev_re(aw881xx);
	else
		return aw881xx_cali_mult_dev_re(aw881xx);

}

static ssize_t aw881xx_cali_re(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	int ret = -1;

	ret = aw881xx_dev_cali_re(aw881xx, is_single_cali);
	if (ret < 0)
		return ret;

	return count;
}

/******************************************************
 *
 * aw881xx cali re show
 *
 ******************************************************/
static int aw881xx_cali_get_devs_cali_re(struct aw881xx *aw881xx, int32_t *re_buf, int num)
{
	struct list_head *dev_list;
	struct list_head *pos = NULL;
	struct aw881xx *local_aw881xx;
	int ret, cnt = 0;

	//get dev list
	ret = aw881xx_dev_get_list_head(&dev_list);
	if (ret) {
		aw_dev_err(aw881xx->dev, "get dev list failed");
		return ret;
	}

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		if (local_aw881xx->channel < num) {
			re_buf[local_aw881xx->channel] = local_aw881xx->cali_attr.re;
			aw_dev_info(aw881xx->dev, "%s: dev[%d]read re value is %d", __func__,
				local_aw881xx->channel, local_aw881xx->cali_attr.re);
			cnt++;
		} else {
			aw_dev_err(aw881xx->dev, "channel num[%d] overflow buf num[%d]",
						local_aw881xx->channel, num);
			return -EINVAL;
		}
	}

	return cnt;
}

static ssize_t aw881xx_cali_re_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret, i;
	struct aw881xx *aw881xx = dev_get_drvdata(dev);

	ssize_t len = 0;
	int32_t cali_re[AW_DEV_CH_MAX] = { 0 };

	if (is_single_cali) {
		len += snprintf(buf+len, PAGE_SIZE-len, "%d mOhms\n", aw881xx->cali_attr.re);
	} else {
		ret = aw881xx_cali_get_devs_cali_re(aw881xx, cali_re, AW_DEV_CH_MAX);
		if (ret <= 0) {
			aw_dev_err(aw881xx->dev, "%s:get re failed\n", __func__);
		} else {
			for (i = 0; i < ret; i++)
				len += snprintf(buf+len, PAGE_SIZE-len, "dev[%d]:%d mOhms ", i, cali_re[i]);

			len += snprintf(buf+len, PAGE_SIZE-len, " \n");
		}
	}

	return len;
}

/******************************************************
 *
 * aw881xx cali f0
 *
 ******************************************************/
static int aw881xx_cali_vol_cfg(struct aw881xx *aw881xx, bool flag)
{
	int ret = -1;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;

	reg_addr = AW881XX_REG_DSPCFG;
	ret = aw881xx_reg_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%02x error\n", __func__, reg_addr);
		return ret;
	}
	if (flag) {
		aw881xx->dft_dsp_cfg = reg_val;
		reg_val &= AW881XX_DSP_VOL_MASK;
		reg_val |= AW881XX_DSP_VOL_NOISE_ST;
	} else {
		reg_val = aw881xx->dft_dsp_cfg;
	}

	ret = aw881xx_reg_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c rwrite reg0x%02x error\n", __func__, reg_addr);
		return ret;
	}
	return 0;
}

static int aw881xx_cali_white_noise(struct aw881xx *aw881xx, bool flag)
{
	int ret = -1;
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;

	/* cfg mbmec glbcfg.bit4*/
	reg_addr = AW881XX_DSP_REG_CFG_MBMEC_GLBCFG;
	ret = aw881xx_dsp_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp read reg0x%02x error\n", __func__, reg_addr);
		return ret;
	}
	if (true == flag)
		reg_val |= (~AW881XX_DSP_REG_CFG_NOISE_MASK);
	else
		reg_val &= AW881XX_DSP_REG_CFG_NOISE_MASK;

	ret = aw881xx_dsp_write(aw881xx, reg_addr, reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: dsp write reg0x%02x error\n", __func__, reg_addr);
		return ret;
	}

	return 0;
}

static int aw881xx_cali_f0_q_cfg(struct aw881xx *aw881xx, bool flag)
{
	int ret = 0;

	if (flag) {
		ret = aw881xx_cali_vol_cfg(aw881xx, true);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s: dsp cali vol cfg error, ret=%d", __func__, ret);
			return ret;
		}

		ret = aw881xx_cali_white_noise(aw881xx, true);
		if (ret < 0) {
			aw_dev_err(aw881xx->dev, "%s：dsp set noise error, ret = %d", __func__, ret);
			aw881xx_cali_vol_cfg(aw881xx, false);
			return ret;
		}
	} else {
		ret = aw881xx_cali_white_noise(aw881xx, false);
		if (ret < 0)
			aw_dev_err(aw881xx->dev, "%s: dsp restore noise error, ret=%d", __func__, ret);

		ret = aw881xx_cali_vol_cfg(aw881xx, false);
		if (ret < 0)
			aw_dev_err(aw881xx->dev, "%s: dsp restore vol cfg error, ret=%d", __func__, ret);
	}

	return 0;
}

int aw881xx_get_fs(struct aw881xx *aw881xx)
{
	int ret = -1;
	uint8_t reg_addr = 0;
	uint16_t reg_val = 0;
	uint32_t aw881xx_fs[AW881XX_FS_CFG_MAX] = {
		8000,
		11000,
		12000,
		16000,
		22000,
		24000,
		32000,
		44100,
		48000,
		96000,
		192000,
	};

	reg_addr = AW881XX_REG_I2SCTRL;
	ret = aw881xx_reg_read(aw881xx, reg_addr, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s: i2c read reg0x%02x error", __func__, reg_addr);
		return ret;
	}

	aw881xx->fs = aw881xx_fs[reg_val&(~AW881XX_BIT_I2SCTRL_SR_MASK)];

	aw_dev_dbg(aw881xx->dev, "%s: fs=%d", __func__, aw881xx->fs);

	return 0;
}

int aw881xx_cali_get_f0(struct aw881xx *aw881xx)
{
	int ret;
	uint16_t reg_addr;
	uint16_t reg_val = 0;
	uint16_t temp_val;
	int16_t f0_a1_val;
	int16_t f0_a2_val;
	uint32_t f0_sum = 0;
	uint32_t q_sum = 0;
	uint8_t cnt;
	uint8_t iv_flag = 0;
	data_str aw_input;
	data_str aw_outout;

	ret = aw881xx_get_fs(aw881xx);
	if (ret)
		return ret;

	for (cnt = 0; cnt < F0_Q_READ_CNT_MAX; cnt++) {
		/* get f0 a1 */
		temp_val = 0;
		reg_addr = AW881XX_REG_ASR4;
		ret = aw881xx_reg_read(aw881xx, reg_addr, &reg_val);
		if (ret < 0)
			return ret;
		temp_val = reg_val;
		f0_a1_val = (int16_t)temp_val;

		/* get f0 a2 */
		temp_val = 0;
		reg_addr = AW881XX_REG_ASR3;
		ret = aw881xx_reg_read(aw881xx, reg_addr, &reg_val);
		if (ret < 0)
			return ret;
		temp_val = reg_val;
		f0_a2_val = (int16_t)temp_val;

		aw_dev_dbg(aw881xx->dev, "%s: cnt[%d], f0_a1_val = %d, f0_a2_val = %d, fs = %d\n",
				__func__, cnt, f0_a1_val, f0_a2_val, aw881xx->fs);

		/* f0 q compute */
		aw_input.data1 = f0_a1_val;
		aw_input.data2 = f0_a2_val;
		aw_outout.data1 = 0;
		aw_outout.data2 = 0;
		ret = aw_mec_f0_calculate(aw881xx->fs, &aw_input, &aw_outout);
		f0_sum += aw_outout.data1;
		q_sum += aw_outout.data2;
		msleep(30); /*delay 30 ms*/
	}

	aw881xx->cali_attr.f0 = f0_sum / cnt;
	aw881xx->cali_attr.q = q_sum / cnt;

	aw881xx_cali_get_iv(aw881xx, &iv_flag);
	if (!iv_flag) {
		aw881xx->cali_attr.f0 = 0;
		aw881xx->cali_attr.q = 0;
	}

	return 0;
}

static int aw881xx_cali_single_dev_f0(struct aw881xx *aw881xx)
{
	int ret = -1;
	bool mute_en = false;

	ret = aw881xx_cali_mode_en(aw881xx, true, mute_en);
	if (ret < 0)
		return ret;
	ret = aw881xx_cali_f0_q_cfg(aw881xx, true);
	if (ret < 0)
		goto f0_cfg_failed;

	msleep(5000);

	ret = aw881xx_cali_get_f0(aw881xx);

	aw881xx_cali_f0_q_cfg(aw881xx, false);

	aw881xx_cali_mode_en(aw881xx, false, mute_en);

	return ret;

f0_cfg_failed:
	aw881xx_cali_mode_en(aw881xx, false, mute_en);
	return ret;
}

static int aw881xx_cali_mult_dev_f0(struct aw881xx *aw881xx)
{
	struct list_head *dev_list;
	struct list_head *pos;
	struct aw881xx *local_aw881xx;
	int ret = -1;
	bool mute_en = false;

	ret = aw881xx_dev_get_list_head(&dev_list);
	if (ret) {
		aw_dev_err(aw881xx->dev, "get dev list failed\n");
		return ret;
	}

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		ret = aw881xx_cali_mode_en(local_aw881xx, true, mute_en);
		if (ret < 0)
			goto cali_mode_failed;
		ret = aw881xx_cali_f0_q_cfg(local_aw881xx, true);
		if (ret < 0) {
			aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
			goto f0_cfg_failed;
		}
	}

	msleep(5000);

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		aw881xx_cali_get_f0(local_aw881xx);
		aw881xx_cali_f0_q_cfg(local_aw881xx, false);

		aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
	}
	return 0;

f0_cfg_failed:
cali_mode_failed:
	if (pos->prev == dev_list)
		return ret;

	for(pos = pos->prev; pos != dev_list; pos= pos->prev) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		aw881xx_cali_f0_q_cfg(local_aw881xx, false);
		aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
	}

	return ret;
}

static int aw881xx_dev_cali_f0(struct aw881xx *aw881xx, bool is_single)
{
	if (is_single)
		return aw881xx_cali_single_dev_f0(aw881xx);
	else
		return aw881xx_cali_mult_dev_f0(aw881xx);

}

static ssize_t aw881xx_cali_f0(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	int ret = -1;

	ret = aw881xx_dev_cali_f0(aw881xx, is_single_cali);
	if (ret)
		return ret;

	return count;
}

static int aw881xx_cali_get_devs_cali_f0(struct aw881xx *aw881xx, int32_t *f0_buf, int num)
{
	struct list_head *dev_list;
	struct list_head *pos = NULL;
	struct aw881xx *local_aw881xx;
	int ret, cnt = 0;

	//get dev list
	ret = aw881xx_dev_get_list_head(&dev_list);
	if (ret) {
		aw_dev_err(aw881xx->dev, "%s:get dev list failed\n", __func__);
		return ret;
	}

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		if (local_aw881xx->channel < num) {
			f0_buf[local_aw881xx->channel] = local_aw881xx->cali_attr.f0;
			aw_dev_info(aw881xx->dev, "%s: dev[%d]read f0 value is %d\n", __func__,
				local_aw881xx->channel, local_aw881xx->cali_attr.f0);
			cnt++;
		} else {
			aw_dev_err(aw881xx->dev, "%s: channel num[%d] overflow buf num[%d]\n", __func__,
						local_aw881xx->channel, num);
			return -EINVAL;
		}
	}
	return cnt;
}


/******************************************************
 *
 * aw881xx cali f0 show
 *
 ******************************************************/
static ssize_t aw881xx_cali_f0_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret, i;
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int32_t cali_f0[AW_DEV_CH_MAX] = { 0 };

	if (is_single_cali) {
		len += snprintf(buf+len, PAGE_SIZE-len, "f0: %d\n", aw881xx->cali_attr.f0);
	} else {
		ret = aw881xx_cali_get_devs_cali_f0(aw881xx, cali_f0, AW_DEV_CH_MAX);
		if (ret <= 0) {
			aw_dev_err(aw881xx->dev, "get re failed\n");
		} else {
			for (i = 0; i < ret; i++)
				len += snprintf(buf+len, PAGE_SIZE-len, "dev[%d]_f0:%d\n", i, cali_f0[i]);
		}
	}

	return len;
}
/******************************************************
 *
 * aw881xx cali q
 *
 ******************************************************/
int aw881xx_cali_get_q(struct aw881xx *aw881xx)
{
	int ret;
	uint16_t reg_addr;
	uint16_t reg_val = 0;
	uint16_t temp_val;
	int16_t f0_a1_val;
	int16_t f0_a2_val;
	uint32_t q_sum = 0;
	uint8_t cnt;
	uint8_t iv_flag = 0;
	data_str aw_input;
	data_str aw_outout;

	ret = aw881xx_get_fs(aw881xx);
	if (ret)
		return ret;

	for (cnt = 0; cnt < F0_Q_READ_CNT_MAX; cnt++) {
		/* get f0 a1 */
		temp_val = 0;
		reg_addr = AW881XX_REG_ASR4;
		ret = aw881xx_reg_read(aw881xx, reg_addr, &reg_val);
		if (ret < 0)
			return ret;
		temp_val = reg_val;
		f0_a1_val = (int16_t)temp_val;

		/* get f0 a2 */
		temp_val = 0;
		reg_addr = AW881XX_REG_ASR3;
		ret = aw881xx_reg_read(aw881xx, reg_addr, &reg_val);
		if (ret < 0)
			return ret;
		temp_val = reg_val;
		f0_a2_val = (int16_t)temp_val;

		aw_dev_dbg(aw881xx->dev, "%s: cnt[%d], f0_a1_val = %d, f0_a2_val = %d, fs = %d\n",
				__func__, cnt, f0_a1_val, f0_a2_val, aw881xx->fs);

		/* f0 q compute */
		aw_input.data1 = f0_a1_val;
		aw_input.data2 = f0_a2_val;
		aw_outout.data1 = 0;
		aw_outout.data2 = 0;
		ret = aw_mec_f0_calculate(aw881xx->fs, &aw_input, &aw_outout);
		q_sum += aw_outout.data2;
		msleep(30); // delay 30 ms
	}

	aw881xx->cali_attr.q = q_sum / cnt;

	aw881xx_cali_get_iv(aw881xx, &iv_flag);
	if (!iv_flag)
		aw881xx->cali_attr.q = 0;

	return 0;
}

static int aw881xx_cali_single_dev_q(struct aw881xx *aw881xx)
{
	int ret = -1;
	bool mute_en = false;

	ret = aw881xx_cali_mode_en(aw881xx, true, mute_en);
	if (ret < 0)
		return ret;
	ret = aw881xx_cali_f0_q_cfg(aw881xx, true);
	if (ret < 0)
		goto f0_cfg_failed;

	msleep(5000);

	ret = aw881xx_cali_get_q(aw881xx);

	aw881xx_cali_f0_q_cfg(aw881xx, false);

	aw881xx_cali_mode_en(aw881xx, false, mute_en);

	return 0;

f0_cfg_failed:
	aw881xx_cali_mode_en(aw881xx, false, mute_en);
	return ret;
}

static int aw881xx_cali_mult_dev_q(struct aw881xx *aw881xx)
{
	struct list_head *dev_list;
	struct list_head *pos;
	struct aw881xx *local_aw881xx;
	int ret = -1;
	bool mute_en = false;

	ret = aw881xx_dev_get_list_head(&dev_list);
	if (ret) {
		aw_dev_err(aw881xx->dev, "%s: get dev list failed\n", __func__);
		return ret;
	}

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		ret = aw881xx_cali_mode_en(local_aw881xx, true, mute_en);
		if (ret < 0)
			goto cali_mode_failed;
		ret = aw881xx_cali_f0_q_cfg(local_aw881xx, true);
		if (ret < 0) {
			aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
			goto q_cfg_failed;
		}
	}

	msleep(5000);

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		aw881xx_cali_get_q(local_aw881xx);
		aw881xx_cali_f0_q_cfg(local_aw881xx, false);
		aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
	}

	return 0;

cali_mode_failed:
q_cfg_failed:
	if (pos->prev == dev_list)
		return ret;

	for(pos = pos->prev; pos != dev_list; pos = pos->prev) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		aw881xx_cali_f0_q_cfg(local_aw881xx, false);
		aw881xx_cali_mode_en(local_aw881xx, false, mute_en);
	}

	return ret;
}

static int aw881xx_dev_cali_q(struct aw881xx *aw881xx, bool is_single)
{
	if (is_single)
		return aw881xx_cali_single_dev_q(aw881xx);
	else
		return aw881xx_cali_mult_dev_q(aw881xx);
}

static ssize_t aw881xx_cali_q(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	int ret = -1;

	ret = aw881xx_dev_cali_q(aw881xx, is_single_cali);
	if (ret)
		return ret;

	return count;
}
/******************************************************
 *
 * aw881xx cali q show
 *
 ******************************************************/
static int aw881xx_cali_get_devs_cali_q(struct aw881xx *aw881xx, int32_t *q_buf, int num)
{
	struct list_head *dev_list;
	struct list_head *pos = NULL;
	struct aw881xx *local_aw881xx;
	int ret, cnt = 0;

	/*get dev list*/
	ret = aw881xx_dev_get_list_head(&dev_list);
	if (ret) {
		aw_dev_err(aw881xx->dev, "%s: get dev list failed\n", __func__);
		return ret;
	}

	list_for_each (pos, dev_list) {
		local_aw881xx = container_of(pos, struct aw881xx, cali_list);
		if (local_aw881xx->channel < num) {
			q_buf[local_aw881xx->channel] = local_aw881xx->cali_attr.q;
			aw_dev_info(aw881xx->dev, "%s: dev[%d]read q value is %d\n", __func__,
				local_aw881xx->channel, local_aw881xx->cali_attr.q);
			cnt++;
		} else {
			aw_dev_err(aw881xx->dev, "%s:channel num[%d] overflow buf num[%d]\n",
						__func__, local_aw881xx->channel, num);
			return -EINVAL;
		}
	}

	return cnt;
}

static ssize_t aw881xx_cali_q_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, i;
	struct aw881xx *aw881xx = dev_get_drvdata(dev);

	ssize_t len = 0;
	int32_t cali_q[AW_DEV_CH_MAX] = {0};

	if (is_single_cali) {
		len += snprintf(buf+len, PAGE_SIZE-len, "q: %d\n", aw881xx->cali_attr.q);
	} else {
		ret = aw881xx_cali_get_devs_cali_q(aw881xx, cali_q, AW_DEV_CH_MAX);
		if (ret <= 0) {
			aw_dev_err(aw881xx->dev, "%s:get q failed\n", __func__);
		} else {
			for (i = 0; i < ret; i++)
				len += snprintf(buf+len, PAGE_SIZE-len, "dev[%d]_q:%d\n", i, cali_q[i]);
		}
	}

	return len;
}

/******************************************************
 *
 * aw881xx cali time
 *
 ******************************************************/
static ssize_t aw881xx_cali_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint32_t time;
	struct aw881xx *aw881xx = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 0, &time);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:read buf %s failed\n", buf, __func__);
		return ret;
	}

	if (time < 1000) {
		aw_dev_err(aw881xx->dev, "%s:time:%d is too short, no set\n",__func__, time);
		return -EINVAL;
	}

	g_cali_re_time = time;
	aw_dev_dbg(aw881xx->dev, "%s:time:%u\n", __func__, time);

	return count;
}

static ssize_t aw881xx_cali_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"time: %u\n", g_cali_re_time);

	return len;
}

static ssize_t aw881xx_dsp_re_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int ret = 0;
	uint32_t read_re;

	aw881xx_get_dsp_config(aw881xx);
	if (aw881xx->dsp_cfg == AW881XX_DSP_BYPASS) {
		len += snprintf((char *)(buf + len), PAGE_SIZE - len,
				"%s: aw881xx dsp bypass\n", __func__);
		return len;
	}

	ret = aw881xx_get_iis_status(aw881xx);
	if (ret < 0) {
		len += snprintf((char *)(buf + len),
				PAGE_SIZE - len,
				"%s: aw881xx no iis signal\n",
				__func__);
		return len;
	}

	ret = aw881xx_read_cali_re_from_dsp(aw881xx, &read_re);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:read dsp re fail\n", __func__);
		return ret;
	}

	len += snprintf((char *)(buf + len),
		PAGE_SIZE - len,
		"dsp_re: %d\n", ((read_re * 1000) >> 12));

	return len;
}

static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO,
			aw881xx_cali_show, aw881xx_cali_store);
static DEVICE_ATTR(re, S_IWUSR | S_IRUGO,
			aw881xx_re_show, aw881xx_re_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO,
			aw881xx_f0_show, aw881xx_f0_store);
static DEVICE_ATTR(q, S_IWUSR | S_IRUGO,
			aw881xx_q_show, aw881xx_q_store);
static DEVICE_ATTR(cali_re, S_IWUSR | S_IRUGO,
			aw881xx_cali_re_show, aw881xx_cali_re);
static DEVICE_ATTR(cali_f0, S_IWUSR | S_IRUGO,
			aw881xx_cali_f0_show, aw881xx_cali_f0);
static DEVICE_ATTR(cali_q, S_IWUSR | S_IRUGO,
			aw881xx_cali_q_show, aw881xx_cali_q);
static DEVICE_ATTR(cali_time, S_IWUSR | S_IRUGO,
			aw881xx_cali_time_show, aw881xx_cali_time_store);
static DEVICE_ATTR(dsp_re, S_IRUGO,
			aw881xx_dsp_re_show, NULL);


static struct attribute *aw881xx_cali_attr[] = {
	&dev_attr_cali.attr,
	&dev_attr_re.attr,
	&dev_attr_f0.attr,
	&dev_attr_q.attr,
	&dev_attr_cali_re.attr,
	&dev_attr_cali_f0.attr,
	&dev_attr_cali_q.attr,
	&dev_attr_cali_time.attr,
	&dev_attr_dsp_re.attr,
	NULL
};

static struct attribute_group aw881xx_cali_attr_group = {
	.attrs = aw881xx_cali_attr
};

void aw881xx_cali_init(struct aw881xx_cali_attr *cali_attr)
{
	int ret = -1;
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);

	INIT_LIST_HEAD(&aw881xx->cali_list);
	
	ret = sysfs_create_group(&aw881xx->dev->kobj, &aw881xx_cali_attr_group);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev,
			"%s error creating sysfs attr files\n", __func__);
	}
	
	mutex_lock(&g_dev_lock);
	list_add(&aw881xx->cali_list, &g_dev_list);
	mutex_unlock(&g_dev_lock);
	
}
