#ifndef __AW881XX_CALI_H__
#define __AW881XX_CALI_H__

#define AW_CALI_STORE_EXAMPLE

#define AW_ERRO_CALI_VALUE (0)

#define AW_INT_DEC_DIGIT	(10)
#define CALI_READ_CNT_MAX	(8)
#define CALI_DATA_SUM_RM	(2)
#define AW881XX_FS_CFG_MAX	(11)
#define F0_Q_READ_CNT_MAX	(5)
#define AW_DEV_CH_MAX		(16)
#define AW_DSP_RE_TO_SHOW_RE(re)	(((re) * 1000) / 4096)
#define AW_CALI_RE_TIME		(3000)

struct aw881xx_cali_attr {
	uint32_t cali_re;
	uint32_t re;
	uint32_t f0;
	uint32_t q;
	int status;
};

void aw881xx_set_cali_re_to_dsp(struct aw881xx_cali_attr *cali_attr);
void aw881xx_get_cali_re(struct aw881xx_cali_attr *cali_attr);
void aw881xx_cali_init(struct aw881xx_cali_attr *cali_attr);

#endif
