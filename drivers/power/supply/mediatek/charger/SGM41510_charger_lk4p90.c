/*
 * SGM SGM41510 charger driver by Stuart Su
 *
 * Copyright (C) 2020 SGMicro Corporation
 *
 * This driver is for Linux kernel 4.9
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>

#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "mtk_charger_intf.h"

#define SGM41510_MANUFACTURER			"SG Micro"
#define SGM41510_IRQ_PIN				"sgm41510_irq"

#define SGM41510_ID						0

enum sgm41510_fields {
	/* Reg00 */
	F_EN_HIZ = 0,
	F_EN_ILIM,
	F_IILIM,

	/* Reg01 */	//reserved for 41510
	F_BHOT,
	F_BCOLD,
	F_VINDPM_OFS,

	/* Reg02 */
	F_CONV_START,
	F_CONV_RATE, //reserved for 41510
	F_BOOSTF,
	F_ICO_EN, //reserved for 41510
	F_HVDCP_EN,  //reserved for 41510
	F_MAXC_EN,  //reserved for 41510
	F_FORCE_DPM,
	F_AUTO_DPDM_EN,

	/* Reg03 */
	F_BAT_LOAD_EN,
	F_WD_RST,
	F_OTG_CFG,
	F_CHG_CFG,
	F_SYSVMIN,

	/* Reg04 */
	F_PUMPX_EN,
	F_ICHG,

	/* Reg05 */
	F_IPRECHG,
	F_ITERM,

	/* Reg06 */
	F_VREG,
	F_BATLOWV,
	F_VRECHG,

	/* Reg07 */
	F_TERM_EN,
	F_STAT_DIS,
	F_WD,
	F_TMR_EN,
	F_CHG_TMR,
	F_JEITA_ISET,  //reserved for 41510

	/* Reg08 */
	F_BATCMP,
	F_VCLAMP,
	F_TREG,

	/* Reg09 */
	F_FORCE_ICO,  //reserved for 41510
	F_TMR2X_EN,
	F_BATFET_DIS,
	F_JEITA_VSET,  //reserved for 41510
	F_BATFET_DLY,
	F_BATFET_RST_EN,
	F_PUMPX_UP,
	F_PUMPX_DN,

	/* Reg0A */
	F_BOOSTV,
	F_BOOSTI,

	/* Reg0B */
	F_VBUS_STAT,
	F_CHG_STAT,
	F_PG_STAT,
	F_SDP_STAT,   //reserved for 41510
	F_VSYS_STAT,

	/* Reg0C */
	F_WD_FAULT,
	F_BOOST_FAULT,
	F_CHG_FAULT,
	F_BAT_FAULT,
	F_NTC_FAULT,   //reserved for 41510

	/* Reg0D */
	F_FORCE_VINDPM,
	F_VINDPM,

	/* Reg0E */		//reserved for 41510
	F_THERM_STAT,
	F_BATV,

	/* Reg0F */		//reserved for 41510
	F_SYSV,

	/* Reg10 */
	F_TSPCT,

	/* Reg11 */
	F_VBUS_GD,
	F_VBUSV,		//reserved for 41510

	/* Reg12 */
	F_ICHGR,

	/* Reg13 */
	F_VDPM_STAT,
	F_IDPM_STAT,
	F_IDPM_LIM,		//reserved for 41510

	/* Reg14 */
	F_REG_RST,
	F_ICO_OPTIMIZED, 	//reserved for 41510
	F_PN,
	F_TS_PROFILE, 	//reserved for 41510
	F_DEV_REV,

	F_MAX_FIELDS
};

/* initial field values, converted to register values */
struct sgm41510_init_data {
	u8 ichg;	/* charge current		*/
	u8 vreg;	/* regulation voltage		*/
	u8 iterm;	/* termination current		*/
	u8 iprechg;	/* precharge current		*/
	u8 sysvmin;	/* minimum system voltage limit */
	u8 boostv;	/* boost regulation voltage	*/
	u8 boosti;	/* boost current limit		*/
	u8 boostf;	/* boost frequency		*/
	u8 ilim_en;	/* enable ILIM pin		*/
	u8 treg;	/* thermal regulation threshold */
};

struct sgm41510_state {
	u8 online;
	u8 chrg_status;
	u8 chrg_fault;
	u8 vsys_status;
	u8 boost_fault;
	u8 bat_fault;
};

struct sgm41510_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;

	struct usb_phy *usb_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;

	struct regmap *rmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];

	int chip_id;
	struct sgm41510_init_data init_data;
	struct sgm41510_state state;

	struct mutex lock; /* protect state data */
	int sgm_en_gpio;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	const char *chg_dev_name;
	atomic_t is_chip_en;
};
struct sgm41510_device *lc_sgm;

static const struct regmap_range sgm41510_readonly_reg_ranges[] = {
	regmap_reg_range(0x0b, 0x0c),
	regmap_reg_range(0x0e, 0x13),
};

static const struct regmap_access_table sgm41510_writeable_regs = {
	.no_ranges = sgm41510_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(sgm41510_readonly_reg_ranges),
};

static const struct regmap_range sgm41510_volatile_reg_ranges[] = {
	regmap_reg_range(0x00, 0x00),
	regmap_reg_range(0x09, 0x09),
	regmap_reg_range(0x0b, 0x0c),
	regmap_reg_range(0x0e, 0x14),
};

static const struct regmap_access_table sgm41510_volatile_regs = {
	.yes_ranges = sgm41510_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(sgm41510_volatile_reg_ranges),
};

static const struct regmap_config sgm41510_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x14,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &sgm41510_writeable_regs,
	.volatile_table = &sgm41510_volatile_regs,
};

static const struct reg_field sgm41510_reg_fields[] = {
	/* REG00 */
	[F_EN_HIZ]			= REG_FIELD(0x00, 7, 7),
	[F_EN_ILIM]			= REG_FIELD(0x00, 6, 6),
	[F_IILIM]			= REG_FIELD(0x00, 0, 5),
	/* REG01 */
	[F_BHOT]			= REG_FIELD(0x01, 6, 7),
	[F_BCOLD]			= REG_FIELD(0x01, 5, 5),
	[F_VINDPM_OFS]		= REG_FIELD(0x01, 0, 4),
	/* REG02 */
	[F_CONV_START]		= REG_FIELD(0x02, 7, 7),
	[F_CONV_RATE]		= REG_FIELD(0x02, 6, 6),
	[F_BOOSTF]			= REG_FIELD(0x02, 5, 5),
	[F_ICO_EN]			= REG_FIELD(0x02, 4, 4),
	[F_HVDCP_EN]		= REG_FIELD(0x02, 3, 3),
	[F_MAXC_EN]			= REG_FIELD(0x02, 2, 2),
	[F_FORCE_DPM]		= REG_FIELD(0x02, 1, 1),
	[F_AUTO_DPDM_EN]	= REG_FIELD(0x02, 0, 0),
	/* REG03 */
	[F_BAT_LOAD_EN]		= REG_FIELD(0x03, 7, 7),
	[F_WD_RST]			= REG_FIELD(0x03, 6, 6),
	[F_OTG_CFG]			= REG_FIELD(0x03, 5, 5),
	[F_CHG_CFG]			= REG_FIELD(0x03, 4, 4),
	[F_SYSVMIN]			= REG_FIELD(0x03, 1, 3),
	/* REG04 */
	[F_PUMPX_EN]		= REG_FIELD(0x04, 7, 7),
	[F_ICHG]			= REG_FIELD(0x04, 0, 6),
	/* REG05 */
	[F_IPRECHG]			= REG_FIELD(0x05, 4, 7),
	[F_ITERM]			= REG_FIELD(0x05, 0, 3),
	/* REG06 */
	[F_VREG]			= REG_FIELD(0x06, 2, 7),
	[F_BATLOWV]			= REG_FIELD(0x06, 1, 1),
	[F_VRECHG]			= REG_FIELD(0x06, 0, 0),
	/* REG07 */
	[F_TERM_EN]			= REG_FIELD(0x07, 7, 7),
	[F_STAT_DIS]		= REG_FIELD(0x07, 6, 6),
	[F_WD]				= REG_FIELD(0x07, 4, 5),
	[F_TMR_EN]			= REG_FIELD(0x07, 3, 3),
	[F_CHG_TMR]			= REG_FIELD(0x07, 1, 2),
	[F_JEITA_ISET]		= REG_FIELD(0x07, 0, 0),
	/* REG08 */
	[F_BATCMP]			= REG_FIELD(0x08, 5, 7),
	[F_VCLAMP]			= REG_FIELD(0x08, 2, 4),
	[F_TREG]			= REG_FIELD(0x08, 0, 1),
	/* REG09 */
	[F_FORCE_ICO]		= REG_FIELD(0x09, 7, 7),
	[F_TMR2X_EN]		= REG_FIELD(0x09, 6, 6),
	[F_BATFET_DIS]		= REG_FIELD(0x09, 5, 5),
	[F_JEITA_VSET]		= REG_FIELD(0x09, 4, 4),
	[F_BATFET_DLY]		= REG_FIELD(0x09, 3, 3),
	[F_BATFET_RST_EN]	= REG_FIELD(0x09, 2, 2),
	[F_PUMPX_UP]		= REG_FIELD(0x09, 1, 1),
	[F_PUMPX_DN]		= REG_FIELD(0x09, 0, 0),
	/* REG0A */
	[F_BOOSTV]			= REG_FIELD(0x0A, 4, 7),
	[F_BOOSTI]			= REG_FIELD(0x0A, 0, 2),
	/* REG0B */
	[F_VBUS_STAT]		= REG_FIELD(0x0B, 5, 7),
	[F_CHG_STAT]		= REG_FIELD(0x0B, 3, 4),
	[F_PG_STAT]			= REG_FIELD(0x0B, 2, 2),
	[F_SDP_STAT]		= REG_FIELD(0x0B, 1, 1),
	[F_VSYS_STAT]		= REG_FIELD(0x0B, 0, 0),
	/* REG0C */
	[F_WD_FAULT]		= REG_FIELD(0x0C, 7, 7),
	[F_BOOST_FAULT]		= REG_FIELD(0x0C, 6, 6),
	[F_CHG_FAULT]		= REG_FIELD(0x0C, 4, 5),
	[F_BAT_FAULT]		= REG_FIELD(0x0C, 3, 3),
	[F_NTC_FAULT]		= REG_FIELD(0x0C, 0, 2),
	/* REG0D */
	[F_FORCE_VINDPM]	= REG_FIELD(0x0D, 7, 7),
	[F_VINDPM]			= REG_FIELD(0x0D, 0, 6),
	/* REG0E */
	[F_THERM_STAT]		= REG_FIELD(0x0E, 7, 7),
	[F_BATV]			= REG_FIELD(0x0E, 0, 6),
	/* REG0F */
	[F_SYSV]			= REG_FIELD(0x0F, 0, 6),
	/* REG10 */
	[F_TSPCT]			= REG_FIELD(0x10, 0, 6),
	/* REG11 */
	[F_VBUS_GD]			= REG_FIELD(0x11, 7, 7),
	[F_VBUSV]			= REG_FIELD(0x11, 0, 6),
	/* REG12 */
	[F_ICHGR]			= REG_FIELD(0x12, 0, 6),
	/* REG13 */
	[F_VDPM_STAT]		= REG_FIELD(0x13, 7, 7),
	[F_IDPM_STAT]		= REG_FIELD(0x13, 6, 6),
	[F_IDPM_LIM]		= REG_FIELD(0x13, 0, 5),
	/* REG14 */
	[F_REG_RST]			= REG_FIELD(0x14, 7, 7),
	[F_ICO_OPTIMIZED]	= REG_FIELD(0x14, 6, 6),
	[F_PN]				= REG_FIELD(0x14, 3, 5),
	[F_TS_PROFILE]		= REG_FIELD(0x14, 2, 2),
	[F_DEV_REV]			= REG_FIELD(0x14, 0, 1)
};

/*
 * Most of the val -> idx conversions can be computed, given the minimum,
 * maximum and the step between values. For the rest of conversions, we use
 * lookup tables.
 */
enum sgm41510_table_ids {
	/* range tables */
	TBL_ICHG,
	TBL_ITERM,
	TBL_IPRECHG,
	TBL_VREG,
	TBL_BATCMP,
	TBL_VCLAMP,
	TBL_BOOSTV,
	TBL_SYSVMIN,

	/* lookup tables */
	TBL_TREG,
	TBL_BOOSTI,
};

/* Thermal Regulation Threshold lookup table, in degrees Celsius */
static const u32 sgm41510_treg_tbl[] = { 60, 80, 100, 120 };

#define SGM41510_TREG_TBL_SIZE		ARRAY_SIZE(sgm41510_treg_tbl)

/* Boost mode current limit lookup table, in uA */
static const u32 sgm41510_boosti_tbl[] = {
	1200000, 1600000, 2000000, 2400000, 2800000, 3200000, 3600000, 4000000
};

#define SGM41510_BOOSTI_TBL_SIZE		ARRAY_SIZE(sgm41510_boosti_tbl)

struct sgm41510_range {
	u32 min;
	u32 max;
	u32 step;
};

struct sgm41510_lookup {
	const u32 *tbl;
	u32 size;
};

static const union {
	struct sgm41510_range  rt;
	struct sgm41510_lookup lt;
} sgm41510_tables[] = {
	/* range tables */
	[TBL_ICHG] =	{ .rt = {0,	  5056000, 64000} },	 /* uA */
	[TBL_ITERM] =	{ .rt = {64000,   1024000, 64000} },	 /* uA */
	[TBL_VREG] =	{ .rt = {3840000, 4608000, 16000} },	 /* uV */
	[TBL_BATCMP] =	{ .rt = {0,	  140,     20} },	 /* mOhm */
	[TBL_VCLAMP] =	{ .rt = {0,	  224000,  32000} },	 /* uV */
	[TBL_BOOSTV] =	{ .rt = {4550000, 5510000, 64000} },	 /* uV */
	[TBL_SYSVMIN] = { .rt = {3000000, 3700000, 100000} },	 /* uV */

	/* lookup tables */
	[TBL_TREG] =	{ .lt = {sgm41510_treg_tbl, SGM41510_TREG_TBL_SIZE} },
	[TBL_BOOSTI] =	{ .lt = {sgm41510_boosti_tbl, SGM41510_BOOSTI_TBL_SIZE} }
};

static int sgm41510_field_read(struct sgm41510_device *sgm,
			      enum sgm41510_fields field_id)
{
	int ret;
	int val;
pr_err("addtest sgm41510_field_read\n");
	ret = regmap_field_read(sgm->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}

static int sgm41510_field_write(struct sgm41510_device *sgm,
			       enum sgm41510_fields field_id, u8 val)
{
	pr_err("addtest sgm41510_field_write\n");
	return regmap_field_write(sgm->rmap_fields[field_id], val);
}

static u8 sgm41510_find_idx(u32 value, enum sgm41510_table_ids id)
{
	u8 idx;
	if (id >= TBL_TREG) {
		const u32 *tbl = sgm41510_tables[id].lt.tbl;
		u32 tbl_size = sgm41510_tables[id].lt.size;

		for (idx = 1; idx < tbl_size && tbl[idx] <= value; idx++)
			;
	} else {
		const struct sgm41510_range *rtbl = &sgm41510_tables[id].rt;
		u8 rtbl_size;

		rtbl_size = (rtbl->max - rtbl->min) / rtbl->step + 1;

		for (idx = 1;
		     idx < rtbl_size && (idx * rtbl->step + rtbl->min <= value);
		     idx++)
			;
	}

	return idx - 1;
}

static u32 sgm41510_find_val(u8 idx, enum sgm41510_table_ids id)
{
	const struct sgm41510_range *rtbl;
	/* lookup table? */
	if (id >= TBL_TREG)
		return sgm41510_tables[id].lt.tbl[idx];

	/* range table */
	rtbl = &sgm41510_tables[id].rt;

	return (rtbl->min + idx * rtbl->step);
}

enum sgm41510_status {
	STATUS_NOT_CHARGING,
	STATUS_PRE_CHARGING,
	STATUS_FAST_CHARGING,
	STATUS_TERMINATION_DONE,
};

enum sgm41510_chrg_fault {
	CHRG_FAULT_NORMAL,
	CHRG_FAULT_INPUT,
	CHRG_FAULT_THERMAL_SHUTDOWN,
	CHRG_FAULT_TIMER_EXPIRED,
};

static int sgm41510_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	int ret;
	struct sgm41510_device *sgm = power_supply_get_drvdata(psy);
	struct sgm41510_state state;
	mutex_lock(&sgm->lock);
	state = sgm->state;
	mutex_unlock(&sgm->lock);
pr_err("addtest sgm41510_power_supply_get_property\n");
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.online)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_status == STATUS_NOT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == STATUS_PRE_CHARGING ||
			 state.chrg_status == STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SGM41510_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (!state.chrg_fault && !state.bat_fault && !state.boost_fault)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (state.bat_fault)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.chrg_fault == CHRG_FAULT_TIMER_EXPIRED)
			val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		else if (state.chrg_fault == CHRG_FAULT_THERMAL_SHUTDOWN)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sgm41510_field_read(sgm, F_ICHG); /* read measured value */
		if (ret < 0)
			return ret;

		/* converted_val = ADC_val * 50mA (table 10.3.19) */
		val->intval = ret * 64;
		pr_err("andy sgm41510_current=%d",val->intval);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = sgm41510_tables[TBL_ICHG].rt.max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:	//reserved for 41510

		//if (!state.online) {
		//	val->intval = 0;
		//	break;
		//}

		//ret = sgm41510_field_read(sgm, F_BATV); /* read measured value */
		//if (ret < 0)
		//	return ret;

		/* converted_val = 2.304V + ADC_val * 20mV (table 10.3.15) */
		//val->intval = 2304000 + ret * 20000;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = sgm41510_tables[TBL_VREG].rt.max;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = sgm41510_find_val(sgm->init_data.iterm, TBL_ITERM);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int sgm41510_get_chip_state(struct sgm41510_device *sgm,
				  struct sgm41510_state *state)
{
	int i, ret;
	struct {
		enum sgm41510_fields id;
		u8 *data;
	} state_fields[] = {
		{F_CHG_STAT,	&state->chrg_status},
		{F_PG_STAT,	&state->online},
		{F_VSYS_STAT,	&state->vsys_status},
		{F_BOOST_FAULT, &state->boost_fault},
		{F_BAT_FAULT,	&state->bat_fault},
		{F_CHG_FAULT,	&state->chrg_fault}
	};
pr_err("addtest sgm41510_get_chip_state\n");
	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = sgm41510_field_read(sgm, state_fields[i].id);
		if (ret < 0)
			return ret;

		*state_fields[i].data = ret;
	}

	dev_err(sgm->dev, "S:CHG/PG/VSYS=%d/%d/%d, F:CHG/BOOST/BAT=%d/%d/%d\n",
		state->chrg_status, state->online, state->vsys_status,
		state->chrg_fault, state->boost_fault, state->bat_fault);

	return 0;
}

static bool sgm41510_state_changed(struct sgm41510_device *sgm,
				  struct sgm41510_state *new_state)
{
	struct sgm41510_state old_state;
pr_err("addtest sgm41510_state_changed\n");
	mutex_lock(&sgm->lock);
	old_state = sgm->state;
	mutex_unlock(&sgm->lock);

	return (old_state.chrg_status != new_state->chrg_status ||
		old_state.chrg_fault != new_state->chrg_fault	||
		old_state.online != new_state->online		||
		old_state.bat_fault != new_state->bat_fault	||
		old_state.boost_fault != new_state->boost_fault ||
		old_state.vsys_status != new_state->vsys_status);
}

static void sgm41510_handle_state_change(struct sgm41510_device *sgm,
					struct sgm41510_state *new_state)
{
	int ret;
	struct sgm41510_state old_state;
pr_err("addtest sgm41510_handle_state_change\n");
	mutex_lock(&sgm->lock);
	old_state = sgm->state;
	mutex_unlock(&sgm->lock);

	if (!new_state->online) {			     /* power removed */
		/* disable ADC */
		ret = sgm41510_field_write(sgm, F_CONV_START, 0);
		if (ret < 0)
			goto error;
	} else if (!old_state.online) {			    /* power inserted */
		/* enable ADC, to have control of charge current/voltage */
		ret = sgm41510_field_write(sgm, F_CONV_START, 0);
		if (ret < 0)
			goto error;
	}

	return;

error:
	dev_err(sgm->dev, "Error communicating with the chip.\n");
}

static irqreturn_t sgm41510_irq_handler_thread(int irq, void *private)
{
	struct sgm41510_device *sgm = private;
	int ret;
	struct sgm41510_state state;
pr_err("addtest sgm41510_irq_handler_thread\n");
	ret = sgm41510_get_chip_state(sgm, &state);
	if (ret < 0)
		goto handled;

	if (!sgm41510_state_changed(sgm, &state))
		goto handled;

	sgm41510_handle_state_change(sgm, &state);

	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	//power_supply_changed(sgm->charger);

handled:
	return IRQ_HANDLED;
}

static int sgm41510_chip_reset(struct sgm41510_device *sgm)
{
	int ret;
	int rst_check_counter = 10;
pr_err("addtest sgm41510_chip_reset\n");
	ret = sgm41510_field_write(sgm, F_REG_RST, 1);
	if (ret < 0)
		return ret;

	do {
		ret = sgm41510_field_read(sgm, F_REG_RST);
		if (ret < 0)
			return ret;

		usleep_range(5, 10);
	} while (ret == 1 && --rst_check_counter);

	if (!rst_check_counter)
		return -ETIMEDOUT;

	return 0;
}
/**********************************************************
*
*   [config]
*
*
***********************************************************/
void sgm41510_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_field_write(lc_sgm, F_EN_HIZ,
				  val);

}


void sgm41510_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_field_write(lc_sgm, F_CHG_CFG,
				  val);

}


static int sgm41510_get_chrg_stat(void)
{
	int value = 0;
	value= sgm41510_field_read(lc_sgm, F_CHG_STAT);
	return value;
}

void sgm41510_set_en_shippingmode(unsigned int val)
{
	unsigned int ret = 0;

	ret = sgm41510_field_write(lc_sgm, F_BATFET_DIS,
				  val);

}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
int sgm41510_dump_register(struct charger_device *chg_dev)
{

	unsigned char i = 0;
	int sgmreg = 0;

	pr_err("%s:[sgm41510]\n",__func__);
	for(i=0;i < F_MAX_FIELDS;i++){
			sgmreg= sgm41510_field_read(lc_sgm, i);
	
		switch (i) {
		case F_EN_HIZ:
			pr_err("andy sgmreg F_EN_HIZ[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_EN_ILIM:
				pr_err("andy sgmreg F_EN_ILIM[%d]val=%d=%x",i,sgmreg,sgmreg);
				break;
		case F_IILIM:
			pr_err("andy sgmreg F_IILIM[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BHOT:
			pr_err("andy sgmreg F_BHOT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BCOLD:
			pr_err("andy sgmreg F_BCOLD[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VINDPM_OFS:
			pr_err("andy sgmreg F_VINDPM_OFS[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_CONV_START:
			pr_err("andy sgmreg F_CONV_START[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_CONV_RATE:
			pr_err("andy sgmreg F_CONV_RATE[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BOOSTF:
			pr_err("andy sgmreg F_BOOSTF[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_ICO_EN:
			pr_err("andy sgmreg F_ICO_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_HVDCP_EN:
			pr_err("andy sgmreg F_HVDCP_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_MAXC_EN:
			pr_err("andy sgmreg F_MAXC_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;	
		case F_FORCE_DPM:
			pr_err("andy sgmreg F_FORCE_DPM[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_AUTO_DPDM_EN:
			pr_err("andy sgmreg F_AUTO_DPDM_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BAT_LOAD_EN:
			pr_err("andy sgmreg F_BAT_LOAD_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_WD_RST:
			pr_err("andy sgmreg F_WD_RST[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_OTG_CFG:
			pr_err("andy sgmreg F_OTG_CFG[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_CHG_CFG:
			pr_err("andy sgmreg F_CHG_CFG[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_SYSVMIN:
			pr_err("andy sgmreg F_SYSVMIN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_PUMPX_EN:
			pr_err("andy sgmreg F_PUMPX_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_ICHG:
			pr_err("andy sgmreg F_ICHG[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_IPRECHG:
			pr_err("andy sgmreg F_IPRECHG[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_ITERM:
			pr_err("andy sgmreg F_ITERM[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VREG:
			pr_err("andy sgmreg F_VREG[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BATLOWV:
			pr_err("andy sgmreg F_BATLOWV[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VRECHG:
			pr_err("andy sgmreg F_VRECHG[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_TERM_EN:
			pr_err("andy sgmreg F_TERM_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_STAT_DIS:
			pr_err("andy sgmreg F_STAT_DIS[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_WD:
			pr_err("andy sgmreg F_WD[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_TMR_EN:
			pr_err("andy sgmreg F_TMR_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_CHG_TMR:
			pr_err("andy sgmreg F_CHG_TMR[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_JEITA_ISET:
			pr_err("andy sgmreg F_JEITA_ISET[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BATCMP:
			pr_err("andy sgmreg F_BATCMP[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VCLAMP:
			pr_err("andy sgmreg F_VCLAMP[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_TREG:
			pr_err("andy sgmreg F_TREG[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_FORCE_ICO:
			pr_err("andy sgmreg F_FORCE_ICO[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_TMR2X_EN:
			pr_err("andy sgmreg F_TMR2X_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BATFET_DIS:
			pr_err("andy sgmreg F_BATFET_DIS[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_JEITA_VSET:
			pr_err("andy sgmreg F_JEITA_VSET[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BATFET_DLY:
			pr_err("andy sgmreg F_BATFET_DLY[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BATFET_RST_EN:
			pr_err("andy sgmreg F_BATFET_RST_EN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_PUMPX_UP:
			pr_err("andy sgmreg F_PUMPX_UP[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_PUMPX_DN:
			pr_err("andy sgmreg F_PUMPX_DN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
	
		case F_BOOSTV:
			pr_err("andy sgmreg F_BOOSTV[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BOOSTI:
			pr_err("andy sgmreg F_BOOSTI[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VBUS_STAT:
			pr_err("andy sgmreg F_VBUS_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_CHG_STAT:
			pr_err("andy sgmreg F_CHG_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_PG_STAT:
			pr_err("andy sgmreg F_PG_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_SDP_STAT:
			pr_err("andy sgmreg F_SDP_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;			
		case F_VSYS_STAT:
			pr_err("andy sgmreg F_VSYS_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_WD_FAULT:
			pr_err("andy sgmreg F_WD_FAULT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BOOST_FAULT:
			pr_err("andy sgmreg F_BOOST_FAULT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_CHG_FAULT:
			pr_err("andy sgmreg F_CHG_FAULT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BAT_FAULT:
			pr_err("andy sgmreg F_BAT_FAULT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_NTC_FAULT:
			pr_err("andy sgmreg F_NTC_FAULT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_FORCE_VINDPM:
			pr_err("andy sgmreg F_FORCE_VINDPM[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VINDPM:
			pr_err("andy sgmreg F_VINDPM[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_THERM_STAT:
			pr_err("andy sgmreg F_THERM_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_BATV:
			pr_err("andy sgmreg F_BATV[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_SYSV:
			pr_err("andy sgmreg F_SYSV[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_TSPCT:
			pr_err("andy sgmreg F_TSPCT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VBUS_GD:
			pr_err("andy sgmreg F_VBUS_GD[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VBUSV:
			pr_err("andy sgmreg F_VBUSV[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_ICHGR:
			pr_err("andy sgmreg F_ICHGR[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_VDPM_STAT:
			pr_err("andy sgmreg F_VDPM_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_IDPM_STAT:
			pr_err("andy sgmreg F_IDPM_STAT[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_IDPM_LIM:
			pr_err("andy sgmreg F_IDPM_LIM[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_REG_RST:
			pr_err("andy sgmreg F_REG_RST[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_ICO_OPTIMIZED:
			pr_err("andy sgmreg F_ICO_OPTIMIZED[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_PN:
			pr_err("andy sgmreg F_PN[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_TS_PROFILE:
			pr_err("andy sgmreg F_TS_PROFILE[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		case F_DEV_REV:
			pr_err("andy sgmreg F_DEV_REV[%d]val=%d=%x",i,sgmreg,sgmreg);
			break;
		default:
			pr_err("error here\n");
			break;
			}
		usleep_range(5, 10);
		}

	return 0;
}
static bool en_back = false;

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/

static int sgm41510_enable_charging(struct charger_device *chg_dev,
				   bool en)
{
	int status = 0;

	pr_err("enable state : %d,en_back=%d\n", en,en_back);
	if(en != en_back)
	{
		if (en) {
			/* sgm41510_config_interface(sgm41510_CON3, 0x1, 0x1, 4); */
			/* enable charging */
			sgm41510_set_en_hiz(0x0);
			sgm41510_set_chg_config(en);
			sgm41510_set_en_shippingmode(0x0);
		} else {
			/* sgm41510_config_interface(sgm41510_CON3, 0x0, 0x1, 4); */
			/* enable charging */
			sgm41510_set_chg_config(en);
			pr_err("[charging_enable] under test mode: disable charging\n");

			sgm41510_set_en_hiz(0x1);
		}
	
		en_back = en;
	}
	//sgm41510_dump_register(chg_dev);
	return status;
}

static int sgm41510_get_current(struct charger_device *chg_dev,
			       u32 *ichg)
{
	unsigned int ret_val = 0;
#if 0 //todo
	unsigned char ret_force_20pct = 0;

	/* Get current level */
	sgm41510_read_interface(sgm41510_CON2, &ret_val, CON2_ICHG_MASK,
			       CON2_ICHG_SHIFT);

	/* Get Force 20% option */
	sgm41510_read_interface(sgm41510_CON2, &ret_force_20pct,
			       CON2_FORCE_20PCT_MASK,
			       CON2_FORCE_20PCT_SHIFT);

	/* Parsing */
	ret_val = (ret_val * 64) + 512;

#endif
	return ret_val;
}

static int sgm41510_set_current(struct charger_device *chg_dev,
			       u32 current_value)
{
	unsigned int status = true;
	unsigned int ret = 0;
	unsigned int sgm_current_value = 25;
	#if 0
	switch(current_value){
		case 2000000:
			sgm_current_value = 31;
			break;
		case 1600000:
			sgm_current_value = 25;
			break;
		case 1536000:
			sgm_current_value = 24;
			break;
		default:
			break;
	}
	#endif
	sgm_current_value = current_value/64000;
	pr_err("%s: current_value=%d\n",__func__,current_value);
	ret = sgm41510_field_write(lc_sgm, F_ICHG,
				  sgm_current_value);
	return status;
}

static int sgm41510_get_input_current(struct charger_device *chg_dev,
				     u32 *aicr)
{
	int ret = 0;
#if 0
	unsigned char val = 0;

	sgm41510_read_interface(sgm41510_CON0, &val, CON0_IINLIM_MASK,
			       CON0_IINLIM_SHIFT);
	ret = (int)val;
	*aicr = INPUT_CS_VTH[val];
#endif
	return ret;
}


static int sgm41510_set_input_current(struct charger_device *chg_dev,
				     u32 current_value)
{
	unsigned int status = true;
	unsigned int ret = 0;
	unsigned int sgm_input_current_value = 0x0a;
	#if 0
	switch(current_value){
		case 3000000:
			sgm_input_current_value = 0x1e;
			break;
		case 2200000:
			sgm_input_current_value = 0x16;
			break;
		case 1000000:
			sgm_input_current_value = 0x0a;
			break;
		case 700000:
			sgm_input_current_value = 0x07;
			break;
		case 600000:
			sgm_input_current_value = 0x06;
			break;
		default:
			break;
	}
	#endif
	sgm_input_current_value = current_value/100000;
	ret = sgm41510_field_write(lc_sgm, F_IILIM,
				  sgm_input_current_value);
	return status;
}

static int sgm41510_set_cv_voltage(struct charger_device *chg_dev,
				  u32 cv)
{
	unsigned int status = true;
	unsigned int ret = 0;
	unsigned int sgm_cv_value = 0x25;
	unsigned int sgm_cv = cv/1000 +2;
	if(sgm_cv > 3840)
		sgm_cv_value = (sgm_cv -3840)/16;
	pr_err("%s: cv=%d\n",__func__,sgm_cv);
	ret = sgm41510_field_write(lc_sgm, F_VREG,
				  sgm_cv_value);
	return status;
}

static int sgm41510_reset_watch_dog_timer(struct charger_device
		*chg_dev)
{
	unsigned int status = true;
#if 0
	pr_info("charging_reset_watch_dog_timer\n");

	sgm41510_set_wdt_rst(0x1);	/* Kick watchdog */
	sgm41510_set_watchdog(0x3);	/* WDT 160s */
#endif
	return status;
}


static int sgm41510_set_vindpm_voltage(struct charger_device *chg_dev,
				      u32 vindpm)
{
	int status = 0;
	#if 0
	unsigned int array_size;

	vindpm /= 1000;
	array_size = ARRAY_SIZE(VINDPM_REG);
	vindpm = bmt_find_closest_level(VINDPM_REG, array_size, vindpm);
	vindpm = charging_parameter_to_value(VINDPM_REG, array_size, vindpm);

	pr_info("%s vindpm =%d\r\n", __func__, vindpm);

	//	charging_set_vindpm(vindpm);
	/*sgm41510_set_en_hiz(en);*/
	#endif
	return status;
}

static int sgm41510_get_charging_status(struct charger_device *chg_dev,
				       bool *is_done)
{
	unsigned int status = true;
	int ret_val;

	ret_val = sgm41510_get_chrg_stat();
	pr_info("%s status = %d\n", __func__, ret_val);
	if (ret_val == 0x3)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int sgm41510_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;

	pr_info("%s en = %d\n", __func__, en);
	#if 0
	if (en) {
		sgm41510_set_chg_config(0);
		sgm41510_set_otg_config(1);
		sgm41510_set_watchdog(0x3);	/* WDT 160s */
	} else {
		sgm41510_set_otg_config(0);
		sgm41510_set_chg_config(1);
	}
	#endif
	return ret;
}

static int sgm41510_set_boost_current_limit(struct charger_device
		*chg_dev, u32 uA)
{
	int ret = 0;
	#if 0
	u32 array_size = 0;
	u32 boost_ilimit = 0;
	u8 boost_reg = 0;

	uA /= 1000;
	array_size = ARRAY_SIZE(BOOST_CURRENT_LIMIT);
	boost_ilimit = bmt_find_closest_level(BOOST_CURRENT_LIMIT, array_size,
					      uA);
	boost_reg = charging_parameter_to_value(BOOST_CURRENT_LIMIT,
						array_size, boost_ilimit);
	sgm41510_set_boost_lim(boost_reg);
	#endif
	return ret;
}

static int sgm41510_enable_safetytimer(struct charger_device *chg_dev,
				      bool en)
{
	int status = 0;
	#if 0
	if (en)
		sgm41510_set_en_timer(0x1);
	else
		sgm41510_set_en_timer(0x0);
	#endif
	return status;
}

static int sgm41510_get_is_safetytimer_enable(struct charger_device
		*chg_dev, bool *en)
{
	unsigned char val = 0;
#if 0
	sgm41510_read_interface(sgm41510_CON5, &val, CON5_EN_TIMER_MASK,
			       CON5_EN_TIMER_SHIFT);
	*en = (bool)val;
#endif
	return val;
}

static int sgm41510_enable_chip(struct charger_device *chg_dev, bool en)
{
	struct sgm41510_device  *info = dev_get_drvdata(&chg_dev->dev);
	//int ret = 0;
	bool is_chip_en = false;
	

	is_chip_en = atomic_read(&info->is_chip_en);
	pr_err("%s: enable=%d,is_chip_en=%d\n", __func__,en,is_chip_en);
	if(en != is_chip_en)
	{
		//if(gpio_is_valid(info->sgm_en_gpio))
		//	ret = gpio_direction_output(info->sgm_en_gpio, en ? 0 : 1);

		//if(ret)
		//{
		//	pr_err("%s: set lc_otg_en_gpio failed\n", __func__);

		//}

		//mdelay(1);
		atomic_set(&info->is_chip_en, en);
	}
	return 0;

}
#if 0
static inline bool __sgm41510_is_chip_en(struct sgm41510_device *info)
{
	int en = 0;

	en = gpio_get_value(info->sgm_en_gpio);
	if ((en && !gpio_get_value(info->sgm_en_gpio)) ||
		(!en && gpio_get_value(info->sgm_en_gpio)))
		chr_err("%s: en not sync(%d, %d)\n", __func__, en,
			gpio_get_value(info->sgm_en_gpio));

	return en;
}
#endif
static int sgm41510_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	struct sgm41510_device *info = dev_get_drvdata(&chg_dev->dev);
#if 0
	//mutex_lock(&info->gpio_access_lock);
	*en = __sgm41510_is_chip_en(info);
	//mutex_unlock(&info->gpio_access_lock);
	pr_err("%s: %d\n",__func__,*en);
#endif
	//bool is_chip_en = false;
		
	*en = atomic_read(&info->is_chip_en);	
	pr_err("%s: enable=%d\n", __func__,*en);
	
	
	
	return 0;
}


static int sgm41510_do_event(struct charger_device *chg_dev, u32 event,
			    u32 args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	pr_err("%s: event = %d\n", __func__, event);
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

static struct charger_ops sgm41510_chg_ops = {
#if 0
	.enable_hz = sgm41510_enable_hz,
#endif

	/* Normal charging */
	.dump_registers = sgm41510_dump_register,
	.enable = sgm41510_enable_charging,
	.is_chip_enabled = sgm41510_is_chip_enabled,
	.enable_chip = sgm41510_enable_chip,
	.get_charging_current = sgm41510_get_current,
	.set_charging_current = sgm41510_set_current,
	.get_input_current = sgm41510_get_input_current,
	.set_input_current = sgm41510_set_input_current,
	/*.get_constant_voltage = sgm41510_get_battery_voreg,*/
	.set_constant_voltage = sgm41510_set_cv_voltage,
	.kick_wdt = sgm41510_reset_watch_dog_timer,
	.set_mivr = sgm41510_set_vindpm_voltage,
	.is_charging_done = sgm41510_get_charging_status,

	/* Safety timer */
	.enable_safety_timer = sgm41510_enable_safetytimer,
	.is_safety_timer_enabled = sgm41510_get_is_safetytimer_enable,


	/* Power path */
	/*.enable_powerpath = sgm41510_enable_power_path, */
	/*.is_powerpath_enabled = sgm41510_get_is_power_path_enable, */


	/* OTG */
	.enable_otg = sgm41510_enable_otg,
	.set_boost_current_limit = sgm41510_set_boost_current_limit,
	.event = sgm41510_do_event,
};


static int sgm41510_hw_init(struct sgm41510_device *sgm)
{
	int ret;
	int i;
	struct sgm41510_state state;
	const struct {
		enum sgm41510_fields id;
		u32 value;
	} init_data[] = {
		{F_ICHG,	 sgm->init_data.ichg},
		{F_VREG,	 sgm->init_data.vreg},
		{F_ITERM,	 sgm->init_data.iterm},
		{F_IPRECHG,	 sgm->init_data.iprechg},
		{F_SYSVMIN,	 sgm->init_data.sysvmin},
		{F_BOOSTV,	 sgm->init_data.boostv},
		{F_BOOSTI,	 sgm->init_data.boosti},
		{F_BOOSTF,	 sgm->init_data.boostf},
		{F_EN_ILIM,	 sgm->init_data.ilim_en},
		{F_TREG,	 sgm->init_data.treg},
		{F_IILIM,	 0x16},
		{F_EN_HIZ,	 0x01},
		{F_VINDPM,	 0x13},
		{F_BATCMP,	 0x01},
		{F_VCLAMP,	 0x01},
		{F_CONV_START,	 0x00}
	};
pr_err("addtest sgm41510_hw_init\n");
	ret = sgm41510_chip_reset(sgm);
	if (ret < 0)
		return ret;

	/* disable watchdog */
	ret = sgm41510_field_write(sgm, F_WD, 0);
	if (ret < 0)
		return ret;

	/* initialize currents/voltages and other parameters */
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = sgm41510_field_write(sgm, init_data[i].id,
					  init_data[i].value);
		if (ret < 0)
			return ret;
	}

	/* Configure ADC for continuous conversions. This does not enable it. */
	/***************************************************************************
	//reserved for 41510
	***************************************************************************
	ret = sgm41510_field_write(sgm, F_CONV_RATE, 1);
	if (ret < 0)
		return ret;
	***************************************************************************/

	ret = sgm41510_get_chip_state(sgm, &state);
	if (ret < 0)
		return ret;

	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	return 0;
}

static enum power_supply_property sgm41510_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static char *sgm41510_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc sgm41510_power_supply_desc = {
	.name = "sgm41510-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sgm41510_power_supply_props,
	.num_properties = ARRAY_SIZE(sgm41510_power_supply_props),
	.get_property = sgm41510_power_supply_get_property,
};

static int sgm41510_power_supply_init(struct sgm41510_device *sgm)
{
	struct power_supply_config psy_cfg = { .drv_data = sgm, };
pr_err("addtest sgm41510_power_supply_init\n");
	psy_cfg.supplied_to = sgm41510_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sgm41510_charger_supplied_to);

	sgm->charger = power_supply_register(sgm->dev, &sgm41510_power_supply_desc,
					    &psy_cfg);

	return PTR_ERR_OR_ZERO(sgm->charger);
}

static void sgm41510_usb_work(struct work_struct *data)
{
	int ret;
	struct sgm41510_device *sgm =
			container_of(data, struct sgm41510_device, usb_work);

	switch (sgm->usb_event) {
	case USB_EVENT_ID:
		/* Enable boost mode */
		ret = sgm41510_field_write(sgm, F_OTG_CFG, 1);
		if (ret < 0)
			goto error;
		break;

	case USB_EVENT_NONE:
		/* Disable boost mode */
		ret = sgm41510_field_write(sgm, F_OTG_CFG, 0);
		if (ret < 0)
			goto error;

		//power_supply_changed(sgm->charger);
		break;
	}

	return;

error:
	dev_err(sgm->dev, "Error switching to boost/charger mode.\n");
}

static int sgm41510_usb_notifier(struct notifier_block *nb, unsigned long val,
				void *priv)
{
	struct sgm41510_device *sgm =
			container_of(nb, struct sgm41510_device, usb_nb);

	sgm->usb_event = val;
	queue_work(system_power_efficient_wq, &sgm->usb_work);

	return NOTIFY_OK;
}

static int sgm41510_irq_probe(struct sgm41510_device *sgm)
{
	struct gpio_desc *irq;
pr_err("addtest sgm41510_irq_probe\n");
	irq = devm_gpiod_get_index(sgm->dev, SGM41510_IRQ_PIN, 0, GPIOD_IN);
	if (IS_ERR(irq)) {
		dev_err(sgm->dev, "Could not probe irq pin.\n");
		return PTR_ERR(irq);
	}

	return gpiod_to_irq(irq);
}

static int sgm41510_fw_read_u32_props(struct sgm41510_device *sgm)
{
	int ret;
	u32 property;
	int i;
	struct sgm41510_init_data *init = &sgm->init_data;
	struct {
		char *name;
		bool optional;
		enum sgm41510_table_ids tbl_id;
		u8 *conv_data; /* holds converted value from given property */
	} props[] = {
		/* required properties */
		{"sgm,charge-current", false, TBL_ICHG, &init->ichg},
		{"sgm,battery-regulation-voltage", false, TBL_VREG, &init->vreg},
		{"sgm,termination-current", false, TBL_ITERM, &init->iterm},
		{"sgm,precharge-current", false, TBL_ITERM, &init->iprechg},
		{"sgm,minimum-sys-voltage", false, TBL_SYSVMIN, &init->sysvmin},
		{"sgm,boost-voltage", false, TBL_BOOSTV, &init->boostv},
		{"sgm,boost-max-current", false, TBL_BOOSTI, &init->boosti},

		/* optional properties */
		{"sgm,thermal-regulation-threshold", true, TBL_TREG, &init->treg}
	};
pr_err("addtest sgm41510_fw_read_u32_props\n");
	/* initialize data for optional properties */
	init->treg = 3; /* 120 degrees Celsius */

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = device_property_read_u32(sgm->dev, props[i].name,
					       &property);
		if (ret < 0) {
			if (props[i].optional)
				continue;

			return ret;
		}

		*props[i].conv_data = sgm41510_find_idx(property,
						       props[i].tbl_id);
	}

	return 0;
}

static int sgm41510_fw_probe(struct sgm41510_device *sgm)
{
	int ret;
	struct sgm41510_init_data *init = &sgm->init_data;
pr_err("addtest sgm41510_fw_probe\n");
	ret = sgm41510_fw_read_u32_props(sgm);
	if (ret < 0)
		return ret;

	init->ilim_en = device_property_read_bool(sgm->dev, "sgm,use-ilim-pin");
	init->boostf = device_property_read_bool(sgm->dev, "sgm,boost-low-freq");

	return 0;
}

static int sgm41510_get_dt(struct sgm41510_device *sgm)
{
	int ret = 0;
	if (of_property_read_string(sgm->dev->of_node, "charger_name",
					&sgm->chg_dev_name) < 0) {
		sgm->chg_dev_name = "secondary_chg";
		pr_err("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(sgm->dev->of_node, "alias_name",
				    &(sgm->chg_props.alias_name)) < 0) {
		sgm->chg_props.alias_name = "sgm41510";
		pr_err("%s: no alias name\n", __func__);
	}

	ret = of_get_named_gpio(sgm->dev->of_node, "sgm_en_gpio", 0);
	if (ret < 0 )
	{
		pr_err("%s: Not support sgm en pin\n", __func__);

	}	
	else
		sgm->sgm_en_gpio = ret;
	ret = gpio_request(sgm->sgm_en_gpio, "sgm_en_gpio");	
	
	if(ret)
	{
		pr_err("%s: request otg_en_gpio failed\n", __func__);
		
	}	
	if(gpio_is_valid(sgm->sgm_en_gpio))
		ret = gpio_direction_output(sgm->sgm_en_gpio, 0);
	
	if(ret)
	{
		pr_err("%s: set sgm_en_gpio failed\n", __func__);
	}
	return 0;
}

static int sgm41510_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct sgm41510_device *sgm;
	int ret;
	int i;
	printk("enter:%s\n",__func__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	sgm->client = client;
	sgm->dev = dev;

	sgm41510_get_dt(sgm);
	lc_sgm = sgm;
	mutex_init(&sgm->lock);

	sgm->rmap = devm_regmap_init_i2c(client, &sgm41510_regmap_config);
	if (IS_ERR(sgm->rmap)) {
		dev_err(dev, "failed to allocate register map\n");
		//return PTR_ERR(sgm->rmap);
		ret = PTR_ERR(sgm->rmap);
		goto regmap_error;
	}

	for (i = 0; i < ARRAY_SIZE(sgm41510_reg_fields); i++) {
		const struct reg_field *reg_fields = sgm41510_reg_fields;

		sgm->rmap_fields[i] = devm_regmap_field_alloc(dev, sgm->rmap,
							     reg_fields[i]);
		if (IS_ERR(sgm->rmap_fields[i])) {
			dev_err(dev, "cannot allocate regmap field\n");
			//return PTR_ERR(sgm->rmap_fields[i]);
			ret = PTR_ERR(sgm->rmap_fields[i]);
			goto regmap_error;
		}
	}

	i2c_set_clientdata(client, sgm);

	sgm->chip_id = sgm41510_field_read(sgm, F_PN);
	if (sgm->chip_id < 0) {
		dev_err(dev, "Cannot read chip ID.\n");
		//return sgm->chip_id;
		ret = sgm->chip_id;
		goto regmap_error;
	}
	pr_err("andy sgm41510chip_id=%d.\n",sgm->chip_id);
	if (sgm->chip_id != SGM41510_ID) {
		dev_err(dev, "Chip with ID=%d, not supported!\n", sgm->chip_id);
		//return -ENODEV;
		ret = -ENODEV;
		goto regmap_error;
	}

	if (!dev->platform_data) {
		ret = sgm41510_fw_probe(sgm);
		if (ret < 0) {
			dev_err(dev, "Cannot read device properties.\n");
			//return ret;
			goto regmap_error;
		}
	} else {
		//return -ENODEV;
		ret = -ENODEV;
		goto regmap_error;
	}

	ret = sgm41510_hw_init(sgm);
	if (ret < 0) {
		dev_err(dev, "Cannot initialize the chip.\n");
		//return ret;
		goto regmap_error;
	}
	pr_err("andy client->irq=%d\n",client->irq);

	if (client->irq <= 0)
		client->irq = sgm41510_irq_probe(sgm);
	pr_err("andy 1client->irq=%d\n",client->irq);

	if (client->irq < 0) {
		dev_err(dev, "No irq resource found.\n");
		//return client->irq;
		ret = client->irq;
		goto regmap_error;
	}
	pr_err("andy 3sgm41510client->irq=%d.\n",client->irq);

	/* OTG reporting */
	sgm->usb_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(sgm->usb_phy)) {
		INIT_WORK(&sgm->usb_work, sgm41510_usb_work);
		sgm->usb_nb.notifier_call = sgm41510_usb_notifier;
		usb_register_notifier(sgm->usb_phy, &sgm->usb_nb);
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					sgm41510_irq_handler_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					SGM41510_IRQ_PIN, sgm);
	dev_err(dev, "andy sgm41510client->irq=%d,ret=%d\n",client->irq,ret);
	if (ret)
		goto irq_fail;

	ret = sgm41510_power_supply_init(sgm);
	if (ret < 0) {
		dev_err(dev, "Failed to register power supply\n");
		goto irq_fail;
	}

	/* Register charger device */
	sgm->chg_dev = charger_device_register(sgm->chg_dev_name,
						&client->dev, sgm,
						&sgm41510_chg_ops,
						&sgm->chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		pr_err("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(sgm->chg_dev);
		//return ret;
		goto irq_fail;
	}

	sgm41510_dump_register(sgm->chg_dev);

	
	
	printk("%s,success\n",__func__);
	return 0;

irq_fail:
	if (!IS_ERR_OR_NULL(sgm->usb_phy))
		usb_unregister_notifier(sgm->usb_phy, &sgm->usb_nb);
	
regmap_error:
	devm_kfree(dev,sgm);
	return ret;
}

static int sgm41510_remove(struct i2c_client *client)
{
	struct sgm41510_device *sgm = i2c_get_clientdata(client);

	power_supply_unregister(sgm->charger);

	if (!IS_ERR_OR_NULL(sgm->usb_phy))
		usb_unregister_notifier(sgm->usb_phy, &sgm->usb_nb);

	/* reset all registers to default values */
	sgm41510_chip_reset(sgm);
	devm_kfree(&client->dev,sgm);
	return 0;
}

static void sgm41510_shutdown(struct i2c_client *client)
{
	struct sgm41510_device *sgm = i2c_get_clientdata(client);

	power_supply_unregister(sgm->charger);
	sgm41510_set_en_shippingmode(0x1);

}


#ifdef CONFIG_PM_SLEEP
static int sgm41510_suspend(struct device *dev)
{
	struct sgm41510_device *sgm = dev_get_drvdata(dev);

	/*
	 * If charger is removed, while in suspend, make sure ADC is diabled
	 * since it consumes slightly more power.
	 */
	return sgm41510_field_write(sgm, F_CONV_START, 0);
}

static int sgm41510_resume(struct device *dev)
{
	int ret;
	struct sgm41510_state state;
	struct sgm41510_device *sgm = dev_get_drvdata(dev);

	ret = sgm41510_get_chip_state(sgm, &state);
	if (ret < 0)
		return ret;

	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	/* Re-enable ADC only if charger is plugged in. */
	if (state.online) {
		ret = sgm41510_field_write(sgm, F_CONV_START, 0);
		if (ret < 0)
			return ret;
	}

	/* signal userspace, maybe state changed while suspended */
	//power_supply_changed(sgm->charger);

	return 0;
}
#endif

static const struct dev_pm_ops sgm41510_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(sgm41510_suspend, sgm41510_resume)
};

static const struct i2c_device_id sgm41510_i2c_ids[] = {
	{ "sgm41510", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm41510_i2c_ids);

static const struct of_device_id sgm41510_of_match[] = {
	{ .compatible = "sgm,sgm41510", },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm41510_of_match);

static const struct acpi_device_id sgm41510_acpi_match[] = {
	{"SGM41510", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, sgm41510_acpi_match);

static struct i2c_driver sgm41510_driver = {
	.driver = {
		.name = "sgm41510-charger",
		.of_match_table = of_match_ptr(sgm41510_of_match),
		.acpi_match_table = ACPI_PTR(sgm41510_acpi_match),
		.pm = &sgm41510_pm,
	},
	.probe = sgm41510_probe,
	.remove = sgm41510_remove,
	.shutdown = sgm41510_shutdown,
	.id_table = sgm41510_i2c_ids,
};
module_i2c_driver(sgm41510_driver);

MODULE_AUTHOR("Stuart Su <stuart_su@sg-micro.com>");
MODULE_DESCRIPTION("sgm41510 charger driver");
MODULE_LICENSE("GPL");
