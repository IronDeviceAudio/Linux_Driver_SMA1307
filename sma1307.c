// SPDX-License-Identifier: GPL-2.0-or-later
// sma1307.c -- sma1307 ALSA SoC Audio driver
//
// Copyright 2023 Iron Device Corporation
//
// Auther: Gyuhwa Park <gyuhwa.park@irondevice.com>
//		   Kiseok Jo <kiseok.jo@irondevice.com>

#include <asm/div64.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#if IS_ENABLED(CONFIG_SND_SOC_APS_ALGO)
#include <sound/ff_prot_spk.h>
#endif

#include "sma1307.h"

#define DRIVER_VERSION "V0.0.8"
#define CHECK_PERIOD_TIME 1 /* sec per HZ */
#define GAIN_CONT_5_MIN 30
#define GAIN_CONT_1_MIN 6

#define PLL_MATCH(_input_clk_name, _output_clk_name, _input_clk,\
		_post_n, _n, _vco,  _p_cp)\
{\
	.input_clk_name		= _input_clk_name,\
	.output_clk_name	= _output_clk_name,\
	.input_clk		= _input_clk,\
	.post_n			= _post_n,\
	.n			= _n,\
	.vco			= _vco,\
	.p_cp		= _p_cp,\
}

enum sma1307_type {
	SMA1307,
};

/* PLL clock setting Table */
struct sma1307_pll_match {
	char *input_clk_name;
	char *output_clk_name;
	unsigned int input_clk;
	unsigned int post_n;
	unsigned int n;
	unsigned int vco;
	unsigned int p_cp;
};

struct sma1307_priv {
	atomic_t irq_enabled;
	bool amp_power_status;
	bool force_mute_status;
	char *driver_version;
	enum sma1307_mode amp_mode;
	enum sma1307_setting spk_rcv_mode;
	enum sma1307_type devtype;
	int dapm_aif_in;
	int dapm_aif_out0;
	int dapm_aif_out1;
	int dapm_amp_en;
	int dapm_amp_mode;
	int dapm_sdo_en;
	int dapm_sdo_setting;
	int gpio_int;
	int irq;
	int num_of_pll_matches;
	int retry_cnt;
	long check_fault_period;
	long check_fault_status;
	long isr_manual_mode;
	struct attribute_group *attr_grp;
	struct delayed_work check_fault_work;
	struct device *dev;
	struct kobject *kobj;
	struct mutex pwr_lock;
	struct mutex routing_lock;
	struct regmap *regmap;
	struct sma1307_pll_match *pll_matches;
	unsigned int format;
	unsigned int frame_size;
	unsigned int last_bclk;
	unsigned int otp_trm2;
	unsigned int otp_trm3;
	unsigned int rev_num;
	unsigned int sys_clk_id;
	unsigned int tdm_slot_rx;
	unsigned int tdm_slot_tx;
};

static struct sma1307_pll_match sma1307_pll_matches[] = {
	/* in_clk_name, out_clk_name, input_clk post_n, n, vco, p_cp */
	PLL_MATCH("1.411MHz", "24.554MHz",
					1411200, 0x06, 0xD1, 0x88, 0x00),
	PLL_MATCH("1.536MHz", "24.576MHz",
					1536000, 0x06, 0xC0, 0x88, 0x00),
	PLL_MATCH("2.822MHz", "24.554MHz",
					2822400, 0x06, 0xD1, 0x88, 0x04),
	PLL_MATCH("3.072MHz", "24.576MHz",
					3072000, 0x06, 0x60, 0x88, 0x00),
	PLL_MATCH("6.144MHz", "24.576MHz",
					6144000, 0x06, 0x60, 0x88, 0x04),
	PLL_MATCH("12.288MHz", "24.576MHz",
					12288000, 0x06, 0x60, 0x88, 0x08),
	PLL_MATCH("19.2MHz", "24.48MHz",
					19200000, 0x06, 0x7B, 0x88, 0x0C),
	PLL_MATCH("24.576MHz", "24.576MHz",
					24576000, 0x06, 0x60, 0x88, 0x0C),
};

static struct snd_soc_component *sma1307_amp_component;

static int sma1307_startup(struct snd_soc_component *);
static int sma1307_shutdown(struct snd_soc_component *);
static int sma1307_spk_rcv_conf(struct snd_soc_component *);

static const struct reg_default sma1307_reg_def[] = {
/*	{ reg,  def },	   Register Name */
	{ 0x00, 0x80 }, /* 0x00 SystemCTRL  */
	{ 0x01, 0x00 }, /* 0x01 InputCTRL1  */
	{ 0x02, 0x64 }, /* 0x02 BrownOut Protection1  */
	{ 0x03, 0x5B }, /* 0x03 BrownOut Protection2  */
	{ 0x04, 0x58 }, /* 0x04 BrownOut Protection3  */
	{ 0x05, 0x52 }, /* 0x05 BrownOut Protection8  */
	{ 0x06, 0x4C }, /* 0x06 BrownOut Protection9  */
	{ 0x07, 0x4A }, /* 0x07 BrownOut Protection10  */
	{ 0x08, 0x47 }, /* 0x08 BrownOut Protection11  */
	{ 0x09, 0x2F }, /* 0x09 OutputCTRL  */
	{ 0x0A, 0x31 }, /* 0x0A SPK_VOL  */
	{ 0x0B, 0x50 }, /* 0x0B BST_TEST  */
	{ 0x0C, 0x8C }, /* 0x0C BST_CTRL8  */
	{ 0x0D, 0x00 }, /* 0x0D SPK_TEST  */
	{ 0x0E, 0x3F }, /* 0x0E MUTE_VOL_CTRL  */
	{ 0x0F, 0x08 }, /* 0x0F VBAT_TEMP_SENSING  */
	{ 0x10, 0x04 }, /* 0x10 SystemCTRL1  */
	{ 0x11, 0x00 }, /* 0x11 SystemCTRL2  */
	{ 0x12, 0x00 }, /* 0x12 SystemCTRL3  */
	{ 0x13, 0x09 }, /* 0x13 Delay  */
	{ 0x14, 0x12 }, /* 0x14 Modulator  */
	{ 0x1C, 0x0F }, /* 0x1C BrownOut Protection20  */
	{ 0x1D, 0x05 }, /* 0x1D BrownOut Protection0  */
	{ 0x1E, 0xA1 }, /* 0x1E Tone Generator  */
	{ 0x1F, 0x67 }, /* 0x1F Tone_Fine volume  */
	{ 0x22, 0x00 }, /* 0x22 Compressor Hysteresis  */
	{ 0x23, 0x1F }, /* 0x23 CompLim1  */
	{ 0x24, 0x7A }, /* 0x24 CompLim2  */
	{ 0x25, 0x00 }, /* 0x25 CompLim3  */
	{ 0x26, 0xFF }, /* 0x26 CompLim4  */
	{ 0x27, 0x1B }, /* 0x27 BrownOut Protection4  */
	{ 0x28, 0x1A }, /* 0x28 BrownOut Protection5  */
	{ 0x29, 0x19 }, /* 0x29 BrownOut Protection12  */
	{ 0x2A, 0x78 }, /* 0x2A BrownOut Protection13  */
	{ 0x2B, 0x97 }, /* 0x2B BrownOut Protection14  */
	{ 0x2C, 0xB6 }, /* 0x2C BrownOut Protection15  */
	{ 0x2D, 0xFF }, /* 0x2D BrownOut Protection6  */
	{ 0x2E, 0xFF }, /* 0x2E BrownOut Protection7  */
	{ 0x2F, 0xFF }, /* 0x2F BrownOut Protection16  */
	{ 0x30, 0xFF }, /* 0x30 BrownOut Protection17  */
	{ 0x31, 0xFF }, /* 0x31 BrownOut Protection18  */
	{ 0x32, 0xFF }, /* 0x32 BrownOut Protection19  */
	{ 0x34, 0x00 }, /* 0x34 OCP_SPK  */
	{ 0x35, 0x16 }, /* 0x35 FDPEC Control0  */
	{ 0x36, 0x91 }, /* 0x36 Protection  */
	{ 0x37, 0x00 }, /* 0x37 SlopeCTRL  */
	{ 0x38, 0x01 }, /* 0x38 Power Meter */
	{ 0x39, 0x10 }, /* 0x39 PMT_NZ_VAL */
	{ 0x3E, 0x01 }, /* 0x3E IDLE_MODE_CTRL */
	{ 0x3F, 0x09 }, /* 0x3F ATEST2  */
	{ 0x8B, 0x06 }, /* 0x8B PLL_POST_N  */
	{ 0x8C, 0xC0 }, /* 0x8C PLL_N  */
	{ 0x8D, 0x88 }, /* 0x8D PLL_A_SETTING  */
	{ 0x8E, 0x00 }, /* 0x8E PLL_P_CP  */
	{ 0x8F, 0x02 }, /* 0x8F Analog Test  */
	{ 0x90, 0x02 }, /* 0x90 CrestLim1  */
	{ 0x91, 0x83 }, /* 0x91 CrestLIm2  */
	{ 0x92, 0xB0 }, /* 0x92 FDPEC Control1  */
	{ 0x93, 0x01 }, /* 0x93 INT Control  */
	{ 0x94, 0xA4 }, /* 0x94 Boost Control9  */
	{ 0x95, 0x54 }, /* 0x95 Boost Control10  */
	{ 0x96, 0x57 }, /* 0x96 Boost Control11  */
	{ 0xA2, 0xCC }, /* 0xA2 TOP_MAN1  */
	{ 0xA3, 0x28 }, /* 0xA3 TOP_MAN2  */
	{ 0xA4, 0x40 }, /* 0xA4 TOP_MAN3  */
	{ 0xA5, 0x01 }, /* 0xA5 TDM1  */
	{ 0xA6, 0x41 }, /* 0xA6 TDM2  */
	{ 0xA7, 0x08 }, /* 0xA7 CLK_MON  */
	{ 0xA8, 0x04 }, /* 0xA8 Boost Control1  */
	{ 0xA9, 0x29 }, /* 0xA9 Boost Control2  */
	{ 0xAA, 0x10 }, /* 0xAA Boost Control3  */
	{ 0xAB, 0x11 }, /* 0xAB Boost Control4  */
	{ 0xAC, 0x10 }, /* 0xAC Boost Control5  */
	{ 0xAD, 0x0F }, /* 0xAD Boost Control6  */
	{ 0xAE, 0xCD }, /* 0xAE Boost Control7  */
	{ 0xAF, 0x41 }, /* 0xAF LPF  */
	{ 0xB0, 0x03 }, /* 0xB0 RMS_TC1  */
	{ 0xB1, 0xEF }, /* 0xB1 RMS_TC2  */
	{ 0xB2, 0x03 }, /* 0xB2 AVG_TC1  */
	{ 0xB3, 0xEF }, /* 0xB3 AVG_TC2  */
	{ 0xB4, 0xF3 }, /* 0xB4 PR_VALUE1  */
	{ 0xB5, 0x3D }, /* 0xB5 PR_VALUE2  */
};

static bool sma1307_readable_register(struct device *dev, unsigned int reg)
{
	bool result;

	if (reg > SMA1307_FF_DEVICE_INDEX)
		return false;

	switch (reg) {
	case SMA1307_00_SYSTEM_CTRL ... SMA1307_1F_TONE_FINE_VOLUME:
	case SMA1307_22_COMP_HYS_SEL ... SMA1307_32_BROWN_OUT_PROT19:
	case SMA1307_34_OCP_SPK ... SMA1307_39_PMT_NZ_VAL:
	case SMA1307_3B_TEST1 ... SMA1307_3F_ATEST2:
	case SMA1307_8B_PLL_POST_N ... SMA1307_9A_OTP_TRM3:
	case SMA1307_A0_PAD_CTRL0 ... SMA1307_B5_PRVALUE2:
	case SMA1307_F7_READY_FOR_T_SAR ... SMA1307_FD_STATUS4:
		result = true;
		break;
	case SMA1307_F5_READY_FOR_V_SAR:
		result = true;
		break;
	case SMA1307_FF_DEVICE_INDEX:
		result = true;
		break;
	default:
		result = false;
		break;
	}
	return result;
}

static bool sma1307_writeable_register(struct device *dev, unsigned int reg)
{
	bool result;

	if (reg > SMA1307_FF_DEVICE_INDEX)
		return false;

	switch (reg) {
	case SMA1307_00_SYSTEM_CTRL ... SMA1307_1F_TONE_FINE_VOLUME:
	case SMA1307_22_COMP_HYS_SEL ... SMA1307_32_BROWN_OUT_PROT19:
	case SMA1307_34_OCP_SPK ... SMA1307_39_PMT_NZ_VAL:
	case SMA1307_3B_TEST1 ... SMA1307_3F_ATEST2:
	case SMA1307_8B_PLL_POST_N ... SMA1307_9A_OTP_TRM3:
	case SMA1307_A0_PAD_CTRL0 ... SMA1307_B5_PRVALUE2:
		result = true;
		break;
	default:
		result = false;
		break;
	}
	return result;
}

static bool sma1307_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SMA1307_FA_STATUS1 ... SMA1307_FB_STATUS2:
	case SMA1307_FF_DEVICE_INDEX:
		return true;
	default:
		return false;
	}
}

/* DB scale conversion of speaker volume */
static const DECLARE_TLV_DB_RANGE(sma1307_spk_tlv, 
			0, 168, TLV_DB_MINMAX_ITEM(2400, -6000));

static int sma1307_regmap_write(struct sma1307_priv *sma1307,
			unsigned int reg, unsigned int val)
{
	int ret = 0;
	int cnt = sma1307->retry_cnt;

	while (cnt--) {
		ret = regmap_write(sma1307->regmap, reg, val);
		if (ret < 0) {
			dev_err(sma1307->dev,
				"Failed to write [0x%02X]\n", reg);
		} else
			break;
	}

	return ret;
}

static int sma1307_regmap_update_bits(struct sma1307_priv *sma1307,
	unsigned int reg, unsigned int mask, unsigned int val, bool *change)
{
	int ret = 0;
	int cnt = sma1307->retry_cnt;

	while (cnt--) {
		ret = regmap_update_bits_check(sma1307->regmap, reg,
				mask, val, change);
		if (ret < 0) {
			dev_err(sma1307->dev,
				"Failed to update [0x%02X]\n", reg);
		} else
			break;
	}

	return ret;
}

static int sma1307_regmap_read(struct sma1307_priv *sma1307,
			unsigned int reg, unsigned int *val)
{
	int ret = 0;
	int cnt = sma1307->retry_cnt;

	while (cnt--) {
		ret = regmap_read(sma1307->regmap, reg, val);
		if (ret < 0) {
			dev_err(sma1307->dev,
				"Failed to read [0x%02X]\n", reg);
		} else
			break;
	}

	return ret;
}

static const char * const sma1307_aif_in_source_text[] = {
	"Mono", "Left", "Right"};
static const char * const sma1307_sdo_setting_text[] = {
	"Data_One_48k", "Data_Two_48k", "Data_Two_24k",
	"Clk_PLL", "Clk_OSC"};
static const char * const sma1307_aif_out_source_text[] = {
	"Disable", "After_FmtC", "After_Mixer", "After_DSP",
	"Vrms2_Avg", "Battery", "Temperature", "After_Delay"};
static const char * const sma1307_amp_mode_text[] = {
	"Speaker_Mode1", "Speaker_Mode2", "Speaker_Mode3",
	"Speaker_Mode4", "Receiver_Mode1", "Receiver_Mode2"};
static const char * const sma1307_tdm_slot_text[] = {
	"Slot0", "Slot1", "Slot2", "Slot3",
	"Slot4", "Slot5", "Slot6", "Slot7"};

static const struct soc_enum sma1307_aif_in_source_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1307_aif_in_source_text),
			sma1307_aif_in_source_text);
static const struct soc_enum sma1307_sdo_setting_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1307_sdo_setting_text),
			sma1307_sdo_setting_text);
static const struct soc_enum sma1307_aif_out_source_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1307_aif_out_source_text),
			sma1307_aif_out_source_text);
static const struct soc_enum sma1307_amp_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1307_amp_mode_text),
			sma1307_amp_mode_text);
static const struct soc_enum sma1307_tdm_slot_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1307_tdm_slot_text),
			sma1307_tdm_slot_text);

static int sma1307_force_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = (int)sma1307->force_mute_status;
	dev_info(sma1307->dev, "%s : Force Mute %s\n", __func__,
			sma1307->force_mute_status ? "ON" : "OFF");

	return 0;
}

static int sma1307_force_mute_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	bool change = false, val = (bool)ucontrol->value.integer.value[0];

	if (sma1307->force_mute_status == val)
		change = false;
	else {
		change = true;
		sma1307->force_mute_status = val;
	}
	dev_info(sma1307->dev, "%s : Force Mute %s\n", __func__,
			sma1307->force_mute_status ? "ON" : "OFF");

	return change;
}

static int sma1307_tdm_slot_rx_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int val, ret;

	ret = sma1307_regmap_read(sma1307, SMA1307_A5_TDM1, &val);
	if (ret < 0)
		return -EINVAL;

	ucontrol->value.integer.value[0] = (val & 0x38) >> 3;
	sma1307->tdm_slot_rx = ucontrol->value.integer.value[0];

	return 0;
}

static int sma1307_tdm_slot_rx_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret, val = (int)ucontrol->value.integer.value[0];
	bool change;

	ret = sma1307_regmap_update_bits(sma1307,
			SMA1307_A5_TDM1, 0x38, (val << 3), &change);
	if (ret < 0)
		return -EINVAL;

	return change;
}

static int sma1307_tdm_slot_tx_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int val, ret;

	ret = sma1307_regmap_read(sma1307, SMA1307_A6_TDM2, &val);
	if (ret < 0)
		return -EINVAL;

	ucontrol->value.integer.value[0] = (val & 0x38) >> 3;
	sma1307->tdm_slot_tx = ucontrol->value.integer.value[0];

	return 0;
}

static int sma1307_tdm_slot_tx_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret, val = (int)ucontrol->value.integer.value[0];
	bool change;

	ret = sma1307_regmap_update_bits(sma1307,
			SMA1307_A6_TDM2, 0x38, (val << 3), &change);
	if (ret < 0)
		return -EINVAL;

	return change;
}

static int sma1307_register_read(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int val, ret;
	int reg = (int)ucontrol->value.bytes.data[0];

	if (sma1307_readable_register(sma1307->dev, reg) == false) {
		dev_err(sma1307->dev,
			"%s : unreadable register [0x%02X]\n",
				__func__, reg);
		return -EINVAL;
	}

	ret = sma1307_regmap_read(sma1307, reg, &val);
	if (ret < 0)
		return -EINVAL;

	ucontrol->value.bytes.data[1] = val;

	return 0;
}

static int sma1307_register_write(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret, origin;
	int reg = (int)ucontrol->value.bytes.data[0];
	int val = (int)ucontrol->value.bytes.data[1];

	if (sma1307_writeable_register(sma1307->dev, reg) == false) {
		dev_err(sma1307->dev,
			"%s : unwriteable register [0x%02X]\n",
				__func__, reg);
		return -EINVAL;
	}

	ret = sma1307_regmap_read(sma1307, reg, &origin);
	if (ret < 0)
		return -EINVAL;
	if (origin == val)
		return false;

	ret = sma1307_regmap_write(sma1307, reg, val);
	if (ret < 0)
		return -EINVAL;
	dev_info(sma1307->dev,
		"%s : write register - 0x%02X:0x%02X\n",
			__func__, reg, val);

	return true;
}

static int sma1307_startup(struct snd_soc_component *component)
{
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	bool change = false, temp = false;

	sma1307_regmap_update_bits(sma1307, SMA1307_A2_TOP_MAN1,
			SMA1307_PLL_MASK, SMA1307_PLL_ON, &temp);
	if (temp == true)
		change = true;

	sma1307_regmap_update_bits(sma1307, SMA1307_00_SYSTEM_CTRL,
			SMA1307_POWER_MASK, SMA1307_POWER_ON, &temp);
	if (temp == true)
		change = true;

	if (sma1307->amp_mode == SMA1307_MONO_MODE) {
		sma1307_regmap_update_bits(sma1307,
				SMA1307_10_SYSTEM_CTRL1,
				SMA1307_SPK_MODE_MASK,
				SMA1307_SPK_MONO,
				&temp);
		if (temp == true)
			change = true;

	} else {
		sma1307_regmap_update_bits(sma1307,
				SMA1307_10_SYSTEM_CTRL1,
				SMA1307_SPK_MODE_MASK,
				SMA1307_SPK_STEREO,
				&temp);
		if (temp == true)
			change = true;
	}

	if (!sma1307->force_mute_status) {
		sma1307_regmap_update_bits(sma1307,
				SMA1307_0E_MUTE_VOL_CTRL,
				SMA1307_SPK_MUTE_MASK,
				SMA1307_SPK_UNMUTE,
				&temp);
		if (temp == true)
			change = true;
	} else {
		dev_info(sma1307->dev,
				"%s : FORCE MUTE!!!\n", __func__);
	}

	if (!gpio_is_valid(sma1307->gpio_int)
		&& (sma1307->check_fault_status)) {
		if (sma1307->check_fault_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma1307->check_fault_work,
					sma1307->check_fault_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma1307->check_fault_work,
					CHECK_PERIOD_TIME * HZ);
	} else {
		sma1307_regmap_update_bits(sma1307,
			SMA1307_93_INT_CTRL,
			SMA1307_DIS_INT_MASK,
			SMA1307_NORMAL_INT,
			&temp);
		if (temp == true)
			change = true;
	}
	sma1307->amp_power_status = true;

	if (sma1307->isr_manual_mode) {
		sma1307_regmap_update_bits(sma1307,
				SMA1307_93_INT_CTRL,
				SMA1307_CLR_INT_MASK,
				SMA1307_INT_CLEAR,
				&temp);
		if (temp == true)
			change = true;
		sma1307_regmap_update_bits(sma1307,
				SMA1307_93_INT_CTRL,
				SMA1307_CLR_INT_MASK,
				SMA1307_INT_READY,
				&temp);
		if (temp == true)
			change = true;
		sma1307_regmap_update_bits(sma1307,
				SMA1307_93_INT_CTRL,
				SMA1307_SEL_INT_MASK,
				SMA1307_INT_CLEAR_MANUAL,
				&temp);
		if (temp == true)
			change = true;
	}

	return change;
}

static int sma1307_shutdown(struct snd_soc_component *component)
{
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	bool change = false, temp = false;

	/* for SMA1307A */
	cancel_delayed_work_sync(&sma1307->check_fault_work);

	sma1307_regmap_update_bits(sma1307, SMA1307_0E_MUTE_VOL_CTRL,
			SMA1307_SPK_MUTE_MASK, 	SMA1307_SPK_MUTE, &temp);
	if (temp == true)
		change = true;

	/* Need to wait time for mute slope */
	msleep(55);

	sma1307_regmap_update_bits(sma1307, SMA1307_10_SYSTEM_CTRL1,
			SMA1307_SPK_MODE_MASK, SMA1307_SPK_OFF, &temp);
	if (temp == true)
		change = true;

	sma1307_regmap_update_bits(sma1307, SMA1307_A2_TOP_MAN1,
			SMA1307_PLL_MASK, SMA1307_PLL_OFF, &temp);
	if (temp == true)
		change = true;

	sma1307_regmap_update_bits(sma1307, SMA1307_00_SYSTEM_CTRL,
			SMA1307_POWER_MASK, SMA1307_POWER_OFF, &temp);
	if (temp == true)
		change = true;

	sma1307_regmap_update_bits(sma1307,
			SMA1307_93_INT_CTRL,
			SMA1307_DIS_INT_MASK,
			SMA1307_HIGH_Z_INT,
			&temp);
	if (temp == true)
		change = true;

	sma1307->amp_power_status = false;

	return change;
}

static int sma1307_aif_in_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	unsigned int mux = sma1307->dapm_aif_in;
	int ret = 0;
	bool change = false, temp = false;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		switch (mux) {
		case SMA1307_MONO_MODE:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_11_SYSTEM_CTRL2,
					SMA1307_MONOMIX_MASK,
					SMA1307_MONOMIX_ON,
					&change);
			break;
		case SMA1307_LEFT_MODE:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_11_SYSTEM_CTRL2,
					SMA1307_MONOMIX_MASK,
					SMA1307_MONOMIX_OFF,
					&temp);
			if (temp == true)
				change = true;
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_11_SYSTEM_CTRL2,
					SMA1307_LR_DATA_SW_MASK,
					SMA1307_LR_DATA_SW_NORMAL,
					&temp);
			if (temp == true)
				change = true;
			break;
		case SMA1307_RIGHT_MODE:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_11_SYSTEM_CTRL2,
					SMA1307_MONOMIX_MASK,
					SMA1307_MONOMIX_OFF,
					&temp);
			if (temp == true)
				change = true;
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_11_SYSTEM_CTRL2,
					SMA1307_LR_DATA_SW_MASK,
					SMA1307_LR_DATA_SW_SWAP,
					&temp);
			if (temp == true)
				change = true;
			break;
		default:
			dev_err(sma1307->dev, "%s : Invalid value (%d)\n",
								__func__, mux);
			return -EINVAL;
		}
		sma1307->amp_mode = mux;
		dev_info(sma1307->dev, "%s : Source : %s\n", __func__,
					sma1307_aif_in_source_text[mux]);
		break;
	}
	if (ret < 0)
		return -EINVAL;
	return change;
}

static int sma1307_sdo_setting_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	unsigned int mux = sma1307->dapm_sdo_setting;
	int ret = 0;
	bool change = false, temp = false;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		switch (mux) {
		case SMA1307_OUT_DATA_ONE_48K:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A2_TOP_MAN1,
					SMA1307_SDO_OUTPUT2_MASK,
					SMA1307_ONE_SDO_PER_CH,
					&temp);
			if (temp == true)
				change = true;

			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A3_TOP_MAN2,
					SMA1307_SDO_OUTPUT3_MASK
					| SMA1307_DATA_CLK_SEL_MASK,
					SMA1307_SDO_OUTPUT3_DIS
					| SMA1307_SDO_DATA,
					&temp);
			if (temp == true)
				change = true;

			break;
		case SMA1307_OUT_DATA_TWO_48K:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A2_TOP_MAN1,
					SMA1307_SDO_OUTPUT2_MASK,
					SMA1307_TWO_SDO_PER_CH,
					&temp);
			if (temp == true)
				change = true;

			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A3_TOP_MAN2,
					SMA1307_SDO_OUTPUT3_MASK
					| SMA1307_DATA_CLK_SEL_MASK,
					SMA1307_SDO_OUTPUT3_DIS
					| SMA1307_SDO_DATA,
					&temp);
			if (temp == true)
				change = true;

			break;
		case SMA1307_OUT_DATA_TWO_24K:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A2_TOP_MAN1,
					SMA1307_SDO_OUTPUT2_MASK,
					SMA1307_TWO_SDO_PER_CH,
					&temp);
			if (temp == true)
				change = true;

			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A3_TOP_MAN2,
					SMA1307_SDO_OUTPUT3_MASK
					| SMA1307_DATA_CLK_SEL_MASK,
					SMA1307_TWO_SDO_PER_CH_24K
					| SMA1307_SDO_DATA,
					&temp);
			if (temp == true)
				change = true;

			break;
		case SMA1307_OUT_CLK_PLL:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A3_TOP_MAN2,
					SMA1307_DATA_CLK_SEL_MASK,
					SMA1307_SDO_CLK_PLL,
					&temp);
			if (temp == true)
				change = true;

			break;
		case SMA1307_OUT_CLK_OSC:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A3_TOP_MAN2,
					SMA1307_DATA_CLK_SEL_MASK,
					SMA1307_SDO_CLK_OSC,
					&temp);

			break;
		default:
			dev_err(sma1307->dev, "%s : Invalid value (%d)\n",
								__func__, mux);
			return -EINVAL;
		}

		dev_info(sma1307->dev, "%s : %s\n", __func__,
					sma1307_sdo_setting_text[mux]);
		break;
	}
	if (ret < 0)
		return -EINVAL;
	return change;
}

static int sma1307_aif_out0_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	unsigned int mux = sma1307->dapm_aif_out0;
	int ret = 0;
	bool change = false;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		switch (mux) {
		case SMA1307_OUT_DISABLE:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_DISABLE,
					&change);
			break;
		case SMA1307_OUT_FORMAT_C:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_FORMAT_C,
					&change);
			break;
		case SMA1307_OUT_MIXER_OUT:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_MONO_MIX,
					&change);
			break;
		case SMA1307_OUT_AFTER_DSP:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_AFTER_DSP,
					&change);
			break;
		case SMA1307_OUT_VRMS2_AVG:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_VRMS2_AVG,
					&change);
			break;
		case SMA1307_OUT_BATTERY:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_VBAT_MON,
					&change);
			break;
		case SMA1307_OUT_TEMP:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_TEMP_MON,
					&change);
			break;
		case SMA1307_OUT_AFTER_DELAY:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT0_SEL_MASK,
					SMA1307_SDO0_AFTER_DELAY,
					&change);
			break;
		default:
			dev_err(sma1307->dev, "%s : Invalid value (%d)\n",
								__func__, mux);
			return -EINVAL;
		}

		dev_info(sma1307->dev, "%s : Source : %s\n", __func__,
					sma1307_aif_out_source_text[mux]);
		break;
	}
	if (ret < 0)
		return -EINVAL;
	return change;
}

static int sma1307_aif_out1_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	unsigned int mux = sma1307->dapm_aif_out1;
	int ret = 0;
	bool change = false;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		switch (mux) {
		case SMA1307_OUT_DISABLE:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_DISABLE,
					&change);
			break;
		case SMA1307_OUT_FORMAT_C:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_FORMAT_C,
					&change);
			break;
		case SMA1307_OUT_MIXER_OUT:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_MONO_MIX,
					&change);
			break;
		case SMA1307_OUT_AFTER_DSP:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_AFTER_DSP,
					&change);
			break;
		case SMA1307_OUT_VRMS2_AVG:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_VRMS2_AVG,
					&change);
			break;
		case SMA1307_OUT_BATTERY:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_VBAT_MON,
					&change);
			break;
		case SMA1307_OUT_TEMP:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_TEMP_MON,
					&change);
			break;
		case SMA1307_OUT_AFTER_DELAY:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_09_OUTPUT_CTRL,
					SMA1307_SDO_OUT1_SEL_MASK,
					SMA1307_SDO1_AFTER_DELAY,
					&change);
			break;
		default:
			dev_err(sma1307->dev, "%s : Invalid value (%d)\n",
								__func__, mux);
			return -EINVAL;
		}

		dev_info(sma1307->dev, "%s : Source : %s\n", __func__,
					sma1307_aif_out_source_text[mux]);
		break;
	}
	if (ret < 0)
		return -EINVAL;
	return change;
}

static int sma1307_amp_mode_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int origin = sma1307->spk_rcv_mode;
	bool change = false;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		sma1307->spk_rcv_mode = sma1307->dapm_amp_mode;
		if (origin != sma1307->spk_rcv_mode) {
			sma1307_spk_rcv_conf(component);
			change = true;
		}
		break;
	}

	return change;
}

static int sma1307_sdo_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret = 0;
	bool change = false, temp = false;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(sma1307->dev,
			"%s : SND_SOC_DAPM_PRE_PMU\n", __func__);
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_09_OUTPUT_CTRL,
				SMA1307_PORT_CONFIG_MASK,
				SMA1307_OUTPUT_PORT_ENABLE,
				&temp);
		if (temp == true)
			change = true;
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_A3_TOP_MAN2,
				SMA1307_SDO_OUTPUT_MASK,
				SMA1307_LOGIC_OUTPUT,
				&temp);
		if (temp == true)
			change = true;
		break;
	case SND_SOC_DAPM_POST_PMD:
		dev_info(sma1307->dev,
			"%s : SND_SOC_DAPM_POST_PMD\n", __func__);
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_09_OUTPUT_CTRL,
				SMA1307_PORT_CONFIG_MASK,
				SMA1307_INPUT_PORT_ONLY,
				&temp);
		if (temp == true)
			change = true;
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_A3_TOP_MAN2,
				SMA1307_SDO_OUTPUT_MASK,
				SMA1307_HIGH_Z_OUTPUT,
				&temp);
		if (temp == true)
			change = true;
		break;
	}
	if (ret < 0)
		return -EINVAL;
	return change;
}

static int sma1307_power_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(sma1307->dev,
			"%s : SND_SOC_DAPM_POST_PMU\n", __func__);
		ret = sma1307_startup(component);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(sma1307->dev,
			"%s : SND_SOC_DAPM_PRE_PMD\n", __func__);
		ret = sma1307_shutdown(component);
		break;
	}
	return ret;
}

static int sma1307_dapm_aif_in_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
		snd_soc_component_get_drvdata(dapm->component);

	ucontrol->value.enumerated.item[0] =
				(unsigned int)sma1307->dapm_aif_in;
	dev_info(sma1307->dev, "%s : AIF IN %s\n", __func__,
		sma1307_aif_in_source_text[sma1307->dapm_aif_in]);
	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return 0;
}

static int sma1307_dapm_aif_in_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
			snd_soc_component_get_drvdata(dapm->component);
	int val = (int)ucontrol->value.enumerated.item[0];
	bool change;

	if ((val < 0) ||
		(val >= ARRAY_SIZE(sma1307_aif_in_source_text))) {
		dev_err(sma1307->dev, "%s : Out of range\n", __func__);
		return -EINVAL;
	}

	if (sma1307->dapm_aif_in != val) {
		change = true;
		sma1307->dapm_aif_in = val;
		dev_info(sma1307->dev, "%s : AIF IN %s\n", __func__,
		sma1307_aif_in_source_text[sma1307->dapm_aif_in]);
	} else
		change = false;

	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return change;
}

static int sma1307_dapm_sdo_setting_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
		snd_soc_component_get_drvdata(dapm->component);

	ucontrol->value.enumerated.item[0] =
				(unsigned int)sma1307->dapm_sdo_setting;
	dev_info(sma1307->dev, "%s : SDO Setting %s\n", __func__,
		sma1307_sdo_setting_text[sma1307->dapm_sdo_setting]);
	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return 0;
}

static int sma1307_dapm_sdo_setting_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
			snd_soc_component_get_drvdata(dapm->component);
	int val = (int)ucontrol->value.enumerated.item[0];
	bool change;

	if ((val < 0) ||
		(val >= ARRAY_SIZE(sma1307_sdo_setting_text))) {
		dev_err(sma1307->dev, "%s : Out of range\n", __func__);
		return -EINVAL;
	}

	if (sma1307->dapm_sdo_setting != val) {
		change = true;
		sma1307->dapm_sdo_setting = val;
		dev_info(sma1307->dev, "%s : SDO Setting %s\n", __func__,
		sma1307_sdo_setting_text[sma1307->dapm_sdo_setting]);
	} else
		change = false;

	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return change;
}

static int sma1307_dapm_aif_out0_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
		snd_soc_component_get_drvdata(dapm->component);

	ucontrol->value.enumerated.item[0] =
				(unsigned int)sma1307->dapm_aif_out0;
	dev_info(sma1307->dev, "%s : AIF OUT1 %s\n", __func__,
		sma1307_aif_out_source_text[sma1307->dapm_aif_out0]);
	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return 0;
}

static int sma1307_dapm_aif_out0_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
			snd_soc_component_get_drvdata(dapm->component);
	int val = (int)ucontrol->value.enumerated.item[0];
	bool change;

	if ((val < 0) ||
		(val >= ARRAY_SIZE(sma1307_aif_out_source_text))) {
		dev_err(sma1307->dev, "%s : Out of range\n", __func__);
		return -EINVAL;
	}

	if (sma1307->dapm_aif_out0 != val) {
		change = true;
		sma1307->dapm_aif_out0 = val;
		dev_info(sma1307->dev, "%s : AIF OUT1 %s\n", __func__,
		sma1307_aif_out_source_text[sma1307->dapm_aif_out0]);
	} else
		change = false;

	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return change;
}

static int sma1307_dapm_aif_out1_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
		snd_soc_component_get_drvdata(dapm->component);

	ucontrol->value.enumerated.item[0] =
				(unsigned int)sma1307->dapm_aif_out1;
	dev_info(sma1307->dev, "%s : AIF OUT1 %s\n", __func__,
		sma1307_aif_out_source_text[sma1307->dapm_aif_out1]);
	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return 0;
}

static int sma1307_dapm_aif_out1_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
			snd_soc_component_get_drvdata(dapm->component);
	int val = (int)ucontrol->value.enumerated.item[0];
	bool change;

	if ((val < 0) ||
		(val >= ARRAY_SIZE(sma1307_aif_out_source_text))) {
		dev_err(sma1307->dev, "%s : Out of range\n", __func__);
		return -EINVAL;
	}

	if (sma1307->dapm_aif_out1 != val) {
		change = true;
		sma1307->dapm_aif_out1 = val;
		dev_info(sma1307->dev, "%s : AIF OUT1 %s\n", __func__,
		sma1307_aif_out_source_text[sma1307->dapm_aif_out1]);
	} else
		change = false;

	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return change;
}

static int sma1307_dapm_amp_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
		snd_soc_component_get_drvdata(dapm->component);

	ucontrol->value.enumerated.item[0] =
				(unsigned int)sma1307->dapm_amp_mode;
	dev_info(sma1307->dev, "%s : AMP Mode %s\n", __func__,
			sma1307_amp_mode_text[sma1307->dapm_amp_mode]);
	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return 0;
}

static int sma1307_dapm_amp_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
			snd_soc_component_get_drvdata(dapm->component);
	int val = (int)ucontrol->value.enumerated.item[0];
	bool change;

	if ((val < 0) ||
		(val >= ARRAY_SIZE(sma1307_amp_mode_text))) {
		dev_err(sma1307->dev, "%s : Out of range\n", __func__);
		return -EINVAL;
	}

	if (sma1307->dapm_amp_mode != val) {
		change = true;
		sma1307->dapm_amp_mode = val;
		dev_info(sma1307->dev, "%s : AMP Mode %s\n", __func__,
			sma1307_amp_mode_text[sma1307->dapm_amp_mode]);
	} else
		change = false;

	snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	return change;
}

static int sma1307_dapm_sdo_enable_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
		snd_soc_component_get_drvdata(dapm->component);

	ucontrol->value.integer.value[0] = (long)sma1307->dapm_sdo_en;
	dev_info(sma1307->dev, "%s : SDO Enable %s\n", __func__,
				sma1307->dapm_sdo_en ? "ON" : "OFF");
	snd_soc_dapm_put_volsw(kcontrol, ucontrol);

	return 0;
}

static int sma1307_dapm_sdo_enable_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
			snd_soc_component_get_drvdata(dapm->component);
	int val = (int)ucontrol->value.integer.value[0];
	bool change;

	if ((val < 0) || (val > 1)) {
		dev_err(sma1307->dev, "%s : Out of range\n", __func__);
		return -EINVAL;
	}

	if (sma1307->dapm_sdo_en != val) {
		change = true;
		sma1307->dapm_sdo_en = val;
		dev_info(sma1307->dev, "%s : SDO Enable %s\n", __func__,
			sma1307->dapm_sdo_en ? "ON" : "OFF");
	} else
		change = false;

	snd_soc_dapm_put_volsw(kcontrol, ucontrol);

	return change;
}

static int sma1307_dapm_amp_enable_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
		snd_soc_component_get_drvdata(dapm->component);

	ucontrol->value.integer.value[0] = (long)sma1307->dapm_amp_en;
	dev_info(sma1307->dev, "%s : AMP Enable %s\n", __func__,
				sma1307->dapm_amp_en ? "ON" : "OFF");
	snd_soc_dapm_put_volsw(kcontrol, ucontrol);

	return 0;
}

static int sma1307_dapm_amp_enable_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct sma1307_priv *sma1307 =
			snd_soc_component_get_drvdata(dapm->component);
	int val = (int)ucontrol->value.integer.value[0];
	bool change;

	if ((val < 0) || (val > 1)) {
		dev_err(sma1307->dev, "%s : Out of range\n", __func__);
		return -EINVAL;
	}

	if (sma1307->dapm_amp_en != val) {
		change = true;
		sma1307->dapm_amp_en = val;
		dev_info(sma1307->dev, "%s : AMP Enable %s\n", __func__,
			sma1307->dapm_amp_en ? "ON" : "OFF");
	} else
		change = false;

	snd_soc_dapm_put_volsw(kcontrol, ucontrol);

	return change;
}

static const struct snd_kcontrol_new sma1307_aif_in_source_control =
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AIF IN Source",
		.info = snd_soc_info_enum_double,
		.get = sma1307_dapm_aif_in_get,
		.put = sma1307_dapm_aif_in_put,
		.private_value =
			(unsigned long)&sma1307_aif_in_source_enum
	};
static const struct snd_kcontrol_new sma1307_sdo_setting_control =
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "SDO Setting",
		.info = snd_soc_info_enum_double,
		.get = sma1307_dapm_sdo_setting_get,
		.put = sma1307_dapm_sdo_setting_put,
		.private_value =
			(unsigned long)&sma1307_sdo_setting_enum
	};
static const struct snd_kcontrol_new sma1307_aif_out0_source_control =
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AIF OUT0 Source",
		.info = snd_soc_info_enum_double,
		.get = sma1307_dapm_aif_out0_get,
		.put = sma1307_dapm_aif_out0_put,
		.private_value =
			(unsigned long)&sma1307_aif_out_source_enum
	};
static const struct snd_kcontrol_new sma1307_aif_out1_source_control =
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AIF OUT1 Source",
		.info = snd_soc_info_enum_double,
		.get = sma1307_dapm_aif_out1_get,
		.put = sma1307_dapm_aif_out1_put,
		.private_value =
			(unsigned long)&sma1307_aif_out_source_enum
	};
static const struct snd_kcontrol_new sma1307_amp_mode_control =
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AMP Mode",
		.info = snd_soc_info_enum_double,
		.get = sma1307_dapm_amp_mode_get,
		.put = sma1307_dapm_amp_mode_put,
		.private_value = (unsigned long)&sma1307_amp_mode_enum
	};
static const struct snd_kcontrol_new sma1307_sdo_control =
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Switch",
		.info = snd_soc_info_volsw,
		.get = sma1307_dapm_sdo_enable_get,
		.put = sma1307_dapm_sdo_enable_put,
		.private_value = SOC_SINGLE_VALUE(SND_SOC_NOPM, 0, 1, 0, 0)
	};
static const struct snd_kcontrol_new sma1307_enable_control =
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Switch",
		.info = snd_soc_info_volsw,
		.get = sma1307_dapm_amp_enable_get,
		.put = sma1307_dapm_amp_enable_put,
		.private_value = SOC_SINGLE_VALUE(SND_SOC_NOPM, 0, 1, 0, 0)
	};

static const struct snd_kcontrol_new sma1307_snd_controls[] = {
	SND_SOC_BYTES_EXT("Register Byte Control", 2,
			sma1307_register_read, sma1307_register_write),
	SOC_SINGLE_TLV("Speaker Volume", SMA1307_0A_SPK_VOL,
			0, 168, 0, sma1307_spk_tlv),
	SOC_SINGLE_BOOL_EXT("Force Mute Switch", 0,
			sma1307_force_mute_get, sma1307_force_mute_put),
	SOC_ENUM_EXT("TDM RX Slot Position", sma1307_tdm_slot_enum,
			sma1307_tdm_slot_rx_get, sma1307_tdm_slot_rx_put),
	SOC_ENUM_EXT("TDM TX Slot Position", sma1307_tdm_slot_enum,
			sma1307_tdm_slot_tx_get, sma1307_tdm_slot_tx_put),
};

static const struct snd_soc_dapm_widget sma1307_dapm_widgets[] = {
	/* platform domain */
	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_INPUT("SDO"),

	/* path domain */
	SND_SOC_DAPM_MUX_E("AIF IN Source", SND_SOC_NOPM, 0, 0,
			&sma1307_aif_in_source_control,
			sma1307_aif_in_event,
			SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_MUX_E("SDO Setting", SND_SOC_NOPM, 0, 0,
			&sma1307_sdo_setting_control,
			sma1307_sdo_setting_event,
			SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_MUX_E("AIF OUT0 Source", SND_SOC_NOPM, 0, 0,
			&sma1307_aif_out0_source_control,
			sma1307_aif_out0_event,
			SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_MUX_E("AIF OUT1 Source", SND_SOC_NOPM, 0, 0,
			&sma1307_aif_out1_source_control,
			sma1307_aif_out1_event,
			SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_MUX_E("AMP Mode", SND_SOC_NOPM, 0, 0,
			&sma1307_amp_mode_control,
			sma1307_amp_mode_event,
			SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_SWITCH_E("SDO Enable", SND_SOC_NOPM, 0, 0,
			&sma1307_sdo_control,
			sma1307_sdo_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("Entry", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_OUT_DRV_E("AMP Power", SND_SOC_NOPM, 0, 0, NULL, 0,
			sma1307_power_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* stream domain */
	SND_SOC_DAPM_AIF_IN("AIF IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route sma1307_audio_map[] = {
	/* Playback */
	{"AIF IN Source", "Mono", "AIF IN"},
	{"AIF IN Source", "Left", "AIF IN"},
	{"AIF IN Source", "Right", "AIF IN"},

	{"SDO Enable", "Switch", "AIF IN"},

	{"SDO Setting", "Data_One_48k", "SDO Enable"},
	{"SDO Setting", "Data_Two_48k", "SDO Enable"},
	{"SDO Setting", "Data_Two_24k", "SDO Enable"},
	{"SDO Setting", "Clk_PLL", "SDO Enable"},
	{"SDO Setting", "Clk_OSC", "SDO Enable"},

	{"AIF OUT0 Source", "Disable", "SDO Setting"},
	{"AIF OUT0 Source", "After_FmtC", "SDO Setting"},
	{"AIF OUT0 Source", "After_Mixer", "SDO Setting"},
	{"AIF OUT0 Source", "After_DSP", "SDO Setting"},
	{"AIF OUT0 Source", "Vrms2_Avg", "SDO Setting"},
	{"AIF OUT0 Source", "Battery", "SDO Setting"},
	{"AIF OUT0 Source", "Temperature", "SDO Setting"},
	{"AIF OUT0 Source", "After_Delay", "SDO Setting"},

	{"AIF OUT1 Source", "Disable", "SDO Setting"},
	{"AIF OUT1 Source", "After_FmtC", "SDO Setting"},
	{"AIF OUT1 Source", "After_Mixer", "SDO Setting"},
	{"AIF OUT1 Source", "After_DSP", "SDO Setting"},
	{"AIF OUT1 Source", "Vrms2_Avg", "SDO Setting"},
	{"AIF OUT1 Source", "Battery", "SDO Setting"},
	{"AIF OUT1 Source", "Temperature", "SDO Setting"},
	{"AIF OUT1 Source", "After_Delay", "SDO Setting"},

	{"Entry", NULL, "AIF OUT0 Source"},
	{"Entry", NULL, "AIF OUT1 Source"},
	{"Entry", NULL, "AIF IN Source"},

	{"AMP Mode", "Speaker_Mode1", "Entry"},
	{"AMP Mode", "Speaker_Mode2", "Entry"},
	{"AMP Mode", "Speaker_Mode3", "Entry"},
	{"AMP Mode", "Speaker_Mode4", "Entry"},
	{"AMP Mode", "Receiver_Mode1", "Entry"},
	{"AMP Mode", "Receiver_Mode2", "Entry"},

	{"AMP Power", NULL, "AMP Mode"},
	{"SPK", NULL, "AMP Power"},

	/* Capture */
	{"AIF OUT", NULL, "AMP Power"},
};

static int sma1307_spk_rcv_conf(struct snd_soc_component *component)
{
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);

	switch (sma1307->spk_rcv_mode) {
	case SMA1307_RECEIVER_MODE2:
		sma1307_regmap_write(sma1307, SMA1307_0A_SPK_VOL, 0x32);
		sma1307_regmap_write(sma1307, SMA1307_0B_BST_TEST, 0xD0);
		sma1307_regmap_write(sma1307,
				SMA1307_0F_VBAT_TEMP_SENSING, 0xE8);
		sma1307_regmap_write(sma1307, SMA1307_11_SYSTEM_CTRL2, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_13_DELAY, 0x19);
		sma1307_regmap_write(sma1307, SMA1307_14_MODULATOR, 0x5C);
		sma1307_regmap_write(sma1307, SMA1307_1E_TONE_GENERATOR, 0xE1);
		sma1307_regmap_write(sma1307, SMA1307_23_COMPLIM1, 0x1F);
		sma1307_regmap_write(sma1307, SMA1307_24_COMPLIM2, 0x7A);
		sma1307_regmap_write(sma1307, SMA1307_34_OCP_SPK, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_35_FDPEC_CTRL0, 0x40);
		sma1307_regmap_write(sma1307, SMA1307_3E_IDLE_MODE_CTRL, 0x07);
		sma1307_regmap_write(sma1307, SMA1307_8F_ANALOG_TEST, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_92_FDPEC_CTRL1, 0xB0);
		sma1307_regmap_write(sma1307, SMA1307_94_BOOST_CTRL9, 0x91);
		sma1307_regmap_write(sma1307, SMA1307_95_BOOST_CTRL10, 0x74);
		sma1307_regmap_write(sma1307, SMA1307_96_BOOST_CTRL11, 0xFF);
		sma1307_regmap_write(sma1307, SMA1307_A8_BOOST_CTRL1, 0x05);
		sma1307_regmap_write(sma1307, SMA1307_A9_BOOST_CTRL2, 0x27);
		sma1307_regmap_write(sma1307, SMA1307_AB_BOOST_CTRL4, 0x14);
		sma1307_regmap_write(sma1307, SMA1307_AC_BOOST_CTRL5, 0x10);
		sma1307_regmap_write(sma1307, SMA1307_AD_BOOST_CTRL6, 0x10);
		break;
	case SMA1307_RECEIVER_MODE1:
		sma1307_regmap_write(sma1307, SMA1307_0A_SPK_VOL, 0x33);
		sma1307_regmap_write(sma1307, SMA1307_0B_BST_TEST, 0xD0);
		sma1307_regmap_write(sma1307,
				SMA1307_0F_VBAT_TEMP_SENSING, 0xE8);
		sma1307_regmap_write(sma1307, SMA1307_11_SYSTEM_CTRL2, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_13_DELAY, 0x19);
		sma1307_regmap_write(sma1307, SMA1307_14_MODULATOR, 0x5C);
		sma1307_regmap_write(sma1307, SMA1307_1E_TONE_GENERATOR, 0xE1);
		sma1307_regmap_write(sma1307, SMA1307_23_COMPLIM1, 0x1F);
		sma1307_regmap_write(sma1307, SMA1307_24_COMPLIM2, 0x7A);
		sma1307_regmap_write(sma1307, SMA1307_34_OCP_SPK, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_35_FDPEC_CTRL0, 0x45);
		sma1307_regmap_write(sma1307, SMA1307_3E_IDLE_MODE_CTRL, 0x07);
		sma1307_regmap_write(sma1307, SMA1307_8F_ANALOG_TEST, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_92_FDPEC_CTRL1, 0xC0);
		sma1307_regmap_write(sma1307, SMA1307_94_BOOST_CTRL9, 0x91);
		sma1307_regmap_write(sma1307, SMA1307_95_BOOST_CTRL10, 0x74);
		sma1307_regmap_write(sma1307, SMA1307_96_BOOST_CTRL11, 0xFF);
		sma1307_regmap_write(sma1307, SMA1307_A8_BOOST_CTRL1, 0x05);
		sma1307_regmap_write(sma1307, SMA1307_A9_BOOST_CTRL2, 0x27);
		sma1307_regmap_write(sma1307, SMA1307_AB_BOOST_CTRL4, 0x14);
		sma1307_regmap_write(sma1307, SMA1307_AC_BOOST_CTRL5, 0x10);
		sma1307_regmap_write(sma1307, SMA1307_AD_BOOST_CTRL6, 0x10);
		break;
	case SMA1307_SPEAKER_MODE4: //bypass mode
		sma1307_regmap_write(sma1307, SMA1307_0A_SPK_VOL, 0x3B);
		sma1307_regmap_write(sma1307, SMA1307_0B_BST_TEST, 0x50);
		sma1307_regmap_write(sma1307,
				SMA1307_0F_VBAT_TEMP_SENSING, 0x08);
		sma1307_regmap_write(sma1307, SMA1307_11_SYSTEM_CTRL2, 0x21);
		sma1307_regmap_write(sma1307, SMA1307_13_DELAY, 0x19);
		sma1307_regmap_write(sma1307, SMA1307_14_MODULATOR, 0x12);
		sma1307_regmap_write(sma1307, SMA1307_1E_TONE_GENERATOR, 0xA1);
		sma1307_regmap_write(sma1307, SMA1307_23_COMPLIM1, 0x53);
		sma1307_regmap_write(sma1307, SMA1307_24_COMPLIM2, 0x0A);
		sma1307_regmap_write(sma1307, SMA1307_34_OCP_SPK, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_35_FDPEC_CTRL0, 0x16);
		sma1307_regmap_write(sma1307, SMA1307_3E_IDLE_MODE_CTRL, 0x05);
		sma1307_regmap_write(sma1307, SMA1307_8F_ANALOG_TEST, 0x02);
		sma1307_regmap_write(sma1307, SMA1307_92_FDPEC_CTRL1, 0x50);
		sma1307_regmap_write(sma1307, SMA1307_94_BOOST_CTRL9, 0xA4);
		sma1307_regmap_write(sma1307, SMA1307_95_BOOST_CTRL10, 0x54);
		sma1307_regmap_write(sma1307, SMA1307_96_BOOST_CTRL11, 0x57);
		sma1307_regmap_write(sma1307, SMA1307_A8_BOOST_CTRL1, 0x04);
		sma1307_regmap_write(sma1307, SMA1307_A9_BOOST_CTRL2, 0x29);
		sma1307_regmap_write(sma1307, SMA1307_AB_BOOST_CTRL4, 0x11);
		sma1307_regmap_write(sma1307, SMA1307_AC_BOOST_CTRL5, 0x50);
		sma1307_regmap_write(sma1307, SMA1307_AD_BOOST_CTRL6, 0x0F);
		break;
	case SMA1307_SPEAKER_MODE3: //boost mode
		sma1307_regmap_write(sma1307, SMA1307_0A_SPK_VOL, 0x3D);
		sma1307_regmap_write(sma1307, SMA1307_0B_BST_TEST, 0x50);
		sma1307_regmap_write(sma1307,
				SMA1307_0F_VBAT_TEMP_SENSING, 0x08);
		sma1307_regmap_write(sma1307, SMA1307_11_SYSTEM_CTRL2, 0x21);
		sma1307_regmap_write(sma1307, SMA1307_13_DELAY, 0x09);
		sma1307_regmap_write(sma1307, SMA1307_14_MODULATOR, 0x12);
		sma1307_regmap_write(sma1307, SMA1307_1E_TONE_GENERATOR, 0xA1);
		sma1307_regmap_write(sma1307, SMA1307_23_COMPLIM1, 0x50);
		sma1307_regmap_write(sma1307, SMA1307_24_COMPLIM2, 0x0A);
		sma1307_regmap_write(sma1307, SMA1307_34_OCP_SPK, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_35_FDPEC_CTRL0, 0x16);
		sma1307_regmap_write(sma1307, SMA1307_3E_IDLE_MODE_CTRL, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_8F_ANALOG_TEST, 0x02);
		sma1307_regmap_write(sma1307, SMA1307_92_FDPEC_CTRL1, 0xB0);
		sma1307_regmap_write(sma1307, SMA1307_94_BOOST_CTRL9, 0xA4);
		sma1307_regmap_write(sma1307, SMA1307_95_BOOST_CTRL10, 0x54);
		sma1307_regmap_write(sma1307, SMA1307_96_BOOST_CTRL11, 0x57);
		sma1307_regmap_write(sma1307, SMA1307_A8_BOOST_CTRL1, 0x04);
		sma1307_regmap_write(sma1307, SMA1307_A9_BOOST_CTRL2, 0x29);
		sma1307_regmap_write(sma1307, SMA1307_AB_BOOST_CTRL4, 0x11);
		sma1307_regmap_write(sma1307, SMA1307_AC_BOOST_CTRL5, 0x10);
		sma1307_regmap_write(sma1307, SMA1307_AD_BOOST_CTRL6, 0x0F);
		break;
	case SMA1307_SPEAKER_MODE2:
		sma1307_regmap_write(sma1307, SMA1307_0A_SPK_VOL, 0x31);
		sma1307_regmap_write(sma1307, SMA1307_0B_BST_TEST, 0x50);
		sma1307_regmap_write(sma1307,
				SMA1307_0F_VBAT_TEMP_SENSING, 0x08);
		sma1307_regmap_write(sma1307, SMA1307_11_SYSTEM_CTRL2, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_13_DELAY, 0x09);
		sma1307_regmap_write(sma1307, SMA1307_14_MODULATOR, 0x12);
		sma1307_regmap_write(sma1307, SMA1307_1E_TONE_GENERATOR, 0xA1);
		sma1307_regmap_write(sma1307, SMA1307_23_COMPLIM1, 0x1F);
		sma1307_regmap_write(sma1307, SMA1307_24_COMPLIM2, 0x7A);
		sma1307_regmap_write(sma1307, SMA1307_34_OCP_SPK, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_35_FDPEC_CTRL0, 0x16);
		sma1307_regmap_write(sma1307, SMA1307_3E_IDLE_MODE_CTRL, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_8F_ANALOG_TEST, 0x02);
		sma1307_regmap_write(sma1307, SMA1307_92_FDPEC_CTRL1, 0xB0);
		sma1307_regmap_write(sma1307, SMA1307_94_BOOST_CTRL9, 0xA4);
		sma1307_regmap_write(sma1307, SMA1307_95_BOOST_CTRL10, 0x54);
		sma1307_regmap_write(sma1307, SMA1307_96_BOOST_CTRL11, 0x57);
		sma1307_regmap_write(sma1307, SMA1307_A8_BOOST_CTRL1, 0x04);
		sma1307_regmap_write(sma1307, SMA1307_A9_BOOST_CTRL2, 0x29);
		sma1307_regmap_write(sma1307, SMA1307_AB_BOOST_CTRL4, 0x11);
		sma1307_regmap_write(sma1307, SMA1307_AC_BOOST_CTRL5, 0x10);
		sma1307_regmap_write(sma1307, SMA1307_AD_BOOST_CTRL6, 0x0F);
		break;
	case SMA1307_SPEAKER_MODE1:
		sma1307_regmap_write(sma1307, SMA1307_0A_SPK_VOL, 0x32);
		sma1307_regmap_write(sma1307, SMA1307_0B_BST_TEST, 0x50);
		sma1307_regmap_write(sma1307,
				SMA1307_0F_VBAT_TEMP_SENSING, 0x08);
		sma1307_regmap_write(sma1307, SMA1307_11_SYSTEM_CTRL2, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_13_DELAY, 0x09);
		sma1307_regmap_write(sma1307, SMA1307_14_MODULATOR, 0x12);
		sma1307_regmap_write(sma1307, SMA1307_1E_TONE_GENERATOR, 0xA1);
		sma1307_regmap_write(sma1307, SMA1307_23_COMPLIM1, 0x1F);
		sma1307_regmap_write(sma1307, SMA1307_24_COMPLIM2, 0x7A);
		sma1307_regmap_write(sma1307, SMA1307_34_OCP_SPK, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_35_FDPEC_CTRL0, 0x17);
		sma1307_regmap_write(sma1307, SMA1307_3E_IDLE_MODE_CTRL, 0x01);
		sma1307_regmap_write(sma1307, SMA1307_8F_ANALOG_TEST, 0x02);
		sma1307_regmap_write(sma1307, SMA1307_92_FDPEC_CTRL1, 0xC0);
		sma1307_regmap_write(sma1307, SMA1307_94_BOOST_CTRL9, 0x64);
		sma1307_regmap_write(sma1307, SMA1307_95_BOOST_CTRL10, 0x74);
		sma1307_regmap_write(sma1307, SMA1307_96_BOOST_CTRL11, 0xDA);
		sma1307_regmap_write(sma1307, SMA1307_A8_BOOST_CTRL1, 0x04);
		sma1307_regmap_write(sma1307, SMA1307_A9_BOOST_CTRL2, 0x27);
		sma1307_regmap_write(sma1307, SMA1307_AB_BOOST_CTRL4, 0x10);
		sma1307_regmap_write(sma1307, SMA1307_AC_BOOST_CTRL5, 0x10);
		sma1307_regmap_write(sma1307, SMA1307_AD_BOOST_CTRL6, 0x0F);
		sma1307_regmap_write(sma1307, SMA1307_99_OTP_TRM2, 0x00);
		break;
	default:
		dev_err(component->dev, "%s : Invalid value (%d)\n",
					__func__, sma1307->spk_rcv_mode);
		return -EINVAL;
	}

	dev_info(component->dev, "%s : [%s] Mode\n", __func__,
			sma1307_amp_mode_text[sma1307->spk_rcv_mode]);

	return 0;
}

static int sma1307_setup_pll(struct snd_soc_component *component,
		unsigned int bclk)
{
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);

	int i = 0;

	dev_info(component->dev, "%s : BCLK = %dHz\n",
		__func__, bclk);

	if (sma1307->sys_clk_id == SMA1307_PLL_CLKIN_MCLK) {
		dev_info(component->dev, "%s : MCLK is not supported\n",
		__func__);
	} else if (sma1307->sys_clk_id == SMA1307_PLL_CLKIN_BCLK) {
		for (i = 0; i < sma1307->num_of_pll_matches; i++) {
			if (sma1307->pll_matches[i].input_clk == bclk)
				break;
		}
		if (i == sma1307->num_of_pll_matches) {
			dev_info(component->dev, "%s : No matching value between pll table and SCK\n",
				__func__);
			return -EINVAL;
		}

		/* PLL operation, PLL Clock, External Clock,
		 * PLL reference SCK clock
		 */
		sma1307_regmap_update_bits(sma1307,
				SMA1307_A2_TOP_MAN1,
				SMA1307_PLL_MASK,
				SMA1307_PLL_ON,
				NULL);

	}

	sma1307_regmap_write(sma1307, SMA1307_8B_PLL_POST_N,
			sma1307->pll_matches[i].post_n);
	sma1307_regmap_write(sma1307, SMA1307_8C_PLL_N,
			sma1307->pll_matches[i].n);
	sma1307_regmap_write(sma1307, SMA1307_8D_PLL_A_SETTING,
			sma1307->pll_matches[i].vco);
	sma1307_regmap_write(sma1307, SMA1307_8E_PLL_P_CP,
			sma1307->pll_matches[i].p_cp);

	return 0;
}

static int sma1307_dai_hw_params_amp(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	unsigned int bclk = 0;
	int ret = 0;

	if (sma1307->format == SND_SOC_DAIFMT_DSP_A)
		bclk = params_rate(params) * sma1307->frame_size;
	else
		bclk = params_rate(params) * params_physical_width(params)
			* params_channels(params);

	dev_info(component->dev,
			"%s : rate = %d : bit size = %d : channel = %d\n",
			__func__, params_rate(params), params_width(params),
			params_channels(params));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (sma1307->sys_clk_id == SMA1307_PLL_CLKIN_BCLK) {
			if (sma1307->last_bclk != bclk) {
				sma1307_setup_pll(component, bclk);
				sma1307->last_bclk = bclk;
			}
		}

		if (gpio_is_valid(sma1307->gpio_int)
			&& (!atomic_read(&sma1307->irq_enabled))) {
			enable_irq((unsigned int)sma1307->irq);
			irq_set_irq_wake(sma1307->irq, 1);

			if (device_may_wakeup(sma1307->dev))
				enable_irq_wake(sma1307->irq);

			atomic_set(&sma1307->irq_enabled, true);
		}

		switch (params_rate(params)) {
		case 8000:
		case 12000:
		case 16000:
		case 24000:
		case 32000:
		case 44100:
		case 48000:
			break;

		case 96000:
			dev_info(component->dev, "%s : %d rate not support SDO\n",
				__func__, params_rate(params));
			break;

		default:
			dev_err(component->dev, "%s not support rate : %d\n",
				__func__, params_rate(params));
			return -EINVAL;
		}

	/* substream->stream is SNDRV_PCM_STREAM_CAPTURE */
	} else {

		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			dev_info(component->dev,
				"%s set format SNDRV_PCM_FORMAT_S16_LE\n",
				__func__);
			sma1307_regmap_update_bits(sma1307,
					SMA1307_A4_TOP_MAN3,
					SMA1307_SCK_RATE_MASK
						| SMA1307_DATA_WIDTH_MASK,
					SMA1307_SCK_32FS
						| SMA1307_DATA_16BIT,
					NULL);
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
			dev_info(component->dev,
				"%s set format SNDRV_PCM_FORMAT_S24_LE\n",
				__func__);
			sma1307_regmap_update_bits(sma1307,
					SMA1307_A4_TOP_MAN3,
					SMA1307_SCK_RATE_MASK
						| SMA1307_DATA_WIDTH_MASK,
					SMA1307_SCK_64FS
						| SMA1307_DATA_24BIT,
					NULL);
			break;

		case SNDRV_PCM_FORMAT_S32_LE:
			dev_info(component->dev,
				"%s set format SNDRV_PCM_FORMAT_S32_LE\n",
				__func__);
			sma1307_regmap_update_bits(sma1307,
					SMA1307_A4_TOP_MAN3,
					SMA1307_SCK_RATE_MASK
						| SMA1307_DATA_WIDTH_MASK,
					SMA1307_SCK_64FS
						| SMA1307_DATA_24BIT,
					NULL);
			break;
		default:
			dev_err(component->dev,
				"%s not support data bit : %d\n", __func__,
						params_format(params));
			return -EINVAL;
		}
	}

	switch (sma1307->format) {
	case SND_SOC_DAIFMT_I2S:
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_I2S_MODE_MASK,
				SMA1307_STANDARD_I2S,
				NULL);
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_A4_TOP_MAN3,
				SMA1307_INTERFACE_MASK,
				SMA1307_I2S_FORMAT,
				NULL);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_I2S_MODE_MASK,
				SMA1307_LJ,
				NULL);
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_A4_TOP_MAN3,
				SMA1307_INTERFACE_MASK,
				SMA1307_LJ_FORMAT,
				NULL);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		switch (params_width(params)) {
		case 16:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_01_INPUT_CTRL1,
					SMA1307_I2S_MODE_MASK,
					SMA1307_RJ_16BIT,
					NULL);
			break;
		case 24:
		case 32:
			ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_01_INPUT_CTRL1,
					SMA1307_I2S_MODE_MASK,
					SMA1307_RJ_24BIT,
					NULL);
			break;
		}
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_I2S_MODE_MASK,
				SMA1307_STANDARD_I2S,
				NULL);
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_A4_TOP_MAN3,
				SMA1307_INTERFACE_MASK,
				SMA1307_TDM_FORMAT,
				NULL);
		break;
	}

	switch (params_width(params)) {
	case 16:
	case 24:
	case 32:
		break;
	default:
		dev_err(component->dev,
			"%s not support data bit : %d\n", __func__,
					params_format(params));
		return -EINVAL;
	}
	if (ret < 0)
		return -EINVAL;

	return 0;
}

static int sma1307_dai_set_sysclk_amp(struct snd_soc_dai *dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);

	switch (clk_id) {
	case SMA1307_EXTERNAL_CLOCK_19_2:
		break;
	case SMA1307_EXTERNAL_CLOCK_24_576:
		break;
	case SMA1307_PLL_CLKIN_MCLK:
		break;
	case SMA1307_PLL_CLKIN_BCLK:
		break;
	default:
		dev_err(component->dev, "Invalid clk id: %d\n", clk_id);
		return -EINVAL;
	}
	sma1307->sys_clk_id = clk_id;
	return 0;
}

static int sma1307_dai_set_fmt_amp(struct snd_soc_dai *dai,
					unsigned int fmt)
{
	struct snd_soc_component *component  = dai->component;
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {

	case SND_SOC_DAIFMT_CBC_CFC:
		dev_info(component->dev,
				"%s : %s\n", __func__, "I2S/TDM Device mode");
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_CONTROLLER_DEVICE_MASK,
				SMA1307_DEVICE_MODE,
				NULL);
		break;

	case SND_SOC_DAIFMT_CBP_CFP:
		dev_info(component->dev,
			"%s : %s\n", __func__, "I2S/TDM Controller mode");
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_CONTROLLER_DEVICE_MASK,
				SMA1307_CONTROLLER_MODE,
				NULL);
		break;

	default:
		dev_err(component->dev,
			"Unsupported Controller/Device : 0x%x\n", fmt);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {

	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		sma1307->format = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
		break;
	default:
		dev_err(component->dev,
			"Unsupported Audio Interface Format : 0x%x\n", fmt);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {

	case SND_SOC_DAIFMT_IB_NF:
		dev_info(component->dev, "%s : %s\n",
			__func__, "Invert BCLK + Normal Frame");
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_SCK_RISING_MASK,
				SMA1307_SCK_RISING_EDGE,
				NULL);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dev_info(component->dev, "%s : %s\n",
			__func__, "Invert BCLK + Invert Frame");
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_LEFTPOL_MASK
					| SMA1307_SCK_RISING_MASK,
				SMA1307_HIGH_FIRST_CH
					| SMA1307_SCK_RISING_EDGE,
				NULL);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dev_info(component->dev, "%s : %s\n",
			__func__, "Normal BCLK + Invert Frame");
		ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_01_INPUT_CTRL1,
				SMA1307_LEFTPOL_MASK,
				SMA1307_HIGH_FIRST_CH,
				NULL);
		break;
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(component->dev, "%s : %s\n",
			__func__, "Normal BCLK + Normal Frame");
		break;
	default:
		dev_err(component->dev,
				"Unsupported Bit & Frameclock : 0x%x\n", fmt);
		return -EINVAL;
	}

	if (ret < 0)
		return -EINVAL;
	return 0;
}

static int sma1307_dai_set_tdm_slot(struct snd_soc_dai *dai,
				   unsigned int tx_mask, unsigned int rx_mask,
				   int slots, int slot_width)
{
	struct snd_soc_component *component  = dai->component;
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_info(component->dev, "%s : slots = %d, slot_width - %d\n",
			__func__, slots, slot_width);

	sma1307->frame_size = slot_width * slots;

	ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_A4_TOP_MAN3,
				SMA1307_INTERFACE_MASK,
				SMA1307_TDM_FORMAT,
				NULL);

	ret += sma1307_regmap_update_bits(sma1307,
				SMA1307_A5_TDM1,
				SMA1307_TDM_TX_MODE_MASK,
				SMA1307_TDM_TX_MONO,
				NULL);

	switch (slot_width) {
	case 16:
		ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A6_TDM2,
					SMA1307_TDM_DL_MASK,
					SMA1307_TDM_DL_16,
					NULL);
		break;
	case 32:
		ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A6_TDM2,
					SMA1307_TDM_DL_MASK,
					SMA1307_TDM_DL_32,
					NULL);
		break;
	default:
		dev_err(component->dev, "%s not support TDM %d slot_width\n",
					__func__, slot_width);
		return -EINVAL;
	}

	switch (slots) {
	case 4:
		ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A6_TDM2,
					SMA1307_TDM_N_SLOT_MASK,
					SMA1307_TDM_N_SLOT_4,
					NULL);
		break;
	case 8:
		ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A6_TDM2,
					SMA1307_TDM_N_SLOT_MASK,
					SMA1307_TDM_N_SLOT_8,
					NULL);
		break;
	default:
		dev_err(component->dev, "%s not support TDM %d slots\n",
				__func__, slots);
		return -EINVAL;
	}

	if (sma1307->tdm_slot_rx < slots)
		ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A5_TDM1,
					SMA1307_TDM_SLOT1_RX_POS_MASK,
					(sma1307->tdm_slot_rx) << 3,
					NULL);
	else
		dev_err(component->dev, "%s Incorrect tdm-slot-rx %d set\n",
					__func__, sma1307->tdm_slot_rx);

	if (sma1307->tdm_slot_tx < slots)
		ret += sma1307_regmap_update_bits(sma1307,
					SMA1307_A6_TDM2,
					SMA1307_TDM_SLOT1_TX_POS_MASK,
					(sma1307->tdm_slot_tx) << 3,
					NULL);
	else
		dev_err(component->dev, "%s Incorrect tdm-slot-tx %d set\n",
				__func__, sma1307->tdm_slot_tx);

	if (ret < 0)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops sma1307_dai_ops_amp = {
	.hw_params = sma1307_dai_hw_params_amp,
	.set_fmt = sma1307_dai_set_fmt_amp,
	.set_sysclk = sma1307_dai_set_sysclk_amp,
	.set_tdm_slot = sma1307_dai_set_tdm_slot,
};

#define SMA1307_RATES_PLAYBACK SNDRV_PCM_RATE_8000_96000
#define SMA1307_RATES_CAPTURE SNDRV_PCM_RATE_8000_48000
#define SMA1307_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver sma1307_dai[] = {
	{
		.name = "sma1307-amplifier",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SMA1307_RATES_PLAYBACK,
			.formats = SMA1307_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SMA1307_RATES_CAPTURE,
			.formats = SMA1307_FORMATS,
		},
		.ops = &sma1307_dai_ops_amp,
	},
};

static irqreturn_t sma1307_isr(int irq, void *data)
{
	struct sma1307_priv *sma1307 = (struct sma1307_priv *) data;

	if (sma1307->check_fault_status)
		queue_delayed_work(system_freezable_wq,
				&sma1307->check_fault_work, 0);

	if (sma1307->isr_manual_mode) {
		sma1307_regmap_update_bits(sma1307,
					SMA1307_93_INT_CTRL,
					SMA1307_CLR_INT_MASK,
					SMA1307_INT_CLEAR,
					NULL);
		sma1307_regmap_update_bits(sma1307,
					SMA1307_93_INT_CTRL,
					SMA1307_CLR_INT_MASK,
					SMA1307_INT_READY,
					NULL);
	}
	return IRQ_HANDLED;
}

static void sma1307_check_fault_worker(struct work_struct *work)
{
	struct sma1307_priv *sma1307 =
		container_of(work, struct sma1307_priv, check_fault_work.work);
	int ret = 0;
	unsigned int status1_val, status2_val;

	if (ret != 0) {
		dev_err(sma1307->dev,
			"failed to read SMA1307_0A_SPK_VOL : %d\n", ret);
		return;
	}

	ret = sma1307_regmap_read(sma1307, SMA1307_FA_STATUS1, &status1_val);
	if (ret != 0) {
		dev_err(sma1307->dev,
			"failed to read SMA1307_FA_STATUS1 : %d\n", ret);
		return;
	}

	ret = sma1307_regmap_read(sma1307, SMA1307_FB_STATUS2, &status2_val);
	if (ret != 0) {
		dev_err(sma1307->dev,
			"failed to read SMA1307_FB_STATUS2 : %d\n", ret);
		return;
	}

	if (~status1_val & SMA1307_OT1_OK_STATUS) {
		dev_crit(sma1307->dev,
			"%s : OT1(Over Temperature Level 1)\n", __func__);
	}
	if (~status1_val & SMA1307_OT2_OK_STATUS) {
		dev_crit(sma1307->dev,
			"%s : OT2(Over Temperature Level 2)\n", __func__);
	}
	if (status1_val & SMA1307_UVLO_STATUS) {
		dev_crit(sma1307->dev,
			"%s : UVLO(Under Voltage Lock Out)\n", __func__);
	}
	if (status1_val & SMA1307_OVP_BST_STATUS) {
		dev_crit(sma1307->dev,
			"%s : OVP_BST(Over Voltage Protection)\n", __func__);
	}
	if (status2_val & SMA1307_OCP_SPK_STATUS) {
		dev_crit(sma1307->dev,
			"%s : OCP_SPK(Over Current Protect SPK)\n", __func__);
	}
	if (status2_val & SMA1307_OCP_BST_STATUS) {
		dev_crit(sma1307->dev,
			"%s : OCP_BST(Over Current Protect Boost)\n", __func__);
	}
	if (status2_val & SMA1307_CLK_MON_STATUS) {
		dev_crit(sma1307->dev,
			"%s : CLK_FAULT(No clock input)\n", __func__);
	}
	if (!gpio_is_valid(sma1307->gpio_int)) {
		if (sma1307->check_fault_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma1307->check_fault_work,
					sma1307->check_fault_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma1307->check_fault_work,
					CHECK_PERIOD_TIME * HZ);
	}

}

static int sma1307_reset(struct snd_soc_component *component)
{
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);
	int ret = 0;
	unsigned int status, i;

	ret = sma1307_regmap_read(sma1307, SMA1307_FF_DEVICE_INDEX, &status);

	if (ret != 0)
		dev_err(sma1307->dev,
			"failed to read SMA1307_FF_DEVICE_INDEX : %d\n", ret);
	else {
		sma1307->rev_num = status & SMA1307_REV_NUM_STATUS;
		dev_info(component->dev,
				"SMA1307 Revision %d\n", sma1307->rev_num);
	}

	sma1307_regmap_read(sma1307, SMA1307_99_OTP_TRM2, &sma1307->otp_trm2);
	sma1307_regmap_read(sma1307, SMA1307_9A_OTP_TRM3, &sma1307->otp_trm3);

	if ((sma1307->otp_trm2 & SMA1307_OTP_STAT_MASK) == SMA1307_OTP_STAT_1)
		dev_info(component->dev, "SMA1307 OTP Status Successful\n");
	else
		dev_info(component->dev, "SMA1307 OTP Status Fail\n");

	/* Register Initial Value Setting */
	for (i = 0; i < (unsigned int) ARRAY_SIZE(sma1307_reg_def); i++)
		sma1307_regmap_write(sma1307,
			sma1307_reg_def[i].reg, sma1307_reg_def[i].def);
	if (sma1307->rev_num == SMA1307_REV_NUM_REV0) {
		sma1307_regmap_write(sma1307, SMA1307_8F_ANALOG_TEST, 0x00);
		sma1307_regmap_write(sma1307, SMA1307_92_FDPEC_CTRL1, 0x80);
		sma1307_regmap_write(sma1307, SMA1307_95_BOOST_CTRL10, 0x74);
		sma1307_regmap_write(sma1307, SMA1307_A8_BOOST_CTRL1, 0x05);
		sma1307_regmap_write(sma1307, SMA1307_A9_BOOST_CTRL2, 0x28);
		sma1307_regmap_write(sma1307, SMA1307_AB_BOOST_CTRL4, 0x14);
		sma1307_regmap_write(sma1307, SMA1307_99_OTP_TRM2, 0x00);
		sma1307_regmap_update_bits(sma1307,
					SMA1307_9A_OTP_TRM3,
					SMA1307_RCV_OFFS2_MASK,
					SMA1307_RCV_OFFS2_DEFAULT,
					NULL);
	}
	sma1307_regmap_update_bits(sma1307,
			SMA1307_93_INT_CTRL,
			SMA1307_DIS_INT_MASK,
			SMA1307_HIGH_Z_INT,
			NULL);
	sma1307_spk_rcv_conf(component);

	return 0;
}

static ssize_t driver_version_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma1307_priv *sma1307 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%s\n", sma1307->driver_version);

	return (ssize_t)rc;
}
static DEVICE_ATTR_RO(driver_version);

static ssize_t check_fault_period_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma1307_priv *sma1307 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma1307->check_fault_period);

	return (ssize_t)rc;
}

static ssize_t check_fault_period_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma1307_priv *sma1307 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma1307->check_fault_period);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR_RW(check_fault_period);

static ssize_t check_fault_status_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma1307_priv *sma1307 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma1307->check_fault_status);

	return (ssize_t)rc;
}

static ssize_t check_fault_status_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma1307_priv *sma1307 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma1307->check_fault_status);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR_RW(check_fault_status);

static ssize_t isr_manual_mode_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma1307_priv *sma1307 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma1307->isr_manual_mode);

	return (ssize_t)rc;
}

static ssize_t isr_manual_mode_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma1307_priv *sma1307 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma1307->isr_manual_mode);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR_RW(isr_manual_mode);

static struct attribute *sma1307_attr[] = {
	&dev_attr_driver_version.attr,
	&dev_attr_check_fault_period.attr,
	&dev_attr_check_fault_status.attr,
	&dev_attr_isr_manual_mode.attr,
	NULL,
};

static struct attribute_group sma1307_attr_group = {
	.attrs = sma1307_attr,
};

static int sma1307_probe(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_get_dapm(component);

	snd_soc_dapm_sync(dapm);

	sma1307_amp_component = component;
	sma1307_reset(component);

	return 0;
}

static void sma1307_remove(struct snd_soc_component *component)
{
	struct sma1307_priv *sma1307 = snd_soc_component_get_drvdata(component);

	cancel_delayed_work_sync(&sma1307->check_fault_work);
}

static const struct snd_soc_component_driver sma1307_component = {
	.probe = sma1307_probe,
	.remove = sma1307_remove,
	.controls = sma1307_snd_controls,
	.num_controls = ARRAY_SIZE(sma1307_snd_controls),
	.dapm_widgets = sma1307_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sma1307_dapm_widgets),
	.dapm_routes = sma1307_audio_map,
	.num_dapm_routes = ARRAY_SIZE(sma1307_audio_map),
};

const struct regmap_config sma_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SMA1307_FF_DEVICE_INDEX,
	.readable_reg = sma1307_readable_register,
	.writeable_reg = sma1307_writeable_register,
	.volatile_reg = sma1307_volatile_register,

	.cache_type = REGCACHE_NONE,
	.reg_defaults = sma1307_reg_def,
	.num_reg_defaults = ARRAY_SIZE(sma1307_reg_def),
};

static int sma1307_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sma1307_priv *sma1307;
	struct device_node *np = client->dev.of_node;
	int ret = 0;
	u32 value;
	unsigned int device_info;

	dev_info(&client->dev, "%s : Driver Version %s\n",
						__func__, DRIVER_VERSION);
	sma1307 = devm_kzalloc(&client->dev,
			sizeof(struct sma1307_priv), GFP_KERNEL);

	if (!sma1307)
		return -ENOMEM;

	sma1307->regmap = devm_regmap_init_i2c(client, &sma_i2c_regmap);
	if (IS_ERR(sma1307->regmap)) {
		ret = PTR_ERR(sma1307->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", ret);

		return ret;
	}

	if (np) {
		if (!of_property_read_u32(np, "amp-mode", &value)) {
			switch (value) {
			case SMA1307_SPEAKER_MODE1:
				dev_info(&client->dev,
					"Set speaker mode1\n");
				break;
			case SMA1307_SPEAKER_MODE2:
				dev_info(&client->dev,
					"Set speaker mode2\n");
				break;
			case SMA1307_SPEAKER_MODE3:
				dev_info(&client->dev,
					"Set speaker mode3\n");
				break;
			case SMA1307_SPEAKER_MODE4:
				dev_info(&client->dev,
					"Set speaker mode4\n");
				break;
			case SMA1307_RECEIVER_MODE1:
				dev_info(&client->dev,
					"Set receiver mode1\n");
				break;
			case SMA1307_RECEIVER_MODE2:
				dev_info(&client->dev,
					"Set receiver mode2\n");
				break;
			default:
				dev_err(&client->dev,
					"Invalid mode: %d\n", value);
				return -EINVAL;
			}
			sma1307->dapm_amp_mode = value;
			sma1307->spk_rcv_mode = value;
		} else {
			dev_info(&client->dev,
				"Set speaker mode3(default)\n");
			sma1307->dapm_amp_mode = SMA1307_SPEAKER_MODE4;
			sma1307->spk_rcv_mode = SMA1307_SPEAKER_MODE4;
		}
		sma1307->gpio_int = of_get_named_gpio(np,
				"gpio-int", 0);
		if (!gpio_is_valid(sma1307->gpio_int)) {
			dev_err(&client->dev,
			"Looking up %s property in node %s failed %d\n",
			"gpio-int", client->dev.of_node->full_name,
			sma1307->gpio_int);
		}
	} else {
		dev_err(&client->dev,
			"device node initialization error\n");
		return -ENODEV;
	}

	ret = regmap_read(sma1307->regmap,
			SMA1307_FF_DEVICE_INDEX, &device_info);

	if ((ret != 0) || ((device_info & 0xF8) != SMA1307_DEVICE_ID)) {
		dev_err(&client->dev, "device initialization error (%d 0x%02X)",
				ret, device_info);
		/*return -ENODEV;*/
	}
	dev_info(&client->dev, "chip version 0x%02X\n", device_info);

	/* set initial value as normal AMP IC status */
	sma1307->amp_power_status = false;
	sma1307->force_mute_status = false;

	sma1307->amp_mode = SMA1307_MONO_MODE;
	sma1307->devtype = (enum sma1307_type) id->driver_data;

	sma1307->irq = -1;
	sma1307->num_of_pll_matches =
		ARRAY_SIZE(sma1307_pll_matches);
	sma1307->retry_cnt = SMA1307_I2C_RETRY_COUNT;

	sma1307->driver_version = DRIVER_VERSION;
	sma1307->check_fault_period = CHECK_PERIOD_TIME;
	sma1307->check_fault_status = true;
	sma1307->isr_manual_mode = true;

	sma1307->format = SND_SOC_DAIFMT_I2S;
	sma1307->frame_size = 0;
	sma1307->last_bclk = 0;
	sma1307->otp_trm2 = 0;
	sma1307->otp_trm3 = 0;
	sma1307->rev_num = 0;
	sma1307->sys_clk_id = SMA1307_PLL_CLKIN_BCLK;
	sma1307->tdm_slot_rx = 0;
	sma1307->tdm_slot_tx = 0;

	sma1307->dapm_aif_in = 0;
	sma1307->dapm_aif_out0 = 0;
	sma1307->dapm_aif_out1 = 0;
	sma1307->dapm_amp_en = 0;
	sma1307->dapm_sdo_en = 0;
	sma1307->dapm_sdo_setting = 0;

	mutex_init(&sma1307->pwr_lock);
	INIT_DELAYED_WORK(&sma1307->check_fault_work,
		sma1307_check_fault_worker);

	mutex_init(&sma1307->routing_lock);

	sma1307->dev = &client->dev;
	sma1307->kobj = &client->dev.kobj;

	i2c_set_clientdata(client, sma1307);

	sma1307->pll_matches = sma1307_pll_matches;

	if (gpio_is_valid(sma1307->gpio_int)) {

		dev_info(&client->dev, "%s , i2c client name: %s\n",
			__func__, dev_name(sma1307->dev));

		ret = devm_gpio_request(&client->dev,
				sma1307->gpio_int, "sma1307-irq");
		if (ret) {
			dev_info(&client->dev, "Duplicated gpio request\n");
			/* return ret; */
		}

		sma1307->irq = gpio_to_irq(sma1307->gpio_int);

		/* Get SMA1307 IRQ */
		if (sma1307->irq < 0) {
			dev_warn(&client->dev, "interrupt disabled\n");
		} else {
		/* Request system IRQ for SMA1307 */
			ret = devm_request_threaded_irq
				(&client->dev, sma1307->irq,
				NULL, sma1307_isr, IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_LOW, "sma1307", sma1307);
			if (ret < 0) {
				dev_err(&client->dev, "failed to request IRQ(%u) [%d]\n",
						sma1307->irq, ret);
				sma1307->irq = -1;
				i2c_set_clientdata(client, NULL);
				return ret;
			}
			disable_irq((unsigned int)sma1307->irq);
		}
	} else {
		dev_err(&client->dev,
			"interrupt signal input pin is not found\n");
	}

	atomic_set(&sma1307->irq_enabled, false);
	i2c_set_clientdata(client, sma1307);

	ret = devm_snd_soc_register_component(&client->dev,
			&sma1307_component, sma1307_dai, 1);

	if (ret) {
		dev_err(&client->dev, "Failed to register component");

		return ret;
	}

	sma1307->attr_grp = &sma1307_attr_group;
	ret = sysfs_create_group(sma1307->kobj, sma1307->attr_grp);
	if (ret) {
		dev_err(&client->dev,
			"failed to create attribute group [%d]\n", ret);
		sma1307->attr_grp = NULL;
	}

	return ret;
}

static void sma1307_i2c_remove(struct i2c_client *client)
{
	struct sma1307_priv *sma1307 =
		(struct sma1307_priv *) i2c_get_clientdata(client);

	cancel_delayed_work_sync(&sma1307->check_fault_work);

	return;
}

static const struct i2c_device_id sma1307_i2c_id[] = {
	{"sma1307", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sma1307_i2c_id);

static const struct of_device_id sma1307_of_match[] = {
	{ .compatible = "irondevice,sma1307", },
	{ }
};
MODULE_DEVICE_TABLE(of, sma1307_of_match);

static struct i2c_driver sma1307_i2c_driver = {
	.driver = {
		.name = "sma1307",
		.of_match_table = sma1307_of_match,
	},
	.probe = sma1307_i2c_probe,
	.remove = sma1307_i2c_remove,
	.id_table = sma1307_i2c_id,
};
module_i2c_driver(sma1307_i2c_driver);

MODULE_DESCRIPTION("ALSA SoC SMA1307 driver");
MODULE_AUTHOR("Gyuhwa Park, <gyuhwa.park@irondevice.com>");
MODULE_AUTHOR("KS Jo, <kiseok.jo@irondevice.com>");
MODULE_LICENSE("GPL v2");
