/*
 * sta309a.c
 *
 *  Created on: Jul 6, 2013
 *      Author: dsueiro
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <sound/soc.h>

#include <sound/sta309a-platform.h>
#include <sound/sta309a-alsa.h>
#include "sta309a-types.h"

#define DRIVER_NAME "sta309a"

// this device structure
struct sta309a_device
{
	struct device						*dev;
	struct i2c_client					*client;
	struct i2c_device_id				*id;
	struct sta309a_platform_data		*pdata;
};
static struct sta309a_device sta309a_dev;

static int sta309a_reg_read(u8 reg)
{
	int ret;
	if(sta309a_dev.client)
	{
		ret = i2c_smbus_read_byte_data(sta309a_dev.client, reg);
		dev_dbg(sta309a_dev.dev,"sta309a_reg_read 0x%X=0x%X\n",reg,ret);
		return ret;
	}
	return -ENODEV;
}

static int sta309a_reg_write(u8 reg, u8 value)
{
	int ret;
	if(sta309a_dev.client)
	{
		ret = i2c_smbus_write_byte_data(sta309a_dev.client, reg, value);
		dev_dbg(sta309a_dev.dev,"sta309a_reg_write 0x%X=0x%X\n",reg,value);
		return ret;
	}
	return -ENODEV;
}

static int sta309a_reg_write_bits(u8 reg, u8 mask, u8 value)
{
	int change;
	unsigned int old, new;
	int ret;

	ret = sta309a_reg_read(reg);
	if (ret < 0)
		return ret;

	old = ret;
	new = (old & ~mask) | value;
	change = old != new;
	if (change) {
		ret = sta309a_reg_write(reg, new);
		if (ret < 0)
			return ret;
	}

	return change;
}

static int sta309a_clock_config(u8 value)
{
	int ret;
	dev_dbg(sta309a_dev.dev, "%s: value=0x%02X\n",__func__,value);
	ret = sta309a_reg_write(STA309A_CONFA, value);
	if ( ret < 0 )
		dev_err(sta309a_dev.dev, "Failed to config clock (%d)\n",ret);
	return ret;
}


static int sta309a_i2s_input_config(u8 value)
{
	int ret;
	dev_dbg(sta309a_dev.dev, "%s: value=0x%02X\n",__func__,value);
	ret = sta309a_reg_write(STA309A_CONFB, value);
	if ( ret < 0 )
		dev_err(sta309a_dev.dev, "Failed to config i2s input (%d)\n",ret);
	return ret;
}


static int sta309a_input_mapping_config(sta309a_input_mapping_channel input_channel, u8 value)
{
	int ret;
	int reg = 0;

	dev_dbg(sta309a_dev.dev, "%s: input_channel=%d, value=0x%02X\n",__func__,input_channel,value);
	switch(input_channel)
	{
	case STA309A_IMC12:
		reg = STA309A_C12IM;
		break;
	case STA309A_IMC34:
		reg = STA309A_C34IM;
		break;
	case STA309A_IMC56:
		reg = STA309A_C56IM;
		break;
	case STA309A_IMC78:
		reg = STA309A_C78IM;
		break;
	default:
		dev_err(sta309a_dev.dev, "Invalid input channel number %d\n",input_channel);
		return -EINVAL;
	}

	ret = sta309a_reg_write(reg, value);
	if ( ret < 0 )
		dev_err(sta309a_dev.dev, "Failed to config input mapping channel=%d (%d)\n",input_channel,ret);

	return ret;
}


static int sta309a_output_mapping_config(sta309a_output_mapping_channel output_channel, u8 value)
{
	int ret;
	int reg = 0;

	dev_dbg(sta309a_dev.dev, "%s: output_channel=%d, value=0x%02X\n",__func__,output_channel,value);
	switch(output_channel)
	{
	case STA309A_OMC12:
		reg = STA309A_C12OM;
		break;
	case STA309A_OMC34:
		reg = STA309A_C34OM;
		break;
	case STA309A_OMC56:
		reg = STA309A_C56OM;
		break;
	case STA309A_OMC78:
		reg = STA309A_C78OM;
		break;
	default:
		dev_err(sta309a_dev.dev, "Invalid output channel number %d\n",output_channel);
		return -EINVAL;
	}

	ret = sta309a_reg_write(reg, value);
	if ( ret < 0 )
		dev_err(sta309a_dev.dev, "Failed to config output mapping channel=%d (%d)\n",output_channel,ret);

	return ret;
}

int sta309a_external_amp(bool status)
{
	return sta309a_reg_write_bits(STA309A_CONFI, STA309A_CONFI_EAPD, status<<STA309A_CONFI_EAPD_SHIFT);
}

static int sta309a_get_mastermute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	ucontrol->value.enumerated.item[0] = sta309a_reg_read(STA309A_MMUTE);;
	return 0;
}

static int sta309a_set_mastermute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	sta309a_reg_write(STA309A_MMUTE, ucontrol->value.enumerated.item[0]);
	return 0;
}

static int sta309a_info_vol(struct snd_kcontrol *kcontrol,  struct snd_ctl_elem_info *uinfo)
{
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0x00; // 0dB- steps of -0.5dB
	uinfo->value.integer.max = 0xFF; // Hardware channel mute
	return 0;
}

static int sta309a_get_vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 addr)
{
	int reg;
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg = sta309a_reg_read(addr);
	if (reg >= 0)
		ucontrol->value.integer.value[0] = 0xff - reg;
	return 0;
}

static int sta309a_set_vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 addr)
{
	int reg;
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg = 0xff - ucontrol->value.integer.value[0];
	sta309a_reg_write(addr, reg);
	return 0;
}

void sta309a_configure_mappings(void)
{
	dev_info(sta309a_dev.dev, "Configuring Input/Output channels\n");
	if (sta309a_dev.pdata->master_volume >= 0)
		sta309a_reg_write(STA309A_MVOL, sta309a_dev.pdata->master_volume);
	if (sta309a_dev.pdata->external_amp >= 0)
		sta309a_external_amp(sta309a_dev.pdata->external_amp);
	if (sta309a_dev.pdata->clock_config >= 0)
		sta309a_clock_config(sta309a_dev.pdata->clock_config);
	if (sta309a_dev.pdata->i2s_config >= 0)
		sta309a_i2s_input_config(sta309a_dev.pdata->i2s_config);
	if (sta309a_dev.pdata->imc12 >= 0)
		sta309a_input_mapping_config(STA309A_IMC12,(u8)sta309a_dev.pdata->imc12);
	if (sta309a_dev.pdata->imc34 >= 0)
		sta309a_input_mapping_config(STA309A_IMC34,(u8)sta309a_dev.pdata->imc34);
	if (sta309a_dev.pdata->imc56 >= 0)
		sta309a_input_mapping_config(STA309A_IMC56,(u8)sta309a_dev.pdata->imc56);
	if (sta309a_dev.pdata->imc78 >= 0)
		sta309a_input_mapping_config(STA309A_IMC78,(u8)sta309a_dev.pdata->imc78);
	if (sta309a_dev.pdata->omc12 >= 0)
		sta309a_output_mapping_config(STA309A_OMC12,(u8)sta309a_dev.pdata->omc12);
	if (sta309a_dev.pdata->omc34 >= 0)
		sta309a_output_mapping_config(STA309A_OMC34,(u8)sta309a_dev.pdata->omc34);
	if (sta309a_dev.pdata->omc56 >= 0)
		sta309a_output_mapping_config(STA309A_OMC56,(u8)sta309a_dev.pdata->omc56);
	if (sta309a_dev.pdata->omc78 >= 0)
		sta309a_output_mapping_config(STA309A_OMC78,(u8)sta309a_dev.pdata->omc78);
}
EXPORT_SYMBOL(sta309a_configure_mappings);

int sta309a_audiodsp_master_mute(unsigned int on)
{
	return sta309a_reg_write(STA309A_MMUTE, on);
}
EXPORT_SYMBOL(sta309a_audiodsp_master_mute);

static int sta309a_get_mastervol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_MVOL);
}

static int sta309a_set_mastervol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_MVOL);
}

static int sta309a_get_channel1vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C1VOL);
}

static int sta309a_set_channel1vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C1VOL);
}

static int sta309a_get_channel2vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C2VOL);
}

static int sta309a_set_channel2vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C2VOL);
}

static int sta309a_get_channel3vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C3VOL);
}

static int sta309a_set_channel3vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C3VOL);
}

static int sta309a_get_channel4vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C4VOL);
}

static int sta309a_set_channel4vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C4VOL);
}

static int sta309a_get_channel5vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C5VOL);
}

static int sta309a_set_channel5vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C5VOL);
}

static int sta309a_get_channel6vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C6VOL);
}

static int sta309a_set_channel6vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C6VOL);
}

static int sta309a_get_channel7vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C7VOL);
}

static int sta309a_set_channel7vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C7VOL);
}

static int sta309a_get_channel8vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_vol(kcontrol, ucontrol, STA309A_C8VOL);
}

static int sta309a_set_channel8vol(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_vol(kcontrol, ucontrol, STA309A_C8VOL);
}

static int sta309a_get_channelmute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 addr)
{
	int mute = sta309a_reg_read(addr);
	int mask = 1 << 7;
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	ucontrol->value.enumerated.item[0] = ((mute & mask) == mask);
	return 0;
}

static int sta309a_set_channelmute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 addr)
{
	int mute = sta309a_reg_read(addr);
	int mask = 1 << 7;	// 1 at the last bit
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	if(ucontrol->value.enumerated.item[0])
		return sta309a_reg_write(addr, (mute | mask));	// mute on
	else
		return sta309a_reg_write(addr, (mute & ~mask));	// mute off
}

static int sta309a_get_channel1mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C1VTMB);
}

static int sta309a_set_channel1mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C1VTMB);
}

static int sta309a_get_channel2mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C2VTMB);
}

static int sta309a_set_channel2mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C2VTMB);
}

static int sta309a_get_channel3mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C3VTMB);
}

static int sta309a_set_channel3mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C3VTMB);
}

static int sta309a_get_channel4mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C4VTMB);
}

static int sta309a_set_channel4mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C4VTMB);
}

static int sta309a_get_channel5mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C5VTMB);
}

static int sta309a_set_channel5mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C5VTMB);
}

static int sta309a_get_channel6mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C6VTMB);
}

static int sta309a_set_channel6mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C6VTMB);
}

static int sta309a_get_channel7mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C7VTMB);
}

static int sta309a_set_channel7mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C7VTMB);
}

static int sta309a_get_channel8mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_channelmute(kcontrol, ucontrol, STA309A_C8VTMB);
}

static int sta309a_set_channel8mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_channelmute(kcontrol, ucontrol, STA309A_C8VTMB);
}

static int sta309a_info_eq(struct snd_kcontrol *kcontrol,  struct snd_ctl_elem_info *uinfo)
{
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = STA309ACTL_EQFREQ_MIN; // -15dB to 16dB steps of -1.0dB
	uinfo->value.integer.max = STA309ACTL_EQFREQ_MAX;
	return 0;
}

static int sta309a_get_eq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 addr)
{
	int reg;
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg = sta309a_reg_read(addr);
	if (reg >= 0)
		ucontrol->value.integer.value[0] = reg + STA309ACTL_EQFREQ_MIN;
	return 0;
}

static int sta309a_set_eq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 addr)
{
	int reg;
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg = ucontrol->value.integer.value[0] - STA309ACTL_EQFREQ_MIN;
	return sta309a_reg_write(addr, reg);
}

static int sta309a_get_eq80hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eq(kcontrol, ucontrol, STA309A_AGEQ);
}

static int sta309a_set_eq80hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eq(kcontrol, ucontrol, STA309A_AGEQ);
}

static int sta309a_get_eq300hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eq(kcontrol, ucontrol, STA309A_BGEQ);
}

static int sta309a_set_eq300hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eq(kcontrol, ucontrol, STA309A_BGEQ);
}

static int sta309a_get_eq1000hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eq(kcontrol, ucontrol, STA309A_CGEQ);
}

static int sta309a_set_eq1000hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eq(kcontrol, ucontrol, STA309A_CGEQ);
}

static int sta309a_get_eq3000hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eq(kcontrol, ucontrol, STA309A_DGEQ);
}

static int sta309a_set_eq3000hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eq(kcontrol, ucontrol, STA309A_DGEQ);
}

static int sta309a_get_eq8000hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eq(kcontrol, ucontrol, STA309A_EGEQ);
}

static int sta309a_set_eq8000hz(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eq(kcontrol, ucontrol, STA309A_EGEQ);
}

static int sta309a_get_eqchannelbypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 channel)
{
	int bypass = sta309a_reg_read(STA309A_EQBP);
	int mask = 1 << (channel - 1);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	ucontrol->value.enumerated.item[0] = ((bypass & mask) == mask);
	return 0;
}

static int sta309a_set_eqchannelbypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 channel)
{
	int bypass = sta309a_reg_read(STA309A_EQBP);
	int mask = 1 << (channel - 1);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	if(ucontrol->value.enumerated.item[0])
		return sta309a_reg_write(STA309A_EQBP, (bypass | mask));	// bypass on
	else
		return sta309a_reg_write(STA309A_EQBP, (bypass & ~mask));	// bypass off
}

static int sta309a_get_eqchannel1bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 1);
}

static int sta309a_set_eqchannel1bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 1);
}

static int sta309a_get_eqchannel2bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 2);
}

static int sta309a_set_eqchannel2bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 2);
}

static int sta309a_get_eqchannel3bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 3);
}

static int sta309a_set_eqchannel3bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 3);
}

static int sta309a_get_eqchannel4bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 4);
}

static int sta309a_set_eqchannel4bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 4);
}

static int sta309a_get_eqchannel5bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 5);
}

static int sta309a_set_eqchannel5bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 5);
}

static int sta309a_get_eqchannel6bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 6);
}

static int sta309a_set_eqchannel6bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 6);
}

static int sta309a_get_eqchannel7bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 7);
}

static int sta309a_set_eqchannel7bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 7);
}

static int sta309a_get_eqchannel8bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_eqchannelbypass(kcontrol, ucontrol, 8);
}

static int sta309a_set_eqchannel8bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_eqchannelbypass(kcontrol, ucontrol, 8);
}

static int sta309a_info_tonebtc(struct snd_kcontrol *kcontrol,  struct snd_ctl_elem_info *uinfo)
{
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = STA309ACTL_TONE_MIN; // -12dB to 12dB steps of 2.0dB
	uinfo->value.integer.max = STA309ACTL_TONE_MAX;
	return 0;
}

static int sta309a_get_tonebtc(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int reg = sta309a_reg_read(STA309A_TONE);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg &= 0x0F;	// clear 4 most significant bits
	if(reg < STA309ACTL_TONE_MIN)
		reg = STA309ACTL_TONE_MIN;
	if(reg > STA309ACTL_TONE_MAX)
		reg = STA309ACTL_TONE_MAX;
	ucontrol->value.enumerated.item[0] = reg;
	return 0;
}

static int sta309a_set_tonebtc(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int reg = sta309a_reg_read(STA309A_TONE);
	int value = ucontrol->value.enumerated.item[0];
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg &= 0xF0;	// clear 4 least significant bits
	reg += value;
	// updates register value
	return sta309a_reg_write(STA309A_TONE, reg);
}

static int sta309a_info_tonettc(struct snd_kcontrol *kcontrol,  struct snd_ctl_elem_info *uinfo)
{
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = STA309ACTL_TONE_MIN; // -12dB to 12dB steps of 2.0dB
	uinfo->value.integer.max = STA309ACTL_TONE_MAX;
	return 0;
}

static int sta309a_get_tonettc(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int reg = sta309a_reg_read(STA309A_TONE);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg = reg >> 4;
	if(reg < STA309ACTL_TONE_MIN)
		reg = STA309ACTL_TONE_MIN;
	if(reg > STA309ACTL_TONE_MAX)
		reg = STA309ACTL_TONE_MAX;
	ucontrol->value.enumerated.item[0] = reg;
	return 0;
}

static int sta309a_set_tonettc(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int reg = sta309a_reg_read(STA309A_TONE);
	int value = ucontrol->value.enumerated.item[0];
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	reg &= 0x0F; // clear 4 most significant bits
	reg += (value << 4);
	// updates register value
	return sta309a_reg_write(STA309A_TONE, reg);
}

static int sta309a_get_tonechannelbypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 channel)
{
	int bypass = sta309a_reg_read(STA309A_TONEBP);
	int mask = 1 << (channel - 1);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	ucontrol->value.enumerated.item[0] = ((bypass & mask) == mask);
	return 0;
}

static int sta309a_set_tonechannelbypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol, u8 channel)
{
	int bypass = sta309a_reg_read(STA309A_TONEBP);
	int mask = 1 << (channel - 1);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	if(ucontrol->value.enumerated.item[0])
		return sta309a_reg_write(STA309A_TONEBP, (bypass | mask));	// bypass on
	else
		return sta309a_reg_write(STA309A_TONEBP, (bypass & ~mask));	// bypass off
}

static int sta309a_get_tonechannel1bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 1);
}

static int sta309a_set_tonechannel1bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 1);
}

static int sta309a_get_tonechannel2bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 2);
}

static int sta309a_set_tonechannel2bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 2);
}

static int sta309a_get_tonechannel3bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 3);
}

static int sta309a_set_tonechannel3bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 3);
}

static int sta309a_get_tonechannel4bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 4);
}

static int sta309a_set_tonechannel4bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 4);
}

static int sta309a_get_tonechannel5bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 5);
}

static int sta309a_set_tonechannel5bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 5);
}

static int sta309a_get_tonechannel6bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 6);
}

static int sta309a_set_tonechannel6bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 6);
}

static int sta309a_get_tonechannel7bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 7);
}

static int sta309a_set_tonechannel7bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 7);
}

static int sta309a_get_tonechannel8bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_get_tonechannelbypass(kcontrol, ucontrol, 8);
}

static int sta309a_set_tonechannel8bypass(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return sta309a_set_tonechannelbypass(kcontrol, ucontrol, 8);
}

static int sta309a_get_preseteq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int preset = sta309a_reg_read(STA309A_PREEQ);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	preset &= 0x1F; // clears 3 most significant bits
	ucontrol->value.enumerated.item[0] = preset;
	return 0;
}

static int sta309a_set_preseteq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int preset = sta309a_reg_read(STA309A_PREEQ);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	preset &= 0xE0;	// clears 5 least significant bits
	preset += ucontrol->value.enumerated.item[0];
	return sta309a_reg_write(STA309A_PREEQ, preset);
}

static int sta309a_get_preq_bass_crossover_freq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int preset = sta309a_reg_read(STA309A_PREEQ);
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	preset = preset >> 5;
	ucontrol->value.enumerated.item[0] = preset;
	return 0;
}

static int sta309a_set_preq_bass_crossover_freq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int preset = sta309a_reg_read(STA309A_PREEQ);
	int value = ucontrol->value.enumerated.item[0];
	dev_dbg(sta309a_dev.dev, "%s\n",__func__);
	preset &= 0x1F;	// clears 3 most significant bits
	preset += (value << 5);
	return sta309a_reg_write(STA309A_PREEQ, preset);
}

// ON/OFF VALUES
static const char *on_off_function[] = {
		STA309ACTL_ONOFFCTL_OFF,
		STA309ACTL_ONOFFCTL_ON
};

static const struct soc_enum sta309a_onoff_enum[] = {
	SOC_ENUM_SINGLE_EXT(
			ARRAY_SIZE(on_off_function),
			on_off_function
			),
};

// PREQ BASS CROSSOVER FREQUENCY VALUES
static const char *sta309a_preq_bass_crossover_freq_function[] = {
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_70HZ,
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_80HZ,
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_90HZ,
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_100HZ,
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_110HZ,
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_120HZ,
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_140HZ,
		STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_160HZ
};

static const struct soc_enum sta309a_preq_bass_crossover_freq_enum[] = {
	SOC_ENUM_SINGLE_EXT(
			ARRAY_SIZE(sta309a_preq_bass_crossover_freq_function),
			sta309a_preq_bass_crossover_freq_function
			),
};

// STA309ACTL_PRESET_EQ VALUES
static const char *sta309a_preq_function[] = {
		STA309ACTL_PREQ_FLAT,
		STA309ACTL_PREQ_ROCK,
		STA309ACTL_PREQ_SOFTROCK,
		STA309ACTL_PREQ_JAZZ,
		STA309ACTL_PREQ_CLASSICAL,
		STA309ACTL_PREQ_DANCE,
		STA309ACTL_PREQ_POP,
		STA309ACTL_PREQ_SOFT,
		STA309ACTL_PREQ_HARD,
		STA309ACTL_PREQ_PARTY,
		STA309ACTL_PREQ_VOCAL,
		STA309ACTL_PREQ_HIPHOP,
		STA309ACTL_PREQ_DIALOG,
		STA309ACTL_PREQ_BASSBOOST1,
		STA309ACTL_PREQ_BASSBOOST2,
		STA309ACTL_PREQ_BASSBOOST3,
		STA309ACTL_PREQ_LOUDNESS1,
		STA309ACTL_PREQ_LOUDNESS2,
		STA309ACTL_PREQ_LOUDNESS3,
		STA309ACTL_PREQ_LOUDNESS4,
		STA309ACTL_PREQ_LOUDNESS5,
		STA309ACTL_PREQ_LOUDNESS6,
		STA309ACTL_PREQ_LOUDNESS7,
		STA309ACTL_PREQ_LOUDNESS8,
		STA309ACTL_PREQ_LOUDNESS9,
		STA309ACTL_PREQ_LOUDNESS10,
		STA309ACTL_PREQ_LOUDNESS11,
		STA309ACTL_PREQ_LOUDNESS12,
		STA309ACTL_PREQ_LOUDNESS13,
		STA309ACTL_PREQ_LOUDNESS14,
		STA309ACTL_PREQ_LOUDNESS15,
		STA309ACTL_PREQ_LOUDNESS16
};

static const struct soc_enum sta309a_preq_enum[] = {
	SOC_ENUM_SINGLE_EXT(
			ARRAY_SIZE(sta309a_preq_function),
			sta309a_preq_function
			),
};

/**
 * ALSA exported controls
 */
static const struct snd_kcontrol_new sta309a_snd_controls[] = {
	/* Master Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_MASTER_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_mastervol,
		.put 	= sta309a_set_mastervol,
	},
	/* Channel 1 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL1_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel1vol,
		.put 	= sta309a_set_channel1vol,
	},
	/* Channel 2 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL2_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel2vol,
		.put 	= sta309a_set_channel2vol,
	},
	/* Channel 3 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL3_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel3vol,
		.put 	= sta309a_set_channel3vol,
	},
	/* Channel 4 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL4_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel4vol,
		.put 	= sta309a_set_channel4vol,
	},
	/* Channel 5 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL5_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel5vol,
		.put 	= sta309a_set_channel5vol,
	},
	/* Channel 6 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL6_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel6vol,
		.put 	= sta309a_set_channel6vol,
	},
	/* Channel 7 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL7_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel7vol,
		.put 	= sta309a_set_channel7vol,
	},
	/* Channel 8 Volume */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_CHANNEL8_VOLUME,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_vol,
		.get 	= sta309a_get_channel8vol,
		.put 	= sta309a_set_channel8vol,
	},
	/* Master Mute */
	SOC_ENUM_EXT(
			STA309ACTL_MASTER_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_mastermute,
			sta309a_set_mastermute
			),
	/* Channel 1 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL1_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel1mute,
			sta309a_set_channel1mute
			),
	/* Channel 2 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL2_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel2mute,
			sta309a_set_channel2mute
			),
	/* Channel 3 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL3_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel3mute,
			sta309a_set_channel3mute
			),
	/* Channel 4 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL4_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel4mute,
			sta309a_set_channel4mute
			),
	/* Channel 5 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL5_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel5mute,
			sta309a_set_channel5mute
			),
	/* Channel 6 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL6_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel6mute,
			sta309a_set_channel6mute
			),
	/* Channel 7 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL7_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel7mute,
			sta309a_set_channel7mute
			),
	/* Channel 8 Mute */
	SOC_ENUM_EXT(
			STA309ACTL_CHANNEL8_MUTE,
			sta309a_onoff_enum[0],
			sta309a_get_channel8mute,
			sta309a_set_channel8mute
			),
	/* 80Hz Equalizer */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_EQ80HZ,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_eq,
		.get 	= sta309a_get_eq80hz,
		.put 	= sta309a_set_eq80hz,
	},
	/* 300Hz Equalizer */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_EQ300HZ,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_eq,
		.get 	= sta309a_get_eq300hz,
		.put 	= sta309a_set_eq300hz,
	},
	/* 1000Hz Equalizer */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_EQ1000HZ,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_eq,
		.get 	= sta309a_get_eq1000hz,
		.put 	= sta309a_set_eq1000hz,
	},
	/* 3000Hz Equalizer */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_EQ3000HZ,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_eq,
		.get 	= sta309a_get_eq3000hz,
		.put 	= sta309a_set_eq3000hz,
	},
	/* 8000Hz Equalizer */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_EQ8000HZ,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_eq,
		.get 	= sta309a_get_eq8000hz,
		.put 	= sta309a_set_eq8000hz,
	},
	/* EQ Channel 1 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL1_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel1bypass,
			sta309a_set_eqchannel1bypass
			),
	/* EQ Channel 2 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL2_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel2bypass,
			sta309a_set_eqchannel2bypass
			),
	/* EQ Channel 3 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL3_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel3bypass,
			sta309a_set_eqchannel3bypass
			),
	/* EQ Channel 4 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL4_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel4bypass,
			sta309a_set_eqchannel4bypass
			),
	/* EQ Channel 5 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL5_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel5bypass,
			sta309a_set_eqchannel5bypass
			),
	/* EQ Channel 6 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL6_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel6bypass,
			sta309a_set_eqchannel6bypass
			),
	/* EQ Channel 7 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL7_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel7bypass,
			sta309a_set_eqchannel7bypass
			),
	/* EQ Channel 8 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_EQCHANNEL8_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_eqchannel8bypass,
			sta309a_set_eqchannel8bypass
			),
	/* Tone BTC Control */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_TONE_BTC,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_tonebtc,
		.get 	= sta309a_get_tonebtc,
		.put 	= sta309a_set_tonebtc,
	},
	/* Tone TTC Control */
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name 	= STA309ACTL_TONE_TTC,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info 	= sta309a_info_tonettc,
		.get 	= sta309a_get_tonettc,
		.put 	= sta309a_set_tonettc,
	},
	/* Tone Channel 1 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL1_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel1bypass,
			sta309a_set_tonechannel1bypass
			),
	/* Tone Channel 2 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL2_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel2bypass,
			sta309a_set_tonechannel2bypass
			),
	/* Tone Channel 3 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL3_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel3bypass,
			sta309a_set_tonechannel3bypass
			),
	/* Tone Channel 4 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL4_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel4bypass,
			sta309a_set_tonechannel4bypass
			),
	/* Tone Channel 5 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL5_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel5bypass,
			sta309a_set_tonechannel5bypass
			),
	/* Tone Channel 6 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL6_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel6bypass,
			sta309a_set_tonechannel6bypass
			),
	/* Tone Channel 7 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL7_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel7bypass,
			sta309a_set_tonechannel7bypass
			),
	/* Tone Channel 8 Bypass */
	SOC_ENUM_EXT(
			STA309ACTL_TONECHANNEL8_BYPASS,
			sta309a_onoff_enum[0],
			sta309a_get_tonechannel8bypass,
			sta309a_set_tonechannel8bypass
			),
	/* EQ Presets */
	SOC_ENUM_EXT(
			STA309ACTL_PRESET_EQ,
			sta309a_preq_enum[0],
			sta309a_get_preseteq,
			sta309a_set_preseteq
			),
	/* EQ Presets Bass CrossOver Frequency */
	SOC_ENUM_EXT(
			STA309ACTL_PREQ_BASS_CROSSOVER_FREQ,
			sta309a_preq_bass_crossover_freq_enum[0],
			sta309a_get_preq_bass_crossover_freq,
			sta309a_set_preq_bass_crossover_freq
			),
};

int sta309a_audiodsp_add_controls(struct snd_soc_codec *codec)
{
	dev_info(sta309a_dev.dev, "Adding STA309A controls to codec.\n");
	return snd_soc_add_controls(codec, sta309a_snd_controls,
						 ARRAY_SIZE(sta309a_snd_controls));
}
EXPORT_SYMBOL(sta309a_audiodsp_add_controls);

static __devinit int sta309a_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret = 0;

	sta309a_dev.pdata = client->dev.platform_data;
	sta309a_dev.client = client;
	sta309a_dev.dev = &(client->dev);
	sta309a_configure_mappings();

	dev_info(sta309a_dev.dev, "STA309A device driver loaded.\n");

	return ret;
}


static __devexit int sta309a_i2c_remove(struct i2c_client *client)
{

	sta309a_dev.client = NULL;
	sta309a_dev.dev = NULL;

	return 0;
}

static const struct i2c_device_id sta309a_id[] = {
	{"sta309a", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sta309a_id);

static struct i2c_driver sta309a_i2c_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = sta309a_i2c_probe,
	.remove = __devexit_p(sta309a_i2c_remove),
	.id_table = sta309a_id,
};

static int __init sta309a_init(void)
{
	return i2c_add_driver(&sta309a_i2c_driver);
}

static void __exit sta309a_exit(void)
{
	i2c_del_driver(&sta309a_i2c_driver);
}

module_init(sta309a_init);
module_exit(sta309a_exit);

MODULE_DESCRIPTION("STA309A audio dsp driver");
MODULE_AUTHOR("Diego Sueiro <diego.sueiro@e-labworks.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
