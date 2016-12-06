/*
 * Copyright 2005-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file adv7180.c
 *
 * @brief Analog Device ADV7180 video decoder functions
 *
 * @ingroup Camera
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define ADV7180_VOLTAGE_ANALOG               1800000
#define ADV7180_VOLTAGE_DIGITAL_CORE         1800000
#define ADV7180_VOLTAGE_DIGITAL_IO           3300000
#define ADV7180_VOLTAGE_PLL                  1800000

static struct regulator *dvddio_regulator;
static struct regulator *dvdd_regulator;
static struct regulator *avdd_regulator;
static struct regulator *pvdd_regulator;
static int pwn_gpio;
static int ain1_gpio;
static int ain2_gpio;
static int ain3_gpio;

static int adv7180_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int adv7180_detach(struct i2c_client *client);

static const struct i2c_device_id adv7180_id[] = {
	{"adv7180", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, adv7180_id);

static struct i2c_driver adv7180_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "adv7180",
		   },
	.probe = adv7180_probe,
	.remove = adv7180_detach,
	.id_table = adv7180_id,
};

/**
 * When running in CVBS mode, ADV7180 can multiplex
 *  3 different analog input sources.
 */
enum v4l2_adv7180_inputs_t {
	ADV7180_AIN1 = 1,
	ADV7180_AIN2,
	ADV7180_AIN3
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct sensor {
	struct sensor_data sen;
	v4l2_std_id std_id;
	bool cvbs;
	enum v4l2_adv7180_inputs_t input;
} adv7180_data;


/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	ADV7180_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	ADV7180_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	ADV7180_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define ADV7180_STD_MAX		(ADV7180_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
	int frame_rate;		/*!< Frame rate. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
	 .frame_rate = 30,
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 25,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 25,
	 },
};

/*!* Standard index of ADV7180. */
static video_fmt_idx video_idx = ADV7180_PAL;

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);

#define IF_NAME                    "adv7180"
#define ADV7180_INPUT_CTL              0x00	/* Input Control */
#define ADV7180_STATUS_1               0x10	/* Status #1 */
#define ADV7180_BRIGHTNESS             0x0a	/* Brightness */
#define ADV7180_IDENT                  0x11	/* IDENT */
#define ADV7180_VSYNC_FIELD_CTL_1      0x31	/* VSYNC Field Control #1 */
#define ADV7180_MANUAL_WIN_CTL         0x3d	/* Manual Window Control */
#define ADV7180_SD_SATURATION_CB       0xe3	/* SD Saturation Cb */
#define ADV7180_SD_SATURATION_CR       0xe4	/* SD Saturation Cr */
#define ADV7180_ADC_SWITCH1            0xC3
#define ADV7180_ADC_SWITCH2            0xC4
#define ADV7180_PWR_MNG                0x0f     /* Power Management */

/* supported controls */
/* This hasn't been fully implemented yet.
 * This is how it should work, though. */
static struct v4l2_queryctrl adv7180_qctrl[] = {
	{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Brightness",
		.minimum = 0,			/* check this value */
		.maximum = 255,			/* check this value */
		.step = 1,				/* check this value */
		.default_value = 127,	/* check this value */
		.flags = 0,
	},
	{
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Saturation",
		.minimum = 0,			/* check this value */
		.maximum = 255,			/* check this value */
		.step = 0x1,			/* check this value */
		.default_value = 127,	/* check this value */
		.flags = 0,
	},
	{
		.id = V4L2_CID_SELECT_INPUT,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Input Source",
		.minimum = ADV7180_AIN1,
		.maximum = ADV7180_AIN3,
		.step = 0x1,
		.default_value = ADV7180_AIN1,	/* check this value */
		.flags = 0,
	}
};

static inline void adv7180_power_down(int enable)
{
	gpio_set_value_cansleep(pwn_gpio, !enable);
	msleep(2);
}

static int adv7180_regulator_enable(struct device *dev)
{
	int ret = 0;

	dvddio_regulator = devm_regulator_get(dev, "DOVDD");

	if (!IS_ERR(dvddio_regulator)) {
		regulator_set_voltage(dvddio_regulator,
				      ADV7180_VOLTAGE_DIGITAL_IO,
				      ADV7180_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(dvddio_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get io voltage\n");
	}

	dvdd_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(dvdd_regulator)) {
		regulator_set_voltage(dvdd_regulator,
				      ADV7180_VOLTAGE_DIGITAL_CORE,
				      ADV7180_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(dvdd_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get core voltage\n");
	}

	avdd_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(avdd_regulator)) {
		regulator_set_voltage(avdd_regulator,
				      ADV7180_VOLTAGE_ANALOG,
				      ADV7180_VOLTAGE_ANALOG);
		ret = regulator_enable(avdd_regulator);
		if (ret) {
			dev_err(dev, "set analog voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set analog voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get analog voltage\n");
	}

	pvdd_regulator = devm_regulator_get(dev, "PVDD");
	if (!IS_ERR(pvdd_regulator)) {
		regulator_set_voltage(pvdd_regulator,
				      ADV7180_VOLTAGE_PLL,
				      ADV7180_VOLTAGE_PLL);
		ret = regulator_enable(pvdd_regulator);
		if (ret) {
			dev_err(dev, "set pll voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set pll voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get pll voltage\n");
	}

	return ret;
}


/***********************************************************************
 * I2C transfert.
 ***********************************************************************/

/*! Read one register from a ADV7180 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int adv7180_read(u8 reg)
{
	int val;

	val = i2c_smbus_read_byte_data(adv7180_data.sen.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&adv7180_data.sen.i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

/*! Write one register of a ADV7180 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static int adv7180_write_reg(u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(adv7180_data.sen.i2c_client, reg, val);
	if (ret < 0) {
		dev_dbg(&adv7180_data.sen.i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

static int adv7180_set_input_source(enum v4l2_adv7180_inputs_t input)
{
	int adc1 = 0;
	int adc2 = 0;
	int ret = 0;

	if(!adv7180_data.cvbs)
	{
		dev_err(&adv7180_data.sen.i2c_client->dev, "selecting input source is only available in cvbs mode!");
		return -EPERM;
	}

	switch (input)
	{
		case ADV7180_AIN1:
			adc1 = 0x01;
			adc2 = 0x80;
			if (gpio_is_valid(ain1_gpio))
			{
				gpio_set_value_cansleep(ain1_gpio, 1);
			}
			if (gpio_is_valid(ain2_gpio))
			{
				gpio_set_value_cansleep(ain2_gpio, 0);
			}
			if (gpio_is_valid(ain3_gpio))
			{
				gpio_set_value_cansleep(ain3_gpio, 0);
			}
			break;
		case ADV7180_AIN2:
			adc1 = 0x44;
			adc2 = 0x80;
			if (gpio_is_valid(ain1_gpio))
			{
				gpio_set_value_cansleep(ain1_gpio, 0);
			}
			if (gpio_is_valid(ain2_gpio))
			{
				gpio_set_value_cansleep(ain2_gpio, 1);
			}
			if (gpio_is_valid(ain3_gpio))
			{
				gpio_set_value_cansleep(ain3_gpio, 0);
			}
			break;
		case ADV7180_AIN3:
			adc1 = 0x55;
			adc2 = 0x85;
			if (gpio_is_valid(ain1_gpio))
			{
				gpio_set_value_cansleep(ain1_gpio, 0);
			}
			if (gpio_is_valid(ain2_gpio))
			{
				gpio_set_value_cansleep(ain2_gpio, 0);
			}
			if (gpio_is_valid(ain3_gpio))
			{
				gpio_set_value_cansleep(ain3_gpio, 1);
			}
			break;
		default:
			return -EINVAL;
	}

	ret |= adv7180_write_reg(ADV7180_ADC_SWITCH1, adc1);
	ret |= adv7180_write_reg(ADV7180_ADC_SWITCH2, adc2);

	if(ret) {
		dev_err(&adv7180_data.sen.i2c_client->dev, "error while selecting input source: %d", ret);
		return ret;
	}

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "selected input source %d", input);
	adv7180_data.input = input;
	msleep(100);
	return 0;
}

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static void adv7180_get_std(v4l2_std_id *std)
{
	int status_1, standard, idx;
	bool locked;

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "In adv7180_get_std\n");

	mdelay(250);

	status_1 = adv7180_read(ADV7180_STATUS_1);
	locked = status_1 & 0x1;
	standard = status_1 & 0x70;

	mutex_lock(&mutex);
	*std = V4L2_STD_ALL;
	idx = ADV7180_NOT_LOCKED;
	if (locked) {
		if (standard == 0x40) {
			*std = V4L2_STD_PAL;
			idx = ADV7180_PAL;
		} else if (standard == 0) {
			*std = V4L2_STD_NTSC;
			idx = ADV7180_NTSC;
		}
	}
	mutex_unlock(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != adv7180_data.std_id) {
		video_idx = idx;
		adv7180_data.std_id = *std;
		adv7180_data.sen.pix.width = video_fmts[video_idx].raw_width;
		adv7180_data.sen.pix.height = video_fmts[video_idx].raw_height;
	}
}

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	dev_dbg(&adv7180_data.sen.i2c_client->dev, "adv7180:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.nobt_hs_inv = 1;
	p->u.bt656.bt_sync_correct = 1;

	/* ADV7180 has a dedicated clock so no clock settings needed. */

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "adv7180:ioctl_s_power: %d\n", on);

	if (on && !sensor->sen.on) {
		if (adv7180_write_reg(ADV7180_PWR_MNG, 0x04) != 0)
			return -EIO;

		/*
		 * FIXME:Additional 400ms to wait the chip to be stable?
		 * This is a workaround for preview scrolling issue.
		 */
		msleep(400);
	} else if (!on && sensor->sen.on) {
		if (adv7180_write_reg(ADV7180_PWR_MNG, 0x24) != 0)
			return -EIO;
	}

	sensor->sen.on = on;

	if(on)
	{
		adv7180_set_input_source(adv7180_data.input);
	}
	else
	{
		if (gpio_is_valid(ain1_gpio))
		{
			gpio_set_value_cansleep(ain1_gpio, 0);
		}
		if (gpio_is_valid(ain2_gpio))
		{
			gpio_set_value_cansleep(ain2_gpio, 0);
		}
		if (gpio_is_valid(ain3_gpio))
		{
			gpio_set_value_cansleep(ain3_gpio, 0);
		}
	}

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "In adv7180:ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	dev_dbg(&adv7180_data.sen.i2c_client->dev, "In adv7180:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "adv7180:ioctl_g_fmt_cap\n");

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		adv7180_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
			   struct v4l2_queryctrl *qc)
{
	int i;

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "adv7180:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(adv7180_qctrl); i++)
		if (qc->id && qc->id == adv7180_qctrl[i].id) {
			memcpy(qc, &(adv7180_qctrl[i]),
				sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	int sat = 0;

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "In adv7180:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		adv7180_data.sen.brightness = adv7180_read(ADV7180_BRIGHTNESS);
		vc->value = adv7180_data.sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = adv7180_data.sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		sat = adv7180_read(ADV7180_SD_SATURATION_CB);
		adv7180_data.sen.saturation = sat;
		vc->value = adv7180_data.sen.saturation;
		break;
	case V4L2_CID_HUE:
		vc->value = adv7180_data.sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = adv7180_data.sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = adv7180_data.sen.blue;
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = adv7180_data.sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_SELECT_INPUT:
		vc->value = (int) adv7180_data.input;
		break;
	default:
		vc->value = 0;
		ret = -EPERM;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	u8 tmp;

	dev_dbg(&adv7180_data.sen.i2c_client->dev, "In adv7180:ioctl_s_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		tmp = vc->value;
		adv7180_write_reg(ADV7180_BRIGHTNESS, tmp);
		adv7180_data.sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		tmp = vc->value;
		adv7180_write_reg(ADV7180_SD_SATURATION_CB, tmp);
		adv7180_write_reg(ADV7180_SD_SATURATION_CR, tmp);
		adv7180_data.sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_SELECT_INPUT:
		retval = adv7180_set_input_source((enum v4l2_adv7180_inputs_t) vc->value);
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height  = video_fmts[video_idx].active_height;

	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	video_fmt_t fmt;
	int i;

	if (fival->index != 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(video_fmts) - 1; i++) {
		fmt = video_fmts[i];
		if (fival->width  == fmt.active_width &&
		    fival->height == fmt.active_height) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator = fmt.frame_rate;
			return 0;
		}
	}

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"adv7180_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_ADV7180;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	dev_dbg(&adv7180_data.sen.i2c_client->dev, "In adv7180:ioctl_init\n");
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	dev_dbg(&adv7180_data.sen.i2c_client->dev, "adv7180:ioctl_dev_init\n");
	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc adv7180_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*)ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func*)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)
				ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave adv7180_slave = {
	.ioctls = adv7180_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(adv7180_ioctl_desc),
};

static struct v4l2_int_device adv7180_int_device = {
	.module = THIS_MODULE,
	.name = "adv7180",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &adv7180_slave,
	},
};


/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

/*! ADV7180 Reset function.
 *
 *  @return		None.
 */
static void adv7180_hard_reset(bool cvbs)
{
	dev_dbg(&adv7180_data.sen.i2c_client->dev,
		"In adv7180:adv7180_hard_reset\n");

	adv7180_data.cvbs = cvbs;

	if (cvbs) {
		/* Set CVBS input on AIN1 */
		adv7180_write_reg(ADV7180_INPUT_CTL, 0x00);
	} else {
		/*
		 * Set YPbPr input on AIN1,4,5 and normal
		 * operations(autodection of all stds).
		 */
		adv7180_write_reg(ADV7180_INPUT_CTL, 0x09);
	}

	/* Datasheet recommends */
	adv7180_write_reg(ADV7180_VSYNC_FIELD_CTL_1, 0x02);
	adv7180_write_reg(ADV7180_MANUAL_WIN_CTL, 0xa2);
}

/*! ADV7180 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * ADV7180 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int adv7180_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rev_id;
	int ret = 0;
	u32 cvbs = true;
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;

	printk(KERN_ERR"DBG sensor data is at %p\n", &adv7180_data);

	/* ov5640 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio))
	{
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,	"adv7180_pwdn");
	if (ret < 0)
	{
		dev_err(dev, "no power pin available!\n");
		return ret;
	}

	ain1_gpio = of_get_named_gpio(dev->of_node, "ain1-gpio", 0);
	if (!gpio_is_valid(ain1_gpio))
	{
		dev_warn(dev, "no ain1 pin available\n");
	}
	else
	{
		dev_info(&client->dev, "ain1 on gpio pin %d\n", ain1_gpio);
		ret = devm_gpio_request_one(dev, ain1_gpio, GPIOF_OUT_INIT_HIGH, "adv7180-ain1-gpio");
		if (ret < 0)
		{
			dev_err(dev, "no ain1 pin available!\n");
			return ret;
		}
		gpio_set_value_cansleep(ain1_gpio, 0);
	}

	ain2_gpio = of_get_named_gpio(dev->of_node, "ain2-gpio", 0);
	if (!gpio_is_valid(ain2_gpio))
	{
		dev_warn(dev, "no ain2 pin available\n");
	}
	else
	{
		dev_info(&client->dev, "ain2 on gpio pin %d\n", ain2_gpio);
		ret = devm_gpio_request_one(dev, ain2_gpio, GPIOF_OUT_INIT_HIGH, "adv7180-ain2-gpio");
		if (ret < 0)
		{
			dev_err(dev, "no ain2 pin available!\n");
			return ret;
		}
		gpio_set_value_cansleep(ain2_gpio, 0);
	}

	ain3_gpio = of_get_named_gpio(dev->of_node, "ain3-gpio", 0);
	if (!gpio_is_valid(ain3_gpio))
	{
		dev_warn(dev, "no ain3 pin available\n");
	}
	else
	{
		dev_info(&client->dev, "ain3 on gpio pin %d\n", ain3_gpio);
		ret = devm_gpio_request_one(dev, ain3_gpio, GPIOF_OUT_INIT_HIGH, "adv7180-ain3-gpio");
		if (ret < 0)
		{
			dev_err(dev, "no ain3 pin available!\n");
			return ret;
		}
		gpio_set_value_cansleep(ain3_gpio, 0);
	}

	adv7180_regulator_enable(dev);

	adv7180_power_down(0);

	msleep(1);

	/* Set initial values for the sensor struct. */
	memset(&adv7180_data, 0, sizeof(adv7180_data));
	adv7180_data.sen.i2c_client = client;
	adv7180_data.sen.streamcap.timeperframe.denominator = 30;
	adv7180_data.sen.streamcap.timeperframe.numerator = 1;
	adv7180_data.std_id = V4L2_STD_ALL;
	video_idx = ADV7180_NOT_LOCKED;
	adv7180_data.sen.pix.width = video_fmts[video_idx].raw_width;
	adv7180_data.sen.pix.height = video_fmts[video_idx].raw_height;
	adv7180_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	adv7180_data.sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	adv7180_data.sen.on = true;
	adv7180_data.input = ADV7180_AIN1;

	adv7180_data.sen.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(adv7180_data.sen.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(adv7180_data.sen.sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "mclk",
					&adv7180_data.sen.mclk);
	if (ret) {
		dev_err(dev, "mclk frequency is invalid\n");
		return ret;
	}

	ret = of_property_read_u32(
		dev->of_node, "mclk_source",
		(u32 *) &(adv7180_data.sen.mclk_source));
	if (ret) {
		dev_err(dev, "mclk_source invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&(adv7180_data.sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	clk_prepare_enable(adv7180_data.sen.sensor_clk);

	dev_dbg(&adv7180_data.sen.i2c_client->dev,
		"%s:adv7180 probe i2c address is 0x%02X\n",
		__func__, adv7180_data.sen.i2c_client->addr);

	/*! Read the revision ID of the tvin chip */
	rev_id = adv7180_read(ADV7180_IDENT);
	dev_info(dev,
		"%s:Analog Device adv7%2X0 detected!\n", __func__,
		rev_id);

	ret = of_property_read_u32(dev->of_node, "cvbs", &(cvbs));
	if (ret) {
		dev_err(dev, "cvbs setting is not found\n");
		cvbs = true;
	}

	/*! ADV7180 initialization. */
	adv7180_hard_reset(cvbs);

	pr_info("   type is %d (expect %d)\n",
		 adv7180_int_device.type, v4l2_int_type_slave);
	pr_info("   num ioctls is %d\n",
		 adv7180_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the adv7180_data structure here.*/
	adv7180_int_device.priv = &adv7180_data;
	ret = v4l2_int_device_register(&adv7180_int_device);

	clk_disable_unprepare(adv7180_data.sen.sensor_clk);

	return ret;
}

/*!
 * ADV7180 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int adv7180_detach(struct i2c_client *client)
{
	dev_dbg(&adv7180_data.sen.i2c_client->dev,
		"%s:Removing %s video decoder @ 0x%02X from adapter %s\n",
		__func__, IF_NAME, client->addr << 1, client->adapter->name);

	/* Power down via i2c */
	adv7180_write_reg(ADV7180_PWR_MNG, 0x24);

	if (dvddio_regulator)
		regulator_disable(dvddio_regulator);

	if (dvdd_regulator)
		regulator_disable(dvdd_regulator);

	if (avdd_regulator)
		regulator_disable(avdd_regulator);

	if (pvdd_regulator)
		regulator_disable(pvdd_regulator);

	v4l2_int_device_unregister(&adv7180_int_device);

	return 0;
}

/*!
 * ADV7180 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int adv7180_init(void)
{
	u8 err = 0;

	pr_debug("In adv7180_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&adv7180_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * ADV7180 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit adv7180_clean(void)
{
	dev_dbg(&adv7180_data.sen.i2c_client->dev, "In adv7180_clean\n");
	i2c_del_driver(&adv7180_i2c_driver);
}

module_init(adv7180_init);
module_exit(adv7180_clean);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Anolog Device ADV7180 video decoder driver");
MODULE_LICENSE("GPL");
