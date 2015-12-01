/*
 * radio-si475x.c
 *
 *	This is the kernel driver for Silicon Labs SI475X AM/FM Tuner device.
 *	It is an I2C device, so, I2C is needed.
 *
 *  Created on: 09/01/2015
 *      Author: mauricio.cirelli
 */


#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <media/radio-si475x.h>

enum si475x_band
{
	Si475x_FM = 1,
	Si475x_AM = 2,
};

#define DEFAULT_BAND  		Si475x_FM
#define DEFAULT_FREQ_FM		89100			// in kHz
#define DEFAULT_FREQ_AM		520				// in kHz

/**
 * V4L2 default radio number
 */
static int radio_nr = -1;
module_param(radio_nr, int, 0);
MODULE_PARM_DESC(radio_nr, "/dev/radioX device number to use");

/**
 * Default band (AM/FM)
 */
static int default_band = Si475x_FM;
module_param(default_band, int, 0);
MODULE_PARM_DESC(default_band, "Default band (1 = FM; 2 = AM)");

/**
 * Default frequency for AM
 */
static int default_freq_am = DEFAULT_FREQ_AM;
module_param(default_freq_am, int, 0);
MODULE_PARM_DESC(default_freq_am, "Default frequency in AM mode (kHz)");

/**
 * Default frequency for FM
 */
static int default_freq_fm = DEFAULT_FREQ_FM;
module_param(default_freq_fm, int, 0);
MODULE_PARM_DESC(default_freq_fm, "Default frequency in FM mode (kHz)");

/**
 * Device control structure
 */
struct si475x_device
{
	struct i2c_client		*client;
	struct video_device		*videodev;
	struct device			*dev;
	struct work_struct 		radio_work;			// Work queue for tuner IRQ
	struct completion 		work_stc;			// Semaphore for STC completions
	enum si475x_band		band;				// AM (2) or FM (1) selection
	unsigned int 			users;				// For Open/Release
	struct mutex 			lock;				// buffer locking
	struct radio_si475x_platform_data *pdata;	// Platform data
};

/**
 * Device status structure
 */
struct si475x_data
{
	u8  status;  		// Contains bits about the status returned from the part.
	u8  rsqInts; 		// Contains bits about the interrupts that have fired related to RSQ.
	u8  valid;   		// The station is valid if this is non-zero
	u8  bltf;    		// If the tune/seek failed this bit will be non-zero
	u16 freq;    		// The currently tuned frequency
	u8  rssi;    		// The current signal strength of the station
	u8  snr;     		// The current audio SNR level of the station
	u8  lassi;   		// The current adjacent channel strength for the channel below the currently tuned station
	u8  hassi;   		// The current adjacent channel strength for the channel above the currently tuned station
	u8  assi200; 		// The current adjacent channel strength for the channel 200k above or below the currently tuned station (for FM only)
	u8  mult;    		// The current multipath value of the station
	u8  freqOff; 		// The frequency offset in kHz of the current station from the tuned frequency.
	u8  dev;     		// The current deviation of the station (for FM only)
	u8	mod;			// The current modulation depth of the current station (for AM only)
	u8	afcrl;			//
};

/**
 * Device Name
 */
#define RADIO_SI475X    						"radio-si475x"

/**
 * General Commands
 */
// STATUS bits - Used by all methods
#define STCINT  								0x01
#define ACFINT  								0x02
#define RDSINT  								0x04
#define SAMEINT 								0x04
#define RSQINT  								0x08
#define ASQINT  								0x10
#define ERR     								0x40
#define CTS     								0x80

// Boot Commands
#define POWER_UP          						0x01
#define PART_INFO         						0x02

// Common Commands
#define POWER_DOWN        						0x11
#define FUNC_INFO         						0x12
#define SET_PROPERTY      						0x13
#define GET_PROPERTY      						0x14
#define GET_INT_STATUS    						0x15
#define AGC_STATUS        						0x17
#define DIG_AUDIO_PIN_CFG 						0x18
#define ZIF_PIN_CFG       						0x19
#define IC_LINK_PIN_CFG   						0x1A
#define ANA_AUDIO_PIN_CFG 						0x1B

// FM Receive Commands
#define FM_TUNE_FREQ      						0x30
#define FM_SEEK_START     						0x31
#define FM_RSQ_STATUS     						0x32
#define FM_ACF_STATUS     						0x35

// FM Receive RDS Commands
#define FM_RDS_STATUS     						0x36
#define FM_RDS_BLOCKCOUNT 						0x37

// AM Receive Commands
#define AM_TUNE_FREQ      						0x40
#define AM_SEEK_START     						0x41
#define AM_RSQ_STATUS     						0x42
#define AM_ACF_STATUS     						0x45
#define AM_AGC_STATUS     						0x47

// WB Receive Commands
#define WB_TUNE_FREQ      						0x50
#define WB_SEEK_START     						0x51
#define WB_RSQ_STATUS     						0x52
#define WB_ASQ_STATUS     						0x54
#define WB_ACF_STATUS     						0x55

// WB Receive SAME Commands
#define WB_SAME_STATUS    						0x56
#define WB_SAME_SOFTBYTE  						0x57

// General Properties
#define INT_CTL_ENABLE                        	0x0000
#define AUDIO_ANALOG_VOLUME                   	0x0300
#define AUDIO_MUTE                            	0x0301

// AM Receive Properties
#define AM_SEEK_BAND_BOTTOM                   	0x1100
#define AM_SEEK_BAND_TOP                      	0x1101
#define AM_SEEK_FREQUENCY_SPACING             	0x1102
#define AM_VALID_SNR_THRESHOLD                	0x2003
#define AM_VALID_RSSI_THRESHOLD               	0x2004

// FM Receive Properties
#define AUDIO_DE_EMPHASIS                     	0x0302
#define FM_AGC_FE_CONFIG                      	0x0700
#define FM_AGC_PD_CONFIG                      	0x0701
#define FM_SEEK_BAND_BOTTOM                   	0x1100
#define FM_SEEK_BAND_TOP                      	0x1101
#define FM_SEEK_FREQUENCY_SPACING             	0x1102
#define FM_VALID_SNR_THRESHOLD                	0x2003
#define FM_VALID_RSSI_THRESHOLD               	0x2004
#define FM_CAL_CAPACITOR					  	0x8000
#define FM_CAL_FMAX							  	0x8001

// FM RDS Receive Properties
#define FM_RDS_INTERRUPT_SOURCE               	0x4000
#define FM_RDS_INTERRUPT_FIFO_COUNT           	0x4001
#define FM_RDS_CONFIG                         	0x4002

// PowerUp delay in milliseconds (for Si475x parts is 450msec)
#define POWERUP_TIME 							450
// Delay required for 4MHz crystal to boot.
#define CRYSTAL_BOOT_TIME 						40

static int si475x_command(struct si475x_device *radio, u8 cmd_size, u8 *cmd, u8 reply_size, u8 *reply)
{
	int ret = 0;

    // Write the command to the part
    ret = i2c_master_send(radio->client, cmd, cmd_size);
    if(ret < 0)
    	return ret;

    // If the calling function would like to have results then read them.
    if(reply_size)
    {
    	ret = i2c_master_recv(radio->client, reply, reply_size);
    	if(ret < reply_size)
    		return -EIO;
    }

    return 0;
}

static int si475x6x_set_property(struct si475x_device *radio, u16 propNumber, u16 propValue)
{
	u8 cmd[6];

    // Put the ID for the command in the first byte.
    cmd[0] = SET_PROPERTY;

	// Initialize the reserved section to 0
    cmd[1] = 0;

	// Put the property number in the third and fourth bytes.
    cmd[2] = (u8)(propNumber >> 8);
	cmd[3] = (u8)(propNumber & 0x00FF);

	// Put the property value in the fifth and sixth bytes.
    cmd[4] = (u8)(propValue >> 8);
    cmd[5] = (u8)(propValue & 0x00FF);

    // Invoke the command
	return si475x_command(radio, 6, cmd, 0, NULL);
}

static void si475x_get_status(struct si475x_device *radio, struct si475x_data *data, u8 attune, u8 cancel, u8 intack)
{
	u8 cmd[2];
	u8 rsp[16];

    // Put the ID for the command in the first byte.
	if (radio->band == Si475x_AM)
		cmd[0] = AM_RSQ_STATUS;
	else
		cmd[0] = FM_RSQ_STATUS;

	// Depending on the passed parameters set the second argument
	cmd[1] = 0;

	if (intack)
		cmd[1] |= 0x01;
	if (cancel)
		cmd[1] |= 0x02;
	if (attune)
		cmd[1] |= 0x04;

	// Parse the results
	if (radio->band == Si475x_AM)
	{
		// Invoke the command
		si475x_command(radio, 2, cmd, 13, rsp);

		data->status 	= rsp[0];
		data->rsqInts 	= rsp[1];
		data->valid 	= !!(rsp[2] & 0x01);
		data->afcrl 	= !!(rsp[2] & 0x02);
		data->bltf 		= !!(rsp[2] & 0x80);
		data->freq 		= ((u16) rsp[3] << 8) | (u16) rsp[4];
		data->freqOff 	= rsp[5];
		data->rssi 		= rsp[6];
		data->snr 		= rsp[7];
		data->lassi 	= rsp[9];
		data->hassi 	= rsp[10];
		data->mod 		= rsp[12];
	}
	else
	{
		// Invoke the command
		si475x_command(radio, 2, cmd, 16, rsp);

	    // Parse the results
		data->status  	= rsp[0];
		data->rsqInts 	= rsp[1];
		data->valid   	= !!(rsp[2] & 0x01);
		data->afcrl   	= !!(rsp[2] & 0x02);
		data->bltf    	= !!(rsp[2] & 0x80);
		data->freq   	= ((u16)rsp[3] << 8) | (u16)rsp[4];
		data->freqOff 	= rsp[5];
		data->rssi   	= rsp[6];
		data->snr   	= rsp[7];
		data->lassi  	= rsp[9];
		data->hassi  	= rsp[10];
		data->mult   	= rsp[11];
		data->dev    	= rsp[12];
		data->assi200	= rsp[15];
	}
}

static u8 si475x_int_status(struct si475x_device *radio)
{
    u8 cmd;
    u8 rsp;

    // Put the ID for the command in the first byte.
    cmd = GET_INT_STATUS;

    // Invoke the command
	si475x_command(radio, 1, &cmd, 1, &rsp);

	// Return the status
	return rsp;
}

static int si475x_powerup(struct si475x_device *radio)
{
	u8 cmd[6];
	u8 rsp;

	switch(radio->pdata->xtalFreq)
	{
		// If a 4MHz crystal is to be used then perform a multiple boot sequence
		// to allow the crystal enough time to boot before doing a full powerup
		// on the Si475x device.
		case Si475x_4000kHz:
		{
			// Put the ID for the command in the first byte.
			cmd[0] = POWER_UP;
			// This parameter should always be set to F7 for any crystal except 4MHz
			cmd[1] = 0x77;
			// Set the crystal load parameter to the second argument
			cmd[2] = radio->pdata->xtalLoad;
			// This value is for starting up the 4MHz crystal.
			cmd[3] = 0x05;

			if(radio->pdata->xtalResistor)
			{
				cmd[3] += 0x22;
			}

			// Set the 4th parameter to the clock frequency.
			cmd[4] = radio->pdata->xtalFreq;

			// Add the boot mode to the forth parameter
			cmd[4] |= (radio->band << 4);

			if(radio->pdata->xtalResistor)
			{
				cmd[5] = 0x00;
			}
			else
			{
				cmd[5] = 0x01;
			}

			i2c_master_send(radio->client, cmd, 6);

			// Wait the crystal to power up
			msleep(CRYSTAL_BOOT_TIME);

			// Write this command to force CTS for the next power up command.
			cmd[0] = 0xFB;
			cmd[1] = 0x06;
			cmd[2] = 0x80;
			i2c_master_send(radio->client, cmd, 3);

			// Set the values back in the command for the final powerup.
			cmd[0] = POWER_UP;
			cmd[1] = 0x77;
			cmd[2] = radio->pdata->xtalLoad;

			// This value is the final crystal value for 4MHz.
			cmd[3] = 0x03;

			if(radio->pdata->xtalResistor)
			{
				cmd[3] += 0x20;
			}

			// Set the 4th parameter to the clock frequency.
			cmd[4] = radio->pdata->xtalFreq;

			// Add the boot mode to the forth parameter.
			cmd[4] |= (radio->band << 4);

			// This is the value required for final boot with a 4MHz crystal.
			cmd[5] = 0x11;

			// Powerup the device
			si475x_command(radio, 6, cmd, 1, &rsp);
			msleep(POWERUP_TIME);
			break;
		}
		case Si475x_37209kHz:
		case Si475x_36400kHz:
		case Si475x_37800kHz:
		{
			// Put the ID for the command in the first byte.
			cmd[0] = POWER_UP;
			// This parameter should always be set to F7 for any crystal except 4MHz
			cmd[1] = 0xF7;
			// Set the crystal load parameter to the second argument
			cmd[2] = radio->pdata->xtalLoad;

			// This is the required boot setting for all crystals besides the 4MHz crystal.
			cmd[3] = 0x05;

			// Set the 4th parameter to the clock frequency.
			cmd[4] = radio->pdata->xtalFreq;

			// Add the boot mode to the forth parameter
			cmd[4] |= (radio->band << 4);

			// Initialize the 5th parameter to 0x10
			cmd[5] = 0x10;

			// Or 1 if a crystal is being used otherwise or in 2
			cmd[5] |= 1; // Crystal clock

			// Powerup the device
			si475x_command(radio, 6, cmd, 1, &rsp);
			msleep(POWERUP_TIME);
			break;
		}
		default:
			BUG();
			return -EINVAL;
	}

	return 0;
}

static int si475x_powerdown(struct si475x_device *radio)
{
	u8 cmd;
	u8 rsp;

	// Put the ID for the command in the first byte.
	cmd = POWER_DOWN;

	// Invoke the command
	return si475x_command(radio, 1, &cmd, 1, &rsp);
}

static int si475x_initialize(struct si475x_device *radio)
{
	int retval = 0;
	u8 cmd[2];

	// Configure Stereo Analog Audio Output
	// --------------------------------------------------------------------------
	cmd[0] = ANA_AUDIO_PIN_CFG;
	cmd[1] = 2;
	retval |= si475x_command(radio, 2, cmd, 0, NULL);

	// Tunning parameters for frequency seek (SNR and RSSI thresholds)
	// --------------------------------------------------------------------------
	retval |= si475x6x_set_property(radio, FM_VALID_SNR_THRESHOLD, 0x08);
	retval |= si475x6x_set_property(radio, FM_VALID_RSSI_THRESHOLD, 0x0C);
	retval |= si475x6x_set_property(radio, AM_VALID_SNR_THRESHOLD, 0x05);
	retval |= si475x6x_set_property(radio, AM_VALID_RSSI_THRESHOLD, 0x0A);

	// Regional Configurations: compatible with Europe, US, BR, Japan, China
	// Let the userspace clamp it according to its regional configuration
	// --------------------------------------------------------------------------
	retval |= si475x6x_set_property(radio, AUDIO_DE_EMPHASIS, 1); 			// 0 = 75us, 1 = 50us
	retval |= si475x6x_set_property(radio, FM_SEEK_BAND_BOTTOM, 7000);		// FM Bottom Band = 70 MHz
	retval |= si475x6x_set_property(radio, FM_SEEK_BAND_TOP, 10800);		// FM Top Band = 108.0 MHz
	retval |= si475x6x_set_property(radio, FM_SEEK_FREQUENCY_SPACING, 10); 	// 100 kHz Spacing for FM
	retval |= si475x6x_set_property(radio, AM_SEEK_FREQUENCY_SPACING, 10); 	// 10 kHz Spacing for AM
	retval |= si475x6x_set_property(radio, AM_SEEK_BAND_BOTTOM, 520); 		// FM Bottom Band = 520 kHz
	retval |= si475x6x_set_property(radio, AM_SEEK_BAND_TOP, 1720);   		// AM Bottom Band = 1720 kHz

	// Enable RDS reception and its Interrupt
	// --------------------------------------------------------------------------
	// Enable the RDS and STC interrupt here
	retval |= si475x6x_set_property(radio, INT_CTL_ENABLE, 0x0005);
	// This interrupt will be used to determine when RDS is available.
	retval |= si475x6x_set_property(radio, FM_RDS_INTERRUPT_SOURCE, 0x0002);
	// Enable the RDS and allow all blocks so we can compute the error rate later.
	retval |= si475x6x_set_property(radio, FM_RDS_CONFIG, 0x00F1);

	return retval;
}

static int si475x_get_rssi(struct si475x_device *radio)
{
	struct si475x_data data;

	// When attune is set to 0 the RSSI returned will be the RSSI at the time
	// the RSQ status command was called.  If 1 is set to attune then the RSSI
	// measured during the last tune or seek will be returned.
	si475x_get_status(radio, &data, 0, 0, 0);

	return data.rssi;
}

static int si475x_tune(struct si475x_device *radio, u8 tuneMode, int frequency)
{
	struct si475x_data data;
	u8 cmd[6];
	u8 rsp;
	int ret = 0;

	// Frequency is received in kHz from userspace
	// But Si475x works with multiples of 10kHz
	frequency /= 10;

	// Put the ID for the command in the first byte.
	if(radio->band == Si475x_AM)
	{
		cmd[0] = AM_TUNE_FREQ;

		// Depending on what tune functionality is needed or in the appropriate bits
		cmd[1] = 0;

		// Determine if HD ready mode is needed (Si4761/63/65/67 only)
		cmd[1] |= 0; // 0 = Normal bandwidth, 0x40 = wide bandwidth/HD mode

		// Put the frequency in the second and third bytes.
		cmd[2] = (u8) (frequency >> 8);
		cmd[3] = (u8) (frequency & 0x00FF);

		// Set the antenna calibration value.
		cmd[4] = (u8) 0;  // Auto
		cmd[5] = (u8) 0;

		// Invoke the command
		ret = si475x_command(radio, 6, cmd, 1, &rsp);
	}
	else
	{
		cmd[0] = FM_TUNE_FREQ;

		// Depending on what tune functionality is needed or in the appropriate bits
		cmd[1] = 0;

		// Determine if the metrics should be smoothly transitioned after a tune
		cmd[1] |= 0; // 0 = Metrics match new channel, 4 = trans. from prev. channel

		// Set the tune mode
		if (tuneMode <= 3)
			cmd[1] |= (tuneMode << 4) & 0xFF;
		else
			cmd[1] |= 0; // Normal tune if value wasn't recognized.

		// Determine if HD ready mode is needed
		cmd[1] |= 0; // 0 = Normal bandwidth, 0x40 = wide bandwidth/HD mode

		// Put the frequency in the second and third bytes.
		cmd[2] = (u8) (frequency >> 8);
		cmd[3] = (u8) (frequency & 0x00FF);

		// Set the antenna calibration value.
		cmd[4] = (u8) 0;  // Auto
		cmd[5] = (u8) 0;

		// Invoke the command
		ret = si475x_command(radio, 6, cmd, 1, &rsp);
	}

	if(ret)
		return ret;

	// Wait for stc bit to be set
	wait_for_completion_interruptible(&radio->work_stc);

	// Clear the STC bit and get the results of the tune.
	si475x_get_status(radio, &data, 1, 0, 1);

	return 0;
}

static int si475x_seek(struct si475x_device *radio, u8 seekUp, u8 wrapAround)
{
	struct si475x_data data;
	u8 cmd[2];
	u8 rsp;
	int ret = 0;

	if(radio->band == Si475x_AM)
	{
		// Put the ID for the command in the first byte.
		cmd[0] = AM_SEEK_START;

		// Put the flags if the bit was set for the input parameters.
		cmd[1] = 0;
		if (seekUp)
			cmd[1] |= 0x08;
		if (wrapAround)
			cmd[1] |= 0x04;

		// Invoke the command
		ret = si475x_command(radio, 2, cmd, 1, &rsp);
	}
	else
	{
		// Put the ID for the command in the first byte.
		cmd[0] = FM_SEEK_START;

		// Put the flags if the bit was set for the input parameters.
		cmd[1] = 0;
		if (seekUp)
			cmd[1] |= 0x08;
		if (wrapAround)
			cmd[1] |= 0x04;

		// Invoke the command
		ret = si475x_command(radio, 2, cmd, 1, &rsp);
	}

	if(ret)
		return ret;

	// Wait for STC bit to be set
	wait_for_completion_interruptible(&radio->work_stc);

	// Clear the STC bit and get the results of the tune.
	si475x_get_status(radio, &data, 1, 0, 1);

	return 0;
}

static int si475x_get_frequency(struct si475x_device *radio)
{
	struct si475x_data data;

	// Get the tune status which contains the current frequency
	si475x_get_status(radio, &data, 1, 0, 0);

	// Returns the frequency in kHz to the userspace
	return (radio->band == Si475x_AM) ? (data.freq) : (data.freq * 10);
}

static int vidioc_s_tuner(struct file *file, void *priv, struct v4l2_tuner *v)
{
	struct si475x_device *radio;
	int freq;
	int ret = 0;

	radio = video_get_drvdata(video_devdata(file));

	switch(v->index)
	{
		case 1:
			radio->band = Si475x_AM;
			freq = default_freq_am;
			break;
		case 2:
			radio->band = Si475x_FM;
			freq = default_freq_fm;
			break;
		default:
			return -EINVAL;
	}

	// Restarts the device
	ret = si475x_powerdown(radio);
	if(ret)
		goto out;

	msleep(100);

	ret = si475x_powerup(radio);
	if(ret)
		goto out;

	ret = si475x_initialize(radio);
	if(ret)
		goto out;

	ret = si475x_tune(radio, 0, freq);
	if(ret)
		goto out;

	return 0;

out:
	return -EIO;
}

static int vidioc_g_tuner(struct file *file, void *priv, struct v4l2_tuner *v)
{
	struct si475x_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	switch(v->index) {
		case 1:
			strlcpy(v->name, "AM", sizeof(v->name));
			v->type = V4L2_TUNER_RADIO;
			v->rangelow = 520;
			v->rangehigh = 1720;
			v->capability = V4L2_TUNER_CAP_LOW;
			v->rxsubchans = V4L2_TUNER_SUB_MONO;
			v->audmode = V4L2_TUNER_MODE_STEREO;
			break;
		case 2:
			strlcpy(v->name, "FM", sizeof(v->name));
			v->type = V4L2_TUNER_RADIO;
			v->rangelow = 70000;
			v->rangehigh = 108000;
			v->capability = V4L2_TUNER_CAP_LOW;
			v->rxsubchans = V4L2_TUNER_SUB_MONO;
			v->audmode = V4L2_TUNER_MODE_STEREO;
			break;
		default:
			return -EINVAL;
	}
	v->signal = si475x_get_rssi(radio);
	return 0;
}

static int vidioc_s_frequency(struct file *file, void *priv, struct v4l2_frequency *f)
{
	struct si475x_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	return si475x_tune(radio, 0, f->frequency);
}

static int vidioc_g_frequency(struct file *file, void *priv, struct v4l2_frequency *f)
{
	struct si475x_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	f->frequency = si475x_get_frequency(radio);
	f->tuner = radio->band;
	f->type = V4L2_TUNER_RADIO;
	return 0;
}

static int vidioc_s_hw_freq_seek(struct file *file, void *fh, struct v4l2_hw_freq_seek *a)
{
	struct si475x_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	return si475x_seek(radio, a->seek_upward, a->wrap_around);
}

static int vidioc_querycap(struct file *file, void  *priv, struct v4l2_capability *v)
{
	struct si475x_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	strlcpy(v->driver, RADIO_SI475X, sizeof(v->driver));
	strlcpy(v->card, "Silicon Labs Si475x AM/FM Tuner", ARRAY_SIZE(v->card));
	v->version = KERNEL_VERSION(0,1,0);
	v->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO | V4L2_CAP_HW_FREQ_SEEK;

	return 0;
}

static int si475x_fops_open(struct file *file)
{
	struct si475x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety check */
	if (!radio)
		return -ENODEV;

	mutex_lock(&radio->lock);
	radio->users++;

	if (radio->users == 1)
	{
		radio->band = default_band;

		// turns on the radio device
		if(radio->pdata->powerup)
			radio->pdata->powerup();

		/* start radio */
		retval |= si475x_powerup(radio);
		retval |= si475x_initialize(radio);
		retval |= si475x_tune(radio, 0, default_freq_fm);
	}

	mutex_unlock(&radio->lock);

	if(retval)
		return -EIO;

	return 0;
}

static int si475x_fops_release(struct file *file)
{
	struct si475x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety check */
	if (!radio)
		return -ENODEV;

	mutex_lock(&radio->lock);

	radio->users--;
	if (radio->users == 0)
	{
		/* stops radio */
		retval = si475x_powerdown(radio);

		// turns off the radio device
		if(radio->pdata->powerdown)
			radio->pdata->powerdown();
	}

	mutex_unlock(&radio->lock);

	return retval;
}

/* File system interface */
static const struct v4l2_file_operations si475x_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	// TODO: For RDS
	.read			= NULL,
	.poll			= NULL,
	.open			= si475x_fops_open,
	.release		= si475x_fops_release,
};

static const struct v4l2_ioctl_ops si475x_ioctl_ops = {
	.vidioc_querycap    	= vidioc_querycap,
	.vidioc_g_tuner     	= vidioc_g_tuner,
	.vidioc_s_tuner     	= vidioc_s_tuner,
	.vidioc_g_frequency 	= vidioc_g_frequency,
	.vidioc_s_frequency 	= vidioc_s_frequency,
	.vidioc_s_hw_freq_seek 	= vidioc_s_hw_freq_seek,
};

/* V4L2 interface */
static struct video_device si475x_radio_template = {
	.name		= "Si475x AM/FM Radio",
	.fops       = &si475x_fops,
	.ioctl_ops 	= &si475x_ioctl_ops,
	.release	= video_device_release,
};

static void si475x_i2c_interrupt_work(struct work_struct *work)
{
	struct si475x_device *radio = container_of(work, struct si475x_device, radio_work);
	struct si475x_data data;

	if (si475x_int_status(radio) & STCINT)
	{
		complete(&radio->work_stc);
		return;
	}

	// Unknown interrupt... just clears it
	si475x_get_status(radio, &data, 1, 0, 0);
}

static irqreturn_t si475x_i2c_interrupt(int irq, void *dev_id)
{
	struct si475x_device *radio = dev_id;

	if (!work_pending(&radio->radio_work))
		schedule_work(&radio->radio_work);

	return IRQ_HANDLED;
}

static int __devinit si475x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct si475x_device *radio;
	int ret;

	radio = kzalloc(sizeof(struct si475x_device), GFP_KERNEL);
	if (!radio)
	{
		dev_err(&(client->dev), "failed to alloc memory for device structure.\n");
		return -ENOMEM;
	}

	INIT_WORK(&radio->radio_work, si475x_i2c_interrupt_work);
	radio->dev = &(client->dev);
	radio->client = client;
	radio->pdata = client->dev.platform_data;
	radio->users = 0;
	mutex_init(&radio->lock);
	init_completion(&radio->work_stc);

	radio->videodev = video_device_alloc();
	if (!(radio->videodev))
	{
		ret = -ENOMEM;
		dev_err(radio->dev, "failed to alloc v4l2 device.\n");
		goto errfr;
	}

	memcpy(radio->videodev, &si475x_radio_template, sizeof(si475x_radio_template));

	i2c_set_clientdata(client, radio);
	video_set_drvdata(radio->videodev, radio);

	ret = video_register_device(radio->videodev, VFL_TYPE_RADIO, radio_nr);
	if (ret < 0)
	{
		dev_err(radio->dev, "failed to register v4l2 device.\n");
		goto errrel;
	}

	ret = request_irq(client->irq, si475x_i2c_interrupt, IRQF_TRIGGER_FALLING, RADIO_SI475X, radio);
	if (ret)
	{
		dev_err(radio->dev, "failed to register interrupt\n");
		goto err_irq;
	}

	dev_info(radio->dev, "probed!.\n");

	return 0;

err_irq:
	video_unregister_device(radio->videodev);
errrel:
	video_device_release(radio->videodev);
errfr:
	mutex_destroy(&radio->lock);
	cancel_work_sync(&radio->radio_work);
	kfree(radio);
	return ret;
}

static int __devexit
si475x_i2c_remove(struct i2c_client *client)
{
	struct si475x_device *radio = i2c_get_clientdata(client);
	struct device *dev = radio->dev;

	if (radio)
	{
		mutex_destroy(&radio->lock);
		free_irq(client->irq, radio);
		cancel_work_sync(&radio->radio_work);
		video_unregister_device(radio->videodev);
		video_device_release(radio->videodev);
		kfree(radio);
	}

	dev_info(dev, "removed!.\n");

	return 0;
}

#ifdef CONFIG_PM
static int si475x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si475x_device *radio = i2c_get_clientdata(client);

	return si475x_powerdown(radio);
}

static int si475x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si475x_device *radio = i2c_get_clientdata(client);
	int freq;
	int ret = 0;

	si475x_powerup(radio);
	ret = si475x_initialize(radio);

	if(ret)
		return -EIO;

	freq = si475x_get_frequency(radio);

	return si475x_tune(radio, 0, freq);
}

// suspend mechanism
static const struct dev_pm_ops si475x_pm_ops = {
		.suspend = si475x_suspend,
		.resume = si475x_resume,
};
#endif

/* I2C subsystem interface */
static const struct i2c_device_id si475x_id[] = {
	{ RADIO_SI475X, 0 },
	{ }							/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, si475x_id);

static struct i2c_driver si475x_i2c_driver = {
	.driver = {
		.name = RADIO_SI475X,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm 	= &si475x_pm_ops,
#endif
	},
	.probe = si475x_i2c_probe,
	.remove = __devexit_p(si475x_i2c_remove),
	.id_table = si475x_id,
};

/* init the driver */
static int __init
si475x_init(void)
{
	int error = 0;
	// register in the i2c-core
	error = i2c_add_driver(&si475x_i2c_driver);
	if(error)
	{
		pr_err("%s: cant add i2c driver\n", RADIO_SI475X);
		goto err_i2c_add;
	}

	pr_info("%s: init ok\n", RADIO_SI475X);
	return 0;

	err_i2c_add:
		return error;
}

/* cleanup the driver */
static void __exit
si475x_exit(void)
{
	i2c_del_driver(&si475x_i2c_driver);

	pr_info("%s: exit ok\n", RADIO_SI475X);
}

MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("Silicon Labs Si475x AM/FM Radio Device Driver");
MODULE_LICENSE("GPL");

module_init(si475x_init);
module_exit(si475x_exit);
