/*
 * quanta-tef6606.c
 *
 *	This is the kernel driver for NXP TEF6606 AM/FM Tuner device.
 *	It is an I2C device, so, I2C is needed.
 *
 *  Created on: 29/08/2013
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

#define RADIO_TEF6606_QUANTA_DRIVER_NAME    "tef6606"

#define FM_MODE               				0x01
#define AM_MODE               				0x02
#define AM_MIN_COUNTER        				0xFF    // 520KHz
#define AM_MAX_COUNTER        				120     // 1720KHz
#define FM_MIN_COUNTER        				0xFF    // 87.5MHz
#define FM_MAX_COUNTER        				205     // 108 MHz

#define TUNER_AM_MIN_FREQ     				520     // 520 KHz
#define TUNER_FM_MIN_FREQ     				1750    // 87.5 MHz

#define TUNER_AM_MIN_FREQ_KHZ				520
#define TUNER_AM_MAX_FREQ_KHZ				1720
#define TUNER_FM_MIN_FREQ_KHZ				87500
#define TUNER_FM_MAX_FREQ_KHZ				108000

// As this driver works on frequencies multiple of 62.5 Hz,
// lets create some additional macros
#define TUNER_AM_MIN_LOW_FREQ				TUNER_AM_MIN_FREQ_KHZ	// (TUNER_AM_MIN_FREQ_KHZ * 1000 / 62.5)
#define TUNER_AM_MAX_LOW_FREQ				TUNER_AM_MAX_FREQ_KHZ	// (TUNER_AM_MAX_FREQ_KHZ * 1000 / 62.5)
#define TUNER_FM_MIN_LOW_FREQ				TUNER_FM_MIN_FREQ_KHZ	// (TUNER_FM_MIN_FREQ_KHZ * 1000 / 62.5)
#define TUNER_FM_MAX_LOW_FREQ				TUNER_FM_MAX_FREQ_KHZ	// (TUNER_FM_MAX_FREQ_KHZ * 1000 / 62.5)

// Main state machine
#define TUNER_INIT            				0x01
#define TUNER_SET_FREQUENCY    				0x03
#define TUNER_SEARCH_UP       				0x04
#define TUNER_SEARCH_DOWN     				0x05
#define TUNER_BAND_SELECT     				0x06
#define TUNER_IDLE            				0xFF

// search state machine
#define  TUNER_SEARCH_UP_SET      			0x01
#define  TUNER_SEARCH_UP_READ     			0x02
#define  TUNER_SEARCH_UP_CHECK    			0x03
#define  TUNER_SEARCH_DOWN_SET    			0x11
#define  TUNER_SEARCH_DOWN_READ   			0x12
#define  TUNER_SEARCH_DOWN_CHECK  			0x13
#define  TUNER_SEARCH_IDLE        			0xFF

// timeouts
#define TUNER_SEARCH_TIMEOUT_TO_READ    	4 		// 20ms
//#define TUNER_TIMEOUT_READ             	100 	// 1s
#define TUNER_TIMEOUT_READ             		30 		// 300ms

// SEARCH parameters
#define FM_LEVEL_MIN                  		50
#define FM_LEVEL_MAX                  		200
#define FM_USN_LEVEL                  		3
#define FM_WAN_LEVEL_MIN              		2
#define FM_WAN_LEVEL_MAX              		6
#define FM_IFC_LEVEL                  		2

#define AM_LEVEL_MIN                  		70
#define AM_LEVEL_MAX                  		140
#define AM_IFC_LEVEL_MIN              		1
#define AM_IFC_LEVEL_MAX              		4
#define TUNER_SEARCH_MAX_CNT          		6 		// 6 retries

// Good tuning
//#define AGC_GAIN_1                    	0		// -1,0V
#define AGC_GAIN_1                    		32    	// -0,5V
#define AGC_GAIN_2                    		64    	//  0,0V
#define AGC_GAIN_3                    		96    	// +0,5V
#define AGC_GAIN_4                    		0       // -1,0V
#define AGC_GAIN_5                    		32      // -0,7V
#define AGC_GAIN_6                    		57      // -0,9V
#define AGC_GAIN_7                    		70      //  0,1V
#define AGC_GAIN_8                    		96      // +0,3V
#define AGC_GAIN_9                    		127     // +1,0V

/*
#define AGC_GAIN_4                    		32      // -0,5V
#define AGC_GAIN_5                    		45      // -0,7V
#define AGC_GAIN_6                    		57      // -0,9V
#define AGC_GAIN_7                    		70      // +1,1V
#define AGC_GAIN_8                    		83      // +1,3V
#define AGC_GAIN_9                    		96      // +1,5V
*/

#define DELTA_AGC_GAIN_1              		25
#define DELTA_AGC_GAIN_2              		50
#define DELTA_AGC_GAIN_3              		75
#define DELTA_AGC_GAIN_4              		100
#define DELTA_AGC_GAIN_5              		125
#define DELTA_AGC_GAIN_6              		150
#define DELTA_AGC_GAIN_7              		175


// I2C Slave Address (0xC0 >> 1)
#define TUNER_SLAVE_ADDRESS   				0x60

// read control registers
#define TUNER_READ_STATUS_ADDR      		0x00
#define TUNER_READ_LEVEL_ADDR       		0x01
#define TUNER_READ_USN_WAN_ADDR     		0x02
#define TUNER_READ_IF_COUNTER_ADDR  		0x03
#define TUNER_READ_ID_ADDR          		0x04

// number of bytes to read/write
#define TUNER_WRITE_READ_QTD              	1
#define TUNER_READ_TOTAL_STATUS_QTD       	5


// write control registers
#define TUNER_ADDR_00         				0x00
#define TUNER_ADDR_01         				0x01
#define TUNER_ADDR_02         				0x02
#define TUNER_ADDR_03         				0x03
#define TUNER_ADDR_04         				0x04
#define TUNER_ADDR_05         				0x05
#define TUNER_ADDR_06         				0x06
#define TUNER_ADDR_07         				0x07
#define TUNER_ADDR_08         				0x08
#define TUNER_ADDR_09         				0x09
#define TUNER_ADDR_0A         				0x0A
#define TUNER_ADDR_0B         				0x0B
#define TUNER_ADDR_0C         				0x0C
#define TUNER_ADDR_0D         				0x0D
#define TUNER_ADDR_0E         				0x0E
#define TUNER_ADDR_0F         				0x0F

// Operating Mode
#define TUNER_STANDARD        				0x00   // write without tuning action
#define TUNER_PRESET          				0x20   // tune to new station with short mute time;
#define TUNER_SEARCH          				0x40   // tune to new station and stay muted;
#define TUNER_AF_UPDATE       				0x60   // tune to AF station; store AF quality and tune back to main station;
#define TUNER_AF_JUMP         				0x80   // tune to AF station in minimum mute time;
#define TUNER_AF_CHECK        				0xA0   // tune to AF station and stay muted;
#define TUNER_MIRROR          				0xC0   // check current image situation and select injection mode for best result;
#define TUNER_END             				0xE0   // end; release mute from search mode or AF check mode


// TUNER_ADDR_00
#define TUNER_SELECT_FM       				0x20
#define TUNER_SELECT_AM       				0x00

//TUNER_ADDR_02
#define TUNER_AGC_THRES_OFF   				0x00
#define TUNER_AGC_THRES_2     				0x40
#define TUNER_AGC_THRES_4     				0x80
#define TUNER_AGC_THRES_6     				0xC0
#define TUNER_INJ_AUTO        				0x00
#define TUNER_INJ_HIGH_LO     				0x10
#define TUNER_INJ_LOW_LO      				0x20
#define TUNER_FM_BW_DINAMIC   				0x00
#define TUNER_FM_N_TO_W_1     				0x01
#define TUNER_FM_N_TO_W_2     				0x02
#define TUNER_FM_N_TO_W_3     				0x03
#define TUNER_FM_N_TO_W_4     				0x04
#define TUNER_FM_N_TO_W_5     				0x05
#define TUNER_FM_N_TO_W_6     				0x06
#define TUNER_FM_N_TO_W_7     				0x07

//TUNER_ADDR_03
#define TUNER_AM_FM_NOISE_OFF 				0x00
#define TUNER_AM_FM_NOISE_LOW 				0x40
#define TUNER_AM_FM_NOISE_MED 				0x80
#define TUNER_AM_FM_NOISE_HIG 				0xC0
#define TUNER_NO_HIGH_PASS    				0x00
#define TUNER_YES_HIGH_PASS   				0x20
#define TUNER_FM_MONO         				0x10
#define TUNER_FM_STEREO       				0x00
#define TUNER_DE_EMPH_50US    				0x00
#define TUNER_DE_EMPH_75US    				0x04
#define TUNER_DE_EMPH_103US   				0x08
#define TUNER_OUT_GAIN_HIGH   				0x01
#define TUNER_OUT_GAIN_LOW    				0x00

//TUNER_ADDR_04
#define TUNER_SOFT_MUTE_TIME_60MS       	0x00
#define TUNER_SOFT_MUTE_TIME_125MS      	0x04
#define TUNER_SOFT_MUTE_TIME_250MS      	0x08
#define TUNER_SOFT_MUTE_TIME_500MS      	0x0C
#define TUNER_SOFT_MUTE_TIME_1S         	0x10
#define TUNER_SOFT_MUTE_TIME_2S         	0x14
#define TUNER_SOFT_MUTE_TIME_4S         	0x18
#define TUNER_SOFT_MUTE_TIME_8S         	0x1C
#define TUNER_SOFT_MUTE_RECOVERY_2      	0x00
#define TUNER_SOFT_MUTE_RECOVERY_4      	0x01
#define TUNER_SOFT_MUTE_RECOVERY_8      	0x02
#define TUNER_SOFT_MUTE_RECOVERY_16     	0x03

// TUNER_ADDR_05
#define TUNER_SOFT_MUTE_ON_FAST      		0x80
#define TUNER_SOFT_MUTE_OFF_FAST     		0x00
#define TUNER_SOFT_MUTE_ON_SLOW      		0x40
#define TUNER_SOFT_MUTE_OFF_SLOW     		0x00
#define TUNER_SOFT_MUTE_START_1      		0x00
#define TUNER_SOFT_MUTE_START_2      		0x04
#define TUNER_SOFT_MUTE_START_3      		0x08
#define TUNER_SOFT_MUTE_START_4      		0x0C
#define TUNER_SOFT_MUTE_START_5      		0x10
#define TUNER_SOFT_MUTE_START_6      		0x14
#define TUNER_SOFT_MUTE_START_7      		0x18
#define TUNER_SOFT_MUTE_START_8      		0x1C
#define TUNER_SOFT_MUTE_SLOPE_1      		0x00
#define TUNER_SOFT_MUTE_SLOPE_2      		0x01
#define TUNER_SOFT_MUTE_SLOPE_3      		0x02
#define TUNER_SOFT_MUTE_SLOPE_4      		0x03


#define TUNER_FM_SOFT_MUTE_NO_FAST_CNTR    0x00
#define TUNER_FM_SOFT_MUTE_FAST_CNTR       0x80
#define TUNER_FM_SOFT_MUTE_NO_SLOW_CNTR    0x00
#define TUNER_FM_SOFT_MUTE_SLOW_CNTR       0x40
#define TUNER_FM_SOFT_MUTE_NOISE_SENS_1    0x00
#define TUNER_FM_SOFT_MUTE_NOISE_SENS_2    0x10
#define TUNER_FM_SOFT_MUTE_NOISE_SENS_3    0x20
#define TUNER_FM_SOFT_MUTE_NOISE_SENS_4    0x30
#define TUNER_FM_SOFT_MUTE_FAST_WAM        0x08
#define TUNER_FM_SOFT_MUTE_NO_SLOW_WAM     0x00
#define TUNER_FM_SOFT_MUTE_SLOW_WAM        0x04
#define TUNER_FM_SOFT_MUTE_WAN_SENS_1      0x00
#define TUNER_FM_SOFT_MUTE_WAN_SENS_2      0x01
#define TUNER_FM_SOFT_MUTE_WAN_SENS_3      0x02
#define TUNER_FM_SOFT_MUTE_WAN_SENS_4      0x03

#define TUNER_AM_SOFT_MUTE                 0x1E // 30dBs

// TUNER_ADDR_07
#define TUNER_NO_MODULATION                0x00
#define TUNER_MODULATION_30                0x40
#define TUNER_MODULATION_50                0x80
#define TUNER_MODULATION_70                0xC0
#define TUNER_HIGH_CUT_LIMITATION_10       0x00
#define TUNER_HIGH_CUT_LIMITATION_6        0x20
#define TUNER_HIGH_CUT_ATT_TIME_60MS       0x00
#define TUNER_HIGH_CUT_ATT_TIME_125MS      0x04
#define TUNER_HIGH_CUT_ATT_TIME_250MS      0x08
#define TUNER_HIGH_CUT_ATT_TIME_500MS      0x0C
#define TUNER_HIGH_CUT_ATT_TIME_1S         0x10
#define TUNER_HIGH_CUT_ATT_TIME_2S         0x14
#define TUNER_HIGH_CUT_ATT_TIME_4S         0x18
#define TUNER_HIGH_CUT_ATT_TIME_8S         0x1C
#define TUNER_HIGH_CUT_REC_TIME_2          0x00
#define TUNER_HIGH_CUT_REC_TIME_4          0x01
#define TUNER_HIGH_CUT_REC_TIME_8          0x02
#define TUNER_HIGH_CUT_REC_TIME_16         0x03

// TUNER_ADDR_08
#define TUNER_HIGH_CUT_NO_FAST_CNTR        0x00
#define TUNER_HIGH_CUT_FAST_CNTR           0x80
#define TUNER_HIGH_CUT_NO_SLOW_CNTR        0x00
#define TUNER_HIGH_CUT_SLOW_CNTR           0x40
#define TUNER_HIGH_CUT_START_1             0x00
#define TUNER_HIGH_CUT_START_2             0x04
#define TUNER_HIGH_CUT_START_3             0x08
#define TUNER_HIGH_CUT_START_4             0x0C
#define TUNER_HIGH_CUT_START_5             0x10
#define TUNER_HIGH_CUT_START_6             0x14
#define TUNER_HIGH_CUT_START_7             0x18
#define TUNER_HIGH_CUT_START_8             0x1C
#define TUNER_HIGH_CUT_SLOPE_1             0x00
#define TUNER_HIGH_CUT_SLOPE_2             0x01
#define TUNER_HIGH_CUT_SLOPE_3             0x02
#define TUNER_HIGH_CUT_SLOPE_4             0x03

// TUNER_ADDR_09
#define TUNER_HIGH_CUT_NO_FAST_CNTR_NOISE  0x00
#define TUNER_HIGH_CUT_FAST_CNTR_NOISE     0x80
#define TUNER_HIGH_CUT_NO_SLOW_CNTR_NOISE  0x00
#define TUNER_HIGH_CUT_SLOW_CNTR_NOISE     0x40
#define TUNER_HIGH_CUT_SENS_NOISE_1        0x00
#define TUNER_HIGH_CUT_SENS_NOISE_2        0x10
#define TUNER_HIGH_CUT_SENS_NOISE_3        0x20
#define TUNER_HIGH_CUT_SENS_NOISE_4        0x30
#define TUNER_HIGH_CUT_SENS_NOISE_2        0x10
#define TUNER_HIGH_CUT_NO_FAST_WAM         0x00
#define TUNER_HIGH_CUT_FAST_WAM            0x08
#define TUNER_HIGH_CUT_NO_SLOW_WAM         0x00
#define TUNER_HIGH_CUT_SLOW_WAM            0x04
#define TUNER_HIGH_CUT_SENS_WAM_1          0x00
#define TUNER_HIGH_CUT_SENS_WAM_2          0x01
#define TUNER_HIGH_CUT_SENS_WAM_3          0x02
#define TUNER_HIGH_CUT_SENS_WAM_4          0x03

// TUNER_ADDR_0A
#define TUNER_NO_STEREO_MODULATION_CNTR    0x00
#define TUNER_STEREO_TO_MONO_30            0x40
#define TUNER_STEREO_TO_6dB_30             0x80
#define TUNER_STEREO_TO_MONO_15            0xC0
#define TUNER_STEREO_SLOW_ATT_TIME_60MS    0x00
#define TUNER_STEREO_SLOW_ATT_TIME_125MS   0x04
#define TUNER_STEREO_SLOW_ATT_TIME_250MS   0x08
#define TUNER_STEREO_SLOW_ATT_TIME_500MS   0x0C
#define TUNER_STEREO_SLOW_ATT_TIME_1S      0x10
#define TUNER_STEREO_SLOW_ATT_TIME_2S      0x14
#define TUNER_STEREO_SLOW_ATT_TIME_4S      0x18
#define TUNER_STEREO_SLOW_ATT_TIME_8S      0x1C
#define TUNER_STEREO_SLOW_REC_TIME_2       0x00
#define TUNER_STEREO_SLOW_REC_TIME_4       0x01
#define TUNER_STEREO_SLOW_REC_TIME_8       0x02
#define TUNER_STEREO_SLOW_REC_TIME_16      0x03

// TUNER_ADDR_0B
#define TUNER_STEREO_NO_FAST_CNTR          0x00
#define TUNER_STEREO_FAST_CNTR             0x80
#define TUNER_STEREO_NO_SLOW_CNTR          0x00
#define TUNER_STEREO_SLOW_CNTR             0x40
#define TUNER_STEREO_NO_FAST_CNTR          0x00
#define TUNER_STEREO_START_1               0x00
#define TUNER_STEREO_START_2               0x04
#define TUNER_STEREO_START_3               0x08
#define TUNER_STEREO_START_4               0x0C
#define TUNER_STEREO_START_5               0x10
#define TUNER_STEREO_START_6               0x14
#define TUNER_STEREO_START_7               0x18
#define TUNER_STEREO_START_8               0x1C
#define TUNER_STEREO_SLOPE_1               0x00
#define TUNER_STEREO_SLOPE_2               0x01
#define TUNER_STEREO_SLOPE_3               0x02
#define TUNER_STEREO_SLOPE_4               0x03


// TUNER_ADDR_0C
#define TUNER_STEREO_NO_FAST_CNTR_NOISE    0x00
#define TUNER_STEREO_FAST_CNTR_NOISE       0x80
#define TUNER_STEREO_NO_SLOW_CNTR_NOISE    0x00
#define TUNER_STEREO_SLOW_CNTR_NOISE       0x40
#define TUNER_STEREO_SENS_NOISE_1          0x00
#define TUNER_STEREO_SENS_NOISE_2          0x10
#define TUNER_STEREO_SENS_NOISE_3          0x20
#define TUNER_STEREO_SENS_NOISE_4          0x30
#define TUNER_STEREO_NO_FAST_CNTR_WAM      0x00
#define TUNER_STEREO_FAST_CNTR_WAM         0x08
#define TUNER_STEREO_NO_SLOW_CNTR_WAM      0x00
#define TUNER_STEREO_SLOW_CNTR_WAM         0x04
#define TUNER_STEREO_SENS_WAM_1            0x00
#define TUNER_STEREO_SENS_WAM_2            0x01
#define TUNER_STEREO_SENS_WAM_3            0x02
#define TUNER_STEREO_SENS_WAM_4            0x03

// TUNER_ADDR_0D
#define TUNER_PIN_TEST_OPEN                0x00
#define TUNER_PIN_TEST_PULL_DOWN           0x80
#define TUNER_FM_NOISE_PULSE_NOT_LIMITED   0x00
#define TUNER_FM_NOISE_PULSE_LIMITED_400HZ 0x40
#define TUNER_FM_BW_MODULATION_HANDLING    0x00
#define TUNER_FM_BW_INTENT_MOD_HANDLING    0x10
#define TUNER_FM_BW_INTENT_CHANNEL_SUPR    0x20
#define TUNER_FM_BW_ADJAC_CHANNEL_SUPR     0x30
#define TUNER_RESERVED_1                   0x00
#define TUNER_RESERVED_2                   0x04
#define TUNER_FM_NARROW_BW_NOISE_REDUC     0x00
#define TUNER_FM_WIDE_BW_MOD_HANDLING      0x02
#define TUNER_FM_ADJAC_CHANNEL_SUPPRESS    0x00
#define TUNER_FM_MOD_HANDLING              0x01

// TUNER_ADDR_0E

// TUNER_ADDR_0F
#define TUNER_AM_AUTO_INJ_TIME_4MS         0x00
#define TUNER_AM_AUTO_INJ_TIME_8MS         0x20
#define TUNER_AM_MUTE_TIME_4MS             0x00
#define TUNER_AM_MUTE_TIME_7MS             0x10
#define TUNER_AM_STEREO_CHANNEL_NO_ALIGN   0x04
#define TUNER_AM_STEREO_OPTION_SEPA        0x00


// bits status tuner
#define TUNER_BIT_STEREO					0x10

/**
 * V4L2 default radio number
 */
static int radio_nr = -1;
module_param(radio_nr, int, 0);
MODULE_PARM_DESC(radio_nr, "/dev/radioX device number to use");

/**
 * Device control structure
 */
struct tef6606_device {
	struct i2c_client		*i2c_client;
	struct video_device		*videodev;
	struct device			*dev;
	__u32 					dwFrequency;					// Current Frequency in kHz
	unsigned char			byAMFM;							// AM/FM selection
};

/**
 * Base V4L2 Private Control ID
 */
#define TEF6606_CID_PRIVATE_BASE 				(V4L2_CID_PRIVATE_BASE)

static __u16 TUNER_ConvertV4L2FmFrequency(__u32 frequency)
{
	__u16 result = 0;
	__u32 offset = frequency - TUNER_FM_MIN_LOW_FREQ;	// offset in kHz (some **hundreds** of kHz)
	result = TUNER_FM_MIN_FREQ + (offset / 50);			// in this tuner, 1 step = 50kHz
	return result;										// each 100 kHz -> 2 steps -> divide by 50
}

static __u16 TUNER_ConvertV4L2AmFrequency(__u32 frequency)
{
	__u16 result = 0;
	__u32 offset = frequency - TUNER_AM_MIN_LOW_FREQ;	// offset in kHz (some **tens** of kHz)
	result = TUNER_AM_MIN_FREQ + (offset);				// in this tuner, 1 step = 1kHz
	return result;										// each 10 kHz -> 10 steps -> dont divide
}

static void TUNER_SetParameters(struct tef6606_device *radio)
{
	__u8 byBuff[2];
	dev_dbg(radio->dev, "Setting tuner parameters\n");

	// FM
	if (radio->byAMFM == FM_MODE) {
		//TUNER_ADDR_02
		byBuff[0] = TUNER_ADDR_02 | TUNER_PRESET;
		byBuff[1] = TUNER_AGC_THRES_OFF | TUNER_INJ_AUTO | TUNER_FM_BW_DINAMIC;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_03
		byBuff[0] = TUNER_ADDR_03 | TUNER_PRESET;
		byBuff[1] = TUNER_AM_FM_NOISE_OFF | TUNER_NO_HIGH_PASS | TUNER_FM_STEREO
				| TUNER_DE_EMPH_50US | TUNER_OUT_GAIN_LOW;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_04
		byBuff[0] = TUNER_ADDR_04 | TUNER_PRESET;
		byBuff[1] = TUNER_SOFT_MUTE_TIME_250MS | TUNER_SOFT_MUTE_RECOVERY_4;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_05
		byBuff[0] = TUNER_ADDR_05 | TUNER_PRESET;
		byBuff[1] = TUNER_SOFT_MUTE_OFF_FAST | TUNER_SOFT_MUTE_OFF_SLOW
				| TUNER_SOFT_MUTE_START_4 | TUNER_SOFT_MUTE_SLOPE_2;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_06
		byBuff[0] = TUNER_ADDR_06 | TUNER_PRESET;
		byBuff[1] =
				TUNER_FM_SOFT_MUTE_NO_SLOW_CNTR
						| TUNER_FM_SOFT_MUTE_NO_SLOW_CNTR
						| TUNER_FM_SOFT_MUTE_NOISE_SENS_1
						| TUNER_FM_SOFT_MUTE_NO_SLOW_WAM
						| TUNER_FM_SOFT_MUTE_WAN_SENS_1;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_07
		byBuff[0] = TUNER_ADDR_07 | TUNER_PRESET;
		byBuff[1] = TUNER_NO_MODULATION | TUNER_HIGH_CUT_ATT_TIME_500MS
				| TUNER_HIGH_CUT_REC_TIME_8;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_08
		byBuff[0] = TUNER_ADDR_08 | TUNER_PRESET;
		byBuff[1] = TUNER_HIGH_CUT_FAST_CNTR | TUNER_HIGH_CUT_SLOW_CNTR
				| TUNER_HIGH_CUT_START_4 | TUNER_HIGH_CUT_SLOPE_2;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_09
		byBuff[0] = TUNER_ADDR_09 | TUNER_PRESET;
		byBuff[1] = TUNER_HIGH_CUT_NO_FAST_CNTR_NOISE
				| TUNER_HIGH_CUT_NO_SLOW_CNTR_NOISE
				| TUNER_HIGH_CUT_SENS_NOISE_1 | TUNER_HIGH_CUT_NO_FAST_WAM
				| TUNER_HIGH_CUT_SLOW_WAM | TUNER_HIGH_CUT_SENS_WAM_1;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0A
		byBuff[0] = TUNER_ADDR_0A | TUNER_PRESET;
		byBuff[1] =
				TUNER_NO_STEREO_MODULATION_CNTR
						| TUNER_STEREO_SLOW_ATT_TIME_60MS
						| TUNER_STEREO_SLOW_REC_TIME_4;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0B
		byBuff[0] = TUNER_ADDR_0B | TUNER_PRESET;
		byBuff[1] = TUNER_STEREO_FAST_CNTR | TUNER_STEREO_SLOW_CNTR
				| TUNER_STEREO_START_4 | TUNER_STEREO_SLOPE_2;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0C
		byBuff[0] = TUNER_ADDR_0C | TUNER_PRESET;
		byBuff[1] = TUNER_STEREO_NO_FAST_CNTR_NOISE
				| TUNER_STEREO_NO_SLOW_CNTR_NOISE | TUNER_STEREO_SENS_NOISE_1
				| TUNER_STEREO_NO_FAST_CNTR_WAM | TUNER_STEREO_NO_SLOW_CNTR_WAM
				| TUNER_STEREO_SENS_WAM_1;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0D
		byBuff[0] = TUNER_ADDR_0D | TUNER_PRESET;
		byBuff[1] = TUNER_PIN_TEST_OPEN | TUNER_FM_NOISE_PULSE_NOT_LIMITED
				| TUNER_FM_BW_INTENT_MOD_HANDLING | TUNER_RESERVED_2
				| TUNER_FM_ADJAC_CHANNEL_SUPPRESS;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0E
		byBuff[0] = TUNER_ADDR_0E | TUNER_PRESET;
		byBuff[1] = 0x00;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0F
		// byBuff[0] = TUNER_ADDR_0F | TUNER_PRESET;
		// byBuff[1] = TUNER_AM_AUTO_INJ_TIME_4MS | TUNER_AM_MUTE_TIME_7MS | TUNER_AM_STEREO_CHANNEL_NO_ALIGN;
		// i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);
	}
	// AM
	else {
		//TUNER_ADDR_02
		byBuff[0] = TUNER_ADDR_02 | TUNER_PRESET;
		byBuff[1] = TUNER_AGC_THRES_6 | TUNER_INJ_AUTO;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_03
		byBuff[0] = TUNER_ADDR_03 | TUNER_PRESET;
		byBuff[1] = TUNER_AM_FM_NOISE_HIG | TUNER_YES_HIGH_PASS
				| TUNER_FM_STEREO | TUNER_DE_EMPH_75US | TUNER_OUT_GAIN_LOW;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_04
		byBuff[0] = TUNER_ADDR_04 | TUNER_PRESET;
		byBuff[1] = TUNER_SOFT_MUTE_TIME_500MS | TUNER_SOFT_MUTE_RECOVERY_8;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_05
		byBuff[0] = TUNER_ADDR_05 | TUNER_PRESET;
		byBuff[1] = TUNER_SOFT_MUTE_ON_FAST | TUNER_SOFT_MUTE_ON_SLOW
				| TUNER_SOFT_MUTE_START_7 | TUNER_SOFT_MUTE_SLOPE_3;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_06
		byBuff[0] = TUNER_ADDR_06 | TUNER_PRESET;
		byBuff[1] = TUNER_FM_SOFT_MUTE_NO_FAST_CNTR
				| TUNER_FM_SOFT_MUTE_NO_SLOW_CNTR | TUNER_FM_SOFT_MUTE_FAST_WAM
				| TUNER_FM_SOFT_MUTE_WAN_SENS_4;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_07
		byBuff[0] = TUNER_ADDR_07 | TUNER_PRESET;
		byBuff[1] = TUNER_NO_MODULATION | TUNER_HIGH_CUT_ATT_TIME_1S
				| TUNER_HIGH_CUT_REC_TIME_4;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_08
		byBuff[0] = TUNER_ADDR_08 | TUNER_PRESET;
		byBuff[1] = TUNER_HIGH_CUT_FAST_CNTR | TUNER_HIGH_CUT_SLOW_CNTR
				| TUNER_HIGH_CUT_START_8 | TUNER_HIGH_CUT_SLOPE_2;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_09
		byBuff[0] = TUNER_ADDR_09 | TUNER_PRESET;
		byBuff[1] = TUNER_HIGH_CUT_NO_FAST_CNTR_NOISE
				| TUNER_HIGH_CUT_NO_SLOW_CNTR_NOISE
				| TUNER_HIGH_CUT_SENS_NOISE_1 | TUNER_HIGH_CUT_NO_FAST_WAM
				| TUNER_HIGH_CUT_NO_SLOW_WAM | TUNER_HIGH_CUT_SENS_WAM_1;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0A
		byBuff[0] = TUNER_ADDR_0A | TUNER_PRESET;
		byBuff[1] =
				TUNER_NO_STEREO_MODULATION_CNTR
						| TUNER_STEREO_SLOW_ATT_TIME_60MS
						| TUNER_STEREO_SLOW_REC_TIME_2;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0B
		byBuff[0] = TUNER_ADDR_0B | TUNER_PRESET;
		byBuff[1] = TUNER_STEREO_NO_FAST_CNTR | TUNER_STEREO_NO_SLOW_CNTR
				| TUNER_STEREO_NO_FAST_CNTR | TUNER_STEREO_START_1
				| TUNER_STEREO_SLOPE_1;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0C
		byBuff[0] = TUNER_ADDR_0C | TUNER_PRESET;
		byBuff[1] = TUNER_STEREO_NO_FAST_CNTR_NOISE
				| TUNER_STEREO_NO_SLOW_CNTR_NOISE | TUNER_STEREO_SENS_NOISE_1
				| TUNER_STEREO_NO_FAST_CNTR_WAM | TUNER_STEREO_NO_SLOW_CNTR_WAM
				| TUNER_STEREO_SENS_WAM_1;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0D
		byBuff[0] = TUNER_ADDR_0D | TUNER_PRESET;
		byBuff[1] = TUNER_PIN_TEST_OPEN | TUNER_FM_NOISE_PULSE_NOT_LIMITED
				| TUNER_FM_BW_INTENT_MOD_HANDLING | TUNER_RESERVED_2
				| TUNER_FM_ADJAC_CHANNEL_SUPPRESS;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0E
		byBuff[0] = TUNER_ADDR_0E | TUNER_PRESET;
		byBuff[1] = 0x2A;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);

		//TUNER_ADDR_0F
		byBuff[0] = TUNER_ADDR_0F | TUNER_PRESET;
		byBuff[1] = TUNER_AM_AUTO_INJ_TIME_4MS | TUNER_AM_MUTE_TIME_4MS
				| TUNER_AM_STEREO_CHANNEL_NO_ALIGN;
		i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);
	}
}

static void Tuner_SetAGCParameters(struct tef6606_device *radio, __u8 byAGCgain)
{
	__u8 byBuff[2];

	byBuff[0] = TUNER_ADDR_0E | TUNER_AF_JUMP;
	if (radio->byAMFM == FM_MODE)
	{
		//TUNER_ADDR_0E
		byBuff[1] = byAGCgain;
	}
	else
	{
		//TUNER_ADDR_0E
		byBuff[1] = 0x2A;
	}

	i2c_smbus_write_byte_data(radio->i2c_client, byBuff[0], byBuff[1]);
}

static int TUNER_SearchBand(struct tef6606_device *radio)
{
	int ret = 0;
	__u16 wFreq;
	__u8 buffer[2];
	__u8 address;

	if (radio->byAMFM == FM_MODE)
	{
		wFreq = radio->dwFrequency;
		buffer[0] =	(__u8) (TUNER_SELECT_FM | (unsigned int)((wFreq & 0x1F00) >> 8));
		buffer[1] = (__u8) (wFreq & 0x00FF);
	}
	else
	{
		wFreq = radio->dwFrequency;
		buffer[0] = (__u8) (TUNER_SELECT_AM | ((wFreq & 0x1F00) >> 8));
		buffer[1] = (__u8) (wFreq & 0x00FF);
	}

	address = TUNER_ADDR_00 | TUNER_END;

	ret = i2c_smbus_write_i2c_block_data(radio->i2c_client, address, ARRAY_SIZE(buffer), buffer);
	if (ret < 0)
	{
		dev_err(radio->dev, "Error while trying to set new frequency: %d\n", ret);
		return ret;
	}

	return 0;
}

static void TUNER_SearchDown(struct tef6606_device *radio)
{
	__u16 wFreq;
	__u8 buffer[2];
	__u8 address;

	TUNER_SetParameters(radio);

	if (radio->byAMFM == FM_MODE)
	{
		radio->dwFrequency -= 100;	// Subtracts 100kHz (hardcoded now)
		if (radio->dwFrequency < TUNER_FM_MIN_FREQ_KHZ)
			radio->dwFrequency = TUNER_FM_MAX_FREQ_KHZ;

		wFreq = TUNER_ConvertV4L2FmFrequency(radio->dwFrequency);
		buffer[0] = (__u8) (TUNER_SELECT_FM | ((wFreq & 0x1F00) >> 8));
		buffer[1] = (__u8) (wFreq & 0x00FF);
		dev_dbg(radio->dev, "Searching FM Frequency: v4l2=%d; device=%d\n", radio->dwFrequency, wFreq);
	}
	else
	{
		radio->dwFrequency -= 10;	// Subtracts 10kHz (hardcoded now)
		if(radio->dwFrequency < TUNER_AM_MIN_FREQ_KHZ)
			radio->dwFrequency = TUNER_AM_MAX_FREQ_KHZ;

		wFreq = TUNER_ConvertV4L2AmFrequency(radio->dwFrequency);
		buffer[1] = (__u8) (TUNER_SELECT_AM | ((wFreq & 0x1F00) >> 8));
		buffer[2] = (__u8) (wFreq & 0x00FF);
		dev_dbg(radio->dev, "Searching AM Frequency: v4l2=%d; device=%d\n", radio->dwFrequency, wFreq);
	}

	address = TUNER_ADDR_00 | TUNER_SEARCH;
	i2c_smbus_write_i2c_block_data(radio->i2c_client, address, ARRAY_SIZE(buffer), buffer);
}

static void TUNER_SearchUp(struct tef6606_device *radio)
{
	__u16 wFreq;
	__u8 buffer[2];
	__u8 address;

	TUNER_SetParameters(radio);

	if (radio->byAMFM == FM_MODE)
	{
		radio->dwFrequency += 100;	// Adds 100kHz (hardcoded now)
		if (radio->dwFrequency > TUNER_FM_MAX_FREQ_KHZ)
			radio->dwFrequency = TUNER_FM_MIN_FREQ_KHZ;

		wFreq = TUNER_ConvertV4L2FmFrequency(radio->dwFrequency);
		buffer[0] = (__u8) (TUNER_SELECT_FM | ((wFreq & 0x1F00) >> 8));
		buffer[1] = (__u8) (wFreq & 0x00FF);
		dev_dbg(radio->dev, "Searching FM Frequency: v4l2=%d; device=%d\n", radio->dwFrequency, wFreq);
	}
	else
	{
		radio->dwFrequency += 10;	// Adds 10kHz (hardcoded now)
		if(radio->dwFrequency > TUNER_AM_MAX_FREQ_KHZ)
			radio->dwFrequency = TUNER_AM_MIN_FREQ_KHZ;

		wFreq = TUNER_ConvertV4L2AmFrequency(radio->dwFrequency);
		buffer[1] = (__u8) (TUNER_SELECT_AM | ((wFreq & 0x1F00) >> 8));
		buffer[2] = (__u8) (wFreq & 0x00FF);
		dev_dbg(radio->dev, "Searching AM Frequency: v4l2=%d; device=%d\n", radio->dwFrequency, wFreq);
	}

	address = TUNER_ADDR_00 | TUNER_SEARCH;
	i2c_smbus_write_i2c_block_data(radio->i2c_client, address, ARRAY_SIZE(buffer), buffer);
}

__u8 TUNER_CheckSearchRead(struct tef6606_device *radio)
{

  __u8 byLevel, byUSN, byWAN, byIFC;
  __u8 byBufferReadStatus[TUNER_READ_TOTAL_STATUS_QTD];
  __u8 address = TUNER_READ_STATUS_ADDR;
  __u8 result = 0;
  int err = 0;

  err = i2c_smbus_read_i2c_block_data(radio->i2c_client, address, ARRAY_SIZE(byBufferReadStatus), byBufferReadStatus);

  if(err <= 0)
  {
	  dev_err(radio->dev, "Error while reading tuner status: %d\n", err);
	  return 0; // false
  }

  byLevel = byBufferReadStatus[1]; // RSSI
  byUSN = (byBufferReadStatus[2] & 0xF0) >> 4;
  byWAN = (byBufferReadStatus[2] & 0x0F);
  byIFC = (byBufferReadStatus[3] & 0x1F);

  if(radio->byAMFM == FM_MODE)       //FM
  {
	  result =  ((byLevel > FM_LEVEL_MIN) 	&&
			(byLevel < FM_LEVEL_MAX) 	&&
			(byUSN <= FM_USN_LEVEL) 	&&
			(byWAN <= FM_WAN_LEVEL_MAX) &&
			(byIFC < FM_IFC_LEVEL));
  }
  else  // AM
  {
	  result =  ((byLevel > AM_LEVEL_MIN) &&
			  (byLevel < AM_LEVEL_MAX)  &&
			  (byIFC <= AM_IFC_LEVEL_MAX));
  }
  dev_dbg(radio->dev, "Status check: %d\n", result);
  return result;
}

static void Tuner_CheckReadState(struct tef6606_device *radio) {

	__u8 byLevel, byUSN, byWAN, byIFC;
	__u8 byBufferReadStatus[TUNER_READ_TOTAL_STATUS_QTD];
	__u8 address = TUNER_READ_STATUS_ADDR;
	int err = 0;

	err = i2c_smbus_read_i2c_block_data(radio->i2c_client, address,
			ARRAY_SIZE(byBufferReadStatus), byBufferReadStatus);

	if (err <= 0) {
		dev_err(radio->dev, "Error while reading tuner status: %d\n", err);
		return;
	}

	byLevel = byBufferReadStatus[1]; // RSSI
	byUSN = (byBufferReadStatus[2] & 0xF0) >> 4;
	byWAN = (byBufferReadStatus[2] & 0x0F);
	byIFC = (byBufferReadStatus[3] & 0x1F);

	if (radio->byAMFM == FM_MODE)
	{
		if ((byUSN > FM_USN_LEVEL) || (byWAN > FM_WAN_LEVEL_MIN)) {
			if (byLevel < (FM_LEVEL_MIN + DELTA_AGC_GAIN_1)) {
				Tuner_SetAGCParameters(radio, AGC_GAIN_4);  // change AGC to -0,5V
			} else if (byLevel < (FM_LEVEL_MIN + DELTA_AGC_GAIN_2)) {
				Tuner_SetAGCParameters(radio, AGC_GAIN_5);  // change AGC to -0,7V
			} else if (byLevel < (FM_LEVEL_MIN + DELTA_AGC_GAIN_3)) {
				Tuner_SetAGCParameters(radio, AGC_GAIN_6);  // change AGC to -0,9V
			} else if (byLevel < (FM_LEVEL_MIN + DELTA_AGC_GAIN_4)) {
				Tuner_SetAGCParameters(radio, AGC_GAIN_7);  // change AGC to +0,1V
			} else if (byLevel < (FM_LEVEL_MIN + DELTA_AGC_GAIN_5)) {
				Tuner_SetAGCParameters(radio, AGC_GAIN_8);  // change AGC to +0,3V
			} else {
				Tuner_SetAGCParameters(radio, AGC_GAIN_2);  // change AGC to -1,0V
			}
		}
	}
}

static void TUNER_Search(struct tef6606_device *radio, __u8 searchUp)
{
	__u32 startFrequency = radio->dwFrequency;
	__u8 retryCount = 0;
	__u8 doneFlag = 0;

	do {
		if(searchUp)
		{
			TUNER_SearchUp(radio);
		}
		else
		{
			TUNER_SearchDown(radio);
		}
		if (radio->byAMFM == FM_MODE)
		{
			Tuner_SetAGCParameters(radio, AGC_GAIN_1);
		}
		msleep(20);
		for (retryCount = 0; retryCount < TUNER_SEARCH_MAX_CNT; retryCount++)
		{
			dev_dbg(radio->dev, "Retries=%d for frequency=%d\n", retryCount, radio->dwFrequency);
			// check if we are done
			if ((startFrequency == radio->dwFrequency) || TUNER_CheckSearchRead(radio))
			{
				dev_dbg(radio->dev, "Done searching: %d\n", radio->dwFrequency);
				TUNER_SearchBand(radio);
				// we are done
				doneFlag = 1;
				break;
			}

			// only for FM
			if (radio->byAMFM == FM_MODE)
			{
				if (retryCount == 0x02)
				{
					// AGC=0V
					Tuner_SetAGCParameters(radio, AGC_GAIN_2);
				}
				else if (retryCount == 0x04)
				{
					// AGC=+1V
					Tuner_SetAGCParameters(radio, AGC_GAIN_3);
				}
			}
		}
	} while (doneFlag == 0);
	msleep(300);
	Tuner_CheckReadState(radio);
}

static int TUNER_BandSelect(struct tef6606_device *radio, __u8 byAMFM)
{
	radio->byAMFM = byAMFM;
	TUNER_SetParameters(radio);
	if(radio->byAMFM == FM_MODE)
	{
		radio->dwFrequency = TUNER_FM_MIN_FREQ_KHZ;
	}
	else
	{
		radio->dwFrequency = TUNER_AM_MIN_FREQ_KHZ;
	}
	return TUNER_SearchBand(radio);
}

/* V4L2 vidioc */
static int vidioc_querycap(struct file *file, void  *priv, struct v4l2_capability *v)
{
	struct tef6606_device *dev;

	dev = video_get_drvdata(video_devdata(file));

	strlcpy(v->driver, "radio-tef6606", sizeof(v->driver));
	strlcpy(v->card, "NXP TEF6606 Advanced Tuner", sizeof(v->card));
	v->version = KERNEL_VERSION(0,1,0);
	v->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO | V4L2_CAP_HW_FREQ_SEEK;

	return 0;
}

static __u16 tef6606_sigstr(struct i2c_client *client)
{
	int err = 0;
	__u8 buf[5];
	__u8 address = TUNER_READ_STATUS_ADDR;
	err = i2c_smbus_read_i2c_block_data(client, address, ARRAY_SIZE(buf), buf);
	if (err == sizeof(buf))
		return buf[1] << 8;
	return 0;
}

static int vidioc_g_tuner(struct file *file, void *priv, struct v4l2_tuner *v)
{
	struct tef6606_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	switch(v->index) {
		case FM_MODE:
			strlcpy(v->name, "FM", sizeof(v->name));
			v->type = V4L2_TUNER_RADIO;
			v->rangelow = TUNER_FM_MIN_LOW_FREQ;
			v->rangehigh = TUNER_FM_MAX_LOW_FREQ;
			v->capability = V4L2_TUNER_CAP_LOW;
			v->rxsubchans = V4L2_TUNER_MODE_STEREO;
			v->audmode = V4L2_TUNER_MODE_STEREO;
			break;
		case AM_MODE:
			strlcpy(v->name, "AM", sizeof(v->name));
			v->type = V4L2_TUNER_RADIO;
			v->rangelow = TUNER_AM_MIN_LOW_FREQ;
			v->rangehigh = TUNER_AM_MAX_LOW_FREQ;
			v->rxsubchans = V4L2_TUNER_SUB_MONO;
			v->capability = V4L2_TUNER_CAP_LOW;
			v->audmode = V4L2_TUNER_MODE_MONO;
			break;
		default:
			return -EINVAL;
	}
	v->signal = tef6606_sigstr(radio->i2c_client);
	return 0;
}

static int vidioc_s_tuner(struct file *file, void *priv, struct v4l2_tuner *v)
{
	struct tef6606_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	switch(v->index)
	{
		case FM_MODE:
		case AM_MODE:
			return TUNER_BandSelect(radio, (__u8) v->index);
		default:
			return -EINVAL;
	}
}

static int vidioc_s_frequency(struct file *file, void *priv, struct v4l2_frequency *f)
{

	// received frequency is in multiples of 62.5Hz
	// we need to convert it accordingly to tef6606 datasheet
	__u16 frequency = 0;
	__u8 buffer[2];
	__u8 address;
	int ret = 0;

	struct tef6606_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	switch(f->tuner)
	{
		case FM_MODE:
			// checking range
			if(f->frequency < TUNER_FM_MIN_LOW_FREQ || f->frequency > TUNER_FM_MAX_LOW_FREQ)
			{
				dev_err(radio->dev, "FM frequency %d is out of range (%d, %d)\n",
						f->frequency, TUNER_FM_MIN_LOW_FREQ, TUNER_FM_MAX_LOW_FREQ);
				return -EINVAL;
			}
			frequency = TUNER_ConvertV4L2FmFrequency(f->frequency);
			dev_dbg(radio->dev, "New FM Frequency: v4l2=%d; device=%d\n", f->frequency, frequency);

			buffer[0] = (__u8) (TUNER_SELECT_FM  | ((frequency & 0x1F00) >> 8));
			buffer[1] = (__u8) (frequency & 0x00FF);
			break;
		case AM_MODE:
			// checking range
			if(f->frequency < TUNER_AM_MIN_LOW_FREQ || f->frequency > TUNER_AM_MAX_LOW_FREQ)
			{
				dev_err(radio->dev, "AM frequency %d is out of range (%d, %d)\n",
						f->frequency, TUNER_AM_MIN_LOW_FREQ, TUNER_AM_MAX_LOW_FREQ);
				return -EINVAL;
			}
			frequency = TUNER_ConvertV4L2AmFrequency(f->frequency);
			dev_dbg(radio->dev, "New AM Frequency: v4l2=%d; device=%d\n", f->frequency, frequency);

			buffer[0] = (__u8) (TUNER_SELECT_AM  | ((frequency & 0x1F00) >> 8));
			buffer[1] = (__u8) (frequency & 0x00FF);
			break;
		default:
			return -EINVAL;
	}

	// checking if we are changing current band
	if(f->tuner != radio->byAMFM)
	{
		ret = TUNER_BandSelect(radio, f->tuner);
		if(ret)
		{
			dev_err(radio->dev, "Error while selecting new band: %d\n", ret);
			return ret;
		}
	}

	TUNER_SetParameters(radio);

	address = TUNER_ADDR_00 | TUNER_PRESET;
	ret = i2c_smbus_write_i2c_block_data(radio->i2c_client, address, ARRAY_SIZE(buffer), buffer);
	if(ret < 0)
	{
		dev_err(radio->dev, "Error while trying to set new frequency: %d\n", ret);
		return ret;
	}

	radio->dwFrequency = f->frequency;
	return 0;
}

static int vidioc_g_frequency(struct file *file, void *priv, struct v4l2_frequency *f)
{
	struct tef6606_device *radio;
	radio = video_get_drvdata(video_devdata(file));
	f->frequency = radio->dwFrequency;
	f->tuner = radio->byAMFM;
	f->type = V4L2_TUNER_RADIO;
	return 0;
}

static int vidioc_s_hw_freq_seek(struct file *file, void *fh, struct v4l2_hw_freq_seek *a)
{
	struct tef6606_device *radio;
	radio = video_get_drvdata(video_devdata(file));

	switch (a->tuner)
	{
		case FM_MODE:
		case AM_MODE:
			dev_dbg(radio->dev, "Init search for band: %d, direction=%d\n", a->tuner, a->seek_upward);
			TUNER_Search(radio, a->seek_upward);
			break;
		default:
			dev_err(radio->dev, "Invalid tuner for seek frequency!: %d\n", a->tuner);
			return -EINVAL;
	}
	dev_dbg(radio->dev, "Search completed for band: %d, frequency=%d\n", a->tuner, radio->dwFrequency);
	return 0;
}

/* File system interface */
static const struct v4l2_file_operations tef6606_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
};

static const struct v4l2_ioctl_ops tef6606_ioctl_ops = {
	.vidioc_querycap    	= vidioc_querycap,
	.vidioc_g_tuner     	= vidioc_g_tuner,
	.vidioc_s_tuner     	= vidioc_s_tuner,
	.vidioc_g_frequency 	= vidioc_g_frequency,
	.vidioc_s_frequency 	= vidioc_s_frequency,
	.vidioc_s_hw_freq_seek 	= vidioc_s_hw_freq_seek,
};

/* V4L2 interface */
static struct video_device tef6606_radio_template = {
	.name		= "TEF6606 AM-FM Radio",
	.fops       = &tef6606_fops,
	.ioctl_ops 	= &tef6606_ioctl_ops,
	.release	= video_device_release,
};

static int __devinit tef6606_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tef6606_device *radio;
	int ret;

	radio = kzalloc(sizeof(struct tef6606_device), GFP_KERNEL);
	if (!radio)
	{
		dev_err(&(client->dev), "Failed to alloc memory for TEF6606 device structure.\n");
		return -ENOMEM;
	}

	radio->dev = &(client->dev);
	radio->i2c_client = client;

	radio->videodev = video_device_alloc();
	if (!(radio->videodev)) {
		ret = -ENOMEM;
		dev_err(radio->dev, "Failed to alloc TEF6606 v4l2 device.\n");
		goto errfr;
	}

	memcpy(radio->videodev, &tef6606_radio_template, sizeof(tef6606_radio_template));

	i2c_set_clientdata(client, radio);
	video_set_drvdata(radio->videodev, radio);

	ret = video_register_device(radio->videodev, VFL_TYPE_RADIO, radio_nr);
	if (ret < 0) {
		dev_err(radio->dev, "Failed to register TEF6606 v4l2 device.\n");
		goto errrel;
	}

	dev_info(radio->dev, "TEF6606 AM-FM Tuner probed!.\n");

	return 0;

errrel:
	video_device_release(radio->videodev);
errfr:
	kfree(radio);
	return ret;
}

static int __devexit tef6606_i2c_remove(struct i2c_client *client)
{
	struct tef6606_device *radio = i2c_get_clientdata(client);
	struct device *dev = radio->dev;

	if (radio)
	{
		video_unregister_device(radio->videodev);
		kfree(radio);
	}

	dev_info(dev, "TEF6606 AM-FM Tuner removed!.\n");

	return 0;
}

/* I2C subsystem interface */
static const struct i2c_device_id tef6606_id[] = {
	{ RADIO_TEF6606_QUANTA_DRIVER_NAME, 0 },
	{ }							/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, tef6606_id);

static struct i2c_driver tef6606_i2c_driver = {
	.driver = {
		.name = RADIO_TEF6606_QUANTA_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = tef6606_i2c_probe,
	.remove = __devexit_p(tef6606_i2c_remove),
	.id_table = tef6606_id,
};

/* init the driver */
static int __init tef6606_init(void)
{
	int error = 0;
	// register in the i2c-core
	error = i2c_add_driver(&tef6606_i2c_driver);
	if(error)
	{
		pr_err("%s: cant add i2c driver\n", RADIO_TEF6606_QUANTA_DRIVER_NAME);
		goto err_i2c_add;
	}

	pr_info("%s: init ok\n", RADIO_TEF6606_QUANTA_DRIVER_NAME);
	return 0;

	err_i2c_add:
		return error;
}

/* cleanup the driver */
static void __exit tef6606_exit(void)
{
	i2c_del_driver(&tef6606_i2c_driver);

	pr_info("%s: exit ok\n", RADIO_TEF6606_QUANTA_DRIVER_NAME);
}

MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("NXP TEF6606 AM-FM Advcanced Tuner device driver");
MODULE_LICENSE("GPL");

module_init(tef6606_init);
module_exit(tef6606_exit);
