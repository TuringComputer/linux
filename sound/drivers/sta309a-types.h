/*
 * sta309a-types.h
 *
 *  Created on: Jul 6, 2013
 *      Author: dsueiro
 */

#ifndef STA309A_TYPES_H_
#define STA309A_TYPES_H_

typedef enum sta309a_input_mapping_channel_e {
	STA309A_IMC12 = 0,
	STA309A_IMC34,
	STA309A_IMC56,
	STA309A_IMC78,
} sta309a_input_mapping_channel;

typedef enum sta309a_output_mapping_channel_e {
	STA309A_OMC12 = 0,
	STA309A_OMC34,
	STA309A_OMC56,
	STA309A_OMC78,
} sta309a_output_mapping_channel;

/*
 * Registers Address.
 */

/* Configuration Registers */
#define STA309A_CONFA	0x00
#define STA309A_CONFB	0x01
#define STA309A_CONFC	0x02
#define STA309A_CONFD	0x03
#define STA309A_CONFE	0x04
#define STA309A_CONFF	0x05
#define STA309A_CONFG	0x06
#define STA309A_CONFH	0x07
#define STA309A_CONFI	0x08

/* Volume Control Registers */
#define STA309A_MMUTE	0x09
#define STA309A_MVOL	0x0A
#define STA309A_C1VOL	0x0B
#define STA309A_C2VOL	0x0C
#define STA309A_C3VOL	0x0D
#define STA309A_C4VOL	0x0E
#define STA309A_C5VOL	0x0F
#define STA309A_C6VOL	0x10
#define STA309A_C7VOL	0x11
#define STA309A_C8VOL	0x12
#define STA309A_C1VTMB 	0x13
#define STA309A_C2VTMB 	0x14
#define STA309A_C3VTMB 	0x15
#define STA309A_C4VTMB 	0x16
#define STA309A_C5VTMB 	0x17
#define STA309A_C6VTMB 	0x18
#define STA309A_C7VTMB 	0x19
#define STA309A_C8VTMB 	0x1A

/* Input Mapping Registers */
#define STA309A_C12IM	0x1B
#define STA309A_C34IM	0x1C
#define STA309A_C56IM	0x1D
#define STA309A_C78IM	0x1E

/* Automode Registers */
#define STA309A_AUTO1	0x1F
#define STA309A_AUTO2	0x20
#define STA309A_AUTO3	0x21
#define STA309A_PREEQ	0x22
#define STA309A_AGEQ	0x23
#define STA309A_BGEQ	0x24
#define STA309A_CGEQ	0x25
#define STA309A_DGEQ	0x26
#define STA309A_EGEQ	0x27

/* Processing loop Registers */
#define STA309A_BQLP	0x28
#define STA309A_MXLP	0x29

/* Processing bypass Registers */
#define STA309A_EQBP	0x2A
#define STA309A_TONEBP	0x2B

/* Tone control Register */
#define STA309A_TONE	0x2C

/* Dynamics control Registers */
#define STA309A_C1234LS	0x2D
#define STA309A_C5678LS	0x2E
#define STA309A_L1AR	0x2F
#define STA309A_L1ART	0x30
#define STA309A_L2AR	0x31
#define STA309A_L2ART	0x32

/* PWM output timing Registers */
#define STA309A_C12OT	0x33
#define STA309A_C34OT	0x34
#define STA309A_C56OT	0x35
#define STA309A_C78OT	0x36

/* I2S output channel mapping Registers */
#define STA309A_C12OM	0x37
#define STA309A_C34OM	0x38
#define STA309A_C56OM	0x39
#define STA309A_C78OM	0x3A

/* User-defined coefficient RAM Registers */
#define STA309A_CFADDR1	0x3B
#define STA309A_CFADDR2	0x3C
#define STA309A_B1CF1 	0x3D
#define STA309A_B1CF2 	0x3E
#define STA309A_B1CF3 	0x3F
#define STA309A_B2CF1 	0x40
#define STA309A_B2CF2 	0x41
#define STA309A_B2CF3 	0x42
#define STA309A_A1CF1 	0x43
#define STA309A_A1CF2 	0x44
#define STA309A_A1CF3 	0x45
#define STA309A_A2CF1 	0x46
#define STA309A_A2CF2 	0x47
#define STA309A_A2CF3 	0x48
#define STA309A_B0CF1 	0x49
#define STA309A_B0CF2 	0x4A
#define STA309A_B0CF3 	0x4B
#define STA309A_CFUD 	0x4C
#define STA309A_MPCC1	0x4D
#define STA309A_MPCC2	0x4E
#define STA309A_DCC1	0x4F
#define STA309A_DCC2	0x50
#define STA309A_PSC1	0x51
#define STA309A_PSC2	0x52
#define STA309A_PSC3	0x53


/* Field Definitions */

/*
 * STA309A_CONFA
 */
#define STA309A_CONFA_MCS_MASK	0x07 // Master Clock Select
#define STA309A_CONFA_MCS0	 	0x01
#define STA309A_CONFA_MCS1	 	0x02
#define STA309A_CONFA_MCS2	 	0x04
#define STA309A_CONFA_IR_MASK	0x18 // Interpolation Ratio
#define STA309A_CONFA_IR0		0x08
#define STA309A_CONFA_IR1		0x10
#define STA309A_CONFA_DSPB		0x20 // DSP Bypass
#define STA309A_CONFA_COS_MASK	0xC0 // Clock Output Select
#define STA309A_CONFA_COS1		0x40
#define STA309A_CONFA_COS2		0x80

/*
 * STA309A_CONFB
 */
#define STA309A_CONFB_SAI_MASK	0x0F // Serial Audio Input Interface Format
#define STA309A_CONFB_SAI0		0x01
#define STA309A_CONFB_SAI1		0x02
#define STA309A_CONFB_SAI2		0x04
#define STA309A_CONFB_SAI3		0x08
#define STA309A_CONFB_SAIFB		0x10

/*
 * STA309A_CONFC
 */
#define STA309A_CONFC_SAO_MASK	0x0F // Serial Audio Output Interface Format
#define STA309A_CONFC_SAO0		0x01
#define STA309A_CONFC_SAO1		0x02
#define STA309A_CONFC_SAO2		0x04
#define STA309A_CONFC_SAO3		0x08
#define STA309A_CONFC_SAOFB		0x10

/*
 * STA309A_CONFD
 */
#define STA309A_CONFD_OM_MASK	0x03 // DDX power output mode
#define STA309A_CONFD_OM0		0x01
#define STA309A_CONFD_OM1		0x02
#define STA309A_CONFD_CSZ_MASK	0xFC // Contra size register
#define STA309A_CONFD_CSZ0		0x04
#define STA309A_CONFD_CSZ1		0x08
#define STA309A_CONFD_CSZ2		0x10
#define STA309A_CONFD_CSZ3		0x20
#define STA309A_CONFD_CSZ4		0x40
#define STA309A_CONFD_MPC		0x80 // Max Power Correction

/*
 * STA309A_CONFE - Binary Output Mode
 */
#define STA309A_CONFE_CIB0		0x01
#define STA309A_CONFE_CIB1		0x02
#define STA309A_CONFE_CIB2		0x04
#define STA309A_CONFE_CIB3		0x08
#define STA309A_CONFE_CIB4		0x10
#define STA309A_CONFE_CIB5		0x20
#define STA309A_CONFE_CIB6		0x40
#define STA309A_CONFE_CIB7		0x80

/*
 * STA309A_CONFF
 */
#define STA309A_CONFF_HPB		0x01 // High-pass filter bypass
#define STA309A_CONFF_DRC		0x02 // Dynamic range compression/anti-clipping
#define STA309A_CONFF_DEMP		0x04 // De-emphasis
#define STA309A_CONFF_PSL		0x08 // Postscale link
#define STA309A_CONFF_BQL		0x10 // Biquad link
#define STA309A_CONFF_PWMS_MASK	0xE0 // PWM Output Speed
#define STA309A_CONFF_PWMS0		0x20
#define STA309A_CONFF_PWMS1		0x40
#define STA309A_CONFF_PWMS2		0x80

/*
 * STA309A_CONFG
 */
#define STA309A_CONFG_PWMD		0x01 // PWM Outupt Disable
#define STA309A_CONFG_SID		0x02 // I2S Out Disable
#define STA309A_CONFG_COD		0x04 // Clock Out Disable
#define STA309A_CONFG_AME		0x08 // AM Mode Enable
#define STA309A_CONFG_AM2E		0x10 // AM2 Mode Enable
#define STA309A_CONFG_HPE		0x20 // DDX Headphone Enable
#define STA309A_CONFG_DCCV		0x40 // Distortion compensation variable enable
#define STA309A_CONFG_MPCV		0x80 // Max power correction variable

/*
 * STA309A_CONFH
 */
#define STA309A_CONFH_NSBW		0x01 // Noise-shaper bandwidth selection
#define STA309A_CONFH_ZCE		0x02 // Zero-crossing volume enable
#define STA309A_CONFH_SVE		0x04 // Soft volume enable
#define STA309A_CONFH_ZDE		0x08 // Zero-detect mute enable
#define STA309A_CONFH_IDE		0x10 // Invalid input detect mute enable
#define STA309A_CONFH_BCLE		0x20 // Binary output mode clock loss detection enable
#define STA309A_CONFH_LDTE		0x40 // LRCLK double trigger protection enable
#define STA309A_CONFH_ECLE		0x80 // Auto EAPD on clock loss

/*
 * STA309A_CONFI
 */
#define STA309A_CONFI_PSCE			0x01 // Power supply ripple correction enable
#define STA309A_CONFI_EAPD			0x80 //External amplifier power down
#define STA309A_CONFI_EAPD_SHIFT	0x07



/*
 * STA309A_AUTO1
 */
#define STA309A_AUTO1_AMEQ_MASK	0x03 // Automode EQ
#define STA309A_AUTO1_AMEQ0		0x01
#define STA309A_AUTO1_AMEQ1		0x02
#define STA309A_AUTO1_AMV_MASK	0x0C // Automode Volume mode
#define STA309A_AUTO1_AMV0		0x04
#define STA309A_AUTO1_AMV1		0x08
#define STA309A_AUTO1_AMGC_MASK	0x30 // Automode gain compression/limiters mode
#define STA309A_AUTO1_AMGC0		0x10
#define STA309A_AUTO1_AMGC1		0x20
#define STA309A_AUTO1_AMGC2		0x40
#define STA309A_AUTO1_AMDM		0x80 // Automode 5.1 downmix

/*
 * STA309A_AUTO2
 */
#define STA309A_AUTO2_AMBMME	0x01 // Automode bass management mix
#define STA309A_AUTO2_AMBMXE	0x02 // Automode bass management crossover
#define STA309A_AUTO2_FSS		0x04 // Front Speaker Size
#define STA309A_AUTO2_CSS_MASK	0x18 // Center Speaker Size
#define STA309A_AUTO2_CSS0		0x08
#define STA309A_AUTO2_CSS1		0x10
#define STA309A_AUTO2_RSS_MASK	0x60 // Rear Speaker Size
#define STA309A_AUTO2_RSS0		0x20
#define STA309A_AUTO2_RSS1		0x40
#define STA309A_AUTO2_SUB		0x80 // Subwoofer

/*
 * STA309A_AUTO3
 */
#define STA309A_AUTO3_AMPS		0x01 // Automode prescale
#define STA309A_AUTO3_MSA		0x02 // Bass management mix scale adjustment
#define STA309A_AUTO3_AMAME		0x04 // Automode AM enable
#define STA309A_AUTO3_AMAM_MASK	0xE0 // Automode AM mode
#define STA309A_AUTO3_AMAM0		0x20
#define STA309A_AUTO3_AMAM1		0x40
#define STA309A_AUTO3_AMAM2		0x80

/*
 * STA309A_PREEQ
 */
#define STA309A_PREEQ_PEQ_MASK			0x1F // Preset Equalization
#define STA309A_PREEQ_PEQ_FLAT			0x00
#define STA309A_PREEQ_PEQ_ROCK			0x01
#define STA309A_PREEQ_PEQ_SOFTROCK		0x02
#define STA309A_PREEQ_PEQ_JAZZ			0x03
#define STA309A_PREEQ_PEQ_CLASSICAL		0x04
#define STA309A_PREEQ_PEQ_DANCE			0x05
#define STA309A_PREEQ_PEQ_POP			0x06
#define STA309A_PREEQ_PEQ_SOFT			0x07
#define STA309A_PREEQ_PEQ_HARD			0x08
#define STA309A_PREEQ_PEQ_PARTY			0x09
#define STA309A_PREEQ_PEQ_VOCAL			0x0A
#define STA309A_PREEQ_PEQ_HIPHOP		0x0B
#define STA309A_PREEQ_PEQ_DIALOG		0x0C
#define STA309A_PREEQ_PEQ_BASSBOOST1	0x0D
#define STA309A_PREEQ_PEQ_BASSBOOST2	0x0E
#define STA309A_PREEQ_PEQ_BASSBOOST3	0x0F
#define STA309A_PREEQ_PEQ_LOUDNESS1		0x10
#define STA309A_PREEQ_PEQ_LOUDNESS2		0x11
#define STA309A_PREEQ_PEQ_LOUDNESS3		0x12
#define STA309A_PREEQ_PEQ_LOUDNESS4		0x13
#define STA309A_PREEQ_PEQ_LOUDNESS5		0x14
#define STA309A_PREEQ_PEQ_LOUDNESS6		0x15
#define STA309A_PREEQ_PEQ_LOUDNESS7		0x16
#define STA309A_PREEQ_PEQ_LOUDNESS8		0x17
#define STA309A_PREEQ_PEQ_LOUDNESS9		0x18
#define STA309A_PREEQ_PEQ_LOUDNESS10	0x19
#define STA309A_PREEQ_PEQ_LOUDNESS11	0x1A
#define STA309A_PREEQ_PEQ_LOUDNESS12	0x1B
#define STA309A_PREEQ_PEQ_LOUDNESS13	0x1C
#define STA309A_PREEQ_PEQ_LOUDNESS14	0x1D
#define STA309A_PREEQ_PEQ_LOUDNESS15	0x1E
#define STA309A_PREEQ_PEQ_LOUDNESS16	0x1F
#define STA309A_PREEQ_XO_MASK			0xE0	//Bass management crossover frequency
#define STA309A_PREEQ_XO_70HZ			0x00
#define STA309A_PREEQ_XO_80HZ			0x01
#define STA309A_PREEQ_XO_90HZ			0x02
#define STA309A_PREEQ_XO_100HZ			0x03
#define STA309A_PREEQ_XO_110HZ			0x04
#define STA309A_PREEQ_XO_120HZ			0x05
#define STA309A_PREEQ_XO_140HZ			0x06
#define STA309A_PREEQ_XO_160HZ			0x07

/*
 * STA309A_xGEQ - graphic EQ x band
 * 	x = from A to E
 * 	xGEQ[4:0]	Boost/cut
 * 	11111		+16
 * 	11110		+15
 * 	11101		+14
 * 	...			...
 * 	10000		+1
 * 	01111		 0
 *	01110		-1
 * 	...			...
 * 	00001		-14
 * 	00000		-15
 *
 */
#define STA309A_xGEQ_MASK	0x1F

/*
 * STA309A_BQLP - graphic EQ x band
 */


#define STA309A_BQLP	0x28

#define STA309A_MXLP	0x29





#endif /* STA309A_TYPES_H_ */
