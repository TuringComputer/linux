/*
 * sta309a-alsa.h
 *
 *	STA309A ALSA Control definitions
 *
 *  Created on: 30/07/2013
 *      Author: mauricio.cirelli
 */

#ifndef STA309A_ALSA_H_
#define STA309A_ALSA_H_

// Volume Controls
#define STA309ACTL_VOLUME_MAX						((int)+128)
#define STA309ACTL_VOLUME_MIN						((int)-127)
#define STA309ACTL_MASTER_VOLUME					"Master Volume" 		// int 0x00 to 0xFF
#define STA309ACTL_MASTER_MUTE						"Master Mute"			// 'off' or 'on'
#define STA309ACTL_CHANNEL1_MUTE					"Channel 1 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL2_MUTE					"Channel 2 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL3_MUTE					"Channel 3 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL4_MUTE					"Channel 4 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL5_MUTE					"Channel 5 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL6_MUTE					"Channel 6 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL7_MUTE					"Channel 7 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL8_MUTE					"Channel 8 Mute"		// 'off' or 'on'
#define STA309ACTL_CHANNEL1_VOLUME					"Channel 1 Offset" 		// int 0x00 to 0xFF (hard mute)
#define STA309ACTL_CHANNEL2_VOLUME					"Channel 2 Offset" 		// int 0x00 to 0xFF (hard mute)
#define STA309ACTL_CHANNEL3_VOLUME					"Channel 3 Offset" 		// int 0x00 to 0xFF (hard mute)
#define STA309ACTL_CHANNEL4_VOLUME					"Channel 4 Offset" 		// int 0x00 to 0xFF (hard mute)
#define STA309ACTL_CHANNEL5_VOLUME					"Channel 5 Offset" 		// int 0x00 to 0xFF (hard mute)
#define STA309ACTL_CHANNEL6_VOLUME					"Channel 6 Offset" 		// int 0x00 to 0xFF (hard mute)
#define STA309ACTL_CHANNEL7_VOLUME					"Channel 7 Offset" 		// int 0x00 to 0xFF (hard mute)
#define STA309ACTL_CHANNEL8_VOLUME					"Channel 8 Offset" 		// int 0x00 to 0xFF (hard mute)

// Equalizer Controls
#define STA309ACTL_PRESET_EQ						"Preset EQ"				// enum presets
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ			"Preset EQ Bass CrossOver Frequency"
#define STA309ACTL_EQCHANNEL1_BYPASS				"EQ Channel 1 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQCHANNEL2_BYPASS				"EQ Channel 2 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQCHANNEL3_BYPASS				"EQ Channel 3 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQCHANNEL4_BYPASS				"EQ Channel 4 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQCHANNEL5_BYPASS				"EQ Channel 5 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQCHANNEL6_BYPASS				"EQ Channel 6 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQCHANNEL7_BYPASS				"EQ Channel 7 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQCHANNEL8_BYPASS				"EQ Channel 8 Bypass"	// 'off' or 'on'
#define STA309ACTL_EQFREQ_MAX						((int)+16)
#define STA309ACTL_EQFREQ_MIN						((int)-15)
#define STA309ACTL_EQ80HZ							"EQ 80Hz"				// int -15 to +16
#define STA309ACTL_EQ300HZ							"EQ 300Hz"				// int -15 to +16
#define STA309ACTL_EQ1000HZ							"EQ 1000Hz"				// int -15 to +16
#define STA309ACTL_EQ3000HZ							"EQ 3000Hz"				// int -15 to +16
#define STA309ACTL_EQ8000HZ							"EQ 8000Hz"				// int -15 to +16

// Tone Controls
#define STA309ACTL_TONE_MAX							((int)13)
#define STA309ACTL_TONE_MIN							((int)1)
#define STA309ACTL_TONE_BTC							"Tone BTC"				// int -12 to +12
#define STA309ACTL_TONE_TTC							"Tone TTC"				// int -12 to +12
#define STA309ACTL_TONECHANNEL1_BYPASS				"Tone Channel 1 Bypass"	// 'off' or 'on'
#define STA309ACTL_TONECHANNEL2_BYPASS				"Tone Channel 2 Bypass"	// 'off' or 'on'
#define STA309ACTL_TONECHANNEL3_BYPASS				"Tone Channel 3 Bypass"	// 'off' or 'on'
#define STA309ACTL_TONECHANNEL4_BYPASS				"Tone Channel 4 Bypass"	// 'off' or 'on'
#define STA309ACTL_TONECHANNEL5_BYPASS				"Tone Channel 5 Bypass"	// 'off' or 'on'
#define STA309ACTL_TONECHANNEL6_BYPASS				"Tone Channel 6 Bypass"	// 'off' or 'on'
#define STA309ACTL_TONECHANNEL7_BYPASS				"Tone Channel 7 Bypass"	// 'off' or 'on'
#define STA309ACTL_TONECHANNEL8_BYPASS				"Tone Channel 8 Bypass"	// 'off' or 'on'

// MUTE and POWER MANAGEMENT values
#define STA309ACTL_ONOFFCTL_ON						"On"
#define STA309ACTL_ONOFFCTL_OFF						"Off"
#define STA309ACTL_ONOFFCTL_TOTAL					2

// STA309ACTL_PREQ_BASS_CROSSOVER_FREQ values
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_70HZ	"70Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_80HZ	"80Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_90HZ	"90Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_100HZ	"100Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_110HZ	"110Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_120HZ	"120Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_140HZ	"140Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_160HZ	"160Hz"
#define STA309ACTL_PREQ_BASS_CROSSOVER_FREQ_TOTAL	8

// STA309ACTL_PRESET_EQ values
#define STA309ACTL_PREQ_FLAT						"Flat"
#define STA309ACTL_PREQ_ROCK						"Rock"
#define STA309ACTL_PREQ_SOFTROCK					"Soft Rock"
#define STA309ACTL_PREQ_JAZZ						"Jazz"
#define STA309ACTL_PREQ_CLASSICAL					"Classical"
#define STA309ACTL_PREQ_DANCE						"Dance"
#define STA309ACTL_PREQ_POP							"Pop"
#define STA309ACTL_PREQ_SOFT						"Soft"
#define STA309ACTL_PREQ_HARD						"Hard"
#define STA309ACTL_PREQ_PARTY						"Party"
#define STA309ACTL_PREQ_VOCAL						"Vocal"
#define STA309ACTL_PREQ_HIPHOP						"Hip-Hop"
#define STA309ACTL_PREQ_DIALOG						"Dialog"
#define STA309ACTL_PREQ_BASSBOOST1					"Bass-boost #1"
#define STA309ACTL_PREQ_BASSBOOST2					"Bass-boost #2"
#define STA309ACTL_PREQ_BASSBOOST3					"Bass-boost #3"
#define STA309ACTL_PREQ_LOUDNESS1					"Loudness 1"
#define STA309ACTL_PREQ_LOUDNESS2					"Loudness 2"
#define STA309ACTL_PREQ_LOUDNESS3					"Loudness 3"
#define STA309ACTL_PREQ_LOUDNESS4					"Loudness 4"
#define STA309ACTL_PREQ_LOUDNESS5					"Loudness 5"
#define STA309ACTL_PREQ_LOUDNESS6					"Loudness 6"
#define STA309ACTL_PREQ_LOUDNESS7					"Loudness 7"
#define STA309ACTL_PREQ_LOUDNESS8					"Loudness 8"
#define STA309ACTL_PREQ_LOUDNESS9					"Loudness 9"
#define STA309ACTL_PREQ_LOUDNESS10					"Loudness 10"
#define STA309ACTL_PREQ_LOUDNESS11					"Loudness 11"
#define STA309ACTL_PREQ_LOUDNESS12					"Loudness 12"
#define STA309ACTL_PREQ_LOUDNESS13					"Loudness 13"
#define STA309ACTL_PREQ_LOUDNESS14					"Loudness 14"
#define STA309ACTL_PREQ_LOUDNESS15					"Loudness 15"
#define STA309ACTL_PREQ_LOUDNESS16					"Loudness 16"
#define STA309ACTL_PREQ_TOTAL						32

#endif /* STA309A_ALSA_H_ */
