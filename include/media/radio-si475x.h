/*
 *	This is the platform data for Si475x device driver for Silicon Labs SI475X AM/FM Tuner device.
 *	It is an I2C device, so, I2C is needed.
 *
 *  Created on: 09/01/2015
 *      Author: mauricio.cirelli
 *
 */

#ifndef RADIO_SI475X_H
#define RADIO_SI475X_H

/**
 * This is the default xtalLoad parameter value used in Si475x evaluation boards.
 */
#define SI475X_EVALBOARD_XTAL_LOAD					0x27

enum radio_si475x_xtal_freqs
{
	Si475x_4000kHz = 0,
	Si475x_37209kHz,
	Si475x_36400kHz,
	Si475x_37800kHz,
};

/*
 * Platform dependent definition
 */
struct radio_si475x_platform_data {
	u8 								xtalResistor;	// Indicates if the crystal has a resistor connected to it (1) or not (0)
	enum radio_si475x_xtal_freqs 	xtalFreq;		// 4000, 37209, 36400 or 37800 kHz if present
	u8 								xtalLoad;		// xcLoad parameter
	void (*powerdown)(void);						// Platform function to power down the device
	void (*powerup)(void);							// Platform function to power up the device
};

#endif /* ifndef RADIO_SI475X_H*/
