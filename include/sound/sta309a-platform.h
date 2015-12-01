/*
 * sta309a.h
 *
 *  Created on: Jul 9, 2013
 *      Author: dsueiro
 */

#ifndef STA309A_H_
#define STA309A_H_

struct sta309a_platform_data {
	int master_volume;
	bool external_amp;
	int clock_config;
	int i2s_config;
	//Input channels mapping
	int imc12;
	int imc34;
	int imc56;
	int imc78;
	//Output channels mapping
	int omc12;
	int omc34;
	int omc56;
	int omc78;
};

/**
 * Adds all controls to the codec
 */
int sta309a_audiodsp_add_controls(struct snd_soc_codec *codec);

#endif /* STA309A_H_ */
