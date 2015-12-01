/*
 * stg3856-audiomux.h
 *
 *  Created on: Jul 7, 2013
 *      Author: dsueiro
 */

#ifndef STG3856_AUDIOMUX_H_
#define STG3856_AUDIOMUX_H_

#define STG3856_INPUTS_NUM	3

struct stg3856_audiomux_platform_data {
        int sel0;
        int sel1;
};

int stg3856_audiomux_add_controls(struct snd_soc_codec *codec);

#endif /* STG3856_AUDIOMUX_H_ */
