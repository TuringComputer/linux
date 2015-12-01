#ifndef __ROTARY_ENCODER_KEYS_H__
#define __ROTARY_ENCODER_KEYS_H__

struct rotary_encoder_keys_platform_data {
	unsigned int key_cw;
	unsigned int key_ccw;
	unsigned int gpio_a;
	unsigned int gpio_b;
	unsigned int inverted_a;
	unsigned int inverted_b;
};

#endif /* __ROTARY_ENCODER_KEYS_H__ */
