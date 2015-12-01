/*
 * stg3856-audiomux.c
 *
 *  Created on: Jul 6, 2013
 *      Author: dsueiro
 */
#define DEBUG 1
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <linux/slab.h>

#include <sound/stg3856-audiomux.h>

#define DRIVER_NAME "stg3856-audiomux"

struct stg3856_audiomux_priv {
	int curr_src;
	int sel0;
	int sel1;
	struct platform_device *pdev;
};

static struct stg3856_audiomux_priv *stg3856;
static const char *input_names[STG3856_INPUTS_NUM] = {"Input 1","Input 2","Input 3"};


static int stg3856_audiomux_info(struct snd_kcontrol *kcontrol,
               struct snd_ctl_elem_info *uinfo) {

       dev_dbg(&stg3856->pdev->dev, "%s\n",__func__);

       uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
       uinfo->count = 1;
       uinfo->value.enumerated.items = STG3856_INPUTS_NUM;
       if (uinfo->value.enumerated.item > STG3856_INPUTS_NUM-1) {
               uinfo->value.enumerated.item = STG3856_INPUTS_NUM-1 ;
       }
       if (strlen(input_names[uinfo->value.enumerated.item]) != 0)
    	   strcpy(uinfo->value.enumerated.name, input_names[uinfo->value.enumerated.item]);
       return 0;
}

/* Select the active input for the audio mux */
static int stg3856_audiomux_set_src(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) {

	int value;

	dev_dbg(&stg3856->pdev->dev, "%s\n",__func__);
	if (stg3856->curr_src == ucontrol->value.enumerated.item[0]) {
		/*Value not changed*/
		return 0;
	}

	if(ucontrol->value.enumerated.item[0] < 0 || ucontrol->value.enumerated.item[0] > 3)
		return -EINVAL;

	value = ucontrol->value.enumerated.item[0] & 0x1;
	gpio_set_value(stg3856->sel0, value);
	value = ucontrol->value.enumerated.item[0] & 0x2;
	gpio_set_value(stg3856->sel1, value);

	stg3856->curr_src = ucontrol->value.enumerated.item[0];
	return 1;
}

static int stg3856_audiomux_get_src(struct snd_kcontrol *kcontrol,
               struct snd_ctl_elem_value *ucontrol) {

	int value = 0;

	dev_dbg(&stg3856->pdev->dev, "%s\n",__func__);

	value = stg3856->curr_src;

	if (value < 0) {
		value = 0;
	}

	ucontrol->value.integer.value[0] = value;

	return 0;
}


static const struct snd_kcontrol_new stg3856_audiomux_snd_controls[] = {
	/* Audio MUX */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Audio Mux",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = stg3856_audiomux_info,
		.put = stg3856_audiomux_set_src,
		.get = stg3856_audiomux_get_src,
	},
};

int stg3856_audiomux_add_controls(struct snd_soc_codec *codec) {

	dev_dbg(&stg3856->pdev->dev, "Adding STG3856 controls to codec.\n");
	return snd_soc_add_controls(codec, stg3856_audiomux_snd_controls, ARRAY_SIZE(stg3856_audiomux_snd_controls));
}
EXPORT_SYMBOL(stg3856_audiomux_add_controls);


static int __devinit stg3856_audiomux_probe(struct platform_device *pdev)
{
	int ret=0;
	struct stg3856_audiomux_platform_data *pdata = pdev->dev.platform_data;

	stg3856 = kzalloc(sizeof(struct stg3856_audiomux_priv), GFP_KERNEL);
	if (!stg3856)
	        goto ret_err_alloc_audiomux;

	stg3856->pdev = pdev;

	/* set GPIOs */
	stg3856->sel0 = pdata->sel0;
	stg3856->sel1 = pdata->sel1;

	/* Check wether GPIO0 and GPIO1 are valid GPIO addresses */
	if (!gpio_is_valid(stg3856->sel0) || !gpio_is_valid(stg3856->sel1)) {
		dev_err(&pdev->dev, "Invalid gpio number.\n");
		ret = -EINVAL;
		goto ret_err_gpio_is_valid;
	}

	/* Request GPIO0 */
	if (gpio_request(stg3856->sel0, "STG3856-AUDIOMUX_SEL0")) {
		dev_err(&pdev->dev, "Can't claim sel0!\n");
		ret = -EFAULT;
		goto ret_err_gpio_request0;
	}

	/* Request GPIO1 */
	if (gpio_request(stg3856->sel1, "STG3856-AUDIOMUX_SEL1")) {
		dev_err(&pdev->dev, "Can't claim sel1!\n");
		ret = -EFAULT;
		goto ret_err_gpio_request1;
	}

	/* define both GPIO0 and GPIO1 as output signals and initialize for AV*/
	gpio_direction_output(stg3856->sel0, 1);
	gpio_direction_output(stg3856->sel1, 1);

	dev_info(&pdev->dev, "STG3856 device driver loaded.\n");

	goto ret_ok;

	ret_err_gpio_request1:
		gpio_free(stg3856->sel0);
	ret_err_gpio_request0:
	ret_err_gpio_is_valid:
	ret_err_alloc_audiomux:
	ret_ok:
	return ret;

}


static int stg3856_audiomux_remove(struct platform_device *pdev)
{
	gpio_free(stg3856->sel0);
	gpio_free(stg3856->sel1);

	return 0;
}

static struct platform_driver stg3856_audiomux_driver = {
	.probe = stg3856_audiomux_probe,
	.remove = __devexit_p(stg3856_audiomux_remove),
	.driver = {
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
	},
};

static int __init stg3856_audiomux_init(void)
{
	return platform_driver_register(&stg3856_audiomux_driver);
}

static void __exit stg3856_audiomux_exit(void)
{
	platform_driver_unregister(&stg3856_audiomux_driver);
}

module_init(stg3856_audiomux_init);
module_exit(stg3856_audiomux_exit);

MODULE_DESCRIPTION("Quanta stg3856 audio mux driver");
MODULE_AUTHOR("Diego Sueiro <diego.sueiro@e-labworks.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
