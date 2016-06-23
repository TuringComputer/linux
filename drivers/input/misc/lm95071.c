/*
 * .c
 *
 *	Digital 14-bits temperature input/hwmon driver for LM95071 from Texas Instruments.
 *
 *  Created on: 22/06/2016
 *      Author: mauricio.cirelli
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input.h>
#include <linux/input-polldev.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#define POLL_INTERVAL_MIN			20
#define POLL_INTERVAL_MAX			200
#define POLL_INTERVAL				100

struct lm95071_chip
{
	struct spi_device				*spi;
	struct input_polled_dev			*input;
};

static int lm95071_spi_read(struct lm95071_chip *this, u16 *dst)
{
	int error = 0;

	if((this == NULL) || IS_ERR(this))
	{
		pr_err("%s: device is null!\n", __FUNCTION__);
		return -ENODEV;
	}

	error = spi_read(this->spi, dst, sizeof(*dst));

	if(error)
	{
		dev_err(&(this->spi->dev), "spi read error (%d)\n", error);
		return error;
	}

	return 0;
}

static int lm95071_read_temperature(struct lm95071_chip *this, int *dst)
{
	s16 temp;
	u16 raw;
	int error = 0;

	error = lm95071_spi_read(this, &raw);

	if(error)
	{
		return error;
	}

	dev_dbg(&(this->spi->dev), "spi read: 0x%04X\n", raw);

	/*
	 * LM90571 Temp register has two unused bits at the end.  After these are
	 * removed, every number remaining corresponds to 0.03125 C of temperature.
	 *
	 * Official docs here: http://www.ti.com/lit/ds/symlink/lm95071.pdf
	 */
	temp = (s16)(raw / 4);
	temp *= 31;

	dev_dbg(&(this->spi->dev), "temp read: %d mC\n", temp);

	*dst = temp;

	return 0;
}

static int lm95071_shutdown(struct lm95071_chip *this)
{
	u16 temp = 0xFF;
	return spi_write(this->spi, &temp, sizeof(temp));
}

static int lm95071_continuous_operation(struct lm95071_chip *this)
{
	u16 temp = 0x00;
	return spi_write(this->spi, &temp, sizeof(temp));
}

static void lm95071_dev_poll(struct input_polled_dev *dev)
{
	int temperature;
	struct lm95071_chip *this = dev->private;

	if(lm95071_read_temperature(this, &temperature))
		return;

	input_report_abs(this->input->input, ABS_MISC, temperature);
	input_sync(this->input->input);
}

#ifdef CONFIG_PM_SLEEP
static int lm95071_spi_suspend(struct device *dev)
{
	return lm95071_shutdown(spi_get_drvdata(to_spi_device(dev)));
}

static int lm95071_spi_resume(struct device *dev)
{
	return lm95071_continuous_operation(spi_get_drvdata(to_spi_device(dev)));
}
#endif

static SIMPLE_DEV_PM_OPS(lm95071_spi_pm, lm95071_spi_suspend, lm95071_spi_resume);

static int lm95071_spi_probe(struct spi_device *spi)
{
	int error = 0;

	/**
	 * Allocating memory for this device
	 */
	struct lm95071_chip *this = (struct lm95071_chip *) kcalloc(sizeof(struct lm95071_chip), 1, GFP_KERNEL);
	if (IS_ERR(this))
	{
		dev_err(&(spi->dev), "error: out of memory for device\n");
		error = -ENOMEM;
		goto err_no_mem_dev;
	}

	this->spi = spi;

	/**
	 * Configuring SPI Bus for this device
	 */
	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_0;
	error = spi_setup(spi);
	if (error)
	{
		dev_err(&(spi->dev), "error while configuring spi bus (%d)\n", error);
		goto err_spi_setup;
	}

	/**
	 * Allocate Input Polled Device
	 */
	this->input = input_allocate_polled_device();
	if (IS_ERR(this->input))
	{
		dev_err(&(spi->dev), "error: out of memory for input device\n");
		error = -ENOMEM;
		goto err_alloc_input;
	}

	/**
	 * Configure the input device
	 */
	this->input->poll = lm95071_dev_poll;
	this->input->poll_interval = POLL_INTERVAL;
	this->input->poll_interval_min = POLL_INTERVAL_MIN;
	this->input->poll_interval_max = POLL_INTERVAL_MAX;
	this->input->input->name = "lm95071-temp";
	this->input->input->id.bustype = BUS_I2C;
	this->input->input->evbit[0] = BIT_MASK(EV_ABS);
	this->input->private = this;

	/**
	 * 14 bits given in 2-complement representation from -40C to +150C in mCelsius.
	 * Each bit represents 0.03125 Celsius = 31 mCelsius.
	 */
	input_set_abs_params(this->input->input, ABS_MISC, -40000, 150000, 0, 0);
	error = input_register_polled_device(this->input);

	if(error)
	{
		dev_err(&(spi->dev), "error: cannot register the input device\n");
		error = -ENOMEM;
		goto err_register_input;
	}

	spi_set_drvdata(spi, this);

	lm95071_continuous_operation(this);

	return 0;

err_register_input:
	input_free_polled_device(this->input);
err_alloc_input:
err_spi_setup:
	kfree(this);
err_no_mem_dev:
	return error;
}

static int lm95071_spi_remove(struct spi_device *spi)
{
	struct lm95071_chip *this = spi_get_drvdata(spi);

	input_unregister_polled_device(this->input);
	input_free_polled_device(this->input);
	kfree(this);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lm95071_dt_id[] = {
	{.compatible = "ti,lm95071",},
	{},
};
MODULE_DEVICE_TABLE(of, lm95071_dt_id);
#endif

static const struct spi_device_id lm95071_id[] = { { "lm95071", 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lsm9ds1_acc_gyr_id);

static struct spi_driver lm95071_spi_driver = {
	.driver = {
		.name	= "lm95071",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lm95071_dt_id),
#endif
		.owner	= THIS_MODULE,
		.pm	= &lm95071_spi_pm,
	},
	.probe		= lm95071_spi_probe,
	.remove		= lm95071_spi_remove,
	.id_table 	= lm95071_id,
};

module_spi_driver(lm95071_spi_driver);

MODULE_DESCRIPTION("Texas Instruments LM95071 Temperature Sensor driver");
MODULE_AUTHOR("Mauricio Cirelli <mauricio@turingcomputer.com.br>");
MODULE_LICENSE("GPL");
