/*
 * bma250e.c
 *
 *	Digital 10-bits 3-axis accelerometer and 8-bit termometer driver for
 *	Bosch BMA250E module.
 *
 *  Created on: 22/11/2013
 *      Author: mauricio.cirelli
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/quanta_sensors.h>

#define BMA250E_DRV_NAME			"bma250e"
#define POLL_INTERVAL_MIN			20
#define POLL_INTERVAL_MAX			200
#define POLL_INTERVAL				100

/**
 * Identification register
 */
#define BMA250E_CHIP_ID				0x00
#define BMA250E_CHIP_ID_VALUE		0x03

/**
 * Data registers
 */
#define BMA250E_XAXIS_LSB			0x02
#define BMA250E_XAXIS_MSB			0x03
#define BMA250E_YAXIS_LSB			0x04
#define BMA250E_YAXIS_MSB			0x05
#define BMA250E_ZAXIS_LSB			0x06
#define BMA250E_ZAXIS_MSB			0x07

// Temperature is SIGNED 8-bits
// 1 bit = 0.5 Kelvin
// 0x00 = 24 Celsius
// thus, range is from -40 to +87.5 Celsius
#define BMA250E_TEMPERATURE			0x08

/**
 * Interrupt status registers
 */
#define BMA250E_INT_STATUS			0x09
#define BMA250E_NEWDATA_STATUS		0x0A
#define BMA250E_INT_SLOPE_STATUS	0x0B
#define BMA250E_INT_ORIENT_STATUS	0x0C

/**
 * Chip's configuration registers
 */
#define BMA250E_RANGE_MODE			0x0F
#define BMA250E_POWER_MODE			0x11
#define BMA250E_ACQUISITION_MODE	0x13
#define BMA250E_SOFT_RESET			0x14

/**
 * Interrupt configuration registers
 */
#define BMA250E_IRQ_MASK0			0x16
#define BMA250E_IRQ_MASK1			0x17
#define BMA250E_IRQ_PINOUT0			0x19
#define BMA250E_IRQ_PINOUT1			0x1A
#define BMA250E_IRQ_PINOUT2			0x1B
#define BMA250E_IRQ_FILTERMASK		0x1E
#define BMA250E_IRQ_LEVEL			0x20
#define BMA250E_IRQ_MODE			0x21

/**
 * Low-g and High-g detection configuration registers
 */
#define BMA250E_LOWG_IRQ_DELAY		0x22
#define BMA250E_LOWG_IRQ_THRESHOLD	0x23
#define BMA250E_LOHIG_IRQ_MODE		0x24
#define BMA250E_HIGHG_IRQ_DELAY		0x25
#define BMA250E_HIGHG_IRQ_THRESHOLD	0x26

/**
 * Slope detection configuration registers
 */
#define BMA250E_SLOPE_SAMPLES		0x27
#define BMA250E_SLOPE_THRESHOLD		0x28

/**
 * Orientation detection configuration registers
 */
#define BMA250E_ORIENT_CONFIG		0x2C
#define BMA250E_ORIENT_THETA		0x2D

/**
 * Flat detection configuration registers
 */
#define BMA250E_FLAT_THRESHOLD		0x2E
#define BMA250E_FLAT_DELAY			0x2F

/**
 * Range modes accordingly to the datasheet
 */
typedef enum {

	/**
	 * 1 bit = 3.91mg
	 * Range = -2g to + 2g
	 */
	BMA250E_2G = 0x00,
	/**
	 * 1 bit = 7.81mg
	 * Range = -4g to +4g
	 */
	BMA250E_4G = 0x01,
	/**
	 * 1 bit = 15.62mg
	 * Range = -8g to +8g
	 */
	BMA250E_8G = 0x02,
	/**
	 * 1 bit = 31.25mg
	 * Range = -16g to +16g
	 */
	BMA250E_16G = 0x03

} bma250e_range_modes;

/**
 * Active modes accordingly to datasheet.
 */
typedef enum {
	/**
	 * Active mode.
	 */
	BMA250E_NORMAL = 0x00,
	/**
	 * Suspended mode.
	 */
	BMA250E_SUSPENDED = 0x80,
	/**
	 * Low Power mode.
	 * Currently it is not supported.
	 * Setting to Suspended Mode
	 */
	BMA250E_LOW_POWER = BMA250E_SUSPENDED,
} bma250e_power_modes;

struct bma250e_dev {
	struct i2c_client 							*client;
	struct input_polled_dev						*idev_accel;
#ifdef CONFIG_BOSCH_BMA250E_TEMP
	struct input_polled_dev						*idev_temp;
#endif
	struct device 								*dev;
	bma250e_range_modes							range_mode;
	bma250e_power_modes							power_mode;
	struct mutex								lock;
	struct quanta_motion_sensors_platform_data 	*platform;
};

static unsigned int bma250e_get_accel_conversion(struct bma250e_dev *sensor)
{

	if(sensor == NULL)
	{
		pr_err("%s: BMA250E sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	switch(sensor->range_mode)
	{
		case BMA250E_2G:
			/**
			 * 1 bit = 3.91 mg
			 * 1 bit = 3910 ug
			 */
			return 3910;
		case BMA250E_4G:
			/**
			 * 1 bit = 7.81 mg
			 * 1 bit = 7810 ug
			 */
			return 7810;
		case BMA250E_8G:
			/**
			 * 1 bit = 15.62 mg
			 * 1 bit = 15620 ug
			 */
			return 15620;
		case BMA250E_16G:
			/**
			 * 1 bit = 31.25 mg
			 * 1 bit = 31250 ug
			 */
			return 31250;
		default:
			/**
			 * This is an error.. should not happen
			 */
			pr_err("%s: Invalid range mode for BMA250E driver!", __FUNCTION__);
			BUG();
			return -EINVAL;
	}
}

static int bma250e_read_reg(struct bma250e_dev *sensor, u8 reg, u8 *value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: BMA250E sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	ret = i2c_smbus_read_byte_data(sensor->client, reg);
	if(ret < 0)
	{
		dev_err(&sensor->client->dev, "I2C read from 0x%02X error: %d", reg, ret);
		return ret;
	}

	*value = ret;

	return 0;
}

static int bma250e_write_reg(struct bma250e_dev *sensor, u8 reg, u8 value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: BMA250E sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	ret = i2c_smbus_write_byte_data(sensor->client, reg, value);
	if(ret < 0)
	{
		dev_err(&sensor->client->dev, "I2C write to 0x%02X error: %d", reg, ret);
		return ret;
	}

	return 0;
}

static int bma250e_read_accel(struct bma250e_dev *sensor, int *x, int *y, int *z)
{
	int error = 0;
	u8 x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;

	if(sensor == NULL)
	{
		pr_err("%s: BMA250E sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	/**
	 * We must read the LSB before the MSB,
	 * as the datasheet recommends.
	 */
	mutex_lock(&sensor->lock);
	error |= bma250e_read_reg(sensor, BMA250E_XAXIS_LSB, &x_lsb);
	error |= bma250e_read_reg(sensor, BMA250E_XAXIS_MSB, &x_msb);
	error |= bma250e_read_reg(sensor, BMA250E_YAXIS_LSB, &y_lsb);
	error |= bma250e_read_reg(sensor, BMA250E_YAXIS_MSB, &y_msb);
	error |= bma250e_read_reg(sensor, BMA250E_ZAXIS_LSB, &z_lsb);
	error |= bma250e_read_reg(sensor, BMA250E_ZAXIS_MSB, &z_msb);
	mutex_unlock(&sensor->lock);

	if(error)
		goto out;

	/**
	 * 10 bits given in two ́2-complement representation:
	 * MSB register contain 8 most significant bits
	 * LSB register contain 2 least significant bits in its 8th and 7th bits.
	 */
	*x = (((int) x_msb << 2) & 0x03FC) + (((int) x_lsb >> 6) & 0x0003);
	*y = (((int) y_msb << 2) & 0x03FC) + (((int) y_lsb >> 6) & 0x0003);
	*z = (((int) z_msb << 2) & 0x03FC) + (((int) z_lsb >> 6) & 0x0003);

	out:
		return error;
}

#ifdef CONFIG_BOSCH_BMA250E_TEMP

static int bma250e_read_temperature(struct bma250e_dev *sensor, int *temp)
{
	u8 temp_data = 0x00;
	int error = 0;

	error |= bma250e_read_reg(sensor, BMA250E_TEMPERATURE, &temp_data);

	if(error)
		goto out;

	/**
	 * Temperature is given in 8-bit 2-complement
	 * 1 bit = 0.5 Kelvin = 500 milli-Kelvin
	 * 0x00 = 24 Celsius = 297150 milli-Kelvin
	 */

	*temp = (temp_data * 500) + 297150;

	out:
		return error;
}

#endif

static int bma250e_set_range(struct bma250e_dev *sensor, bma250e_range_modes mode)
{
	int value = 0x00;
	int error = 0x00;

	switch(mode)
	{
		case BMA250E_2G:
			value = 0x03;
			break;
		case BMA250E_4G:
			value = 0x05;
			break;
		case BMA250E_8G:
			value = 0x08;
			break;
		case BMA250E_16G:
			value = 0x0C;
			break;
		default:
			return -EINVAL;
	}

	mutex_lock(&sensor->lock);
	error = bma250e_write_reg(sensor, BMA250E_RANGE_MODE, value);
	mutex_unlock(&sensor->lock);

	if(error)
		return error;

	sensor->range_mode = mode;
	return 0;
}

static int bma250e_set_power(struct bma250e_dev *sensor, bma250e_power_modes mode)
{
	return bma250e_write_reg(sensor, BMA250E_POWER_MODE, mode);
}

#ifdef CONFIG_BOSCH_BMA250E_TEMP

static void bma250e_dev_poll_temp(struct input_polled_dev *dev)
{
	struct bma250e_dev *sensor = input_get_drvdata(dev->input);
	int temp;
	int error = 0;

	if(sensor->power_mode != BMA250E_NORMAL)
		goto out;

	error = bma250e_read_temperature(sensor, &temp);
	if(error)
	{
		dev_err(&sensor->client->dev, "error reading temperature: %d\n", error);
		goto out;
	}

	/* Report temperature data in milli-Kelvins */
	input_report_abs(sensor->idev_temp->input, ABS_MISC, temp);
	input_sync(sensor->idev_temp->input);

	out:
		return;
}

#endif

static void bma250e_dev_poll_accel(struct input_polled_dev *dev)
{
	struct bma250e_dev *sensor = input_get_drvdata(dev->input);
	int x, y, z;
	int accel_conversion;
	int error = 0;

	if(sensor->power_mode != BMA250E_NORMAL)
		goto out;

	error = bma250e_read_accel(sensor, &x, &y, &z);
	if(error)
	{
		dev_err(&sensor->client->dev, "error reading acceleration: %d\n", error);
		goto out;
	}
	quanta_motion_sensors_adjust_position(sensor->platform, &x, &y, &z);

	accel_conversion = bma250e_get_accel_conversion(sensor);

	/* Report XYZ data in micro-G units */
	input_report_abs(sensor->idev_accel->input, ABS_X, (x * accel_conversion));
	input_report_abs(sensor->idev_accel->input, ABS_Y, (y * accel_conversion));
	input_report_abs(sensor->idev_accel->input, ABS_Z, (z * accel_conversion));
	input_sync(sensor->idev_accel->input);

	out:
		return;
}

static ssize_t bma250e_sysfs_set_range_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bma250e_range_modes range_mode;
	int error = 0;
	unsigned long new_range = simple_strtoul(buf, NULL, 10);
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct bma250e_dev *sensor = ipdev->private;

	switch(new_range)
	{
	case BMA250E_2G:
	case BMA250E_4G:
	case BMA250E_8G:
	case BMA250E_16G:
		range_mode = (bma250e_range_modes) new_range;
		error = bma250e_set_range(sensor, range_mode);
		if(error)
		{
			dev_err(&sensor->client->dev, "error setting new range mode: %d\n", error);
			return error;
		}
		return count;
	default:
		dev_err(&sensor->client->dev, "invalid range mode: %lu\n", new_range);
		return -EINVAL;
	}

}

static ssize_t bma250e_sysfs_get_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct bma250e_dev *sensor = ipdev->private;

	if(sensor == NULL)
	{
		pr_err("%s: BMA250E sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sensor->range_mode);
}

static DEVICE_ATTR(range_mode, S_IWUSR | S_IRUGO, bma250e_sysfs_get_range, bma250e_sysfs_set_range_mode);

static struct attribute *bma250e_attributes[] =
{
	&dev_attr_range_mode.attr,
	NULL
};

static const struct attribute_group bma250e_attr_group =
{
	.name = "bma250e",
	.attrs = bma250e_attributes,
};

static int __devinit
bma250e_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 client_id = 0;
	int error = 0;
	struct bma250e_dev *this = (struct bma250e_dev *) kcalloc(sizeof(struct bma250e_dev), 1, GFP_KERNEL);
	if (IS_ERR(this))
	{
		dev_err(&(client->dev), "error: out of memory for device\n");
		error = -ENOMEM;
		goto err_no_mem_dev;
	}

	mutex_init(&this->lock);
	this->client = client;
	/**
	 * Temporary storage, to log error messages until we
	 * create the hwmon device
	 */
	this->dev = &(client->dev);

	/**
	 * Probes the device first
	 */
	error = bma250e_read_reg(this, BMA250E_CHIP_ID, &client_id);
	if(error)
	{
		dev_err(&(client->dev), "error: device does not exist\n");
		error = -ENODEV;
		goto err_no_dev;
	}

	if(client_id != BMA250E_CHIP_ID_VALUE)
	{
		dev_err(&(client->dev), "error: device id 0x%X is not the expected (0x%x)\n", client_id, BMA250E_CHIP_ID_VALUE);
		error = -ENODEV;
		goto err_no_dev;
	}

	/**
	 * Initial configuration:
	 * 2g, Normal Mode and no IRQs (polled device)
	 */
	error = 0;
	error |= bma250e_set_range(this, BMA250E_2G);
	error |= bma250e_set_power(this, BMA250E_NORMAL);
	error |= bma250e_write_reg(this, BMA250E_IRQ_PINOUT0, 0x00);
	error |= bma250e_write_reg(this, BMA250E_IRQ_PINOUT1, 0x00);
	error |= bma250e_write_reg(this, BMA250E_IRQ_PINOUT2, 0x00);

	if(error)
	{
		dev_err(&(client->dev), "error: configuring the device\n");
		error = -EIO;
		goto err_config;
	}

	/**
	 * Allocate the hwmon device
	 */
	this->dev = hwmon_device_register(&client->dev);
	if (IS_ERR(this->dev))
	{
		dev_err(&(client->dev), "error: out of memory for hwmon device\n");
		error = -ENOMEM;
		goto err_hw_mon;
	}

	/**
	 * Allocate the polled input device for accelerometer measures
	 */
	this->idev_accel = input_allocate_polled_device();
	if (IS_ERR(this->idev_accel))
	{
		dev_err(&(client->dev), "error: out of memory for accelerometer input device\n");
		error = -ENOMEM;
		goto err_alloc_input_accel;
	}

	/**
	 * Configure the input device
	 */
	this->idev_accel->poll = bma250e_dev_poll_accel;
	this->idev_accel->poll_interval = POLL_INTERVAL;
	this->idev_accel->poll_interval_min = POLL_INTERVAL_MIN;
	this->idev_accel->poll_interval_max = POLL_INTERVAL_MAX;
	this->idev_accel->input->name = "accelerometer";
	this->idev_accel->input->id.bustype = BUS_I2C;
	this->idev_accel->input->evbit[0] = BIT_MASK(EV_ABS);
	this->idev_accel->private = this;

	/**
	 * 10 bits given in 2-complement representation:
	 * [-512, 511] interval
	 * The acceleration will be given in multiples of
	 * micro-G, depending on the resolution.
	 * The current resolution is given by the range_mode attribute.
	 */
	input_set_abs_params(this->idev_accel->input, ABS_X, -16000000, 16000000, 0, 0);
	input_set_abs_params(this->idev_accel->input, ABS_Y, -16000000, 16000000, 0, 0);
	input_set_abs_params(this->idev_accel->input, ABS_Z, -16000000, 16000000, 0, 0);

	error = input_register_polled_device(this->idev_accel);

	if(error)
	{
		dev_err(&(client->dev), "error: cannot register the accelerometer input device\n");
		error = -ENOMEM;
		goto err_register_input_accel;
	}

	error = sysfs_create_group(&this->idev_accel->input->dev.kobj, &bma250e_attr_group);
	if (error)
	{
		dev_err(&client->dev, "create device file failed!\n");
		error = -EINVAL;
		goto err_register_sys_fs;
	}

#ifdef CONFIG_BOSCH_BMA250E_TEMP

	/**
	 * Allocate the polled input device for temperature measures
	 */
	this->idev_temp = input_allocate_polled_device();
	if (IS_ERR(this->idev_temp))
	{
		dev_err(&(client->dev), "error: out of memory for temperature input device\n");
		error = -ENOMEM;
		goto err_alloc_input_temp;
	}

	/**
	 * Configure the input device
	 */
	this->idev_temp->poll = bma250e_dev_poll_temp;
	this->idev_temp->poll_interval = POLL_INTERVAL;
	this->idev_temp->poll_interval_min = POLL_INTERVAL_MIN;
	this->idev_temp->poll_interval_max = POLL_INTERVAL_MAX;
	this->idev_temp->input->name = "termometer";
	this->idev_temp->input->id.bustype = BUS_I2C;
	this->idev_temp->input->evbit[0] = BIT_MASK(EV_ABS);

	/**
	 * 8 bits given in 2-complement representation
	 * [-128, 127]
	 * The range goes from -40 to 87.5 Celsius
	 * temp = 0x00 => 24 Celsius = 297150 milli-Kelvin
	 * The temperature will be given in milli-Kelvins
	 */
	input_set_abs_params(this->idev_temp->input, ABS_MISC, 233150, 360650, 0, 0);

	error = input_register_polled_device(this->idev_temp);

	if(error)
	{
		dev_err(&(client->dev), "error: cannot register the temperature input device\n");
		error = -ENOMEM;
		goto err_register_input_temp;
	}

#endif

	this->platform = (struct quanta_motion_sensors_platform_data *)client->dev.platform_data;
	i2c_set_clientdata(client, this);

	return 0;

#ifdef CONFIG_BOSCH_BMA250E_TEMP
	err_register_input_temp:
		input_free_polled_device(this->idev_temp);
	err_alloc_input_temp:
		sysfs_remove_group(&this->idev_accel->input->dev.kobj, &bma250e_attr_group);
#endif
	err_register_sys_fs:
		input_unregister_polled_device(this->idev_accel);
	err_register_input_accel:
			input_free_polled_device(this->idev_accel);
	err_alloc_input_accel:
		hwmon_device_unregister(this->dev);
	err_hw_mon:
	err_config:
	err_no_dev:
		mutex_destroy(this->lock);
		kfree(this);
	err_no_mem_dev:
		return error;

}

static int __devexit bma250e_remove(struct i2c_client *client)
{
	struct bma250e_dev *sensor = i2c_get_clientdata(client);

#ifdef CONFIG_BOSCH_BMA250E_TEMP
	input_unregister_polled_device(sensor->idev_temp);
	input_free_polled_device(sensor->idev_temp);
#endif
	sysfs_remove_group(&sensor->idev_accel->input->dev.kobj, &bma250e_attr_group);
	input_unregister_polled_device(sensor->idev_accel);
	input_free_polled_device(sensor->idev_accel);
	hwmon_device_unregister(sensor->dev);
	mutex_destroy(sensor->lock);
	kfree(sensor);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bma250e_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250e_dev *sensor = i2c_get_clientdata(client);

	int error = bma250e_set_power(sensor, BMA250E_SUSPENDED);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into suspended mode: %d\n", error);
	}

	return error;
}

static int bma250e_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250e_dev *sensor = i2c_get_clientdata(client);

	int error = bma250e_set_power(sensor, BMA250E_NORMAL);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into normal mode: %d\n", error);
	}

	return error;

}
#else
static int bma250e_suspend(struct device *dev) { return 0; }
static int bma250e_resume(struct device *dev) { return 0; }
#endif

static const struct dev_pm_ops bma250e_pm_ops = {
		.suspend = bma250e_suspend,
		.resume = bma250e_resume,
};

static const struct i2c_device_id bma250e_id[] = {
	{BMA250E_DRV_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, bma250e_id);

static struct i2c_driver bma250e_driver = {
	.driver = {
		.name = BMA250E_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &bma250e_pm_ops,
	},
	.probe = bma250e_probe,
	.remove = __devexit_p(bma250e_remove),
	.id_table = bma250e_id,
};

static int __init
bma250e_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&bma250e_driver);
	if (res < 0)
	{
		pr_err("%s: add bma250e i2c driver failed\n", __FUNCTION__);
		return -ENODEV;
	}
	return res;
}

static void __exit
bma250e_exit(void)
{
	i2c_del_driver(&bma250e_driver);
}

MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("Bosch BMA250E 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(bma250e_init);
module_exit(bma250e_exit);
