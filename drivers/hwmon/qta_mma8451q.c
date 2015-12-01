/*
 * qta_mma8451q.c
 *
 *	Digital 14-bits 3-axis accelerometer driver for
 *	Freescale MMA8451Q module.
 *
 *  Created on: 26/11/2013
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

#define MMA8451Q_DRV_NAME			"mma8451q"
#define POLL_INTERVAL_MIN			20
#define POLL_INTERVAL_MAX			160
#define POLL_INTERVAL				80

/**
 * Identification register
 */
#define MMA8451Q_CHIP_ID			0x0D
#define MMA8451Q_CHIP_ID_VALUE		0x1A

/**
 * Accelerometer data registers
 */
#define MMA8451Q_XAXIS_MSB			0x01
#define MMA8451Q_XAXIS_LSB			0x02
#define MMA8451Q_YAXIS_MSB			0x03
#define MMA8451Q_YAXIS_LSB			0x04
#define MMA8451Q_ZAXIS_MSB			0x05
#define MMA8451Q_ZAXIS_LSB			0x06

/**
 * Configuration registers
 */
#define MMA8451Q_FIFO_SETUP			0x09
#define MMA8451Q_DATA_CONFIG		0x0E
#define MMA8451Q_CTRL1				0x2A
#define MMA8451Q_CTRL2				0x2B
#define MMA8451Q_CTRL3				0x2C
#define MMA8451Q_CTRL4				0x2D
#define MMA8451Q_CTRL5				0x2E

/**
 * Interrupt registers
 */
#define MMA8451Q_INT_SRC			0x0C

/**
 * Range modes accordingly to the datasheet
 */
typedef enum {

	/**
	 * 1 bit = 250 ug
	 * Range = -2g to + 2g
	 */
	MMA8451Q_2G = 0x00,
	/**
	 * 1 bit = 500 ug
	 * Range = -4g to +4g
	 */
	MMA8451Q_4G = 0x01,
	/**
	 * 1 bit = 1000 ug
	 * Range = -8g to +8g
	 */
	MMA8451Q_8G = 0x02,

} mma8451q_range_modes;

typedef enum
{
	/**
	 * Normal operating mode.
	 */
	MMA8451Q_ACTIVE = 0x00,

	/**
	 * Low power mode.
	 */
	MMA8451Q_STANDBY,

} mma8451q_power_modes;

struct mma8451q_dev {
	struct i2c_client 							*client;
	struct input_polled_dev						*idev;
	struct device 								*dev;
	mma8451q_range_modes						range_mode;
	mma8451q_power_modes						power_mode;
	struct mutex								lock;
	struct quanta_motion_sensors_platform_data 	*platform;
};

static unsigned int mma8451q_get_accel_conversion(struct mma8451q_dev *sensor)
{

	if(sensor == NULL)
	{
		pr_err("%s: MMA8451Q sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	switch(sensor->range_mode)
	{
		case MMA8451Q_2G:
			/**
			 * 1 bit = 250 ug
			 */
			return 250;
		case MMA8451Q_4G:
			/**
			 * 1 bit = 500 ug
			 */
			return 500;
		case MMA8451Q_8G:
			/**
			 * 1 bit = 1000 ug
			 */
			return 1000;
		default:
			/**
			 * This is an error.. should not happen
			 */
			pr_err("%s: Invalid range mode for MMA8451Q driver!", __FUNCTION__);
			BUG();
			return -EINVAL;
	}
}

static int mma8451q_read_reg(struct mma8451q_dev *sensor, u8 reg, u8 *value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: MMA8451Q sensor structure is null", __FUNCTION__);
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

static int mma8451q_write_reg(struct mma8451q_dev *sensor, u8 reg, u8 value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: MMA8451Q sensor structure is null", __FUNCTION__);
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

static int mma8451q_set_range(struct mma8451q_dev *sensor, mma8451q_range_modes mode)
{
	int value = 0x00;
	int error = 0x00;

	switch(mode)
	{
		case MMA8451Q_2G:
			value = 0x00;
			break;
		case MMA8451Q_4G:
			value = 0x01;
			break;
		case MMA8451Q_8G:
			value = 0x02;
			break;
		default:
			return -EINVAL;
	}

	mutex_lock(&sensor->lock);
	error = mma8451q_write_reg(sensor, MMA8451Q_DATA_CONFIG, value);
	mutex_unlock(&sensor->lock);

	if(error)
		return error;

	sensor->range_mode = mode;
	return 0;
}

static int mma8451q_set_power(struct mma8451q_dev *sensor, mma8451q_power_modes mode)
{
	int error = 0x00;

	switch(mode)
	{
	case MMA8451Q_ACTIVE:
		error = mma8451q_write_reg(sensor, MMA8451Q_CTRL1, 0x6D);
		break;
	case MMA8451Q_STANDBY:
		error = mma8451q_write_reg(sensor, MMA8451Q_CTRL1, 0x6C);
		break;
	default:
		return -EINVAL;
	}

	if(error)
	{
		dev_err(&sensor->client->dev, "error while setting new power mode: %d", error);
		return error;
	}

	sensor->power_mode = mode;
	return 0;
}

static int mma8451q_read_accel(struct mma8451q_dev *sensor, int *x, int *y, int *z)
{
	int error = 0;
	u8 x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3DH sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	mutex_lock(&sensor->lock);
	error |= mma8451q_read_reg(sensor, MMA8451Q_XAXIS_LSB, &x_lsb);
	error |= mma8451q_read_reg(sensor, MMA8451Q_XAXIS_MSB, &x_msb);
	error |= mma8451q_read_reg(sensor, MMA8451Q_YAXIS_LSB, &y_lsb);
	error |= mma8451q_read_reg(sensor, MMA8451Q_YAXIS_MSB, &y_msb);
	error |= mma8451q_read_reg(sensor, MMA8451Q_ZAXIS_LSB, &z_lsb);
	error |= mma8451q_read_reg(sensor, MMA8451Q_ZAXIS_MSB, &z_msb);
	mutex_unlock(&sensor->lock);

	if(error)
		goto out;

	/**
	 * 14 bits given in two ́2-complement representation:
	 * MSB register contain 8 most significant bits
	 * LSB register contain 6 least significant bits in its 8th, 7th, 6th, 5th, 4th and 3rd bits
	 */
	*x = (((int)x_msb << 8) && 0x3FC0) + (((int) x_lsb >> 2) & 0x003F);
	*y = (((int)y_msb << 8) && 0x3FC0) + (((int) y_lsb >> 2) & 0x003F);
	*z = (((int)z_msb << 8) && 0x3FC0) + (((int) z_lsb >> 2) & 0x003F);

	out:
		return error;
}

static void mma8451q_dev_poll(struct input_polled_dev *dev)
{
	struct mma8451q_dev *sensor = input_get_drvdata(dev->input);
	int x, y, z;
	int accel_conversion;
	int error = 0;

	if(sensor->power_mode != MMA8451Q_ACTIVE)
		goto out;

	error = mma8451q_read_accel(sensor, &x, &y, &z);
	if(error)
	{
		dev_err(&sensor->client->dev, "error reading acceleration: %d\n", error);
		goto out;
	}
	quanta_motion_sensors_adjust_position(sensor->platform, &x, &y, &z);

	accel_conversion = mma8451q_get_accel_conversion(sensor);

	/* Report XYZ data in micro-G units */
	input_report_abs(sensor->idev->input, ABS_X, (x * accel_conversion));
	input_report_abs(sensor->idev->input, ABS_Y, (y * accel_conversion));
	input_report_abs(sensor->idev->input, ABS_Z, (z * accel_conversion));

	input_sync(sensor->idev->input);

	out:
		return;
}

static ssize_t mma8451q_sysfs_set_range_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	mma8451q_range_modes range_mode;
	int error = 0;
	unsigned long new_range = simple_strtoul(buf, NULL, 10);
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct mma8451q_dev *sensor = ipdev->private;

	switch(new_range)
	{
	case MMA8451Q_2G:
	case MMA8451Q_4G:
	case MMA8451Q_8G:
		range_mode = (mma8451q_range_modes) new_range;
		error = mma8451q_set_range(sensor, range_mode);
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

static ssize_t mma8451q_sysfs_get_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct mma8451q_dev *sensor = ipdev->private;

	if(sensor == NULL)
	{
		pr_err("%s: MMA8451Q sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sensor->range_mode);
}

static DEVICE_ATTR(range_mode, S_IWUSR | S_IRUGO, mma8451q_sysfs_get_range, mma8451q_sysfs_set_range_mode);

static struct attribute *mma8451q_attributes[] =
{
	&dev_attr_range_mode.attr,
	NULL
};

static const struct attribute_group mma8451q_attr_group =
{
	.name = "mma8451q",
	.attrs = mma8451q_attributes,
};

static int __devinit
mma8451q_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 client_id = 0;
	int error = 0;
	struct mma8451q_dev *this = (struct mma8451q_dev *) kcalloc(sizeof(struct mma8451q_dev), 1, GFP_KERNEL);
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
	error = mma8451q_read_reg(this, MMA8451Q_CHIP_ID, &client_id);
	if(error)
	{
		dev_err(&(client->dev), "error: device does not exist\n");
		error = -ENODEV;
		goto err_no_dev;
	}

	if(client_id != MMA8451Q_CHIP_ID_VALUE)
	{
		dev_err(&(client->dev), "error: device id 0x%X is not the expected (0x%x)\n", client_id, MMA8451Q_CHIP_ID_VALUE);
		error = -ENODEV;
		goto err_no_dev;
	}

	/**
	 * Initial configuration:
	 * 2g, Normal Mode, FIFO disabled, 12.5Hz sampling rate and no IRQs (polled device)
	 */
	error = 0;
	error |= mma8451q_set_range(this, MMA8451Q_2G);
	error |= mma8451q_set_power(this, MMA8451Q_ACTIVE);
	error |= mma8451q_write_reg(this, MMA8451Q_FIFO_SETUP, 0x00);
	error |= mma8451q_write_reg(this, MMA8451Q_CTRL4, 0x00);
	error |= mma8451q_write_reg(this, MMA8451Q_CTRL5, 0xFF);

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
	 * Allocate the polled input device
	 */
	this->idev = input_allocate_polled_device();
	if (IS_ERR(this->idev))
	{
		dev_err(&(client->dev), "error: out of memory for input device\n");
		error = -ENOMEM;
		goto err_alloc_input;
	}

	/**
	 * Configure the input device
	 */
	this->idev->poll = mma8451q_dev_poll;
	this->idev->poll_interval = POLL_INTERVAL;
	this->idev->poll_interval_min = POLL_INTERVAL_MIN;
	this->idev->poll_interval_max = POLL_INTERVAL_MAX;
	this->idev->input->name = "accelerometer";
	this->idev->input->id.bustype = BUS_I2C;
	this->idev->input->evbit[0] = BIT_MASK(EV_ABS);
	this->idev->private = this;

	/**
	 * 14 bits given in 2-complement representation:
	 * The acceleration will be given in multiples of
	 * micro-G, depending on the resolution.
	 * The current resolution is given by the range_mode attribute.
	 */
	input_set_abs_params(this->idev->input, ABS_X, -8000000, 8000000, 0, 0);
	input_set_abs_params(this->idev->input, ABS_Y, -8000000, 8000000, 0, 0);
	input_set_abs_params(this->idev->input, ABS_Z, -8000000, 8000000, 0, 0);

	error = input_register_polled_device(this->idev);

	if(error)
	{
		dev_err(&(client->dev), "error: cannot register the input device\n");
		error = -ENOMEM;
		goto err_register_input;
	}

	error = sysfs_create_group(&this->idev->input->dev.kobj, &mma8451q_attr_group);
	if (error)
	{
		dev_err(&client->dev, "create device file failed!\n");
		error = -EINVAL;
		goto err_register_sys_fs;
	}

	this->platform = (struct quanta_motion_sensors_platform_data *)client->dev.platform_data;
	i2c_set_clientdata(client, this);

	return 0;

	err_register_sys_fs:
		input_unregister_polled_device(this->idev);
	err_register_input:
		input_free_polled_device(this->idev);
	err_alloc_input:
		hwmon_device_unregister(this->dev);
	err_hw_mon:
	err_config:
	err_no_dev:
		mutex_destroy(this->lock);
		kfree(this);
	err_no_mem_dev:
		return error;
}

static int __devexit mma8451q_remove(struct i2c_client *client)
{
	struct mma8451q_dev *sensor = i2c_get_clientdata(client);

	sysfs_remove_group(&sensor->idev->input->dev.kobj, &mma8451q_attr_group);
	input_unregister_polled_device(sensor->idev);
	input_free_polled_device(sensor->idev);
	hwmon_device_unregister(sensor->dev);
	mutex_destroy(sensor->lock);
	kfree(sensor);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mma8451q_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mma8451q_dev *sensor = i2c_get_clientdata(client);

	int error = mma8451q_set_power(sensor, MMA8451Q_STANDBY);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into suspended mode: %d\n", error);
	}

	return error;
}

static int mma8451q_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mma8451q_dev *sensor = i2c_get_clientdata(client);

	int error = mma8451q_set_power(sensor, MMA8451Q_ACTIVE);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into normal mode: %d\n", error);
	}

	return error;

}
#else
static int mma8451q_suspend(struct device *dev) { return 0; }
static int mma8451q_resume(struct device *dev) { return 0; }
#endif

static const struct dev_pm_ops mma8451q_pm_ops = {
		.suspend = mma8451q_suspend,
		.resume = mma8451q_resume,
};

static const struct i2c_device_id mma8451q_id[] = {
	{MMA8451Q_DRV_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mma8451q_id);

static struct i2c_driver mma8451q_driver = {
	.driver = {
		.name = MMA8451Q_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &mma8451q_pm_ops,
	},
	.probe = mma8451q_probe,
	.remove = __devexit_p(mma8451q_remove),
	.id_table = mma8451q_id,
};

static int __init
mma8451q_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&mma8451q_driver);
	if (res < 0)
	{
		pr_err("%s: add mma8451q i2c driver failed\n", __FUNCTION__);
		return -ENODEV;
	}
	return res;
}

static void __exit
mma8451q_exit(void)
{
	i2c_del_driver(&mma8451q_driver);
}

MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("Freescale MMA8451Q 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(mma8451q_init);
module_exit(mma8451q_exit);
