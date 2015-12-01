/*
 * qta_lis3dh.c
 *
 *	Digital 16-bits 3-axis accelerometer and 8-bit termometer driver for
 *	ST LIS3DH module.
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

#define LIS3DH_DRV_NAME				"lis3dh"
#define POLL_INTERVAL_MIN			20
#define POLL_INTERVAL_MAX			200
#define POLL_INTERVAL				100

/**
 * Status register
 */
#define LIS3DH_STATUS_AUX			0x07

/**
 * Identification register
 */
#define LIS3DH_CHIP_ID				0x0F
#define LIS3DH_CHIP_ID_VALUE		0x33

/**
 * Temperature configuration
 */
#define LIS3DH_TEMP_CONFIG			0x1F

/**
 * Control registers
 */
#define LIS3DH_CTRL1				0x20
#define LIS3DH_CTRL2				0x21
#define LIS3DH_CTRL3				0x22
#define LIS3DH_CTRL4				0x23
#define LIS3DH_CTRL5				0x24
#define LIS3DH_CTRL6				0x25

/**
 * Interrupt management
 */
#define LIS3DH_INT_REFERENCE		0x26
#define LIS3DH_INT1_CONFIG			0x30
#define LIS3DH_INT1_SRC				0x31
#define LIS3DH_INT1_THRESHOLD		0x32
#define LIS3DH_INT1_DURATION		0x33

/**
 * Click (Tap) IRQ
 */
#define LIS3DH_CLICK_CONFIG			0x38
#define LIS3DH_CLICK_SRC			0x39
#define LIS3DH_CLICK_THRESHOLD		0x3A
#define LIS3DH_CLICK_TIME_LIMIT		0x3B
#define LIS3DH_CLICK_TIME_LATENCY	0x3C
#define LIS3DH_CLICK_TIME_WINDOW	0x3D


/**
 * Accelerometer outputs.
 * 16-bit 2-complement data.
 */
#define LIS3DH_OUT_STATUS			0x27
#define LIS3DH_XAXIS_LSB			0x28
#define LIS3DH_XAXIS_MSB			0x29
#define LIS3DH_YAXIS_LSB			0x2A
#define LIS3DH_YAXIS_MSB			0x2B
#define LIS3DH_ZAXIS_LSB			0x2C
#define LIS3DH_ZAXIS_MSB			0x2D
#define LIS3DH_FIFO_CTRL			0x2E
#define LIS3DH_FIFO_SRC				0x2F

/**
 * Temperature outputs (ADC3)
 */
#define LIS3DH_OUT_ADC3_L			0x0C
#define LIS3DH_OUT_ADC3_H			0x0D


/**
 * Range modes accordingly to the datasheet
 */
typedef enum {

	/**
	 * 1 bit = 1 mg
	 * Range = -2g to + 2g
	 */
	LIS3DH_2G = 0x00,
	/**
	 * 1 bit = 2 mg
	 * Range = -4g to +4g
	 */
	LIS3DH_4G = 0x01,
	/**
	 * 1 bit = 4 mg
	 * Range = -8g to +8g
	 */
	LIS3DH_8G = 0x02,
	/**
	 * 1 bit = 12 mg
	 * Range = -16g to +16g
	 */
	LIS3DH_16G = 0x03

} lis3dh_range_modes;

typedef enum
{
	/**
	 * Normal operating mode.
	 */
	LIS3DH_NORMAL = 0x00,

	/**
	 * Low power mode.
	 * Not supported right now.
	 */
	LIS3DH_LOW_POWER,

	/**
	 * Power down mode.
	 */
	LIS3DH_POWER_DOWN,

} lis3dh_power_modes;

struct lis3dh_dev {
	struct i2c_client 							*client;
	struct input_polled_dev						*idev_accel;
#ifdef CONFIG_ST_LIS3DH_TEMP
	struct input_polled_dev						*idev_temp;
#endif
	struct device 								*dev;
	lis3dh_range_modes							range_mode;
	lis3dh_power_modes							power_mode;
	struct mutex								lock;
	struct quanta_motion_sensors_platform_data 	*platform;
};

static unsigned int lis3dh_get_accel_conversion(struct lis3dh_dev *sensor)
{

	if(sensor == NULL)
	{
		pr_err("%s: LIS3DH sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	switch(sensor->range_mode)
	{
		case LIS3DH_2G:
			/**
			 * 1 bit = 1000 ug
			 */
			return 1000;
		case LIS3DH_4G:
			/**
			 * 1 bit = 2000 ug
			 */
			return 2000;
		case LIS3DH_8G:
			/**
			 * 1 bit = 4000 ug
			 */
			return 4000;
		case LIS3DH_16G:
			/**
			 * 1 bit = 12000 ug
			 */
			return 12000;
		default:
			/**
			 * This is an error.. should not happen
			 */
			pr_err("%s: Invalid range mode for LIS3DH driver!", __FUNCTION__);
			BUG();
			return -EINVAL;
	}
}

static int lis3dh_read_reg(struct lis3dh_dev *sensor, u8 reg, u8 *value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3DH sensor structure is null", __FUNCTION__);
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

static int lis3dh_write_reg(struct lis3dh_dev *sensor, u8 reg, u8 value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3DH sensor structure is null", __FUNCTION__);
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

static int lis3dh_set_range(struct lis3dh_dev *sensor, lis3dh_range_modes mode)
{
	int value = 0x00;
	int error = 0x00;

#ifdef CONFIG_ST_LIS3DH_TEMP
	// Enabling BDU: Block Data Update to continuous mode
	// 7th bit = 1
	value = 0x80;
#endif

	switch(mode)
	{
		case LIS3DH_2G:
			value |= 0x00;
			break;
		case LIS3DH_4G:
			value |= 0x10;
			break;
		case LIS3DH_8G:
			value |= 0x20;
			break;
		case LIS3DH_16G:
			value |= 0x30;
			break;
		default:
			return -EINVAL;
	}

	mutex_lock(&sensor->lock);
	error = lis3dh_write_reg(sensor, LIS3DH_CTRL4, value);
	mutex_unlock(&sensor->lock);

	if(error)
		return error;

	sensor->range_mode = mode;
	return 0;
}

static int lis3dh_set_power(struct lis3dh_dev *sensor, lis3dh_power_modes mode)
{
	int error = 0x00;

	mutex_lock(&sensor->lock);
	switch(mode)
	{
	case LIS3DH_NORMAL:
		error = lis3dh_write_reg(sensor, LIS3DH_CTRL1, 0x37);
		break;
	case LIS3DH_LOW_POWER:
	case LIS3DH_POWER_DOWN:
		error = lis3dh_write_reg(sensor, LIS3DH_CTRL1, 0x00);
		break;
	default:
		return -EINVAL;
	}
	mutex_unlock(&sensor->lock);

	if(error)
	{
		dev_err(&sensor->client->dev, "error while setting new power mode: %d", error);
		return error;
	}

	sensor->power_mode = mode;
	return 0;
}

static int lis3dh_read_accel(struct lis3dh_dev *sensor, int *x, int *y, int *z)
{
	int error = 0;
	u8 status = 0;
	short value_x, value_y, value_z;
	u8 *lsb_x = ((u8*) &value_x);
	u8 *msb_x = ((u8*) &value_x) + 1;
	u8 *lsb_y = ((u8*) &value_y);
	u8 *msb_y = ((u8*) &value_y) + 1;
	u8 *lsb_z = ((u8*) &value_z);
	u8 *msb_z = ((u8*) &value_z) + 1;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3DH sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	/**
	 * Check for new data...
	 */

	mutex_lock(&sensor->lock);
	error |= lis3dh_read_reg(sensor, LIS3DH_OUT_STATUS, &status);
	mutex_unlock(&sensor->lock);

	if(error)
		goto out;

	if(!(status & 0x03))
	{
		dev_err(&sensor->client->dev, "data not ready\n");
		error = -EAGAIN;
		goto out;
	}

	/**
	 * Read new data...
	 */

	mutex_lock(&sensor->lock);
	error |= lis3dh_read_reg(sensor, LIS3DH_XAXIS_LSB, lsb_x);
	error |= lis3dh_read_reg(sensor, LIS3DH_XAXIS_MSB, msb_x);
	error |= lis3dh_read_reg(sensor, LIS3DH_YAXIS_LSB, lsb_y);
	error |= lis3dh_read_reg(sensor, LIS3DH_YAXIS_MSB, msb_y);
	error |= lis3dh_read_reg(sensor, LIS3DH_ZAXIS_LSB, lsb_z);
	error |= lis3dh_read_reg(sensor, LIS3DH_ZAXIS_MSB, msb_z);
	mutex_unlock(&sensor->lock);

	if(error)
		goto out;

	/**
	 * Store data read.
	 */

	*x = ((int) value_x) >> 4;
	*y = ((int) value_y) >> 4;
	*z = ((int) value_z) >> 4;

	return 0;

	out:
		return error;
}

#ifdef CONFIG_ST_LIS3DH_TEMP

static int lis3dh_read_temperature(struct lis3dh_dev *sensor, int *temp)
{
	int error = 0;
	char adc3_lsb = 0;
	char adc3_msb = 0;

	/**
	 * Accordingly to the specs, we need to read both registers..
	 * Useful data will be in the LIS3DH_OUT_ADC3_H one in 2-complement
	 */
	mutex_lock(&sensor->lock);
	error |= lis3dh_read_reg(sensor, LIS3DH_OUT_ADC3_L, &adc3_lsb);
	error |= lis3dh_read_reg(sensor, LIS3DH_OUT_ADC3_H, &adc3_msb);
	mutex_unlock(&sensor->lock);

	if(error)
		goto out;

	/**
	 * The scale is 1 bit/Celsius..
	 * Now, temp stores the temperature
	 * variation accordingly to a referece...
	 */
	*temp = (int) adc3_lsb;

	return 0;

	out:
		return error;
}


static void lis3dh_dev_poll_temp(struct input_polled_dev *dev)
{
	struct lis3dh_dev *sensor = input_get_drvdata(dev->input);
	int temp = 0;
	int error = 0;

	if(sensor->power_mode != LIS3DH_NORMAL)
		goto out;

	error = lis3dh_read_temperature(sensor, &temp);
	if(error)
	{
		dev_err(&sensor->client->dev, "error reading temperature: %d\n", error);
		goto out;
	}

	/* Report temperature data in milli-Kelvin */
	input_report_abs(sensor->idev_accel->input, REL_MISC, temp + 273150);
	input_sync(sensor->idev_temp->input);

	out:
		return;
}

#endif

static void lis3dh_dev_poll_accel(struct input_polled_dev *dev)
{
	struct lis3dh_dev *sensor = dev->private;
	int x, y, z;
	int accel_conversion;
	int error = 0;

	dev_dbg(&sensor->client->dev, "polling lis3dh\n");

	if(sensor->power_mode != LIS3DH_NORMAL)
	{
		dev_err(&sensor->client->dev, "lis3dh is in low power mode!\n");
		goto out;
	}

	error = lis3dh_read_accel(sensor, &x, &y, &z);
	if(error)
	{
		dev_err(&sensor->client->dev, "error reading acceleration: %d\n", error);
		goto out;
	}
	quanta_motion_sensors_adjust_position(sensor->platform, &x, &y, &z);

	dev_dbg(&sensor->client->dev, "Raw X: %04X\n", x);
	dev_dbg(&sensor->client->dev, "Raw Y: %04X\n", y);
	dev_dbg(&sensor->client->dev, "Raw Z: %04X\n", z);

	accel_conversion = lis3dh_get_accel_conversion(sensor);

	x *= accel_conversion;
	y *= accel_conversion;
	z *= accel_conversion;

	dev_dbg(&sensor->client->dev, "X: %dmg\n", x/1000);
	dev_dbg(&sensor->client->dev, "Y: %dmg\n", y/1000);
	dev_dbg(&sensor->client->dev, "Z: %dmg\n", z/1000);


	/* Report XYZ data in micro-G units */
	input_report_abs(sensor->idev_accel->input, ABS_X, x);
	input_report_abs(sensor->idev_accel->input, ABS_Y, y);
	input_report_abs(sensor->idev_accel->input, ABS_Z, z);
	input_sync(sensor->idev_accel->input);

	out:
		return;
}

static ssize_t lis3dh_sysfs_set_range_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	lis3dh_range_modes range_mode;
	int error = 0;
	unsigned long new_range = simple_strtoul(buf, NULL, 10);
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct lis3dh_dev *sensor = ipdev->private;

	switch(new_range)
	{
	case LIS3DH_2G:
	case LIS3DH_4G:
	case LIS3DH_8G:
	case LIS3DH_16G:
		range_mode = (lis3dh_range_modes) new_range;
		error = lis3dh_set_range(sensor, range_mode);
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

static ssize_t lis3dh_sysfs_get_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct lis3dh_dev *sensor = ipdev->private;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3DH sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sensor->range_mode);
}

static DEVICE_ATTR(range_mode, S_IWUSR | S_IRUGO, lis3dh_sysfs_get_range, lis3dh_sysfs_set_range_mode);

static struct attribute *lis3dh_attributes[] =
{
	&dev_attr_range_mode.attr,
	NULL
};

static const struct attribute_group lis3dh_attr_group =
{
	.name = "lis3dh",
	.attrs = lis3dh_attributes,
};


static int __devinit
lis3dh_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 client_id = 0;
	int error = 0;
	struct lis3dh_dev *this = (struct lis3dh_dev *) kcalloc(sizeof(struct lis3dh_dev), 1, GFP_KERNEL);
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
	error = lis3dh_read_reg(this, LIS3DH_CHIP_ID, &client_id);
	if(error)
	{
		dev_err(&(client->dev), "error: device does not exist\n");
		error = -ENODEV;
		goto err_no_dev;
	}

	if(client_id != LIS3DH_CHIP_ID_VALUE)
	{
		dev_err(&(client->dev), "error: device id 0x%X is not the expected (0x%x)\n", client_id, LIS3DH_CHIP_ID_VALUE);
		error = -ENODEV;
		goto err_no_dev;
	}

	/**
	 * Initial configuration:
	 * 4g, Normal Mode, FIFO bypass and no IRQs (polled device)
	 * Temperature enabled, ADC disabled
	 */
	error = 0;
	error |= lis3dh_set_range(this, LIS3DH_2G);
	error |= lis3dh_set_power(this, LIS3DH_NORMAL);
	error |= lis3dh_write_reg(this, LIS3DH_CTRL3, 0x00);
	error |= lis3dh_write_reg(this, LIS3DH_FIFO_CTRL, 0x00);
#ifdef CONFIG_ST_LIS3DH_TEMP
	error |= lis3dh_write_reg(this, LIS3DH_TEMP_CONFIG, 0xC0);
#endif

	if(error)
	{
		dev_err(&(client->dev), "error: error configuring the device\n");
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
	this->idev_accel->poll = lis3dh_dev_poll_accel;
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

	error = sysfs_create_group(&this->idev_accel->input->dev.kobj, &lis3dh_attr_group);
	if (error)
	{
		dev_err(&client->dev, "create device file failed!\n");
		error = -EINVAL;
		goto err_register_sys_fs;
	}

#ifdef CONFIG_ST_LIS3DH_TEMP

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
	 * Configure the temperature input device
	 */
	this->idev_temp->poll = lis3dh_dev_poll_temp;
	this->idev_temp->poll_interval = POLL_INTERVAL;
	this->idev_temp->poll_interval_min = POLL_INTERVAL_MIN;
	this->idev_temp->poll_interval_max = POLL_INTERVAL_MAX;
	this->idev_temp->input->name = "termometer";
	this->idev_temp->input->id.bustype = BUS_I2C;
	this->idev_temp->input->evbit[0] = BIT_MASK(EV_REL);
	this->idev_temp->private = this;

	// Max range: -40 to +85 Celsius: 8 bits => 1 bit/Celsius degree
	input_set_abs_params(this->idev_temp->input, REL_MISC, -40, +85, 0, 0);

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

#ifdef CONFIG_ST_LIS3DH_TEMP
	err_register_input_temp:
		input_free_polled_device(this->idev_temp);
	err_alloc_input_temp:
		sysfs_remove_group(&this->idev_accel->input->dev.kobj, &lis3dh_attr_group);
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

static int __devexit lis3dh_remove(struct i2c_client *client)
{
	struct lis3dh_dev *sensor = i2c_get_clientdata(client);

#ifdef CONFIG_ST_LIS3DH_TEMP
	input_unregister_polled_device(sensor->idev_temp);
	input_free_polled_device(sensor->idev_temp);
#endif
	sysfs_remove_group(&sensor->idev_accel->input->dev.kobj, &lis3dh_attr_group);
	input_unregister_polled_device(sensor->idev_accel);
	input_free_polled_device(sensor->idev_accel);
	hwmon_device_unregister(sensor->dev);
	mutex_destroy(sensor->lock);
	kfree(sensor);

	return 0;
}


#ifdef CONFIG_PM_SLEEP
static int lis3dh_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_dev *sensor = i2c_get_clientdata(client);

	int error = lis3dh_set_power(sensor, LIS3DH_POWER_DOWN);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into suspended mode: %d\n", error);
	}

	return error;
}

static int lis3dh_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_dev *sensor = i2c_get_clientdata(client);

	int error = lis3dh_set_power(sensor, LIS3DH_NORMAL);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into normal mode: %d\n", error);
	}

	return error;

}
#else
static int lis3dh_suspend(struct device *dev) { return 0; }
static int lis3dh_resume(struct device *dev) { return 0; }
#endif

static const struct dev_pm_ops lis3dh_pm_ops = {
		.suspend = lis3dh_suspend,
		.resume = lis3dh_resume,
};

static const struct i2c_device_id lis3dh_id[] = {
	{LIS3DH_DRV_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, lis3dh_id);

static struct i2c_driver lis3dh_driver = {
	.driver = {
		.name = LIS3DH_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &lis3dh_pm_ops,
	},
	.probe = lis3dh_probe,
	.remove = __devexit_p(lis3dh_remove),
	.id_table = lis3dh_id,
};

static int __init
lis3dh_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&lis3dh_driver);
	if (res < 0)
	{
		pr_err("%s: add lis3dh i2c driver failed\n", __FUNCTION__);
		return -ENODEV;
	}
	return res;
}

static void __exit
lis3dh_exit(void)
{
	i2c_del_driver(&lis3dh_driver);
}

MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("ST LIS3DH 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(lis3dh_init);
module_exit(lis3dh_exit);
