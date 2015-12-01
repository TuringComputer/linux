/*
 * qta-lis3mdl.c
 *
 *	Digital 16-bits 3-axis eCompass and termometer driver for
 *	ST Microelectronics LIS3MDL module.
 *
 *	Accordingly to Quanta's standard, eCompass measures are reported in micro-Gauss.
 *	Temperature measures are reported in milli-Kelvin.
 *
 *  Created on: 21/05/2014
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

#define LIS3MDL_DRV_NAME			"lis3mdl"
#define POLL_INTERVAL_MIN			20
#define POLL_INTERVAL_MAX			200
#define POLL_INTERVAL				100

/**
 * Identification register
 */
#define LIS3MDL_CHIP_ID				0x0F
#define LIS3MDL_CHIP_ID_VALUE		0x3D

/**
 * Chip's configuration registers
 */
#define LIS3MDL_CTRL_REG1			0x20
#define LIS3MDL_CTRL_REG2			0x21
#define LIS3MDL_CTRL_REG3			0x22
#define LIS3MDL_CTRL_REG4			0x23
#define LIS3MDL_CTRL_REG5			0x24

/**
 * Status registers
 */
#define LIS3MDL_STATUS_REG			0x27

/**
 * Data registers
 */
#define LIS3MDL_XAXIS_LSB			0x28
#define LIS3MDL_XAXIS_MSB			0x29
#define LIS3MDL_YAXIS_LSB			0x2A
#define LIS3MDL_YAXIS_MSB			0x2B
#define LIS3MDL_ZAXIS_LSB			0x2C
#define LIS3MDL_ZAXIS_MSB			0x2D
#define LIS3MDL_TEMP_LSB			0x2E
#define LIS3MDL_TEMP_MSB			0x2F

/**
 * Interrupt control registers
 */
#define LIS3MDL_INT_CFG				0x30
#define LIS3MDL_INT_SRC				0x31
#define LIS3MDL_INT_THRS_L			0x32
#define LIS3MDL_INT_THRS_H			0x33

/**
 * Range modes accordingly to the datasheet
 */
typedef enum {

	/**
	 * 6842 LSB = 1 Gauss
	 * 1 bit = 0.14615 mGauss
	 * 1 bit = 146 uGauss
	 */
	LIS3MDL_4GAUSS = 0x00,
	/**
	 * 3421 LSB = 1 Gauss
	 * 1 bit = 0.29231 mGauss
	 * 1 bit = 292 uGauss
	 */
	LIS3MDL_8GAUSS = 0x01,
	/**
	 * 2281 LSB = 1 Gauss
	 * 1 bit = 0.43840 mGauss
	 * 1 bit = 438 uGauss
	 */
	LIS3MDL_12GAUSS = 0x02,
	/**
	 * 1711 LSB = 1 Gauss
	 * 1 bit = 0.58445 mGauss
	 * 1 bit = 584 uGauss
	 */
	LIS3MDL_16GAUSS = 0x03

} lis3mdl_range_modes;

/**
 * Active modes accordingly to datasheet.
 */
typedef enum {
	/**
	 * Active mode.
	 */
	LIS3MDL_NORMAL = 0x00,
	/**
	 * Suspended mode.
	 */
	LIS3MDL_SUSPENDED = 0x23,
	/**
	 * Low Power mode.
	 * Currently it is not supported.
	 * Setting to Suspended Mode
	 */
	LIS3MDL_LOW_POWER = LIS3MDL_SUSPENDED,
} lis3mdl_power_modes;

struct lis3mdl_dev {
	struct i2c_client 					*client;
	struct input_polled_dev				*idev_ecompass;
#ifdef CONFIG_ST_LIS3MDL_TEMP
	struct input_polled_dev				*idev_temp;
#endif
	struct device 						*dev;
	lis3mdl_range_modes					range_mode;
	lis3mdl_power_modes					power_mode;
	struct mutex						lock;
	struct quanta_motion_sensors_platform_data 	*platform;
};

static unsigned int lis3mdl_get_conversion(struct lis3mdl_dev *sensor)
{

	if(sensor == NULL)
	{
		pr_err("%s: LIS3MDL sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	switch(sensor->range_mode)
	{
		case LIS3MDL_4GAUSS:
			/**
			 * 6842 LSB = 1 Gauss
			 * 1 bit = 0.14615 mGauss
			 * 1 bit = 146 uGauss
			 */
			return 146;
		case LIS3MDL_8GAUSS:
			/**
			 * 3421 LSB = 1 Gauss
			 * 1 bit = 0.29231 mGauss
			 * 1 bit = 292 uGauss
			 */
			return 292;
		case LIS3MDL_12GAUSS:
			/**
			 * 2281 LSB = 1 Gauss
			 * 1 bit = 0.43840 mGauss
			 * 1 bit = 438 uGauss
			 */
			return 438;
		case LIS3MDL_16GAUSS:
			/**
			 * 1711 LSB = 1 Gauss
			 * 1 bit = 0.58445 mGauss
			 * 1 bit = 584 uGauss
			 */
			return 584;
		default:
			/**
			 * This is an error.. should not happen
			 */
			pr_err("%s: Invalid range mode for LIS3MDL driver!", __FUNCTION__);
			BUG();
			return -EINVAL;
	}
}

static int lis3mdl_read_reg(struct lis3mdl_dev *sensor, u8 reg, u8 *value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3MDL sensor structure is null", __FUNCTION__);
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

static int lis3mdl_write_reg(struct lis3mdl_dev *sensor, u8 reg, u8 value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3MDL sensor structure is null", __FUNCTION__);
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

static int lis3mdl_read_values(struct lis3mdl_dev *sensor, int *x, int *y, int *z)
{
	int error = 0;
	u8 x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;

	if(sensor == NULL)
	{
		pr_err("%s: LIS3MDL sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	/**
	 * We must read the LSB before the MSB,
	 * as the datasheet recommends.
	 */
	mutex_lock(&sensor->lock);
	error |= lis3mdl_read_reg(sensor, LIS3MDL_XAXIS_LSB, &x_lsb);
	error |= lis3mdl_read_reg(sensor, LIS3MDL_XAXIS_MSB, &x_msb);
	error |= lis3mdl_read_reg(sensor, LIS3MDL_YAXIS_LSB, &y_lsb);
	error |= lis3mdl_read_reg(sensor, LIS3MDL_YAXIS_MSB, &y_msb);
	error |= lis3mdl_read_reg(sensor, LIS3MDL_ZAXIS_LSB, &z_lsb);
	error |= lis3mdl_read_reg(sensor, LIS3MDL_ZAXIS_MSB, &z_msb);
	mutex_unlock(&sensor->lock);

	if(error)
		goto out;

	/**
	 * 16 bits given in two ́2-complement representation:
	 * MSB register contain 8 most significant bits
	 * LSB register contain 8 least significant bits
	 */
	*x = (((int) x_msb << 8) & 0xFFFF0000) + (((int) x_lsb) & 0x0000FFFF);
	*y = (((int) y_msb << 8) & 0xFFFF0000) + (((int) y_lsb) & 0x0000FFFF);
	*z = (((int) z_msb << 8) & 0xFFFF0000) + (((int) z_lsb) & 0x0000FFFF);

	out:
		return error;
}

#ifdef CONFIG_ST_LIS3MDL_TEMP

static int lis3mdl_read_temperature(struct lis3mdl_dev *sensor, int *temp)
{
	u8 lsb = 0x00;
	u8 msb = 0x00;
	int temp_data = 0x00;
	int error = 0;

	error |= lis3mdl_read_reg(sensor, LIS3MDL_TEMP_LSB, &lsb);
	error |= lis3mdl_read_reg(sensor, LIS3MDL_TEMP_MSB, &msb);

	if(error)
		goto out;

	/**
	 * Temperature is given in 16-bit 2-complement
	 * 8 LSB = 1 Celsius
	 * 1 bit = 0.125 Kelvin = 125 milli-Kelvin
	 * 0x00 = 24 Celsius = 297150 milli-Kelvin
	 */
	temp_data = (((int) msb << 8) & 0xFFFF0000) + (((int) lsb) & 0x0000FFFF);
	*temp = (temp_data * 125) + 297150;

	out:
		return error;
}

#endif

static int lis3mdl_set_range(struct lis3mdl_dev *sensor, lis3mdl_range_modes mode)
{
	int value = 0x00;
	int error = 0x00;

	switch(mode)
	{
		case LIS3MDL_4GAUSS:
			value = 0x00;
			break;
		case LIS3MDL_8GAUSS:
			value = 0x20;
			break;
		case LIS3MDL_12GAUSS:
			value = 0x40;
			break;
		case LIS3MDL_16GAUSS:
			value = 0x60;
			break;
		default:
			return -EINVAL;
	}

	mutex_lock(&sensor->lock);
	error = lis3mdl_write_reg(sensor, LIS3MDL_CTRL_REG2, value);
	mutex_unlock(&sensor->lock);

	if(error)
		return error;

	sensor->range_mode = mode;
	return 0;
}

static int lis3mdl_set_power(struct lis3mdl_dev *sensor, lis3mdl_power_modes mode)
{
	return lis3mdl_write_reg(sensor, LIS3MDL_CTRL_REG3, mode);
}

#ifdef CONFIG_ST_LIS3MDL_TEMP

static void lis3mdl_dev_poll_temp(struct input_polled_dev *dev)
{
	struct lis3mdl_dev *sensor = input_get_drvdata(dev->input);
	int temp;
	int error = 0;

	if(sensor->power_mode != LIS3MDL_NORMAL)
		goto out;

	error = lis3mdl_read_temperature(sensor, &temp);
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

static void lis3mdl_dev_poll_values(struct input_polled_dev *dev)
{
	struct lis3mdl_dev *sensor = input_get_drvdata(dev->input);
	int x, y, z;
	int conversion;
	int error = 0;

	if(sensor->power_mode != LIS3MDL_NORMAL)
		goto out;

	error = lis3mdl_read_values(sensor, &x, &y, &z);
	if(error)
	{
		dev_err(&sensor->client->dev, "error reading eCompass values: %d\n", error);
		goto out;
	}
	quanta_motion_sensors_adjust_position(sensor->platform, &x, &y, &z);

	conversion = lis3mdl_get_conversion(sensor);

	/* Report XYZ data in micro-Gauss units */
	input_report_abs(sensor->idev_ecompass->input, ABS_X, (x * conversion));
	input_report_abs(sensor->idev_ecompass->input, ABS_Y, (y * conversion));
	input_report_abs(sensor->idev_ecompass->input, ABS_Z, (z * conversion));
	input_sync(sensor->idev_ecompass->input);

	out:
		return;
}

static ssize_t lis3mdl_sysfs_set_range_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	lis3mdl_range_modes range_mode;
	int error = 0;
	unsigned long new_range = simple_strtoul(buf, NULL, 10);
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct lis3mdl_dev *sensor = ipdev->private;

	switch(new_range)
	{
	case LIS3MDL_4GAUSS:
	case LIS3MDL_8GAUSS:
	case LIS3MDL_12GAUSS:
	case LIS3MDL_16GAUSS:
		range_mode = (lis3mdl_range_modes) new_range;
		error = lis3mdl_set_range(sensor, range_mode);
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

static ssize_t lis3mdl_sysfs_get_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct lis3mdl_dev *sensor = ipdev->private;

	if(sensor == NULL)
	{
		pr_err("%s: lis3mdl sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sensor->range_mode);
}

static DEVICE_ATTR(range_mode, S_IWUSR | S_IRUGO, lis3mdl_sysfs_get_range, lis3mdl_sysfs_set_range_mode);

static struct attribute *lis3mdl_attributes[] =
{
	&dev_attr_range_mode.attr,
	NULL
};

static const struct attribute_group lis3mdl_attr_group =
{
	.name = "lis3mdl",
	.attrs = lis3mdl_attributes,
};

static int __devinit
lis3mdl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 client_id = 0;
	int error = 0;
	struct lis3mdl_dev *this = (struct lis3mdl_dev *) kcalloc(sizeof(struct lis3mdl_dev), 1, GFP_KERNEL);
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
	error = lis3mdl_read_reg(this, LIS3MDL_CHIP_ID, &client_id);
	if(error)
	{
		dev_err(&(client->dev), "error: device does not exist\n");
		error = -ENODEV;
		goto err_no_dev;
	}

	if(client_id != LIS3MDL_CHIP_ID_VALUE)
	{
		dev_err(&(client->dev), "error: device id 0x%X is not the expected (0x%x)\n", client_id, LIS3MDL_CHIP_ID_VALUE);
		error = -ENODEV;
		goto err_no_dev;
	}

	/**
	 * Initial configuration:
	 * 4gauss, Normal Mode and no IRQs (polled device)
	 */
	error = 0;
	error |= lis3mdl_set_range(this, LIS3MDL_4GAUSS);
	error |= lis3mdl_set_power(this, LIS3MDL_NORMAL);
	error |= lis3mdl_write_reg(this, LIS3MDL_INT_CFG, 0x00);

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
	this->idev_ecompass = input_allocate_polled_device();
	if (IS_ERR(this->idev_ecompass))
	{
		dev_err(&(client->dev), "error: out of memory for eCompass input device\n");
		error = -ENOMEM;
		goto err_alloc_input_accel;
	}

	/**
	 * Configure the input device
	 */
	this->idev_ecompass->poll = lis3mdl_dev_poll_values;
	this->idev_ecompass->poll_interval = POLL_INTERVAL;
	this->idev_ecompass->poll_interval_min = POLL_INTERVAL_MIN;
	this->idev_ecompass->poll_interval_max = POLL_INTERVAL_MAX;
	this->idev_ecompass->input->name = "ecompass";
	this->idev_ecompass->input->id.bustype = BUS_I2C;
	this->idev_ecompass->input->evbit[0] = BIT_MASK(EV_ABS);
	this->idev_ecompass->private = this;

	/**
	 * The eCompass values will be given in multiples of
	 * micro-Gauss, depending on the resolution.
	 * The current resolution is given by the range_mode attribute.
	 */
	input_set_abs_params(this->idev_ecompass->input, ABS_X, -16000000, 16000000, 0, 0);
	input_set_abs_params(this->idev_ecompass->input, ABS_Y, -16000000, 16000000, 0, 0);
	input_set_abs_params(this->idev_ecompass->input, ABS_Z, -16000000, 16000000, 0, 0);

	error = input_register_polled_device(this->idev_ecompass);

	if(error)
	{
		dev_err(&(client->dev), "error: cannot register the accelerometer input device\n");
		error = -ENOMEM;
		goto err_register_input_accel;
	}

	error = sysfs_create_group(&this->idev_ecompass->input->dev.kobj, &lis3mdl_attr_group);
	if (error)
	{
		dev_err(&client->dev, "create device file failed!\n");
		error = -EINVAL;
		goto err_register_sys_fs;
	}

#ifdef CONFIG_ST_LIS3MDL_TEMP

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
	this->idev_temp->poll = lis3mdl_dev_poll_temp;
	this->idev_temp->poll_interval = POLL_INTERVAL;
	this->idev_temp->poll_interval_min = POLL_INTERVAL_MIN;
	this->idev_temp->poll_interval_max = POLL_INTERVAL_MAX;
	this->idev_temp->input->name = "termometer";
	this->idev_temp->input->id.bustype = BUS_I2C;
	this->idev_temp->input->evbit[0] = BIT_MASK(EV_ABS);

	/**
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

#ifdef CONFIG_ST_LIS3MDL_TEMP
	err_register_input_temp:
		input_free_polled_device(this->idev_temp);
	err_alloc_input_temp:
		sysfs_remove_group(&this->idev_ecompass->input->dev.kobj, &lis3mdl_attr_group);
#endif
	err_register_sys_fs:
		input_unregister_polled_device(this->idev_ecompass);
	err_register_input_accel:
			input_free_polled_device(this->idev_ecompass);
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

static int __devexit lis3mdl_remove(struct i2c_client *client)
{
	struct lis3mdl_dev *sensor = i2c_get_clientdata(client);

#ifdef CONFIG_ST_LIS3MDL_TEMP
	input_unregister_polled_device(sensor->idev_temp);
	input_free_polled_device(sensor->idev_temp);
#endif
	sysfs_remove_group(&sensor->idev_ecompass->input->dev.kobj, &lis3mdl_attr_group);
	input_unregister_polled_device(sensor->idev_ecompass);
	input_free_polled_device(sensor->idev_ecompass);
	hwmon_device_unregister(sensor->dev);
	mutex_destroy(sensor->lock);
	kfree(sensor);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lis3mdl_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3mdl_dev *sensor = i2c_get_clientdata(client);

	int error = lis3mdl_set_power(sensor, LIS3MDL_SUSPENDED);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into suspended mode: %d\n", error);
	}

	return error;
}

static int lis3mdl_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3mdl_dev *sensor = i2c_get_clientdata(client);

	int error = lis3mdl_set_power(sensor, LIS3MDL_NORMAL);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into normal mode: %d\n", error);
	}

	return error;

}
#else
static int lis3mdl_suspend(struct device *dev) { return 0; }
static int lis3mdl_resume(struct device *dev) { return 0; }
#endif

static const struct dev_pm_ops lis3mdl_pm_ops = {
		.suspend = lis3mdl_suspend,
		.resume = lis3mdl_resume,
};

static const struct i2c_device_id lis3mdl_id[] = {
	{LIS3MDL_DRV_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, lis3mdl_id);

static struct i2c_driver lis3mdl_driver = {
	.driver = {
		.name = LIS3MDL_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &lis3mdl_pm_ops,
	},
	.probe = lis3mdl_probe,
	.remove = __devexit_p(lis3mdl_remove),
	.id_table = lis3mdl_id,
};

static int __init
lis3mdl_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&lis3mdl_driver);
	if (res < 0)
	{
		pr_err("%s: add lis3mdl i2c driver failed\n", __FUNCTION__);
		return -ENODEV;
	}
	return res;
}

static void __exit
lis3mdl_exit(void)
{
	i2c_del_driver(&lis3mdl_driver);
}

MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("ST LIS3MDL 3-Axis eCompass Sensor driver");
MODULE_LICENSE("GPL");

module_init(lis3mdl_init);
module_exit(lis3mdl_exit);
