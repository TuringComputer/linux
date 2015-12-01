/*
 * qta_l3g4200d.c
 *
 *	Digital 16-bits 3-axis Gyroscope and termometer driver for
 *	ST Microelectronics L3G4200D module.
 *
 *	Accordingly to Quanta's standard, Gyroscope measures are
 *	reported in micro-degrees per second (udps).
 *
 *	Positive values are counterclockwise movements.
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

#define L3G4200D_DRV_NAME			"l3g4200d"
#define POLL_INTERVAL_MIN			20
#define POLL_INTERVAL_MAX			200
#define POLL_INTERVAL				100

/**
 * Identification register
 */
#define L3G4200D_CHIP_ID			0x0F
#define L3G4200D_CHIP_ID_VALUE		0xD3

/**
 * Chip's configuration registers
 */
#define L3G4200D_CTRL_REG1			0x20
#define L3G4200D_CTRL_REG2			0x21
#define L3G4200D_CTRL_REG3			0x22
#define L3G4200D_CTRL_REG4			0x23
#define L3G4200D_CTRL_REG5			0x24
#define L3G4200D_REFERENCE_REG		0x25
#define L3G4200D_FIFOCTRL_REG		0x2E
#define L3G4200D_FIFO_SRC_REG		0x2F

/**
 * Status registers
 */
#define L3G4200D_STATUS_REG			0x27

/**
 * Data registers
 */
#define L3G4200D_XAXIS_LSB			0x28
#define L3G4200D_XAXIS_MSB			0x29
#define L3G4200D_YAXIS_LSB			0x2A
#define L3G4200D_YAXIS_MSB			0x2B
#define L3G4200D_ZAXIS_LSB			0x2C
#define L3G4200D_ZAXIS_MSB			0x2D
#define L3G4200D_TEMP				0x26

/**
 * Interrupt control registers
 */
#define L3G4200D_INT_CFG			0x30
#define L3G4200D_INT_SRC			0x31
#define L3G4200D_INT_THRS_XL		0x32
#define L3G4200D_INT_THRS_XH		0x33
#define L3G4200D_INT_THRS_YL		0x34
#define L3G4200D_INT_THRS_YH		0x35
#define L3G4200D_INT_THRS_ZL		0x36
#define L3G4200D_INT_THRS_ZH		0x37
#define L3G4200D_INT_DURATION		0x38

/**
 * Range modes accordingly to the datasheet
 */
typedef enum {

	/**
	 * 1 bit = 8.75mdps
	 * Range = -250dps to + 250dps
	 */
	L3G4200D_250DPS = 0x00,
	/**
	 * 1 bit = 17.50mdps
	 * Range = -500dps to + 500dps
	 */
	L3G4200D_500DPS = 0x01,
	/**
	 * 1 bit = 70mdps
	 * Range = -2000dps to + 2000dps
	 */
	L3G4200D_2000DPS = 0x02,

} l3g4200d_range_modes;

/**
 * Active modes accordingly to datasheet.
 */
typedef enum {
	/**
	 * Active mode.
	 */
	L3G4200D_NORMAL = 0x0F,
	/**
	 * Suspended mode.
	 */
	L3G4200D_SUSPENDED = 0x07,
	/**
	 * Low Power mode.
	 * Currently it is not supported.
	 * Setting to Suspended Mode
	 */
	L3G4200D_LOW_POWER = L3G4200D_SUSPENDED,
} l3g4200d_power_modes;

struct l3g4200d_dev {
	struct i2c_client 					*client;
	struct input_polled_dev				*idev_gyro;
#ifdef CONFIG_ST_L3G4200D_TEMP
	struct input_polled_dev				*idev_temp;
#endif
	struct device 						*dev;
	l3g4200d_range_modes				range_mode;
	l3g4200d_power_modes				power_mode;
	struct mutex						lock;
	struct quanta_motion_sensors_platform_data 	*platform;
};

static unsigned int l3g4200d_get_conversion(struct l3g4200d_dev *sensor)
{

	if(sensor == NULL)
	{
		pr_err("%s: L3G4200D sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	switch(sensor->range_mode)
	{
		case L3G4200D_250DPS:
			/**
			 * 1 bit = 8.75mdps = 8750 udps
			 * Range = -250dps to + 250dps
			 */
			return 8750;
		case L3G4200D_500DPS:
			/**
			 * 1 bit = 17.50mdps = 17500 udps
			 * Range = -500dps to + 500dps
			 */
			return 17500;
		case L3G4200D_2000DPS:
			/**
			 * 1 bit = 70mdps = 70000 udps
			 * Range = -2000dps to + 2000dps
			 */
			return 70000;
		default:
			/**
			 * This is an error.. should not happen
			 */
			pr_err("%s: Invalid range mode for L3G4200D driver!", __FUNCTION__);
			BUG();
			return -EINVAL;
	}
}

static int l3g4200d_read_reg(struct l3g4200d_dev *sensor, u8 reg, u8 *value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: L3G4200D sensor structure is null", __FUNCTION__);
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

static int l3g4200d_write_reg(struct l3g4200d_dev *sensor, u8 reg, u8 value)
{
	int ret;

	if(sensor == NULL)
	{
		pr_err("%s: L3G4200D sensor structure is null", __FUNCTION__);
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

static int l3g4200d_read_values(struct l3g4200d_dev *sensor, int *x, int *y, int *z)
{
	int error = 0;
	u8 x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;

	if(sensor == NULL)
	{
		pr_err("%s: L3G4200D sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	/**
	 * We must read the LSB before the MSB,
	 * as the datasheet recommends.
	 */
	mutex_lock(&sensor->lock);
	error |= l3g4200d_read_reg(sensor, L3G4200D_XAXIS_LSB, &x_lsb);
	error |= l3g4200d_read_reg(sensor, L3G4200D_XAXIS_MSB, &x_msb);
	error |= l3g4200d_read_reg(sensor, L3G4200D_YAXIS_LSB, &y_lsb);
	error |= l3g4200d_read_reg(sensor, L3G4200D_YAXIS_MSB, &y_msb);
	error |= l3g4200d_read_reg(sensor, L3G4200D_ZAXIS_LSB, &z_lsb);
	error |= l3g4200d_read_reg(sensor, L3G4200D_ZAXIS_MSB, &z_msb);
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

#ifdef CONFIG_ST_L3G4200D_TEMP

static int l3g4200d_read_temperature(struct l3g4200d_dev *sensor, int *temp)
{
	u8 temp_data = 0x00;
	int error = 0;

	error = l3g4200d_read_reg(sensor, L3G4200D_TEMP, &temp_data);

	if(error)
		goto out;

	/**
	 * Temperature is given in 8-bit
	 * 1 LSB = 1 Celsius
	 * 1 bit = 1 Kelvin = 1000 milli-Kelvin
	 * 0x00 = 24 Celsius = 297150 milli-Kelvin
	 */
	*temp = (temp_data * 1000) + 297150;

	out:
		return error;
}

#endif

static int l3g4200d_set_range(struct l3g4200d_dev *sensor, l3g4200d_range_modes mode)
{
	int value = 0x00;
	int error = 0x00;

	switch(mode)
	{
		case L3G4200D_250DPS:
			value = 0x00;
			break;
		case L3G4200D_500DPS:
			value = 0x10;
			break;
		case L3G4200D_2000DPS:
			value = 0x20;
			break;
		default:
			return -EINVAL;
	}

	mutex_lock(&sensor->lock);
	error = l3g4200d_write_reg(sensor, L3G4200D_CTRL_REG4, value);
	mutex_unlock(&sensor->lock);

	if(error)
		return error;

	sensor->range_mode = mode;
	return 0;
}

static int l3g4200d_set_power(struct l3g4200d_dev *sensor, l3g4200d_power_modes mode)
{
	return l3g4200d_write_reg(sensor, L3G4200D_CTRL_REG1, mode);
}

#ifdef CONFIG_ST_L3G4200D_TEMP

static void l3g4200d_dev_poll_temp(struct input_polled_dev *dev)
{
	struct l3g4200d_dev *sensor = input_get_drvdata(dev->input);
	int temp;
	int error = 0;

	if(sensor->power_mode != L3G4200D_NORMAL)
		goto out;

	error = l3g4200d_read_temperature(sensor, &temp);
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

static void l3g4200d_dev_poll_values(struct input_polled_dev *dev)
{
	struct l3g4200d_dev *sensor = input_get_drvdata(dev->input);
	int x, y, z;
	int conversion;
	int error = 0;

	if(sensor->power_mode != L3G4200D_NORMAL)
		goto out;

	error = l3g4200d_read_values(sensor, &x, &y, &z);
	if(error)
	{
		dev_err(&sensor->client->dev, "error reading gyroscope values: %d\n", error);
		goto out;
	}
	quanta_motion_sensors_adjust_position(sensor->platform, &x, &y, &z);

	conversion = l3g4200d_get_conversion(sensor);

	/* Report XYZ data in micro-Gauss units */
	input_report_abs(sensor->idev_gyro->input, ABS_X, (x * conversion));
	input_report_abs(sensor->idev_gyro->input, ABS_Y, (y * conversion));
	input_report_abs(sensor->idev_gyro->input, ABS_Z, (z * conversion));
	input_sync(sensor->idev_gyro->input);

	out:
		return;
}

static ssize_t l3g4200d_sysfs_set_range_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	l3g4200d_range_modes range_mode;
	int error = 0;
	unsigned long new_range = simple_strtoul(buf, NULL, 10);
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct l3g4200d_dev *sensor = ipdev->private;

	switch(new_range)
	{
	case L3G4200D_250DPS:
	case L3G4200D_500DPS:
	case L3G4200D_2000DPS:
		range_mode = (l3g4200d_range_modes) new_range;
		error = l3g4200d_set_range(sensor, range_mode);
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

static ssize_t l3g4200d_sysfs_get_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *idev = to_input_dev(dev);
	struct input_polled_dev *ipdev = input_get_drvdata(idev);
	struct l3g4200d_dev *sensor = ipdev->private;

	if(sensor == NULL)
	{
		pr_err("%s: L3G4200D sensor structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", sensor->range_mode);
}

static DEVICE_ATTR(range_mode, S_IWUSR | S_IRUGO, l3g4200d_sysfs_get_range, l3g4200d_sysfs_set_range_mode);

static struct attribute *l3g4200d_attributes[] =
{
	&dev_attr_range_mode.attr,
	NULL
};

static const struct attribute_group l3g4200d_attr_group =
{
	.name = "l3g4200d",
	.attrs = l3g4200d_attributes,
};

static int __devinit
l3g4200d_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 client_id = 0;
	int error = 0;
	struct l3g4200d_dev *this = (struct l3g4200d_dev *) kcalloc(sizeof(struct l3g4200d_dev), 1, GFP_KERNEL);
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
	error = l3g4200d_read_reg(this, L3G4200D_CHIP_ID, &client_id);
	if(error)
	{
		dev_err(&(client->dev), "error: device does not exist\n");
		error = -ENODEV;
		goto err_no_dev;
	}

	if(client_id != L3G4200D_CHIP_ID_VALUE)
	{
		dev_err(&(client->dev), "error: device id 0x%X is not the expected (0x%x)\n", client_id, L3G4200D_CHIP_ID_VALUE);
		error = -ENODEV;
		goto err_no_dev;
	}

	/**
	 * Initial configuration:
	 * 250DPS, Normal Mode and no IRQs (polled device)
	 */
	error = 0;
	error |= l3g4200d_set_range(this, L3G4200D_250DPS);
	error |= l3g4200d_set_power(this, L3G4200D_NORMAL);
	error |= l3g4200d_write_reg(this, L3G4200D_INT_CFG, 0x00);

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
	this->idev_gyro = input_allocate_polled_device();
	if (IS_ERR(this->idev_gyro))
	{
		dev_err(&(client->dev), "error: out of memory for eCompass input device\n");
		error = -ENOMEM;
		goto err_alloc_input_accel;
	}

	/**
	 * Configure the input device
	 */
	this->idev_gyro->poll = l3g4200d_dev_poll_values;
	this->idev_gyro->poll_interval = POLL_INTERVAL;
	this->idev_gyro->poll_interval_min = POLL_INTERVAL_MIN;
	this->idev_gyro->poll_interval_max = POLL_INTERVAL_MAX;
	this->idev_gyro->input->name = "gyroscope";
	this->idev_gyro->input->id.bustype = BUS_I2C;
	this->idev_gyro->input->evbit[0] = BIT_MASK(EV_ABS);
	this->idev_gyro->private = this;

	/**
	 * The eCompass values will be given in multiples of
	 * uDPS, depending on the resolution.
	 * The current resolution is given by the range_mode attribute.
	 */
	input_set_abs_params(this->idev_gyro->input, ABS_X, -2000000000, 2000000000, 0, 0);
	input_set_abs_params(this->idev_gyro->input, ABS_Y, -2000000000, 2000000000, 0, 0);
	input_set_abs_params(this->idev_gyro->input, ABS_Z, -2000000000, 2000000000, 0, 0);

	error = input_register_polled_device(this->idev_gyro);

	if(error)
	{
		dev_err(&(client->dev), "error: cannot register the accelerometer input device\n");
		error = -ENOMEM;
		goto err_register_input_accel;
	}

	error = sysfs_create_group(&this->idev_gyro->input->dev.kobj, &l3g4200d_attr_group);
	if (error)
	{
		dev_err(&client->dev, "create device file failed!\n");
		error = -EINVAL;
		goto err_register_sys_fs;
	}

#ifdef CONFIG_ST_L3G4200D_TEMP

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
	this->idev_temp->poll = l3g4200d_dev_poll_temp;
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

#ifdef CONFIG_ST_L3G4200D_TEMP
	err_register_input_temp:
		input_free_polled_device(this->idev_temp);
	err_alloc_input_temp:
		sysfs_remove_group(&this->idev_gyro->input->dev.kobj, &l3g4200d_attr_group);
#endif
	err_register_sys_fs:
		input_unregister_polled_device(this->idev_gyro);
	err_register_input_accel:
			input_free_polled_device(this->idev_gyro);
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

static int __devexit l3g4200d_remove(struct i2c_client *client)
{
	struct l3g4200d_dev *sensor = i2c_get_clientdata(client);

#ifdef CONFIG_ST_L3G4200D_TEMP
	input_unregister_polled_device(sensor->idev_temp);
	input_free_polled_device(sensor->idev_temp);
#endif
	sysfs_remove_group(&sensor->idev_gyro->input->dev.kobj, &l3g4200d_attr_group);
	input_unregister_polled_device(sensor->idev_gyro);
	input_free_polled_device(sensor->idev_gyro);
	hwmon_device_unregister(sensor->dev);
	mutex_destroy(sensor->lock);
	kfree(sensor);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int l3g4200d_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g4200d_dev *sensor = i2c_get_clientdata(client);

	int error = l3g4200d_set_power(sensor, L3G4200D_SUSPENDED);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into suspended mode: %d\n", error);
	}

	return error;
}

static int l3g4200d_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g4200d_dev *sensor = i2c_get_clientdata(client);

	int error = l3g4200d_set_power(sensor, L3G4200D_NORMAL);

	if(error)
	{
		dev_err(&sensor->client->dev, "error: cannot put device into normal mode: %d\n", error);
	}

	return error;

}
#else
static int l3g4200d_suspend(struct device *dev) { return 0; }
static int l3g4200d_resume(struct device *dev) { return 0; }
#endif

static const struct dev_pm_ops l3g4200d_pm_ops = {
		.suspend = l3g4200d_suspend,
		.resume = l3g4200d_resume,
};

static const struct i2c_device_id l3g4200d_id[] = {
	{L3G4200D_DRV_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, l3g4200d_id);

static struct i2c_driver l3g4200d_driver = {
	.driver = {
		.name = L3G4200D_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &l3g4200d_pm_ops,
	},
	.probe = l3g4200d_probe,
	.remove = __devexit_p(l3g4200d_remove),
	.id_table = l3g4200d_id,
};

static int __init
l3g4200d_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&l3g4200d_driver);
	if (res < 0)
	{
		pr_err("%s: add l3g4200d i2c driver failed\n", __FUNCTION__);
		return -ENODEV;
	}
	return res;
}

static void __exit
l3g4200d_exit(void)
{
	i2c_del_driver(&l3g4200d_driver);
}

MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("ST L3G4200D 3-Axis Gyroscope Sensor driver");
MODULE_LICENSE("GPL");

module_init(l3g4200d_init);
module_exit(l3g4200d_exit);
