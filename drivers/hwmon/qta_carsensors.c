/*
 * qta_carsensors.c
 *
 *	This is the Quanta's Car Sensors device driver.
 *	It is used to read car sensors data through its
 *	Auxiliar CPU I2C protocol.
 *
 *	The available fields are:
 *
 *	CAN interface:
 *	a) Speed
 *	b) External temperature
 *	c) Motor temperature
 *	d) Motor speed
 *	e) Fuel level
 *	d) Distance traveled
 *	f) Lifetime distance traveled
 *	g) Motor hours
 *	h) Fuel used
 *	i) Oil temperature
 *	j) Oil level
 *	k) Axle weight
 *	l) Break switch
 *	m) Clutch switch
 *
 *	IO interface:
 *	n) Ignition switch
 *	o) Cluster lamp switch
 *	p) Parking switch
 *
 *	Control interface:
 *	q) Enabled
 *	r) Poll Interval
 *	s) Filters
 *	t) Baud Rate
 *	u) Mode
 *
 *  Created on: 27/11/2013
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
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/platform_device.h>
#include <linux/mfd/quanta-indash-mfd.h>
#include <linux/workqueue.h>

#define CARSENSORS_DRV_NAME					"indash-carsensors"
#define CARSENSORS_DATA_SUBDIR				"data"
#define CARSENSORS_CONTROL_SUBDIR			"control"
#define CARSENSORS_POLL_INTERVAL			100

#define INDASH_FLAGS_SPEED					(1 << 0)
#define INDASH_FLAGS_EXT_TEMPERATURE		(1 << 1)
#define INDASH_FLAGS_MOTOR_TEMPERATURE		(1 << 2)
#define INDASH_FLAGS_MOTOR_SPEED			(1 << 3)
#define INDASH_FLAGS_FUEL_LEVEL				(1 << 4)
#define INDASH_FLAGS_DIST_TRAVELED			(1 << 5)
#define INDASH_FLAGS_LT_DIST_TRAVELED		(1 << 6)
#define INDASH_FLAGS_MOTOR_HOURS			(1 << 7)
#define INDASH_FLAGS_FUEL_USED				(1 << 8)
#define INDASH_FLAGS_OIL_TEMPERATURE		(1 << 9)
#define INDASH_FLAGS_OIL_LEVEL				(1 << 10)
#define INDASH_FLAGS_AXLE_WEIGHT			(1 << 11)
#define INDASH_FLAGS_BREAK_SWITCH			(1 << 12)
#define INDASH_FLAGS_CLUTCH_SWITCH			(1 << 13)

struct carsensors_data {
	u16 	speed;
	s8 		ext_temperature;
	u8 		motor_temperature;
	u16 	motor_speed;
	u8		fuel_level;
	u32		distance_traveled;
	u32		lifetime_distance_traveled;
	u32		motor_hours;
	u32		fuel_used;
	u8		oil_temperature;
	u8		oil_level;
	u16		axle_weight;
	u8		break_switch;
	u8		clutch_switch;
	u8		ignition_switch;
	u8		cluster_lamp_switch;
	u8		parking_switch;
	u16		flags;
};

struct carsensors_dev {
	struct delayed_work			work;
	struct mutex				lock;
	struct device 				*dev;
	struct indash_mfd_handler 	handler;
	struct carsensors_data		data;
	struct workqueue_struct *   wq;
	unsigned int				poll_interval;
	unsigned int				enabled;
	unsigned int				baud_rate;
	unsigned char				mode;

};

/**
 * Just a helper macro to create the sysfs attribute's read functions
 */
#define SYSFS_GET_FUNCTION(field, format) 																	\
static ssize_t carsensors_sysfs_get_##field(struct device *dev, struct device_attribute *attr, char *buf) 	\
{																											\
	struct platform_device *pdev = to_platform_device(dev);													\
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);												\
	ssize_t count = 0;																						\
																											\
	if(sensor == NULL)																						\
	{																										\
		pr_err("%s: CarSensors structure is null", __FUNCTION__);											\
		BUG();																								\
		return -ENODEV;																						\
	}																										\
																											\
	mutex_lock(&sensor->lock);																				\
	count = sprintf(buf, format, sensor->data.field);														\
	mutex_unlock(&sensor->lock);																			\
	return count;																							\
}																											\

SYSFS_GET_FUNCTION(speed, "%u\n")
SYSFS_GET_FUNCTION(ext_temperature, "%d\n")
SYSFS_GET_FUNCTION(motor_temperature, "%u\n")
SYSFS_GET_FUNCTION(motor_speed, "%u\n")
SYSFS_GET_FUNCTION(fuel_level, "%u\n")
SYSFS_GET_FUNCTION(distance_traveled, "%u\n")
SYSFS_GET_FUNCTION(lifetime_distance_traveled, "%u\n")
SYSFS_GET_FUNCTION(motor_hours, "%u\n")
SYSFS_GET_FUNCTION(fuel_used, "%u\n")
SYSFS_GET_FUNCTION(oil_temperature, "%u\n")
SYSFS_GET_FUNCTION(oil_level, "%u\n")
SYSFS_GET_FUNCTION(axle_weight, "%u\n")
SYSFS_GET_FUNCTION(break_switch, "%u\n")
SYSFS_GET_FUNCTION(clutch_switch, "%u\n")
SYSFS_GET_FUNCTION(ignition_switch, "%u\n")
SYSFS_GET_FUNCTION(cluster_lamp_switch, "%u\n")
SYSFS_GET_FUNCTION(parking_switch, "%u\n")
SYSFS_GET_FUNCTION(flags, "%u\n")

#define SYSFS_GET_ATTRIBUTE(field) DEVICE_ATTR(field, S_IRUGO, carsensors_sysfs_get_##field, NULL)
static SYSFS_GET_ATTRIBUTE(speed);
static SYSFS_GET_ATTRIBUTE(ext_temperature);
static SYSFS_GET_ATTRIBUTE(motor_temperature);
static SYSFS_GET_ATTRIBUTE(motor_speed);
static SYSFS_GET_ATTRIBUTE(fuel_level);
static SYSFS_GET_ATTRIBUTE(distance_traveled);
static SYSFS_GET_ATTRIBUTE(lifetime_distance_traveled);
static SYSFS_GET_ATTRIBUTE(motor_hours);
static SYSFS_GET_ATTRIBUTE(fuel_used);
static SYSFS_GET_ATTRIBUTE(oil_temperature);
static SYSFS_GET_ATTRIBUTE(oil_level);
static SYSFS_GET_ATTRIBUTE(axle_weight);
static SYSFS_GET_ATTRIBUTE(break_switch);
static SYSFS_GET_ATTRIBUTE(clutch_switch);
static SYSFS_GET_ATTRIBUTE(ignition_switch);
static SYSFS_GET_ATTRIBUTE(cluster_lamp_switch);
static SYSFS_GET_ATTRIBUTE(parking_switch);
static SYSFS_GET_ATTRIBUTE(flags);

/**
 * Another macro to generate the dev_attr_<name> for readable field
 */
#define SYSFS_ATTRIBUTE_NAME(field) dev_attr_##field

/**
 * Sets the task delay (polling delay) for getting new carsensors data.
 */
static ssize_t carsensors_sysfs_set_poll_interval(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	unsigned long new_delay = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	new_delay = simple_strtoul(buf, NULL, 10);

	if(new_delay == 0)
	{
		dev_err(sensor->dev, "polling interval must be greater than 0 ms.\n");
		return -EINVAL;
	}

	sensor->poll_interval = new_delay;

	return count;
}

/**
 * Gets the task delay (polling delay) for getting new carsensors data.
 */
static ssize_t carsensors_sysfs_get_poll_interval(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	ssize_t count = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	count = sprintf(buf, "%d\n", sensor->poll_interval);

	return count;
}

/**
 * Sets if the polling work is enabled.
 */
static ssize_t carsensors_sysfs_set_enabled(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	unsigned long enabled = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	enabled = simple_strtoul(buf, NULL, 10);

	if(enabled && !sensor->enabled)
	{
		dev_info(sensor->dev, "enabling the carsensors data polling\n");
		queue_delayed_work(sensor->wq, &sensor->work, 0);
		sensor->enabled = 0x01;
		return count;
	}

	if(!enabled && sensor->enabled)
	{
		dev_info(sensor->dev, "disabling the carsensors data polling\n");
		cancel_delayed_work_sync(&sensor->work);
		sensor->enabled = 0x00;
		return count;
	}

	return count;
}

/**
 * Gets if the polling work is enabled.
 */
static ssize_t carsensors_sysfs_get_enabled(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	ssize_t count = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	count = sprintf(buf, "%d\n", sensor->enabled);

	return count;
}

/**
 * Updates the CAN filters on the Auxiliar CPU
 */
static ssize_t carsensors_sysfs_set_filters(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int error = 0;
	int i = 0;
	char *newBuf = NULL;
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	*newBuf = (char *) kzalloc(count, GFP_KERNEL);
	if (IS_ERR(newBuf))
	{
		dev_err(sensor->dev, "Unsufficient memory to create the filters buffer.\n");
		error = -ENOMEM;
		goto err_no_mem;
	}

	// buffer is read from the LSB to the MSB... we need to write each int value in it in the reverse order
	for(i = 0; i < count; i+=4)
	{
		newBuf[i + 0] = buf[i + 3];
		newBuf[i + 1] = buf[i + 2];
		newBuf[i + 2] = buf[i + 1];
		newBuf[i + 3] = buf[i + 0];
	}

	indash_mfd_lock();
	error = indash_mfd_canfilters_cmd(newBuf);
	indash_mfd_unlock();


	if(error)
	{
		dev_err(sensor->dev, "Error while updating the CAN filters settings: %d\n", error);
		goto err_cmd;
	}

	kfree(newBuf);
	return count;

	err_cmd:
		kfree(newBuf);
	err_no_mem:
		return error;
}

/**
 * Sets the CAN baud rate
 */
static ssize_t carsensors_sysfs_set_baud_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	unsigned int baud_rate = 0;
	int error = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	baud_rate = simple_strtoul(buf, NULL, 10);

	indash_mfd_lock();
	error = indash_mfd_canbaud_cmd(baud_rate);
	indash_mfd_unlock();

	if(error)
	{
		dev_err(sensor->dev, "Error while updating the CAN baud rate: %d\n", error);
		return error;
	}

	return count;
}

/**
 * Gets the CAN baud rate.
 */
static ssize_t carsensors_sysfs_get_baud_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	ssize_t count = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	count = sprintf(buf, "%d\n", sensor->baud_rate);

	return count;
}

/**
 * Sets the CAN operating mode
 */
static ssize_t carsensors_sysfs_set_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	enum indash_mfd_can_modes mode = 0;
	int error = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	mode = (enum indash_mfd_can_modes) simple_strtoul(buf, NULL, 10);

	indash_mfd_lock();
	error = indash_mfd_canmode_cmd(mode);
	indash_mfd_unlock();

	if(error)
	{
		dev_err(sensor->dev, "Error while updating the CAN operating mode: %d\n", error);
		return error;
	}

	return count;
}

/**
 * Gets the CAN operating mode.
 */
static ssize_t carsensors_sysfs_get_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);
	ssize_t count = 0;

	if(sensor == NULL)
	{
		pr_err("%s: CarSensors structure is null", __FUNCTION__);
		BUG();
		return -ENODEV;
	}

	count = sprintf(buf, "%d\n", sensor->mode);

	return count;
}

DEVICE_ATTR(enabled, S_IRUGO | S_IWUGO, carsensors_sysfs_get_enabled, carsensors_sysfs_set_enabled);
DEVICE_ATTR(mode, S_IRUGO | S_IWUGO, carsensors_sysfs_get_mode, carsensors_sysfs_set_mode);
DEVICE_ATTR(baud_rate, S_IRUGO | S_IWUGO, carsensors_sysfs_get_baud_rate, carsensors_sysfs_set_baud_rate);
DEVICE_ATTR(poll_interval, S_IRUGO | S_IWUGO, carsensors_sysfs_get_poll_interval, carsensors_sysfs_set_poll_interval);
DEVICE_ATTR(filters, S_IWUGO, NULL, carsensors_sysfs_set_filters);

static struct attribute *carsensors_control_attributes[] =
{
	&SYSFS_ATTRIBUTE_NAME(enabled).attr,
	&SYSFS_ATTRIBUTE_NAME(mode).attr,
	&SYSFS_ATTRIBUTE_NAME(baud_rate).attr,
	&SYSFS_ATTRIBUTE_NAME(poll_interval).attr,
	&SYSFS_ATTRIBUTE_NAME(filters).attr,
	NULL
};

static struct attribute *carsensors_can_attributes[] =
{
	&SYSFS_ATTRIBUTE_NAME(speed).attr,
	&SYSFS_ATTRIBUTE_NAME(ext_temperature).attr,
	&SYSFS_ATTRIBUTE_NAME(motor_temperature).attr,
	&SYSFS_ATTRIBUTE_NAME(motor_speed).attr,
	&SYSFS_ATTRIBUTE_NAME(fuel_level).attr,
	&SYSFS_ATTRIBUTE_NAME(distance_traveled).attr,
	&SYSFS_ATTRIBUTE_NAME(lifetime_distance_traveled).attr,
	&SYSFS_ATTRIBUTE_NAME(motor_hours).attr,
	&SYSFS_ATTRIBUTE_NAME(fuel_used).attr,
	&SYSFS_ATTRIBUTE_NAME(oil_temperature).attr,
	&SYSFS_ATTRIBUTE_NAME(oil_level).attr,
	&SYSFS_ATTRIBUTE_NAME(axle_weight).attr,
	&SYSFS_ATTRIBUTE_NAME(break_switch).attr,
	&SYSFS_ATTRIBUTE_NAME(clutch_switch).attr,
	&SYSFS_ATTRIBUTE_NAME(ignition_switch).attr,
	&SYSFS_ATTRIBUTE_NAME(cluster_lamp_switch).attr,
	&SYSFS_ATTRIBUTE_NAME(parking_switch).attr,
	&SYSFS_ATTRIBUTE_NAME(flags).attr,
	NULL
};

static const struct attribute_group carsensors_control_attr_group =
{
	.name = CARSENSORS_CONTROL_SUBDIR,
	.attrs = carsensors_control_attributes,
};

static const struct attribute_group carsensors_data_attr_group =
{
	.name = CARSENSORS_DATA_SUBDIR,
	.attrs = carsensors_can_attributes,
};

/**
 * Some helper functions to parse the CAN data
 */

static s8 carsensors_parse_s8(char *buf)
{
	s8 data = (s8) *buf;
	return data;
}

static u8 carsensors_parse_u8(char *buf)
{
	u8 data = (u8) *buf;
	return data;
}

static u16 carsensors_parse_u16(char *buf)
{
	u16 data = 0;
	u8 data_msb = carsensors_parse_u8(buf);
	u8 data_lsb = carsensors_parse_u8(buf+1);

	data = (((u16) data_msb) << 8) +
			((u16) data_lsb);

	return data;
}

static u32 carsensors_parse_u32(char *buf)
{
	u32 data = 0;
	u16 data_msb = carsensors_parse_u16(buf);
	u16 data_lsb = carsensors_parse_u16(buf+2);

	data = (((u32) data_msb) << 16) +
			((u32) data_lsb);

	return data;
}

/**
 * Parses the acquired data and stores at the given data structure.
 */
static void carsensors_parse_data(const char *buf, struct carsensors_data *data)
{
	char *pBuf = buf;
	/**
	 * Now, parses in the correct order, accordingly to the protocol
	 */
	data->speed 						= carsensors_parse_u16(pBuf);
	pBuf += 2;
	data->ext_temperature 				= carsensors_parse_s8(pBuf);
	pBuf += 1;
	data->motor_temperature 			= carsensors_parse_u8(pBuf);
	pBuf += 1;
	data->motor_speed 					= carsensors_parse_u16(pBuf);
	pBuf += 2;
	data->fuel_level 					= carsensors_parse_u8(pBuf);
	pBuf += 1;
	data->distance_traveled 			= carsensors_parse_u32(pBuf);
	pBuf += 4;
	data->lifetime_distance_traveled 	= carsensors_parse_u32(pBuf);
	pBuf += 4;
	data->motor_hours 					= carsensors_parse_u32(pBuf);
	pBuf += 4;
	data->fuel_used 					= carsensors_parse_u32(pBuf);
	pBuf += 4;
	data->oil_temperature 				= carsensors_parse_u8(pBuf);
	pBuf += 1;
	data->oil_level 					= carsensors_parse_u8(pBuf);
	pBuf += 1;
	data->axle_weight 					= carsensors_parse_u16(pBuf);
	pBuf += 2;
	data->break_switch 					= carsensors_parse_u8(pBuf);
	pBuf += 1;
	data->clutch_switch 				= carsensors_parse_u8(pBuf);
	pBuf += 1;
	data->flags 						= carsensors_parse_u16(pBuf);
	pBuf += 2;

	pr_debug("%s: speed = 0x%02X\n", __FUNCTION__, data->speed);
	pr_debug("%s: ext_temperature = 0x%X\n", __FUNCTION__,  data->ext_temperature);
	pr_debug("%s: motor_temperature = 0x%X\n", __FUNCTION__,  data->motor_temperature);
	pr_debug("%s: motor_speed = 0x%02X\n", __FUNCTION__,  data->motor_speed);
	pr_debug("%s: fuel_level = 0x%X\n", __FUNCTION__,  data->fuel_level);
	pr_debug("%s: distance_traveled = 0x%04X\n", __FUNCTION__,  data->distance_traveled);
	pr_debug("%s: lifetime_distance_traveled = 0x%04X\n", __FUNCTION__,  data->lifetime_distance_traveled);
	pr_debug("%s: motor_hours = 0x%04X\n", __FUNCTION__,  data->motor_hours);
	pr_debug("%s: fuel_used = 0x%04X\n", __FUNCTION__,  data->fuel_used);
	pr_debug("%s: oil_temperature = 0x%X\n", __FUNCTION__,  data->oil_temperature);
	pr_debug("%s: oil_level = 0x%X\n", __FUNCTION__,  data->oil_level);
	pr_debug("%s: axle_weight = 0x%02X\n", __FUNCTION__,  data->axle_weight);
	pr_debug("%s: break_switch = 0x%X\n", __FUNCTION__,  data->break_switch);
	pr_debug("%s: clutch_switch = 0x%X\n", __FUNCTION__,  data->clutch_switch);
	pr_debug("%s: flags = 0x%02X\n", __FUNCTION__,  data->flags);

}

static void carsensors_schedule_work(struct carsensors_dev *sensor)
{
	unsigned long delay;

	delay = msecs_to_jiffies(sensor->poll_interval);
	if (delay >= HZ)
		delay = round_jiffies_relative(delay);

	queue_delayed_work(sensor->wq, &sensor->work, delay);
}

static void carsensors_do_work(struct work_struct *work)
{
	struct carsensors_dev *sensor = container_of(work, struct carsensors_dev, work.work);
	char buf[INDASH_PROTOCOL_CANREAD_BUFFER_SIZE];
	int error = 0;

	// acquiring data
	memset(buf, 0x00, sizeof(buf));
	indash_mfd_lock();
	error = indash_mfd_canread_cmd(buf);
	indash_mfd_unlock();

	if(error)
	{
		dev_err(sensor->dev, "error reading CAN data: %d\n", error);
		return;
	}

	// parsing data
	mutex_lock(&sensor->lock);
	carsensors_parse_data(buf, &sensor->data);
	mutex_unlock(&sensor->lock);

	// notifying
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_SPEED))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(speed).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_EXT_TEMPERATURE))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(ext_temperature).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_MOTOR_TEMPERATURE))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(motor_temperature).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_MOTOR_SPEED))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(motor_speed).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_FUEL_LEVEL))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(fuel_level).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_DIST_TRAVELED))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(distance_traveled).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_LT_DIST_TRAVELED))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(lifetime_distance_traveled).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_MOTOR_HOURS))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(motor_hours).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_FUEL_USED))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(fuel_used).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_OIL_TEMPERATURE))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(oil_temperature).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_OIL_LEVEL))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(oil_level).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_AXLE_WEIGHT))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(axle_weight).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_BREAK_SWITCH))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(break_switch).attr.name);
	if(INDASH_BITMASK(sensor->data.flags, INDASH_FLAGS_CLUTCH_SWITCH))
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(clutch_switch).attr.name);
	sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(flags).attr.name);

	carsensors_schedule_work(sensor);
}


static void carsensors_inputs_changed(struct indash_mfd_handler *handler, unsigned short inputs)
{
	struct carsensors_dev *sensor = (struct carsensors_dev *) handler->priv;
	u8 new_ignition = INDASH_BITMASK(inputs, INDASH_INPUT_IGNITION);
	u8 new_parking = INDASH_BITMASK(inputs, INDASH_INPUT_PARKING);
	u8 new_cluster_lamp = INDASH_BITMASK(inputs, INDASH_INPUT_CLUSTER_LAMP);

	if(sensor->data.ignition_switch != new_ignition)
	{
		mutex_lock(&sensor->lock);
		sensor->data.ignition_switch = new_ignition;
		mutex_unlock(&sensor->lock);
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(ignition_switch).attr.name);
	}

	if(sensor->data.parking_switch != new_parking)
	{
		mutex_lock(&sensor->lock);
		sensor->data.parking_switch = new_parking;
		mutex_unlock(&sensor->lock);
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(parking_switch).attr.name);
	}

	if(sensor->data.cluster_lamp_switch != new_cluster_lamp)
	{
		mutex_lock(&sensor->lock);
		sensor->data.cluster_lamp_switch = new_cluster_lamp;
		mutex_unlock(&sensor->lock);
		sysfs_notify(&sensor->dev->kobj, carsensors_data_attr_group.name, SYSFS_ATTRIBUTE_NAME(cluster_lamp_switch).attr.name);
	}

}

static int __devinit
carsensors_probe(struct platform_device *pdev)
{
	int error = 0;
	struct carsensors_dev *this = (struct carsensors_dev *) kzalloc(sizeof(struct carsensors_dev), GFP_KERNEL);
	if (IS_ERR(this))
	{
		dev_err(&pdev->dev, "error: out of memory for device\n");
		error = -ENOMEM;
		goto err_no_mem_dev;
	}

	this->wq = create_freezeable_workqueue("qta-carsensors");
	if (IS_ERR(this->wq))
	{
		dev_err(&pdev->dev, "error: out of memory for workqueue\n");
		error = -ENOMEM;
		goto err_no_mem_queue;
	}

	INIT_DELAYED_WORK(&this->work, carsensors_do_work);
	mutex_init(&this->lock);

	this->dev = &(pdev->dev);
	error = sysfs_create_group(&this->dev->kobj, &carsensors_data_attr_group);
	if(error)
	{
		dev_err(this->dev, "error registering sysfs can files: %d\n", error);
		goto err_can_attr;
	}
	error = sysfs_create_group(&this->dev->kobj, &carsensors_control_attr_group);
	if(error)
	{
		dev_err(this->dev, "error registering sysfs control files: %d\n", error);
		goto err_ctrl_attr;
	}

	this->handler.inputs_changed = carsensors_inputs_changed;
	this->handler.name = CARSENSORS_DRV_NAME;
	this->handler.priv = this;
	this->poll_interval = CARSENSORS_POLL_INTERVAL;

	error = indash_mfd_register_handler(&this->handler);

	if(error)
	{
		dev_err(this->dev, "error registering MFD handler: %d\n", error);
		goto err_reg_handler;
	}

	platform_set_drvdata(pdev, this);
	dev_info(this->dev, "CarSensors driver load succesfully\n");

	return 0;


	err_reg_handler:
		sysfs_remove_group(&this->dev->kobj, &carsensors_control_attr_group);
	err_ctrl_attr:
		sysfs_remove_group(&this->dev->kobj, &carsensors_data_attr_group);
	err_can_attr:
		mutex_destroy(this->lock);
		destroy_workqueue(this->wq);
	err_no_mem_queue:
		kfree(this);
	err_no_mem_dev:
		return error;
}

static int __devexit
carsensors_remove(struct platform_device *pdev)
{
	struct carsensors_dev *sensor = platform_get_drvdata(pdev);

	sysfs_remove_group(&sensor->dev->kobj, &carsensors_control_attr_group);
	sysfs_remove_group(&sensor->dev->kobj, &carsensors_data_attr_group);
	mutex_destroy(sensor->lock);
	indash_mfd_unregister_handler(&sensor->handler);
	destroy_workqueue(sensor->wq);
	kfree(sensor);

	return 0;
}

static struct platform_driver carsensors_driver = {
	.driver	= {
		.name	= CARSENSORS_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= carsensors_probe,
	.remove		= __devexit_p(carsensors_remove),
};

static int __init carsensors_init(void)
{
	return platform_driver_register(&carsensors_driver);
}

static void __exit carsensors_exit(void)
{
	platform_driver_unregister(&carsensors_driver);
}

module_init(carsensors_init);
module_exit(carsensors_exit);

MODULE_ALIAS("platform:" CARSENSORS_DRV_NAME);
MODULE_AUTHOR("Mauricio Cirelli <mauricio.cirelli@quantatec.com.br>");
MODULE_DESCRIPTION("Quanta Car Sensors device driver");
MODULE_LICENSE("GPL");
