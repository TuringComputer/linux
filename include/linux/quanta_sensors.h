/*
 * quanta_sensors.h
 *
 *	Definitions for all Quanta's Sensors Platform Drivers,
 *	such as Accelerometers, Magnetometers, Gyroscopes,
 *	among others.
 *
 *	Currently supported sensors are:
 *
 *	Accelerometers:
 *		ST Microelectronics LIS3DH (qta_lis3dh.c)
 *		Bosch Sensortech BMA250E (qta_bma250e.c)
 *		Freescale's MMA8451Q (qta_mma8451q.c)
 *
 *  Created on: 05/02/2014
 *      Author: mauricio.cirelli
 */

#ifndef QUANTA_SENSORS_H_
#define QUANTA_SENSORS_H_

struct quanta_motion_sensors_platform_data {
	/*
	 * 3-Axis base which should correct
	 * the 3-axis data accordingly
	 * to the position the device has been set up
	 * on the board.
	 *
	 * This should correct the position in order to make
	 * it fit in the following definition:
	 *
	 *  x<0         x>0
	 *                ^
	 *                |
	 *    +-----------+-->  y>0
	 *    |           |
	 *    |           |
	 *    |           |
	 *    |           |   / z<0
	 *    |           |  /
	 *    |           | /
	 *    O-----------+/
	 *    |[]  [ ]  []/
	 *    +----------/+     y<0
	 *              /
	 *             /
	 *           |/ z>0 (toward the sky)
	 *
	 *    O: Origin (x=0,y=0,z=0)
	 *
	 *	Chose one of following QUANTA_ACCEL_POSITION_XX macros here.
	 *
	 */
	int position[3][3];
};

#define QUANTA_MOTION_SENSORS_POSITION_00 	{{1,0,0},{0,1,0},{0,0,1}}
#define QUANTA_MOTION_SENSORS_POSITION_01 	{{0,1,0},{0,0,1},{1,0,0}}
#define QUANTA_MOTION_SENSORS_POSITION_02 	{{0,0,1},{1,0,0},{0,1,0}}

#define QUANTA_MOTION_SENSORS_POSITION_03 	{{-1,0,0},{0,-1,0},{0,0,-1}}
#define QUANTA_MOTION_SENSORS_POSITION_04 	{{0,-1,0},{0,0,-1},{-1,0,0}}
#define QUANTA_MOTION_SENSORS_POSITION_05 	{{0,0,-1},{-1,0,0},{0,-1,0}}

#define QUANTA_MOTION_SENSORS_POSITION_06 	{{-1,0,0},{0,1,0},{0,0,1}}
#define QUANTA_MOTION_SENSORS_POSITION_07 	{{1,0,0},{0,-1,0},{0,0,1}}
#define QUANTA_MOTION_SENSORS_POSITION_08 	{{1,0,0},{0,1,0},{0,0,-1}}

#define QUANTA_MOTION_SENSORS_POSITION_09 	{{-1,0,0},{0,-1,0},{0,0,1}}
#define QUANTA_MOTION_SENSORS_POSITION_10 	{{1,0,0},{0,-1,0},{0,0,-1}}
#define QUANTA_MOTION_SENSORS_POSITION_11 	{{-1,0,0},{0,1,0},{0,0,-1}}

#define QUANTA_MOTION_SENSORS_POSITION_12 	{{1,0,0},{0,0,1},{0,-1,0}}
#define QUANTA_MOTION_SENSORS_POSITION_13 	{{-1,0,0},{0,0,1},{0,1,0}}
#define QUANTA_MOTION_SENSORS_POSITION_14 	{{1,0,0},{0,0,-1},{0,0,1}}

static inline void quanta_motion_sensors_adjust_position(struct quanta_motion_sensors_platform_data *platform, int *x, int *y, int *z)
{
	int rawdata[3], data[3];
	int i, j;

	rawdata[0] = *x;
	rawdata[1] = *y;
	rawdata[2] = *z;

	for (i = 0; i < 3 ; i++)
	{
		data[i] = 0;
		for (j = 0; j < 3; j++)
		{
			data[i] += rawdata[j] * platform->position[i][j];
		}
	}

	*x = data[0];
	*y = data[1];
	*z = data[2];
}


#endif /* QUANTA_SENSORS_H_ */
