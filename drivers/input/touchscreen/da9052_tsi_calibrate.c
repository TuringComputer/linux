/*
 * da9052_tsi_calibrate.c  --  TSI Calibration driver for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/mfd/da9052/tsi.h>

static struct Calib_xform_matrix_t xform = {
		.An = DA9052_TSI_CALIB_AN,
		.Bn = DA9052_TSI_CALIB_BN,
		.Cn = DA9052_TSI_CALIB_CN,
		.Dn = DA9052_TSI_CALIB_DN,
		.En = DA9052_TSI_CALIB_EN,
		.Fn = DA9052_TSI_CALIB_FN,
		.Divider = DA9052_TSI_CALIB_DIVIDER
};

static struct calib_cfg_t calib = {
	.calibrate_flag = TSI_USE_CALIBRATION,
};

struct calib_cfg_t *get_calib_config(void)
{
	return &calib;
}

ssize_t da9052_tsi_set_calib_matrix(int *calibration_data)
{

	int  retValue = SUCCESS ;

	xform.Divider = calibration_data[6];
	xform.An = calibration_data[0];
	xform.Bn = calibration_data[1];
	xform.Cn = calibration_data[2];
	xform.Dn = calibration_data[3];
	xform.En = calibration_data[4];
	xform.Fn = calibration_data[5];

	return retValue;
}

ssize_t da9052_tsi_get_calib_display_point(struct da9052_tsi_data *displayPtr)
{
	int  retValue = TRUE;
	int x, y;
	struct da9052_tsi_data screen_coord;

	screen_coord = *displayPtr;
	if (xform.Divider == 0) {
		x = displayPtr->x;
		y = displayPtr->y;
	} else {
		x = xform.An * (int)displayPtr->x +
				xform.Bn * (int)displayPtr->y +
				xform.Cn;
		x /= xform.Divider;
		if (x < 0)
			x = 0;
		y = xform.Dn * (int)displayPtr->x +
				xform.En * (int)displayPtr->y +
				xform.Fn;
		y /= xform.Divider;
		if (y < 0)
			y = 0;
	}

	displayPtr->x = x;
	displayPtr->y = y;

#if DA9052_TSI_CALIB_DATA_PROFILING
	printk("C\tX\t%4d\tY\t%4d\n",
					displayPtr->x,
					displayPtr->y);
#endif
	return retValue;
}
