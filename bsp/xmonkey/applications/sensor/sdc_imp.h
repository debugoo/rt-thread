/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdc_imp.h
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 03-11-2016
 * Description:
 *              implements sensor data collector
 *
 * Change Logs:
 * Date            Author           Notes
 * 03-11-2016      xujun            created
 */

/*
 * This file defined the interface which used to implements a collector
 */
#ifndef __SENSOR_DATA_COLLECTOR_IMP_H
#define __SENSOR_DATA_COLLECTOR_IMP_H

#include "sdc.h"
#include <rtthread.h>

struct sdc_gps
{
	struct sdc parent;
	struct sdr_gps sd;

	/* The capsulation for GPS implements */
	void* _raw_imp;
};
extern void sdc_gps_init(sdc_gps* p, void* raw_imp);

struct sdc_mot
{
	struct sdc parent;
	struct sdr_mot sd;
};
extern void sdc_mot_init(sdc_mot* p, rt_device_t i2c_dev);

struct sdc_baro
{
	struct sdc parent;
	struct sdr_baro sd;
};
extern void sdc_baro_init(sdc_baro* p, rt_device_t i2c_dev);

struct sdc_hygro
{
	struct sdc parent;
	struct sdr_hygro sd;
};
extern void sdc_hygro_init(sdc_hygro* p, rt_device_t i2c_dev);

////////////////////////////////////////////////////////////////////////////////
void test(void)
{
	struct scx* c[4];
	struct sc_gps scgps;
	struct sc_mot scmot;
	struct sc_baro scbaro;
	struct sc_hygro schygro;
	int i;

	sc_gps_init(&scgps, &gps_src);
	sc_mot_init(&col_mot, &i2c_bus1);
	sc_baro_init(&col_baro, &i2c_bus1);
	sc_hygro_init(&col_hygro, &i2c_bus1);

	c[0] = &col_gps;
	c[1] = &col_motion;
	c[2] = &col_baro;
	c[3] = &col_hygro;
	for (i = 0; i < SENSOR_COUNT;++i)
	{
	}
	while(1)
	{
		ev = event_wait(&ev, WAIT_FOREVER);
		if (ev & EV_GPS)
		{
			c[0]->collect(&c[0]);
		}
	}
}

#endif
