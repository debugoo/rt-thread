/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdc_imp.c
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 03-11-2016
 * Description:
 *              implements sensor collector
 *
 * Change Logs:
 * Date            Author           Notes
 * 03-11-2016      xujun            created
 */

#include "sdc_imp.h"
#include <rtthread.h>

/*
 * gps collector
 */
struct sdc_gps
{
	struct sdc parent;
	struct sdr_gps sd;
};

int sdc_gps_fetch(struct sdc* p)
{
	struct sdc_gps* inst = (struct sdc_gps*)p;
	return -1;
}
int sdc_gps_read(struct sdc* p, void* sdr)
{
	return -1;
}
void sdc_gps_init(sdc_gps* p, void* raw_imp)
{
	p->parent.fetch = sdc_gps_fetch;
	p->parent.read = sdc_gps_read;

	p->sd.parent.time = 0;
	p->sd.lon = 0;
	p->sd.lat = 0;
	p->sd.alt = 0;
	p->sd.speed = 0;

	p->_raw_imp = raw_imp;
}

/*
 * motion collector
 */
struct sdc_mot
{
	struct sdc parent;
	struct sdr_mot sd;
};
extern void sc_mot_init(sc_mot* p, rt_device_t i2c_dev);

struct sc_baro
{
	struct scx parent;
	struct sdr_baro sd;
};
extern void sc_baro_init(sc_baro* p, rt_device_t i2c_dev);

struct sc_hygro
{
	struct scx parent;
	struct sdr_hygro sd;
};
extern void sc_hygro_init(sc_hygro* p, rt_device_t i2c_dev);

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
