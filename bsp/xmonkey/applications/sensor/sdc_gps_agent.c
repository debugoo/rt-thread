/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdc_gps_raw.h
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 07-11-2016
 * Description:
 *              implements gps data collector agent
 *
 * Change Logs:
 * Date            Author           Notes
 * 07-11-2016      xujun            created
 */
#ifndef __SENSOR_DATA_COLLECTOR_GPS_AGENT_H
#define __SENSOR_DATA_COLLECTOR_GPS_AGENT_H

#include "sdr.h"
#include <rtthread.h>

typedef void (*gps_notify_cb)(void);

struct sdc_gps_agent
{
	time_t last_time;
	struct sdr_gps gps;
	gps_notify_cb cb;
};

void sdc_gps_agent_init(struct sdc_gps_agent* inst)
{
	inst->last_time = 0;
	sdr_gps_init(&inst->gps);
	cb = NULL;
}

void set_gps_notify_cb(struct sdc_gps_agent* agent, gps_notify_cb cb)
{
	agent->cb = cb;
}

void push_gps(struct sdc_gps_agent* agent, struct sdr_gps* gps)
{
	agent->
}

#endif
