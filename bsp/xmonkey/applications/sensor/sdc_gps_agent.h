/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdc_gps_agent.h
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

struct sdc_gps_raw
{
	time_t last_time;
	struct sdr_gps gps;
};

extern void set_gps_notify_cb(gps_notify_cb);

extern void push_gps(struct sdr_gps* gps);

#endif
