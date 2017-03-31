/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdc.h
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 03-11-2016
 * Description:
 *              define interface of sensor collector module
 *
 * Change Logs:
 * Date            Author           Notes
 * 03-11-2016      xujun            created
 */

/*
 * This file defined the interface which used to implements a collector
 */
#ifndef __SENSOR_DATA_COLLECTOR_H
#define __SENSOR_DATA_COLLECTOR_H

#include "sdr.h"

struct sdc
{
	/*
	 * fetch sensor data
	 * return: 0 indicate successfully
	 *         otherwise indicate failed
	 */
	int (*fetch)(struct sdc* p);

	/*
	 * read last sensor data
	 * return: 0 indicate successfully
	 *         otherwise indicate failed
	 */
	int (*read)(struct sdc* p, void* sdr);
};

#endif
