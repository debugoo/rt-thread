/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdh.h
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 31-10-2016
 * Description:
 *              define the interface for handling sensor data
 *
 * Change Logs:
 * Date            Author           Notes
 * 31-10-2016      xujun            created
 */

#ifndef __SENSOR_DATA_HANDLER_H
#define __SENSOR_DATA_HANDLER_H

/*
 * we can use 'sdh' as the abbreviation for 'sensor data handler'
 */
struct sd_handler
{
    void (*handle)(sd_handler* h, int st, void* sdr);
};

#endif
