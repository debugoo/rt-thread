/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdr.h
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 31-10-2016
 * Description:
 *              define sensor data record and the interface for sdr
 *
 * Change Logs:
 * Date            Author           Notes
 * 31-10-2016      xujun            created
 */

/*
 * This file defined the interface which used to recieve&processing the sensor data after
 * collecting from physical sensor
 */
#ifndef __SENSOR_DATA_RECORD_H
#define __SENSOR_DATA_RECORD_H

/*
 * define sensor types
 */
enum SENSOR_TYPE
{
    ST_GPS = 1,
    ST_MOT = 2,
    ST_BARO = 3,
    ST_HYGRO = 4,
    ST_BT = 5,
    ST_HB = 6,
};

/*
 * define sensor data structure
 * the 'sdr' is the abbreviation for 'sensor data record'
 */
struct sdr_base
{
	time_t time;
};

struct sdr_gps
{
	struct sdr_base parent;

    float lon;
    float lat;
    float alt;
    float speed;
};

struct sdr_mot
{
	struct sdr_base parent;

    float acc;
    unsigned long steps;
    unsigned long dur;
};

struct sdr_baro
{
	struct sdr_base parent;

    float hpa;
    float alt;
    float temp;
};

struct sdr_hygro
{
	struct sdr_base parent;

    float temp;
    float rh;
};

/*
 * we can use 'sdh' as the abbreviation for 'sensor data handler'
 */
struct sd_handler
{
    void (*handle)(sd_handler* h, int st, void* sdr);
};
extern void sdh_reg(int st, sd_handler *h);
extern void sdh_dereg(int st, sd_handler *h);

#endif
