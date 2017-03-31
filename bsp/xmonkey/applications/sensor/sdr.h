/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdr.h
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 31-10-2016
 * Description:
 *              defined the sensor data record 
 *
 * Change Logs:
 * Date            Author           Notes
 * 31-10-2016      xujun            created
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
    ST_HB = 5,
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

extern void sdr_gps_init(struct sdr_gps* data);
extern void sdr_gps_copy(struct sdr_gps* dst, struct sdr_gps* src);

extern void sdr_mot_init(struct sdr_mot* data);
extern void sdr_mot_copy(struct sdr_mot* dst, struct sdr_mot* src);

extern void sdr_baro_init(struct sdr_baro* data);
extern void sdr_baro_copy(struct sdr_baro* dst, struct sdr_baro* src);

extern void sdr_hygro_init(struct sdr_hygro* data);
extern void sdr_hygro_copy(struct sdr_hygro* dst, struct sdr_hygro* src);

#endif
