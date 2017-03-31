/**
 * Copyright (C), 2015-2016, Chengdu QiPeng technology. Co., Ltd.
 *   File name: sdr.c
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 07-11-2016
 * Description:
 *              defined the sensor data record 
 *
 * Change Logs:
 * Date            Author           Notes
 * 07-11-2016      xujun            created
 */

#include "sdr.h"

static void sdr_base_init(struct sdr_base* inst)
{
	inst->time = 0;
}

void sdr_gps_init(struct sdr_gps* data)
{
	sdr_base_init((struct sdr_base*)inst);
	data->lon = 0;
	data->lat = 0;
	data->alt = 0;
	data->speed = 0;
}
void sdr_gps_copy(struct sdr_gps* dst, struct sdr_gps* src)
{
	dst->lon = src->lon;
	dst->lat = src->lat;
	dst->alt = src->alt;
	dst->speed = src->speed;
}

void sdr_mot_init(struct sdr_mot* data)
{
	sdr_base_init((struct sdr_base*)inst);
	data->acc = 0;
	data->steps = 0;
	data->dur = 0;
}

void sdr_baro_init(struct sdr_baro* data)
{
	sdr_base_init((struct sdr_base*)inst);
	data->hpa = 0;
	data->alt = 0;
	data->temp = 0;
}

void sdr_hygro_init(struct sdr_hygro* data)
{
	sdr_base_init((struct sdr_base*)inst);
	data->temp = 0;
	data->rh = 0;
}

