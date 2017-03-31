/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: config.h
*      Author: xujun
*     Version: 1.0.0
*        Date: 13-06-2016
* Description: 
*              read configuration of system
*            
* 
* Change Logs:
* Date            Author           Notes
* 13-06-2016      xujun            created
* 
*/
#ifndef __CONFIG_H
#define __CONFIG_H

#include <rtthread.h>
#include "log_trace.h"

struct sys_config
{
    char        ip[32];
    rt_uint16_t port;
};

extern struct sys_config config;

rt_err_t config_load(void);


/* log trace session defines */
#define LTS_SIM     (1)
#define LTS_SENS    (2)
#define LTS_AT      (3)

extern struct log_trace_session lts_sim;
extern struct log_trace_session lts_sens;
extern struct log_trace_session lts_at;
extern struct log_trace_session lts_fail;
extern rt_err_t log_trace_app_init(void);


#endif /*__CONFIG_H*/

