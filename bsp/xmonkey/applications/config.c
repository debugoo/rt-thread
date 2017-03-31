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
#include "config.h"
#include "minIni.h"
#include "debug.h"

const char inifile[] = "/data/cfg.ini";

struct sys_config config;
rt_err_t config_load(void)
{
    int n;
    n = ini_gets("server", "ip", "", config.ip, sizeof(config.ip), inifile);
    config.port = (rt_uint16_t)ini_getl("server", "port", 0, inifile);
    rt_kprintf("SERVER IP: %s, PORT: %u\n", config.ip, config.port);
    
    return RT_EOK;
}

static void config_make(const char* ip, rt_uint16_t port)
{
    ini_puts("server", "ip", ip, inifile);
    ini_putl("server", "port", (long)port, inifile);
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(config_make, config system);
#endif


struct log_trace_session lts_sim;
struct log_trace_session lts_sens;
struct log_trace_session lts_at;
struct log_trace_session lts_fail;

static void assert_log_imp(const char* exp, const char* func, rt_size_t line)
{
    log_session_lvl(&lts_fail, LOG_TRACE_LEVEL_ERROR, "ASSERT FALSE!!!%s,  %s, %d.\n", exp, func, line);
    //rt_assert_handler(exp, func, line);

}
rt_err_t log_trace_app_init(void)
{
    rt_err_t err;
    /* log_trace init */
    err = log_trace_init();
    if (err != RT_EOK)
        return err;
    /* log_trace: sim */
    //lts_sim.id.num = LTS_SIM;
    snprintf(lts_sim.id.name, sizeof(log_trace_idnum_t), "SIM");
    lts_sim.lvl = LOG_TRACE_LEVEL_DEBUG;
    err = log_trace_register_session(&lts_sim);
    if (err != RT_EOK)
        return err;
    /* log_trace: sens */
    //lts_sens.id.num = LTS_SENS;
    snprintf(lts_sens.id.name, sizeof(log_trace_idnum_t), "SENS");
    lts_sens.lvl = LOG_TRACE_LEVEL_DEBUG;
    err = log_trace_register_session(&lts_sens);
    if (err != RT_EOK)
        return err;

    /* log_trace: at */
    snprintf(lts_at.id.name, sizeof(log_trace_idnum_t), "AT");
    lts_at.lvl = LOG_TRACE_LEVEL_DEBUG;
    err = log_trace_register_session(&lts_at);
    if (err != RT_EOK)
        return err;

    /* log_trace: fail */
    snprintf(lts_fail.id.name, sizeof(log_trace_idnum_t), "FAIL");
    lts_fail.lvl = LOG_TRACE_LEVEL_DEBUG;
    err = log_trace_register_session(&lts_fail);
    if (err != RT_EOK)
        return err;

    log_trace_file_init("/log.txt");
    log_trace_set_file("/log.txt");

    assert_log_stub = assert_log_imp;
    return RT_EOK;
}


