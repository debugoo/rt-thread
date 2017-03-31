/*
 * File      : debug.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-09-15     xujun        the first version
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <rtthread.h>
extern void (*assert_log_stub)(const char* exp, const char* func, rt_size_t line);

#define XASSERT(EX) \
if (!(EX))                                                                    \
{                                                                             \
    assert_log_stub(#EX, __FUNCTION__, __LINE__);                             \
    rt_assert_handler(#EX, __FUNCTION__, __LINE__);                           \
}


#endif
