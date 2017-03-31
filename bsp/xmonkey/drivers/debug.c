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

#include "debug.h"

void (*assert_log_stub)(const char* exp, const char* func, rt_size_t line) = RT_NULL;
