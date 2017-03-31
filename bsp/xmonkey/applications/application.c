/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif


#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif
#include "main.h"
#include "dbg.h"
#include <sys/fcntl.h>
#include "config.h"
#include "log_trace.h"


void rt_init_thread_entry(void* parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    //rt_components_init();
    /*
     * I have did this job manually
     */
#endif

#ifdef RT_USING_DFS
    void rt_hw_sdcard_init(void);
    rt_hw_sdcard_init();
#endif

#ifdef RT_USING_DFS
    int dfs_init(void);
    /* initialize the device file system */
    dfs_init();

#ifdef RT_USING_DFS_ELMFAT
    void elm_init(void);
    /* initialize the elm chan FatFS file system*/
    elm_init();
#endif

#if defined(RT_USING_DFS_NFS) && defined(RT_USING_LWIP)
    void nfs_init(void);
    /* initialize NFSv3 client file system */
    nfs_init();
#endif

#ifdef RT_USING_DFS_YAFFS2
    void dfs_yaffs2_init(void);
    dfs_yaffs2_init();
#endif

#ifdef RT_USING_DFS_UFFS
    void dfs_uffs_init(void);
    dfs_uffs_init();
#endif

#ifdef RT_USING_DFS_JFFS2
    void dfs_jffs2_init(void);
    dfs_jffs2_init();
#endif

#ifdef RT_USING_DFS_ROMFS
    void dfs_romfs_init(void);
    dfs_romfs_init();
#endif

#ifdef RT_USING_DFS_RAMFS
    void dfs_ramfs_init(void);
    dfs_ramfs_init();
#endif

#ifdef RT_USING_DFS_DEVFS
    void devfs_init(void);
    devfs_init();
#endif
#endif /* end of RT_USING_DFS */

#ifdef RT_USING_NEWLIB
    void libc_system_init(void);
    libc_system_init();
#else
    /* the pthread system initialization will be initiallized in libc */
#ifdef RT_USING_PTHREADS
    pthread_system_init();
#endif
#endif

    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif

    /* Filesystem Initialization */
#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
    int dfs_mount(const char   *device_name,
                  const char   *path,
                  const char   *filesystemtype,
                  unsigned long rwflag,
                  const void   *data);
    /* mount sd card fat partition 1 as root directory */
    if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("File System initialized!\n");
    }
    else
        rt_kprintf("File System initialzation failed!\n");
#endif  /* RT_USING_DFS */

    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif
    RT_ASSERT(RT_EOK == dbg_init());
    DBG_PRINT("DBG_UART opened...\n");

    RT_ASSERT(RT_EOK == log_trace_app_init());

    start_thread_led();
}

int rt_application_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}

/*@}*/
