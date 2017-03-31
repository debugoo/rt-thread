/**
 * Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
 *   File name: mot.cpp
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 12-06-2016
 * Description: 
 *              implements test case of MPU9250
 *            
 *   Reference:           
 *              refer to <>
 * 
 * Change Logs:
 * Date            Author           Notes
 * 12-06-2016      xujun            created
 */

#include "mot.h"
#include "stm32f4xx.h"
#include <rtthread.h>


//#include <rtdevice.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "i2cx.h"
#include "dbg.h"
#include "sim_cache.h"
#include "sim_data.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "mpu_log.h"
#include "msh.h"
#include <finsh.h>
#include "protocol.h"
#include <stm32f4xx_exti.h>
#include "log_trace.h"
#include "debug.h"

#define _XJ_USING_ENV

#ifdef _XJ_USING_ENV
#include "bmp180.h"
#include "sht20.h"
#endif

//struct rt_semaphore sem;
static struct rt_timer env_timer;

static struct rt_event event;

#define EVENT_MOT_INT   (0x01)
#define EVENT_ENV_TIMER (0x02)


/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (5)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    /*
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
     */
     /*
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
     */
    .orientation = { 1, 0, 0,
                     0,-1, 0,
                     0, 0,-1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    /*
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
     */
     /*
    .orientation = { 0,-1, 0,
                    -1, 0, 0,
                     0, 0, 1}
     */
    .orientation = { 0,-1, 0,
                     1, 0, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif


/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0, lp_accel_was_on = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON) {
        mask |= INV_XYZ_GYRO;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#ifdef COMPASS_ENABLED
    if (hal.sensors & COMPASS_ON) {
        mask |= INV_XYZ_COMPASS;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#endif
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    if (lp_accel_was_on) {
        unsigned short rate;
        hal.lp_accel_mode = 0;
        /* Switching out of LP accel, notify MPL of new accel sampling rate. */
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}

static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
    case TAP_X_UP:
        MPL_LOGI("Tap X+ ");
        break;
    case TAP_X_DOWN:
        MPL_LOGI("Tap X- ");
        break;
    case TAP_Y_UP:
        MPL_LOGI("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        MPL_LOGI("Tap Y- ");
        break;
    case TAP_Z_UP:
        MPL_LOGI("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        MPL_LOGI("Tap Z- ");
        break;
    default:
        return;
    }
    MPL_LOGI("x%d\n", count);
    return;
}

static void android_orient_cb(unsigned char orientation)
{
    switch (orientation) {
    case ANDROID_ORIENT_PORTRAIT:
        MPL_LOGI("Portrait\n");
        break;
    case ANDROID_ORIENT_LANDSCAPE:
        MPL_LOGI("Landscape\n");
        break;
    case ANDROID_ORIENT_REVERSE_PORTRAIT:
        MPL_LOGI("Reverse Portrait\n");
        break;
    case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        MPL_LOGI("Reverse Landscape\n");
        break;
    default:
        return;
    }
}


static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
    MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
            if (!(result & 0x1))
                MPL_LOGE("Gyro failed.\n");
            if (!(result & 0x2))
                MPL_LOGE("Accel failed.\n");
            if (!(result & 0x4))
                MPL_LOGE("Compass failed.\n");
     }

}


void init_EXTI(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure; 
    EXTI_InitTypeDef EXTI_InitStructure;

    /* RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Enable the EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    /* GPIO Init */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Connect EXTI Line0 to PD.0 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, GPIO_PinSource0);
    /* GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0); */
    EXTI_ClearITPendingBit(EXTI_Line0);
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
}
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        //rt_sem_release(&sem);
        rt_event_send(&event, EVENT_MOT_INT);
    }
}
#define MOT_DATA_TYPE   float
struct acc_help
{
    MOT_DATA_TYPE sens;
    MOT_DATA_TYPE x,y,z;
    MOT_DATA_TYPE g[3];
    MOT_DATA_TYPE scalar;
};
static struct acc_help acch = 
{
    .sens = 0,
    .x = 0,
    .y = 0,
    .z = 0,
    .g[0] = 0,
    .g[1] = 0,
    .g[2] = 0,
    .scalar = 0,
};
struct mot_help
{
    MOT_DATA_TYPE acc;
    rt_uint32_t   count;
    rt_uint32_t   step;
    rt_uint32_t   dur;
};
static struct mot_help moth = 
{
    .acc = 0,
    .count = 0,
    .step = 0,
    .dur = 0,
};
static struct cache_mot cmot = 
{
    .acc = 0,
    .step = 0,
    .dur = 0,
};
static rt_err_t (*mot_notify)(void) = RT_NULL;
static rt_err_t (*env_notify)(void) = RT_NULL;


void register_mot_notify(rt_err_t (*entry)(void))
{
    mot_notify = entry;
}
void register_env_notify(rt_err_t (*entry)(void))
{
    env_notify = entry;
}

#define MOT_BIT_ACC     (0X01)
#define MOT_BIT_PEDO    (0X02)
#define MOT_BIT_ALL     (MOT_BIT_ACC|MOT_BIT_PEDO)
static rt_uint32_t _mot_flag;
static struct mot_data _mot;

/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
       /* Sends a quaternion packet to the PC. Since this is used by the Python
        * test app to visually represent a 3D quaternion, it's sent each time
        * the MPL has new data.
        */
#ifdef MOTION_TO_PYTHON
        eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
        if (hal.report & PRINT_QUAT)
            eMPL_send_data(PACKET_DATA_QUAT, data);
#endif
    }

    if (hal.report & PRINT_ACCEL) {
        memset(data, 0, sizeof(data));
        if (inv_get_sensor_type_accel(data, &accuracy,
            (inv_time_t*)&timestamp))
        {
#ifdef MOTION_TO_PYTHON
            eMPL_send_data(PACKET_DATA_ACCEL, data);
#elif DBG_PRINT_MOTION
            DBG_PRINT("ACCEL: X:%+3.4f Y:%+3.4f Z:%+3.4f\n", acch.x,acch.y,acch.z);
            DBG_PRINT("AVG ACCEL: %+3.4f\n", moth.acc);
#endif
        }
    }
    if (hal.report & PRINT_GYRO) {
#ifdef MOTION_TO_PYTHON
        if (inv_get_sensor_type_gyro(data, &accuracy,

            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_GYRO, data);
#endif
    }
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS) {
#ifdef MOTION_TO_PYTHON
        if (inv_get_sensor_type_compass(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_COMPASS, data);
#endif
    }
#endif
    if (hal.report & PRINT_EULER) {
#ifdef MOTION_TO_PYTHON
        if (inv_get_sensor_type_euler(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_EULER, data);
            //MPL_LOGI("Euler: %7.5f %7.5f %7.5f\r\n",
            //        data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
#endif
    }
    if (hal.report & PRINT_ROT_MAT) {
#ifdef MOTION_TO_PYTHON
        if (inv_get_sensor_type_rot_mat(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ROT, data);
#endif
    }
    if (hal.report & PRINT_HEADING) {
#ifdef MOTION_TO_PYTHON
        if (inv_get_sensor_type_heading(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_HEADING, data);
#endif
    }
    if (hal.report & PRINT_LINEAR_ACCEL) {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
            
            _mot.acc = sqrt(float_data[0]*float_data[0] + float_data[1]*float_data[1] + float_data[2]*float_data[2]);
            _mot_flag |= MOT_BIT_ACC;
            
#ifdef MOTION_TO_PYTHON
            MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
                float_data[0], float_data[1], float_data[2]);
#elif DBG_PRINT_MOTION
            DBG_PRINT("Linear Accel: %7.5f %7.5f %7.5f\n",float_data[0], float_data[1], float_data[2]);
#endif
         }
    }
    if (hal.report & PRINT_GRAVITY_VECTOR) {
#ifdef MOTION_TO_PYTHON
        if (inv_get_sensor_type_gravity(float_data, &accuracy,
            (inv_time_t*)&timestamp))
        MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
            float_data[0], float_data[1], float_data[2]);
#endif
    }
    if (hal.report & PRINT_PEDO) {
        unsigned long timestamp;
        get_tick_count(&timestamp);
        if(1) {
        //if (timestamp > hal.next_pedo_ms) {
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            unsigned long step_count, walk_time;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);

            moth.step = step_count;
            moth.dur = walk_time/1000;
#ifdef MOTION_TO_PYTHON
            MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count, walk_time);
#elif DBG_PRINT_MOTION
            DBG_PRINT("Walked %ld steps over %ld milliseconds..\n", 
                step_count, walk_time);
#endif
            
            _mot.step_count = step_count;
            _mot.walk_time  = walk_time;
            _mot_flag |= MOT_BIT_PEDO;

            cmot.step = moth.step;
            cmot.dur  = moth.dur;
            
            if (moth.count % 10 == 0)
            {
                cache_box_put(&cbox, CACHE_DATA_MOT, (void*)&cmot, sizeof(struct cache_mot));
                mot_notify();
            }
            
        }
    }

    /* Whenever the MPL detects a change in motion state, the application can
     * be notified. For this example, we use an LED to represent the current
     * motion state.
     */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
            INV_MSG_NO_MOTION_EVENT);
    if (msg) {
        if (msg & INV_MSG_MOTION_EVENT) {
#ifdef MOTION_TO_PYTHON
            MPL_LOGI("Motion!\n");
#elif DBG_PRINT_MOTION
            DBG_PRINT("Motion!\n");
#endif
        } else if (msg & INV_MSG_NO_MOTION_EVENT) {
#ifdef MOTION_TO_PYTHON
            MPL_LOGI("No motion!\n");
#elif DBG_PRINT_MOTION
            DBG_PRINT("No motion!\n");
#endif
        }
    }
}

#define MPU_PRINT_EULER     "euler"
#define MPU_PRINT_QUAT      "quat"
#define MPU_PRINT_ACCEL     "accel"
#define MPU_PRINT_GYRO      "gyro"
#define MPU_PRINT_PEDO      "pedo"
#define MPU_PRINT_GRAVITY   "gravity"
#define MPU_PRINT_ROT       "rot"
#define MPU_PRINT_LINEAR    "linear"
#define MPU_PRINT_COMPASS   "compass"
#define MPU_PRINT_HEADING   "heading"


#define MPU_PRINT_MODE_COUNT    (10)
struct mpu_print_option
{
    const char* cmd;
    unsigned int mask;
    
};
struct mpu_print_option mpu_print_table[] =
{
    {MPU_PRINT_EULER,   PRINT_EULER},
    {MPU_PRINT_QUAT,    PRINT_QUAT},
    {MPU_PRINT_ACCEL,   PRINT_ACCEL},
    {MPU_PRINT_GYRO,    PRINT_GYRO},
    {MPU_PRINT_PEDO,    PRINT_PEDO},
    {MPU_PRINT_GRAVITY, PRINT_GRAVITY_VECTOR},
    {MPU_PRINT_ROT,     PRINT_ROT_MAT},
    {MPU_PRINT_LINEAR,  PRINT_LINEAR_ACCEL},
    {MPU_PRINT_COMPASS, PRINT_COMPASS},
    {MPU_PRINT_HEADING, PRINT_HEADING}
};

/*
const char* mpu_switchs[MPU_PRINT_MODE_COUNT] = 
    {
        MPU_PRINT_EULER,
        MPU_PRINT_QUAT,
        MPU_PRINT_ACCEL,
        MPU_PRINT_GYRO,
        MPU_PRINT_PEDO,
        MPU_PRINT_GRAVITY,
        MPU_PRINT_MOTION,
        MPU_PRINT_LINEAR,
        MPU_PRINT_COMPASS,
        MPU_PRINT_HEADING
    };
    */
int mpu_report(int argc, char** argv)
{
	if (argc != 1)
	{
	    rt_kprintf("This command take none parameter.\n");
	}
	else
	{
	    int i = 0;
	    int count = 0;
	    rt_kprintf("Current mpu report options: \n");
	    for (i = 0; i < MPU_PRINT_MODE_COUNT; ++i)
	    {
	        if (hal.report & mpu_print_table[i].mask)
	        {
	            if (count != 0)
	            {
	                rt_kprintf(" | ");
	            }
                rt_kprintf("%s", mpu_print_table[i].cmd);
	            count++;
	        }
	    }
	    rt_kprintf("\n");
	}
	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(mpu_report, __cmd_mpu_report, Query mpu hal.report.);

int mpu_print(int argc, char** argv)
{
    //extern void ls(const char *pathname);

	if (argc != 3)
	{
	    int i = 0;
		rt_kprintf("Usage: mpu_print [PRINT_OPTION] [SWITCH]\n");
		rt_kprintf("Switch the mpu print. optional below: \n");
		for (i = 0; i < MPU_PRINT_MODE_COUNT; ++i)
		{
		    rt_kprintf("%s\n", mpu_print_table[i].cmd);
		}
	}
	else
	{
	    const char* s = (const char*)argv[1];
	    int i = 0;
	    unsigned int mask = 0;
	    const char on = argv[2][0];
	    for (i = 0; i < MPU_PRINT_MODE_COUNT; ++i)
	    {
            if (rt_strcasecmp(mpu_print_table[i].cmd, s) == 0)
            {
                mask = mpu_print_table[i].mask;
                break;
            }
	    }
	    if (mask == 0)
	    {
	        rt_kprintf("wrong [PRINT_OPTION].\n");
	        return 0;
	    }
	    //if (rt_strcasecmp(on, "1") == 0)
        if (on == '1')
	    {
	        hal.report |= mask;
	    }
	    //else if (rt_strcasecmp(on, '0') == 0)
        else if (on == '0')
	    {
	        hal.report &= ~mask;
	    }
	    else
	    {
	        rt_kprintf("wrong [SWITCH].\n");
	    }
	}

	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(mpu_print, __cmd_mpu_print, set mpu print.);
//MSH_CMD_EXPORT(mpu_print, set mpu print);
/*
int fuck(int argc, char** argv)
{
    return 0;
}
//MSH_CMD_EXPORT(fuck, do sum in finsh)
FINSH_FUNCTION_EXPORT_ALIAS(fuck, __cmd_fuck, dxxx);
//MSH_CMD_EXPORT(fuck, gfuck);
*/
static void process_motion(void)
{
    unsigned long sensor_timestamp;
    int new_data = 0;
    unsigned char new_temp = 0;
    short gyro[3], accel_short[3], sensors;
    unsigned char more;
    long accel[3], quat[4], temperature;

    do
    {
        /* This function gets new data from the FIFO when the DMP is in
         * use. The FIFO can contain any combination of gyro, accel,
         * quaternion, and gesture data. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
         * the FIFO isn't being filled with accel data.
         * The driver parses the gesture data to determine if a gesture
         * event has occurred; on an event, the application will be notified
         * via a callback (assuming that a callback function was properly
         * registered). The more parameter is non-zero if there are
         * leftover packets in the FIFO.
         */
        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
        if (!more)
            hal.new_gyro = 0;
        if (sensors & INV_XYZ_GYRO) {
            /* Push the new data to the MPL. */
            inv_build_gyro(gyro, sensor_timestamp);
            new_data = 1;
            if (new_temp) {
                new_temp = 0;
                /* Temperature only used for gyro temp comp. */
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
            }
        }
        if (sensors & INV_XYZ_ACCEL) {

            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;

            acch.x = accel_short[0] / acch.sens;
            acch.y = accel_short[1] / acch.sens;
            acch.z = accel_short[2] / acch.sens;
            //acch.g[0] = 0.9*acch.g[0] + 0.1*acch.x;
            //acch.g[1] = 0.9*acch.g[1] + 0.1*acch.y;
            //acch.g[2] = 0.9*acch.g[2] + 0.1*acch.z;
            //acch.x -= acch.g[0];
            //acch.y -= acch.g[1];
            //acch.z -= acch.g[2];
            //acch.scalar = sqrt(acch.x*acch.x + acch.y*acch.y + acch.z*acch.z);
            acch.scalar = sqrt(acch.x*acch.x + acch.y*acch.y + acch.z*acch.z);
            acch.scalar -= 0.98;
            
            moth.acc = (moth.acc*moth.count)/(moth.count+1) + acch.scalar/(moth.count+1);
            moth.count++;

            cmot.acc = moth.acc;
#ifdef DBG_PRINT_MOTION
            DBG_PRINT("ACC:%f, AVG:%f\n", acch.scalar, moth.acc);
#endif

        }
        if (sensors & INV_WXYZ_QUAT) {
            inv_build_quat(quat, 0, sensor_timestamp);
            new_data = 1;
        }
        if (new_data) {
            inv_execute_on_data();
            /* This function reads bias-compensated sensor data and sensor
             * fusion outputs from the MPL. The outputs are formatted as seen
             * in eMPL_outputs.c. This function only needs to be called at the
             * rate requested by the host.
             */
            read_from_mpl();
            new_data = 0;
        }
    } while(more);
}
#ifdef _XJ_USING_ENV
static struct BMP180_EEPROM  bmp180_ee;
static struct BMP180_Context bmp180_context = 
{
    .ee = &bmp180_ee,
    .B5 = 0,
    .oss = BMP180_OSS_3,
};

static void process_env(void)
{
    rt_uint16_t BMP180_UT;
    rt_uint32_t BMP180_UP;
    float BMP180_T;
    rt_int32_t BMP180_P;
    rt_err_t err;
    struct cache_env env;
    
    err = SHT20_Get_T(&env.t);
    if (err == RT_EOK)
    {
        err = SHT20_Get_RH(&env.rh);
    }
    if (err == RT_EOK)
    {
        err = BMP180_Get_T(&BMP180_UT);
    }
    if (err == RT_EOK)
    {
        err = BMP180_Get_P(bmp180_context.oss, &BMP180_UP);
    }
    if (err == RT_EOK)
    {
        BMP180_T = BMP180_Caculate_T(&bmp180_context, BMP180_UT);
        BMP180_P = BMP180_Caculate_P(&bmp180_context, BMP180_UP);
        env.hpa  = BMP180_P/100.0;
        env.alt  = BMP180_Caculate_Altitude(BMP180_P);

        cache_box_put(&cbox, CACHE_DATA_ENV, (void*)&env, sizeof(struct cache_env));
        env_notify();
    }
}
#endif

static void run()
{
    inv_error_t result;
    unsigned char accel_fsr,  new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;
    rt_uint16_t sens;
    
    _mot_flag = 0;
    _mot.acc = 0;
    _mot.step_count = 0;
    _mot.walk_time = 0;
#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
    unsigned short compass_fsr;
#endif

    rt_thread_delay(20);

    result = mpu_init(&int_param);
    XASSERT(0 == result);
    if (result)
    {
        MPL_LOGE("Could not initialize gyro.\n");
    }

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    result = inv_init_mpl();
    XASSERT(0 == result);
    if (result)
    {
        MPL_LOGE("Could not initialize MPL.\n");
    }
    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();

    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    XASSERT(0 == result);
    if (result == INV_ERROR_NOT_AUTHORIZED)
    {
        while (1)
        {
            MPL_LOGE("Not authorized.\n");
        }
    }
    if (result)
    {
        MPL_LOGE("Could not start the MPL.\n");
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    XASSERT(0 == result);
#else
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    XASSERT(0 == result);
#endif
    /* Push both gyro and accel data into the FIFO. */
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    XASSERT(0 == result);

    result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    XASSERT(0 == result);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    //result = mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
    result = mpu_set_compass_sample_rate(DEFAULT_MPU_HZ);
    XASSERT(0 == result);
#endif
    /* Read back configuration in case it was set improperly. */
    result = mpu_get_sample_rate(&gyro_rate);
    XASSERT(0 == result);

    result = mpu_get_gyro_fsr(&gyro_fsr);
    XASSERT(0 == result);

    result = mpu_get_accel_fsr(&accel_fsr);
    XASSERT(0 == result);
#ifdef COMPASS_ENABLED
    result = mpu_get_compass_fsr(&compass_fsr);
    XASSERT(0 == result);
#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
#endif
    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = PRINT_ACCEL | PRINT_GRAVITY_VECTOR| PRINT_LINEAR_ACCEL| PRINT_PEDO;
    //hal.report = PRINT_ACCEL | PRINT_EULER | PRINT_PEDO;
    hal.report = PRINT_COMPASS | PRINT_HEADING | PRINT_GYRO | PRINT_EULER | PRINT_QUAT| PRINT_ACCEL;
    hal.report = PRINT_ACCEL | PRINT_PEDO;
    
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /* Compass reads are handled by scheduler. */
    get_tick_count(&timestamp);

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    XASSERT(0 == dmp_load_motion_driver_firmware() );
    XASSERT(0 == dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation)) );
    XASSERT(0 == dmp_register_tap_cb(tap_cb) );
    XASSERT(0 == dmp_register_android_orient_cb(android_orient_cb) );
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    result = dmp_enable_feature(hal.dmp_features);
    XASSERT(0 == result);

    XASSERT(0 == dmp_set_fifo_rate(DEFAULT_MPU_HZ) );
    XASSERT(0 == mpu_set_dmp_state(1) );
    hal.dmp_on = 1;
    XASSERT(mpu_get_accel_sens(&sens) == 0);
    acch.sens = (MOT_DATA_TYPE)sens;

#ifdef DBG_PRINT_MOTION
    DBG_PRINT("MPU9250 MPL ready...\n");
#endif
    
    rt_timer_start(&env_timer);
    while(1)
    {
        rt_uint32_t recv = 0;

        rt_event_recv(&event, 
            EVENT_MOT_INT|EVENT_ENV_TIMER, 
            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
            RT_WAITING_FOREVER, 
            &recv);

        if (recv&EVENT_MOT_INT)
        {
            process_motion();
        }
        if (recv&EVENT_ENV_TIMER)
        {
#ifdef _XJ_USING_ENV
            process_env();
#endif
        }
    }
}
static void env_timeout(void* parameter)
{
    rt_event_send(&event, EVENT_ENV_TIMER);
}

static void thread_entry(void* parameter)
{
    rt_err_t err;
    rt_device_t i2c1 = RT_NULL;

    /* initialize semaphore for mpu interrupt notification */
    //err = rt_sem_init(&sem, "mot_sem", 0, RT_IPC_FLAG_FIFO);
    //XASSERT(err == RT_EOK);

    err = rt_event_init(&event, "motev", RT_IPC_FLAG_FIFO);
    XASSERT(err == RT_EOK);

    rt_timer_init(&env_timer, "envt", env_timeout, 0, RT_TICK_PER_SECOND*10, RT_TIMER_FLAG_PERIODIC);

    
    /* open I2C bus */
    i2c1 = rt_device_find(I2C1_BUS_NAME);
    XASSERT(i2c1 != RT_NULL);
    err = rt_device_open(i2c1, RT_DEVICE_OFLAG_RDWR);
    XASSERT(err == RT_EOK);

    /* init EXIT interrupt */
    init_EXTI();

    /* power on sensors */
    //power_on_sensor();

#ifdef _XJ_USING_ENV
    /* init BMP180 */
    err = BMP180_Get_EEPROM(&bmp180_ee);
    if (err != RT_EOK)
    {
        DBG_PRINT("Initialize BMP180 failed!\n");
    }
#endif

    run();
    while(1);
}

/* thread */
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t thread_stack[ 1024*2 ];
static struct rt_thread thread_handle;
int start_mot_thread(void)
{
    rt_err_t err;
    err = rt_thread_init(&thread_handle,
                         "MPL",
                         thread_entry,
                         RT_NULL,
                         (rt_uint8_t*)&thread_stack[0],
                         sizeof(thread_stack),
                         19,
                         20);
    if (err != RT_EOK)
    {
        return err;
    }

    return rt_thread_startup(&thread_handle);
}
