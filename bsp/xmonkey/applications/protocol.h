/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: Protocol.h
*      Author: xujun
*     Version: 1.0.0
*        Date: 22-04-2016
* Description: 
*              this file defines and implements the protocol of XDemoDevice
*            
* Change Logs:
* Date            Author           Notes
* 22-04-2016      xujun            created
*/
#ifndef __AT_PROTOCOL_H
#define __AT_PROTOCOL_H

struct netio
{
    int (* input)(struct netio*, char* ptr, int len);
    int (*output)(struct netio*, char* ptr, int len);
    int   (*size)(struct netio*);
};

struct p_control
{
    struct netio     parent;
    
    unsigned short   priority:4;
    unsigned short   ack:1;
};
void p_control_init(struct p_control* obj);
void p_control_copy(struct p_control* to, struct p_control* from);


#define P_DEVICE_TYPE_TERM  (1)
#define P_DEVICE_TYPE_SRV   (2)

struct p_devid
{
    struct netio   parent;
    
    unsigned char  type;
    unsigned int   high;
    unsigned int   low;
};
void p_devid_init(struct p_devid* obj);
void p_devid_copy(struct p_devid* to, struct p_devid* from);

struct p_header
{
    struct netio     parent;

    unsigned short   ver;
    unsigned short   msg;
    
    struct p_devid   devid;
    
    unsigned int     sn;
    unsigned int     ack;
    unsigned int     time;
    
    struct p_control control;
    
    unsigned short   len;
};
void p_header_init(struct p_header* obj);
void p_header_copy(struct p_header* to, struct p_header* from);

#define DEV_TYPE_XDEMO          (1)
#define DEV_TYPE_SERVER         (2)


/* SYSTEM message */
#define MSG_SYS_HEART           (0x0101)
#define MSG_SYS_SYNCT           (0x0102)
#define MSG_SYS_REG             (0x0103)


/* SENSOR message */
#define MSG_SENS_GPS            (0x0201)
#define MSG_SENS_MOT            (0x0202)
#define MSG_SENS_ENV            (0x0203)

struct p_synct
{
    struct netio    parent;
    
    unsigned int    value;
};
void p_synct_init(struct p_synct* obj);

struct p_gps
{
    struct netio    parent;
    
    float lat;
        /* north when > 0 */
    float lon;
        /* east when > 0 */
    float alt;
        /* unit in meter */
    float speed;
        /* unit in knot */
};
void p_gps_init(struct p_gps* obj);

struct p_motion
{
    struct netio    parent;
    
    float           acc;
    unsigned long   step;
    unsigned long   dur;
};
void p_motion_init(struct p_motion* obj);

struct p_env
{
    struct netio    parent;
    
    float t;
    float rh;
    float hpa;
    float alt;
};
void p_env_init(struct p_env* obj);


#endif
