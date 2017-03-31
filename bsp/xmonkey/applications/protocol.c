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

#include "protocol.h"
#include <rtthread.h>

/*
 * p_control
 */
static int p_control_input(struct netio* obj, char* ptr, int len)
{
    int expected = sizeof(unsigned short);
    /*
    struct p_control *p = (struct p_control*)obj;
    unsigned short* tmp = &p->priority;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    *tmp = *((unsigned short)ptr);
    */
    return expected;
}
static int p_control_output(struct netio* obj, char* ptr, int len)
{
    int expected = sizeof(unsigned short);
    /*
    struct p_control *p = (struct p_control*)obj;
    unsigned short* tmp = &p->priority;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    *((unsigned short)ptr) = *tmp;
    */
    return expected;
}
static int p_control_size(struct netio* obj)
{
    return sizeof(unsigned short);
}

void p_control_init(struct p_control* obj)
{
    obj->parent.input  = p_control_input;
    obj->parent.output = p_control_output;
    obj->parent.size   = p_control_size;
    
    obj->ack = 0;
    obj->priority = 0;
}
void p_control_copy(struct p_control* to, struct p_control* from)
{
    to->priority = from->priority;
    to->ack      = from->ack;
}


/*
 * p_devid
 */
static int p_devid_input(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_devid *p = (struct p_devid*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    p->type = *((unsigned char*)ptr);
    n = sizeof(unsigned char);
    ptr += n;

    p->high = *((unsigned int*)ptr);
    n = sizeof(unsigned int);
    ptr += n;

    p->low  = *((unsigned int*)ptr);
    n = sizeof(unsigned int);
    ptr += n;

    return expected;
}
static int p_devid_output(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_devid *p = (struct p_devid*)obj;
    int n;
    
    if (len < expected)
    {
        return -RT_ERROR;
    }
    *((unsigned char*)ptr) = p->type;
    n = sizeof(unsigned char);
    ptr += n;

    *((unsigned int*)ptr)  = p->high;
    n = sizeof(unsigned int);
    ptr += n;

    *((unsigned int*)ptr)  = p->low;
    n = sizeof(unsigned int);
    ptr += n;
    return expected;
}
static int p_devid_size(struct netio* obj)
{
    return sizeof(rt_uint8_t) + sizeof(unsigned int)*2;
}

void p_devid_init(struct p_devid* obj)
{
    obj->parent.input  = p_devid_input;
    obj->parent.output = p_devid_output;
    obj->parent.size   = p_devid_size;
    
    obj->type = 0;
    obj->high = 0;
    obj->low  = 0;
}
void p_devid_copy(struct p_devid* to, struct p_devid* from)
{
    to->type = from->type;
    to->high = from->high;
    to->low  = from->low;
}


/*
 * p_header
 */
#define P_HEAD_MAGIC    (0x585F)
static int p_header_input(struct netio* obj, char* ptr, int len)
{
    struct p_header *p = (struct p_header*)obj;
    int expected = obj->size(obj);
    int n, idle=len;
    unsigned short magic;
    if (len < expected)
    {
        return -RT_ERROR;
    }

    magic = *((unsigned short*)ptr);
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;
    if (magic != P_HEAD_MAGIC)
    {
        return -RT_ERROR;
    }

    p->ver = *((unsigned short*)ptr);
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;

    p->msg = *((unsigned short*)ptr);
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;

    n = p->devid.parent.input((struct netio*)&p->devid, ptr, len);
    if (n < 0)
    {
        return -RT_ERROR;
    }
    ptr += n;
    len -= n;

    p->sn = *((unsigned int*)ptr);
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    p->ack = *((unsigned int*)ptr);
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    p->time = *((unsigned int*)ptr);
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    /*reserved*/
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    n = p->control.parent.input((struct netio*)&p->control, ptr, len);
    if (n < 0)
    {
        return -RT_ERROR;
    }
    ptr += n;
    len -= n;
    
    p->len = *((unsigned short*)ptr);
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;

    if(expected != (idle - len))
    {
        return -RT_ERROR;
    }
    return expected;
}
static int p_header_output(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_header *p = (struct p_header*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    
    *((unsigned short*)ptr) = P_HEAD_MAGIC;
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;

    *((unsigned short*)ptr) = p->ver;
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;

    *((unsigned short*)ptr) = p->msg;
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;

    n = p->devid.parent.output((struct netio*)&p->devid, ptr, len);
    if (n < 0)
    {
        return -RT_ERROR;
    }
    ptr += n;
    len -= n;

    *((unsigned int*)ptr) = p->sn;
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    *((unsigned int*)ptr) = p->ack;
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    *((unsigned int*)ptr) = p->time;
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    /*reserved*/
    *((unsigned int*)ptr) = 0;
    n = sizeof(unsigned int);
    ptr += n;
    len -= n;

    n = p->control.parent.output((struct netio*)&p->control, ptr, len);
    if (n < 0)
    {
        return -RT_ERROR;
    }
    ptr += n;
    len -= n;
    
    *((unsigned short*)ptr) = p->len;
    n = sizeof(unsigned short);
    ptr += n;
    len -= n;    return expected;
}
static int p_header_size(struct netio* obj)
{
    struct p_header *p = (struct p_header*)obj;
    int expected = sizeof(unsigned short) +
                   sizeof(unsigned short) +
                   sizeof(unsigned short) +
                   p->devid.parent.size((struct netio*)&p->devid) + 
                   sizeof(unsigned int) +
                   sizeof(unsigned int) +
                   sizeof(unsigned int) +
                   sizeof(unsigned int) +
                   p->control.parent.size((struct netio*)&p->control) +
                   sizeof(unsigned short);
    return expected;
}

void p_header_init(struct p_header* obj)
{
    obj->parent.input  = p_header_input;
    obj->parent.output = p_header_output;
    obj->parent.size   = p_header_size;
    
    obj->ver        = 0;
    obj->msg        = 0;
    p_devid_init(&obj->devid);
    obj->sn         = 0;
    obj->ack        = 0;
    obj->time       = 0;
    p_control_init(&obj->control);
    obj->len        = 0;
}
void p_header_copy(struct p_header* to, struct p_header* from)
{
    to->ver     = from->ver;
    to->msg  = from->msg;
    p_devid_copy(&to->devid, &from->devid);
    to->sn   = from->sn;
    to->ack  = from->ack;
    to->time = from->time;
    p_control_copy(&to->control, &from->control);
    to->len  = from->len;
}

/*
 * p_synct
 */
static int p_synct_input(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_synct *p = (struct p_synct*)obj;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    p->value = *((unsigned int*)ptr);
    return expected;
}
static int p_synct_output(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_synct *p = (struct p_synct*)obj;
    
    if (len < expected)
    {
        return -RT_ERROR;
    }
    *((unsigned int*)ptr) = p->value;
    return expected;
}
static int p_synct_size(struct netio* obj)
{
    return sizeof(unsigned int);
}

void p_synct_init(struct p_synct* obj)
{
    obj->parent.input  = p_synct_input;
    obj->parent.output = p_synct_output;
    obj->parent.size   = p_synct_size;
    
    obj->value = 0;
}

/*
 * p_gps
 */
static int p_gps_input(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_gps *p = (struct p_gps*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    p->lat = *((float*)ptr);
    n = sizeof(float);
    ptr += n;
    
    p->lon = *((float*)ptr);
    n = sizeof(float);
    ptr += n;

    p->alt = *((float*)ptr);
    n = sizeof(float);
    ptr += n;

    p->speed = *((float*)ptr);
    n = sizeof(float);
    ptr += n;
    
    return expected;
}
static int p_gps_output(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_gps *p = (struct p_gps*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    *((float*)ptr) = p->lat;
    n = sizeof(float);
    ptr += n;
    
    *((float*)ptr) = p->lon;
    n = sizeof(float);
    ptr += n;

    *((float*)ptr) = p->alt;
    n = sizeof(float);
    ptr += n;

    *((float*)ptr) = p->speed;
    n = sizeof(float);
    ptr += n;
    
    return expected;
}
static int p_gps_size(struct netio* obj)
{
    return sizeof(float)*4;
}

void p_gps_init(struct p_gps* obj)
{
    obj->parent.input  = p_gps_input;
    obj->parent.output = p_gps_output;
    obj->parent.size   = p_gps_size;
    
    obj->lat   = 0;
    obj->lon   = 0;
    obj->alt   = 0;
    obj->speed = 0;
}

/*
 * p_motion
 */
static int p_motion_input(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_motion *p = (struct p_motion*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    p->acc = *((float*)ptr);
    n = sizeof(float);
    ptr += n;
    
    p->step = *((unsigned long*)ptr);
    n = sizeof(float);
    ptr += n;

    p->dur = *((unsigned long*)ptr);
    n = sizeof(float);
    ptr += n;

    return expected;
}
static int p_motion_output(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_motion *p = (struct p_motion*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    *((float*)ptr) = p->acc;
    n = sizeof(float);
    ptr += n;
    
    *((unsigned long*)ptr) = p->step;
    n = sizeof(float);
    ptr += n;

    *((unsigned long*)ptr) = p->dur;
    n = sizeof(float);
    ptr += n;

    return expected;
}
static int p_motion_size(struct netio* obj)
{
    return sizeof(float) + sizeof(unsigned long)*2;
}

void p_motion_init(struct p_motion* obj)
{
    obj->parent.input  = p_motion_input;
    obj->parent.output = p_motion_output;
    obj->parent.size   = p_motion_size;
    
    obj->acc   = 0;
    obj->step  = 0;
    obj->dur   = 0;
}

/*
 * p_env
 */
static int p_env_input(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_env *p = (struct p_env*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    p->t = *((float*)ptr);
    n = sizeof(float);
    ptr += n;
    
    p->rh = *((float*)ptr);
    n = sizeof(float);
    ptr += n;

    p->hpa = *((float*)ptr);
    n = sizeof(float);
    ptr += n;

    p->alt = *((float*)ptr);
    n = sizeof(float);
    ptr += n;

    return expected;
}
static int p_env_output(struct netio* obj, char* ptr, int len)
{
    int expected = obj->size(obj);
    struct p_env *p = (struct p_env*)obj;
    int n;
    if (len < expected)
    {
        return -RT_ERROR;
    }
    *((float*)ptr) = p->t;
    n = sizeof(float);
    ptr += n;
    
    *((float*)ptr) = p->rh;
    n = sizeof(float);
    ptr += n;

    *((float*)ptr) = p->hpa;
    n = sizeof(float);
    ptr += n;

    *((float*)ptr) = p->alt;
    n = sizeof(float);
    ptr += n;
    
    return expected;
}
static int p_env_size(struct netio* obj)
{
    return sizeof(float)*4;
}

void p_env_init(struct p_env* obj)
{
    obj->parent.input  = p_env_input;
    obj->parent.output = p_env_output;
    obj->parent.size   = p_env_size;
    
    obj->t     = 0;
    obj->rh    = 0;
    obj->hpa   = 0;
    obj->alt   = 0;
}


