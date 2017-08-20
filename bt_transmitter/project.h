
/* Copyright (c) 2017 duvallee.lee. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef _PROJECT_H_
#define _PROJECT_H_

#define MILLISECOND                                      1000

typedef struct _BT_TRANSMITTER_TIME
{
   uint32_t milli_second;
   uint32_t second;
} BT_TRANSMITTER_TIME;


typedef void (*BT_TRANSMITTER_TIMER_FN)(void*);
typedef struct _BT_TRANSMITTER_TIMER
{
   uint32_t tick_count;

   uint32_t ms_elapse;
   int call_count;
   int call_fn_count;

   TIMER_FN fn_timer;

   void* pData;
} BT_TRANSMITTER_TIMER;


typedef struct _BT_PROJECT
{
   BT_TRANSMITTER_TIME system_time;

} BT_PROJECT;


#endif   // _PROJECT_H_

