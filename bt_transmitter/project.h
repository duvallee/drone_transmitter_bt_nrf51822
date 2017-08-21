
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

// -----------------------------------------------------------------------------
// time
typedef struct _BT_TRANSMITTER_TIME
{
   uint32_t milli_second;
   uint32_t second;
} BT_TRANSMITTER_TIME;

// -----------------------------------------------------------------------------
// timer
#define BT_TRANSMITTER_MAX_TIMER                         10
typedef void (*BT_TRANSMITTER_TIMER_FN)(void*);
typedef struct _BT_TRANSMITTER_TIMER
{
   uint32_t tick_count;
   uint8_t is_allocate;

   uint32_t ms_elapse;
   int call_count;
   int call_fn_count;

   BT_TRANSMITTER_TIMER_FN fn_timer;

   void* pData;
} BT_TRANSMITTER_TIMER;

// -----------------------------------------------------------------------------
// channel info
#define MAX_CHANNEL_INFO_COUNT                           32
typedef void (*SEND_UART_FN)(uint8_t*, uint8_t);
typedef struct _BT_TRANSMITTER_CHANNEL_INFO
{
   uint8_t channel_num;
   uint8_t channel_update;
   SEND_UART_FN send_uart_fn;
   
   uint32_t channel[MAX_CHANNEL_INFO_COUNT];                                     // high 16 bit : channel id, low 16 bit : value
} BT_TRANSMITTER_CHANNEL_INFO;

typedef struct _BT_PROJECT
{
   uint32_t tick_count;
   BT_TRANSMITTER_TIME system_time;

   BT_TRANSMITTER_TIMER timer[BT_TRANSMITTER_MAX_TIMER];

   BT_TRANSMITTER_CHANNEL_INFO update_channel_data_for_fc;                       // channel info for the flight controller

} BT_PROJECT;

extern BT_PROJECT g_bt_Project[1];
#define DEBUG_RTT_ERROR

#define DEBUG_RTT_DEBUG


#define DEBUG_GPIO_PIN                                   22

#endif   // _PROJECT_H_

