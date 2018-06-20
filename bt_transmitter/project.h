
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

// -----------------------------------------------------------------------------
#define MILLISECOND                                      1000

// -----------------------------------------------------------------------------
// Protocol v.1.0 - 2018-06-18
#define PROTOCOL_VERSION                                 0x10A0

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
// protocol (20 bytes)
typedef struct _BT_PROTOCOL_V1
{
   // 2 bytes header
   uint16_t header;                                                              // major ver(4bits) - minor ver(4bits) - sub ver(4bits) - command(4bits)

   uint16_t throttle;                                                            // up/down
   uint16_t roll;                                                                // left/right move
   uint16_t pitch;                                                               // forward/backward move
   uint16_t yaw;                                                                 // left/right rotation
   uint16_t ch1;                                                                 // armming
   uint16_t ch2;                                                                 //
   uint16_t ch3;                                                                 //
   uint16_t ch4;                                                                 //
   uint16_t ch5;                                                                 //
} __attribute__ ((__packed__)) BT_PROTOCOL_V1;

// -----------------------------------------------------------------------------
// project
typedef struct _BT_PROJECT
{
   uint32_t tick_count;

   BT_TRANSMITTER_TIME system_time;
   BT_TRANSMITTER_TIMER timer[BT_TRANSMITTER_MAX_TIMER];

   ble_nus_t ble_nordic_uart_service;

   uint8_t bt_connected;
   BT_PROTOCOL_V1 bt_protocol_packet;
} BT_PROJECT;

extern BT_PROJECT g_bt_Project[1];

#define DEBUG_RTT_ERROR
#define DEBUG_RTT_DEBUG

#if defined(DEBUG_RTT_DEBUG)
#define DEBUG_GPIO_PIN                                   22
#define DEBUG_GPIO_EXT_PIN                               24
#endif

#endif   // _PROJECT_H_


