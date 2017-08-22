
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
// Command (byte : 4 bit (phone -> transmitter : 0x0?, transmitter -> phone : 0xF?)
#define PROTOCOL_REGISTER_MESSAGE                        0x01
#define PROTOCOL_REGISTER_RESPONSE                       0xF1
#define PROTOCOL_ALIVE_MESSAGE                           0x02
#define PROTOCOL_ALIVE_RESPONSE                          0xF2
#define PROTOCOL_CHANNEL_MESSAGE                         0x03
#define PROTOCOL_CHANNEL_RESPONSE                        0xF3
#define PROTOCOL_UNKNOWN_RESPONSE                        0xFF

#define PROTOCOL_HEADER_HIGH_SYNC                        0xD7
#define PROTOCOL_HEADER_LOW_SYNC                         0x5E


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


// -----------------------------------------------------------------------------
// protocol

// base
typedef struct _BT_PROTOCOL_BASE
{
   // 4 bytes
   uint8_t high_sync_byte;
   uint8_t low_sync_byte;
   uint8_t command;
   uint8_t size;

   // 4 bytes
   uint8_t option_1_high;
   uint8_t option_1_low;
   uint8_t option_2_high;
   uint8_t option_2_low;
} __attribute__ ((__packed__)) BT_PROTOCOL_BASE;

// channel
typedef struct _BT_PROTOCOL_CHANNEL_DATA
{
   // 8 bytes
   BT_PROTOCOL_BASE base_protocol;

   // 4 bytes
   uint16_t channel_1;                                   // ROLL
   uint16_t channel_2;                                   // PITCH

   // 4 bytes
   uint16_t channel_3;                                   // YAW
   uint16_t channel_4;                                   // THROTTLE

   // 4 bytes
   uint16_t channel_5;                                   // GEAR (Armming)
   uint16_t channel_6;                                   // AUX_1

   // 4 bytes
   uint16_t channel_7;                                   // AUX_2
   uint16_t channel_8;                                   // AUX_3

   // 4 bytes
   uint16_t channel_9;                                   // AUX_4
   uint16_t channel_10;                                  // AUX_5

   // 4 bytes
   uint16_t channel_11;                                  // AUX_6
   uint16_t channel_12;                                  // AUX_7
} __attribute__ ((__packed__)) BT_PROTOCOL_CHANNEL_DATA;

// channel
typedef struct _BT_PROTOCOL_DATA
{
   BT_PROTOCOL_CHANNEL_DATA bt_protocol_channel_data;

   uint8_t remain_packet_size;
   uint8_t received_packet_size;

   uint8_t complete_packet;
} __attribute__ ((__packed__)) BT_PROTOCOL_DATA;


// -----------------------------------------------------------------------------
// project
typedef struct _BT_PROJECT
{
   uint32_t tick_count;
   BT_TRANSMITTER_TIME system_time;

   BT_TRANSMITTER_TIMER timer[BT_TRANSMITTER_MAX_TIMER];

   // received from bt
   BT_PROTOCOL_DATA bt_protocol_data;

   uint32_t alive_receive_tick_count;

   // channel data for fc
   BT_TRANSMITTER_CHANNEL_INFO update_channel_data_for_fc;                       // channel info for the flight controller

} BT_PROJECT;

extern BT_PROJECT g_bt_Project[1];
#define DEBUG_RTT_ERROR

#define DEBUG_RTT_DEBUG


#define DEBUG_GPIO_PIN                                   22

#endif   // _PROJECT_H_

