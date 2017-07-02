/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"


#define SERIAL_RECEIVER
#define WAVESHARE_BOARD

#if defined(WAVESHARE_BOARD)
#define WAVESHARE_LED_1                                  18
#define WAVESHARE_LED_2                                  19
#define WAVESHARE_LED_3                                  20
#define WAVESHARE_LED_4                                  21
#define WAVESHARE_LED_5                                  22
#endif

#if 1
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define START_STRING                                     "Start...\r\n"

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
#endif

#if defined(SERIAL_RECEIVER)

// receiver type
#define SERIAL_RX_SPEKTRUM_1024
//#define SERIAL_RX_SPEKTRUM_2048
// #define SERIAL_RX_S_BUS
// #define SERIAL_RX_SUMD
// #define SERIAL_RX_SUMH
// #define SERIAL_RX_MODE_B
// #define SERIAL_RX_MODE_B_BJ01
// #define SERIAL_RX_IBUS
// #define SERIAL_RX_MSP

#define RX_CHANNEL_ROLL                                  0
#define RX_CHANNEL_PITCH                                 1
#define RX_CHANNEL_YAW                                   2
#define RX_CHANNEL_THROTTLE                              3
#define RX_CHANNEL_ARMING                                4

#if defined(SERIAL_RX_SPEKTRUM_1024)

// UART
// BAUDRATE : 125000 bps, or 1152000 bps
// DATA     : 8 bits
// PARITY   : NO Parity
// stop bit : 1 stop

#define SERIAL_RX_BAUDRATE                               (UART_BAUDRATE_BAUDRATE_Baud115200)
#define DEVICE_NAME                                      "RX-SPEKTRUM-1024"         /**< Name of device. Will be included in the advertising data. */
// 22 ms : (32768 * 22) / 1000
#define SERIAL_RX_UPDATE_TICK_COUNT                      720

#define SERIAL_RX_ROLL_MIN                               0
#define SERIAL_RX_ROLL_MAX                               1023
#define SERIAL_RX_ROLL_DEFAULT                           512

#define SERIAL_RX_PITCH_MIN                              0
#define SERIAL_RX_PITCH_MAX                              1023
#define SERIAL_RX_PITCH_DEFAULT                          512

#define SERIAL_RX_YAW_MIN                                0
#define SERIAL_RX_YAW_MAX                                1023
#define SERIAL_RX_YAW_DEFAULT                            512

#define SERIAL_RX_THROTTLE_MIN                           0
#define SERIAL_RX_THROTTLE_MAX                           1023
#define SERIAL_RX_THROTTLE_DEFAULT                       100

#elif defined(SERIAL_RX_SPEKTRUM_2048)

// UART
// BAUDRATE : 125000 bps, or 1152000 bps
// DATA     : 8 bits
// PARITY   : NO Parity
// stop bit : 1 stop

#define SERIAL_RX_BAUDRATE                               (UART_BAUDRATE_BAUDRATE_Baud115200)
#define DEVICE_NAME                                      "RX-SPEKTRUM-2048"         /**< Name of device. Will be included in the advertising data. */
// 11 ms : (32768 * 11) / 1000
#define SERIAL_RX_UPDATE_TICK_COUNT                      360

#define SERIAL_RX_ROLL_MIN                               0
#define SERIAL_RX_ROLL_MAX                               2047
#define SERIAL_RX_ROLL_DEFAULT                           1024

#define SERIAL_RX_PITCH_MIN                              0
#define SERIAL_RX_PITCH_MAX                              2047
#define SERIAL_RX_PITCH_DEFAULT                          1024

#define SERIAL_RX_YAW_MIN                                0
#define SERIAL_RX_YAW_MAX                                2047
#define SERIAL_RX_YAW_DEFAULT                            1024

#define SERIAL_RX_THROTTLE_MIN                           0
#define SERIAL_RX_THROTTLE_MAX                           2047
#define SERIAL_RX_THROTTLE_DEFAULT                       200

#elif defined(SERIAL_RX_S_BUS)
#define DEVICE_NAME                                      "RX-S-BUS"                 /**< Name of device. Will be included in the advertising data. */
#elif defined(SERIAL_RX_SUMD)
#define DEVICE_NAME                                      "RX-SUMD"                  /**< Name of device. Will be included in the advertising data. */
#elif defined(SERIAL_RX_SUMH)
#define DEVICE_NAME                                      "RX-SUMH"                  /**< Name of device. Will be included in the advertising data. */
#elif defined(SERIAL_RX_MODE_B)
#define DEVICE_NAME                                      "RX-MODE-B"                /**< Name of device. Will be included in the advertising data. */
#elif defined(SERIAL_RX_MODE_B_BJ01)
#define DEVICE_NAME                                      "RX-MODE-B-BJ01"           /**< Name of device. Will be included in the advertising data. */
#elif defined(SERIAL_RX_IBUS)
#define DEVICE_NAME                                      "RX-MODE-IBUS"             /**< Name of device. Will be included in the advertising data. */
#elif defined(SERIAL_RX_MSP)
#define DEVICE_NAME                                      "RX-MODE-MSP"              /**< Name of device. Will be included in the advertising data. */
#else
#define DEVICE_NAME                                      "Nordic_UART"              /**< Name of device. Will be included in the advertising data. */
#endif


// -----------------------------------------------------------------------------
// protocol for phone and bt's receiver
#define SERIAL_BT_COMMAND_MAX_SIZE                       32
static uint8_t g_bt_rx_command[SERIAL_BT_COMMAND_MAX_SIZE];

typedef struct _BT_PROTOCOL_DATA
{
   uint8_t version_high;
   uint8_t version_low;
   uint8_t command;
   uint8_t size;
   uint16_t channel_1;                                   // ROLL
   uint16_t channel_2;                                   // PITCH
   uint16_t channel_3;                                   // YAW
   uint16_t channel_4;                                   // THROTTLE
   uint16_t channel_5;                                   // GEAR
   uint16_t channel_6;                                   // AUX_1
   uint16_t channel_7;                                   // AUX_2
   uint16_t channel_8;                                   // AUX_3
   uint16_t channel_9;                                   // AUX_4
   uint16_t channel_10;                                  // AUX_5
   uint16_t channel_11;                                  // AUX_6
   uint16_t channel_12;                                  // AUX_7
   uint16_t crc;                                         // CRC
} __attribute__ ((__packed__)) BT_PROTOCOL_DATA;

typedef struct _BT_PROTOCOL_RESPONSE
{
   uint8_t version_high;
   uint8_t version_low;
   uint8_t command;
   uint8_t size;
   uint16_t status;
   uint16_t crc;                                         // CRC
} __attribute__ ((__packed__)) BT_PROTOCOL_RESPONSE;

#define  PROTOCOL_SEND_TO_TRANSMITTER                    0x01
#define  PROTOCOL_RESPONSE_FROM_TRANSMITTER              0x02
#define  PROTOCOL_ALIVE_FROM_TRANSMITTER                 0x03

static uint8_t g_bt_rx_command_complete                  = 0;

static uint32_t g_prev_tick_count                        = 0;
static uint32_t g_update_tick_count                      = 0;

static uint8_t g_update_packet_complete                  = 0;
// -----------------------------------------------------------------------------

typedef struct
{
   uint8_t channel_id;
   uint16_t min;
   uint16_t max;
   uint16_t failsafe_value;
   uint16_t value;
} RX_CHANNEL;

static RX_CHANNEL g_rx_channel[]                         =
{
   { RX_CHANNEL_ROLL,         SERIAL_RX_ROLL_MIN,     SERIAL_RX_ROLL_MAX,     SERIAL_RX_ROLL_DEFAULT,       SERIAL_RX_ROLL_DEFAULT},
   { RX_CHANNEL_PITCH,        SERIAL_RX_PITCH_MIN,    SERIAL_RX_PITCH_MAX,    SERIAL_RX_PITCH_DEFAULT,      SERIAL_RX_PITCH_DEFAULT},
   { RX_CHANNEL_YAW,          SERIAL_RX_YAW_MIN,      SERIAL_RX_YAW_MAX,      SERIAL_RX_YAW_DEFAULT,        SERIAL_RX_YAW_DEFAULT},
   { RX_CHANNEL_THROTTLE,     SERIAL_RX_THROTTLE_MIN, SERIAL_RX_THROTTLE_MAX, SERIAL_RX_THROTTLE_DEFAULT,   SERIAL_RX_THROTTLE_DEFAULT},
   { RX_CHANNEL_ARMING,       0,                      1023,                   0,                            0},
};

#if 1

// #define  PROTOCOL_SEND_TO_TRANSMITTER                    0x01
// #define  PROTOCOL_RESPONSE_FROM_TRANSMITTER              0x02
// #define  PROTOCOL_ALIVE_FROM_TRANSMITTER                 0x03

uint8_t update_receiver_command()
{
   uint32_t err_code;
   BT_PROTOCOL_DATA* pProtocolData                          = (BT_PROTOCOL_DATA*) g_bt_rx_command;
   BT_PROTOCOL_RESPONSE reponse;

   reponse.version_high                                     = pProtocolData->version_high;
   reponse.version_low                                      = pProtocolData->version_low;
   reponse.command                                          = PROTOCOL_RESPONSE_FROM_TRANSMITTER;
   reponse.size                                             = sizeof(BT_PROTOCOL_RESPONSE);
   reponse.status                                           = 0;
   reponse.crc                                              = pProtocolData->crc;

   err_code                                                 = ble_nus_string_send(&m_nus, (uint8_t *) &reponse, sizeof(BT_PROTOCOL_RESPONSE));
//   err_code                                                 = ble_nus_string_send(&m_nus, (uint8_t *) "test data   ", 9);
   if (err_code != NRF_ERROR_INVALID_STATE)
   {
      APP_ERROR_CHECK(err_code);
   }

   return 0;
}
#else
uint8_t update_receiver_command()
{
static uint8_t debug_led                                 = 0;
   uint16_t header                                       = (g_bt_rx_command[0] << 8) | g_bt_rx_command[1];

   if (header != 0x2392)
   {
      nrf_gpio_pin_write(WAVESHARE_LED_1, 1);
      return -1;
   }
   
   g_rx_channel[RX_CHANNEL_ROLL].value                   = (g_bt_rx_command[2] << 8) | g_bt_rx_command[3];
   g_rx_channel[RX_CHANNEL_PITCH].value                  = (g_bt_rx_command[4] << 8) | g_bt_rx_command[5];
   g_rx_channel[RX_CHANNEL_THROTTLE].value               = (g_bt_rx_command[6] << 8) | g_bt_rx_command[7];
   g_rx_channel[RX_CHANNEL_YAW].value                    = (g_bt_rx_command[8] << 8) | g_bt_rx_command[9];
   g_rx_channel[RX_CHANNEL_ARMING].value                 = (g_bt_rx_command[10] << 8) | g_bt_rx_command[11];

   if (g_rx_channel[RX_CHANNEL_ARMING].value != 0)
   {
      g_rx_channel[RX_CHANNEL_ARMING].value              = 1023;
   }

   if (debug_led == 0)
   {
      debug_led                                          = 1;
   }
   else
   {
      debug_led                                          = 0;
   }
   nrf_gpio_pin_write(WAVESHARE_LED_2, debug_led);
   return 0;
}
#endif
#if defined(SERIAL_RX_SPEKTRUM_1024)

#define MASK_1024_CHANID                                 0xFC00
#define MASK_1024_SXPOS                                  0x03FF

#define SPEKTRUM_1024_CHANNEL_SHIFT_BITS                 10

#define MAX_CHANNEL_COUNT                                7

#define DSM2_22MS                                        0x01                    // 1024
#define DSM2_11MS                                        0x12                    // 2048
#define DSMS_22MS                                        0xA2                    // 2048
#define DSMX_11MS                                        0xB2                    // 2048

#if 1
#define SPEKTRUM_CHANNEL_THROTTLE                        3
#define SPEKTRUM_CHANNEL_AILERON                         0
#define SPEKTRUM_CHANNEL_ELEVATOR                        1
#define SPEKTRUM_CHANNEL_RUDDER                          2
#else
#define SPEKTRUM_CHANNEL_THROTTLE                        0
#define SPEKTRUM_CHANNEL_AILERON                         1
#define SPEKTRUM_CHANNEL_ELEVATOR                        2
#define SPEKTRUM_CHANNEL_RUDDER                          3
#endif
#define SPEKTRUM_CHANNEL_GEAR                            4
#define SPEKTRUM_CHANNEL_AUX_1                           5
#define SPEKTRUM_CHANNEL_AUX_2                           6
#define SPEKTRUM_CHANNEL_AUX_3                           7
#define SPEKTRUM_CHANNEL_AUX_4                           8
#define SPEKTRUM_CHANNEL_AUX_5                           9
#define SPEKTRUM_CHANNEL_AUX_6                           10
#define SPEKTRUM_CHANNEL_AUX_7                           11

typedef struct
{
   uint8_t fades;
   uint8_t system;
   uint16_t channel[MAX_CHANNEL_COUNT];
} RX_PACKET_SPEKTRUM_1024;

#define RX_PACKET_SPEKTRUM_CHANNEL_ID_0                  0
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_1                  1
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_2                  2
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_3                  3
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_4                  4
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_5                  5
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_6                  6


#define UART_RETRY_SEND_COUNT                            5
uint8_t send_receiver_command()
{
   RX_PACKET_SPEKTRUM_1024 packet;
   uint8_t retry_count;
   uint8_t* stream_packet                                = (uint8_t*) &packet;
   int i;
   memset(&packet, 0, sizeof(RX_PACKET_SPEKTRUM_1024));

   UNUSED_VARIABLE(retry_count);
   UNUSED_VARIABLE(i);

   // missed frames for remote
   // always is null
   packet.fades                                          = 0;
   packet.system                                         = 0;

   // throttle (드론 상승, 하강)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0]       = (MASK_1024_CHANID & (SPEKTRUM_CHANNEL_THROTTLE << SPEKTRUM_1024_CHANNEL_SHIFT_BITS))       |
                                                           (g_rx_channel[RX_CHANNEL_THROTTLE].value & MASK_1024_SXPOS);

   // Aileron (좌측 이동, 우측 이동)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1]       = (MASK_1024_CHANID & (SPEKTRUM_CHANNEL_AILERON << SPEKTRUM_1024_CHANNEL_SHIFT_BITS))        |
                                                           (g_rx_channel[RX_CHANNEL_ROLL].value & MASK_1024_SXPOS);

   // Elevator (전진, 후진)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2]       = (MASK_1024_CHANID & (SPEKTRUM_CHANNEL_ELEVATOR << SPEKTRUM_1024_CHANNEL_SHIFT_BITS))       |
                                                           (g_rx_channel[RX_CHANNEL_PITCH].value & MASK_1024_SXPOS);

   // Rudder (시계방향 회전, 반 시계방향 회전)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3]       = (MASK_1024_CHANID & (SPEKTRUM_CHANNEL_RUDDER << SPEKTRUM_1024_CHANNEL_SHIFT_BITS))         |
                                                           (g_rx_channel[RX_CHANNEL_YAW].value & MASK_1024_SXPOS);

   // GEAR
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4]       = (MASK_1024_CHANID & (SPEKTRUM_CHANNEL_GEAR << SPEKTRUM_1024_CHANNEL_SHIFT_BITS))           |
                                                           (g_rx_channel[RX_CHANNEL_ARMING].value & MASK_1024_SXPOS);
   // ADC 1
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5]       = (MASK_1024_CHANID & (SPEKTRUM_CHANNEL_AUX_2 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS))          |
                                                           (0x200 & MASK_1024_SXPOS);

   // ADC 2
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6]       = (MASK_1024_CHANID & (SPEKTRUM_CHANNEL_AUX_3 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS))          |
                                                           (0x200 & MASK_1024_SXPOS);

   static uint8_t debug_count                            = 0;
   static uint8_t led_count                              = 0;
   ++debug_count;

   if ((debug_count % 100) == 0)
   {
      debug_count                                        = 0;
      if (led_count == 0)
      {
         led_count                                       = 1;
      }
      else
      {
         led_count                                       = 0;
      }
      nrf_gpio_pin_write(WAVESHARE_LED_3, led_count);
   }

#if 0
   {
      printf("\r\n");
      for (i = 0; i < sizeof(RX_PACKET_SPEKTRUM_1024); i++)
      {
         printf("[%02x] ", *(stream_packet + i));
      }
  }
#endif

#if 1
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 1)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 0)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 3)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 2)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 5)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 4)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 7)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 6)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 9)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 8)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 11)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 10)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 13)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 12)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 15)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + 14)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }

#else
   for (i = 0; i < sizeof(RX_PACKET_SPEKTRUM_1024); i++)
   {
      retry_count                                        = 0;
      while(app_uart_put(*(stream_packet + i)) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
            return 1;
         }
      }
   }
#endif

#if 0
   {
      printf("\r\n");
      printf("[%02x] [%02x] ",   packet.fades, packet.system);
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5]     ) & 0xFF));
      printf("[%02x] [%02x] ",   (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6]     ) & 0xFF));
   }
#endif
   return 0;
}

#elif defined(SERIAL_RX_SPEKTRUM_2048)

#define MASK_2048_PHASE                                  0x8000
#define MASK_2048_CHANID                                 0x7800
#define MASK_2048_SXPOS                                  0x07FF

#define SPEKTRUM_2048_CHANNEL_SHIFT_BITS                 11

#define MAX_CHANNEL_COUNT                                7

#define DSM2_22MS                                        0x01                    // 1024
#define DSM2_11MS                                        0x12                    // 2048
#define DSMS_22MS                                        0xA2                    // 2048
#define DSMX_11MS                                        0xB2                    // 2048

#define SPEKTRUM_CHANNEL_THROTTLE                        0
#define SPEKTRUM_CHANNEL_AILERON                         1
#define SPEKTRUM_CHANNEL_ELEVATOR                        2
#define SPEKTRUM_CHANNEL_RUDDER                          3
#define SPEKTRUM_CHANNEL_GEAR                            4
#define SPEKTRUM_CHANNEL_AUX_1                           5
#define SPEKTRUM_CHANNEL_AUX_2                           6
#define SPEKTRUM_CHANNEL_AUX_3                           7
#define SPEKTRUM_CHANNEL_AUX_4                           8
#define SPEKTRUM_CHANNEL_AUX_5                           9
#define SPEKTRUM_CHANNEL_AUX_6                           10
#define SPEKTRUM_CHANNEL_AUX_7                           11

typedef struct
{
   uint8_t fades;
   uint8_t system;
   uint16_t channel[MAX_CHANNEL_COUNT];
} RX_PACKET_SPEKTRUM_1024;

#define RX_PACKET_SPEKTRUM_CHANNEL_ID_0                  0
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_1                  1
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_2                  2
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_3                  3
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_4                  4
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_5                  5
#define RX_PACKET_SPEKTRUM_CHANNEL_ID_6                  6

uint8_t send_receiver_command()
{
   RX_PACKET_SPEKTRUM_1024 packet;
   memset(&packet, 0, sizeof(RX_PACKET_SPEKTRUM_1024));

   // missed frames for remote
   // always is null
   packet.fades                                          = 0;
   // type of receiver
   packet.system                                         = DSMX_11MS;

   // throttle (드론 상승, 하강)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0]       = (MASK_2048_CHANID & (SPEKTRUM_CHANNEL_THROTTLE << SPEKTRUM_2048_CHANNEL_SHIFT_BITS))       |
                                                           (g_rx_channel[RX_CHANNEL_THROTTLE].value & MASK_2048_SXPOS);

   // Aileron (좌측 이동, 우측 이동)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1]       = (MASK_2048_CHANID & (SPEKTRUM_CHANNEL_AILERON << SPEKTRUM_2048_CHANNEL_SHIFT_BITS))        |
                                                           (g_rx_channel[RX_CHANNEL_ROLL].value & MASK_2048_SXPOS);

   // Elevator (전진, 후진)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2]       = (MASK_2048_CHANID & (SPEKTRUM_CHANNEL_ELEVATOR << SPEKTRUM_2048_CHANNEL_SHIFT_BITS))       |
                                                           (g_rx_channel[RX_CHANNEL_PITCH].value & MASK_2048_SXPOS);

   // Rudder (시계방향 회전, 반 시계방향 회전)
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3]       = (MASK_2048_CHANID & (SPEKTRUM_CHANNEL_RUDDER << SPEKTRUM_2048_CHANNEL_SHIFT_BITS))         |
                                                           (g_rx_channel[RX_CHANNEL_YAW].value & MASK_2048_SXPOS);

   // GEAR
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4]       = (MASK_2048_CHANID & (SPEKTRUM_CHANNEL_GEAR << SPEKTRUM_2048_CHANNEL_SHIFT_BITS))           |
                                                           (0x400 & MASK_2048_SXPOS);
   // ADC 1
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5]       = (MASK_2048_CHANID & (SPEKTRUM_CHANNEL_AUX_2 << SPEKTRUM_2048_CHANNEL_SHIFT_BITS))          |
                                                           (0x400 & MASK_2048_SXPOS);

   // ADC 2
   packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6]       = (MASK_2048_CHANID & (SPEKTRUM_CHANNEL_AUX_3 << SPEKTRUM_2048_CHANNEL_SHIFT_BITS))          |
                                                           (0x400 & MASK_2048_SXPOS);

#if 0
   {
      printf("\r\n");
      printf("[%04x] ", packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0]);
      printf("[%04x] ", packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1]);
      printf("[%04x] ", packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2]);
      printf("[%04x] ", packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3]);
      printf("[%04x] ", packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4]);
      printf("[%04x] ", packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5]);
      printf("[%04x] ", packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6]);
   }
#endif

#if 1
   // fades
   while(app_uart_put(packet.fades) != NRF_SUCCESS)
   {
   }
   // system
   while(app_uart_put(packet.system) != NRF_SUCCESS)
   {
   }
   
   // throttle
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0] >> 8) & 0xFF)) != NRF_SUCCESS)
   {
   }
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0]     ) & 0xFF)) != NRF_SUCCESS)
   {
   }

   // aileron
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1] >> 8) & 0xFF)) != NRF_SUCCESS)
   {
   }
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1]     ) & 0xFF)) != NRF_SUCCESS)
   {
   }

   // aileron
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2] >> 8) & 0xFF)) != NRF_SUCCESS)
   {
   }
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2]     ) & 0xFF)) != NRF_SUCCESS)
   {
   }

   // rudder
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3] >> 8) & 0xFF)) != NRF_SUCCESS)
   {
   }
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3]     ) & 0xFF)) != NRF_SUCCESS)
   {
   }

   // adc 1
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4] >> 8) & 0xFF)) != NRF_SUCCESS)
   {
   }
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4]     ) & 0xFF)) != NRF_SUCCESS)
   {
   }

   // adc 2
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5] >> 8) & 0xFF)) != NRF_SUCCESS)
   {
   }
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5]     ) & 0xFF)) != NRF_SUCCESS)
   {
   }

   // adc 3
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6] >> 8) & 0xFF)) != NRF_SUCCESS)
   {
   }
   while(app_uart_put((uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6]     ) & 0xFF)) != NRF_SUCCESS)
   {
   }
#endif

#if 0
   {
      printf("\r\n");
      printf("[%02x] [%02x] ",   packet.fades, packet.system);
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_0]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_1]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_2]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_3]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_4]     ) & 0xFF));
      printf("[%02x] [%02x] : ", (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_5]     ) & 0xFF));
      printf("[%02x] [%02x] ",   (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6] >> 8) & 0xFF),
                                 (uint8_t) ((packet.channel[RX_PACKET_SPEKTRUM_CHANNEL_ID_6]     ) & 0xFF));
   }
#endif
   return 0;
}

#elif defined(SERIAL_RX_S_BUS)

uint8_t send_receiver_command()
{
   return 0;
}

#elif defined(SERIAL_RX_SUMD)

uint8_t send_receiver_command()
{
   return 0;
}

#elif defined(SERIAL_RX_SUMH)

uint8_t send_receiver_command()
{
   return 0;
}

#elif defined(SERIAL_RX_MODE_B)

uint8_t send_receiver_command()
{
   return 0;
}

#elif defined(SERIAL_RX_MODE_B_BJ01)

uint8_t send_receiver_command()
{
   return 0;
}

#elif defined(SERIAL_RX_IBUS)

uint8_t send_receiver_command()
{
   return 0;
}

#elif defined(SERIAL_RX_MSP)

uint8_t send_receiver_command()
{
   return 0;
}

#else

uint8_t send_receiver_command()
{
   return 0;
}

#endif

void serial_receiver_timer_handler(void * p_context)
{
   uint32_t cur_tick_count                               = 0;
   uint32_t diff_tick_count                              = 0;
   if (g_prev_tick_count == 0)
   {
      if (app_timer_cnt_get(&g_prev_tick_count) != NRF_SUCCESS)
      {
         printf("Can't get tick count \r\n");
      }
      return;
   }
   if (app_timer_cnt_get(&cur_tick_count) != NRF_SUCCESS)
   {
      printf("Can't get tick count \r\n");
      return;
   }

   if (app_timer_cnt_diff_compute(cur_tick_count, g_prev_tick_count, &diff_tick_count) != NRF_SUCCESS)
   {
      printf("Can't diff tick count \r\n");
      return;
   }
   g_prev_tick_count                                     = cur_tick_count;

   g_update_tick_count                                   += diff_tick_count;
   if (g_update_tick_count > SERIAL_RX_UPDATE_TICK_COUNT)
   {
      g_update_tick_count                                = 0;
      g_update_packet_complete                           = 1;
   }
}

#endif   // SERIAL_RECEIVER

#if 0
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define START_STRING                                     "Start...\r\n"

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
#endif

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
int copy_byte                                            = 0;
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
   int i;

#if defined(SERIAL_RECEIVER)
   if (g_bt_rx_command_complete == 0)
   {
      for (i = 0; i < SERIAL_BT_COMMAND_MAX_SIZE; i++)
      {
         g_bt_rx_command[i]                              = 0;
      }

      copy_byte                                          = SERIAL_BT_COMMAND_MAX_SIZE > length ? length : SERIAL_BT_COMMAND_MAX_SIZE;
      for (i = 0; i < copy_byte; i++)
      {
         g_bt_rx_command[i]                              = *(p_data + i);
      }

      g_bt_rx_command_complete                           = 1;
   }
#if 0
   char bt_rx_command[SERIAL_BT_COMMAND_MAX_SIZE + 1];
   if (length < SERIAL_BT_COMMAND_MAX_SIZE)
   {
      memset(bt_rx_command, 0, sizeof(bt_rx_command) + 1);
      memcpy(bt_rx_command, p_data, length);
      update_receiver_command(bt_rx_command);
   }
#endif

#else
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
  #if 0
    while(app_uart_put('\n') != NRF_SUCCESS);
  #endif
#endif
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
#if 1
            {
                UNUSED_VARIABLE(err_code);
                index                                        = 0;
                while (app_uart_get(&data_array[index]) == NRF_SUCCESS)
                {
                    if (index >= BLE_NUS_MAX_DATA_LEN)
                    {
                        break;
                    }
                    index++;
                }
                if (index != 0)
                {
                    err_code                                  = ble_nus_string_send(&m_nus, data_array, index);
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                }
            }
#else
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
#endif
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
#if defined(SERIAL_RECEIVER)
        APP_UART_FLOW_CONTROL_DISABLED,
#else
        APP_UART_FLOW_CONTROL_ENABLED,
#endif
        false,
#if defined(SERIAL_RECEIVER)
        SERIAL_RX_BAUDRATE
#else
        UART_BAUDRATE_BAUDRATE_Baud115200
#endif
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
#if !defined(SERIAL_RECEIVER)
    uint8_t start_string[] = START_STRING;
#endif
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
#if defined(SERIAL_RECEIVER)
    APP_TIMER_DEF(serial_rx_timer);
#endif
    uart_init();

    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

#if !defined(SERIAL_RECEIVER)
    printf("%s", start_string);
#else
    printf("program start \r\n ");
#endif
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

#if defined(SERIAL_RECEIVER)
   if (app_timer_create(&serial_rx_timer, APP_TIMER_MODE_REPEATED, serial_receiver_timer_handler) != NRF_SUCCESS)
   {
      printf("Can't create app timer \r\n");
   }
   if (app_timer_start(serial_rx_timer, APP_TIMER_TICKS(1, APP_TIMER_PRESCALER), NULL) != NRF_SUCCESS)
   {
      printf("Can't start app timer \r\n");
   }
#endif

#if defined(WAVESHARE_BOARD)
   nrf_gpio_cfg_output(WAVESHARE_LED_1);
   nrf_gpio_cfg_output(WAVESHARE_LED_2);
   nrf_gpio_cfg_output(WAVESHARE_LED_3);
   nrf_gpio_cfg_output(WAVESHARE_LED_4);
   nrf_gpio_cfg_output(WAVESHARE_LED_5);

   nrf_gpio_pin_write(WAVESHARE_LED_1, 0);
   nrf_gpio_pin_write(WAVESHARE_LED_2, 0);
   nrf_gpio_pin_write(WAVESHARE_LED_3, 0);
   nrf_gpio_pin_write(WAVESHARE_LED_4, 0);
   nrf_gpio_pin_write(WAVESHARE_LED_5, 0);

#endif

    // Enter main loop.
    for (;;)
    {
#if defined(SERIAL_RECEIVER)
      if (g_bt_rx_command_complete != 0)
      {
         update_receiver_command();

         g_bt_rx_command_complete                        = 0;
      }
      if (g_update_packet_complete != 0)
      {
         send_receiver_command();

         g_update_packet_complete                        = 0;
      }
#endif
        power_manage();
    }
}


/**
 * @}
 */
