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

// -----------------------------------------------------------------------------
// for global definition
#define SERIAL_RECEIVER
#define WAVESHARE_BOARD

// -----------------------------------------------------------------------------
// for debugging...
#if defined(WAVESHARE_BOARD)
#define WAVESHARE_LED_1                                  18
#define WAVESHARE_LED_2                                  19
#define WAVESHARE_LED_3                                  20
#define WAVESHARE_LED_4                                  21
#define WAVESHARE_LED_5                                  22
#endif

// -----------------------------------------------------------------------------
// for serial rx transmitter
#define SERIAL_RX_TRANSMITTER_MAX_SIZE                   64

// -----------------------------------------------------------------------------
// protocol for phone and bt's receiver
#define PROTOCOL_BASIC_MAX_SIZE                          8
#define PROTOCOL_CHANNEL_MAX_SIZE                        32

#define PROTOCOL_HEADER_HIGH_VERSION                     0x10
#define PROTOCOL_HEADER_LOW_VERSION                      0x01

// Command (byte : 4 bit (phone -> transmitter : 0x0?, transmitter -> phone : 0xF?)
#define PROTOCOL_REGISTER_MESSAGE                        0x01
#define PROTOCOL_REGISTER_RESPONSE                       0xF1
#define PROTOCOL_ALIVE_MESSAGE                           0x02
#define PROTOCOL_ALIVE_RESPONSE                          0xF2
#define PROTOCOL_CHANNEL_MESSAGE                         0x03
#define PROTOCOL_CHANNEL_RESPONSE                        0xF3
#define PROTOCOL_UNKNOWN_RESPONSE                        0xFF

// channel id
// if it need modified id of channel, must be changed in the android.
#define SPEKTRUM_CHANNEL_ROLL                            0
#define SPEKTRUM_CHANNEL_PITCH                           1
#define SPEKTRUM_CHANNEL_YAW                             2
#define SPEKTRUM_CHANNEL_THROTTLE                        3
#define SPEKTRUM_CHANNEL_GEAR                            4
#define SPEKTRUM_CHANNEL_AUX_1                           5
#define SPEKTRUM_CHANNEL_AUX_2                           6
#define SPEKTRUM_CHANNEL_AUX_3                           7
#define SPEKTRUM_CHANNEL_AUX_4                           8
#define SPEKTRUM_CHANNEL_AUX_5                           9
#define SPEKTRUM_CHANNEL_AUX_6                           10
#define SPEKTRUM_CHANNEL_AUX_7                           11
#define SPEKTRUM_MAX_CHANNEL                             12


// command & response
typedef struct _BT_PROTOCOL_COMMAND
{
   // 4 bytes
   uint8_t version_high;
   uint8_t version_low;
   uint8_t command;
   uint8_t size;

   // 4 bytes
   uint8_t option_1_high;
   uint8_t option_1_low;
   uint8_t option_2_high;
   uint8_t option_2_low;
} __attribute__ ((__packed__)) BT_PROTOCOL_COMMAND;

// channel
typedef struct _BT_PROTOCOL_CHANNEL_DATA
{
   // 8 bytes
   BT_PROTOCOL_COMMAND base_protocol;

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
} __attribute__ ((__packed__)) BT_PROTOCOL_DATA;

// -----------------------------------------------------------------------------
enum BT_COMMAND_STATUS
{
   BT_COMMAND_READY                                      = 0,
   BT_COMMAND_RECEIVED                                   = 1,
   BT_COMMAND_PROCESS                                    = 3,
};

// -----------------------------------------------------------------------------
enum PROJECT_MODE
{
   PROJECT_INIT_STATUS                                   = 0,
   PROJECT_READY_STATUS                                  = 1,
   PROJECT_NORMAL_STATUS                                 = 2,
   PROJECT_FAILSAFE_STATUS                               = 3,
   
   PROJECT_FAILED                                        = -1,
};

// -----------------------------------------------------------------------------
enum RESPONSE_ERROR_CODE
{
   SUCCESS_RESPONSE                                      = 0,
   ERROR_RESPONSE_VERSION                                = 1,
   ERROR_RESPONSE_CRC_ERROR                              = 2,
   ERROR_RESPONSE_CHANNEL_FAILED                         = 3,
   ERROR_ALREADY_REGISTERED                              = 4,
   ERROR_UNKNOWN_COMMAND                                 = 5,
};

// -----------------------------------------------------------------------------
// for timer
#define MAX_TIMER_COUNT                                  10
typedef void (*TIMER_FN)(void*);
typedef struct _TIMER
{
   uint32_t tick_count;

   uint32_t ms_elapse;
   int call_count;
   int call_fn_count;

   TIMER_FN fn_timer;

   void* pData;
} TIMER;


// -----------------------------------------------------------------------------
// for project
typedef struct _PROJECT
{
   // remote controller status
   uint8_t mode;
   // timer tick count
   uint32_t tick_count;

   // check alive
   uint32_t last_alive_tick_count;

   uint8_t register_client;

   // for remote controller
   uint8_t serial_tx_buffer[SERIAL_RX_TRANSMITTER_MAX_SIZE];

   // for receive data from bt
   uint8_t bt_rx_buffer[PROTOCOL_CHANNEL_MAX_SIZE];
   uint8_t received_rx_length;
   uint32_t received_drop_frame_cout;

   // for transmit data to bt
   uint8_t bt_tx_buffer[PROTOCOL_BASIC_MAX_SIZE];
   uint8_t parsing_status;
   uint16_t parsing_result_code;

   TIMER timer[MAX_TIMER_COUNT];

} PROJECT;

#define RTC_HZ_FOR_TICK_COUNT                            32768
#define MILLI_SECOND                                     1000

#define MILLI_SECOND_TO_TICK_COUNT(ms)                   ((RTC_HZ_FOR_TICK_COUNT * ms) / MILLI_SECOND)
#define SECOND_TO_TICK_COUNT(s)                          (RTC_HZ_FOR_TICK_COUNT * s)
#define TICK_COUNT_TO_MILLI_SECOND(count)                ((count * MILLI_SECOND) / RTC_HZ_FOR_TICK_COUNT)
#define TICK_COUNT_TO_SECOND(count)                      (count * MILLI_SECOND / RTC_HZ_FOR_TICK_COUNT)

// -----------------------------------------------------------------------------
static PROJECT gProject[1]                               =
{
   {
      .mode                                              = PROJECT_INIT_STATUS,
   },
};


// -----------------------------------------------------------------------------
// for failsafe mode

typedef struct
{
   uint8_t channel_id;
   uint16_t value;
} FAILSAFE_MODE_VALUE;

static FAILSAFE_MODE_VALUE g_failsafe_value[SPEKTRUM_MAX_CHANNEL] =
{
   { SPEKTRUM_CHANNEL_ROLL,      512},
   { SPEKTRUM_CHANNEL_PITCH,     512},
   { SPEKTRUM_CHANNEL_YAW,       512},
   { SPEKTRUM_CHANNEL_THROTTLE,  100},
   { SPEKTRUM_CHANNEL_GEAR,      100},
   { SPEKTRUM_CHANNEL_AUX_1,     512},
   { SPEKTRUM_CHANNEL_AUX_2,     512},
   { SPEKTRUM_CHANNEL_AUX_3,     512},
   { SPEKTRUM_CHANNEL_AUX_4,     512},
   { SPEKTRUM_CHANNEL_AUX_5,     512},
   { SPEKTRUM_CHANNEL_AUX_6,     512},
   { SPEKTRUM_CHANNEL_AUX_7,     512},
};


// -----------------------------------------------------------------------------
// ...
#define IS_SRVC_CHANGED_CHARACT_PRESENT                  0                       /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT                               0                       /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                            1                       /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define NUS_SERVICE_UUID_TYPE                            BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                                 64                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS                       180                     /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER                              0                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE                          4                       /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                                MSEC_TO_UNITS(20, UNIT_1_25_MS)  /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL                                MSEC_TO_UNITS(75, UNIT_1_25_MS)  /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                                    0                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                                 MSEC_TO_UNITS(4000, UNIT_10_MS)  /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY                   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY                    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT                     3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define START_STRING                                     "Start...\r\n"

#define DEAD_BEEF                                        0xDEADBEEF              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                                 256                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                                 256                     /**< UART RX buffer size. */

static ble_nus_t                                         m_nus;                  /**< Structure to identify the Nordic UART Service. */
static uint16_t m_conn_handle                            = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[]                          = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};    /**< Universally unique service identifier. */
// -----------------------------------------------------------------------------


/******************************************************************
 *
 * Function Name : add_timer
 *
 *
 ******************************************************************/
static int add_timer(TIMER_FN fn, uint32_t ms_elapse, int count, void* pdata)
{
   int i;
   if (fn == NULL || ms_elapse == 0)
   { 
      return -1;
   }
   for (i = 0; i < MAX_TIMER_COUNT; i++)
   {
      if (gProject->timer[i].fn_timer == NULL)
      {
         gProject->timer[i].tick_count                   = 0;
         gProject->timer[i].ms_elapse                    = ms_elapse;
         gProject->timer[i].call_count                   = count;
         gProject->timer[i].fn_timer                     = fn;
         gProject->timer[i].pData                        = pdata;
      }
   }
   return -1;
}

/******************************************************************
 *
 * Function Name : delete_timer
 *
 *
 ******************************************************************/
static int delete_timer(TIMER_FN fn)
{
   int i;
   if (fn == NULL)
   { 
      return -1;
   }
   for (i = 0; i < MAX_TIMER_COUNT; i++)
   {
      if (gProject->timer[i].fn_timer == fn)
      {
         gProject->timer[i].tick_count                   = 0;
         gProject->timer[i].ms_elapse                    = 0;
         gProject->timer[i].call_count                   = 0;
         gProject->timer[i].call_fn_count                = 0;
         gProject->timer[i].fn_timer                     = 0;
         gProject->timer[i].pData                        = NULL;
      }
   }
   return -1;
}


/******************************************************************
 *
 * Function Name : serial_receiver_timer_handler()
 *
 *
 ******************************************************************/
static void serial_receiver_timer_handler(void * p_context)
{
   uint32_t tick_count                                   = 0;
   uint32_t diff_tick_count                              = 0;
   int i;

   if (app_timer_cnt_get(&tick_count) != NRF_SUCCESS)
   {
      printf("Can't get tick count \r\n");
      gProject->mode                                     = PROJECT_FAILED;
   }

   for (i = 0; i < MAX_TIMER_COUNT; i++)
   {
      if (gProject->timer[i].tick_count == 0)
      {
         gProject->timer[i].tick_count                   = tick_count;
      }
      else
      {
         diff_tick_count                                 = 0;
         app_timer_cnt_diff_compute(tick_count, gProject->timer[i].tick_count, &diff_tick_count);

         if (diff_tick_count > MILLI_SECOND_TO_TICK_COUNT(gProject->timer[i].ms_elapse))
         {
            gProject->timer[i].tick_count                = tick_count;
            if (gProject->timer[i].call_count < 0)
            {
               (*gProject->timer[i].fn_timer)(gProject->timer[i].pData);
            }
            else if (gProject->timer[i].call_fn_count < gProject->timer[i].call_count)
            {
               (*gProject->timer[i].fn_timer)(gProject->timer[i].pData);
            }
         }
      }
   }
}


// -----------------------------------------------------------------------------
// for protocol of transmitter
#if defined(SERIAL_RECEIVER)

// -----------------------------------------------------------------------------
// protocol type
#define SERIAL_RX_SPEKTRUM_1024
// #define SERIAL_RX_SPEKTRUM_2048
// #define SERIAL_RX_S_BUS
// #define SERIAL_RX_SUMD
// #define SERIAL_RX_SUMH
// #define SERIAL_RX_MODE_B
// #define SERIAL_RX_MODE_B_BJ01
// #define SERIAL_RX_IBUS
// #define SERIAL_RX_MSP


// -----------------------------------------------------------------------------
#if defined(SERIAL_RX_SPEKTRUM_1024)

// -----------------------------------------------------------------------------
// for timer function
// specktrum 1024 : 22 ms
#define SERIAL_RX_SEND_TO_FC_TIMER                       22
// 60 ms
#define SERIAL_RX_COMMAND_PARSING_TIMER                  30
// 60 ms
#define SERIAL_RX_COMMAND_RESPONSE_TIMER                 30
// 500 ms
#define SERIAL_RX_COMMAND_FAILSAFE_TIMER                 100

// UART
// BAUDRATE : 125000 bps, or 1152000 bps
// DATA     : 8 bits
// PARITY   : NO Parity
// stop bit : 1 stop

#define SERIAL_RX_BAUDRATE                               (UART_BAUDRATE_BAUDRATE_Baud115200)
#define DEVICE_NAME                                      "DRONE-BT-CONTROLLER"                     /**< Name of device. Will be included in the advertising data. */

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_SPEKTRUM_2048)
// -----------------------------------------------------------------------------
// for timer function
// specktrum 1024 : 22 ms
#define SERIAL_RX_SPEKTRUM_1024_TIMER                    22
// 60 ms
#define SERIAL_RX_COMMAND_PARSING_TIMER                  60
// 60 ms
#define SERIAL_RX_COMMAND_RESPONSE_TIMER                 60
// 500 ms
#define SERIAL_RX_COMMAND_FAILSAFE_TIMER                 100

// UART
// BAUDRATE : 125000 bps, or 1152000 bps
// DATA     : 8 bits
// PARITY   : NO Parity
// stop bit : 1 stop

#define SERIAL_RX_BAUDRATE                               (UART_BAUDRATE_BAUDRATE_Baud115200)
#define DEVICE_NAME                                      "DRONE-BT-CONTROLLER"                     /**< Name of device. Will be included in the advertising data. */

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_S_BUS)

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_SUMD)

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_SUMH)

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_MODE_B)

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_MODE_B_BJ01)

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_IBUS)

// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_MSP)

// -----------------------------------------------------------------------------
#else
#error "Not supported"
#endif

/******************************************************************
 *
 * Function Name : timer_send_msg_to_fc_controller()
 *
 *
 ******************************************************************/
static void timer_send_msg_to_fc_controller(void* pdata)
{

}

/******************************************************************
 *
 * Function Name : timer_bt_command_parser()
 *
 *
 ******************************************************************/
static void timer_bt_command_parser(void* pdata)
{
   BT_PROTOCOL_COMMAND* pbtProtocolCommand               = NULL;
   BT_PROTOCOL_COMMAND* pbtProtocolResponse              = NULL;

   if (gProject->parsing_status != 0)
   {
      return;
   }

   if (gProject->received_rx_length > 0)
   {
      pbtProtocolCommand                                 = (BT_PROTOCOL_COMMAND*) gProject->bt_rx_buffer;
      pbtProtocolResponse                                = (BT_PROTOCOL_COMMAND*) gProject->bt_tx_buffer;

      gProject->parsing_result_code                      = SUCCESS_RESPONSE;
      gProject->parsing_status                           = 1;
      gProject->received_rx_length                       = 0;

      pbtProtocolResponse->version_high                  = PROTOCOL_HEADER_HIGH_VERSION;
      pbtProtocolResponse->version_low                   = PROTOCOL_REGISTER_RESPONSE;

      pbtProtocolResponse->size                          = PROTOCOL_BASIC_MAX_SIZE;

      // error code
      pbtProtocolResponse->option_1_high                 = 0;
      pbtProtocolResponse->option_1_low                  = SUCCESS_RESPONSE;

      // reserved
      pbtProtocolResponse->option_2_high                 = 0;
      pbtProtocolResponse->option_2_low                  = 0;


      switch (pbtProtocolCommand->command)
      {
         case PROTOCOL_REGISTER_MESSAGE :
            pbtProtocolResponse->command                 = PROTOCOL_REGISTER_RESPONSE;
            break;

         case PROTOCOL_ALIVE_MESSAGE :
            pbtProtocolResponse->command                 = PROTOCOL_ALIVE_RESPONSE;
            break;

         case PROTOCOL_CHANNEL_MESSAGE :
            pbtProtocolResponse->command                 = PROTOCOL_UNKNOWN_RESPONSE;
            break;

         default :
            gProject->parsing_result_code                = ERROR_UNKNOWN_COMMAND;
            pbtProtocolResponse->command                 = PROTOCOL_UNKNOWN_RESPONSE;

            // error code
            pbtProtocolResponse->option_1_high           = 0;
            pbtProtocolResponse->option_1_low            = ERROR_UNKNOWN_COMMAND;
            return;
      }


      if (pbtProtocolCommand->version_high != PROTOCOL_HEADER_HIGH_VERSION)
      {
         gProject->parsing_result_code                   = ERROR_RESPONSE_VERSION;

         // error code
         pbtProtocolResponse->option_1_high              = 0;
         pbtProtocolResponse->option_1_low               = ERROR_RESPONSE_VERSION;
         return;
      }

      if (pbtProtocolCommand->version_low != PROTOCOL_HEADER_LOW_VERSION)
      {
         gProject->parsing_result_code                   = ERROR_RESPONSE_VERSION;

         // error code
         pbtProtocolResponse->option_1_high              = 0;
         pbtProtocolResponse->option_1_low               = ERROR_RESPONSE_VERSION;
         return;
      }

      switch (pbtProtocolCommand->command)
      {
         case PROTOCOL_REGISTER_MESSAGE :
            if (gProject->register_client != 0)
            {
               gProject->parsing_result_code             = ERROR_ALREADY_REGISTERED;

               // error code
               pbtProtocolResponse->option_1_high        = 0;
               pbtProtocolResponse->option_1_low         = ERROR_ALREADY_REGISTERED;
               return;
            }
            else
            {
               // register client
               gProject->register_client                 = 1;
               return;
            }
            break;

         case PROTOCOL_ALIVE_MESSAGE :
            // alive time 관련 처리
            return;

         case PROTOCOL_CHANNEL_MESSAGE :
            // channel information parsing
            return;

         default :
            gProject->parsing_result_code                = ERROR_UNKNOWN_COMMAND;

            // result code
            pbtProtocolResponse->option_1_high           = 0;
            pbtProtocolResponse->option_1_low            = ERROR_UNKNOWN_COMMAND;
            return;
      }
   }
   return;
}

/******************************************************************
 *
 * Function Name : timer_bt_response()
 *
 *
 ******************************************************************/
static void timer_bt_response(void* pdata)
{
   uint32_t err_code;

#if 0
   BT_PROTOCOL_COMMAND* pbtProtocolResponse              = NULL;
   pbtProtocolResponse                                   = (BT_PROTOCOL_COMMAND*) gProject->bt_tx_buffer;
#endif

   if (gProject->parsing_status != 0)
   {
      return;
   }

   err_code                                              = ble_nus_string_send(&m_nus, gProject->bt_tx_buffer, PROTOCOL_BASIC_MAX_SIZE);
   if (err_code != NRF_SUCCESS)
   {
   }
   gProject->parsing_status                              = 0;
}

/******************************************************************
 *
 * Function Name : timer_failsafe_mode()
 *
 *
 ******************************************************************/
static void timer_failsafe_mode(void* pdata)
{

}




#if 1

uint8_t update_receiver_command()
{
#if 0
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
#endif
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

#if 0
typedef struct
{
   uint8_t fades;
   uint8_t system;
   uint16_t channel[MAX_CHANNEL_COUNT];
} RX_PACKET_SPEKTRUM_1024;

#endif

#define UART_RETRY_SEND_COUNT                            5
uint8_t send_receiver_command()
{

#if 0
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
#endif

   return 0;
}

#elif defined(SERIAL_RX_SPEKTRUM_2048)

// Does not porting...

#elif defined(SERIAL_RX_S_BUS)

// Does not porting...

#elif defined(SERIAL_RX_SUMD)

// Does not porting...

#elif defined(SERIAL_RX_SUMH)

// Does not porting...

#elif defined(SERIAL_RX_MODE_B)

// Does not porting...

#elif defined(SERIAL_RX_MODE_B_BJ01)

// Does not porting...

#elif defined(SERIAL_RX_IBUS)

// Does not porting...

#elif defined(SERIAL_RX_MSP)

// Does not porting...

#else

// Does not supported ...

#endif

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

   if (gProject->received_rx_length > 0)
   {
      gProject->received_drop_frame_cout++;
      return;
   }

   gProject->received_rx_length                       = (length > PROTOCOL_CHANNEL_MAX_SIZE) ? (PROTOCOL_CHANNEL_MAX_SIZE) : (length);

   for (i = 0; i < gProject->received_rx_length; i++)
   {
      gProject->bt_rx_buffer[i]                       = *(p_data + i);
   }
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

   UNUSED_VARIABLE(gProject);
   UNUSED_VARIABLE(g_failsafe_value);
   UNUSED_VARIABLE(add_timer);
   UNUSED_VARIABLE(delete_timer);

   // clear variable ...
   memset(gProject, 0, sizeof(gProject));

   gProject->mode                                        = PROJECT_INIT_STATUS;

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

#if !defined(SERIAL_RECEIVER)
   printf("%s", start_string);
#else
   printf("program start \r\n ");
#endif
   err_code                                              = ble_advertising_start(BLE_ADV_MODE_FAST);
   if (err_code != NRF_SUCCESS)
   {
      printf("ble_advertising_start() failed : %d \r\n", (int) err_code);
      gProject->mode                                     = PROJECT_FAILED;
   }
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

   add_timer(timer_send_msg_to_fc_controller, SERIAL_RX_SEND_TO_FC_TIMER, -1, NULL);
   add_timer(timer_bt_command_parser, SERIAL_RX_COMMAND_PARSING_TIMER, -1, NULL);
   add_timer(timer_bt_response, SERIAL_RX_COMMAND_RESPONSE_TIMER, -1, NULL);
   add_timer(timer_failsafe_mode, SERIAL_RX_COMMAND_FAILSAFE_TIMER, -1, NULL);
#endif

   // Enter main loop.
   for (;;)
   {
      power_manage();
   }
}


/**
 * @}
 */
