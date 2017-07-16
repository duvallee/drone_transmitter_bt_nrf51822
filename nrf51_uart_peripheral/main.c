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

// for debugging
#define DEBUG_RTT_ERROR

#define DEBUG_OUTPUT_TIMER
//#define DEBUG_OUTPUT_BT_RECEIVED_DATA
#define DEBUG_OUTPUT_BT_COMMAND_PARSER
#define DEBUG_OUTPUT_BT_CHANNEL_PARSER
// #define DEBUG_OUTPUT_FC_CHANNEL_DATA
// #define DEBUG_OUTPUT_BT_RESPONSE

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

// 20 bytes is max size of bluetooth in the android.
// And then, send channels data by twice.
#define PROTOCOL_PACKET_BASIC_SIZE                       8
#define PROTOCOL_PACKET_EXTRA_SIZE                       16

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
#define SPEKTRUM_CHANNEL_ROLL                            1
#define SPEKTRUM_CHANNEL_PITCH                           2
#define SPEKTRUM_CHANNEL_YAW                             3
#define SPEKTRUM_CHANNEL_THROTTLE                        0
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
   PROJECT_INIT_STATUS                                   = 0x00000000,
   PROJECT_READY_STATUS                                  = 0x00000001,
   PROJECT_NORMAL_STATUS                                 = 0x00000002,
   PROJECT_STATUS_MASK                                   = 0x0000FFFF,
   PROJECT_STATUS_UNMASK                                 = 0xFFFF0000,

   PROJECT_FAILSAFE_MODE                                 = 0x08000000,
   PROJECT_FAILSAFE_MASK                                 = 0x0F000000,
   PROJECT_FAILSAFE_UNMASK                               = 0xF0FFFFFF,

   PROJECT_FAIL_STATUS                                   = 0x80000000,
};

#define SET_PROJECT_INIT_STATUS(x)                       (x = ((x & PROJECT_STATUS_UNMASK) | PROJECT_INIT_STATUS))
#define IS_PROJECT_INIT_STATUS(x)                        (x & PROJECT_INIT_STATUS)

#define SET_PROJECT_READY_STATUS(x)                      (x = ((x & PROJECT_STATUS_UNMASK) | PROJECT_READY_STATUS))
#define IS_PROJECT_READY_STATUS(x)                       (x & PROJECT_READY_STATUS)

#define SET_PROJECT_NORMAL_STATUS(x)                     (x = ((x & PROJECT_STATUS_UNMASK) | PROJECT_NORMAL_STATUS))
#define IS_PROJECT_NORMAL_STATUS(x)                      (x & PROJECT_NORMAL_STATUS)

#define SET_PROJECT_FAILSAFE_MODE(x)                     (x = ((x & PROJECT_FAILSAFE_UNMASK) | PROJECT_FAILSAFE_MODE))
#define RESET_PROJECT_FAILSAFE_MODE(x)                   (x = (x & PROJECT_FAILSAFE_UNMASK))
#define IS_PROJECT_FAILSAFE_MODE(x)                      (x & PROJECT_FAILSAFE_MODE)

#define SET_PROJECT_FAIL_STATUS(x)                       (x = (x | PROJECT_FAIL_STATUS))
#define IS_PROJECT_FAIL_STATUS(x)                        (x & PROJECT_FAIL_STATUS)

// -----------------------------------------------------------------------------
enum RESPONSE_ERROR_CODE
{
   SUCCESS_RESPONSE                                      = 0,
   ERROR_RESPONSE_VERSION                                = 1,
   ERROR_RESPONSE_CRC_ERROR                              = 2,
   ERROR_RESPONSE_CHANNEL_FAILED                         = 3,
   ERROR_ALREADY_REGISTERED                              = 4,
   ERROR_UNKNOWN_COMMAND                                 = 5,
   ERROR_INTERNAL_PACKET_SIZE                            = 6,
   ERROR_PACKET_SIZE                                     = 7,
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
   uint32_t mode;
   // timer tick count
   uint32_t tick_count;

   // check alive
   uint32_t last_alive_tick_count;

   uint32_t client_alive_count;
   uint8_t register_client;

   // tick counter when enter failsafe mode
   uint32_t enter_failsafe_mode_tick_count;

   // use internal : packet for fc
   uint8_t internal_tx_buffer[SERIAL_RX_TRANSMITTER_MAX_SIZE];
   uint8_t internal_tx_packet_size;

   // send to fc
   uint8_t fc_tx_buffer[SERIAL_RX_TRANSMITTER_MAX_SIZE];
   uint8_t fc_tx_packet_size;

   // for receive data from bt
   uint8_t bt_rx_buffer[PROTOCOL_CHANNEL_MAX_SIZE];
   uint8_t received_rx_length;
   uint8_t received_rx_packet_complete;
   uint32_t received_drop_frame_cout;

   // for response to bt
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
#define FAILSAFE_MODE_TIME_OUT_SECOND                    30
typedef struct
{
   uint16_t channel_id;
   uint16_t value;
} FAILSAFE_MODE_VALUE;

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
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] function is null(%p) or ms_slapse is 0 (%d) \r\n", __FUNCTION__, __LINE__, fn, ms_elapse);
#endif
      return -1;
   }

#if defined(DEBUG_OUTPUT_TIMER)
      NRF_LOG_PRINTF("[%s-%d] fn=%p, ms_elapse = %d, count = %d, data = %p \r\n", __FUNCTION__, __LINE__, fn, ms_elapse, count, pdata);
#endif
   for (i = 0; i < MAX_TIMER_COUNT; i++)
   {
#if defined(DEBUG_OUTPUT_TIMER)
      NRF_LOG_PRINTF("[%s-%d] [%d] : %p \r\n", __FUNCTION__, __LINE__, i, gProject->timer[i].fn_timer);
#endif
      if (gProject->timer[i].fn_timer == NULL)
      {
         gProject->timer[i].tick_count                   = 0;
         gProject->timer[i].ms_elapse                    = ms_elapse;
         gProject->timer[i].call_count                   = count;
         gProject->timer[i].fn_timer                     = fn;
         gProject->timer[i].pData                        = pdata;
         return 0;
      }
   }
#if defined(DEBUG_RTT_ERROR)
   NRF_LOG_PRINTF("[%s-%d] timer is full (total timer = %d) \r\n", __FUNCTION__, __LINE__, MAX_TIMER_COUNT);
#endif
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
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] function is null \r\n", __FUNCTION__, __LINE__);
#endif
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
#if defined(DEBUG_RTT_ERROR)
   NRF_LOG_PRINTF("[%s-%d] Not found timer (%p) \r\n", __FUNCTION__, __LINE__, fn);
#endif
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

   app_timer_cnt_get(&tick_count);

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
// spektrum 1024 : 22 ms
#define SERIAL_RX_SEND_TO_FC_TIMER                       22
// 60 ms
#define SERIAL_RX_COMMAND_PARSING_TIMER                  60
// 60 ms
#define SERIAL_RX_COMMAND_RESPONSE_TIMER                 60
// 500 ms
#define SERIAL_RX_COMMAND_FAILSAFE_TIMER                 100

// -----------------------------------------------------------------------------
// UART
// BAUDRATE : 125000 bps, or 1152000 bps
// DATA     : 8 bits
// PARITY   : NO Parity
// stop bit : 1 stop

// -----------------------------------------------------------------------------
#define SERIAL_RX_BAUDRATE                               (UART_BAUDRATE_BAUDRATE_Baud115200)
#define DEVICE_NAME                                      "DRONE-BT-CONTROLLER"                     /**< Name of device. Will be included in the advertising data. */

// -----------------------------------------------------------------------------
// for channel protocol
#define MAX_SPEKTRUM_CHANNEL_NUM                         7
#define SPEKTRUM_1024_CHANID_MASK                        0xFC00
#define SPEKTRUM_1024_SXPOS_MASK                         0x03FF
#define SPEKTRUM_1024_CHANNEL_SHIFT_BITS                 10

#define SPEKTRUM_DSM2_22MS                               0x01                    // 1024
#define SPEKTRUM_DSM2_11MS                               0x12                    // 2048
#define SPEKTRUM_DSMS_22MS                               0xA2                    // 2048
#define SPEKTRUM_DSMX_11MS                               0xB2                    // 2048

// struct for SPEKTRUM 1024 Protocol
typedef struct _RX_PACKET_SPEKTRUM_1024
{
   uint8_t fades;
   uint8_t system;
   uint16_t channel[MAX_SPEKTRUM_CHANNEL_NUM];
} __attribute__ ((__packed__)) RX_PACKET_SPEKTRUM_1024;

/******************************************************************
 *
 * Function Name : bt_packet_to_fc_packet()
 *   covert packet of bt to packet of fc 
 *
 ******************************************************************/
static int bt_packet_to_fc_packet(uint16_t* p_bt_channel_info, uint8_t bt_channel_count, void* p_fc_packet, uint8_t* p_fc_acket_size)
{
   uint16_t bt_channel_index;
   uint16_t spektrum_channel_index;

   uint16_t channel_id;
   uint16_t channel_value;

   RX_PACKET_SPEKTRUM_1024* pRxPacketSpektrum1024        = (RX_PACKET_SPEKTRUM_1024*) p_fc_packet;

   if (*p_fc_acket_size < sizeof(RX_PACKET_SPEKTRUM_1024))
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] wrong channel data : %d less than %d \r\n", __FUNCTION__, __LINE__, *p_fc_acket_size, sizeof(RX_PACKET_SPEKTRUM_1024));
#endif
      return -1;
   }

   // packet size of SPEKTRUM 1024
   *p_fc_acket_size                                      = sizeof(RX_PACKET_SPEKTRUM_1024);

   // header of spektrum's 1024
   pRxPacketSpektrum1024->fades                          = 0x0;
   pRxPacketSpektrum1024->system                         = SPEKTRUM_DSM2_22MS;

#if defined(DEBUG_OUTPUT_BT_CHANNEL_PARSER)
   NRF_LOG_PRINTF("------------------------------------------------------------- \r\n");
#endif

   for (bt_channel_index = 0, spektrum_channel_index = 0;
        bt_channel_index < bt_channel_count && spektrum_channel_index < MAX_SPEKTRUM_CHANNEL_NUM;
        bt_channel_index++, spektrum_channel_index++)
   {
      channel_id                                         = ((p_bt_channel_info[bt_channel_index] >> 4) & 0x0F);
      channel_value                                      = ((p_bt_channel_info[bt_channel_index] >> 8) & 0x00FF) |
                                                           ((p_bt_channel_info[bt_channel_index] << 8) & 0x0F00) ;

      pRxPacketSpektrum1024->channel[spektrum_channel_index] = (SPEKTRUM_1024_CHANID_MASK & (channel_id << SPEKTRUM_1024_CHANNEL_SHIFT_BITS)) |
                                                               (SPEKTRUM_1024_SXPOS_MASK & channel_value);

      pRxPacketSpektrum1024->channel[spektrum_channel_index] = (channel_id << 2) | ((channel_value << 8) & 0x0F) | ((channel_value >> 8) & 0x3);

      NRF_LOG_PRINTF("[%d] : id = %2d, v = %4d    [0x%04X] \r\n", bt_channel_index, channel_id, channel_value, pRxPacketSpektrum1024->channel[spektrum_channel_index] );
#if 0
#if defined(DEBUG_OUTPUT_BT_CHANNEL_PARSER)
      NRF_LOG_PRINTF("[%d] \r\n", bt_channel_index);

      NRF_LOG_PRINTF("Channel Data of original \r\n");
      NRF_LOG_HEX(p_bt_channel_info[bt_channel_index]);

      NRF_LOG_PRINTF("\r\nChannel id    : %d \r\n", channel_id);
      NRF_LOG_PRINTF("Channel value : %d \r\n", channel_value);
      NRF_LOG_PRINTF("hexa : 0x%04X \r\n", pRxPacketSpektrum1024->channel[spektrum_channel_index]);
      NRF_LOG_PRINTF("[%2d] : CH(%2d), VALUE(%4d) : [0x%04X] org - [0x%04X] \r\n", bt_channel_index
                                                            channel_id,
                                                            channel_value,
                                                            spektrum_channel_index,
                                                            p_bt_channel_info[bt_channel_index],
                                                            pRxPacketSpektrum1024->channel[spektrum_channel_index]);
#endif
#endif

#if 0
      pRxPacketSpektrum1024->channel[spektrum_channel_index] = (SPEKTRUM_1024_CHANID_MASK & (p_bt_channel_info[bt_channel_index] >> 2)) |
                                                               (SPEKTRUM_1024_SXPOS_MASK & p_bt_channel_info[bt_channel_index]);
#endif
   }
#if defined(DEBUG_OUTPUT_BT_CHANNEL_PARSER)
   NRF_LOG_PRINTF("------------------------------------------------------------- \r\n");
#endif
   return 0;
}

// -----------------------------------------------------------------------------
// default value for failsafe mode ...
static FAILSAFE_MODE_VALUE g_failsafe_value[SPEKTRUM_MAX_CHANNEL] =
{
   { (SPEKTRUM_CHANNEL_ROLL << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),      650},
   { (SPEKTRUM_CHANNEL_PITCH << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
   { (SPEKTRUM_CHANNEL_YAW << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),       650},
   { (SPEKTRUM_CHANNEL_THROTTLE << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),  300},
   { (SPEKTRUM_CHANNEL_GEAR << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),      300},
   { (SPEKTRUM_CHANNEL_AUX_1 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
   { (SPEKTRUM_CHANNEL_AUX_2 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
   { (SPEKTRUM_CHANNEL_AUX_3 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
   { (SPEKTRUM_CHANNEL_AUX_4 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
   { (SPEKTRUM_CHANNEL_AUX_5 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
   { (SPEKTRUM_CHANNEL_AUX_6 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
   { (SPEKTRUM_CHANNEL_AUX_7 << SPEKTRUM_1024_CHANNEL_SHIFT_BITS),     650},
};

/******************************************************************
 *
 * Function Name : bt_packet_to_fc_packet()
 *   covert packet of bt to packet of fc 
 *
 ******************************************************************/
static int default_fc_packet(void* p_fc_packet, uint8_t* p_fc_acket_size)
{
   int i;
   RX_PACKET_SPEKTRUM_1024* pRxPacketSpektrum1024        = (RX_PACKET_SPEKTRUM_1024*) p_fc_packet;
   if (*p_fc_acket_size < sizeof(RX_PACKET_SPEKTRUM_1024))
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] packet size %d less than %d \r\n", __FUNCTION__, __LINE__, *p_fc_acket_size, sizeof(RX_PACKET_SPEKTRUM_1024));
#endif
      return -1;
   }

   // packet size of SPEKTRUM 1024
   *p_fc_acket_size                                      = sizeof(RX_PACKET_SPEKTRUM_1024);

   // header of spektrum's 1024
   pRxPacketSpektrum1024->fades                          = 0x0;
   pRxPacketSpektrum1024->system                         = SPEKTRUM_DSM2_22MS;

   for (i = 0; i < MAX_SPEKTRUM_CHANNEL_NUM; i++)
   {
      pRxPacketSpektrum1024->channel[i]                  = (g_failsafe_value[i].channel_id | g_failsafe_value[i].value);
   }
   return 0;
}



// -----------------------------------------------------------------------------
#elif defined(SERIAL_RX_SPEKTRUM_2048)

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
#endif      // SERIAL_RX_SPEKTRUM_1024
#endif      // SERIAL_RECEIVER

/******************************************************************
 *
 * Function Name : timer_send_msg_to_fc_controller()
 *
 *
 ******************************************************************/
#define UART_RETRY_SEND_COUNT                            1
static void timer_send_msg_to_fc_controller(void* pdata)
{
   int i;
   int retry_count                                       = 0;
#if defined(DEBUG_OUTPUT_FC_CHANNEL_DATA)
// To output string per about 2 seconds
static int msg_output_count                              = 0;

   if (((++msg_output_count++) % 90) == 0)
   {
      NRF_LOG_PRINTF("send packet to the FC \r\n");
   }
#endif

   if (gProject->internal_tx_packet_size > 0)
   {
      memcpy(gProject->fc_tx_buffer, gProject->internal_tx_buffer, gProject->internal_tx_packet_size);
      gProject->fc_tx_packet_size                        = gProject->internal_tx_packet_size;
      gProject->internal_tx_packet_size                  = 0;
   }
   for (i = 0; i < (gProject->fc_tx_packet_size); i++)
   {
      retry_count                                        = 0;
      while(app_uart_put(gProject->fc_tx_buffer[i]) != NRF_SUCCESS)
      {
         if (UART_RETRY_SEND_COUNT < retry_count++)
         {
#if defined(DEBUG_RTT_ERROR)
            NRF_LOG_PRINTF("[%s-%d] failed send to the UART [%d] [0x%02X] \r\n", __FUNCTION__, __LINE__, i, gProject->fc_tx_buffer[i]);
#endif
            return;
         }
#if defined(DEBUG_OUTPUT_FC_CHANNEL_DATA)
         if (((msg_output_count) % 90) == 0)
         {
            NRF_LOG_PRINTF("[0x%02X] ", gProject->fc_tx_buffer[i]);
         }
#endif
      }
   }
#if defined(DEBUG_OUTPUT_FC_CHANNEL_DATA)
   if (((msg_output_count) % 90) == 0)
   {
      NRF_LOG_PRINTF("\r\n");
   }
#endif

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
   BT_PROTOCOL_DATA* pbtProtocolChannelData              = NULL;
   uint8_t rx_data_length                                = gProject->received_rx_length;

   if (gProject->parsing_status != 0)
   {
#if defined(DEBUG_RTT_ERROR)
static int parsing_status_index                          = 0;
      if ((parsing_status_index++ % 100) == 0)
      {
         NRF_LOG_PRINTF("[%s-%d] Does not process previous command !!! \r\n", __FUNCTION__, __LINE__);
      }
#endif
      return;
   }

#if defined(DEBUG_OUTPUT_BT_COMMAND_PARSER)
//   NRF_LOG_PRINTF("[%s-%d] cammand parser : %d \r\n", __FUNCTION__, __LINE__, gProject->received_rx_length);
#endif

   if (gProject->received_rx_packet_complete != 0)
   {
      pbtProtocolCommand                                 = (BT_PROTOCOL_COMMAND*) gProject->bt_rx_buffer;
      pbtProtocolResponse                                = (BT_PROTOCOL_COMMAND*) gProject->bt_tx_buffer;

      gProject->parsing_result_code                      = SUCCESS_RESPONSE;
      gProject->parsing_status                           = 1;
      gProject->received_rx_packet_complete              = 0;
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

            // check internal packet size
            if (rx_data_length != PROTOCOL_BASIC_MAX_SIZE)
            {
               gProject->parsing_result_code             = ERROR_INTERNAL_PACKET_SIZE;

               // error code
               pbtProtocolResponse->option_1_high        = 0;
               pbtProtocolResponse->option_1_low         = ERROR_INTERNAL_PACKET_SIZE;
#if defined(DEBUG_RTT_ERROR)
               NRF_LOG_PRINTF("[%s-%d] Wrong size or mismathed size from size of receive from bt(register cmd) \r\n", __FUNCTION__, __LINE__);
#endif
               return;
            }

            // check packet size
            if (pbtProtocolCommand->size != PROTOCOL_BASIC_MAX_SIZE)
            {
               gProject->parsing_result_code             = ERROR_PACKET_SIZE;

               // error code
               pbtProtocolResponse->option_1_high        = 0;
               pbtProtocolResponse->option_1_low         = ERROR_PACKET_SIZE;
#if defined(DEBUG_RTT_ERROR)
               NRF_LOG_PRINTF("[%s-%d] Wrong packet size (register cmd) \r\n", __FUNCTION__, __LINE__);
#endif
               return;
            }
            break;

         case PROTOCOL_ALIVE_MESSAGE :
            pbtProtocolResponse->command                 = PROTOCOL_ALIVE_RESPONSE;

            // check internal packet size
            if (rx_data_length != PROTOCOL_BASIC_MAX_SIZE)
            {
               gProject->parsing_result_code             = ERROR_INTERNAL_PACKET_SIZE;

               // error code
               pbtProtocolResponse->option_1_high        = 0;
               pbtProtocolResponse->option_1_low         = ERROR_INTERNAL_PACKET_SIZE;
#if defined(DEBUG_RTT_ERROR)
               NRF_LOG_PRINTF("[%s-%d] Wrong size or mismathed size from size of receive from bt(register cmd) \r\n", __FUNCTION__, __LINE__);
#endif
               return;
            }

            // check packet size
            if (pbtProtocolCommand->size != PROTOCOL_BASIC_MAX_SIZE)
            {
               gProject->parsing_result_code             = ERROR_PACKET_SIZE;

               // error code
               pbtProtocolResponse->option_1_high        = 0;
               pbtProtocolResponse->option_1_low         = ERROR_PACKET_SIZE;
#if defined(DEBUG_RTT_ERROR)
               NRF_LOG_PRINTF("[%s-%d] Wrong packet size (alive msg) \r\n", __FUNCTION__, __LINE__);
#endif
               return;
            }
            break;
            break;

         case PROTOCOL_CHANNEL_MESSAGE :
            pbtProtocolResponse->command                 = PROTOCOL_UNKNOWN_RESPONSE;

            // check internal packet size
            if (rx_data_length != PROTOCOL_CHANNEL_MAX_SIZE)
            {
               gProject->parsing_result_code             = ERROR_INTERNAL_PACKET_SIZE;

               // error code
               pbtProtocolResponse->option_1_high        = 0;
               pbtProtocolResponse->option_1_low         = ERROR_INTERNAL_PACKET_SIZE;
#if defined(DEBUG_RTT_ERROR)
               NRF_LOG_PRINTF("[%s-%d] Wrong size or mismathed size from size of receive from bt(register cmd) - %d \r\n", __FUNCTION__, __LINE__, rx_data_length);
#endif
               return;
            }

            // check packet size
            if (pbtProtocolCommand->size != PROTOCOL_CHANNEL_MAX_SIZE)
            {
               gProject->parsing_result_code             = ERROR_PACKET_SIZE;

               // error code
               pbtProtocolResponse->option_1_high        = 0;
               pbtProtocolResponse->option_1_low         = ERROR_PACKET_SIZE;
#if defined(DEBUG_RTT_ERROR)
               NRF_LOG_PRINTF("[%s-%d] Wrong packet size (alive msg) \r\n", __FUNCTION__, __LINE__);
#endif
               return;
            }
            break;

         default :
            gProject->parsing_result_code                = ERROR_UNKNOWN_COMMAND;
            pbtProtocolResponse->command                 = PROTOCOL_UNKNOWN_RESPONSE;

            // error code
            pbtProtocolResponse->option_1_high           = 0;
            pbtProtocolResponse->option_1_low            = ERROR_UNKNOWN_COMMAND;
#if defined(DEBUG_RTT_ERROR)
            NRF_LOG_PRINTF("[%s-%d] unknown command \r\n", __FUNCTION__, __LINE__);
#endif
            return;
      }


      if (pbtProtocolCommand->version_high != PROTOCOL_HEADER_HIGH_VERSION)
      {
         gProject->parsing_result_code                   = ERROR_RESPONSE_VERSION;

         // error code
         pbtProtocolResponse->option_1_high              = 0;
         pbtProtocolResponse->option_1_low               = ERROR_RESPONSE_VERSION;
#if defined(DEBUG_RTT_ERROR)
         NRF_LOG_PRINTF("[%s-%d] mismatched version : %d, %d \r\n", __FUNCTION__, __LINE__, pbtProtocolCommand->version_high, PROTOCOL_HEADER_HIGH_VERSION);
#endif
         return;
      }

      if (pbtProtocolCommand->version_low != PROTOCOL_HEADER_LOW_VERSION)
      {
         gProject->parsing_result_code                   = ERROR_RESPONSE_VERSION;

         // error code
         pbtProtocolResponse->option_1_high              = 0;
         pbtProtocolResponse->option_1_low               = ERROR_RESPONSE_VERSION;
#if defined(DEBUG_RTT_ERROR)
         NRF_LOG_PRINTF("[%s-%d] mismatched version : %d, %d \r\n", __FUNCTION__, __LINE__, pbtProtocolCommand->version_low, PROTOCOL_HEADER_LOW_VERSION);
#endif
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
#if defined(DEBUG_OUTPUT_BT_COMMAND_PARSER)
//               NRF_LOG_PRINTF("already register \r\n");
#endif
               return;
            }
            else
            {
               // register client
               gProject->register_client                 = 1;
               app_timer_cnt_get(&gProject->last_alive_tick_count);
               return;
            }
            break;

         case PROTOCOL_ALIVE_MESSAGE :
            gProject->client_alive_count                 = (pbtProtocolCommand->option_1_high << 24) |
                                                           (pbtProtocolCommand->option_1_low  << 16) |
                                                           (pbtProtocolCommand->option_2_high << 8) |
                                                           (pbtProtocolCommand->option_1_low);
            app_timer_cnt_get(&gProject->last_alive_tick_count);
            return;

         case PROTOCOL_CHANNEL_MESSAGE :
            pbtProtocolChannelData                       = (BT_PROTOCOL_DATA*) gProject->bt_rx_buffer;

            gProject->internal_tx_packet_size            = SERIAL_RX_TRANSMITTER_MAX_SIZE;
            if (bt_packet_to_fc_packet(&(pbtProtocolChannelData->channel_1), SPEKTRUM_MAX_CHANNEL,
                                       (void*) gProject->internal_tx_buffer, &gProject->internal_tx_packet_size) < 0)
            {
            }
            app_timer_cnt_get(&gProject->last_alive_tick_count);
            SET_PROJECT_NORMAL_STATUS(gProject->mode);
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

   if (gProject->parsing_status == 0)
   {
      return;
   }

#if defined(DEBUG_OUTPUT_BT_RESPONSE)
   {
static int timer_bt_response_count                       = 0;
      if ((timer_bt_response_count++ % 10) == 0)
      {
         int i;

         NRF_LOG_PRINTF("BT RESPONSE \r\n");
         for (i = 0; i < PROTOCOL_BASIC_MAX_SIZE; i++)
         {
            NRF_LOG_PRINTF("[0x%02X] ", gProject->bt_tx_buffer[i]);
         }
         NRF_LOG_PRINTF("\r\n");
      }
   }
#endif

   err_code                                              = ble_nus_string_send(&m_nus, gProject->bt_tx_buffer, PROTOCOL_BASIC_MAX_SIZE);
   if (err_code != NRF_SUCCESS)
   {
#if defined(DEBUG_OUTPUT_BT_RESPONSE)
      NRF_LOG_PRINTF("ble_nus_string_send() failed : %d \r\n", err_code);
#endif
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
   uint32_t diff_tick_count                              = 0;
   uint32_t cur_tick_count                               = 0;

   app_timer_cnt_get(&cur_tick_count);
   app_timer_cnt_diff_compute(cur_tick_count, gProject->last_alive_tick_count, &diff_tick_count);

   if (TICK_COUNT_TO_SECOND(diff_tick_count) > FAILSAFE_MODE_TIME_OUT_SECOND)
   {
      if (IS_PROJECT_FAILSAFE_MODE(gProject->mode) == 0)
      {
         gProject->enter_failsafe_mode_tick_count        = cur_tick_count;
      }

      SET_PROJECT_FAILSAFE_MODE(gProject->mode);
   }
}



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
#if defined(DEBUG_OUTPUT_BT_RECEIVED_DATA)
static int test_nus_data_handler_count                   = 0;
#endif
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
   int i;

#if defined(DEBUG_OUTPUT_BT_RECEIVED_DATA)
   if ((test_nus_data_handler_count++ % 10) == 0)
   {
      NRF_LOG_PRINTF("BT Received Data : %d bytes(internal length : %d) \r\n", length, gProject->received_rx_length);
   }
#endif

   if (gProject->received_rx_packet_complete != 0)
   {
      return;
   }

#if 0
   if (gProject->received_rx_length > 0)
   {
      gProject->received_drop_frame_cout++;
#if defined(DEBUG_RTT_ERROR)
      if ((gProject->received_drop_frame_cout % 10) == 0)
      {
         NRF_LOG_PRINTF("[%s-%d] Drop received frame from bt, because not process previos packet (%d) \r\n", __FUNCTION__, __LINE__, gProject->received_rx_length);
      }
#endif
      return;
   }
#endif

   if (length == PROTOCOL_PACKET_BASIC_SIZE)
   {
      // register or alive packet
      for (i = 0; i < PROTOCOL_PACKET_BASIC_SIZE; i++)
      {
         gProject->bt_rx_buffer[i]                       = *(p_data + i);
      }
      gProject->received_rx_length                       = length;
      gProject->received_rx_packet_complete              = 1;
   }
   else if (length == PROTOCOL_PACKET_EXTRA_SIZE)
   {
      if (gProject->received_rx_length > 0)
      {
         // second packet of channel
         NRF_LOG_PRINTF("second 16 bytes \r\n");
         for (i = 0; i < PROTOCOL_PACKET_EXTRA_SIZE; i++)
         {
            gProject->bt_rx_buffer[PROTOCOL_PACKET_EXTRA_SIZE + i] = *(p_data + i);
            NRF_LOG_HEX(gProject->bt_rx_buffer[PROTOCOL_PACKET_EXTRA_SIZE + i]);
            NRF_LOG_PRINTF("  ");
         }
         NRF_LOG_PRINTF("\r\n");
         gProject->received_rx_length                    += length;
         gProject->received_rx_packet_complete              = 1;
#if defined(DEBUG_OUTPUT_BT_RECEIVED_DATA)
         NRF_LOG_PRINTF("complete packet of channel ");
#endif
      }
      else
      {
         // first packet of channel
         NRF_LOG_PRINTF("first 16 bytes \r\n");
         for (i = 0; i < PROTOCOL_PACKET_EXTRA_SIZE; i++)
         {
            gProject->bt_rx_buffer[i]                    = *(p_data + i);
            NRF_LOG_HEX(gProject->bt_rx_buffer[i]);
            NRF_LOG_PRINTF("  ");
         }
         NRF_LOG_PRINTF("\r\n");
         gProject->received_rx_length                    = length;
#if defined(DEBUG_OUTPUT_BT_RECEIVED_DATA)
         NRF_LOG_PRINTF("first packet of channel ");
#endif
      }
   }
   else
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] wrong packet size (%d) \r\n", __FUNCTION__, __LINE__, length);
#endif
   }

#if 0
   gProject->received_rx_length                       = (length > PROTOCOL_CHANNEL_MAX_SIZE) ? (PROTOCOL_CHANNEL_MAX_SIZE) : (length);

   for (i = 0; i < gProject->received_rx_length; i++)
   {
      gProject->bt_rx_buffer[i]                       = *(p_data + i);

#if defined(DEBUG_OUTPUT_BT_RECEIVED_DATA)
      if ((test_nus_data_handler_count++ % 10) == 0)
      {
         NRF_LOG_PRINTF("[0x%02X] ", gProject->bt_rx_buffer[i]);
      }
#endif
   }
#if defined(DEBUG_OUTPUT_BT_RECEIVED_DATA)
   NRF_LOG_PRINTF("\r\n");
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

   UNUSED_VARIABLE(gProject);
   UNUSED_VARIABLE(g_failsafe_value);
   UNUSED_VARIABLE(add_timer);
   UNUSED_VARIABLE(delete_timer);

   // clear variable ...
   memset(gProject, 0, sizeof(gProject));
   SET_PROJECT_INIT_STATUS(gProject->mode);

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

   APP_ERROR_CHECK(NRF_LOG_INIT());
   NRF_LOG_PRINTF("Drone Remote Controller ver : %d.%d.%d  \r\n",
                   (PROTOCOL_HEADER_HIGH_VERSION >> 4),
                   (PROTOCOL_HEADER_HIGH_VERSION & 0xF),
                   PROTOCOL_HEADER_LOW_VERSION);

   err_code                                              = ble_advertising_start(BLE_ADV_MODE_FAST);
   if (err_code != NRF_SUCCESS)
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] ble_advertising_start() failed : %d \r\n", __FUNCTION__, __LINE__, (int) err_code);
#endif
      SET_PROJECT_FAIL_STATUS(gProject->mode);
   }
   APP_ERROR_CHECK(err_code);

#if defined(SERIAL_RECEIVER)
   if (app_timer_create(&serial_rx_timer, APP_TIMER_MODE_REPEATED, serial_receiver_timer_handler) != NRF_SUCCESS)
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] Can't create app timer \r\n", __FUNCTION__, __LINE__);
#endif
   }
   if (app_timer_start(serial_rx_timer, APP_TIMER_TICKS(1, APP_TIMER_PRESCALER), NULL) != NRF_SUCCESS)
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] Can't create app timer \r\n", __FUNCTION__, __LINE__);
#endif
   }

   // make default packet
   gProject->fc_tx_packet_size                        = SERIAL_RX_TRANSMITTER_MAX_SIZE;
   default_fc_packet(gProject->fc_tx_buffer, &gProject->fc_tx_packet_size);

   add_timer(timer_send_msg_to_fc_controller, SERIAL_RX_SEND_TO_FC_TIMER, -1, NULL);
   add_timer(timer_bt_command_parser, SERIAL_RX_COMMAND_PARSING_TIMER, -1, NULL);
   add_timer(timer_bt_response, SERIAL_RX_COMMAND_RESPONSE_TIMER, -1, NULL);
   add_timer(timer_failsafe_mode, SERIAL_RX_COMMAND_FAILSAFE_TIMER, -1, NULL);
#endif
   SET_PROJECT_READY_STATUS(gProject->mode);

   // Enter main loop.
   for (;;)
   {
      power_manage();
   }
}


/**
 * @}
 */
