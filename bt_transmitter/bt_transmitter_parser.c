
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
#include "project.h"
#include "bt_transmitter_timer.h"
#include "bt_transmitter_parser.h"


/******************************************************************
 *
 * Function Name : bt_transmitter_base_pcaket_process
 *
 *
 ******************************************************************/
void bt_transmitter_send_response(uint8_t cmd, uint8_t option_1_high, uint8_t option_1_low, uint8_t option_2_high, uint8_t option_2_low)
{
   BT_PROTOCOL_BASE bt_protocol_base;
   uint32_t err_code                                     = 0;
   memset(&bt_protocol_base, 0, sizeof(bt_protocol_base));

   bt_protocol_base.high_sync_byte                       = PROTOCOL_HEADER_LOW_SYNC;
   bt_protocol_base.low_sync_byte                        = PROTOCOL_HEADER_HIGH_SYNC;
   bt_protocol_base.command                              = cmd;
   bt_protocol_base.size                                 = sizeof(bt_protocol_base);
   bt_protocol_base.option_1_high                        = option_1_high;
   bt_protocol_base.option_1_low                         = option_1_low;
   bt_protocol_base.option_2_high                        = option_2_high;
   bt_protocol_base.option_2_low                         = option_2_low;

//   err_code                                              = ble_nus_string_send(&m_nus, gProject->bt_tx_buffer, PROTOCOL_BASIC_MAX_SIZE);

   if (err_code != NRF_SUCCESS)
   {
   }
}

/******************************************************************
 *
 * Function Name : bt_transmitter_base_pcaket_process
 *
 *
 ******************************************************************/
void bt_transmitter_base_pcaket_process(BT_PROTOCOL_BASE* pBtProtocolBase)
{
   switch (pBtProtocolBase->command)
   {
      case PROTOCOL_REGISTER_MESSAGE :
#if defined(DEBUG_RTT_DEBUG)
         NRF_LOG_PRINTF("[%s-%d] register message \r\n", __FUNCTION__, __LINE__);
#endif
         break;

      case PROTOCOL_ALIVE_MESSAGE :
#if defined(DEBUG_RTT_DEBUG)
         NRF_LOG_PRINTF("[%s-%d] alive message \r\n", __FUNCTION__, __LINE__);
#endif
         break;

      default :
         break;
   }
}


#if 0
#define PROTOCOL_REGISTER_MESSAGE                        0x01
#define PROTOCOL_REGISTER_RESPONSE                       0xF1
#define PROTOCOL_ALIVE_MESSAGE                           0x02
#define PROTOCOL_ALIVE_RESPONSE                          0xF2
#define PROTOCOL_CHANNEL_MESSAGE                         0x03
#define PROTOCOL_CHANNEL_RESPONSE                        0xF3
#define PROTOCOL_UNKNOWN_RESPONSE                        0xFF
#endif

/******************************************************************
 *
 * Function Name : bt_transmitter_base_pcaket_process
 *
 *
 ******************************************************************/
void bt_transmitter_channel_pcaket_process(BT_PROTOCOL_CHANNEL_DATA* pBtProtocolChannelData)
{
#if defined(DEBUG_RTT_DEBUG)
   NRF_LOG_PRINTF("[%s-%d] packet cmd : 0x%02x, %d \r\n", __FUNCTION__, __LINE__, pBtProtocolChannelData->base_protocol.command);
#endif

}

/******************************************************************
 *
 * Function Name : bt_transmitter_parser
 *
 *
 ******************************************************************/
void bt_transmitter_parser()
{

   if (g_bt_Project->bt_protocol_data.complete_packet == 1)
   {
      if (g_bt_Project->bt_protocol_data.bt_protocol_channel_data.base_protocol.size == sizeof(BT_PROTOCOL_BASE))
      {
         bt_transmitter_base_pcaket_process(&(g_bt_Project->bt_protocol_data.bt_protocol_channel_data.base_protocol));
      }
      else if (g_bt_Project->bt_protocol_data.bt_protocol_channel_data.base_protocol.size == sizeof(BT_PROTOCOL_CHANNEL_DATA))
      {
         bt_transmitter_channel_pcaket_process(&(g_bt_Project->bt_protocol_data.bt_protocol_channel_data));
      }
      else
      {
#if defined(DEBUG_RTT_ERROR)
         NRF_LOG_PRINTF("[%s-%d] missmatched packet size : $d \r\n", __FUNCTION__, __LINE__, g_bt_Project->bt_protocol_data.bt_protocol_channel_data.base_protocol.size);
#endif
      }
      g_bt_Project->bt_protocol_data.complete_packet     = 0;
   }

}


