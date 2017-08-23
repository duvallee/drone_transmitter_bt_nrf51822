
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
#include "bt_transmitter_ble_service.h"
#include "bt_transmitter_parser.h"

/******************************************************************
 *
 * Function Name : bt_transmitter_ble_data_handler()
 *
 *
 ******************************************************************/
#define BT_BASE_PROTOCOL_LENGTH                          8
#define BT_CHANNEL_PROTOCOL_LENGTH                       16
void bt_transmitter_ble_data_handler(ble_nus_t* p_nus, uint8_t* p_data, uint16_t length)
{
   uint32_t i                                            = 0;
   BT_PROTOCOL_BASE* pBt_Protocol_Base                   = (BT_PROTOCOL_BASE*) p_data;
   UNUSED_VARIABLE(i);

#if 0
#if defined(DEBUG_RTT_DEBUG)
   NRF_LOG_PRINTF("[%s-%d] received len : %d \r\n", __FUNCTION__, __LINE__, length);
#endif
#endif

   if (g_bt_Project->bt_protocol_data.complete_packet == 1)
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] buffer overrun : %d \r\n", __FUNCTION__, __LINE__, length);
#endif
      bt_transmitter_send_response(PROTOCOL_UNKNOWN_RESPONSE, 0, PROTOCOL_BUSY_ERROR, 0, 0);
      return;
   }

   if (length == BT_BASE_PROTOCOL_LENGTH)
   {
      if (pBt_Protocol_Base->high_sync_byte == PROTOCOL_HEADER_HIGH_SYNC &&
          pBt_Protocol_Base->low_sync_byte == PROTOCOL_HEADER_LOW_SYNC)
      {
         memcpy(&(g_bt_Project->bt_protocol_data.bt_protocol_channel_data), p_data, length);
         g_bt_Project->bt_protocol_data.received_packet_size = length;
         g_bt_Project->bt_protocol_data.complete_packet  = 1;
      }
      else
      {
#if defined(DEBUG_RTT_ERROR)
         NRF_LOG_PRINTF("[%s-%d] can found sync byte : 0x%02x, 0x%02x \r\n", __FUNCTION__, __LINE__, pBt_Protocol_Base->high_sync_byte,
                                                                                                     pBt_Protocol_Base->low_sync_byte);
#endif
         g_bt_Project->bt_protocol_data.received_packet_size = 0;
         g_bt_Project->bt_protocol_data.complete_packet  = 0;
      }
   }
   else if (length == BT_CHANNEL_PROTOCOL_LENGTH)
   {
      if (pBt_Protocol_Base->high_sync_byte == PROTOCOL_HEADER_HIGH_SYNC &&
          pBt_Protocol_Base->low_sync_byte == PROTOCOL_HEADER_LOW_SYNC)
      {
         memcpy(&(g_bt_Project->bt_protocol_data.bt_protocol_channel_data), p_data, length);
         g_bt_Project->bt_protocol_data.received_packet_size = length;
         g_bt_Project->bt_protocol_data.complete_packet  = 0;
      }
      else
      {
         memcpy((((uint8_t *) (&(g_bt_Project->bt_protocol_data.bt_protocol_channel_data))) + length),
                  p_data, length);
         g_bt_Project->bt_protocol_data.received_packet_size += length;
         g_bt_Project->bt_protocol_data.complete_packet  = 1;
      }
   }
   else
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] Unknown size : %d \r\n", __FUNCTION__, __LINE__, length);
#endif
      g_bt_Project->bt_protocol_data.received_packet_size = 0;
      g_bt_Project->bt_protocol_data.complete_packet     = 0;
      bt_transmitter_send_response(PROTOCOL_UNKNOWN_RESPONSE, 0, PROTOCOL_MISSMATCHED_SIZE, 0, 0);
   }
}



