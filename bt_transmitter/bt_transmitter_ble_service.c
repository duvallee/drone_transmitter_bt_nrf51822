
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

/******************************************************************
 *
 * Function Name : bt_transmitter_ble_data_handler()
 *
 *
 ******************************************************************/
void bt_transmitter_ble_data_handler(ble_nus_t* p_nus, uint8_t* p_data, uint16_t length)
{
   uint32_t i                                            = 0;
   uint8_t first_sync                                    = 0;
   uint8_t second_sync                                   = 0;
   BT_PROTOCOL_BASE* pBt_Protocol_Base                   = NULL;
   UNUSED_VARIABLE(i);

   if (g_bt_Project->bt_protocol_data.remain_packet_size > 0)
   {
      if (g_bt_Project->bt_protocol_data.remain_packet_size == length)
      {
         uint8_t* p_des_data                             = (uint8_t*) &(g_bt_Project->bt_protocol_data.bt_protocol_channel_data);
         memcpy ((p_des_data + g_bt_Project->bt_protocol_data.received_packet_size), p_data, length);
         g_bt_Project->bt_protocol_data.remain_packet_size = 0;
         g_bt_Project->bt_protocol_data.received_packet_size = 0;
         g_bt_Project->bt_protocol_data.complete_packet  = 1;
      }
      else
      {
#if defined(DEBUG_RTT_ERROR)
         NRF_LOG_PRINTF("[%s-%d] size missmatched : %d \r\n", __FUNCTION__, __LINE__, length);
#endif
         g_bt_Project->bt_protocol_data.remain_packet_size = 0;
         g_bt_Project->bt_protocol_data.received_packet_size = 0;
         g_bt_Project->bt_protocol_data.complete_packet  = 0;
      }
   }

   for (i = 0; i < length; i++)
   {
      if ((*(p_data + i)) == PROTOCOL_HEADER_HIGH_SYNC)
      {
         first_sync                                      = 1;
         continue;
      }
      if (first_sync == 1)
      {
         if ((*(p_data + i)) == PROTOCOL_HEADER_LOW_SYNC)
         {
            second_sync                                  = 1;
            i                                            -= 1;
            break;
         }
      }
   }

   // found sync of packet (0xD7 0x5E)
   if (second_sync == 1)
   {
      uint8_t* p_des_data                                = (uint8_t*) &(g_bt_Project->bt_protocol_data.bt_protocol_channel_data);
      pBt_Protocol_Base                                  = (BT_PROTOCOL_BASE *) &(p_data[i]);

      if (pBt_Protocol_Base->size == length)
      {
         memcpy (p_des_data, p_data, length);
         g_bt_Project->bt_protocol_data.remain_packet_size = 0;
         g_bt_Project->bt_protocol_data.received_packet_size = 0;
         g_bt_Project->bt_protocol_data.complete_packet  = 1;
      }
      else
      {
         memcpy (p_des_data, p_data, length);
         g_bt_Project->bt_protocol_data.remain_packet_size = pBt_Protocol_Base->size - length;
         g_bt_Project->bt_protocol_data.received_packet_size = length;
         g_bt_Project->bt_protocol_data.complete_packet  = 0;
      }

#if defined(DEBUG_RTT_DEBUG)
      NRF_LOG_PRINTF("[%s-%d] received len : %d \r\n", __FUNCTION__, __LINE__, length);
      NRF_LOG_PRINTF("[%s-%d] packet cmd : 0x%02x, %d \r\n", __FUNCTION__, __LINE__, pBt_Protocol_Base->command, pBt_Protocol_Base->size);
#endif
   }

#if 0
#if defined(DEBUG_RTT_DEBUG)
   NRF_LOG_PRINTF("[%s-%d] received data : [0x%02X] [0x%02X]  %d \r\n", __FUNCTION__, __LINE__, *(p_data), *(p_data + 1), length);
#endif
#endif

}



