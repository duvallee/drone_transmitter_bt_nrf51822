
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

   err_code                                              = ble_nus_string_send(&(g_bt_Project->ble_nordic_uart_service), (uint8_t *) &bt_protocol_base, sizeof(bt_protocol_base));

   if (err_code != NRF_SUCCESS)
   {
#if defined(DEBUG_RTT_DEBUG)
      NRF_LOG_PRINTF("[%s-%d] ble_nus_string_send() failed !!! \r\n", __FUNCTION__, __LINE__);
#endif
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
   uint32_t tick_count                                   = 0;
   uint32_t diff_tick_count                              = 0;
   uint32_t diff_milli_second                            = 0;

   UNUSED_VARIABLE(tick_count);
   UNUSED_VARIABLE(diff_tick_count);
   UNUSED_VARIABLE(diff_milli_second);

   switch (pBtProtocolBase->command)
   {
      case PROTOCOL_REGISTER_MESSAGE :
#if defined(DEBUG_RTT_DEBUG)
         NRF_LOG_PRINTF("[%s-%d] register message \r\n", __FUNCTION__, __LINE__);
#endif
         bt_transmitter_send_response(PROTOCOL_REGISTER_RESPONSE, 0, PROTOCOL_SUCCESS, 0, 0);
         break;

      case PROTOCOL_ALIVE_MESSAGE :
         app_timer_cnt_get(&tick_count);

#if 0
         if (g_bt_Project->alive_receive_tick_count == 0)
         {
            g_bt_Project->alive_receive_tick_count       = tick_count;
         }
//         app_timer_cnt_diff_compute(g_bt_Project->alive_receive_tick_count, tick_count, &diff_tick_count);
         diff_tick_count                                 = tick_count - g_bt_Project->alive_receive_tick_count;
         diff_milli_second                               = ((diff_tick_count * 1000) / APP_TIMER_CLOCK_FREQ);
#if defined(DEBUG_RTT_DEBUG)
         NRF_LOG_PRINTF("[%s-%d] alive message : %d ms (%d - %d = %d) \r\n", __FUNCTION__, __LINE__, diff_milli_second,
                                                                             g_bt_Project->alive_receive_tick_count,
                                                                             tick_count,
                                                                             diff_tick_count);
#endif
#endif
         g_bt_Project->alive_receive_tick_count          = tick_count;

#if defined(DEBUG_GPIO_EXT_PIN)
         if (nrf_gpio_pin_read(DEBUG_GPIO_EXT_PIN) == 0)
         {
            nrf_gpio_pin_write(DEBUG_GPIO_EXT_PIN, 1);
         }
         else
         {
            nrf_gpio_pin_write(DEBUG_GPIO_EXT_PIN, 0);
         }
#endif
         bt_transmitter_send_response(PROTOCOL_ALIVE_RESPONSE, 0, PROTOCOL_SUCCESS, 0, 0);
         break;

      default :
         bt_transmitter_send_response(PROTOCOL_UNKNOWN_RESPONSE, 0, PROTOCOL_UNKNOWN_COMMAND, 0, 0);
         break;
   }
}


/******************************************************************
 *
 * Function Name : bt_transmitter_base_pcaket_process
 *
 *
 ******************************************************************/
void bt_transmitter_channel_pcaket_process(BT_PROTOCOL_CHANNEL_DATA* pBtProtocolChannelData)
{
   g_bt_Project->update_channel_data_for_fc.channel_num  = 12;

   memset(g_bt_Project->update_channel_data_for_fc.channel, 0, sizeof(g_bt_Project->update_channel_data_for_fc.channel));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_1 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_1 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_1 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_1 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_2 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_2 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_2 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_2 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_3 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_3 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_3 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_3 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_4 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_4 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_4 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_4 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_5 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_5 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_5 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_5 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_6 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_6 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_6 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_6 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_7 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_7 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_7 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_7 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_8 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_8 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_8 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_8 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_9 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_9 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_9 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_9 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_10 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_10 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_10 << 8) & 0x0F00) | ((pBtProtocolChannelData->channel_10 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_11 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_11 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_11 << 8) & 0x0F00) | ((pBtProtocolChannelData->channel_11 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel[((pBtProtocolChannelData->channel_12 >> 4) & 0xF)]  =
                  (((pBtProtocolChannelData->channel_12 << 12) & 0xF0000) |
                  (((pBtProtocolChannelData->channel_12 <<  8) & 0x0F00) | ((pBtProtocolChannelData->channel_12 >> 8) & 0xFF)));

   g_bt_Project->update_channel_data_for_fc.channel_data_complete = 1;
   g_bt_Project->update_channel_data_for_fc.transmitter_start = 1;

#if 0
#if defined(DEBUG_RTT_DEBUG)
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[0]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[1]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[2]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[3]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[4]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[5]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[7]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[8]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[9]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[10]);
   NRF_LOG_PRINTF("[%s-%d] 0x%08X \r\n", __FUNCTION__, __LINE__, g_bt_Project->update_channel_data_for_fc.channel[11]);
#endif
#endif

   bt_transmitter_send_response(PROTOCOL_CHANNEL_RESPONSE, 0, PROTOCOL_SUCCESS, 0, 0);
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
         bt_transmitter_send_response(PROTOCOL_UNKNOWN_RESPONSE, 0, PROTOCOL_MISSMATCHED_SIZE, 0, 0);
      }
      g_bt_Project->bt_protocol_data.received_packet_size = 0;
      g_bt_Project->bt_protocol_data.complete_packet     = 0;
   }
}


