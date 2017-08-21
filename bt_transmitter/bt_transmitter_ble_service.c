
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
   UNUSED_VARIABLE(i);

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
      NRF_LOG_PRINTF("[%s-%d] found sync signal :  %d \r\n", __FUNCTION__, __LINE__, i);
   }

#if defined(DEBUG_RTT_DEBUG)
   NRF_LOG_PRINTF("[%s-%d] received data : [0x%02X] [0x%02X]  %d \r\n", __FUNCTION__, __LINE__, *(p_data), *(p_data + 1), length);
#endif

}



