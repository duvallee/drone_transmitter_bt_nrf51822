
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
#include "bt_transmitter_spektrum_1024.h"

/******************************************************************
 *
 * Function Name : bt_transmitter_ble_data_handler()
 *
 *
 ******************************************************************/
void bt_transmitter_ble_data_handler(ble_nus_t* p_nus, uint8_t* p_data, uint16_t length)
{
#if defined(DEBUG_RTT_DEBUG)
   NRF_LOG_PRINTF("[%s-%d] received packet data : %d \r\n", __FUNCTION__, __LINE__, length);
#endif
   if (length != sizeof(BT_PROTOCOL_V1))
   {
#if defined(DEBUG_RTT_ERROR)
      NRF_LOG_PRINTF("[%s-%d] error : received packet data: %d != %d \r\n", __FUNCTION__, __LINE__, length, sizeof(BT_PROTOCOL_V1));
#endif
      return;
   }
   memcpy((uint8_t*) &(g_bt_Project->bt_protocol_packet), p_data, length);
   bt_transmitter_make_packet();
}


