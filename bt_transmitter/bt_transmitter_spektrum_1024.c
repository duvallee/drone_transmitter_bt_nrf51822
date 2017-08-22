

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

#if defined(SERIAL_RX_SPEKTRUM_1024)
#include "bt_transmitter_spektrum_1024.h"

/******************************************************************
 *
 * Function Name : bt_transmitter_serial_rx_spektrum_1024_timer
 *
 *
 ******************************************************************/
void bt_transmitter_serial_rx_spektrum_1024_timer(void* pArg)
{
   BT_TRANSMITTER_CHANNEL_INFO* pBtTransmitterChannelInfo = (BT_TRANSMITTER_CHANNEL_INFO*) pArg;
   UNUSED_VARIABLE(pBtTransmitterChannelInfo);

#if 1
#if defined(DEBUG_GPIO_PIN)
   if (nrf_gpio_pin_read(DEBUG_GPIO_PIN) == 0)
   {
      nrf_gpio_pin_write(DEBUG_GPIO_PIN, 1);
   }
   else
   {
      nrf_gpio_pin_write(DEBUG_GPIO_PIN, 0);
   }
#endif
#endif

   if (pBtTransmitterChannelInfo->channel_update)
   {
      if (pBtTransmitterChannelInfo->send_uart_fn)
      {
         (*pBtTransmitterChannelInfo->send_uart_fn)(NULL, 0);
      }
   }
}


/******************************************************************
 *
 * Function Name : fc_serial_rx_init
 *
 *
 ******************************************************************/
void fc_serial_rx_init(BT_TRANSMITTER_CHANNEL_INFO* pBtTransmitterChannelInfo)
{
   add_timer(bt_transmitter_serial_rx_spektrum_1024_timer, SPEKTRUM_1024_SERIAL_RX_TIME, -1, (void*) pBtTransmitterChannelInfo);
}

#endif


