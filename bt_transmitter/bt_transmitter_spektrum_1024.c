

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

void send_msg_to_fc_controller(uint8_t* pdata, uint8_t bytes);

// ----------------------------------------------------------------
#define MAX_SPEKTRUM_CHANNEL_NUM                         7
#define SPEKTRUM_1024_CHANID_MASK                        0xFC00
#define SPEKTRUM_1024_SXPOS_MASK                         0x03FF
#define SPEKTRUM_1024_CHANNEL_SHIFT_BITS                 10

#define SPEKTRUM_DSM2_22MS                               0x01                    // 1024
#define SPEKTRUM_DSM2_11MS                               0x12                    // 2048
#define SPEKTRUM_DSMS_22MS                               0xA2                    // 2048
#define SPEKTRUM_DSMX_11MS                               0xB2                    // 2048

#define SPEKTRUM_1024_SERIAL_RX_TIME                     22                      // 22 ms

// struct for SPEKTRUM 1024 Protocol
typedef struct _RX_PACKET_SPEKTRUM_1024
{
   uint8_t fades;
   uint8_t system;
   uint16_t channel[MAX_SPEKTRUM_CHANNEL_NUM];
} __attribute__ ((__packed__)) RX_PACKET_SPEKTRUM_1024;

static RX_PACKET_SPEKTRUM_1024 g_RxPacket                = {0, };

/******************************************************************
 *
 * Function Name : bt_transmitter_serial_rx_spektrum_1024_timer
 *
 *
 ******************************************************************/
void bt_transmitter_serial_rx_spektrum_1024_timer(void* pArg)
{

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

   send_msg_to_fc_controller((uint8_t *) &g_RxPacket, sizeof(g_RxPacket));
}

/******************************************************************
 *
 * Function Name : bt_transmitter_make_packet
 *
 *
 ******************************************************************/
void bt_transmitter_make_packet()
{
   int i;
   uint16_t* pChannelData                                   = &(g_bt_Project->bt_protocol_packet.throttle);

   if ((g_bt_Project->bt_protocol_packet.header & 0xFFF0) == PROTOCOL_VERSION && g_bt_Project->bt_connected == 1)
   {
      g_RxPacket.fades                                      = 0;
      g_RxPacket.system                                     = SPEKTRUM_DSM2_22MS;

      for (i = 0; i < MAX_SPEKTRUM_CHANNEL_NUM; i++)
      {
         g_RxPacket.channel[i]                              = ((*pChannelData >> 14) & 0x00FC) |
                                                              ((*pChannelData <<  8) & 0xFF00) |
                                                              ((*pChannelData >>  8) & 0x0003);
      }
   }
   else
   {
      memset(&g_RxPacket, 0, sizeof(RX_PACKET_SPEKTRUM_1024));
      g_RxPacket.fades                                      = 0;
      g_RxPacket.system                                     = SPEKTRUM_DSM2_22MS;

      for (i = 0; i < MAX_SPEKTRUM_CHANNEL_NUM; i++)
      {
         g_RxPacket.channel[i]                              = ((i >> 14) & 0x00FC);
      }
      return;
   }
}

/******************************************************************
 *
 * Function Name : fc_serial_rx_init
 *
 *
 ******************************************************************/
void fc_serial_rx_init(void)
{
   bt_transmitter_make_packet();
   add_timer(bt_transmitter_serial_rx_spektrum_1024_timer, SPEKTRUM_1024_SERIAL_RX_TIME, -1, (void*) NULL);
}

#endif


