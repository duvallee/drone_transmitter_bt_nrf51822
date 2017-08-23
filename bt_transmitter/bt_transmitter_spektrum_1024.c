

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


// ----------------------------------------------------------------
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

static RX_PACKET_SPEKTRUM_1024 g_RxPacket                = {0, };
static RX_PACKET_SPEKTRUM_1024 g_SendRxPacket            = {0, };

static uint8_t g_Active_Transmitter                      = 0;
static uint8_t g_ChannelDataUpdating                     = 0;
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

   if (g_Active_Transmitter == 1)
   {
      if (g_ChannelDataUpdating == 0)
      {
         memcpy(&g_SendRxPacket, &g_RxPacket, sizeof(g_RxPacket));
      }
      (*pBtTransmitterChannelInfo->send_uart_fn)((uint8_t *) &g_SendRxPacket, sizeof(g_SendRxPacket));
   }
}

/******************************************************************
 *
 * Function Name : bt_transmitter_make_packet
 *
 *
 ******************************************************************/
void bt_transmitter_make_packet(BT_TRANSMITTER_CHANNEL_INFO* pBtTransmitterChannelInfo)
{
   uint16_t i;

   if (pBtTransmitterChannelInfo->channel_data_complete == 1)
   {
      g_ChannelDataUpdating                              = 1;
      g_RxPacket.fades                                   = 0;
      g_RxPacket.system                                  = SPEKTRUM_DSM2_22MS;

      for (i = 0; i < MAX_SPEKTRUM_CHANNEL_NUM; i++)
      {
         g_RxPacket.channel[i]                           = ((pBtTransmitterChannelInfo->channel[i] >> 14) & 0x00FC) |
                                                           ((pBtTransmitterChannelInfo->channel[i] <<  8) & 0xFF00) |
                                                           ((pBtTransmitterChannelInfo->channel[i] >>  8) & 0x0003);
#if 0
#if defined(DEBUG_RTT_DEBUG)
         NRF_LOG_PRINTF("[%s-%d] [%d] : 0x%04X (%d) \r\n", __FUNCTION__, __LINE__, i, g_RxPacket.channel[i], sizeof(g_SendRxPacket));
#endif
#endif
      }
      g_ChannelDataUpdating                              = 0;
      pBtTransmitterChannelInfo->channel_data_complete = 0;
   }
   if (pBtTransmitterChannelInfo->transmitter_start == 1)
   {
      g_Active_Transmitter                               = 1;
   }
   else
   {
      memset(&g_RxPacket, 0, sizeof(g_RxPacket));
      g_Active_Transmitter                               = 0;
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


