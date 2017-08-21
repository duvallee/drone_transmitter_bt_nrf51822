
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

#define DEBUG_OUTPUT_TIMER

/******************************************************************
 *
 * Function Name : add_timer
 *
 *
 ******************************************************************/
int add_timer(BT_TRANSMITTER_TIMER_FN fn, uint32_t ms_elapse, int count, void* pdata)
{
   uint32_t i                                            = 0;
   UNUSED_VARIABLE(i);

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

   for (i = 0; i < BT_TRANSMITTER_MAX_TIMER; i++)
   {
#if defined(DEBUG_OUTPUT_TIMER)
      NRF_LOG_PRINTF("[%s-%d] [%d] : %p \r\n", __FUNCTION__, __LINE__, i, g_bt_Project->timer[i].fn_timer);
#endif
      if (g_bt_Project->timer[i].is_allocate == 0)
      {
         g_bt_Project->timer[i].is_allocate                  = 1;
         g_bt_Project->timer[i].tick_count                   = 0;
         g_bt_Project->timer[i].ms_elapse                    = ms_elapse;
         g_bt_Project->timer[i].call_count                   = count;
         g_bt_Project->timer[i].fn_timer                     = fn;
         g_bt_Project->timer[i].pData                        = pdata;
         return 0;
      }
   }
#if defined(DEBUG_RTT_ERROR)
   NRF_LOG_PRINTF("[%s-%d] timer is full (max timer count = %d) \r\n", __FUNCTION__, __LINE__, BT_TRANSMITTER_MAX_TIMER);
#endif
   return -1;
}

/******************************************************************
 *
 * Function Name : delete_timer
 *
 *
 ******************************************************************/
void delete_timer(BT_TRANSMITTER_TIMER_FN fn)
{
   uint32_t i                                            = 0;
   UNUSED_VARIABLE(i);

   if (fn == NULL)
   { 
#if defined(DEBUG_RTT_DEBUG)
      NRF_LOG_PRINTF("[%s-%d] function is null \r\n", __FUNCTION__, __LINE__);
#endif
      return;
   }

   for (i = 0; i < BT_TRANSMITTER_MAX_TIMER; i++)
   {
      if (g_bt_Project->timer[i].fn_timer == fn)
      {
         g_bt_Project->timer[i].is_allocate                  = 01;
         g_bt_Project->timer[i].tick_count                   = 0;
         g_bt_Project->timer[i].ms_elapse                    = 0;
         g_bt_Project->timer[i].call_count                   = 0;
         g_bt_Project->timer[i].fn_timer                     = NULL;
         g_bt_Project->timer[i].pData                        = NULL;
      }
   }
}


/******************************************************************
 *
 * Function Name : bt_transmitter_timer_handler()
 *
 *
 ******************************************************************/
void bt_transmitter_timer_handler(void* p_context)
{
   uint32_t tick_count                                   = 0;
   uint32_t diff_tick_count                              = 0;
   uint32_t diff_milli_second                            = 0;
   uint32_t i                                            = 0;

   UNUSED_VARIABLE(tick_count);
   UNUSED_VARIABLE(diff_tick_count);
   UNUSED_VARIABLE(diff_milli_second);
   UNUSED_VARIABLE(i);

   app_timer_cnt_get(&tick_count);
   if (g_bt_Project->tick_count < tick_count)
   {
      g_bt_Project->tick_count                           = tick_count;

   }
   else     // reset system time ... max about 36 hours
   {
      g_bt_Project->system_time.milli_second             = 0;
      g_bt_Project->system_time.second                   = 0;
      g_bt_Project->tick_count                           = tick_count;
   }

   g_bt_Project->system_time.second                      = (g_bt_Project->tick_count / APP_TIMER_CLOCK_FREQ);
   g_bt_Project->system_time.milli_second                = ((g_bt_Project->tick_count * 1000) % APP_TIMER_CLOCK_FREQ);
   g_bt_Project->system_time.milli_second                %= MILLISECOND;

   for (i = 0; i < BT_TRANSMITTER_MAX_TIMER; i++)
   {
      if (g_bt_Project->timer[i].is_allocate != 0)
      {
         diff_tick_count                                 = g_bt_Project->tick_count - g_bt_Project->timer[i].tick_count;
         diff_milli_second                               = ((diff_tick_count * 1000) / (APP_TIMER_CLOCK_FREQ));
         if (diff_milli_second > g_bt_Project->timer[i].ms_elapse)
         {
            g_bt_Project->timer[i].tick_count            = g_bt_Project->tick_count;
            if (g_bt_Project->timer[i].call_count < 0)
            {
               (*g_bt_Project->timer[i].fn_timer)(g_bt_Project->timer[i].pData);
            }
            else if (g_bt_Project->timer[i].call_count > g_bt_Project->timer[i].call_fn_count)
            {
               (*g_bt_Project->timer[i].fn_timer)(g_bt_Project->timer[i].pData);
               g_bt_Project->timer[i].call_fn_count++;
            }
            else
            {
               g_bt_Project->timer[i].is_allocate        = 0;
            }
         }
      }
   }
}


 
