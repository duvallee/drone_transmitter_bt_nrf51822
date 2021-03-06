
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

#ifndef _BT_TRANSMITTER_TIMER_H_
#define _BT_TRANSMITTER_TIMER_H_

int add_timer(BT_TRANSMITTER_TIMER_FN fn, uint32_t ms_elapse, int count, void* pdata);
void delete_timer(BT_TRANSMITTER_TIMER_FN fn);

void bt_transmitter_timer_handler(void* p_context);

#endif   // _BT_TRANSMITTER_TIMER_H_

