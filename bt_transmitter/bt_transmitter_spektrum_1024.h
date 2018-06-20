
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

#ifndef _BT_TRANSMITTER_SPEKTRUM_1024_H_
#define _BT_TRANSMITTER_SPEKTRUM_1024_H_

#if defined(SERIAL_RX_SPEKTRUM_1024)
void fc_serial_rx_init(void);
void bt_transmitter_make_packet();
#endif

#endif   // _BT_TRANSMITTER_SPEKTRUM_1024_H_

