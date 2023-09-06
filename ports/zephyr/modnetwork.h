/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Mariano Goluboff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_ZEPHYR_MODNETWORK_H
#define MICROPY_INCLUDED_ZEPHYR_MODNETWORK_H

#ifdef MICROPY_PY_NETWORK_WLAN
#include <zephyr/net/wifi.h>
#include "py/ringbuf.h"

#define MP_WLAN_IRQ_STA_CONNECT         (1)
#define MP_WLAN_IRQ_STA_DISCONNECT      (2)

typedef struct {
    mp_obj_t irq_handler;
} mp_obj_network_wlan_settings_t;

typedef struct _wlan_if_obj_t {
    mp_obj_base_t base;
    int if_id;
    mp_obj_network_wlan_settings_t *settings;
} wlan_if_obj_t;
#endif // MICROPY_PY_NETWORK_WLAN

#if MICROPY_PY_NETWORK_NRF91
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>

#define LTE_MODE_LTEM   1
#define LTE_MODE_NBIOT  2
#define LTE_MODE_GPS    4
#define LTE_PLMN_PREF   4

#endif // MICROPY_PY_NETWORK_NRF91

void usocket_events_deinit(void);

#endif
