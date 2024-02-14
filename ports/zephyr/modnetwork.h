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

MP_DECLARE_CONST_FUN_OBJ_0(zephyr_network_initialize_obj);
#endif // MICROPY_PY_NETWORK_WLAN

#if MICROPY_PY_NETWORK_NRF91

#ifdef CONFIG_LOCATION
#define MP_CELL_IRQ_NW_REG_STATUS       (0x1)
#define MP_CELL_IRQ_PSM_UPDATE          (0x2)
#define MP_CELL_IRQ_EDRX_UPDATE         (0x4)
#define MP_CELL_IRQ_RRC_UPDATE          (0x8)
#define MP_CELL_IRQ_CELL_UPDATE         (0x10)
#define MP_CELL_IRQ_LTE_MODE_UPDATE     (0x20)
#define MP_CELL_IRQ_TAU_PRE_WARN        (0x40)
#define MP_CELL_IRQ_NEIGHBOR_CELL_MEAS  (0x80)
#define MP_CELL_IRQ_LOCATION_FOUND      (0x100)
#define MP_CELL_IRQ_LOCATION_TIMEOUT    (0x200)
#define MP_CELL_IRQ_LOCATION_ERROR      (0x400)
#define MP_CELL_IRQ_GNSS_ASSISTANCE_REQUEST (0x800)
#define MP_CELL_IRQ_CLOUD_LOCATION_REQUEST  (0x1000)
#endif // CONFIG_LOCATION

#define LTE_MODE_LTEM   1
#define LTE_MODE_NBIOT  2
#define LTE_MODE_GPS    4
#define LTE_PLMN_PREF   4

#ifdef CONFIG_PDN
#include <modem/pdn.h>
#endif

#endif // MICROPY_PY_NETWORK_NRF91

void usocket_events_deinit(void);

#endif
