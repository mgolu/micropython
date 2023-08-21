/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 * and Mnemote Pty Ltd
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023  Mariano Goluboff
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

#include <string.h>

#include <zephyr/kernel.h>
#include "py/objlist.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/modnetwork.h"
#include "modnetwork.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mpy_network, LOG_LEVEL_DBG);

#if MICROPY_PY_NETWORK_NRF91
#include <modem/lte_lc.h>

static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
        default:
            // TODO
            break;
    }
}

STATIC mp_obj_t network_cell_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    int err;
    lte_lc_register_handler(lte_handler);
    err = lte_lc_init();
    LOG_DBG("LTE setup error: %d", err);
    return mp_const_true;
}

STATIC mp_obj_t network_cell_active(size_t n_args, const mp_obj_t *args) {

    if (n_args > 1) {
        bool activate = mp_obj_is_true(args[1]);
        if (activate) {
            lte_lc_normal();
        } else {
            lte_lc_offline();
        }
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_cell_active_obj, 1, 2, network_cell_active);

STATIC const mp_rom_map_elem_t cell_if_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_cell_active_obj) },
 /*   { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&network_wlan_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&network_wlan_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_wlan_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan), MP_ROM_PTR(&network_wlan_scan_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_wlan_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_wlan_config_obj) },*/
};
STATIC MP_DEFINE_CONST_DICT(cell_if_locals_dict, cell_if_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    zephyr_network_cell_type,
    MP_QSTR_CELL,
    MP_TYPE_FLAG_NONE,
    make_new, network_cell_make_new,
    locals_dict, &cell_if_locals_dict
    );

#endif // MICROPY_PY_NETWORK_NRF91