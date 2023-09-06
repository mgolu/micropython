/*
 * This file is part of the MicroPython project, http://micropython.org/
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

STATIC const mp_obj_base_t cell_if;

static bool initialized = false;

static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
        //reg_status = evt->nw_reg_status;
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_INF("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: 
		LOG_INF("Updated eDRX");
		break;
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d",
			evt->cell.id, evt->cell.tac);
		break;
	default:
        LOG_DBG("Other LTE event: %d", evt->type);
		break;
	}
}

STATIC mp_obj_t network_cell_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    int ret;
    if (!nrf_modem_is_initialized()) {
		ret = nrf_modem_lib_init();
		if (ret) {
			mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Modem init failed"));
		}
	}
    return MP_OBJ_FROM_PTR(&cell_if);
}

STATIC mp_obj_t network_cell_active(size_t n_args, const mp_obj_t *args) {

    if (n_args > 1) {
        bool activate = mp_obj_is_true(args[1]);
        if (activate && !initialized) {
            lte_lc_register_handler(lte_handler);
            int ret = lte_lc_init();
            if (ret) {
                mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("LC Init failed"));
            }
            initialized = true;
        } else if (!activate && initialized) {
            int ret = lte_lc_deregister_handler(lte_handler);
            if (ret) {
                mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("handler deregister error: %d"), ret);
            }
            ret = lte_lc_deinit();
            if (ret) {
                mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("deinit error: %d"), ret);
            }
            initialized = false;
        }
        return mp_obj_new_bool(initialized);
    }
    return mp_obj_new_bool(initialized);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_cell_active_obj, 1, 2, network_cell_active);

STATIC mp_obj_t network_cell_connect(mp_obj_t self_in) {
    int ret = lte_lc_connect_async(NULL);
    if (ret) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Connect failed"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_cell_connect_obj, network_cell_connect);

STATIC mp_obj_t network_cell_isconnected(mp_obj_t self_in) {
    enum lte_lc_nw_reg_status reg_status;
    int ret = lte_lc_nw_reg_status_get(&reg_status);
    if (ret) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Could not get status from modem"));
    }
    if (reg_status == LTE_LC_NW_REG_REGISTERED_HOME ||
        reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING) {
            return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_cell_isconnected_obj, network_cell_isconnected);

STATIC mp_obj_t network_cell_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    if (n_args != 1 && kwargs->used != 0) {
        mp_raise_TypeError(MP_ERROR_TEXT("either pos or kw args are allowed"));
    }
    if (kwargs->used != 0) {
        for (size_t i = 0; i < kwargs->alloc; i++) {
            if (mp_map_slot_is_filled(kwargs, i)) {
                switch(mp_obj_str_get_qstr(kwargs->table[i].key)) {
                    case MP_QSTR_mode: {
                        if (!mp_obj_is_type(kwargs->table[i].value, &mp_type_tuple)) {
                            mp_raise_ValueError("mode config must be a tuple");
                        }
                        size_t len;
                        mp_obj_t *items;
                        mp_obj_tuple_get(kwargs->table[i].value, &len, &items);
                        if (len != 2) {
                            mp_raise_ValueError("mode tuple must have 2 values: (mode, preference)");
                        }
                        enum lte_lc_system_mode mode;
                        enum lte_lc_system_mode_preference preference;
                        switch (mp_obj_get_int(items[0])) {
                            case 0:
                                mode = LTE_LC_SYSTEM_MODE_NONE;
                                break;
                            case LTE_MODE_LTEM: // MP_QSTR_LTE_MODE_LTEM
                                mode = LTE_LC_SYSTEM_MODE_LTEM;
                                break;
                            case LTE_MODE_NBIOT: // MP_QSTR_LTE_MODE_NBIOT
                                mode = LTE_LC_SYSTEM_MODE_NBIOT;
                                break;
                            case LTE_MODE_LTEM | LTE_MODE_NBIOT: // MP_QSTR_LTE_MODE_LTEM | MP_QSTR_LTE_MODE_NBIOT
                                mode = LTE_LC_SYSTEM_MODE_LTEM_NBIOT;
                                break;
                            case LTE_MODE_GPS: // MP_QSTR_LTE_MODE_GPS
                                mode = LTE_LC_SYSTEM_MODE_GPS;
                                break;
                            case LTE_MODE_LTEM | LTE_MODE_GPS: // MP_QSTR_LTE_MODE_LTEM | MP_QSTR_LTE_MODE_GPS
                                mode = LTE_LC_SYSTEM_MODE_LTEM_GPS;
                                break;
                            case LTE_MODE_NBIOT | LTE_MODE_GPS: // MP_QSTR_LTE_MODE_NBIOT | MP_QSTR_LTE_MODE_GPS
                                mode = LTE_LC_SYSTEM_MODE_NBIOT_GPS;
                                break;
                            case LTE_MODE_LTEM | LTE_MODE_NBIOT | LTE_MODE_GPS: // MP_QSTR_LTE_MODE_LTEM | MP_QSTR_LTE_MODE_NBIOT | MP_QSTR_LTE_MODE_GPS
                                mode = LTE_LC_SYSTEM_MODE_LTEM_NBIOT_GPS;
                                break;
                            default:
                                mp_raise_ValueError("Mode value is not valid");
                        }
                        switch (mp_obj_get_int(items[1])) {
                            case 0:
                                preference = LTE_LC_SYSTEM_MODE_PREFER_AUTO;
                                break;
                            case LTE_MODE_LTEM: // MP_QSTR_LTE_MODE_LTEM
                                preference = LTE_LC_SYSTEM_MODE_PREFER_LTEM;
                                break;
                            case LTE_MODE_NBIOT: // MP_QSTR_LTE_MODE_NBIOT
                                preference = LTE_LC_SYSTEM_MODE_PREFER_NBIOT;
                                break;
                            case LTE_MODE_LTEM | LTE_PLMN_PREF: // MP_QSTR_LTE_MODE_LTEM | MP_QSTR_PLMN_PREF
                                preference = LTE_LC_SYSTEM_MODE_PREFER_LTEM_PLMN_PRIO;
                                break;
                            case LTE_MODE_NBIOT | LTE_PLMN_PREF: // MP_QSTR_LTE_MODE_NBIOT | MP_QSTR_PLMN_PREF
                                preference = LTE_LC_SYSTEM_MODE_PREFER_NBIOT_PLMN_PRIO;
                                break;
                            default:
                                mp_raise_ValueError("Preference value is not valid");
                        }
                        int ret = lte_lc_system_mode_set(mode, preference);
                        if (ret) {
                            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Setting mode failed"));
                        }
                        break;
                    }
                }
            }
        }
        return mp_const_none;
    }
    // Get config

    if (n_args != 2) {
        mp_raise_TypeError(MP_ERROR_TEXT("can query only one param"));
    }

    switch (mp_obj_str_get_qstr(args[1])) { 
        case MP_QSTR_mode: {
            enum lte_lc_system_mode mode;
            enum lte_lc_system_mode_preference preference;
            int ret = lte_lc_system_mode_get(&mode, &preference);
            if (ret) {
                mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Geting mode failed"));
            }
            mp_obj_t tuple[2] = {
                mp_obj_new_int_from_uint(mode),
                mp_obj_new_int_from_uint(preference),
            };
            return mp_obj_new_tuple(2, tuple);
        }
        default:
            mp_raise_ValueError(MP_ERROR_TEXT("Unknown config param"));
    }
}
MP_DEFINE_CONST_FUN_OBJ_KW(network_cell_config_obj, 1, network_cell_config);

STATIC mp_obj_t network_cell_status(size_t n_args, const mp_obj_t *args) {
    if (n_args == 1) {
        enum lte_lc_nw_reg_status reg_status;
        int ret = lte_lc_nw_reg_status_get(&reg_status);
        if (ret) {
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Could not get status from modem"));
        }
        return mp_obj_new_int_from_uint(reg_status);
    }
    // There's an argument, get that status
    switch ((uintptr_t)args[1]) {
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_mode): {
            enum lte_lc_lte_mode mode;
            int ret = lte_lc_lte_mode_get(&mode);
            if (ret) {
                mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Geting mode failed"));
            }
            switch (mode) {
                case LTE_LC_LTE_MODE_NONE:
                    return mp_obj_new_int_from_uint(0);
                case LTE_LC_LTE_MODE_LTEM:
                    return mp_obj_new_int_from_uint(LTE_MODE_LTEM);
                case LTE_LC_LTE_MODE_NBIOT:
                    return mp_obj_new_int_from_uint(LTE_MODE_NBIOT);
            }
        }
        default:
            mp_raise_ValueError(MP_ERROR_TEXT("unknown status param"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_cell_status_obj, 1, 2, network_cell_status);

STATIC const mp_rom_map_elem_t cell_if_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_cell_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&network_cell_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_cell_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_cell_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_cell_status_obj) },
/*    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&network_wlan_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan), MP_ROM_PTR(&network_wlan_scan_obj) },
    */
};
STATIC MP_DEFINE_CONST_DICT(cell_if_locals_dict, cell_if_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    zephyr_network_cell_type,
    MP_QSTR_CELL,
    MP_TYPE_FLAG_NONE,
    make_new, network_cell_make_new,
    locals_dict, &cell_if_locals_dict
    );

STATIC const mp_obj_base_t cell_if = {&zephyr_network_cell_type};

#endif // MICROPY_PY_NETWORK_NRF91