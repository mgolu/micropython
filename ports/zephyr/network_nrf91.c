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

#if MICROPY_PY_NETWORK_NRF91
#include <string.h>

#include <zephyr/kernel.h>
#include "py/objlist.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/modnetwork.h"
#include "modnetwork.h"

#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <modem/lte_lc.h>

#include <zephyr/logging/log.h>

#ifdef CONFIG_LOCATION
#include <modem/location.h>
#endif

LOG_MODULE_REGISTER(mpy_network, LOG_LEVEL_DBG);

STATIC const mp_obj_base_t cell_if;

static bool initialized = false;
static mp_obj_t irq_handler = mp_const_none;
static uint32_t irq_mask = 0x0;

STATIC mp_obj_t cell_invoke_irq(mp_obj_t arg) {
    // Send a tuple with the following:
    // ( IRQ event, data)
    // Can skip the data if there's none
    if (irq_handler == mp_const_none) {
        return mp_const_none;
    }
    mp_obj_t *items;
    size_t len;
    mp_obj_get_array(arg, &len, &items);

    mp_obj_t event = items[0];
    mp_obj_t data = (len == 2) ? items[1] : mp_const_none;

    mp_call_function_2(irq_handler, event, data);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cell_invoke_irq_obj, cell_invoke_irq);

static void lte_handler(const struct lte_lc_evt *const evt)
{
    if (irq_handler != mp_const_none && irq_mask != 0) {
        mp_obj_t tuple[2] = { 0, 0 };
        switch (evt->type) {
        case LTE_LC_EVT_NW_REG_STATUS:
            if (irq_mask & MP_CELL_IRQ_NW_REG_STATUS) {
                tuple[0] = mp_obj_new_int(MP_CELL_IRQ_NW_REG_STATUS);
                tuple[1] = MP_OBJ_NEW_SMALL_INT(evt->nw_reg_status);
            }
            break;
        case LTE_LC_EVT_PSM_UPDATE:
        {
            if (irq_mask & MP_CELL_IRQ_PSM_UPDATE) {
            tuple[0] = mp_obj_new_int(MP_CELL_IRQ_PSM_UPDATE);
            mp_obj_t params[2] = {
                mp_obj_new_int(evt->psm_cfg.tau), 
                mp_obj_new_int(evt->psm_cfg.active_time),
            };
            tuple[1] = mp_obj_new_tuple(2, params);
            }
            break;
        }
        case LTE_LC_EVT_EDRX_UPDATE: 
            LOG_INF("Updated eDRX");
            break;
        case LTE_LC_EVT_RRC_UPDATE:
            if (irq_mask & MP_CELL_IRQ_RRC_UPDATE) {
                tuple[0] = mp_obj_new_int(MP_CELL_IRQ_RRC_UPDATE);
                tuple[1] = mp_obj_new_bool(evt->rrc_mode);
            }
            break;
        case LTE_LC_EVT_CELL_UPDATE:
        {
            if (irq_mask & MP_CELL_IRQ_CELL_UPDATE) {
                tuple[0] = mp_obj_new_int(MP_CELL_IRQ_CELL_UPDATE);
                mp_obj_t params[2] = {
                    mp_obj_new_int(evt->cell.id), 
                    mp_obj_new_int(evt->cell.tac),
                };
                tuple[1] = mp_obj_new_tuple(2, params);
            }
            break;
        }
        case LTE_LC_EVT_LTE_MODE_UPDATE:
            LOG_INF("LTE mode update %d", evt->lte_mode);
            break;
        default:
            LOG_DBG("Other LTE event: %d", evt->type);
            break;
        }
        if (tuple[0] != 0) {
            mp_sched_schedule(MP_OBJ_FROM_PTR(&cell_invoke_irq_obj), mp_obj_new_tuple(2, tuple));
        }
    }
}
#ifdef CONFIG_LOCATION
static void location_event_handler(const struct location_event_data *event_data)
{
	switch (event_data->id) {
	case LOCATION_EVT_LOCATION: {
        if (irq_mask & MP_CELL_IRQ_LOCATION_FOUND) {
            mp_obj_t tuple[2] = { 0, 0 };
            tuple[0] = mp_obj_new_int(MP_CELL_IRQ_LOCATION_FOUND);
            mp_obj_t params[11];
            const char *method_str = location_method_str(event_data->method);
            params[0] = mp_obj_new_str(method_str, strlen(method_str));
            params[1] = mp_obj_new_float(event_data->location.latitude);
            params[2] = mp_obj_new_float(event_data->location.longitude);
            params[3] = mp_obj_new_float(event_data->location.accuracy);
            if (event_data->location.datetime.valid) {
                params[4] = mp_obj_new_int(event_data->location.datetime.year);
                params[5] = mp_obj_new_int(event_data->location.datetime.month);
                params[6] = mp_obj_new_int(event_data->location.datetime.day);
                params[7] = mp_obj_new_int(event_data->location.datetime.hour);
                params[8] = mp_obj_new_int(event_data->location.datetime.minute);
                params[9] = mp_obj_new_int(event_data->location.datetime.second);
                params[10] = mp_obj_new_int(event_data->location.datetime.ms);
                tuple[1] = mp_obj_new_tuple(11, params);
            } else {
                tuple[1] = mp_obj_new_tuple(4, params);
            }
            mp_sched_schedule(MP_OBJ_FROM_PTR(&cell_invoke_irq_obj), mp_obj_new_tuple(2, tuple));
        }
		break;
    }
	case LOCATION_EVT_TIMEOUT: {
        if (irq_mask & MP_CELL_IRQ_LOCATION_TIMEOUT) {
            mp_obj_t tuple[1] = { mp_obj_new_int(MP_CELL_IRQ_LOCATION_TIMEOUT)};
            mp_sched_schedule(MP_OBJ_FROM_PTR(&cell_invoke_irq_obj), mp_obj_new_tuple(1, tuple));
        }
		break;
    }

	case LOCATION_EVT_ERROR:
		if (irq_mask & MP_CELL_IRQ_LOCATION_ERROR) {
            mp_obj_t tuple[1] = { mp_obj_new_int(MP_CELL_IRQ_LOCATION_ERROR)};
            mp_sched_schedule(MP_OBJ_FROM_PTR(&cell_invoke_irq_obj), mp_obj_new_tuple(1, tuple));
        }
		break;

	default:
		LOG_WRN("Getting location: Unknown event");
		break;
	}
}
#endif

STATIC mp_obj_t network_cell_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    int ret;
    if (!nrf_modem_is_initialized()) {
		ret = nrf_modem_lib_init();
		if (ret) {
			mp_raise_OSError(ret);
		}
	}
#ifdef CONFIG_LOCATION
    location_init(location_event_handler);
#endif
    return MP_OBJ_FROM_PTR(&cell_if);
}

STATIC mp_obj_t network_cell_active(size_t n_args, const mp_obj_t *args) {

    if (n_args > 1) {
        bool activate = mp_obj_is_true(args[1]);
        if (activate && !initialized) {
            lte_lc_register_handler(lte_handler);
            int ret = lte_lc_init();
            if (ret) {
                mp_raise_OSError(ret);
            }
            initialized = true;
        } else if (!activate && initialized) {
            int ret = lte_lc_deregister_handler(lte_handler);
            if (ret) {
                mp_raise_OSError(ret);
            }
            ret = lte_lc_deinit();
            if (ret) {
                mp_raise_OSError(ret);
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
        mp_raise_OSError(ret);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_cell_connect_obj, network_cell_connect);

STATIC mp_obj_t network_cell_isconnected(mp_obj_t self_in) {
    enum lte_lc_nw_reg_status reg_status;
    int ret = lte_lc_nw_reg_status_get(&reg_status);
    if (ret) {
        mp_raise_OSError(ret);
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
                            mp_raise_OSError(ret);
                        }
                        break;
                    }
                    case MP_QSTR_psm_params: {
                        if (!mp_obj_is_type(kwargs->table[i].value, &mp_type_tuple)) {
                            mp_raise_ValueError("PSM params must be a tuple with TAU and RAT");
                        }
                        size_t len;
                        mp_obj_t *items;
                        mp_obj_tuple_get(kwargs->table[i].value, &len, &items);
                        if (len != 2) {
                            mp_raise_ValueError("PSM params tuple must have 2 values: (TAU, RAT)");
                        }
                        if (!mp_obj_is_str(items[0]) || !mp_obj_is_str(items[1])) {
                            mp_raise_ValueError("TAU and RAT must be strings");
                        }
                        int ret = lte_lc_psm_param_set(mp_obj_str_get_str(items[0]), mp_obj_str_get_str(items[1]));
                        if (ret) {
                            mp_raise_ValueError("Input parameter not valid");
                        }
                        break;
                    }
                    case MP_QSTR_psm_enable: {
                        int ret = lte_lc_psm_req(mp_obj_is_true(kwargs->table[i].value));
                        if (ret) {
                            mp_raise_OSError(ret);
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
                mp_raise_OSError(ret);
            }
            uint8_t mode_ret = 0;
            uint8_t pref_ret = 0;
            switch (mode) {
                case LTE_LC_SYSTEM_MODE_LTEM:
                    mode_ret = LTE_MODE_LTEM;
                    break;
                case LTE_LC_SYSTEM_MODE_NBIOT:
                    mode_ret = LTE_MODE_NBIOT;
                    break;
                case LTE_LC_SYSTEM_MODE_GPS:
                    mode_ret = LTE_MODE_GPS;
                    break;
                case LTE_LC_SYSTEM_MODE_LTEM_GPS:
                    mode_ret = LTE_MODE_LTEM | LTE_MODE_GPS;
                    break;
                case LTE_LC_SYSTEM_MODE_NBIOT_GPS:
                    mode_ret = LTE_MODE_NBIOT | LTE_MODE_GPS;
                    break;
                case LTE_LC_SYSTEM_MODE_LTEM_NBIOT:
                    mode_ret = LTE_MODE_LTEM | LTE_MODE_NBIOT;
                    break;
                case LTE_LC_SYSTEM_MODE_LTEM_NBIOT_GPS:
                    mode_ret = LTE_MODE_LTEM | LTE_MODE_NBIOT | LTE_MODE_GPS;
                    break;
            }
            switch (preference) {
                case LTE_LC_SYSTEM_MODE_PREFER_AUTO:
                    pref_ret = 0;
                    break;
                case LTE_LC_SYSTEM_MODE_PREFER_LTEM:
                    pref_ret = LTE_MODE_LTEM;
                    break;
                case LTE_LC_SYSTEM_MODE_PREFER_NBIOT:
                    pref_ret = LTE_MODE_NBIOT;
                    break;
                case LTE_LC_SYSTEM_MODE_PREFER_LTEM_PLMN_PRIO:
                    pref_ret = LTE_MODE_LTEM | LTE_PLMN_PREF;
                    break;
                case LTE_LC_SYSTEM_MODE_PREFER_NBIOT_PLMN_PRIO:
                    pref_ret = LTE_MODE_NBIOT | LTE_PLMN_PREF;
                    break;
            }
            mp_obj_t tuple[2] = {
                mp_obj_new_int_from_uint(mode_ret),
                mp_obj_new_int_from_uint(pref_ret),
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
            mp_raise_OSError(ret);
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
            mp_raise_ValueError(MP_ERROR_TEXT("Unknown param"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_cell_status_obj, 1, 2, network_cell_status);

STATIC mp_obj_t network_cell_at(mp_obj_t self_in, mp_obj_t at_string) {
    const char *at = mp_obj_str_get_str(at_string);
    char buf[50];

    int ret = nrf_modem_at_cmd(buf, sizeof(buf), at);
    if (ret) {
        mp_raise_OSError(ret);
    }
    return mp_obj_new_str(buf, strlen(buf));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(network_cell_at_obj, network_cell_at);

STATIC mp_obj_t network_cell_irq(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    if (n_args > 1) {
        mp_raise_ValueError(MP_ERROR_TEXT("Arguments must be keyword based"));
    }
    if (kwargs->used != 0) {
        for (size_t i = 0; i < kwargs->alloc; i++) {
            if (mp_map_slot_is_filled(kwargs, i)) {
                switch(mp_obj_str_get_qstr(kwargs->table[i].key)) {
                    case MP_QSTR_handler: {
                        mp_obj_t handler_in = kwargs->table[i].value;
                        if (handler_in != mp_const_none && !mp_obj_is_callable(handler_in)) {
                            mp_raise_ValueError(MP_ERROR_TEXT("invalid handler"));
                        }
                        irq_handler = handler_in;
                        break;
                    }
                    case MP_QSTR_mask: {
                        irq_mask = (uint32_t)mp_obj_get_int(kwargs->table[i].value);
                        break;
                    }
                    case MP_QSTR_add_mask: {
                        irq_mask |= (uint32_t)mp_obj_get_int(kwargs->table[i].value);
                        break;
                    }
                    case MP_QSTR_remove_mask: {
                        irq_mask &= ~(uint32_t)mp_obj_get_int(kwargs->table[i].value);
                    }
                }
            }
        }
    } else {
        mp_obj_t tuple[2] = { 
            mp_obj_new_bool(irq_handler != mp_const_none),
            mp_obj_new_int(irq_mask),
         };
         return mp_obj_new_tuple(2, tuple);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_cell_irq_obj, 1, network_cell_irq);

#ifdef CONFIG_LOCATION
static int find_method_index(struct location_config *config, enum location_method method) {
    for (int i = 0; i < CONFIG_LOCATION_METHODS_LIST_SIZE; i++) {
        if (config->methods[i].method == method) {
            return i;
        }
    }
    return -ENODEV;
}
STATIC mp_obj_t network_cell_location(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    if (n_args > 1) {
        mp_raise_ValueError(MP_ERROR_TEXT("Arguments must be keyword based"));
    }
    if (kwargs->used != 0) {
        size_t method_num = 0;
        struct location_config config;
        enum location_method methods[CONFIG_LOCATION_METHODS_LIST_SIZE];
        /* First figure out what methods we're using. This is necessary
         * to be able to set the defaults on all the config */
        for (size_t i = 0; i < kwargs->alloc; i++) {
            if (mp_map_slot_is_filled(kwargs, i)) {
                switch(mp_obj_str_get_qstr(kwargs->table[i].key)) {
                    case MP_QSTR_gnss: {
                        methods[method_num++] = LOCATION_METHOD_GNSS;
                        break;
                    }
                    case MP_QSTR_cell: {
                        methods[method_num++] = LOCATION_METHOD_CELLULAR;
                        break;
                    }
                    case MP_QSTR_wifi: {
                        methods[method_num++] = LOCATION_METHOD_WIFI;
                        break;
                    }
                }
            }
        }
        if (method_num == 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("Must select at least 1 method"));
        }
        location_config_defaults_set(&config, method_num, methods);
        
        /* Now go get the user configuration to change any defaults */
        int32_t total_timeout = 0;
        for (size_t i = 0; i < kwargs->alloc; i++) {
            if (mp_map_slot_is_filled(kwargs, i)) {
                switch(mp_obj_str_get_qstr(kwargs->table[i].key)) {
                    case MP_QSTR_gnss: {
                        size_t index = find_method_index(&config, LOCATION_METHOD_GNSS);
                        if (index >= 0) {
                            if (!mp_obj_is_type(kwargs->table[i].value, &mp_type_tuple)) {
                                config.methods[index].gnss.timeout = (mp_obj_get_int(kwargs->table[i].value) * MSEC_PER_SEC);
                                total_timeout += config.methods[index].gnss.timeout;
                            } else {
                                size_t len;
                                mp_obj_t *items;
                                mp_obj_tuple_get(kwargs->table[i].value, &len, &items);
                                switch(len) {
                                    case 3:
                                        config.methods[index].gnss.visibility_detection = mp_obj_is_true(items[2]);
                                    case 2:
                                        config.methods[index].gnss.accuracy = mp_obj_get_int(items[1]);
                                    case 1:
                                        config.methods[index].gnss.timeout = (mp_obj_get_int(items[0]) * MSEC_PER_SEC);
                                        total_timeout += config.methods[index].gnss.timeout;
                                        break;
                                }
                            }
                        }
                        break;
                    }
                    case MP_QSTR_cell: {
                        size_t index = find_method_index(&config, LOCATION_METHOD_CELLULAR);
                        if (index >= 0) {
                            if (!mp_obj_is_type(kwargs->table[i].value, &mp_type_tuple)) {
                                config.methods[index].cellular.timeout = (mp_obj_get_int(kwargs->table[i].value) * MSEC_PER_SEC);
                                total_timeout += config.methods[index].cellular.timeout;
                            } else {
                                size_t len;
                                mp_obj_t *items;
                                mp_obj_tuple_get(kwargs->table[i].value, &len, &items);
                                switch(len) {
                                    case 2:
                                        config.methods[index].cellular.cell_count = mp_obj_get_int(items[1]);
                                    case 1:
                                        config.methods[index].cellular.timeout = (mp_obj_get_int(items[0]) * MSEC_PER_SEC);
                                        total_timeout += config.methods[index].cellular.timeout;
                                        break;
                                }
                            }
                        }
                        break;
                    }
                    case MP_QSTR_wifi: {
                        size_t index = find_method_index(&config, LOCATION_METHOD_WIFI);
                        if (index >= 0) {
                            config.methods[index].wifi.timeout = (mp_obj_get_int(kwargs->table[i].value) * MSEC_PER_SEC);
                            total_timeout += config.methods[index].wifi.timeout;
                        }
                        break;
                    }
                    case MP_QSTR_interval: {
                        config.interval = (mp_obj_get_int(kwargs->table[i].value));
                        break;
                    }
                    case MP_QSTR_all: {
                        config.mode = (mp_obj_is_true(kwargs->table[i].value) ? LOCATION_REQ_MODE_ALL : LOCATION_REQ_MODE_FALLBACK);
                        break;
                    }
                }
            }
        }
        config.timeout = total_timeout + 60000;     // Add an extra minute to the total timeout for network related delays
        return mp_obj_new_int(location_request(&config));
    }
   
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_cell_location_obj, 1, network_cell_location);

STATIC mp_obj_t network_cell_location_cancel(mp_obj_t self_in) {
    return mp_obj_new_bool(location_request_cancel() == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_cell_location_cancel_obj, network_cell_location_cancel);
#endif

STATIC const mp_rom_map_elem_t cell_if_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_cell_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&network_cell_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_cell_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_cell_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_cell_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_at), MP_ROM_PTR(&network_cell_at_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq), MP_ROM_PTR(&network_cell_irq_obj) },
#ifdef CONFIG_LOCATION
    { MP_ROM_QSTR(MP_QSTR_location), MP_ROM_PTR(&network_cell_location_obj) },
    { MP_ROM_QSTR(MP_QSTR_location_cancel), MP_ROM_PTR(&network_cell_location_cancel_obj) },
#endif
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