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

#if MICROPY_PY_NETWORK_NRF91
#include <stdio.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <modem/lte_lc.h>
#include <modem/modem_jwt.h>
#include <modem/modem_info.h>
#include <math.h>

#include <zephyr/logging/log.h>

#ifdef CONFIG_LOCATION
#include <modem/location.h>
#include <nrf_modem_gnss.h>
#include <nrf_cloud_agnss_schema_v1.h>
#endif

LOG_MODULE_REGISTER(mpy_network, LOG_LEVEL_DBG);

STATIC const mp_obj_base_t cell_if;

K_SEM_DEFINE(at_sem, 0, 1);
static const char *at_resp = NULL;

static mp_obj_t irq_handler = mp_const_none;
static uint32_t irq_mask = 0x0;

#define CHECK_RET(f) if ( (ret = (f)) < 0 ){ return mp_obj_new_int(ret);}

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
        {
            if (irq_mask & MP_CELL_IRQ_EDRX_UPDATE) {
                tuple[0] = mp_obj_new_int(MP_CELL_IRQ_EDRX_UPDATE);
                mp_obj_t params[2] = {
                    mp_obj_new_float(evt->edrx_cfg.edrx),
                    mp_obj_new_float(evt->edrx_cfg.ptw),
                };
                tuple[1] = mp_obj_new_tuple(2, params);
            }
            break;
        }
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
static void get_requested_data_types(mp_obj_t list, struct nrf_modem_gnss_agnss_data_frame request)
{
    for(int i = 0; i < request.system_count; ++i) {
        if (request.system[i].system_id == NRF_MODEM_GNSS_SYSTEM_GPS) {
            if (request.system[i].sv_mask_ephe) {
                mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_GPS_EPHEMERIDES));
            }
            if (request.system[i].sv_mask_alm) {
                mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_GPS_ALMANAC));
            }
            if (request.data_flags & NRF_MODEM_GNSS_AGNSS_INTEGRITY_REQUEST) {
                mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_GPS_INTEGRITY));
            }
        } else if (request.system[i].system_id == NRF_MODEM_GNSS_SYSTEM_QZSS) {
            if (request.system[i].sv_mask_ephe) {
                mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_QZSS_EPHEMERIDES));
            }
            if (request.system[i].sv_mask_alm) {
                mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_QZSS_ALMANAC));
            }
            if (request.data_flags & NRF_MODEM_GNSS_AGNSS_INTEGRITY_REQUEST) {
                mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_QZSS_INTEGRITY));
            }
        }
    }
    if (request.data_flags & NRF_MODEM_GNSS_AGNSS_KLOBUCHAR_REQUEST) {
        mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_KLOBUCHAR_CORRECTION));
    }
    if (request.data_flags & NRF_MODEM_GNSS_AGNSS_NEQUICK_REQUEST) {
        mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_NEQUICK_CORRECTION));
    }
    if (request.data_flags & NRF_MODEM_GNSS_AGNSS_GPS_SYS_TIME_AND_SV_TOW_REQUEST) {
        mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_GPS_TOWS));
        mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_GPS_SYSTEM_CLOCK));
    }
    if (request.data_flags & NRF_MODEM_GNSS_AGNSS_POSITION_REQUEST) {
        mp_obj_list_append(list, mp_obj_new_int(NRF_CLOUD_AGNSS_LOCATION));
    }
}

static void location_event_handler(const struct location_event_data *event_data)
{
	switch (event_data->id) {
	case LOCATION_EVT_LOCATION: {
        if (irq_mask & MP_CELL_IRQ_LOCATION_FOUND) {
            mp_obj_t tuple[2];
            tuple[0] = mp_obj_new_int(MP_CELL_IRQ_LOCATION_FOUND);
            mp_obj_t params[14];
            uint8_t idx = 0;
            const char *method_str = location_method_str(event_data->method);
            params[idx++] = mp_obj_new_str(method_str, strlen(method_str));
            params[idx++] = mp_obj_new_float(event_data->location.latitude);
            params[idx++] = mp_obj_new_float(event_data->location.longitude);
            params[idx++] = mp_obj_new_float(event_data->location.accuracy);
            if (event_data->location.datetime.valid) {
                params[idx++] = mp_obj_new_int(event_data->location.datetime.year);
                params[idx++] = mp_obj_new_int(event_data->location.datetime.month);
                params[idx++] = mp_obj_new_int(event_data->location.datetime.day);
                params[idx++] = mp_obj_new_int(event_data->location.datetime.hour);
                params[idx++] = mp_obj_new_int(event_data->location.datetime.minute);
                params[idx++] = mp_obj_new_int(event_data->location.datetime.second);
                params[idx++] = mp_obj_new_int(event_data->location.datetime.ms);   
            }
#if defined(CONFIG_LOCATION_DATA_DETAILS)
            if (event_data->method == LOCATION_METHOD_GNSS) {
                params[idx++] = mp_obj_new_float(event_data->location.details.gnss.pvt_data.altitude);
                params[idx++] = mp_obj_new_float(event_data->location.details.gnss.pvt_data.heading);
                params[idx++] = mp_obj_new_float(event_data->location.details.gnss.pvt_data.speed);
            }
#endif
            tuple[1] = mp_obj_new_tuple(idx, params);
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
    case LOCATION_EVT_GNSS_ASSISTANCE_REQUEST: {
        if (irq_mask & MP_CELL_IRQ_GNSS_ASSISTANCE_REQUEST) {
            mp_obj_t tuple[2];
            tuple[0] = mp_obj_new_int(MP_CELL_IRQ_GNSS_ASSISTANCE_REQUEST);
            tuple[1] = mp_obj_new_list(0, NULL);
            get_requested_data_types(tuple[1], event_data->agnss_request);
            mp_sched_schedule(MP_OBJ_FROM_PTR(&cell_invoke_irq_obj), mp_obj_new_tuple(2, tuple));
        }
        break;
    }
    case LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST: {
        if (irq_mask & MP_CELL_IRQ_CLOUD_LOCATION_REQUEST) {
            mp_obj_t tuple[2];
            tuple[0] = mp_obj_new_int(MP_CELL_IRQ_CLOUD_LOCATION_REQUEST);
            mp_obj_t data[3];
            LOG_DBG("Current cell: %u", event_data->cloud_location_request.cell_data->current_cell.id);
            mp_obj_t ccell[7] = {
                mp_obj_new_int(event_data->cloud_location_request.cell_data->current_cell.mcc),
                mp_obj_new_int(event_data->cloud_location_request.cell_data->current_cell.mnc),
                mp_obj_new_int_from_uint(event_data->cloud_location_request.cell_data->current_cell.tac),
                mp_obj_new_int_from_uint(event_data->cloud_location_request.cell_data->current_cell.id),
                mp_obj_new_int(RSRP_IDX_TO_DBM(event_data->cloud_location_request.cell_data->current_cell.rsrp)),
                mp_obj_new_int(RSRQ_IDX_TO_DB(event_data->cloud_location_request.cell_data->current_cell.rsrq)),
                mp_obj_new_int_from_uint(event_data->cloud_location_request.cell_data->current_cell.earfcn)
            };
            data[0] = mp_obj_new_tuple(7, ccell);
            data[1] = mp_obj_new_list(0, NULL);
            LOG_DBG("Neighbor cells: %u", event_data->cloud_location_request.cell_data->ncells_count);
            for (int i = 0; i < event_data->cloud_location_request.cell_data->ncells_count; ++i) {
                mp_obj_t ncell[4] = {
                    mp_obj_new_int_from_uint(event_data->cloud_location_request.cell_data->neighbor_cells[i].phys_cell_id),
                    mp_obj_new_int_from_uint(event_data->cloud_location_request.cell_data->neighbor_cells[i].earfcn),
                    mp_obj_new_int(RSRP_IDX_TO_DBM(event_data->cloud_location_request.cell_data->neighbor_cells[i].rsrp)),
                    mp_obj_new_int(RSRQ_IDX_TO_DB(event_data->cloud_location_request.cell_data->neighbor_cells[i].rsrq))
                };
                mp_obj_list_append(data[1], mp_obj_new_tuple(4, ncell));
            }
            data[2] = mp_const_none; // TODO: Add Wi-Fi support
            tuple[1] = mp_obj_new_tuple(3, data);
            mp_sched_schedule(MP_OBJ_FROM_PTR(&cell_invoke_irq_obj), mp_obj_new_tuple(2, tuple));
        }
        break;
    }
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
    // The LC library doesn't re-register the same handler, so we can safely call it 
    // multiple times. Init can also be called again and will return 0
    lte_lc_register_handler(lte_handler);
    return MP_OBJ_FROM_PTR(&cell_if);
}

STATIC mp_obj_t network_cell_connect(mp_obj_t self_in) {
    int ret;
    CHECK_RET(lte_lc_connect_async(NULL));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_cell_connect_obj, network_cell_connect);

STATIC mp_obj_t network_cell_isconnected(mp_obj_t self_in) {
    enum lte_lc_nw_reg_status reg_status;
    int ret;

    CHECK_RET(lte_lc_nw_reg_status_get(&reg_status));
    if (reg_status == LTE_LC_NW_REG_REGISTERED_HOME ||
        reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING) {
            return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_cell_isconnected_obj, network_cell_isconnected);

#define EDRXCODE_TO_BINARY_FORMAT "%c%c%c%c"
#define EDRXCODE_TO_BINARY(byte) \
    ((byte) & 0x08 ? '1' : '0'), \
    ((byte) & 0x04 ? '1' : '0'), \
    ((byte) & 0x02 ? '1' : '0'), \
    ((byte) & 0x01 ? '1' : '0')
/*
 * eDRX values are a string of 4 bits. The array below will represent
 * them as uint8_t to save space, and the conversion will be done later.
 * 
 * Valid values for cycle length (in seconds):
 * 
 * 20.48    40.96   81.92   163.84  327.68  655.36  1310.72 2621.44
 * 0010     0011    0101    1001    1010    1011    1100    1101
 * 
 */
static uint8_t edrx_cycle_encoded[8] = {2, 3, 5, 9, 10, 11, 12, 13};
static uint8_t get_cycle_value(float seconds) {
    if (seconds <= 20.48) {
        return edrx_cycle_encoded[0];
    }
    uint8_t cycle_index = MIN(sqrt(seconds / 20.48), sizeof(edrx_cycle_encoded) - 1);
    return edrx_cycle_encoded[cycle_index];
}

/*
 *
 * Valid values for PTW (in seconds):
 * 
 * LTE-M    1.28    2.56    3.84    5.12    6.4     7.68    8.96    10.24   11.52   12.8    14.08   15.36   16.64   17.92   19.20   20.48
 * NB-IoT   2.56    5.12    7.68    10.24   12.8    15.36   17.92   20.48   23.04   25.6    28.16   30.72   33.28   35.84   38.4    40.96
 * Code     0000    0001    0010    0011    0100    0101    0110    0111    1000    1001    1010    1011    1100    1101    1110    1111
 */

static uint8_t get_lte_ptw(float seconds, bool nbiot) {
    float divisor = nbiot ? 2.56 : 1.28;
    if (seconds <= divisor) {
        return 0;
    }
    uint8_t lte_ptw_index = MIN((seconds - 0.01) / divisor, 15);
    return lte_ptw_index;
}

STATIC mp_obj_t network_cell_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    if (n_args != 1 && kwargs->used != 0) {
        mp_raise_TypeError(MP_ERROR_TEXT("either pos or kw args are allowed"));
    }
    int ret;
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
                        CHECK_RET(lte_lc_system_mode_set(mode, preference));
                        break;
                    }
                    case MP_QSTR_edrx_params:
                    case MP_QSTR_edrx: {
                        if (!mp_obj_is_type(kwargs->table[i].value, &mp_type_tuple)) {
                            mp_raise_ValueError("Parameters must be a tuple");
                        }
                        size_t len;
                        mp_obj_t *items;
                        mp_obj_tuple_get(kwargs->table[i].value, &len, &items);
                        if (len != 2) {
                            mp_raise_ValueError("Parameter tuple must have 2 values");
                        }
                        float edrx_cycle = mp_obj_is_float(items[0]) ? mp_obj_float_get(items[0]) : (float)mp_obj_get_int(items[0]);
                        float edrx_ptw = mp_obj_is_float(items[1]) ? mp_obj_float_get(items[1]) : (float)mp_obj_get_int(items[1]);
                        char value[5];
                        snprintf(value, sizeof(value), EDRXCODE_TO_BINARY_FORMAT,
                                EDRXCODE_TO_BINARY(edrx_cycle_encoded[get_cycle_value(edrx_cycle)]));
                        CHECK_RET(lte_lc_edrx_param_set(LTE_LC_LTE_MODE_LTEM, value));
                        CHECK_RET(lte_lc_edrx_param_set(LTE_LC_LTE_MODE_NBIOT, value));

                        snprintf(value, sizeof(value), EDRXCODE_TO_BINARY_FORMAT,
                            EDRXCODE_TO_BINARY(get_lte_ptw(edrx_ptw, false)));
                        CHECK_RET(lte_lc_ptw_set(LTE_LC_LTE_MODE_LTEM, value));
                        
                        snprintf(value, sizeof(value), EDRXCODE_TO_BINARY_FORMAT,
                            EDRXCODE_TO_BINARY(get_lte_ptw(edrx_ptw, true)));
                        CHECK_RET(lte_lc_ptw_set(LTE_LC_LTE_MODE_NBIOT, value));
                        break;
                    }
                    case MP_QSTR_edrx_enable: {
                        CHECK_RET(lte_lc_edrx_req(mp_obj_is_true(kwargs->table[i].value)));
                        break;
                    }
                    case MP_QSTR_psm:
                    case MP_QSTR_psm_params: {
                        if (!mp_obj_is_type(kwargs->table[i].value, &mp_type_tuple)) {
                            mp_raise_ValueError("Parameters must be a tuple");
                        }
                        size_t len;
                        mp_obj_t *items;
                        mp_obj_tuple_get(kwargs->table[i].value, &len, &items);
                        if (len != 2) {
                            mp_raise_ValueError("Parameter tuple must have 2 values");
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
                        CHECK_RET(lte_lc_psm_req(mp_obj_is_true(kwargs->table[i].value)));
                        break;
                    }
                    default:
                        mp_raise_ValueError("Parameter is not recognized");
                        break;
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
            CHECK_RET(lte_lc_system_mode_get(&mode, &preference));

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

STATIC mp_obj_t string_status_value(enum modem_info info) {
    char data[MODEM_INFO_MAX_RESPONSE_SIZE];
    int ret;
    CHECK_RET(modem_info_init());
    CHECK_RET(modem_info_string_get(info, data, sizeof(data)));
    return mp_obj_new_str(data, strlen(data));
}

STATIC mp_obj_t int_status_value(enum modem_info info) {
    uint16_t data;
    int ret;
    CHECK_RET(modem_info_init());
    CHECK_RET(modem_info_short_get(info, &data));
    return mp_obj_new_int_from_uint(data);
}

STATIC mp_obj_t network_cell_status(size_t n_args, const mp_obj_t *args) {
    int ret;
    if (n_args == 1) {
        enum lte_lc_nw_reg_status reg_status;
        CHECK_RET(lte_lc_nw_reg_status_get(&reg_status));
        return mp_obj_new_int_from_uint(reg_status);
    }
    // There's an argument, get that status
    switch (mp_obj_str_get_qstr(args[1])) {
        case MP_QSTR_mode: {
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
        case MP_QSTR_uuid: {
            struct nrf_device_uuid dev = {0};
            CHECK_RET(modem_jwt_get_uuids(&dev, NULL));
            return mp_obj_new_str(dev.str, strlen(dev.str));
        }
        case MP_QSTR_rsrp: {
            uint16_t rsrp;
            CHECK_RET(modem_info_init());
            CHECK_RET(modem_info_short_get(MODEM_INFO_RSRP, &rsrp))
            return mp_obj_new_int(RSRP_IDX_TO_DBM(rsrp));
        }
        case MP_QSTR_band: 
            return int_status_value(MODEM_INFO_CUR_BAND);
        case MP_QSTR_uiccMode: 
            return int_status_value(MODEM_INFO_UICC);
        case MP_QSTR_mccmnc:
            return string_status_value(MODEM_INFO_OPERATOR);
        case MP_QSTR_cellid:
            return string_status_value(MODEM_INFO_CELLID);
        case MP_QSTR_area:
            return string_status_value(MODEM_INFO_AREA_CODE);
        case MP_QSTR_imei:
            return string_status_value(MODEM_INFO_IMEI);
        case MP_QSTR_ipAddress:
            return string_status_value(MODEM_INFO_IP_ADDRESS);
        case MP_QSTR_apn:
            return string_status_value(MODEM_INFO_APN);
        case MP_QSTR_iccid:
            return string_status_value(MODEM_INFO_ICCID);
        case MP_QSTR_imsi:
            return string_status_value(MODEM_INFO_IMSI);
        default:
            mp_raise_ValueError(MP_ERROR_TEXT("Unknown param"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_cell_status_obj, 1, 2, network_cell_status);

STATIC mp_obj_t network_cell_at(mp_obj_t self_in, mp_obj_t at_string) {
    const char *at = mp_obj_str_get_str(at_string);
    char buf[500];

    int ret;
    CHECK_RET(nrf_modem_at_cmd(buf, sizeof(buf), at));
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

void resp_callback(const char *at_response) {
    at_resp = at_response;
    k_sem_give(&at_sem);
}

STATIC mp_obj_t network_cell_cert(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    //wlan_if_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int ret;
    switch (mp_obj_str_get_qstr(args[1])) {
        case MP_QSTR_list: {
            switch (n_args) {
                case 2:
                    ret = nrf_modem_at_cmd_async(resp_callback, "AT%%CMNG=1");
                    break;
                case 3:
                    ret = nrf_modem_at_cmd_async(resp_callback, "AT%%CMNG=1,%u", mp_obj_get_int(args[2]));
                    break;
                case 4:
                    ret = nrf_modem_at_cmd_async(resp_callback, "AT%%CMNG=1,%u,%u", mp_obj_get_int(args[2]), mp_obj_get_int(args[3]));
                    break;
                default:
                    ret = -1;
            }
            if (ret) {
                return mp_obj_new_int(ret);
            }
            k_sem_take(&at_sem, K_FOREVER);
            mp_obj_t list = mp_obj_new_list(0, NULL);
            while (at_resp != NULL) {
                if (strncmp(at_resp, "\r\n", 2) == 0) {
                    at_resp += 2;
                }
                if (strncmp(at_resp, "%CMNG: ", strlen("%CMNG: ")) == 0) {
                    uint32_t sec_tag, type;
                    sscanf(at_resp, "%%CMNG: %u,%u", &sec_tag, &type);
                    mp_obj_t tuple[2] = { mp_obj_new_int_from_uint(sec_tag), mp_obj_new_int(type) };
                    mp_obj_list_append(list, mp_obj_new_tuple(2, tuple));
                } else {
                    // TODO: Handle OK\r\n
                    break;
                }
                at_resp = strstr(at_resp, "\r\n");
            }
            return list;
        }
        case MP_QSTR_write: {
            if (n_args != 5) {
                mp_raise_ValueError(MP_ERROR_TEXT("Not enough arguments"));
            }
            uint32_t sec_tag = mp_obj_get_int(args[2]);
            if (sec_tag > 2147483647) {
                mp_raise_ValueError(MP_ERROR_TEXT("Invalid security tag"));
            }
            uint32_t type = mp_obj_get_int(args[3]);
            if (type > 6) {
                mp_raise_ValueError(MP_ERROR_TEXT("Invalid type"));
            }
            CHECK_RET(nrf_modem_at_printf("AT%%CMNG=0,%u,%u,\"%s\"", sec_tag, type, mp_obj_str_get_str(args[4])));
            return mp_const_none;
        }
        case MP_QSTR_delete: {
            if (n_args != 4) {
                mp_raise_ValueError(MP_ERROR_TEXT("Not enough arguments"));
            }
            uint32_t sec_tag = mp_obj_get_int(args[2]);
            if (sec_tag > 2147483647) {
                mp_raise_ValueError(MP_ERROR_TEXT("Invalid security tag"));
            }
            uint32_t type = mp_obj_get_int(args[3]);
            if (type > 6) {
                mp_raise_ValueError(MP_ERROR_TEXT("Invalid type"));
            }
            CHECK_RET(nrf_modem_at_printf("AT%%CMNG=3,%u,%u", sec_tag, type));
            return mp_const_none;
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_cell_cert_obj, 2, network_cell_cert);

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

STATIC mp_obj_t network_cell_agnss_data_process(mp_obj_t self_in, mp_obj_t data) {
    mp_buffer_info_t bufinfo;
    if (!mp_get_buffer(data, &bufinfo, MP_BUFFER_READ)) {
        return mp_const_none;
    }
    return mp_obj_new_int(location_agnss_data_process(bufinfo.buf, bufinfo.len));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(network_cell_agnss_data_process_obj, network_cell_agnss_data_process);

STATIC mp_obj_t network_cell_location_cloud_fix(size_t n_args, const mp_obj_t *args) {
    struct location_data loc;
    loc.latitude = mp_obj_get_float(args[1]);
    loc.longitude = mp_obj_get_float(args[2]);
    loc.accuracy = mp_obj_get_float(args[3]);
    loc.datetime.valid = false;
    location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_SUCCESS, &loc);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_cell_location_cloud_fix_obj, 4, 4, network_cell_location_cloud_fix);
#endif // CONFIG_LOCATION

#ifdef CONFIG_PDN
STATIC mp_obj_t network_cell_pdn_id(mp_obj_t self_in, mp_obj_t cid) {
    if (!mp_obj_is_int(cid)) {
        mp_raise_TypeError(MP_ERROR_TEXT("CID must be an integer"));
    }
    return mp_obj_new_int(pdn_id_get(mp_obj_get_int(cid)));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(network_cell_pdn_id_obj, network_cell_pdn_id);

STATIC mp_obj_t network_cell_pdn_create(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kwargs) {
    enum { ARG_apn, ARG_family, ARG_ip4_addr_alloc, ARG_nslpi, ARG_secure_pco, ARG_auth, ARG_user, ARG_password };
    
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_apn, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_family, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_ip4_addr_alloc, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_nslpi, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_secure_pco, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_auth, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_user, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_password, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kwargs, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    uint8_t cid = -1;
    int ret;

    ret = pdn_ctx_create(&cid, NULL);
    if (ret) {
        goto fail;
    }
    
    if (args[ARG_ip4_addr_alloc].u_int != 0 || args[ARG_nslpi].u_int != 0 || args[ARG_secure_pco].u_int != 0) {
        struct pdn_pdp_opt opts = {
            .ip4_addr_alloc = args[ARG_ip4_addr_alloc].u_int,
            .nslpi = args[ARG_nslpi].u_int,
            .secure_pco = args[ARG_secure_pco].u_int
        };
        ret = pdn_ctx_configure(cid, mp_obj_str_get_str(args[ARG_apn].u_obj), args[ARG_family].u_int, &opts);
    } else {
        ret = pdn_ctx_configure(cid, mp_obj_str_get_str(args[ARG_apn].u_obj), args[ARG_family].u_int, NULL);
    }
    if (ret) {
        goto fail;
    }
    if (args[ARG_user].u_obj != mp_const_none && args[ARG_password].u_obj != mp_const_none) {
        ret = pdn_ctx_auth_set(cid, args[ARG_auth].u_int, mp_obj_str_get_str(args[ARG_user].u_obj), mp_obj_str_get_str(args[ARG_password].u_obj));
        if (ret) {
            goto fail;
        }
    }
    return mp_obj_new_int(cid);
    
fail:
    if (cid != -1) {
        pdn_ctx_destroy(cid);
    }
    return mp_obj_new_int(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_cell_pdn_create_obj, 2, network_cell_pdn_create);

STATIC mp_obj_t network_cell_pdn_activate(mp_obj_t self_in, mp_obj_t cid, mp_obj_t activate) {
    if (!mp_obj_is_int(cid)) {
        mp_raise_TypeError(MP_ERROR_TEXT("CID must be an integer"));
    }
    if (!mp_obj_is_bool(activate)) {
        mp_raise_TypeError(MP_ERROR_TEXT("Argument must be True or False"));
    }
    int ret;
    if (mp_obj_is_true(activate)) {
        CHECK_RET(pdn_activate(mp_obj_get_int(cid), NULL, NULL));
    } else {
        CHECK_RET(pdn_deactivate(mp_obj_get_int(cid)));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(network_cell_pdn_activate_obj, network_cell_pdn_activate);

STATIC mp_obj_t network_cell_pdn_destroy(mp_obj_t self_in, mp_obj_t cid) {
    if (!mp_obj_is_int(cid)) {
        mp_raise_TypeError(MP_ERROR_TEXT("CID must be an integer"));
    }
    int ret;
    CHECK_RET(pdn_ctx_destroy(mp_obj_get_int(cid)));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(network_cell_pdn_destroy_obj, network_cell_pdn_destroy);

STATIC mp_obj_t network_cell_pdn_default_apn(mp_obj_t self_in) {
    char buf[32];
    int ret;
    CHECK_RET(pdn_default_apn_get(buf, sizeof(buf)));
    return mp_obj_new_str(buf, strlen(buf));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_cell_pdn_default_apn_obj, network_cell_pdn_default_apn);
#endif

STATIC const mp_rom_map_elem_t cell_if_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&network_cell_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_cell_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_cell_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_cell_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_at), MP_ROM_PTR(&network_cell_at_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq), MP_ROM_PTR(&network_cell_irq_obj) },
// Certificate management
    { MP_ROM_QSTR(MP_QSTR_cert), MP_ROM_PTR(&network_cell_cert_obj) },
    #ifdef CONFIG_LOCATION
    { MP_ROM_QSTR(MP_QSTR_location), MP_ROM_PTR(&network_cell_location_obj) },
    { MP_ROM_QSTR(MP_QSTR_location_cancel), MP_ROM_PTR(&network_cell_location_cancel_obj) },
    { MP_ROM_QSTR(MP_QSTR_agnss_data), MP_ROM_PTR(&network_cell_agnss_data_process_obj) },
    { MP_ROM_QSTR(MP_QSTR_location_cloud_fix), MP_ROM_PTR(&network_cell_location_cloud_fix_obj) },
    #endif
    #ifdef CONFIG_PDN
    { MP_ROM_QSTR(MP_QSTR_pdn_id), MP_ROM_PTR(&network_cell_pdn_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_pdn_create), MP_ROM_PTR(&network_cell_pdn_create_obj) },
    { MP_ROM_QSTR(MP_QSTR_pdn_activate), MP_ROM_PTR(&network_cell_pdn_activate_obj) },
    { MP_ROM_QSTR(MP_QSTR_pdn_destroy), MP_ROM_PTR(&network_cell_pdn_destroy_obj) },
    { MP_ROM_QSTR(MP_QSTR_pdn_default_apn), MP_ROM_PTR(&network_cell_pdn_default_apn_obj) },
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