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

#include "py/objlist.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/modnetwork.h"
#include "modnetwork.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mpy_network, LOG_LEVEL_DBG);

#if MICROPY_PY_NETWORK_WLAN

#include <nrfx_clock.h>

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_config.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/net_ip.h>

#ifdef CONFIG_WIFI_CREDENTIALS
#include <net/wifi_credentials.h>
#endif

STATIC const wlan_if_obj_t wlan_sta_obj;
STATIC const wlan_if_obj_t wlan_ap_obj;


// Set to "true" if the STA interface is connected to wifi and has IP address.
static bool wifi_sta_connected = false;

#if MICROPY_HW_ENABLE_MDNS_QUERIES || MICROPY_HW_ENABLE_MDNS_RESPONDER
// Whether mDNS has been initialised or not
static bool mdns_initialised = false;
#endif

static struct net_mgmt_event_callback wifi_mgmt_cb;

K_SEM_DEFINE(scan_sem, 0, 1);
K_SEM_DEFINE(twt_sem, 0, 1);
mp_obj_t *scan_list;
mp_obj_t *twt_resp;

STATIC mp_obj_t zephyr_initialize() {
    static int initialized = 0;
    if (!initialized) {
        initialized = 1;
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(zephyr_network_initialize_obj, zephyr_initialize);

STATIC mp_obj_t wlan_invoke_irq(mp_obj_t arg) {
    // Send a tuple with the following:
    // ( self, IRQ event, data tuple)
    // Can skip the data tuple if there's no data
    mp_obj_t *items;
    size_t len;
    mp_obj_get_array(arg, &len, &items);

    wlan_if_obj_t *self = MP_OBJ_TO_PTR(items[0]);
    if (self->settings->irq_handler == mp_const_none) {
        return mp_const_none;
    }
    mp_obj_t event = items[1];
    mp_obj_t data = (len == 3) ? items[2] : mp_const_none;

    mp_call_function_2(self->settings->irq_handler, event, data);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(wlan_invoke_irq_obj, wlan_invoke_irq);

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb) {
    const struct wifi_status *status =
        (const struct wifi_status *)cb->info;

    if (!status->status) {
        wifi_sta_connected = true;
    }

    mp_obj_t stat[1] = { MP_OBJ_NEW_SMALL_INT(status->status), };
    mp_obj_t tuple[3] = {
        MP_OBJ_FROM_PTR(&wlan_sta_obj),
        MP_OBJ_NEW_SMALL_INT(MP_WLAN_IRQ_STA_CONNECT),
        mp_obj_new_tuple(1, stat),
    };
    mp_sched_schedule(MP_OBJ_FROM_PTR(&wlan_invoke_irq_obj), mp_obj_new_tuple(3, tuple));
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb) {
    if (wifi_sta_connected) {
        wifi_sta_connected = false;
    }
    mp_obj_t tuple[2] = {
        MP_OBJ_FROM_PTR(&wlan_sta_obj),
        MP_OBJ_NEW_SMALL_INT(MP_WLAN_IRQ_STA_DISCONNECT),
    };
    mp_sched_schedule(MP_OBJ_FROM_PTR(&wlan_invoke_irq_obj), mp_obj_new_tuple(2, tuple));
}

static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb) {
    const struct wifi_scan_result *entry =
        (const struct wifi_scan_result *)cb->info;

    mp_obj_tuple_t *ap = mp_obj_new_tuple(6, NULL);
    ap->items[0] = mp_obj_new_bytes(entry->ssid, entry->ssid_length); // SSID
    ap->items[1] = mp_obj_new_bytes(entry->mac, entry->mac_length);   // BSSID
    ap->items[2] = MP_OBJ_NEW_SMALL_INT(entry->channel); // Channel
    ap->items[3] = MP_OBJ_NEW_SMALL_INT(entry->rssi); // RSSI
    ap->items[4] = MP_OBJ_NEW_SMALL_INT(entry->security); // Security
    ap->items[5] = mp_const_false; // hidden
    mp_obj_list_append(*scan_list, MP_OBJ_FROM_PTR(ap));
}

static void handle_wifi_twt_result(struct net_mgmt_event_callback *cb) {
    const struct wifi_twt_params *resp = (const struct wifi_twt_params *)cb->info;

    LOG_INF("TWT callback");

    if (resp->resp_status == WIFI_TWT_RESP_RECEIVED) {
        // Successful response
        if (resp->operation == WIFI_TWT_SETUP) {
            switch (resp->setup_cmd) {
                case WIFI_TWT_SETUP_CMD_ACCEPT: {
                    // TWT proposal accepted
                    twt_resp[0] = mp_obj_new_int(0);
                    twt_resp[1] = mp_obj_new_int(resp->setup.twt_wake_interval);
                    twt_resp[2] = mp_obj_new_int_from_ll(resp->setup.twt_interval);
                    break;
                }
                case WIFI_TWT_SETUP_CMD_REQUEST:
                case WIFI_TWT_SETUP_CMD_SUGGEST:
                case WIFI_TWT_SETUP_CMD_DEMAND:
                case WIFI_TWT_SETUP_CMD_GROUPING:
                case WIFI_TWT_SETUP_CMD_ALTERNATE:
                case WIFI_TWT_SETUP_CMD_DICTATE:
                case WIFI_TWT_SETUP_CMD_REJECT:
                    // TODO: handle other cases
                    twt_resp[0] = mp_obj_new_int(-1);
                    break;
            }
        }
    } else {
        // Timed out
        twt_resp[0] = mp_obj_new_int(-2);
    }
    k_sem_give(&twt_sem);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
    uint32_t mgmt_event, struct net_if *iface) {
    switch (mgmt_event) {
        case NET_EVENT_WIFI_CONNECT_RESULT:
            handle_wifi_connect_result(cb);
            break;
        case NET_EVENT_WIFI_DISCONNECT_RESULT:
            handle_wifi_disconnect_result(cb);
            break;
        case NET_EVENT_WIFI_SCAN_RESULT:
            handle_wifi_scan_result(cb);
            break;
        case NET_EVENT_WIFI_SCAN_DONE:
            k_sem_give(&scan_sem);
            break;
        case NET_EVENT_WIFI_IFACE_STATUS:
            LOG_INF("WiFi iface Status event");
            break;
        case NET_EVENT_WIFI_TWT:
            handle_wifi_twt_result(cb);
            break;
        case NET_EVENT_WIFI_TWT_SLEEP_STATE:
            // TODO: send callback to user
            break;
        default:
            break;
    }
}

void zephyr_initialise_wifi() {
    static int wifi_initialized = 0;
    if (!wifi_initialized) {
        net_mgmt_init_event_callback(&wifi_mgmt_cb,
            wifi_mgmt_event_handler,
            NET_EVENT_WIFI_CONNECT_RESULT |
            NET_EVENT_WIFI_DISCONNECT_RESULT |
            NET_EVENT_WIFI_SCAN_RESULT |
            NET_EVENT_WIFI_SCAN_DONE |
            NET_EVENT_WIFI_IFACE_STATUS |
            NET_EVENT_WIFI_TWT);

        net_mgmt_add_event_callback(&wifi_mgmt_cb);
        wifi_initialized = 1;
    }
}

STATIC mp_obj_t network_wlan_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    zephyr_initialise_wifi();

    int idx = (n_args > 0) ? mp_obj_get_int(args[0]) : MOD_NETWORK_STA_IF;
    if (idx == MOD_NETWORK_STA_IF) {
        wlan_sta_obj.settings->irq_handler = mp_const_none;
        return MP_OBJ_FROM_PTR(&wlan_sta_obj);
    } else if (idx == MOD_NETWORK_AP_IF) {
        wlan_ap_obj.settings->irq_handler = mp_const_none;
        return MP_OBJ_FROM_PTR(&wlan_ap_obj);
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid WLAN interface identifier"));
    }
}

STATIC mp_obj_t network_wlan_active(size_t n_args, const mp_obj_t *args) {
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (self->if_id == MOD_NETWORK_AP_IF) {
        if (n_args > 1) {
            bool activate = mp_obj_is_true(args[1]);
            struct net_if *iface = net_if_get_default();
            if (activate) {
                mp_raise_ValueError("Enable AP by configuring it");
            } else {
                if (net_mgmt(NET_REQUEST_WIFI_AP_DISABLE, iface, NULL, 0)) {
                    return mp_const_false;
                }
                return mp_const_false;
            }
        }
    } else {
        return mp_const_true;
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_wlan_active_obj, 1, 2, network_wlan_active);


STATIC mp_obj_t network_wlan_connect(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_ssid, ARG_key, ARG_auth, ARG_mfp, ARG_channel };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_key, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_auth, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = WIFI_SECURITY_TYPE_UNKNOWN}},
        { MP_QSTR_mfp, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = WIFI_MFP_OPTIONAL}},
        { MP_QSTR_channel, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = WIFI_CHANNEL_ANY}},
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    struct net_if *iface = net_if_get_default();

    if (args[ARG_auth].u_int > WIFI_SECURITY_TYPE_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("Auth value not valid"));
    }
    if (args[ARG_auth].u_int != WIFI_SECURITY_TYPE_NONE && args[ARG_key].u_obj == mp_const_none) {
        mp_raise_ValueError(MP_ERROR_TEXT("Key is required"));
    }
    if (args[ARG_channel].u_int != WIFI_CHANNEL_ANY &&
        args[ARG_channel].u_int > WIFI_CHANNEL_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid channel number"));
    }


    static struct wifi_connect_req_params cnx_params;

    size_t len;
    const char *p;
    p = mp_obj_str_get_data(args[ARG_ssid].u_obj, &len);
    cnx_params.ssid = p;
    cnx_params.ssid_length = MIN(len, WIFI_SSID_MAX_LEN);

    if (args[ARG_key].u_obj != mp_const_none) {
        p = mp_obj_str_get_data(args[ARG_key].u_obj, &len);
        cnx_params.psk = (uint8_t *)p;
        cnx_params.psk_length = MIN(len, WIFI_PSK_MAX_LEN);
    }
    cnx_params.security = args[ARG_auth].u_int;
    cnx_params.channel = args[ARG_channel].u_int;
    cnx_params.mfp = args[ARG_mfp].u_int;
    cnx_params.timeout = SYS_FOREVER_MS;

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
        &cnx_params, sizeof(struct wifi_connect_req_params))) {

        return mp_obj_new_bool(false);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_wlan_connect_obj, 1, network_wlan_connect);

STATIC mp_obj_t network_wlan_disconnect(mp_obj_t self_in) {
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->if_id == MOD_NETWORK_AP_IF) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("Disconnect only on Station interface"));
    }
    struct net_if *iface = net_if_get_default();
    if (net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0)) {
        LOG_ERR("Disconnect request failed");
        return mp_obj_new_bool(false);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_disconnect_obj, network_wlan_disconnect);

STATIC mp_obj_t network_wlan_status(size_t n_args, const mp_obj_t *args) {
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (self->if_id == MOD_NETWORK_AP_IF) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("Status only on Station interface"));
    }

    struct wifi_iface_status iface_status;
    struct net_if *iface = net_if_get_default();

    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &iface_status, sizeof(struct wifi_iface_status))) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error getting status"));
    }

    if (n_args == 1) {
        if (self->if_id != MOD_NETWORK_STA_IF) {
            return mp_const_none;
        }
        uint8_t retval = WIFI_STATE_DISCONNECTED;

        switch (iface_status.state) {
            case WIFI_STATE_DISCONNECTED:
            case WIFI_STATE_INTERFACE_DISABLED:
            case WIFI_STATE_INACTIVE:
                retval = WIFI_STATE_DISCONNECTED;
                break;
            case WIFI_STATE_SCANNING:
            case WIFI_STATE_ASSOCIATING:
            case WIFI_STATE_ASSOCIATED:
            case WIFI_STATE_4WAY_HANDSHAKE:
            case WIFI_STATE_GROUP_HANDSHAKE:
                retval = WIFI_STATE_AUTHENTICATING;
                break;
            case WIFI_STATE_COMPLETED:
                retval = WIFI_STATE_COMPLETED;
                break;
        }
        return mp_obj_new_int_from_uint(retval);
    }

    // There's one more argument
    if (iface_status.state != WIFI_STATE_COMPLETED) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Device not connected"));
    }
    switch ((uintptr_t)args[1]) {
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_rssi): {
            return MP_OBJ_NEW_SMALL_INT(iface_status.rssi);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_ssid): {
            return mp_obj_new_str(iface_status.ssid, iface_status.ssid_len);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_band): {
            return mp_obj_new_int_from_uint(iface_status.band);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_channel): {
            return mp_obj_new_int(iface_status.channel);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_link_mode): {
            return mp_obj_new_int_from_uint(iface_status.link_mode);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_security): {
            return mp_obj_new_int_from_uint(iface_status.security);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_mfp): {
            return mp_obj_new_int_from_uint(iface_status.mfp);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_bssid): {
            return mp_obj_new_bytes(iface_status.bssid, WIFI_MAC_ADDR_LEN);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_dtim): {
            return mp_obj_new_int_from_uint(iface_status.dtim_period);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_beacon_interval): {
            return mp_obj_new_int_from_uint(iface_status.beacon_interval);
        }
        case (uintptr_t)MP_OBJ_NEW_QSTR(MP_QSTR_twt_capable): {
            return mp_obj_new_bool(iface_status.twt_capable);
        }
        default:
            mp_raise_ValueError(MP_ERROR_TEXT("unknown status param"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_wlan_status_obj, 1, 2, network_wlan_status);

STATIC mp_obj_t network_wlan_scan(mp_obj_t self_in) {
    static bool wifi_is_scanning = false;
    if (wifi_is_scanning) {
        mp_raise_ValueError(MP_ERROR_TEXT("Already scanning"));
    }
    struct net_if *iface = net_if_get_default();

    mp_obj_t list = mp_obj_new_list(0, NULL);
    scan_list = &list;
    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0)) {
        LOG_ERR("Scan request failed");
        mp_raise_OSError(EBUSY);
    }
    wifi_is_scanning = true;
    MP_THREAD_GIL_EXIT();
    k_sem_take(&scan_sem, K_FOREVER);
    MP_THREAD_GIL_ENTER();
    wifi_is_scanning = false;
    return list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_scan_obj, network_wlan_scan);

STATIC mp_obj_t network_wlan_isconnected(mp_obj_t self_in) {
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->if_id == MOD_NETWORK_STA_IF) {
        return mp_obj_new_bool(wifi_sta_connected);
    } else {
        return mp_obj_new_bool(false);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_isconnected_obj, network_wlan_isconnected);

STATIC mp_obj_t network_ifconfig(size_t n_args, const mp_obj_t *args) {
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (self->if_id == MOD_NETWORK_AP_IF) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("ipconfig only on Station interface"));
    }
    if (n_args == 1) {
        struct net_if *iface = net_if_get_default();
        struct dns_resolve_context *ctx = dns_resolve_get_default();

        char ip_info[32];

        mp_obj_t tuple[4] = {
            // TODO: Handle IPv6 cases
            mp_obj_new_str(net_addr_ntop(AF_INET, &iface->config.ip.ipv4->unicast[0].address.in_addr.s_addr, ip_info, sizeof(ip_info)), strlen(ip_info)),
            mp_obj_new_str(net_addr_ntop(AF_INET, &iface->config.ip.ipv4->netmask, ip_info, sizeof(ip_info)), strlen(ip_info)),
            mp_obj_new_str(net_addr_ntop(AF_INET, &iface->config.ip.ipv4->gw, ip_info, sizeof(ip_info)), strlen(ip_info)),
            mp_obj_new_str(net_addr_ntop(AF_INET, &((struct sockaddr_in *)&ctx->servers[0].dns_server)->sin_addr.s_addr, ip_info, sizeof(ip_info)), strlen(ip_info)),
        };
        return mp_obj_new_tuple(4, tuple);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_ifconfig_obj, 1, 2, network_ifconfig);

STATIC mp_obj_t network_wlan_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (n_args != 1 && kwargs->used != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("Can only query or set params"));
    }

    struct net_if *iface = net_if_get_default();

    if (kwargs->used != 0) {
        enum { config_none, config_ps, config_ap };
        for (size_t i = 0; i < kwargs->alloc; i++) {
            struct wifi_ps_params params;
            // static struct wifi_connect_req_params ap_params;
            size_t config = config_none;
            if (mp_map_slot_is_filled(kwargs, i)) {
                switch (mp_obj_str_get_qstr(kwargs->table[i].key)) {
                    case MP_QSTR_pm: {
                        params.type = WIFI_PS_PARAM_STATE;
                        params.enabled = (uint8_t)mp_obj_get_int(kwargs->table[i].value);
                        if (params.enabled > WIFI_PS_ENABLED) {
                            mp_raise_ValueError("pm value invalid");
                        }
                        config = config_ps;
                        break;
                    }
                    case MP_QSTR_wmm: {
                        params.type = WIFI_PS_PARAM_MODE;
                        params.mode = (uint8_t)mp_obj_get_int(kwargs->table[i].value);
                        if (params.mode > WIFI_PS_MODE_WMM) {
                            mp_raise_ValueError("wmm must be 0 (legacy) or 1 (wmm)");
                        }
                        config = config_ps;
                        break;
                    }
                    case MP_QSTR_wakeup: {
                        params.type = WIFI_PS_PARAM_WAKEUP_MODE;
                        params.wakeup_mode = (uint8_t)mp_obj_get_int(kwargs->table[i].value);
                        if (params.wakeup_mode > WIFI_PS_WAKEUP_MODE_LISTEN_INTERVAL) {
                            mp_raise_ValueError("Wake up mode must be 0 (dtim) or 1 (interval)");
                        }
                        config = config_ps;
                        break;
                    }
                    case MP_QSTR_listen_interval: {
                        params.type = WIFI_PS_PARAM_LISTEN_INTERVAL;
                        params.listen_interval = (uint8_t)mp_obj_get_int(kwargs->table[i].value);
                        config = config_ps;
                        break;
                    }
                    case MP_QSTR_timeout_ms: {
                        params.type = WIFI_PS_PARAM_TIMEOUT;
                        params.timeout_ms = mp_obj_get_int(kwargs->table[i].value);
                        config = config_ps;
                        break;
                    }
                    /* case MP_QSTR_ssid: {
                         size_t len;
                         const char *s = mp_obj_str_get_data(kwargs->table[i].value, &len);
                         ap_params.ssid = s;
                         ap_params.ssid_length = MIN(len, WIFI_SSID_MAX_LEN);
                         config = config_ap;
                         break;
                     }
                     case MP_QSTR_security:
                     case MP_QSTR_authmode: {
                         ap_params.security = mp_obj_get_int(kwargs->table[i].value);
                         config = config_ap;
                         break;
                     }
                     case MP_QSTR_key:
                     case MP_QSTR_password: {
                         size_t len;
                         const char *s = mp_obj_str_get_data(kwargs->table[i].value, &len);
                         ap_params.psk = (uint8_t *)s;
                         ap_params.psk_length = MIN(len, WIFI_PSK_MAX_LEN);
                         config = config_ap;
                         break;
                     }
                     case MP_QSTR_band: {
                         ap_params.band = mp_obj_get_int(kwargs->table[i].value);
                         config = config_ap;
                         break;
                     }
                     case MP_QSTR_channel: {
                         ap_params.channel = mp_obj_get_int(kwargs->table[i].value);
                         config = config_ap;
                         break;
                     } */
                    default:
                        mp_raise_ValueError("Unknown param");
                }
                if (config == config_ps) {
                    if (self->if_id == MOD_NETWORK_AP_IF) {
                        mp_raise_NotImplementedError(MP_ERROR_TEXT("Power config valid only on Station interface"));
                    } else if (net_mgmt(NET_REQUEST_WIFI_PS, iface, &params, sizeof(params))) {
                        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("%s"), wifi_ps_get_config_err_code_str(params.fail_reason));
                    }
                }/*
                if (config == config_ap) {
                    #ifdef CONFIG_WIFI_NRF700X
                    mp_raise_NotImplementedError(MP_ERROR_TEXT("AP not yet implemented on nRF700x"));
                    #endif
                    if (self->if_id != MOD_NETWORK_AP_IF) {
                        mp_raise_NotImplementedError(MP_ERROR_TEXT("AP config valid only on AP interface"));
                    } else if (net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, iface, &ap_params, sizeof(struct wifi_connect_req_params))) {
                        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error configuring AP"));
                        return mp_const_false;
                    }
                }*/
            }
        }
        return mp_const_none;
    }

    // Get config
    if (n_args != 2) {
        mp_raise_TypeError(MP_ERROR_TEXT("can query only one param"));
    }

    if (self->if_id == MOD_NETWORK_STA_IF) {
        struct wifi_ps_config config = { 0 };
        if (net_mgmt(NET_REQUEST_WIFI_PS_CONFIG, iface, &config, sizeof(config))) {
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed to get configuration"));
        }

        switch (mp_obj_str_get_qstr(args[1])) {
            case MP_QSTR_pm:
                return mp_obj_new_int_from_uint(config.ps_params.enabled);
            case MP_QSTR_wmm:
                return mp_obj_new_int_from_uint(config.ps_params.mode);
            case MP_QSTR_wakeup:
                return mp_obj_new_int_from_uint(config.ps_params.wakeup_mode);
            case MP_QSTR_listen_interval:
                return mp_obj_new_int_from_uint(config.ps_params.listen_interval);
            case MP_QSTR_timeout_ms:
                return mp_obj_new_int(config.ps_params.timeout_ms);
            case MP_QSTR_twt: {
                mp_obj_t flows[config.num_twt_flows];
                for (size_t i = 0; i < config.num_twt_flows; i++) {
                    mp_obj_t tuple[9] = {
                        mp_obj_new_int_from_uint(config.twt_flows[i].dialog_token),
                        mp_obj_new_int_from_uint(config.twt_flows[i].flow_id),
                        mp_obj_new_int_from_uint(config.twt_flows[i].negotiation_type),
                        mp_obj_new_bool(config.twt_flows[i].responder),
                        mp_obj_new_bool(config.twt_flows[i].implicit),
                        mp_obj_new_bool(config.twt_flows[i].announce),
                        mp_obj_new_bool(config.twt_flows[i].trigger),
                        mp_obj_new_int_from_uint(config.twt_flows[i].twt_wake_interval),
                        mp_obj_new_int_from_ull(config.twt_flows[i].twt_interval),
                    };
                    flows[i] = mp_obj_new_tuple(9, tuple);
                }
                return mp_obj_new_tuple(config.num_twt_flows, flows);
            }
            default:
                mp_raise_ValueError(MP_ERROR_TEXT("unknown config param"));
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_wlan_config_obj, 1, network_wlan_config);

STATIC mp_obj_t network_wlan_twt(size_t n_args, const mp_obj_t *args) {
    /* There are a lot of variables that can be set to customize TWT flows, including
     * having mutltiple flows per connection. See more details here:
     * https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/device_guides/working_with_nrf/nrf70/developing/powersave.html#target-wake-time-twt
     *
     * For our purposes, it's probably enough to pick some defaults that make sense
     * for a single flow, and let the user customize the two main ones (wake period and duration).
     */
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (self->if_id != MOD_NETWORK_STA_IF) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("TWT valid only on Station interface"));
    }

    struct net_if *iface = net_if_get_default();
    struct wifi_twt_params params = { 0 };

    if (n_args == 2) {
        switch (mp_obj_str_get_qstr(args[1])) {
            case MP_QSTR_teardown: {
                params.operation = WIFI_TWT_TEARDOWN;
                params.teardown.teardown_all = 1;
                if (net_mgmt(NET_REQUEST_WIFI_TWT, iface, &params, sizeof(params))) {
                    mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed. Reason: %s"), wifi_twt_get_err_code_str(params.fail_reason));
                }
                return mp_const_true;
            }
            default:
                mp_raise_ValueError(MP_ERROR_TEXT("unknown config param"));
        }
    } else if (n_args > 2) {
        if (mp_obj_get_int(args[1]) > WIFI_MAX_TWT_WAKE_INTERVAL_US ||
            mp_obj_get_int(args[1]) == 0) {
            mp_raise_ValueError("Invalid wake duration value");
        }
        if (mp_obj_get_int(args[2]) > WIFI_MAX_TWT_INTERVAL_US ||
            mp_obj_get_int(args[2]) == 0) {
            mp_raise_ValueError("Invalid wake interval value");
        }
        struct wifi_ps_config config = { 0 };
        if (net_mgmt(NET_REQUEST_WIFI_PS_CONFIG, iface, &config, sizeof(config))) {
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed to get configuration"));
        }
        if (config.num_twt_flows > 0) {
            mp_raise_ValueError("TWT already configured");
        }
        params.operation = WIFI_TWT_SETUP;
        params.negotiation_type = WIFI_TWT_INDIVIDUAL;
        params.setup_cmd = WIFI_TWT_SETUP_CMD_DEMAND; // For now demand because we're not accepting other values for negotiation
        params.dialog_token = 1;
        params.flow_id = 0;
        params.setup.responder = 0;
        params.setup.implicit = 1;
        params.setup.trigger = 1;
        params.setup.announce = 0;
        params.setup.twt_wake_interval = mp_obj_get_int(args[1]);
        params.setup.twt_interval = mp_obj_get_int(args[2]);
        mp_obj_t twt_resp_items[3];
        twt_resp = twt_resp_items;
        if (net_mgmt(NET_REQUEST_WIFI_TWT, iface, &params, sizeof(params))) {
            mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed. Reason: %s"), wifi_twt_get_err_code_str(params.fail_reason));
        }
        MP_THREAD_GIL_EXIT();
        k_sem_take(&twt_sem, K_FOREVER);
        MP_THREAD_GIL_ENTER();

        if (mp_obj_get_int(twt_resp_items[0]) == 0) {
            if (n_args == 4) {
                // TODO: register callback to handle wifi_twt_sleep_state interrupts
            }
            return mp_obj_new_tuple(3, twt_resp_items);
        }
        return mp_obj_new_tuple(1, twt_resp_items);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_wlan_twt_obj, 2, 3, network_wlan_twt);

STATIC mp_obj_t network_wlan_irq(mp_obj_t self_in, mp_obj_t handler_in) {
    wlan_if_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (handler_in != mp_const_none && !mp_obj_is_callable(handler_in)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid handler"));
    }
    self->settings->irq_handler = handler_in;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(network_wlan_irq_obj, network_wlan_irq);

#ifdef CONFIG_WIFI_CREDENTIALS
struct profile_connect_ssid {
    const char *ssid;
    size_t ssid_len;
    int timeout;
};

STATIC void connect_stored_profiles_cb(void *cb_arg, const char *ssid, size_t ssid_len) {
    struct profile_connect_ssid *data = (struct profile_connect_ssid *)cb_arg;
    if ( (data->ssid_len == ssid_len) && memcmp(data->ssid, ssid, ssid_len) == 0) {
        struct wifi_credentials_personal creds = { 0 };
        static struct wifi_connect_req_params cnx_params;
        struct net_if *iface = net_if_get_default();

        wifi_credentials_get_by_ssid_personal_struct(ssid, ssid_len, &creds);
        cnx_params.ssid = ssid;
        cnx_params.ssid_length = ssid_len;
        cnx_params.security = creds.header.type;
        if (cnx_params.security != WIFI_SECURITY_TYPE_NONE) {
            cnx_params.psk = creds.password;
            cnx_params.psk_length = creds.password_len;
        }
        cnx_params.timeout = data->timeout;
        cnx_params.channel = WIFI_CHANNEL_ANY;
        cnx_params.mfp = WIFI_MFP_OPTIONAL;
        net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params, sizeof(struct wifi_connect_req_params));
    }
}

STATIC void each_ssid_cb(void *cb_arg, const char *ssid, size_t ssid_len) {
    int ret = 0;
    mp_obj_t *list = (mp_obj_t *)cb_arg;
    struct wifi_credentials_personal creds = { 0 };

    ret = wifi_credentials_get_by_ssid_personal_struct(ssid, ssid_len, &creds);
    if (ret) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed to load credentials. Reason: %d"), ret);
    }
    mp_obj_tuple_t *profile = mp_obj_new_tuple(4, NULL);
    profile->items[0] = mp_obj_new_bytes(ssid, ssid_len); // SSID
    profile->items[1] = (creds.header.flags & WIFI_CREDENTIALS_FLAG_BSSID) ? mp_obj_new_bytes(creds.header.bssid, WIFI_MAC_ADDR_LEN) : mp_const_none;   // BSSID
    profile->items[2] = MP_OBJ_NEW_SMALL_INT(creds.header.type); // Auth
    if ((creds.header.flags & WIFI_CREDENTIALS_FLAG_2_4GHz) && !(creds.header.flags & WIFI_CREDENTIALS_FLAG_5GHz)) {
        profile->items[3] = MP_OBJ_NEW_SMALL_INT(WIFI_FREQ_BAND_2_4_GHZ);
    } else if (!(creds.header.flags & WIFI_CREDENTIALS_FLAG_2_4GHz) && (creds.header.flags & WIFI_CREDENTIALS_FLAG_5GHz)) {
        profile->items[3] = MP_OBJ_NEW_SMALL_INT(WIFI_FREQ_BAND_5_GHZ);
    } else if (creds.header.flags & (WIFI_CREDENTIALS_FLAG_2_4GHz || WIFI_CREDENTIALS_FLAG_5GHz)) {
        profile->items[3] = MP_OBJ_NEW_SMALL_INT(__WIFI_FREQ_BAND_AFTER_LAST);
    } else {
        profile->items[3] = MP_OBJ_NEW_SMALL_INT(WIFI_FREQ_BAND_UNKNOWN);
    }
    mp_obj_list_append(*list, MP_OBJ_FROM_PTR(profile));
}

STATIC void delete_all_ssid_cb(void *cb_arg, const char *ssid, size_t ssid_len) {
    wifi_credentials_delete_by_ssid(ssid, ssid_len);
}

STATIC mp_obj_t network_wlan_credential(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    //wlan_if_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    switch (mp_obj_str_get_qstr(args[1])) {
        case MP_QSTR_list: {
            mp_obj_t list = mp_obj_new_list(0, NULL);
            wifi_credentials_for_each_ssid(each_ssid_cb, &list);
            return list;
        }
        case MP_QSTR_delete_all: {
            wifi_credentials_for_each_ssid(delete_all_ssid_cb, NULL);
            return mp_const_none;
        }
        case MP_QSTR_add: {
            struct wifi_credentials_personal config_buf = { 0 };
            config_buf.header.flags |= (WIFI_CREDENTIALS_FLAG_2_4GHz | WIFI_CREDENTIALS_FLAG_5GHz);
            config_buf.header.type = WIFI_SECURITY_TYPE_UNKNOWN;

            for (size_t i = 0; i < kwargs->alloc; i++) {
                switch (mp_obj_str_get_qstr(kwargs->table[i].key)) {
                    case MP_QSTR_ssid: {
                        const char *s = mp_obj_str_get_data(kwargs->table[i].value, &config_buf.header.ssid_len);
                        memcpy(config_buf.header.ssid, s, config_buf.header.ssid_len);
                        break;
                    }
                    case MP_QSTR_bssid: {
                        size_t bssid_len;
                        const char *s = mp_obj_str_get_data(kwargs->table[i].value, &bssid_len);
                        if (bssid_len != 0 && bssid_len != WIFI_MAC_ADDR_LEN) {
                            mp_raise_ValueError("BSSID must be empty or 6 bytes");
                        }
                        if (bssid_len == WIFI_MAC_ADDR_LEN) {
                            config_buf.header.flags |= WIFI_CREDENTIALS_FLAG_BSSID;
                            memcpy(config_buf.header.bssid, s, WIFI_MAC_ADDR_LEN);
                        }
                        break;
                    }
                    case MP_QSTR_auth: {
                        config_buf.header.type = mp_obj_get_int(kwargs->table[i].value);
                        break;
                    }
                    case MP_QSTR_key: {
                        const char *s = mp_obj_str_get_data(kwargs->table[i].value, &config_buf.password_len);
                        memcpy(config_buf.password, s, MIN(config_buf.password_len, WIFI_CREDENTIALS_MAX_PASSWORD_LEN));
                        break;
                    }
                }
            }
            if (config_buf.header.ssid_len == 0) {
                mp_raise_ValueError("ssid: cannot be empty string");
            }
            if (config_buf.header.type == WIFI_SECURITY_TYPE_UNKNOWN) {
                mp_raise_ValueError("auth: value is required");
            }
            if ((config_buf.header.type == WIFI_SECURITY_TYPE_PSK || config_buf.header.type == WIFI_SECURITY_TYPE_PSK) &&
                (config_buf.password_len == 0 || config_buf.password_len > WIFI_PSK_MAX_LEN)) {
                mp_raise_ValueError("Invalid key length for PSK type");
            }
            if ((config_buf.header.type == WIFI_SECURITY_TYPE_SAE) && (config_buf.password_len == 0 || config_buf.password_len > CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH)) {
                mp_raise_ValueError("Invalid key length for SAE type");
            }
            wifi_credentials_set_personal_struct(&config_buf);
            break;
        }
        case MP_QSTR_connect: {
            if (n_args < 3) {
                mp_raise_ValueError("Must pass SSID");
            }
            size_t ssid_len;
            const char *s = mp_obj_str_get_data(args[2], &ssid_len);
            struct profile_connect_ssid data;
            data.ssid = s;
            data.ssid_len = ssid_len;
            if (n_args > 3) {
                data.timeout = mp_obj_get_int(args[3]);
            } else {
                data.timeout = SYS_FOREVER_MS;
            }
            wifi_credentials_for_each_ssid(connect_stored_profiles_cb, &data);
            break;
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_wlan_credential_obj, 2, network_wlan_credential);
#endif // CONFIG_WIFI_CREDENTIALS

STATIC const mp_rom_map_elem_t wlan_if_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_wlan_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&network_wlan_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&network_wlan_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_wlan_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan), MP_ROM_PTR(&network_wlan_scan_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_wlan_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_wlan_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_ifconfig), MP_ROM_PTR(&network_ifconfig_obj) },
    { MP_ROM_QSTR(MP_QSTR_twt), MP_ROM_PTR(&network_wlan_twt_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq), MP_ROM_PTR(&network_wlan_irq_obj) },
    #ifdef CONFIG_WIFI_CREDENTIALS
    { MP_ROM_QSTR(MP_QSTR_credential), MP_ROM_PTR(&network_wlan_credential_obj) },
    #endif // CONFIG_WIFI_CREDENTIALS

    // Constants
    { MP_ROM_QSTR(MP_QSTR_PM_NONE), MP_ROM_INT(WIFI_PS_DISABLED) },
    { MP_ROM_QSTR(MP_QSTR_PM_PERFORMANCE), MP_ROM_INT(WIFI_PS_ENABLED) },
    { MP_ROM_QSTR(MP_QSTR_PM_POWERSAVE), MP_ROM_INT(WIFI_PS_ENABLED) },
};
STATIC MP_DEFINE_CONST_DICT(wlan_if_locals_dict, wlan_if_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    zephyr_network_wlan_type,
    MP_QSTR_WLAN,
    MP_TYPE_FLAG_NONE,
    make_new, network_wlan_make_new,
    locals_dict, &wlan_if_locals_dict
    );

STATIC mp_obj_network_wlan_settings_t wlan_sta_settings;
STATIC mp_obj_network_wlan_settings_t wlan_ap_settings;

STATIC const wlan_if_obj_t wlan_sta_obj = {{&zephyr_network_wlan_type}, MOD_NETWORK_STA_IF, &wlan_sta_settings};
STATIC const wlan_if_obj_t wlan_ap_obj = {{&zephyr_network_wlan_type}, MOD_NETWORK_AP_IF, &wlan_ap_settings};

#endif // MICROPY_PY_NETWORK_WLAN
