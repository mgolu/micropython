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
mp_obj_t *scan_list;

STATIC mp_obj_t zephyr_initialize() {
    static int initialized = 0;
    if (!initialized) {
        LOG_INF("Initializing interface");
        initialized = 1;
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(zephyr_network_initialize_obj, zephyr_initialize);

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb) {
    const struct wifi_status *status =
        (const struct wifi_status *)cb->info;

    if (wifi_sta_connected) {
        return;
    }

    if (status->status) {
        LOG_ERR("Connection failed (%d)", status->status);
    } else {
        LOG_INF("Connected");
        wifi_sta_connected = true;
    }

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

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
    uint32_t mgmt_event, struct net_if *iface) {
    switch (mgmt_event) {
        case NET_EVENT_WIFI_CONNECT_RESULT:
            handle_wifi_connect_result(cb);
            break;
        case NET_EVENT_WIFI_DISCONNECT_RESULT:
            if (wifi_sta_connected) {
                wifi_sta_connected = false;
                LOG_INF("Received Disconnected");
            }
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
            NET_EVENT_WIFI_IFACE_STATUS);

        net_mgmt_add_event_callback(&wifi_mgmt_cb);
        wifi_initialized = 1;
        LOG_INF("WiFi initialized");
    }
}

STATIC mp_obj_t network_wlan_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    zephyr_initialise_wifi();

    int idx = (n_args > 0) ? mp_obj_get_int(args[0]) : MOD_NETWORK_STA_IF;
    if (idx == MOD_NETWORK_STA_IF) {
        return MP_OBJ_FROM_PTR(&wlan_sta_obj);
    } else if (idx == MOD_NETWORK_AP_IF) {
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
        LOG_ERR("Connection request failed");

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

    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface,
        &iface_status, sizeof(struct wifi_iface_status))) {
        LOG_ERR("Status request failed");
        return mp_const_none;
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
            static struct wifi_connect_req_params ap_params;
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
                    case MP_QSTR_ssid: {
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
                    }
                    default:
                        mp_raise_ValueError("Unknown param");
                }
                if (config == config_ps) {
                    if (self->if_id == MOD_NETWORK_AP_IF) {
                        mp_raise_NotImplementedError(MP_ERROR_TEXT("Power config valid only on Station interface"));
                    } else if (net_mgmt(NET_REQUEST_WIFI_PS, iface, &params, sizeof(params))) {
                        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("%s"), get_ps_config_err_code_str(params.fail_reason));
                    }
                }
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
                }
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

STATIC const mp_rom_map_elem_t wlan_if_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_wlan_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&network_wlan_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&network_wlan_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_wlan_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan), MP_ROM_PTR(&network_wlan_scan_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_wlan_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_wlan_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_ifconfig), MP_ROM_PTR(&network_ifconfig_obj) },

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

STATIC const wlan_if_obj_t wlan_sta_obj = {{&zephyr_network_wlan_type}, MOD_NETWORK_STA_IF};
STATIC const wlan_if_obj_t wlan_ap_obj = {{&zephyr_network_wlan_type}, MOD_NETWORK_AP_IF};

#endif // MICROPY_PY_NETWORK_WLAN
