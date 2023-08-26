/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023      Mariano Goluboff
 * Copyright (c) 2019-2021 Damien P. George
 * Copyright (c) 2019-2020 Jim Mussared
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

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"

#if MICROPY_PY_BLUETOOTH

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include "extmod/modbluetooth.h"

#define DEBUG_printf(...) // printk("BLE: " __VA_ARGS__)

#define BLE_HCI_SCAN_ITVL_MIN 0x10
#define BLE_HCI_SCAN_ITVL_MAX 0xffff
#define BLE_HCI_SCAN_WINDOW_MIN 0x10
#define BLE_HCI_SCAN_WINDOW_MAX 0xffff

#define ERRNO_BLUETOOTH_NOT_ACTIVE MP_ENODEV

enum {
    MP_BLUETOOTH_ZEPHYR_BLE_STATE_OFF,
    MP_BLUETOOTH_ZEPHYR_BLE_STATE_ACTIVE,
    MP_BLUETOOTH_ZEPHYR_BLE_STATE_SUSPENDED,
};

enum {
    MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_INACTIVE,
    MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_DEACTIVATING,
    MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_ACTIVE,
};

#define MP_BLUETOOTH_ZEPHYR_MAX_SERVICES    (8)

typedef struct _mp_bluetooth_zephyr_root_pointers_t {
    // Characteristic (and descriptor) value storage.
    mp_gatts_db_t gatts_db;

    size_t n_services;
    struct bt_gatt_service *services[MP_BLUETOOTH_ZEPHYR_MAX_SERVICES];

    struct bt_conn *connections[CONFIG_BT_MAX_CONN];
} mp_bluetooth_zephyr_root_pointers_t;

STATIC int mp_bluetooth_zephyr_ble_state;
STATIC struct bt_conn_cb zephyr_connection_cb;
#if MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING
STATIC struct bt_conn_auth_cb zephyr_conn_auth_cb;
STATIC struct bt_conn_auth_info_cb zephyr_conn_auth_info_cb;
#endif // MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING

#if MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE
STATIC int mp_bluetooth_zephyr_gap_scan_state;
STATIC struct k_timer mp_bluetooth_zephyr_gap_scan_timer;
STATIC struct bt_le_scan_cb mp_bluetooth_zephyr_gap_scan_cb_struct;
#endif

STATIC int bt_err_to_errno(int err) {
    // Zephyr uses errno codes directly, but they are negative.
    return -err;
}

// modbluetooth (and the layers above it) work in BE for addresses, Zephyr works in LE.
STATIC void reverse_addr_byte_order(uint8_t *addr_out, const bt_addr_le_t *addr_in) {
    for (int i = 0; i < 6; ++i) {
        addr_out[i] = addr_in->a.val[5 - i];
    }
}

#if MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE
void gap_scan_cb_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf) {
    DEBUG_printf("gap_scan_cb_recv: adv_type=%d\n", info->adv_type);

    if (!mp_bluetooth_is_active()) {
        return;
    }

    if (mp_bluetooth_zephyr_gap_scan_state != MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_ACTIVE) {
        return;
    }

    uint8_t addr[6];
    reverse_addr_byte_order(addr, info->addr);
    mp_bluetooth_gap_on_scan_result(info->addr->type, addr, info->adv_type, info->rssi, buf->data, buf->len);
}

STATIC mp_obj_t gap_scan_stop(mp_obj_t unused) {
    (void)unused;
    mp_bluetooth_gap_scan_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(gap_scan_stop_obj, gap_scan_stop);

void gap_scan_cb_timeout(struct k_timer *timer_id) {
    DEBUG_printf("gap_scan_cb_timeout\n");
    // Cannot call bt_le_scan_stop from a timer callback because this callback may be
    // preempting the BT stack.  So schedule it to be called from the main thread.
    while (!mp_sched_schedule(MP_OBJ_FROM_PTR(&gap_scan_stop_obj), mp_const_none)) {
        k_yield();
    }
    // Indicate scanning has stopped so that no more scan result events are generated
    // (they may still come in until bt_le_scan_stop is called by gap_scan_stop).
    mp_bluetooth_zephyr_gap_scan_state = MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_DEACTIVATING;
}
#endif

STATIC void zephyr_connected_callback(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;

    if (err) {
        DEBUG_printf("Connection error %u\n", err);
        return;
    }

    bt_conn_get_info(conn, &info);
    MP_STATE_PORT(bluetooth_zephyr_root_pointers)->connections[bt_conn_index(conn)] = bt_conn_ref(conn);

    switch (info.role) {
        case BT_CONN_ROLE_CENTRAL:
            mp_bluetooth_gap_on_connected_disconnected(MP_BLUETOOTH_IRQ_PERIPHERAL_CONNECT, bt_conn_index(conn), bt_conn_get_dst(conn)->type, bt_conn_get_dst(conn)->a.val);
            break;
        case BT_CONN_ROLE_PERIPHERAL:
            mp_bluetooth_gap_on_connected_disconnected(MP_BLUETOOTH_IRQ_CENTRAL_CONNECT, bt_conn_index(conn), bt_conn_get_dst(conn)->type, bt_conn_get_dst(conn)->a.val);
            break;
    }      
}

STATIC void zephyr_disconnected_callback(struct bt_conn *conn, uint8_t reason) {
    struct bt_conn_info info;
    
    bt_conn_get_info(conn, &info);

    switch (info.role) {
        case BT_CONN_ROLE_CENTRAL:
            mp_bluetooth_gap_on_connected_disconnected(MP_BLUETOOTH_IRQ_PERIPHERAL_DISCONNECT, bt_conn_index(conn), bt_conn_get_dst(conn)->type, bt_conn_get_dst(conn)->a.val);
            break;
        case BT_CONN_ROLE_PERIPHERAL:
            mp_bluetooth_gap_on_connected_disconnected(MP_BLUETOOTH_IRQ_CENTRAL_DISCONNECT, bt_conn_index(conn), bt_conn_get_dst(conn)->type, bt_conn_get_dst(conn)->a.val);
            break;        
    }
    MP_STATE_PORT(bluetooth_zephyr_root_pointers)->connections[bt_conn_index(conn)] = NULL;
    bt_conn_unref(conn);
}

#if MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING
STATIC void zephyr_security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    if (err != BT_SECURITY_ERR_SUCCESS) {
        return; // TODO: handle event for unsuccessful pairing
    }
    bool encrypted, authenticated, bonded = false;
    uint8_t key_size = 0;
    switch (level) {
        case BT_SECURITY_L0:
        case BT_SECURITY_L1:
            break;
        case BT_SECURITY_L4:
        case BT_SECURITY_L3:
            authenticated = true;
        case BT_SECURITY_L2:
            encrypted = true;
            break;
    }
    struct bt_conn_info info;
    if (encrypted) {
        bt_conn_get_info(conn, &info);
        key_size = info.security.enc_key_size;
    }
    mp_bluetooth_gatts_on_encryption_update(bt_conn_index(conn), encrypted, authenticated, bonded, key_size);
}
#endif // MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING

int mp_bluetooth_init(void) {
    DEBUG_printf("mp_bluetooth_init\n");

    // Clean up if necessary.
    mp_bluetooth_deinit();

    // Allocate memory for state.
    MP_STATE_PORT(bluetooth_zephyr_root_pointers) = m_new0(mp_bluetooth_zephyr_root_pointers_t, 1);
    mp_bluetooth_gatts_db_create(&MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db);

    #if MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE
    mp_bluetooth_zephyr_gap_scan_state = MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_INACTIVE;
    k_timer_init(&mp_bluetooth_zephyr_gap_scan_timer, gap_scan_cb_timeout, NULL);
    mp_bluetooth_zephyr_gap_scan_cb_struct.recv = gap_scan_cb_recv;
    mp_bluetooth_zephyr_gap_scan_cb_struct.timeout = NULL; // currently not implemented in Zephyr
    bt_le_scan_cb_register(&mp_bluetooth_zephyr_gap_scan_cb_struct);
    #endif

    if (mp_bluetooth_zephyr_ble_state == MP_BLUETOOTH_ZEPHYR_BLE_STATE_OFF) {
        // bt_enable can only be called once.
        int ret = bt_enable(NULL);
        if (ret) {
            return bt_err_to_errno(ret);
        }
        zephyr_connection_cb.connected = zephyr_connected_callback;
        zephyr_connection_cb.disconnected = zephyr_disconnected_callback;
#if MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING
        zephyr_connection_cb.security_changed = zephyr_security_changed_cb;
#endif
        bt_conn_cb_register(&zephyr_connection_cb);
    }
    mp_bluetooth_zephyr_ble_state = MP_BLUETOOTH_ZEPHYR_BLE_STATE_ACTIVE;

    DEBUG_printf("mp_bluetooth_init: ready\n");

    return 0;
}

void mp_bluetooth_deinit(void) {
    DEBUG_printf("mp_bluetooth_deinit %d\n", mp_bluetooth_zephyr_ble_state);
    if (mp_bluetooth_zephyr_ble_state == MP_BLUETOOTH_ZEPHYR_BLE_STATE_OFF
        || mp_bluetooth_zephyr_ble_state == MP_BLUETOOTH_ZEPHYR_BLE_STATE_SUSPENDED) {
        return;
    }

    mp_bluetooth_gap_advertise_stop();

    #if MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE
    mp_bluetooth_gap_scan_stop();
    bt_le_scan_cb_unregister(&mp_bluetooth_zephyr_gap_scan_cb_struct);
    #endif

    // There is no way to turn off the BT stack in Zephyr, so just set the
    // state as suspended so it can be correctly reactivated later.
    mp_bluetooth_zephyr_ble_state = MP_BLUETOOTH_ZEPHYR_BLE_STATE_SUSPENDED;

    MP_STATE_PORT(bluetooth_zephyr_root_pointers) = NULL;
}

bool mp_bluetooth_is_active(void) {
    return mp_bluetooth_zephyr_ble_state == MP_BLUETOOTH_ZEPHYR_BLE_STATE_ACTIVE;
}

void mp_bluetooth_get_current_address(uint8_t *addr_type, uint8_t *addr) {
    if (!mp_bluetooth_is_active()) {
        mp_raise_OSError(ERRNO_BLUETOOTH_NOT_ACTIVE);
    }
    bt_addr_le_t le_addr;
    size_t count = 1;
    bt_id_get(&le_addr, &count);
    if (count == 0) {
        mp_raise_OSError(EIO);
    }
    reverse_addr_byte_order(addr, &le_addr);
    *addr_type = le_addr.type;
}

void mp_bluetooth_set_address_mode(uint8_t addr_mode) {
    // TODO: implement
}

size_t mp_bluetooth_gap_get_device_name(const uint8_t **buf) {
    const char *name = bt_get_name();
    *buf = (const uint8_t *)name;
    return strlen(name);
}

int mp_bluetooth_gap_set_device_name(const uint8_t *buf, size_t len) {
    char tmp_buf[CONFIG_BT_DEVICE_NAME_MAX + 1];
    if (len + 1 > sizeof(tmp_buf)) {
        return MP_EINVAL;
    }
    memcpy(tmp_buf, buf, len);
    tmp_buf[len] = '\0';
    return bt_err_to_errno(bt_set_name(tmp_buf));
}

// Zephyr takes advertising/scan data as an array of (type, len, payload) packets,
// and this function constructs such an array from raw advertising/scan data.
STATIC void mp_bluetooth_prepare_bt_data(const uint8_t *data, size_t len, struct bt_data *bt_data, size_t *bt_len) {
    size_t i = 0;
    const uint8_t *d = data;
    while (d < data + len && i < *bt_len) {
        bt_data[i].type = d[1];
        bt_data[i].data_len = d[0] - 1;
        bt_data[i].data = &d[2];
        i += 1;
        d += 1 + d[0];
    }
    *bt_len = i;
}

int mp_bluetooth_gap_advertise_start(bool connectable, int32_t interval_us, const uint8_t *adv_data, size_t adv_data_len, const uint8_t *sr_data, size_t sr_data_len) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }

    mp_bluetooth_gap_advertise_stop();

    struct bt_data bt_ad_data[8];
    size_t bt_ad_len = 0;
    if (adv_data) {
        bt_ad_len = MP_ARRAY_SIZE(bt_ad_data);
        mp_bluetooth_prepare_bt_data(adv_data, adv_data_len, bt_ad_data, &bt_ad_len);
    }

    struct bt_data bt_sd_data[8];
    size_t bt_sd_len = 0;
    if (sr_data) {
        bt_sd_len = MP_ARRAY_SIZE(bt_sd_data);
        mp_bluetooth_prepare_bt_data(sr_data, sr_data_len, bt_sd_data, &bt_sd_len);
    }

    struct bt_le_adv_param param = {
        .id = 0,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = (connectable ? BT_LE_ADV_OPT_CONNECTABLE : 0)
            | BT_LE_ADV_OPT_ONE_TIME
            | BT_LE_ADV_OPT_USE_IDENTITY
            | BT_LE_ADV_OPT_SCANNABLE,
        .interval_min = interval_us / 625,
        .interval_max = interval_us / 625 + 1, // min/max cannot be the same value
        .peer = NULL,
    };

    return bt_err_to_errno(bt_le_adv_start(&param, bt_ad_data, bt_ad_len, bt_sd_data, bt_sd_len));
}

void mp_bluetooth_gap_advertise_stop(void) {
    // Note: bt_le_adv_stop returns 0 if adv is already stopped.
    int ret = bt_le_adv_stop();
    if (ret != 0) {
        mp_raise_OSError(bt_err_to_errno(ret));
    }
}

static struct bt_uuid *uuid_chrc = BT_UUID_GATT_CHRC;
static struct bt_uuid *uuid_serv = BT_UUID_GATT_PRIMARY;
static struct bt_uuid *uuid_ccc = BT_UUID_GATT_CCC;
static struct bt_uuid *uuid_cep = BT_UUID_GATT_CEP;
static struct bt_uuid *uuid_cpf = BT_UUID_GATT_CPF;
static struct bt_uuid *uuid_cud = BT_UUID_GATT_CUD;

STATIC ssize_t attr_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset) {
    // TODO: Implement onreadrequest to issue IRQ_GATTS_WRITE?
    MICROPY_PY_BLUETOOTH_ENTER
    mp_bluetooth_gatts_db_entry_t *entry = mp_bluetooth_gatts_db_lookup(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db, attr->handle);
    uint16_t ret = bt_gatt_attr_read(conn, attr, buf, len, offset, entry->data, entry->data_len);
    MICROPY_PY_BLUETOOTH_EXIT
    return ret;
}

STATIC ssize_t attr_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
        return BT_GATT_ERR(BT_ATT_ERR_SUCCESS);
    }
    MICROPY_PY_BLUETOOTH_ENTER
    mp_bluetooth_gatts_db_entry_t *entry = mp_bluetooth_gatts_db_lookup(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db, attr->handle);
    if (!entry) {
        MICROPY_PY_BLUETOOTH_EXIT
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_HANDLE);
    }
    if (entry->append) {
        offset += entry->data_len;
    }

    ssize_t write_len = MIN(entry->data_alloc - offset, len);
    memcpy(entry->data + offset, buf, write_len);
    entry->data_len += write_len;
    MICROPY_PY_BLUETOOTH_EXIT
    mp_bluetooth_gatts_on_write(bt_conn_index(conn), attr->handle);
    return write_len;
}

STATIC void mp_db_reset_service(struct bt_gatt_service *service) {
    switch (service->attrs[0].uuid->type) {
        case BT_UUID_SIZE_16:
            m_del(struct bt_uuid16, service->attrs[0].user_data, 1);
            break;
        case BT_UUID_SIZE_32:
            m_del(struct bt_uuid32, service->attrs[0].user_data, 1);
            break;
        case BT_UUID_SIZE_128:
            m_del(struct bt_uuid128, service->attrs[0].user_data, 1);
            break;
    }   
    for (size_t i = 1; i < service->attr_count; ++i) {        
        if (service->attrs[i].uuid == uuid_ccc) {
            m_del(struct _bt_gatt_ccc, service->attrs[i].user_data, 1);
        } else if (service->attrs[i].uuid == uuid_chrc) {
            m_del(struct bt_gatt_chrc, service->attrs[i].user_data, 1); 
        } else if (service->attrs[i].read == attr_read) {
            switch (service->attrs[i].uuid->type) {
            case BT_UUID_SIZE_16:
                m_del(struct bt_uuid16, (void *)service->attrs[i].uuid, 1);
                break;
            case BT_UUID_SIZE_32:
                m_del(struct bt_uuid32, (void *)service->attrs[i].uuid, 1);
                break;
            case BT_UUID_SIZE_128:
                m_del(struct bt_uuid128, (void *)service->attrs[i].uuid, 1);
                break;
            }
        }
    }
    m_del(struct bt_gatt_attr, service->attrs, service->attr_count);
}

int mp_bluetooth_gatts_register_service_begin(bool append) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }

    if (append) {
        // Don't support append yet (modbluetooth.c doesn't support it yet anyway).
        return MP_EOPNOTSUPP;
    }
    for (size_t i = 0; i < MP_STATE_PORT(bluetooth_zephyr_root_pointers)->n_services; ++i) {
        bt_gatt_service_unregister(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->services[i]);
        mp_db_reset_service(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->services[i]);
    }
    if (MP_STATE_PORT(bluetooth_zephyr_root_pointers)->n_services > 0) {
        m_del(struct bt_gatt_service, MP_STATE_PORT(bluetooth_zephyr_root_pointers)->services, 
            MP_STATE_PORT(bluetooth_zephyr_root_pointers)->n_services);
    }

    // Reset the gatt characteristic value db.
    mp_bluetooth_gatts_db_reset(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db);
    MP_STATE_PORT(bluetooth_zephyr_root_pointers)->n_services = 0;

    return 0;
}

int mp_bluetooth_gatts_register_service_end(void) {
    return 0;
}

STATIC struct bt_uuid *create_uuid_pointer(const mp_obj_bluetooth_uuid_t *uuid) {
    if (uuid->type == MP_BLUETOOTH_UUID_TYPE_16) {
        return (struct bt_uuid *)m_new(struct bt_uuid_16, 1);
    } else if (uuid->type == MP_BLUETOOTH_UUID_TYPE_32) {
        return (struct bt_uuid *)m_new(struct bt_uuid_32, 1);
    } else if (uuid->type == MP_BLUETOOTH_UUID_TYPE_128) {
        return (struct bt_uuid *)m_new(struct bt_uuid_128, 1);
    } else {
        return NULL;
    }
}

STATIC void client_config_change(const struct bt_gatt_attr *attr, uint16_t value) {
    /* Not yet implemented in Micropython */
    return;
}

STATIC uint16_t get_bt_gatt_perm(uint16_t mp_flags) {
    uint16_t ret = BT_GATT_PERM_NONE;

    if (mp_flags & MP_BLUETOOTH_CHARACTERISTIC_FLAG_READ) {
        ret |= BT_GATT_PERM_READ;
    }
    if (mp_flags & ( MP_BLUETOOTH_CHARACTERISTIC_FLAG_WRITE | 
            MP_BLUETOOTH_CHARACTERISTIC_FLAG_WRITE_NO_RESPONSE |
            MP_BLUETOOTH_CHARACTERISTIC_FLAG_AUTHENTICATED_SIGNED_WRITE)) {
        ret |= BT_GATT_PERM_WRITE;
    }
    if (mp_flags & MP_BLUETOOTH_CHARACTERISTIC_FLAG_READ_ENCRYPTED) {
        ret |= BT_GATT_PERM_READ_ENCRYPT;
    }
    if (mp_flags & MP_BLUETOOTH_CHARACTERISTIC_FLAG_READ_AUTHENTICATED) {
        ret |= BT_GATT_PERM_READ_AUTHEN;
    }
    if (mp_flags & MP_BLUETOOTH_CHARACTERISTIC_FLAG_WRITE_ENCRYPTED) {
        ret |= BT_GATT_PERM_WRITE_ENCRYPT;
    }
    if (mp_flags & MP_BLUETOOTH_CHARACTERISTIC_FLAG_WRITE_AUTHENTICATED) {
        ret |= BT_GATT_PERM_WRITE_AUTHEN;
    }
    return ret;
}

STATIC uint8_t get_bt_gatt_prop(uint16_t mp_flags) {
    uint8_t ret = 0;
    ret |= mp_flags & 0x7f;
    // TODO: Handle Extended Properties
    return ret;
}

int mp_bluetooth_gatts_register_service(mp_obj_bluetooth_uuid_t *service_uuid, mp_obj_bluetooth_uuid_t **characteristic_uuids, uint16_t *characteristic_flags, mp_obj_bluetooth_uuid_t **descriptor_uuids, uint16_t *descriptor_flags, uint8_t *num_descriptors, uint16_t *handles, size_t num_characteristics) {
    if (MP_STATE_PORT(bluetooth_zephyr_root_pointers)->n_services == MP_BLUETOOTH_ZEPHYR_MAX_SERVICES) {
        return MP_E2BIG;
    }
    /* Calculate how many attributes will be in the service */
    size_t num_attrs = 1; // Start with the attribute for the service
    num_attrs += num_characteristics * 2; // Each characteristic is 2 attributes, plus the descriptors
    for (size_t i = 0; i < num_characteristics; ++i) {
        if ((characteristic_flags[i] & (MP_BLUETOOTH_CHARACTERISTIC_FLAG_NOTIFY | MP_BLUETOOTH_CHARACTERISTIC_FLAG_INDICATE)) != 0) {
            num_attrs++;  // Add Client Characteristic Configuration
        }
        num_attrs += num_descriptors[i];
    }

    struct bt_gatt_attr *attrs = m_new(struct bt_gatt_attr, num_attrs);
    size_t attr_idx = 0;
    size_t handle_idx = 0;
    size_t descriptor_idx = 0;
    for (size_t i = 0; i < num_characteristics; ++i) {
        attr_idx++;
        attrs[attr_idx].uuid = uuid_chrc;
        attrs[attr_idx].perm = BT_GATT_PERM_READ;
        attrs[attr_idx].read = bt_gatt_attr_read_chrc;
        attrs[attr_idx].write = NULL;
        attrs[attr_idx].handle = 0;
        struct bt_gatt_chrc *char_attr_val = m_new(struct bt_gatt_chrc, 1);
        struct bt_uuid *char_uu = create_uuid_pointer(characteristic_uuids[i]);
        if (char_uu != NULL) {
            bt_uuid_create(char_uu, characteristic_uuids[i]->data, characteristic_uuids[i]->type);
        }
        char_attr_val[0].uuid = char_uu;
        char_attr_val[0].value_handle = 0;
        char_attr_val[0].properties = get_bt_gatt_prop(characteristic_flags[i]);
        attrs[attr_idx].user_data = char_attr_val;
        /* Create the actual characteristic */
        attr_idx++;
        attrs[attr_idx].uuid = char_uu;
        attrs[attr_idx].perm = get_bt_gatt_perm(characteristic_flags[i]);
        attrs[attr_idx].read = attr_read;
        attrs[attr_idx].write = attr_write;
        handles[handle_idx] = attr_idx; // Relative position, we'll adjust later
        handle_idx++;
        if ((characteristic_flags[i] & (MP_BLUETOOTH_CHARACTERISTIC_FLAG_NOTIFY | MP_BLUETOOTH_CHARACTERISTIC_FLAG_INDICATE)) != 0) {
            struct _bt_gatt_ccc *ccc = m_new(struct _bt_gatt_ccc, 1);
            // ccc[0].cfg = {};
            ccc[0].cfg_changed = client_config_change;
            ccc[0].cfg_write = NULL; // Let the Zephyr stack manage confirmation
            ccc[0].cfg_match = NULL;
            attr_idx++;
            attrs[attr_idx].uuid = uuid_ccc;
            attrs[attr_idx].perm = BT_GATT_PERM_READ | BT_GATT_PERM_WRITE;
            attrs[attr_idx].read = bt_gatt_attr_read_ccc;
            attrs[attr_idx].write = bt_gatt_attr_write_ccc;
            attrs[attr_idx].user_data = ccc;
            attrs[attr_idx].handle = 0;
            /* We don't allocate a handle as micropython doesn't expect handle for CCC */
        }
        for (size_t j = 0; j < num_descriptors[i]; ++j) {
            if (descriptor_uuids[descriptor_idx]->type != MP_BLUETOOTH_UUID_TYPE_16) {
                return MP_EINVAL;
            }
            attr_idx++;
            uint16_t value = (descriptor_uuids[descriptor_idx]->data[1] << 8) | descriptor_uuids[descriptor_idx]->data[0];
            attrs[attr_idx].write = NULL;
            if (value == BT_UUID_GATT_CEP_VAL) {
                attrs[attr_idx].uuid = uuid_cep;
                attrs[attr_idx].read = bt_gatt_attr_read_cep;
                // TODO: Add write and manage user_data
            } else if (value == BT_UUID_GATT_CUD_VAL) {
                attrs[attr_idx].uuid = uuid_cud;
                attrs[attr_idx].read = bt_gatt_attr_read_cud;
                attrs[attr_idx].user_data = "";
            } else if (value == BT_UUID_GATT_CPF_VAL) {
                attrs[attr_idx].uuid = uuid_cpf;
                attrs[attr_idx].read = bt_gatt_attr_read_cpf;
                // TODO: Add write and manage user_data
            } else {
                return MP_EINVAL;
            }
            attrs[attr_idx].perm = descriptor_flags[descriptor_idx];
            attrs[attr_idx].handle = 0;
            handles[handle_idx] = attr_idx;
            handle_idx++;
            descriptor_idx++;
        }
    }
    attrs[0].uuid = uuid_serv;
    attrs[0].perm = BT_GATT_PERM_READ;
    attrs[0].read = bt_gatt_attr_read_service;
    attrs[0].handle = 0;
    struct bt_uuid *service_uuid_zephyr = create_uuid_pointer(service_uuid);
    if (service_uuid_zephyr != NULL) {
        bt_uuid_create(service_uuid_zephyr, service_uuid->data, service_uuid->type);
    }
    attrs[0].user_data = service_uuid_zephyr;
    struct bt_gatt_service *service = m_new(struct bt_gatt_service, 1);
    service[0].attrs = attrs;
    service[0].attr_count = num_attrs;
    MP_STATE_PORT(bluetooth_zephyr_root_pointers)->services[MP_STATE_PORT(bluetooth_zephyr_root_pointers)->n_services++] = service;

    int ret = bt_gatt_service_register(service);
    if (ret < 0) {
        return bt_err_to_errno(ret);
    }

    uint16_t offset = attrs[0].handle;
    for (size_t i = 0; i < handle_idx; ++i) {
        handles[i] += offset;
    }
    for (size_t i = 0; i < num_attrs; ++i) {
        if (attrs[i].read == attr_read) {
            mp_bluetooth_gatts_db_create_entry(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db, attrs[i].handle, MP_BLUETOOTH_DEFAULT_ATTR_LEN);
        }
    }
    return 0;
}

STATIC int find_attr_by_handle(uint16_t handle, struct bt_gatt_attr **attr_p) {
    for (size_t i = 0; i < MP_STATE_PORT(bluetooth_zephyr_root_pointers)->n_services; ++i) {
        for (size_t j = 0; j < MP_STATE_PORT(bluetooth_zephyr_root_pointers)->services[i]->attr_count; ++j) {
            if (MP_STATE_PORT(bluetooth_zephyr_root_pointers)->services[i]->attrs[j].handle == handle) {
                *attr_p = &MP_STATE_PORT(bluetooth_zephyr_root_pointers)->services[i]->attrs[j];
                return 0;
            }
        }
    }
    return -1;
}

int mp_bluetooth_gap_disconnect(uint16_t conn_handle) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gatts_read(uint16_t value_handle, const uint8_t **value, size_t *value_len) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    return mp_bluetooth_gatts_db_read(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db, value_handle, value, value_len);
}

int mp_bluetooth_gatts_write(uint16_t value_handle, const uint8_t *value, size_t value_len, bool send_update) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    int err = mp_bluetooth_gatts_db_write(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db, value_handle, value, value_len);
    if (err == 0 && send_update) {
        struct bt_gatt_attr *attr;
        if (find_attr_by_handle(value_handle - 1, &attr) == 0 && attr->uuid == uuid_chrc) {
            /* Notify and indicate with a handle out of range to send to all that are subscribed */
            if ( ((struct bt_gatt_chrc *)attr->user_data)[0].properties & BT_GATT_CHRC_NOTIFY) {
                mp_bluetooth_gatts_notify_indicate(CONFIG_BT_MAX_CONN, value_handle, MP_BLUETOOTH_GATTS_OP_NOTIFY, value, value_len);
            }
            if ( ((struct bt_gatt_chrc *)attr->user_data)[0].properties & BT_GATT_CHRC_INDICATE) {
                mp_bluetooth_gatts_notify_indicate(CONFIG_BT_MAX_CONN, value_handle, MP_BLUETOOTH_GATTS_OP_INDICATE, value, value_len);
            }
        }
    }
    return err;
}

STATIC void indicate_complete(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err) {
    mp_bluetooth_gatts_on_indicate_complete(bt_conn_index(conn), params->attr->handle, err);
}

STATIC void free_indicate_params(struct bt_gatt_indicate_params *params) {
    m_del(struct bt_gatt_indicate_params, params, 1);
}

int mp_bluetooth_gatts_notify_indicate(uint16_t conn_handle, uint16_t value_handle, int gatts_op, const uint8_t *value, size_t value_len) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    int err = 0;
    struct bt_gatt_attr *attr;
    struct bt_conn *conn = NULL;

    if (value == NULL) {
        err = mp_bluetooth_gatts_read(value_handle, &value, &value_len);
        if (err) {
            return err;
        }
    }
    if (conn_handle < CONFIG_BT_MAX_CONN) {
        conn = MP_STATE_PORT(bluetooth_zephyr_root_pointers)->connections[conn_handle];
    }
    if (find_attr_by_handle(value_handle, &attr) == 0) {
        switch (gatts_op) {
            case MP_BLUETOOTH_GATTS_OP_NOTIFY:
                err = bt_gatt_notify(conn, attr, value, value_len);
                if (err == -EINVAL) {
                    /* We're going to ignore the error when the peer is not subscribed.
                     * micropythong documentation says that data will be sent anyway, but
                     * it is against BLE spec, so Zephyr returns an error and does not send.
                     */
                    err = 0;
                }
                break;
            case MP_BLUETOOTH_GATTS_OP_INDICATE:
            {
                struct bt_gatt_indicate_params *zephyr_indicate_params = m_new(struct bt_gatt_indicate_params, 1);
                memset(zephyr_indicate_params, 0, sizeof(struct bt_gatt_indicate_params));
                zephyr_indicate_params->attr = attr;
                zephyr_indicate_params->data = value;
                zephyr_indicate_params->len = value_len;
                zephyr_indicate_params->func = indicate_complete;
                zephyr_indicate_params->destroy = free_indicate_params;
                bt_gatt_indicate(conn, zephyr_indicate_params);
                break;
            }

        }
    } else {
        return MP_EINVAL;
    }
    return err;
}

int mp_bluetooth_gatts_set_buffer(uint16_t value_handle, size_t len, bool append) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    return mp_bluetooth_gatts_db_resize(MP_STATE_PORT(bluetooth_zephyr_root_pointers)->gatts_db, value_handle, len, append);
}

int mp_bluetooth_get_preferred_mtu(void) {
    if (!mp_bluetooth_is_active()) {
        mp_raise_OSError(ERRNO_BLUETOOTH_NOT_ACTIVE);
    }
    mp_raise_OSError(MP_EOPNOTSUPP);
}

int mp_bluetooth_set_preferred_mtu(uint16_t mtu) {
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    return MP_EOPNOTSUPP;
}

#if MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE

int mp_bluetooth_gap_scan_start(int32_t duration_ms, int32_t interval_us, int32_t window_us, bool active_scan) {
    // Stop any ongoing GAP scan.
    int ret = mp_bluetooth_gap_scan_stop();
    if (ret) {
        return ret;
    }

    struct bt_le_scan_param param = {
        .type = active_scan ? BT_HCI_LE_SCAN_ACTIVE : BT_HCI_LE_SCAN_PASSIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = MAX(BLE_HCI_SCAN_ITVL_MIN, MIN(BLE_HCI_SCAN_ITVL_MAX, interval_us / 625)),
        .window = MAX(BLE_HCI_SCAN_WINDOW_MIN, MIN(BLE_HCI_SCAN_WINDOW_MAX, window_us / 625)),
    };
    k_timer_start(&mp_bluetooth_zephyr_gap_scan_timer, K_MSEC(duration_ms), K_NO_WAIT);
    mp_bluetooth_zephyr_gap_scan_state = MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_ACTIVE;
    int err = bt_le_scan_start(&param, NULL);
    return bt_err_to_errno(err);
}

int mp_bluetooth_gap_scan_stop(void) {
    DEBUG_printf("mp_bluetooth_gap_scan_stop\n");
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    if (mp_bluetooth_zephyr_gap_scan_state == MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_INACTIVE) {
        // Already stopped.
        return 0;
    }
    mp_bluetooth_zephyr_gap_scan_state = MP_BLUETOOTH_ZEPHYR_GAP_SCAN_STATE_INACTIVE;
    k_timer_stop(&mp_bluetooth_zephyr_gap_scan_timer);
    int err = bt_le_scan_stop();
    if (err == 0) {
        mp_bluetooth_gap_on_scan_complete();
        return 0;
    }
    return bt_err_to_errno(err);
}

int mp_bluetooth_gap_peripheral_connect(uint8_t addr_type, const uint8_t *addr, int32_t duration_ms, int32_t min_conn_interval_us, int32_t max_conn_interval_us) {
    DEBUG_printf("mp_bluetooth_gap_peripheral_connect\n");
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gap_peripheral_connect_cancel(void) {
    DEBUG_printf("mp_bluetooth_gap_peripheral_connect_cancel\n");
    if (!mp_bluetooth_is_active()) {
        return ERRNO_BLUETOOTH_NOT_ACTIVE;
    }
    return MP_EOPNOTSUPP;
}

#endif // MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE

#if MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING
int mp_bluetooth_gap_pair(uint16_t conn_handle) {
    DEBUG_printf("mp_bluetooth_gap_pair: conn_handle=%d\n", conn_handle);
    // TODO: handle initiation from micropython side
    return -1;
}

int mp_bluetooth_gap_passkey(uint16_t conn_handle, uint8_t action, mp_int_t passkey) {
    struct bt_conn *conn = NULL;
    if (conn_handle >= CONFIG_BT_MAX_CONN) {
        return MP_EINVAL;
    }
    conn = MP_STATE_PORT(bluetooth_zephyr_root_pointers)->connections[conn_handle];
    switch (action) {
        case MP_BLUETOOTH_PASSKEY_ACTION_INPUT:
            bt_conn_auth_passkey_entry(conn, passkey);
            break;
        case MP_BLUETOOTH_PASSKEY_ACTION_NUMERIC_COMPARISON: {
            if (passkey != 0) {
                bt_conn_auth_passkey_confirm(conn);
            } else {
                bt_conn_auth_cancel(conn);
            }
            break;
        }
        default: {
            return MP_EINVAL;
        }
    }
    return 0;
}

void mp_bluetooth_set_bonding(bool enabled) {
    bt_set_bondable(enabled);
}

void mp_bluetooth_set_mitm_protection(bool enabled) {
    // TODO
}

void mp_bluetooth_set_le_secure(bool enabled) {
    // TODO
}

STATIC void zephyr_passkey_display(struct bt_conn *conn, unsigned int passkey) {
    DEBUG_printf("mp_bluetooth_passkey_display: passkey=%d\n", passkey);
    mp_bluetooth_gap_on_passkey_action(bt_conn_index(conn), MP_BLUETOOTH_PASSKEY_ACTION_DISPLAY, passkey);
}

STATIC void zephyr_passkey_entry(struct bt_conn *conn) {
    DEBUG_printf("mp_bluetooth_passkey_entry\n");
    mp_bluetooth_gap_on_passkey_action(bt_conn_index(conn), MP_BLUETOOTH_PASSKEY_ACTION_INPUT, 0);
}

STATIC void zephyr_passkey_confirm(struct bt_conn *conn, unsigned int passkey) {
    DEBUG_printf("mp_bluetooth_passkey_confirm: passkey=%d\n", passkey);
    mp_bluetooth_gap_on_passkey_action(bt_conn_index(conn), MP_BLUETOOTH_PASSKEY_ACTION_NUMERIC_COMPARISON, passkey);
}

STATIC void zephyr_cancel_display(struct bt_conn *conn) {
    DEBUG_printf("mp_bluetooth_passkey_cancel\n");
    mp_bluetooth_gap_on_passkey_action(bt_conn_index(conn), MP_BLUETOOTH_PASSKEY_ACTION_NONE, 0);
}

void mp_bluetooth_set_io_capability(uint8_t capability) {
    bt_conn_auth_cb_register(NULL); // Unregister previous callback structure 
    memset(&zephyr_conn_auth_cb, 0, sizeof(struct bt_conn_auth_cb));
    switch (capability) {
        case MP_BLUETOOTH_IO_CAPABILITY_KEYBOARD_DISPLAY:
            zephyr_conn_auth_cb.passkey_entry = zephyr_passkey_entry;
        case MP_BLUETOOTH_IO_CAPABILITY_DISPLAY_YESNO:
            zephyr_conn_auth_cb.passkey_confirm = zephyr_passkey_confirm;
        case MP_BLUETOOTH_IO_CAPABILITY_DISPLAY_ONLY:
            zephyr_conn_auth_cb.passkey_display = zephyr_passkey_display;
            zephyr_conn_auth_cb.cancel = zephyr_cancel_display;
            break;
        case MP_BLUETOOTH_IO_CAPABILITY_KEYBOARD_ONLY:
            zephyr_conn_auth_cb.passkey_entry = zephyr_passkey_entry;
            zephyr_conn_auth_cb.cancel = zephyr_cancel_display;
            break;
        case MP_BLUETOOTH_IO_CAPABILITY_NO_INPUT_OUTPUT:
            return;
        default:
            return;
    }
    bt_conn_auth_cb_register(&zephyr_conn_auth_cb);
}
#endif // MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING

MP_REGISTER_ROOT_POINTER(struct _mp_bluetooth_zephyr_root_pointers_t *bluetooth_zephyr_root_pointers);

#endif // MICROPY_PY_BLUETOOTH
