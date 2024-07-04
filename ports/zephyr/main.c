/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2016-2017 Linaro Limited
 * Copyright (c) 2020 NXP
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#ifdef CONFIG_NETWORKING
#include <zephyr/net/net_context.h>
#endif

#ifdef CONFIG_USB_DEVICE_STACK
#include <zephyr/usb/usb_device.h>
#endif

#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/gpio.h>

#include "py/mperrno.h"
#include "py/builtin.h"
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "py/stackctrl.h"
#include "shared/runtime/pyexec.h"
#include "shared/readline/readline.h"
#include "extmod/modbluetooth.h"

#if MICROPY_VFS
#include "extmod/vfs.h"
#include "extmod/vfs_lfs.h"
#include "py/stream.h"
#include "src/zephyr_getchar.h"
#endif

#include "modmachine.h"
#include "modzephyr.h"

#ifdef TEST
#include "shared/upytesthelper/upytesthelper.h"
#include "lib/tinytest/tinytest.c"
#include "shared/upytesthelper/upytesthelper.c"
#include TEST
#endif

static char heap[MICROPY_HEAP_SIZE];

#if MICROPY_VFS
STATIC void vfs_init(void) {
    mp_obj_t bdev = NULL;
    mp_obj_t mount_point;
    const char *mount_point_str = NULL;
    int ret = 0;

    #ifdef CONFIG_DISK_DRIVER_SDMMC
    mp_obj_t args[] = { mp_obj_new_str(CONFIG_SDMMC_VOLUME_NAME, strlen(CONFIG_SDMMC_VOLUME_NAME)) };
    bdev = MP_OBJ_TYPE_GET_SLOT(&zephyr_disk_access_type, make_new)(&zephyr_disk_access_type, ARRAY_SIZE(args), 0, args);
    mount_point_str = "/sd";
    #elif defined(CONFIG_FLASH_MAP)
    #ifdef CONFIG_PARTITION_MANAGER_ENABLED
    mp_obj_t args[] = { MP_OBJ_NEW_SMALL_INT(FIXED_PARTITION_ID(mp_filesystem)), MP_OBJ_NEW_SMALL_INT(4096) };
    #elif DT_HAS_CHOSEN(micropython_filesystem)
    mp_obj_t args[] = { MP_OBJ_NEW_SMALL_INT(DT_FIXED_PARTITION_ID(DT_CHOSEN(micropython_filesystem))), MP_OBJ_NEW_SMALL_INT(4096) };
    #elif FIXED_PARTITION_EXISTS(storage_partition)
    mp_obj_t args[] = { MP_OBJ_NEW_SMALL_INT(FIXED_PARTITION_ID(storage_partition)), MP_OBJ_NEW_SMALL_INT(4096) };
    #endif
    bdev = MP_OBJ_TYPE_GET_SLOT(&zephyr_flash_area_type, make_new)(&zephyr_flash_area_type, ARRAY_SIZE(args), 0, args);
    mount_point_str = "/flash";
    #endif

    if ((bdev != NULL)) {
        mount_point = mp_obj_new_str(mount_point_str, strlen(mount_point_str));
        ret = mp_vfs_mount_and_chdir_protected(bdev, mount_point);
        if (ret != 0) {
            printk("Creating filesystem\n");
        #if MICROPY_VFS_LFS2
            mp_map_t *locals_map = &MP_OBJ_TYPE_GET_SLOT(&mp_type_vfs_lfs2, locals_dict)->map;
            mp_map_elem_t *elem = mp_map_lookup(locals_map, MP_OBJ_NEW_QSTR(MP_QSTR_mkfs), MP_MAP_LOOKUP);
            mp_obj_t mkfs[3];
            mkfs[1] = MP_OBJ_NULL;
            mp_convert_member_lookup(MP_OBJ_NULL, &mp_type_vfs_lfs2, elem->value, mkfs);
            mkfs[2] = bdev;
            mp_call_method_n_kw(1, 0, mkfs);
            ret = mp_vfs_mount_and_chdir_protected(bdev, mount_point);
        #endif
        }
        if (ret != 0) {
            return;
        }
        nlr_buf_t nlr;
        mp_obj_t path = mp_obj_new_str("settings", strlen("settings"));
        if (nlr_push(&nlr) == 0) {
            mp_vfs_stat(path);
            nlr_pop();
        } else {
            // Assume error is ENOENT and create the directory
            // TODO: check the actual error
            printk("Settings directory doesn't exist, create it\n");
            mp_vfs_mkdir(path);
        }
        mp_vfs_chdir(path);
        mp_obj_t filename = mp_obj_new_str("start_delay", strlen("start_delay"));
        if (nlr_push(&nlr) == 0) {
            mp_vfs_stat(filename);
            nlr_pop();
        } else {
            // Assume error is ENOENT and create the file
            // TODO: check the actual error
            printk("Creating settings delay file\n");
            mp_obj_t args[2] = {
                filename,
                MP_OBJ_NEW_QSTR(MP_QSTR_wb),
            };
            mp_obj_t file = mp_vfs_open(MP_ARRAY_SIZE(args), &args[0], (mp_map_t *)&mp_const_empty_map);
            mp_stream_write_exactly(file, "2", 1, &ret);
            mp_stream_close(file);
        }
        mp_vfs_chdir(mount_point);
    }
}
#endif // MICROPY_VFS

int real_main(void) {
    mp_stack_ctrl_init();
    // Make MicroPython's stack limit somewhat smaller than full stack available
    mp_stack_set_limit(CONFIG_MAIN_STACK_SIZE - 512);

    mp_hal_init();

    #ifdef TEST
    static const char *argv[] = {"test"};
    upytest_set_heap(heap, heap + sizeof(heap));
    int r = tinytest_main(1, argv, groups);
    printf("status: %d\n", r);
    #endif

soft_reset:
    #if MICROPY_ENABLE_GC
    gc_init(heap, heap + sizeof(heap));
    #endif
    mp_init();

    #ifdef CONFIG_USB_DEVICE_STACK
    usb_enable(NULL);
    #endif

    #if MICROPY_VFS
    vfs_init();
    #endif

    #if MICROPY_MODULE_FROZEN || MICROPY_VFS
    #if DT_HAS_CHOSEN(micropython_skip_main)
    static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_CHOSEN(micropython_skip_main), gpios);
    if (!device_is_ready(button.port)) {
        printk("Main skip button device is not ready\n");
        goto skip_main;
    }
    if (gpio_pin_configure_dt(&button, GPIO_INPUT)) {
        printk("Failed to initialize button\n");
        goto skip_main;
    }
    if (gpio_pin_get_dt(&button) == 1) {
        goto skip_main;
    }
    #endif // DT_HAS_CHOSEN(micropython_skip_main)
    #if MICROPY_VFS && !CONFIG_CONSOLE_SUBSYS
    nlr_buf_t nlr;
    mp_obj_t filename = mp_obj_new_str("/flash/settings/start_delay", strlen("/flash/settings/start_delay"));
    if (nlr_push(&nlr) == 0) {
        mp_vfs_stat(filename);
        nlr_pop();
    } else {
        /* There is no start_delay settings file */
        goto start_main;
    }
    mp_obj_t args[2] = {
        filename,
        MP_OBJ_NEW_QSTR(MP_QSTR_rb),
    };
    mp_obj_t file = mp_vfs_open(MP_ARRAY_SIZE(args), &args[0], (mp_map_t *)&mp_const_empty_map);
    uint8_t buf[3]; /* Limits delay to 99 seconds */
    int ret;
    int len = mp_stream_read_exactly(file, &buf[0], sizeof(buf) - 1, &ret);
    mp_stream_close(file);
    buf[len] = '\0';

    uint32_t delay;
    ret = sscanf(buf, "%u", &delay);
    if (ret != 1) {
        /* Didn't find just an unsigned int */
        goto start_main;
    }
    printk("Press a key in the next %u seconds to stop main.py execution\n", delay);
    if (zephyr_getchar_timeout(delay * 1000, (uint8_t *)&ret) == 0) {
    /* Received a character. Skip main */
        goto skip_main;
    }

    start_main:
    #endif  // MICROPY_VFS && !CONFIG_CONSOLE_SUBSYS
    pyexec_file_if_exists("main.py");
    skip_main:
    #endif  // MICROPY_MODULE_FROZEN || MICROPY_VFS

    for (;;) {
        if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
            if (pyexec_raw_repl() != 0) {
                break;
            }
        } else {
            if (pyexec_friendly_repl() != 0) {
                break;
            }
        }
    }

    printf("soft reboot\n");

    #if MICROPY_PY_BLUETOOTH
    mp_bluetooth_deinit();
    #endif
    #if MICROPY_PY_MACHINE
    machine_pin_deinit();
    #endif

    goto soft_reset;

    return 0;
}

void gc_collect(void) {
    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    void *dummy;
    gc_collect_start();
    gc_collect_root(&dummy, ((mp_uint_t)MP_STATE_THREAD(stack_top) - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
    gc_collect_end();
}

#if !MICROPY_READER_VFS
mp_lexer_t *mp_lexer_new_from_file(qstr filename) {
    mp_raise_OSError(ENOENT);
}
#endif

#if !MICROPY_VFS
mp_import_stat_t mp_import_stat(const char *path) {
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);
#endif

NORETURN void nlr_jump_fail(void *val) {
    while (1) {
        ;
    }
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func, const char *expr) {
    printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("Assertion failed");
}
#endif
