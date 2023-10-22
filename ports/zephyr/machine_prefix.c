/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Jim Mussared
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
#include "py/obj.h"
#include "py/mphal.h"

#include "modmachine.h"

#define Pin(p_name, p_port_label, p_pin) \
    { \
        .base = { &machine_pin_type }, \
        .name = MP_QSTR_##p_name, \
        .port = DEVICE_DT_GET(DT_NODELABEL(p_port_label)), \
        .pin = p_pin, \
    }

#define SPI(dt_label) \
    { \
        .base = { &machine_spi_type }, \
        .name = MP_QSTR_##dt_label, \
        .device = DEVICE_DT_GET(DT_NODELABEL(dt_label)), \
    }

#define I2C(dt_label) \
    { \
        .base = { &machine_i2c_type }, \
        .name = MP_QSTR_##dt_label, \
        .device = DEVICE_DT_GET(DT_NODELABEL(dt_label)), \
    }
