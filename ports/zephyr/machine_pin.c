/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014, 2015 Damien P. George
 * Copyright (c) 2016 Linaro Limited
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "py/runtime.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "modmachine.h"
#include "shared/runtime/mpirq.h"

#if MICROPY_PY_MACHINE

typedef struct _machine_pin_irq_obj_t {
    mp_irq_obj_t base;
    struct _machine_pin_irq_obj_t *next;
    struct gpio_callback callback;
} machine_pin_irq_obj_t;

STATIC const mp_irq_methods_t machine_pin_irq_methods;

void machine_pin_deinit(void) {
    for (machine_pin_irq_obj_t *irq = MP_STATE_PORT(machine_pin_irq_list); irq != NULL; irq = irq->next) {
        machine_pin_obj_t *pin = MP_OBJ_TO_PTR(irq->base.parent);
        gpio_pin_interrupt_configure(pin->port, pin->pin, GPIO_INT_DISABLE);
        gpio_remove_callback(pin->port, &irq->callback);
    }
    MP_STATE_PORT(machine_pin_irq_list) = NULL;
}

STATIC void gpio_callback_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    // cb is a pointer to the callback field of machine_pin_irq_obj_t.
    machine_pin_irq_obj_t *irq = CONTAINER_OF(cb, machine_pin_irq_obj_t, callback);

    #if MICROPY_STACK_CHECK
    // This callback executes in an ISR context so the stack-limit check must be changed to
    // use the ISR stack for the duration of this function (so that hard IRQ callbacks work).
    char *orig_stack_top = MP_STATE_THREAD(stack_top);
    size_t orig_stack_limit = MP_STATE_THREAD(stack_limit);
    MP_STATE_THREAD(stack_top) = (void *)&irq;
    MP_STATE_THREAD(stack_limit) = CONFIG_ISR_STACK_SIZE - 512;
    #endif

    mp_irq_handler(&irq->base);

    #if MICROPY_STACK_CHECK
    // Restore original stack-limit checking values.
    MP_STATE_THREAD(stack_top) = orig_stack_top;
    MP_STATE_THREAD(stack_limit) = orig_stack_limit;
    #endif
}

STATIC void machine_pin_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "<Pin %q>", self->name);
}

// pin.init(mode, pull=None, *, value)
STATIC mp_obj_t machine_pin_obj_init_helper(const machine_pin_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode, ARG_pull, ARG_value };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_pull, MP_ARG_OBJ, {.u_obj = mp_const_none}},
        { MP_QSTR_value, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get io mode
    uint mode = args[ARG_mode].u_int;

    // get pull mode
    uint pull = 0;
    if (args[ARG_pull].u_obj != mp_const_none) {
        pull = mp_obj_get_int(args[ARG_pull].u_obj);
    }

    // get initial value
    uint init = 0;
    if (args[ARG_value].u_obj != MP_OBJ_NULL) {
        init = mp_obj_is_true(args[ARG_value].u_obj) ? GPIO_OUTPUT_INIT_HIGH : GPIO_OUTPUT_INIT_LOW;
    }

    int ret = gpio_pin_configure(self->port, self->pin, mode | pull | init);
    if (ret) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid pin"));
    }

    return mp_const_none;
}

STATIC const machine_pin_obj_t *machine_pin_find(mp_obj_t pin) {
    if (mp_obj_is_type(pin, &machine_pin_type)) {
        return MP_OBJ_TO_PTR(pin);
    }
    mp_map_elem_t *elem = mp_map_lookup((mp_map_t *)&machine_pin_cpu_pins_locals_dict.map, pin, MP_MAP_LOOKUP);
    if (elem != NULL) {
        return MP_OBJ_TO_PTR(elem->value);
    }
    mp_raise_ValueError("invalid pin");
}

// constructor(drv_name, pin, ...)
mp_obj_t mp_pin_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    const machine_pin_obj_t *pin = machine_pin_find(args[0]);

    if (n_args > 1 || n_kw > 0) {
        // pin mode given, so configure this GPIO
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        machine_pin_obj_init_helper(pin, n_args - 1, args + 1, &kw_args);
    }

    return MP_OBJ_FROM_PTR(pin);
}

// fast method for getting/setting pin value
STATIC mp_obj_t machine_pin_call(mp_obj_t self_in, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (n_args == 0) {
        int pin_val = gpio_pin_get(self->port, self->pin);
        return MP_OBJ_NEW_SMALL_INT(pin_val);
    } else {
        (void)gpio_pin_set(self->port, self->pin, mp_obj_is_true(args[0]));
        return mp_const_none;
    }
}

// pin.init(mode, pull)
STATIC mp_obj_t machine_pin_obj_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return machine_pin_obj_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
MP_DEFINE_CONST_FUN_OBJ_KW(machine_pin_init_obj, 1, machine_pin_obj_init);

// pin.value([value])
STATIC mp_obj_t machine_pin_value(size_t n_args, const mp_obj_t *args) {
    return machine_pin_call(args[0], n_args - 1, 0, args + 1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pin_value_obj, 1, 2, machine_pin_value);

STATIC mp_obj_t machine_pin_off(mp_obj_t self_in) {
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    (void)gpio_pin_set(self->port, self->pin, 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pin_off_obj, machine_pin_off);

STATIC mp_obj_t machine_pin_on(mp_obj_t self_in) {
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    (void)gpio_pin_set(self->port, self->pin, 1);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pin_on_obj, machine_pin_on);

STATIC machine_pin_irq_obj_t *get_irq_for_pin(const machine_pin_obj_t *pin) {
    // Search the linked list for an existing irq object assigned to this pin.
    for (machine_pin_irq_obj_t *irq = MP_STATE_PORT(machine_pin_irq_list); irq != NULL; irq = irq->next) {
        machine_pin_obj_t *irq_pin = MP_OBJ_TO_PTR(irq->base.parent);
        if (irq_pin == pin) {
            return irq;
        }
    }

    // Create and insert this irq at the head of the list.
    machine_pin_irq_obj_t *irq = m_new_obj(machine_pin_irq_obj_t);
    mp_irq_init(&irq->base, &machine_pin_irq_methods, MP_OBJ_FROM_PTR(pin));
    irq->next = MP_STATE_PORT(machine_pin_irq_list);
    gpio_init_callback(&irq->callback, gpio_callback_handler, BIT(pin->pin));
    int ret = gpio_add_callback(pin->port, &irq->callback);
    if (ret != 0) {
        mp_raise_OSError(-ret);
    }
    MP_STATE_PORT(machine_pin_irq_list) = irq;
    return irq;
}

// pin.irq(handler=None, trigger=IRQ_FALLING|IRQ_RISING, hard=False)
STATIC mp_obj_t machine_pin_irq(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_handler, ARG_trigger, ARG_hard };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_trigger, MP_ARG_INT, {.u_int = GPIO_INT_EDGE_BOTH} },
        { MP_QSTR_hard, MP_ARG_BOOL, {.u_bool = false} },
    };
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Find or create IRQ instance for this pin.
    machine_pin_irq_obj_t *irq = get_irq_for_pin(self);

    if (n_args > 1 || kw_args->used != 0) {
        // Remove the existing configuration.
        int ret = gpio_pin_interrupt_configure(self->port, self->pin, GPIO_INT_DISABLE);
        if (ret != 0) {
            mp_raise_OSError(-ret);
        }

        // Update the configuration.
        irq->base.handler = args[ARG_handler].u_obj;
        irq->base.ishard = args[ARG_hard].u_bool;
        if (args[ARG_handler].u_obj != mp_const_none) {
            ret = gpio_pin_interrupt_configure(self->port, self->pin, args[ARG_trigger].u_int);
            if (ret != 0) {
                mp_raise_OSError(-ret);
            }
        }
    }

    return MP_OBJ_FROM_PTR(irq);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_pin_irq_obj, 1, machine_pin_irq);

STATIC mp_uint_t machine_pin_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    (void)errcode;
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    switch (request) {
        case MP_PIN_READ: {
            return gpio_pin_get_raw(self->port, self->pin);
        }
        case MP_PIN_WRITE: {
            return gpio_pin_set_raw(self->port, self->pin, arg);
        }
    }
    return -1;
}

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    machine_pin_cpu_pins_obj_type,
    MP_QSTR_cpu,
    MP_TYPE_FLAG_NONE,
    locals_dict, &machine_pin_cpu_pins_locals_dict
    );

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    machine_pin_board_pins_obj_type,
    MP_QSTR_board,
    MP_TYPE_FLAG_NONE,
    locals_dict, &machine_pin_board_pins_locals_dict
    );

STATIC const mp_rom_map_elem_t machine_pin_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_init),    MP_ROM_PTR(&machine_pin_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_value),   MP_ROM_PTR(&machine_pin_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_off),     MP_ROM_PTR(&machine_pin_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_on),      MP_ROM_PTR(&machine_pin_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq),     MP_ROM_PTR(&machine_pin_irq_obj) },

    // class constants
    { MP_ROM_QSTR(MP_QSTR_IN),        MP_ROM_INT(GPIO_INPUT) },
    { MP_ROM_QSTR(MP_QSTR_OUT),       MP_ROM_INT(GPIO_OUTPUT) },
    { MP_ROM_QSTR(MP_QSTR_PULL_UP),   MP_ROM_INT(GPIO_PULL_UP) },
    { MP_ROM_QSTR(MP_QSTR_PULL_DOWN), MP_ROM_INT(GPIO_PULL_DOWN) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_RISING), MP_ROM_INT(GPIO_INT_EDGE_RISING) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_FALLING), MP_ROM_INT(GPIO_INT_EDGE_FALLING) },

    // class attributes
    { MP_ROM_QSTR(MP_QSTR_board), MP_ROM_PTR(&machine_pin_board_pins_obj_type) },
    { MP_ROM_QSTR(MP_QSTR_cpu), MP_ROM_PTR(&machine_pin_cpu_pins_obj_type) },
};

STATIC MP_DEFINE_CONST_DICT(machine_pin_locals_dict, machine_pin_locals_dict_table);

STATIC const mp_pin_p_t machine_pin_pin_p = {
    .ioctl = machine_pin_ioctl,
};

MP_DEFINE_CONST_OBJ_TYPE(
    machine_pin_type,
    MP_QSTR_Pin,
    MP_TYPE_FLAG_NONE,
    make_new, mp_pin_make_new,
    print, machine_pin_print,
    call, machine_pin_call,
    protocol, &machine_pin_pin_p,
    locals_dict, &machine_pin_locals_dict
    );

STATIC mp_uint_t machine_pin_irq_trigger(mp_obj_t self_in, mp_uint_t new_trigger) {
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (new_trigger == 0) {
        new_trigger = GPIO_INT_DISABLE;
    }
    int ret = gpio_pin_interrupt_configure(self->port, self->pin, new_trigger);
    if (ret != 0) {
        mp_raise_OSError(-ret);
    }
    return 0;
}

STATIC mp_uint_t machine_pin_irq_info(mp_obj_t self_in, mp_uint_t info_type) {
    const machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (info_type == MP_IRQ_INFO_FLAGS) {
        return gpio_get_pending_int(self->port);
    } else if (info_type == MP_IRQ_INFO_TRIGGERS) {
        return 0; // TODO
    }
    return 0;
}

STATIC const mp_irq_methods_t machine_pin_irq_methods = {
    .trigger = machine_pin_irq_trigger,
    .info = machine_pin_irq_info,
};

// Linked list of pin irq objects.
MP_REGISTER_ROOT_POINTER(void *machine_pin_irq_list);

#endif // MICROPY_PY_MACHINE
