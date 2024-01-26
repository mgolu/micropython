# MicroPython port to Zephyr RTOS

This is a work-in-progress port of MicroPython to the
[Zephyr RTOS](http://zephyrproject.org). This allows MicroPython to run on the
wide range of boards supported by Zephyr.

This port requires Zephyr version v3.4.0, and may also work on higher
versions.

All boards supported by Zephyr (with standard level of features support, like
UART console) should work with MicroPython (but not all were tested).

Features supported at this time:

* REPL (interactive prompt) over Zephyr UART console.
* `time` module for time measurements and delays.
* `machine.Pin` class for GPIO control, with IRQ support.
* `machine.I2C` class for I2C control.
* `machine.SPI` class for SPI control.
* `socket` module for networking (IPv4/IPv6).
* `zsensor` module for accessing the ZSensor-supported devices.
* "Frozen modules" support to allow to bundle Python modules together
  with firmware. Including complete applications, including with
  run-on-boot capability.
* virtual filesystem with FAT and littlefs formats, backed by either
  DiskAccess or FlashArea (flash map).

Over time, bindings for various Zephyr subsystems may be added.

## Building

### Getting started

If you are new to Zephyr, then we recommend following Zephyr's
[Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html)
to set up the required Zephyr tools (e.g. `west`), compiler toolchains, and environment.

Working with Zephyr requires that you set up a [workspace](https://docs.zephyrproject.org/latest/develop/west/basics.html)
(using `west init`), which is tied to a particular version of Zephyr. The getting
started documentation creates this workspace at `~/zephyrproject`. We will follow
this convention here, but you can call this anything you like.

A good way to verify that you are all set up for working with Zephyr is
to build and run (via QEMU) their [Hello World](https://docs.zephyrproject.org/latest/samples/hello_world/README.html)
sample, for both the `qemu_x86` and `qemu_cortex_m3` boards.

```
$ cd ~/zephyrproject
# Build for x86
$ west build -b qemu_x86 samples/hello_world
$ west build -t run
# Build for Cortex M3 (note the -p flag to do a "pristine build" when switching board).
$ west build -b qemu_cortex_m3 samples/hello_world -p
$ west build -t run
```

### Zephyr version

MicroPython currently supports the 3.4 release of Zephyr. You can create a new
workspace, at a particular version by passing the `--mr` flag to `west init`.
For example the following will create a new workspace at
`~/src/zephyrproject-v3.4`:

```
$ cd ~/src
$ west init zephyrproject-v3.4 -m https://github.com/zephyrproject-rtos/zephyr --mr v3.4.0
$ cd zephyrproject-v3.4
$ west update
```

If you already have a Zephyr workspace, you can switch it to a given Zephyr
version by checking out a particular tag from within the `zephyr`
subdirectory. For example:

```
$ cd ~/zephyrproject/zephyr
$ git fetch
$ git checkout v3.4.0
$ cd ..
$ west update
```

### Configuring the environment

To use your workspace from a different location (e.g. the MicroPython repo),
you can source the workspace's `zephyr-env.sh` script. For example

```
$ source ~/zephyrproject/zephyr/zephyr-env.sh
```

### Building MicroPython

Once your Zephyr workspace is ready, you can use it to build MicroPython just
like any other Zephyr application. For example, to build for the `frdm_k64f`
board:

```
$ cd path/to/micropython/ports/zephyr
$ west build -b frdm_k64f
```

Or to build for (x86) QEMU:

```
$ west build -b qemu_x86
```

Consult the Zephyr documentation linked above for the list of supported
boards. Board configuration files appearing in `ports/zephyr/boards/`
correspond to boards that have been tested with MicroPython and may have
additional options enabled, like filesystem support.

Build with configuration options
--------------------------------

You can add optional features to the build by using a configuration overlay file.
For example, to build for the nRF52840DK and enabling Bluetooth support:

    $ west build --board nrf52840dk_nrf52840 --DOVERLAY_CONFIG:STRING="overlay-bluetooth.conf"

Running
-------

To flash the resulting firmware to your board:

```
$ west flash
```

Or, to flash it to your board and start a gdb debug session:

```
$ west debug
```

To run the resulting firmware in QEMU (for boards like `qemu_x86`,
`qemu_cortex_m3`):

```
$ west build -t run
```

Networking is enabled with the default configuration, so you need to follow
the [Networking with QEMU](https://docs.zephyrproject.org/latest/connectivity/networking/qemu_setup.html)
instructions to set up TAP/SLIP networking. If you see any errors about
connecting to `unix:/tmp/slip.sock`, then there is an issue with this
configuration.

## Quick example

To blink an LED:

```py
import time
from machine import Pin

LED = Pin(("GPIO_1", 21), Pin.OUT)
while True:
    LED.value(1)
    time.sleep(0.5)
    LED.value(0)
    time.sleep(0.5)
```

The above code uses an LED location for a FRDM-K64F board (port B, pin 21;
following Zephyr conventions port are identified by "GPIO_x", where *x*
starts from 0). You will need to adjust it for another board (using board's
reference materials). To execute the above sample, copy it to clipboard, in
MicroPython REPL enter "paste mode" using Ctrl+E, paste clipboard, press
Ctrl+D to finish paste mode and start execution.



Build with the nRF Connect SDK
------------------------------

You might want to build MicroPython using the nRF Connect SDK for a few
reasons. For example, the nRF Connect SDK has a graphical installer that
makes it easier to install Zephyr and all the requirements to build it.
There is also the nRF Connect for VS Code Extension that turns VS Code
into an IDE for developing applications, including flashing and debugging.

Another reason is if you are trying to support a Nordic device that has
drivers which are not yet available in the upstream Zephyr tree, or a
development kit where the board definition files are not available. 

To install the nRF Connect SDK, follow the instructions here:
https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation/assistant.html 

* Open VS Code, and open the nRF Connect extension.
* Click on "Open an existing application" and navigate to <micropython install>/ports/zephyr
* Now the Zephyr port will be listed as an Application
* Add a build configuration
    * Select the board to build for
    * Add Kconfig fragments (such as board/<board file>, or overlay-bluetooth.conf)
    * Click on Build configuration

Now you can use the nRF Connect extension to flash to your board.

