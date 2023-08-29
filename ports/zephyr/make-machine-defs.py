#!/usr/bin/env python

# This file is part of the MicroPython project, http://micropython.org/
#
# The MIT License (MIT)
#
# Copyright (c) 2023 Jim Mussared
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
import pickle
import re
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../tools"))
import boardgen

# The Zephyr build (cmake/modules/dts.cmake) will write a serialized (pickle)
# instance of the edtlib.EDT object to build/zephyr/edt.pickle. To load this,
# we need to add add Zephyr's edtlib to sys.path.
zephyr_base = os.environ["ZEPHYR_BASE"]
sys.path.insert(0, os.path.join(zephyr_base, "scripts", "dts", "python-devicetree", "src"))


# Zephyr's devicetree does not track the names of the connector pins, they are
# only entered as comments into the .dts/.yaml files.
# This maps the "compatible" field for a given header to the known pin names.
CONNECTOR_PIN_NAMES = {
    "arduino-header-r3": [
        "A0",
        "A1",
        "A2",
        "A3",
        "A4",
        "A5",
        "D0",
        "D1",
        "D2",
        "D3",
        "D4",
        "D5",
        "D6",
        "D7",
        "D8",
        "D9",
        "D10",
        "D11",
        "D12",
        "D13",
        "D14",
        "D15",
    ],
    "microbit,edge-connector": [
        "P0",
        "P1",
        "P2",
        "P3",
        "P4",
        "P5",
        "P6",
        "P7",
        "P8",
        "P9",
        "P10",
        "P11",
        "P12",
        "P13",
        "P14",
        "P15",
        "P16",
        None,
        None,
        "P19",
        "P20",
    ],
    "adafruit-feather-header": [
        "A0",
        "A1",
        "A2",
        "A3",
        "A4",
        "A5",
        "SCK",
        "MOSI",
        "MISO",
        "RX",
        "TX",
        "D4",
        "SDA",
        "SCL",
        "D5",
        "D6",
        "D9",
        "D10",
        "D11",
        "D12",
        "D13",
    ],
}

# For known connectors, we don't include the node label in the board name.
CONNECTOR_PREFIX = {
    "arduino-header-r3": "",
    "microbit,edge-connector": "",
    "adafruit-feather-header": "",
}


class ZephyrPin(boardgen.Pin):
    def definition(self):
        # Pin(p_name, p_port_label, p_pin)
        return "Pin({:s}, {:s}, {:d})".format(self.name(), self._port, self._pin)


class ZephyrPinGenerator(boardgen.PinGenerator):
    def __init__(self):
        # Use custom pin type above.
        super().__init__(pin_type=ZephyrPin)

    # Add the --edt-pickle argument.
    def extra_args(self, parser):
        parser.add_argument("--edt-pickle")

    # Load the EDT after the standard inputs.
    def load_inputs(self, out_source):
        super().load_inputs(out_source)

        print("// --edt-pickle {:s}".format(self.args.edt_pickle), file=out_source)
        print(file=out_source)

        with open(self.args.edt_pickle, "rb") as f:
            edt = pickle.load(f)
        self.load_edt(edt)

    def get_connector_prefix(self, node):
        for compat in node.compats:
            if compat in CONNECTOR_PREFIX:
                return CONNECTOR_PREFIX[compat]
        return node.labels[0]

    def get_connector_pin_name(self, node, connector_index):
        for compat in node.compats:
            if compat in CONNECTOR_PIN_NAMES:
                return CONNECTOR_PIN_NAMES[compat][connector_index]
        return str(connector_index)

    def load_edt(self, edt):
        for node in edt.nodes:
            # Only process enabled nodes.
            if node.status != "okay":
                continue

            # defined in gpio-controller.yaml -- presence of this property
            # "indicates that this node is a GPIO controller"
            if "gpio-controller" in node.props:
                for i in range(node.props["ngpios"].val):
                    pin = self.add_cpu_pin("{:s}_{:d}".format(node.labels[0], i))
                    pin._port = node.labels[0]
                    pin._pin = i

            if node.name == "connector":
                dt = node._node.props["gpio-map"]
                for i in range(0, len(dt._markers), 2):
                    cn, _, _, pin, _ = struct.unpack(">IIIII", dt.value[i * 10 : i * 10 + 20])
                    ref = dt._markers[i + 1][2]
                    cpu_pin = self.find_pin_by_cpu_pin_name("{}_{}".format(ref, pin), create=False)
                    board_name = self.get_connector_prefix(node)
                    if board_name:
                        board_name += "_"
                    board_name += self.get_connector_pin_name(node, cn)
                    cpu_pin.add_board_pin_name(board_name, False)


if __name__ == "__main__":
    ZephyrPinGenerator().main()
