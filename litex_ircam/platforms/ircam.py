#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2021 Miodrag Milanovic <mmicko@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from litex.build.generic_platform import *
from litex.build.efinix.platform import EfinixPlatform
from litex.build.efinix import EfinixProgrammer

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk
    ("clk48", 0, Pins("E8"), IOStandard("3.3_V_LVTTL_/_LVCMOS")),
    # Serial
    (
        "serial",
        0,
        Subsignal("tx", Pins("M13")),  # TP1
        Subsignal("rx", Pins("M12")),  # TP2
        IOStandard("3.3_V_LVTTL_/_LVCMOS"),
        Misc("WEAK_PULLUP"),
    ),
    # Leds
    (
        "user_led",
        0,
        Pins("B8"),
        IOStandard("3.3_V_LVTTL_/_LVCMOS"),
        Misc("DRIVE_STRENGTH=3"),
    ),  # D2 "NSTATUS"
    # SPIFlash
    (
        "spiflash",
        0,
        Subsignal("cs_n", Pins("P3")),
        Subsignal("clk", Pins("M3")),
        Subsignal("mosi", Pins("L3")),
        Subsignal("miso", Pins("N1")),
        IOStandard("3.3_V_LVTTL_/_LVCMOS"),
    ),
    (
        "utmi",
        0,
        Subsignal("d_out", Pins("C16 B16 C10 B14 A14 C11 D12 C12")),  # D0..D7
        Subsignal("d_in", Pins("D11 B12 B11 A11 B10 A10 B9 A9")),  # D8..D15
        Subsignal("opmode", Pins("E11 E12")),
        Subsignal("txvalid", Pins("A15")),
        Subsignal("txready", Pins("A13")),
        Subsignal("rxactive", Pins("B13")),
        Subsignal("rxvalid", Pins("B15")),
        Subsignal("rxerror", Pins("C13")),
        Subsignal("validh", Pins("C14")),
        Subsignal("databus16_8", Pins("D13")),
        Subsignal("suspendn", Pins("D14")),
        Subsignal("reset", Pins("E13")),
        Subsignal("linestate", Pins("E14 F12")),
        Subsignal("xcvrselect", Pins("C15")),
        Subsignal("termselect", Pins("F13")),
        Subsignal("clkout", Pins("C9")),
        IOStandard("3.3_V_LVTTL_/_LVCMOS"),
    ),
]

# Bank voltage ---------------------------------------------------------------------------------------

_bank_info = [
    ("1A", "3.3 V LVTTL / LVCMOS"),
    ("1B_1C", "3.3 V LVTTL / LVCMOS"),
    ("1D_1E", "3.3 V LVTTL / LVCMOS"),
    ("3A_3B_3C", "3.3 V LVTTL / LVCMOS"),
    ("3D_3E", "3.3 V LVTTL / LVCMOS"),
    ("4A", "3.3 V LVTTL / LVCMOS"),
    ("4B", "3.3 V LVTTL / LVCMOS"),
    ("BR", "1.2 V"),
    ("TL", "1.2 V"),
    ("TR", "1.2 V"),
]

# Connectors ---------------------------------------------------------------------------------------

_connectors = []

# Platform -----------------------------------------------------------------------------------------


class Platform(EfinixPlatform):
    default_clk_name = "clk48"
    default_clk_period = 1e9 / 48e6

    def __init__(self, toolchain="efinity"):
        EfinixPlatform.__init__(
            self,
            "T13F256C4",
            _io,
            _connectors,
            iobank_info=_bank_info,
            toolchain=toolchain,
        )

    def create_programmer(self):
        return EfinixProgrammer()

    def do_finalize(self, fragment):
        EfinixPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk48", loose=True), 1e9 / 48e6)
        self.add_period_constraint(
            self.lookup_request("utmi:clkout", loose=True), 1e9 / 60e6
        )
