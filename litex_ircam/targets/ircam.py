#!/usr/bin/env python3

#
# This file is based on LiteX-Boards.
#
# Copyright (c) 2023 Felix Domke <Felix@Dom.ke>
# Copyright (c) 2021 Miodrag Milanovic <mmicko@gmail.com>
# Copyright (c) 2021 Franck Jullien <franck.jullien@collshade.fr>
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex_boards.platforms import ircam

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream
from litex.soc.cores.led import LedChaser
from litex.soc.cores.uart import UARTInterface

# CRG ----------------------------------------------------------------------------------------------


class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys = ClockDomain()

        # # #

        clk48 = platform.request("clk48")
        # rst_n = platform.request("user_btn", 0)
        rst_n = True

        # PLL
        self.pll = pll = TRIONPLL(platform)
        self.comb += pll.reset.eq(~rst_n)
        pll.register_clkin(clk48, 48e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=True)


# BaseSoC ------------------------------------------------------------------------------------------


class UsbSerial(Module, UARTInterface):
    def __init__(self, pads):
        Module.__init__(self)
        UARTInterface.__init__(self)

        self.clock_domains.cd_utmi = ClockDomain()

        self.comb += [
            self.cd_utmi.clk.eq(~pads.clkout),
            self.cd_utmi.rst.eq(False),
        ]

        USBRST = Signal()
        HIGHSPEED = Signal()
        ONLINE = Signal()
        RXVAL = Signal()
        RXDAT = Signal(8)
        RXRDY = Signal()
        RXLEN = Signal()
        TXDAT = Signal(8)
        TXVAL = Signal()
        TXRDY = Signal()
        TXROOM = Signal()
        TXCORK = Signal()

        # source write clock domain is USB,
        depth = 8
        self.submodules.fifo_source = ClockDomainsRenamer(
            {"write": "utmi", "read": "sys"}
        )(stream.AsyncFIFO([("data", 8)], depth))
        self.submodules.fifo_sink = ClockDomainsRenamer(
            {"write": "sys", "read": "utmi"}
        )(stream.AsyncFIFO([("data", 8)], depth))

        self.comb += self.fifo_source.source.connect(self.source)
        self.comb += self.sink.connect(self.fifo_sink.sink)

        self.comb += [
            # SINK (FIFO) -> USB
            TXDAT.eq(self.fifo_sink.source.data),
            TXVAL.eq(self.fifo_sink.source.valid),
            self.fifo_sink.source.ready.eq(TXRDY),
            # USB -> SOURCE (FIFO)
            self.fifo_source.sink.data.eq(RXDAT),
            self.fifo_source.sink.valid.eq(RXVAL),
            RXRDY.eq(self.fifo_source.sink.ready),
            TXCORK.eq(False),
        ]

        PHY_DATAIN = Signal(8)
        PHY_DATAOUT = Signal(8)
        PHY_TXVALID = Signal()
        PHY_TXREADY = Signal()
        PHY_RXACTIVE = Signal()
        PHY_RXVALID = Signal()
        PHY_RXERROR = Signal()
        PHY_LINESTATE = Signal(2)
        PHY_OPMODE = Signal(2)
        PHY_XCVRSELECT = Signal()
        PHY_TERMSELECT = Signal()
        PHY_RESET = Signal()

        self.comb += [
            pads.d_out.eq(PHY_DATAOUT),
            PHY_DATAIN.eq(pads.d_in),
            pads.opmode.eq(PHY_OPMODE),
            pads.txvalid.eq(PHY_TXVALID),
            PHY_TXREADY.eq(pads.txready),
            PHY_RXACTIVE.eq(pads.rxactive),
            PHY_RXVALID.eq(pads.rxvalid),
            PHY_RXERROR.eq(pads.rxerror),
            pads.databus16_8.eq(0),  # 8 bit mode
            pads.suspendn.eq(1),
            pads.reset.eq(PHY_RESET),
            PHY_LINESTATE.eq(pads.linestate),
            pads.xcvrselect.eq(PHY_XCVRSELECT),
            pads.termselect.eq(PHY_TERMSELECT),
        ]

        self.specials += Instance(
            "usb_serial",
            p_VENDORID=0xAAAA,
            p_PRODUCTID=0xBBBB,
            p_VERSIONBCD=0xCCCC,
            p_VENDORSTR="Waldobjekt",
            p_PRODUCTSTR="IRCAM",
            p_HSSUPPORT=False,
            p_SELFPOWERED=True,
            p_RXBUFSIZE_BITS=11,
            p_TXBUFSIZE_BITS=10,
            i_CLK=self.cd_utmi.clk,
            i_RESET=self.cd_utmi.rst,
            o_USBRST=USBRST,
            o_HIGHSPEED=HIGHSPEED,
            o_ONLINE=ONLINE,
            o_RXVAL=RXVAL,
            o_RXDAT=RXDAT,
            i_RXRDY=RXRDY,
            o_RXLEN=RXLEN,
            i_TXVAL=TXVAL,
            i_TXDAT=TXDAT,
            o_TXRDY=TXRDY,
            o_TXROOM=TXROOM,
            i_TXCORK=TXCORK,
            i_PHY_DATAIN=PHY_DATAIN,
            o_PHY_DATAOUT=PHY_DATAOUT,
            o_PHY_TXVALID=PHY_TXVALID,
            i_PHY_TXREADY=PHY_TXREADY,
            i_PHY_RXACTIVE=PHY_RXACTIVE,
            i_PHY_RXVALID=PHY_RXVALID,
            i_PHY_RXERROR=PHY_RXERROR,
            i_PHY_LINESTATE=PHY_LINESTATE,
            o_PHY_OPMODE=PHY_OPMODE,
            o_PHY_XCVRSELECT=PHY_XCVRSELECT,
            o_PHY_TERMSELECT=PHY_TERMSELECT,
            o_PHY_RESET=PHY_RESET,
        )


class BaseSoC(SoCCore):
    def __init__(
        self,
        sys_clk_freq=100e6,
        with_spi_flash=False,
        with_led_chaser=True,
        with_uart=False,
        **kwargs
    ):
        platform = ircam.Platform()

        platform.add_sources(
            "./usb/fpga-usb-serial-20131205/",
            "usb_control.vhdl",
            "usb_init.vhdl",
            "usb_packet.vhdl",
            "usb_pkg.vhdl",
            "usb_serial.vhdl",
            "usb_transact.vhdl",
            library="default",
        )

        self.submodules.usb_serial = UsbSerial(pads=platform.request("utmi"))

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(
            self, platform, sys_clk_freq, ident="LiteX SoC on IRCAM Device", **kwargs
        )

        from litex.soc.cores.uart import UART

        self.uart = UART(self.usb_serial)

        # IRQ.
        if self.irq.enabled:
            self.irq.add("uart", use_loc_if_exists=True)
        else:
            self.add_constant("UART_POLLING")

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q32JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes

            self.add_spi_flash(
                mode="1x", module=W25Q32JV(Codes.READ_1_1_1), with_master=True
            )

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads=platform.request_all("user_led"), sys_clk_freq=sys_clk_freq
            )


# Build --------------------------------------------------------------------------------------------


def main():
    from litex.build.parser import LiteXArgumentParser

    parser = LiteXArgumentParser(
        platform=ircam.Platform, description="LiteX on IRCAM Device"
    )
    parser.add_target_argument("--flash", action="store_true", help="Flash bitstream.")
    parser.add_target_argument(
        "--sys-clk-freq", default=100e6, type=float, help="System clock frequency."
    )
    parser.add_target_argument(
        "--with-spi-flash", action="store_true", help="Enable SPI Flash (MMAPed)."
    )
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq=args.sys_clk_freq,
        with_spi_flash=args.with_spi_flash,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        from litex.build.openfpgaloader import OpenFPGALoader

        prog = OpenFPGALoader("trion_t120_bga576")
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=".hex"))  # FIXME


if __name__ == "__main__":
    main()
