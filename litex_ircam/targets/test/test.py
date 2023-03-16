#!/usr/bin/env python3

from litex import RemoteClient

from peripherals.spi import *

bus = RemoteClient(host="djipi")
bus.open()

print(bus.regs)


class MCP2515:
    INSTRUCTION_WRITE = 0x02
    INSTRUCTION_READ = 0x03
    INSTRUCTION_BITMOD = 0x05
    INSTRUCTION_LOAD_TX0 = 0x40
    INSTRUCTION_LOAD_TX1 = 0x42
    INSTRUCTION_LOAD_TX2 = 0x44
    INSTRUCTION_RTS_TX0 = 0x81
    INSTRUCTION_RTS_TX1 = 0x82
    INSTRUCTION_RTS_TX2 = 0x84
    INSTRUCTION_RTS_ALL = 0x87
    INSTRUCTION_READ_RX0 = 0x90
    INSTRUCTION_READ_RX1 = 0x94
    INSTRUCTION_READ_STATUS = 0xA0
    INSTRUCTION_RX_STATUS = 0xB0
    INSTRUCTION_RESET = 0xC0

    def __init__(self, spi):
        self.spi = spi

    def reset(self):
        self.spi.transmit(SPI_CS_MCP2515, [self.INSTRUCTION_RESET])

    def read_reg(self, reg):
        res = self.spi.transmit(SPI_CS_MCP2515, [self.INSTRUCTION_READ, reg, 0])
        return res[2]


spi = SPIDriver(bus)

m = MCP2515(spi)
m.reset()
for i in range(0x10):
    print("%02x" % i, "%02x" % m.read_reg(i))


# bus.regs.can_spi_mosi.write(123)

# print("%02x" % bus.regs.can_spi_miso.read())


bus.close()
