This is a work-in-progress design of a Autoliv NV2 interface board.

Hardware is an Efinix T13 + DS90C124 Deserializer + MCP2515 CAN interface,
with the longer-term goal of eliminating both and replacing them with FPGA
logic.

USB communication is based on http://jorisvr.nl/article/usb-serial using an
SMSC USB3250 UTMI transceiver. 

To build, run ./ircam.py --build --soc-csv test/csr.csv in the
`litex_ircam/targets` directory.

By default, there is a litex console on TP1/TP2 (TX/RX), and the UART serial
port is used as a uartbone. With --no-uart, the UART pins can be disabled,
and the USB ACM port is used for the litex console (and no uartbone is
present).

Use `litex_server --uart --uart-port=/dev/ttyACM0 --bind-ip 0.0.0.0` to run
a litex server, and then run `test.py` (from within test/) to connect to it
and enumerate the MCP2515 registers.
