This is a work-in-progress design of a Autoliv NV2 interface board.

Hardware is an Efinix T13 + DS90C124 Deserializer + MCP2515 CAN interface,
with the longer-term goal of eliminating both and replacing them with FPGA
logic.

USB communication is based on http://jorisvr.nl/article/usb-serial using an 
SMSC USB3250 UTMI transceiver. 

To build, run ./ircam.py --build in the `litex_ircam/targets` directory.
