# MCP3x6xR Development

Development for ADC-facing code on the ADPC board. 

Note that symlinks to the driver code need to be instantiated; git does not do this.

A `script.py` file is included to automate the serial interfacing tasks (as opposed to using a serial monitor);
this script also decodes the raw data stream. It takes as an argument the path of the usb device; on MacOS this is
something like `/dev/cu.usbmodem####`.