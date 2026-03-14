# ADA4255 Driver for RP2040/RP2350 [WIP]

This is a simple SPI driver for the ADA4255 programmable-gain instrumentation amplifier (PGIA) that I am using as a thermocouple front-end for my thesis project.

## ADA4255 Command Structure

Refer to the ADA4255 datasheet for any further details. The command interface is very simple; there are two bytes (a third if you are use CRC; I don't wanna) where the first contains:

1. A read/write bit
2. A register address

And the second bit contains the data, streamed either of MOSI or MISO depending on the R/W bit provided in the previous byte.

This driver provides macros for all registers and fields (WIP: not actually done yet!). To figure out what they actually do, see the datasheet. 
