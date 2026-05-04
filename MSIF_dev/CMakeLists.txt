cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(msif_dev)

set(CMAKE_C_STANDARD 11)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of  project
add_executable(msif_dev
	main.c
	msif_gpio.c
	msif_analog.c
	msif_adc.c
	msif_peak.c
	msif_qms.c
	msif_proto.c
	../src2/dac80504_driver/dac80504.c
	../src2/mcp3x6xR_driver/mcp3x6xR.c
)

target_include_directories(msif_dev
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/../
)

target_link_libraries(msif_dev
		pico_stdlib
		hardware_spi
		hardware_dma
		hardware_adc
		hardware_i2c
		hardware_pwm
		hardware_gpio
		hardware_timer
		hardware_uart
		pico_bootrom
		pico_multicore
)

# Remap the SDK's default UART (used by stdio_uart) to GP12/GP13 per the
# MSIF MCU schematic (MSIF_UART_TX/RX nets to ISO6721 isolator). Default
# is GP0/GP1 which on this board are unused but reserved-feeling — the
# explicit override matches MSIF_cfg.h and keeps a single source of truth.
target_compile_definitions(msif_dev PRIVATE
		PICO_DEFAULT_UART=0
		PICO_DEFAULT_UART_TX_PIN=12
		PICO_DEFAULT_UART_RX_PIN=13
		PICO_DEFAULT_UART_BAUD_RATE=115200
)

# Enable BOTH stdio backends. printf is mirrored to USB-CDC and UART;
# stdio_getchar reads from whichever has data first. msif_proto.c's
# line-mode parser then accepts ':CMD ...' lines from either channel.
pico_enable_stdio_usb(msif_dev 1)
pico_enable_stdio_uart(msif_dev 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(msif_dev)

