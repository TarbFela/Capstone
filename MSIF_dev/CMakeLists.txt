cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(msif_dev)

set(CMAKE_C_STANDARD 11)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(msif_dev
	main.c
	msif_gpio.c
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
		pico_bootrom
		pico_multicore
)

pico_enable_stdio_usb(msif_dev 1)
pico_enable_stdio_uart(msif_dev 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(msif_dev)

