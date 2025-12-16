cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(capstone_pwm)

set(CMAKE_C_STANDARD 11)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(capstone_pwm
	main.c
	capstone_pwm.c
	capstone_dsp.c
	capstone_adc.c
)



target_include_directories(capstone_pwm
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/../
)

target_link_libraries(capstone_pwm
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

pico_enable_stdio_usb(capstone_pwm 1)
pico_enable_stdio_uart(capstone_pwm 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(capstone_pwm)

