cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(adpc)

set(CMAKE_C_STANDARD 11)
set(PICO_BOARD pico2)


# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(adpc
		adpc_main.c
		mcp3x6xR_driver/mcp3x6xR.c
		mcp3x6xR_driver/mcp_pio.c
		ADPC_ADC.c
		adpc_dsp.c
		adpc_core1.c
)

pico_generate_pio_header(adpc ${CMAKE_CURRENT_LIST_DIR}/mcp3x6xR_driver/mcp_pio.pio)

target_include_directories(adpc
		PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/../
)


target_link_libraries(adpc
		pico_stdlib
		hardware_spi
		hardware_dma
		hardware_i2c
		hardware_pio
		hardware_gpio
		hardware_pwm
		pico_bootrom
		pico_multicore
)

pico_enable_stdio_usb(adpc 1)
pico_enable_stdio_uart(adpc 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(adpc)

