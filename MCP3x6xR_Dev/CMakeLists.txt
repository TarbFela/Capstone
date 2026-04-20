cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(mcp3x6xR_dev)

set(CMAKE_C_STANDARD 11)
set(PICO_BOARD pico2)


# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(mcp3x6xR_dev
		mcp3x6xR_test.c
	../src2/capstone_w25.c
		mcp3x6xR_driver/mcp3x6xR.c
		mcp3x6xR_driver/mcp_pio.c
		../ADPC_Dev/ADPC_ADC.c
)

pico_generate_pio_header(mcp3x6xR_dev ${CMAKE_CURRENT_LIST_DIR}/mcp3x6xR_driver/mcp_pio.pio)

target_include_directories(mcp3x6xR_dev
		PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/../
)


target_link_libraries(mcp3x6xR_dev
		pico_stdlib
		hardware_spi
		hardware_dma
		hardware_i2c
		hardware_pio
		hardware_gpio
		hardware_pwm
		hardware_flash
		pico_bootrom
)

target_compile_definitions(mcp3x6xR_dev PRIVATE
		CFG_TUD_CDC_TX_BUFSIZE=512
		CFG_TUD_CDC_RX_BUFSIZE=512
)

pico_enable_stdio_usb(mcp3x6xR_dev 1)
pico_enable_stdio_uart(mcp3x6xR_dev 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(mcp3x6xR_dev)

