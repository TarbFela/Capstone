cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(adpc_streaming)

set(CMAKE_C_STANDARD 11)
set(PICO_BOARD pico2)


# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(adpc_streaming
		adpc_data_streaming.c
	../src2/capstone_w25.c
		mcp3x6xR_driver/mcp3x6xR.c
		mcp3x6xR_driver/mcp_pio.c
		ada4255_driver/ada4255.c
		../ADPC_Dev/ADPC_ADC.c
)

pico_generate_pio_header(adpc_streaming ${CMAKE_CURRENT_LIST_DIR}/mcp3x6xR_driver/mcp_pio.pio)

target_include_directories(adpc_streaming
		PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/../
)


target_link_libraries(adpc_streaming
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

target_compile_definitions(adpc_streaming PRIVATE
		CFG_TUD_CDC_TX_BUFSIZE=512
		CFG_TUD_CDC_RX_BUFSIZE=512
)

pico_enable_stdio_usb(adpc_streaming 1)
pico_enable_stdio_uart(adpc_streaming 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(adpc_streaming)

