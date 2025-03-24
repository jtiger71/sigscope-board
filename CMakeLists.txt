cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_STANDARD 11)
set(PICO_PLATFORM rp2040) #not needed when set in command line after cmake
set(PICO_COMPILER pico_arm_gcc) #not needed when set in command line after cmake
set(CMAKE_BUILD_TYPE Debug) #not needed when set in command line after cmake
set(PICO_BOARD "pico_w") #not needed when set in command line after cmake

include(pico_sdk_import.cmake)

# ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

project(sigscope)
pico_sdk_init()
add_executable(sigscope
    main.c
    ble/picow_bt_base.c
    ble/picow_bt_if.c
    adc_dma/adc_fft.c
    adc_dma/kiss_fftr.c
    adc_dma/kiss_fft.c
    )

target_link_libraries(sigscope
    pico_stdlib

    pico_btstack_ble
    pico_btstack_classic  # 필요없으므로 추후 삭제
    pico_btstack_cyw43
    pico_cyw43_arch_none

    hardware_adc
	hardware_dma
	hardware_spi
    )

pico_enable_stdio_usb(sigscope 1)
pico_enable_stdio_uart(sigscope 0)

target_include_directories(sigscope PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    )
pico_add_extra_outputs(sigscope)    


