cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project).
include(../pico-sdk/external/pico_sdk_import.cmake)

# We also need PICO EXTRAS.
include(../pico-extras/external/pico_extras_import.cmake)

project(pico_playground C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK.
pico_sdk_init()

if (TARGET pico_scanvideo_dpi)
    add_executable(pico_lcd
            )

    target_sources(pico_lcd PRIVATE pico_lcd.c)

    target_compile_definitions(pico_lcd PRIVATE
        -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=500
        )

    # RGB332
    add_compile_definitions(PICO_SCANVIDEO_COLOR_PIN_COUNT=8)    # scanvideo_base.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_RSHIFT=5)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_GSHIFT=2)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_BSHIFT=0)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_RCOUNT=3)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_GCOUNT=3)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_BCOUNT=2)   # scanvideo.h

    add_compile_definitions(PICO_SCANVIDEO_ENABLE_CLOCK_PIN=1)
    add_compile_definitions(PICO_SCANVIDEO_ENABLE_DEN_PIN=1)
    add_compile_definitions(PICO_SCANVIDEO_SYNC_PIN_BASE=8)

    target_link_libraries(pico_lcd PRIVATE
            pico_multicore
            pico_stdlib
            pico_scanvideo_dpi
            hardware_pwm
            )

    # pico_enable_stdio_usb(pico_lcd 1)
    # pico_enable_stdio_uart(pico_lcd 0)

    pico_add_extra_outputs(pico_lcd)
endif ()
