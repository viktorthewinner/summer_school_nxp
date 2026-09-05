
# Copyright 2026 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.22.0)

cmake_path(APPEND conf_file_path ${CMAKE_CURRENT_LIST_DIR} ${board} prj.conf)
set(CONF_FILE ${conf_file_path} ${CONF_FILE})
include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(distributed_ecu LANGUAGES C CXX ASM PROJECT_BOARD_PORT_PATH ${board_root}/${board}/demo_apps/led_blinky)

include(${SdkRootDirPath}/CMakeLists.txt)


include(${board}/board_files.cmake)

# Bound every LPI2C wait.
#
# The SDK default is I2C_RETRY_TIMES = 0, which its own header documents as
# "keep waiting until the flag is assert/deassert" - an UNBOUNDED wait. The
# I2C bus here has no pull-ups on any board; they are two resistors in the
# loom. Leave them out, or drop one of the two wires, and SCL or SDA sits low
# - so LPI2C_MasterTransferBlocking() never returns and the whole vehicle
# control loop stops dead, silently, with no console output at all.
#
# A car must not be one loose wire away from a hang.
#
# The count is a flag-wait poll loop, not a transaction, so it only has to
# cover one byte time: 25 us at 400 kHz. This core runs at 12 MHz, so 10000
# iterations is several milliseconds - two orders of magnitude of margin over
# a healthy bus, and a bounded few ms rather than tens of ms when it is dead.
# Do not raise it: every failed transfer costs this much, and the drive loop
# pays it. led_blinky.c backs off the polling rate on top.
mcux_add_macro(
    CC "-DI2C_RETRY_TIMES=10000"
)

# Bound every LPUART wait, for the same reason and in the same spirit.
#
# UART_RETRY_TIMES defaults to 0 in fsl_lpuart.h, which that header defines as
# "keep waiting for the flag until it is assert/deassert" - an UNBOUNDED wait,
# exactly like the I2C one above. LPUART_WriteBlocking() spins on TDRE, and
# LINK_SendLine() reaches it from the 1 Hz heartbeat, the 20 Hz drive repeat
# and the horn. A transmitter that never raises that flag therefore stops the
# main loop dead - no drive commands, no telemetry, and no console input,
# because console_service() is in the same loop and never gets its turn. The
# symptom is a board that prints nothing and will not accept a keystroke,
# which is indistinguishable from a dead board.
#
# One character at 38400 is 260 us. The loop body is a load, a test and a
# decrement, so at this 12 MHz core an iteration is roughly a microsecond:
# 20000 is about 20 ms, some eighty character times of margin on a healthy
# link and a bounded few tens of milliseconds on a broken one. A healthy
# transmitter exits on the first pass and pays nothing.
mcux_add_macro(
    CC "-DUART_RETRY_TIMES=20000"
)

# Stack: 4 KB, not the linker script's default 2 KB.
#
# The deepest path here is console_service -> drive_try_command -> test_link,
# which is 608 bytes of declared frames before newlib's PRINTF and sscanf
# internals are counted - and those are hundreds of bytes each. 2 KB left almost no
# margin, and a stack that grows into the heap on this part faults with no
# clue as to why. There are 24 KB of SRAM and static data uses 5 KB, so the
# extra 2 KB costs nothing worth having.
target_link_options(${MCUX_SDK_PROJECT_NAME} PRIVATE "-Wl,--defsym=__stack_size__=0x1000")

mcux_add_source(
    SOURCES led_blinky.c
            link.c
            link.h
)

mcux_add_source(
    SOURCES led_blinky/peripherals.c
            led_blinky/peripherals.h
)
mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
mcux_add_config_mex_path( PATH ${board} )
mcux_add_include(INCLUDES led_blinky .)
