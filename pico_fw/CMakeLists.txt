# Pico 2 컨트롤러 펌웨어 (Pico SDK 2.1+)
#   cmake -B build -DPICO_SDK_PATH=<pico-sdk> [-DFW_BOARD=pico2_w5500]
#   cmake --build build -j
# 핀/파라미터는 config/*.yaml -> tools/gen_config.py -> build/generated/fw_config.h
cmake_minimum_required(VERSION 3.13)

set(FW_BOARD "pico2_w5500" CACHE STRING "config/boards/<FW_BOARD>.yaml")
set(FW_CONFIG "${CMAKE_CURRENT_LIST_DIR}/config/controller.yaml" CACHE FILEPATH "펌웨어 설정 YAML")
set(FW_BOARD_YAML "${CMAKE_CURRENT_LIST_DIR}/config/boards/${FW_BOARD}.yaml")
set(FW_GEN_DIR "${CMAKE_BINARY_DIR}/generated")

find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/tools/gen_config.py
            --config ${FW_CONFIG} --board ${FW_BOARD_YAML} --out ${FW_GEN_DIR}
    RESULT_VARIABLE FW_GEN_RESULT)
if(NOT FW_GEN_RESULT EQUAL 0)
    message(FATAL_ERROR "gen_config.py 실패: YAML 설정을 확인하세요")
endif()
include(${FW_GEN_DIR}/board.cmake)   # PICO_BOARD, PICO_PLATFORM, FW_CONFIG_SOURCES
# YAML 이 바뀌면 cmake 를 다시 돌려 fw_config.h 를 갱신한다
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${FW_CONFIG_SOURCES}
             ${CMAKE_CURRENT_LIST_DIR}/tools/gen_config.py)
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)

include(pico_sdk_import.cmake)
project(pico_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(pico_controller
    src/main.c
    src/proto.c
    src/session.c
    src/w5500.c
    src/sts_bus.c
    src/actuators.c
    src/diag.c
)
target_include_directories(pico_controller PRIVATE src ${FW_GEN_DIR})
target_compile_options(pico_controller PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
target_link_libraries(pico_controller
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_uart
    hardware_pwm
    hardware_sync
    hardware_watchdog
    hardware_clocks
)
# 디버그 로그는 USB CDC 로만 (UART 는 서보 버스 전용)
pico_enable_stdio_usb(pico_controller 1)
pico_enable_stdio_uart(pico_controller 0)
pico_add_extra_outputs(pico_controller)
