set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
cmake_minimum_required(VERSION 3.20)

# https://github.com/openwch/risc-none-embed-gcc/releases/tag/8.2.0
set(PREFIX "riscv-none-embed-")

if (WIN32)
    MESSAGE(STATUS "Now is windows!")
    set(CMAKE_C_COMPILER ${PREFIX}gcc.exe)
    set(CMAKE_CXX_COMPILER ${PREFIX}g++.exe)
    set(CMAKE_ASM_COMPILER ${PREFIX}gcc.exe)
    set(CMAKE_AR ${PREFIX}ar.exe)
    set(CMAKE_OBJCOPY ${PREFIX}objcopy.exe)
    set(CMAKE_OBJDUMP ${PREFIX}objdump.exe)
    set(SIZE ${PREFIX}size.exe)
elseif (UNIX)
    MESSAGE(STATUS "Now is UNIX-like OS!")
    set(CMAKE_C_COMPILER ${PREFIX}gcc)
    set(CMAKE_CXX_COMPILER ${PREFIX}g++)
    set(CMAKE_ASM_COMPILER ${PREFIX}gcc)
    set(CMAKE_AR ${PREFIX}ar)
    set(CMAKE_OBJCOPY ${PREFIX}objcopy)
    set(CMAKE_OBJDUMP ${PREFIX}objdump)
    set(SIZE ${PREFIX}size)
else ()
    MESSAGE(STATUS "Unsupported system!")
endif ()

project(ch32-keyboard C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(
        -march=rv32imac
        -mabi=ilp32
        -mcmodel=medany
        -msmall-data-limit=8
        -mno-save-restore
)

add_compile_options(
        -fmessage-length=0
        -ffunction-sections
        -fdata-sections
        -fno-common
)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    message(STATUS "Debug mode")
    add_compile_options(-Og -g)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

add_compile_options(-Wall -Wextra -Wstack-usage=2048)

include_directories(
        Core
        Debug
        Peripheral/inc

        FreeRTOS/include
        FreeRTOS/portable/GCC/RISC-V
        FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions

        CherryUSB/common
        CherryUSB/core
        CherryUSB/class/hub
        CherryUSB/class/hid

        Bsp/usbd
        Bsp/keyboard
        Bsp/ws2812

        User/Inc
)

# add_definitions(-DDEBUG=1)

file(GLOB_RECURSE SOURCES
        "Core/core_riscv.c"
        "Debug/debug.c"
        "Peripheral/src/*.c"

        "FreeRTOS/*.c"
        "FreeRTOS/portable/GCC/RISC-V/*.c"
        "FreeRTOS/portable/MemMang/heap_4.c"
        "FreeRTOS/portable/GCC/RISC-V/portASM.S"

        "CherryUSB/core/*.c"
        "CherryUSB/class/hid/*.c"
        "CherryUSB/port/fsdev/usb_dc_fsdev.c"
        "CherryUSB/port/fsdev/ch32v20x_fsdev.c"

        "Startup/startup_ch32v20x_D8.S"

        "User/Src/*.c"
)

list(REMOVE_ITEM SOURCES ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/Common/mpu_wrappers.c)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Ld/Link.ld)

add_link_options(
        -march=rv32imac
        -mabi=ilp32
        -nostartfiles
        -Xlinker --gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map,${PROJECT_NAME}.map
        --specs=nano.specs
        --specs=nosys.specs
)

add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# target_link_libraries(${PROJECT_NAME}.elf printfloat)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(LST_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.lst)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND ${CMAKE_OBJDUMP} --all-headers --demangle --disassemble $<TARGET_FILE:${PROJECT_NAME}.elf> > ${LST_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)