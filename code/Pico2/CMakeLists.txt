# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Pico2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
add_subdirectory(lib/BNO08x_Pico_Library)
# Add executable. Default name is the project name, version 0.1

add_executable(Pico2
    src/main.cpp
    src/motor.cpp
    src/servo.cpp
    src/encoder.cpp
    src/imu.cpp
    src/pi_i2c.cpp
)

# Add the standard include files to the build
target_include_directories(Pico2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# Add the standard library to the build
target_link_libraries(Pico2
    pico_stdlib
    hardware_pwm
    hardware_i2c
    BNO08x_Pico_Library
    pico_i2c_slave
)

pico_set_program_name(Pico2 "Pico2")
pico_set_program_version(Pico2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Pico2 0)
pico_enable_stdio_usb(Pico2 1)





pico_add_extra_outputs(Pico2)

