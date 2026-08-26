cmake_minimum_required(VERSION 3.16)

project(RaspberryPi5Controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==========================================================
# OpenCV
# ==========================================================

find_package(OpenCV REQUIRED)

# ==========================================================
# RPLIDAR SDK
# ==========================================================

set(RPLIDAR_SDK_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/external/rplidar_sdk
)

set(RPLIDAR_SDK_LIBRARY
    ${RPLIDAR_SDK_DIR}/output/Linux/Release/libsl_lidar_sdk.a
)

# ==========================================================
# EXECUTABLE
# ==========================================================

add_executable(rpi5_controller
    src/main.cpp
    src/pico_i2c.cpp
    src/camera.cpp
    src/lidar.cpp
    src/navigation.cpp
    src/openchallenge.cpp
    src/obstaclechallenge.cpp
    src/parking.cpp
)

# ==========================================================
# INCLUDE DIRECTORIES
# ==========================================================

target_include_directories(rpi5_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    ${RPLIDAR_SDK_DIR}/sdk/include
    ${RPLIDAR_SDK_DIR}/sdk/src

    ${OpenCV_INCLUDE_DIRS}
)

# ==========================================================
# LIBRARIES
# ==========================================================

target_link_libraries(rpi5_controller PRIVATE
    ${RPLIDAR_SDK_LIBRARY}
    ${OpenCV_LIBS}
    pthread
)