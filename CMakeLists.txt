cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME hh)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/OLED.c

    # VL53L0X 激光测距：工程封装层
    Core/Src/VL53L0X.c

    # 测距数据获取层（只给距离 + 模块启动状态）
    Core/Src/GetData.c

    # 电机驱动层（TB6612 + 编码器）
    Core/Src/Motor.c

    # VL53L0X 驱动层：ST 官方 API v1.0.4（原样移植，勿改）
    Drivers/VL53L0X/Src/vl53l0x_api.c
    Drivers/VL53L0X/Src/vl53l0x_api_core.c
    Drivers/VL53L0X/Src/vl53l0x_api_ranging.c
    Drivers/VL53L0X/Src/vl53l0x_api_calibration.c
    Drivers/VL53L0X/Src/vl53l0x_api_strings.c
    # 平台层：I2C 收发 + 延时（这两个是本工程自己写的 STM32 版）
    Drivers/VL53L0X/Src/vl53l0x_i2c_platform.c
    Drivers/VL53L0X/Src/vl53l0x_platform.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    # ST VL53L0X 官方 API 头文件所在的驱动层目录
    Drivers/VL53L0X/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
