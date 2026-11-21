cmake_minimum_required(VERSION 3.16)
project(visual_tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Release by default (single-config generators only)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

add_executable(rilevamento_imx500
    camera/detection_imx500.cpp
    camera/imx500_network.cpp
    camera/frame_logger.cpp
)

target_link_libraries(rilevamento_imx500 PRIVATE PkgConfig::LIBCAMERA)
target_compile_options(rilevamento_imx500 PRIVATE -Wall -Wextra)
