cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-MQTT-TEMPLATE VERSION "0.0.1")

# Auto-generate version header from CMakeLists.txt VERSION
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/project_version.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/project_version.h"
)
