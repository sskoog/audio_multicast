cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32c6")

# Include shared components from repository root
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c6_ble_audio_broadcast)
