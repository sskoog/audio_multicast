cmake_minimum_required(VERSION 3.16)

# Specify Custom 8MB Partition Table with 2MB App Partitions
set(PARTITION_TABLE_CSV_PATH "${CMAKE_CURRENT_LIST_DIR}/partitions.csv")

# Include shared components from repository root
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp32c6_ble_audio_receiver)
