cmake_minimum_required(VERSION 3.16)

# Include shared components from repository root
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# One lock file per target so concurrent S3 + C6 builds never rewrite each other's lock
# (a shared lock records the target and forces a re-solve on every target switch).
idf_build_set_property(DEPENDENCIES_LOCK "${CMAKE_CURRENT_LIST_DIR}/dependencies.lock.${IDF_TARGET}")

# Vendored components/esp_tinyusb is S3-only (USB-OTG). Exclude it elsewhere so its Kconfig
# does not leak into sdkconfig.c6.
if(NOT IDF_TARGET STREQUAL "esp32s3")
    list(APPEND EXCLUDE_COMPONENTS esp_tinyusb)
endif()

# liblc3_dual (components/liblc3_dual) is a fork of liblc3 with dual-rate encoding (lc3_encode_dual).
# Both export the same symbols, so the original liblc3 is kept intact but excluded from this app.
list(APPEND EXCLUDE_COMPONENTS liblc3)

add_compile_definitions(
    CFG_TUD_AUDIO=1
    CFG_TUD_AUDIO_ENABLE_EP_OUT=1
    CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP=1
    CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX=192
    CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ=7680
    CFG_TUD_AUDIO_TX_BUFSIZE=7680
    CFG_TUD_AUDIO_RX_BUFSIZE=7680
)

project(audio_VSAF)
