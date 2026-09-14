cmake_minimum_required(VERSION 3.16)

# Include shared components from repository root
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_definitions(
    CFG_TUD_AUDIO=1
    CFG_TUD_AUDIO_ENABLE_EP_OUT=1
    CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX=192
    CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ=192
    CFG_TUD_AUDIO_TX_BUFSIZE=192
    CFG_TUD_AUDIO_RX_BUFSIZE=192
)

project(audio_VSAF_broadcast2)
