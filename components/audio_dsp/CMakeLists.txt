if(ESP_PLATFORM)
    idf_component_register(
        SRCS
            "src/dsp_primitives.cpp"
            "src/subharmonic_pre_stage.cpp"
            "src/subharmonic_zc_flipper.cpp"
            "src/subharmonic_nccf_synth.cpp"
            "src/hilbert_quadrature_ms.cpp"
            "src/audio_dsp_c_api.cpp"
        INCLUDE_DIRS
            "include"
    )
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -std=gnu++20
        -O3
        -ffast-math
        -fno-exceptions
        -fno-rtti
    )
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(audio_dsp LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(AUDIO_DSP_SOURCES
    src/dsp_primitives.cpp
    src/subharmonic_pre_stage.cpp
    src/subharmonic_zc_flipper.cpp
    src/subharmonic_nccf_synth.cpp
    src/hilbert_quadrature_ms.cpp
    src/audio_dsp_c_api.cpp
)

add_library(audio_dsp_static STATIC ${AUDIO_DSP_SOURCES})
target_include_directories(audio_dsp_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(audio_dsp SHARED ${AUDIO_DSP_SOURCES})
target_include_directories(audio_dsp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(audio_dsp PROPERTIES OUTPUT_NAME "audio_dsp" PREFIX "lib")

if(MSVC)
    target_compile_options(audio_dsp_static PRIVATE /O2 /fp:fast /W4)
    target_compile_options(audio_dsp PRIVATE /O2 /fp:fast /W4)
else()
    target_compile_options(audio_dsp_static PRIVATE -O3 -ffast-math -Wall -Wextra)
    target_compile_options(audio_dsp PRIVATE -O3 -ffast-math -Wall -Wextra)
    if(WIN32)
        target_link_options(audio_dsp PRIVATE -static-libgcc -static-libstdc++ -static)
    endif()
endif()

add_executable(test_audio_dsp test/test_audio_dsp.cpp)
target_link_libraries(test_audio_dsp PRIVATE audio_dsp_static)
if(NOT MSVC)
    target_compile_options(test_audio_dsp PRIVATE -O3 -ffast-math -Wall -Wextra)
    if(WIN32)
        target_link_options(test_audio_dsp PRIVATE -static-libgcc -static-libstdc++ -static)
    endif()
endif()

