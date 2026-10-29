set(srcs
    "console.cpp"
    "main.cpp"
    "config.c"
    "lc3_codec.cpp"
    "tone_generator.cpp"
    "i2s_audio.cpp"
    "status_led.cpp"
    "button.cpp"
    "vsaf_broadcast_engine.cpp"
    "vsaf_source_pipeline.cpp"
    "vsaf_time_sync.cpp"
    "diagnostics.cpp"
    # "lc3_benchmark.cpp" (Disabled)
)

set(requires
    stats
    audio_metering
    nvs_flash
    driver
    esp_driver_gpio
    esp_driver_i2s
    esp_driver_pcnt
    esp_driver_ledc
    esp_driver_uart
    esp_driver_tsens
    esp_driver_usb_serial_jtag
    esp_timer
    esp_hw_support
    esp-dsp
    led_strip
    esp_wifi
    esp_netif
    lwip
    espressif__esp_audio_codec
    liblc3_dual  # fork of components/liblc3 with lc3_encode_dual(); the original is excluded in ../CMakeLists.txt
    vfs
    esp_partition
    spi_flash
)

if(CONFIG_IDF_TARGET_ESP32S3)
    list(APPEND srcs "usb_audio.cpp")
endif()

# REQUIRES is evaluated before Kconfig (CONFIG_*) values exist, so gate it on the build target.
idf_build_get_property(target IDF_TARGET)
if(target STREQUAL "esp32s3")
    list(APPEND requires esp_tinyusb)  # vendored: ../components/esp_tinyusb
endif()

set(ldfragments)
if(CONFIG_IDF_TARGET_ESP32C6)
    set(ldfragments "linker.lf")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires}
    LDFRAGMENTS ${ldfragments}
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
