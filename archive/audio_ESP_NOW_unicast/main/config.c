#include "config.h"
#include "esp_mac.h"
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
static system_config_t s_active_config = {
    .node_id = 16,
    .node_role = NODE_ROLE_SOURCE,
    .device_name = "ESP32-S3-SOURCE",
    .i2s_bclk_gpio = -1,
    .i2s_ws_gpio = -1,
    .i2s_dout_gpio = -1,
    .status_led_gpio = 21,
    .status_led_num = 0,
    .user_button_gpio = 0,
    .has_display = false,
    .default_channel = 1,
    .max98357a_gain_db = 3
};
#else
static system_config_t s_active_config = {
    .node_id = 23,
    .node_role = NODE_ROLE_SINK,
    .device_name = "ESP32-C6-SINK",
    .i2s_bclk_gpio = 2,
    .i2s_ws_gpio = 3,
    .i2s_dout_gpio = 1,
    .status_led_gpio = 8,
    .status_led_num = 1,
    .user_button_gpio = 9,
    .has_display = false,
    .default_channel = 1,
    .max98357a_gain_db = 3
};
#endif

static bool s_config_initialized = false;

const system_config_t* get_system_config(void) {
    if (!s_config_initialized) {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
        uint8_t mac[6] = {0};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        if (mac[5] == 0x44 || mac[4] == 0x38) {
            s_active_config.node_id = 23;
            s_active_config.device_name = "ESP32-C6-23-LEFT";
        } else if (mac[5] == 0xE4 || mac[4] == 0x18) {
            s_active_config.node_id = 24;
            s_active_config.device_name = "ESP32-C6-24-SUB";
        }
#endif
        s_config_initialized = true;
    }
    return &s_active_config;
}

void set_node_role(uint8_t new_role) {
    s_active_config.node_role = new_role;
}
