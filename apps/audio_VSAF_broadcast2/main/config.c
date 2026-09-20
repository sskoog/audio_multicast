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
    .amp_mute_gpio = -1,
    .status_led_gpio = 21,
    .status_led_num = 0,
    .user_button_gpio = 0,
    .has_display = false,
    .default_channel = 1,
    .max98357a_gain_db = 3,
    .is_pcm5102a = false
};
#else
static system_config_t s_active_config = {
    .node_id = 23,
    .node_role = NODE_ROLE_SINK,
    .device_name = "ESP32-C6-SINK",
    .i2s_bclk_gpio = 2,
    .i2s_ws_gpio = 3,
    .i2s_dout_gpio = 1,
    .amp_mute_gpio = -1,
    .status_led_gpio = 8,
    .status_led_num = 1,
    .user_button_gpio = 9,
    .has_display = false,
    .default_channel = 1,
    .max98357a_gain_db = 3,
    .is_pcm5102a = false
};
#endif

static bool s_config_initialized = false;

const system_config_t* get_system_config(void) {
    if (!s_config_initialized) {
        uint8_t mac[6] = {0};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (mac[5] == 0x3C && mac[4] == 0xE8 && mac[3] == 0xFB) {
            // Node 4: Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B + PCM5102A DAC + TPA3118 Amp
            s_active_config.node_id = 4;
            s_active_config.node_role = NODE_ROLE_SINK;
            s_active_config.device_name = "ESP32-S3-04-SURR";
            s_active_config.i2s_bclk_gpio = 6;  // D5 of XIAO
            s_active_config.i2s_ws_gpio = 4;    // D3 of XIAO
            s_active_config.i2s_dout_gpio = 5;  // D4 of XIAO
            s_active_config.amp_mute_gpio = 3;  // D2 of XIAO (Active LOW Mute / High-Z Unmute)
            s_active_config.status_led_gpio = 21; // Active LOW user LED
            s_active_config.status_led_num = 0;
            s_active_config.user_button_gpio = 0; // BOOT button
            s_active_config.has_display = false;
            s_active_config.default_channel = 4; // Surround Right (Ch 4)
            s_active_config.max98357a_gain_db = -1;
            s_active_config.is_pcm5102a = true;
        } else {
            // Node 16: Seeed Studio XIAO ESP32-S3 (Audio SOURCE)
            s_active_config.node_id = 16;
            s_active_config.node_role = NODE_ROLE_SOURCE;
            s_active_config.device_name = "ESP32-S3-SOURCE";
            s_active_config.i2s_bclk_gpio = -1;
            s_active_config.i2s_ws_gpio = -1;
            s_active_config.i2s_dout_gpio = -1;
            s_active_config.amp_mute_gpio = -1;
            s_active_config.status_led_gpio = 21;
            s_active_config.status_led_num = 0;
            s_active_config.user_button_gpio = 0;
            s_active_config.has_display = false;
            s_active_config.default_channel = 1;
            s_active_config.max98357a_gain_db = 3;
            s_active_config.is_pcm5102a = false;
        }
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
        if (mac[5] == 0x44 && mac[4] == 0x38) {
            s_active_config.node_id = 23;
            s_active_config.device_name = "ESP32-C6-23-LEFT";
            s_active_config.default_channel = 0;
        } else if (mac[5] == 0xE4 && mac[4] == 0x18) {
            s_active_config.node_id = 24;
            s_active_config.device_name = "ESP32-C6-24-RIGHT";
            s_active_config.default_channel = 1;
        } else if (mac[5] == 0xEC && mac[4] == 0x57) {
            s_active_config.node_id = 21;
            s_active_config.device_name = "ESP32-C6-21-CNTR";
            s_active_config.default_channel = 2;
        } else if (mac[5] == 0xC4 && mac[4] == 0xDC) {
            s_active_config.node_id = 25;
            s_active_config.device_name = "ESP32-C6-25-LSUR";
            s_active_config.default_channel = 3;
        } else if (mac[5] == 0x38 && mac[4] == 0x13) {
            s_active_config.node_id = 26;
            s_active_config.device_name = "ESP32-C6-26-RSUR";
            s_active_config.default_channel = 4;
        } else if (mac[5] == 0x24 && mac[4] == 0xDC) {
            s_active_config.node_id = 20;
            s_active_config.device_name = "ESP32-C6-20-SUB";
            s_active_config.default_channel = 5;
        }
#endif
        s_config_initialized = true;
    }
    return &s_active_config;
}

void set_node_role(uint8_t new_role) {
    s_active_config.node_role = new_role;
}
