#include "config.h"

static system_config_t s_active_config = {
    .node_id = 24,
    .node_role = NODE_ROLE_SINK,
    .device_name = "ESP32-C6-24",
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

const system_config_t* get_system_config(void) {
    return &s_active_config;
}

void set_node_role(uint8_t new_role) {
    s_active_config.node_role = new_role;
}
