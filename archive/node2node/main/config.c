#include "config.h"

static const system_config_t g_system_config = {
    .node_role = CONFIG_ACTIVE_NODE_ROLE,
    .board_type = ACTIVE_BOARD_TYPE,
    .device_name = NODE_DEVICE_NAME,
    .sample_rate_hz = AUDIO_SAMPLE_RATE_HZ,
    .channels = AUDIO_CHANNELS_NUM,
    .bit_depth = AUDIO_BIT_DEPTH,
    .bitrate_kbps = AUDIO_BITRATE_KBPS
};

const system_config_t* get_system_config(void) {
    return &g_system_config;
}
