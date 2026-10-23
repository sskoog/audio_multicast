#include "config.h"

static const system_config_t g_system_config = {
    .sample_rate_hz     = AUDIO_SAMPLE_RATE_DEFAULT_HZ,
    .pcm_buffer_samples = PCM_BUFFER_LENGTH_SAMPLES,
    .hp_cutoff_freq_hz  = HP_FILTER_CUTOFF_FREQ_HZ,
    .bclk_gpio          = I2S_DAC_BCLK_PIN,
    .ws_gpio            = I2S_DAC_WS_PIN,
    .dout_gpio          = I2S_DAC_DOUT_PIN
};

const system_config_t* get_system_config(void) {
    return &g_system_config;
}
