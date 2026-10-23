#pragma once

#include <cstdint>
#include <cinttypes>
#include <cstddef>
#include <string>
#include "esp_err.h"
#include "config.h"
#include "dsp_filter.hpp"
#include "i2s_dac.hpp"

namespace Bluetooth {

enum class ConnectionState {
    IDLE,
    SCANNING,
    CONNECTED_TO_PIXEL10,
    BIS_SYNCED,
    STREAMING,
    PAUSED,
    ERROR
};

enum class AseState : uint8_t {
    IDLE = 0x00,
    CODEC_CONFIGURED = 0x01,
    QOS_CONFIGURED = 0x02,
    ENABLING = 0x03,
    STREAMING = 0x04,
    DISABLING = 0x05,
    RELEASING = 0x06
};

/**
 * @brief Bluetooth SIG LE Audio Published Audio Capabilities (PACS / PAC Record).
 * Defines what audio formats, sample rates, frame durations, and bitrates this ESP32-C6 sink node accepts over BLE Audio.
 */
struct PacCapabilities {
    uint8_t  codec_id{0x06};                        /* 0x06 = Standard Bluetooth SIG LC3 Codec */
    uint16_t supported_sampling_freq_mask{0x0060};   /* 0x0040 (48 kHz) | 0x0020 (44.1 kHz) */
    uint8_t  supported_frame_durations_mask{0x03};   /* 0x01 (7.5 ms) | 0x02 (10 ms) */
    uint8_t  supported_channels_mask{0x03};          /* 0x01 (Mono) | 0x02 (Stereo) */
    uint16_t min_octets_per_frame{20};              /* Min octets/frame (~16 kbps) */
    uint16_t max_octets_per_frame{120};             /* Max octets/frame (~160 kbps) */
    uint8_t  max_codec_frames_per_sdu{1};
};

struct StreamInfo {
    std::string codec_name{"LC3 fixpt"};
    uint32_t sample_rate{AUDIO_SAMPLE_RATE_DEFAULT_HZ};
    uint32_t bitrate_kbps{AUDIO_BITRATE_DEFAULT_KBPS};
    uint8_t channels{AUDIO_CHANNELS_DEFAULT};
    uint8_t frame_duration_ms{AUDIO_FRAME_DURATION_MS};
    int8_t volume_percent{AUDIO_VOLUME_DEFAULT_PERCENT};
    int8_t rssi_dbm{-55};
};

/**
 * @brief Bluetooth 5.3 LE Audio / Auracast Receiver Controller Class.
 * 
 * Manages NimBLE LE Audio scanning, BIS stream synchronization, LC3 fixed-point decoding,
 * and passes PCM audio through the 80 Hz Low-Pass Filter to the MAX98357A I²S DAC.
 */
class BleAudioReceiver {
public:
    BleAudioReceiver(Audio::DspHighPassFilter& filter, Hardware::I2sDacDriver& dac);
    ~BleAudioReceiver();

    /**
     * @brief Initialize Bluetooth 5.3 NVS flash, NimBLE host stack, and LC3 decoder.
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief Start scanning for Google Pixel 10 BLE Audio / Auracast transmitter.
     */
    esp_err_t startScanning();

    /**
     * @brief Get current connection state.
     */
    ConnectionState getConnectionState() const { return m_state; }
    const char* getConnectionStateStr() const;

    /**
     * @brief Get current ASCS ASE state.
     */
    AseState getAseState() const { return m_ase_state; }
    const char* getAseStateStr() const;
    void setAseState(AseState state) { m_ase_state = state; }

    /**
     * @brief Get current streaming specs.
     */
    const StreamInfo& getStreamInfo() const { return m_stream_info; }

    /**
     * @brief Dynamic Sampling Frequency Change Handler.
     * Re-initializes the 80 Hz Low-Pass DSP filter coefficients, LC3 decoder, and MAX98357A I²S DAC clock rate.
     * @param new_sample_rate New sampling frequency in Hz (e.g. 48000, 44100, 32000, 16000)
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t updateSampleRate(uint32_t new_sample_rate);

    /**
     * @brief Get Published Audio Capabilities (PACS / PAC Record).
     */
    const PacCapabilities& getPacCapabilities() const { return m_pac_capabilities; }

    /**
     * @brief Set VCS volume setting (0 to 100 %).
     */
    void setVolumePercent(uint8_t vol_pct);

    /**
     * @brief Get current BASS Broadcast ID tuned into.
     */
    uint32_t getBroadcastId() const { return m_broadcast_id; }
    void setBroadcastId(uint32_t broadcast_id) { m_broadcast_id = broadcast_id; }

    /**
     * @brief Simulate audio frame receipt and processing loop (called in background task).
     */
    void processAudioFrame(const uint8_t* lc3_packet, size_t packet_len);

private:
    Audio::DspHighPassFilter& m_filter;
    Hardware::I2sDacDriver& m_dac;
    ConnectionState m_state{ConnectionState::IDLE};
    AseState m_ase_state{AseState::IDLE};
    PacCapabilities m_pac_capabilities;
    StreamInfo m_stream_info;
    uint32_t m_broadcast_id{0x415552}; // Default "AUR" Broadcast_ID (0x41, 0x55, 0x52)
    bool m_is_initialized{false};
    void* m_lc3_decoder{nullptr}; // Pointer to esp_lc3_decoder_handle_t

    // Buffer for LC3 decoded 48 kHz PCM audio (480 samples * 2 channels = 960 int16_t)
    int16_t m_pcm_buffer[PCM_BUFFER_LENGTH_SAMPLES]{0};

    esp_err_t initLc3Decoder();
    void cleanupLc3Decoder();
};

} // namespace Bluetooth
