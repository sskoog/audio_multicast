#pragma once

#include "lc3_codec.hpp"
#include "tone_generator.hpp"
#include "i2s_audio.hpp"
#include "config.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <string>
#include <cmath>
#include <algorithm>
#include <array>

#ifndef CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES
#ifdef ESPNOW_PREFILL_THRESHOLD_FRAMES_DEFAULT
#define CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES ESPNOW_PREFILL_THRESHOLD_FRAMES_DEFAULT
#else
#define CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES 5
#endif
#endif

#ifndef CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES
#ifdef ESPNOW_WATCHDOG_TIMEOUT_FRAMES_DEFAULT
#define CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES ESPNOW_WATCHDOG_TIMEOUT_FRAMES_DEFAULT
#else
#define CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES 5
#endif
#endif

namespace AudioNet {

enum class Lc3SampleRateCode : uint8_t {
    SR_8000  = 0,
    SR_16000 = 1,
    SR_24000 = 2,
    SR_32000 = 3,
    SR_48000 = 4,
    SR_96000 = 5,
};

static inline uint8_t sampleRateToCode(uint32_t hz) {
    switch (hz) {
        case 8000:  return 0;
        case 16000: return 1;
        case 24000: return 2;
        case 32000: return 3;
        case 48000: return 4;
        case 96000: return 5;
        default:    return 3; // Default 32 kHz
    }
}

static inline uint32_t codeToSampleRate(uint8_t code) {
    switch (code & 0x07) {
        case 0: return 8000;
        case 1: return 16000;
        case 2: return 24000;
        case 3: return 32000;
        case 4: return 48000;
        case 5: return 96000;
        default: return 32000;
    }
}

static constexpr uint16_t VSAF_DEFAULT_MAGIC = 0x1337;
static constexpr size_t VSAF_HEADER_LEN = 8;

// 8-Byte Word-Aligned VSAF Header
struct EspNowAudioHeader {
    uint16_t magic;          // 0x1337 (Offsets 0..1, 16-bit aligned)
    uint8_t  seq;            // Sequence counter 0..255 (Offset 2)
    uint8_t  cfg;            // [0..2: ch_id 0..7] [3..5: sr_code] [6: 0=10ms/1=7.5ms] [7: sync] (Offset 3)
    uint32_t pts_us;         // 32-bit Microsecond Presentation Timestamp (Offsets 4..7, 32-bit aligned)
} __attribute__((packed));

// 248-Byte Word-Aligned Dual-Frame Audio Packet
struct EspNowAudioPacket {
    uint16_t magic;          // 0x1337 (Offsets 0..1, 16-bit aligned)
    uint8_t  seq;            // Sequence counter 0..255 (Offset 2)
    uint8_t  cfg;            // [0..2: ch_id 0..7] [3..5: sr_code] [6: 0=10ms/1=7.5ms] [7: sync] (Offset 3)
    uint32_t pts_us;         // 32-bit Microsecond Presentation Timestamp (Offsets 4..7, 32-bit aligned)
    uint8_t  curr_frame[120];// Primary Frame N   (Offsets 8..127, 32-bit aligned)
    uint8_t  prev_frame[120];// Redundant Frame N-1 (Offsets 128..247, 32-bit aligned)
} __attribute__((packed));

enum class NetworkState {
    OFF,           // WiFi disabled
    IDLE,          // WiFi on, passive, sound off/muted
    SCANNING,      // WiFi on, actively searching for audio stream
    PREFILL,       // Found audio stream, decoding the first LC3-frames
    STREAMING,     // Locked to WiFi audio stream, I2S clocks running, continously decoding LC3
    BROADCASTING   // Transmitting audio stream over ESP-NOW (SOURCE node)
};

struct StreamTelemetry {
    uint32_t sample_rate = CONFIG_ESPNOW_SAMPLE_RATE_HZ;
    uint32_t frame_duration_us = 10000;
    uint8_t  bit_depth = 16;
    uint8_t  channels = 1;
    uint32_t bitrate_kbps = (CONFIG_ESPNOW_FRAME_LEN_OCTETS * 8) / 10;
    int8_t   rssi_dbm = -26;
    bool     is_muted = false;
    uint8_t  volume_percent = 5; // 5% whisper-quiet volume

    std::string getStatusString() const {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s %u-bit %.1f kHz",
                 (channels == 1) ? "Mono" : "Stereo", bit_depth, sample_rate / 1000.0f);
        return std::string(buf);
    }

    std::string getCodecString() const {
        char buf[64];
        snprintf(buf, sizeof(buf), "LC3 %lu kbps (%.1fms)", (unsigned long)bitrate_kbps, frame_duration_us / 1000.0f);
        return std::string(buf);
    }
};

class AudioLevelMeter {
public:
    AudioLevelMeter() : m_rms_dbfs(-INFINITY), m_peak_dbfs(-INFINITY), m_rms_int16(0), m_peak_int16(0) {}

    void pushFramePcm(const int16_t* samples, size_t count) {
        if (!samples || count == 0) {
            pushSilence();
            return;
        }

        int32_t peak = 0;
        int64_t sum_sq = 0;

        for (size_t i = 0; i < count; ++i) {
            int16_t s = samples[i];
            int32_t abs_s = std::abs(static_cast<int32_t>(s));
            if (abs_s > peak) peak = abs_s;
            sum_sq += static_cast<int64_t>(s) * s;
        }

        float mean_sq = static_cast<float>(sum_sq) / static_cast<float>(count);
        float rms_linear = std::sqrt(mean_sq);

        m_rms_int16 = static_cast<int16_t>(rms_linear);
        m_peak_int16 = static_cast<int16_t>(peak);

        float rms_norm = rms_linear / 32768.0f;
        m_rms_dbfs = (rms_norm > 0.00001f) ? 20.0f * std::log10(rms_norm) : -100.0f;

        float peak_norm = static_cast<float>(peak) / 32768.0f;
        m_peak_dbfs = (peak_norm > 0.00001f) ? 20.0f * std::log10(peak_norm) : -100.0f;
    }

    void pushSilence() {
        m_rms_dbfs = -INFINITY;
        m_peak_dbfs = -INFINITY;
        m_rms_int16 = 0;
        m_peak_int16 = 0;
    }

    float getAudioFrameRMS_dBFS() const { return m_rms_dbfs; }
    float getAudioFramePeak_dBFS() const { return m_peak_dbfs; }
    int16_t getAudioFrameRMS_int16() const { return m_rms_int16; }
    int16_t getAudioFramePeak_int16() const { return m_peak_int16; }

private:
    float m_rms_dbfs;
    float m_peak_dbfs;
    int16_t m_rms_int16;
    int16_t m_peak_int16;
};

template<typename T, size_t Capacity = 10>
class SpscDurationRingBuffer {
public:
    void push(T value) {
        size_t head = m_head.load(std::memory_order_relaxed);
        m_buffer[head] = value;
        m_head.store((head + 1) % Capacity, std::memory_order_release);
        size_t count = m_count.load(std::memory_order_relaxed);
        if (count < Capacity) {
            m_count.store(count + 1, std::memory_order_relaxed);
        }
    }

    void getStats(float& out_avg_ms, float& out_peak_ms, bool& out_has_data) const {
        size_t count = m_count.load(std::memory_order_acquire);
        if (count == 0) {
            out_avg_ms = 0.0f;
            out_peak_ms = 0.0f;
            out_has_data = false;
            return;
        }

        uint32_t peak_us = 0;
        uint64_t sum_us = 0;
        for (size_t i = 0; i < count; ++i) {
            uint32_t val = m_buffer[i];
            if (val > peak_us) peak_us = val;
            sum_us += val;
        }

        out_avg_ms = (static_cast<float>(sum_us) / static_cast<float>(count)) / 1000.0f;
        out_peak_ms = static_cast<float>(peak_us) / 1000.0f;
        out_has_data = true;
    }

    void reset() {
        m_head.store(0, std::memory_order_relaxed);
        m_count.store(0, std::memory_order_relaxed);
    }

private:
    T m_buffer[Capacity] = {0};
    std::atomic<size_t> m_head{0};
    std::atomic<size_t> m_count{0};
};


class TimeOffsetRingBuffer {
public:
    static constexpr size_t CAPACITY = 64;

    TimeOffsetRingBuffer() { reset(); }

    void push(int64_t offset_us) {
        size_t head = m_head.load(std::memory_order_relaxed);
        m_buffer[head] = offset_us;
        m_head.store((head + 1) % CAPACITY, std::memory_order_release);
        size_t count = m_count.load(std::memory_order_relaxed);
        if (count < CAPACITY) {
            m_count.store(count + 1, std::memory_order_relaxed);
        }
    }

    void reset() {
        m_head.store(0, std::memory_order_relaxed);
        m_count.store(0, std::memory_order_relaxed);
    }

    size_t getCount() const {
        return m_count.load(std::memory_order_acquire);
    }

    bool computeStats(float& out_median_ms, float& out_range_ms) const {
        size_t count = m_count.load(std::memory_order_acquire);
        if (count == 0) {
            out_median_ms = 0.0f;
            out_range_ms = 0.0f;
            return false;
        }

        int64_t temp[CAPACITY];
        size_t head = m_head.load(std::memory_order_relaxed);
        for (size_t i = 0; i < count; ++i) {
            size_t idx = (head + CAPACITY - count + i) % CAPACITY;
            temp[i] = m_buffer[idx];
        }

        int64_t min_val = temp[0];
        int64_t max_val = temp[0];
        for (size_t i = 1; i < count; ++i) {
            if (temp[i] < min_val) min_val = temp[i];
            if (temp[i] > max_val) max_val = temp[i];
        }
        out_range_ms = static_cast<float>(max_val - min_val) / 1000.0f;

        std::sort(temp, temp + count);
        int64_t median_val = (count % 2 == 1) ? temp[count / 2] : ((temp[count / 2 - 1] + temp[count / 2]) / 2);
        out_median_ms = static_cast<float>(median_val) / 1000.0f;

        return true;
    }

private:
    std::array<int64_t, CAPACITY> m_buffer{};
    std::atomic<size_t> m_head{0};
    std::atomic<size_t> m_count{0};
};

class EspNowAudioBroadcast {
public:
    EspNowAudioBroadcast(Codec::Lc3CodecEngine& lc3_codec,
                         Audio::ToneGenerator* tone_gen = nullptr,
                         Hardware::I2sAudioDriver* i2s_dac = nullptr);
    ~EspNowAudioBroadcast();

    esp_err_t init(uint8_t node_role);
    esp_err_t enableWifiEspNow();
    esp_err_t startAudioTask();

    // On-the-fly audio stream reconfiguration (SOURCE node)
    esp_err_t setAudioConfig(uint32_t sample_rate_hz, uint16_t frame_len_octets, uint32_t frame_duration_us = 0);
    esp_err_t setSampleRate(uint32_t sample_rate_hz);
    esp_err_t setBitDepth(uint8_t bit_depth);
    uint8_t getBitDepth() const;
    esp_err_t setFrameLen(uint16_t frame_len_octets);
    esp_err_t setFrameDuration(uint32_t frame_duration_us);
    esp_err_t setWifiPhyRate(wifi_phy_mode_t phymode, wifi_phy_rate_t rate);
    wifi_phy_rate_t getWifiPhyRate() const { return m_tx_phy_rate; }
    uint32_t getSampleRate() const { return m_telemetry.sample_rate; }
    uint32_t getFrameDurationUs() const { return m_frame_duration_us; }
    uint16_t getFrameLen() const { return m_octets_per_frame; }

    // Multi-Channel Target Selection (SINK node: 0..7)
    void setTargetChannel(uint8_t channel_id) { m_target_channel = channel_id & 0x07; }
    uint8_t getTargetChannel() const { return m_target_channel; }

    // Real-Time USB Audio Stream Ingestion (SOURCE node)
    void processUsbVsafPacket(const uint8_t* data, size_t len);
    bool isUsbStreamActive() const { return m_usb_stream_active.load(std::memory_order_relaxed); }

    void onPacketReceived(const uint8_t* mac_addr, const uint8_t* data, int data_len, int8_t rssi = -127, uint8_t rate = 0);
    void transitionTo(NetworkState new_state);

    NetworkState getState() const { return m_state; }
    const char* getStateString() const;
    const char* getActiveCodecName() const;
    const char* getWifiPhyRateString() const;
    int8_t getLastRssi() const { return m_last_rx_rssi.load(std::memory_order_relaxed); }
    bool hasLocalAudioOutput() const { return (m_i2s_dac != nullptr && m_node_role == NODE_ROLE_SINK); }
    const StreamTelemetry& getStreamTelemetry() const { return m_telemetry; }

    int16_t getAudioFrameRMS_int16() { return m_audio_meter.getAudioFrameRMS_int16(); }
    int16_t getAudioPeak_int16() { return m_audio_meter.getAudioFramePeak_int16(); }
    int16_t getAudioFramePeak_int16() { return m_audio_meter.getAudioFramePeak_int16(); }
    float getAudioFrameRMS_dBFS() { return m_audio_meter.getAudioFrameRMS_dBFS(); }
    float getAudioPeak_dBFS() { return m_audio_meter.getAudioFramePeak_dBFS(); }
    float getAudioFramePeak_dBFS() { return m_audio_meter.getAudioFramePeak_dBFS(); }

    uint32_t getTxPacketsTotal() const { return m_tx_packets_total.load(std::memory_order_relaxed); }
    uint32_t getAndResetTxPacketsSec() { return m_tx_packets_sec.exchange(0, std::memory_order_relaxed); }

    uint32_t getRxPacketsTotal() const { return m_rx_packets_total.load(std::memory_order_relaxed); }
    uint32_t getAndResetRxPacketsSec() { return m_rx_packets_sec.exchange(0, std::memory_order_relaxed); }

    uint32_t getFifoOverflowCount() const { return m_fifo_overflow.load(std::memory_order_relaxed); }
    uint32_t getFifoUnderrunCount() const { return m_fifo_underrun.load(std::memory_order_relaxed); }
    uint32_t getAndResetFifoOverflowCount() { return m_fifo_overflow.exchange(0, std::memory_order_relaxed); }
    uint32_t getAndResetFifoUnderrunCount() { return m_fifo_underrun.exchange(0, std::memory_order_relaxed); }

    uint32_t getAndResetDmaUnderrunCount() { return m_i2s_dac ? m_i2s_dac->getAndResetUnderrunCount() : 0; }
    uint32_t getDmaUnderrunCount() const { return m_i2s_dac ? m_i2s_dac->getUnderrunCount() : 0; }
    uint8_t getHardwareGainDb() const { return m_i2s_dac ? m_i2s_dac->getHardwareGainDb() : 0; }
    void setHardwareGain(Hardware::Max98357Gain gain) { if (m_i2s_dac) m_i2s_dac->setHardwareGain(gain); }
    uint32_t getPlcCount() const { return m_lc3_codec.getPlcCount(); }
    uint32_t getAndResetPlcCount() { return m_lc3_codec.getAndResetPlcCount(); }

    void resetStreamingCounters();
    void resetErrorCounters();

    void setPrefillThresholdFrames(uint32_t frames) { m_prefill_threshold_frames = frames; }
    uint32_t getPrefillThresholdFrames() const { return m_prefill_threshold_frames; }
    void setWatchdogTimeoutFrames(uint32_t frames) { m_watchdog_timeout_frames = frames; }
    uint32_t getWatchdogTimeoutFrames() const { return m_watchdog_timeout_frames; }

    // Time Synchronization & Diagnostics Getters
    uint64_t getMasterTimeMs() const;
    bool isMasterTimeValid() const;
    uint32_t getClockSyncAdjustCount() const { return m_clock_sync_micro_adjust_count.load(std::memory_order_relaxed); }
    uint32_t getPrevFrameRecoveryCount() const { return m_prev_frame_recoveries.load(std::memory_order_relaxed); }
    // 10 Hz Polling & Diagnostics for Master Time Offset Ring Buffer
    void update10HzTimeOffsetStats();
    void getTimeOffsetStats(float& out_ema_ms, float& out_rb_med_ms, float& out_rb_rng_ms, bool& out_has_stats) const;

    void getCodecDurationStats(float& out_avg_ms, float& out_peak_ms, bool& out_has_data) const {
        m_codec_duration_ring_buffer.getStats(out_avg_ms, out_peak_ms, out_has_data);
    }

    // Stereo / Mono Broadcast Mode (SOURCE node)
    void setStereo(bool stereo) { m_is_stereo.store(stereo, std::memory_order_relaxed); }
    bool isStereo() const { return m_is_stereo.load(std::memory_order_relaxed); }

    // Test Hooks
    void setTestMagicWord(uint16_t magic) { m_active_magic = magic; }
    uint16_t getTestMagicWord() const { return m_active_magic; }
    void triggerSimulatedPacketDrop() { m_simulated_drop_count = 1; }

private:
    static void audioTaskRoutine(void* pvParameters);
    void runSourceLoop();
    void runSinkLoop();

    Codec::Lc3CodecEngine&     m_lc3_codec;    // Primary encoder (Left / Mono) or SINK decoder
    Codec::Lc3CodecEngine      m_lc3_codec_r;  // Secondary encoder (Right Channel for Stereo)
    Audio::ToneGenerator*      m_tone_gen;     // Primary tone generator (Left / Mono)
    Audio::ToneGenerator       m_tone_gen_r;   // Secondary tone generator (Right Channel for Stereo)
    Hardware::I2sAudioDriver*  m_i2s_dac;

    uint8_t                    m_node_role;
    uint8_t                    m_node_id;
    uint8_t                    m_target_channel = 0; // SINK listens to this channel (0..7)
    uint16_t                   m_octets_per_frame = CONFIG_ESPNOW_FRAME_LEN_OCTETS;
    uint32_t                   m_frame_duration_us = 10000;
    NetworkState               m_state = NetworkState::OFF;
    StreamTelemetry            m_telemetry;
    AudioLevelMeter            m_audio_meter;
    SpscDurationRingBuffer<uint32_t, 10> m_codec_duration_ring_buffer;
    TimeOffsetRingBuffer       m_time_offset_ring_buffer;
    std::atomic<float>         m_cached_rb_median_ms{0.0f};
    std::atomic<float>         m_cached_rb_range_ms{0.0f};
    std::atomic<bool>          m_has_cached_offset_stats{false};

    bool                       m_wifi_initialized = false;
    bool                       m_audio_task_running = false;
    uint8_t                    m_last_rx_seq = 0;
    bool                       m_has_last_rx_seq = false;

    uint32_t                   m_prefill_threshold_frames = CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES;
    uint32_t                   m_watchdog_timeout_frames  = CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES;

    uint16_t                   m_active_magic = VSAF_DEFAULT_MAGIC;
    int64_t                    m_master_time_offset_us = 0;
    std::atomic<int64_t>       m_last_sync_time_us{0};
    std::atomic<uint32_t>      m_clock_sync_micro_adjust_count{0};
    std::atomic<uint32_t>      m_prev_frame_recoveries{0};
    std::atomic<uint32_t>      m_simulated_drop_count{0};

    // Broadcast Mode & Ingest State (SOURCE node)
    std::atomic<bool>          m_is_stereo{false};
    std::atomic<bool>          m_usb_stream_active{false};
    std::atomic<int64_t>       m_last_usb_packet_time_us{0};

    std::atomic<uint32_t>      m_tx_packets_total{0};
    std::atomic<uint32_t>      m_tx_packets_sec{0};
    std::atomic<uint32_t>      m_rx_packets_total{0};
    std::atomic<uint32_t>      m_rx_packets_sec{0};
    std::atomic<uint32_t>      m_fifo_overflow{0};
    std::atomic<uint32_t>      m_fifo_underrun{0};
    std::atomic<int8_t>        m_last_rx_rssi{-127};
    std::atomic<uint8_t>       m_last_rx_rate{0};
    wifi_phy_mode_t            m_tx_phy_mode = WIFI_PHY_MODE_11G;
    wifi_phy_rate_t            m_tx_phy_rate = WIFI_PHY_RATE_12M;
};

} // namespace AudioNet
