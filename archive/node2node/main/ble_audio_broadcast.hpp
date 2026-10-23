#include "audio_metering.hpp"
#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include "esp_err.h"
#include "config.h"
#include "lc3_codec.hpp"
#include "tone_generator.hpp"
#include "i2s_audio.hpp"

extern "C" {
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
}

namespace Bluetooth {

enum class BluetoothState {
    OFF,
    SCANNING,
    STREAMING,
    BROADCASTING
};

const char* bluetoothStateToString(BluetoothState state);

/* =====================================================================
 *        STANDARDIZED BLUETOOTH SIG LE AUDIO DATA STRUCTURES
 * ===================================================================== */

/**
 * @brief Standard Bluetooth SIG BASS Receive State (UUID 0x2BC8)
 * Exposes the active synchronization status of an Auracast broadcast stream.
 */
struct __attribute__((packed)) BassReceiveState {
    uint8_t  source_id = 0;              /* Source ID assigned by SINK (0 to 255) */
    uint8_t  source_addr_type = 0;       /* Public (0x00) or Random (0x01) Address */
    uint8_t  source_addr[6] = {0};       /* 6-byte Bluetooth Device Address of Broadcaster */
    uint8_t  source_adv_sid = 1;         /* Advertising SID (1-15) of Periodic Advertising */
    uint8_t  broadcast_id[3] = {0x56, 0x34, 0x12}; /* 3-byte unique Broadcast ID (0x123456) */
    uint8_t  pa_sync_state = 0;          /* 0x00=Not Synced, 0x01=SyncInfo Req, 0x02=PA Synced, 0x03=Failed */
    uint8_t  big_encryption = 0;         /* 0x00=Unencrypted, 0x01=Code Req, 0x02=Decrypting, 0x03=Bad Code */
    uint8_t  bad_code[16] = {0};         /* 16-byte bad broadcast encryption key if decryption failed */
    uint8_t  num_subgroups = 1;          /* Number of audio subgroups in the broadcast */
    uint32_t bis_sync_bitfield = 0x00000001; /* Bitfield: bit 0=BIS 1, bit 1=BIS 2 (Stereo = 0x03) */
    uint8_t  metadata_len = 0;           /* Additional LTV formatted metadata bytes */
};

/**
 * @brief Standard Bluetooth SIG VCS Volume State (UUID 0x2B7D)
 * Exposes current volume level and mute state.
 */
struct __attribute__((packed)) VcsVolumeState {
    uint8_t volume_setting = 77;         /* Volume setting: 0 (Min/Silence) to 255 (Max/0dB). Default ~30% */
    uint8_t mute = 0;                    /* Mute state: 0x00 = Unmuted, 0x01 = Muted */
    uint8_t change_counter = 0;          /* Monotonically increasing counter per state change */
};

/**
 * @brief Multi-SINK Tracking Table Entry on SOURCE Node
 * Tracks up to MAX_GATT_SINK_NODES connected via BLE GATT.
 */
struct DiscoveredSinkNode {
    uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ble_addr_t addr = {};
    std::string device_name = "Unknown";
    int8_t   rssi = -45;
    uint8_t  volume_setting = 77;        /* Volume level (0 - 255) */
    float    volume_percent = 30.0f;     /* Volume level in % */
    bool     is_muted = false;
    uint8_t  pa_sync_state = 0;          /* 0=Not synced, 2=PA Synced */
    uint32_t bis_sync_bitfield = 0;
    uint32_t last_seen_tick = 0;         /* FreeRTOS tick when last packet/notification was received */
    bool     connected = false;
    bool     connecting = false;
    bool     bass_configured = false;
    uint16_t bass_state_handle = 0;      /* GATT value handle for BASS Receive State (0x2BC8) */
    uint16_t bass_cp_handle = 0;         /* GATT value handle for BASS Control Point (0x2BC7) */
    uint16_t vcs_state_handle = 0;       /* GATT value handle for VCS Volume State (0x2B7D) */
    uint16_t vcs_cp_handle = 0;          /* GATT value handle for VCS Volume Control Point (0x2B7E) */
};

/**
 * @brief Diagnostic Telemetry structure for periodic logging and UI rendering
 */
struct StreamTelemetry {
    std::string source_name = "N/A";
    std::string codec_name = "LC3 fixp";
    uint32_t sample_rate = AUDIO_SAMPLE_RATE_HZ;
    uint8_t  bit_depth = AUDIO_BIT_DEPTH;
    uint8_t  channels = AUDIO_CHANNELS_NUM;
    uint32_t bitrate_kbps = AUDIO_BITRATE_KBPS;
    uint8_t  bis_index = 1;
    int8_t   rssi_dbm = 0;
    uint32_t broadcast_id = 0x123456;
    uint32_t packets_count = 0;
    bool     is_synced = false;
    uint8_t  volume_setting = 77;        /* Current volume 0 - 255 */
    uint8_t  volume_percent = 30;        /* Current volume in percent */
    bool     is_muted = false;

    std::string getStatusString() const {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s %u-bit %.1f kHz",
                 (channels == 1) ? "Mono" : "Stereo",
                 bit_depth,
                 static_cast<float>(sample_rate) / 1000.0f);
        return std::string(buf);
    }

    std::string getCodecString() const {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s @ %lu kbps",
                 codec_name.c_str(),
                 bitrate_kbps);
        return std::string(buf);
    }
};

/* Convenience human-readable GATT debug printout helpers */
void printGATTnotification(const char* service_name, uint16_t chr_uuid, const uint8_t* data, size_t len);
void printGATTcommand(const char* service_name, uint16_t chr_uuid, const uint8_t* data, size_t len);

class BleAudioBroadcast {
public:
    bool m_is_connecting = false;
    BleAudioBroadcast(Codec::Lc3CodecEngine& lc3_codec, 
                      Audio::ToneGenerator* tone_gen = nullptr, 
                      Hardware::I2sAudioDriver* i2s_dac = nullptr);
    ~BleAudioBroadcast();

    esp_err_t init(uint8_t node_role);
    void startAudioTask();

    /* State Machine Framework Interface */
    esp_err_t transitionTo(BluetoothState target_state);
    BluetoothState getState() const { return m_state; }
    const char* getStateString() const { return bluetoothStateToString(m_state); }

    const StreamTelemetry& getStreamTelemetry() const { return m_telemetry; }
    uint32_t getPacketsCount() const { return m_telemetry.packets_count; }

    uint8_t getVolumeSetting() const { return m_vcs_state.volume_setting; }
    uint8_t getVolumePercent() const { return (m_vcs_state.volume_setting * 100) / 255; }
    bool isMuted() const { return m_vcs_state.mute != 0; }

    void setVolumeSetting(uint8_t vol, bool notify = true);
    void setMute(bool mute, bool notify = true);

    void setLfoEnabled(bool enabled) { m_lfo_enabled = enabled; }
    bool isLfoEnabled() const { return m_lfo_enabled; }
    void sendManualVolumeToAllSinks(uint8_t vol_pct);

    /**
     * @brief Reconfigures codec and DMA buffer length based on sample rate, frame duration & channels
     */
    esp_err_t configureAudioFormat(uint32_t sample_rate_hz, uint32_t frame_duration_us, uint16_t octets_per_frame, uint8_t channels) {
        m_telemetry.sample_rate = sample_rate_hz;
        m_telemetry.channels = channels;
        m_telemetry.bitrate_kbps = (static_cast<uint32_t>(octets_per_frame) * 8 * 1000) / (frame_duration_us / 1000);
        if (m_i2s_dac) {
            m_i2s_dac->reconfigurePipeline(sample_rate_hz, frame_duration_us, 2);
        }
        return m_lc3_codec.initDecoder(sample_rate_hz, 1, frame_duration_us, octets_per_frame);
    }
    /* Audio Signal Metering & Statistics Component */
    int16_t getAudioFramePeak_int16(unsigned int frameCount = AudioMetering::AudioSignalMeter::DEFAULT_HISTORY_FRAMES) const { return m_audio_meter.getAudioFramePeak_int16(frameCount); }
    float getAudioFramePeak_pct(unsigned int frameCount = AudioMetering::AudioSignalMeter::DEFAULT_HISTORY_FRAMES) const { return m_audio_meter.getAudioFramePeak_pct(frameCount); }
    float getAudioFramePeak_dBFS(unsigned int frameCount = AudioMetering::AudioSignalMeter::DEFAULT_HISTORY_FRAMES) const { return m_audio_meter.getAudioFramePeak_dBFS(frameCount); }
    int16_t getAudioFrameRMS_int16(unsigned int frameCount = AudioMetering::AudioSignalMeter::DEFAULT_HISTORY_FRAMES) const { return m_audio_meter.getAudioFrameRMS_int16(frameCount); }
    float getAudioFrameRMS_dBFS(unsigned int frameCount = AudioMetering::AudioSignalMeter::DEFAULT_HISTORY_FRAMES) const { return m_audio_meter.getAudioFrameRMS_dBFS(frameCount); }
    float getAudioFrameRMS_pct(unsigned int frameCount = AudioMetering::AudioSignalMeter::DEFAULT_HISTORY_FRAMES) const { return m_audio_meter.getAudioFrameRMS_pct(frameCount); }

    uint32_t getAndResetDmaUnderrunCount() { return m_i2s_dac ? m_i2s_dac->getAndResetUnderrunCount() : 0; }
    uint32_t getAndResetPlcCount() { return m_lc3_codec.getAndResetPlcCount(); }
    uint32_t getPlcCount() const { return m_lc3_codec.getPlcCount(); }
    
    // Thread-safe SPSC FIFO & os_mbuf Diagnostic Counters
    uint32_t getAndResetFifoUnderrunCount();
    uint32_t getAndResetFifoOverflowCount();
    uint32_t getFifoUnderrunCount() const;
    uint32_t getFifoOverflowCount() const;

    uint32_t getAndResetMbufUnderrunCount();
    uint32_t getAndResetMbufOverflowCount();
    uint32_t getMbufUnderrunCount() const;
    uint32_t getMbufOverflowCount() const;

    AudioMetering::AudioSignalMeter& getAudioMeter() { return m_audio_meter; }
    const AudioMetering::AudioSignalMeter& getAudioMeter() const { return m_audio_meter; }

    const std::vector<DiscoveredSinkNode>& getTrackedSinks() const { return m_tracked_sinks; }
    std::vector<DiscoveredSinkNode>& getTrackedSinksMutable() { return m_tracked_sinks; }

    void notifyAdvReceived(const char* name, int8_t rssi);
    void checkSyncState();
    void parseAdvReport(const uint8_t* data, uint8_t length_data, int8_t rssi, const ble_addr_t* addr);
    void startScanning();

    /* SINK GATT Server Access Callbacks */
    static int gattAccessPacs(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
    static int gattAccessBass(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
    static int gattAccessVcs(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

    /* SOURCE GATT Client Discovery Callbacks */
    static int onGattDiscSvcCb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg);
    static int onGattDiscChrCb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);

    /* SOURCE Multi-Sink Helpers */
    void addOrUpdateTrackedSink(uint16_t conn_handle, const ble_addr_t* addr, const char* name);
    void configureSinkBass(DiscoveredSinkNode& sink);
    void sendVcsVolumeToSink(DiscoveredSinkNode& sink, uint8_t volume_setting);
    void handleIncomingNotification(uint16_t conn_handle, uint16_t val_handle, const uint8_t* data, uint16_t len);

private:
    esp_err_t enableBluetoothHardware();
    esp_err_t disableBluetoothHardware();
    esp_err_t registerGattServices();

    static void hostTaskRoutine(void* param);
    static void audioTaskRoutine(void* param);
    static void vcsOscillatorTaskRoutine(void* param);
    static void onSyncCb(void);
    static void onResetCb(int reason);

    void runSourceLoop();
    void runSinkLoop();
    void runVcsOscillatorLoop();

    Codec::Lc3CodecEngine& m_lc3_codec;
    Audio::ToneGenerator* m_tone_gen = nullptr;
    Hardware::I2sAudioDriver* m_i2s_dac = nullptr;

    uint8_t m_node_role = CONFIG_ACTIVE_NODE_ROLE;
    bool m_initialized = false;
    bool m_ble_hw_enabled = false;
    bool m_audio_task_running = false;
    
    bool m_vcs_task_running = false;
    uint32_t m_last_adv_tick = 0;
    BluetoothState m_state = BluetoothState::OFF;
    AudioMetering::AudioSignalMeter m_audio_meter;
    StreamTelemetry m_telemetry;

    /* SINK GATT Server States & Value Handles */
    BassReceiveState m_bass_state;
    VcsVolumeState   m_vcs_state;
    uint8_t          m_vcs_flags = 0x01; /* Volume Setting Persisted */

    uint16_t m_bass_recv_state_handle = 0;
    uint16_t m_vcs_state_handle = 0;
    uint16_t m_vcs_flags_handle = 0;
    bool m_lfo_enabled = true;

    /* SOURCE Multi-SINK Tracking Table */
    std::vector<DiscoveredSinkNode> m_tracked_sinks;
};

} // namespace Bluetooth
