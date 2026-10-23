#include "ble_audio_broadcast.hpp"
#include "nimble/hci_common.h"
#include "host/ble_hs_hci.h"
#include "host/ble_hs_iso.h"
#include "status_led.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cmath>
#include <cstring>

static const char* TAG = "BLE_AUDIO";

extern "C" int ble_hs_hci_cmd_tx(uint16_t opcode, const void *cmd, uint8_t cmd_len, void *rsp, uint8_t rsp_len);
extern "C" int ble_hci_trans_hs_iso_tx(uint8_t *frag, uint16_t len, void *arg) { (void)frag; (void)len; (void)arg; return 0; }

#ifndef BLE_HCI_OCF_LE_BIG_CREATE_SYNC
#define BLE_HCI_OCF_LE_BIG_CREATE_SYNC 0x006b
#endif

#include <atomic>
#include <cmath>

namespace Bluetooth {




const char* bluetoothStateToString(BluetoothState state) {
    switch (state) {
        case BluetoothState::OFF: return "OFF";
        case BluetoothState::SCANNING: return "SCANNING";
        case BluetoothState::STREAMING: return "STREAMING";
        case BluetoothState::BROADCASTING: return "BROADCASTING";
        default: return "UNKNOWN";
    }
}

static BleAudioBroadcast* s_broadcast_instance = nullptr;
static TaskHandle_t s_audio_task_handle = nullptr;
static portMUX_TYPE s_lc3_rx_mux = portMUX_INITIALIZER_UNLOCKED;
struct Lc3RxPacket {
    uint8_t data[256];
    uint16_t len;
    uint8_t seq;
};
static constexpr size_t LC3_RX_FIFO_SIZE = 16;
static Lc3RxPacket s_rx_fifo[LC3_RX_FIFO_SIZE];
static size_t s_rx_fifo_head = 0;
static size_t s_rx_fifo_tail = 0;
static size_t s_rx_fifo_count = 0;
static int s_last_seen_seq = -1;
static bool s_is_periodic_synced = false;
static bool s_periodic_sync_pending = false;
static uint32_t s_rx_iso_pkt_count = 0;

static std::atomic<uint32_t> s_fifo_overflow_count{0};
static std::atomic<uint32_t> s_fifo_underrun_count{0};
static std::atomic<uint32_t> s_mbuf_overflow_count{0};
static std::atomic<uint32_t> s_mbuf_underrun_count{0};

static inline bool push_rx_lc3_frame(const uint8_t* data, size_t len, uint8_t seq = 0) {
    if (!data || len == 0 || len > sizeof(s_rx_fifo[0].data)) return false;
    taskENTER_CRITICAL(&s_lc3_rx_mux);
    if (s_rx_fifo_count >= LC3_RX_FIFO_SIZE) {
        s_rx_fifo_tail = (s_rx_fifo_tail + 1) % LC3_RX_FIFO_SIZE;
        s_rx_fifo_count--;
        s_fifo_overflow_count.fetch_add(1, std::memory_order_relaxed);
    }
    s_rx_fifo[s_rx_fifo_head].len = static_cast<uint16_t>(len);
    s_rx_fifo[s_rx_fifo_head].seq = seq;
    memcpy(s_rx_fifo[s_rx_fifo_head].data, data, len);
    s_rx_fifo_head = (s_rx_fifo_head + 1) % LC3_RX_FIFO_SIZE;
    s_rx_fifo_count++;
    taskEXIT_CRITICAL(&s_lc3_rx_mux);
    return true;
}

static inline bool pop_rx_lc3_frame(uint8_t* out_data, size_t* out_len, uint8_t* out_seq = nullptr) {
    if (!out_data || !out_len) return false;
    taskENTER_CRITICAL(&s_lc3_rx_mux);
    if (s_rx_fifo_count == 0) {
        taskEXIT_CRITICAL(&s_lc3_rx_mux);
        return false;
    }
    *out_len = s_rx_fifo[s_rx_fifo_tail].len;
    if (out_seq) *out_seq = s_rx_fifo[s_rx_fifo_tail].seq;
    memcpy(out_data, s_rx_fifo[s_rx_fifo_tail].data, *out_len);
    s_rx_fifo_tail = (s_rx_fifo_tail + 1) % LC3_RX_FIFO_SIZE;
    s_rx_fifo_count--;
    taskEXIT_CRITICAL(&s_lc3_rx_mux);
    return true;
}

static uint16_t s_bass_recv_state_val_handle = 0;
static uint16_t s_vcs_state_val_handle = 0;
static uint16_t s_vcs_flags_val_handle = 0;

static int on_iso_packet_rx(const uint8_t *data, uint16_t len, void *arg) {
    if (data == nullptr || len < 4) return 0;

    const uint8_t* payload = nullptr;
    size_t payload_len = 0;
    uint8_t seq = 0;

    // Parse HCI ISO Data Header (Bluetooth 5.3 Core Spec Vol 4 Part E 5.4.5)
    uint16_t handle_flags = data[0] | (data[1] << 8);
    uint8_t ts_flag = (handle_flags >> 14) & 0x01;
    uint16_t iso_load_len = data[2] | ((data[3] & 0x3F) << 8);

    size_t header_sz = 4 + (ts_flag ? 4 : 0) + 4; // 8 or 12 bytes
    if (len >= header_sz && iso_load_len > 0) {
        size_t sdu_hdr_offset = 4 + (ts_flag ? 4 : 0);
        uint16_t seq_num = data[sdu_hdr_offset] | (data[sdu_hdr_offset + 1] << 8);
        uint16_t sdu_info = data[sdu_hdr_offset + 2] | (data[sdu_hdr_offset + 3] << 8);
        uint16_t sdu_len = sdu_info & 0x0FFF;
        uint8_t status_flag = (sdu_info >> 14) & 0x03;

        if (status_flag == 0 && sdu_len > 0 && len >= header_sz + sdu_len) {
            payload = &data[header_sz];
            payload_len = sdu_len;
            seq = static_cast<uint8_t>(seq_num & 0xFF);
        }
    }

    // Direct SDU fallback if transport passes unencapsulated SDU
    if (!payload && len >= 20 && len <= 256) {
        payload = data;
        payload_len = len;
        seq = static_cast<uint8_t>(s_rx_iso_pkt_count & 0xFF);
    }

    if (payload && payload_len >= 20) {
        if (push_rx_lc3_frame(payload, payload_len, seq)) {
            s_rx_iso_pkt_count++;
            if (s_audio_task_handle) {
                vTaskNotifyGiveFromISR(s_audio_task_handle, nullptr);
            }
        }
    }

    if (s_rx_iso_pkt_count % 100 == 1) {
        ESP_LOGI(TAG, ">>> [NATIVE BLE 5.3 ISO CIS RX] Pkt #%lu | Size: %u B | Synced: %d <<<",
                 (unsigned long)s_rx_iso_pkt_count, (unsigned int)payload_len, (int)s_is_periodic_synced);
    }
    return 0;
}

/* =====================================================================
 *            CONVENIENCE DEBUG FUNCTIONS FOR GATT STRUCTURES
 * ===================================================================== */

void printGATTnotification(const char* service_name, uint16_t chr_uuid, const uint8_t* data, size_t len) {
    if (!service_name || !data || len == 0) return;

    if (chr_uuid == BLE_GATT_CHR_BASS_RECV_STATE) {
        /* Decode BASS Receive State Notification (UUID 0x2BC8) */
        uint8_t src_id = data[0];
        uint8_t pa_sync = (len > 8) ? data[8] : 0;
        uint8_t enc = (len > 9) ? data[9] : 0;
        uint32_t bis_sync = 0;
        if (len >= 14) {
            bis_sync = data[10] | (data[11] << 8) | (data[12] << 16) | (data[13] << 24);
        }
        ESP_LOGI("GATT_NOTIF", "[BASS 0x2BC8 Notification] SrcID: %u | PA Sync: %s (0x%02X) | Encryption: 0x%02X | BIS Sync: 0x%08lX",
                 src_id, (pa_sync == 0x02) ? "PA_SYNCED" : (pa_sync == 0x01) ? "SYNC_REQ" : "NOT_SYNCED", pa_sync, enc, (unsigned long)bis_sync);
    } else if (chr_uuid == BLE_GATT_CHR_VCS_STATE) {
        /* Decode VCS Volume State Notification (UUID 0x2B7D) */
        uint8_t vol = data[0];
        uint8_t mute = (len > 1) ? data[1] : 0;
        uint8_t counter = (len > 2) ? data[2] : 0;
        float pct = (static_cast<float>(vol) * 100.0f) / 255.0f;
        ESP_LOGI("GATT_NOTIF", "[VCS 0x2B7D Notification] Volume: %.1f%% (%u/255) | Mute: %s | Counter: #%u",
                 pct, vol, mute ? "MUTED" : "UNMUTED", counter);
    } else {
        ESP_LOGI("GATT_NOTIF", "[%s 0x%04X Notification] Length: %u bytes", service_name, chr_uuid, (unsigned int)len);
    }
}

void printGATTcommand(const char* service_name, uint16_t chr_uuid, const uint8_t* data, size_t len) {
    if (!service_name || !data || len == 0) return;

    if (chr_uuid == BLE_GATT_CHR_BASS_CP) {
        /* Decode BASS Control Point Write (UUID 0x2BC7) */
        uint8_t opcode = data[0];
        const char* op_str = "UNKNOWN";
        if (opcode == 0x01) op_str = "SET_BROADCAST_CODE";
        else if (opcode == 0x02) op_str = "ADD_SOURCE";
        else if (opcode == 0x03) op_str = "MODIFY_SOURCE";
        else if (opcode == 0x04) op_str = "SET_BROADCAST_CODE_RSP";
        else if (opcode == 0x05) op_str = "REMOVE_SOURCE";

        if (opcode == 0x02 && len >= 15) {
            uint32_t b_id = data[8] | (data[9] << 8) | (data[10] << 16);
            uint8_t pa_sync = data[11];
            uint32_t bis_sync = data[14] | (data[15] << 8) | (data[16] << 16) | (data[17] << 24);
            ESP_LOGI("GATT_CMD", "[BASS 0x2BC7 Write: %s] Broadcast_ID: 0x%06lX | PA Sync: %u | BIS Sync: 0x%08lX",
                     op_str, (unsigned long)b_id, pa_sync, (unsigned long)bis_sync);
        } else {
            ESP_LOGI("GATT_CMD", "[BASS 0x2BC7 Write: %s] Opcode: 0x%02X | Length: %u bytes", op_str, opcode, (unsigned int)len);
        }
    } else if (chr_uuid == BLE_GATT_CHR_VCS_CP) {
        /* Decode VCS Control Point Write (UUID 0x2B7E) */
        uint8_t opcode = data[0];
        const char* op_str = "UNKNOWN";
        if (opcode == 0x00) op_str = "REL_VOL_DOWN";
        else if (opcode == 0x01) op_str = "REL_VOL_UP";
        else if (opcode == 0x02) op_str = "UNMUTE";
        else if (opcode == 0x03) op_str = "MUTE";
        else if (opcode == 0x04) op_str = "SET_ABSOLUTE_VOLUME";

        if (opcode == 0x04 && len >= 2) {
            uint8_t vol = data[1];
            float pct = (static_cast<float>(vol) * 100.0f) / 255.0f;
            ESP_LOGI("GATT_CMD", "[VCS 0x2B7E Write: %s] Target: %.1f%% (%u/255)", op_str, pct, vol);
        } else {
            ESP_LOGI("GATT_CMD", "[VCS 0x2B7E Write: %s] Opcode: 0x%02X", op_str, opcode);
        }
    } else {
        ESP_LOGI("GATT_CMD", "[%s 0x%04X Write] Length: %u bytes", service_name, chr_uuid, (unsigned int)len);
    }
}

/* =====================================================================
 *                GATT SERVER ACCESS CALLBACKS (SINK NODE)
 * ===================================================================== */

int BleAudioBroadcast::gattAccessPacs(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == BLE_GATT_CHR_PACS_SINK_PAC) {
        /*
         * Published Audio Capabilities (PACS) - Sink PAC Record (0x2BC9)
         * Standard Bluetooth SIG PAC Record LTV Structure:
         * - Num PAC Records: 1
         * - Codec ID: 0x06 (LC3), Company ID: 0x0000, Vendor Codec: 0x0000
         * - Capabilities LTVs:
         *   * 0x03, 0x01, 0x60, 0x00: Sampling Freq (44.1 kHz 0x0020 & 48 kHz 0x0040)
         *   * 0x02, 0x02, 0x03: Frame Duration (7.5 ms 0x01 | 10 ms 0x02 = 0x03)
         *   * 0x02, 0x03, 0x03: Channel Count (1 & 2 channels = 0x03 Stereo Capable)
         *   * 0x05, 0x04, 0x14, 0x00, 0x78, 0x00: Octets/Frame (Min 20 to Max 120 octets = 16 to 160 kbps)
         * - Metadata LTV:
         *   * 0x03, 0x02, 0x04, 0x00: Preferred Audio Context = Media (0x0004)
         */
        static const uint8_t sink_pac_record[] = {
            0x01,                                           /* Num PAC Records = 1 */
            0x06, 0x00, 0x00, 0x00, 0x00,                   /* Codec: LC3 (0x06) */
            16,                                             /* Codec Specific Capabilities Length = 16 */
            0x03, 0x01, 0x60, 0x00,                         /* Sampling Freq: 44.1 & 48 kHz */
            0x02, 0x02, 0x03,                               /* Frame Duration: 7.5 & 10 ms */
            0x02, 0x03, 0x03,                               /* Channels: 1 & 2 (Stereo) */
            0x05, 0x04, 0x14, 0x00, 0x78, 0x00,             /* Min 20 octets, Max 120 octets */
            0x04,                                           /* Metadata Length = 4 */
            0x03, 0x02, 0x04, 0x00                          /* Audio Context: Media (0x0004) */
        };
        int rc = os_mbuf_append(ctxt->om, sink_pac_record, sizeof(sink_pac_record));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    } 
    else if (uuid == BLE_GATT_CHR_PACS_SINK_LOC) {
        /*
         * Sink Audio Locations (0x2BCA)
         * 32-bit bitfield: Front Left (0x00000001) | Front Right (0x00000002) = 0x00000003 (Stereo)
         */
        static const uint32_t audio_locations = 0x00000003;
        int rc = os_mbuf_append(ctxt->om, &audio_locations, sizeof(audio_locations));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (uuid == BLE_GATT_CHR_PACS_SINK_CTX) {
        /*
         * Sink Supported & Available Audio Contexts (0x2BCE)
         * 16-bit Supported (Media 0x0004) + 16-bit Available (Media 0x0004) = 0x00040004
         */
        static const uint32_t audio_contexts = 0x00040004;
        int rc = os_mbuf_append(ctxt->om, &audio_contexts, sizeof(audio_contexts));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

int BleAudioBroadcast::gattAccessBass(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (!s_broadcast_instance) return BLE_ATT_ERR_UNLIKELY;
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == BLE_GATT_CHR_BASS_RECV_STATE) {
        /* Read Broadcast Receive State (0x2BC8) */
        int rc = os_mbuf_append(ctxt->om, &s_broadcast_instance->m_bass_state, sizeof(BassReceiveState));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (uuid == BLE_GATT_CHR_BASS_CP) {
        /* Write to Broadcast Audio Scan Control Point (0x2BC7) */
        uint8_t buf[64] = {0};
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(buf)) len = sizeof(buf);
        os_mbuf_copydata(ctxt->om, 0, len, buf);

        printGATTcommand("BASS", BLE_GATT_CHR_BASS_CP, buf, len);

        uint8_t opcode = buf[0];
        if (opcode == 0x02 && len >= 15) {
            /*
             * Opcode 0x02: Add Source
             * Format: [0x02, Adv_Addr_Type, Addr(6), Adv_SID, Broadcast_ID(3), PA_Sync_State, PA_Interval(2), Num_Subgroups, BIS_Sync(4)]
             */
            s_broadcast_instance->m_bass_state.source_addr_type = buf[1];
            memcpy(s_broadcast_instance->m_bass_state.source_addr, &buf[2], 6);
            s_broadcast_instance->m_bass_state.source_adv_sid = buf[8];
            s_broadcast_instance->m_bass_state.broadcast_id[0] = buf[9];
            s_broadcast_instance->m_bass_state.broadcast_id[1] = buf[10];
            s_broadcast_instance->m_bass_state.broadcast_id[2] = buf[11];
            s_broadcast_instance->m_bass_state.pa_sync_state = 0x02; /* PA Synced */
            s_broadcast_instance->m_bass_state.big_encryption = 0x00; /* Unencrypted */
            
            if (len >= 18) {
                s_broadcast_instance->m_bass_state.bis_sync_bitfield = buf[14] | (buf[15] << 8) | (buf[16] << 16) | (buf[17] << 24);
            }

            s_broadcast_instance->m_telemetry.is_synced = true;
            s_broadcast_instance->m_telemetry.bis_index = 1;
            s_broadcast_instance->m_last_adv_tick = xTaskGetTickCount();

            /* Notify connected GATT client of updated BASS Receive State */
            if (s_bass_recv_state_val_handle != 0) {
                struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_broadcast_instance->m_bass_state, sizeof(BassReceiveState));
                if (om) {
                    ble_gatts_notify_custom(conn_handle, s_bass_recv_state_val_handle, om);
                }
            }

            s_broadcast_instance->transitionTo(BluetoothState::STREAMING);
            ESP_LOGI(TAG, "SINK: BASS Add Source Accepted! Locked to Broadcast ID 0x%02X%02X%02X (BIS #%u)",
                     buf[11], buf[10], buf[9], (unsigned int)s_broadcast_instance->m_telemetry.bis_index);
        }
        else if (opcode == 0x05) {
            /* Opcode 0x05: Remove Source */
            s_broadcast_instance->m_bass_state.pa_sync_state = 0x00; /* Not Synced */
            s_broadcast_instance->m_bass_state.bis_sync_bitfield = 0x00000000;
            s_broadcast_instance->transitionTo(BluetoothState::SCANNING);
            ESP_LOGI(TAG, "SINK: BASS Remove Source Accepted. Returning to SCANNING.");
        }

        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

int BleAudioBroadcast::gattAccessVcs(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (!s_broadcast_instance) return BLE_ATT_ERR_UNLIKELY;
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == BLE_GATT_CHR_VCS_STATE) {
        /* Read Volume State (0x2B7D) - 3 bytes: Volume Setting (0-255), Mute (0/1), Change Counter */
        int rc = os_mbuf_append(ctxt->om, &s_broadcast_instance->m_vcs_state, sizeof(VcsVolumeState));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (uuid == BLE_GATT_CHR_VCS_FLAGS) {
        /* Read Volume Flags (0x2B7F) - 1 byte (0x01 = Persisted) */
        int rc = os_mbuf_append(ctxt->om, &s_broadcast_instance->m_vcs_flags, 1);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (uuid == BLE_GATT_CHR_VCS_CP) {
        /* Write Volume Control Point (0x2B7E) */
        uint8_t buf[16] = {0};
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(buf)) len = sizeof(buf);
        os_mbuf_copydata(ctxt->om, 0, len, buf);

        printGATTcommand("VCS", BLE_GATT_CHR_VCS_CP, buf, len);

        uint8_t opcode = buf[0];
        switch (opcode) {
            case 0x00: /* Relative Volume Down */
                if (s_broadcast_instance->m_vcs_state.volume_setting >= 10)
                    s_broadcast_instance->m_vcs_state.volume_setting -= 10;
                else
                    s_broadcast_instance->m_vcs_state.volume_setting = 0;
                break;
            case 0x01: /* Relative Volume Up */
                if (s_broadcast_instance->m_vcs_state.volume_setting <= 245)
                    s_broadcast_instance->m_vcs_state.volume_setting += 10;
                else
                    s_broadcast_instance->m_vcs_state.volume_setting = 255;
                break;
            case 0x02: /* Unmute */
                s_broadcast_instance->m_vcs_state.mute = 0;
                break;
            case 0x03: /* Mute */
                s_broadcast_instance->m_vcs_state.mute = 1;
                break;
            case 0x04: /* Set Absolute Volume: [0x04, Volume_Setting (0-255), Change_Counter] */
                if (len >= 2) {
                    s_broadcast_instance->m_vcs_state.volume_setting = buf[1];
                }
                break;
            default:
                break;
        }

        s_broadcast_instance->m_vcs_state.change_counter++;
        s_broadcast_instance->m_telemetry.volume_setting = s_broadcast_instance->m_vcs_state.volume_setting;
        s_broadcast_instance->m_telemetry.volume_percent = (s_broadcast_instance->m_vcs_state.volume_setting * 100) / 255;
        s_broadcast_instance->m_telemetry.is_muted = (s_broadcast_instance->m_vcs_state.mute != 0);

        /* Notify connected GATT clients of updated volume state */
        if (s_vcs_state_val_handle != 0) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_broadcast_instance->m_vcs_state, sizeof(VcsVolumeState));
            if (om) {
                ble_gatts_notify_custom(conn_handle, s_vcs_state_val_handle, om);
            }
        }

        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* =====================================================================
 *                GATT SERVER DEFINITIONS (NIMBLE SERVICES TABLE)
 * ===================================================================== */

/* Static 16-bit UUID structures for C++ compatibility */
static const ble_uuid16_t uuid_pacs_svc = BLE_UUID16_INIT(BLE_GATT_SVC_PACS_UUID16);
static const ble_uuid16_t uuid_pacs_sink_pac = BLE_UUID16_INIT(BLE_GATT_CHR_PACS_SINK_PAC);
static const ble_uuid16_t uuid_pacs_sink_loc = BLE_UUID16_INIT(BLE_GATT_CHR_PACS_SINK_LOC);
static const ble_uuid16_t uuid_pacs_sink_ctx = BLE_UUID16_INIT(BLE_GATT_CHR_PACS_SINK_CTX);

static const ble_uuid16_t uuid_bass_svc = BLE_UUID16_INIT(BLE_GATT_SVC_BASS_UUID16);
static const ble_uuid16_t uuid_bass_recv_state = BLE_UUID16_INIT(BLE_GATT_CHR_BASS_RECV_STATE);
static const ble_uuid16_t uuid_bass_cp = BLE_UUID16_INIT(BLE_GATT_CHR_BASS_CP);

static const ble_uuid16_t uuid_vcs_svc = BLE_UUID16_INIT(BLE_GATT_SVC_VCS_UUID16);
static const ble_uuid16_t uuid_vcs_state = BLE_UUID16_INIT(BLE_GATT_CHR_VCS_STATE);
static const ble_uuid16_t uuid_vcs_cp = BLE_UUID16_INIT(BLE_GATT_CHR_VCS_CP);
static const ble_uuid16_t uuid_vcs_flags = BLE_UUID16_INIT(BLE_GATT_CHR_VCS_FLAGS);

static const struct ble_gatt_chr_def s_pacs_characteristics[] = {
    {
        /* Sink PAC (0x2BC9) - Characteristic: Read */
        .uuid = &uuid_pacs_sink_pac.u,
        .access_cb = BleAudioBroadcast::gattAccessPacs,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        /* Sink Audio Locations (0x2BCA) - Characteristic: Read */
        .uuid = &uuid_pacs_sink_loc.u,
        .access_cb = BleAudioBroadcast::gattAccessPacs,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        /* Sink Audio Contexts (0x2BCE) - Characteristic: Read */
        .uuid = &uuid_pacs_sink_ctx.u,
        .access_cb = BleAudioBroadcast::gattAccessPacs,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    { 0 } /* Terminating characteristic */
};

static const struct ble_gatt_chr_def s_bass_characteristics[] = {
    {
        /* Broadcast Receive State (0x2BC8) - Characteristic: Read | Notify */
        .uuid = &uuid_bass_recv_state.u,
        .access_cb = BleAudioBroadcast::gattAccessBass,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &s_bass_recv_state_val_handle,
    },
    {
        /* Broadcast Audio Scan Control Point (0x2BC7) - Characteristic: Write */
        .uuid = &uuid_bass_cp.u,
        .access_cb = BleAudioBroadcast::gattAccessBass,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    { 0 } /* Terminating characteristic */
};

static const struct ble_gatt_chr_def s_vcs_characteristics[] = {
    {
        /* Volume State (0x2B7D) - Characteristic: Read | Notify */
        .uuid = &uuid_vcs_state.u,
        .access_cb = BleAudioBroadcast::gattAccessVcs,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &s_vcs_state_val_handle,
    },
    {
        /* Volume Control Point (0x2B7E) - Characteristic: Write */
        .uuid = &uuid_vcs_cp.u,
        .access_cb = BleAudioBroadcast::gattAccessVcs,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        /* Volume Flags (0x2B7F) - Characteristic: Read | Notify */
        .uuid = &uuid_vcs_flags.u,
        .access_cb = BleAudioBroadcast::gattAccessVcs,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &s_vcs_flags_val_handle,
    },
    { 0 } /* Terminating characteristic */
};

static const struct ble_gatt_svc_def s_gatt_services[] = {
    {
        /* Service: Published Audio Capabilities Service (PACS - 0x184E) */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_pacs_svc.u,
        .includes = nullptr,
        .characteristics = s_pacs_characteristics,
    },
    {
        /* Service: Broadcast Audio Scan Service (BASS - 0x184F) */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_bass_svc.u,
        .includes = nullptr,
        .characteristics = s_bass_characteristics,
    },
    {
        /* Service: Volume Control Service (VCS - 0x1844) */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_vcs_svc.u,
        .includes = nullptr,
        .characteristics = s_vcs_characteristics,
    },
    { 0 } /* Terminating service */
};

esp_err_t BleAudioBroadcast::registerGattServices() {
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "GATT Server Services Registered: PACS (0x184E), BASS (0x184F), VCS (0x1844)");
    return ESP_OK;
}

/* =====================================================================
 *            GATT CLIENT & CENTRAL DISCOVERY (SOURCE NODE)
 * ===================================================================== */

void BleAudioBroadcast::addOrUpdateTrackedSink(uint16_t conn_handle, const ble_addr_t* addr, const char* name) {
    /* 1. Try to find existing sink by MAC address */
    if (addr) {
        for (auto& sink : m_tracked_sinks) {
            if (memcmp(sink.addr.val, addr->val, 6) == 0) {
                if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                    sink.conn_handle = conn_handle;
                    sink.connected = true;
                }
                if (name && strlen(name) > 0) sink.device_name = name;
                sink.last_seen_tick = xTaskGetTickCount();
                return;
            }
        }
    }

    /* 2. Try to find existing sink by conn_handle */
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        for (auto& sink : m_tracked_sinks) {
            if (sink.conn_handle == conn_handle) {
                sink.connected = true;
                if (addr) sink.addr = *addr;
                if (name && strlen(name) > 0) sink.device_name = name;
                sink.last_seen_tick = xTaskGetTickCount();
                return;
            }
        }
    }

    /* 3. Add new sink entry if table not full */
    if (m_tracked_sinks.size() < MAX_GATT_SINK_NODES) {
        DiscoveredSinkNode new_sink;
        new_sink.conn_handle = conn_handle;
        if (addr) new_sink.addr = *addr;
        if (name) new_sink.device_name = name;
        new_sink.connected = (conn_handle != BLE_HS_CONN_HANDLE_NONE);
        new_sink.last_seen_tick = xTaskGetTickCount();
        m_tracked_sinks.push_back(new_sink);
        ESP_LOGI(TAG, "SOURCE: Added new SINK to tracking table (Total Tracked: %u / %u)",
                 (unsigned int)m_tracked_sinks.size(), MAX_GATT_SINK_NODES);
    }
}

int BleAudioBroadcast::onGattDiscSvcCb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                       const struct ble_gatt_svc *service, void *arg) {
    if (!s_broadcast_instance) return 0;
    if (error->status == 0 && service) {
        uint16_t uuid = ble_uuid_u16(&service->uuid.u);
        if (uuid == BLE_GATT_SVC_BASS_UUID16 || uuid == BLE_GATT_SVC_VCS_UUID16) {
            /* Discover characteristics in this service */
            ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, onGattDiscChrCb, s_broadcast_instance);
        }
    }
    return 0;
}

int BleAudioBroadcast::onGattDiscChrCb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                       const struct ble_gatt_chr *chr, void *arg) {
    if (!s_broadcast_instance) return 0;
    if (error->status == 0 && chr) {
        uint16_t uuid = ble_uuid_u16(&chr->uuid.u);

        for (auto& sink : s_broadcast_instance->m_tracked_sinks) {
            if (sink.conn_handle == conn_handle) {
                if (uuid == BLE_GATT_CHR_BASS_RECV_STATE) {
                    sink.bass_state_handle = chr->val_handle;
                    /* Subscribe to BASS Receive State notifications by writing 0x0001 to CCCD (val_handle + 1) */
                    uint16_t cccd_val = 0x0001;
                    ble_gattc_write_flat(conn_handle, chr->val_handle + 1, &cccd_val, sizeof(cccd_val), nullptr, nullptr);
                }
                else if (uuid == BLE_GATT_CHR_BASS_CP) {
                    sink.bass_cp_handle = chr->val_handle;
                    /* Immediately assign our active BIS Auracast stream to this SINK */
                    s_broadcast_instance->configureSinkBass(sink);
                }
                else if (uuid == BLE_GATT_CHR_VCS_STATE) {
                    sink.vcs_state_handle = chr->val_handle;
                    /* Subscribe to VCS Volume State notifications by writing 0x0001 to CCCD (val_handle + 1) */
                    uint16_t cccd_val = 0x0001;
                    ble_gattc_write_flat(conn_handle, chr->val_handle + 1, &cccd_val, sizeof(cccd_val), nullptr, nullptr);
                }
                else if (uuid == BLE_GATT_CHR_VCS_CP) {
                    sink.vcs_cp_handle = chr->val_handle;
                }
                break;
            }
        }
    }
    return 0;
}

void BleAudioBroadcast::handleIncomingNotification(uint16_t conn_handle, uint16_t val_handle, const uint8_t* data, uint16_t len) {
    for (auto& sink : m_tracked_sinks) {
        if (sink.conn_handle == conn_handle) {
            sink.last_seen_tick = xTaskGetTickCount();
            if (val_handle == sink.bass_state_handle) {
                printGATTnotification("BASS", BLE_GATT_CHR_BASS_RECV_STATE, data, len);
                if (len > 8) sink.pa_sync_state = data[8];
                if (len >= 14) sink.bis_sync_bitfield = data[10] | (data[11] << 8) | (data[12] << 16) | (data[13] << 24);
            }
            else if (val_handle == sink.vcs_state_handle) {
                printGATTnotification("VCS", BLE_GATT_CHR_VCS_STATE, data, len);
                sink.volume_setting = data[0];
                sink.volume_percent = (static_cast<float>(data[0]) * 100.0f) / 255.0f;
                sink.is_muted = (len > 1 && data[1] != 0);
            }
            break;
        }
    }
}

void BleAudioBroadcast::configureSinkBass(DiscoveredSinkNode& sink) {
    if (sink.conn_handle == BLE_HS_CONN_HANDLE_NONE || sink.bass_cp_handle == 0) return;

    /*
     * Build standard BASS Opcode 0x02: Add Source command
     * [0x02, Adv_Addr_Type, Addr(6), Adv_SID, Broadcast_ID(3), PA_Sync_State, PA_Interval(2), Num_Subgroups, BIS_Sync(4)]
     */
    uint8_t cmd[18] = {0};
    cmd[0] = 0x02; /* Add Source */
    cmd[1] = 0x00; /* Public Address */
    /* Addr */
    cmd[8] = 0x01; /* Adv SID = 1 */
    cmd[9] = 0x56; cmd[10] = 0x34; cmd[11] = 0x12; /* Broadcast ID: 0x123456 */
    cmd[12] = 0x02; /* PA_Sync = Synchronize to PA */
    cmd[13] = 0x00; cmd[14] = 0x00; /* PA Interval */
    cmd[15] = 0x01; /* Num Subgroups = 1 */
    cmd[16] = 0x01; cmd[17] = 0x00; /* BIS Sync: BIS #1 (bit 0) */

    printGATTcommand("BASS", BLE_GATT_CHR_BASS_CP, cmd, sizeof(cmd));

    int rc = ble_gattc_write_flat(sink.conn_handle, sink.bass_cp_handle, cmd, sizeof(cmd), nullptr, nullptr);
    if (rc == 0) {
        sink.bass_configured = true;
        ESP_LOGI(TAG, "SOURCE: Sent BASS Add Source to SINK (Handle: %u | Broadcast ID: 0x123456 | BIS #1)", sink.conn_handle);
    } else {
        ESP_LOGW(TAG, "SOURCE: Failed to send BASS Add Source command (rc = %d)", rc);
    }
}

void BleAudioBroadcast::sendManualVolumeToAllSinks(uint8_t vol_pct) {
    uint8_t vol_setting = (static_cast<uint16_t>(vol_pct) * 255) / 100;
    ESP_LOGI(TAG, "Manual SINK Volume broadcast: %d%% (%d/255) to %u sinks", 
             vol_pct, vol_setting, (unsigned int)m_tracked_sinks.size());
    for (auto& sink : m_tracked_sinks) {
        if (sink.connected && sink.vcs_cp_handle != 0) {
            sendVcsVolumeToSink(sink, vol_setting);
        }
    }
}

void BleAudioBroadcast::sendVcsVolumeToSink(DiscoveredSinkNode& sink, uint8_t volume_setting) {
    if (sink.conn_handle == BLE_HS_CONN_HANDLE_NONE || sink.vcs_cp_handle == 0) return;

    /*
     * Build standard VCS Opcode 0x04: Set Absolute Volume command
     * [0x04, Volume_Setting (0-255)]
     */
    uint8_t cmd[2] = { 0x04, volume_setting };

    ble_gattc_write_flat(sink.conn_handle, sink.vcs_cp_handle, cmd, sizeof(cmd), nullptr, nullptr);
    sink.volume_setting = volume_setting;
    sink.volume_percent = (static_cast<float>(volume_setting) * 100.0f) / 255.0f;
    sink.last_seen_tick = xTaskGetTickCount();
}

/* =====================================================================
 *                       GAP EVENT HANDLERS
 * ===================================================================== */

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (!s_broadcast_instance) return 0;

    switch (event->type) {
        case BLE_GAP_EVENT_EXT_DISC: {
            const auto &disc = event->ext_disc;
            s_broadcast_instance->parseAdvReport(disc.data, disc.length_data, disc.rssi, &disc.addr);
            for (auto& s : s_broadcast_instance->getTrackedSinksMutable()) {
                if (memcmp(s.addr.val, disc.addr.val, 6) == 0) {
                    s.rssi = disc.rssi;
                    s.last_seen_tick = xTaskGetTickCount();
                }
            }
            break;
        }
#if defined(CONFIG_BT_NIMBLE_ISO)
        case BLE_GAP_EVENT_CIS_REQUEST: {
            uint16_t cis_h = event->cis_req.cis_handle;
            uint16_t conn_h = event->cis_req.conn_handle;
            uint8_t cig_id = event->cis_req.cig_id;
            uint8_t cis_id = event->cis_req.cis_id;
            ESP_LOGI(TAG, "SINK: *** Received BLE_GAP_EVENT_CIS_REQUEST *** CIS Handle: %u, ACL Handle: %u, CIG: %u, CIS ID: %u",
                     cis_h, conn_h, cig_id, cis_id);

            int rc = ble_gap_accept_cis_request(cis_h, ble_gap_event_cb, nullptr);
            if (rc == 0) {
                ESP_LOGI(TAG, "SINK: Successfully accepted CIS Request (CIS Handle: %u)", cis_h);
            } else {
                ESP_LOGE(TAG, "SINK: Failed to accept CIS Request (rc = %d)", rc);
            }
            break;
        }
        case BLE_GAP_EVENT_CIS_ESTAB: {
            uint16_t cis_h = event->cis_estab.cis_handle;
            int status = event->cis_estab.status;
            if (status == 0) {
                ESP_LOGI(TAG, "=========================================================");
                ESP_LOGI(TAG, "*** BLE 5.3 CONNECTED ISO STREAM (CIS) ESTABLISHED! ***");
                ESP_LOGI(TAG, "CIS Handle: %u | Sync Delay: %lu us | Latency: %lu us",
                         cis_h, (unsigned long)event->cis_estab.cis_sync_delay, (unsigned long)event->cis_estab.transport_latency_c_to_p);
                ESP_LOGI(TAG, "=========================================================");
                s_broadcast_instance->transitionTo(BluetoothState::STREAMING);
            } else {
                ESP_LOGE(TAG, "BLE 5.3 CIS Establishment Failed! Status: %d", status);
            }
            break;
        }
#endif
        case BLE_GAP_EVENT_NOTIFY_RX: {
            if (event->notify_rx.om != nullptr) {
                uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
                if (len <= 256) {
                    uint8_t temp[256];
                    os_mbuf_copydata(event->notify_rx.om, 0, len, temp);
                    if (push_rx_lc3_frame(temp, len)) {
                        if (s_audio_task_handle) {
                            xTaskNotifyGive(s_audio_task_handle);
                        }
                    }
                }
            }
            break;
        }
        case BLE_GAP_EVENT_DISC: {
            const auto &disc = event->disc;
            s_broadcast_instance->parseAdvReport(disc.data, disc.length_data, disc.rssi, &disc.addr);
            break;
        }
                case BLE_GAP_EVENT_CONNECT: {
            s_broadcast_instance->m_is_connecting = false;
            if (event->connect.status == 0) {
                uint16_t conn_handle = event->connect.conn_handle;
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(conn_handle, &desc) == 0) {
                    ESP_LOGI(TAG, "BLE GAP Connect Success! Conn Handle: %u, Peer: %02x:%02x:%02x:%02x:%02x:%02x",
                             conn_handle, desc.peer_id_addr.val[5], desc.peer_id_addr.val[4], desc.peer_id_addr.val[3],
                             desc.peer_id_addr.val[2], desc.peer_id_addr.val[1], desc.peer_id_addr.val[0]);
                    s_broadcast_instance->addOrUpdateTrackedSink(conn_handle, &desc.peer_id_addr, nullptr);
                } else {
                    ESP_LOGI(TAG, "BLE GAP Connect Success! Conn Handle: %u", conn_handle);
                    s_broadcast_instance->addOrUpdateTrackedSink(conn_handle, nullptr, nullptr);
                }

                for (auto& s : s_broadcast_instance->getTrackedSinksMutable()) {
                    if (s.conn_handle == conn_handle) {
                        s.connected = true;
                        s.connecting = false;
                        break;
                    }
                }

                /* If SOURCE, initiate GATT service discovery on the connected SINK */
                if (get_system_config()->node_role == NODE_ROLE_SOURCE) {
                    int drc = ble_gattc_disc_all_svcs(conn_handle, BleAudioBroadcast::onGattDiscSvcCb, s_broadcast_instance);
                    ESP_LOGI(TAG, "SOURCE: Initiated GATT service discovery on conn_handle %u (rc = %d)", conn_handle, drc);
                }
            } else {
                ESP_LOGW(TAG, "BLE GAP Connect Failed, status = %d", event->connect.status);
                for (auto& s : s_broadcast_instance->getTrackedSinksMutable()) {
                    s.connecting = false;
                }
            }
            break;
        }
                case BLE_GAP_EVENT_DISCONNECT: {
            uint16_t conn_handle = event->disconnect.conn.conn_handle;
            ESP_LOGW(TAG, "BLE GAP Disconnected! Conn Handle: %u, Reason: %d", conn_handle, event->disconnect.reason);
            s_broadcast_instance->m_is_connecting = false;
            for (auto it = s_broadcast_instance->getTrackedSinksMutable().begin(); it != s_broadcast_instance->getTrackedSinksMutable().end(); ) {
                if (it->conn_handle == conn_handle) {
                    ESP_LOGI(TAG, "Removing disconnected SINK node '%s' from tracking table", it->device_name.c_str());
                    it = s_broadcast_instance->getTrackedSinksMutable().erase(it);
                } else {
                    ++it;
                }
            }

            if (get_system_config()->node_role == NODE_ROLE_SOURCE) {
                /* SOURCE: resume scanning to rediscover and reconnect to SINK */
                s_broadcast_instance->startScanning();
            } else {
                /* SINK: restart connectable advertising so SOURCE can discover and reconnect! */
                ble_gap_ext_adv_start(1, 0, 0);
                ESP_LOGI(TAG, "SINK: Restarted Connectable Extended Advertising (Instance 1)");
            }
            break;
        }
        
        case BLE_GAP_EVENT_DISC_COMPLETE: {
            s_broadcast_instance->startScanning();
            break;
        }
        case BLE_GAP_EVENT_ADV_COMPLETE: {
            if (get_system_config()->node_role == NODE_ROLE_SINK) {
                /* SINK: Always keep connectable advertising active */
                ble_gap_ext_adv_start(1, 0, 0);
            }
            break;
        }
        default:
            break;
    }
    return 0;
}

/* =====================================================================
 *                  LIFECYCLE & STATE MACHINE
 * ===================================================================== */

BleAudioBroadcast::BleAudioBroadcast(Codec::Lc3CodecEngine& lc3_codec, 
                                     Audio::ToneGenerator* tone_gen, 
                                     Hardware::I2sAudioDriver* i2s_dac)
    : m_lc3_codec(lc3_codec), m_tone_gen(tone_gen), m_i2s_dac(i2s_dac) {
    s_broadcast_instance = this;
}

BleAudioBroadcast::~BleAudioBroadcast() {
    transitionTo(BluetoothState::OFF);
    if (s_broadcast_instance == this) {
        s_broadcast_instance = nullptr;
    }
}

esp_err_t BleAudioBroadcast::init(uint8_t node_role) {
    m_node_role = node_role;
    m_state = BluetoothState::OFF;
    m_ble_hw_enabled = false;
    m_initialized = true;

    m_vcs_state.volume_setting = 77; /* Default ~30% volume */
    m_vcs_state.mute = 0;
    m_vcs_state.change_counter = 1;

    m_telemetry.volume_setting = 77;
    m_telemetry.volume_percent = 30;
    m_telemetry.is_muted = false;

    ESP_LOGI(TAG, "BleAudioBroadcast initialized in state OFF (Node Role: %d).", m_node_role);
    return ESP_OK;
}

void BleAudioBroadcast::setVolumeSetting(uint8_t vol, bool notify) {
    m_vcs_state.volume_setting = vol;
    m_vcs_state.change_counter++;
    m_telemetry.volume_setting = vol;
    m_telemetry.volume_percent = (vol * 100) / 255;

    if (notify && s_vcs_state_val_handle != 0) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&m_vcs_state, sizeof(VcsVolumeState));
        if (om) {
            ble_gatts_notify_custom(0, s_vcs_state_val_handle, om);
        }
    }
}

void BleAudioBroadcast::setMute(bool mute, bool notify) {
    m_vcs_state.mute = mute ? 1 : 0;
    m_vcs_state.change_counter++;
    m_telemetry.is_muted = mute;

    if (notify && s_vcs_state_val_handle != 0) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&m_vcs_state, sizeof(VcsVolumeState));
        if (om) {
            ble_gatts_notify_custom(0, s_vcs_state_val_handle, om);
        }
    }
}

esp_err_t BleAudioBroadcast::enableBluetoothHardware() {
    if (m_ble_hw_enabled) return ESP_OK;

    ESP_LOGI(TAG, "Configuring system parameters and enabling Bluetooth hardware...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize nimble port: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.reset_cb = onResetCb;
    ble_hs_cfg.sync_cb = onSyncCb;

    /* Set standard LE Audio Headphone Appearance (0x0841) */
    ble_svc_gap_device_appearance_set(BLE_GAP_APPEARANCE_HEADPHONES);
    ble_svc_gap_device_name_set(get_system_config()->device_name);

    /* Register full GATT Server (PACS, BASS, VCS) on SINK */
    if (m_node_role == NODE_ROLE_SINK) {
        registerGattServices();
    } else {
        ble_svc_gap_init();
        ble_svc_gatt_init();
    }

    /* Initialize LC3 Codec Engine */
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.initEncoder(AUDIO_SAMPLE_RATE_HZ, AUDIO_CHANNELS_NUM, AUDIO_FRAME_DURATION_MS * 1000, AUDIO_LC3_OCTETS_PER_FRAME);
    } else {
        m_lc3_codec.initDecoder(AUDIO_SAMPLE_RATE_HZ, AUDIO_CHANNELS_NUM, AUDIO_FRAME_DURATION_MS * 1000, AUDIO_LC3_OCTETS_PER_FRAME);
    }

    nimble_port_freertos_init(hostTaskRoutine);

#if defined(CONFIG_BT_NIMBLE_ISO)
    int iso_rc = ble_hs_iso_pkt_rx_cb_set(on_iso_packet_rx);
    ESP_LOGI(TAG, "SINK: Registered NimBLE Native ISO RX Packet Callback (rc = %d)", iso_rc);
#endif

    m_ble_hw_enabled = true;
    ESP_LOGI(TAG, "Bluetooth Hardware & NimBLE Host Stack successfully enabled.");
    return ESP_OK;
}

esp_err_t BleAudioBroadcast::disableBluetoothHardware() {
    if (!m_ble_hw_enabled) return ESP_OK;

    ESP_LOGI(TAG, "Disabling Bluetooth Hardware & NimBLE Host...");
    nimble_port_stop();
    nimble_port_deinit();
    m_ble_hw_enabled = false;
    return ESP_OK;
}

esp_err_t BleAudioBroadcast::transitionTo(BluetoothState target_state) {
    if (m_state == target_state) return ESP_OK;

    ESP_LOGI(TAG, "Bluetooth State Transition: [%s] ---> [%s]", 
             bluetoothStateToString(m_state), bluetoothStateToString(target_state));

    BluetoothState old_state = m_state;

    if (old_state == BluetoothState::OFF && target_state != BluetoothState::OFF) {
        esp_err_t err = enableBluetoothHardware();
        if (err != ESP_OK) return err;
    }

    switch (target_state) {
        case BluetoothState::OFF:
            disableBluetoothHardware();
            m_telemetry.is_synced = false;
            m_telemetry.source_name = "N/A";
            m_telemetry.rssi_dbm = 0;
            m_telemetry.packets_count = 0;
            break;

        case BluetoothState::SCANNING:
            m_telemetry.is_synced = false;
            m_telemetry.source_name = "SEARCHING...";
            m_telemetry.rssi_dbm = 0;
            m_telemetry.packets_count = 0;
            if (m_ble_hw_enabled && ble_hs_is_enabled()) {
                startScanning();
                if (m_node_role == NODE_ROLE_SINK) {
                    /* SINK: ensure connectable advertising is running */
                    ble_gap_ext_adv_start(1, 0, 0);
                }
            }
            break;

        case BluetoothState::STREAMING:
            m_telemetry.is_synced = true;
            break;

        case BluetoothState::BROADCASTING:
            m_telemetry.is_synced = true;
            break;
    }

    m_state = target_state;
    return ESP_OK;
}

void BleAudioBroadcast::notifyAdvReceived(const char* name, int8_t rssi) {
    if (name && name[0] != 0) m_telemetry.source_name = name;
    m_telemetry.rssi_dbm = rssi;
    m_last_adv_tick = xTaskGetTickCount();
}

void BleAudioBroadcast::parseAdvReport(const uint8_t* data, uint8_t length_data, int8_t rssi, const ble_addr_t* addr) {
    if (!data || length_data == 0) return;

    bool is_le_audio_broadcast = false;
    char found_name[48] = {0};

    size_t offset = 0;
    while (offset < length_data) {
        uint8_t ad_len = data[offset];
        if (ad_len == 0 || offset + 1 >= length_data) break;

        uint8_t ad_type = data[offset + 1];
        const uint8_t *ad_payload = &data[offset + 2];
        uint8_t available_payload = (length_data > offset + 2) ? (length_data - (offset + 2)) : 0;
        uint8_t payload_len = (ad_len > 0) ? (ad_len - 1) : 0;
        if (payload_len > available_payload) {
            payload_len = available_payload;
        }

        /* Extract Complete or Shortened Local Name (AD Types 0x09, 0x08) */
        if ((ad_type == 0x09 || ad_type == 0x08) && payload_len > 0) {
            size_t copy_len = (payload_len < sizeof(found_name) - 1) ? payload_len : (sizeof(found_name) - 1);
            memcpy(found_name, ad_payload, copy_len);
            found_name[copy_len] = 0;
        }

        /* Filter for 16-bit Service UUIDs (AD Types 0x02, 0x03): BASE (0x1851), BAA/PBA (0x1852), BASS (0x184F), PACS (0x184E), PBA (0x1856) */
        if ((ad_type == 0x02 || ad_type == 0x03) && payload_len >= 2) {
            for (size_t i = 0; i + 1 < payload_len; i += 2) {
                uint16_t uuid = ad_payload[i] | (ad_payload[i + 1] << 8);
                if (uuid == 0x1851 || uuid == 0x1852 || uuid == 0x184F || uuid == 0x184E || uuid == 0x1856 || uuid == 0x1850) {
                    is_le_audio_broadcast = true;
                }
            }
        }

        /* Filter for Service Data (AD Type 0x16): BASE (0x1851), BAA/PBA (0x1852), BASS (0x184F), PBA (0x1856) */
        if (ad_type == 0x16 && payload_len >= 2) {
            uint16_t uuid = ad_payload[0] | (ad_payload[1] << 8);
            if (uuid == 0x1851 || uuid == 0x1852 || uuid == 0x184F || uuid == 0x1856 || uuid == 0x1850) {
                is_le_audio_broadcast = true;
            }
        }

        /* Filter for Manufacturer Specific Data (AD Type 0xFF, Company ID 0x02E5) */
        if (ad_type == 0xFF && payload_len >= 2) {
            uint16_t company_id = ad_payload[0] | (ad_payload[1] << 8);
            if (company_id == 0x02E5) {
                is_le_audio_broadcast = true;
            }
        }

        offset += (1 + ad_len);
    }

    if (is_le_audio_broadcast) {
        notifyAdvReceived(found_name[0] != 0 ? found_name : "Auracast Stream", rssi);
    }
}

void BleAudioBroadcast::startScanning() {
    ESP_LOGI(TAG, "Starting BLE Extended Discovery (Node Role: %d)...", m_node_role);

    struct ble_gap_ext_disc_params uncoded_params = {};
    uncoded_params.itvl = 160;   /* 100 ms */
    uncoded_params.window = 160; /* 100 ms continuous scan */
    uncoded_params.passive = (m_node_role == NODE_ROLE_SINK) ? 1 : 0; /* Active scan for Central */

    int rc = ble_gap_ext_disc(BLE_OWN_ADDR_PUBLIC, 0, 0, 0, 0, 0, &uncoded_params, nullptr, ble_gap_event_cb, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Failed to start ble_gap_ext_disc, rc = %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE Extended Discovery Active.");
    }
}

void BleAudioBroadcast::checkSyncState() {
    if (m_node_role == NODE_ROLE_SINK) {
        if (m_state == BluetoothState::STREAMING) {
            TickType_t now = xTaskGetTickCount();
            if ((now - m_last_adv_tick) > pdMS_TO_TICKS(AUDIO_SYNC_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "Node20: Broadcast Sync Lost (Source Inactive/Powered Off). Transitioning to SCANNING...");
                transitionTo(BluetoothState::SCANNING);
            }
        }
    }
}

void BleAudioBroadcast::onResetCb(int reason) {
    ESP_LOGE(TAG, "NimBLE Host Reset! Reason = %d", reason);
}

void BleAudioBroadcast::onSyncCb(void) {
    ESP_LOGI(TAG, "NimBLE Host Synced.");
    if (s_broadcast_instance) {
        if (s_broadcast_instance->m_node_role == NODE_ROLE_SOURCE) {
            ESP_LOGI(TAG, "Node21: Starting BLE 5.3 Audio Broadcast & PBP Announcement...");

            struct ble_gap_ext_adv_params ext_params = {};
            ext_params.legacy_pdu = 0;
            ext_params.connectable = 0;
            ext_params.scannable = 0;
            ext_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
            ext_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;
            ext_params.primary_phy = BLE_HCI_LE_PHY_1M;
            ext_params.secondary_phy = BLE_HCI_LE_PHY_1M;
            ext_params.sid = 1;
            ext_params.own_addr_type = BLE_OWN_ADDR_PUBLIC;

            int rc = ble_gap_ext_adv_configure(0, &ext_params, nullptr, ble_gap_event_cb, nullptr);
            if (rc != 0) ESP_LOGE(TAG, "ble_gap_ext_adv_configure failed: %d", rc);

            uint8_t raw_adv[64] = {0};
            size_t idx = 0;
            raw_adv[idx++] = 0x02; raw_adv[idx++] = 0x01; raw_adv[idx++] = 0x06;

            const char* name_str = "ESP32-C6-21";
            uint8_t nlen = static_cast<uint8_t>(strlen(name_str));
            raw_adv[idx++] = nlen + 1; raw_adv[idx++] = 0x09;
            memcpy(&raw_adv[idx], name_str, nlen);
            idx += nlen;

            raw_adv[idx++] = 0x03; raw_adv[idx++] = 0x03; raw_adv[idx++] = 0x52; raw_adv[idx++] = 0x18;

            struct os_mbuf *data = os_msys_get_pkthdr(idx, 0);
            if (data) {
                os_mbuf_append(data, raw_adv, idx);
                ble_gap_ext_adv_set_data(0, data);
            }

            ble_gap_ext_adv_start(0, 0, 0);
            ESP_LOGI(TAG, "Node21: BLE Extended Audio Broadcaster Active with Name: %s", name_str);

            /* SOURCE also starts scanning to discover SINK nodes */
            s_broadcast_instance->startScanning();
        } else {
            /* SINK Node: Start Extended Connectable Advertising to be discoverable by SOURCE */
            struct ble_gap_ext_adv_params adv_params = {};
            adv_params.legacy_pdu = 1;
            adv_params.connectable = 1;
            adv_params.scannable = 1;
            adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
            adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;
            adv_params.primary_phy = BLE_HCI_LE_PHY_1M;
            adv_params.secondary_phy = BLE_HCI_LE_PHY_1M;
            adv_params.sid = 2;
            adv_params.own_addr_type = BLE_OWN_ADDR_PUBLIC;

            ble_gap_ext_adv_configure(1, &adv_params, nullptr, ble_gap_event_cb, nullptr);

            uint8_t adv_data[64] = {0};
            size_t a_idx = 0;
            adv_data[a_idx++] = 0x02; adv_data[a_idx++] = 0x01; adv_data[a_idx++] = 0x06;
            
            /* Appearance: 0x0841 (Stereo Headphones) */
            adv_data[a_idx++] = 0x03; adv_data[a_idx++] = 0x19; adv_data[a_idx++] = 0x41; adv_data[a_idx++] = 0x08;

            const char* sink_name = get_system_config()->device_name;
            uint8_t slen = static_cast<uint8_t>(strlen(sink_name));
            adv_data[a_idx++] = slen + 1; adv_data[a_idx++] = 0x09;
            memcpy(&adv_data[a_idx], sink_name, slen);
            a_idx += slen;

            /* Service UUIDs: PACS (0x184E), BASS (0x184F), VCS (0x1844) */
            adv_data[a_idx++] = 0x07; adv_data[a_idx++] = 0x03;
            adv_data[a_idx++] = 0x4E; adv_data[a_idx++] = 0x18;
            adv_data[a_idx++] = 0x4F; adv_data[a_idx++] = 0x18;
            adv_data[a_idx++] = 0x44; adv_data[a_idx++] = 0x18;

            struct os_mbuf *s_data = os_msys_get_pkthdr(a_idx, 0);
            if (s_data) {
                os_mbuf_append(s_data, adv_data, a_idx);
                ble_gap_ext_adv_set_data(1, s_data);
            }

            ble_gap_ext_adv_start(1, 0, 0);
            ESP_LOGI(TAG, "Node20: SINK GATT Server Advertising (Appearance: Stereo Headphones 0x0841, Services: PACS/BASS/VCS)");

            s_broadcast_instance->startScanning();
        }
    }
}

void BleAudioBroadcast::hostTaskRoutine(void* param) {
    ESP_LOGI(TAG, "NimBLE Host Task Started.");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void BleAudioBroadcast::startAudioTask() {
    if (!m_audio_task_running) {
        m_audio_task_running = true;
        xTaskCreate(audioTaskRoutine, "ble_audio_task", AUDIO_TASK_STACK_SIZE, this, 3, &s_audio_task_handle);
    }

    if (m_node_role == NODE_ROLE_SOURCE && !m_vcs_task_running) {
        m_vcs_task_running = true;
        xTaskCreate(vcsOscillatorTaskRoutine, "vcs_lfo_task", 3072, this, 2, nullptr);
    }
}

void BleAudioBroadcast::audioTaskRoutine(void* param) {
    auto* self = static_cast<BleAudioBroadcast*>(param);
    if (self->m_node_role == NODE_ROLE_SOURCE) {
        self->runSourceLoop();
    } else {
        self->runSinkLoop();
    }
}

void BleAudioBroadcast::vcsOscillatorTaskRoutine(void* param) {
    auto* self = static_cast<BleAudioBroadcast*>(param);
    self->runVcsOscillatorLoop();
}


void BleAudioBroadcast::runSourceLoop() {
    ESP_LOGI(TAG, "Node21: Audio Source Processing Loop Started (10 ms Frame Interval)...");

    static int16_t pcm_buffer[AUDIO_SAMPLES_PER_FRAME] = {0};
    static uint8_t lc3_buffer[AUDIO_LC3_OCTETS_PER_FRAME] = {0};
    size_t actual_encoded_bytes = 0;

    const TickType_t interval = pdMS_TO_TICKS(AUDIO_FRAME_DURATION_MS);

    while (m_audio_task_running) {
        if (m_state == BluetoothState::BROADCASTING) {
            if (m_tone_gen) {
                m_tone_gen->generateFrame(pcm_buffer, AUDIO_SAMPLES_PER_FRAME);
            }
            m_lc3_codec.encodeFrame(pcm_buffer, AUDIO_SAMPLES_PER_FRAME, lc3_buffer, sizeof(lc3_buffer), &actual_encoded_bytes);
            m_telemetry.packets_count++;

            /* Fast inline single-pass Peak & RMS computation via audio_metering component */
            m_audio_meter.pushFramePcm(pcm_buffer, AUDIO_SAMPLES_PER_FRAME);
        }
        vTaskDelay(interval > 0 ? interval : 1);
    }
}

void BleAudioBroadcast::runSinkLoop() {
    ESP_LOGI(TAG, "Node20: Audio Sink Processing Loop Started (Live BLE5.3 LC3 Stream Decoder)...");

    static uint8_t current_lc3_buf[256] = {0};
    static int16_t decoded_pcm[AUDIO_SAMPLES_PER_FRAME] = {0};
    static int16_t stereo_pcm[AUDIO_SAMPLES_PER_FRAME * 2] = {0};
    size_t actual_samples = 0;
    size_t bytes_written = 0;

    enum class SinkDmaState {
        IDLE_WAIT_FRAME_0,
        PRIMING_WAIT_FRAME_1,
        STREAMING_ACTIVE
    };

    SinkDmaState dma_state = SinkDmaState::IDLE_WAIT_FRAME_0;
    uint32_t system_plc_consecutive_count = 0; // SYSTEM watchdog counter (resets on frame ready, triggers IDLE at 5)

    while (m_audio_task_running) {
        if (m_state == BluetoothState::STREAMING) {

            // --- STATE 1: IDLE, WAITING FOR WATERMARK CUSHION (6 frames = 60ms) ---
            if (dma_state == SinkDmaState::IDLE_WAIT_FRAME_0) {
                // Ensure at least 6 frames in queue before starting (absorbs Python HCI UART & RF beat jitter)
                taskENTER_CRITICAL(&s_lc3_rx_mux);
                size_t ready_count = s_rx_fifo_count;
                taskEXIT_CRITICAL(&s_lc3_rx_mux);

                if (ready_count < 6) {
                    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(40));
                    continue;
                }

                size_t current_lc3_len = 0;
                bool has_packet = pop_rx_lc3_frame(current_lc3_buf, &current_lc3_len);

                if (has_packet && current_lc3_len >= 20) {
                    system_plc_consecutive_count = 0;
                    m_telemetry.packets_count++;
                    m_lc3_codec.decodeFrame(current_lc3_buf, current_lc3_len, decoded_pcm, AUDIO_SAMPLES_PER_FRAME, &actual_samples);
                    m_audio_meter.pushFramePcm(decoded_pcm, actual_samples);

                    uint8_t vol_scale = (m_vcs_state.mute != 0) ? 0 : m_vcs_state.volume_setting;
                    if (m_telemetry.channels == 1) {
                        for (size_t i = 0; i < actual_samples; ++i) {
                            int32_t scaled = (static_cast<int32_t>(decoded_pcm[i]) * vol_scale) / 255;
                            int16_t sample = static_cast<int16_t>(scaled);
                            stereo_pcm[i * 2] = sample;     // Left Channel
                            stereo_pcm[i * 2 + 1] = sample; // Right Channel (Mono duplicated)
                        }
                    } else {
                        for (size_t i = 0; i < actual_samples; ++i) {
                            int32_t scaled = (static_cast<int32_t>(decoded_pcm[i]) * vol_scale) / 255;
                            int16_t sample = static_cast<int16_t>(scaled);
                            stereo_pcm[i * 2] = sample;
                            stereo_pcm[i * 2 + 1] = sample;
                        }
                    }

                    if (m_i2s_dac) {
                        m_i2s_dac->preload(stereo_pcm, actual_samples * 2, &bytes_written);
                    }
                    dma_state = SinkDmaState::PRIMING_WAIT_FRAME_1;
                }
                continue;
            }

            // --- STATE 2: PRIMING, WAITING FOR SECOND FRAME (DESCRIPTOR 1) ---
            if (dma_state == SinkDmaState::PRIMING_WAIT_FRAME_1) {
                size_t current_lc3_len = 0;
                bool has_packet = pop_rx_lc3_frame(current_lc3_buf, &current_lc3_len);
                if (!has_packet) {
                    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
                    has_packet = pop_rx_lc3_frame(current_lc3_buf, &current_lc3_len);
                }

                if (has_packet && current_lc3_len >= 20) {
                    system_plc_consecutive_count = 0; // Valid frame: Reset SYSTEM PLC counter
                    m_telemetry.packets_count++;
                    m_lc3_codec.decodeFrame(current_lc3_buf, current_lc3_len, decoded_pcm, AUDIO_SAMPLES_PER_FRAME, &actual_samples);
                    m_audio_meter.pushFramePcm(decoded_pcm, actual_samples);

                    uint8_t vol_scale = (m_vcs_state.mute != 0) ? 0 : m_vcs_state.volume_setting;
                    if (m_telemetry.channels == 1) {
                        for (size_t i = 0; i < actual_samples; ++i) {
                            int32_t scaled = (static_cast<int32_t>(decoded_pcm[i]) * vol_scale) / 255;
                            int16_t sample = static_cast<int16_t>(scaled);
                            stereo_pcm[i * 2] = sample;
                            stereo_pcm[i * 2 + 1] = sample;
                        }
                    } else {
                        for (size_t i = 0; i < actual_samples; ++i) {
                            int32_t scaled = (static_cast<int32_t>(decoded_pcm[i]) * vol_scale) / 255;
                            int16_t sample = static_cast<int16_t>(scaled);
                            stereo_pcm[i * 2] = sample;
                            stereo_pcm[i * 2 + 1] = sample;
                        }
                    }

                    if (m_i2s_dac) {
                        m_i2s_dac->preload(stereo_pcm, actual_samples * 2, &bytes_written);
                        // BOTH DESCRIPTORS (20ms) ARE NOW FULL! FIRST NOW START THE I2S CLOCK!
                        m_i2s_dac->start();
                    }
                    dma_state = SinkDmaState::STREAMING_ACTIVE;
                }
                continue;
            }

            // --- STATE 3: STREAMING ACTIVE (INTERRUPT-DRIVEN CONSUMER PULL) ---
            if (dma_state == SinkDmaState::STREAMING_ACTIVE) {
                // Wait until 1 DMA descriptor has completed playing (triggered by DMA on_sent ISR)
                if (m_i2s_dac && m_i2s_dac->isRunning()) {
                    m_i2s_dac->waitForDmaSlot(25);
                }

                // Pull next frame from FIFO
                size_t current_lc3_len = 0;
                bool has_packet = pop_rx_lc3_frame(current_lc3_buf, &current_lc3_len);

                if (has_packet && current_lc3_len >= 20) {
                    system_plc_consecutive_count = 0; // [ Frame Ready ]: Reset SYSTEM PLC watchdog counter!
                    m_telemetry.packets_count++;
                    m_lc3_codec.decodeFrame(current_lc3_buf, current_lc3_len, decoded_pcm, AUDIO_SAMPLES_PER_FRAME, &actual_samples);
                    m_audio_meter.pushFramePcm(decoded_pcm, actual_samples);

                    uint8_t vol_scale = (m_vcs_state.mute != 0) ? 0 : m_vcs_state.volume_setting;
                    if (m_telemetry.channels == 1) {
                        for (size_t i = 0; i < actual_samples; ++i) {
                            int32_t scaled = (static_cast<int32_t>(decoded_pcm[i]) * vol_scale) / 255;
                            int16_t sample = static_cast<int16_t>(scaled);
                            stereo_pcm[i * 2] = sample;     // Left Channel
                            stereo_pcm[i * 2 + 1] = sample; // Right Channel (Mono duplicated)
                        }
                    } else {
                        for (size_t i = 0; i < actual_samples; ++i) {
                            int32_t scaled = (static_cast<int32_t>(decoded_pcm[i]) * vol_scale) / 255;
                            int16_t sample = static_cast<int16_t>(scaled);
                            stereo_pcm[i * 2] = sample;
                            stereo_pcm[i * 2 + 1] = sample;
                        }
                    }

                    if (m_i2s_dac && m_i2s_dac->isInitialized()) {
                        m_i2s_dac->write(stereo_pcm, actual_samples * 2, &bytes_written, 20);
                    }
                } else {
                    // FIFO was empty: increment both SYSTEM consecutive counter and STATISTICAL counter
                    s_fifo_underrun_count.fetch_add(1, std::memory_order_relaxed);
                    system_plc_consecutive_count++; // SYSTEM counter: consecutive missing frame count

                    if (system_plc_consecutive_count < 5) {
                        // Synthesize up to 4 consecutive PLC frames (up to 40 ms)
                        m_lc3_codec.decodeFrame(nullptr, 0, decoded_pcm, AUDIO_SAMPLES_PER_FRAME, &actual_samples);
                        uint8_t vol_scale = (m_vcs_state.mute != 0) ? 0 : m_vcs_state.volume_setting;
                        for (size_t i = 0; i < AUDIO_SAMPLES_PER_FRAME; ++i) {
                            int32_t scaled = (static_cast<int32_t>(decoded_pcm[i]) * vol_scale) / 255;
                            int16_t sample = static_cast<int16_t>(scaled);
                            stereo_pcm[i * 2] = sample;
                            stereo_pcm[i * 2 + 1] = sample;
                        }
                        if (m_i2s_dac && m_i2s_dac->isInitialized()) {
                            m_i2s_dac->write(stereo_pcm, AUDIO_SAMPLES_PER_FRAME * 2, &bytes_written, 20);
                        }
                    } else {
                        // Reached 5 consecutive PLC frames (50 ms of missing audio):
                        // Put audio pipeline into IDLE immediately, stop I2S clock, and mute sound!
                        ESP_LOGW(TAG, "SINK: Reached 5 consecutive PLC frames. Setting audio pipeline to IDLE / Muted...");
                        m_audio_meter.pushSilence();
                        if (m_i2s_dac) {
                            m_i2s_dac->stop(); // Gated: Stops I2S clock and enters ultra-low power shutdown
                        }
                        s_is_periodic_synced = false;
                        dma_state = SinkDmaState::IDLE_WAIT_FRAME_0;
                        system_plc_consecutive_count = 0;
                        transitionTo(BluetoothState::SCANNING);
                        startScanning();
                    }
                }
            }
        } else {
            // Standby / Scanning: Keep I2S clock stopped and reset state
            if (m_i2s_dac && m_i2s_dac->isRunning()) {
                m_i2s_dac->stop();
            }
            dma_state = SinkDmaState::IDLE_WAIT_FRAME_0;
            system_plc_consecutive_count = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void BleAudioBroadcast::runVcsOscillatorLoop() {
    ESP_LOGI(TAG, "Node21: 0.33 Hz Sine VCS Oscillator Task Started (3 Hz Modulation Rate)...");

    const TickType_t interval = pdMS_TO_TICKS(VCS_UPDATE_INTERVAL_MS); // 333 ms
    uint32_t step_count = 0;

    while (m_vcs_task_running) {
        step_count++;
        float t = static_cast<float>(step_count) * (1.0f / VCS_UPDATE_RATE_HZ);
        
        /* 0.33 Hz Sine Wave: Period = 1.0 / 0.33 = ~3.03 seconds */
        float phase = 2.0f * M_PI * VCS_SINE_MOD_FREQ_HZ * t;
        float norm_sine = (sinf(phase) + 1.0f) * 0.5f; /* 0.0 to 1.0 */
        float vol_pct = VCS_VOLUME_MIN_PCT + norm_sine * (VCS_VOLUME_MAX_PCT - VCS_VOLUME_MIN_PCT); /* 10% to 50% */
        uint8_t vol_units = static_cast<uint8_t>((vol_pct * 255.0f) / 100.0f + 0.5f); /* ~25 to 128 */

        /* Transmit VCS Absolute Volume to all connected SINKs */
        for (auto& sink : m_tracked_sinks) {
            if (sink.connected && sink.vcs_cp_handle != 0) {
                sendVcsVolumeToSink(sink, vol_units);
            }
        }

        vTaskDelay(interval > 0 ? interval : 1);
    }
}

uint32_t BleAudioBroadcast::getAndResetFifoUnderrunCount() {
    return s_fifo_underrun_count.exchange(0, std::memory_order_relaxed);
}

uint32_t BleAudioBroadcast::getAndResetFifoOverflowCount() {
    return s_fifo_overflow_count.exchange(0, std::memory_order_relaxed);
}

uint32_t BleAudioBroadcast::getFifoUnderrunCount() const {
    return s_fifo_underrun_count.load(std::memory_order_relaxed);
}

uint32_t BleAudioBroadcast::getFifoOverflowCount() const {
    return s_fifo_overflow_count.load(std::memory_order_relaxed);
}

uint32_t BleAudioBroadcast::getAndResetMbufUnderrunCount() {
    return s_mbuf_underrun_count.exchange(0, std::memory_order_relaxed);
}

uint32_t BleAudioBroadcast::getAndResetMbufOverflowCount() {
    return s_mbuf_overflow_count.exchange(0, std::memory_order_relaxed);
}

uint32_t BleAudioBroadcast::getMbufUnderrunCount() const {
    return s_mbuf_underrun_count.load(std::memory_order_relaxed);
}

uint32_t BleAudioBroadcast::getMbufOverflowCount() const {
    return s_mbuf_overflow_count.load(std::memory_order_relaxed);
}

} // namespace Bluetooth
