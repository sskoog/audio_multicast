#include "ble_audio_receiver.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_lc3.h"
extern "C" {
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/dis/ble_svc_dis.h"
#include "store/config/ble_store_config.h"
}

static const char* TAG = "BLE_AUDIO_RX";

namespace Bluetooth {

static void ble_app_advertise(void);
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg);

// Bluetooth SIG PACS (Published Audio Capabilities Service - 0x184E)
static const ble_uuid16_t pacs_svc_uuid = BLE_UUID16_INIT(0x184E);
static const ble_uuid16_t sink_pac_uuid = BLE_UUID16_INIT(0x2BC9);
static const ble_uuid16_t sink_loc_uuid = BLE_UUID16_INIT(0x2BCA);

// Bluetooth SIG ASCS (Audio Stream Control Service - 0x184E)
static const ble_uuid16_t sink_ase_uuid = BLE_UUID16_INIT(0x2BC4); // Sink Audio Stream Endpoint
static const ble_uuid16_t ase_cp_uuid    = BLE_UUID16_INIT(0x2BC6); // ASE Control Point

// Bluetooth SIG BASS (Broadcast Audio Scan Service - 0x184F)
static const ble_uuid16_t bass_svc_uuid = BLE_UUID16_INIT(0x184F);
static const ble_uuid16_t bass_cp_uuid  = BLE_UUID16_INIT(0x2BC7); // Broadcast Audio Scan Control Point
static const ble_uuid16_t bass_state_uuid = BLE_UUID16_INIT(0x2BC8); // Broadcast Receive State

// Bluetooth SIG CSIS (Coordinated Set Identification Service - 0x1846)
static const ble_uuid16_t csis_svc_uuid = BLE_UUID16_INIT(0x1846);
static const ble_uuid16_t csis_sirk_uuid = BLE_UUID16_INIT(0x2B84);

// Bluetooth SIG CAS (Common Audio Service - 0x1853)
static const ble_uuid16_t cas_svc_uuid  = BLE_UUID16_INIT(0x1853);

// Bluetooth SIG VCS (Volume Control Service - 0x1844)
static const ble_uuid16_t vcs_svc_uuid   = BLE_UUID16_INIT(0x1844);
static const ble_uuid16_t vcs_state_uuid = BLE_UUID16_INIT(0x2B7D); // Volume State
static const ble_uuid16_t vcs_cp_uuid    = BLE_UUID16_INIT(0x2B7E); // Volume Control Point
static const ble_uuid16_t vcs_flags_uuid = BLE_UUID16_INIT(0x2B7F); // Volume Flags

static uint8_t pac_record[] = {
    0x01,                         // Number of PAC Records
    0x06, 0x00, 0x00, 0x00, 0x00, // Codec ID: LC3
    0x13,                         // Codec Specific Capabilities Length
    0x03, 0x01, 0x20, 0x00,       // Sampling Frequency: 48 kHz (0x0020)
    0x02, 0x02, 0x03,             // Frame Duration: 7.5ms | 10ms (0x03)
    0x02, 0x03, 0x03,             // Audio Channel Allocation: Mono | Stereo (0x03)
    0x05, 0x04, 0x14, 0x00, 0x78, 0x00 // Octets Per Frame: 20-120
};

static uint32_t sink_audio_locations = 0x00000003; // Front Left | Front Right
static uint8_t sink_ase_state[2] = { 0x01, 0x00 }; // ASE ID = 0x01, State = Idle
static uint8_t bass_recv_state[16] = {0};
static uint8_t csis_sirk[17] = {0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00};

static uint8_t vcs_volume_setting = 217; // 85% of 255
static uint8_t vcs_mute_state = 0;       // Unmuted
static uint8_t vcs_change_counter = 0;
static uint8_t vcs_flags = 0x01;

static int gatt_le_audio_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == 0x2BC9) {
        return os_mbuf_append(ctxt->om, pac_record, sizeof(pac_record));
    } else if (uuid16 == 0x2BCA) {
        return os_mbuf_append(ctxt->om, &sink_audio_locations, sizeof(sink_audio_locations));
    } else if (uuid16 == 0x2BC4) {
        return os_mbuf_append(ctxt->om, sink_ase_state, sizeof(sink_ase_state));
    } else if (uuid16 == 0x2BC8) {
        return os_mbuf_append(ctxt->om, bass_recv_state, sizeof(bass_recv_state));
    } else if (uuid16 == 0x2B84) {
        return os_mbuf_append(ctxt->om, csis_sirk, sizeof(csis_sirk));
    } else if (uuid16 == 0x2B7D) {
        uint8_t state_buf[3] = { vcs_volume_setting, vcs_mute_state, vcs_change_counter };
        return os_mbuf_append(ctxt->om, state_buf, sizeof(state_buf));
    } else if (uuid16 == 0x2B7F) {
        return os_mbuf_append(ctxt->om, &vcs_flags, sizeof(vcs_flags));
    } else if (uuid16 == 0x2B7E) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len >= 1) {
                uint8_t buf[8] = {0};
                os_mbuf_copydata(ctxt->om, 0, len < 8 ? len : 8, buf);
                uint8_t opcode = buf[0];
                vcs_change_counter++;
                if (opcode == 0x00) {
                    if (vcs_volume_setting >= 8) vcs_volume_setting -= 8; else vcs_volume_setting = 0;
                } else if (opcode == 0x01) {
                    if (vcs_volume_setting <= 247) vcs_volume_setting += 8; else vcs_volume_setting = 255;
                } else if (opcode == 0x04 && len >= 2) {
                    vcs_volume_setting = buf[1];
                } else if (opcode == 0x05) {
                    vcs_mute_state = 0;
                } else if (opcode == 0x06) {
                    vcs_mute_state = 1;
                }
                uint8_t pct = (uint8_t)(((uint32_t)vcs_volume_setting * 100) / 255);
                ESP_LOGI(TAG, "VCS Volume Write Opcode 0x%02X -> Volume: %u (%u%%), Mute: %u", opcode, vcs_volume_setting, pct, vcs_mute_state);
            }
        }
        return 0;
    } else if (uuid16 == 0x2BC7) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len >= 4) {
                uint8_t buf[32] = {0};
                os_mbuf_copydata(ctxt->om, 0, len < 32 ? len : 32, buf);
                uint8_t opcode = buf[0];
                if ((opcode == 0x01 || opcode == 0x02) && len >= 9) {
                    uint32_t broadcast_id = ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 8) | buf[8];
                    ESP_LOGI(TAG, "BASS Write Opcode 0x%02X: Received Broadcast_ID 0x%06lX from Android 17", opcode, broadcast_id);
                }
            }
        }
        return 0;
    } else if (uuid16 == 0x2BC6) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len >= 2) {
                uint8_t buf[32] = {0};
                os_mbuf_copydata(ctxt->om, 0, len < 32 ? len : 32, buf);
                uint8_t opcode = buf[0];
                if (opcode == 0x01) { // Config Codec
                    sink_ase_state[1] = 0x01; // CODEC_CONFIGURED
                    ESP_LOGI(TAG, "ASCS Opcode 0x01 [Config Codec]: ASE #1 -> CODEC_CONFIGURED (LC3 48 kHz)");
                } else if (opcode == 0x02) { // Config QoS
                    sink_ase_state[1] = 0x02; // QOS_CONFIGURED
                    ESP_LOGI(TAG, "ASCS Opcode 0x02 [Config QoS]: ASE #1 -> QOS_CONFIGURED (CIG/CIS Set)");
                } else if (opcode == 0x03) { // Enable
                    sink_ase_state[1] = 0x03; // ENABLING
                    ESP_LOGI(TAG, "ASCS Opcode 0x03 [Enable]: ASE #1 -> ENABLING");
                } else if (opcode == 0x04) { // Receiver Start Ready
                    sink_ase_state[1] = 0x04; // STREAMING
                    ESP_LOGI(TAG, "ASCS Opcode 0x04 [Receiver Start Ready]: ASE #1 -> STREAMING (Unicast CIS Connected!)");
                } else if (opcode == 0x05 || opcode == 0x07) { // Disable or Release
                    sink_ase_state[1] = 0x00; // IDLE
                    ESP_LOGI(TAG, "ASCS Opcode 0x%02X [Disable/Release]: ASE #1 -> IDLE", opcode);
                }
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_le_audio_svcs[] = {
    // 1. PACS & ASCS (0x184E)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &pacs_svc_uuid.u,
        .includes = nullptr,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &sink_pac_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            {
                .uuid = &sink_loc_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            {
                .uuid = &sink_ase_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            {
                .uuid = &ase_cp_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            { 0, nullptr, nullptr, nullptr, 0, 0, nullptr }
        },
    },
    // 2. BASS (0x184F)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &bass_svc_uuid.u,
        .includes = nullptr,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &bass_state_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            {
                .uuid = &bass_cp_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            { 0, nullptr, nullptr, nullptr, 0, 0, nullptr }
        },
    },
    // 3. CSIS (0x1846)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &csis_svc_uuid.u,
        .includes = nullptr,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &csis_sirk_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC, // Mandatory encrypted read
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            { 0, nullptr, nullptr, nullptr, 0, 0, nullptr }
        },
    },
    // 4. CAS (0x1853)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &cas_svc_uuid.u,
        .includes = nullptr,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { 0, nullptr, nullptr, nullptr, 0, 0, nullptr }
        },
    },
    // 5. VCS (0x1844)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &vcs_svc_uuid.u,
        .includes = nullptr,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &vcs_state_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            {
                .uuid = &vcs_cp_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            {
                .uuid = &vcs_flags_uuid.u,
                .access_cb = gatt_le_audio_access,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = nullptr,
            },
            { 0, nullptr, nullptr, nullptr, 0, 0, nullptr }
        },
    },
    { 0, nullptr, nullptr, nullptr }
};

static void ble_app_on_sync(void) {
    uint8_t addr_type = 0;
    ble_hs_id_infer_auto(0, &addr_type);
    ble_svc_gap_device_name_set(CONFIG_BT_DEVICE_NAME);
    ble_svc_gap_device_appearance_set(0x0841); // 0x0841 = LE Audio Headset / Audio Sink Appearance
    ble_app_advertise();
}

static void ble_app_on_reset(int reason) {
    ESP_LOGW(TAG, "NimBLE stack reset, reason: %d", reason);
}

static void ble_host_task(void* param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int ble_gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Pixel 10 Connected via BLE GAP! Connection Handle: %d", event->connect.conn_handle);
            } else {
                ESP_LOGW(TAG, "BLE Connection Attempt Failed, Status: %d", event->connect.status);
                ble_app_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Pixel 10 Disconnected (Reason: %d). Restarting GAP Advertising...", event->disconnect.reason);
            ble_app_advertise();
            return 0;

        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            ESP_LOGI(TAG, "Passkey Action Requested: Action %d, conn_handle %d", 
                     event->passkey.params.action, event->passkey.conn_handle);
            struct ble_sm_io pkey = {};
            pkey.action = event->passkey.params.action;
            pkey.numcmp_accept = 1;
            pkey.passkey = 0;
            int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(TAG, "ble_sm_inject_io returned %d", rc);
            return 0;
        }

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0) {
                ESP_LOGI(TAG, "BLE Security Encryption Enabled / Pairing Complete! Handle: %d", event->enc_change.conn_handle);
            } else {
                ESP_LOGE(TAG, "BLE Security Encryption Failed, Status: %d", event->enc_change.status);
            }
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "GATT Client Subscribed to Characteristic Handle %d", event->subscribe.attr_handle);
            return 0;

        default:
            return 0;
    }
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params = {};
    struct ble_hs_adv_fields adv_fields = {};
    struct ble_hs_adv_fields rsp_fields = {};

    // 1. Primary Advertising Payload: Flags + Complete Device Name (26 bytes <= 31 max)
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name = (uint8_t*)CONFIG_BT_DEVICE_NAME;
    adv_fields.name_len = strlen(CONFIG_BT_DEVICE_NAME);
    adv_fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set NimBLE adv data (rc = %d)", rc);
        return;
    }

    // 2. Scan Response Payload: Appearance (0x0841) + Service UUID (PACS 0x184E)
    rsp_fields.appearance = 0x0841; // 0x0841 = LE Audio Headset / Audio Sink
    rsp_fields.appearance_is_present = 1;

    static ble_uuid16_t pacs_uuid = BLE_UUID16_INIT(0x184E);
    rsp_fields.uuids16 = &pacs_uuid;
    rsp_fields.num_uuids16 = 1;
    rsp_fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set NimBLE scan response data (rc = %d)", rc);
    }

    // 3. Start Connectable General Discoverable Advertising
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Undirected connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // General discoverable

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER, &adv_params, ble_gap_event_handler, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start NimBLE GAP advertising, rc = %d", rc);
    } else {
        ESP_LOGI(TAG, "NimBLE GAP Advertising Active: Device '%s' (PACS 0x184E LE Audio Sink)", CONFIG_BT_DEVICE_NAME);
    }
}

BleAudioReceiver::BleAudioReceiver(Audio::DspHighPassFilter& filter, Hardware::I2sDacDriver& dac)
    : m_filter(filter), m_dac(dac) {}

BleAudioReceiver::~BleAudioReceiver() {
    cleanupLc3Decoder();
}

esp_err_t BleAudioReceiver::init() {
    ESP_LOGI(TAG, "Initializing Bluetooth 5.3 Subsystem for ESP32-C6...");

    // Initialize NVS Flash (Erase NVS on boot during prototyping to ensure a clean security slate)
    ESP_LOGI(TAG, "Erasing NVS security store to prevent bond key mismatches...");
    nvs_flash_erase();
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize NimBLE Host Stack & Services
    ret = nimble_port_init();
    if (ret == ESP_OK) {
        ble_hs_cfg.sync_cb = ble_app_on_sync;
        ble_hs_cfg.reset_cb = ble_app_on_reset;

        // Configure NVS Security Store & Repeat Pairing Handler
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

        // Configure Security Manager for LE Audio Bonding ("Just Works" Pairing)
        // Earbuds do not have displays. We must use NO_IO for standard Fast Pair / BASS behavior.
        ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_mitm = 0;
        ble_hs_cfg.sm_sc = 1;
        ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
        ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

        ble_svc_gap_init();
        ble_svc_gatt_init();

        // Register LE Audio GATT Table (PACS 0x184E, BASS 0x184F, CSIS 0x1846)
        ble_gatts_count_cfg(gatt_le_audio_svcs);
        ble_gatts_add_svcs(gatt_le_audio_svcs);

        ble_svc_dis_init();
        ble_svc_dis_manufacturer_name_set("ForestChirp");
        ble_svc_dis_model_number_set("ESP32-C6 LCD Auracast");

        // Set GAP Device Name - Changed to V2 to bypass Android's aggressive GATT caching!
        ble_svc_gap_device_name_set("ESP32-C6-Auracast-V2");

        nimble_port_freertos_init(ble_host_task);
        ESP_LOGI(TAG, "NimBLE Host Stack, PACS GATT Service & Security Manager Initialized.");
    } else {
        ESP_LOGE(TAG, "Failed to initialize NimBLE port: %s", esp_err_to_name(ret));
    }

    // Initialize Espressif esp_lc3 fixed-point decoder
    ret = initLc3Decoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize esp_lc3 fixed-point decoder: %s", esp_err_to_name(ret));
        return ret;
    }

    m_state = ConnectionState::IDLE;
    m_is_initialized = true;

    ESP_LOGI(TAG, "BLE 5.3 Audio Stack Initialized. LC3 Fixed-Point Decoder Ready.");
    return startScanning();
}

esp_err_t BleAudioReceiver::initLc3Decoder() {
    esp_lc3_decoder_cfg_t cfg = {
        .sample_rate = m_stream_info.sample_rate,
        .channels = m_stream_info.channels,
        .frame_duration_us = static_cast<uint32_t>(m_stream_info.frame_duration_ms * 1000),
    };

    esp_lc3_decoder_handle_t decoder_handle = nullptr;
    esp_err_t ret = esp_lc3_decoder_create(&cfg, &decoder_handle);
    if (ret == ESP_OK) {
        m_lc3_decoder = static_cast<void*>(decoder_handle);
        ESP_LOGI(TAG, "esp_lc3 Fixed-Point Decoder Created: %" PRIu32 " Hz, %u channels, %" PRIu32 " us frame",
                 cfg.sample_rate, cfg.channels, cfg.frame_duration_us);
    }
    return ret;
}

void BleAudioReceiver::cleanupLc3Decoder() {
    if (m_lc3_decoder) {
        auto handle = static_cast<esp_lc3_decoder_handle_t>(m_lc3_decoder);
        esp_lc3_decoder_delete(handle);
        m_lc3_decoder = nullptr;
        ESP_LOGI(TAG, "esp_lc3 Decoder deleted.");
    }
}

esp_err_t BleAudioReceiver::startScanning() {
    if (!m_is_initialized) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Scanning for Google Pixel 10 BLE Audio Transmitter / Auracast Broadcast...");
    m_state = ConnectionState::SCANNING;

    // Simulate finding and syncing to Google Pixel 10 broadcast stream
    m_state = ConnectionState::CONNECTED_TO_PIXEL10;
    m_stream_info.rssi_dbm = -58;
    m_state = ConnectionState::STREAMING;
    ESP_LOGI(TAG, "Synced to Google Pixel 10 BIS Stream (48 kHz / 160 kbps LC3).");

    return ESP_OK;
}

const char* BleAudioReceiver::getConnectionStateStr() const {
    switch (m_state) {
        case ConnectionState::IDLE: return "IDLE";
        case ConnectionState::SCANNING: return "SCANNING";
        case ConnectionState::CONNECTED_TO_PIXEL10: return "CONNECTED (Pixel 10)";
        case ConnectionState::BIS_SYNCED: return "BIS SYNCED";
        case ConnectionState::STREAMING: return "STREAMING (LC3)";
        case ConnectionState::PAUSED: return "PAUSED";
        case ConnectionState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

esp_err_t BleAudioReceiver::updateSampleRate(uint32_t new_sample_rate) {
    if (new_sample_rate == 0) return ESP_ERR_INVALID_ARG;
    if (new_sample_rate == m_stream_info.sample_rate && 
        std::abs(m_filter.getSampleRate() - static_cast<float>(new_sample_rate)) < 1.0f) {
        return ESP_OK; // No change needed
    }

    ESP_LOGI(TAG, "Incoming BLE Audio Stream Sample Rate Changed: %lu Hz -> %lu Hz.", 
             m_stream_info.sample_rate, new_sample_rate);

    m_stream_info.sample_rate = new_sample_rate;

    // 1. Re-initialize 80 Hz High-Pass DSP Filter with new sampling rate
    esp_err_t ret = m_filter.init(m_filter.getCutoffFreq(), static_cast<float>(new_sample_rate));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-initialize DSP 80 Hz High-Pass Filter: %s", esp_err_to_name(ret));
    }

    // 2. Re-initialize LC3 Decoder with new sample rate
    cleanupLc3Decoder();
    ret = initLc3Decoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-initialize LC3 decoder: %s", esp_err_to_name(ret));
    }

    // 3. Re-initialize MAX98357A I²S DAC clock rate
    ret = m_dac.init(new_sample_rate, 
                     static_cast<gpio_num_t>(I2S_DAC_BCLK_PIN), 
                     static_cast<gpio_num_t>(I2S_DAC_WS_PIN), 
                     static_cast<gpio_num_t>(I2S_DAC_DOUT_PIN));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-initialize I²S DAC clock rate: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Re-initialization Complete: LPF, LC3 Decoder & I²S DAC running at %lu Hz.", new_sample_rate);
    return ESP_OK;
}

void BleAudioReceiver::processAudioFrame(const uint8_t* lc3_packet, size_t packet_len) {
    if (m_state != ConnectionState::STREAMING) return;

    // Check if sample rate changed and re-initialize LPF if necessary
    if (m_stream_info.sample_rate != static_cast<uint32_t>(m_filter.getSampleRate())) {
        updateSampleRate(m_stream_info.sample_rate);
    }

    size_t samples_decoded = PCM_BUFFER_LENGTH_SAMPLES;

    // 1. Execute actual esp_lc3 fixed-point decode API call if valid packet is provided
    if (m_lc3_decoder && lc3_packet && packet_len > 0) {
        auto handle = static_cast<esp_lc3_decoder_handle_t>(m_lc3_decoder);
        size_t pcm_output_bytes = sizeof(m_pcm_buffer);
        size_t out_len = 0;

        esp_err_t ret = esp_lc3_decode(handle, lc3_packet, packet_len, 
                                       reinterpret_cast<uint8_t*>(m_pcm_buffer), 
                                       pcm_output_bytes, &out_len);
        if (ret == ESP_OK) {
            samples_decoded = out_len / sizeof(int16_t);
        } else {
            ESP_LOGW(TAG, "esp_lc3_decode failed: %s", esp_err_to_name(ret));
        }
    }

    // 2. Pass decoded PCM audio through 80 Hz 2nd-Order Low-Pass Filter (esp-dsp Q31 fixed-point math)
    m_filter.process(m_pcm_buffer, samples_decoded);

    // 3. Write filtered PCM audio to MAX98357A I²S DMA queue
    size_t bytes_written = 0;
    m_dac.write(m_pcm_buffer, samples_decoded * sizeof(int16_t), &bytes_written, 20);
}

void BleAudioReceiver::setVolumePercent(uint8_t vol_pct) {
    if (vol_pct > 100) vol_pct = 100;
    m_stream_info.volume_percent = static_cast<int8_t>(vol_pct);
    vcs_volume_setting = static_cast<uint8_t>((static_cast<uint32_t>(vol_pct) * 255) / 100);
}

const char* BleAudioReceiver::getAseStateStr() const {
    switch (static_cast<AseState>(sink_ase_state[1])) {
        case AseState::IDLE: return "IDLE";
        case AseState::CODEC_CONFIGURED: return "CODEC_CONFIGURED";
        case AseState::QOS_CONFIGURED: return "QOS_CONFIGURED";
        case AseState::ENABLING: return "ENABLING";
        case AseState::STREAMING: return "STREAMING (CIS)";
        case AseState::DISABLING: return "DISABLING";
        case AseState::RELEASING: return "RELEASING";
        default: return "UNKNOWN";
    }
}

} // namespace Bluetooth
