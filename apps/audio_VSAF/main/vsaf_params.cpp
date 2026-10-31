/**
 * @file vsaf_params.cpp
 * @brief VSAF parameter table (see vsaf_params.hpp).
 */
#include "vsaf_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "vsaf_broadcast_engine.hpp"

namespace AudioNet {
namespace Params {

namespace {

EspNowBroadcastEngine* s_engine = nullptr;
portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
StatusSnapshot s_status = {};

const char* const kModeNames[] = {"pause", "cast", "synth"};
enum Mode : uint8_t { MODE_PAUSE = 0, MODE_CAST = 1, MODE_SYNTH = 2 };

// IDs: high byte = group (0x01 system, 0x02 SOURCE pipeline, 0x03 radio, 0x04 audio status, 0x10 controls).
// Never reuse an ID for a different meaning; add new parameters with new IDs.
constexpr uint16_t ID_CPU_PCT = 0x0100;
constexpr uint16_t ID_TEMP_C = 0x0101;
constexpr uint16_t ID_PEERS_ONLINE = 0x0110;
constexpr uint16_t ID_PEERS_MASK = 0x0111;
constexpr uint16_t ID_DSP_MS = 0x0200;
constexpr uint16_t ID_ENC_HQ_MS = 0x0201;
constexpr uint16_t ID_ENC_MQ_MS = 0x0202;
constexpr uint16_t ID_TX_END_MS = 0x0300;
constexpr uint16_t ID_TX_PKTS_S = 0x0301;
constexpr uint16_t ID_RMS_DBFS = 0x0400;
constexpr uint16_t ID_MASTER_DB = 0x1000;
constexpr uint16_t ID_SOURCE_MODE = 0x1001;
constexpr uint16_t ID_RF_CHANNEL = 0x1002;

const Desc kTable[] = {
    {ID_CPU_PCT, "sys.cpu_pct", "%", Type::INT, Access::RO, 0, 100, 0, nullptr, 0,
     "CPU load: SOURCE (DSP + encode) / 2 cores, SINK decode / frame"},
    {ID_TEMP_C, "sys.temp_c", "C", Type::FLOAT, Access::RO, -40, 125, 0, nullptr, 0, "On-chip temperature"},
    {ID_PEERS_ONLINE, "net.peers_online", "", Type::INT, Access::RO, 0, 6, 0, nullptr, 0, "SINKs online"},
    {ID_PEERS_MASK, "net.peers_mask", "", Type::INT, Access::RO, 0, 63, 0, nullptr, 0,
     "Online channels, bit c = channel c (0 L, 1 R, 2 C, 3 LS, 4 RS, 5 Sub)"},
    {ID_DSP_MS, "dsp.time_ms", "ms", Type::FLOAT, Access::RO, 0, 10, 0, nullptr, 0, "DSP stage per frame"},
    {ID_ENC_HQ_MS, "enc.hq_ms", "ms", Type::FLOAT, Access::RO, 0, 20, 0, nullptr, 0,
     "HQ + sub LC3 encode per frame, both cores"},
    {ID_ENC_MQ_MS, "enc.mq_ms", "ms", Type::FLOAT, Access::RO, 0, 20, 0, nullptr, 0,
     "MQ (redundancy) LC3 encode per frame, both cores"},
    {ID_TX_END_MS, "rf.tx_end_ms", "ms", Type::FLOAT, Access::RO, 0, 10, 0, nullptr, 0,
     "Last packet of the frame on air, after the frame tick (deadline 9.5 ms)"},
    {ID_TX_PKTS_S, "rf.tx_pkts_s", "1/s", Type::INT, Access::RO, 0, 1000, 0, nullptr, 0, "Packets sent per second"},
    {ID_RMS_DBFS, "audio.rms_dbfs", "dBFS", Type::FLOAT, Access::RO, -100, 0, -100, nullptr, 0,
     "Audio RMS (SOURCE input / SINK output), -100 = silent or off"},
    {ID_MASTER_DB, "audio.master_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Master volume on the SOURCE (all channels), -96 = mute; ramped"},
    {ID_SOURCE_MODE, "source.mode", "", Type::ENUM, Access::RW, 0, 2, 1, kModeNames, 3,
     "SOURCE input / state: pause (IDLE), cast (USB audio), synth (test synth)"},
    {ID_RF_CHANNEL, "rf.channel", "", Type::INT, Access::RW, 1, 13, 13, nullptr, 0,
     "Wi-Fi channel (applied at the next frame boundary)"},
};
constexpr size_t kCount = sizeof(kTable) / sizeof(kTable[0]);

float currentMode() {
    if (!s_engine) return MODE_PAUSE;
    const NetworkState st = s_engine->getState();
    if (st != NetworkState::CAST) return MODE_PAUSE;
    return s_engine->isToneTestMode() ? MODE_SYNTH : MODE_CAST;
}

}  // namespace

const char* setStatusName(SetStatus s) {
    switch (s) {
        case SetStatus::OK: return "ok";
        case SetStatus::CLAMPED: return "clamped";
        case SetStatus::READ_ONLY: return "read_only";
        case SetStatus::UNKNOWN: return "unknown";
        case SetStatus::INVALID: return "invalid";
    }
    return "?";
}

void bindEngine(EspNowBroadcastEngine* engine) { s_engine = engine; }

void publishStatus(const StatusSnapshot& s) {
    portENTER_CRITICAL(&s_status_mux);
    const uint32_t seq = s_status.seq + 1;
    s_status = s;
    s_status.seq = seq ? seq : 1;
    portEXIT_CRITICAL(&s_status_mux);
}

StatusSnapshot latestStatus() {
    portENTER_CRITICAL(&s_status_mux);
    const StatusSnapshot s = s_status;
    portEXIT_CRITICAL(&s_status_mux);
    return s;
}

size_t count() { return kCount; }
const Desc& descAt(size_t index) { return kTable[index < kCount ? index : 0]; }

const Desc* find(uint16_t id) {
    for (const Desc& d : kTable) {
        if (d.id == id) return &d;
    }
    return nullptr;
}

const Desc* findByText(const char* text) {
    if (!text || !*text) return nullptr;
    for (const Desc& d : kTable) {
        if (strcasecmp(d.name, text) == 0) return &d;
    }
    char* end = nullptr;
    const unsigned long v = strtoul(text, &end, 0);  // "0x0100" or "256"
    if (end && *end == '\0' && v <= 0xFFFF) return find(static_cast<uint16_t>(v));
    return nullptr;
}

bool get(uint16_t id, float& value) {
    const StatusSnapshot s = latestStatus();
    switch (id) {
        case ID_CPU_PCT: value = s.cpu_pct; return true;
        case ID_TEMP_C: value = s.temp_c; return true;
        case ID_PEERS_ONLINE: value = s.peers_online; return true;
        case ID_PEERS_MASK: value = s.peers_mask; return true;
        case ID_DSP_MS: value = s.dsp_ms; return true;
        case ID_ENC_HQ_MS: value = s.enc_hq_ms; return true;
        case ID_ENC_MQ_MS: value = s.enc_mq_ms; return true;
        case ID_TX_END_MS: value = s.tx_end_ms; return true;
        case ID_TX_PKTS_S: value = s.tx_pkts_s; return true;
        case ID_RMS_DBFS: value = s.rms_dbfs; return true;
        case ID_MASTER_DB: value = s_engine ? s_engine->getMasterGainDb() : 0.0f; return true;
        case ID_SOURCE_MODE: value = currentMode(); return true;
        case ID_RF_CHANNEL: value = s_engine ? s_engine->getWifiChannel() : 0; return true;
        default: return false;
    }
}

SetStatus set(uint16_t id, float value, float& applied) {
    const Desc* d = find(id);
    if (!d) return SetStatus::UNKNOWN;
    if (d->access != Access::RW) return SetStatus::READ_ONLY;
    if (!s_engine || std::isnan(value)) return SetStatus::INVALID;
    float v = std::clamp(value, d->min, d->max);
    if (d->type != Type::FLOAT) v = std::round(v);
    const SetStatus st = (v != value) ? SetStatus::CLAMPED : SetStatus::OK;
    switch (id) {
        case ID_MASTER_DB:
            s_engine->setMasterGainDb(v);
            applied = s_engine->getMasterGainDb();
            return st;
        case ID_SOURCE_MODE: {
            const auto m = static_cast<Mode>(static_cast<int>(v));
            if (m == MODE_PAUSE) {
                s_engine->setToneTestMode(false);
                s_engine->transitionTo(NetworkState::IDLE);
            } else {
                s_engine->setToneTestMode(m == MODE_SYNTH);
                s_engine->transitionTo(NetworkState::CAST);
            }
            applied = currentMode();
            return st;
        }
        case ID_RF_CHANNEL:
            s_engine->requestWifiChannel(static_cast<uint8_t>(v));  // SOURCE: at the next frame boundary
            s_engine->lockChannel(true);
            applied = v;
            return st;
        default:
            return SetStatus::READ_ONLY;
    }
}

bool parseValue(const Desc& d, const char* text, float& value) {
    if (!text || !*text) return false;
    if (d.type == Type::ENUM) {
        for (uint8_t i = 0; i < d.enum_count; ++i) {
            if (strcasecmp(text, d.enum_names[i]) == 0) {
                value = i;
                return true;
            }
        }
    }
    char* end = nullptr;
    const float v = strtof(text, &end);
    if (!end || *end != '\0') return false;
    value = v;
    return true;
}

void formatValue(const Desc& d, float value, char* out, size_t out_len) {
    if (d.type == Type::ENUM) {
        const int i = static_cast<int>(value);
        snprintf(out, out_len, "%s", (i >= 0 && i < d.enum_count) ? d.enum_names[i] : "?");
    } else if (d.type == Type::INT) {
        snprintf(out, out_len, "%ld", static_cast<long>(std::lround(value)));
    } else {
        snprintf(out, out_len, "%.2f", static_cast<double>(value));
    }
}

}  // namespace Params
}  // namespace AudioNet
