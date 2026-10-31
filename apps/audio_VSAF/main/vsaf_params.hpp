#pragma once
/**
 * @file vsaf_params.hpp
 * @brief VSAF parameter table: one place that names, describes, reads and writes runtime parameters.
 *
 * Transport-independent. The console (`param list|get|set`) uses it today; a USB HID or MIDI interface, or a
 * control node, can use the same calls later.
 *
 * - Every parameter has a stable numeric ID (never reuse an ID for something else) and a dotted name.
 * - Read-only status values come from a snapshot that the 1 s heartbeat publishes (publishStatus()), so reading a
 *   parameter never resets a counter. The snapshot carries a sequence number so a client can see fresh data.
 * - Writes go through the same engine calls as the console commands and return the value actually applied.
 * - Values are exchanged as float (integers and enum indices are exact in float up to 2^24).
 */
#include <cstddef>
#include <cstdint>

namespace AudioNet {
class EspNowBroadcastEngine;

namespace Params {

enum class Type : uint8_t { INT, FLOAT, ENUM };
enum class Access : uint8_t { RO, RW };

/** @brief Static description of one parameter. */
struct Desc {
    uint16_t id;                     ///< Stable numeric ID (group in the high byte)
    const char* name;                ///< Dotted name, e.g. "audio.master_db"
    const char* unit;                ///< "%", "C", "ms", "dB", "" ...
    Type type;
    Access access;
    float min, max, def;             ///< Range and default (enum: index range)
    const char* const* enum_names;   ///< ENUM only: labels, enum_count entries
    uint8_t enum_count;
    const char* help;                ///< One-line description
    uint8_t decimals;                ///< Display precision (decimal places), for the console and UIs
};

/** @brief Result of a write. */
enum class SetStatus : uint8_t { OK, CLAMPED, READ_ONLY, UNKNOWN, INVALID };
const char* setStatusName(SetStatus s);

/** @brief Status values published by the heartbeat once per second (SOURCE fields stay 0 on a SINK). */
struct StatusSnapshot {
    uint32_t seq;            ///< Increments with every publish (0 = never published)
    float cpu_pct;           ///< Heartbeat CPU load (%)
    float temp_c;            ///< On-chip temperature (C)
    float dsp_ms;            ///< SOURCE DSP stage per frame (ms, 1 s average)
    float enc_hq_ms;         ///< SOURCE HQ + sub encode per frame, both cores (ms)
    float enc_mq_ms;         ///< SOURCE MQ encode per frame, both cores (ms)
    float tx_end_ms;         ///< SOURCE last TX completion after the frame tick (ms)
    float tx_pkts_s;         ///< SOURCE packets sent per second
    float rms_dbfs;          ///< Audio RMS (dBFS), -100 when silent / off
    uint8_t peers_online;    ///< SOURCE: online SINKs
    uint8_t peers_mask;      ///< SOURCE: bit c = channel c online
};

/** @brief Give the table access to the engine (call once at boot, before any get / set). */
void bindEngine(EspNowBroadcastEngine* engine);

/** @brief Publish a new status snapshot (diagnostics task, 1 Hz); seq is assigned here. */
void publishStatus(const StatusSnapshot& s);
/** @brief Copy of the latest snapshot. */
StatusSnapshot latestStatus();

size_t count();
const Desc& descAt(size_t index);
/** @brief Find by numeric ID or by name (also accepts "0x0100" / "256" strings via findByText). */
const Desc* find(uint16_t id);
const Desc* findByText(const char* text);

/**
 * @brief Current value of a parameter.
 * @return false for an unknown ID.
 */
bool get(uint16_t id, float& value);

/**
 * @brief Write a parameter.
 * @param value    Requested value (enum: index).
 * @param applied  Out: the value now in effect (after clamping / rounding).
 */
SetStatus set(uint16_t id, float value, float& applied);

/**
 * @brief Parse a textual value for a parameter (number, or enum label such as "synth").
 * @return false if it cannot be parsed.
 */
bool parseValue(const Desc& d, const char* text, float& value);

/** @brief Format a value for display (enum label, integer, or float with 2 decimals). */
void formatValue(const Desc& d, float value, char* out, size_t out_len);

}  // namespace Params
}  // namespace AudioNet
