/**
 * @file usb_ctrl_hid.cpp
 * @brief VSAF control over USB HID: request handling and status push (see usb_ctrl_hid.hpp for the protocol).
 *
 * TinyUSB delivers output reports in its own task (tud_hid_set_report_cb); they are copied into a queue and
 * handled by a low-priority control task, which also sends the periodic status reports. Nothing here runs in
 * the audio pipeline; the queue and the task are created once at start.
 */
#include "usb_ctrl_hid.hpp"

#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tusb.h"
#include "vsaf_params.hpp"

namespace {

const char* TAG = "USB_HID";

struct Request {
    uint8_t data[VSAF_HID_PAYLOAD];
};

QueueHandle_t s_queue = nullptr;

void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}
void put_u32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
void put_f32(uint8_t* p, float f) {
    uint32_t v;
    memcpy(&v, &f, 4);
    put_u32(p, v);
}
uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
float get_f32(const uint8_t* p) {
    const uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    float f;
    memcpy(&f, &v, 4);
    return f;
}

/** @brief Send one input report, waiting up to ~50 ms for the IN endpoint (control task only). */
bool send_report(uint8_t report_id, const uint8_t* payload) {
    for (int i = 0; i < 50; ++i) {
        if (!tud_mounted()) return false;
        if (tud_hid_ready()) {
            return tud_hid_report(report_id, payload, VSAF_HID_PAYLOAD);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

void handle_request(const uint8_t* rq) {
    namespace P = AudioNet::Params;
    uint8_t out[VSAF_HID_PAYLOAD] = {};
    const uint8_t op = rq[0];
    out[0] = op;
    out[1] = rq[1];
    switch (op) {
        case VSAF_HID_OP_GET:
        case VSAF_HID_OP_SET: {
            const uint16_t id = get_u16(rq + 2);
            put_u16(out + 3, id);
            float v = 0;
            if (op == VSAF_HID_OP_GET) {
                out[2] = P::get(id, v) ? 0 : static_cast<uint8_t>(P::SetStatus::UNKNOWN);
            } else {
                const P::SetStatus st = P::set(id, get_f32(rq + 4), v);
                out[2] = static_cast<uint8_t>(st);
                if (st != P::SetStatus::OK && st != P::SetStatus::CLAMPED) {
                    P::get(id, v);  // report the value still in effect
                }
            }
            put_f32(out + 5, v);
            break;
        }
        case VSAF_HID_OP_DESC: {
            const uint16_t idx = get_u16(rq + 2);
            put_u16(out + 3, idx);
            out[5] = static_cast<uint8_t>(P::count());
            if (idx >= P::count()) {
                out[2] = static_cast<uint8_t>(P::SetStatus::UNKNOWN);
                break;
            }
            const P::Desc& d = P::descAt(idx);
            put_u16(out + 6, d.id);
            out[8] = d.type == P::Type::INT ? 0 : (d.type == P::Type::FLOAT ? 1 : 2);
            out[9] = d.access == P::Access::RW ? 1 : 0;
            put_f32(out + 10, d.min);
            put_f32(out + 14, d.max);
            put_f32(out + 18, d.def);
            out[22] = d.enum_count;
            strncpy(reinterpret_cast<char*>(out + 23), d.name, 24);
            strncpy(reinterpret_cast<char*>(out + 47), d.unit, 8);
            out[55] = d.decimals;
            break;
        }
        case VSAF_HID_OP_ENUM: {
            const uint16_t id = get_u16(rq + 2);
            const uint8_t i = rq[4];
            put_u16(out + 3, id);
            out[5] = i;
            const P::Desc* d = P::find(id);
            if (!d || d->type != P::Type::ENUM || i >= d->enum_count) {
                out[2] = static_cast<uint8_t>(P::SetStatus::UNKNOWN);
                break;
            }
            strncpy(reinterpret_cast<char*>(out + 6), d->enum_names[i], 24);
            break;
        }
        case VSAF_HID_OP_STATUS:
            return;  // the caller sends the status pages right away
        default:
            out[2] = 5;  // bad op
            break;
    }
    send_report(VSAF_HID_REPORT_RESPONSE, out);
}

/**
 * @brief Push parameter values as (index, value) pairs, 11 per report.
 * @param rw_only true: only read-write parameters (10 Hz control refresh); false: all parameters.
 */
void send_values(bool rw_only) {
    namespace P = AudioNet::Params;
    constexpr size_t PER_REPORT = 11;
    const uint32_t seq = P::latestStatus().seq;
    uint8_t out[VSAF_HID_PAYLOAD] = {};
    size_t n = 0;
    auto flush = [&]() {
        if (n == 0) return;
        out[0] = 2;
        put_u32(out + 1, seq);
        out[5] = static_cast<uint8_t>(n);
        send_report(VSAF_HID_REPORT_STATUS, out);
        memset(out, 0, sizeof(out));
        n = 0;
    };
    for (size_t i = 0; i < P::count(); ++i) {
        const P::Desc& d = P::descAt(i);
        if (rw_only && d.access != P::Access::RW) continue;
        float v = 0;
        P::get(d.id, v);
        out[6 + 5 * n] = static_cast<uint8_t>(i);
        put_f32(out + 7 + 5 * n, v);
        if (++n == PER_REPORT) flush();
    }
    flush();
}

void ctrl_task(void*) {
    Request rq;
    TickType_t last_fast = xTaskGetTickCount();
    uint32_t sent_seq = 0;
    while (true) {
        if (xQueueReceive(s_queue, &rq, pdMS_TO_TICKS(20)) == pdTRUE) {
            handle_request(rq.data);
            if (rq.data[0] == VSAF_HID_OP_STATUS) {
                send_values(false);
            }
        }
        if (!tud_mounted()) continue;
        // All values when the heartbeat (1 Hz) has published a new snapshot
        const uint32_t seq = AudioNet::Params::latestStatus().seq;
        if (seq != sent_seq) {
            sent_seq = seq;
            send_values(false);
        }
        // Controls at 10 Hz
        if (xTaskGetTickCount() - last_fast >= pdMS_TO_TICKS(100)) {
            last_fast = xTaskGetTickCount();
            send_values(true);
        }
    }
}

}  // namespace

void usb_ctrl_hid_start(void) {
    if (s_queue) return;
    s_queue = xQueueCreate(8, sizeof(Request));
    // Low priority (2), core 0 next to the USB task: control traffic never preempts the audio pipeline
    if (!s_queue || xTaskCreatePinnedToCore(ctrl_task, "usb_ctrl", 3072, nullptr, 2, nullptr, 0) != pdPASS) {
        ESP_LOGE(TAG, "HID control task not started");
        return;
    }
    ESP_LOGI(TAG, "HID control interface ready (vendor page 0xFF00)");
}

// ---------------------------------------------------------------------------
// TinyUSB HID callbacks
// ---------------------------------------------------------------------------

extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                                      uint8_t const* buffer, uint16_t bufsize) {
    (void)instance;
    (void)report_type;
    // Interrupt OUT delivers the report ID as the first byte (report_id = 0); SET_REPORT passes it separately
    if (report_id == 0 && bufsize > 0) {
        report_id = buffer[0];
        buffer++;
        bufsize--;
    }
    if (report_id != VSAF_HID_REPORT_REQUEST || !s_queue) return;
    Request rq = {};
    memcpy(rq.data, buffer, bufsize < VSAF_HID_PAYLOAD ? bufsize : VSAF_HID_PAYLOAD);
    xQueueSend(s_queue, &rq, 0);  // drop when full: the host retries on missing responses
}

extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                                          uint8_t* buffer, uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;  // input reports are pushed on the interrupt endpoint
}
