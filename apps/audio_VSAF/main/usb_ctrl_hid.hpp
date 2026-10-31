#pragma once
/**
 * @file usb_ctrl_hid.hpp
 * @brief VSAF control over USB HID (SOURCE, ESP32-S3): the parameter table (vsaf_params.hpp) for a WebHID page.
 *
 * Vendor-defined HID interface (usage page 0xFF00) in the SOURCE's composite USB device. All reports are 63
 * bytes of payload plus the report ID; little-endian.
 *
 * Report 1, output (host -> SOURCE), request:
 *   [0] op   [1] tag (echoed)   [2..3] u16 param id or index   [4..7] f32 value (SET) / [4] u8 enum index (ENUM)
 * Report 2, input (SOURCE -> host), response:
 *   [0] op   [1] tag   [2] status (0 ok, 1 clamped, 2 read_only, 3 unknown, 4 invalid, 5 bad_op)
 *   GET / SET:  [3..4] id, [5..8] f32 value (applied)
 *   DESC:       [3..4] index, [5] count, [6..7] id, [8] type (0 int, 1 float, 2 enum), [9] access (0 ro, 1 rw),
 *               [10..13] f32 min, [14..17] f32 max, [18..21] f32 default, [22] enum count,
 *               [23..46] name (24 bytes, NUL padded), [47..54] unit (8 bytes UTF-8, NUL padded), [55] decimals
 *   ENUM:       [3..4] id, [5] index, [6..29] label (24 bytes, NUL padded)
 * Report 3, input (SOURCE -> host), value push:
 *   [0] version (2)   [1..4] u32 status snapshot seq   [5] count n (<= 11)   [6..] n x (u8 table index, f32 value)
 *   Read-write parameters every 100 ms (10 Hz); all parameters when the 1 s heartbeat publishes a new
 *   snapshot, and on a STATUS request.
 */
#include <cstdint>

#define VSAF_HID_OP_GET 1
#define VSAF_HID_OP_SET 2
#define VSAF_HID_OP_DESC 3
#define VSAF_HID_OP_ENUM 4
#define VSAF_HID_OP_STATUS 5

#define VSAF_HID_REPORT_REQUEST 1
#define VSAF_HID_REPORT_RESPONSE 2
#define VSAF_HID_REPORT_STATUS 3
#define VSAF_HID_PAYLOAD 63

// Report descriptor: vendor page 0xFF00, one output and two input reports of 63 bytes each
#define VSAF_HID_REPORT_DESC                                                                 \
    HID_USAGE_PAGE_N(0xFF00, 2), HID_USAGE(0x01), HID_COLLECTION(HID_COLLECTION_APPLICATION), \
        HID_REPORT_ID(VSAF_HID_REPORT_REQUEST) HID_USAGE(0x02), HID_LOGICAL_MIN(0x00),        \
        HID_LOGICAL_MAX_N(0x00FF, 2), HID_REPORT_SIZE(8), HID_REPORT_COUNT(VSAF_HID_PAYLOAD),  \
        HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                    \
        HID_REPORT_ID(VSAF_HID_REPORT_RESPONSE) HID_USAGE(0x03), HID_LOGICAL_MIN(0x00),       \
        HID_LOGICAL_MAX_N(0x00FF, 2), HID_REPORT_SIZE(8), HID_REPORT_COUNT(VSAF_HID_PAYLOAD),  \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                     \
        HID_REPORT_ID(VSAF_HID_REPORT_STATUS) HID_USAGE(0x04), HID_LOGICAL_MIN(0x00),         \
        HID_LOGICAL_MAX_N(0x00FF, 2), HID_REPORT_SIZE(8), HID_REPORT_COUNT(VSAF_HID_PAYLOAD),  \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), HID_COLLECTION_END

/**
 * @brief Start the HID control task (SOURCE only, after usb_audio_init()). Allocates its queue and task once.
 */
void usb_ctrl_hid_start(void);
