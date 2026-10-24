# Vendored: espressif/esp_tinyusb 3.0.0

Copied from the ESP Component Registry (bundles TinyUSB under `tinyusb/`) so that:
- the version is pinned (the manifest used `'*'`, which silently moved 2.3.0 -> 3.0.0), and
- S3 and C6 builds can run in parallel: a registry-managed copy lives in the shared
  `managed_components/` and is deleted by every C6 configure (esp_tinyusb is S3-only).

Source: https://github.com/espressif/esp-usb/tree/master/device/esp_tinyusb
Commit: 68bf7f9613dbb1a31094fdb903209ae3e4bf388d (see `idf_component.yml.upstream`)
License: Apache-2.0 (esp_tinyusb), MIT (tinyusb/) - see LICENSE files.

Local changes vs upstream:
- `idf_component.yml` renamed to `idf_component.yml.upstream` (keeps metadata, not processed).
- `CMakeLists.txt`: early empty `idf_component_register()` on targets without USB-OTG device.
- Removed: `test_apps/`, `CHECKSUMS.json`, `tinyusb/AGENTS.md`, `tinyusb/CLAUDE.md`.

To update: download the new release from the registry, re-apply the changes above.
