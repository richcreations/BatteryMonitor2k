# ESP32 Dual Battery Monitor (NMEA2000) - Baseline v0.8.0 (Full)

This is the **baseline stable release** with the full sketch included.

- Firmware version: **v0.8.0** (`FW_VERSION` in `src/main.cpp`)
- Version is also embedded in **Product Information** (PGN 126996) via `SetProductInformation`,
  so NMEA2000 loggers can read the firmware version from the "Model version" field.
- Defaults: 12 V, Flooded Lead Acid, 100 Ah Engine / 100 Ah House.
- PGNs: 127508 & 127506 every 1s; 127513 at startup + every 30s.
