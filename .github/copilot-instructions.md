<!-- .github/copilot-instructions.md - guidance for AI coding agents -->
# ESPHeatEnergyReporter — Copilot instructions

This repository is a small PlatformIO project for an ESP8266-based heat energy meter reporter that: wakes an Allmess IR interface, reads M-Bus-like frames, parses meter values, and reports them over Serial and UDP. Use these instructions to make focused, safe changes.

- **Big picture**: `ESPHeatEnergyReporter.ino` is the entrypoint. It:
  - Initializes Wi-Fi (optionally; credentials come from an external `MyCreds.h`).
  - Mounts LittleFS and reads `/text.txt` from `data/`.
  - Listens on `Serial` for command `'r'` to trigger `AllmessRead()` and then `mbusParser.ParseLL(...)`.
  - Broadcasts a short UDP packet on port `4210` after Wi‑Fi connects.

- **Major components / files**:
  - `src/ESPHeatEnergyReporter.ino` — main flow, WiFi/NTP, LittleFS usage, CLI trigger (`'r'`).
  - `src/Allmess.cpp` / `src/Allmess.h` — hardware/IR comms (wakeup, SND/NKE, RSP_UD). Uses `EspSoftwareSerial`.
  - `src/MBusParser.cpp` / `src/MBusParser.h` — frame parsing and value extraction (energy, volume, temps, timepoint).
  - `data/text.txt` — LittleFS file read on boot; upload with PlatformIO's `uploadfs` target.
  - `platformio.ini` — project build config (use PlatformIO CLI / VS Code PlatformIO).

- **Why things are structured this way**:
  - Low-level IR timing/sequence handled in `Allmess*` to keep hardware/comm details separate from parsing logic.
  - `MBusParser` focuses on interpreting payload bytes into domain values (kWh, m3, temps).
  - LittleFS is used for small on-device text/config; therefore `data/` must be uploaded to the device filesystem for runtime reads.

- **Key project-specific conventions** (follow these precisely):
  - Wi‑Fi credentials are NOT in this repo. The code expects a `MyCreds` header provided outside the repo (comment references `/<HOME>/.platformio/lib/MyCreds/MyCreds.h`). If adding credentials for testing, either:
    - Create a `lib/MyCreds/MyCreds.h` inside the project with `#define WIFI_SSID "..."` and `#define WIFI_PASSWORD "..."`, or
    - Add preprocessor defs via `platformio.ini` build flags.
  - Serial trigger: Send the ASCII character `r` to the serial console to start an Allmess read; the code then fills `SerRxBuf` and sets `SerRxBufLen`.
  - Debug baud: `DEBUG_ESP_BAUDRATE` is `74880`. Use `pio device monitor -b 74880`.

- **Build / flash / filesystem commands** (PlatformIO):
  - Build: `pio run`
  - Upload firmware: `pio run -t upload`
  - Upload LittleFS `data/` folder to the device: `pio run -t uploadfs`
  - Serial monitor (74880 baud): `pio device monitor -b 74880`

- **Runtime examples** (how you as an agent can trigger behavior):
  - Connect serial monitor, press `r` → device wakes Allmess IR, reads bytes into `SerRxBuf`, then `mbusParser.ParseLL(SerRxBuf, SerRxBufLen)` prints parsed values.
  - Watch UDP broadcasts on port 4210 (uses broadcast IP 255.255.255.255).

- **Common change patterns you may be asked to implement**:
  - Add structured logging: follow existing `Serial.printf` usage and avoid changing frame parsing offsets unless you can test with sample captures.
  - Add a web endpoint or MQTT: keep parsing inside `MBusParser` and publish only already-parsed values (avoid duplicating parsing logic).
  - Improve robustness: checks are done in `MBusParser::ParseLL` (e.g., header checks, checksum). When adding guards, keep existing serial dump behavior for easier debugging.

- **Dependencies and libraries to be aware of**:
  - PlatformIO handles build; `EspSoftwareSerial` (the included `SoftwareSerial` wrapper) is used for IR signalling.
  - `LittleFS`, `NTPClient`, `WS2812Write` are used — make changes assuming these APIs are available via PlatformIO libs.

- **Files to inspect when debugging issues**:
  - `src/Allmess.cpp` for IR timing and reads (common failure point if pins or timings change).
  - `src/MBusParser.cpp` for how values are decoded from raw bytes — unit scaling is explicit (see `EnergykWh` and `Volumem3`).
  - `platformio.ini` to confirm board, framework and library configuration before changing includes or build flags.

If anything here is unclear or you'd like me to include example unit tests, CI instructions, or a short README section, tell me what to add and I'll iterate.
