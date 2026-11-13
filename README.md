# LineForge LFR v1.1.0 — ESP32-S3 / HC-06 / QTR-8A

Native Android (Java) Bluetooth Classic SPP controller + ESP32-S3 firmware. The application talks to an **HC-06** at **9600 baud**. No BLE or internet connection is required.

## What changed in v1.1.0

- **Live animated 8-channel QTR-8A sensor strip**, updated about **8 times per second**, even with motors STOPPED. Bar height follows the calibrated sensor's black-line reflectance; the glowing marker estimates line position. While calibrating, the bars show raw ADC signals.
- **Reliable, checksum-validated telemetry**. Garbage/boot-log bytes and incomplete packets are discarded, and CRC failures are counted, rather than reporting every byte as an error. The Bluetooth SPP byte stream is **not** assumed to preserve packet boundaries.
- **START confirmation**. The app now says START SENT until the ESP32 returns an authenticated status packet. Motor heartbeat continues during this pending state; failure to confirm is reported.
- **READ SAVED QTR CALIBRATION** shows the eight stored white/black ADC reference pairs (saved in ESP32 NVS).
- **READ FROM ROBOT** retrieves all six PID numerator/divider fields from ESP32 RAM/NVS, along with the true motor and calibration status.
- Firmware uses **Serial0** (UART0) on native ESP32-S3 **RX GPIO44, TX GPIO43**. Keep **USB CDC On Boot = Enabled** so Serial Monitor stays on USB; all the other original GPIOs remain unchanged.
- PID changes are saved to NVS when motors are stopped (no flash stalls while driving); calibration is saved after successful recalibration.
- **GitHub Actions** tests the protocol, builds APK and automatically attaches both the APK and matching firmware to a preview/stable GitHub Release.

## Upload and build

1. Unzip the project. **Copy the contents of `LineForge_LFR` into the ROOT of your GitHub repository**, so that `app/`, `build.gradle`, `settings.gradle`, `.github/workflows/build-apk.yml`, `tests/`, and `ESP32S3_LFR_BT_NVS_LineForge_v2/` are at the top level. Preserve `.github` (hidden on some file managers).
2. Commit and push to `main`. GitHub Actions → Build and Release LineForge v1.1 APK will build, test and publish a `build-N` preview Release. Or push a tag like `v1.1.0` to publish a versioned Release.
3. Download the APK under GitHub **Releases** → latest preview (or v1.1.0). The APK is created by GitHub Actions, **not included in this source ZIP**.
4. Flash `ESP32S3_LFR_BT_NVS_LineForge_v2/ESP32S3_LFR_BT_NVS_LineForge_v2.ino` using Arduino-ESP32 3.x, board **ESP32S3 Dev Module**, **USB CDC On Boot: Enabled**, monitor 115200 USB.
5. Pair HC-06 via Android Bluetooth settings; open the app, connect and check that both **PID status** and **moving eight sensor bars** appear. Use READ SAVED QTR CALIBRATION to verify saved min/max. Test START with wheels raised.

**Signing note:** the workflow currently builds **debug-signed** APKs. GitHub's ephemeral runners may generate different debug keys. Android may refuse to update an older APK; uninstalling the old app may be necessary. A permanent release signing key is the next step for seamless upgrades. Keep app settings backed up as uninstalling deletes the phone app's local settings; the ESP32 NVS data is unaffected.

## Hardware checklist

- HC-06 TX → **ESP32-S3 GPIO44 RX**, HC-06 RX ← **GPIO43 TX**, GND common. HC-06 serial configured at 9600 baud (8N1).
- HC-06 logic supply: regulated 3.3 V after regulator bypass; do not put 5 V onto the bypassed module.
- TB6612FNG `STBY` HIGH, `VCC` 3.3 V, `VM` motor supply, common GND. GPIOs motor A: AIN1 42, AIN2 41, PWMA 21; motor B: BIN1 40, BIN2 39, PWMB 47.
- QTR-8A ADC outputs must remain within the ESP32-S3's safe input voltage. Supply QTR at 3.3 V or adapt its outputs safely.
- If strange bytes persist, check UART baud, module power, common ground, wiring, and **whether the ESP32 resets when motors engage**. Framing can detect/recover from corruption; it cannot correct an unstable electrical supply.

## Bluetooth protocol

Inbound, existing two-byte binary commands remain compatible:

| Command | Value | Meaning |
|---|---|---|
| 1 / 2 | 0–255 / 0–4 | Kp numerator / divider |
| 3 / 4 | 0–255 / 0–4 | Ki numerator / divider |
| 5 / 6 | 0–255 / 0–4 | Kd numerator / divider |
| 7 | 0 or 1 | STOP / START |
| 8 | 1 | QTR recalibration (2s white then 5s sweep) |
| 9 | 0 | PID and motor status request |
| 10 | 1 | Heartbeat every 600 ms while running |
| 11 | 0 or 1 | Disable / enable 8Hz telemetry; enabling refreshes a four-second lease |
| 12 | 0 | Read NVS white/black calibration arrays |

The phone sends `[11,1]` about every 1.8 seconds to keep telemetry enabled. The previous commands 1–10 still work if your original app is used without enabling telemetry. START is refused by firmware if calibration is invalid or in progress.

**Framed ESP32 → phone replies** (used after `[11,1]`):

`A5 5A TYPE LEN SEQ PAYLOAD CRC8`

CRC8 polynomial `0x07`, initial `0x00`, no reflection or final XOR, calculated over TYPE, LEN, SEQ and PAYLOAD. A frame is at most **54 bytes** including headers/CRC; status frame 14 bytes; telemetry frame **19 bytes**, sent at 8Hz (~152 bytes/sec UART payload). This is comfortably below ~960 bytes/sec available with UART 9600, 8N1, subject to radio and scheduling overhead. No individual message exceeds 256 bytes.

| Packet type | Data |
|---|---|
| `0x51` | 8 bytes: Kp num, Kp div, Ki num, Ki div, Kd num, Kd div, flags, reserved. Flags bit 0 calibration ready, bit 1 motors running, bit 2 calibrating. |
| `0x52` | 13 bytes: sensor levels S1–S8 (8× uint8, 0 white/255 black); line position uint16 little-endian (0–7000; 0xFFFF=line lost); flags; signed left/right PWM percentage int8. |
| `0x53` | 32 bytes: for each of eight sensors, `uint16 LE white`, `uint16 LE black`. |

Legacy replies accepted: `0x06` ACK, `0x15` NAK, `0xC0` calibration failure, `0xC1` calibration success, `0xC2` NVS calibration loaded, `0xD1` heartbeat timeout; legacy 9-byte `0xD0` status frame accepted too.

## Limitations

- **No real hardware or full Android SDK build was available here.** The ESP32 sketch was syntax-checked using interface stubs, and the Android source was Java-compiled using API interface stubs. Actual Arduino IDE compilation and GitHub Actions `:app:assembleDebug` are still required. Protocol parser tests passed.
- A Bluetooth connection to HC-06 alone does **not** mean ESP32 UART responses are valid. If sensor/status frames never arrive, check the electrical UART path and 9600 baud; do not trust the START button as proof of motor activation.
- The command side retains the original 2-byte format for backward compatibility; it has no checksum. Only the **new outgoing telemetry/status frames** have CRC8 error detection. For more fault-tolerant competition use, a later protocol version should frame and acknowledge commands too.
- Robot control is safety-oriented: STOP when app leaves foreground; firmware also has 2.5s heartbeat timeout. These protections do not replace a physical motor power cutoff.
