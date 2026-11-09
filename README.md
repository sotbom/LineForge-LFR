# LineForge LFR — Android Bluetooth PID controller

A dark, streamlined native Android app for an ESP32-S3 QTR-8A + TB6612FNG line follower using **HC-06 or HC-05 Bluetooth Classic SPP**, 9600 baud.

## What is included

- `app/`: complete Android Studio Java source (no third-party UI libraries)
- `ESP32S3_LFR_BT_NVS_LineForge/ESP32S3_LFR_BT_NVS_LineForge.ino`: your existing NVS firmware extended with PID readback and a heartbeat watchdog
- `.github/workflows/build-apk.yml`: a GitHub Actions job that builds an installable `app-debug.apk` and uploads it as an artifact

**The APK cannot be prebuilt in this environment because Android SDK/build tools are unavailable.** Build it in Android Studio or via the included GitHub workflow. An APK must come from an actual Android toolchain; do not rename this ZIP to `.apk`.

## Building the APK (Windows / Android Studio)

1. Unzip the project and open the **LineForge_LFR** folder in Android Studio.
2. Let Android Studio install Android SDK Platform 35 and sync Gradle. Use **JDK 17** (Android Studio's bundled JDK works).
3. Select **Build > Build App Bundle(s) / APK(s) > Build APK(s)**. Depending on Studio version this may be **Build > Generate App Bundles or APKs > Generate APKs**.
4. Find the debug APK at `app/build/outputs/apk/debug/app-debug.apk`.
5. Copy the APK to your phone, allow installation from your file manager, and install. No account or internet is required in the app.

### Build an APK on GitHub (no local Android Studio installation)

1. Create a GitHub repository and upload all the contents of this project, including the hidden `.github` folder.
2. Select **Actions > Build LineForge Android APK > Run workflow**, or push to the `main` branch.
3. When the workflow completes, open the workflow run and download the **LineForge-LFR-Android-APK** artifact ZIP.
4. Extract the ZIP to get `app-debug.apk`, and install it on Android.

The debug APK is development-signed by Android tooling. For personal use it is installable; for publication you need a release keystore and a signed release build.

## Install and connect

1. Power your ESP32-S3 and HC-06. Confirm the module has 3.3 V power, 3.3 V UART logic and shared GND.
2. Open phone **Bluetooth settings** and pair with `HC-06` / `HC-05`. Typical module PINs are `1234` or `0000` (your module may differ).
3. Open LineForge, allow **Nearby devices**, select your paired module, and connect.
4. The app requests the actual Kp/Ki/Kd settings saved in ESP32 NVS. If no readback appears, flash the updated `.ino`.
5. Adjust Kp / Ki / Kd; each uses numerator `0..255` and a divider `10^0 .. 10^4`.
6. Press **APPLY ALL** to send all six settings (12 bytes); firmware saves them to NVS after 2 seconds of inactivity.
7. Place the robot on the track and press **START RUN**. STOP sends an immediate two-byte stop command.
8. **RECALIBRATE SENSORS** prompts you to place the array on white, followed by sweeping it over black; the calibration is stored on the ESP32.

For live slider adjustments, enable **Live tuning**. The app sends the two associated commands when you release a slider, not on every movement.

## Binary protocol

| Direction | Message | Bytes |
|---|---|---:|
| App -> ESP32 | `[1,kpVal]` / `[2,kpDiv]` | 2 each |
| App -> ESP32 | `[3,kiVal]` / `[4,kiDiv]` | 2 each |
| App -> ESP32 | `[5,kdVal]` / `[6,kdDiv]` | 2 each |
| App -> ESP32 | `[7,0]` stop; `[7,1]` start | 2 |
| App -> ESP32 | `[8,1]` recalibrate | 2 |
| App -> ESP32 | `[9,0]` read PID and status (new) | 2 |
| App -> ESP32 | `[10,1]` heartbeat (new) | 2 |
| ESP32 -> App | `0x06` ACK; `0x15` NAK | 1 |
| ESP32 -> App | `0xC0` calibration fail / `0xC1` success / `0xC2` NVS loaded | 1 |
| ESP32 -> App | `0xD1` safety timeout | 1 |
| ESP32 -> App | `0xD0 + six PID bytes + flags + XOR` | **9** |

Status flags: bit 0 calibration valid, bit 1 robot running, bit 2 calibrating. Checksum XORs all six PID data bytes and flags (seven bytes after the `0xD0` marker).

The full upload is **12 UART bytes**, each acknowledgement is one byte. Heartbeat traffic is **two bytes per 600 ms** plus one-byte ACK. All individual messages are below 256 bytes. The current ESP32 serial implementation has no framing for the incoming two-byte messages; corrupted/lost bytes can cause parser misalignment. For robust competition telemetry, a framed protocol would be a future improvement.

## Safety and limitations

- Test with wheels lifted before placing the robot on a track.
- **STOP is a software command** and requires a working Bluetooth link; provide a physical power switch / kill switch.
- With `WAIT_FOR_APP = true` (as shipped), if the phone stops sending `[10,1]`, updated ESP32 firmware stops the motors after **2.5 seconds**. If you flash an older sketch, this timeout does **not** exist.
- The app also sends STOP when it leaves the foreground. Keeping the screen awake while running helps avoid involuntary disconnects.
- The existing ESP32 sketch runs a blocking calibration for 7 s and prevents motor movement until success. Avoid triggering calibration while the robot is in motion.
- There is no continuous sensor/error telemetry because the current ESP32 firmware only logs that information over USB. Connection status is based on the Bluetooth socket, not proof of motor motion or perfect delivery.
- HC-06 uses Bluetooth **Classic SPP** and does not work with Web Bluetooth / BLE-only controllers. iOS does not provide general HC-06 SPP access; this project is for Android.
- On some ESP32-S3 DevKit variants the onboard RGB LED is GPIO38 instead of GPIO48; adjust the firmware to match your board.
- QTR-8A analog outputs must not exceed the ESP32-S3 ADC input range. Use a 3.3 V sensor supply or proper scaling, and ensure TB6612FNG STBY is high.

## Version and build validation

Source was prepared against Android API 35 and Arduino-ESP32 3.x. The app is untested on a physical Android phone and the code has not been compiled with an actual Android SDK in this environment. Treat this as a build-ready project, not a verified prebuilt APK.
