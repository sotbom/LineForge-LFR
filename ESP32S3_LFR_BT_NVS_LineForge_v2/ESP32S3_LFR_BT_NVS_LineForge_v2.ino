/*
  ESP32-S3 LFR: QTR-8A + TB6612FNG + HC-05/HC-06 UART
  Arduino-ESP32 core 3.x | Board: ESP32S3 Dev Module | USB CDC On Boot: Enabled

  - First boot (or invalid NVS calibration): 2 s stationary on WHITE,
    then 5 s sweeping all 8 sensors over the BLACK line.
  - Valid calibration is saved in NVS and reused at the next boot.
  - PID tuning is saved in NVS 2 s after the last changed parameter;
    STOP also saves pending PID changes immediately.
  - Press BOOT (GPIO0) to stop immediately; hold for 2 s AFTER boot,
    or send [8, 1], to recalibrate. Put the array on WHITE first.
  - Recalibration stops the motors and requires a fresh START command.
  - START/STOP emit checksummed status replies so the
    unchanged phone app shows the actual ESP32 run state, not just a request.
  - START includes an adjustable 180 ms motor breakaway pulse. Test wheels up.
  - UART at 9600 baud: incoming commands are ALWAYS 2 binary bytes.
      [1,value] Kp numerator    [2,value] Kp decimal divider (0..4)
      [3,value] Ki numerator    [4,value] Ki decimal divider (0..4)
      [5,value] Kd numerator    [6,value] Kd decimal divider (0..4)
      [7,0] STOP               [7,1] START
      [8,1] Recalibrate (BOOT button is alternative)
    [9,0] Read back PID and status: reply = 9 bytes:
      [0xD0, kpVal, kpDiv, kiVal, kiDiv, kdVal, kdDiv, flags, checksum]
      flags bit 0=calibration OK, bit 1=running, bit 2=calibrating
      checksum = XOR of 7 bytes after 0xD0 (values + flags).
    [10,1] Heartbeat while running, every 600 ms; timeout 2500 ms.
    [11,1] Enable framed telemetry for 4 s (refresh periodically)
    [11,0] Disable telemetry; [12,0] Read white/black calibration arrays.
    Frame: A5 5A type length seq payload CRC8(poly=07 over type..payload).
    Type 51: status (8 payload bytes: six PID fields, flags, reserved).
    Type 52: eight QTR intensities + position (u16) + flags + signed PWM L/R.
    Type 53: stored white and black ADC raw readings (32 bytes).
    Replies: 0x06 ACK, 0x15 NAK, 0xC1 calibration success,
             0xC0 calibration failed, 0xC2 stored calibration loaded,
             0xD1 motor safety timeout.
    The old app's 2-byte command format is unchanged for commands 1..7.
    If the old app expects text responses, adapt its receive handler.

  IMPORTANT ELECTRICAL REQUIREMENTS:
    - QTR-8A analog outputs MUST stay within the ESP32-S3 input range.
      Power the QTR at 3.3 V (check sensitivity/LED resistor configuration).
    - TB6612FNG STBY must be held HIGH; common ground is essential.
    - Separate motor power and USB/battery connection safely in hardware.
    - GPIO48 is RGB LED on some boards; DevKitC-1 v1.1 uses GPIO38.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <stddef.h>   // offsetof
#include <math.h>

// ---------------- User options ----------------
const bool WAIT_FOR_APP = true;
const int RGB_LED_PIN = 48;              // use 38 for official DevKitC-1 v1.1
// 25/30% was often too weak to overcome motor static friction.
// Start with the wheels raised, then tune these for your motors and track.
const int BASE_SPEED = 45;              // percent of full PWM in PID mode
const int MAX_SPEED  = 65;              // maximum magnitude of either motor
const int START_PULSE_SPEED = 60;       // short straight startup pulse, percent
const uint32_t START_PULSE_MS = 180;    // disable by setting to 0
const uint32_t PID_SAVE_DELAY_MS = 2000;
const uint32_t CONTROL_PERIOD_US = 5000;
const uint32_t BOOT_HOLD_MS = 2000;
const uint32_t TELEMETRY_PERIOD_MS = 125; // 8Hz, 19-byte packet, ~152B/s
const uint32_t TELEMETRY_LEASE_MS = 4000; // off after app stops refreshing
// When WAIT_FOR_APP=false, the robot may run without a phone.
// Otherwise a lost Bluetooth connection stops motors within 2.5 s.
const bool REQUIRE_APP_HEARTBEAT = WAIT_FOR_APP;
const uint32_t HEARTBEAT_TIMEOUT_MS = 2500;
const int MIN_CAL_RANGE = 300;          // raw 12-bit ADC counts

// ---------------- Pin connections ----------------
const int AIN1 = 42, AIN2 = 41, PWMA = 21; // left motor
const int BIN1 = 40, BIN2 = 39, PWMB = 47; // right motor
const int QTR[8] = {7, 15, 16, 17, 18, 8, 9, 10};
const int BT_TX_PIN = 43;               // ESP TX -> HC-05 RX
const int BT_RX_PIN = 44;               // ESP RX <- HC-05 TX
const int USER_BOOT_GPIO  = 0;                // hold after startup to recalibrate

const int BASE_PWM = BASE_SPEED * 255 / 100;
const int MAX_PWM = MAX_SPEED * 255 / 100;
const int START_PULSE_PWM = START_PULSE_SPEED * 255 / 100;
// The HC-06 is wired to GPIO43/44, which are native UART0 pins.
// USB CDC On Boot must be ENABLED, so Serial stays on the USB port.
// Using Serial0 avoids remapping UART1 onto UART0's default pins.
HardwareSerial &BT = Serial0;
Preferences prefs;
bool prefsReady = false;

// ---------------- Bluetooth one-byte replies ----------------
const uint8_t ACK = 0x06;
const uint8_t NAK = 0x15;
const uint8_t CAL_SUCCESS = 0xC1;
const uint8_t CAL_FAILURE = 0xC0;
const uint8_t CAL_LOADED = 0xC2;
const uint8_t STATUS_REPLY = 0xD0;
const uint8_t PKT_STATUS = 0x51;
const uint8_t PKT_SENSOR = 0x52;
const uint8_t PKT_CAL = 0x53;
const uint8_t HEARTBEAT_TIMEOUT_REPLY = 0xD1;

// ---------------- Working state ----------------
int kpVal = 40, kpDiv = 0;
int kiVal = 0,  kiDiv = 0;
int kdVal = 1,  kdDiv = 0;
float Kp = 40.0f, Ki = 0.0f, Kd = 1.0f;
int white[8] = {0};
int black[8] = {0};
bool calOk = false;
bool calibrating = false;
bool calibrationPending = false;
bool robotOn = false;
bool pidLast = false;
bool startPulseActive = false;
uint32_t startPulseAtMs = 0;
bool pidDirty = false;
float lastError = 0.0f;
float lastSeenSide = 1.0f;
float iTerm = 0.0f;
uint32_t lastUs = 0;
uint32_t lastPIDChangeMs = 0;
uint32_t lastSaveAttemptMs = 0;
uint32_t lastHeartbeatMs = 0;
bool telemetryEnabled = false;
uint32_t lastTelemetryLeaseMs = 0;
uint32_t lastTelemetryMs = 0;
uint8_t frameSequence = 0;
int lastMotorLeft = 0, lastMotorRight = 0;

// ---------------- NVS blob definitions ----------------
const uint32_t CAL_MAGIC = 0x4C465243;  // 'LFRC'
const uint32_t PID_MAGIC = 0x4C465250;  // 'LFRP'
const uint16_t STORAGE_VERSION = 1;

struct CalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint16_t white[8];
  uint16_t black[8];
  uint32_t checksum;
};

struct PIDRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t kpVal, kpDiv;
  uint8_t kiVal, kiDiv;
  uint8_t kdVal, kdDiv;
  uint32_t checksum;
};

// FNV-1a detects mismatched/corrupted saved records; not a security MAC.
uint32_t checksum32(const void *data, size_t len) {
  const uint8_t *p = (const uint8_t *)data;
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    h ^= p[i];
    h *= 16777619UL;
  }
  return h;
}

void updatePID() {
  const float tenPow[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};
  Kp = kpVal / tenPow[kpDiv];
  Ki = kiVal / tenPow[kiDiv];
  Kd = kdVal / tenPow[kdDiv];
  if (Ki == 0.0f) iTerm = 0.0f;
}

bool loadPID() {
  if (!prefsReady || prefs.getBytesLength("pid") != sizeof(PIDRecord))
    return false;

  PIDRecord r = {};
  if (prefs.getBytes("pid", &r, sizeof(r)) != sizeof(r)) return false;
  if (r.magic != PID_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(PIDRecord, checksum)))
    return false;
  if (r.kpDiv > 4 || r.kiDiv > 4 || r.kdDiv > 4) return false;

  kpVal = r.kpVal; kpDiv = r.kpDiv;
  kiVal = r.kiVal; kiDiv = r.kiDiv;
  kdVal = r.kdVal; kdDiv = r.kdDiv;
  updatePID();
  return true;
}

bool savePID() {
  if (!prefsReady) return false;

  PIDRecord r = {};
  r.magic = PID_MAGIC;
  r.version = STORAGE_VERSION;
  r.kpVal = (uint8_t)kpVal; r.kpDiv = (uint8_t)kpDiv;
  r.kiVal = (uint8_t)kiVal; r.kiDiv = (uint8_t)kiDiv;
  r.kdVal = (uint8_t)kdVal; r.kdDiv = (uint8_t)kdDiv;
  r.checksum = checksum32(&r, offsetof(PIDRecord, checksum));

  lastSaveAttemptMs = millis();
  if (prefs.putBytes("pid", &r, sizeof(r)) == sizeof(r)) {
    pidDirty = false;
    Serial.println("PID saved to NVS");
    return true;
  }
  Serial.println("ERROR: PID NVS save failed");
  return false;
}

bool calibrationDataValid(const CalibrationRecord &r) {
  if (r.magic != CAL_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(CalibrationRecord, checksum)))
    return false;
  for (int s = 0; s < 8; ++s) {
    if (r.white[s] > 4095 || r.black[s] > 4095) return false;
    if ((int)r.black[s] - (int)r.white[s] < MIN_CAL_RANGE) return false;
  }
  return true;
}

bool loadCalibration() {
  if (!prefsReady ||
      prefs.getBytesLength("cal") != sizeof(CalibrationRecord))
    return false;

  CalibrationRecord r = {};
  if (prefs.getBytes("cal", &r, sizeof(r)) != sizeof(r)) return false;
  if (!calibrationDataValid(r)) return false;

  for (int s = 0; s < 8; ++s) {
    white[s] = r.white[s];
    black[s] = r.black[s];
  }
  return true;
}

bool saveCalibration() {
  if (!prefsReady) return false;

  CalibrationRecord r = {};
  r.magic = CAL_MAGIC;
  r.version = STORAGE_VERSION;
  for (int s = 0; s < 8; ++s) {
    r.white[s] = (uint16_t)white[s];
    r.black[s] = (uint16_t)black[s];
  }
  r.checksum = checksum32(&r, offsetof(CalibrationRecord, checksum));
  if (prefs.putBytes("cal", &r, sizeof(r)) == sizeof(r)) {
    Serial.println("Calibration saved to NVS");
    return true;
  }
  Serial.println("ERROR: Calibration NVS save failed");
  return false;
}

// ---------------- Hardware helpers ----------------
void showLED(uint8_t r, uint8_t g, uint8_t b) {
  rgbLedWrite(RGB_LED_PIN, r, g, b);
}

int readRaw(int pin) {
  analogRead(pin);  // discard the first read after channel selection
  uint32_t sum = 0;
  for (int k = 0; k < 4; ++k) sum += analogRead(pin);
  return (int)(sum / 4);
}

void driveMotor(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, (speed >= 0) ? HIGH : LOW);
  digitalWrite(in2, (speed >= 0) ? LOW : HIGH);
  ledcWrite(pwmPin, abs(speed));
}

void setMotors(int left, int right) {
  lastMotorLeft = left; lastMotorRight = right;
  driveMotor(AIN1, AIN2, PWMA, left);
  driveMotor(BIN1, BIN2, PWMB, right);
}

void stopRobot() {
  startPulseActive = false;
  robotOn = false;
  pidLast = false;
  iTerm = 0.0f;
  setMotors(0, 0);
}

// CRC8 (poly x^8 + x^2 + x + 1), initial value 0, no final XOR.
uint8_t crc8Update(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (uint8_t n = 0; n < 8; n++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  return crc;
}

void sendFramed(uint8_t type, const uint8_t *payload, uint8_t len) {
  if (len > 48) return;
  uint8_t frame[54];
  frame[0] = 0xA5; frame[1] = 0x5A;
  frame[2] = type; frame[3] = len; frame[4] = frameSequence++;
  uint8_t crc = 0;
  for (int i = 2; i < 5; i++) crc = crc8Update(crc, frame[i]);
  for (int i = 0; i < len; i++) {
    frame[5 + i] = payload[i];
    crc = crc8Update(crc, payload[i]);
  }
  frame[5 + len] = crc;
  BT.write(frame, 6 + len);
}

void sendCalibrationFrame() {
  uint8_t p[32];
  for (int i = 0; i < 8; i++) {
    p[4*i+0] = (uint8_t)(white[i] & 0xff);
    p[4*i+1] = (uint8_t)(white[i] >> 8);
    p[4*i+2] = (uint8_t)(black[i] & 0xff);
    p[4*i+3] = (uint8_t)(black[i] >> 8);
  }
  sendFramed(PKT_CAL, p, sizeof(p));
}

// Eight calibrated intensities (0 white to 255 black) plus line position.
// During calibration send raw 12-bit ADC scaled down to 8 bits instead.
void sendSensorFrameIfDue() {
  if (!telemetryEnabled) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - lastTelemetryLeaseMs) > TELEMETRY_LEASE_MS) {
    telemetryEnabled = false;
    return;
  }
  if ((uint32_t)(now - lastTelemetryMs) < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  uint8_t p[13] = {};
  long weighted = 0, total = 0;
  for (int s = 0; s < 8; s++) {
    int raw = readRaw(QTR[s]);
    int signal = 0;
    if (calOk && black[s] - white[s] >= MIN_CAL_RANGE)
      signal = constrain((int32_t)(raw - white[s]) * 1000 /
                (black[s] - white[s]), (int32_t)0, (int32_t)1000);
    else
      signal = raw * 1000L / 4095L;
    p[s] = (uint8_t)((signal * 255L + 500) / 1000);
    if (calOk && signal > 250) {
      weighted += (long)(signal - 250) * s;
      total += (long)(signal - 250);
    }
  }
  uint16_t pos = total > 0 ? (uint16_t)(weighted * 1000L / total) : 0xffff;
  p[8] = (uint8_t)(pos & 255); p[9] = (uint8_t)(pos >> 8);
  p[10] = ((total > 0) ? 1 : 0) | (calOk ? 2 : 0) |
          (robotOn ? 4 : 0) | (calibrating ? 8 : 0);
  p[11] = (uint8_t)(int8_t)constrain(lastMotorLeft * 100 / 255, -100, 100);
  p[12] = (uint8_t)(int8_t)constrain(lastMotorRight * 100 / 255, -100, 100);
  sendFramed(PKT_SENSOR, p, sizeof(p));
}

// 9-byte legacy status message or CRC-framed status (when telemetry enabled).
// 9-byte status message. This also lets the app read NVS-restored PID settings.
void sendStatusFrame() {
  uint8_t frame[9];
  frame[0] = STATUS_REPLY;
  frame[1] = (uint8_t)kpVal;
  frame[2] = (uint8_t)kpDiv;
  frame[3] = (uint8_t)kiVal;
  frame[4] = (uint8_t)kiDiv;
  frame[5] = (uint8_t)kdVal;
  frame[6] = (uint8_t)kdDiv;
  frame[7] = (calOk ? 0x01 : 0) | (robotOn ? 0x02 : 0) |
             (calibrating ? 0x04 : 0);
  uint8_t crc = 0;
  for (int i = 1; i <= 7; i++) crc ^= frame[i];
  frame[8] = crc;
  if (telemetryEnabled) {
    uint8_t p[8];
    for (int i = 0; i < 7; ++i) p[i] = frame[i+1];
    p[7] = 0; // reserved for future protocol versions
    sendFramed(PKT_STATUS, p, sizeof(p));
  } else BT.write(frame, sizeof(frame));
}

// ---------------- Bluetooth command reader ----------------
// Called frequently, including inside the blocking calibration routine.
void readBluetooth() {
  while (BT.available() >= 2) {
    int cmd = BT.peek();
    if (cmd < 1 || cmd > 12) {
      BT.read();  // skip invalid leading byte
      continue;
    }
    BT.read();
    int val = BT.read();

    if (((cmd == 2 || cmd == 4 || cmd == 6) && val > 4) ||
        (cmd == 7 && val > 1) ||
        (cmd == 8 && val != 1) ||
        (cmd == 9 && val != 0) ||
        (cmd == 10 && val != 1) ||
        (cmd == 11 && val > 1) ||
        (cmd == 12 && val != 0)) {
      BT.write(NAK);
      continue;
    }

    if (cmd == 11) {
      telemetryEnabled = (val == 1);
      lastTelemetryLeaseMs = millis();
      if (val == 1) lastTelemetryMs = 0;
      BT.write(ACK);
      continue;
    }
    if (cmd == 12) {
      if (telemetryEnabled) sendCalibrationFrame();
      else BT.write(NAK);
      continue;
    }
    if (cmd == 9) {
      sendStatusFrame();  // 9 bytes, no additional ACK
      continue;
    }
    if (cmd == 10) {
      lastHeartbeatMs = millis();
      // Heartbeats are deliberately silent in telemetry mode.
      if (!telemetryEnabled) BT.write(ACK);
      continue;
    }
    if (cmd == 8) {
      if (calibrating || calibrationPending) {
        BT.write(NAK);
      } else {
        calibrationPending = true;
        stopRobot();
        BT.write(ACK);
      }
      continue;
    }

    if (cmd == 7) {
      if (val == 0) {
        stopRobot();
        BT.write(ACK);
        sendStatusFrame(); // App already parses this: confirms motors DISARMED.
        Serial.println("BT STOP accepted; motor PWM = 0");
        if (pidDirty) savePID();
      } else if (calOk && !calibrating && !calibrationPending) {
        robotOn = true;
        pidLast = false;
        iTerm = 0.0f;
        lastHeartbeatMs = millis();
        lastUs = micros();
        startPulseAtMs = millis();
        startPulseActive = START_PULSE_MS > 0;

        // Drive both motors immediately, before the first QTR PID sample.
        // Gives low-speed motors an initial kick against static friction.
        if (startPulseActive) {
          setMotors(START_PULSE_PWM, START_PULSE_PWM);
        }
        BT.write(ACK);
        sendStatusFrame(); // App already parses this: confirms START accepted.
        Serial.printf("BT START accepted: startup PWM=%d/%d, PID base=%d max=%d\n",
                      START_PULSE_PWM, 255, BASE_PWM, MAX_PWM);
      } else {
        BT.write(NAK);
        sendStatusFrame(); // Revert app's optimistic RUNNING UI to actual state.
        Serial.printf("BT START rejected: calOk=%d calibrating=%d pending=%d\n",
                      calOk, calibrating, calibrationPending);
      }
      continue;
    }

    bool changed = false;
    switch (cmd) {
      case 1: changed = (kpVal != val); kpVal = val; break;
      case 2: changed = (kpDiv != val); kpDiv = val; break;
      case 3: changed = (kiVal != val); kiVal = val; break;
      case 4: changed = (kiDiv != val); kiDiv = val; break;
      case 5: changed = (kdVal != val); kdVal = val; break;
      case 6: changed = (kdDiv != val); kdDiv = val; break;
    }
    if (changed) {
      updatePID();
      pidDirty = true;
      lastPIDChangeMs = millis();
    }
    BT.write(ACK);
  }
}

// ---------------- Calibration routine ----------------
// Returns true only when all eight sensors have usable white-to-black range.
// Does NOT overwrite valid saved calibration if a new calibration fails.
bool performCalibration() {
  calibrating = true;
  calibrationPending = false;
  calOk = false;
  stopRobot();
  showLED(40, 40, 40);  // WHITE: calibrating

  uint32_t sum[8] = {0};
  uint32_t samples = 0;
  uint32_t t0 = millis();

  Serial.println("Calibration: keep all sensors on WHITE for 2 s");
  while ((uint32_t)(millis() - t0) < 2000U) {
    for (int s = 0; s < 8; ++s) sum[s] += (uint32_t)readRaw(QTR[s]);
    ++samples;
    readBluetooth();
    sendSensorFrameIfDue();
    yield();
  }
  int newWhite[8];
  int newBlack[8] = {0};
  for (int s = 0; s < 8; ++s)
    newWhite[s] = (int)(sum[s] / samples);

  Serial.println("Calibration: sweep all sensors over BLACK for 5 s");
  t0 = millis();
  while ((uint32_t)(millis() - t0) < 5000U) {
    for (int s = 0; s < 8; ++s) {
      int raw = readRaw(QTR[s]);
      if (raw > newBlack[s]) newBlack[s] = raw;
    }
    readBluetooth();
    sendSensorFrameIfDue();
    yield();
  }

  bool good = true;
  for (int s = 0; s < 8; ++s) {
    Serial.printf("D%d white %4d black %4d range %4d\n", s + 1,
                  newWhite[s], newBlack[s], newBlack[s] - newWhite[s]);
    if (newBlack[s] - newWhite[s] < MIN_CAL_RANGE) good = false;
  }

  if (good) {
    for (int s = 0; s < 8; ++s) {
      white[s] = newWhite[s];
      black[s] = newBlack[s];
    }
    calOk = true;
    saveCalibration();  // valid in RAM even if NVS write fails
    showLED(0, 40, 0);  // GREEN: calibration OK
    Serial.println("Calibration OK");
    BT.write(CAL_SUCCESS);
    if (telemetryEnabled) { sendCalibrationFrame(); sendStatusFrame(); }
  } else {
    calOk = false;
    showLED(40, 0, 0);  // RED: calibration failed
    Serial.println("Calibration FAILED: motors disabled");
    BT.write(CAL_FAILURE);
    if (telemetryEnabled) sendStatusFrame();
  }
  calibrating = false;
  lastUs = micros();
  return good;
}

// Hold BOOT after startup to force recalibration, no app update required.
// Do not hold BOOT through reset/power-up; that enters flash download mode.
void checkBootButton() {
  static bool previouslyDown = false;
  static bool triggered = false;
  static uint32_t pressedAt = 0;

  bool down = (digitalRead(USER_BOOT_GPIO) == LOW);
  if (down && !previouslyDown) {
    pressedAt = millis();
    triggered = false;
    stopRobot();  // immediate stop, even for a short press
  }
  if (down && !triggered && !calibrating &&
      (uint32_t)(millis() - pressedAt) >= BOOT_HOLD_MS) {
    triggered = true;
    calibrationPending = true;
    stopRobot();
  }
  if (!down) triggered = false;
  previouslyDown = down;
}

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(USER_BOOT_GPIO, INPUT_PULLUP);

  // Start outputs at 0 duty; STBY on TB6612 must be held HIGH in hardware.
  bool leftPWMok = ledcAttach(PWMA, 20000, 8);
  bool rightPWMok = ledcAttach(PWMB, 20000, 8);
  setMotors(0, 0);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.begin(115200);
  BT.setRxBufferSize(512);  // default size, made explicit
  BT.begin(9600, SERIAL_8N1, BT_RX_PIN, BT_TX_PIN);
  Serial.println("HC-06 opened on UART0 GPIO44/RX and GPIO43/TX at 9600 baud");

  prefsReady = prefs.begin("lfr", false);
  if (!prefsReady) Serial.println("WARNING: NVS unavailable; settings won't persist");
  if (!leftPWMok || !rightPWMok) {
    Serial.println("FATAL: PWM setup failed");
    showLED(40, 0, 0);
    while (true) { setMotors(0, 0); delay(100); }
  }

  if (loadPID()) {
    Serial.println("Stored PID loaded from NVS");
  } else {
    updatePID();
    Serial.println("Using default PID");
    savePID();
  }
  Serial.printf("Kp %.4f Ki %.4f Kd %.4f\n", Kp, Ki, Kd);

  if (loadCalibration()) {
    calOk = true;
    showLED(0, 40, 0);
    Serial.println("Stored calibration loaded; skipping 7 s calibration");
    BT.write(CAL_LOADED);
  } else {
    Serial.println("No valid saved calibration; calibrating now");
    performCalibration();
  }

  // Only an initial startup is allowed to auto-start when configured.
  // Runtime recalibration always stops until START is explicitly requested.
  if (calOk && !WAIT_FOR_APP) {
    robotOn = true;
    lastHeartbeatMs = millis();
  }
  lastUs = micros();
}

void loop() {
  readBluetooth();
  checkBootButton();
  sendSensorFrameIfDue();

  if (REQUIRE_APP_HEARTBEAT && robotOn &&
      (uint32_t)(millis() - lastHeartbeatMs) > HEARTBEAT_TIMEOUT_MS) {
    stopRobot();
    BT.write(HEARTBEAT_TIMEOUT_REPLY);
    sendStatusFrame();
    Serial.println("Safety stop: no app heartbeat for 2.5 seconds");
  }

  if (calibrationPending) {
    performCalibration();
    // Deliberately remain stopped, even when WAIT_FOR_APP is false.
    stopRobot();
    return;
  }

  // Save only after the user stops changing PID values, to limit flash wear.
  // NVS write may briefly interrupt the control loop.
  if (!robotOn && pidDirty &&
      (uint32_t)(millis() - lastPIDChangeMs) >= PID_SAVE_DELAY_MS &&
      (lastSaveAttemptMs == 0 ||
       (uint32_t)(millis() - lastSaveAttemptMs) >= 5000U)) {
    savePID();
  }

  if (!calOk || !robotOn) return; // stopRobot() already set zero duty

  // Nonblocking startup pulse: still checks Bluetooth, BOOT and heartbeat.
  // No delay() here; STOP always acts immediately.
  if (startPulseActive) {
    if ((uint32_t)(millis() - startPulseAtMs) < START_PULSE_MS) return;
    startPulseActive = false;
    pidLast = false;
    lastUs = micros();
    Serial.println("Startup pulse complete; PID mode entered");
    return;
  }

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastUs) < CONTROL_PERIOD_US) return;
  float dt = (uint32_t)(nowUs - lastUs) * 1.0e-6f;
  lastUs = nowUs;

  int calibrated[8];
  int32_t weighted = 0, total = 0;
  for (int s = 0; s < 8; ++s) {
    int raw = readRaw(QTR[s]);
    int32_t span = black[s] - white[s]; // >= MIN_CAL_RANGE if calOk
    calibrated[s] = constrain((int32_t)(raw - white[s]) * 1000 / span,
                              (int32_t)0, (int32_t)1000);
    if (calibrated[s] > 250) {
      weighted += (int32_t)(calibrated[s] - 250) * s;
      total += calibrated[s] - 250;
    }
  }

  float error;
  if (total > 0) {
    error = (float)weighted / (float)total - 3.5f;
    if (error > 0.25f) lastSeenSide = 1.0f;
    else if (error < -0.25f) lastSeenSide = -1.0f;
  } else {
    error = 3.5f * lastSeenSide;  // search toward last known line side
    iTerm = 0.0f;               // prevent windup while line is lost
  }

  if (!pidLast) lastError = error;  // no derivative spike on START
  if (Ki == 0.0f) iTerm = 0.0f;
  else if (total > 0) iTerm = constrain(iTerm + Ki * error * dt, -40.0f, 40.0f);

  float correction = Kp * error + iTerm + Kd * (error - lastError) / dt;
  lastError = error;
  pidLast = true;

  correction = constrain(correction, (float)(-2 * MAX_PWM),
                         (float)(2 * MAX_PWM));
  int left = BASE_PWM + (int)correction;
  int right = BASE_PWM - (int)correction;
  if (left > MAX_PWM) { right -= left - MAX_PWM; left = MAX_PWM; }
  if (right > MAX_PWM) { left -= right - MAX_PWM; right = MAX_PWM; }
  left = constrain(left, -MAX_PWM, MAX_PWM);
  right = constrain(right, -MAX_PWM, MAX_PWM);
  setMotors(left, right);

  // USB-only debugging, never transmitted over Bluetooth.
  static uint32_t lastPrintMs = 0;
  if (Serial && (uint32_t)(millis() - lastPrintMs) >= 100U) {
    lastPrintMs = millis();
    for (int s = 0; s < 8; ++s) Serial.printf("%5d", calibrated[s]);
    Serial.printf(" | err %+.2f | L %+4d R %+4d | Kp %.3f Ki %.3f Kd %.3f | dt %.1fms\n",
                  error, left, right, Kp, Ki, Kd, dt * 1000.0f);
  }
}
