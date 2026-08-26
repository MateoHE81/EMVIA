/*
  Prototype A: Event-Triggered Feedback
  -------------------------------------
  ESP32 receives one-time football event commands from Sync.py:

    PING
    STOP
    EVENT,SHOT
    EVENT,FOUL
    EVENT,PENALTY
    EVENT,GOAL

  Feedback logic:
    SHOT     Short medium vibration + LED flash, no PTC
    FOUL     Two short vibration/LED pulses, no PTC
    PENALTY  Escalating tension pattern + low PTC power
    GOAL     Strong rhythmic feedback + limited PTC power

  Sensor logging:
    DATA,esp_ms,bpm,ir,ax,ay,az,gx,gy,gz,active_event,led,vib,ptc

  Hardware:
    MAX30102 SDA -> GPIO21
    MAX30102 SCL -> GPIO22
    IMU SDA      -> GPIO21
    IMU SCL      -> GPIO22

    Vibration MOSFET PWM -> GPIO25
    PTC MOSFET PWM       -> GPIO26
    LED MOSFET PWM       -> GPIO27

  Notes:
    - All grounds must be connected together.
    - Each vibration motor needs its own flyback diode.
    - PTC is open-loop. Test away from skin first.
    - This code supports a standard MPU6050 WHO_AM_I 0x68 and the user's
      compatible module returning WHO_AM_I 0x70 by reading registers directly.
*/

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

// -----------------------------------------------------------------------------
// Pins and serial
// -----------------------------------------------------------------------------

constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

constexpr uint8_t PIN_VIBRATION = 25;
constexpr uint8_t PIN_PTC       = 26;
constexpr uint8_t PIN_LED       = 27;

constexpr uint32_t SERIAL_BAUD = 115200;

// -----------------------------------------------------------------------------
// PWM settings
// -----------------------------------------------------------------------------

constexpr uint8_t PWM_BITS = 8;
constexpr uint16_t VIBRATION_PWM_FREQUENCY = 250;
constexpr uint16_t PTC_PWM_FREQUENCY       = 100;
constexpr uint16_t LED_PWM_FREQUENCY       = 1000;

constexpr uint8_t CHANNEL_VIBRATION = 0;
constexpr uint8_t CHANNEL_PTC       = 1;
constexpr uint8_t CHANNEL_LED       = 2;

// PTC hard safety limits.
// 90 / 255 is about 35% maximum electrical duty.
constexpr uint8_t PTC_HARD_MAX_DUTY = 90;
constexpr uint32_t EVENT_HARD_TIMEOUT_MS = 6500;

// -----------------------------------------------------------------------------
// Sensor timing
// -----------------------------------------------------------------------------

constexpr uint32_t HEART_UPDATE_INTERVAL_MS = 10;
constexpr uint32_t IMU_UPDATE_INTERVAL_MS   = 20;
constexpr uint32_t LOG_INTERVAL_MS          = 100;

constexpr uint32_t FINGER_IR_THRESHOLD = 50000;
constexpr uint8_t BPM_AVERAGE_SIZE = 4;

// -----------------------------------------------------------------------------
// MPU-compatible direct register driver
// -----------------------------------------------------------------------------

constexpr uint8_t MPU_ADDRESS = 0x68;

constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr float ACCEL_SCALE_LSB_PER_G = 4096.0f;  // ±8 g
constexpr float GYRO_SCALE_LSB_PER_DPS = 65.5f;   // ±500 degrees/s

// -----------------------------------------------------------------------------
// Global sensor state
// -----------------------------------------------------------------------------

MAX30105 particleSensor;

bool max30102Ready = false;
bool imuReady = false;
uint8_t imuWhoAmI = 0x00;

uint32_t irValue = 0;
float currentBpm = 0.0f;

uint32_t lastBeatMs = 0;
uint8_t bpmValues[BPM_AVERAGE_SIZE] = {0};
uint8_t bpmIndex = 0;
uint8_t bpmCount = 0;

float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

// -----------------------------------------------------------------------------
// Event state
// -----------------------------------------------------------------------------

enum class EventType : uint8_t {
  NONE = 0,
  SHOT,
  FOUL,
  PENALTY,
  GOAL
};

EventType activeEvent = EventType::NONE;
uint32_t eventStartMs = 0;

uint8_t currentLedDuty = 0;
uint8_t currentVibrationDuty = 0;
uint8_t currentPtcDuty = 0;

// -----------------------------------------------------------------------------
// Timing and serial buffer
// -----------------------------------------------------------------------------

uint32_t lastHeartUpdateMs = 0;
uint32_t lastImuUpdateMs = 0;
uint32_t lastLogMs = 0;

char serialBuffer[80];
size_t serialBufferLength = 0;

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------

uint8_t clampDuty(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

const char* eventName(EventType event) {
  switch (event) {
    case EventType::SHOT:
      return "SHOT";
    case EventType::FOUL:
      return "FOUL";
    case EventType::PENALTY:
      return "PENALTY";
    case EventType::GOAL:
      return "GOAL";
    default:
      return "NONE";
  }
}

uint8_t eventPriority(EventType event) {
  switch (event) {
    case EventType::SHOT:
      return 1;
    case EventType::FOUL:
      return 2;
    case EventType::PENALTY:
      return 3;
    case EventType::GOAL:
      return 4;
    default:
      return 0;
  }
}

EventType parseEventName(const char* value) {
  if (strcmp(value, "SHOT") == 0) {
    return EventType::SHOT;
  }
  if (strcmp(value, "FOUL") == 0) {
    return EventType::FOUL;
  }
  if (strcmp(value, "PENALTY") == 0) {
    return EventType::PENALTY;
  }
  if (strcmp(value, "GOAL") == 0) {
    return EventType::GOAL;
  }
  return EventType::NONE;
}

// -----------------------------------------------------------------------------
// PWM compatibility for ESP32 Arduino core 2.x and 3.x
// -----------------------------------------------------------------------------

void setupPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_VIBRATION, VIBRATION_PWM_FREQUENCY, PWM_BITS);
  ledcAttach(PIN_PTC, PTC_PWM_FREQUENCY, PWM_BITS);
  ledcAttach(PIN_LED, LED_PWM_FREQUENCY, PWM_BITS);
#else
  ledcSetup(CHANNEL_VIBRATION, VIBRATION_PWM_FREQUENCY, PWM_BITS);
  ledcSetup(CHANNEL_PTC, PTC_PWM_FREQUENCY, PWM_BITS);
  ledcSetup(CHANNEL_LED, LED_PWM_FREQUENCY, PWM_BITS);

  ledcAttachPin(PIN_VIBRATION, CHANNEL_VIBRATION);
  ledcAttachPin(PIN_PTC, CHANNEL_PTC);
  ledcAttachPin(PIN_LED, CHANNEL_LED);
#endif
}

void writeVibration(uint8_t duty) {
  currentVibrationDuty = duty;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_VIBRATION, duty);
#else
  ledcWrite(CHANNEL_VIBRATION, duty);
#endif
}

void writePtc(uint8_t duty) {
  duty = min(duty, PTC_HARD_MAX_DUTY);
  currentPtcDuty = duty;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PTC, duty);
#else
  ledcWrite(CHANNEL_PTC, duty);
#endif
}

void writeLed(uint8_t duty) {
  currentLedDuty = duty;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_LED, duty);
#else
  ledcWrite(CHANNEL_LED, duty);
#endif
}

void applyOutputs(uint8_t ledDuty, uint8_t vibrationDuty, uint8_t ptcDuty) {
  writeLed(ledDuty);
  writeVibration(vibrationDuty);
  writePtc(ptcDuty);
}

void stopFeedback() {
  activeEvent = EventType::NONE;
  eventStartMs = 0;
  applyOutputs(0, 0, 0);
}

// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------

bool i2cWriteByte(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cReadBytes(
  uint8_t address,
  uint8_t startReg,
  uint8_t* destination,
  size_t length
) {
  Wire.beginTransmission(address);
  Wire.write(startReg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t requested = Wire.requestFrom(
    address,
    static_cast<uint8_t>(length),
    true
  );

  if (requested != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    if (!Wire.available()) {
      return false;
    }
    destination[index] = Wire.read();
  }

  return true;
}

bool readWhoAmI(uint8_t& identity) {
  return i2cReadBytes(
    MPU_ADDRESS,
    REG_WHO_AM_I,
    &identity,
    1
  );
}

bool acceptedImuIdentity(uint8_t identity) {
  return (
    identity == 0x68 ||  // standard MPU6050
    identity == 0x70 ||  // user's compatible module
    identity == 0x71 ||  // common MPU9250 family ID
    identity == 0x73
  );
}

// -----------------------------------------------------------------------------
// Sensor setup
// -----------------------------------------------------------------------------

void setupMax30102() {
  max30102Ready = particleSensor.begin(
    Wire,
    I2C_SPEED_STANDARD
  );

  if (!max30102Ready) {
    Serial.println("READY,MAX30102,NOT_FOUND");
    return;
  }

  // brightness, sample average, LED mode, sample rate,
  // pulse width, ADC range
  particleSensor.setup(
    60,
    4,
    2,
    100,
    411,
    4096
  );

  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0x00);

  Serial.println("READY,MAX30102,OK");
}

void setupImu() {
  if (!readWhoAmI(imuWhoAmI)) {
    Serial.println("READY,IMU,NOT_FOUND");
    imuReady = false;
    return;
  }

  if (!acceptedImuIdentity(imuWhoAmI)) {
    Serial.print("READY,IMU,UNSUPPORTED,WHO_AM_I=0x");
    if (imuWhoAmI < 0x10) {
      Serial.print('0');
    }
    Serial.println(imuWhoAmI, HEX);
    imuReady = false;
    return;
  }

  bool ok = true;

  // Wake device and use X-axis gyro PLL where supported.
  ok &= i2cWriteByte(MPU_ADDRESS, REG_PWR_MGMT_1, 0x01);
  delay(50);

  // About 200 Hz sample generation before loop downsampling.
  ok &= i2cWriteByte(MPU_ADDRESS, REG_SMPLRT_DIV, 0x04);

  // DLPF setting 4, around 20 Hz class filtering.
  ok &= i2cWriteByte(MPU_ADDRESS, REG_CONFIG, 0x04);

  // ±500 degrees/s.
  ok &= i2cWriteByte(MPU_ADDRESS, REG_GYRO_CONFIG, 0x08);

  // ±8 g.
  ok &= i2cWriteByte(MPU_ADDRESS, REG_ACCEL_CONFIG, 0x10);

  imuReady = ok;

  if (imuReady) {
    Serial.print("READY,IMU,OK,WHO_AM_I=0x");
  } else {
    Serial.print("READY,IMU,CONFIG_FAILED,WHO_AM_I=0x");
  }

  if (imuWhoAmI < 0x10) {
    Serial.print('0');
  }
  Serial.println(imuWhoAmI, HEX);
}

// -----------------------------------------------------------------------------
// Sensor updates
// -----------------------------------------------------------------------------

void updateHeartRate(uint32_t nowMs) {
  if (!max30102Ready) {
    irValue = 0;
    currentBpm = 0.0f;
    return;
  }

  if (nowMs - lastHeartUpdateMs < HEART_UPDATE_INTERVAL_MS) {
    return;
  }

  lastHeartUpdateMs = nowMs;

  particleSensor.check();

  while (particleSensor.available()) {
    irValue = particleSensor.getFIFOIR();

    if (irValue >= FINGER_IR_THRESHOLD) {
      if (checkForBeat(irValue)) {
        const uint32_t beatTimeMs = millis();

        if (lastBeatMs != 0) {
          const uint32_t deltaMs = beatTimeMs - lastBeatMs;

          if (deltaMs > 0) {
            const float bpm = 60000.0f / static_cast<float>(deltaMs);

            if (bpm >= 25.0f && bpm <= 220.0f) {
              bpmValues[bpmIndex] = static_cast<uint8_t>(bpm + 0.5f);
              bpmIndex = (bpmIndex + 1) % BPM_AVERAGE_SIZE;

              if (bpmCount < BPM_AVERAGE_SIZE) {
                bpmCount++;
              }

              uint16_t total = 0;

              for (uint8_t index = 0; index < bpmCount; ++index) {
                total += bpmValues[index];
              }

              currentBpm = static_cast<float>(total) /
                           static_cast<float>(bpmCount);
            }
          }
        }

        lastBeatMs = beatTimeMs;
      }
    } else {
      currentBpm = 0.0f;
      lastBeatMs = 0;
      bpmCount = 0;
      bpmIndex = 0;
    }

    particleSensor.nextSample();
  }
}

void updateImu(uint32_t nowMs) {
  if (!imuReady) {
    ax = ay = az = 0.0f;
    gx = gy = gz = 0.0f;
    return;
  }

  if (nowMs - lastImuUpdateMs < IMU_UPDATE_INTERVAL_MS) {
    return;
  }

  lastImuUpdateMs = nowMs;

  uint8_t raw[14];

  if (!i2cReadBytes(
        MPU_ADDRESS,
        REG_ACCEL_XOUT_H,
        raw,
        sizeof(raw)
      )) {
    return;
  }

  const int16_t rawAx = static_cast<int16_t>(
    (static_cast<uint16_t>(raw[0]) << 8) | raw[1]
  );
  const int16_t rawAy = static_cast<int16_t>(
    (static_cast<uint16_t>(raw[2]) << 8) | raw[3]
  );
  const int16_t rawAz = static_cast<int16_t>(
    (static_cast<uint16_t>(raw[4]) << 8) | raw[5]
  );

  const int16_t rawGx = static_cast<int16_t>(
    (static_cast<uint16_t>(raw[8]) << 8) | raw[9]
  );
  const int16_t rawGy = static_cast<int16_t>(
    (static_cast<uint16_t>(raw[10]) << 8) | raw[11]
  );
  const int16_t rawGz = static_cast<int16_t>(
    (static_cast<uint16_t>(raw[12]) << 8) | raw[13]
  );

  ax = static_cast<float>(rawAx) / ACCEL_SCALE_LSB_PER_G;
  ay = static_cast<float>(rawAy) / ACCEL_SCALE_LSB_PER_G;
  az = static_cast<float>(rawAz) / ACCEL_SCALE_LSB_PER_G;

  gx = static_cast<float>(rawGx) / GYRO_SCALE_LSB_PER_DPS;
  gy = static_cast<float>(rawGy) / GYRO_SCALE_LSB_PER_DPS;
  gz = static_cast<float>(rawGz) / GYRO_SCALE_LSB_PER_DPS;
}

// -----------------------------------------------------------------------------
// Feedback pattern helpers
// -----------------------------------------------------------------------------

uint8_t triangularPulse(
  uint32_t elapsedMs,
  uint32_t periodMs,
  uint8_t minimumDuty,
  uint8_t maximumDuty
) {
  if (periodMs == 0 || maximumDuty <= minimumDuty) {
    return maximumDuty;
  }

  const uint32_t phase = elapsedMs % periodMs;
  const float halfPeriod = static_cast<float>(periodMs) / 2.0f;

  float normalized;

  if (phase <= halfPeriod) {
    normalized = static_cast<float>(phase) / halfPeriod;
  } else {
    normalized = static_cast<float>(periodMs - phase) / halfPeriod;
  }

  const float duty = static_cast<float>(minimumDuty) +
    normalized * static_cast<float>(maximumDuty - minimumDuty);

  return clampDuty(static_cast<int>(duty + 0.5f));
}

bool pulseWindow(
  uint32_t elapsedMs,
  uint32_t startMs,
  uint32_t durationMs
) {
  return (
    elapsedMs >= startMs &&
    elapsedMs < startMs + durationMs
  );
}

// -----------------------------------------------------------------------------
// Event patterns
// -----------------------------------------------------------------------------

bool updateShotPattern(uint32_t elapsedMs) {
  constexpr uint32_t DURATION_MS = 700;

  if (elapsedMs >= DURATION_MS) {
    return false;
  }

  uint8_t led = 0;
  uint8_t vibration = 0;

  if (elapsedMs < 180) {
    led = 240;
    vibration = 175;
  } else if (elapsedMs < 280) {
    led = 0;
    vibration = 0;
  } else if (elapsedMs < 500) {
    led = 185;
    vibration = 135;
  } else {
    const float fade = 1.0f -
      static_cast<float>(elapsedMs - 500) / 200.0f;

    led = clampDuty(static_cast<int>(120.0f * fade));
    vibration = clampDuty(static_cast<int>(90.0f * fade));
  }

  applyOutputs(led, vibration, 0);
  return true;
}

bool updateFoulPattern(uint32_t elapsedMs) {
  constexpr uint32_t DURATION_MS = 1200;

  if (elapsedMs >= DURATION_MS) {
    return false;
  }

  const bool pulseOne = pulseWindow(elapsedMs, 0, 220);
  const bool pulseTwo = pulseWindow(elapsedMs, 430, 220);

  if (pulseOne || pulseTwo) {
    applyOutputs(220, 165, 0);
  } else {
    applyOutputs(0, 0, 0);
  }

  return true;
}

bool updatePenaltyPattern(uint32_t elapsedMs) {
  constexpr uint32_t DURATION_MS = 4500;

  if (elapsedMs >= DURATION_MS) {
    return false;
  }

  const float progress = min(
    1.0f,
    static_cast<float>(elapsedMs) /
    static_cast<float>(DURATION_MS)
  );

  const uint32_t periodMs = static_cast<uint32_t>(
    620.0f - 300.0f * progress
  );

  const uint8_t ledMinimum = clampDuty(
    static_cast<int>(50.0f + 30.0f * progress)
  );
  const uint8_t ledMaximum = clampDuty(
    static_cast<int>(150.0f + 80.0f * progress)
  );

  const uint8_t vibrationMinimum = clampDuty(
    static_cast<int>(45.0f + 25.0f * progress)
  );
  const uint8_t vibrationMaximum = clampDuty(
    static_cast<int>(120.0f + 65.0f * progress)
  );

  const uint8_t led = triangularPulse(
    elapsedMs,
    periodMs,
    ledMinimum,
    ledMaximum
  );

  const uint8_t vibration = triangularPulse(
    elapsedMs,
    periodMs,
    vibrationMinimum,
    vibrationMaximum
  );

  uint8_t ptc = 0;

  if (elapsedMs >= 500 && elapsedMs < 4100) {
    ptc = 58;
  }

  applyOutputs(led, vibration, ptc);
  return true;
}

bool updateGoalPattern(uint32_t elapsedMs) {
  constexpr uint32_t DURATION_MS = 6000;

  if (elapsedMs >= DURATION_MS) {
    return false;
  }

  // Strong rhythmic celebration pattern.
  const uint32_t phase = elapsedMs % 800;

  uint8_t led = 0;
  uint8_t vibration = 0;

  if (phase < 180) {
    led = 255;
    vibration = 220;
  } else if (phase < 260) {
    led = 100;
    vibration = 70;
  } else if (phase < 440) {
    led = 235;
    vibration = 200;
  } else if (phase < 540) {
    led = 80;
    vibration = 50;
  } else {
    led = 190;
    vibration = 155;
  }

  uint8_t ptc = 0;

  if (elapsedMs < 5200) {
    ptc = 82;
  }

  // Gentle fade in the final 800 ms.
  if (elapsedMs >= 5200) {
    const float fade = 1.0f -
      static_cast<float>(elapsedMs - 5200) / 800.0f;

    led = clampDuty(static_cast<int>(led * fade));
    vibration = clampDuty(static_cast<int>(vibration * fade));
  }

  applyOutputs(led, vibration, ptc);
  return true;
}

void updateFeedback(uint32_t nowMs) {
  if (activeEvent == EventType::NONE) {
    if (
      currentLedDuty != 0 ||
      currentVibrationDuty != 0 ||
      currentPtcDuty != 0
    ) {
      applyOutputs(0, 0, 0);
    }
    return;
  }

  const uint32_t elapsedMs = nowMs - eventStartMs;

  if (elapsedMs >= EVENT_HARD_TIMEOUT_MS) {
    Serial.println("EVENT,SAFETY_TIMEOUT");
    stopFeedback();
    return;
  }

  bool stillActive = false;

  switch (activeEvent) {
    case EventType::SHOT:
      stillActive = updateShotPattern(elapsedMs);
      break;

    case EventType::FOUL:
      stillActive = updateFoulPattern(elapsedMs);
      break;

    case EventType::PENALTY:
      stillActive = updatePenaltyPattern(elapsedMs);
      break;

    case EventType::GOAL:
      stillActive = updateGoalPattern(elapsedMs);
      break;

    default:
      stillActive = false;
      break;
  }

  if (!stillActive) {
    Serial.print("EVENT,FINISHED,");
    Serial.println(eventName(activeEvent));
    stopFeedback();
  }
}

void startEvent(EventType incomingEvent, uint32_t nowMs) {
  if (incomingEvent == EventType::NONE) {
    Serial.println("ERROR,UNKNOWN_EVENT");
    return;
  }

  if (
    activeEvent != EventType::NONE &&
    eventPriority(incomingEvent) <= eventPriority(activeEvent)
  ) {
    Serial.print("EVENT,IGNORED,");
    Serial.print(eventName(incomingEvent));
    Serial.print(",ACTIVE=");
    Serial.println(eventName(activeEvent));
    return;
  }

  activeEvent = incomingEvent;
  eventStartMs = nowMs;

  // Clear the previous pattern before starting the new one.
  applyOutputs(0, 0, 0);

  Serial.print("ACK,EVENT,");
  Serial.println(eventName(activeEvent));
}

// -----------------------------------------------------------------------------
// Serial command parser
// -----------------------------------------------------------------------------

void trimWhitespace(char* text) {
  if (text == nullptr) {
    return;
  }

  size_t length = strlen(text);

  while (
    length > 0 &&
    (
      text[length - 1] == ' ' ||
      text[length - 1] == '\t' ||
      text[length - 1] == '\r' ||
      text[length - 1] == '\n'
    )
  ) {
    text[length - 1] = '\0';
    length--;
  }

  size_t start = 0;

  while (
    text[start] == ' ' ||
    text[start] == '\t'
  ) {
    start++;
  }

  if (start > 0) {
    memmove(text, text + start, strlen(text + start) + 1);
  }
}

void toUpperCase(char* text) {
  if (text == nullptr) {
    return;
  }

  for (size_t index = 0; text[index] != '\0'; ++index) {
    if (text[index] >= 'a' && text[index] <= 'z') {
      text[index] = static_cast<char>(
        text[index] - 'a' + 'A'
      );
    }
  }
}

void processCommand(char* command, uint32_t nowMs) {
  trimWhitespace(command);
  toUpperCase(command);

  if (command[0] == '\0') {
    return;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println("ACK,PONG");
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    stopFeedback();
    Serial.println("ACK,STOP");
    return;
  }

  constexpr char EVENT_PREFIX[] = "EVENT,";

  if (
    strncmp(
      command,
      EVENT_PREFIX,
      strlen(EVENT_PREFIX)
    ) == 0
  ) {
    const char* eventText = command + strlen(EVENT_PREFIX);
    const EventType event = parseEventName(eventText);
    startEvent(event, nowMs);
    return;
  }

  Serial.print("ERROR,UNKNOWN_COMMAND,");
  Serial.println(command);
}

void updateSerial(uint32_t nowMs) {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\n') {
      serialBuffer[serialBufferLength] = '\0';
      processCommand(serialBuffer, nowMs);
      serialBufferLength = 0;
      continue;
    }

    if (incoming == '\r') {
      continue;
    }

    if (serialBufferLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialBufferLength++] = incoming;
    } else {
      serialBufferLength = 0;
      Serial.println("ERROR,COMMAND_TOO_LONG");
    }
  }
}

// -----------------------------------------------------------------------------
// Logging
// -----------------------------------------------------------------------------

void logData(uint32_t nowMs) {
  if (nowMs - lastLogMs < LOG_INTERVAL_MS) {
    return;
  }

  lastLogMs = nowMs;

  Serial.print("DATA,");
  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(currentBpm, 2);
  Serial.print(',');
  Serial.print(irValue);
  Serial.print(',');
  Serial.print(ax, 4);
  Serial.print(',');
  Serial.print(ay, 4);
  Serial.print(',');
  Serial.print(az, 4);
  Serial.print(',');
  Serial.print(gx, 3);
  Serial.print(',');
  Serial.print(gy, 3);
  Serial.print(',');
  Serial.print(gz, 3);
  Serial.print(',');
  Serial.print(eventName(activeEvent));
  Serial.print(',');
  Serial.print(currentLedDuty);
  Serial.print(',');
  Serial.print(currentVibrationDuty);
  Serial.print(',');
  Serial.println(currentPtcDuty);
}

// -----------------------------------------------------------------------------
// Arduino setup and loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(800);

  setupPwm();
  stopFeedback();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(100);

  setupMax30102();
  setupImu();

  Serial.println("READY,PROTOTYPE_A_EVENT_TRIGGERED");
  Serial.println("READY,COMMANDS=EVENT_STOP_PING");
  Serial.println(
    "READY,DATA=esp_ms_bpm_ir_accel_g_gyro_dps_event_led_vib_ptc"
  );
}

void loop() {
  const uint32_t nowMs = millis();

  updateSerial(nowMs);
  updateHeartRate(nowMs);
  updateImu(nowMs);
  updateFeedback(nowMs);
  logData(nowMs);

  delay(1);
}