/*
  Prototype B: Real-Time Crowd-Volume Mapping
  ESP32 + MAX30102 + MPU6050 + vibration motor + LED strip + PTC heater

  Core logic
  ----------
  1. MAX30102 continuously monitors heart rate.
  2. MPU6050 continuously monitors posture and motion.
  3. The computer extracts the crowd-volume envelope from the football video.
  4. The computer sends a normalized value from 0.00 to 1.00 every 20-40 ms.
  5. LED, vibration PWM and PTC electrical power follow the same volume curve.

  Serial command
  --------------
    ATMOS,0.42
    STOP
    PING

  Recommended update rate from the computer:
    25-50 updates per second

  Wiring
  ------
    MAX30102 SDA -> GPIO21
    MAX30102 SCL -> GPIO22
    MPU6050  SDA -> GPIO21
    MPU6050  SCL -> GPIO22

    Vibration MOSFET HIGH/PWM -> GPIO25
    PTC MOSFET HIGH/PWM       -> GPIO26
    LED MOSFET HIGH/PWM       -> GPIO27

  Notes
  -----
  - LED electrical output can follow the volume curve almost immediately.
  - ERM vibration follows closely, but the motor has mechanical inertia.
  - PTC electrical power follows immediately, but skin-perceived temperature
    still changes slowly because the heater has thermal inertia.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "esp_arduino_version.h"

// -----------------------------------------------------------------------------
// Pin configuration
// -----------------------------------------------------------------------------

constexpr uint8_t PIN_VIB = 25;
constexpr uint8_t PIN_PTC = 26;
constexpr uint8_t PIN_LED = 27;
constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

// PWM carrier frequencies.
// These are electrical switching frequencies, not mechanical vibration rates.
constexpr uint32_t VIB_PWM_FREQ = 250;
constexpr uint32_t LED_PWM_FREQ = 1000;
constexpr uint32_t PTC_PWM_FREQ = 100;
constexpr uint8_t PWM_RESOLUTION = 8;

#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr uint8_t VIB_CHANNEL = 0;
constexpr uint8_t LED_CHANNEL = 1;
constexpr uint8_t PTC_CHANNEL = 2;
#endif

// -----------------------------------------------------------------------------
// Timing and mapping parameters
// -----------------------------------------------------------------------------

constexpr unsigned long MPU_SAMPLE_INTERVAL_MS = 20;
constexpr unsigned long SENSOR_LOG_INTERVAL_MS = 100;

// The PC should send a value every 20-40 ms.
// If the stream stops, feedback fades to zero after this timeout.
constexpr unsigned long ATMOS_COMMAND_TIMEOUT_MS = 500;

// Fast attack preserves sudden crowd peaks.
// Slightly slower release avoids noisy flicker.
constexpr float ATTACK_TAU_SECONDS = 0.025f;
constexpr float RELEASE_TAU_SECONDS = 0.080f;

// ERM motors usually need a practical minimum duty to start reliably.
constexpr float VIBRATION_START_THRESHOLD = 0.035f;
constexpr uint8_t VIBRATION_MIN_DUTY = 72;

// PTC safety cap.
// 110/255 is about 43% maximum electrical duty.
// Increase only after temperature testing away from the skin.
constexpr uint8_t PTC_MAX_DUTY = 110;
constexpr float PTC_START_THRESHOLD = 0.08f;

constexpr uint32_t IR_FINGER_THRESHOLD = 50000;

// -----------------------------------------------------------------------------
// Sensor objects and state
// -----------------------------------------------------------------------------

MAX30105 max30102;
Adafruit_MPU6050 mpu;

bool maxReady = false;
bool mpuReady = false;

float bpmAverage = 0.0f;
float bpmBuffer[4] = {0, 0, 0, 0};
uint8_t bpmIndex = 0;
uint8_t bpmCount = 0;
unsigned long lastBeatMs = 0;
uint32_t irValue = 0;

float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

// -----------------------------------------------------------------------------
// Atmosphere and output state
// -----------------------------------------------------------------------------

float atmosphereTarget = 0.0f;
float atmosphereRealtime = 0.0f;

unsigned long lastAtmosCommandMs = 0;
unsigned long lastFeedbackUpdateUs = 0;

uint8_t currentLedDuty = 0;
uint8_t currentVibDuty = 0;
uint8_t currentPtcDuty = 0;

String serialLine;

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------

float clamp01(float value) {
  return constrain(value, 0.0f, 1.0f);
}

uint8_t toDuty(float value) {
  return (uint8_t)constrain((int)roundf(value), 0, 255);
}

float smoothAsymmetric(
    float current,
    float target,
    float dtSeconds
) {
  const float tau =
      target > current
          ? ATTACK_TAU_SECONDS
          : RELEASE_TAU_SECONDS;

  const float alpha =
      1.0f - expf(-dtSeconds / tau);

  return current + (target - current) * alpha;
}

// -----------------------------------------------------------------------------
// PWM output compatibility
// -----------------------------------------------------------------------------

void attachPwmOutputs() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_VIB, VIB_PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_LED, LED_PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_PTC, PTC_PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(VIB_CHANNEL, VIB_PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LED_CHANNEL, LED_PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PTC_CHANNEL, PTC_PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PIN_VIB, VIB_CHANNEL);
  ledcAttachPin(PIN_LED, LED_CHANNEL);
  ledcAttachPin(PIN_PTC, PTC_CHANNEL);
#endif
}

void writeVibration(uint8_t duty) {
  currentVibDuty = duty;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_VIB, duty);
#else
  ledcWrite(VIB_CHANNEL, duty);
#endif
}

void writeLed(uint8_t duty) {
  currentLedDuty = duty;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_LED, duty);
#else
  ledcWrite(LED_CHANNEL, duty);
#endif
}

void writePtc(uint8_t duty) {
  currentPtcDuty = duty;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PTC, duty);
#else
  ledcWrite(PTC_CHANNEL, duty);
#endif
}

void allFeedbackOff() {
  writeVibration(0);
  writeLed(0);
  writePtc(0);
}

// -----------------------------------------------------------------------------
// Real-time crowd-volume mapping
// -----------------------------------------------------------------------------

void updateRealtimeFeedback() {
  const unsigned long nowMs = millis();
  const unsigned long nowUs = micros();

  if (lastFeedbackUpdateUs == 0) {
    lastFeedbackUpdateUs = nowUs;
  }

  float dtSeconds =
      (nowUs - lastFeedbackUpdateUs) / 1000000.0f;

  lastFeedbackUpdateUs = nowUs;

  // Avoid extreme dt values after reset or a long pause.
  dtSeconds = constrain(dtSeconds, 0.001f, 0.100f);

  // Fail-safe: if the PC stream stops, immediately target silence.
  if (nowMs - lastAtmosCommandMs >
      ATMOS_COMMAND_TIMEOUT_MS) {
    atmosphereTarget = 0.0f;
  }

  // Very light smoothing preserves the curve while suppressing serial/audio noise.
  atmosphereRealtime = smoothAsymmetric(
      atmosphereRealtime,
      atmosphereTarget,
      dtSeconds
  );

  const float intensity =
      clamp01(atmosphereRealtime);

  // ---------------------------------------------------------------------------
  // LED
  // Direct linear mapping from crowd volume to brightness.
  // ---------------------------------------------------------------------------

  const uint8_t ledDuty =
      toDuty(255.0f * intensity);

  writeLed(ledDuty);

  // ---------------------------------------------------------------------------
  // Vibration
  // Direct amplitude mapping with a minimum starting duty for the ERM motor.
  // ---------------------------------------------------------------------------

  uint8_t vibrationDuty = 0;

  if (intensity >= VIBRATION_START_THRESHOLD) {
    const float normalized =
        (intensity - VIBRATION_START_THRESHOLD) /
        (1.0f - VIBRATION_START_THRESHOLD);

    vibrationDuty = toDuty(
        VIBRATION_MIN_DUTY +
        normalized * (255.0f - VIBRATION_MIN_DUTY)
    );
  }

  writeVibration(vibrationDuty);

  // ---------------------------------------------------------------------------
  // PTC
  // Electrical power follows the same real-time volume curve.
  // The maximum duty is capped for safer initial testing.
  // ---------------------------------------------------------------------------

  uint8_t ptcDuty = 0;

  if (intensity >= PTC_START_THRESHOLD) {
    const float normalized =
        (intensity - PTC_START_THRESHOLD) /
        (1.0f - PTC_START_THRESHOLD);

    ptcDuty = toDuty(
        normalized * PTC_MAX_DUTY
    );
  }

  writePtc(ptcDuty);
}

// -----------------------------------------------------------------------------
// Serial command parser
// -----------------------------------------------------------------------------

void handleAtmosCommand(const String& line) {
  // Expected:
  // ATMOS,intensity

  const int comma = line.indexOf(',');

  if (comma < 0) {
    Serial.println("ERR,ATMOS_FORMAT");
    return;
  }

  const String intensityText =
      line.substring(comma + 1);

  atmosphereTarget =
      clamp01(intensityText.toFloat());

  lastAtmosCommandMs = millis();

  Serial.printf(
      "ACK,ATMOS,%.3f\n",
      atmosphereTarget
  );
}

void handleSerialCommand(String line) {
  line.trim();
  line.toUpperCase();

  if (line.length() == 0) return;

  if (line == "PING") {
    Serial.println("ACK,PONG");
    return;
  }

  if (line == "STOP" ||
      line == "ALL_OFF") {
    atmosphereTarget = 0.0f;
    atmosphereRealtime = 0.0f;
    allFeedbackOff();

    Serial.println("ACK,STOPPED");
    return;
  }

  if (line.startsWith("ATMOS,")) {
    handleAtmosCommand(line);
    return;
  }

  Serial.printf(
      "ERR,UNKNOWN_COMMAND,%s\n",
      line.c_str()
  );
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c =
        (char)Serial.read();

    if (c == '\n') {
      handleSerialCommand(serialLine);
      serialLine = "";
    } else if (c != '\r') {
      if (serialLine.length() < 80) {
        serialLine += c;
      } else {
        serialLine = "";
        Serial.println("ERR,LINE_TOO_LONG");
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Heart-rate monitoring
// -----------------------------------------------------------------------------

void updateHeartRate() {
  if (!maxReady) return;

  irValue = max30102.getIR();

  if (irValue < IR_FINGER_THRESHOLD) {
    bpmAverage = 0.0f;
    bpmCount = 0;
    return;
  }

  if (checkForBeat(irValue)) {
    const unsigned long now = millis();
    const unsigned long delta =
        now - lastBeatMs;

    lastBeatMs = now;

    if (delta > 0) {
      const float bpm =
          60.0f / (delta / 1000.0f);

      if (bpm >= 35.0f &&
          bpm <= 220.0f) {
        bpmBuffer[bpmIndex] = bpm;
        bpmIndex =
            (bpmIndex + 1) % 4;

        if (bpmCount < 4) {
          bpmCount++;
        }

        float sum = 0.0f;

        for (uint8_t i = 0;
             i < bpmCount;
             i++) {
          sum += bpmBuffer[i];
        }

        bpmAverage =
            sum / bpmCount;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Posture and motion monitoring
// -----------------------------------------------------------------------------

void updateMpu() {
  static unsigned long lastMpuMs = 0;

  const unsigned long now =
      millis();

  if (!mpuReady ||
      now - lastMpuMs <
          MPU_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastMpuMs = now;

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  mpu.getEvent(
      &accelEvent,
      &gyroEvent,
      &tempEvent
  );

  ax = accelEvent.acceleration.x;
  ay = accelEvent.acceleration.y;
  az = accelEvent.acceleration.z;

  gx = gyroEvent.gyro.x;
  gy = gyroEvent.gyro.y;
  gz = gyroEvent.gyro.z;
}

// -----------------------------------------------------------------------------
// Data logging
// -----------------------------------------------------------------------------

void logSensorData() {
  static unsigned long lastLogMs = 0;

  const unsigned long now =
      millis();

  if (now - lastLogMs <
      SENSOR_LOG_INTERVAL_MS) {
    return;
  }

  lastLogMs = now;

  Serial.printf(
      "DATA,%lu,%.2f,%lu,"
      "%.3f,%.3f,%.3f,"
      "%.3f,%.3f,%.3f,"
      "%.3f,%.3f,%u,%u,%u\n",
      now,
      bpmAverage,
      (unsigned long)irValue,
      ax, ay, az,
      gx, gy, gz,
      atmosphereTarget,
      atmosphereRealtime,
      currentLedDuty,
      currentVibDuty,
      currentPtcDuty
  );
}

// -----------------------------------------------------------------------------
// Sensor setup
// -----------------------------------------------------------------------------

void setupSensors() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(200);

  maxReady = max30102.begin(Wire, I2C_SPEED_STANDARD);

  if (maxReady) {
    max30102.setup(60, 4, 2, 100, 411, 4096);
    max30102.setPulseAmplitudeRed(0x0A);
    max30102.setPulseAmplitudeGreen(0);
    Serial.println("READY,MAX30102,OK");
  } else {
    Serial.println("READY,MAX30102,NOT_FOUND");
  }

  delay(100);

  mpuReady = mpu.begin(0x68, &Wire);

  if (mpuReady) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("READY,MPU6050,OK");
  } else {
    Serial.println("READY,MPU6050,NOT_FOUND");
  }
}

// -----------------------------------------------------------------------------
// Arduino lifecycle
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  attachPwmOutputs();
  allFeedbackOff();
  setupSensors();

  lastAtmosCommandMs = millis();

  Serial.println(
      "READY,PROTOTYPE_B_REALTIME"
  );

  Serial.println(
      "READY,COMMANDS=ATMOS_STOP_PING"
  );
}

void loop() {
  pollSerial();
  updateHeartRate();
  updateMpu();
  updateRealtimeFeedback();
  logSensorData();
}
