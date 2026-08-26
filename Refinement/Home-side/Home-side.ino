/*
  Home-side Multimodal Feedback Device

  Hardware
  --------
  ESP32
  MAX30102 heart-rate sensor
  MPU6050-compatible IMU
  2 independent vibration motors through MOSFET drivers
  2 independent PTC heaters through MOSFET drivers
  2 independent 10 kΩ NTC thermistors, B = 3950 K
  1 single-colour LED through a MOSFET driver

  Frozen GPIO mapping
  -------------------
  I2C SDA                  GPIO21
  I2C SCL                  GPIO22
  Vibration motor 1 PWM    GPIO25
  Vibration motor 2 PWM    GPIO32
  PTC heater 1 PWM         GPIO26
  PTC heater 2 PWM         GPIO33
  LED PWM                  GPIO27
  NTC 1 ADC                GPIO34
  NTC 2 ADC                GPIO35

  NTC divider for each channel
  ----------------------------
  3.3 V -> 10 kΩ fixed resistor -> ADC node -> 10 kΩ NTC -> GND
  Add 100 nF from each ADC node to GND.

  BLE responsibility
  ------------------
  The Home App is BLE Central / GATT Client.
  This ESP32 is BLE Peripheral / GATT Server.

  Home App writes compact START_EVENT / UPDATE_EVENT / STOP_EVENT packets.
  ESP32 executes the physical state machine:

      IDLE -> ONSET -> SUSTAIN -> DECAY -> COOLDOWN -> IDLE

  Feedback rules implemented
  --------------------------
  Vibration:
    - CES controls amplitude.
    - Perceived rhythm remains low and stable.
    - Motors alternate with soft, intermittent pulses.
    - Long events never mean continuous full-power vibration.

  LED:
    - Smooth rise during ONSET.
    - Holds the highest brightness reached during SUSTAIN.
    - No flash and no breathing during SUSTAIN.
    - Smooth 7-second fade during DECAY.

  PTC:
    - Enabled only for the highest emotional peaks.
    - Two channels are controlled independently.
    - Each channel uses its own NTC temperature feedback.
    - Overtemperature, invalid NTC, BLE loss, command timeout, or emergency
      stop disables thermal output immediately.

  Home sensors:
    - MAX30102 measures the home viewer's heart-rate response.
    - MPU-compatible IMU measures motion and approximate pitch/roll.
    - These signals are telemetry and do not recreate the stadium CES.

  Required Arduino library
  ------------------------
  SparkFun MAX3010x Sensor Library, providing MAX30105.h and heartRate.h.

  Important
  ---------
  All motors, PTC heaters and the LED must be powered through suitable
  MOSFET driver stages. Do not power them directly from ESP32 GPIO pins.
  Use a common ground. Add flyback diodes across brushed vibration motors.

  Thermal constants below are conservative prototype defaults, not a medical
  certification. Validate temperatures on the assembled product before any
  supervised on-body test.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stddef.h>

#include "MAX30105.h"
#include "heartRate.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------

constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

constexpr uint8_t PIN_VIBRATION_1 = 25;
constexpr uint8_t PIN_VIBRATION_2 = 32;
constexpr uint8_t PIN_PTC_1 = 26;
constexpr uint8_t PIN_PTC_2 = 33;
constexpr uint8_t PIN_LED = 27;

constexpr uint8_t PIN_NTC_1 = 34;
constexpr uint8_t PIN_NTC_2 = 35;

constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr uint8_t DEVICE_ID = 1;

// -----------------------------------------------------------------------------
// BLE configuration
// -----------------------------------------------------------------------------

constexpr char BLE_DEVICE_NAME[] = "HomeFeedback-01";

constexpr char BLE_SERVICE_UUID[] =
  "8e3a0001-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr char BLE_COMMAND_CHARACTERISTIC_UUID[] =
  "8e3a0002-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr char BLE_ACK_CHARACTERISTIC_UUID[] =
  "8e3a0003-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr char BLE_TELEMETRY_CHARACTERISTIC_UUID[] =
  "8e3a0004-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr char BLE_CONTROL_CHARACTERISTIC_UUID[] =
  "8e3a0005-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t FEEDBACK_COMMAND_PACKET_TYPE = 0xC;
constexpr uint8_t HOME_TELEMETRY_PACKET_TYPE = 0xD;
constexpr uint8_t COMMAND_ACK_PACKET_TYPE = 0xB;

constexpr uint8_t makeTypeVersion(
  uint8_t packetType,
  uint8_t version
) {
  return static_cast<uint8_t>(
    ((packetType & 0x0F) << 4) |
    (version & 0x0F)
  );
}

// -----------------------------------------------------------------------------
// Compact BLE protocol
// -----------------------------------------------------------------------------

#pragma pack(push, 1)

/*
  20-byte command written by the Home App.

  Normalized values use 0...255.
  Duration fields use deciseconds, so 675 means 67.5 seconds.
*/
struct FeedbackCommandPacket {
  uint8_t typeVersion;
  uint8_t command;
  uint8_t flags;
  uint8_t stopReason;

  uint32_t eventId;
  uint16_t commandSequence;

  uint8_t ces;
  uint8_t peakCes;
  uint8_t onsetSpike;

  uint16_t maximumDurationDs;
  uint16_t remainingDurationDs;

  uint8_t reserved;
  uint16_t crc16;
};

/*
  12-byte acknowledgement returned after every valid or rejected command.
*/
struct CommandAckPacket {
  uint8_t typeVersion;
  uint8_t command;
  uint8_t status;
  uint8_t feedbackState;

  uint32_t eventId;
  uint16_t commandSequence;
  uint16_t crc16;
};

/*
  20-byte telemetry notification.

  heartResponse and motionScore use 0...255.
  pitchHalfDegrees and rollHalfDegrees encode angle / 2.
  temperatureHalfC encodes degrees Celsius * 2.
*/
struct HomeTelemetryPacket {
  uint8_t typeVersion;
  uint8_t deviceId;
  uint8_t statusFlags;
  uint8_t feedbackState;

  uint16_t sequence;
  uint32_t timestampMs;

  uint8_t bpm;
  uint8_t heartResponse;
  uint8_t motionScore;

  int8_t pitchHalfDegrees;
  int8_t rollHalfDegrees;
  int8_t temperature1HalfC;
  int8_t temperature2HalfC;

  uint8_t faultFlags;
  uint16_t crc16;
};

/*
  6-byte maintenance command written by the Home App.
*/
struct ControlCommandPacket {
  uint8_t command;
  uint8_t version;
  uint16_t requestId;
  uint16_t crc16;
};

#pragma pack(pop)

static_assert(
  sizeof(FeedbackCommandPacket) == 20,
  "FeedbackCommandPacket must remain exactly 20 bytes."
);

static_assert(
  sizeof(CommandAckPacket) == 12,
  "CommandAckPacket must remain exactly 12 bytes."
);

static_assert(
  sizeof(HomeTelemetryPacket) == 20,
  "HomeTelemetryPacket must remain exactly 20 bytes."
);

static_assert(
  sizeof(ControlCommandPacket) == 6,
  "ControlCommandPacket must remain exactly 6 bytes."
);

// -----------------------------------------------------------------------------
// Command and state identifiers
// -----------------------------------------------------------------------------

enum FeedbackCommandKind : uint8_t {
  COMMAND_START_EVENT = 0x01,
  COMMAND_UPDATE_EVENT = 0x02,
  COMMAND_STOP_EVENT = 0x03
};

enum FeedbackCommandFlag : uint8_t {
  COMMAND_FLAG_RESUME = 1u << 0
};

enum FeedbackState : uint8_t {
  STATE_IDLE = 0,
  STATE_ONSET = 1,
  STATE_SUSTAIN = 2,
  STATE_DECAY = 3,
  STATE_COOLDOWN = 4
};

enum AckStatus : uint8_t {
  ACK_OK = 0,
  ACK_DUPLICATE = 1,
  ACK_BAD_LENGTH = 2,
  ACK_BAD_CRC = 3,
  ACK_BAD_TYPE = 4,
  ACK_BAD_VERSION = 5,
  ACK_UNKNOWN_COMMAND = 6,
  ACK_EVENT_MISMATCH = 7,
  ACK_FAULT_LATCHED = 8
};

enum ControlCommandKind : uint8_t {
  CONTROL_RECALIBRATE_HOME_SENSORS = 0x01,
  CONTROL_CLEAR_THERMAL_FAULTS = 0x02,
  CONTROL_EMERGENCY_STOP = 0x03
};

enum HomeStatusFlag : uint8_t {
  STATUS_MAX30102_READY = 1u << 0,
  STATUS_FINGER_PRESENT = 1u << 1,
  STATUS_HEART_BASELINE_READY = 1u << 2,
  STATUS_IMU_READY = 1u << 3,
  STATUS_NTC_1_VALID = 1u << 4,
  STATUS_NTC_2_VALID = 1u << 5,
  STATUS_BLE_CONNECTED = 1u << 6,
  STATUS_FEEDBACK_ACTIVE = 1u << 7
};

enum FaultFlag : uint8_t {
  FAULT_NTC_1_INVALID = 1u << 0,
  FAULT_NTC_2_INVALID = 1u << 1,
  FAULT_PTC_1_OVERTEMP = 1u << 2,
  FAULT_PTC_2_OVERTEMP = 1u << 3,
  FAULT_COMMAND_TIMEOUT = 1u << 4,
  FAULT_BLE_DISCONNECTED = 1u << 5,
  FAULT_EMERGENCY_STOP = 1u << 6,
  FAULT_PROTOCOL = 1u << 7
};

// -----------------------------------------------------------------------------
// Timing configuration
// -----------------------------------------------------------------------------

constexpr uint32_t SENSOR_HEART_INTERVAL_MS = 10;
constexpr uint32_t SENSOR_IMU_INTERVAL_MS = 20;
constexpr uint32_t SENSOR_NTC_INTERVAL_MS = 100;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;
constexpr uint32_t SERIAL_LOG_INTERVAL_MS = 500;
constexpr uint32_t CONTROL_LOOP_INTERVAL_MS = 20;

constexpr uint32_t ONSET_DURATION_MS = 2000;
constexpr uint32_t RESUME_ONSET_DURATION_MS = 700;
constexpr uint32_t DECAY_DURATION_MS = 7000;
constexpr uint32_t LOCAL_COOLDOWN_MS = 2500;

constexpr uint32_t ACTIVE_COMMAND_TIMEOUT_MS = 1800;
constexpr uint32_t MAX_ACCEPTED_EVENT_DURATION_MS = 70000;

// -----------------------------------------------------------------------------
// Output configuration
// -----------------------------------------------------------------------------

constexpr uint8_t PWM_BITS = 8;
constexpr uint16_t MOTOR_PWM_FREQUENCY = 250;
constexpr uint16_t PTC_PWM_FREQUENCY = 100;
constexpr uint16_t LED_PWM_FREQUENCY = 1000;

constexpr uint8_t PWM_CHANNEL_MOTOR_1 = 0;
constexpr uint8_t PWM_CHANNEL_MOTOR_2 = 1;
constexpr uint8_t PWM_CHANNEL_PTC_1 = 2;
constexpr uint8_t PWM_CHANNEL_PTC_2 = 3;
constexpr uint8_t PWM_CHANNEL_LED = 4;

constexpr uint8_t MOTOR_MIN_ACTIVE_DUTY = 85;
constexpr uint8_t MOTOR_MAX_DUTY = 230;

constexpr uint32_t MOTOR_ENVELOPE_PERIOD_MS = 2400;
constexpr uint32_t MOTOR_PULSE_WIDTH_MS = 760;
constexpr uint32_t MOTOR_EDGE_RAMP_MS = 170;
constexpr uint32_t MOTOR_2_PHASE_OFFSET_MS = 1200;

constexpr uint8_t LED_MIN_EVENT_DUTY = 70;
constexpr uint8_t LED_MAX_DUTY = 255;

// -----------------------------------------------------------------------------
// Thermal configuration
// -----------------------------------------------------------------------------

constexpr float NTC_FIXED_RESISTOR_OHMS = 10000.0f;
constexpr float NTC_NOMINAL_RESISTANCE_OHMS = 10000.0f;
constexpr float NTC_NOMINAL_TEMPERATURE_K = 298.15f;
constexpr float NTC_BETA_K = 3950.0f;
constexpr uint16_t ADC_MAX_COUNT = 4095;
constexpr uint8_t NTC_AVERAGE_SAMPLES = 16;

constexpr float NTC_MIN_PLAUSIBLE_C = 0.0f;
constexpr float NTC_MAX_PLAUSIBLE_C = 60.0f;

constexpr float THERMAL_TRIGGER_PEAK_CES = 0.90f;
constexpr float THERMAL_TRIGGER_SPIKE = 0.65f;

constexpr float THERMAL_TARGET_C = 38.5f;
constexpr float THERMAL_RESTART_C = 37.5f;
constexpr float THERMAL_HARD_CUTOFF_C = 42.0f;
constexpr float THERMAL_FAULT_CLEAR_MAX_C = 37.5f;

constexpr uint8_t PTC_MAX_DUTY = 90;
constexpr uint32_t THERMAL_MAX_EVENT_MS = 15000;

// -----------------------------------------------------------------------------
// MAX30102 configuration
// -----------------------------------------------------------------------------

constexpr uint32_t FINGER_IR_THRESHOLD = 50000;
constexpr float BPM_MIN = 35.0f;
constexpr float BPM_MAX = 220.0f;
constexpr uint32_t HEART_CALIBRATION_MAX_MS = 30000;
constexpr uint8_t HEART_BASELINE_MIN_BEATS = 12;
constexpr uint8_t HEART_BASELINE_TARGET_BEATS = 20;

// -----------------------------------------------------------------------------
// MPU-compatible register configuration
// -----------------------------------------------------------------------------

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float ACCEL_SCALE_LSB_PER_G = 4096.0f;
constexpr float GYRO_SCALE_LSB_PER_DPS = 65.5f;
constexpr float RAD_TO_DEG_F = 57.2957795f;

// -----------------------------------------------------------------------------
// Global BLE objects
// -----------------------------------------------------------------------------

BLEServer* bleServer = nullptr;
BLECharacteristic* commandCharacteristic = nullptr;
BLECharacteristic* ackCharacteristic = nullptr;
BLECharacteristic* telemetryCharacteristic = nullptr;
BLECharacteristic* controlCharacteristic = nullptr;

volatile bool bleClientConnected = false;

// -----------------------------------------------------------------------------
// Global packet state and callback handoff
// -----------------------------------------------------------------------------

portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

FeedbackCommandPacket pendingFeedbackPacket = {};
bool pendingFeedbackAvailable = false;
size_t pendingFeedbackLength = 0;

ControlCommandPacket pendingControlPacket = {};
bool pendingControlAvailable = false;
size_t pendingControlLength = 0;

uint16_t lastAcceptedCommandSequence = 0;
bool hasAcceptedCommandSequence = false;

uint16_t telemetrySequence = 0;
HomeTelemetryPacket outgoingTelemetry = {};
CommandAckPacket outgoingAck = {};

// -----------------------------------------------------------------------------
// Global feedback state
// -----------------------------------------------------------------------------

FeedbackState feedbackState = STATE_IDLE;
uint32_t stateStartedMs = 0;
uint32_t activeEventStartedMs = 0;
uint32_t activeEventDeadlineMs = 0;
uint32_t thermalEventStartedMs = 0;
uint32_t lastValidActiveCommandMs = 0;

uint32_t activeEventId = 0;
float currentEventCes = 0.0f;
float currentEventPeakCes = 0.0f;
float currentEventOnsetSpike = 0.0f;
bool activeEventThermalEligible = false;
bool resumedOnset = false;

float vibrationIntensity = 0.0f;
uint8_t ledHeldDuty = 0;
uint8_t currentMotor1Duty = 0;
uint8_t currentMotor2Duty = 0;
uint8_t currentPtc1Duty = 0;
uint8_t currentPtc2Duty = 0;
uint8_t currentLedDuty = 0;

float decayStartVibrationIntensity = 0.0f;
uint8_t decayStartLedDuty = 0;

uint8_t activeFaultFlags = 0;
uint8_t latchedFaultFlags = 0;

// -----------------------------------------------------------------------------
// Global heart-rate state
// -----------------------------------------------------------------------------

MAX30105 heartSensor;
bool max30102Ready = false;
bool fingerPresent = false;
uint32_t irValue = 0;
float currentBpm = 0.0f;
float baselineBpm = 0.0f;
float baselineBpmSum = 0.0f;
uint8_t baselineBeatCount = 0;
uint32_t heartCalibrationStartMs = 0;
uint32_t lastBeatMs = 0;
float heartResponse = 0.0f;

// -----------------------------------------------------------------------------
// Global IMU state
// -----------------------------------------------------------------------------

bool imuReady = false;
uint8_t imuWhoAmI = 0;
float motionScore = 0.0f;
float previousAccelerationMagnitude = 1.0f;
float pitchDegrees = 0.0f;
float rollDegrees = 0.0f;
bool orientationInitialized = false;
uint32_t lastImuSampleMs = 0;

// -----------------------------------------------------------------------------
// Global NTC state
// -----------------------------------------------------------------------------

float temperature1C = NAN;
float temperature2C = NAN;
bool ntc1Valid = false;
bool ntc2Valid = false;

// -----------------------------------------------------------------------------
// Loop timing
// -----------------------------------------------------------------------------

uint32_t lastHeartUpdateMs = 0;
uint32_t lastNtcUpdateMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastSerialLogMs = 0;
uint32_t lastControlLoopMs = 0;

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------

float clamp01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }

  if (value > 1.0f) {
    return 1.0f;
  }

  return value;
}

float smoothAttackRelease(
  float current,
  float target,
  float attack,
  float release
) {
  const float coefficient = target > current ? attack : release;
  return current + coefficient * (target - current);
}

uint8_t quantizeUnitFloat(float value) {
  return static_cast<uint8_t>(
    lroundf(clamp01(value) * 255.0f)
  );
}

float dequantizeUnitFloat(uint8_t value) {
  return static_cast<float>(value) / 255.0f;
}

uint8_t quantizeBpm(float bpm) {
  if (!isfinite(bpm) || bpm <= 0.0f) {
    return 0;
  }

  if (bpm > 255.0f) {
    return 255;
  }

  return static_cast<uint8_t>(lroundf(bpm));
}

int8_t quantizeHalfDegree(float value) {
  if (!isfinite(value)) {
    return INT8_MIN;
  }

  const float encoded = value * 0.5f;

  if (encoded > 127.0f) {
    return 127;
  }

  if (encoded < -127.0f) {
    return -127;
  }

  return static_cast<int8_t>(lroundf(encoded));
}

int8_t quantizeTemperatureHalfC(float temperatureC) {
  if (!isfinite(temperatureC)) {
    return INT8_MIN;
  }

  const float encoded = temperatureC * 2.0f;

  if (encoded > 127.0f) {
    return 127;
  }

  if (encoded < -127.0f) {
    return -127;
  }

  return static_cast<int8_t>(lroundf(encoded));
}

uint32_t durationDsToMs(uint16_t durationDs) {
  const uint32_t durationMs = static_cast<uint32_t>(durationDs) * 100u;
  return min(durationMs, MAX_ACCEPTED_EVENT_DURATION_MS);
}

bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

uint16_t crc16Ccitt(
  const uint8_t* data,
  size_t length
) {
  uint16_t crc = 0xFFFF;

  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

bool verifyPacketCrc(
  const uint8_t* data,
  size_t crcOffset,
  uint16_t receivedCrc
) {
  return crc16Ccitt(data, crcOffset) == receivedCrc;
}

float easeInOut(float progress) {
  progress = clamp01(progress);
  return progress * progress * (3.0f - 2.0f * progress);
}

uint8_t mapScoreToDuty(
  float score,
  float inputLow,
  uint8_t outputLow,
  uint8_t outputHigh
) {
  const float normalized = clamp01(
    (score - inputLow) / (1.0f - inputLow)
  );

  return static_cast<uint8_t>(
    lroundf(
      static_cast<float>(outputLow) +
      normalized * static_cast<float>(outputHigh - outputLow)
    )
  );
}

// -----------------------------------------------------------------------------
// PWM compatibility for ESP32 Arduino core 2.x and 3.x
// -----------------------------------------------------------------------------

void setupPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_VIBRATION_1, MOTOR_PWM_FREQUENCY, PWM_BITS);
  ledcAttach(PIN_VIBRATION_2, MOTOR_PWM_FREQUENCY, PWM_BITS);
  ledcAttach(PIN_PTC_1, PTC_PWM_FREQUENCY, PWM_BITS);
  ledcAttach(PIN_PTC_2, PTC_PWM_FREQUENCY, PWM_BITS);
  ledcAttach(PIN_LED, LED_PWM_FREQUENCY, PWM_BITS);
#else
  ledcSetup(PWM_CHANNEL_MOTOR_1, MOTOR_PWM_FREQUENCY, PWM_BITS);
  ledcSetup(PWM_CHANNEL_MOTOR_2, MOTOR_PWM_FREQUENCY, PWM_BITS);
  ledcSetup(PWM_CHANNEL_PTC_1, PTC_PWM_FREQUENCY, PWM_BITS);
  ledcSetup(PWM_CHANNEL_PTC_2, PTC_PWM_FREQUENCY, PWM_BITS);
  ledcSetup(PWM_CHANNEL_LED, LED_PWM_FREQUENCY, PWM_BITS);

  ledcAttachPin(PIN_VIBRATION_1, PWM_CHANNEL_MOTOR_1);
  ledcAttachPin(PIN_VIBRATION_2, PWM_CHANNEL_MOTOR_2);
  ledcAttachPin(PIN_PTC_1, PWM_CHANNEL_PTC_1);
  ledcAttachPin(PIN_PTC_2, PWM_CHANNEL_PTC_2);
  ledcAttachPin(PIN_LED, PWM_CHANNEL_LED);
#endif
}

void writePwmPin(
  uint8_t pin,
  uint8_t channel,
  uint8_t duty
) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void writeMotor1(uint8_t duty) {
  currentMotor1Duty = duty;
  writePwmPin(PIN_VIBRATION_1, PWM_CHANNEL_MOTOR_1, duty);
}

void writeMotor2(uint8_t duty) {
  currentMotor2Duty = duty;
  writePwmPin(PIN_VIBRATION_2, PWM_CHANNEL_MOTOR_2, duty);
}

void writePtc1(uint8_t duty) {
  duty = min(duty, PTC_MAX_DUTY);
  currentPtc1Duty = duty;
  writePwmPin(PIN_PTC_1, PWM_CHANNEL_PTC_1, duty);
}

void writePtc2(uint8_t duty) {
  duty = min(duty, PTC_MAX_DUTY);
  currentPtc2Duty = duty;
  writePwmPin(PIN_PTC_2, PWM_CHANNEL_PTC_2, duty);
}

void writeLed(uint8_t duty) {
  currentLedDuty = duty;
  writePwmPin(PIN_LED, PWM_CHANNEL_LED, duty);
}

void stopThermalImmediately() {
  writePtc1(0);
  writePtc2(0);
}

void stopAllOutputsImmediately() {
  writeMotor1(0);
  writeMotor2(0);
  stopThermalImmediately();
  writeLed(0);

  vibrationIntensity = 0.0f;
  ledHeldDuty = 0;
}

// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------

bool i2cWriteByte(
  uint8_t address,
  uint8_t reg,
  uint8_t value
) {
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

    destination[index] = static_cast<uint8_t>(Wire.read());
  }

  return true;
}

// -----------------------------------------------------------------------------
// MAX30102 setup and processing
// -----------------------------------------------------------------------------

void setupMax30102() {
  max30102Ready = heartSensor.begin(
    Wire,
    I2C_SPEED_STANDARD
  );

  if (!max30102Ready) {
    Serial.println("READY,MAX30102,NOT_FOUND");
    return;
  }

  heartSensor.setup(
    60,
    4,
    2,
    100,
    411,
    4096
  );

  heartSensor.setPulseAmplitudeRed(0x0A);
  heartSensor.setPulseAmplitudeIR(0x1F);
  heartSensor.setPulseAmplitudeGreen(0x00);

  Serial.println("READY,MAX30102,OK");
}

void resetHeartContactState() {
  fingerPresent = false;
  currentBpm = 0.0f;
  lastBeatMs = 0;

  if (baselineBpm <= 0.0f) {
    baselineBpmSum = 0.0f;
    baselineBeatCount = 0;
    heartCalibrationStartMs = 0;
  }

  heartResponse = smoothAttackRelease(
    heartResponse,
    0.0f,
    0.35f,
    0.12f
  );
}

void registerBaselineBeat(
  float bpm,
  uint32_t nowMs
) {
  if (heartCalibrationStartMs == 0) {
    heartCalibrationStartMs = nowMs;
  }

  baselineBpmSum += bpm;

  if (baselineBeatCount < 255) {
    baselineBeatCount++;
  }

  const bool enoughBeats =
    baselineBeatCount >= HEART_BASELINE_TARGET_BEATS;

  const bool calibrationTimedOut =
    nowMs - heartCalibrationStartMs >= HEART_CALIBRATION_MAX_MS &&
    baselineBeatCount >= HEART_BASELINE_MIN_BEATS;

  if (enoughBeats || calibrationTimedOut) {
    baselineBpm =
      baselineBpmSum / static_cast<float>(baselineBeatCount);

    Serial.printf(
      "READY,HOME_HEART_BASELINE,%.1f\n",
      baselineBpm
    );
  }
}

void updateHeartRateSensor() {
  if (!max30102Ready) {
    return;
  }

  heartSensor.check();

  while (heartSensor.available()) {
    irValue = heartSensor.getFIFOIR();
    fingerPresent = irValue >= FINGER_IR_THRESHOLD;

    if (!fingerPresent) {
      resetHeartContactState();
      heartSensor.nextSample();
      continue;
    }

    if (checkForBeat(irValue)) {
      const uint32_t nowMs = millis();

      if (lastBeatMs != 0) {
        const uint32_t intervalMs = nowMs - lastBeatMs;

        if (intervalMs > 0) {
          const float measuredBpm =
            60000.0f / static_cast<float>(intervalMs);

          if (measuredBpm >= BPM_MIN && measuredBpm <= BPM_MAX) {
            if (currentBpm <= 0.0f) {
              currentBpm = measuredBpm;
            } else {
              currentBpm =
                0.75f * currentBpm +
                0.25f * measuredBpm;
            }

            if (baselineBpm <= 0.0f) {
              registerBaselineBeat(currentBpm, nowMs);
            }
          }
        }
      }

      lastBeatMs = nowMs;
    }

    heartSensor.nextSample();
  }
}

void updateHeartResponse() {
  float target = 0.0f;

  if (
    max30102Ready &&
    fingerPresent &&
    baselineBpm > 0.0f &&
    currentBpm > 0.0f
  ) {
    const float fullScaleIncrease =
      max(20.0f, baselineBpm * 0.35f);

    target = clamp01(
      (currentBpm - baselineBpm) /
      fullScaleIncrease
    );

    if (
      feedbackState == STATE_IDLE &&
      motionScore < 0.15f &&
      currentBpm <= baselineBpm + 3.0f
    ) {
      baselineBpm =
        0.999f * baselineBpm +
        0.001f * currentBpm;
    }
  }

  heartResponse = smoothAttackRelease(
    heartResponse,
    target,
    0.35f,
    0.08f
  );
}

void restartHomeSensorCalibration() {
  baselineBpm = 0.0f;
  baselineBpmSum = 0.0f;
  baselineBeatCount = 0;
  heartCalibrationStartMs = 0;
  currentBpm = 0.0f;
  lastBeatMs = 0;
  heartResponse = 0.0f;

  motionScore = 0.0f;
  previousAccelerationMagnitude = 1.0f;
  pitchDegrees = 0.0f;
  rollDegrees = 0.0f;
  orientationInitialized = false;

  Serial.println("CALIBRATION,HOME_SENSORS,RESTARTED");
}

// -----------------------------------------------------------------------------
// IMU setup and processing
// -----------------------------------------------------------------------------

bool acceptedImuIdentity(uint8_t identity) {
  return (
    identity == 0x68 ||
    identity == 0x70 ||
    identity == 0x71 ||
    identity == 0x73
  );
}

void setupImu() {
  if (!i2cReadBytes(
        MPU_ADDRESS,
        REG_WHO_AM_I,
        &imuWhoAmI,
        1
      )) {
    Serial.println("READY,IMU,NOT_FOUND");
    imuReady = false;
    return;
  }

  if (!acceptedImuIdentity(imuWhoAmI)) {
    Serial.printf(
      "READY,IMU,UNSUPPORTED,WHO_AM_I=0x%02X\n",
      imuWhoAmI
    );
    imuReady = false;
    return;
  }

  bool ok = true;

  ok &= i2cWriteByte(MPU_ADDRESS, REG_PWR_MGMT_1, 0x01);
  delay(50);

  ok &= i2cWriteByte(MPU_ADDRESS, REG_SMPLRT_DIV, 0x04);
  ok &= i2cWriteByte(MPU_ADDRESS, REG_CONFIG, 0x04);
  ok &= i2cWriteByte(MPU_ADDRESS, REG_GYRO_CONFIG, 0x08);
  ok &= i2cWriteByte(MPU_ADDRESS, REG_ACCEL_CONFIG, 0x10);

  imuReady = ok;

  Serial.printf(
    "READY,IMU,%s,WHO_AM_I=0x%02X\n",
    imuReady ? "OK" : "CONFIG_FAILED",
    imuWhoAmI
  );
}

void updateMotionAndPosture(uint32_t nowMs) {
  if (!imuReady) {
    motionScore = smoothAttackRelease(
      motionScore,
      0.0f,
      0.35f,
      0.08f
    );
    return;
  }

  if (nowMs - lastImuSampleMs < SENSOR_IMU_INTERVAL_MS) {
    return;
  }

  const float dt = lastImuSampleMs == 0
    ? static_cast<float>(SENSOR_IMU_INTERVAL_MS) / 1000.0f
    : static_cast<float>(nowMs - lastImuSampleMs) / 1000.0f;

  lastImuSampleMs = nowMs;

  uint8_t raw[14];

  if (!i2cReadBytes(
        MPU_ADDRESS,
        REG_ACCEL_XOUT_H,
        raw,
        sizeof(raw)
      )) {
    motionScore = smoothAttackRelease(
      motionScore,
      0.0f,
      0.35f,
      0.08f
    );
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

  const float ax =
    static_cast<float>(rawAx) / ACCEL_SCALE_LSB_PER_G;

  const float ay =
    static_cast<float>(rawAy) / ACCEL_SCALE_LSB_PER_G;

  const float az =
    static_cast<float>(rawAz) / ACCEL_SCALE_LSB_PER_G;

  const float gx =
    static_cast<float>(rawGx) / GYRO_SCALE_LSB_PER_DPS;

  const float gy =
    static_cast<float>(rawGy) / GYRO_SCALE_LSB_PER_DPS;

  const float gz =
    static_cast<float>(rawGz) / GYRO_SCALE_LSB_PER_DPS;

  const float accelerationMagnitude =
    sqrtf(ax * ax + ay * ay + az * az);

  const float linearAcceleration =
    fabsf(accelerationMagnitude - 1.0f);

  const float jerk = fabsf(
    accelerationMagnitude -
    previousAccelerationMagnitude
  );

  previousAccelerationMagnitude = accelerationMagnitude;

  const float gyroMagnitude =
    sqrtf(gx * gx + gy * gy + gz * gz);

  const float accelerationComponent =
    clamp01(linearAcceleration / 1.20f);

  const float jerkComponent =
    clamp01(jerk / 0.80f);

  const float gyroComponent =
    clamp01(gyroMagnitude / 250.0f);

  const float motionTarget =
    0.45f * accelerationComponent +
    0.25f * jerkComponent +
    0.30f * gyroComponent;

  motionScore = smoothAttackRelease(
    motionScore,
    clamp01(motionTarget),
    0.40f,
    0.08f
  );

  const float accelPitch = atan2f(
    -ax,
    sqrtf(ay * ay + az * az)
  ) * RAD_TO_DEG_F;

  const float accelRoll = atan2f(ay, az) * RAD_TO_DEG_F;

  if (!orientationInitialized) {
    pitchDegrees = accelPitch;
    rollDegrees = accelRoll;
    orientationInitialized = true;
  } else {
    const float boundedDt = min(max(dt, 0.001f), 0.100f);

    pitchDegrees =
      0.98f * (pitchDegrees + gy * boundedDt) +
      0.02f * accelPitch;

    rollDegrees =
      0.98f * (rollDegrees + gx * boundedDt) +
      0.02f * accelRoll;
  }
}

// -----------------------------------------------------------------------------
// NTC temperature processing
// -----------------------------------------------------------------------------

float readNtcTemperatureC(
  uint8_t pin,
  bool& valid
) {
  uint32_t sum = 0;

  for (uint8_t index = 0; index < NTC_AVERAGE_SAMPLES; ++index) {
    sum += analogRead(pin);
    delayMicroseconds(120);
  }

  const float raw =
    static_cast<float>(sum) /
    static_cast<float>(NTC_AVERAGE_SAMPLES);

  if (raw < 40.0f || raw > static_cast<float>(ADC_MAX_COUNT - 40)) {
    valid = false;
    return NAN;
  }

  const float resistance =
    NTC_FIXED_RESISTOR_OHMS *
    raw /
    (static_cast<float>(ADC_MAX_COUNT) - raw);

  if (!isfinite(resistance) || resistance <= 0.0f) {
    valid = false;
    return NAN;
  }

  const float inverseKelvin =
    1.0f / NTC_NOMINAL_TEMPERATURE_K +
    logf(resistance / NTC_NOMINAL_RESISTANCE_OHMS) /
    NTC_BETA_K;

  const float temperatureC =
    1.0f / inverseKelvin - 273.15f;

  valid =
    isfinite(temperatureC) &&
    temperatureC >= NTC_MIN_PLAUSIBLE_C &&
    temperatureC <= NTC_MAX_PLAUSIBLE_C;

  return valid ? temperatureC : NAN;
}

void updateNtcTemperatures(uint32_t nowMs) {
  if (nowMs - lastNtcUpdateMs < SENSOR_NTC_INTERVAL_MS) {
    return;
  }

  lastNtcUpdateMs = nowMs;

  bool newNtc1Valid = false;
  bool newNtc2Valid = false;

  const float newTemperature1 =
    readNtcTemperatureC(PIN_NTC_1, newNtc1Valid);

  const float newTemperature2 =
    readNtcTemperatureC(PIN_NTC_2, newNtc2Valid);

  if (newNtc1Valid) {
    temperature1C = isfinite(temperature1C)
      ? 0.75f * temperature1C + 0.25f * newTemperature1
      : newTemperature1;
  } else {
    temperature1C = NAN;
  }

  if (newNtc2Valid) {
    temperature2C = isfinite(temperature2C)
      ? 0.75f * temperature2C + 0.25f * newTemperature2
      : newTemperature2;
  } else {
    temperature2C = NAN;
  }

  ntc1Valid = newNtc1Valid;
  ntc2Valid = newNtc2Valid;

  activeFaultFlags &= static_cast<uint8_t>(
    ~(FAULT_NTC_1_INVALID | FAULT_NTC_2_INVALID)
  );

  if (!ntc1Valid) {
    activeFaultFlags |= FAULT_NTC_1_INVALID;
    latchedFaultFlags |= FAULT_NTC_1_INVALID;
    writePtc1(0);
  }

  if (!ntc2Valid) {
    activeFaultFlags |= FAULT_NTC_2_INVALID;
    latchedFaultFlags |= FAULT_NTC_2_INVALID;
    writePtc2(0);
  }

  if (ntc1Valid && temperature1C >= THERMAL_HARD_CUTOFF_C) {
    activeFaultFlags |= FAULT_PTC_1_OVERTEMP;
    latchedFaultFlags |= FAULT_PTC_1_OVERTEMP;
    writePtc1(0);
  }

  if (ntc2Valid && temperature2C >= THERMAL_HARD_CUTOFF_C) {
    activeFaultFlags |= FAULT_PTC_2_OVERTEMP;
    latchedFaultFlags |= FAULT_PTC_2_OVERTEMP;
    writePtc2(0);
  }
}

bool thermalFaultForChannel1() {
  return latchedFaultFlags & (
    FAULT_NTC_1_INVALID |
    FAULT_PTC_1_OVERTEMP |
    FAULT_EMERGENCY_STOP
  );
}

bool thermalFaultForChannel2() {
  return latchedFaultFlags & (
    FAULT_NTC_2_INVALID |
    FAULT_PTC_2_OVERTEMP |
    FAULT_EMERGENCY_STOP
  );
}

uint8_t thermalDutyForTemperature(
  float temperatureC,
  bool valid,
  bool channelFaulted
) {
  if (!valid || channelFaulted || !isfinite(temperatureC)) {
    return 0;
  }

  if (temperatureC >= THERMAL_TARGET_C) {
    return 0;
  }

  if (temperatureC <= THERMAL_RESTART_C) {
    return PTC_MAX_DUTY;
  }

  const float normalized = clamp01(
    (THERMAL_TARGET_C - temperatureC) /
    (THERMAL_TARGET_C - THERMAL_RESTART_C)
  );

  return static_cast<uint8_t>(
    lroundf(normalized * static_cast<float>(PTC_MAX_DUTY))
  );
}

void clearThermalFaultsIfSafe() {
  const bool channel1Safe =
    ntc1Valid &&
    isfinite(temperature1C) &&
    temperature1C <= THERMAL_FAULT_CLEAR_MAX_C;

  const bool channel2Safe =
    ntc2Valid &&
    isfinite(temperature2C) &&
    temperature2C <= THERMAL_FAULT_CLEAR_MAX_C;

  if (feedbackState != STATE_IDLE || !channel1Safe || !channel2Safe) {
    Serial.println("FAULT,CLEAR_REJECTED");
    return;
  }

  latchedFaultFlags &= static_cast<uint8_t>(
    ~(FAULT_NTC_1_INVALID |
      FAULT_NTC_2_INVALID |
      FAULT_PTC_1_OVERTEMP |
      FAULT_PTC_2_OVERTEMP |
      FAULT_EMERGENCY_STOP)
  );

  activeFaultFlags &= static_cast<uint8_t>(
    ~(FAULT_NTC_1_INVALID |
      FAULT_NTC_2_INVALID |
      FAULT_PTC_1_OVERTEMP |
      FAULT_PTC_2_OVERTEMP |
      FAULT_EMERGENCY_STOP)
  );

  Serial.println("FAULT,THERMAL,CLEARED");
}

// -----------------------------------------------------------------------------
// Feedback output mapping
// -----------------------------------------------------------------------------

float motorPulseEnvelope(
  uint32_t nowMs,
  uint32_t phaseOffsetMs
) {
  const uint32_t phase =
    (nowMs + phaseOffsetMs) % MOTOR_ENVELOPE_PERIOD_MS;

  if (phase >= MOTOR_PULSE_WIDTH_MS) {
    return 0.0f;
  }

  if (phase < MOTOR_EDGE_RAMP_MS) {
    return easeInOut(
      static_cast<float>(phase) /
      static_cast<float>(MOTOR_EDGE_RAMP_MS)
    );
  }

  const uint32_t rampDownStart =
    MOTOR_PULSE_WIDTH_MS - MOTOR_EDGE_RAMP_MS;

  if (phase > rampDownStart) {
    return easeInOut(
      static_cast<float>(MOTOR_PULSE_WIDTH_MS - phase) /
      static_cast<float>(MOTOR_EDGE_RAMP_MS)
    );
  }

  return 1.0f;
}

float currentStateOutputScale(uint32_t nowMs) {
  switch (feedbackState) {
    case STATE_ONSET: {
      const uint32_t onsetDuration = resumedOnset
        ? RESUME_ONSET_DURATION_MS
        : ONSET_DURATION_MS;

      return easeInOut(
        static_cast<float>(nowMs - stateStartedMs) /
        static_cast<float>(onsetDuration)
      );
    }

    case STATE_SUSTAIN:
      return 1.0f;

    case STATE_DECAY:
      return 1.0f - easeInOut(
        static_cast<float>(nowMs - stateStartedMs) /
        static_cast<float>(DECAY_DURATION_MS)
      );

    default:
      return 0.0f;
  }
}

void updateVibrationOutputs(uint32_t nowMs) {
  if (
    feedbackState == STATE_IDLE ||
    feedbackState == STATE_COOLDOWN
  ) {
    vibrationIntensity = smoothAttackRelease(
      vibrationIntensity,
      0.0f,
      0.25f,
      0.12f
    );

    writeMotor1(0);
    writeMotor2(0);
    return;
  }

  const float targetScore = feedbackState == STATE_DECAY
    ? decayStartVibrationIntensity
    : currentEventCes;

  if (feedbackState != STATE_DECAY) {
    vibrationIntensity = smoothAttackRelease(
      vibrationIntensity,
      targetScore,
      0.20f,
      0.025f
    );
  }

  const float outputScale = currentStateOutputScale(nowMs);

  const uint8_t baseDuty = mapScoreToDuty(
    vibrationIntensity,
    0.50f,
    MOTOR_MIN_ACTIVE_DUTY,
    MOTOR_MAX_DUTY
  );

  const float envelope1 = motorPulseEnvelope(nowMs, 0);
  const float envelope2 = motorPulseEnvelope(
    nowMs,
    MOTOR_2_PHASE_OFFSET_MS
  );

  const uint8_t duty1 = static_cast<uint8_t>(
    lroundf(
      static_cast<float>(baseDuty) *
      envelope1 *
      outputScale
    )
  );

  const uint8_t duty2 = static_cast<uint8_t>(
    lroundf(
      static_cast<float>(baseDuty) *
      envelope2 *
      outputScale
    )
  );

  writeMotor1(duty1);
  writeMotor2(duty2);
}

void updateLedOutput(uint32_t nowMs) {
  if (
    feedbackState == STATE_IDLE ||
    feedbackState == STATE_COOLDOWN
  ) {
    writeLed(0);
    return;
  }

  if (feedbackState == STATE_DECAY) {
    const float scale = currentStateOutputScale(nowMs);

    writeLed(
      static_cast<uint8_t>(
        lroundf(static_cast<float>(decayStartLedDuty) * scale)
      )
    );

    return;
  }

  const uint8_t targetDuty = mapScoreToDuty(
    currentEventPeakCes,
    0.50f,
    LED_MIN_EVENT_DUTY,
    LED_MAX_DUTY
  );

  if (targetDuty > ledHeldDuty) {
    const uint8_t step = feedbackState == STATE_ONSET ? 4 : 2;
    ledHeldDuty = static_cast<uint8_t>(
      min(
        static_cast<uint16_t>(targetDuty),
        static_cast<uint16_t>(
          static_cast<uint16_t>(ledHeldDuty) +
          static_cast<uint16_t>(step)
        )
      )
    );
  }

  const float scale = feedbackState == STATE_ONSET
    ? currentStateOutputScale(nowMs)
    : 1.0f;

  writeLed(
    static_cast<uint8_t>(
      lroundf(static_cast<float>(ledHeldDuty) * scale)
    )
  );
}

void updateThermalOutputs(uint32_t nowMs) {
  const bool stateAllowsThermal =
    feedbackState == STATE_ONSET ||
    feedbackState == STATE_SUSTAIN;

  if (
    !stateAllowsThermal ||
    !activeEventThermalEligible ||
    !bleClientConnected
  ) {
    stopThermalImmediately();
    return;
  }

  if (
    thermalEventStartedMs == 0 ||
    nowMs - thermalEventStartedMs >= THERMAL_MAX_EVENT_MS
  ) {
    stopThermalImmediately();
    return;
  }

  if (
    activeFaultFlags & (
      FAULT_COMMAND_TIMEOUT |
      FAULT_BLE_DISCONNECTED
    )
  ) {
    stopThermalImmediately();
    return;
  }

  const uint8_t duty1 = thermalDutyForTemperature(
    temperature1C,
    ntc1Valid,
    thermalFaultForChannel1()
  );

  const uint8_t duty2 = thermalDutyForTemperature(
    temperature2C,
    ntc2Valid,
    thermalFaultForChannel2()
  );

  writePtc1(duty1);
  writePtc2(duty2);
}

// -----------------------------------------------------------------------------
// Feedback state machine
// -----------------------------------------------------------------------------

void enterIdle(uint32_t nowMs) {
  feedbackState = STATE_IDLE;
  stateStartedMs = nowMs;

  activeEventId = 0;
  activeEventStartedMs = 0;
  activeEventDeadlineMs = 0;
  thermalEventStartedMs = 0;
  lastValidActiveCommandMs = 0;

  currentEventCes = 0.0f;
  currentEventPeakCes = 0.0f;
  currentEventOnsetSpike = 0.0f;
  activeEventThermalEligible = false;
  resumedOnset = false;

  stopAllOutputsImmediately();
}

void beginDecay(uint32_t nowMs) {
  if (
    feedbackState == STATE_IDLE ||
    feedbackState == STATE_DECAY ||
    feedbackState == STATE_COOLDOWN
  ) {
    return;
  }

  stopThermalImmediately();

  decayStartVibrationIntensity = vibrationIntensity;
  decayStartLedDuty = currentLedDuty;

  feedbackState = STATE_DECAY;
  stateStartedMs = nowMs;
}

void beginEvent(
  const FeedbackCommandPacket& packet,
  uint32_t nowMs
) {
  const uint32_t remainingMs =
    durationDsToMs(packet.remainingDurationDs);

  if (remainingMs == 0) {
    beginDecay(nowMs);
    return;
  }

  activeEventId = packet.eventId;
  currentEventCes = dequantizeUnitFloat(packet.ces);
  currentEventPeakCes = dequantizeUnitFloat(packet.peakCes);
  currentEventOnsetSpike = dequantizeUnitFloat(packet.onsetSpike);

  activeEventStartedMs = nowMs;
  activeEventDeadlineMs = nowMs + remainingMs;
  lastValidActiveCommandMs = nowMs;

  activeEventThermalEligible =
    currentEventPeakCes >= THERMAL_TRIGGER_PEAK_CES &&
    currentEventOnsetSpike >= THERMAL_TRIGGER_SPIKE;

  thermalEventStartedMs = activeEventThermalEligible
    ? nowMs
    : 0;

  resumedOnset = packet.flags & COMMAND_FLAG_RESUME;

  vibrationIntensity = currentEventCes;
  ledHeldDuty = 0;

  feedbackState = STATE_ONSET;
  stateStartedMs = nowMs;

  activeFaultFlags &= static_cast<uint8_t>(
    ~(FAULT_COMMAND_TIMEOUT | FAULT_BLE_DISCONNECTED)
  );
}

bool updateEvent(
  const FeedbackCommandPacket& packet,
  uint32_t nowMs
) {
  if (
    feedbackState != STATE_ONSET &&
    feedbackState != STATE_SUSTAIN
  ) {
    return false;
  }

  if (packet.eventId != activeEventId) {
    return false;
  }

  currentEventCes = dequantizeUnitFloat(packet.ces);
  currentEventPeakCes = max(
    currentEventPeakCes,
    dequantizeUnitFloat(packet.peakCes)
  );

  currentEventOnsetSpike = max(
    currentEventOnsetSpike,
    dequantizeUnitFloat(packet.onsetSpike)
  );

  const uint32_t remainingMs =
    durationDsToMs(packet.remainingDurationDs);

  activeEventDeadlineMs = nowMs + remainingMs;
  lastValidActiveCommandMs = nowMs;

  const bool newlyThermalEligible =
    currentEventPeakCes >= THERMAL_TRIGGER_PEAK_CES &&
    currentEventOnsetSpike >= THERMAL_TRIGGER_SPIKE;

  if (newlyThermalEligible && !activeEventThermalEligible) {
    activeEventThermalEligible = true;
    thermalEventStartedMs = nowMs;
  }

  return true;
}

void updateFeedbackStateMachine(uint32_t nowMs) {
  if (
    feedbackState == STATE_ONSET ||
    feedbackState == STATE_SUSTAIN
  ) {
    if (
      lastValidActiveCommandMs != 0 &&
      nowMs - lastValidActiveCommandMs > ACTIVE_COMMAND_TIMEOUT_MS
    ) {
      activeFaultFlags |= FAULT_COMMAND_TIMEOUT;
      stopThermalImmediately();
      beginDecay(nowMs);
    }

    if (
      activeEventDeadlineMs != 0 &&
      timeReached(nowMs, activeEventDeadlineMs)
    ) {
      beginDecay(nowMs);
    }
  }

  switch (feedbackState) {
    case STATE_ONSET: {
      const uint32_t onsetDuration = resumedOnset
        ? RESUME_ONSET_DURATION_MS
        : ONSET_DURATION_MS;

      if (nowMs - stateStartedMs >= onsetDuration) {
        feedbackState = STATE_SUSTAIN;
        stateStartedMs = nowMs;
        resumedOnset = false;
      }
      break;
    }

    case STATE_DECAY:
      if (nowMs - stateStartedMs >= DECAY_DURATION_MS) {
        stopAllOutputsImmediately();
        feedbackState = STATE_COOLDOWN;
        stateStartedMs = nowMs;
      }
      break;

    case STATE_COOLDOWN:
      if (nowMs - stateStartedMs >= LOCAL_COOLDOWN_MS) {
        enterIdle(nowMs);
      }
      break;

    default:
      break;
  }

  updateVibrationOutputs(nowMs);
  updateLedOutput(nowMs);
  updateThermalOutputs(nowMs);
}

// -----------------------------------------------------------------------------
// BLE packet validation and acknowledgement
// -----------------------------------------------------------------------------

bool isValidFeedbackCommandKind(uint8_t command) {
  return (
    command == COMMAND_START_EVENT ||
    command == COMMAND_UPDATE_EVENT ||
    command == COMMAND_STOP_EVENT
  );
}

void publishAck(
  const FeedbackCommandPacket& packet,
  AckStatus status
) {
  if (ackCharacteristic == nullptr) {
    return;
  }

  outgoingAck.typeVersion = makeTypeVersion(
    COMMAND_ACK_PACKET_TYPE,
    PROTOCOL_VERSION
  );

  outgoingAck.command = packet.command;
  outgoingAck.status = status;
  outgoingAck.feedbackState = feedbackState;
  outgoingAck.eventId = packet.eventId;
  outgoingAck.commandSequence = packet.commandSequence;
  outgoingAck.crc16 = 0;

  outgoingAck.crc16 = crc16Ccitt(
    reinterpret_cast<const uint8_t*>(&outgoingAck),
    offsetof(CommandAckPacket, crc16)
  );

  ackCharacteristic->setValue(
    reinterpret_cast<uint8_t*>(&outgoingAck),
    sizeof(outgoingAck)
  );

  if (bleClientConnected) {
    ackCharacteristic->notify();
  }
}

AckStatus validateFeedbackPacket(
  const FeedbackCommandPacket& packet,
  size_t length
) {
  if (length != sizeof(FeedbackCommandPacket)) {
    return ACK_BAD_LENGTH;
  }

  const uint8_t packetType = packet.typeVersion >> 4;
  const uint8_t packetVersion = packet.typeVersion & 0x0F;

  if (packetType != FEEDBACK_COMMAND_PACKET_TYPE) {
    return ACK_BAD_TYPE;
  }

  if (packetVersion != PROTOCOL_VERSION) {
    return ACK_BAD_VERSION;
  }

  if (!verifyPacketCrc(
        reinterpret_cast<const uint8_t*>(&packet),
        offsetof(FeedbackCommandPacket, crc16),
        packet.crc16
      )) {
    return ACK_BAD_CRC;
  }

  if (!isValidFeedbackCommandKind(packet.command)) {
    return ACK_UNKNOWN_COMMAND;
  }

  return ACK_OK;
}

void processFeedbackCommand(
  const FeedbackCommandPacket& packet,
  size_t length,
  uint32_t nowMs
) {
  AckStatus status = validateFeedbackPacket(packet, length);

  if (status != ACK_OK) {
    activeFaultFlags |= FAULT_PROTOCOL;
    publishAck(packet, status);
    return;
  }

  activeFaultFlags &= static_cast<uint8_t>(~FAULT_PROTOCOL);

  if (
    hasAcceptedCommandSequence &&
    packet.commandSequence == lastAcceptedCommandSequence
  ) {
    publishAck(packet, ACK_DUPLICATE);
    return;
  }

  lastAcceptedCommandSequence = packet.commandSequence;
  hasAcceptedCommandSequence = true;

  switch (packet.command) {
    case COMMAND_START_EVENT:
      if (latchedFaultFlags & FAULT_EMERGENCY_STOP) {
        status = ACK_FAULT_LATCHED;
      } else {
        if (
          feedbackState == STATE_ONSET ||
          feedbackState == STATE_SUSTAIN
        ) {
          stopThermalImmediately();
        }

        beginEvent(packet, nowMs);
      }
      break;

    case COMMAND_UPDATE_EVENT:
      if (!updateEvent(packet, nowMs)) {
        status = ACK_EVENT_MISMATCH;
      }
      break;

    case COMMAND_STOP_EVENT:
      if (
        activeEventId != 0 &&
        packet.eventId != activeEventId
      ) {
        status = ACK_EVENT_MISMATCH;
      } else {
        lastValidActiveCommandMs = nowMs;
        beginDecay(nowMs);
      }
      break;

    default:
      status = ACK_UNKNOWN_COMMAND;
      break;
  }

  publishAck(packet, status);
}

void emergencyStop(uint32_t nowMs) {
  latchedFaultFlags |= FAULT_EMERGENCY_STOP;
  activeFaultFlags |= FAULT_EMERGENCY_STOP;

  stopAllOutputsImmediately();
  feedbackState = STATE_IDLE;
  stateStartedMs = nowMs;

  activeEventId = 0;
  activeEventStartedMs = 0;
  activeEventDeadlineMs = 0;
  thermalEventStartedMs = 0;

  Serial.println("FAULT,EMERGENCY_STOP");
}

void processControlCommand(
  const ControlCommandPacket& packet,
  size_t length,
  uint32_t nowMs
) {
  if (length != sizeof(ControlCommandPacket)) {
    activeFaultFlags |= FAULT_PROTOCOL;
    return;
  }

  if (packet.version != PROTOCOL_VERSION) {
    activeFaultFlags |= FAULT_PROTOCOL;
    return;
  }

  if (!verifyPacketCrc(
        reinterpret_cast<const uint8_t*>(&packet),
        offsetof(ControlCommandPacket, crc16),
        packet.crc16
      )) {
    activeFaultFlags |= FAULT_PROTOCOL;
    return;
  }

  activeFaultFlags &= static_cast<uint8_t>(~FAULT_PROTOCOL);

  switch (packet.command) {
    case CONTROL_RECALIBRATE_HOME_SENSORS:
      restartHomeSensorCalibration();
      break;

    case CONTROL_CLEAR_THERMAL_FAULTS:
      clearThermalFaultsIfSafe();
      break;

    case CONTROL_EMERGENCY_STOP:
      emergencyStop(nowMs);
      break;

    default:
      activeFaultFlags |= FAULT_PROTOCOL;
      break;
  }
}

void processPendingBleWrites(uint32_t nowMs) {
  FeedbackCommandPacket feedbackCopy = {};
  bool feedbackReady = false;
  size_t feedbackLength = 0;

  ControlCommandPacket controlCopy = {};
  bool controlReady = false;
  size_t controlLength = 0;

  portENTER_CRITICAL(&commandMux);

  if (pendingFeedbackAvailable) {
    feedbackCopy = pendingFeedbackPacket;
    feedbackLength = pendingFeedbackLength;
    pendingFeedbackAvailable = false;
    feedbackReady = true;
  }

  if (pendingControlAvailable) {
    controlCopy = pendingControlPacket;
    controlLength = pendingControlLength;
    pendingControlAvailable = false;
    controlReady = true;
  }

  portEXIT_CRITICAL(&commandMux);

  if (feedbackReady) {
    processFeedbackCommand(
      feedbackCopy,
      feedbackLength,
      nowMs
    );
  }

  if (controlReady) {
    processControlCommand(
      controlCopy,
      controlLength,
      nowMs
    );
  }
}

// -----------------------------------------------------------------------------
// Telemetry construction
// -----------------------------------------------------------------------------

uint8_t buildStatusFlags() {
  uint8_t flags = 0;

  if (max30102Ready) {
    flags |= STATUS_MAX30102_READY;
  }

  if (fingerPresent) {
    flags |= STATUS_FINGER_PRESENT;
  }

  if (baselineBpm > 0.0f) {
    flags |= STATUS_HEART_BASELINE_READY;
  }

  if (imuReady) {
    flags |= STATUS_IMU_READY;
  }

  if (ntc1Valid) {
    flags |= STATUS_NTC_1_VALID;
  }

  if (ntc2Valid) {
    flags |= STATUS_NTC_2_VALID;
  }

  if (bleClientConnected) {
    flags |= STATUS_BLE_CONNECTED;
  }

  if (
    feedbackState == STATE_ONSET ||
    feedbackState == STATE_SUSTAIN ||
    feedbackState == STATE_DECAY
  ) {
    flags |= STATUS_FEEDBACK_ACTIVE;
  }

  return flags;
}

void buildTelemetryPacket(uint32_t nowMs) {
  outgoingTelemetry.typeVersion = makeTypeVersion(
    HOME_TELEMETRY_PACKET_TYPE,
    PROTOCOL_VERSION
  );

  outgoingTelemetry.deviceId = DEVICE_ID;
  outgoingTelemetry.statusFlags = buildStatusFlags();
  outgoingTelemetry.feedbackState = feedbackState;
  outgoingTelemetry.sequence = telemetrySequence++;
  outgoingTelemetry.timestampMs = nowMs;

  outgoingTelemetry.bpm = quantizeBpm(currentBpm);
  outgoingTelemetry.heartResponse = quantizeUnitFloat(heartResponse);
  outgoingTelemetry.motionScore = quantizeUnitFloat(motionScore);

  outgoingTelemetry.pitchHalfDegrees =
    quantizeHalfDegree(pitchDegrees);

  outgoingTelemetry.rollHalfDegrees =
    quantizeHalfDegree(rollDegrees);

  outgoingTelemetry.temperature1HalfC =
    quantizeTemperatureHalfC(temperature1C);

  outgoingTelemetry.temperature2HalfC =
    quantizeTemperatureHalfC(temperature2C);

  outgoingTelemetry.faultFlags =
    activeFaultFlags | latchedFaultFlags;

  outgoingTelemetry.crc16 = 0;

  outgoingTelemetry.crc16 = crc16Ccitt(
    reinterpret_cast<const uint8_t*>(&outgoingTelemetry),
    offsetof(HomeTelemetryPacket, crc16)
  );
}

void publishTelemetry(uint32_t nowMs) {
  if (
    telemetryCharacteristic == nullptr ||
    nowMs - lastTelemetryMs < TELEMETRY_INTERVAL_MS
  ) {
    return;
  }

  lastTelemetryMs = nowMs;
  buildTelemetryPacket(nowMs);

  telemetryCharacteristic->setValue(
    reinterpret_cast<uint8_t*>(&outgoingTelemetry),
    sizeof(outgoingTelemetry)
  );

  if (bleClientConnected) {
    telemetryCharacteristic->notify();
  }
}

// -----------------------------------------------------------------------------
// BLE callbacks and setup
// -----------------------------------------------------------------------------

class HomeServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    (void)server;

    bleClientConnected = true;
    activeFaultFlags &= static_cast<uint8_t>(~FAULT_BLE_DISCONNECTED);

    Serial.println("READY,BLE,CLIENT_CONNECTED");
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;

    bleClientConnected = false;
    activeFaultFlags |= FAULT_BLE_DISCONNECTED;

    stopThermalImmediately();

    if (
      feedbackState == STATE_ONSET ||
      feedbackState == STATE_SUSTAIN
    ) {
      beginDecay(millis());
    }

    BLEDevice::startAdvertising();
    Serial.println("READY,BLE,CLIENT_DISCONNECTED");
  }
};

class FeedbackCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const String value = characteristic->getValue();

    portENTER_CRITICAL(&commandMux);

    pendingFeedbackLength = value.length();
    memset(&pendingFeedbackPacket, 0, sizeof(pendingFeedbackPacket));

    const size_t copyLength = min(
      static_cast<size_t>(value.length()),
      sizeof(pendingFeedbackPacket)
    );

    memcpy(
      &pendingFeedbackPacket,
      value.c_str(),
      copyLength
    );

    pendingFeedbackAvailable = true;

    portEXIT_CRITICAL(&commandMux);
  }
};

class ControlCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const String value = characteristic->getValue();

    portENTER_CRITICAL(&commandMux);

    pendingControlLength = value.length();
    memset(&pendingControlPacket, 0, sizeof(pendingControlPacket));

    const size_t copyLength = min(
      static_cast<size_t>(value.length()),
      sizeof(pendingControlPacket)
    );

    memcpy(
      &pendingControlPacket,
      value.c_str(),
      copyLength
    );

    pendingControlAvailable = true;

    portEXIT_CRITICAL(&commandMux);
  }
};

void setupBle() {
  BLEDevice::init(BLE_DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new HomeServerCallbacks());

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);

  commandCharacteristic = service->createCharacteristic(
    BLE_COMMAND_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  ackCharacteristic = service->createCharacteristic(
    BLE_ACK_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  telemetryCharacteristic = service->createCharacteristic(
    BLE_TELEMETRY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  controlCharacteristic = service->createCharacteristic(
    BLE_CONTROL_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );

  commandCharacteristic->setCallbacks(
    new FeedbackCommandCallbacks()
  );

  controlCharacteristic->setCallbacks(
    new ControlCommandCallbacks()
  );

  ackCharacteristic->addDescriptor(new BLE2902());
  telemetryCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  advertising->start();

  Serial.println("READY,BLE,ADVERTISING");
}

// -----------------------------------------------------------------------------
// Serial diagnostics
// -----------------------------------------------------------------------------

const char* feedbackStateName(FeedbackState state) {
  switch (state) {
    case STATE_ONSET:
      return "ONSET";
    case STATE_SUSTAIN:
      return "SUSTAIN";
    case STATE_DECAY:
      return "DECAY";
    case STATE_COOLDOWN:
      return "COOLDOWN";
    default:
      return "IDLE";
  }
}

void printDiagnostics(uint32_t nowMs) {
  if (nowMs - lastSerialLogMs < SERIAL_LOG_INTERVAL_MS) {
    return;
  }

  lastSerialLogMs = nowMs;

  Serial.printf(
    "HOME,%lu,%s,EVENT=%lu,CES=%.3f,PEAK=%.3f,"
    "M1=%u,M2=%u,LED=%u,PTC1=%u,PTC2=%u,"
    "T1=%.2f,T2=%.2f,BPM=%.1f,HEART=%.3f,"
    "MOTION=%.3f,PITCH=%.1f,ROLL=%.1f,FAULT=0x%02X\n",
    static_cast<unsigned long>(nowMs),
    feedbackStateName(feedbackState),
    static_cast<unsigned long>(activeEventId),
    currentEventCes,
    currentEventPeakCes,
    currentMotor1Duty,
    currentMotor2Duty,
    currentLedDuty,
    currentPtc1Duty,
    currentPtc2Duty,
    temperature1C,
    temperature2C,
    currentBpm,
    heartResponse,
    motionScore,
    pitchDegrees,
    rollDegrees,
    activeFaultFlags | latchedFaultFlags
  );
}

// -----------------------------------------------------------------------------
// Arduino setup and loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_VIBRATION_1, OUTPUT);
  pinMode(PIN_VIBRATION_2, OUTPUT);
  pinMode(PIN_PTC_1, OUTPUT);
  pinMode(PIN_PTC_2, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_NTC_1, INPUT);
  pinMode(PIN_NTC_2, INPUT);

  setupPwm();
  stopAllOutputsImmediately();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_NTC_1, ADC_ATTEN_DB_11);
  analogSetPinAttenuation(PIN_NTC_2, ADC_ATTEN_DB_11);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(100);

  setupMax30102();
  setupImu();

  updateNtcTemperatures(millis());
  setupBle();

  enterIdle(millis());

  Serial.println("READY,HOME_MULTIMODAL_FEEDBACK_NODE");
  Serial.println(
    "READY,STATE_MACHINE=IDLE_ONSET_SUSTAIN_DECAY_COOLDOWN"
  );
  Serial.println(
    "READY,OUTPUTS=2_MOTORS_2_PTC_1_LED"
  );
  Serial.println(
    "READY,SENSORS=MAX30102_MPU_COMPATIBLE_2_NTC"
  );
}

void loop() {
  const uint32_t nowMs = millis();

  processPendingBleWrites(nowMs);

  if (nowMs - lastHeartUpdateMs >= SENSOR_HEART_INTERVAL_MS) {
    lastHeartUpdateMs = nowMs;
    updateHeartRateSensor();
    updateHeartResponse();
  }

  updateMotionAndPosture(nowMs);
  updateNtcTemperatures(nowMs);

  if (nowMs - lastControlLoopMs >= CONTROL_LOOP_INTERVAL_MS) {
    lastControlLoopMs = nowMs;
    updateFeedbackStateMachine(nowMs);
  }

  publishTelemetry(nowMs);
  printDiagnostics(nowMs);

  delay(1);
}