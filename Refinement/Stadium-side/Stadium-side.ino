/*
Stadium-side Multimodal Emotion Sensing 
Final BLE Version

Purpose:
Capture live stadium atmosphere and supporter responses, convert them into
an estimated real-time Crowd Emotion Score (CES), detect sudden emotional
spikes, and stream the resulting digital emotion data directly to the
stadium spectator's mobile app through Bluetooth Low Energy (BLE).

Sensors:
- MAX9814  -> crowd audio excitement
- MAX30102 -> heart-rate response
- MPU6050 / compatible WHO_AM_I 0x70 module -> body-motion intensity

Fusion model:
CES = wa*A + wh*H + wm*M

Default prototype weights:
wa = 0.60, wh = 0.25, wm = 0.15

Wireless architecture:
Stadium-side ESP32 -> BLE -> Stadium mobile app -> Internet / cloud

BLE service:
- Emotion characteristic: 20-byte binary packet, READ + NOTIFY, 10 Hz
- Control characteristic: 6-byte binary command, WRITE
- Time-sync characteristic: 14-byte binary response, READ + NOTIFY

Important:
1. GPIO34 is used for MAX9814 OUT.
2. MAX30102 and MPU-compatible IMU share SDA GPIO21 / SCL GPIO22.
3. The app must decode all multibyte integer fields as little-endian.
4. The 20-byte emotion packet fits the default BLE ATT payload, so the
   system does not depend on a larger negotiated MTU.
5. Keep the wearer relatively calm and avoid abrupt loud sounds during
   startup calibration.
6. MAX30102 data is used for interaction estimation, not medical diagnosis.
*/

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "heartRate.h"

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------

constexpr uint8_t MIC_PIN = 34;
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t MPU_ADDRESS = 0x68;

// -----------------------------------------------------------------------------
// Device and BLE configuration
// -----------------------------------------------------------------------------

constexpr uint8_t DEVICE_ID = 1;
constexpr char BLE_DEVICE_NAME[] = "StadiumEmotion-01";

constexpr char BLE_SERVICE_UUID[] =
  "7d2f0001-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr char BLE_EMOTION_CHARACTERISTIC_UUID[] =
  "7d2f0002-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr char BLE_CONTROL_CHARACTERISTIC_UUID[] =
  "7d2f0003-7f30-4e7f-a9e4-8f5e8a3d1c01";

constexpr char BLE_TIME_SYNC_CHARACTERISTIC_UUID[] =
  "7d2f0004-7f30-4e7f-a9e4-8f5e8a3d1c01";

// Packet type occupies the high nibble and protocol version occupies the low.
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t EMOTION_PACKET_TYPE = 0xE;
constexpr uint8_t TIME_SYNC_PACKET_TYPE = 0xA;

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
// CES model configuration
// -----------------------------------------------------------------------------

constexpr float AUDIO_WEIGHT = 0.60f;
constexpr float HEART_WEIGHT = 0.25f;
constexpr float MOTION_WEIGHT = 0.15f;

static_assert(
  AUDIO_WEIGHT + HEART_WEIGHT + MOTION_WEIGHT > 0.999f &&
  AUDIO_WEIGHT + HEART_WEIGHT + MOTION_WEIGHT < 1.001f,
  "CES weights must sum to 1.0."
);

// -----------------------------------------------------------------------------
// Timing configuration
// -----------------------------------------------------------------------------

constexpr uint32_t SENSOR_FUSION_INTERVAL_MS = 100;
constexpr uint32_t MOTION_UPDATE_INTERVAL_MS = 20;
constexpr uint32_t SERIAL_LOG_INTERVAL_MS = 500;

constexpr uint32_t AUDIO_CALIBRATION_MS = 5000;
constexpr uint32_t HEART_CALIBRATION_MAX_MS = 30000;
constexpr uint8_t HEART_BASELINE_MIN_BEATS = 12;
constexpr uint8_t HEART_BASELINE_TARGET_BEATS = 20;

// -----------------------------------------------------------------------------
// MAX9814 audio configuration
// -----------------------------------------------------------------------------

constexpr uint16_t AUDIO_SAMPLE_COUNT = 192;
constexpr uint32_t AUDIO_SAMPLE_INTERVAL_US = 125;
constexpr float AUDIO_DYNAMIC_RANGE_DB = 22.0f;
constexpr float AUDIO_GATE_DB = 2.0f;

// -----------------------------------------------------------------------------
// MAX30102 configuration
// -----------------------------------------------------------------------------

constexpr uint32_t FINGER_IR_THRESHOLD = 50000;
constexpr float BPM_MIN = 35.0f;
constexpr float BPM_MAX = 220.0f;

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

// -----------------------------------------------------------------------------
// Status flags contained in each emotion packet
// -----------------------------------------------------------------------------

enum StatusFlag : uint8_t {
  STATUS_AUDIO_CALIBRATED = 1u << 0,
  STATUS_MAX30102_READY = 1u << 1,
  STATUS_FINGER_PRESENT = 1u << 2,
  STATUS_HEART_BASELINE_READY = 1u << 3,
  STATUS_IMU_READY = 1u << 4,
  STATUS_BLE_CONNECTED = 1u << 5
};

// -----------------------------------------------------------------------------
// Compact BLE protocol
// -----------------------------------------------------------------------------

#pragma pack(push, 1)

/*
20-byte emotion packet.

Normalized values are quantized to 0...255:
value = encoded / 255.0

quality represents the fraction of the CES model currently supported by
valid sensors. Example: audio + IMU available = 0.75 -> approximately 191.
*/
struct EmotionPacket {
  uint8_t typeVersion;
  uint8_t deviceId;
  uint8_t statusFlags;
  uint8_t quality;

  uint32_t sequence;
  uint32_t timestampMs;

  uint8_t ces;
  uint8_t spike;
  uint8_t audio;
  uint8_t heart;
  uint8_t motion;
  uint8_t bpm;

  uint16_t crc16;
};

/*
6-byte command written by the stadium mobile app.

command = 0x01: time-sync request
command = 0x02: restart sensor calibration
*/
struct ControlCommandPacket {
  uint8_t command;
  uint8_t version;
  uint16_t requestId;
  uint16_t crc16;
};

/*
14-byte time-sync response.

The app records its own monotonic send and receive timestamps. Together with
receiveTimestampMs and transmitTimestampMs, it can estimate the BLE offset
using an NTP-style midpoint calculation without requiring Internet time on
this ESP32.
*/
struct TimeSyncResponsePacket {
  uint8_t typeVersion;
  uint8_t deviceId;
  uint16_t requestId;
  uint32_t receiveTimestampMs;
  uint32_t transmitTimestampMs;
  uint16_t crc16;
};

#pragma pack(pop)

static_assert(
  sizeof(EmotionPacket) == 20,
  "EmotionPacket must remain exactly 20 bytes."
);

static_assert(
  sizeof(ControlCommandPacket) == 6,
  "ControlCommandPacket must remain exactly 6 bytes."
);

static_assert(
  sizeof(TimeSyncResponsePacket) == 14,
  "TimeSyncResponsePacket must remain exactly 14 bytes."
);

// -----------------------------------------------------------------------------
// BLE command identifiers
// -----------------------------------------------------------------------------

constexpr uint8_t COMMAND_TIME_SYNC = 0x01;
constexpr uint8_t COMMAND_RECALIBRATE = 0x02;

// -----------------------------------------------------------------------------
// Global sensor and system state
// -----------------------------------------------------------------------------

MAX30105 heartSensor;

bool max30102Ready = false;
bool imuReady = false;
bool audioCalibrated = false;
bool fingerPresent = false;
bool fusionInitialized = false;

uint8_t imuWhoAmI = 0;

float audioScore = 0.0f;
float heartScore = 0.0f;
float motionScore = 0.0f;
float crowdEmotionScore = 0.0f;
float emotionSpike = 0.0f;
float fusionQuality = 0.0f;

float audioNoiseFloorDb = 0.0f;
float audioSlowLevel = 0.0f;
float cesSlowBaseline = 0.0f;

float currentBpm = 0.0f;
float baselineBpm = 0.0f;
float baselineBpmSum = 0.0f;

uint8_t baselineBeatCount = 0;
uint32_t heartCalibrationStartMs = 0;
uint32_t lastBeatMs = 0;
uint32_t irValue = 0;

float previousAccelerationMagnitude = 1.0f;

uint32_t bootMs = 0;
uint32_t lastFusionMs = 0;
uint32_t lastMotionMs = 0;
uint32_t lastSerialLogMs = 0;
uint32_t packetSequence = 0;

EmotionPacket outgoingEmotionPacket = {};
TimeSyncResponsePacket outgoingTimeSyncPacket = {};

// -----------------------------------------------------------------------------
// BLE state
// -----------------------------------------------------------------------------

BLEServer* bleServer = nullptr;
BLECharacteristic* emotionCharacteristic = nullptr;
BLECharacteristic* controlCharacteristic = nullptr;
BLECharacteristic* timeSyncCharacteristic = nullptr;

volatile bool bleClientConnected = false;
bool previousBleClientConnected = false;

volatile bool timeSyncPending = false;
volatile uint16_t pendingSyncRequestId = 0;
volatile uint32_t pendingSyncReceiveTimestampMs = 0;
volatile bool recalibrationPending = false;

portMUX_TYPE controlStateMux = portMUX_INITIALIZER_UNLOCKED;

// -----------------------------------------------------------------------------
// General utility functions
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
  float attackAlpha,
  float releaseAlpha
) {
  const float alpha = target > current ? attackAlpha : releaseAlpha;
  return current + alpha * (target - current);
}

uint8_t quantizeUnitFloat(float value) {
  return static_cast<uint8_t>(
    lroundf(clamp01(value) * 255.0f)
  );
}

uint8_t quantizeBpm(float bpm) {
  if (bpm <= 0.0f) {
    return 0;
  }

  if (bpm >= 255.0f) {
    return 255;
  }

  return static_cast<uint8_t>(lroundf(bpm));
}

uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

// -----------------------------------------------------------------------------
// I2C helpers and direct MPU-compatible driver
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

    destination[index] = static_cast<uint8_t>(Wire.read());
  }

  return true;
}

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

// -----------------------------------------------------------------------------
// MAX30102 setup and heart-rate processing
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

  heartScore = smoothAttackRelease(
    heartScore,
    0.0f,
    0.35f,
    0.12f
  );
}

void registerBaselineBeat(float bpm, uint32_t nowMs) {
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
      "READY,HEART_BASELINE,%.1f\n",
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

void updateHeartScore() {
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
      audioScore < 0.20f &&
      motionScore < 0.15f &&
      currentBpm <= baselineBpm + 3.0f
    ) {
      baselineBpm =
        0.999f * baselineBpm +
        0.001f * currentBpm;
    }
  }

  heartScore = smoothAttackRelease(
    heartScore,
    target,
    0.35f,
    0.08f
  );
}

// -----------------------------------------------------------------------------
// MAX9814 audio processing
// -----------------------------------------------------------------------------

float sampleMicrophoneRms() {
  double sum = 0.0;
  double sumSquares = 0.0;

  uint32_t nextSampleUs = micros();

  for (
    uint16_t index = 0;
    index < AUDIO_SAMPLE_COUNT;
    ++index
  ) {
    while (
      static_cast<int32_t>(micros() - nextSampleUs) < 0
    ) {
      delayMicroseconds(1);
    }

    const float sample =
      static_cast<float>(analogRead(MIC_PIN));

    sum += sample;
    sumSquares +=
      static_cast<double>(sample) *
      static_cast<double>(sample);

    nextSampleUs += AUDIO_SAMPLE_INTERVAL_US;
  }

  const double mean =
    sum / static_cast<double>(AUDIO_SAMPLE_COUNT);

  double variance =
    sumSquares / static_cast<double>(AUDIO_SAMPLE_COUNT) -
    mean * mean;

  if (variance < 0.0) {
    variance = 0.0;
  }

  return static_cast<float>(sqrt(variance));
}

void updateAudioScore(uint32_t nowMs) {
  const float rms = sampleMicrophoneRms();
  const float currentDb =
    20.0f * log10f(rms + 1.0f);

  if (!audioCalibrated) {
    if (audioNoiseFloorDb <= 0.0f) {
      audioNoiseFloorDb = currentDb;
    }

    if (currentDb < audioNoiseFloorDb) {
      audioNoiseFloorDb =
        0.80f * audioNoiseFloorDb +
        0.20f * currentDb;
    } else {
      audioNoiseFloorDb =
        0.98f * audioNoiseFloorDb +
        0.02f * currentDb;
    }

    if (nowMs - bootMs >= AUDIO_CALIBRATION_MS) {
      audioCalibrated = true;
      audioSlowLevel = 0.0f;

      Serial.printf(
        "READY,AUDIO,CALIBRATED,NOISE_DB=%.2f\n",
        audioNoiseFloorDb
      );
    }

    audioScore = 0.0f;
    return;
  }

  if (currentDb < audioNoiseFloorDb + 3.0f) {
    audioNoiseFloorDb =
      0.998f * audioNoiseFloorDb +
      0.002f * currentDb;
  }

  const float level = clamp01(
    (
      currentDb -
      audioNoiseFloorDb -
      AUDIO_GATE_DB
    ) /
    AUDIO_DYNAMIC_RANGE_DB
  );

  audioSlowLevel =
    0.985f * audioSlowLevel +
    0.015f * level;

  const float rise = clamp01(
    (level - audioSlowLevel) / 0.35f
  );

  const float target =
    0.75f * level +
    0.25f * rise;

  audioScore = smoothAttackRelease(
    audioScore,
    clamp01(target),
    0.45f,
    0.08f
  );
}

// -----------------------------------------------------------------------------
// MPU motion processing
// -----------------------------------------------------------------------------

void updateMotionScore(uint32_t nowMs) {
  if (!imuReady) {
    motionScore = smoothAttackRelease(
      motionScore,
      0.0f,
      0.35f,
      0.08f
    );
    return;
  }

  if (
    nowMs - lastMotionMs <
    MOTION_UPDATE_INTERVAL_MS
  ) {
    return;
  }

  lastMotionMs = nowMs;

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
    static_cast<float>(rawAx) /
    ACCEL_SCALE_LSB_PER_G;

  const float ay =
    static_cast<float>(rawAy) /
    ACCEL_SCALE_LSB_PER_G;

  const float az =
    static_cast<float>(rawAz) /
    ACCEL_SCALE_LSB_PER_G;

  const float gx =
    static_cast<float>(rawGx) /
    GYRO_SCALE_LSB_PER_DPS;

  const float gy =
    static_cast<float>(rawGy) /
    GYRO_SCALE_LSB_PER_DPS;

  const float gz =
    static_cast<float>(rawGz) /
    GYRO_SCALE_LSB_PER_DPS;

  const float accelerationMagnitude =
    sqrtf(ax * ax + ay * ay + az * az);

  const float linearAcceleration =
    fabsf(accelerationMagnitude - 1.0f);

  const float jerk =
    fabsf(
      accelerationMagnitude -
      previousAccelerationMagnitude
    );

  previousAccelerationMagnitude =
    accelerationMagnitude;

  const float gyroMagnitude =
    sqrtf(gx * gx + gy * gy + gz * gz);

  const float accelerationComponent =
    clamp01(linearAcceleration / 1.20f);

  const float jerkComponent =
    clamp01(jerk / 0.80f);

  const float gyroComponent =
    clamp01(gyroMagnitude / 250.0f);

  const float target =
    0.45f * accelerationComponent +
    0.25f * jerkComponent +
    0.30f * gyroComponent;

  motionScore = smoothAttackRelease(
    motionScore,
    clamp01(target),
    0.40f,
    0.08f
  );
}

// -----------------------------------------------------------------------------
// CES fusion and emotional-spike detection
// -----------------------------------------------------------------------------

void updateEmotionFusion() {
  float weightedSum = 0.0f;
  float activeWeight = 0.0f;

  if (audioCalibrated) {
    weightedSum +=
      AUDIO_WEIGHT * audioScore;

    activeWeight += AUDIO_WEIGHT;
  }

  if (
    max30102Ready &&
    fingerPresent &&
    baselineBpm > 0.0f
  ) {
    weightedSum +=
      HEART_WEIGHT * heartScore;

    activeWeight += HEART_WEIGHT;
  }

  if (imuReady) {
    weightedSum +=
      MOTION_WEIGHT * motionScore;

    activeWeight += MOTION_WEIGHT;
  }

  fusionQuality = clamp01(activeWeight);

  const float targetCes =
    activeWeight > 0.0f
    ? clamp01(weightedSum / activeWeight)
    : 0.0f;

  if (!fusionInitialized && activeWeight > 0.0f) {
    crowdEmotionScore = targetCes;
    cesSlowBaseline = targetCes;
    emotionSpike = 0.0f;
    fusionInitialized = true;
    return;
  }

  crowdEmotionScore = smoothAttackRelease(
    crowdEmotionScore,
    targetCes,
    0.35f,
    0.06f
  );

  const float spikeTarget = clamp01(
    (
      crowdEmotionScore -
      cesSlowBaseline
    ) /
    0.30f
  );

  emotionSpike = smoothAttackRelease(
    emotionSpike,
    spikeTarget,
    0.55f,
    0.12f
  );

  cesSlowBaseline =
    0.992f * cesSlowBaseline +
    0.008f * crowdEmotionScore;
}

// -----------------------------------------------------------------------------
// Calibration control
// -----------------------------------------------------------------------------

void restartCalibration(uint32_t nowMs) {
  bootMs = nowMs;

  audioCalibrated = false;
  audioNoiseFloorDb = 0.0f;
  audioSlowLevel = 0.0f;
  audioScore = 0.0f;

  baselineBpm = 0.0f;
  baselineBpmSum = 0.0f;
  baselineBeatCount = 0;
  heartCalibrationStartMs = 0;
  currentBpm = 0.0f;
  lastBeatMs = 0;
  heartScore = 0.0f;

  motionScore = 0.0f;
  previousAccelerationMagnitude = 1.0f;

  crowdEmotionScore = 0.0f;
  emotionSpike = 0.0f;
  cesSlowBaseline = 0.0f;
  fusionQuality = 0.0f;
  fusionInitialized = false;

  Serial.println(
    "CALIBRATION,RESTARTED,"
    "KEEP_WEARER_CALM_AND_AVOID_LOUD_TRANSIENTS"
  );
}

// -----------------------------------------------------------------------------
// BLE status and packet construction
// -----------------------------------------------------------------------------

uint8_t buildStatusFlags() {
  uint8_t flags = 0;

  if (audioCalibrated) {
    flags |= STATUS_AUDIO_CALIBRATED;
  }

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

  if (bleClientConnected) {
    flags |= STATUS_BLE_CONNECTED;
  }

  return flags;
}

void buildEmotionPacket(uint32_t nowMs) {
  outgoingEmotionPacket.typeVersion =
    makeTypeVersion(
      EMOTION_PACKET_TYPE,
      PROTOCOL_VERSION
    );

  outgoingEmotionPacket.deviceId = DEVICE_ID;
  outgoingEmotionPacket.statusFlags = buildStatusFlags();
  outgoingEmotionPacket.quality =
    quantizeUnitFloat(fusionQuality);

  outgoingEmotionPacket.sequence = packetSequence++;
  outgoingEmotionPacket.timestampMs = nowMs;

  outgoingEmotionPacket.ces =
    quantizeUnitFloat(crowdEmotionScore);

  outgoingEmotionPacket.spike =
    quantizeUnitFloat(emotionSpike);

  outgoingEmotionPacket.audio =
    quantizeUnitFloat(audioScore);

  outgoingEmotionPacket.heart =
    quantizeUnitFloat(heartScore);

  outgoingEmotionPacket.motion =
    quantizeUnitFloat(motionScore);

  outgoingEmotionPacket.bpm =
    quantizeBpm(currentBpm);

  outgoingEmotionPacket.crc16 = 0;

  outgoingEmotionPacket.crc16 = crc16Ccitt(
    reinterpret_cast<const uint8_t*>(
      &outgoingEmotionPacket
    ),
    offsetof(EmotionPacket, crc16)
  );
}

void publishEmotionPacket() {
  if (emotionCharacteristic == nullptr) {
    return;
  }

  emotionCharacteristic->setValue(
    reinterpret_cast<uint8_t*>(
      &outgoingEmotionPacket
    ),
    sizeof(outgoingEmotionPacket)
  );

  if (bleClientConnected) {
    emotionCharacteristic->notify();
  }
}

// -----------------------------------------------------------------------------
// BLE callbacks
// -----------------------------------------------------------------------------

class StadiumServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    (void)server;
    bleClientConnected = true;
    Serial.println("READY,BLE,CLIENT_CONNECTED");
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    bleClientConnected = false;
    Serial.println("READY,BLE,CLIENT_DISCONNECTED");
  }
};

class StadiumControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const String value = characteristic->getValue();

    if (value.length() != sizeof(ControlCommandPacket)) {
      Serial.printf(
        "ERROR,BLE,CONTROL_LENGTH,%u\n",
        static_cast<unsigned int>(value.length())
      );
      return;
    }

    ControlCommandPacket command = {};

    memcpy(
      &command,
      value.c_str(),
      sizeof(command)
    );

    const uint16_t receivedCrc = command.crc16;
    command.crc16 = 0;

    const uint16_t expectedCrc = crc16Ccitt(
      reinterpret_cast<const uint8_t*>(&command),
      offsetof(ControlCommandPacket, crc16)
    );

    if (receivedCrc != expectedCrc) {
      Serial.println("ERROR,BLE,CONTROL_CRC");
      return;
    }

    if (command.version != PROTOCOL_VERSION) {
      Serial.printf(
        "ERROR,BLE,CONTROL_VERSION,%u\n",
        command.version
      );
      return;
    }

    const uint32_t receiveTimestampMs = millis();

    portENTER_CRITICAL(&controlStateMux);

    if (command.command == COMMAND_TIME_SYNC) {
      pendingSyncRequestId = command.requestId;
      pendingSyncReceiveTimestampMs = receiveTimestampMs;
      timeSyncPending = true;
    } else if (command.command == COMMAND_RECALIBRATE) {
      recalibrationPending = true;
    }

    portEXIT_CRITICAL(&controlStateMux);

    if (
      command.command != COMMAND_TIME_SYNC &&
      command.command != COMMAND_RECALIBRATE
    ) {
      Serial.printf(
        "ERROR,BLE,UNKNOWN_COMMAND,0x%02X\n",
        command.command
      );
    }
  }
};

// -----------------------------------------------------------------------------
// BLE setup and time synchronization
// -----------------------------------------------------------------------------

void setupBle() {
  BLEDevice::init(BLE_DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new StadiumServerCallbacks());

  BLEService* service =
    bleServer->createService(BLE_SERVICE_UUID);

  emotionCharacteristic = service->createCharacteristic(
    BLE_EMOTION_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  emotionCharacteristic->addDescriptor(new BLE2902());

  controlCharacteristic = service->createCharacteristic(
    BLE_CONTROL_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  controlCharacteristic->setCallbacks(
    new StadiumControlCallbacks()
  );

  timeSyncCharacteristic = service->createCharacteristic(
    BLE_TIME_SYNC_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  timeSyncCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  advertising->start();

  Serial.printf(
    "READY,BLE,ADVERTISING,NAME=%s\n",
    BLE_DEVICE_NAME
  );
}

void processBleConnectionState() {
  if (
    !bleClientConnected &&
    previousBleClientConnected
  ) {
    delay(200);

    if (bleServer != nullptr) {
      bleServer->startAdvertising();
    }

    previousBleClientConnected = false;
    Serial.println("READY,BLE,ADVERTISING_RESTARTED");
  }

  if (
    bleClientConnected &&
    !previousBleClientConnected
  ) {
    previousBleClientConnected = true;
  }
}

void processControlRequests(uint32_t nowMs) {
  bool sendTimeSync = false;
  bool restartSensors = false;
  uint16_t requestId = 0;
  uint32_t receiveTimestampMs = 0;

  portENTER_CRITICAL(&controlStateMux);

  if (timeSyncPending) {
    sendTimeSync = true;
    requestId = pendingSyncRequestId;
    receiveTimestampMs = pendingSyncReceiveTimestampMs;
    timeSyncPending = false;
  }

  if (recalibrationPending) {
    restartSensors = true;
    recalibrationPending = false;
  }

  portEXIT_CRITICAL(&controlStateMux);

  if (restartSensors) {
    restartCalibration(nowMs);
  }

  if (
    sendTimeSync &&
    timeSyncCharacteristic != nullptr
  ) {
    outgoingTimeSyncPacket.typeVersion =
      makeTypeVersion(
        TIME_SYNC_PACKET_TYPE,
        PROTOCOL_VERSION
      );

    outgoingTimeSyncPacket.deviceId = DEVICE_ID;
    outgoingTimeSyncPacket.requestId = requestId;
    outgoingTimeSyncPacket.receiveTimestampMs =
      receiveTimestampMs;

    outgoingTimeSyncPacket.transmitTimestampMs =
      millis();

    outgoingTimeSyncPacket.crc16 = 0;

    outgoingTimeSyncPacket.crc16 = crc16Ccitt(
      reinterpret_cast<const uint8_t*>(
        &outgoingTimeSyncPacket
      ),
      offsetof(TimeSyncResponsePacket, crc16)
    );

    timeSyncCharacteristic->setValue(
      reinterpret_cast<uint8_t*>(
        &outgoingTimeSyncPacket
      ),
      sizeof(outgoingTimeSyncPacket)
    );

    if (bleClientConnected) {
      timeSyncCharacteristic->notify();
    }
  }
}

// -----------------------------------------------------------------------------
// Serial diagnostics
// -----------------------------------------------------------------------------

void printDiagnostics(uint32_t nowMs) {
  if (
    nowMs - lastSerialLogMs <
    SERIAL_LOG_INTERVAL_MS
  ) {
    return;
  }

  lastSerialLogMs = nowMs;

  Serial.printf(
    "EMOTION,%u,%u,%lu,%lu,"
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
    "%.1f,%lu,0x%02X,%s\n",
    PROTOCOL_VERSION,
    DEVICE_ID,
    static_cast<unsigned long>(
      outgoingEmotionPacket.sequence
    ),
    static_cast<unsigned long>(nowMs),
    crowdEmotionScore,
    emotionSpike,
    audioScore,
    heartScore,
    motionScore,
    fusionQuality,
    currentBpm,
    static_cast<unsigned long>(irValue),
    buildStatusFlags(),
    bleClientConnected ? "BLE_CONNECTED" : "BLE_WAITING"
  );
}

// -----------------------------------------------------------------------------
// Arduino setup and loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  bootMs = millis();

  analogReadResolution(12);
  analogSetPinAttenuation(
    MIC_PIN,
    ADC_ATTEN_DB_11
  );

  pinMode(MIC_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(100);

  setupMax30102();
  setupImu();
  setupBle();

  Serial.println(
    "READY,STADIUM_MULTIMODAL_EMOTION_NODE_BLE"
  );

  Serial.println(
    "READY,EMOTION_PACKET=20_BYTES,"
    "TYPE_VERSION_DEVICE_STATUS_QUALITY_SEQUENCE_TIMESTAMP_"
    "CES_SPIKE_AUDIO_HEART_MOTION_BPM_CRC16"
  );

  Serial.println(
    "READY,CONTROL_COMMANDS="
    "0x01_TIME_SYNC,0x02_RECALIBRATE"
  );

  Serial.println(
    "CALIBRATION,"
    "KEEP_WEARER_CALM_AND_AVOID_LOUD_TRANSIENTS"
  );
}

void loop() {
  const uint32_t nowMs = millis();

  processBleConnectionState();
  processControlRequests(nowMs);

  updateHeartRateSensor();
  updateMotionScore(nowMs);

  if (
    nowMs - lastFusionMs >=
    SENSOR_FUSION_INTERVAL_MS
  ) {
    lastFusionMs = nowMs;

    updateAudioScore(nowMs);
    updateHeartScore();
    updateEmotionFusion();

    buildEmotionPacket(nowMs);
    publishEmotionPacket();
  }

  printDiagnostics(nowMs);

  delay(1);
}