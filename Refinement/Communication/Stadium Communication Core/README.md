Stadium Communication Core

Files

- `StadiumCommunicationManager.swift`
  - scans and connects to the Stadium ESP32 through CoreBluetooth
  - subscribes to Emotion and Time Sync notifications
  - sends calibration and time-sync control commands
  - validates packet order and detects ESP32 restarts
  - forwards synchronized emotion samples to the cloud transport

- `StadiumProtocol.swift`
  - decodes the 20-byte Emotion Packet
  - decodes the 14-byte Time Sync response
  - encodes the 6-byte Control Command
  - performs CRC-16/CCITT-FALSE validation
  - handles little-endian fields and ESP32 millis() rollover

- `DeviceTimeSynchronizer.swift`
  - converts the ESP32 monotonic timestamp into UTC with an NTP-style four-timestamp calculation

- `EmotionCloudTransport.swift`
  - converts samples into JSON
  - sends them to the cloud relay over WebSocket
  - maintains ping and automatic reconnection

Data path

Stadium ESP32
  -> BLE Emotion Packet
  -> iPhone packet validation
  -> timestamp synchronization
  -> CloudEmotionMessage JSON
  -> WebSocket cloud relay

Integration

Add these four Swift files to an iOS Xcode target, then construct:

swift
let configuration = StadiumCommunicationConfiguration(
    expectedDeviceNamePrefix: "StadiumEmotion-",
    matchID: "demo-match",
    cloudWebSocketURL: URL(string: "wss://your-server.example/ws")
)

let communication = StadiumCommunicationManager(
    configuration: configuration
)

communication.onEmotionSample = { sample in
    print(sample.packet.ces, sample.eventDate)
}

communication.start()


The Xcode target still requires `NSBluetoothAlwaysUsageDescription` in its Info settings. This package deliberately does not include App or View files because it is intended to show only the communication logic.
