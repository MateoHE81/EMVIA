import Foundation

public struct HomeDeviceStatusFlags: OptionSet, Hashable, Sendable {
    public let rawValue: UInt8

    public init(rawValue: UInt8) {
        self.rawValue = rawValue
    }

    public static let max30102Ready =
        HomeDeviceStatusFlags(rawValue: 1 << 0)
    public static let fingerPresent =
        HomeDeviceStatusFlags(rawValue: 1 << 1)
    public static let heartBaselineReady =
        HomeDeviceStatusFlags(rawValue: 1 << 2)
    public static let imuReady =
        HomeDeviceStatusFlags(rawValue: 1 << 3)
    public static let ntc1Valid =
        HomeDeviceStatusFlags(rawValue: 1 << 4)
    public static let ntc2Valid =
        HomeDeviceStatusFlags(rawValue: 1 << 5)
    public static let bleConnected =
        HomeDeviceStatusFlags(rawValue: 1 << 6)
    public static let feedbackActive =
        HomeDeviceStatusFlags(rawValue: 1 << 7)
}

public struct HomeDeviceFaultFlags: OptionSet, Hashable, Sendable {
    public let rawValue: UInt8

    public init(rawValue: UInt8) {
        self.rawValue = rawValue
    }

    public static let ntc1Invalid =
        HomeDeviceFaultFlags(rawValue: 1 << 0)
    public static let ntc2Invalid =
        HomeDeviceFaultFlags(rawValue: 1 << 1)
    public static let ptc1Overtemperature =
        HomeDeviceFaultFlags(rawValue: 1 << 2)
    public static let ptc2Overtemperature =
        HomeDeviceFaultFlags(rawValue: 1 << 3)
    public static let commandTimeout =
        HomeDeviceFaultFlags(rawValue: 1 << 4)
    public static let bleDisconnected =
        HomeDeviceFaultFlags(rawValue: 1 << 5)
    public static let emergencyStop =
        HomeDeviceFaultFlags(rawValue: 1 << 6)
    public static let protocolFault =
        HomeDeviceFaultFlags(rawValue: 1 << 7)

    public var hasThermalFault: Bool {
        contains(.ntc1Invalid) ||
            contains(.ntc2Invalid) ||
            contains(.ptc1Overtemperature) ||
            contains(.ptc2Overtemperature)
    }

    public var hasCriticalFault: Bool {
        hasThermalFault || contains(.emergencyStop)
    }
}

/// Exact 20-byte telemetry packet emitted by the Home-side ESP32 at 10 Hz.
public struct HomeTelemetry: Hashable, Sendable {
    public static let packetLength = 20
    public static let packetType: UInt8 = 0x0D
    public static let protocolVersion: UInt8 = 1

    public let deviceID: UInt8
    public let status: HomeDeviceStatusFlags
    public let feedbackState: HomeFeedbackState
    public let sequence: UInt16
    public let deviceTimestampMs: UInt32

    public let bpm: UInt8?
    public let heartResponse: Double
    public let motionScore: Double
    public let pitchDegrees: Double?
    public let rollDegrees: Double?
    public let ptc1TemperatureC: Double?
    public let ptc2TemperatureC: Double?

    public let faults: HomeDeviceFaultFlags
    public let crc16: UInt16

    public static func decode(_ data: Data) throws -> HomeTelemetry {
        let bytes = try HomeWireCodec.validatePacket(
            data,
            expectedLength: packetLength,
            expectedType: packetType,
            expectedVersion: protocolVersion
        )

        guard let state = HomeFeedbackState(rawValue: bytes[3]) else {
            throw HomeBLEProtocolError.unknownFeedbackState(bytes[3])
        }

        let pitchRaw = HomeWireCodec.signedInt8(bytes[13])
        let rollRaw = HomeWireCodec.signedInt8(bytes[14])
        let temperature1Raw = HomeWireCodec.signedInt8(bytes[15])
        let temperature2Raw = HomeWireCodec.signedInt8(bytes[16])

        return HomeTelemetry(
            deviceID: bytes[1],
            status: HomeDeviceStatusFlags(rawValue: bytes[2]),
            feedbackState: state,
            sequence: HomeWireCodec.uint16LE(bytes, at: 4),
            deviceTimestampMs: HomeWireCodec.uint32LE(bytes, at: 6),
            bpm: bytes[10] == 0 ? nil : bytes[10],
            heartResponse: HomeWireCodec.dequantizeUnit(bytes[11]),
            motionScore: HomeWireCodec.dequantizeUnit(bytes[12]),
            pitchDegrees: decodeHalfDegreeAngle(pitchRaw),
            rollDegrees: decodeHalfDegreeAngle(rollRaw),
            ptc1TemperatureC: decodeHalfCelsius(temperature1Raw),
            ptc2TemperatureC: decodeHalfCelsius(temperature2Raw),
            faults: HomeDeviceFaultFlags(rawValue: bytes[17]),
            crc16: HomeWireCodec.uint16LE(bytes, at: 18)
        )
    }

    private static func decodeHalfDegreeAngle(_ value: Int8) -> Double? {
        guard value != Int8.min else { return nil }
        return Double(value) * 2.0
    }

    private static func decodeHalfCelsius(_ value: Int8) -> Double? {
        guard value != Int8.min else { return nil }
        return Double(value) / 2.0
    }
}
