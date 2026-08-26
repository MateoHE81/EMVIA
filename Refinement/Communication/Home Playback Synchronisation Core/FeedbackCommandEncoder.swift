import Foundation

public enum HomeBLEProtocolError: Error, LocalizedError, Sendable {
    case unsupportedFeedbackProtocolVersion(Int)
    case invalidPacketLength(expected: Int, actual: Int)
    case invalidPacketType(expected: UInt8, actual: UInt8)
    case unsupportedPacketVersion(UInt8)
    case crcMismatch(expected: UInt16, received: UInt16)
    case unknownCommand(UInt8)
    case unknownFeedbackState(UInt8)
    case unknownAcknowledgementStatus(UInt8)
    case invalidMaintenanceCommand(UInt8)
    case bluetoothUnavailable(String)
    case missingCharacteristic(String)
    case transportNotReady
    case acknowledgementTimeout(commandSequence: UInt16)
    case commandRejected(status: FeedbackAcknowledgementStatus)
    case writeFailed(String)

    public var errorDescription: String? {
        switch self {
        case .unsupportedFeedbackProtocolVersion(let version):
            return "Unsupported FeedbackCommand protocol version: \(version)."
        case .invalidPacketLength(let expected, let actual):
            return "Expected a \(expected)-byte packet, but received \(actual) bytes."
        case .invalidPacketType(let expected, let actual):
            return String(
                format: "Expected packet type 0x%X, but received 0x%X.",
                expected,
                actual
            )
        case .unsupportedPacketVersion(let version):
            return "Unsupported Home-side BLE packet version: \(version)."
        case .crcMismatch(let expected, let received):
            return String(
                format: "CRC mismatch. Expected 0x%04X, received 0x%04X.",
                expected,
                received
            )
        case .unknownCommand(let value):
            return "Unknown feedback command value: \(value)."
        case .unknownFeedbackState(let value):
            return "Unknown Home-side feedback state: \(value)."
        case .unknownAcknowledgementStatus(let value):
            return "Unknown Home-side acknowledgement status: \(value)."
        case .invalidMaintenanceCommand(let value):
            return "Unknown Home-side maintenance command: \(value)."
        case .bluetoothUnavailable(let message):
            return "Bluetooth is unavailable: \(message)."
        case .missingCharacteristic(let uuid):
            return "The Home-side ESP32 is missing BLE characteristic \(uuid)."
        case .transportNotReady:
            return "The Home-side BLE transport is not connected and ready."
        case .acknowledgementTimeout(let sequence):
            return "No acknowledgement was received for command sequence \(sequence)."
        case .commandRejected(let status):
            return "The Home-side ESP32 rejected the command: \(status.description)."
        case .writeFailed(let message):
            return "BLE write failed: \(message)."
        }
    }
}

public enum FeedbackCommandWireKind: UInt8, Sendable {
    case startEvent = 0x01
    case updateEvent = 0x02
    case stopEvent = 0x03

    init(_ kind: FeedbackCommandKind) {
        switch kind {
        case .startEvent:
            self = .startEvent
        case .updateEvent:
            self = .updateEvent
        case .stopEvent:
            self = .stopEvent
        }
    }
}

/// Numeric stop-reason mapping used only on the BLE wire.
/// The ESP32 currently records this field but does not use it to map outputs.
public enum FeedbackStopReasonWireValue: UInt8, Sendable {
    case none = 0
    case emotionReturnedToBaseline = 1
    case maximumDurationReached = 2
    case emotionStreamTimeout = 3
    case playbackPaused = 4
    case playbackTimelineChanged = 5
    case synchronisationUnavailable = 6
    case manualStop = 7

    init(_ reason: FeedbackStopReason?) {
        guard let reason else {
            self = .none
            return
        }

        switch reason {
        case .emotionReturnedToBaseline:
            self = .emotionReturnedToBaseline
        case .maximumDurationReached:
            self = .maximumDurationReached
        case .emotionStreamTimeout:
            self = .emotionStreamTimeout
        case .playbackPaused:
            self = .playbackPaused
        case .playbackTimelineChanged:
            self = .playbackTimelineChanged
        case .synchronisationUnavailable:
            self = .synchronisationUnavailable
        case .manualStop:
            self = .manualStop
        }
    }
}

/// Encodes the high-level Swift `FeedbackCommand` as the exact 20-byte binary
/// packet expected by `HomeSide_Feedback_Final.ino`.
public struct FeedbackCommandEncoder: Sendable {
    public static let protocolVersion: UInt8 = 1
    public static let packetType: UInt8 = 0x0C
    public static let packetLength = 20

    public init() {}

    public func encode(
        _ command: FeedbackCommand,
        commandSequence: UInt16
    ) throws -> Data {
        guard command.protocolVersion == Int(Self.protocolVersion) else {
            throw HomeBLEProtocolError.unsupportedFeedbackProtocolVersion(
                command.protocolVersion
            )
        }

        var bytes = [UInt8]()
        bytes.reserveCapacity(Self.packetLength)

        bytes.append(
            HomeWireCodec.makeTypeVersion(
                packetType: Self.packetType,
                version: Self.protocolVersion
            )
        )
        bytes.append(FeedbackCommandWireKind(command.kind).rawValue)
        bytes.append(command.isResume ? 0x01 : 0x00)
        bytes.append(FeedbackStopReasonWireValue(command.stopReason).rawValue)

        HomeWireCodec.appendUInt32LE(command.eventID, to: &bytes)
        HomeWireCodec.appendUInt16LE(commandSequence, to: &bytes)

        bytes.append(HomeWireCodec.quantizeUnit(command.ces))
        bytes.append(HomeWireCodec.quantizeUnit(command.peakCES))
        bytes.append(HomeWireCodec.quantizeUnit(command.onsetSpike))

        HomeWireCodec.appendUInt16LE(
            HomeWireCodec.durationDeciseconds(
                command.maximumEventDurationMs
            ),
            to: &bytes
        )
        HomeWireCodec.appendUInt16LE(
            HomeWireCodec.durationDeciseconds(
                command.remainingEventDurationMs
            ),
            to: &bytes
        )

        bytes.append(0)

        precondition(bytes.count == 18)
        let crc = HomeWireCodec.crc16CCITTFalse(bytes)
        HomeWireCodec.appendUInt16LE(crc, to: &bytes)

        precondition(bytes.count == Self.packetLength)
        return Data(bytes)
    }
}

public enum HomeMaintenanceCommand: UInt8, Sendable {
    case recalibrateHomeSensors = 0x01
    case clearThermalFaults = 0x02
    case emergencyStop = 0x03
}

public struct HomeMaintenanceCommandEncoder: Sendable {
    public static let packetLength = 6
    public static let protocolVersion: UInt8 = 1

    public init() {}

    public func encode(
        _ command: HomeMaintenanceCommand,
        requestID: UInt16
    ) -> Data {
        var bytes = [UInt8]()
        bytes.reserveCapacity(Self.packetLength)
        bytes.append(command.rawValue)
        bytes.append(Self.protocolVersion)
        HomeWireCodec.appendUInt16LE(requestID, to: &bytes)

        let crc = HomeWireCodec.crc16CCITTFalse(bytes)
        HomeWireCodec.appendUInt16LE(crc, to: &bytes)
        return Data(bytes)
    }
}

enum HomeWireCodec {
    static func makeTypeVersion(
        packetType: UInt8,
        version: UInt8
    ) -> UInt8 {
        ((packetType & 0x0F) << 4) | (version & 0x0F)
    }

    static func packetType(_ byte: UInt8) -> UInt8 {
        byte >> 4
    }

    static func packetVersion(_ byte: UInt8) -> UInt8 {
        byte & 0x0F
    }

    static func quantizeUnit(_ value: Double) -> UInt8 {
        guard value.isFinite else { return 0 }
        let clamped = min(max(value, 0), 1)
        return UInt8((clamped * 255.0).rounded())
    }

    static func dequantizeUnit(_ value: UInt8) -> Double {
        Double(value) / 255.0
    }

    /// The ESP32 protocol uses deciseconds. Rounding upward prevents a positive
    /// remaining duration from becoming zero merely because it is below 100 ms.
    static func durationDeciseconds(_ milliseconds: Int64) -> UInt16 {
        guard milliseconds > 0 else { return 0 }
        let roundedUp = (milliseconds + 99) / 100
        return UInt16(min(roundedUp, Int64(UInt16.max)))
    }

    static func appendUInt16LE(
        _ value: UInt16,
        to bytes: inout [UInt8]
    ) {
        bytes.append(UInt8(truncatingIfNeeded: value))
        bytes.append(UInt8(truncatingIfNeeded: value >> 8))
    }

    static func appendUInt32LE(
        _ value: UInt32,
        to bytes: inout [UInt8]
    ) {
        bytes.append(UInt8(truncatingIfNeeded: value))
        bytes.append(UInt8(truncatingIfNeeded: value >> 8))
        bytes.append(UInt8(truncatingIfNeeded: value >> 16))
        bytes.append(UInt8(truncatingIfNeeded: value >> 24))
    }

    static func uint16LE(_ bytes: [UInt8], at offset: Int) -> UInt16 {
        UInt16(bytes[offset]) |
            (UInt16(bytes[offset + 1]) << 8)
    }

    static func uint32LE(_ bytes: [UInt8], at offset: Int) -> UInt32 {
        UInt32(bytes[offset]) |
            (UInt32(bytes[offset + 1]) << 8) |
            (UInt32(bytes[offset + 2]) << 16) |
            (UInt32(bytes[offset + 3]) << 24)
    }

    static func signedInt8(_ byte: UInt8) -> Int8 {
        Int8(bitPattern: byte)
    }

    static func crc16CCITTFalse<S: Sequence>(
        _ bytes: S
    ) -> UInt16 where S.Element == UInt8 {
        var crc: UInt16 = 0xFFFF

        for byte in bytes {
            crc ^= UInt16(byte) << 8

            for _ in 0 ..< 8 {
                if (crc & 0x8000) != 0 {
                    crc = (crc << 1) ^ 0x1021
                } else {
                    crc <<= 1
                }
            }
        }

        return crc
    }

    static func validatePacket(
        _ data: Data,
        expectedLength: Int,
        expectedType: UInt8,
        expectedVersion: UInt8 = 1
    ) throws -> [UInt8] {
        guard data.count == expectedLength else {
            throw HomeBLEProtocolError.invalidPacketLength(
                expected: expectedLength,
                actual: data.count
            )
        }

        let bytes = [UInt8](data)
        let actualType = packetType(bytes[0])
        let actualVersion = packetVersion(bytes[0])

        guard actualType == expectedType else {
            throw HomeBLEProtocolError.invalidPacketType(
                expected: expectedType,
                actual: actualType
            )
        }

        guard actualVersion == expectedVersion else {
            throw HomeBLEProtocolError.unsupportedPacketVersion(
                actualVersion
            )
        }

        let crcOffset = expectedLength - 2
        let expectedCRC = crc16CCITTFalse(bytes.prefix(crcOffset))
        let receivedCRC = uint16LE(bytes, at: crcOffset)

        guard expectedCRC == receivedCRC else {
            throw HomeBLEProtocolError.crcMismatch(
                expected: expectedCRC,
                received: receivedCRC
            )
        }

        return bytes
    }
}
