import Foundation

// MARK: - Protocol errors

enum StadiumProtocolError: Error, LocalizedError, Equatable {
    case wrongLength(expected: Int, actual: Int)
    case crcMismatch(expected: UInt16, received: UInt16)
    case wrongPacketType(expected: UInt8, actual: UInt8)
    case unsupportedVersion(expected: UInt8, actual: UInt8)
    case unknownTimeSyncRequest(UInt16)

    var errorDescription: String? {
        switch self {
        case .wrongLength(let expected, let actual):
            return "Wrong packet length. Expected \(expected), received \(actual)."
        case .crcMismatch(let expected, let received):
            return String(
                format: "CRC mismatch. Expected 0x%04X, received 0x%04X.",
                expected,
                received
            )
        case .wrongPacketType(let expected, let actual):
            return String(
                format: "Wrong packet type. Expected 0x%X, received 0x%X.",
                expected,
                actual
            )
        case .unsupportedVersion(let expected, let actual):
            return "Unsupported protocol version. Expected \(expected), received \(actual)."
        case .unknownTimeSyncRequest(let requestID):
            return "Unknown time-sync request ID \(requestID)."
        }
    }
}

// MARK: - Binary helpers

enum BinaryCodec {
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

    static func appendUInt16LE(_ value: UInt16, to bytes: inout [UInt8]) {
        bytes.append(UInt8(truncatingIfNeeded: value))
        bytes.append(UInt8(truncatingIfNeeded: value >> 8))
    }
}

enum CRC16 {
    /// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF.
    static func ccittFalse<C: Collection>(_ bytes: C) -> UInt16 where C.Element == UInt8 {
        var crc: UInt16 = 0xFFFF

        for byte in bytes {
            crc ^= UInt16(byte) << 8

            for _ in 0..<8 {
                if (crc & 0x8000) != 0 {
                    crc = (crc << 1) ^ 0x1021
                } else {
                    crc <<= 1
                }
            }
        }

        return crc
    }
}

// MARK: - Emotion packet

struct SensorStatusFlags: OptionSet, Equatable, Sendable {
    let rawValue: UInt8

    static let audioCalibrated = SensorStatusFlags(rawValue: 1 << 0)
    static let max30102Ready = SensorStatusFlags(rawValue: 1 << 1)
    static let fingerPresent = SensorStatusFlags(rawValue: 1 << 2)
    static let heartBaselineReady = SensorStatusFlags(rawValue: 1 << 3)
    static let imuReady = SensorStatusFlags(rawValue: 1 << 4)
    static let bleConnected = SensorStatusFlags(rawValue: 1 << 5)
}

struct EmotionPacket: Equatable, Sendable {
    static let byteCount = 20
    static let packetType: UInt8 = 0xE
    static let protocolVersion: UInt8 = 1

    let version: UInt8
    let deviceID: UInt8
    let status: SensorStatusFlags
    let qualityByte: UInt8
    let sequence: UInt32
    let timestampMs: UInt32
    let cesByte: UInt8
    let spikeByte: UInt8
    let audioByte: UInt8
    let heartByte: UInt8
    let motionByte: UInt8
    let bpm: UInt8
    let crc16: UInt16

    var quality: Double { Double(qualityByte) / 255.0 }
    var ces: Double { Double(cesByte) / 255.0 }
    var spike: Double { Double(spikeByte) / 255.0 }
    var audio: Double { Double(audioByte) / 255.0 }
    var heart: Double { Double(heartByte) / 255.0 }
    var motion: Double { Double(motionByte) / 255.0 }

    static func decode(_ data: Data) throws -> EmotionPacket {
        guard data.count == byteCount else {
            throw StadiumProtocolError.wrongLength(
                expected: byteCount,
                actual: data.count
            )
        }

        let bytes = [UInt8](data)
        let receivedCRC = BinaryCodec.uint16LE(bytes, at: 18)
        let expectedCRC = CRC16.ccittFalse(bytes.prefix(18))

        guard receivedCRC == expectedCRC else {
            throw StadiumProtocolError.crcMismatch(
                expected: expectedCRC,
                received: receivedCRC
            )
        }

        let typeVersion = bytes[0]
        let type = typeVersion >> 4
        let version = typeVersion & 0x0F

        guard type == packetType else {
            throw StadiumProtocolError.wrongPacketType(
                expected: packetType,
                actual: type
            )
        }

        guard version == protocolVersion else {
            throw StadiumProtocolError.unsupportedVersion(
                expected: protocolVersion,
                actual: version
            )
        }

        return EmotionPacket(
            version: version,
            deviceID: bytes[1],
            status: SensorStatusFlags(rawValue: bytes[2]),
            qualityByte: bytes[3],
            sequence: BinaryCodec.uint32LE(bytes, at: 4),
            timestampMs: BinaryCodec.uint32LE(bytes, at: 8),
            cesByte: bytes[12],
            spikeByte: bytes[13],
            audioByte: bytes[14],
            heartByte: bytes[15],
            motionByte: bytes[16],
            bpm: bytes[17],
            crc16: receivedCRC
        )
    }
}

struct EmotionSample: Equatable, Sendable {
    let packet: EmotionPacket
    let extendedDeviceTimestampMs: UInt64
    let eventDate: Date
    let receivedAt: Date
}

// MARK: - Control and time synchronization packets

enum ControlCommand: UInt8, Sendable {
    case timeSync = 0x01
    case recalibrate = 0x02

    static let protocolVersion: UInt8 = 1
    static let byteCount = 6

    func encode(requestID: UInt16) -> Data {
        var bytes: [UInt8] = [rawValue, Self.protocolVersion]
        BinaryCodec.appendUInt16LE(requestID, to: &bytes)

        let crc = CRC16.ccittFalse(bytes)
        BinaryCodec.appendUInt16LE(crc, to: &bytes)

        precondition(bytes.count == Self.byteCount)
        return Data(bytes)
    }
}

struct TimeSyncResponsePacket: Equatable, Sendable {
    static let byteCount = 14
    static let packetType: UInt8 = 0xA
    static let protocolVersion: UInt8 = 1

    let version: UInt8
    let deviceID: UInt8
    let requestID: UInt16
    let receiveTimestampMs: UInt32
    let transmitTimestampMs: UInt32
    let crc16: UInt16

    static func decode(_ data: Data) throws -> TimeSyncResponsePacket {
        guard data.count == byteCount else {
            throw StadiumProtocolError.wrongLength(
                expected: byteCount,
                actual: data.count
            )
        }

        let bytes = [UInt8](data)
        let receivedCRC = BinaryCodec.uint16LE(bytes, at: 12)
        let expectedCRC = CRC16.ccittFalse(bytes.prefix(12))

        guard receivedCRC == expectedCRC else {
            throw StadiumProtocolError.crcMismatch(
                expected: expectedCRC,
                received: receivedCRC
            )
        }

        let typeVersion = bytes[0]
        let type = typeVersion >> 4
        let version = typeVersion & 0x0F

        guard type == packetType else {
            throw StadiumProtocolError.wrongPacketType(
                expected: packetType,
                actual: type
            )
        }

        guard version == protocolVersion else {
            throw StadiumProtocolError.unsupportedVersion(
                expected: protocolVersion,
                actual: version
            )
        }

        return TimeSyncResponsePacket(
            version: version,
            deviceID: bytes[1],
            requestID: BinaryCodec.uint16LE(bytes, at: 2),
            receiveTimestampMs: BinaryCodec.uint32LE(bytes, at: 4),
            transmitTimestampMs: BinaryCodec.uint32LE(bytes, at: 8),
            crc16: receivedCRC
        )
    }
}

// MARK: - ESP32 millis() rollover handling

struct DeviceTimestampUnwrapper {
    private static let wrapSize: UInt64 = 1 << 32
    private var latestExtendedTimestamp: UInt64?

    mutating func reset() {
        latestExtendedTimestamp = nil
    }

    mutating func unwrap(_ rawTimestamp: UInt32) -> UInt64 {
        let raw = UInt64(rawTimestamp)

        guard let latest = latestExtendedTimestamp else {
            latestExtendedTimestamp = raw
            return raw
        }

        let epochBase = latest & ~(Self.wrapSize - 1)
        var candidates: [UInt64] = [epochBase + raw]

        if epochBase >= Self.wrapSize {
            candidates.append(epochBase - Self.wrapSize + raw)
        }

        if epochBase <= UInt64.max - Self.wrapSize {
            candidates.append(epochBase + Self.wrapSize + raw)
        }

        let selected = candidates.min {
            absoluteDifference($0, latest) < absoluteDifference($1, latest)
        } ?? raw

        if selected > latest {
            latestExtendedTimestamp = selected
        }

        return selected
    }

    private func absoluteDifference(_ lhs: UInt64, _ rhs: UInt64) -> UInt64 {
        lhs >= rhs ? lhs - rhs : rhs - lhs
    }
}
