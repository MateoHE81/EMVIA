import Foundation

public enum HomeFeedbackState: UInt8, Codable, Sendable, CaseIterable {
    case idle = 0
    case onset = 1
    case sustain = 2
    case decay = 3
    case cooldown = 4
}

public enum FeedbackAcknowledgementStatus: UInt8, Codable, Sendable {
    case ok = 0
    case duplicate = 1
    case badLength = 2
    case badCRC = 3
    case badType = 4
    case badVersion = 5
    case unknownCommand = 6
    case eventMismatch = 7
    case faultLatched = 8

    public var isAccepted: Bool {
        self == .ok || self == .duplicate
    }

    public var description: String {
        switch self {
        case .ok:
            return "accepted"
        case .duplicate:
            return "duplicate command already accepted"
        case .badLength:
            return "invalid packet length"
        case .badCRC:
            return "CRC validation failed"
        case .badType:
            return "invalid packet type"
        case .badVersion:
            return "unsupported packet version"
        case .unknownCommand:
            return "unknown command"
        case .eventMismatch:
            return "event ID does not match the active event"
        case .faultLatched:
            return "a safety fault is latched"
        }
    }
}

/// Exact 12-byte acknowledgement emitted by the Home-side ESP32 after each
/// feedback command is accepted or rejected.
public struct CommandAcknowledgement: Hashable, Sendable {
    public static let packetLength = 12
    public static let packetType: UInt8 = 0x0B
    public static let protocolVersion: UInt8 = 1

    public let command: FeedbackCommandWireKind
    public let status: FeedbackAcknowledgementStatus
    public let feedbackState: HomeFeedbackState
    public let eventID: UInt32
    public let commandSequence: UInt16
    public let crc16: UInt16

    public var isAccepted: Bool {
        status.isAccepted
    }

    public static func decode(_ data: Data) throws -> CommandAcknowledgement {
        let bytes = try HomeWireCodec.validatePacket(
            data,
            expectedLength: packetLength,
            expectedType: packetType,
            expectedVersion: protocolVersion
        )

        guard let command = FeedbackCommandWireKind(rawValue: bytes[1]) else {
            throw HomeBLEProtocolError.unknownCommand(bytes[1])
        }

        guard let status = FeedbackAcknowledgementStatus(
            rawValue: bytes[2]
        ) else {
            throw HomeBLEProtocolError.unknownAcknowledgementStatus(bytes[2])
        }

        guard let state = HomeFeedbackState(rawValue: bytes[3]) else {
            throw HomeBLEProtocolError.unknownFeedbackState(bytes[3])
        }

        return CommandAcknowledgement(
            command: command,
            status: status,
            feedbackState: state,
            eventID: HomeWireCodec.uint32LE(bytes, at: 4),
            commandSequence: HomeWireCodec.uint16LE(bytes, at: 8),
            crc16: HomeWireCodec.uint16LE(bytes, at: 10)
        )
    }
}
