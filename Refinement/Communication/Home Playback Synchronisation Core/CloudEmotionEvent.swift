import Foundation

public enum HomeSyncProtocolError: Error, LocalizedError, Sendable {
    case invalidMessageType(String)
    case unsupportedProtocolVersion(Int)
    case unexpectedMatchID(expected: String, actual: String)
    case invalidNormalizedValue(field: String, value: Double)
    case invalidTimestamp(field: String, value: Int64)
    case malformedServerMessage
    case serverError(code: String, message: String)

    public var errorDescription: String? {
        switch self {
        case .invalidMessageType(let type):
            return "Unsupported Cloud Relay message type: \(type)."
        case .unsupportedProtocolVersion(let version):
            return "Unsupported Cloud Relay protocol version: \(version)."
        case .unexpectedMatchID(let expected, let actual):
            return "Expected matchID \(expected), but received \(actual)."
        case .invalidNormalizedValue(let field, let value):
            return "\(field) must be a finite value from 0 to 1; received \(value)."
        case .invalidTimestamp(let field, let value):
            return "\(field) must be a non-negative millisecond timestamp; received \(value)."
        case .malformedServerMessage:
            return "The Cloud Relay sent a malformed JSON message."
        case .serverError(let code, let message):
            return "Cloud Relay error \(code): \(message)"
        }
    }
}

/// Exact JSON object forwarded by the JavaScript Cloud Relay.
/// The Home App never recalculates CES or Spike.
public struct CloudEmotionEvent: Codable, Hashable, Sendable {
    public let type: String
    public let protocolVersion: Int
    public let matchID: String
    public let source: String

    public let deviceID: UInt8
    public let sequence: UInt32
    public let eventUtcMs: Int64
    public let receivedUtcMs: Int64
    public let deviceTimestampMs: UInt64

    public let ces: Double
    public let spike: Double
    public let audio: Double
    public let heart: Double
    public let motion: Double
    public let bpm: UInt8
    public let fusionQuality: Double
    public let statusFlags: UInt8

    public var eventDate: Date {
        Date(timeIntervalSince1970: Double(eventUtcMs) / 1_000.0)
    }

    public var identity: Identity {
        Identity(
            deviceID: deviceID,
            sequence: sequence,
            eventUtcMs: eventUtcMs
        )
    }

    public struct Identity: Hashable, Sendable {
        public let deviceID: UInt8
        public let sequence: UInt32
        public let eventUtcMs: Int64
    }

    public func validated(
        expectedProtocolVersion: Int,
        expectedMatchID: String
    ) throws -> CloudEmotionEvent {
        guard type == "emotion" else {
            throw HomeSyncProtocolError.invalidMessageType(type)
        }

        guard protocolVersion == expectedProtocolVersion else {
            throw HomeSyncProtocolError.unsupportedProtocolVersion(
                protocolVersion
            )
        }

        guard matchID == expectedMatchID else {
            throw HomeSyncProtocolError.unexpectedMatchID(
                expected: expectedMatchID,
                actual: matchID
            )
        }

        try Self.validateUnitValue(ces, field: "ces")
        try Self.validateUnitValue(spike, field: "spike")
        try Self.validateUnitValue(audio, field: "audio")
        try Self.validateUnitValue(heart, field: "heart")
        try Self.validateUnitValue(motion, field: "motion")
        try Self.validateUnitValue(
            fusionQuality,
            field: "fusionQuality"
        )

        guard eventUtcMs >= 0 else {
            throw HomeSyncProtocolError.invalidTimestamp(
                field: "eventUtcMs",
                value: eventUtcMs
            )
        }

        guard receivedUtcMs >= 0 else {
            throw HomeSyncProtocolError.invalidTimestamp(
                field: "receivedUtcMs",
                value: receivedUtcMs
            )
        }

        return self
    }

    private static func validateUnitValue(
        _ value: Double,
        field: String
    ) throws {
        guard value.isFinite, (0.0 ... 1.0).contains(value) else {
            throw HomeSyncProtocolError.invalidNormalizedValue(
                field: field,
                value: value
            )
        }
    }
}

struct RelayMessageEnvelope: Decodable {
    let type: String
}

struct RelayHelloMessage: Decodable {
    let type: String
    let protocolVersion: Int
    let connectionID: String
    let serverUtcMs: Int64
}

struct RelaySubscribedMessage: Decodable {
    let type: String
    let protocolVersion: Int
    let role: String
    let matchID: String
    let homeClients: Int
    let replayed: Int
    let serverUtcMs: Int64
}

struct RelayErrorMessage: Decodable {
    let type: String
    let protocolVersion: Int
    let code: String
    let message: String
    let serverUtcMs: Int64
}

struct RelaySubscriptionMessage: Encodable {
    let type = "subscribe"
    let protocolVersion: Int
    let role = "home"
    let matchID: String
    let resumeAfterEventUtcMs: Int64?
}

struct RelayUnsubscribeMessage: Encodable {
    let type = "unsubscribe"
    let protocolVersion: Int
}
