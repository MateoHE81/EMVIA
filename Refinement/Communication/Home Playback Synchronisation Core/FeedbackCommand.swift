import Foundation

public enum FeedbackCommandKind: String, Codable, Sendable {
    case startEvent = "START_EVENT"
    case updateEvent = "UPDATE_EVENT"
    case stopEvent = "STOP_EVENT"
}

public enum FeedbackStopReason: String, Codable, Sendable {
    case emotionReturnedToBaseline
    case maximumDurationReached
    case emotionStreamTimeout
    case playbackPaused
    case playbackTimelineChanged
    case synchronisationUnavailable
    case manualStop
}

/// High-level command only. It deliberately contains no motor PWM, LED PWM,
/// PTC GPIO state or temperature-control instruction.
public struct FeedbackCommand: Codable, Sendable {
    public let protocolVersion: Int
    public let kind: FeedbackCommandKind

    public let eventID: UInt32
    public let sourceDeviceID: UInt8
    public let sourceSequence: UInt32
    public let playbackUtcMs: Int64

    public let ces: Double
    public let peakCES: Double
    public let onsetSpike: Double

    public let maximumEventDurationMs: Int64
    public let remainingEventDurationMs: Int64
    public let isResume: Bool
    public let stopReason: FeedbackStopReason?

    public init(
        protocolVersion: Int = 1,
        kind: FeedbackCommandKind,
        eventID: UInt32,
        sourceDeviceID: UInt8,
        sourceSequence: UInt32,
        playbackUtcMs: Int64,
        ces: Double,
        peakCES: Double,
        onsetSpike: Double,
        maximumEventDurationMs: Int64,
        remainingEventDurationMs: Int64,
        isResume: Bool = false,
        stopReason: FeedbackStopReason? = nil
    ) {
        self.protocolVersion = protocolVersion
        self.kind = kind
        self.eventID = eventID
        self.sourceDeviceID = sourceDeviceID
        self.sourceSequence = sourceSequence
        self.playbackUtcMs = playbackUtcMs
        self.ces = min(max(ces, 0), 1)
        self.peakCES = min(max(peakCES, 0), 1)
        self.onsetSpike = min(max(onsetSpike, 0), 1)
        self.maximumEventDurationMs = max(0, maximumEventDurationMs)
        self.remainingEventDurationMs = max(0, remainingEventDurationMs)
        self.isResume = isResume
        self.stopReason = stopReason
    }
}

@MainActor
public protocol FeedbackCommandSink: AnyObject {
    func send(_ command: FeedbackCommand)
}

/// Useful while the later Home BLE transport is still separate.
@MainActor
public final class ClosureFeedbackCommandSink: FeedbackCommandSink {
    private let handler: (FeedbackCommand) -> Void

    public init(handler: @escaping (FeedbackCommand) -> Void) {
        self.handler = handler
    }

    public func send(_ command: FeedbackCommand) {
        handler(command)
    }
}
