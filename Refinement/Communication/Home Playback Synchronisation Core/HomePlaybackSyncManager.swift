import Foundation

public struct HomePlaybackSyncConfiguration: Sendable {
    public let maximumLateSampleAgeMs: Int64
    public let backwardJumpToleranceMs: Int64
    public let forwardJumpToleranceMs: Int64
    public let acceptedDeviceID: UInt8?

    public static let prototypeDefault = HomePlaybackSyncConfiguration(
        maximumLateSampleAgeMs: 1_500,
        backwardJumpToleranceMs: 1_000,
        forwardJumpToleranceMs: 3_000,
        acceptedDeviceID: nil
    )

    public init(
        maximumLateSampleAgeMs: Int64,
        backwardJumpToleranceMs: Int64,
        forwardJumpToleranceMs: Int64,
        acceptedDeviceID: UInt8?
    ) {
        self.maximumLateSampleAgeMs = max(100, maximumLateSampleAgeMs)
        self.backwardJumpToleranceMs = max(100, backwardJumpToleranceMs)
        self.forwardJumpToleranceMs = max(500, forwardJumpToleranceMs)
        self.acceptedDeviceID = acceptedDeviceID
    }
}

/// Orchestrates Cloud reception, playback-time alignment and high-level
/// START_EVENT / UPDATE_EVENT / STOP_EVENT decisions.
@MainActor
public final class HomePlaybackSyncManager {
    public var onError: ((Error) -> Void)?
    public var onReplaySuppressionChanged: ((Bool) -> Void)?

    public private(set) var isReplaySuppressed = false {
        didSet {
            guard oldValue != isReplaySuppressed else { return }
            onReplaySuppressionChanged?(isReplaySuppressed)
        }
    }

    private let receiver: CloudEmotionStreamReceiver
    private let playbackTimeline: PlaybackTimelineProviding
    private let matchContextProvider: MatchContextProviding
    private let decisionEngine: FeedbackDecisionEngine
    private let configuration: HomePlaybackSyncConfiguration
    private let buffer: EmotionTimelineBuffer

    private var lastPlaybackUtcMs: Int64?
    private var lastSnapshot: PlaybackSnapshot?
    private var isStarted = false

    public init(
        receiver: CloudEmotionStreamReceiver,
        playbackTimeline: PlaybackTimelineProviding,
        matchContextProvider: MatchContextProviding,
        commandSink: FeedbackCommandSink,
        syncConfiguration: HomePlaybackSyncConfiguration = .prototypeDefault,
        decisionConfiguration: FeedbackDecisionConfiguration = .prototypeDefault,
        durationPolicy: FeedbackDurationPolicy = FeedbackDurationPolicy(),
        buffer: EmotionTimelineBuffer = EmotionTimelineBuffer()
    ) {
        self.receiver = receiver
        self.playbackTimeline = playbackTimeline
        self.matchContextProvider = matchContextProvider
        self.configuration = syncConfiguration
        self.buffer = buffer
        self.decisionEngine = FeedbackDecisionEngine(
            configuration: decisionConfiguration,
            durationPolicy: durationPolicy,
            commandSink: commandSink
        )

        bindDependencies()
    }

    public func start() {
        guard !isStarted else { return }
        isStarted = true
        playbackTimeline.start()
        receiver.connect()
    }

    public func stop() {
        guard isStarted else { return }
        isStarted = false

        let playbackUtcMs = lastPlaybackUtcMs ?? Self.utcMs(Date())
        decisionEngine.stopManually(playbackUtcMs: playbackUtcMs)

        receiver.disconnect()
        playbackTimeline.stop()
        buffer.removeAll()
        lastPlaybackUtcMs = nil
        lastSnapshot = nil
        isReplaySuppressed = false
    }

    private func bindDependencies() {
        receiver.onEvent = { [weak self] event in
            self?.receive(event)
        }

        receiver.onError = { [weak self] error in
            self?.onError?(error)
        }

        playbackTimeline.onTick = { [weak self] snapshot in
            self?.handlePlaybackTick(snapshot)
        }

        playbackTimeline.onTimeJump = { [weak self] snapshot in
            self?.handlePlaybackTimeJump(snapshot)
        }
    }

    private func receive(_ event: CloudEmotionEvent) {
        if let acceptedDeviceID = configuration.acceptedDeviceID,
           event.deviceID != acceptedDeviceID {
            return
        }

        // A deliberate rewind suppresses live feedback until the viewer returns
        // to the live edge. Normal pausing does not discard the queue.
        guard !isReplaySuppressed else { return }
        buffer.insert(event)
    }

    private func handlePlaybackTick(_ snapshot: PlaybackSnapshot) {
        guard let contentDate = snapshot.contentDate else {
            lastSnapshot = snapshot
            let now = lastPlaybackUtcMs ?? Self.utcMs(Date())
            decisionEngine.setPlaybackSuspended(
                true,
                playbackUtcMs: now
            )
            return
        }

        let playbackUtcMs = Self.utcMs(contentDate)
        defer {
            lastPlaybackUtcMs = playbackUtcMs
            lastSnapshot = snapshot
        }

        detectUnannouncedJump(
            currentPlaybackUtcMs: playbackUtcMs,
            snapshot: snapshot
        )

        if isReplaySuppressed {
            if snapshot.isNearLiveEdge {
                isReplaySuppressed = false
                buffer.removeAll()
                decisionEngine.resetForTimelineChange(
                    playbackUtcMs: playbackUtcMs
                )
            } else {
                return
            }
        }

        if !snapshot.isPlaying {
            decisionEngine.setPlaybackSuspended(
                true,
                playbackUtcMs: playbackUtcMs
            )
            return
        }

        decisionEngine.setPlaybackSuspended(
            false,
            playbackUtcMs: playbackUtcMs
        )

        let oldestAcceptedUtcMs = playbackUtcMs
            - configuration.maximumLateSampleAgeMs

        buffer.removeEvents(olderThanUtcMs: oldestAcceptedUtcMs)

        let dueEvents = buffer.drain(upToUtcMs: playbackUtcMs)
        for event in dueEvents {
            let lateness = playbackUtcMs - event.eventUtcMs
            guard lateness <= configuration.maximumLateSampleAgeMs else {
                continue
            }

            let context = matchContextProvider.context(
                atPlaybackDate: event.eventDate
            )

            decisionEngine.process(
                event,
                playbackUtcMs: playbackUtcMs,
                matchContext: context
            )
        }

        decisionEngine.handlePlaybackTick(
            playbackUtcMs: playbackUtcMs
        )
    }

    private func handlePlaybackTimeJump(_ snapshot: PlaybackSnapshot) {
        guard let currentDate = snapshot.contentDate else {
            return
        }

        let currentUtcMs = Self.utcMs(currentDate)
        let previousUtcMs = lastPlaybackUtcMs
        let delta = previousUtcMs.map { currentUtcMs - $0 }

        buffer.removeAll()
        decisionEngine.resetForTimelineChange(
            playbackUtcMs: currentUtcMs
        )

        if let delta, delta < -configuration.backwardJumpToleranceMs {
            isReplaySuppressed = true
        } else {
            isReplaySuppressed = !snapshot.isNearLiveEdge
        }

        lastPlaybackUtcMs = currentUtcMs
        lastSnapshot = snapshot
    }

    private func detectUnannouncedJump(
        currentPlaybackUtcMs: Int64,
        snapshot: PlaybackSnapshot
    ) {
        guard
            let previousUtcMs = lastPlaybackUtcMs,
            let previousSnapshot = lastSnapshot,
            previousSnapshot.isPlaying,
            snapshot.isPlaying
        else {
            return
        }

        let delta = currentPlaybackUtcMs - previousUtcMs

        if delta < -configuration.backwardJumpToleranceMs {
            buffer.removeAll()
            decisionEngine.resetForTimelineChange(
                playbackUtcMs: currentPlaybackUtcMs
            )
            isReplaySuppressed = true
        } else if delta > configuration.forwardJumpToleranceMs {
            buffer.removeAll()
            decisionEngine.resetForTimelineChange(
                playbackUtcMs: currentPlaybackUtcMs
            )
            isReplaySuppressed = !snapshot.isNearLiveEdge
        }
    }

    private static func utcMs(_ date: Date) -> Int64 {
        Int64((date.timeIntervalSince1970 * 1_000.0).rounded())
    }
}
