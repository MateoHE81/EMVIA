import Foundation

public struct FeedbackDecisionConfiguration: Sendable {
    public let triggerCESThreshold: Double
    public let triggerSpikeThreshold: Double
    public let sustainCESThreshold: Double
    public let resetCESThreshold: Double
    public let minimumFusionQuality: Double

    public let belowSustainGraceMs: Int64
    public let minimumCooldownMs: Int64
    public let resetHoldMs: Int64
    public let updateIntervalMs: Int64
    public let emotionStreamTimeoutMs: Int64

    /// Prototype defaults. The feedback logic is fixed, while these numerical
    /// thresholds remain calibration parameters for user testing.
    public static let prototypeDefault = FeedbackDecisionConfiguration(
        triggerCESThreshold: 0.70,
        triggerSpikeThreshold: 0.45,
        sustainCESThreshold: 0.50,
        resetCESThreshold: 0.35,
        minimumFusionQuality: 0.60,
        belowSustainGraceMs: 1_200,
        minimumCooldownMs: 2_500,
        resetHoldMs: 1_500,
        updateIntervalMs: 250,
        emotionStreamTimeoutMs: 1_500
    )

    public init(
        triggerCESThreshold: Double,
        triggerSpikeThreshold: Double,
        sustainCESThreshold: Double,
        resetCESThreshold: Double,
        minimumFusionQuality: Double,
        belowSustainGraceMs: Int64,
        minimumCooldownMs: Int64,
        resetHoldMs: Int64,
        updateIntervalMs: Int64,
        emotionStreamTimeoutMs: Int64
    ) {
        precondition(triggerCESThreshold > sustainCESThreshold)
        precondition(sustainCESThreshold > resetCESThreshold)

        self.triggerCESThreshold = triggerCESThreshold
        self.triggerSpikeThreshold = triggerSpikeThreshold
        self.sustainCESThreshold = sustainCESThreshold
        self.resetCESThreshold = resetCESThreshold
        self.minimumFusionQuality = minimumFusionQuality
        self.belowSustainGraceMs = max(0, belowSustainGraceMs)
        self.minimumCooldownMs = max(0, minimumCooldownMs)
        self.resetHoldMs = max(0, resetHoldMs)
        self.updateIntervalMs = max(50, updateIntervalMs)
        self.emotionStreamTimeoutMs = max(250, emotionStreamTimeoutMs)
    }
}

@MainActor
public final class FeedbackDecisionEngine {
    public enum Phase: Equatable, Sendable {
        case idle
        case active
        case cooldown
    }

    public private(set) var phase: Phase = .idle

    private struct ActiveEvent {
        let eventID: UInt32
        let sourceDeviceID: UInt8
        let sourceSequence: UInt32
        let startedAtPlaybackUtcMs: Int64
        let onsetSpike: Double
        let maximumDurationMs: Int64

        var latestCES: Double
        var peakCES: Double
        var lastEmotionSampleUtcMs: Int64
        var lastAboveSustainUtcMs: Int64
        var lastUpdateSentUtcMs: Int64
        var suspended: Bool
    }

    private struct CooldownState {
        let startedAtPlaybackUtcMs: Int64
        var lowEmotionSinceUtcMs: Int64?
    }

    private let configuration: FeedbackDecisionConfiguration
    private let durationPolicy: FeedbackDurationPolicy
    private weak var commandSink: FeedbackCommandSink?

    private var activeEvent: ActiveEvent?
    private var cooldownState: CooldownState?

    public init(
        configuration: FeedbackDecisionConfiguration = .prototypeDefault,
        durationPolicy: FeedbackDurationPolicy = FeedbackDurationPolicy(),
        commandSink: FeedbackCommandSink
    ) {
        self.configuration = configuration
        self.durationPolicy = durationPolicy
        self.commandSink = commandSink
    }

    public func process(
        _ sample: CloudEmotionEvent,
        playbackUtcMs: Int64,
        matchContext: MatchContextSnapshot
    ) {
        switch phase {
        case .idle:
            attemptStart(
                sample,
                playbackUtcMs: playbackUtcMs,
                matchContext: matchContext
            )

        case .active:
            updateActive(
                sample,
                playbackUtcMs: playbackUtcMs
            )

        case .cooldown:
            updateCooldown(
                sample,
                playbackUtcMs: playbackUtcMs
            )
        }
    }

    public func handlePlaybackTick(playbackUtcMs: Int64) {
        guard var active = activeEvent, !active.suspended else {
            return
        }

        let elapsed = max(
            0,
            playbackUtcMs - active.startedAtPlaybackUtcMs
        )

        if elapsed >= active.maximumDurationMs {
            finishActive(
                active,
                playbackUtcMs: playbackUtcMs,
                reason: .maximumDurationReached
            )
            return
        }

        if playbackUtcMs - active.lastEmotionSampleUtcMs
            > configuration.emotionStreamTimeoutMs {
            active.latestCES = 0
            activeEvent = active
            finishActive(
                active,
                playbackUtcMs: playbackUtcMs,
                reason: .emotionStreamTimeout
            )
        }
    }

    /// Pausing the player stops physical output but retains the active event.
    /// On resume, the same event continues using playback time, not wall time.
    public func setPlaybackSuspended(
        _ suspended: Bool,
        playbackUtcMs: Int64
    ) {
        guard var active = activeEvent else { return }
        guard active.suspended != suspended else { return }

        active.suspended = suspended
        activeEvent = active

        if suspended {
            emitStop(
                active,
                playbackUtcMs: playbackUtcMs,
                reason: .playbackPaused
            )
        } else {
            let elapsed = max(
                0,
                playbackUtcMs - active.startedAtPlaybackUtcMs
            )

            if elapsed >= active.maximumDurationMs {
                finishActive(
                    active,
                    playbackUtcMs: playbackUtcMs,
                    reason: .maximumDurationReached
                )
            } else {
                emitStart(
                    active,
                    playbackUtcMs: playbackUtcMs,
                    isResume: true
                )
            }
        }
    }

    public func resetForTimelineChange(
        playbackUtcMs: Int64,
        reason: FeedbackStopReason = .playbackTimelineChanged
    ) {
        if let active = activeEvent {
            emitStop(
                active,
                playbackUtcMs: playbackUtcMs,
                reason: reason
            )
        }

        activeEvent = nil
        cooldownState = nil
        phase = .idle
    }

    public func stopManually(playbackUtcMs: Int64) {
        resetForTimelineChange(
            playbackUtcMs: playbackUtcMs,
            reason: .manualStop
        )
    }

    private func attemptStart(
        _ sample: CloudEmotionEvent,
        playbackUtcMs: Int64,
        matchContext: MatchContextSnapshot
    ) {
        guard sample.fusionQuality >= configuration.minimumFusionQuality else {
            return
        }

        guard
            sample.ces >= configuration.triggerCESThreshold,
            sample.spike >= configuration.triggerSpikeThreshold
        else {
            return
        }

        let maximumDuration = durationPolicy.maximumDurationMs(
            for: matchContext
        )

        let active = ActiveEvent(
            eventID: sample.sequence,
            sourceDeviceID: sample.deviceID,
            sourceSequence: sample.sequence,
            startedAtPlaybackUtcMs: sample.eventUtcMs,
            onsetSpike: sample.spike,
            maximumDurationMs: maximumDuration,
            latestCES: sample.ces,
            peakCES: sample.ces,
            lastEmotionSampleUtcMs: sample.eventUtcMs,
            lastAboveSustainUtcMs: sample.eventUtcMs,
            lastUpdateSentUtcMs: sample.eventUtcMs,
            suspended: false
        )

        activeEvent = active
        cooldownState = nil
        phase = .active

        emitStart(
            active,
            playbackUtcMs: playbackUtcMs,
            isResume: false
        )
    }

    private func updateActive(
        _ sample: CloudEmotionEvent,
        playbackUtcMs: Int64
    ) {
        guard var active = activeEvent else {
            phase = .idle
            return
        }

        active.latestCES = sample.ces
        active.peakCES = max(active.peakCES, sample.ces)
        active.lastEmotionSampleUtcMs = sample.eventUtcMs

        if
            sample.fusionQuality >= configuration.minimumFusionQuality,
            sample.ces >= configuration.sustainCESThreshold
        {
            active.lastAboveSustainUtcMs = sample.eventUtcMs
        }

        let elapsed = max(
            0,
            sample.eventUtcMs - active.startedAtPlaybackUtcMs
        )

        if elapsed >= active.maximumDurationMs {
            activeEvent = active
            finishActive(
                active,
                playbackUtcMs: playbackUtcMs,
                reason: .maximumDurationReached
            )
            return
        }

        let belowSustainDuration = max(
            0,
            sample.eventUtcMs - active.lastAboveSustainUtcMs
        )

        if belowSustainDuration > configuration.belowSustainGraceMs {
            activeEvent = active
            finishActive(
                active,
                playbackUtcMs: playbackUtcMs,
                reason: .emotionReturnedToBaseline
            )
            return
        }

        if
            !active.suspended,
            sample.eventUtcMs - active.lastUpdateSentUtcMs
                >= configuration.updateIntervalMs
        {
            active.lastUpdateSentUtcMs = sample.eventUtcMs
            activeEvent = active
            emitUpdate(active, playbackUtcMs: playbackUtcMs)
        } else {
            activeEvent = active
        }
    }

    private func updateCooldown(
        _ sample: CloudEmotionEvent,
        playbackUtcMs: Int64
    ) {
        guard var cooldown = cooldownState else {
            phase = .idle
            return
        }

        if sample.ces <= configuration.resetCESThreshold {
            if cooldown.lowEmotionSinceUtcMs == nil {
                cooldown.lowEmotionSinceUtcMs = sample.eventUtcMs
            }
        } else {
            cooldown.lowEmotionSinceUtcMs = nil
        }

        cooldownState = cooldown

        let minimumCooldownSatisfied =
            playbackUtcMs - cooldown.startedAtPlaybackUtcMs
                >= configuration.minimumCooldownMs

        let resetHoldSatisfied: Bool
        if let lowSince = cooldown.lowEmotionSinceUtcMs {
            resetHoldSatisfied = playbackUtcMs - lowSince
                >= configuration.resetHoldMs
        } else {
            resetHoldSatisfied = false
        }

        if minimumCooldownSatisfied && resetHoldSatisfied {
            cooldownState = nil
            phase = .idle
        }
    }

    private func finishActive(
        _ active: ActiveEvent,
        playbackUtcMs: Int64,
        reason: FeedbackStopReason
    ) {
        emitStop(
            active,
            playbackUtcMs: playbackUtcMs,
            reason: reason
        )

        activeEvent = nil
        cooldownState = CooldownState(
            startedAtPlaybackUtcMs: playbackUtcMs,
            lowEmotionSinceUtcMs: active.latestCES
                <= configuration.resetCESThreshold
                ? playbackUtcMs
                : nil
        )
        phase = .cooldown
    }

    private func emitStart(
        _ active: ActiveEvent,
        playbackUtcMs: Int64,
        isResume: Bool
    ) {
        commandSink?.send(
            command(
                kind: .startEvent,
                active: active,
                playbackUtcMs: playbackUtcMs,
                isResume: isResume,
                stopReason: nil
            )
        )
    }

    private func emitUpdate(
        _ active: ActiveEvent,
        playbackUtcMs: Int64
    ) {
        commandSink?.send(
            command(
                kind: .updateEvent,
                active: active,
                playbackUtcMs: playbackUtcMs,
                isResume: false,
                stopReason: nil
            )
        )
    }

    private func emitStop(
        _ active: ActiveEvent,
        playbackUtcMs: Int64,
        reason: FeedbackStopReason
    ) {
        commandSink?.send(
            command(
                kind: .stopEvent,
                active: active,
                playbackUtcMs: playbackUtcMs,
                isResume: false,
                stopReason: reason
            )
        )
    }

    private func command(
        kind: FeedbackCommandKind,
        active: ActiveEvent,
        playbackUtcMs: Int64,
        isResume: Bool,
        stopReason: FeedbackStopReason?
    ) -> FeedbackCommand {
        let elapsed = max(
            0,
            playbackUtcMs - active.startedAtPlaybackUtcMs
        )

        return FeedbackCommand(
            kind: kind,
            eventID: active.eventID,
            sourceDeviceID: active.sourceDeviceID,
            sourceSequence: active.sourceSequence,
            playbackUtcMs: playbackUtcMs,
            ces: active.latestCES,
            peakCES: active.peakCES,
            onsetSpike: active.onsetSpike,
            maximumEventDurationMs: active.maximumDurationMs,
            remainingEventDurationMs: max(
                0,
                active.maximumDurationMs - elapsed
            ),
            isResume: isResume,
            stopReason: stopReason
        )
    }
}
