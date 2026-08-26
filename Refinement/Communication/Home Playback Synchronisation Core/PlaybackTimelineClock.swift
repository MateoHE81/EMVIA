import Foundation

public enum PlaybackDateSource: String, Sendable {
    case hlsProgramDateTime
    case manualAnchor
    case unavailable
}

public struct PlaybackSnapshot: Sendable {
    public let contentDate: Date?
    public let mediaTimeSeconds: TimeInterval?
    public let isPlaying: Bool
    public let isNearLiveEdge: Bool
    public let dateSource: PlaybackDateSource

    public init(
        contentDate: Date?,
        mediaTimeSeconds: TimeInterval?,
        isPlaying: Bool,
        isNearLiveEdge: Bool,
        dateSource: PlaybackDateSource
    ) {
        self.contentDate = contentDate
        self.mediaTimeSeconds = mediaTimeSeconds
        self.isPlaying = isPlaying
        self.isNearLiveEdge = isNearLiveEdge
        self.dateSource = dateSource
    }
}

@MainActor
public protocol PlaybackTimelineProviding: AnyObject {
    var onTick: ((PlaybackSnapshot) -> Void)? { get set }
    var onTimeJump: ((PlaybackSnapshot) -> Void)? { get set }

    func start()
    func stop()
    func snapshot() -> PlaybackSnapshot
}

#if canImport(AVFoundation)
import AVFoundation

/// Maps AVPlayer playback to the UTC date represented by the live HLS stream.
/// AVPlayerItem.currentDate() is preferred. A manual anchor is available for
/// streams that do not expose a program-date-time mapping.
@MainActor
public final class AVPlayerPlaybackTimeline: PlaybackTimelineProviding {
    public var onTick: ((PlaybackSnapshot) -> Void)?
    public var onTimeJump: ((PlaybackSnapshot) -> Void)?

    private struct ManualAnchor {
        let mediaTimeSeconds: TimeInterval
        let utcDate: Date
    }

    private let player: AVPlayer
    private let tickInterval: TimeInterval
    private let liveEdgeToleranceSeconds: TimeInterval

    private var manualAnchor: ManualAnchor?
    private var periodicObserver: Any?
    private var timeControlObservation: NSKeyValueObservation?
    private var currentItemObservation: NSKeyValueObservation?
    private var timeJumpObserver: NSObjectProtocol?

    public init(
        player: AVPlayer,
        tickInterval: TimeInterval = 0.10,
        liveEdgeToleranceSeconds: TimeInterval = 3.0
    ) {
        self.player = player
        self.tickInterval = max(0.05, tickInterval)
        self.liveEdgeToleranceSeconds = max(
            0.5,
            liveEdgeToleranceSeconds
        )
    }

    deinit {
        if let periodicObserver {
            player.removeTimeObserver(periodicObserver)
        }
        if let timeJumpObserver {
            NotificationCenter.default.removeObserver(timeJumpObserver)
        }
    }

    public func start() {
        guard periodicObserver == nil else { return }

        let interval = CMTime(
            seconds: tickInterval,
            preferredTimescale: 1_000
        )

        periodicObserver = player.addPeriodicTimeObserver(
            forInterval: interval,
            queue: .main
        ) { [weak self] _ in
            guard let self else { return }
            self.onTick?(self.snapshot())
        }

        timeControlObservation = player.observe(
            \.timeControlStatus,
            options: [.new]
        ) { [weak self] _, _ in
            Task { @MainActor in
                guard let self else { return }
                self.onTick?(self.snapshot())
            }
        }

        currentItemObservation = player.observe(
            \.currentItem,
            options: [.new]
        ) { [weak self] _, _ in
            Task { @MainActor in
                guard let self else { return }
                self.installTimeJumpObserver()
                self.onTimeJump?(self.snapshot())
            }
        }

        installTimeJumpObserver()
        onTick?(snapshot())
    }

    public func stop() {
        if let periodicObserver {
            player.removeTimeObserver(periodicObserver)
            self.periodicObserver = nil
        }

        timeControlObservation = nil
        currentItemObservation = nil

        if let timeJumpObserver {
            NotificationCenter.default.removeObserver(timeJumpObserver)
            self.timeJumpObserver = nil
        }
    }

    public func setManualUTCAnchor(
        mediaTimeSeconds: TimeInterval,
        utcDate: Date
    ) {
        manualAnchor = ManualAnchor(
            mediaTimeSeconds: mediaTimeSeconds,
            utcDate: utcDate
        )
    }

    public func clearManualUTCAnchor() {
        manualAnchor = nil
    }

    public func snapshot() -> PlaybackSnapshot {
        guard let item = player.currentItem else {
            return PlaybackSnapshot(
                contentDate: nil,
                mediaTimeSeconds: nil,
                isPlaying: false,
                isNearLiveEdge: false,
                dateSource: .unavailable
            )
        }

        let mediaSeconds = Self.finiteSeconds(item.currentTime())

        let contentDate: Date?
        let dateSource: PlaybackDateSource

        if let hlsDate = item.currentDate() {
            contentDate = hlsDate
            dateSource = .hlsProgramDateTime
        } else if let manualAnchor, let mediaSeconds {
            contentDate = manualAnchor.utcDate.addingTimeInterval(
                mediaSeconds - manualAnchor.mediaTimeSeconds
            )
            dateSource = .manualAnchor
        } else {
            contentDate = nil
            dateSource = .unavailable
        }

        return PlaybackSnapshot(
            contentDate: contentDate,
            mediaTimeSeconds: mediaSeconds,
            isPlaying: player.timeControlStatus == .playing && player.rate != 0,
            isNearLiveEdge: isNearLiveEdge(
                item: item,
                currentMediaTimeSeconds: mediaSeconds
            ),
            dateSource: dateSource
        )
    }

    private func installTimeJumpObserver() {
        if let timeJumpObserver {
            NotificationCenter.default.removeObserver(timeJumpObserver)
        }

        guard let item = player.currentItem else {
            timeJumpObserver = nil
            return
        }

        timeJumpObserver = NotificationCenter.default.addObserver(
            forName: .AVPlayerItemTimeJumped,
            object: item,
            queue: .main
        ) { [weak self] _ in
            Task { @MainActor in
                guard let self else { return }
                self.onTimeJump?(self.snapshot())
            }
        }
    }

    private func isNearLiveEdge(
        item: AVPlayerItem,
        currentMediaTimeSeconds: TimeInterval?
    ) -> Bool {
        guard
            let currentMediaTimeSeconds,
            let lastRange = item.seekableTimeRanges.last?.timeRangeValue,
            let liveEdgeSeconds = Self.finiteSeconds(
                CMTimeRangeGetEnd(lastRange)
            )
        else {
            // Some streams do not expose a seekable range. Do not disable
            // feedback solely because this optional signal is unavailable.
            return true
        }

        return max(0, liveEdgeSeconds - currentMediaTimeSeconds)
            <= liveEdgeToleranceSeconds
    }

    private static func finiteSeconds(_ time: CMTime) -> TimeInterval? {
        let seconds = CMTimeGetSeconds(time)
        return seconds.isFinite ? seconds : nil
    }
}
#endif
