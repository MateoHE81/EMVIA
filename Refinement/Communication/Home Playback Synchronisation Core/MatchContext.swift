import Foundation

public struct MatchContextSnapshot: Sendable {
    public let matchMinute: Double
    public let isFinal: Bool
    public let isCriticalFinalStoppageMoment: Bool

    public init(
        matchMinute: Double,
        isFinal: Bool = false,
        isCriticalFinalStoppageMoment: Bool = false
    ) {
        self.matchMinute = max(0, matchMinute)
        self.isFinal = isFinal
        self.isCriticalFinalStoppageMoment =
            isFinal && isCriticalFinalStoppageMoment
    }
}

@MainActor
public protocol MatchContextProviding: AnyObject {
    func context(atPlaybackDate date: Date) -> MatchContextSnapshot
}

/// A simple provider that can later be updated by the match-data layer.
/// The synchronisation core does not infer semantic events such as goals.
@MainActor
public final class MutableMatchContextProvider: MatchContextProviding {
    public var current: MatchContextSnapshot

    public init(
        current: MatchContextSnapshot = MatchContextSnapshot(
            matchMinute: 0
        )
    ) {
        self.current = current
    }

    public func context(
        atPlaybackDate date: Date
    ) -> MatchContextSnapshot {
        current
    }
}

public struct FeedbackDurationPolicy: Sendable {
    public init() {}

    /// Upper limits derived from the user's selected goal-celebration timing
    /// references. Short peaks end earlier when CES falls below sustain level.
    public func maximumDurationMs(
        for context: MatchContextSnapshot
    ) -> Int64 {
        if context.isCriticalFinalStoppageMoment {
            return 67_500
        }

        switch context.matchMinute {
        case ..<15:
            return 27_200
        case ..<45:
            return 30_000
        case ..<75:
            return 30_500
        case ..<90:
            return 30_500
        default:
            return 45_200
        }
    }
}
