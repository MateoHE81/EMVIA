import Foundation

/// Ordered, de-duplicated buffer keyed by the stadium event UTC timestamp.
public final class EmotionTimelineBuffer {
    private var events: [CloudEmotionEvent] = []
    private var identities: Set<CloudEmotionEvent.Identity> = []
    private let maximumEventCount: Int

    public init(maximumEventCount: Int = 12_000) {
        self.maximumEventCount = max(100, maximumEventCount)
    }

    public var count: Int { events.count }

    @discardableResult
    public func insert(_ event: CloudEmotionEvent) -> Bool {
        guard identities.insert(event.identity).inserted else {
            return false
        }

        let insertionIndex = events.partitioningIndex {
            if $0.eventUtcMs == event.eventUtcMs {
                return $0.sequence >= event.sequence
            }
            return $0.eventUtcMs >= event.eventUtcMs
        }

        events.insert(event, at: insertionIndex)

        if events.count > maximumEventCount {
            let excess = events.count - maximumEventCount
            let removed = events.prefix(excess)
            for event in removed {
                identities.remove(event.identity)
            }
            events.removeFirst(excess)
        }

        return true
    }

    public func drain(upToUtcMs utcMs: Int64) -> [CloudEmotionEvent] {
        guard !events.isEmpty else { return [] }

        let endIndex = events.partitioningIndex {
            $0.eventUtcMs > utcMs
        }

        guard endIndex > 0 else { return [] }

        let drained = Array(events[..<endIndex])
        events.removeSubrange(..<endIndex)

        for event in drained {
            identities.remove(event.identity)
        }

        return drained
    }

    public func removeEvents(olderThanUtcMs utcMs: Int64) {
        let endIndex = events.partitioningIndex {
            $0.eventUtcMs >= utcMs
        }

        guard endIndex > 0 else { return }

        let removed = events[..<endIndex]
        for event in removed {
            identities.remove(event.identity)
        }
        events.removeSubrange(..<endIndex)
    }

    public func removeAll() {
        events.removeAll(keepingCapacity: true)
        identities.removeAll(keepingCapacity: true)
    }
}

private extension Array {
    /// First index where predicate is true, assuming all false values precede
    /// all true values.
    func partitioningIndex(
        where predicate: (Element) -> Bool
    ) -> Int {
        var low = 0
        var high = count

        while low < high {
            let middle = low + (high - low) / 2
            if predicate(self[middle]) {
                high = middle
            } else {
                low = middle + 1
            }
        }

        return low
    }
}
