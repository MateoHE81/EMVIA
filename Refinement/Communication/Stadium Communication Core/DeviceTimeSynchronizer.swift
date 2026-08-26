import Foundation

/// Converts the ESP32 monotonic millis() timeline into phone UTC time.
final class DeviceTimeSynchronizer {
    private struct PendingRequest {
        let phoneSendMonotonicMs: Double
    }

    private struct SyncSample {
        let roundTripTimeMs: Double
        let phoneMinusDeviceOffsetMs: Double
        let utcMinusPhoneMonotonicMs: Double
    }

    private var pendingRequests: [UInt16: PendingRequest] = [:]
    private var samples: [SyncSample] = []
    private var bestSample: SyncSample?

    var isSynchronized: Bool {
        bestSample != nil
    }

    func reset() {
        pendingRequests.removeAll()
        samples.removeAll()
        bestSample = nil
    }

    func registerRequest(
        requestID: UInt16,
        phoneSendMonotonicMs: Double
    ) {
        pendingRequests[requestID] = PendingRequest(
            phoneSendMonotonicMs: phoneSendMonotonicMs
        )
    }

    func complete(
        response: TimeSyncResponsePacket,
        phoneReceiveMonotonicMs: Double,
        phoneReceiveUTCms: Double,
        timestampUnwrapper: inout DeviceTimestampUnwrapper
    ) throws {
        guard let pending = pendingRequests.removeValue(forKey: response.requestID) else {
            throw StadiumProtocolError.unknownTimeSyncRequest(response.requestID)
        }

        let t0 = pending.phoneSendMonotonicMs
        let t1 = Double(timestampUnwrapper.unwrap(response.receiveTimestampMs))
        let t2 = Double(timestampUnwrapper.unwrap(response.transmitTimestampMs))
        let t3 = phoneReceiveMonotonicMs

        let deviceProcessingMs = max(0, t2 - t1)
        let roundTripTimeMs = max(0, (t3 - t0) - deviceProcessingMs)

        // NTP-style midpoint estimate of phone monotonic time minus ESP32 time.
        let phoneMinusDeviceOffsetMs = (
            (t0 - t1) +
            (t3 - t2)
        ) / 2.0

        let sample = SyncSample(
            roundTripTimeMs: roundTripTimeMs,
            phoneMinusDeviceOffsetMs: phoneMinusDeviceOffsetMs,
            utcMinusPhoneMonotonicMs: phoneReceiveUTCms - t3
        )

        samples.append(sample)

        if samples.count > 8 {
            samples.removeFirst(samples.count - 8)
        }

        // The lowest-RTT sample is normally least distorted by BLE queueing.
        bestSample = samples.min {
            $0.roundTripTimeMs < $1.roundTripTimeMs
        }
    }

    func date(forExtendedDeviceTimestampMs timestampMs: UInt64) -> Date? {
        guard let sample = bestSample else {
            return nil
        }

        let phoneMonotonicMs =
            Double(timestampMs) + sample.phoneMinusDeviceOffsetMs

        let utcMs =
            phoneMonotonicMs + sample.utcMinusPhoneMonotonicMs

        return Date(timeIntervalSince1970: utcMs / 1000.0)
    }
}
