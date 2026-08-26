import Foundation

struct CloudEmotionMessage: Codable, Sendable {
    let type: String
    let protocolVersion: UInt8
    let matchID: String
    let source: String

    let deviceID: UInt8
    let sequence: UInt32
    let eventUtcMs: Int64
    let receivedUtcMs: Int64
    let deviceTimestampMs: UInt64

    let ces: Double
    let spike: Double
    let audio: Double
    let heart: Double
    let motion: Double
    let bpm: UInt8
    let fusionQuality: Double
    let statusFlags: UInt8

    init(sample: EmotionSample, matchID: String) {
        let packet = sample.packet

        type = "emotion"
        protocolVersion = packet.version
        self.matchID = matchID
        source = "stadium-ios"

        deviceID = packet.deviceID
        sequence = packet.sequence
        eventUtcMs = Int64(
            (sample.eventDate.timeIntervalSince1970 * 1000.0).rounded()
        )
        receivedUtcMs = Int64(
            (sample.receivedAt.timeIntervalSince1970 * 1000.0).rounded()
        )
        deviceTimestampMs = sample.extendedDeviceTimestampMs

        ces = packet.ces
        spike = packet.spike
        audio = packet.audio
        heart = packet.heart
        motion = packet.motion
        bpm = packet.bpm
        fusionQuality = packet.quality
        statusFlags = packet.status.rawValue
    }
}

/// Real-time WebSocket transport from the stadium phone to the cloud relay.
final class EmotionCloudTransport: NSObject {
    enum State: Equatable {
        case notConfigured
        case disconnected
        case connecting
        case connected
        case failed(String)
    }

    var onStateChange: ((State) -> Void)?
    var onError: ((Error) -> Void)?

    private(set) var state: State {
        didSet { onStateChange?(state) }
    }

    private let endpoint: URL?
    private var session: URLSession!
    private var webSocketTask: URLSessionWebSocketTask?
    private var pingTimer: Timer?
    private var reconnectWorkItem: DispatchWorkItem?
    private var shouldReconnect = false

    init(endpoint: URL?) {
        self.endpoint = endpoint
        state = endpoint == nil ? .notConfigured : .disconnected
        super.init()

        let delegateQueue = OperationQueue.main
        delegateQueue.maxConcurrentOperationCount = 1

        session = URLSession(
            configuration: .default,
            delegate: self,
            delegateQueue: delegateQueue
        )
    }

    deinit {
        disconnect()
        session.invalidateAndCancel()
    }

    func connect() {
        guard let endpoint else {
            state = .notConfigured
            return
        }

        guard webSocketTask == nil else {
            return
        }

        shouldReconnect = true
        state = .connecting

        var request = URLRequest(url: endpoint)
        request.timeoutInterval = 15

        let task = session.webSocketTask(with: request)
        webSocketTask = task
        task.resume()
    }

    func disconnect() {
        shouldReconnect = false
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        stopPingTimer()

        webSocketTask?.cancel(with: .normalClosure, reason: nil)
        webSocketTask = nil
        state = endpoint == nil ? .notConfigured : .disconnected
    }

    func send(_ message: CloudEmotionMessage) {
        guard state == .connected, let task = webSocketTask else {
            return
        }

        do {
            let data = try JSONEncoder().encode(message)

            guard let text = String(data: data, encoding: .utf8) else {
                return
            }

            task.send(.string(text)) { [weak self] error in
                guard let error else { return }

                DispatchQueue.main.async {
                    self?.onError?(error)
                    self?.handleConnectionFailure(error.localizedDescription)
                }
            }
        } catch {
            onError?(error)
        }
    }

    private func receiveNextMessage() {
        webSocketTask?.receive { [weak self] result in
            DispatchQueue.main.async {
                guard let self else { return }

                switch result {
                case .success:
                    self.receiveNextMessage()
                case .failure(let error):
                    self.onError?(error)
                    self.handleConnectionFailure(error.localizedDescription)
                }
            }
        }
    }

    private func startPingTimer() {
        stopPingTimer()

        pingTimer = Timer.scheduledTimer(
            withTimeInterval: 20,
            repeats: true
        ) { [weak self] _ in
            self?.webSocketTask?.sendPing { error in
                guard let error else { return }

                DispatchQueue.main.async {
                    self?.onError?(error)
                    self?.handleConnectionFailure(error.localizedDescription)
                }
            }
        }
    }

    private func stopPingTimer() {
        pingTimer?.invalidate()
        pingTimer = nil
    }

    private func handleConnectionFailure(_ message: String) {
        state = .failed(message)
        stopPingTimer()

        webSocketTask?.cancel(with: .goingAway, reason: nil)
        webSocketTask = nil
        scheduleReconnect()
    }

    private func scheduleReconnect() {
        guard shouldReconnect, endpoint != nil else {
            return
        }

        reconnectWorkItem?.cancel()

        let item = DispatchWorkItem { [weak self] in
            guard let self, self.shouldReconnect else { return }
            self.connect()
        }

        reconnectWorkItem = item
        DispatchQueue.main.asyncAfter(deadline: .now() + 3, execute: item)
    }
}

extension EmotionCloudTransport: URLSessionWebSocketDelegate {
    func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didOpenWithProtocol protocol: String?
    ) {
        state = .connected
        startPingTimer()
        receiveNextMessage()
    }

    func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didCloseWith closeCode: URLSessionWebSocketTask.CloseCode,
        reason: Data?
    ) {
        self.webSocketTask = nil
        stopPingTimer()

        if shouldReconnect {
            state = .disconnected
            scheduleReconnect()
        } else {
            state = endpoint == nil ? .notConfigured : .disconnected
        }
    }
}
