import Foundation
#if canImport(FoundationNetworking)
import FoundationNetworking
#endif

/// Receives the timestamped Stadium emotion stream from the Cloud Relay.
/// This class contains no UI and performs no feedback mapping.
@MainActor
public final class CloudEmotionStreamReceiver: NSObject {
    public enum State: Equatable, Sendable {
        case disconnected
        case connecting
        case connected
        case subscribed
        case failed(String)
    }

    public var onEvent: ((CloudEmotionEvent) -> Void)?
    public var onStateChange: ((State) -> Void)?
    public var onError: ((Error) -> Void)?

    public private(set) var state: State = .disconnected {
        didSet { onStateChange?(state) }
    }

    public private(set) var lastReceivedEventUtcMs: Int64?

    private let endpoint: URL
    private let matchID: String
    private let protocolVersion: Int
    private let bearerToken: String?

    private var session: URLSession!
    private var webSocketTask: URLSessionWebSocketTask?
    private var pingTimer: Timer?
    private var reconnectWorkItem: DispatchWorkItem?
    private var shouldReconnect = false

    public init(
        endpoint: URL,
        matchID: String,
        protocolVersion: Int = 1,
        bearerToken: String? = nil
    ) {
        self.endpoint = endpoint
        self.matchID = matchID
        self.protocolVersion = protocolVersion
        self.bearerToken = bearerToken
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
        pingTimer?.invalidate()
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        session.invalidateAndCancel()
    }

    public func connect() {
        guard webSocketTask == nil else { return }

        shouldReconnect = true
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        state = .connecting

        var request = URLRequest(url: endpoint)
        request.timeoutInterval = 15

        if let bearerToken, !bearerToken.isEmpty {
            request.setValue(
                "Bearer \(bearerToken)",
                forHTTPHeaderField: "Authorization"
            )
        }

        let task = session.webSocketTask(with: request)
        webSocketTask = task
        task.resume()
    }

    public func disconnect() {
        shouldReconnect = false
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        stopPingTimer()

        if state == .subscribed {
            sendEncodable(
                RelayUnsubscribeMessage(
                    protocolVersion: protocolVersion
                )
            )
        }

        webSocketTask?.cancel(with: .normalClosure, reason: nil)
        webSocketTask = nil
        state = .disconnected
    }

    private func sendSubscription() {
        sendEncodable(
            RelaySubscriptionMessage(
                protocolVersion: protocolVersion,
                matchID: matchID,
                resumeAfterEventUtcMs: lastReceivedEventUtcMs
            )
        )
    }

    private func sendEncodable<T: Encodable>(_ message: T) {
        guard let task = webSocketTask else { return }

        do {
            let data = try JSONEncoder().encode(message)
            guard let text = String(data: data, encoding: .utf8) else {
                throw HomeSyncProtocolError.malformedServerMessage
            }

            task.send(.string(text)) { [weak self] error in
                guard let error else { return }
                let failure = TransportFailure(error.localizedDescription)
                Task { @MainActor [weak self] in
                    self?.handleFailure(failure)
                }
            }
        } catch {
            onError?(error)
        }
    }

    private func receiveNextMessage() {
        webSocketTask?.receive { [weak self] result in
            switch result {
            case .success(let message):
                Task { @MainActor [weak self] in
                    guard let self else { return }
                    self.handle(message)
                    self.receiveNextMessage()
                }

            case .failure(let error):
                let failure = TransportFailure(error.localizedDescription)
                Task { @MainActor [weak self] in
                    self?.handleFailure(failure)
                }
            }
        }
    }

    private func handle(_ message: URLSessionWebSocketTask.Message) {
        do {
            let data: Data

            switch message {
            case .string(let text):
                guard let encoded = text.data(using: .utf8) else {
                    throw HomeSyncProtocolError.malformedServerMessage
                }
                data = encoded

            case .data(let binary):
                data = binary

            @unknown default:
                throw HomeSyncProtocolError.malformedServerMessage
            }

            let envelope = try JSONDecoder().decode(
                RelayMessageEnvelope.self,
                from: data
            )

            switch envelope.type {
            case "emotion":
                let event = try JSONDecoder()
                    .decode(CloudEmotionEvent.self, from: data)
                    .validated(
                        expectedProtocolVersion: protocolVersion,
                        expectedMatchID: matchID
                    )

                lastReceivedEventUtcMs = max(
                    lastReceivedEventUtcMs ?? 0,
                    event.eventUtcMs
                )
                onEvent?(event)

            case "hello":
                let hello = try JSONDecoder().decode(
                    RelayHelloMessage.self,
                    from: data
                )
                guard hello.protocolVersion == protocolVersion else {
                    throw HomeSyncProtocolError.unsupportedProtocolVersion(
                        hello.protocolVersion
                    )
                }

            case "subscribed":
                let subscribed = try JSONDecoder().decode(
                    RelaySubscribedMessage.self,
                    from: data
                )

                guard subscribed.protocolVersion == protocolVersion else {
                    throw HomeSyncProtocolError.unsupportedProtocolVersion(
                        subscribed.protocolVersion
                    )
                }

                guard subscribed.matchID == matchID else {
                    throw HomeSyncProtocolError.unexpectedMatchID(
                        expected: matchID,
                        actual: subscribed.matchID
                    )
                }

                state = .subscribed

            case "error":
                let serverError = try JSONDecoder().decode(
                    RelayErrorMessage.self,
                    from: data
                )
                throw HomeSyncProtocolError.serverError(
                    code: serverError.code,
                    message: serverError.message
                )

            case "unsubscribed", "publisherReady":
                break

            default:
                throw HomeSyncProtocolError.invalidMessageType(
                    envelope.type
                )
            }
        } catch {
            onError?(error)
        }
    }

    private func startPingTimer() {
        stopPingTimer()

        pingTimer = Timer.scheduledTimer(
            withTimeInterval: 20,
            repeats: true
        ) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self, let task = self.webSocketTask else { return }

                task.sendPing { [weak self] error in
                    guard let error else { return }
                    let failure = TransportFailure(error.localizedDescription)
                    Task { @MainActor [weak self] in
                        self?.handleFailure(failure)
                    }
                }
            }
        }
    }

    private func stopPingTimer() {
        pingTimer?.invalidate()
        pingTimer = nil
    }

    private func handleFailure(_ error: Error) {
        onError?(error)
        state = .failed(error.localizedDescription)
        stopPingTimer()

        webSocketTask?.cancel(with: .goingAway, reason: nil)
        webSocketTask = nil
        scheduleReconnect()
    }

    private func scheduleReconnect() {
        guard shouldReconnect else { return }

        reconnectWorkItem?.cancel()

        let item = DispatchWorkItem { [weak self] in
            guard let self, self.shouldReconnect else { return }
            self.connect()
        }

        reconnectWorkItem = item
        DispatchQueue.main.asyncAfter(
            deadline: .now() + 3,
            execute: item
        )
    }
}

private struct TransportFailure: Error, LocalizedError, Sendable {
    let message: String

    init(_ message: String) {
        self.message = message
    }

    var errorDescription: String? { message }
}

extension CloudEmotionStreamReceiver: URLSessionWebSocketDelegate {
    nonisolated public func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didOpenWithProtocol protocol: String?
    ) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.state = .connected
            self.startPingTimer()
            self.receiveNextMessage()
            self.sendSubscription()
        }
    }

    nonisolated public func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didCloseWith closeCode: URLSessionWebSocketTask.CloseCode,
        reason: Data?
    ) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.webSocketTask = nil
            self.stopPingTimer()

            if self.shouldReconnect {
                self.state = .disconnected
                self.scheduleReconnect()
            } else {
                self.state = .disconnected
            }
        }
    }
}
