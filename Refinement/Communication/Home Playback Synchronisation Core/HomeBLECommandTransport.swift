import Foundation

public enum HomeBLETransportState: Equatable, Sendable {
    case idle
    case bluetoothUnavailable(String)
    case scanning
    case connecting(String)
    case discoveringServices
    case ready
    case reconnecting
    case stopped
    case failed(String)
}

public struct HomeBLETransportConfiguration: Sendable {
    public let expectedDeviceNamePrefix: String
    public let acknowledgementTimeout: TimeInterval
    public let reconnectDelay: TimeInterval
    public let maximumQueuedCommandAge: TimeInterval
    public let maximumRetries: Int
    public let recoverUpdateEventMismatchAsResumeStart: Bool

    public static let prototypeDefault = HomeBLETransportConfiguration(
        expectedDeviceNamePrefix: HomeBLEUUIDs.deviceNamePrefix,
        acknowledgementTimeout: 0.70,
        reconnectDelay: 1.0,
        maximumQueuedCommandAge: 1.5,
        maximumRetries: 2,
        recoverUpdateEventMismatchAsResumeStart: true
    )

    public init(
        expectedDeviceNamePrefix: String,
        acknowledgementTimeout: TimeInterval,
        reconnectDelay: TimeInterval,
        maximumQueuedCommandAge: TimeInterval,
        maximumRetries: Int,
        recoverUpdateEventMismatchAsResumeStart: Bool
    ) {
        self.expectedDeviceNamePrefix = expectedDeviceNamePrefix
        self.acknowledgementTimeout = max(0.2, acknowledgementTimeout)
        self.reconnectDelay = max(0.1, reconnectDelay)
        self.maximumQueuedCommandAge = max(0.2, maximumQueuedCommandAge)
        self.maximumRetries = max(0, maximumRetries)
        self.recoverUpdateEventMismatchAsResumeStart =
            recoverUpdateEventMismatchAsResumeStart
    }
}

#if canImport(CoreBluetooth)
@preconcurrency import CoreBluetooth

/// CoreBluetooth transport connecting the Home App decision engine to the
/// Home-side ESP32. It performs no CES, Spike, vibration, LED or thermal
/// mapping. Its only job is binary encoding, reliable BLE delivery, ACK
/// handling and Home telemetry reception.
@MainActor
public final class HomeBLECommandTransport: NSObject, FeedbackCommandSink {
    public var onStateChange: ((HomeBLETransportState) -> Void)?
    public var onAcknowledgement: ((CommandAcknowledgement) -> Void)?
    public var onTelemetry: ((HomeTelemetry) -> Void)?
    public var onError: ((Error) -> Void)?

    public private(set) var state: HomeBLETransportState = .idle {
        didSet {
            guard oldValue != state else { return }
            onStateChange?(state)
        }
    }

    public private(set) var latestAcknowledgement: CommandAcknowledgement?
    public private(set) var latestTelemetry: HomeTelemetry?
    public private(set) var acceptedCommandCount = 0
    public private(set) var rejectedCommandCount = 0
    public private(set) var acknowledgementTimeoutCount = 0
    public private(set) var telemetryPacketCount = 0
    public private(set) var protocolErrorCount = 0

    public var isReady: Bool {
        state == .ready
    }

    private struct QueuedTransmission {
        let command: FeedbackCommand
        let commandSequence: UInt16
        let data: Data
        let enqueuedAt: Date
        var retryCount: Int
        var attemptedResumeRecovery: Bool
    }

    private let configuration: HomeBLETransportConfiguration
    private let commandEncoder = FeedbackCommandEncoder()
    private let maintenanceEncoder = HomeMaintenanceCommandEncoder()

    private lazy var centralManager = CBCentralManager(
        delegate: self,
        queue: nil,
        options: nil
    )

    private var peripheral: CBPeripheral?
    private var commandCharacteristic: CBCharacteristic?
    private var acknowledgementCharacteristic: CBCharacteristic?
    private var telemetryCharacteristic: CBCharacteristic?
    private var maintenanceCharacteristic: CBCharacteristic?

    private var acknowledgementNotificationsReady = false
    private var telemetryNotificationsReady = false
    private var wantsConnection = false

    private var queuedTransmissions = [QueuedTransmission]()
    private var inFlightTransmission: QueuedTransmission?
    private var acknowledgementTimeoutWorkItem: DispatchWorkItem?
    private var reconnectWorkItem: DispatchWorkItem?

    private var nextCommandSequence: UInt16 = 0
    private var nextMaintenanceRequestID: UInt16 = 0

    public init(
        configuration: HomeBLETransportConfiguration = .prototypeDefault
    ) {
        self.configuration = configuration
        super.init()
        _ = centralManager
    }

    deinit {
        acknowledgementTimeoutWorkItem?.cancel()
        reconnectWorkItem?.cancel()
    }

    /// Starts scanning and maintains an automatic connection to the Home-side
    /// ESP32. Safe to call more than once.
    public func start() {
        wantsConnection = true
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil

        switch centralManager.state {
        case .poweredOn:
            beginScanning()
        case .unknown, .resetting:
            state = .idle
        default:
            updateUnavailableState(centralManager.state)
        }
    }

    /// Stops scanning and disconnects. Pending commands are discarded because
    /// replaying old physical feedback after a deliberate shutdown is unsafe.
    public func stop() {
        wantsConnection = false
        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        acknowledgementTimeoutWorkItem?.cancel()
        acknowledgementTimeoutWorkItem = nil

        centralManager.stopScan()
        queuedTransmissions.removeAll()
        inFlightTransmission = nil

        if let peripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }

        clearPeripheralState()
        state = .stopped
    }

    /// `FeedbackDecisionEngine` calls this through `FeedbackCommandSink`.
    /// START and STOP commands remain ordered. Rapid UPDATE commands are
    /// coalesced so the newest CES and peak values supersede stale updates.
    public func send(_ command: FeedbackCommand) {
        do {
            let sequence = allocateCommandSequence()
            let data = try commandEncoder.encode(
                command,
                commandSequence: sequence
            )

            let transmission = QueuedTransmission(
                command: command,
                commandSequence: sequence,
                data: data,
                enqueuedAt: Date(),
                retryCount: 0,
                attemptedResumeRecovery: false
            )

            enqueue(transmission)
            pumpCommandQueue()
        } catch {
            report(error)
        }
    }

    /// Sends sensor recalibration, thermal-fault clearing or emergency-stop
    /// control to the Home-side ESP32. The current ESP32 maintenance protocol
    /// has no separate ACK packet, so CoreBluetooth's write response is used.
    public func sendMaintenance(_ command: HomeMaintenanceCommand) {
        guard
            state == .ready,
            let peripheral,
            let maintenanceCharacteristic
        else {
            report(HomeBLEProtocolError.transportNotReady)
            return
        }

        let requestID = allocateMaintenanceRequestID()
        let data = maintenanceEncoder.encode(
            command,
            requestID: requestID
        )

        peripheral.writeValue(
            data,
            for: maintenanceCharacteristic,
            type: .withResponse
        )
    }

    private func enqueue(_ transmission: QueuedTransmission) {
        switch transmission.command.kind {
        case .updateEvent:
            queuedTransmissions.removeAll {
                $0.command.kind == .updateEvent &&
                    $0.command.eventID == transmission.command.eventID
            }

        case .stopEvent:
            // Once STOP exists, unsent UPDATE packets for the same event have
            // no value and would only delay decay.
            queuedTransmissions.removeAll {
                $0.command.kind == .updateEvent &&
                    $0.command.eventID == transmission.command.eventID
            }

        case .startEvent:
            // A new START for the same event supersedes an older unsent START.
            queuedTransmissions.removeAll {
                $0.command.kind == .startEvent &&
                    $0.command.eventID == transmission.command.eventID
            }
        }

        queuedTransmissions.append(transmission)

        // Keep bounded memory during an extended BLE outage.
        if queuedTransmissions.count > 32 {
            queuedTransmissions.removeFirst(
                queuedTransmissions.count - 32
            )
        }
    }

    private func enqueueAtFront(_ transmission: QueuedTransmission) {
        queuedTransmissions.insert(transmission, at: 0)
    }

    private func pumpCommandQueue() {
        guard state == .ready else { return }
        guard inFlightTransmission == nil else { return }
        guard
            let peripheral,
            let commandCharacteristic
        else {
            report(HomeBLEProtocolError.transportNotReady)
            return
        }

        removeStaleQueuedCommands()
        guard !queuedTransmissions.isEmpty else { return }

        let transmission = queuedTransmissions.removeFirst()
        inFlightTransmission = transmission

        peripheral.writeValue(
            transmission.data,
            for: commandCharacteristic,
            type: .withResponse
        )

        scheduleAcknowledgementTimeout(
            for: transmission.commandSequence
        )
    }

    private func removeStaleQueuedCommands() {
        let now = Date()
        queuedTransmissions.removeAll { transmission in
            guard transmission.command.kind != .stopEvent else {
                return false
            }

            return now.timeIntervalSince(transmission.enqueuedAt)
                > configuration.maximumQueuedCommandAge
        }
    }

    private func scheduleAcknowledgementTimeout(
        for commandSequence: UInt16
    ) {
        acknowledgementTimeoutWorkItem?.cancel()

        let workItem = DispatchWorkItem { [weak self] in
            guard let self else { return }
            Task { @MainActor in
                self.handleAcknowledgementTimeout(
                    commandSequence: commandSequence
                )
            }
        }

        acknowledgementTimeoutWorkItem = workItem
        DispatchQueue.main.asyncAfter(
            deadline: .now() + configuration.acknowledgementTimeout,
            execute: workItem
        )
    }

    private func handleAcknowledgementTimeout(
        commandSequence: UInt16
    ) {
        guard
            let transmission = inFlightTransmission,
            transmission.commandSequence == commandSequence
        else {
            return
        }

        acknowledgementTimeoutCount += 1
        retryOrFail(
            transmission,
            error: HomeBLEProtocolError.acknowledgementTimeout(
                commandSequence: commandSequence
            )
        )
    }

    private func retryOrFail(
        _ transmission: QueuedTransmission,
        error: Error
    ) {
        acknowledgementTimeoutWorkItem?.cancel()
        acknowledgementTimeoutWorkItem = nil

        guard transmission.retryCount < configuration.maximumRetries else {
            inFlightTransmission = nil
            report(error)
            pumpCommandQueue()
            return
        }

        guard
            state == .ready,
            let peripheral,
            let commandCharacteristic
        else {
            inFlightTransmission = nil
            var queued = transmission
            queued.retryCount += 1
            enqueueAtFront(queued)
            return
        }

        var retry = transmission
        retry.retryCount += 1
        inFlightTransmission = retry

        // Retrying the same command sequence is deliberate. If the ESP32
        // accepted the first write but its ACK was lost, it responds DUPLICATE.
        peripheral.writeValue(
            retry.data,
            for: commandCharacteristic,
            type: .withResponse
        )
        scheduleAcknowledgementTimeout(
            for: retry.commandSequence
        )
    }

    private func handleAcknowledgement(
        _ acknowledgement: CommandAcknowledgement
    ) {
        latestAcknowledgement = acknowledgement
        onAcknowledgement?(acknowledgement)

        guard
            let transmission = inFlightTransmission,
            acknowledgement.commandSequence ==
                transmission.commandSequence,
            acknowledgement.eventID == transmission.command.eventID
        else {
            // It may be a delayed ACK from an already completed retry.
            return
        }

        acknowledgementTimeoutWorkItem?.cancel()
        acknowledgementTimeoutWorkItem = nil

        if acknowledgement.status.isAccepted {
            acceptedCommandCount += 1
            inFlightTransmission = nil
            pumpCommandQueue()
            return
        }

        if acknowledgement.status == .eventMismatch {
            if transmission.command.kind == .stopEvent {
                // The device has no matching active event, so the intended
                // physical result of STOP is already true.
                acceptedCommandCount += 1
                inFlightTransmission = nil
                pumpCommandQueue()
                return
            }

            if
                transmission.command.kind == .updateEvent,
                configuration.recoverUpdateEventMismatchAsResumeStart,
                !transmission.attemptedResumeRecovery
            {
                inFlightTransmission = nil
                enqueueResumeRecovery(from: transmission)
                pumpCommandQueue()
                return
            }
        }

        rejectedCommandCount += 1
        inFlightTransmission = nil
        report(
            HomeBLEProtocolError.commandRejected(
                status: acknowledgement.status
            )
        )
        pumpCommandQueue()
    }

    /// If BLE reconnects after the ESP32 has locally decayed to IDLE, its next
    /// UPDATE receives EVENT_MISMATCH. The transport can safely reconstruct a
    /// resume START because UPDATE already carries event ID, CES, peak, onset
    /// Spike and remaining duration.
    private func enqueueResumeRecovery(
        from transmission: QueuedTransmission
    ) {
        let command = transmission.command
        let recoveryCommand = FeedbackCommand(
            protocolVersion: command.protocolVersion,
            kind: .startEvent,
            eventID: command.eventID,
            sourceDeviceID: command.sourceDeviceID,
            sourceSequence: command.sourceSequence,
            playbackUtcMs: command.playbackUtcMs,
            ces: command.ces,
            peakCES: command.peakCES,
            onsetSpike: command.onsetSpike,
            maximumEventDurationMs: command.maximumEventDurationMs,
            remainingEventDurationMs: command.remainingEventDurationMs,
            isResume: true,
            stopReason: nil
        )

        do {
            let sequence = allocateCommandSequence()
            let data = try commandEncoder.encode(
                recoveryCommand,
                commandSequence: sequence
            )

            enqueueAtFront(
                QueuedTransmission(
                    command: recoveryCommand,
                    commandSequence: sequence,
                    data: data,
                    enqueuedAt: Date(),
                    retryCount: 0,
                    attemptedResumeRecovery: true
                )
            )
        } catch {
            report(error)
        }
    }

    private func allocateCommandSequence() -> UInt16 {
        let value = nextCommandSequence
        nextCommandSequence &+= 1
        return value
    }

    private func allocateMaintenanceRequestID() -> UInt16 {
        let value = nextMaintenanceRequestID
        nextMaintenanceRequestID &+= 1
        return value
    }

    private func beginScanning() {
        guard wantsConnection else { return }
        guard centralManager.state == .poweredOn else {
            updateUnavailableState(centralManager.state)
            return
        }

        if let peripheral,
           peripheral.state == .connected ||
            peripheral.state == .connecting {
            return
        }

        reconnectWorkItem?.cancel()
        reconnectWorkItem = nil
        centralManager.stopScan()
        state = .scanning

        centralManager.scanForPeripherals(
            withServices: [CBUUID(string: HomeBLEUUIDs.service)],
            options: [
                CBCentralManagerScanOptionAllowDuplicatesKey: false
            ]
        )
    }

    private func scheduleReconnect() {
        guard wantsConnection else { return }

        reconnectWorkItem?.cancel()
        state = .reconnecting

        let workItem = DispatchWorkItem { [weak self] in
            guard let self else { return }
            Task { @MainActor in
                self.beginScanning()
            }
        }

        reconnectWorkItem = workItem
        DispatchQueue.main.asyncAfter(
            deadline: .now() + configuration.reconnectDelay,
            execute: workItem
        )
    }

    private func evaluateReadiness() {
        guard
            peripheral?.state == .connected,
            commandCharacteristic != nil,
            acknowledgementCharacteristic != nil,
            telemetryCharacteristic != nil,
            maintenanceCharacteristic != nil,
            acknowledgementNotificationsReady,
            telemetryNotificationsReady
        else {
            return
        }

        state = .ready
        pumpCommandQueue()
    }

    private func clearPeripheralState() {
        peripheral?.delegate = nil
        peripheral = nil
        commandCharacteristic = nil
        acknowledgementCharacteristic = nil
        telemetryCharacteristic = nil
        maintenanceCharacteristic = nil
        acknowledgementNotificationsReady = false
        telemetryNotificationsReady = false
    }

    private func updateUnavailableState(_ state: CBManagerState) {
        let message: String
        switch state {
        case .poweredOff:
            message = "Bluetooth is turned off."
        case .unauthorized:
            message = "Bluetooth permission was denied."
        case .unsupported:
            message = "This device does not support Bluetooth Low Energy."
        case .resetting:
            message = "Bluetooth is resetting."
        case .unknown:
            message = "Bluetooth state is not known yet."
        case .poweredOn:
            return
        @unknown default:
            message = "Unknown Bluetooth state."
        }

        self.state = .bluetoothUnavailable(message)
    }

    private func report(_ error: Error) {
        onError?(error)
    }
}

extension HomeBLECommandTransport: @preconcurrency CBCentralManagerDelegate {
    public func centralManagerDidUpdateState(
        _ central: CBCentralManager
    ) {
        if central.state == .poweredOn {
            if wantsConnection {
                beginScanning()
            }
        } else {
            central.stopScan()
            updateUnavailableState(central.state)
        }
    }

    public func centralManager(
        _ central: CBCentralManager,
        didDiscover peripheral: CBPeripheral,
        advertisementData: [String: Any],
        rssi RSSI: NSNumber
    ) {
        let advertisedName =
            advertisementData[CBAdvertisementDataLocalNameKey] as? String
        let name = advertisedName ?? peripheral.name ?? ""

        guard
            configuration.expectedDeviceNamePrefix.isEmpty ||
            name.hasPrefix(configuration.expectedDeviceNamePrefix)
        else {
            return
        }

        central.stopScan()
        self.peripheral = peripheral
        peripheral.delegate = self
        state = .connecting(name.isEmpty ? "Home-side ESP32" : name)
        central.connect(peripheral, options: nil)
    }

    public func centralManager(
        _ central: CBCentralManager,
        didConnect peripheral: CBPeripheral
    ) {
        state = .discoveringServices
        peripheral.delegate = self
        peripheral.discoverServices([
            CBUUID(string: HomeBLEUUIDs.service)
        ])
    }

    public func centralManager(
        _ central: CBCentralManager,
        didFailToConnect peripheral: CBPeripheral,
        error: Error?
    ) {
        clearPeripheralState()
        if let error {
            report(HomeBLEProtocolError.writeFailed(error.localizedDescription))
        }
        scheduleReconnect()
    }

    public func centralManager(
        _ central: CBCentralManager,
        didDisconnectPeripheral peripheral: CBPeripheral,
        error: Error?
    ) {
        acknowledgementTimeoutWorkItem?.cancel()
        acknowledgementTimeoutWorkItem = nil

        if var inFlight = inFlightTransmission {
            inFlight.retryCount += 1
            enqueueAtFront(inFlight)
            inFlightTransmission = nil
        }

        clearPeripheralState()

        if let error {
            report(HomeBLEProtocolError.writeFailed(error.localizedDescription))
        }

        scheduleReconnect()
    }
}

extension HomeBLECommandTransport: @preconcurrency CBPeripheralDelegate {
    public func peripheral(
        _ peripheral: CBPeripheral,
        didDiscoverServices error: Error?
    ) {
        if let error {
            state = .failed(error.localizedDescription)
            report(error)
            return
        }

        guard let service = peripheral.services?.first(
            where: {
                $0.uuid == CBUUID(string: HomeBLEUUIDs.service)
            }
        ) else {
            let error = HomeBLEProtocolError.missingCharacteristic(
                HomeBLEUUIDs.service
            )
            state = .failed(error.localizedDescription)
            report(error)
            return
        }

        peripheral.discoverCharacteristics(
            [
                CBUUID(string: HomeBLEUUIDs.feedbackCommand),
                CBUUID(string: HomeBLEUUIDs.commandAcknowledgement),
                CBUUID(string: HomeBLEUUIDs.homeTelemetry),
                CBUUID(string: HomeBLEUUIDs.maintenanceControl)
            ],
            for: service
        )
    }

    public func peripheral(
        _ peripheral: CBPeripheral,
        didDiscoverCharacteristicsFor service: CBService,
        error: Error?
    ) {
        if let error {
            state = .failed(error.localizedDescription)
            report(error)
            return
        }

        for characteristic in service.characteristics ?? [] {
            switch characteristic.uuid.uuidString.lowercased() {
            case HomeBLEUUIDs.feedbackCommand.lowercased():
                commandCharacteristic = characteristic

            case HomeBLEUUIDs.commandAcknowledgement.lowercased():
                acknowledgementCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)

            case HomeBLEUUIDs.homeTelemetry.lowercased():
                telemetryCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)

            case HomeBLEUUIDs.maintenanceControl.lowercased():
                maintenanceCharacteristic = characteristic

            default:
                break
            }
        }

        let required: [(CBCharacteristic?, String)] = [
            (commandCharacteristic, HomeBLEUUIDs.feedbackCommand),
            (
                acknowledgementCharacteristic,
                HomeBLEUUIDs.commandAcknowledgement
            ),
            (telemetryCharacteristic, HomeBLEUUIDs.homeTelemetry),
            (maintenanceCharacteristic, HomeBLEUUIDs.maintenanceControl)
        ]

        for (characteristic, uuid) in required where characteristic == nil {
            let error = HomeBLEProtocolError.missingCharacteristic(uuid)
            state = .failed(error.localizedDescription)
            report(error)
            return
        }

        evaluateReadiness()
    }

    public func peripheral(
        _ peripheral: CBPeripheral,
        didUpdateNotificationStateFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        if let error {
            report(error)
            return
        }

        if characteristic.uuid == CBUUID(
            string: HomeBLEUUIDs.commandAcknowledgement
        ) {
            acknowledgementNotificationsReady = characteristic.isNotifying
        } else if characteristic.uuid == CBUUID(
            string: HomeBLEUUIDs.homeTelemetry
        ) {
            telemetryNotificationsReady = characteristic.isNotifying
        }

        evaluateReadiness()
    }

    public func peripheral(
        _ peripheral: CBPeripheral,
        didUpdateValueFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        if let error {
            report(error)
            return
        }

        guard let data = characteristic.value else { return }

        do {
            if characteristic.uuid == CBUUID(
                string: HomeBLEUUIDs.commandAcknowledgement
            ) {
                let acknowledgement = try CommandAcknowledgement.decode(data)
                handleAcknowledgement(acknowledgement)

            } else if characteristic.uuid == CBUUID(
                string: HomeBLEUUIDs.homeTelemetry
            ) {
                let telemetry = try HomeTelemetry.decode(data)
                latestTelemetry = telemetry
                telemetryPacketCount += 1
                onTelemetry?(telemetry)
            }
        } catch {
            protocolErrorCount += 1
            report(error)
        }
    }

    public func peripheral(
        _ peripheral: CBPeripheral,
        didWriteValueFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        guard let error else { return }

        if characteristic.uuid == CBUUID(
            string: HomeBLEUUIDs.feedbackCommand
        ), let transmission = inFlightTransmission {
            retryOrFail(
                transmission,
                error: HomeBLEProtocolError.writeFailed(
                    error.localizedDescription
                )
            )
        } else {
            report(
                HomeBLEProtocolError.writeFailed(
                    error.localizedDescription
                )
            )
        }
    }
}

#else

/// Build-time fallback for non-Apple environments. The actual transport is
/// available on iOS where CoreBluetooth exists.
@MainActor
public final class HomeBLECommandTransport: FeedbackCommandSink {
    public var onStateChange: ((HomeBLETransportState) -> Void)?
    public var onAcknowledgement: ((CommandAcknowledgement) -> Void)?
    public var onTelemetry: ((HomeTelemetry) -> Void)?
    public var onError: ((Error) -> Void)?

    public private(set) var state: HomeBLETransportState = .idle {
        didSet { onStateChange?(state) }
    }

    public let configuration: HomeBLETransportConfiguration

    public var isReady: Bool { false }

    public init(
        configuration: HomeBLETransportConfiguration = .prototypeDefault
    ) {
        self.configuration = configuration
    }

    public func start() {
        let error = HomeBLEProtocolError.bluetoothUnavailable(
            "CoreBluetooth is available only on Apple platforms."
        )
        state = .bluetoothUnavailable(error.localizedDescription)
        onError?(error)
    }

    public func stop() {
        state = .stopped
    }

    public func send(_ command: FeedbackCommand) {
        _ = command
        onError?(HomeBLEProtocolError.transportNotReady)
    }

    public func sendMaintenance(_ command: HomeMaintenanceCommand) {
        _ = command
        onError?(HomeBLEProtocolError.transportNotReady)
    }
}

#endif
