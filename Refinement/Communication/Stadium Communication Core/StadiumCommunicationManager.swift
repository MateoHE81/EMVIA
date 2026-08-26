import Foundation
import CoreBluetooth

/// Configuration for the stadium-side communication chain.
struct StadiumCommunicationConfiguration {
    let expectedDeviceNamePrefix: String
    let matchID: String
    let cloudWebSocketURL: URL?

    var initialTimeSyncSampleCount = 5
    var initialTimeSyncInterval: TimeInterval = 0.25
    var periodicTimeSyncInterval: TimeInterval = 60

    static let serviceUUID = CBUUID(
        string: "7d2f0001-7f30-4e7f-a9e4-8f5e8a3d1c01"
    )

    static let emotionUUID = CBUUID(
        string: "7d2f0002-7f30-4e7f-a9e4-8f5e8a3d1c01"
    )

    static let controlUUID = CBUUID(
        string: "7d2f0003-7f30-4e7f-a9e4-8f5e8a3d1c01"
    )

    static let timeSyncUUID = CBUUID(
        string: "7d2f0004-7f30-4e7f-a9e4-8f5e8a3d1c01"
    )
}

/// BLE reception, packet verification, time conversion and cloud forwarding.
/// This class contains no SwiftUI or view-layer code.
final class StadiumCommunicationManager: NSObject {
    enum State: Equatable {
        case idle
        case bluetoothUnavailable
        case scanning
        case connecting
        case discoveringServices
        case synchronizingTime
        case streaming
        case failed(String)
    }

    struct PacketStatistics: Equatable {
        var valid: UInt64 = 0
        var lost: UInt64 = 0
        var duplicate: UInt64 = 0
        var outOfOrder: UInt64 = 0
        var rejected: UInt64 = 0
    }

    var onStateChange: ((State) -> Void)?
    var onEmotionSample: ((EmotionSample) -> Void)?
    var onStatisticsChange: ((PacketStatistics) -> Void)?
    var onError: ((Error) -> Void)?

    private(set) var state: State = .idle {
        didSet { onStateChange?(state) }
    }

    private(set) var statistics = PacketStatistics() {
        didSet { onStatisticsChange?(statistics) }
    }

    private let configuration: StadiumCommunicationConfiguration
    private let cloudTransport: EmotionCloudTransport

    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var emotionCharacteristic: CBCharacteristic?
    private var controlCharacteristic: CBCharacteristic?
    private var timeSyncCharacteristic: CBCharacteristic?

    private var emotionNotificationsReady = false
    private var timeSyncNotificationsReady = false
    private var sessionStarted = false
    private var shouldReconnect = true

    private var nextRequestID: UInt16 = 1
    private var lastSequence: UInt32?
    private var lastRawTimestamp: UInt32?

    private var timestampUnwrapper = DeviceTimestampUnwrapper()
    private let timeSynchronizer = DeviceTimeSynchronizer()
    private var periodicSyncTimer: Timer?

    init(configuration: StadiumCommunicationConfiguration) {
        self.configuration = configuration
        cloudTransport = EmotionCloudTransport(
            endpoint: configuration.cloudWebSocketURL
        )

        super.init()

        centralManager = CBCentralManager(
            delegate: self,
            queue: nil
        )
    }

    deinit {
        stop()
    }

    func start() {
        shouldReconnect = true
        cloudTransport.connect()

        if centralManager.state == .poweredOn {
            scan()
        }
    }

    func stop() {
        shouldReconnect = false
        periodicSyncTimer?.invalidate()
        periodicSyncTimer = nil
        centralManager.stopScan()
        cloudTransport.disconnect()

        if let peripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }

        state = .idle
    }

    func restartSensorCalibration() {
        sendControlCommand(.recalibrate)
    }

    func requestTimeSynchronization() {
        sendControlCommand(.timeSync)
    }

    private func scan() {
        guard centralManager.state == .poweredOn else {
            state = .bluetoothUnavailable
            return
        }

        resetSession()
        state = .scanning

        centralManager.scanForPeripherals(
            withServices: [StadiumCommunicationConfiguration.serviceUUID],
            options: [CBCentralManagerScanOptionAllowDuplicatesKey: false]
        )
    }

    private func resetSession() {
        periodicSyncTimer?.invalidate()
        periodicSyncTimer = nil

        emotionCharacteristic = nil
        controlCharacteristic = nil
        timeSyncCharacteristic = nil
        emotionNotificationsReady = false
        timeSyncNotificationsReady = false
        sessionStarted = false
        lastSequence = nil
        lastRawTimestamp = nil

        timestampUnwrapper.reset()
        timeSynchronizer.reset()
    }

    private func beginStreamingIfReady() {
        guard
            !sessionStarted,
            emotionNotificationsReady,
            timeSyncNotificationsReady
        else {
            return
        }

        sessionStarted = true
        state = .synchronizingTime
        performInitialTimeSyncBurst()

        periodicSyncTimer = Timer.scheduledTimer(
            withTimeInterval: configuration.periodicTimeSyncInterval,
            repeats: true
        ) { [weak self] _ in
            self?.requestTimeSynchronization()
        }
    }

    private func performInitialTimeSyncBurst() {
        for index in 0..<configuration.initialTimeSyncSampleCount {
            DispatchQueue.main.asyncAfter(
                deadline: .now() +
                Double(index) * configuration.initialTimeSyncInterval
            ) { [weak self] in
                self?.requestTimeSynchronization()
            }
        }
    }

    private func sendControlCommand(_ command: ControlCommand) {
        guard
            let peripheral,
            let controlCharacteristic,
            peripheral.state == .connected
        else {
            return
        }

        let requestID = makeRequestID()

        if command == .timeSync {
            timeSynchronizer.registerRequest(
                requestID: requestID,
                phoneSendMonotonicMs: Self.monotonicMilliseconds()
            )
        }

        peripheral.writeValue(
            command.encode(requestID: requestID),
            for: controlCharacteristic,
            type: .withResponse
        )
    }

    private func makeRequestID() -> UInt16 {
        let current = nextRequestID
        nextRequestID &+= 1

        if nextRequestID == 0 {
            nextRequestID = 1
        }

        return current
    }

    private func handleEmotionData(_ data: Data) {
        do {
            let packet = try EmotionPacket.decode(data)
            detectDeviceRestart(packet)
            updateSequenceStatistics(packet.sequence)

            let extendedTimestamp = timestampUnwrapper.unwrap(packet.timestampMs)
            let receivedAt = Date()

            guard let eventDate = timeSynchronizer.date(
                forExtendedDeviceTimestampMs: extendedTimestamp
            ) else {
                // The first few packets arrive before time synchronization finishes.
                return
            }

            let sample = EmotionSample(
                packet: packet,
                extendedDeviceTimestampMs: extendedTimestamp,
                eventDate: eventDate,
                receivedAt: receivedAt
            )

            lastRawTimestamp = packet.timestampMs
            statistics.valid += 1
            state = .streaming
            onEmotionSample?(sample)

            cloudTransport.send(
                CloudEmotionMessage(
                    sample: sample,
                    matchID: configuration.matchID
                )
            )
        } catch {
            statistics.rejected += 1
            onError?(error)
        }
    }

    private func handleTimeSyncData(_ data: Data) {
        do {
            let response = try TimeSyncResponsePacket.decode(data)

            try timeSynchronizer.complete(
                response: response,
                phoneReceiveMonotonicMs: Self.monotonicMilliseconds(),
                phoneReceiveUTCms: Date().timeIntervalSince1970 * 1000.0,
                timestampUnwrapper: &timestampUnwrapper
            )
        } catch {
            statistics.rejected += 1
            onError?(error)
        }
    }

    private func detectDeviceRestart(_ packet: EmotionPacket) {
        guard
            let previousSequence = lastSequence,
            let previousTimestamp = lastRawTimestamp
        else {
            return
        }

        let sequenceReset = previousSequence > 1_000 && packet.sequence < 100
        let timestampReset =
            previousTimestamp > 10_000 &&
            packet.timestampMs < previousTimestamp - 10_000

        guard sequenceReset && timestampReset else {
            return
        }

        lastSequence = nil
        lastRawTimestamp = nil
        timestampUnwrapper.reset()
        timeSynchronizer.reset()
        state = .synchronizingTime
        performInitialTimeSyncBurst()
    }

    private func updateSequenceStatistics(_ sequence: UInt32) {
        guard let previous = lastSequence else {
            lastSequence = sequence
            return
        }

        let forwardDelta = sequence &- previous

        if forwardDelta == 0 {
            statistics.duplicate += 1
            return
        }

        if forwardDelta < 0x8000_0000 {
            if forwardDelta > 1 {
                statistics.lost += UInt64(forwardDelta - 1)
            }
            lastSequence = sequence
        } else {
            statistics.outOfOrder += 1
        }
    }

    private static func monotonicMilliseconds() -> Double {
        ProcessInfo.processInfo.systemUptime * 1000.0
    }
}

extension StadiumCommunicationManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        guard central.state == .poweredOn else {
            central.stopScan()
            state = .bluetoothUnavailable
            return
        }

        if shouldReconnect {
            scan()
        }
    }

    func centralManager(
        _ central: CBCentralManager,
        didDiscover peripheral: CBPeripheral,
        advertisementData: [String: Any],
        rssi RSSI: NSNumber
    ) {
        let advertisedName =
            advertisementData[CBAdvertisementDataLocalNameKey] as? String

        let name = advertisedName ?? peripheral.name ?? ""

        guard name.hasPrefix(configuration.expectedDeviceNamePrefix) else {
            return
        }

        central.stopScan()
        self.peripheral = peripheral
        state = .connecting
        central.connect(peripheral, options: nil)
    }

    func centralManager(
        _ central: CBCentralManager,
        didConnect peripheral: CBPeripheral
    ) {
        peripheral.delegate = self
        state = .discoveringServices
        peripheral.discoverServices([
            StadiumCommunicationConfiguration.serviceUUID
        ])
    }

    func centralManager(
        _ central: CBCentralManager,
        didFailToConnect peripheral: CBPeripheral,
        error: Error?
    ) {
        state = .failed(error?.localizedDescription ?? "BLE connection failed")
        self.peripheral = nil

        if shouldReconnect {
            DispatchQueue.main.asyncAfter(deadline: .now() + 1) { [weak self] in
                self?.scan()
            }
        }
    }

    func centralManager(
        _ central: CBCentralManager,
        didDisconnectPeripheral peripheral: CBPeripheral,
        error: Error?
    ) {
        resetSession()
        self.peripheral = nil

        if shouldReconnect {
            DispatchQueue.main.asyncAfter(deadline: .now() + 1) { [weak self] in
                self?.scan()
            }
        } else {
            state = .idle
        }
    }
}

extension StadiumCommunicationManager: CBPeripheralDelegate {
    func peripheral(
        _ peripheral: CBPeripheral,
        didDiscoverServices error: Error?
    ) {
        if let error {
            state = .failed(error.localizedDescription)
            return
        }

        guard let service = peripheral.services?.first(where: {
            $0.uuid == StadiumCommunicationConfiguration.serviceUUID
        }) else {
            state = .failed("Required BLE service was not found")
            return
        }

        peripheral.discoverCharacteristics(
            [
                StadiumCommunicationConfiguration.emotionUUID,
                StadiumCommunicationConfiguration.controlUUID,
                StadiumCommunicationConfiguration.timeSyncUUID
            ],
            for: service
        )
    }

    func peripheral(
        _ peripheral: CBPeripheral,
        didDiscoverCharacteristicsFor service: CBService,
        error: Error?
    ) {
        if let error {
            state = .failed(error.localizedDescription)
            return
        }

        for characteristic in service.characteristics ?? [] {
            switch characteristic.uuid {
            case StadiumCommunicationConfiguration.emotionUUID:
                emotionCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)

            case StadiumCommunicationConfiguration.controlUUID:
                controlCharacteristic = characteristic

            case StadiumCommunicationConfiguration.timeSyncUUID:
                timeSyncCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)

            default:
                break
            }
        }

        guard
            emotionCharacteristic != nil,
            controlCharacteristic != nil,
            timeSyncCharacteristic != nil
        else {
            state = .failed("One or more required BLE characteristics are missing")
            return
        }
    }

    func peripheral(
        _ peripheral: CBPeripheral,
        didUpdateNotificationStateFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        if let error {
            state = .failed(error.localizedDescription)
            return
        }

        if characteristic.uuid == StadiumCommunicationConfiguration.emotionUUID {
            emotionNotificationsReady = characteristic.isNotifying
        } else if characteristic.uuid == StadiumCommunicationConfiguration.timeSyncUUID {
            timeSyncNotificationsReady = characteristic.isNotifying
        }

        beginStreamingIfReady()
    }

    func peripheral(
        _ peripheral: CBPeripheral,
        didUpdateValueFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        if let error {
            onError?(error)
            return
        }

        guard let data = characteristic.value else {
            statistics.rejected += 1
            return
        }

        if characteristic.uuid == StadiumCommunicationConfiguration.emotionUUID {
            handleEmotionData(data)
        } else if characteristic.uuid == StadiumCommunicationConfiguration.timeSyncUUID {
            handleTimeSyncData(data)
        }
    }

    func peripheral(
        _ peripheral: CBPeripheral,
        didWriteValueFor characteristic: CBCharacteristic,
        error: Error?
    ) {
        if let error {
            onError?(error)
        }
    }
}
