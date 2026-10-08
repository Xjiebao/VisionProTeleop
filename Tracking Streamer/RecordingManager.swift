import SwiftUI
import Foundation
import Combine
import simd
import UniformTypeIdentifiers
import AVFoundation
import ARKit
import VideoToolbox
import QuartzCore

// MARK: - Recording Data Structures

/// A single frame of recorded data containing tracking and video
struct RecordedFrame: Codable {
    let timestamp: Double  // Seconds since recording started
    let systemTime: Double  // Unix timestamp for synchronization
    
    // Head tracking (4x4 matrix flattened)
    let headMatrix: [Float]?
    
    // Hand tracking data
    let leftHand: HandJointData?
    let rightHand: HandJointData?
    
    // Video frame metadata
    let videoFrameIndex: Int  // Index into video frames
    let videoWidth: Int
    let videoHeight: Int
}

/// One source pose timed at receipt. recordingTimestamp is primary; ARKit time fields are diagnostic.
struct EgoPoseSample: Codable {
    let sequenceNumber: Int  // Source record order within this recording
    let receivedTimestamp: Double  // Consumer/query-return time, CACurrentMediaTime seconds
    let systemTime: Double  // Date at receipt, Unix seconds; not the pose timestamp
    let recordingTimestamp: Double  // Primary time: receipt relative to the monotonic recording start
    let wallClockRecordingTimestamp: Double  // Date difference from recording start, without clamping
    let queryStartedTimestamp: Double?  // Only queried device anchors
    let predictionOffset: Double?  // Query target offset; nil for hand events
    let trackingSessionId: String
    let source: String  // head, leftHand, rightHand
    let timestamp: Double  // Diagnostic ARKit anchor timestamp, mach absolute seconds (not Unix time)
    let updateTimestamp: Double?  // Hand AnchorUpdate only; unavailable for queried device anchors
    let queryTimestamp: Double?  // Device query target; not a replacement for timestamp
    let event: String?  // Hand AnchorUpdate event, including removed
    let isTracked: Bool
    let originFromAnchorTransform: [Float]  // Anchor/device -> ARKit world, meters, column-major
    let joints: [EgoJointSample]?  // Nil when ARKit supplies no skeleton
}

struct EgoJointSample: Codable {
    let name: String
    let isTracked: Bool
    let anchorFromJointTransform: [Float]  // Joint -> hand anchor, meters, column-major
}

/// A value snapshot of a hand anchor and its joints.
struct EgoPoseSnapshot {
    let timestamp: Double
    let isTracked: Bool
    let originFromAnchorTransform: [Float]
    let joints: [EgoJointSample]?
}

/// A single frame of simulation data
struct SimulationFrame: Codable {
    let timestamp: Double  // Seconds since recording started
    let poses: [String: [Float]]
    let qpos: [Float]?
    let ctrl: [Float]?
}

/// Hand joint positions for all 27 joints tracked by ARKit HandSkeleton
/// Joint order matches 🥽AppModel.swift jointTypes array:
///   0: wrist
///   1-4: thumb (knuckle, intermediateBase, intermediateTip, tip)
///   5-9: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   10-14: middle (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   15-19: ring (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   20-24: little (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   25: forearmWrist
///   26: forearmArm
/// Note: Indices 0-24 are identical between 25-joint and 27-joint formats
struct HandJointData: Codable {
    // Wrist (index 0)
    let wrist: [Float]  // 4x4 matrix flattened
    
    // Thumb (4 joints, indices 1-4) - no metacarpal for thumb
    let thumbKnuckle: [Float]          // thumbCMC equivalent
    let thumbIntermediateBase: [Float] // thumbMP equivalent  
    let thumbIntermediateTip: [Float]  // thumbIP equivalent
    let thumbTip: [Float]
    
    // Index finger (5 joints, indices 5-9)
    let indexMetacarpal: [Float]
    let indexKnuckle: [Float]          // indexMCP equivalent
    let indexIntermediateBase: [Float] // indexPIP equivalent
    let indexIntermediateTip: [Float]  // indexDIP equivalent
    let indexTip: [Float]
    
    // Middle finger (5 joints, indices 10-14)
    let middleMetacarpal: [Float]
    let middleKnuckle: [Float]
    let middleIntermediateBase: [Float]
    let middleIntermediateTip: [Float]
    let middleTip: [Float]
    
    // Ring finger (5 joints, indices 15-19)
    let ringMetacarpal: [Float]
    let ringKnuckle: [Float]
    let ringIntermediateBase: [Float]
    let ringIntermediateTip: [Float]
    let ringTip: [Float]
    
    // Little finger (5 joints, indices 20-24)
    let littleMetacarpal: [Float]
    let littleKnuckle: [Float]
    let littleIntermediateBase: [Float]
    let littleIntermediateTip: [Float]
    let littleTip: [Float]
    
    // Forearm joints (indices 25-26) - optional, may not be tracked
    let forearmWrist: [Float]?  // 4x4 matrix flattened (index 25)
    let forearmArm: [Float]?    // 4x4 matrix flattened (index 26)
}

/// Recording type for categorizing recordings
enum RecordingType: String, Codable {
    case egorecord = "egorecord"
    case teleopVideo = "teleoperation-video"
    case teleopMujoco = "teleoperation-mujoco"
    case teleopIsaac = "teleoperation-isaac"
}

/// Metadata for the entire recording session
struct RecordingMetadata: Codable {
    let version: String = "1.0"
    let createdAt: Date
    let duration: Double  // Total duration in seconds
    let frameCount: Int
    let hasVideo: Bool
    let hasLeftHand: Bool
    let hasRightHand: Bool
    let hasSimulationData: Bool
    let recordingType: RecordingType
    let hasUSDZ: Bool
    let videoSource: String  // "network" or "uvc"
    let averageFPS: Double
    let deviceInfo: DeviceInfo
    // Calibration data
    let intrinsicCalibration: [String: Any]?
    let extrinsicCalibration: [String: Any]?
    let poseData: [String: String]?  // Source pose timing contract; alignment is not yet validated
    let captureSessionID: String?
    let captureBoardID: String?
    
    enum CodingKeys: String, CodingKey {
        case version, createdAt, duration, frameCount, hasVideo
        case hasLeftHand, hasRightHand, hasSimulationData, hasUSDZ, videoSource, averageFPS, deviceInfo
        case intrinsicCalibration, extrinsicCalibration, recordingType, poseData, captureSessionID, captureBoardID
    }
    
    func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(version, forKey: .version)
        try container.encode(createdAt, forKey: .createdAt)
        try container.encode(duration, forKey: .duration)
        try container.encode(frameCount, forKey: .frameCount)
        try container.encode(hasVideo, forKey: .hasVideo)
        try container.encode(hasLeftHand, forKey: .hasLeftHand)
        try container.encode(hasRightHand, forKey: .hasRightHand)
        try container.encode(hasSimulationData, forKey: .hasSimulationData)
        try container.encode(recordingType, forKey: .recordingType)
        try container.encode(hasUSDZ, forKey: .hasUSDZ)
        try container.encode(videoSource, forKey: .videoSource)
        try container.encode(averageFPS, forKey: .averageFPS)
        try container.encode(deviceInfo, forKey: .deviceInfo)
        try container.encodeIfPresent(poseData, forKey: .poseData)
        try container.encodeIfPresent(captureSessionID, forKey: .captureSessionID)
        try container.encodeIfPresent(captureBoardID, forKey: .captureBoardID)
        // Encode intrinsic calibration as JSON data
        if let intrinsic = intrinsicCalibration {
            let jsonData = try JSONSerialization.data(withJSONObject: intrinsic, options: [])
            let jsonString = String(data: jsonData, encoding: .utf8) ?? "{}"
            try container.encode(jsonString, forKey: .intrinsicCalibration)
        }
        // Encode extrinsic calibration as JSON data
        if let extrinsic = extrinsicCalibration {
            let jsonData = try JSONSerialization.data(withJSONObject: extrinsic, options: [])
            let jsonString = String(data: jsonData, encoding: .utf8) ?? "{}"
            try container.encode(jsonString, forKey: .extrinsicCalibration)
        }
    }
    
    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        // version has default value
        createdAt = try container.decode(Date.self, forKey: .createdAt)
        duration = try container.decode(Double.self, forKey: .duration)
        frameCount = try container.decode(Int.self, forKey: .frameCount)
        hasVideo = try container.decode(Bool.self, forKey: .hasVideo)
        hasLeftHand = try container.decode(Bool.self, forKey: .hasLeftHand)
        hasRightHand = try container.decode(Bool.self, forKey: .hasRightHand)
        hasSimulationData = try container.decodeIfPresent(Bool.self, forKey: .hasSimulationData) ?? false
        recordingType = try container.decodeIfPresent(RecordingType.self, forKey: .recordingType) ?? .teleopVideo
        hasUSDZ = try container.decodeIfPresent(Bool.self, forKey: .hasUSDZ) ?? false
        videoSource = try container.decode(String.self, forKey: .videoSource)
        averageFPS = try container.decode(Double.self, forKey: .averageFPS)
        deviceInfo = try container.decode(DeviceInfo.self, forKey: .deviceInfo)
        poseData = try container.decodeIfPresent([String: String].self, forKey: .poseData)
        captureSessionID = try container.decodeIfPresent(String.self, forKey: .captureSessionID)
        captureBoardID = try container.decodeIfPresent(String.self, forKey: .captureBoardID)
        // Decode intrinsic calibration from JSON string
        if let jsonString = try container.decodeIfPresent(String.self, forKey: .intrinsicCalibration),
           let jsonData = jsonString.data(using: .utf8),
           let dict = try JSONSerialization.jsonObject(with: jsonData, options: []) as? [String: Any] {
            intrinsicCalibration = dict
        } else {
            intrinsicCalibration = nil
        }
        // Decode extrinsic calibration from JSON string
        if let jsonString = try container.decodeIfPresent(String.self, forKey: .extrinsicCalibration),
           let jsonData = jsonString.data(using: .utf8),
           let dict = try JSONSerialization.jsonObject(with: jsonData, options: []) as? [String: Any] {
            extrinsicCalibration = dict
        } else {
            extrinsicCalibration = nil
        }
    }
    
    init(createdAt: Date, duration: Double, frameCount: Int, hasVideo: Bool, hasLeftHand: Bool,
         hasRightHand: Bool, hasSimulationData: Bool, hasUSDZ: Bool, videoSource: String, averageFPS: Double,
         deviceInfo: DeviceInfo, recordingType: RecordingType, intrinsicCalibration: [String: Any]? = nil, extrinsicCalibration: [String: Any]? = nil, poseData: [String: String]? = nil,
         captureSessionID: String? = nil, captureBoardID: String? = nil) {
        self.createdAt = createdAt
        self.duration = duration
        self.frameCount = frameCount
        self.hasVideo = hasVideo
        self.hasLeftHand = hasLeftHand
        self.hasRightHand = hasRightHand
        self.hasSimulationData = hasSimulationData
        self.hasUSDZ = hasUSDZ
        self.videoSource = videoSource
        self.averageFPS = averageFPS
        self.deviceInfo = deviceInfo
        self.recordingType = recordingType
        self.intrinsicCalibration = intrinsicCalibration
        self.extrinsicCalibration = extrinsicCalibration
        self.poseData = poseData
        self.captureSessionID = captureSessionID
        self.captureBoardID = captureBoardID
    }
}

struct DeviceInfo: Codable {
    let model: String
    let systemVersion: String
    let appVersion: String
}

/// Storage location options - simplified to Local vs Cloud
enum RecordingStorageLocation: String, CaseIterable {
    case local = "Local"
    case cloud = "Cloud"
    
    var icon: String {
        switch self {
        case .local: return "internaldrive"
        case .cloud: return "icloud"
        }
    }
    
    var description: String {
        switch self {
        case .local: return "Documents folder (Files app)"
        case .cloud: return "Synced via cloud provider"
        }
    }
}

// MARK: - Video Frame for storage
struct VideoFrameData {
    let index: Int
    let timestamp: Double
    let image: UIImage
    let presentationTime: CMTime
}

private struct PendingBoardUpload: Codable {
    let folderName: String
    let sessionID: String
    let boardID: String
}

struct PendingBoardConfirmation: Codable {
    let sessionID: String
    let boardID: String
    let folderName: String?
}

private enum BoardCaptureError: LocalizedError {
    case message(String)
    var errorDescription: String? {
        if case let .message(text) = self { return text }
        return nil
    }
}

// MARK: - Recording Manager

/// Manages synchronized recording of tracking data and video frames.
/// Recording is VIDEO-DRIVEN: each new video frame triggers a recording of the latest tracking data.
/// Auto-recording: Recording starts automatically when video frames arrive and stops when disconnected.
@MainActor
class RecordingManager: ObservableObject {
    static let shared = RecordingManager()
    
    // MARK: - Published Properties
    @Published var isRecording: Bool = false
    @Published var recordingDuration: TimeInterval = 0
    @Published var frameCount: Int = 0
    @Published var storageLocation: RecordingStorageLocation {
        didSet {
            UserDefaults.standard.set(storageLocation.rawValue, forKey: "recordingStorageLocation")
        }
    }
    @Published var lastRecordingURL: URL? = nil
    @Published var recordingError: String? = nil
    @Published var isSaving: Bool = false
    @Published private(set) var clockSyncStatus = "电脑对钟正在启动…"
    @Published private(set) var isBoardOperationInProgress = false
    @Published private(set) var boardCaptureStatus = "等待连接采集板"
    @Published private(set) var boardClockSummary = "尚未检测网络延迟与钟差"
    @Published private(set) var captureSessionID: String? = nil
    @Published private(set) var pendingBoardUploadCount = 0
    @Published private(set) var pendingBoardConfirmation: PendingBoardConfirmation? = nil
    @Published private(set) var boardUploadError: String? = nil

    // Cloud storage (synced from iOS companion app)
    @Published var cloudProvider: CloudStorageProvider = .iCloudDrive
    @Published var isUploadingToCloud: Bool = false
    @Published var cloudUploadProgress: String = ""
    @Published var cloudUploadCurrentFile: Int = 0
    @Published var cloudUploadTotalFiles: Int = 0
    @Published var cloudUploadCurrentFileName: String = ""  // Current file being uploaded
    
    // Auto-recording state
    @Published var autoRecordingEnabled: Bool {
        didSet {
            UserDefaults.standard.set(autoRecordingEnabled, forKey: "autoRecordingEnabled")
        }
    }
    @Published var isAutoRecording: Bool = false  // True if current recording was auto-started
    private var userManuallyStopped: Bool = false // Prevent auto-restart after manual stop
    
    // Hand-tracking-only recording timer
    private var handTrackingRecordingTimer: Timer?
    private let handTrackingRecordingHz: Double = 60.0  // Record at 60Hz when no video/sim
    
    // MARK: - Private Properties
    private var recordingStartTime: Date?
    private var recordingStartMonotonicTimestamp: TimeInterval = 0
    private var isEgoRecording = false  // Freeze the mode at recording start
    private var egoPoseSamples: [EgoPoseSample] = []  // MainActor-owned immutable source samples
    private var egoSequenceNumber = 0
    private let clockSyncServer = ClockSyncServer()
    private var captureBoardID: String?
    private var boardCaptureWarning: String?
    private var boardStopRequested = false
    private var localSaveTask: Task<Void, Never>?
    private var boardMonitorTask: Task<Void, Never>?
    private var boardUploadTask: Task<Void, Never>?
    private var pendingBoardUploads: [PendingBoardUpload] = []

    private var isChoosingBoardSessionID = false
    var hasBoardCapture: Bool { captureSessionID != nil || isChoosingBoardSessionID }

    var isRecordingEgoPoses: Bool { isRecording && isEgoRecording }
    private var durationTimer: Timer?
    private var sessionID: String = ""
    private let recordingQueue = DispatchQueue(label: "com.visionproteleop.recording", qos: .userInitiated)
    private let videoWriterQueue = DispatchQueue(label: "com.visionproteleop.videowriter", qos: .userInitiated)
    
    // Frame data (accessed from recordingQueue - use nonisolated(unsafe) for background access)
    nonisolated(unsafe) private var recordedFrames: [RecordedFrame] = []
    nonisolated(unsafe) private var videoFrames: [VideoFrameData] = []
    nonisolated(unsafe) private var pendingFrameCount: Int = 0
    nonisolated(unsafe) private var simulationFrames: [SimulationFrame] = []
    nonisolated(unsafe) private var simulationFrameCount: Int = 0
    private var usdzURL: URL?
    
    // Recording settings
    var videoQuality: CGFloat = 0.7  // JPEG compression quality (0.0-1.0)
    nonisolated(unsafe) var videoBitRate: Int = 10_000_000  // 10 Mbps for H.264 encoding
    
    // Video writer components
    nonisolated(unsafe) private var assetWriter: AVAssetWriter?
    nonisolated(unsafe) private var videoWriterInput: AVAssetWriterInput?
    nonisolated(unsafe) private var pixelBufferAdaptor: AVAssetWriterInputPixelBufferAdaptor?
    nonisolated(unsafe) private var videoSize: CGSize = .zero
    nonisolated(unsafe) private var isWriterSessionStarted: Bool = false
    nonisolated(unsafe) private var pixelBufferPool: CVPixelBufferPool?
    nonisolated(unsafe) private var recordingFolderURL: URL?
    nonisolated(unsafe) private var lastPresentationTime: CMTime?
    
    // Notification observers
    private var cancellables = Set<AnyCancellable>()
    
    // MARK: - Initialization
    
    private init() {
        // Load saved storage location
        if CloudStorageSettings.isEnabled,
           let savedLocation = UserDefaults.standard.string(forKey: "recordingStorageLocation"),
           let location = RecordingStorageLocation(rawValue: savedLocation) {
            self.storageLocation = location
        } else {
            self.storageLocation = .local  // Default to local storage
        }
        // Load auto-recording preference (default to true for auto-record by default)
        self.autoRecordingEnabled = UserDefaults.standard.object(forKey: "autoRecordingEnabled") as? Bool ?? true
        captureSessionID = UserDefaults.standard.string(forKey: "captureSessionID")
        captureBoardID = UserDefaults.standard.string(forKey: "activeCaptureBoardID")
        if let data = UserDefaults.standard.data(forKey: "pendingBoardUploads"),
           let pending = try? JSONDecoder().decode([PendingBoardUpload].self, from: data) {
            pendingBoardUploads = pending
            pendingBoardUploadCount = pending.count
        }
        if let data = UserDefaults.standard.data(forKey: "pendingBoardConfirmation"),
           let pending = try? JSONDecoder().decode(PendingBoardConfirmation.self, from: data) {
            pendingBoardConfirmation = pending
            if captureSessionID == pending.sessionID && captureBoardID == pending.boardID {
                clearBoardCapture()
            }
            boardCaptureStatus = "本轮文件已保留，请选择保存或放弃"
        } else if captureSessionID != nil {
            boardCaptureStatus = "有未确认结束的采集，请连接原采集板后重试结束"
        }

        // The background video writer reads this preference directly.
        UserDefaults.standard.set(storageLocation.rawValue, forKey: "recordingStorageLocation")
        
        // Load cloud provider from keychain (synced from iOS)
        if CloudStorageSettings.isEnabled {
            loadCloudSettings()
            setupCloudSettingsObserver()
        }
    }
    
    /// Setup observer for cloud settings changes (from iCloud Keychain sync)
    private func setupCloudSettingsObserver() {
        NotificationCenter.default.publisher(for: .cloudStorageSettingsDidChange)
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in
                Task { @MainActor in
                    self?.loadCloudSettings()
                }
            }
            .store(in: &cancellables)
    }
    
    /// Load cloud storage settings from iCloud Keychain (set by iOS companion app)
    func loadCloudSettings() {
        guard CloudStorageSettings.isEnabled else { return }
        CloudStorageSettings.shared.loadSettings()
        cloudProvider = CloudStorageSettings.shared.getActiveProvider()
        // dlog("☁️ [RecordingManager] Cloud provider: \(cloudProvider.displayName)")
    }
    
    // MARK: - Auto-Recording Control
    
    /// Called when first video frame is received. Starts recording if auto-recording is enabled.
    /// Video source can be UVC camera or network stream.
    func onFirstVideoFrame() {
        guard UserDefaults.standard.string(forKey: "appMode") != "egorecord" else { return }
        guard autoRecordingEnabled && !isRecording && !userManuallyStopped else { return }
        
        dlog("🔴 [RecordingManager] Auto-starting recording on first video frame")
        isAutoRecording = true
        startRecording()
    }
    
    /// Called when video source is disconnected (UVC camera unplugged, Python client disconnected, or WebRTC disconnected).
    /// Stops recording if it was auto-started.
    func onVideoSourceDisconnected(reason: String) {
        // Reset manual stop flag so auto-recording can work on next connection
        userManuallyStopped = false
        
        guard isRecording else { return }
        
        dlog("🔴 [RecordingManager] Stopping recording due to: \(reason)")
        stopRecording()
        isAutoRecording = false
    }
    
    /// Explicitly stop recording (user action). This also clears auto-recording state.
    func stopRecordingManually() {
        guard isRecording || hasBoardCapture else { return }
        
        dlog("🔴 [RecordingManager] User manually stopped recording")
        userManuallyStopped = true // Prevent immediate auto-restart
        stopRecording()
        isAutoRecording = false
    }
    
    // MARK: - Independent Clock Sync Control

    func startClockSync() {
        clockSyncServer.start { [weak self] status in
            self?.clockSyncStatus = status
        }
    }

    func restartClockSync() {
        clockSyncServer.stop()
        startClockSync()
    }

    // MARK: - External Capture Board

    func checkBoardClock() {
        guard !isBoardOperationInProgress, !hasBoardCapture else { return }
        isBoardOperationInProgress = true
        recordingError = nil
        startClockSync()
        Task {
            defer { isBoardOperationInProgress = false }
            do {
                let status = try await CaptureBoardClient.shared.checkClock()
                showBoardClock(status)
            } catch {
                recordingError = "延迟检测失败：\(error.localizedDescription)"
            }
        }
    }

    private func showBoardClock(_ status: BoardStatus) {
        guard let summary = status.syncSummary else { return }
        boardClockSummary = String(format: "网络往返 %.2f ms · VP−板子钟差 %+.2f ms",
                                   summary.networkRTTMS, summary.offsetVPMinusMacSeconds * 1000)
    }

    private func nextBoardSessionID(boardID: String) async throws -> String {
        let formatter = DateFormatter()
        formatter.dateFormat = "yyMMddHHmm"
        let base = formatter.string(from: Date())
        let storage = try getStorageURL()
        var number = 1
        while true {
            if boardStopRequested { throw BoardCaptureError.message("采集已取消") }
            let id = number == 1 ? base : "\(base)_\(String(format: "%02d", number))"
            if !FileManager.default.fileExists(atPath: storage.appendingPathComponent(id).path) {
                do {
                    _ = try await CaptureBoardClient.shared.status(sessionID: id, boardID: boardID)
                } catch let error as CaptureBoardError where error.statusCode == 404 {
                    return id
                }
            }
            number += 1
        }
    }

    private func startBoardCapture() {
        guard !isRecording, !isSaving, !isBoardOperationInProgress, !hasBoardCapture,
              pendingBoardConfirmation == nil else { return }
        guard let board = CaptureBoardClient.shared.selectedBoard else {
            recordingError = "尚未找到采集板，请确认板子和 VP 已接入同一局域网。"
            return
        }
        boardStopRequested = false
        boardCaptureWarning = nil
        isChoosingBoardSessionID = true
        isBoardOperationInProgress = true
        boardCaptureStatus = "准备采集、执行录前对钟…"
        recordingError = nil
        startClockSync()
        Task {
            do {
                let id = try await nextBoardSessionID(boardID: board.id)
                if boardStopRequested { throw BoardCaptureError.message("采集已取消") }
                isChoosingBoardSessionID = false
                captureSessionID = id
                captureBoardID = board.id
                UserDefaults.standard.set(id, forKey: "captureSessionID")
                UserDefaults.standard.set(board.id, forKey: "activeCaptureBoardID")
                let prepared = try await CaptureBoardClient.shared.prepare(sessionID: id, boardID: board.id)
                showBoardClock(prepared)
                if boardStopRequested { throw BoardCaptureError.message("采集已取消") }
                startLocalRecording(captureSession: id)
                boardCaptureStatus = "等待 VP 头部跟踪开始…"
                let deadline = CACurrentMediaTime() + 3
                while !egoPoseSamples.contains(where: { $0.source == "head" && $0.isTracked }) {
                    if boardStopRequested { throw BoardCaptureError.message("采集已取消") }
                    if CACurrentMediaTime() >= deadline {
                        throw BoardCaptureError.message("VP 未产生有效头部数据，请保持采集视图开启并检查跟踪权限。")
                    }
                    try await Task.sleep(nanoseconds: 50_000_000)
                }
                if boardStopRequested { throw BoardCaptureError.message("采集已取消") }
                boardCaptureStatus = "VP 已开始，等待相机首帧…"
                let running = try await CaptureBoardClient.shared.start(sessionID: id, boardID: board.id)
                guard running.state == "recording" else {
                    throw BoardCaptureError.message(running.error ?? "相机未确认开始录像")
                }
                boardCaptureStatus = "VP 与采集板正在录制"
                monitorBoardCapture(sessionID: id, boardID: board.id)
            } catch {
                isChoosingBoardSessionID = false
                recordingError = "开始采集失败：\(error.localizedDescription)"
                boardCaptureWarning = recordingError
                if captureSessionID == nil { boardCaptureStatus = "采集未启动" }
                boardStopRequested = true
            }
            isBoardOperationInProgress = false
            if boardStopRequested { await finishBoardCapture() }
        }
    }

    private func monitorBoardCapture(sessionID id: String, boardID: String) {
        boardMonitorTask?.cancel()
        boardMonitorTask = Task {
            while !Task.isCancelled && captureSessionID == id && !boardStopRequested {
                do {
                    try await Task.sleep(nanoseconds: 2_000_000_000)
                    if Task.isCancelled || boardStopRequested { return }
                    let status = try await CaptureBoardClient.shared.status(sessionID: id, boardID: boardID)
                    if status.state != "recording" {
                        recordingError = status.error ?? "采集板录像已停止，正在保存 VP 数据。"
                        boardCaptureWarning = recordingError
                        requestBoardStop()
                        return
                    }
                    boardCaptureStatus = "VP 与采集板正在录制"
                } catch {
                    if Task.isCancelled { return }
                    boardCaptureStatus = "板子连接中断，录像状态未确认；VP 仍在本地采集"
                }
            }
        }
    }

    private func requestBoardStop() {
        boardStopRequested = true
        guard !isBoardOperationInProgress else { return }
        // Set the guard before creating the task, including when the user taps twice.
        isBoardOperationInProgress = true
        Task { await finishBoardCapture() }
    }

    /// Called before ARKit stops, and when the app enters the background.
    func trackingDidStop() {
        guard hasBoardCapture else { return }
        recordingError = "跟踪已停止，正在结束采集；请在电脑检查轨迹覆盖范围。"
        boardCaptureWarning = recordingError
        // ARKit has already stopped producing data: save immediately even if the network is down.
        stopLocalRecording()
        requestBoardStop()
    }

    private func finishBoardCapture() async {
        guard let id = captureSessionID, let boardID = captureBoardID else {
            isBoardOperationInProgress = false
            return
        }
        isBoardOperationInProgress = true
        boardStopRequested = true
        boardMonitorTask?.cancel()
        boardMonitorTask = nil
        boardCaptureStatus = "正在停止采集板并保存视频…"
        defer {
            isBoardOperationInProgress = false
            resumeBoardTransfers()
        }
        var boardStopped = false
        do {
            let stopped = try await CaptureBoardClient.shared.stop(sessionID: id, boardID: boardID)
            guard stopped.state == "saved" || stopped.state == "failed" else {
                throw BoardCaptureError.message("板子仍未确认停止，请重试结束")
            }
            boardStopped = true
            let wasRecording = isRecording
            stopLocalRecording()
            await localSaveTask?.value
            if !wasRecording && !egoPoseSamples.isEmpty && lastRecordingURL?.lastPathComponent != id {
                // An explicit retry of "结束" may retry a failed local save while its source samples remain.
                isSaving = true
                await saveRecording()
            }
            if recordingError == nil { recordingError = boardCaptureWarning }
            let folder = try savedBoardRecording(sessionID: id)
            if folder == nil && !egoPoseSamples.isEmpty {
                throw BoardCaptureError.message("VP 文件保存失败；请重试结束以重新保存。")
            }
            if stopped.state == "saved" {
                if folder == nil {
                    recordingError = "板子视频已保存，但本轮缺少 VP 原始文件；该轮不完整。"
                }
                boardCaptureStatus = "采集已停止，正在录后对钟…"
                let synced = try await CaptureBoardClient.shared.syncAfter(sessionID: id, boardID: boardID)
                showBoardClock(synced)
            } else {
                if recordingError == nil { recordingError = stopped.error }
            }
            if let folder {
                try markBoardRecording(folder: folder, sessionID: id, boardID: boardID, reviewState: "pending")
            }
            let pending = PendingBoardConfirmation(sessionID: id, boardID: boardID,
                                                    folderName: folder?.lastPathComponent)
            UserDefaults.standard.set(try JSONEncoder().encode(pending), forKey: "pendingBoardConfirmation")
            pendingBoardConfirmation = pending
            boardCaptureStatus = recordingError == nil
                ? "本轮文件已保留，请选择保存或放弃"
                : "本轮采集有异常，现有文件已保留，请选择保存或放弃"
            clearBoardCapture()
        } catch {
            // A failed network stop must not discard VP data or claim that the camera stopped.
            stopLocalRecording()
            await localSaveTask?.value
            if recordingError == nil { recordingError = boardCaptureWarning }
            if !boardStopped, let requestError = error as? CaptureBoardError, requestError.statusCode == 404 {
                clearBoardCapture()
                boardCaptureStatus = "采集未启动"
            } else {
                boardCaptureStatus = "VP 已停止；本轮结束未完成，请重试结束"
                recordingError = "结束采集失败：\(error.localizedDescription)"
            }
        }
    }

    func confirmBoardCapture(keep: Bool) {
        guard let pending = pendingBoardConfirmation, !isBoardOperationInProgress else { return }
        isBoardOperationInProgress = true
        let decision = keep ? "kept" : "discarded"
        boardCaptureStatus = keep ? "正在确认保存…" : "正在标记作废…"
        Task {
            defer { isBoardOperationInProgress = false }
            do {
                let status = try await CaptureBoardClient.shared.review(
                    sessionID: pending.sessionID, boardID: pending.boardID, decision: decision)
                guard status.reviewState == decision else {
                    throw BoardCaptureError.message("板子尚未确认本轮处理结果")
                }
                if let folderName = pending.folderName {
                    let folder = try getStorageURL().appendingPathComponent(folderName)
                    try markBoardRecording(folder: folder, sessionID: pending.sessionID,
                                           boardID: pending.boardID, reviewState: decision)
                    if keep { enqueueBoardUpload(folder: folder, sessionID: pending.sessionID, boardID: pending.boardID) }
                }
                if !keep {
                    pendingBoardUploads.removeAll {
                        $0.sessionID == pending.sessionID && $0.boardID == pending.boardID
                    }
                    persistBoardUploads()
                }
                pendingBoardConfirmation = nil
                UserDefaults.standard.removeObject(forKey: "pendingBoardConfirmation")
                recordingError = nil
                boardCaptureStatus = keep
                    ? (pending.folderName == nil ? "已确认保留，但本轮缺少 VP 文件" : "本轮已确认保存")
                    : "本轮已标记作废，所有现有文件仍保留"
                resumeBoardTransfers()
            } catch {
                boardCaptureStatus = "本轮尚未确认，请重试保存或放弃"
                recordingError = "确认\(keep ? "保存" : "放弃")失败：\(error.localizedDescription)"
            }
        }
    }

    private func markBoardRecording(folder: URL, sessionID: String, boardID: String, reviewState: String) throws {
        let metadataURL = folder.appendingPathComponent("metadata.json")
        guard var metadata = try JSONSerialization.jsonObject(with: Data(contentsOf: metadataURL)) as? [String: Any],
              metadata["captureSessionID"] as? String == sessionID,
              metadata["captureBoardID"] as? String == boardID else {
            throw BoardCaptureError.message("VP 文件的采集编号或板子身份不匹配")
        }
        guard metadata["reviewState"] as? String != reviewState else { return }
        metadata["reviewState"] = reviewState
        try JSONSerialization.data(withJSONObject: metadata, options: .prettyPrinted)
            .write(to: metadataURL, options: .atomic)
    }

    private func clearBoardCapture() {
        captureSessionID = nil
        captureBoardID = nil
        boardStopRequested = false
        UserDefaults.standard.removeObject(forKey: "captureSessionID")
        UserDefaults.standard.removeObject(forKey: "activeCaptureBoardID")
    }

    private func savedBoardRecording(sessionID id: String) throws -> URL? {
        let folder = try getStorageURL().appendingPathComponent(id)
        let metadata = folder.appendingPathComponent("metadata.json")
        let tracking = folder.appendingPathComponent("tracking_events.jsonl")
        guard FileManager.default.fileExists(atPath: metadata.path),
              FileManager.default.fileExists(atPath: tracking.path) else { return nil }
        let data = try JSONSerialization.jsonObject(with: Data(contentsOf: metadata)) as? [String: Any]
        guard data?["captureSessionID"] as? String == id,
              data?["captureBoardID"] as? String == captureBoardID else {
            throw BoardCaptureError.message("VP 文件的采集编号或板子身份不匹配")
        }
        return folder
    }

    private func enqueueBoardUpload(folder: URL, sessionID: String, boardID: String) {
        guard !pendingBoardUploads.contains(where: { $0.sessionID == sessionID && $0.boardID == boardID }) else { return }
        pendingBoardUploads.append(PendingBoardUpload(folderName: folder.lastPathComponent, sessionID: sessionID, boardID: boardID))
        persistBoardUploads()
    }

    private func persistBoardUploads() {
        pendingBoardUploadCount = pendingBoardUploads.count
        do {
            UserDefaults.standard.set(try JSONEncoder().encode(pendingBoardUploads), forKey: "pendingBoardUploads")
        } catch {
            boardUploadError = "待传输列表保存失败：\(error.localizedDescription)"
        }
    }

    func resumeBoardTransfers() {
        guard boardUploadTask == nil, !pendingBoardUploads.isEmpty else { return }
        boardUploadTask = Task {
            defer { boardUploadTask = nil }
            var deferredItems: [PendingBoardUpload] = []
            while let item = pendingBoardUploads.first(where: { item in
                !(pendingBoardConfirmation?.sessionID == item.sessionID
                  && pendingBoardConfirmation?.boardID == item.boardID)
                    && !deferredItems.contains { $0.sessionID == item.sessionID && $0.boardID == item.boardID }
                    && CaptureBoardClient.shared.boards.contains { $0.id == item.boardID }
            }) {
                boardUploadError = nil
                do {
                    let folder = try getStorageURL().appendingPathComponent(item.folderName)
                    let existing = try await CaptureBoardClient.shared.status(sessionID: item.sessionID, boardID: item.boardID)
                    if (existing.state == "saved" || existing.state == "failed") && existing.reviewState == "pending" {
                        if pendingBoardConfirmation == nil && !hasBoardCapture {
                            try markBoardRecording(folder: folder, sessionID: item.sessionID,
                                                   boardID: item.boardID, reviewState: "pending")
                            let pending = PendingBoardConfirmation(sessionID: item.sessionID, boardID: item.boardID,
                                                                   folderName: item.folderName)
                            UserDefaults.standard.set(try JSONEncoder().encode(pending), forKey: "pendingBoardConfirmation")
                            pendingBoardConfirmation = pending
                            boardCaptureStatus = "已恢复上次采集，请选择保存或放弃"
                        } else {
                            deferredItems.append(item)
                        }
                        continue
                    }
                    let status = try await CaptureBoardClient.shared.uploadRecording(
                        folder: folder, sessionID: item.sessionID, boardID: item.boardID)
                    guard status.vpUploaded else { throw BoardCaptureError.message("板子尚未确认收到两份文件") }
                    pendingBoardUploads.removeAll { $0.sessionID == item.sessionID && $0.boardID == item.boardID }
                    persistBoardUploads()
                } catch {
                    CaptureBoardClient.shared.failUpload(sessionID: item.sessionID, error: error)
                    boardUploadError = "传输未完成，原件保留在 VP：\(error.localizedDescription)"
                    return
                }
            }
        }
    }

    // MARK: - Recording Control
    
    func startRecording() {
        guard pendingBoardConfirmation == nil else { return }
        if UserDefaults.standard.string(forKey: "appMode") == "egorecord" {
            startBoardCapture()
            return
        }
        startLocalRecording()
    }

    private func startLocalRecording(captureSession: String? = nil) {
        guard !isRecording, !isSaving else { return }
        isEgoRecording = captureSession != nil || UserDefaults.standard.string(forKey: "appMode") == "egorecord"
        egoPoseSamples.removeAll()
        egoSequenceNumber = 0
        
        dlog("🔴 [RecordingManager] Starting recording (video-driven mode)...")
        
        // Reset state on background queue to avoid blocking
        recordingQueue.async { [weak self] in
            self?.recordedFrames.removeAll()
            self?.videoFrames.removeAll()
            self?.simulationFrames.removeAll()
            self?.pendingFrameCount = 0
            self?.simulationFrameCount = 0
            self?.isWriterSessionStarted = false
            self?.videoSize = .zero
            self?.lastPresentationTime = nil
            self?.recordingFolderURL = nil
        }
        
        recordingStartMonotonicTimestamp = CACurrentMediaTime()
        recordingStartTime = Date()
        frameCount = 0
        recordingDuration = 0
        recordingError = nil
        lastRecordingURL = nil
        
        // Generate session ID with UUID to ensure uniqueness even within same second
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyyMMdd_HHmmss"
        let uuidShort = UUID().uuidString.prefix(4)
        sessionID = captureSession ?? "recording_\(formatter.string(from: Date()))_\(uuidShort)"
        
        isRecording = true
        
        // Start duration timer - also updates frame count periodically
        // Also checks if we should fall back to hand-tracking-only mode
        durationTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor in
                guard let self = self, let startTime = self.recordingStartTime else { return }
                self.recordingDuration = Date().timeIntervalSince(startTime)
                self.frameCount = self.isEgoRecording ? self.egoPoseSamples.count : max(self.pendingFrameCount, self.simulationFrameCount)
                
                // After 0.5 seconds, if no video/sim data has arrived, start hand-tracking-only mode
                if self.recordingDuration > 0.5 {
                    self.checkAndStartHandTrackingOnlyMode()
                }
            }
        }
        
        dlog("🔴 [RecordingManager] Recording started with session ID: \(sessionID)")
    }
    
    /// Set up the video writer for MP4 output
    nonisolated private func setupVideoWriter(at url: URL, size: CGSize) throws {
        // Clean up any existing file
        try? FileManager.default.removeItem(at: url)
        
        let writer = try AVAssetWriter(outputURL: url, fileType: .mp4)
        
        // Use slightly lower settings for better real-time performance
        // especially when running with debugger attached
        let videoSettings: [String: Any] = [
            AVVideoCodecKey: AVVideoCodecType.h264,
            AVVideoWidthKey: Int(size.width),
            AVVideoHeightKey: Int(size.height),
            AVVideoCompressionPropertiesKey: [
                AVVideoAverageBitRateKey: videoBitRate,
                AVVideoProfileLevelKey: AVVideoProfileLevelH264BaselineAutoLevel,  // Baseline is faster to encode
                AVVideoExpectedSourceFrameRateKey: 30,
                AVVideoAllowFrameReorderingKey: false,  // No B-frames = lower latency
                AVVideoMaxKeyFrameIntervalKey: 30  // Keyframe every second
            ]
        ]
        
        let writerInput = AVAssetWriterInput(mediaType: .video, outputSettings: videoSettings)
        writerInput.expectsMediaDataInRealTime = true
        
        // Pixel buffer attributes with pool for efficiency
        let sourcePixelBufferAttributes: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey as String: Int(size.width),
            kCVPixelBufferHeightKey as String: Int(size.height),
            kCVPixelBufferIOSurfacePropertiesKey as String: [:] as [String: Any]
        ]
        
        let adaptor = AVAssetWriterInputPixelBufferAdaptor(
            assetWriterInput: writerInput,
            sourcePixelBufferAttributes: sourcePixelBufferAttributes
        )
        
        if writer.canAdd(writerInput) {
            writer.add(writerInput)
        } else {
            throw RecordingError.videoWriterSetupFailed
        }
        
        guard writer.startWriting() else {
            if let error = writer.error {
                dlog("❌ [RecordingManager] Writer failed to start: \(error)")
            }
            throw RecordingError.videoWriterSetupFailed
        }
        
        self.assetWriter = writer
        self.videoWriterInput = writerInput
        self.pixelBufferAdaptor = adaptor
        self.videoSize = size
        
        // Get the pixel buffer pool from the adaptor (will be available after starting session)
        
        dlog("🎬 [RecordingManager] Video writer set up for \(Int(size.width))x\(Int(size.height))")
    }
    
    /// Create a pixel buffer from a UIImage using pool if available
    nonisolated private func createPixelBuffer(from image: UIImage) -> CVPixelBuffer? {
        guard let cgImage = image.cgImage else { return nil }
        
        let width = cgImage.width
        let height = cgImage.height
        
        var pixelBuffer: CVPixelBuffer?
        
        // Try to use the adaptor's pool first (more efficient)
        if let pool = pixelBufferAdaptor?.pixelBufferPool {
            let status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &pixelBuffer)
            if status != kCVReturnSuccess {
                pixelBuffer = nil
            }
        }
        
        // Fall back to creating a new buffer if pool isn't available
        if pixelBuffer == nil {
            let attributes: [String: Any] = [
                kCVPixelBufferCGImageCompatibilityKey as String: true,
                kCVPixelBufferCGBitmapContextCompatibilityKey as String: true,
                kCVPixelBufferWidthKey as String: width,
                kCVPixelBufferHeightKey as String: height,
                kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
            ]
            
            let status = CVPixelBufferCreate(
                kCFAllocatorDefault,
                width,
                height,
                kCVPixelFormatType_32BGRA,
                attributes as CFDictionary,
                &pixelBuffer
            )
            
            guard status == kCVReturnSuccess else { return nil }
        }
        
        guard let buffer = pixelBuffer else { return nil }
        
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        
        guard let context = CGContext(
            data: CVPixelBufferGetBaseAddress(buffer),
            width: width,
            height: height,
            bitsPerComponent: 8,
            bytesPerRow: CVPixelBufferGetBytesPerRow(buffer),
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
        ) else {
            return nil
        }
        
        context.draw(cgImage, in: CGRect(x: 0, y: 0, width: width, height: height))
        
        return buffer
    }
    
    func stopRecording() {
        if hasBoardCapture {
            requestBoardStop()
            return
        }
        stopLocalRecording()
    }

    private func stopLocalRecording() {
        guard isRecording else { return }
        
        dlog("🔴 [RecordingManager] Stopping recording...")
        
        isRecording = false
        durationTimer?.invalidate()
        durationTimer = nil
        
        // Stop hand-tracking-only timer if running
        handTrackingRecordingTimer?.invalidate()
        handTrackingRecordingTimer = nil
        
        // Sync final count
        frameCount = isEgoRecording ? egoPoseSamples.count : pendingFrameCount
        
        // Calculate final duration
        if let startTime = recordingStartTime {
            recordingDuration = Date().timeIntervalSince(startTime)
        }
        
        dlog("🔴 [RecordingManager] Recording stopped.")
        dlog("   Duration: \(String(format: "%.1f", recordingDuration))s")
        dlog("   Frames: \(frameCount) (~\(String(format: "%.0f", Double(frameCount) / max(recordingDuration, 0.1))) fps)")
        
        // Block a new recording before the asynchronous save can yield and release its buffers.
        isSaving = true
        // Save the recording
        localSaveTask = Task {
            await saveRecording()
        }
    }
    
    // MARK: - EgoRecord Source Poses

    func recordDeviceAnchor(_ anchor: DeviceAnchor?, queryStartedTimestamp: TimeInterval,
                            queryTimestamp: TimeInterval, receivedTimestamp: TimeInterval,
                            receivedTime: Date, trackingSessionId: String) {
        guard isRecordingEgoPoses, let startTime = recordingStartTime, let anchor else { return }
        egoSequenceNumber += 1
        let relativeTime = receivedTimestamp - recordingStartMonotonicTimestamp
        let wallClockRelativeTime = receivedTime.timeIntervalSince(startTime)
        egoPoseSamples.append(EgoPoseSample(
            sequenceNumber: egoSequenceNumber,
            receivedTimestamp: receivedTimestamp,
            systemTime: receivedTime.timeIntervalSince1970,
            recordingTimestamp: relativeTime,
            wallClockRecordingTimestamp: wallClockRelativeTime,
            queryStartedTimestamp: queryStartedTimestamp,
            predictionOffset: 0,
            trackingSessionId: trackingSessionId,
            source: "head",
            timestamp: anchor.timestamp,
            updateTimestamp: nil,
            queryTimestamp: queryTimestamp,
            event: nil,
            isTracked: anchor.isTracked,
            originFromAnchorTransform: matrixToArray(anchor.originFromAnchorTransform),
            joints: nil
        ))
    }

    func recordHandAnchorUpdate(_ update: AnchorUpdate<HandAnchor>, trackingSessionId: String) {
        guard isRecordingEgoPoses, let startTime = recordingStartTime else { return }
        // Capture consumer time before reading/copying the anchor and its skeleton.
        let receivedTimestamp = CACurrentMediaTime()
        let receivedTime = Date()
        let updateTimestamp = update.timestamp
        let anchor = update.anchor
        let pose = handPoseSnapshot(anchor)
        egoSequenceNumber += 1
        let event: String
        switch update.event {
        case .added: event = "added"
        case .updated: event = "updated"
        case .removed: event = "removed"
        }
        egoPoseSamples.append(EgoPoseSample(
            sequenceNumber: egoSequenceNumber,
            receivedTimestamp: receivedTimestamp,
            systemTime: receivedTime.timeIntervalSince1970,
            recordingTimestamp: receivedTimestamp - recordingStartMonotonicTimestamp,
            wallClockRecordingTimestamp: receivedTime.timeIntervalSince(startTime),
            queryStartedTimestamp: nil,
            predictionOffset: nil,
            trackingSessionId: trackingSessionId,
            source: anchor.chirality == .left ? "leftHand" : "rightHand",
            timestamp: pose.timestamp,
            updateTimestamp: updateTimestamp,
            queryTimestamp: nil,
            event: event,
            isTracked: pose.isTracked,
            originFromAnchorTransform: pose.originFromAnchorTransform,
            joints: pose.joints
        ))
    }

    private func handPoseSnapshot(_ anchor: HandAnchor) -> EgoPoseSnapshot {
        EgoPoseSnapshot(
            timestamp: anchor.timestamp,
            isTracked: anchor.isTracked,
            originFromAnchorTransform: matrixToArray(anchor.originFromAnchorTransform),
            joints: anchor.handSkeleton?.allJoints.map { joint in
                EgoJointSample(name: joint.name.description, isTracked: joint.isTracked,
                               anchorFromJointTransform: matrixToArray(joint.anchorFromJointTransform))
            }
        )
    }

    // MARK: - Frame Recording (Video-Driven)
    
    /// Record a video frame with the current tracking data.
    /// This is VIDEO-DRIVEN: call this whenever a new video frame arrives.
    /// Non-Ego modes pair the latest tracking data with this video frame.
    nonisolated func recordVideoFrame(_ videoFrame: UIImage) {
        // Capture time immediately
        let captureTime = Date()
        
        // Dispatch to background immediately to avoid blocking
        Task.detached(priority: .userInitiated) { [weak self] in
            await self?.processVideoFrame(videoFrame, captureTime: captureTime)
        }
    }
    
    /// Process a video frame on the background queue
    private func processVideoFrame(_ videoFrame: UIImage, captureTime: Date) async {
        guard isRecording, let startTime = recordingStartTime else { return }
        
        let timestamp = max(0, captureTime.timeIntervalSince(startTime))
        let systemTime = captureTime.timeIntervalSince1970
        
        // EgoRecord saves source events independently of video frames.
        let trackingData = isEgoRecording ? nil : DataManager.shared.latestHandTrackingData
        
        // Get image dimensions
        let width = Int(videoFrame.size.width)
        let height = Int(videoFrame.size.height)
        
        // Do the rest on recording queue
        recordingQueue.async { [weak self] in
            guard let self = self else { return }
            
            let frameIndex = self.pendingFrameCount
            
            // Set up video writer on first frame
            if !self.isWriterSessionStarted {
                do {
                    let baseURL = try self.getStorageURLSync()
                    let recordingFolder = baseURL.appendingPathComponent(self.sessionID)
                    try FileManager.default.createDirectory(at: recordingFolder, withIntermediateDirectories: true)
                    self.recordingFolderURL = recordingFolder
                    
                    let videoURL = recordingFolder.appendingPathComponent("video.mp4")
                    try self.setupVideoWriter(at: videoURL, size: videoFrame.size)
                    self.assetWriter?.startSession(atSourceTime: .zero)
                    self.isWriterSessionStarted = true
                    dlog("🎬 [RecordingManager] Video writer session started")
                } catch {
                    dlog("❌ [RecordingManager] Failed to set up video writer: \(error)")
                    return
                }
            }
            
            // Create presentation time based on actual timestamp for accurate timing
            var presentationTime = CMTime(seconds: timestamp, preferredTimescale: 600)
            
            // Ensure strictly increasing timestamps (AVAssetWriter requirement)
            if let lastTime = self.lastPresentationTime {
                if presentationTime <= lastTime {
                    // If timestamp is not increasing, bump it slightly
                    presentationTime = CMTimeAdd(lastTime, CMTime(value: 1, timescale: 600))
                    // dlog("⚠️ [RecordingManager] Adjusted timestamp for frame \(frameIndex) to maintain order")
                }
            }
            self.lastPresentationTime = presentationTime
            
            // Write video frame - skip if writer isn't ready (don't block!)
            if let writer = self.assetWriter,
               let writerInput = self.videoWriterInput,
               let adaptor = self.pixelBufferAdaptor,
               writer.status == .writing,
               writerInput.isReadyForMoreMediaData {
                
                if let pixelBuffer = self.createPixelBuffer(from: videoFrame) {
                    if adaptor.append(pixelBuffer, withPresentationTime: presentationTime) {
                        // Success
                        if frameIndex % 60 == 0 {
                            dlog("DEBUG: Appended frame \(frameIndex) at \(presentationTime.seconds)s")
                        }
                    } else {
                        dlog("⚠️ [RecordingManager] Failed to append frame \(frameIndex). Writer status: \(writer.status.rawValue), Error: \(String(describing: writer.error))")
                    }
                }
            } else if self.assetWriter?.status == .writing {
                // Writer not ready, skip this frame but log occasionally
                if frameIndex % 30 == 0 {
                    dlog("⚠️ [RecordingManager] Writer busy, skipping frame \(frameIndex)")
                }
            }
            
            if let trackingData {
                // Convert tracking data
                let headMatrixArray: [Float] = self.matrixToArray(trackingData.Head)
                let leftHand = self.extractHandJointData(wrist: trackingData.leftWrist, skeleton: trackingData.leftSkeleton)
                let rightHand = self.extractHandJointData(wrist: trackingData.rightWrist, skeleton: trackingData.rightSkeleton)
            
                // Create recorded frame (without video data - that's in the MP4)
                let recordedFrame = RecordedFrame(
                    timestamp: timestamp,
                    systemTime: systemTime,
                    headMatrix: headMatrixArray,
                    leftHand: leftHand,
                    rightHand: rightHand,
                    videoFrameIndex: frameIndex,
                    videoWidth: width,
                    videoHeight: height
                )
            
                // Append to tracking data array
                self.recordedFrames.append(recordedFrame)
            }
            self.pendingFrameCount += 1
        }
    }
    
    // MARK: - Hand Tracking Only Recording
    
    /// Start recording hand tracking data only (no video/simulation required).
    /// This starts a timer that samples hand tracking at the configured Hz.
    /// Call this when you want to record hand tracking without video or simulation.
    func startHandTrackingOnlyRecording() {
        guard !isEgoRecording else { return }
        guard isRecording else {
            dlog("⚠️ [RecordingManager] Cannot start hand tracking recording - recording not active")
            return
        }
        
        // Don't start if already have a timer
        guard handTrackingRecordingTimer == nil else { return }
        
        dlog("🔴 [RecordingManager] Starting hand tracking only recording at \(handTrackingRecordingHz) Hz")
        
        let interval = 1.0 / handTrackingRecordingHz
        handTrackingRecordingTimer = Timer.scheduledTimer(withTimeInterval: interval, repeats: true) { [weak self] _ in
            Task { @MainActor in
                self?.recordHandTrackingFrame()
            }
        }
    }
    
    /// Record a single hand tracking frame (used by timer-based recording)
    private func recordHandTrackingFrame() {
        guard isRecording, !isEgoRecording, let startTime = recordingStartTime else { return }
        
        let captureTime = Date()
        let timestamp = max(0, captureTime.timeIntervalSince(startTime))
        let systemTime = captureTime.timeIntervalSince1970
        
        // Capture the latest tracking data
        let trackingData = DataManager.shared.latestHandTrackingData
        
        recordingQueue.async { [weak self] in
            guard let self = self else { return }
            
            let frameIndex = self.pendingFrameCount
            
            // Create recording folder if needed (first frame)
            if self.recordingFolderURL == nil {
                do {
                    let baseURL = try self.getStorageURLSync()
                    let recordingFolder = baseURL.appendingPathComponent(self.sessionID)
                    try FileManager.default.createDirectory(at: recordingFolder, withIntermediateDirectories: true)
                    self.recordingFolderURL = recordingFolder
                    dlog("📁 [RecordingManager] Created recording folder for hand tracking only: \(recordingFolder.lastPathComponent)")
                } catch {
                    dlog("❌ [RecordingManager] Failed to create recording folder: \(error)")
                    return
                }
            }
            
            // Convert tracking data
            let headMatrixArray: [Float] = self.matrixToArray(trackingData.Head)
            let leftHand = self.extractHandJointData(wrist: trackingData.leftWrist, skeleton: trackingData.leftSkeleton)
            let rightHand = self.extractHandJointData(wrist: trackingData.rightWrist, skeleton: trackingData.rightSkeleton)
            
            // Create recorded frame (no video data)
            let recordedFrame = RecordedFrame(
                timestamp: timestamp,
                systemTime: systemTime,
                headMatrix: headMatrixArray,
                leftHand: leftHand,
                rightHand: rightHand,
                videoFrameIndex: -1,  // No video frame
                videoWidth: 0,
                videoHeight: 0
            )
            
            self.recordedFrames.append(recordedFrame)
            self.pendingFrameCount += 1
        }
    }
    
    /// Check if we should use hand-tracking-only mode (no video/sim data source)
    /// Called periodically to detect when to start hand-tracking-only recording
    func checkAndStartHandTrackingOnlyMode() {
        guard isRecording, !isEgoRecording else { return }
        guard handTrackingRecordingTimer == nil else { return }  // Already running
        guard !isWriterSessionStarted else { return }  // Video is active
        guard simulationFrameCount == 0 else { return }  // Simulation is active
        
        // No video or simulation - start hand tracking only mode
        startHandTrackingOnlyRecording()
    }
    
    // MARK: - Simulation Data Recording
    
    /// Record simulation data (poses, qpos, ctrl) along with tracking data (hands, head)
    /// This ensures hand tracking is recorded even when video is not streaming
    nonisolated func recordSimulationData(timestamp: Double, poses: [String: [Float]], qpos: [Float]?, ctrl: [Float]?, trackingData: HandTrackingData?) {
        Task { @MainActor [weak self] in
            guard let self = self else { return }
            
            // Trigger auto-recording if needed (just like video)
            if !self.isRecording && self.autoRecordingEnabled && !self.userManuallyStopped {
                self.onFirstSimulationFrame()
            }
            
            self.processSimulationData(timestamp: timestamp, poses: poses, qpos: qpos, ctrl: ctrl, trackingData: trackingData)
        }
    }
    
    /// Called when first simulation frame is received. Starts recording if auto-recording is enabled.
    func onFirstSimulationFrame() {
        guard UserDefaults.standard.string(forKey: "appMode") != "egorecord" else { return }
        guard autoRecordingEnabled && !isRecording && !userManuallyStopped else { return }
        
        dlog("🔴 [RecordingManager] Auto-starting recording on first simulation frame")
        isAutoRecording = true
        startRecording()
    }
    
    /// Process simulation data on the main thread (to access recordingStartTime) then dispatch to recording queue
    private func processSimulationData(timestamp: Double, poses: [String: [Float]], qpos: [Float]?, ctrl: [Float]?, trackingData: HandTrackingData?) {
        guard isRecording, let startTime = recordingStartTime else { return }
        
        let relativeTimestamp = timestamp - startTime.timeIntervalSince1970
        
        // Prepare tracking data if available
        var headMatrixArray: [Float]? = nil
        var leftHand: HandJointData? = nil
        var rightHand: HandJointData? = nil
        
        if !isEgoRecording, let trackingData = trackingData {
            headMatrixArray = matrixToArray(trackingData.Head)
            leftHand = extractHandJointData(wrist: trackingData.leftWrist, skeleton: trackingData.leftSkeleton)
            rightHand = extractHandJointData(wrist: trackingData.rightWrist, skeleton: trackingData.rightSkeleton)
        }
        
        recordingQueue.async { [weak self] in
            guard let self = self else { return }
            
            // Append simulation frame
            self.simulationFrames.append(SimulationFrame(
                timestamp: relativeTimestamp,
                poses: poses,
                qpos: qpos,
                ctrl: ctrl
            ))
            self.simulationFrameCount += 1
            
            // Also append tracking frame if we have tracking data
            if let head = headMatrixArray {
                let recordedFrame = RecordedFrame(
                    timestamp: relativeTimestamp,
                    systemTime: Date().timeIntervalSince1970,
                    headMatrix: head,
                    leftHand: leftHand,
                    rightHand: rightHand,
                    videoFrameIndex: -1,  // No video frame for simulation-only recordings
                    videoWidth: 0,
                    videoHeight: 0
                )
                self.recordedFrames.append(recordedFrame)
            }
        }
    }
    
    /// Set the URL of the USDZ file to be saved with the recording
    func setUsdzUrl(_ url: URL) {
        self.usdzURL = url
    }
    
    // MARK: - Hand Data Extraction
    
    nonisolated private func matrixToArray(_ m: simd_float4x4) -> [Float] {
        [
            m.columns.0.x, m.columns.0.y, m.columns.0.z, m.columns.0.w,
            m.columns.1.x, m.columns.1.y, m.columns.1.z, m.columns.1.w,
            m.columns.2.x, m.columns.2.y, m.columns.2.z, m.columns.2.w,
            m.columns.3.x, m.columns.3.y, m.columns.3.z, m.columns.3.w
        ]
    }
    
    nonisolated private func extractHandJointData(wrist: simd_float4x4, skeleton: Skeleton) -> HandJointData? {
        let joints = skeleton.joints
        guard joints.count >= 27 else { return nil }
        
        // Joint order from 🥽AppModel.swift (indices 0-24 shared with 25-joint format):
        //   0: wrist
        //   1-4: thumb (knuckle, intermediateBase, intermediateTip, tip)
        //   5-9: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        //   10-14: middle (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        //   15-19: ring (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        //   20-24: little (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        //   25: forearmWrist
        //   26: forearmArm
        
        // Check if forearm joints are valid (not identity matrix = tracked)
        // Forearm joints may not always be tracked, so we make them optional
        let forearmWristMatrix = joints[25]
        let forearmArmMatrix = joints[26]
        
        // Check if forearm is at origin (identity matrix indicates not tracked)
        let isForearmWristTracked = forearmWristMatrix.columns.3.x != 0 || forearmWristMatrix.columns.3.y != 0 || forearmWristMatrix.columns.3.z != 0
        let isForearmArmTracked = forearmArmMatrix.columns.3.x != 0 || forearmArmMatrix.columns.3.y != 0 || forearmArmMatrix.columns.3.z != 0
        
        return HandJointData(
            wrist: matrixToArray(wrist),
            
            // Thumb (indices 1-4)
            thumbKnuckle: matrixToArray(joints[1]),
            thumbIntermediateBase: matrixToArray(joints[2]),
            thumbIntermediateTip: matrixToArray(joints[3]),
            thumbTip: matrixToArray(joints[4]),
            
            // Index (indices 5-9)
            indexMetacarpal: matrixToArray(joints[5]),
            indexKnuckle: matrixToArray(joints[6]),
            indexIntermediateBase: matrixToArray(joints[7]),
            indexIntermediateTip: matrixToArray(joints[8]),
            indexTip: matrixToArray(joints[9]),
            
            // Middle (indices 10-14)
            middleMetacarpal: matrixToArray(joints[10]),
            middleKnuckle: matrixToArray(joints[11]),
            middleIntermediateBase: matrixToArray(joints[12]),
            middleIntermediateTip: matrixToArray(joints[13]),
            middleTip: matrixToArray(joints[14]),
            
            // Ring (indices 15-19)
            ringMetacarpal: matrixToArray(joints[15]),
            ringKnuckle: matrixToArray(joints[16]),
            ringIntermediateBase: matrixToArray(joints[17]),
            ringIntermediateTip: matrixToArray(joints[18]),
            ringTip: matrixToArray(joints[19]),
            
            // Little (indices 20-24)
            littleMetacarpal: matrixToArray(joints[20]),
            littleKnuckle: matrixToArray(joints[21]),
            littleIntermediateBase: matrixToArray(joints[22]),
            littleIntermediateTip: matrixToArray(joints[23]),
            littleTip: matrixToArray(joints[24]),
            
            // Forearm joints (indices 25-26, optional - may not be tracked)
            forearmWrist: isForearmWristTracked ? matrixToArray(forearmWristMatrix) : nil,
            forearmArm: isForearmArmTracked ? matrixToArray(forearmArmMatrix) : nil
        )
    }
    
    // MARK: - Saving
    
    private var isSaveInProgress = false  // Guard against concurrent saves
    
    private func saveRecording() async {
        // Guard against concurrent save calls
        guard !isSaveInProgress else {
            dlog("⚠️ [RecordingManager] Save already in progress, ignoring duplicate call")
            return
        }
        isSaveInProgress = true
        defer {
            isSaveInProgress = false
            isSaving = false
        }
        
        // Wait for recording queue to finish processing
        await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
            recordingQueue.async {
                continuation.resume()
            }
        }
        
        guard assetWriter != nil || !recordedFrames.isEmpty || !simulationFrames.isEmpty || !egoPoseSamples.isEmpty else {
            recordingError = "No frames recorded"
            return
        }
        
        guard let recordingStartTime else {
            recordingError = "录制起始时间缺失，无法保存时间诊断。"
            return
        }
        isSaving = true
        
        dlog("💾 [RecordingManager] Saving recording to \(storageLocation.rawValue)...")
        dlog("   Frames to save: \(recordedFrames.count) (Video), \(simulationFrames.count) (Sim)")
        
        do {
            // Use the folder we already created during recording
            let recordingFolder: URL
            if let existingFolder = recordingFolderURL {
                recordingFolder = existingFolder
            } else {
                let baseURL = try getStorageURL()
                recordingFolder = baseURL.appendingPathComponent(sessionID)
                try FileManager.default.createDirectory(at: recordingFolder, withIntermediateDirectories: true)
            }
            
            // Finalize video file
            if let writer = assetWriter, writer.status == .writing {
                // Clear reference immediately to prevent other code from using it
                assetWriter = nil
                videoWriterInput?.markAsFinished()
                
                await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
                    writer.finishWriting {
                        continuation.resume()
                    }
                }
                
                if writer.status == .completed {
                    let videoURL = recordingFolder.appendingPathComponent("video.mp4")
                    if let attributes = try? FileManager.default.attributesOfItem(atPath: videoURL.path),
                       let fileSize = attributes[.size] as? Int64 {
                        let sizeMB = Double(fileSize) / (1024 * 1024)
                        dlog("🎬 [RecordingManager] Video saved: \(String(format: "%.1f", sizeMB)) MB")
                    }
                } else if let error = writer.error {
                    dlog("❌ [RecordingManager] Video writer error: \(error)")
                }
            } else {
                if let writer = assetWriter {
                    dlog("⚠️ [RecordingManager] Video writer not writing (Status: \(writer.status.rawValue)). Error: \(String(describing: writer.error))")
                } else {
                    dlog("⚠️ [RecordingManager] No asset writer active")
                }
            }
            
            // Clean up writer references (assetWriter may already be nil if we finalized above)
            assetWriter = nil
            videoWriterInput = nil
            pixelBufferAdaptor = nil
            pixelBufferPool = nil
            isWriterSessionStarted = false
            recordingFolderURL = nil
            
            if isEgoRecording && egoPoseSamples.isEmpty {
                throw RecordingError.noSourcePoses
            }

            // Get extrinsic calibration if available
            let extrinsicCalibrationDict: [String: Any]?
            if let currentCalibration = ExtrinsicCalibrationManager.shared.currentCalibration {
                extrinsicCalibrationDict = currentCalibration.toMetadataDictionary()
            } else {
                extrinsicCalibrationDict = nil
            }
            
            // Get intrinsic calibration if available
            let intrinsicCalibrationDict: [String: Any]?
            if let currentIntrinsic = CameraCalibrationManager.shared.currentCalibration {
                intrinsicCalibrationDict = currentIntrinsic.toMetadataDictionary()
            } else {
                intrinsicCalibrationDict = nil
            }
            
            // EgoRecord counts individual head/hand source records.
            let actualFrameCount = isEgoRecording ? egoPoseSamples.count : max(recordedFrames.count, simulationFrames.count)
            
            // Check if video file was actually written
            let videoURL = recordingFolder.appendingPathComponent("video.mp4")
            let hasActualVideo = FileManager.default.fileExists(atPath: videoURL.path) &&
                ((try? FileManager.default.attributesOfItem(atPath: videoURL.path)[.size] as? Int64) ?? 0) > 0
            
            // Determine recording type based on app mode and data present
            let recordingType: RecordingType
            if isEgoRecording {
                recordingType = .egorecord
            } else if let url = usdzURL {
                // Check if USDZ filename contains "isaac" to distinguish simulation types
                if url.lastPathComponent.lowercased().contains("isaac") {
                    recordingType = .teleopIsaac
                } else {
                    recordingType = .teleopMujoco
                }
            } else {
                recordingType = .teleopVideo
            }
            
            // Save metadata
            let metadata = RecordingMetadata(
                createdAt: recordingStartTime,
                duration: recordingDuration,
                frameCount: actualFrameCount,
                hasVideo: hasActualVideo,
                hasLeftHand: isEgoRecording ? egoPoseSamples.contains { $0.source == "leftHand" && $0.isTracked } : recordedFrames.contains { $0.leftHand != nil },
                hasRightHand: isEgoRecording ? egoPoseSamples.contains { $0.source == "rightHand" && $0.isTracked } : recordedFrames.contains { $0.rightHand != nil },
                hasSimulationData: !simulationFrames.isEmpty,
                hasUSDZ: usdzURL != nil,
                videoSource: DataManager.shared.videoSource.rawValue,
                averageFPS: recordingDuration > 0 ? Double(actualFrameCount) / recordingDuration : 0,
                deviceInfo: DeviceInfo(
                    model: "Apple Vision Pro",
                    systemVersion: UIDevice.current.systemVersion,
                    appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
                ),
                recordingType: recordingType,
                intrinsicCalibration: intrinsicCalibrationDict,
                extrinsicCalibration: extrinsicCalibrationDict,
                poseData: isEgoRecording ? [
                    "file": "tracking_events.jsonl",
                    "formatVersion": "3.0",
                    "frameCountUnit": "source_records",
                    "averageFPSUnit": "source_records_per_second",
                    "primaryTimestampField": "recordingTimestamp",
                    "unixTimestampField": "systemTime",
                    "arkitTimestampRole": "diagnostic_only",
                    "sequenceNumberScope": "source_records_within_recording",
                    "receivedTimestampSource": "CACurrentMediaTime_at_consumer_or_query_return",
                    "systemTimeSource": "Date.timeIntervalSince1970_at_receipt",
                    "recordingTimestampSource": "receivedTimestamp-recordingStartMonotonicTimestamp",
                    "wallClockRecordingTimestampSource": "receipt_Date-recording_start_Date; unclamped",
                    "recordingStartMonotonicTimestamp": String(recordingStartMonotonicTimestamp),
                    "recordingStartSystemTime": String(recordingStartTime.timeIntervalSince1970),
                    "sourcePoseCount": String(egoPoseSamples.count),
                    "poseAlignmentStatus": "unverified",
                    "timestampSource": "anchor.timestamp",
                    "clock": "mach_absolute_time",
                    "timeUnit": "seconds",
                    "matrixLayout": "column_major",
                    "translationUnit": "meters",
                    "handPoseSource": "anchorUpdates",
                    "headPoseSource": "queryDeviceAnchor_current_time"
                ] : nil,
                captureSessionID: isEgoRecording ? captureSessionID : nil,
                captureBoardID: isEgoRecording ? captureBoardID : nil
            )
            
            let metadataURL = recordingFolder.appendingPathComponent("metadata.json")
            let encoder = JSONEncoder()
            encoder.outputFormatting = .prettyPrinted
            let metadataData = try encoder.encode(metadata)
            try metadataData.write(to: metadataURL)
            
            // Save source events for EgoRecord, or frame snapshots for other modes.
            let trackingURL = recordingFolder.appendingPathComponent(isEgoRecording ? "tracking_events.jsonl" : "tracking.jsonl")
            let lineEncoder = JSONEncoder()
            lineEncoder.outputFormatting = .sortedKeys
            if isEgoRecording {
                var trackingData = Data()
                for sample in egoPoseSamples {
                    trackingData.append(try lineEncoder.encode(sample))
                    trackingData.append(0x0A)
                }
                try trackingData.write(to: trackingURL, options: .atomic)
            } else {
                var trackingContent = ""
                for frame in recordedFrames {
                    if let data = try? lineEncoder.encode(frame),
                       let string = String(data: data, encoding: .utf8) {
                        trackingContent += string + "\n"
                    }
                }
                try trackingContent.write(to: trackingURL, atomically: true, encoding: .utf8)
            }
            dlog("💾 [RecordingManager] Tracking data saved")

            // Save simulation data if available
            if !simulationFrames.isEmpty {
                let simURL = recordingFolder.appendingPathComponent("mjdata.jsonl")
                var simContent = ""
                
                for frame in simulationFrames {
                    if let data = try? lineEncoder.encode(frame),
                       let string = String(data: data, encoding: .utf8) {
                        simContent += string + "\n"
                    }
                }
                
                try simContent.write(to: simURL, atomically: true, encoding: .utf8)
                dlog("💾 [RecordingManager] Simulation data saved: \(simulationFrames.count) frames")
            }
            
            // Copy USDZ file if available
            if let usdzURL = usdzURL {
                let destURL = recordingFolder.appendingPathComponent("scene.usdz")
                do {
                    if FileManager.default.fileExists(atPath: destURL.path) {
                        try FileManager.default.removeItem(at: destURL)
                    }
                    try FileManager.default.copyItem(at: usdzURL, to: destURL)
                    dlog("💾 [RecordingManager] USDZ file copied")
                } catch {
                    dlog("⚠️ [RecordingManager] Failed to copy USDZ file: \(error)")
                    // Don't fail the whole save for this
                }
            }
            
            // Update metadata with actual file existence
            let finalMetadata = RecordingMetadata(
                createdAt: metadata.createdAt,
                duration: metadata.duration,
                frameCount: metadata.frameCount,
                hasVideo: metadata.hasVideo,
                hasLeftHand: metadata.hasLeftHand,
                hasRightHand: metadata.hasRightHand,
                hasSimulationData: !simulationFrames.isEmpty,
                hasUSDZ: usdzURL != nil,
                videoSource: metadata.videoSource,
                averageFPS: metadata.averageFPS,
                deviceInfo: metadata.deviceInfo,
                recordingType: metadata.recordingType,
                intrinsicCalibration: metadata.intrinsicCalibration,
                extrinsicCalibration: metadata.extrinsicCalibration,
                poseData: metadata.poseData,
                captureSessionID: metadata.captureSessionID,
                captureBoardID: metadata.captureBoardID
            )
            
            // Re-save metadata with updated flags
            let finalMetadataData = try encoder.encode(finalMetadata)
            try finalMetadataData.write(to: metadataURL)
            lastRecordingURL = recordingFolder
            recordingError = nil
            
            dlog("✅ [RecordingManager] Recording saved successfully to: \(recordingFolder.path)")
            
            if CloudStorageSettings.isEnabled && storageLocation == .cloud {
                await uploadToCloudIfNeeded(recordingFolder: recordingFolder)
            }
            
            // Clear memory
            recordedFrames.removeAll()
            videoFrames.removeAll()
            egoPoseSamples.removeAll()
            
        } catch {
            recordingError = "Failed to save: \(error.localizedDescription)"
            dlog("❌ [RecordingManager] Failed to save recording: \(error)")
            
            // Clean up writer on error
            assetWriter = nil
            videoWriterInput = nil
            pixelBufferAdaptor = nil
            pixelBufferPool = nil
            isWriterSessionStarted = false
            recordingFolderURL = nil
        }
    }
    
    /// Synchronous version of getStorageURL for use on background queue
    nonisolated private func getStorageURLSync() throws -> URL {
        let fileManager = FileManager.default
        
        // Read storage location from UserDefaults directly since we're nonisolated
        let savedLocation = UserDefaults.standard.string(forKey: "recordingStorageLocation") ?? RecordingStorageLocation.local.rawValue
        let location = RecordingStorageLocation(rawValue: savedLocation) ?? .local
        
        switch location {
        case .local:
            // Save to Documents folder - accessible via Files app
            guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .cloud:
            // Use iCloud Drive container for cloud sync
            guard let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) else {
                dlog("⚠️ [RecordingManager] iCloud not available, falling back to Documents...")
                guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                    throw RecordingError.storageNotAvailable
                }
                return url.appendingPathComponent("Recordings")
            }
            return containerURL.appendingPathComponent("Documents").appendingPathComponent("VisionProTeleop")
        }
    }
    
    private func getStorageURL() throws -> URL {
        let fileManager = FileManager.default
        
        switch storageLocation {
        case .local:
            // Save to Documents folder - accessible via Files app
            guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .cloud:
            // Use the app's iCloud container - syncs to cloud
            guard let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) else {
                dlog("⚠️ [RecordingManager] iCloud not available, falling back to Documents...")
                guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                    throw RecordingError.storageNotAvailable
                }
                return url.appendingPathComponent("Recordings")
            }
            return containerURL.appendingPathComponent("Documents").appendingPathComponent("VisionProTeleop")
        }
    }
    
    // MARK: - Utility
    
    func formatDuration(_ duration: TimeInterval) -> String {
        let hours = Int(duration) / 3600
        let minutes = (Int(duration) % 3600) / 60
        let seconds = Int(duration) % 60
        let tenths = Int((duration.truncatingRemainder(dividingBy: 1)) * 10)
        
        if hours > 0 {
            return String(format: "%d:%02d:%02d", hours, minutes, seconds)
        } else {
            return String(format: "%d:%02d.%d", minutes, seconds, tenths)
        }
    }
    
    /// Get the current storage directory URL (for display purposes)
    func getCurrentStorageURL() -> URL? {
        return try? getStorageURL()
    }
    
    /// Open the Files app at the recordings folder
    func openRecordingsInFilesApp() {
        let urlToOpen: URL
        
        if let lastURL = lastRecordingURL {
            urlToOpen = lastURL
        } else if let storageURL = try? getStorageURL() {
            urlToOpen = storageURL
        } else {
            dlog("❌ [RecordingManager] Cannot determine storage location")
            return
        }
        
        let path = urlToOpen.path
        
        if let filesURL = URL(string: "shareddocuments://\(path)") {
            Task { @MainActor in
                if await UIApplication.shared.canOpenURL(filesURL) {
                    await UIApplication.shared.open(filesURL)
                    dlog("📂 [RecordingManager] Opened Files app at: \(path)")
                } else {
                    dlog("⚠️ [RecordingManager] Cannot open shareddocuments URL, path: \(path)")
                    UIPasteboard.general.string = path
                    dlog("📋 [RecordingManager] Path copied to clipboard: \(path)")
                }
            }
        }
    }
    
    /// Get a user-friendly description of where recordings are stored
    func getStorageDescription() -> String {
        switch storageLocation {
        case .local:
            return "On My Vision Pro → Tracking Streamer → Recordings"
        case .cloud:
            return "\(cloudProvider.displayName) → VisionProTeleop"
        }
    }
    
    /// Get a description of the cloud provider setting
    func getCloudProviderDescription() -> String {
        // Refresh cloud settings
        loadCloudSettings()
        
        switch cloudProvider {
        case .iCloudDrive:
            return "iCloud Drive (default)"
        case .dropbox:
            if DropboxUploader.shared.isAvailable() {
                return "Dropbox (configured via iOS)"
            } else {
                return "Dropbox (sign in on iOS app)"
            }
        case .googleDrive:
            if GoogleDriveUploader.shared.isAvailable() {
                return "Google Drive (configured via iOS)"
            } else {
                return "Google Drive (sign in on iOS app)"
            }
        }
    }
    
    // MARK: - Cloud Upload
    
    /// Upload recording to cloud storage if configured (Dropbox or Google Drive)
    /// iCloud Drive uploads happen automatically via the file system
    private func uploadToCloudIfNeeded(recordingFolder: URL) async {
        // Refresh cloud settings in case they changed
        loadCloudSettings()
        
        dlog("☁️ [RecordingManager] Upload requested. Provider: \(cloudProvider.displayName)")
        
        // iCloud Drive doesn't need manual upload - files are in the iCloud container
        guard cloudProvider == .dropbox || cloudProvider == .googleDrive else {
            dlog("☁️ [RecordingManager] Using iCloud Drive (or none) - no manual upload needed")
            return
        }
        
        let recordingName = recordingFolder.lastPathComponent
        
        // Progress callback to update UI (now includes current filename)
        let progressCallback: (Int, Int, String) -> Void = { [weak self] current, total, currentFileName in
            Task { @MainActor in
                self?.cloudUploadCurrentFile = current
                self?.cloudUploadTotalFiles = total
                self?.cloudUploadCurrentFileName = currentFileName
                if let provider = self?.cloudProvider {
                    self?.cloudUploadProgress = "Uploading to \(provider.displayName)... (\(current)/\(total))"
                }
            }
        }
        
        if cloudProvider == .dropbox {
            // Check if Dropbox is available
            guard DropboxUploader.shared.isAvailable() else {
                dlog("⚠️ [RecordingManager] Dropbox selected but not configured - sign in via iOS app")
                return
            }
            
            isUploadingToCloud = true
            cloudUploadCurrentFile = 0
            cloudUploadTotalFiles = 0
            cloudUploadCurrentFileName = ""
            cloudUploadProgress = "Preparing upload to Dropbox..."
            
            dlog("☁️ [RecordingManager] Uploading to Dropbox...")
            
            let success = await DropboxUploader.shared.uploadRecording(
                folderURL: recordingFolder,
                recordingName: recordingName,
                progressCallback: progressCallback
            )
            
            isUploadingToCloud = false
            cloudUploadCurrentFileName = ""
            
            if success {
                cloudUploadProgress = "Uploaded to Dropbox ✓"
                dlog("✅ [RecordingManager] Successfully uploaded to Dropbox")
            } else {
                cloudUploadProgress = "Dropbox upload failed"
                dlog("❌ [RecordingManager] Failed to upload to Dropbox")
            }
        } else if cloudProvider == .googleDrive {
            // Check if Google Drive is available
            guard GoogleDriveUploader.shared.isAvailable() else {
                dlog("⚠️ [RecordingManager] Google Drive selected but not configured - sign in via iOS app")
                return
            }
            
            isUploadingToCloud = true
            cloudUploadCurrentFile = 0
            cloudUploadTotalFiles = 0
            cloudUploadCurrentFileName = ""
            cloudUploadProgress = "Preparing upload to Google Drive..."
            
            dlog("☁️ [RecordingManager] Uploading to Google Drive...")
            
            let success = await GoogleDriveUploader.shared.uploadRecording(
                folderURL: recordingFolder,
                recordingName: recordingName,
                progressCallback: progressCallback
            )
            
            isUploadingToCloud = false
            cloudUploadCurrentFileName = ""
            
            if success {
                cloudUploadProgress = "Uploaded to Google Drive ✓"
                dlog("✅ [RecordingManager] Successfully uploaded to Google Drive")
            } else {
                cloudUploadProgress = "Google Drive upload failed"
                dlog("❌ [RecordingManager] Failed to upload to Google Drive")
            }
        }
    }
}

// MARK: - Errors

enum RecordingError: Error, LocalizedError {
    case storageNotAvailable
    case encodingFailed
    case writeFailed
    case videoWriterSetupFailed
    case noSourcePoses
    
    var errorDescription: String? {
        switch self {
        case .storageNotAvailable:
            return "Storage location is not available"
        case .encodingFailed:
            return "Failed to encode recording data"
        case .writeFailed:
            return "Failed to write recording to disk"
        case .videoWriterSetupFailed:
            return "Failed to set up video writer"
        case .noSourcePoses:
            return "未采集到 ARKit 源位姿，请确认追踪正在运行后重新录制。"
        }
    }
}
