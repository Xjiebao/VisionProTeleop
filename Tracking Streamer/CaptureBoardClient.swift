import Combine
import Foundation
import Network

struct CaptureBoard: Identifiable, Equatable {
    let id: String
    let name: String
    let baseURL: URL
}

struct BoardSyncSummary: Codable {
    let networkRTTMS: Double
    let offsetVPMinusMacSeconds: Double
    let maxAbsOffsetResidualMS: Double
    let samplesRetained: Int

    enum CodingKeys: String, CodingKey {
        case networkRTTMS = "network_rtt_ms"
        case offsetVPMinusMacSeconds = "offset_vp_minus_mac_seconds"
        case maxAbsOffsetResidualMS = "max_abs_offset_residual_ms"
        case samplesRetained = "samples_retained"
    }
}

struct BoardCalibrationStatus: Codable {
    let state: String
    let sessionID: String
    let error: String?
    let sampleCount: Int?
    let validationCount: Int?
    let validationTranslationMM: Double?
    let validationRotationDegrees: Double?
    let active: Bool
    let warnings: [String]?

    enum CodingKeys: String, CodingKey {
        case state, sessionID = "session_id", error, active, warnings
        case sampleCount = "sample_count", validationCount = "validation_count"
        case validationTranslationMM = "validation_translation_mm"
        case validationRotationDegrees = "validation_rotation_deg"
    }
}

struct BoardLivePreview: Codable {
    let boardID: String
    let state: String
    let recording: Bool
    let sessionID: String?
    let ageSeconds: Double?
    let frameTime: Double?
    let imageBase64: String?
    let leftDetected: Bool
    let rightDetected: Bool
    let stableSeconds: Double
    let poseCount: Int
    let guidance: String
    let error: String?

    enum CodingKeys: String, CodingKey {
        case boardID = "board_id", state, recording, sessionID = "session_id"
        case ageSeconds = "age_seconds", frameTime = "frame_time", imageBase64 = "image_base64"
        case leftDetected = "left_detected", rightDetected = "right_detected"
        case stableSeconds = "stable_seconds", poseCount = "pose_count", guidance, error
    }

    func isFresh(receivedAt: Date, now: Date) -> Bool {
        guard state == "live", let ageSeconds else { return false }
        return ageSeconds + max(0, now.timeIntervalSince(receivedAt)) <= 3
    }
}

struct BoardStatus: Codable {
    let boardID: String
    let boardName: String
    let state: String
    let sessionID: String?
    let error: String?
    let vpUploaded: Bool
    let reviewState: String?
    let syncSummary: BoardSyncSummary?
    let syncAfterFile: String?
    let calibration: BoardCalibrationStatus?
    let storageFreeBytes: Int64?
    let storageWarning: String?

    enum CodingKeys: String, CodingKey {
        case boardID = "board_id", boardName = "board_name", state
        case sessionID = "session_id", error, vpUploaded = "vp_uploaded", reviewState = "review_state"
        case syncSummary = "sync_summary", syncAfterFile = "sync_after"
        case calibration
        case storageFreeBytes = "storage_free_bytes", storageWarning = "storage_warning"
    }
}

enum CaptureBoardError: LocalizedError {
    case message(String)
    case http(statusCode: Int, message: String)

    var statusCode: Int? {
        if case .http(let code, _) = self { return code }
        return nil
    }

    var errorDescription: String? {
        switch self {
        case .message(let message), .http(_, let message): return message
        }
    }
}

enum BoardUploadState {
    case idle, uploading, completed, failed
}

/// Discovers boards and transports commands/files. Recording order belongs to RecordingManager.
@MainActor
final class CaptureBoardClient: ObservableObject {
    static let shared = CaptureBoardClient()

    @Published private(set) var boards: [CaptureBoard] = []
    @Published var selectedBoardID: String? {
        didSet {
            guard oldValue != selectedBoardID else { return }
            UserDefaults.standard.set(selectedBoardID, forKey: "captureBoardID")
            lastStatus = nil
            updateConnectionStatus()
            if let boardID = selectedBoardID, selectedBoard != nil {
                Task { try? await status(boardID: boardID) }
            }
        }
    }
    @Published private(set) var connectionStatus = "正在搜索采集板…"
    @Published private(set) var lastStatus: BoardStatus?
    @Published private(set) var isUploading = false
    @Published private(set) var uploadProgress = 0.0
    @Published private(set) var uploadStatus = "尚未传输"
    @Published private(set) var uploadState: BoardUploadState = .idle
    @Published private(set) var uploadSessionID: String?

    var selectedBoard: CaptureBoard? { boards.first { $0.id == selectedBoardID } }

    private struct ErrorResponse: Decodable { let error: String }
    private struct SessionRequest: Encodable { let session_id: String }

    private var browser: NWBrowser?
    private var discovered: [String: NWBrowser.Result] = [:]
    private var resolutions: [String: NWConnection] = [:]
    private var resolutionTimeouts: [String: Task<Void, Never>] = [:]
    private var selectionTask: Task<Void, Never>?
    private var activeUploads: Set<String> = []
    private var displayedUpload: String?
    private let session: URLSession

    private init() {
        selectedBoardID = UserDefaults.standard.string(forKey: "captureBoardID")
        let configuration = URLSessionConfiguration.ephemeral
        configuration.timeoutIntervalForRequest = 35
        configuration.timeoutIntervalForResource = 1800
        configuration.requestCachePolicy = .reloadIgnoringLocalCacheData
        session = URLSession(configuration: configuration)
    }

    func startDiscovery() {
        guard browser == nil else { return }
        let browser = NWBrowser(for: .bonjourWithTXTRecord(type: "_egocapture._tcp", domain: "local."), using: .tcp)
        self.browser = browser
        browser.stateUpdateHandler = { [weak self, weak browser] state in
            Task { @MainActor in
                guard let self, let browser, self.browser === browser else { return }
                switch state {
                case .failed(let error), .waiting(let error):
                    self.connectionStatus = "搜索采集板失败：\(error.localizedDescription)"
                default: break
                }
            }
        }
        browser.browseResultsChangedHandler = { [weak self, weak browser] results, _ in
            Task { @MainActor in
                guard let self, let browser, self.browser === browser else { return }
                self.updateDiscoveries(results)
            }
        }
        browser.start(queue: .main)
        updateConnectionStatus()
    }

    func stopDiscovery() {
        browser?.cancel()
        browser = nil
        selectionTask?.cancel()
        for connection in resolutions.values { connection.cancel() }
        for timeout in resolutionTimeouts.values { timeout.cancel() }
        resolutions.removeAll()
        resolutionTimeouts.removeAll()
        discovered.removeAll()
        boards.removeAll()
        lastStatus = nil
        connectionStatus = "搜索已停止"
    }

    func refreshDiscovery() {
        stopDiscovery()
        startDiscovery()
    }

    private func updateDiscoveries(_ results: Set<NWBrowser.Result>) {
        var current: [String: NWBrowser.Result] = [:]
        for result in results {
            guard case .bonjour(let txt) = result.metadata,
                  let id = txt["id"], !id.isEmpty else { continue }
            current[id] = result
        }
        let changed = Set(current.keys).filter { discovered[$0] != current[$0] }
        for id in Set(discovered.keys).subtracting(current.keys).union(changed) {
            resolutions.removeValue(forKey: id)?.cancel()
            resolutionTimeouts.removeValue(forKey: id)?.cancel()
            boards.removeAll { $0.id == id }
        }
        discovered = current
        for (id, result) in current where !boards.contains(where: { $0.id == id }) && resolutions[id] == nil {
            resolve(id: id, result: result)
        }
        updateConnectionStatus()
        scheduleSingleBoardSelection()
    }

    private func resolve(id: String, result: NWBrowser.Result) {
        let parameters = NWParameters.tcp
        (parameters.defaultProtocolStack.internetProtocol as? NWProtocolIP.Options)?.version = .v4
        let connection = NWConnection(to: result.endpoint, using: parameters)
        resolutions[id] = connection
        connection.stateUpdateHandler = { [weak self, weak connection] state in
            Task { @MainActor in
                guard let self, let connection, self.resolutions[id] === connection else { return }
                switch state {
                case .ready:
                    guard case .hostPort(let host, let port) = connection.currentPath?.remoteEndpoint,
                          case .ipv4(let address) = host,
                          let url = URL(string: "http://\(address.rawValue.map { String($0) }.joined(separator: ".")):\(port.rawValue)") else {
                        self.finishResolution(id: id)
                        self.connectionStatus = "采集板未提供可连接的 IPv4 地址"
                        return
                    }
                    var name = id
                    if case .bonjour(let txt) = result.metadata { name = txt["name"] ?? id }
                    self.boards.removeAll { $0.id == id }
                    self.boards.append(CaptureBoard(id: id, name: name, baseURL: url))
                    self.boards.sort { $0.name == $1.name ? $0.id < $1.id : $0.name < $1.name }
                    self.finishResolution(id: id)
                    self.updateConnectionStatus()
                    self.scheduleSingleBoardSelection()
                    if self.selectedBoardID == id { Task { try? await self.status(boardID: id) } }
                case .failed(let error):
                    self.finishResolution(id: id)
                    self.connectionStatus = "采集板连接失败：\(error.localizedDescription)"
                default: break
                }
            }
        }
        connection.start(queue: .main)
        resolutionTimeouts[id] = Task { @MainActor [weak self, weak connection] in
            do { try await Task.sleep(nanoseconds: 8_000_000_000) } catch { return }
            guard let self, let connection, self.resolutions[id] === connection else { return }
            self.finishResolution(id: id)
            self.connectionStatus = "采集板连接超时，请检查网络后重新搜索"
        }
    }

    private func finishResolution(id: String) {
        resolutionTimeouts.removeValue(forKey: id)?.cancel()
        resolutions.removeValue(forKey: id)?.cancel()
    }

    private func scheduleSingleBoardSelection() {
        selectionTask?.cancel()
        guard selectedBoardID == nil, discovered.count == 1, boards.count == 1 else { return }
        selectionTask = Task { @MainActor [weak self] in
            do { try await Task.sleep(nanoseconds: 1_000_000_000) } catch { return }
            guard let self, self.selectedBoardID == nil, self.discovered.count == 1, self.boards.count == 1 else { return }
            self.selectedBoardID = self.boards[0].id
        }
    }

    private func updateConnectionStatus() {
        if let board = selectedBoard {
            connectionStatus = "已发现：\(board.name)"
        } else if selectedBoardID != nil {
            lastStatus = nil
            connectionStatus = "正在搜索已选采集板…"
        } else if boards.count > 1 {
            connectionStatus = "发现多块采集板，请选择"
        } else {
            connectionStatus = "正在搜索采集板…"
        }
    }

    func status(sessionID: String? = nil, boardID: String? = nil) async throws -> BoardStatus {
        try await request(method: "GET", path: "status", boardID: boardID, querySessionID: sessionID)
    }

    func checkClock(boardID: String? = nil) async throws -> BoardStatus {
        try await request(method: "POST", path: "sync", boardID: boardID)
    }

    func prepare(sessionID: String, boardID: String? = nil) async throws -> BoardStatus {
        let body = try JSONEncoder().encode(SessionRequest(session_id: sessionID))
        return try await request(method: "POST", path: "sessions", boardID: boardID, body: body, timeoutInterval: 90)
    }

    func start(sessionID: String, boardID: String) async throws -> BoardStatus {
        try await request(method: "POST", path: "sessions/\(sessionID)/start", boardID: boardID)
    }

    func stop(sessionID: String, boardID: String) async throws -> BoardStatus {
        try await request(method: "POST", path: "sessions/\(sessionID)/stop", boardID: boardID, timeoutInterval: 90)
    }

    func syncAfter(sessionID: String, boardID: String) async throws -> BoardStatus {
        try await request(method: "POST", path: "sessions/\(sessionID)/sync-after", boardID: boardID)
    }

    func review(sessionID: String, boardID: String, decision: String) async throws -> BoardStatus {
        let body = try JSONEncoder().encode(["decision": decision])
        return try await request(method: "POST", path: "sessions/\(sessionID)/review", boardID: boardID, body: body)
    }

    func calibrate(sessionID: String, boardID: String) async throws -> BoardStatus {
        try await request(method: "POST", path: "sessions/\(sessionID)/calibrate", boardID: boardID)
    }

    func applyCalibration(sessionID: String, boardID: String) async throws -> BoardStatus {
        try await request(method: "POST", path: "sessions/\(sessionID)/apply-calibration", boardID: boardID)
    }

    func calibrationPreview(sessionID: String, boardID: String) async throws -> Data {
        guard let board = boards.first(where: { $0.id == boardID }) else {
            throw CaptureBoardError.message("未找到本轮绑定的采集板，请接入同一网络后重新搜索")
        }
        let url = board.baseURL.appendingPathComponent("sessions/\(sessionID)/calibration-preview")
        var request = URLRequest(url: url, timeoutInterval: 35)
        request.httpMethod = "GET"
        request.setValue(boardID, forHTTPHeaderField: "X-Capture-Board-ID")
        request.setValue("image/jpeg", forHTTPHeaderField: "Accept")
        do {
            let (data, response) = try await session.data(for: request)
            guard let http = response as? HTTPURLResponse else { throw CaptureBoardError.message("采集板返回了无效响应") }
            guard (200..<300).contains(http.statusCode) else {
                let message = (try? JSONDecoder().decode(ErrorResponse.self, from: data).error) ?? "采集板请求失败（HTTP \(http.statusCode)）"
                throw CaptureBoardError.http(statusCode: http.statusCode, message: message)
            }
            return data
        } catch {
            if Task.isCancelled || (error as? URLError)?.code == .cancelled { throw error }
            if selectedBoardID == boardID { connectionStatus = "读取标定预览失败：\(error.localizedDescription)" }
            throw error
        }
    }

    /// Live preview failures stay local to the calibration page, without changing discovery or capture state.
    func liveCalibrationPreview(boardID: String, method: String = "GET", stop: Bool = false) async throws -> BoardLivePreview {
        guard let board = boards.first(where: { $0.id == boardID }) else {
            throw CaptureBoardError.message("未找到本轮绑定的采集板")
        }
        let path = stop ? "calibration/preview/stop" : "calibration/preview"
        var request = URLRequest(url: board.baseURL.appendingPathComponent(path), timeoutInterval: 5)
        request.httpMethod = method
        request.setValue(boardID, forHTTPHeaderField: "X-Capture-Board-ID")
        request.setValue("application/json", forHTTPHeaderField: "Accept")
        if method == "POST" {
            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
            request.httpBody = Data("{}".utf8)
        }
        let (data, response) = try await session.data(for: request)
        guard let http = response as? HTTPURLResponse else {
            throw CaptureBoardError.message("采集板返回了无效响应")
        }
        guard (200..<300).contains(http.statusCode) else {
            let message = (try? JSONDecoder().decode(ErrorResponse.self, from: data).error)
                ?? "预览请求失败（HTTP \(http.statusCode)）"
            throw CaptureBoardError.http(statusCode: http.statusCode, message: message)
        }
        let preview = try JSONDecoder().decode(BoardLivePreview.self, from: data)
        guard preview.boardID == boardID else {
            throw CaptureBoardError.message("预览与所选采集板不一致")
        }
        return preview
    }

    func failUpload(sessionID: String, error: Error) {
        uploadSessionID = sessionID
        uploadState = .failed
        uploadStatus = "传输未完成：\(error.localizedDescription)"
    }

    /// File uploads suspend independently of recording controls and retain the original VP files.
    func uploadRecording(folder: URL, sessionID: String, boardID: String) async throws -> BoardStatus {
        let key = "\(boardID)/\(sessionID)"
        guard activeUploads.insert(key).inserted else { throw CaptureBoardError.message("本轮文件正在传输") }
        displayedUpload = key
        uploadSessionID = sessionID
        uploadState = .uploading
        isUploading = true
        uploadProgress = 0
        defer {
            activeUploads.remove(key)
            isUploading = !activeUploads.isEmpty
        }
        do {
            var result: BoardStatus?
            let filenames = ["metadata.json", "tracking_events.jsonl"]
            for (index, filename) in filenames.enumerated() {
                if displayedUpload == key { uploadStatus = "正在传输 \(sessionID) · \(filename)" }
                result = try await request(method: "PUT", path: "sessions/\(sessionID)/files/\(filename)", boardID: boardID, file: folder.appendingPathComponent(filename))
                if displayedUpload == key { uploadProgress = Double(index + 1) / Double(filenames.count) }
            }
            guard let result, result.vpUploaded else { throw CaptureBoardError.message("采集板尚未确认本轮文件接收完整") }
            if displayedUpload == key {
                uploadState = .completed
                uploadStatus = "采集板已确认完整收到本轮 VP 数据"
            }
            return result
        } catch {
            if displayedUpload == key { failUpload(sessionID: sessionID, error: error) }
            throw error
        }
    }

    private func request(method: String, path: String, boardID: String?, querySessionID: String? = nil, body: Data? = nil, file: URL? = nil, timeoutInterval: TimeInterval = 35) async throws -> BoardStatus {
        guard let id = boardID ?? selectedBoardID else { throw CaptureBoardError.message("请先选择采集板") }
        guard let board = boards.first(where: { $0.id == id }) else { throw CaptureBoardError.message("未找到本轮绑定的采集板，请接入同一网络后重新搜索") }
        var components = URLComponents(url: board.baseURL.appendingPathComponent(path), resolvingAgainstBaseURL: false)!
        if let querySessionID { components.queryItems = [URLQueryItem(name: "session_id", value: querySessionID)] }
        var request = URLRequest(url: components.url!, timeoutInterval: file == nil ? timeoutInterval : 1800)
        request.httpMethod = method
        request.setValue(id, forHTTPHeaderField: "X-Capture-Board-ID")
        request.setValue("application/json", forHTTPHeaderField: "Accept")
        request.setValue(file?.pathExtension == "jsonl" ? "application/x-ndjson" : "application/json", forHTTPHeaderField: "Content-Type")
        do {
            let data: Data
            let response: URLResponse
            if let file {
                let attributes = try FileManager.default.attributesOfItem(atPath: file.path)
                guard let size = attributes[.size] as? NSNumber else { throw CaptureBoardError.message("无法读取待传输文件大小") }
                request.setValue(size.stringValue, forHTTPHeaderField: "Content-Length")
                (data, response) = try await session.upload(for: request, fromFile: file)
            } else {
                if method == "POST" { request.httpBody = body ?? Data("{}".utf8) }
                (data, response) = try await session.data(for: request)
            }
            guard let http = response as? HTTPURLResponse else { throw CaptureBoardError.message("采集板返回了无效响应") }
            guard (200..<300).contains(http.statusCode) else {
                let message = (try? JSONDecoder().decode(ErrorResponse.self, from: data).error) ?? "采集板请求失败（HTTP \(http.statusCode)）"
                throw CaptureBoardError.http(statusCode: http.statusCode, message: message)
            }
            let status = try JSONDecoder().decode(BoardStatus.self, from: data)
            guard status.boardID == id else { throw CaptureBoardError.message("采集板身份与本轮不一致，请重新搜索") }
            if selectedBoardID == id, file == nil {
                lastStatus = status
                connectionStatus = "已连接：\(status.boardName)"
            }
            return status
        } catch {
            if Task.isCancelled || (error as? URLError)?.code == .cancelled { throw error }
            if selectedBoardID == id, file == nil { connectionStatus = "采集板通信失败：\(error.localizedDescription)" }
            // A stale address must be rediscovered; ambiguous commands are never retried here.
            if error is URLError { refreshDiscovery() }
            throw error
        }
    }
}
