import Foundation
import AVFoundation
import CoreMedia
import CoreVideo
import AppKit
import Darwin

// macOS 14+. Video PTS is not a verified hardware exposure timestamp.
struct CaptureError: LocalizedError {
    let message: String
    var errorDescription: String? { message }
    init(_ message: String) { self.message = message }
}

func hostSeconds() -> Double {
    CMTimeGetSeconds(CMClockGetTime(CMClockGetHostTimeClock()))
}

func appendJSON(_ object: [String: Any], to file: FileHandle) throws {
    var data = try JSONSerialization.data(withJSONObject: object, options: [.sortedKeys])
    data.append(10)
    try file.write(contentsOf: data)
}

func createLog(_ url: URL) throws -> FileHandle {
    guard FileManager.default.createFile(atPath: url.path, contents: nil) else {
        throw CaptureError("无法创建 \(url.path)")
    }
    return try FileHandle(forWritingTo: url)
}

func cameraDevices() -> [AVCaptureDevice] {
    AVCaptureDevice.DiscoverySession(
        deviceTypes: [.external, .builtInWideAngleCamera], mediaType: .video,
        position: .unspecified
    ).devices.sorted { $0.uniqueID < $1.uniqueID }
}

func fullFormats(_ device: AVCaptureDevice) -> [AVCaptureDevice.Format] {
    device.formats.filter {
        let size = CMVideoFormatDescriptionGetDimensions($0.formatDescription)
        return size.width == 4000 && size.height == 1200
    }
}

func listCamerasJSON() throws {
    let devices: [[String: Any]] = cameraDevices().map { device in
        ["name": device.localizedName,
         "unique_id": device.uniqueID,
         "supports_4000x1200": !fullFormats(device).isEmpty,
         "external": device.deviceType == .external]
    }
    var data = try JSONSerialization.data(withJSONObject: devices, options: [.sortedKeys])
    data.append(10)
    FileHandle.standardOutput.write(data)
}

func listCameras() {
    let authorization = AVCaptureDevice.authorizationStatus(for: .video)
    let status: String
    switch authorization {
    case .authorized: status = "已授权"
    case .notDetermined: status = "尚未申请（--list 不申请权限）"
    case .denied: status = "已拒绝"
    case .restricted: status = "受系统限制"
    @unknown default: status = "未知"
    }
    print("相机权限：\(status)")
    let devices = cameraDevices()
    if devices.isEmpty { print("没有发现摄像头。检查系统信息中的 USB 设备及相机权限。") }
    for (index, device) in devices.enumerated() {
        let formats = fullFormats(device)
        let rates = formats.flatMap { $0.videoSupportedFrameRateRanges }.map {
            String(format: "%.8f–%.8f", $0.minFrameRate, $0.maxFrameRate)
        }
        print("[\(index)] \(device.localizedName)\n    uniqueID: \(device.uniqueID)")
        print("    4000×1200: \(formats.isEmpty ? "不支持" : "支持，fps " + Set(rates).sorted().joined(separator: ", "))")
    }
}

// All methods run on one serial capture queue. Append original sample buffers;
// startSession makes the movie timeline begin at the first accepted source PTS.
final class VideoLogWriter {
    let writer: AVAssetWriter
    let input: AVAssetWriterInput
    let frames: FileHandle
    private(set) var frameCount = 0
    private(set) var backpressureDrops = 0
    private var firstPTS: CMTime?
    private var previousPTS: CMTime?
    var durationSeconds: Double {
        guard let firstPTS, let previousPTS else { return 0 }
        return CMTimeGetSeconds(CMTimeSubtract(previousPTS, firstPTS))
    }

    init(out: URL, fps: Double) throws {
        frames = try createLog(out.appendingPathComponent("frames.jsonl"))
        writer = try AVAssetWriter(outputURL: out.appendingPathComponent("video.mov"), fileType: .mov)
        writer.movieTimeScale = 1_000_000_000
        input = AVAssetWriterInput(mediaType: .video, outputSettings: [
            AVVideoCodecKey: AVVideoCodecType.h264,
            AVVideoWidthKey: 4000, AVVideoHeightKey: 1200,
            AVVideoCompressionPropertiesKey: [
                AVVideoAverageBitRateKey: 60_000_000,
                AVVideoExpectedSourceFrameRateKey: fps,
                AVVideoAllowFrameReorderingKey: false,
                AVVideoMaxKeyFrameIntervalKey: max(1, Int(fps.rounded()))
            ]
        ])
        input.mediaTimeScale = 1_000_000_000
        input.expectsMediaDataInRealTime = true
        guard writer.canAdd(input) else { throw CaptureError("无法创建 4000×1200 H.264 视频编码器") }
        writer.add(input)
    }

    func append(_ sample: CMSampleBuffer, captureHost: Double, receivedHost: Double,
                receivedUnix: Double) throws {
        let pts = CMSampleBufferGetPresentationTimeStamp(sample)
        guard pts.isNumeric, CMTimeGetSeconds(pts).isFinite,
              captureHost.isFinite, receivedHost.isFinite, receivedUnix.isFinite,
              let image = CMSampleBufferGetImageBuffer(sample),
              CVPixelBufferGetWidth(image) == 4000, CVPixelBufferGetHeight(image) == 1200 else {
            throw CaptureError("帧时间无效，或实际图像不是 4000×1200；停止录制")
        }
        if let previousPTS, CMTimeCompare(pts, previousPTS) <= 0 {
            throw CaptureError("采集 PTS 没有严格递增；停止录制")
        }
        if writer.status == .unknown {
            guard writer.startWriting() else {
                throw writer.error ?? CaptureError("视频写入器启动失败")
            }
        }
        guard writer.status == .writing else {
            throw writer.error ?? CaptureError("视频写入器退出 writing 状态")
        }
        guard input.isReadyForMoreMediaData else { backpressureDrops += 1; return }
        if firstPTS == nil { writer.startSession(atSourceTime: pts) }
        guard input.append(sample) else { throw writer.error ?? CaptureError("视频帧写入失败") }
        if firstPTS == nil { firstPTS = pts }
        previousPTS = pts
        try appendJSON([
            "frame_index": frameCount,
            "pts_seconds": CMTimeGetSeconds(CMTimeSubtract(pts, firstPTS!)),
            "systemTime": receivedUnix + (captureHost - receivedHost),
            "capture_host_seconds": captureHost,
            "received_host_seconds": receivedHost
        ], to: frames)
        frameCount += 1
    }

    func finish(_ completion: @escaping (Error?) -> Void) {
        do { try frames.synchronize(); try frames.close() }
        catch { writer.cancelWriting(); completion(error); return }
        guard frameCount > 0 else {
            writer.cancelWriting()
            completion(CaptureError("没有保存任何视频帧"))
            return
        }
        guard writer.status == .writing else {
            completion(writer.error ?? CaptureError("视频写入没有完成"))
            return
        }
        input.markAsFinished()
        writer.finishWriting { [self] in
            completion(writer.status == .completed ? nil : writer.error ?? CaptureError("视频封装失败"))
        }
    }
}

final class Recorder: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    let session = AVCaptureSession()
    let queue = DispatchQueue(label: "calibration.video", qos: .userInitiated)
    let video: VideoLogWriter
    let device: AVCaptureDevice
    let fps: Double
    let frameDuration: CMTime
    let format: AVCaptureDevice.Format
    var onFailure: ((String) -> Void)?
    private(set) var captureDrops = 0
    private var failed = false
    private(set) var lastFrameHost: Double?
    private(set) var capturedFrameCount = 0
    private(set) var firstCaptureHost: Double?
    private(set) var lastCaptureHost: Double?

    init(device: AVCaptureDevice, out: URL) throws {
        self.device = device
        let candidates: [(AVCaptureDevice.Format, Double, CMTime)] = fullFormats(device).flatMap { format in
            format.videoSupportedFrameRateRanges.map { range in
                let rate = min(max(30, range.minFrameRate), range.maxFrameRate)
                let duration: CMTime
                if rate == range.maxFrameRate { duration = range.minFrameDuration }
                else if rate == range.minFrameRate { duration = range.maxFrameDuration }
                else { duration = CMTime(value: 1, timescale: 30) }
                return (format, rate, duration)
            }
        }
        guard let selected = candidates.min(by: { abs($0.1 - 30) < abs($1.1 - 30) }) else {
            throw CaptureError("所选相机没有可用的 4000×1200 帧率模式")
        }
        format = selected.0
        fps = selected.1
        frameDuration = selected.2
        video = try VideoLogWriter(out: out, fps: fps)
        super.init()
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        let cameraInput = try AVCaptureDeviceInput(device: device)
        guard session.canAddInput(cameraInput) else { throw CaptureError("无法接入所选相机") }
        session.addInput(cameraInput)
        try device.lockForConfiguration()
        device.activeFormat = format
        device.activeVideoMinFrameDuration = frameDuration
        device.activeVideoMaxFrameDuration = frameDuration
        device.unlockForConfiguration()
        let output = AVCaptureVideoDataOutput()
        output.alwaysDiscardsLateVideoFrames = true
        output.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange]
        output.setSampleBufferDelegate(self, queue: queue)
        guard session.canAddOutput(output) else { throw CaptureError("无法配置视频输出") }
        session.addOutput(output)
        if let connection = output.connection(with: .video) {
            if connection.isVideoMirroringSupported {
                connection.automaticallyAdjustsVideoMirroring = false
                connection.isVideoMirrored = false
            }
            if connection.isVideoRotationAngleSupported(0) { connection.videoRotationAngle = 0 }
        }
    }

    func start() throws {
        // On macOS there is no inputPriority preset. Hold the configuration
        // lock across startRunning so session startup cannot replace this mode.
        try device.lockForConfiguration()
        defer { device.unlockForConfiguration() }
        device.activeFormat = format
        device.activeVideoMinFrameDuration = frameDuration
        device.activeVideoMaxFrameDuration = frameDuration
        session.startRunning()
        let size = CMVideoFormatDescriptionGetDimensions(device.activeFormat.formatDescription)
        guard session.isRunning, size.width == 4000, size.height == 1200 else {
            throw CaptureError("采集启动失败，或系统把相机格式改成了非 4000×1200")
        }
    }

    func captureOutput(_ output: AVCaptureOutput, didOutput sample: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        let received = hostSeconds()
        let receivedUnix = Date().timeIntervalSince1970
        guard !failed else { return }
        do {
            let pts = CMSampleBufferGetPresentationTimeStamp(sample)
            guard pts.isNumeric, let sourceClock = session.synchronizationClock else {
                throw CaptureError("缺少有效的采集 PTS 或 session.synchronizationClock")
            }
            let hostTime = CMSyncConvertTime(pts, from: sourceClock, to: CMClockGetHostTimeClock())
            guard hostTime.isNumeric else { throw CaptureError("无法把采集 PTS 转换到 Mac 主机时钟") }
            let captureHost = CMTimeGetSeconds(hostTime)
            try video.append(sample, captureHost: captureHost, receivedHost: received,
                             receivedUnix: receivedUnix)
            if firstCaptureHost == nil { firstCaptureHost = captureHost }
            lastCaptureHost = captureHost
            capturedFrameCount += 1
            lastFrameHost = received
        } catch {
            failed = true
            DispatchQueue.main.async { [weak self] in self?.onFailure?(error.localizedDescription) }
        }
    }

    func captureOutput(_ output: AVCaptureOutput, didDrop sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) { captureDrops += 1 }
}

final class PreviewWindow: NSObject, NSWindowDelegate {
    let window: NSWindow
    var onClose: (() -> Void)?

    init(session: AVCaptureSession) {
        window = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 1100, height: 330),
                          styleMask: [.titled, .closable, .miniaturizable, .resizable],
                          backing: .buffered, defer: false)
        super.init()
        window.title = "采集预览：码带 | 右眼 | 左眼 — 关闭窗口停止录制"
        window.isReleasedWhenClosed = false
        window.delegate = self
        let preview = AVCaptureVideoPreviewLayer(session: session)
        preview.videoGravity = .resizeAspect
        if let connection = preview.connection {
            if connection.isVideoMirroringSupported {
                connection.automaticallyAdjustsVideoMirroring = false
                connection.isVideoMirrored = false
            }
            if connection.isVideoRotationAngleSupported(0) { connection.videoRotationAngle = 0 }
        }
        window.contentView?.wantsLayer = true
        window.contentView?.layer = preview
        window.center()
        window.makeKeyAndOrderFront(nil)
    }

    func windowWillClose(_ notification: Notification) { onClose?() }
}

func main() throws {
    let args = Array(CommandLine.arguments.dropFirst())
    if args == ["--list"] { listCameras(); return }
    if args == ["--list-json"] { try listCamerasJSON(); return }
    if args.isEmpty || args == ["--help"] {
        print("列出相机：record_calibration --list 或 --list-json\n录制：record_calibration --out 新会话目录 [--camera 编号或uniqueID]\n未指定相机时，自动选择唯一支持 4000×1200 的外置 DECXIN。\n独立录像，无需 VP 或网络；录制中按 Enter 或 Ctrl-C 停止。")
        return
    }
    guard args.count == 2 || args.count == 4 else {
        throw CaptureError("需要 --out 参数，可选 --camera；用 --help 查看")
    }
    var options: [String: String] = [:]
    for index in stride(from: 0, to: args.count, by: 2) {
        guard ["--camera", "--out"].contains(args[index]), options[args[index]] == nil else {
            throw CaptureError("未知或重复参数：\(args[index])")
        }
        options[args[index]] = args[index + 1]
    }
    guard let path = options["--out"] else {
        throw CaptureError("必须指定 --out 输出目录")
    }
    let devices = cameraDevices()
    let device: AVCaptureDevice
    if let camera = options["--camera"] {
        let selected: AVCaptureDevice?
        if let index = Int(camera), devices.indices.contains(index) { selected = devices[index] }
        else { selected = devices.first { $0.uniqueID == camera } }
        guard let selected else { throw CaptureError("找不到指定相机；先运行 --list") }
        device = selected
    } else {
        let candidates = devices.filter {
            $0.deviceType == .external &&
            $0.localizedName.localizedCaseInsensitiveContains("DECXIN") &&
            !fullFormats($0).isEmpty
        }
        guard candidates.count == 1 else {
            throw CaptureError(candidates.isEmpty
                ? "没有找到支持 4000×1200 的外置 DECXIN 相机；运行 --list 检查连接，或用 --camera 指定相机"
                : "发现多台支持 4000×1200 的外置 DECXIN 相机；运行 --list 后用 --camera 指定相机")
        }
        device = candidates[0]
    }
    guard !fullFormats(device).isEmpty else { throw CaptureError("所选相机 \(device.localizedName) 不支持 4000×1200") }
    if AVCaptureDevice.authorizationStatus(for: .video) == .notDetermined {
        let permission = DispatchSemaphore(value: 0)
        AVCaptureDevice.requestAccess(for: .video) { _ in permission.signal() }
        permission.wait()
    }
    guard AVCaptureDevice.authorizationStatus(for: .video) == .authorized else {
        throw CaptureError("没有相机权限。请在系统设置 → 隐私与安全性 → 相机中允许本启动程序或终端。")
    }
    let out = URL(fileURLWithPath: NSString(string: path).expandingTildeInPath, isDirectory: true)
    if FileManager.default.fileExists(atPath: out.path),
       !(try FileManager.default.contentsOfDirectory(atPath: out.path)).isEmpty {
        throw CaptureError("输出目录已有文件，请选择新的会话目录")
    }
    try FileManager.default.createDirectory(at: out, withIntermediateDirectories: true)
    let metaURL = out.appendingPathComponent("session.json")
    var metadata: [String: Any] = [
        "schema_version": 2, "status": "starting", "video": "video.mov",
        "device_name": device.localizedName, "device_unique_id": device.uniqueID,
        "width": 4000, "height": 1200, "capture_mode": "independent",
        "host_clock": "CMClockGetHostTimeClock",
        "alignment_clock": "unix", "frame_unix_field": "systemTime",
        "systemTime": "Unix seconds: callback Date + (sample PTS converted to HostClock - callback HostClock)",
        "capture_host_seconds": "CMSyncConvertTime(sample PTS, session.synchronizationClock, CMClockGetHostTimeClock)",
        "pts_seconds": "original sample PTS minus first written sample PTS; movie begins at zero",
        "timestamp_note": "systemTime 是由帧媒体时间换算的 Unix 秒；VP systemTime 是收到位姿的 Unix 秒。物理曝光与位姿延迟未验证。",
        "pixel_layout": "[160 code band][1920 right][1920 left]; no resize/rotation/mirroring",
        "encoding": "H.264 60 Mbps; re-encoded at original dimensions, not lossless or original UVC bitstream",
        "started_host_seconds": hostSeconds()
    ]
    func saveMetadata() throws {
        try JSONSerialization.data(withJSONObject: metadata, options: [.prettyPrinted, .sortedKeys])
            .write(to: metaURL, options: .atomic)
    }
    try saveMetadata()
    var recorder: Recorder?
    do {
        let capture = try Recorder(device: device, out: out)
        recorder = capture
        metadata["requested_fps"] = 30
        metadata["selected_fps"] = capture.fps
        metadata["actual_frame_duration_seconds"] = CMTimeGetSeconds(device.activeVideoMinFrameDuration)
        metadata["source_media_subtype"] = CMFormatDescriptionGetMediaSubType(device.activeFormat.formatDescription)
        metadata["status"] = "recording"
        try saveMetadata()
        var stopping = false
        var finished = false
        var stopError: String?
        let started = hostSeconds()
        func stop(_ error: String?) {
            guard !stopping else { return }
            stopping = true
            stopError = error
            capture.session.stopRunning()
            capture.queue.sync {}
            capture.queue.async {
                capture.video.finish { error in
                    DispatchQueue.main.async {
                        if let error { stopError = stopError ?? error.localizedDescription }
                        metadata["frames_written"] = capture.video.frameCount
                        metadata["dropped_capture_frames"] = capture.captureDrops
                        metadata["dropped_writer_frames"] = capture.video.backpressureDrops
                        metadata["capture_frames_received"] = capture.capturedFrameCount
                        let captureSpan = (capture.lastCaptureHost ?? 0) - (capture.firstCaptureHost ?? 0)
                        metadata["capture_pts_span_seconds"] = captureSpan
                        metadata["written_pts_span_seconds"] = capture.video.durationSeconds
                        if captureSpan > 0 {
                            metadata["actual_capture_fps"] = Double(capture.capturedFrameCount - 1) / captureSpan
                        }
                        if capture.video.durationSeconds > 0 {
                            metadata["actual_written_fps"] = Double(capture.video.frameCount - 1) / capture.video.durationSeconds
                        }
                        metadata["finished_host_seconds"] = hostSeconds()
                        metadata["status"] = stopError == nil ? "completed" : "failed"
                        if let stopError { metadata["error"] = stopError }
                        do { try saveMetadata() }
                        catch { stopError = stopError ?? error.localizedDescription }
                        finished = true
                    }
                }
            }
        }
        capture.onFailure = { stop($0) }
        let application = NSApplication.shared
        application.setActivationPolicy(.regular)
        application.finishLaunching()
        let preview = PreviewWindow(session: capture.session)
        preview.onClose = { stop(nil) }
        application.activate(ignoringOtherApps: true)
        let runtimeError = NotificationCenter.default.addObserver(
            forName: AVCaptureSession.runtimeErrorNotification, object: capture.session, queue: .main
        ) { note in
            stop((note.userInfo?[AVCaptureSessionErrorKey] as? Error)?.localizedDescription ?? "摄像头采集运行失败")
        }
        signal(SIGINT, SIG_IGN)
        let interrupt = DispatchSource.makeSignalSource(signal: SIGINT, queue: .main)
        interrupt.setEventHandler { stop(nil) }
        interrupt.resume()
        let stdinSource = DispatchSource.makeReadSource(fileDescriptor: STDIN_FILENO, queue: .main)
        stdinSource.setEventHandler {
            var bytes = [UInt8](repeating: 0, count: 256)
            if read(STDIN_FILENO, &bytes, bytes.count) > 0 { stop(nil) }
            else { stdinSource.cancel() }
        }
        stdinSource.resume()
        let watchdog = DispatchSource.makeTimerSource(queue: .main)
        watchdog.schedule(deadline: .now() + 2, repeating: 1)
        watchdog.setEventHandler {
            let last = capture.queue.sync { capture.lastFrameHost }
            if hostSeconds() - (last ?? started) > 10 { stop("连续 10 秒未收到有效视频帧") }
        }
        watchdog.resume()
        print("相机：\(device.localizedName)，4000×1200，目标 \(capture.fps) fps。\n正在录制。按 Enter、Ctrl-C 或关闭预览窗口停止。\n输出：\(out.path)")
        try capture.start()
        metadata["actual_frame_duration_seconds"] = CMTimeGetSeconds(device.activeVideoMinFrameDuration)
        try saveMetadata()
        while !finished {
            if let event = application.nextEvent(matching: .any, until: Date(timeIntervalSinceNow: 0.1),
                                                 inMode: .default, dequeue: true) {
                application.sendEvent(event)
            }
            application.updateWindows()
        }
        preview.window.orderOut(nil)
        interrupt.cancel()
        stdinSource.cancel()
        watchdog.cancel()
        NotificationCenter.default.removeObserver(runtimeError)
        if let stopError { throw CaptureError(stopError) }
        print("录制完成：\(capture.video.frameCount) 帧；采集丢帧 \(capture.captureDrops)，编码忙丢帧 \(capture.video.backpressureDrops)。")
    } catch {
        recorder?.session.stopRunning()
        metadata["status"] = "failed"
        metadata["error"] = error.localizedDescription
        metadata["finished_host_seconds"] = hostSeconds()
        try saveMetadata()
        throw error
    }
}

// CLI entry point. Synthetic writer tests use the declarations above only.
do { try main() }
catch {
    FileHandle.standardError.write(Data("失败：\(error.localizedDescription)\n".utf8))
    exit(1)
}
