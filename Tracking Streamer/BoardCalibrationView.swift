import SwiftUI
import UIKit

/// Uses the normal synchronized capture flow; the board runs the existing calibration solver.
struct BoardCalibrationView: View {
    let boardID: String
    let onDismiss: () -> Void
    @ObservedObject private var captureBoard = CaptureBoardClient.shared
    @ObservedObject private var recordingManager = RecordingManager.shared
    @State private var status: BoardStatus?
    @State private var isRequesting = false
    @State private var connectionError: String?
    @State private var actionError: String?
    @State private var calibrationSessionID: String?
    @State private var awaitingCaptureID = false
    @State private var resultPreviewImage: UIImage?
    @State private var livePreview: BoardLivePreview?
    @State private var liveImage: UIImage?
    @State private var livePreviewReceivedAt: Date?
    @State private var livePreviewError: String?
    @State private var livePreviewAttempt = 0

    init(boardID: String, onDismiss: @escaping () -> Void) {
        self.boardID = boardID
        self.onDismiss = onDismiss
        _calibrationSessionID = State(initialValue: UserDefaults.standard.string(forKey: "boardCalibrationSession.\(boardID)"))
    }

    private func rememberSession(_ sessionID: String) {
        if calibrationSessionID != sessionID {
            status = nil
            resultPreviewImage = nil
        }
        calibrationSessionID = sessionID
        UserDefaults.standard.set(sessionID, forKey: "boardCalibrationSession.\(boardID)")
    }

    private var connected: Bool { captureBoard.boards.contains { $0.id == boardID } }
    private var isComputing: Bool { status?.calibration?.state == "running" }
    private var captureBusy: Bool {
        recordingManager.hasBoardCapture || recordingManager.isSaving || recordingManager.isBoardOperationInProgress
    }
    private var canCompute: Bool {
        connected && !captureBusy && !isRequesting && !isComputing &&
        recordingManager.pendingBoardConfirmation == nil &&
        (status?.reviewState == nil || status?.reviewState == "kept") &&
        status?.state == "saved" && status?.vpUploaded == true &&
        status?.syncAfterFile != nil && status?.calibration?.state != "completed"
    }

    private var livePreviewPanel: some View {
        TimelineView(.periodic(from: .now, by: 0.5)) { context in
            let fresh = livePreviewError == nil && liveImage != nil && livePreviewReceivedAt.map {
                livePreview?.isFresh(receivedAt: $0, now: context.date) == true
            } == true
            VStack(alignment: .leading, spacing: 10) {
                HStack {
                    Text("Ego 相机取景").font(.headline)
                    Spacer()
                    Text("左眼 L · 右眼 R").font(.footnote).foregroundStyle(.secondary)
                }
                ZStack {
                    Color.black.opacity(0.5)
                    if let liveImage {
                        Image(uiImage: liveImage).resizable().scaledToFit()
                            .opacity(fresh ? 1 : 0.35)
                    }
                    if !fresh {
                        Text(livePreview?.state == "starting" ? "正在连接相机…" : "预览无信号 / 画面已过期")
                            .font(.callout.bold())
                            .padding(10)
                            .background(.black.opacity(0.7), in: RoundedRectangle(cornerRadius: 8))
                    }
                }
                .frame(height: 178)
                .clipShape(RoundedRectangle(cornerRadius: 10))
                HStack {
                    Label(fresh && livePreview?.leftDetected == true ? "左眼：棋盘完整" : "左眼：未确认",
                          systemImage: fresh && livePreview?.leftDetected == true ? "checkmark.circle.fill" : "exclamationmark.circle")
                        .foregroundStyle(fresh && livePreview?.leftDetected == true ? Color.green : Color.red)
                    Spacer()
                    Label(fresh && livePreview?.rightDetected == true ? "右眼：棋盘完整" : "右眼：未确认",
                          systemImage: fresh && livePreview?.rightDetected == true ? "checkmark.circle.fill" : "exclamationmark.circle")
                        .foregroundStyle(fresh && livePreview?.rightDetected == true ? Color.green : Color.red)
                }
                .font(.callout)
                if fresh, let livePreview {
                    Text(livePreview.guidance).font(.callout)
                    ProgressView(value: min(max(livePreview.stableSeconds, 0), 1)) {
                        Text(String(format: "停稳 %.1f 秒", livePreview.stableSeconds)).font(.footnote)
                    }
                } else if let message = livePreviewError ?? livePreview?.error {
                    Text(message).font(.footnote).foregroundStyle(.orange)
                } else if let livePreview {
                    Text(livePreview.guidance).font(.footnote).foregroundStyle(.secondary)
                }
                if let livePreview, livePreview.recording {
                    Text("候选姿态 \(livePreview.poseCount) 组" + (fresh ? "" : "（上次收到）"))
                        .font(.headline)
                }
                Text("预览计数仅作引导，最终有效数以完整视频计算为准。")
                    .font(.footnote).foregroundStyle(.secondary)
                if !fresh {
                    Button("重新预览") { livePreviewAttempt += 1 }
                        .disabled(!connected || isComputing || isRequesting || recordingManager.isBoardOperationInProgress)
                }
            }
        }
    }

    private func pollLivePreview() async {
        var startRequested = false
        while !Task.isCancelled {
            do {
                let method = startRequested ? "GET" : "POST"
                startRequested = true
                let preview = try await captureBoard.liveCalibrationPreview(boardID: boardID, method: method)
                if Task.isCancelled { return }
                livePreview = preview
                livePreviewReceivedAt = Date()
                livePreviewError = nil
                if let encoded = preview.imageBase64 {
                    guard let data = Data(base64Encoded: encoded), let image = UIImage(data: data) else {
                        throw CaptureBoardError.message("预览图像无法读取")
                    }
                    liveImage = image
                } else if preview.state == "live" {
                    liveImage = nil
                }
            } catch {
                if Task.isCancelled { return }
                livePreviewError = "预览连接失败：\(error.localizedDescription)"
            }
            do {
                try await Task.sleep(for: .seconds(livePreviewError == nil ? 0.5 : 2))
            } catch { return }
        }
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 16) {
            HStack {
                Text("外参标定").font(.title2.bold())
                Spacer()
                Button("关闭", action: onDismiss)
                    .disabled(awaitingCaptureID)
            }
            Button(recordingManager.hasBoardCapture ? "结束标定采集" : "开始标定采集") {
                actionError = nil
                if recordingManager.hasBoardCapture {
                    recordingManager.stopRecordingManually()
                } else {
                    awaitingCaptureID = true
                    recordingManager.startRecording()
                }
            }
            .buttonStyle(.borderedProminent)
            .disabled(isRequesting || isComputing || recordingManager.isSaving ||
                      captureBoard.selectedBoardID != boardID ||
                      (!recordingManager.hasBoardCapture && (!connected || recordingManager.isBoardOperationInProgress ||
                       recordingManager.pendingBoardConfirmation != nil)))
            BoardStorageWarningView()
            Text(recordingManager.boardCaptureStatus).font(.callout)
            BoardCaptureConfirmationView()
            BoardTransferStatusView()
            ScrollView {
                VStack(alignment: .leading, spacing: 16) {
                    livePreviewPanel

                    Text("在 VP 操作，由采集板计算")
                        .foregroundStyle(.secondary)
                    Text("固定棋盘和相机安装。改变头部朝向，每个姿态停稳约 2 秒，建议采集 20 组以上，并包含抬头、低头和侧倾。棋盘应完整入镜，始终保持同一边朝上。")
                    Text("棋盘规格沿用板子当前标定配置。普通动作录像不能代替棋盘标定录像。")
                        .font(.footnote).foregroundStyle(.secondary)

                    if let error = recordingManager.recordingError,
                       recordingManager.pendingBoardConfirmation == nil {
                        Text(error).foregroundStyle(.orange)
                    }

                    Divider()
                    if let sessionID = status?.sessionID {
                        Text("本轮：\(sessionID)").font(.callout.monospaced())
                    } else {
                        Text("尚无采集轮次").foregroundStyle(.secondary)
                    }
                    if status?.reviewState == "discarded" {
                        Label("本轮已作废；文件保留，不参与外参计算", systemImage: "xmark.circle")
                            .font(.callout).foregroundStyle(.orange)
                    } else {
                        Text("结束后自动录后对时；选择保存本轮并完成上传后，即可计算外参。")
                            .font(.footnote).foregroundStyle(.secondary)
                    }
                    Button(status?.calibration?.state == "failed" ? "重新计算外参" : "计算本轮外参") {
                        guard let sessionID = status?.sessionID else { return }
                        rememberSession(sessionID)
                        isRequesting = true
                        actionError = nil
                        Task {
                            defer { isRequesting = false }
                            do {
                                status = try await captureBoard.calibrate(sessionID: sessionID, boardID: boardID)
                            } catch {
                                actionError = "计算请求失败：\(error.localizedDescription)"
                            }
                        }
                    }
                    .disabled(!canCompute)

                    if let calibration = status?.calibration {
                        if calibration.state == "running" {
                            ProgressView("板子正在计算，可稍后重新打开此页面查看结果")
                        } else if calibration.state == "completed" {
                            Text("计算完成").font(.headline)
                            if let count = calibration.sampleCount, let heldOut = calibration.validationCount {
                                Text("可用姿态 \(count) 组，其中 \(heldOut) 组用于独立验证")
                            }
                            if let translation = calibration.validationTranslationMM,
                               let rotation = calibration.validationRotationDegrees {
                                Text(String(format: "留出姿态差：%.2f mm / %.2f°", translation, rotation))
                            }
                            Text("上述数值用于检查标定一致性，不等于手部标注精度。")
                                .font(.footnote).foregroundStyle(.secondary)
                            if let warnings = calibration.warnings, !warnings.isEmpty {
                                Text(warnings.joined(separator: "\n"))
                                    .font(.footnote).foregroundStyle(.orange)
                            }
                            Button("查看棋盘角点") {
                                isRequesting = true
                                actionError = nil
                                Task {
                                    defer { isRequesting = false }
                                    do {
                                        let data = try await captureBoard.calibrationPreview(
                                            sessionID: calibration.sessionID, boardID: boardID)
                                        guard let image = UIImage(data: data) else {
                                            throw CaptureBoardError.message("角点检查图无法读取")
                                        }
                                        resultPreviewImage = image
                                    } catch {
                                        actionError = "读取检查图失败：\(error.localizedDescription)"
                                    }
                                }
                            }
                            .disabled(!connected || isRequesting)
                            if let resultPreviewImage {
                                Text("检查各图中的 O 是否始终对应棋盘同一个实体角点。")
                                    .font(.footnote)
                                Image(uiImage: resultPreviewImage).resizable().scaledToFit()
                            }
                            if calibration.active {
                                Label("已启用；下一轮采集使用此标定", systemImage: "checkmark.circle.fill")
                                    .foregroundStyle(.green)
                            } else {
                                Button("启用此标定") {
                                    isRequesting = true
                                    actionError = nil
                                    Task {
                                        defer { isRequesting = false }
                                        do {
                                            status = try await captureBoard.applyCalibration(
                                                sessionID: calibration.sessionID, boardID: boardID)
                                        } catch {
                                            actionError = "启用失败：\(error.localizedDescription)"
                                        }
                                    }
                                }
                                .buttonStyle(.borderedProminent)
                                .disabled(!connected || captureBusy || isRequesting)
                                Text("启用后用于新采集，已有轮次保留原来的标定副本。")
                                    .font(.footnote).foregroundStyle(.secondary)
                            }
                        } else if calibration.state == "failed" {
                            Text(calibration.error ?? "计算失败，请检查本轮棋盘采集。")
                                .foregroundStyle(.orange)
                        }
                    }
                    if let error = actionError ?? connectionError {
                        Text(error).foregroundStyle(.orange)
                    }
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }
        }
        .padding(24)
        .frame(width: 620, height: 740)
        .onChange(of: recordingManager.captureSessionID) { _, sessionID in
            if awaitingCaptureID, let sessionID {
                rememberSession(sessionID)
                awaitingCaptureID = false
            }
        }
        .onChange(of: recordingManager.isBoardOperationInProgress) { _, inProgress in
            if !inProgress { awaitingCaptureID = false }
        }
        .task(id: livePreviewAttempt) {
            await pollLivePreview()
        }
        .onDisappear {
            Task { try? await captureBoard.liveCalibrationPreview(boardID: boardID, method: "POST", stop: true) }
        }
        .task(id: calibrationSessionID) {
            while !Task.isCancelled {
                do {
                    let updated = try await captureBoard.status(sessionID: calibrationSessionID, boardID: boardID)
                    if Task.isCancelled { return }
                    status = updated
                    connectionError = nil
                } catch {
                    if Task.isCancelled { return }
                    connectionError = "读取采集板状态失败：\(error.localizedDescription)"
                }
                do { try await Task.sleep(for: .seconds(2)) } catch { return }
            }
        }
    }
}
