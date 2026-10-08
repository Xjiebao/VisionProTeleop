import SwiftUI

struct PoseCalibrationView: View {
    @ObservedObject var appModel: 🥽AppModel
    let onDismiss: () -> Void
    @State private var trackingFailure: String? = "等待设备追踪"
    @State private var saveResult: String?
    @State private var saveFailed = false
    @State private var showShareSheet = false
    @State private var exportURL: URL?

    var body: some View {
        VStack(alignment: .leading, spacing: 20) {
            HStack {
                Text("位姿采样")
                    .font(.title2.bold())
                Spacer()
                Button("关闭", action: onDismiss)
            }
            ScrollView {
                VStack(alignment: .leading, spacing: 18) {
                    Label(trackingFailure == nil ? "可采样" : "追踪不可用",
                          systemImage: trackingFailure == nil ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                        .foregroundStyle(trackingFailure == nil ? Color.green : Color.orange)
                    if let trackingFailure {
                        Text(trackingFailure)
                            .font(.callout)
                    }
                    Text("当前批次")
                        .font(.headline)
                    Text(appModel.poseCalibrationSession.sessionId)
                        .font(.callout.monospaced())
                        .textSelection(.enabled)
                    Text("下一条样本编号：\(appModel.poseCalibrationSession.nextSampleId)")
                        .font(.title3.bold())
                    Text("停稳后保存位姿，并拍摄同编号的 ego 相机照片。")
                        .font(.callout)
                    Button("保存位姿", action: savePose)
                        .buttonStyle(.borderedProminent)
                    if let saveResult {
                        Text(saveResult)
                            .foregroundStyle(saveFailed ? Color.red : Color.green)
                    }
                    Divider()
                    Text("本地文件位置")
                        .font(.headline)
                    Text("Documents/Recordings/pose_calibration_\(appModel.poseCalibrationSession.sessionId)/poses.json")
                        .font(.caption.monospaced())
                        .textSelection(.enabled)
                        .fixedSize(horizontal: false, vertical: true)
                    if appModel.poseCalibrationSession.samples.isEmpty {
                        Text("首次保存成功后创建文件。")
                            .font(.callout)
                    } else {
                        Button("导出 JSON", systemImage: "square.and.arrow.up") {
                            do {
                                exportURL = try appModel.preparePoseExport()
                                showShareSheet = true
                            } catch {
                                saveFailed = true
                                saveResult = "导出失败：\(error.localizedDescription)"
                            }
                        }
                        .sheet(isPresented: $showShareSheet) {
                            if let exportURL {
                                ShareSheet(activityItems: [exportURL])
                            }
                        }
                        Text("导出文件名包含批次、样本范围和导出序号，每次生成独立文件。")
                            .font(.footnote)
                            .foregroundStyle(.secondary)
                    }
                    Text("关闭此页面保留批次和编号；重新启动追踪后使用新批次，旧文件保留。")
                        .font(.footnote)
                        .foregroundStyle(.secondary)
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }
        }
        .padding(24)
        .task {
            // Read tracking outside view/RealityView updates; never cache a pose for saving.
            while !Task.isCancelled {
                refreshTrackingStatus()
                do {
                    try await Task.sleep(for: .milliseconds(250))
                } catch {
                    return
                }
            }
        }
        .onChange(of: appModel.poseCalibrationSession.sessionId) { _, _ in
            saveResult = nil
            refreshTrackingStatus()
        }
    }

    private func refreshTrackingStatus() {
        do {
            _ = try appModel.queryDevicePoseForSampling()
            trackingFailure = nil
        } catch {
            trackingFailure = error.localizedDescription
        }
    }

    private func savePose() {
        do {
            let sampleId = try appModel.savePoseSample()
            saveFailed = false
            saveResult = "样本 \(sampleId) 已保存"
        } catch {
            saveFailed = true
            saveResult = "保存失败：\(error.localizedDescription)"
        }
        refreshTrackingStatus()
    }
}
