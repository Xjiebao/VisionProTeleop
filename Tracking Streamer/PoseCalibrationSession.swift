import Foundation
import simd

/// One ARKit tracking run, with transforms from device coordinates to ARKit world coordinates.
struct PoseCalibrationSession: Encodable {
    struct Sample: Encodable {
        let sampleId: String
        let isTracked: Bool
        let headMatrix: [Float]
    }

    let sessionId = UUID().uuidString
    let transformDirection = "device_to_arkit_world"
    let matrixLayout = "column_major"
    let translationUnit = "meters"
    private(set) var samples: [Sample] = []

    var nextSampleId: String { String(format: "%03d", samples.count + 1) }

    func fileURL(in recordingsURL: URL) -> URL {
        recordingsURL
            .appendingPathComponent("pose_calibration_\(sessionId)", isDirectory: true)
            .appendingPathComponent("poses.json")
    }

    /// The caller must query and validate tracking immediately before saving.
    /// Commit the sample in memory only after the complete JSON is atomically written.
    mutating func save(matrix m: simd_float4x4, in recordingsURL: URL) throws -> String {
        let sample = Sample(sampleId: nextSampleId, isTracked: true, headMatrix: [
            m.columns.0.x, m.columns.0.y, m.columns.0.z, m.columns.0.w,
            m.columns.1.x, m.columns.1.y, m.columns.1.z, m.columns.1.w,
            m.columns.2.x, m.columns.2.y, m.columns.2.z, m.columns.2.w,
            m.columns.3.x, m.columns.3.y, m.columns.3.z, m.columns.3.w
        ])
        var updatedSession = self
        updatedSession.samples.append(sample)
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        let data = try encoder.encode(updatedSession)
        let url = fileURL(in: recordingsURL)
        try FileManager.default.createDirectory(at: url.deletingLastPathComponent(), withIntermediateDirectories: true)
        try data.write(to: url, options: .atomic)
        self = updatedSession
        return sample.sampleId
    }
}
