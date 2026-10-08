import SwiftUI

@main
struct VisionProTeleopApp: App {
    @Environment(\.scenePhase) private var scenePhase
    @StateObject private var imageData = ImageData()
    @StateObject private var appModel = 🥽AppModel()
    @StateObject private var captureBoard = CaptureBoardClient.shared
    
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
        .windowResizability(.contentSize)
        .onChange(of: scenePhase) { _, phase in
            if phase == .active {
                RecordingManager.shared.startClockSync()
                captureBoard.refreshDiscovery()
                RecordingManager.shared.resumeBoardTransfers()
            } else if phase == .background {
                RecordingManager.shared.trackingDidStop()
            }
        }
        .onChange(of: captureBoard.boards) { _, _ in
            RecordingManager.shared.resumeBoardTransfers()
        }
        
        // Hand tracking view (existing)
        ImmersiveSpace(id: "immersiveSpace") {
            🌐RealityView(model: appModel)
        }
        
        // Video streaming view (new)
        ImmersiveSpace(id: "videoStreamSpace") {
            ImmersiveView()
                .environmentObject(imageData)
        }
        
        // MuJoCo streaming view (new)
        ImmersiveSpace(id: "mujocoStreamSpace") {
            MuJoCoStreamingView()
        }
        
        // Combined streaming view (Video + Audio + MuJoCo Sim)
        ImmersiveSpace(id: "combinedStreamSpace") {
            CombinedStreamingView()
                .environmentObject(imageData)
        }
    }
    
    init() {
        dlog("🚀 [DEBUG] VisionProTeleopApp.init() - App launching...")
        🧑HeadTrackingComponent.registerComponent()
        🧑HeadTrackingSystem.registerSystem()
        
        // The clock responder belongs to the app, independent of recording and views.
        Task { @MainActor in
            RecordingManager.shared.startClockSync()
            CaptureBoardClient.shared.startDiscovery()
            RecordingManager.shared.resumeBoardTransfers()
        }

        // Start gRPC server immediately when app launches
        dlog("🌐 [DEBUG] Starting gRPC server on app launch...")
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            dlog("🔧 [DEBUG] Calling startServer() from app init...")
            startServer()
        }
        
        // Configure settings sync from iOS
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
            Task { @MainActor in
                VisionOSSettingsSync.shared.configure(
                    dataManager: DataManager.shared,
                    recordingManager: RecordingManager.shared
                )
                dlog("☁️ [DEBUG] VisionOS settings sync configured")
            }
        }
    }
}
