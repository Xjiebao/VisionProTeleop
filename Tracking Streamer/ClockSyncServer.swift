import Foundation
import Network

/// Exchanges Unix clock readings only; never changes either device's clock.
@MainActor
final class ClockSyncServer {
    private struct Probe: Decodable {
        let type: String
        let clock: String
        let seq: Int
        let t1: Double
    }

    private struct Reply: Encodable {
        let type = "clock_reply"
        let seq: Int
        let t1: Double
        let t2: Double
        let t3: Double
        let clock = "unix"
    }

    private let queue = DispatchQueue(label: "com.visionproteleop.clock-sync")
    private var listener: NWListener?
    private var connections: [NWConnection] = []
    private var idleTasks: [ObjectIdentifier: Task<Void, Never>] = [:]
    private var onStatus: ((String) -> Void)?

    func start(onStatus: @escaping (String) -> Void) {
        guard listener == nil else { return }
        self.onStatus = onStatus
        onStatus("电脑对钟正在启动…")
        do {
            let listener = try NWListener(using: .udp, on: 8766)
            self.listener = listener
            listener.stateUpdateHandler = { [weak self, weak listener] state in
                Task { @MainActor in
                    guard let self, let listener, self.listener === listener else { return }
                    switch state {
                    case .ready:
                        self.onStatus?("电脑对钟等待连接 · UDP 8766")
                    case .waiting(let error):
                        self.onStatus?("对钟失败：\(error.localizedDescription)")
                    case .failed(let error):
                        self.stop()
                        self.onStatus?("对钟失败：\(error.localizedDescription)")
                    default:
                        break
                    }
                }
            }
            listener.newConnectionHandler = { [weak self, weak listener] connection in
                Task { @MainActor in
                    guard let self, let listener, self.listener === listener else {
                        connection.cancel()
                        return
                    }
                    self.connections.append(connection)
                    connection.start(queue: self.queue)
                    self.refreshIdleTimeout(connection)
                    self.receive(connection, listener: listener)
                }
            }
            listener.start(queue: queue)
        } catch {
            onStatus("对钟失败：\(error.localizedDescription)")
        }
    }

    func stop() {
        listener?.cancel()
        listener = nil
        for task in idleTasks.values { task.cancel() }
        idleTasks.removeAll()
        for connection in connections {
            connection.cancel()
        }
        connections.removeAll()
        onStatus?("电脑对钟已关闭")
    }

    // Each board measurement uses a new UDP source port. Release idle peers while
    // leaving the listener available for the next measurement.
    private func refreshIdleTimeout(_ connection: NWConnection) {
        let id = ObjectIdentifier(connection)
        idleTasks[id]?.cancel()
        idleTasks[id] = Task { @MainActor [weak self, weak connection] in
            do { try await Task.sleep(nanoseconds: 30_000_000_000) }
            catch { return }
            guard !Task.isCancelled, let self, let connection else { return }
            self.removeConnection(connection)
        }
    }

    private func removeConnection(_ connection: NWConnection) {
        idleTasks.removeValue(forKey: ObjectIdentifier(connection))?.cancel()
        connection.cancel()
        connections.removeAll { $0 === connection }
    }

    private func receive(_ connection: NWConnection, listener: NWListener) {
        connection.receiveMessage { [weak self, weak listener] data, _, _, error in
            // Capture receipt before hopping to MainActor or decoding the request.
            let t2 = Date().timeIntervalSince1970
            Task { @MainActor in
                guard let self, let listener, self.listener === listener,
                      self.connections.contains(where: { $0 === connection }) else { return }
                if let error {
                    self.onStatus?("对钟失败：\(error.localizedDescription)")
                    self.removeConnection(connection)
                    return
                }
                self.refreshIdleTimeout(connection)
                guard let data, let probe = try? JSONDecoder().decode(Probe.self, from: data),
                      probe.type == "clock_probe", probe.clock == "unix", probe.t1.isFinite else {
                    self.onStatus?("对钟失败：请求格式无效，未回复")
                    self.receive(connection, listener: listener)
                    return
                }
                do {
                    let reply = Reply(seq: probe.seq, t1: probe.t1, t2: t2, t3: Date().timeIntervalSince1970)
                    let response = try JSONEncoder().encode(reply)
                    connection.send(content: response, completion: .contentProcessed { [weak self, weak listener] error in
                        Task { @MainActor in
                            guard let self, let listener, self.listener === listener,
                                  self.connections.contains(where: { $0 === connection }) else { return }
                            if let error {
                                self.onStatus?("对钟失败：\(error.localizedDescription)")
                            } else {
                                self.onStatus?("电脑对钟已响应 · UDP 8766")
                            }
                        }
                    })
                } catch {
                    self.onStatus?("对钟失败：\(error.localizedDescription)")
                }
                self.receive(connection, listener: listener)
            }
        }
    }
}
