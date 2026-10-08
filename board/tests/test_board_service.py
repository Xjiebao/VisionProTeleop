"""无硬件行为测试：使用临时模拟相机，不接触真实采集目录。"""
from contextlib import contextmanager
import errno
import http.client
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import board_service as board


class ServiceTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        (self.root / "calibration").mkdir()
        board.write_json(self.root / "calibration" / "intrinsics.json", {"camera": "fixture"})
        board.write_json(self.root / "calibration" / "extrinsics.json", {
            "transform": "fixture", "board": {"inner_corners": [11, 8], "square_mm": 30}})
        board.write_json(self.root / "ego_config.json", {
            "intrinsics": "calibration/intrinsics.json", "calibration": "calibration/extrinsics.json"})
        script = self.root / "start_capture_linux.sh"
        script.write_text("#!" + sys.executable + "\n" + '''import json, pathlib, sys, time
if '--list-json' in sys.argv:
    print(json.dumps([{'name': 'DECXIN fixture', 'unique_id': 'fixture-camera', 'external': True, 'supports_4000x1200': True}]))
    sys.exit(0)
if '--preview-only' in sys.argv:
    folder = pathlib.Path(sys.argv[sys.argv.index('--preview') + 1])
    (folder / 'latest.jpg').write_bytes(b'fixture-jpeg')
    while True:
        time.sleep(.05)
folder = pathlib.Path(sys.argv[sys.argv.index('--out') + 1])
folder.mkdir()
mode = (folder.parent.parent.parent / 'fixture_mode')
mode = mode.read_text() if mode.exists() else 'ok'
if mode == 'start_failure':
    print('fixture camera unavailable', flush=True)
    sys.exit(3)
(folder / 'video.mov').write_bytes(b'fixture-video')
(folder / 'frames.jsonl').write_text(json.dumps({'systemTime': time.time()}) + '\\n')
(folder / 'session.json').write_text(json.dumps({'status': 'starting', 'frames_written': 1}))
if mode != 'no_ready':
    print('CAPTURE_READY', flush=True)
if mode == 'crash':
    time.sleep(.15)
    sys.exit(9)
sys.stdin.readline()
time.sleep(.12)
(folder / 'session.json').write_text(json.dumps({'status': 'failed' if mode == 'save_failure' else 'completed', 'frames_written': 1}))
''')
        script.chmod(0o755)
        self.service = board.BoardService(self.root, "board-1", "采集板 01")
        self.service.measure = self.measure
        self.service.start_timeout = 1
        self.server = board.ThreadingHTTPServer(("127.0.0.1", 0), board.Handler)
        self.server.service = self.service
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.service.shutdown()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.temporary.cleanup()

    @staticmethod
    def measure(peer):
        now = time.time()
        return {"schema_version": 1, "kind": "clock_sync", "clock": "unix", "status": "completed",
                "vp_ip": peer, "started_system_time": now, "finished_system_time": now + .001,
                "exchanges": [{"t1": now, "t2": now, "t3": now, "t4": now}],
                "summary": {"network_rtt_ms": 1, "offset_vp_minus_mac_seconds": .002, "samples_retained": 20}}

    def request(self, method, path, body=None, headers=None):
        if not isinstance(body, bytes):
            body = json.dumps(body or {}).encode()
        connection = http.client.HTTPConnection(*self.server.server_address, timeout=5)
        connection.request(method, path, body, {"Content-Type": "application/json", **(headers or {})})
        response = connection.getresponse()
        payload = json.loads(response.read())
        connection.close()
        return response.status, payload

    def prepare(self, session="session-1"):
        status, payload = self.request("POST", "/sessions", {"session_id": session})
        self.assertEqual(status, 200, payload)
        self.assertEqual(payload["state"], "prepared")
        return payload

    def saved(self, session="session-1", review=True):
        self.prepare(session)
        self.assertEqual(self.request("POST", f"/sessions/{session}/start")[0], 200)
        status, payload = self.request("POST", f"/sessions/{session}/stop")
        self.assertEqual(status, 200, payload)
        self.assertEqual(payload["state"], "saved")
        if review:
            status, payload = self.request("POST", f"/sessions/{session}/review", {"decision": "kept"})
            self.assertEqual((status, payload["review_state"]), (200, "kept"))
        return payload

    def metadata(self, session="session-1", **changes):
        return json.dumps({"captureSessionID": session, "captureBoardID": "board-1", "recordingType": "egorecord",
                           "frameCount": 1, "poseData": {"recordingStartMonotonicTimestamp": "100"}, **changes}).encode()

    @staticmethod
    def events(start=100):
        return (json.dumps({"source": "head", "sequenceNumber": 1,
                            "receivedTimestamp": start + .2, "recordingTimestamp": .2}) + "\n").encode()

    def upload(self, name, data, session="session-1"):
        return self.request("PUT", f"/sessions/{session}/files/{name}", data)

    def calibration_ready(self, session="session-1"):
        self.saved(session)
        self.assertEqual(self.upload("metadata.json", self.metadata(session), session)[0], 200)
        self.assertEqual(self.upload("tracking_events.jsonl", self.events(), session)[0], 200)
        self.assertEqual(self.request("POST", f"/sessions/{session}/sync-after")[0], 200)
        python = self.root / ".venv/bin/python"
        if not python.exists():
            python.parent.mkdir(parents=True)
            python.symlink_to(sys.executable)
        solver = self.root / "video_extrinsics.py"
        solver.write_text('''import json, pathlib, sys, time
root = pathlib.Path(__file__).parent
output = pathlib.Path(sys.argv[sys.argv.index('--output') + 1])
(output.parent / 'arguments.json').write_text(json.dumps(sys.argv[1:]))
while not (root / 'release_calibration').exists():
    time.sleep(.01)
mode = root / 'calibration_mode'
mode = mode.read_text() if mode.exists() else 'ok'
if mode == 'fail':
    print('fixture insufficient stationary poses', flush=True)
    sys.exit(7)
if mode == 'missing_result':
    sys.exit(0)
result = {'training_ids': list(range(6)), 'validation_ids': [6, 7], 'warnings': ['fixture solver warning'],
          'board': {'inner_corners': [11, 8], 'square_mm': 30},
          'quality': {'validation': {
              'left': {'translation_mm': {'rms': 1.25}, 'rotation_deg': {'rms': .4}},
              'right': {'translation_mm': {'rms': 2.5}, 'rotation_deg': {'rms': .2}}}}}
output.write_text(json.dumps(result))
output.with_name(output.stem + '_角点检查.jpg').write_bytes(b'fixture-jpeg')
''')

    def finish_calibration(self, session="session-1"):
        process = self.service.calibration_processes[session]
        (self.root / "release_calibration").touch()
        process.wait(timeout=3)
        code, reply = self.request("GET", f"/status?session_id={session}")
        self.assertEqual(code, 200, reply)
        return reply["calibration"]

    def test_pending_review_allows_post_sync_but_requires_keep_before_upload(self):
        reply = self.saved(review=False)
        self.assertEqual(reply["review_state"], "pending")
        self.assertEqual(self.upload("metadata.json", self.metadata())[0], 409)
        self.assertEqual(self.upload("tracking_events.jsonl", self.events())[0], 409)
        self.assertFalse(list((self.root / "data/session-1/vp_recording").iterdir()))
        code, reply = self.request("POST", "/sessions/session-1/sync-after")
        self.assertEqual((code, reply["review_state"]), (200, "pending"))
        self.assertTrue(reply["sync_after"])
        self.assertEqual(self.request("POST", "/sessions/session-1/calibrate")[0], 409)
        self.assertEqual(self.request("POST", "/sessions/session-1/apply-calibration")[0], 409)
        # 板端不全局阻塞旧 pending；当前待确认的交互限制由 VP 管理。
        self.prepare("session-2")
        self.request("POST", "/sessions/session-2/stop")
        code, reply = self.request("POST", "/sessions/session-1/review", {"decision": "kept"})
        self.assertEqual((code, reply["review_state"]), (200, "kept"))
        self.assertEqual(self.upload("metadata.json", self.metadata())[0], 200)
        self.assertTrue(self.upload("tracking_events.jsonl", self.events())[1]["vp_uploaded"])

    def test_discard_preserves_files_and_parameters_and_blocks_all_consumers(self):
        self.calibration_ready()
        self.request("POST", "/sessions/session-1/calibrate")
        self.finish_calibration()
        folder = self.root / "data/session-1"
        originals = {path.relative_to(folder): path.read_bytes() for path in folder.rglob("*")
                     if path.is_file() and path.name != "recording.json"}
        original_config = (self.root / "ego_config.json").read_bytes()
        for decision in ("pending", "discarded"):
            with self.subTest(review_state=decision):
                if decision == "pending":
                    self.service.change(self.service.record("session-1"), review_state="pending")
                else:
                    code, reply = self.request("POST", "/sessions/session-1/review", {"decision": decision})
                    self.assertEqual((code, reply["review_state"]), (200, "discarded"))
                self.assertEqual(self.upload("metadata.json", self.metadata())[0], 409)
                self.assertEqual(self.request("POST", "/sessions/session-1/calibrate")[0], 409)
                self.assertEqual(self.request("POST", "/sessions/session-1/apply-calibration")[0], 409)
        self.assertEqual(originals, {path.relative_to(folder): path.read_bytes() for path in folder.rglob("*")
                                     if path.is_file() and path.name != "recording.json"})
        self.assertEqual((self.root / "ego_config.json").read_bytes(), original_config)
        # 更改确认选择不会删除重建原始文件。
        self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "kept"})[1]["review_state"], "kept")
        self.assertEqual(self.upload("metadata.json", self.metadata())[0], 200)

    def test_review_is_idempotent_persists_and_old_recordings_default_to_kept(self):
        self.saved(review=False)
        for decision in ("kept", "discarded", "discarded", "kept", "kept"):
            code, reply = self.request("POST", "/sessions/session-1/review", {"decision": decision})
            self.assertEqual((code, reply["review_state"]), (200, decision))
            recovered = board.BoardService(self.root, "board-1", "采集板 01")
            self.assertEqual(recovered.status("session-1")["review_state"], decision)
        info = self.service.record("session-1")
        info.pop("review_state")
        self.service.persist(info)
        recovered = board.BoardService(self.root, "board-1", "采集板 01")
        self.assertEqual(recovered.status("session-1")["review_state"], "kept")
        self.assertEqual(self.upload("metadata.json", self.metadata())[0], 200)

    def test_cannot_discard_running_calibration_or_either_active_parameter(self):
        self.calibration_ready()
        original_config = board.read_json(self.root / "ego_config.json")
        self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "kept"})[0], 200)
        self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "discarded"})[0], 409)
        self.assertEqual(self.service.status("session-1")["review_state"], "kept")
        self.finish_calibration()
        self.assertEqual(self.request("POST", "/sessions/session-1/apply-calibration")[0], 200)
        active_config = board.read_json(self.root / "ego_config.json")
        for key in ("intrinsics", "calibration"):
            with self.subTest(active_parameter=key):
                board.write_json(self.root / "ego_config.json", {**original_config, key: active_config[key]})
                self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "discarded"})[0], 409)
                self.assertEqual(self.service.status("session-1")["review_state"], "kept")
        board.write_json(self.root / "ego_config.json", original_config)
        self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "discarded"})[1]["review_state"], "discarded")

    def test_review_rejects_invalid_decisions_active_capture_and_wrong_board(self):
        self.prepare()
        for body in ({}, {"decision": None}, {"decision": []}, {"decision": "pending"}):
            self.assertEqual(self.request("POST", "/sessions/session-1/review", body)[0], 400)
        self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "kept"},
                                      {"X-Capture-Board-ID": "other-board"})[0], 409)
        self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "kept"})[0], 409)
        self.request("POST", "/sessions/session-1/start")
        self.assertEqual(self.request("POST", "/sessions/session-1/review", {"decision": "discarded"})[0], 409)
        self.assertEqual(self.service.status("session-1")["review_state"], "pending")
        self.assertEqual(self.request("POST", "/sessions/missing/review", {"decision": "kept"})[0], 404)

    def test_failed_round_can_be_kept_without_post_sync_and_uploaded(self):
        self.prepare()
        self.request("POST", "/sessions/session-1/stop")
        code, reply = self.request("POST", "/sessions/session-1/review", {"decision": "kept"})
        self.assertEqual((code, reply["state"], reply["review_state"]), (200, "failed", "kept"))
        self.assertIsNone(reply["sync_after"])
        self.assertEqual(self.upload("metadata.json", self.metadata())[0], 200)
        self.assertTrue(self.upload("tracking_events.jsonl", self.events())[1]["vp_uploaded"])
        self.assertEqual(self.request("POST", "/sessions/session-1/calibrate")[0], 409)

    def start_preview(self):
        python = self.root / ".venv/bin/python"
        if not python.exists():
            python.parent.mkdir(parents=True)
            python.symlink_to(sys.executable)
        (self.root / "calibration_preview.py").write_text("import time\nwhile True:\n    time.sleep(.05)\n")
        code, reply = self.request("POST", "/calibration/preview", headers={"X-Capture-Board-ID": "board-1"})
        self.assertEqual((code, reply["state"]), (200, "starting"), reply)
        return Path(self.service.preview_directory.name)

    def test_live_preview_protocol_board_identity_and_stale_clear_detection(self):
        self.assertEqual(self.request("GET", "/calibration/preview")[1]["state"], "stopped")
        for method, path in [("POST", "/calibration/preview"), ("GET", "/calibration/preview"),
                             ("POST", "/calibration/preview/stop")]:
            code, reply = self.request(method, path, headers={"X-Capture-Board-ID": "other-board"})
            self.assertEqual((code, reply["board_id"]), (409, "board-1"))
        self.assertFalse(self.service.preview_enabled)
        folder = self.start_preview()
        camera, worker = self.service.preview_camera, self.service.preview_worker
        sample = {"frame_time": time.time(), "image_base64": "Zml4dHVyZQ==", "left_detected": True,
                  "right_detected": True, "stable_seconds": 1.5, "pose_count": 5, "error": None}
        board.write_json(folder / "live.json", sample)
        code, reply = self.request("GET", "/calibration/preview")
        self.assertEqual((code, reply["state"], reply["left_detected"], reply["pose_count"]), (200, "live", True, 5))
        board.write_json(folder / "live.json", dict(sample, frame_time=time.time() - 4))
        reply = self.request("GET", "/calibration/preview")[1]
        self.assertEqual((reply["state"], reply["left_detected"], reply["right_detected"], reply["stable_seconds"]),
                         ("stale", False, False, 0.))
        self.assertEqual(self.request("POST", "/calibration/preview/stop")[1]["state"], "stopped")
        self.assertIsNotNone(camera.poll())
        self.assertIsNotNone(worker.poll())
        self.assertEqual(self.request("GET", "/calibration/preview")[1]["state"], "stopped")

    def test_preview_prepare_cancel_releases_then_resumes_standalone(self):
        self.start_preview()
        camera, worker = self.service.preview_camera, self.service.preview_worker
        self.prepare()
        self.assertIsNotNone(camera.poll())
        self.assertIsNotNone(worker.poll())
        self.assertTrue(self.service.preview_enabled)
        self.assertIsNone(self.service.preview_camera)
        self.assertIsNone(self.service.preview_worker)
        self.assertEqual(self.request("GET", "/calibration/preview")[1]["state"], "starting")
        self.request("POST", "/sessions/session-1/stop")
        reply = self.request("GET", "/calibration/preview")[1]
        self.assertEqual((reply["state"], reply["pose_count"], reply["recording"]), ("starting", 0, False))
        self.assertIsNotNone(self.service.preview_camera)
        self.assertIsNotNone(self.service.preview_worker)

    def test_capture_preview_uses_same_stream_resets_count_and_resumes_after_stop(self):
        folder = self.start_preview()
        board.write_json(folder / "live.json", {"frame_time": time.time(), "pose_count": 7})
        self.prepare()
        with patch.object(board.subprocess, "Popen", wraps=board.subprocess.Popen) as launch:
            code, reply = self.request("POST", "/sessions/session-1/start")
        self.assertEqual((code, reply["state"]), (200, "recording"), reply)
        commands = [call.args[0] for call in launch.call_args_list]
        camera_args = next(command for command in commands if "--out" in command)
        self.assertEqual(camera_args[camera_args.index("--preview") + 1], str(folder))
        self.assertNotIn("--preview-only", camera_args)
        worker_args = next(command for command in commands if "--source" in command)
        self.assertEqual(worker_args[worker_args.index("--calibration") + 1],
                         str(self.root / "data/session-1/calibration/ego_extrinsics.json"))
        self.assertFalse((folder / "live.json").exists())
        self.assertIsNone(self.service.preview_camera)
        worker = self.service.preview_worker
        reply = self.request("GET", "/calibration/preview")[1]
        self.assertEqual((reply["recording"], reply["pose_count"]), (True, 0))
        self.assertEqual(self.request("POST", "/sessions/session-1/stop")[1]["state"], "saved")
        reply = self.request("GET", "/calibration/preview")[1]
        self.assertEqual((reply["recording"], reply["pose_count"]), (False, 0))
        self.assertIsNotNone(worker.poll())
        self.assertIsNotNone(self.service.preview_camera)
        self.assertIsNone(self.service.preview_session)

    def test_stopping_preview_preserves_active_recording(self):
        self.start_preview()
        self.prepare()
        self.request("POST", "/sessions/session-1/start")
        process, worker = self.service.processes["session-1"], self.service.preview_worker
        code, reply = self.request("POST", "/calibration/preview/stop")
        self.assertEqual((code, reply["state"], reply["recording"]), (200, "stopped", True))
        self.assertIsNotNone(worker.poll())
        self.assertIsNone(process.poll())
        self.assertEqual(self.request("GET", "/status")[1]["state"], "recording")

    def test_preview_expiry_preserves_recording_and_can_resume_worker(self):
        self.start_preview()
        self.prepare()
        self.request("POST", "/sessions/session-1/start")
        process, worker = self.service.processes["session-1"], self.service.preview_worker
        self.service.preview_shutdown.set()
        self.service.preview_watchdog.join(timeout=2)
        self.assertFalse(self.service.preview_watchdog.is_alive())
        self.service.preview_last_request = time.monotonic() - 16
        with patch.object(self.service.preview_shutdown, "wait", side_effect=[False, True]):
            self.service.preview_expiry_loop()
        self.assertFalse(self.service.preview_enabled)
        self.assertIsNotNone(worker.poll())
        self.assertIsNone(process.poll())
        reply = self.request("POST", "/calibration/preview")[1]
        self.assertEqual((reply["state"], reply["recording"], reply["pose_count"]), ("starting", True, 0))
        self.assertIsNotNone(self.service.preview_worker)
        self.assertIsNone(self.service.preview_camera)
        self.assertIs(self.service.processes["session-1"], process)

    def test_preview_camera_exit_is_explicit_and_retry_restarts_camera(self):
        self.start_preview()
        camera, worker = self.service.preview_camera, self.service.preview_worker
        camera.terminate()
        camera.wait(timeout=2)
        reply = self.request("GET", "/calibration/preview")[1]
        self.assertEqual(reply["state"], "failed")
        self.assertTrue(reply["error"])
        reply = self.request("POST", "/calibration/preview")[1]
        self.assertEqual(reply["state"], "starting", reply)
        self.assertIsNot(self.service.preview_camera, camera)
        self.assertIsNone(self.service.preview_camera.poll())
        self.assertIsNotNone(worker.poll())

    def test_calibration_stops_preview_and_rejects_restart_until_finished(self):
        self.calibration_ready()
        self.start_preview()
        camera, worker = self.service.preview_camera, self.service.preview_worker
        code, reply = self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual((code, reply["calibration"]["state"]), (200, "running"))
        self.assertFalse(self.service.preview_enabled)
        self.assertIsNotNone(camera.poll())
        self.assertIsNotNone(worker.poll())
        self.assertEqual(self.request("POST", "/calibration/preview")[0], 409)
        self.finish_calibration()
        self.assertEqual(self.request("POST", "/calibration/preview")[1]["state"], "starting")

    def test_cannot_start_preview_midway_through_normal_recording(self):
        self.prepare()
        self.request("POST", "/sessions/session-1/start")
        process = self.service.processes["session-1"]
        code, reply = self.request("POST", "/calibration/preview")
        self.assertEqual(code, 409)
        self.assertTrue(reply["error"])
        self.assertFalse(self.service.preview_enabled)
        self.assertIsNone(process.poll())

    def test_calibration_runs_async_with_original_solver_parameters_and_idempotence(self):
        self.calibration_ready()
        old_config = (self.root / "ego_config.json").read_bytes()
        code, reply = self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual((code, reply["calibration"]["state"]), (200, "running"))
        process = self.service.calibration_processes["session-1"]
        duplicate = self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual((duplicate[0], duplicate[1]["calibration"]["state"]), (200, "running"))
        self.assertIs(self.service.calibration_processes["session-1"], process)
        result = self.finish_calibration()
        self.assertEqual((result["state"], result["sample_count"], result["validation_count"]), ("completed", 8, 2))
        self.assertEqual((result["validation_translation_mm"], result["validation_rotation_deg"]), (2.5, .4))
        self.assertFalse(result["active"])
        self.assertEqual((self.root / "ego_config.json").read_bytes(), old_config)
        arguments = board.read_json(self.root / "data/session-1/calibration_result/arguments.json")
        flags = dict(zip(arguments[::2], arguments[1::2]))
        folder = self.root / "data/session-1"
        self.assertEqual(flags, {"--session": str(folder / "camera"), "--vp-recording": str(folder / "vp_recording"),
                                "--sync-before": str(folder / "sync_before.json"),
                                "--sync-after": str(folder / self.service.record("session-1")["sync_after"]),
                                "--intrinsics": str(folder / "calibration/stereo_calibration.json"),
                                "--output": str(folder / "calibration_result/ego_extrinsics.json"),
                                "--cols": "11", "--rows": "8", "--square-mm": "30"})
        self.assertEqual(self.request("POST", "/sessions/session-1/calibrate")[1]["calibration"]["state"], "completed")
        self.assertEqual(len(list(folder.glob("calibration_result*"))), 1)

    def test_calibration_warnings_are_preserved_in_status_and_manifest(self):
        self.calibration_ready()
        self.request("POST", "/sessions/session-1/calibrate")
        calibration = self.finish_calibration()
        self.assertEqual(calibration["warnings"], ["fixture solver warning"])
        manifest = board.read_json(self.root / "data/session-1/recording.json")
        self.assertEqual(manifest["calibration"]["warnings"], calibration["warnings"])
        self.assertEqual(self.request("GET", "/status")[1]["calibration"]["warnings"], calibration["warnings"])

    def test_calibration_preview_returns_only_completed_fixed_image_and_checks_board_id(self):
        self.calibration_ready()
        path = "/sessions/session-1/calibration-preview"
        self.assertEqual(self.request("GET", path)[0], 409)
        self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual(self.request("GET", path)[0], 409)
        self.finish_calibration()
        connection = http.client.HTTPConnection(*self.server.server_address, timeout=5)
        connection.request("GET", path, headers={"X-Capture-Board-ID": "board-1"})
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Content-Type"), "image/jpeg")
        content = response.read()
        self.assertEqual(content, b"fixture-jpeg")
        self.assertEqual(int(response.getheader("Content-Length")), len(content))
        connection.close()
        self.assertEqual(self.request("GET", path, headers={"X-Capture-Board-ID": "board-2"})[0], 409)
        self.assertEqual(self.request("GET", path + "/metadata.json")[0], 404)
        (self.root / "data/session-1/calibration_result/ego_extrinsics_角点检查.jpg").unlink()
        code, reply = self.request("GET", path)
        self.assertEqual(code, 404)
        self.assertIn("缺少角点检查图", reply["error"])

    def test_failed_calibration_preserves_log_and_retries_in_new_directory(self):
        self.calibration_ready()
        (self.root / "calibration_mode").write_text("fail")
        self.request("POST", "/sessions/session-1/calibrate")
        failed = self.finish_calibration()
        self.assertEqual(failed["state"], "failed")
        self.assertIn("insufficient stationary poses", failed["error"])
        self.assertEqual(self.request("POST", "/sessions/session-1/apply-calibration")[0], 409)
        old_log = (self.root / failed["log_path"]).read_bytes()
        (self.root / "release_calibration").unlink()
        (self.root / "calibration_mode").write_text("ok")
        code, reply = self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual((code, reply["calibration"]["state"]), (200, "running"))
        self.assertIn("calibration_result_02/", reply["calibration"]["output_path"])
        self.assertEqual(self.finish_calibration()["state"], "completed")
        self.assertEqual((self.root / failed["log_path"]).read_bytes(), old_log)

    def test_zero_exit_without_calibration_result_is_failure(self):
        self.calibration_ready()
        (self.root / "calibration_mode").write_text("missing_result")
        self.request("POST", "/sessions/session-1/calibrate")
        result = self.finish_calibration()
        self.assertEqual(result["state"], "failed")
        self.assertTrue(result["error"])
        self.assertIsNone(result["sample_count"])

    def test_apply_calibration_keeps_matching_intrinsics_and_old_snapshots(self):
        self.calibration_ready()
        folder = self.root / "data/session-1"
        snapshot = {path.name: path.read_bytes() for path in (folder / "calibration").iterdir()}
        board.write_json(self.root / "ego_config.json", {**board.read_json(self.root / "ego_config.json"), "camera_unique_id": "fixture-camera"})
        self.request("POST", "/sessions/session-1/calibrate")
        self.finish_calibration()
        code, reply = self.request("POST", "/sessions/session-1/apply-calibration")
        self.assertEqual((code, reply["calibration"]["active"]), (200, True))
        settings = board.read_json(self.root / "ego_config.json")
        self.assertEqual(settings["calibration"], "data/session-1/calibration_result/ego_extrinsics.json")
        self.assertEqual(settings["intrinsics"], "data/session-1/calibration/stereo_calibration.json")
        self.assertEqual(settings["camera_unique_id"], "fixture-camera")
        self.assertEqual(snapshot, {path.name: path.read_bytes() for path in (folder / "calibration").iterdir()})
        self.assertTrue(self.request("POST", "/sessions/session-1/apply-calibration")[1]["calibration"]["active"])
        self.prepare("session-2")
        new_snapshot = board.read_json(self.root / "data/session-2/calibration/ego_extrinsics.json")
        self.assertEqual(new_snapshot["training_ids"], list(range(6)))
        self.assertEqual((self.root / "data/session-2/calibration/stereo_calibration.json").read_bytes(), snapshot["stereo_calibration.json"])
        self.assertEqual(self.request("POST", "/sessions/session-1/apply-calibration")[0], 409)

    def test_calibration_requires_uploaded_vp_and_both_syncs(self):
        self.saved()
        self.assertEqual(self.request("POST", "/sessions/session-1/calibrate")[0], 409)
        self.upload("metadata.json", self.metadata())
        self.upload("tracking_events.jsonl", self.events())
        code, reply = self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual(code, 409)
        self.assertIn("录后对钟", reply["error"])
        self.assertNotIn("calibration", self.service.record("session-1"))
        self.assertFalse((self.root / "data/session-1/calibration_result").exists())

    def test_calibration_rejects_missing_input_file(self):
        self.calibration_ready()
        (self.root / "data/session-1/vp_recording/tracking_events.jsonl").unlink()
        code, reply = self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual(code, 409)
        self.assertIn("不完整", reply["error"])
        self.assertFalse(self.service.calibration_processes)

    def test_calibration_and_capture_are_mutually_exclusive(self):
        self.calibration_ready()
        self.prepare("session-2")
        self.assertEqual(self.request("POST", "/sessions/session-1/calibrate")[0], 409)
        self.request("POST", "/sessions/session-2/start")
        self.assertEqual(self.request("POST", "/sessions/session-1/calibrate")[0], 409)
        self.request("POST", "/sessions/session-2/stop")
        code, reply = self.request("POST", "/sessions/session-1/calibrate")
        self.assertEqual((code, reply["calibration"]["state"]), (200, "running"))
        self.assertEqual(self.request("POST", "/sessions", {"session_id": "session-3"})[0], 409)
        self.assertEqual(self.request("POST", "/sessions/session-2/calibrate")[0], 409)
        self.assertEqual(self.request("POST", "/sessions/session-1/apply-calibration")[0], 409)
        self.finish_calibration()
        self.prepare("session-3")

    def test_calibration_restart_marks_running_failed_and_shutdown_terminates_child(self):
        self.calibration_ready()
        self.request("POST", "/sessions/session-1/calibrate")
        process = self.service.calibration_processes["session-1"]
        recovered = board.BoardService(self.root, "board-1", "采集板 01")
        reply = recovered.status("session-1")
        self.assertEqual((reply["state"], reply["calibration"]["state"]), ("saved", "failed"))
        self.assertIn("中断", reply["calibration"]["error"])
        self.service.shutdown()
        self.assertIsNotNone(process.poll())
        self.assertFalse(self.service.calibration_processes)
        self.assertEqual(self.service.status("session-1")["calibration"]["state"], "failed")
        self.assertTrue((self.root / "data/session-1/camera/video.mov").exists())

    def test_prepare_and_start_retries_keep_same_process_and_files(self):
        self.prepare()
        original = (self.root / "data/session-1/sync_before.json").read_bytes()
        self.prepare()
        self.assertEqual((self.root / "data/session-1/sync_before.json").read_bytes(), original)
        code, reply = self.request("POST", "/sessions/session-1/start")
        self.assertEqual((code, reply["state"]), (200, "recording"))
        pid = self.service.processes["session-1"].pid
        self.assertEqual(self.request("POST", "/sessions/session-1/start")[0], 200)
        self.assertEqual(self.service.processes["session-1"].pid, pid)
        self.assertEqual(self.request("POST", "/sessions", {"session_id": "session-2"})[0], 409)

    def test_stop_waits_for_completed_and_is_idempotent(self):
        self.prepare()
        self.request("POST", "/sessions/session-1/start")
        started = time.monotonic()
        code, reply = self.request("POST", "/sessions/session-1/stop")
        self.assertEqual((code, reply["state"]), (200, "saved"))
        self.assertGreaterEqual(time.monotonic() - started, .1)
        self.assertEqual(board.read_json(self.root / "data/session-1/camera/session.json")["status"], "completed")
        self.assertEqual(self.request("POST", "/sessions/session-1/stop")[0], 200)
        self.assertEqual(self.request("POST", "/sessions/session-1/start")[0], 409)

    def test_cancel_prepared_is_repeatable_and_releases_camera(self):
        self.prepare()
        for _ in range(2):
            code, reply = self.request("POST", "/sessions/session-1/stop")
            self.assertEqual((code, reply["state"], reply["error"]), (200, "failed", board.CANCELLED))
        self.prepare("session-2")
        self.assertFalse((self.root / "data/session-1/camera").exists())

    def test_start_failure_returns_error_and_preserves_log(self):
        self.prepare()
        (self.root / "fixture_mode").write_text("start_failure")
        code, reply = self.request("POST", "/sessions/session-1/start")
        self.assertGreaterEqual(code, 400)
        self.assertEqual(reply["state"], "failed")
        self.assertTrue(reply["error"])
        self.assertIn("fixture camera unavailable", (self.root / "data/session-1/capture.log").read_text())
        code, stopped = self.request("POST", "/sessions/session-1/stop")
        self.assertEqual((code, stopped["state"]), (200, "failed"))

    def test_first_frame_timeout_is_failed_not_recording(self):
        self.prepare()
        (self.root / "fixture_mode").write_text("no_ready")
        self.service.start_timeout = .2
        code, reply = self.request("POST", "/sessions/session-1/start")
        self.assertEqual((code, reply["state"]), (504, "failed"))
        self.assertFalse(self.service.processes)

    def test_failed_camera_save_never_returns_saved(self):
        self.prepare()
        (self.root / "fixture_mode").write_text("save_failure")
        self.request("POST", "/sessions/session-1/start")
        code, reply = self.request("POST", "/sessions/session-1/stop")
        self.assertEqual((code, reply["state"]), (500, "failed"))
        code, stopped = self.request("POST", "/sessions/session-1/stop")
        self.assertEqual((code, stopped["state"]), (200, "failed"))

    def test_unexpected_exit_is_reported_on_status(self):
        self.prepare()
        (self.root / "fixture_mode").write_text("crash")
        self.request("POST", "/sessions/session-1/start")
        self.service.processes["session-1"].wait(timeout=2)
        code, reply = self.request("GET", "/status")
        self.assertEqual((code, reply["state"]), (200, "failed"))
        self.assertIn("意外退出", reply["error"])
        self.prepare("session-2")

    def test_clock_check_does_not_create_recording(self):
        code, reply = self.request("POST", "/sync")
        self.assertEqual((code, reply["state"]), (200, "idle"))
        self.assertEqual(reply["sync_summary"]["network_rtt_ms"], 1)
        self.assertEqual(len(list((self.root / "work/clock_checks").glob("*.json"))), 1)
        self.assertFalse(list((self.root / "data").iterdir()))

    def test_sync_after_coverage_and_idempotence(self):
        self.saved()
        code, reply = self.request("POST", "/sessions/session-1/sync-after")
        self.assertEqual(code, 200, reply)
        filename = reply["sync_after"]
        self.assertTrue(filename)
        self.assertEqual(self.request("POST", "/sessions/session-1/sync-after")[1]["sync_after"], filename)
        self.assertEqual(board.read_json(self.root / "data/session-1/recording.json")["sync_after"], filename)

    def test_sync_after_rejects_uncovered_camera_timestamp(self):
        self.saved()
        (self.root / "data/session-1/camera/frames.jsonl").write_text(json.dumps({"systemTime": time.time() + 1000}) + "\n")
        code, reply = self.request("POST", "/sessions/session-1/sync-after")
        self.assertEqual(code, 409)
        self.assertIsNone(reply["sync_after"])

    def test_storage_warning_threshold_and_recovery(self):
        for free in (30_000_000_000, 15_000_000_001, 15_000_000_000, 14_999_999_999, 20_000_000_000):
            with self.subTest(free=free), patch.object(board.shutil, "disk_usage", return_value=SimpleNamespace(free=free)):
                code, reply = self.request("GET", "/status")
                self.assertEqual(code, 200, reply)
                self.assertEqual(reply["storage_free_bytes"], free)
                if free < 15_000_000_000:
                    self.assertTrue(reply["storage_warning"])
                    self.assertIn("15 GB", reply["storage_warning"])
                    self.assertIn("清理", reply["storage_warning"])
                else:
                    self.assertIsNone(reply["storage_warning"])

    def test_full_disk_upload_returns_507_and_can_retry_after_cleanup(self):
        original_open = Path.open
        for error_number in (errno.ENOSPC, errno.EDQUOT):
            with self.subTest(errno=error_number):
                session = f"full-{error_number}"
                self.saved(session)
                metadata = self.metadata(session)
                self.assertEqual(self.upload("metadata.json", metadata, session)[0], 200)
                folder = self.root / "data" / session / "vp_recording"
                temporary = folder / "tracking_events.jsonl.uploading"

                @contextmanager
                def failing_open(path, *args, **kwargs):
                    with original_open(path, *args, **kwargs) as output:
                        if path == temporary:
                            original_write = output.write

                            def full_disk(data):
                                original_write(data[:3])
                                raise OSError(error_number, "fixture storage full")

                            with patch.object(output, "write", side_effect=full_disk):
                                yield output
                        else:
                            yield output

                with patch.object(Path, "open", failing_open):
                    code, reply = self.upload("tracking_events.jsonl", self.events(), session)
                self.assertEqual(code, 507, reply)
                for message in ("板", "空间不足", "VP", "原件", "导出", "清理", "重试"):
                    self.assertIn(message, reply["error"])
                self.assertFalse(reply.get("vp_uploaded", False))
                self.assertFalse(board.read_json(folder.parent / "recording.json")["vp_uploaded"])
                self.assertEqual([path.name for path in folder.iterdir()], ["metadata.json"])
                self.assertEqual((folder / "metadata.json").read_bytes(), metadata)
                code, reply = self.upload("tracking_events.jsonl", self.events(), session)
                self.assertEqual((code, reply["vp_uploaded"]), (200, True), reply)
                self.assertEqual((folder / "tracking_events.jsonl").read_bytes(), self.events())

    def test_storage_error_response_does_not_repeat_failed_status(self):
        with patch.object(self.service, "status", side_effect=OSError(errno.ENOSPC, "fixture storage full")) as status:
            code, reply = self.request("GET", "/status")
        self.assertEqual(code, 507, reply)
        self.assertIn("空间不足", reply["error"])
        status.assert_called_once()

    def test_other_os_error_is_not_reported_as_full_disk(self):
        self.saved()
        with patch.object(self.service, "upload", side_effect=OSError(errno.EIO, "fixture I/O failure")):
            code, reply = self.upload("metadata.json", self.metadata())
        self.assertEqual(code, 500, reply)
        self.assertIn("fixture I/O failure", reply["error"])
        self.assertNotIn("空间不足", reply["error"])
        self.assertFalse(reply["vp_uploaded"])

    def test_upload_is_complete_only_with_matched_pair(self):
        self.saved()
        self.assertFalse(self.upload("metadata.json", self.metadata())[1]["vp_uploaded"])
        code, reply = self.upload("tracking_events.jsonl", self.events())
        self.assertEqual((code, reply["vp_uploaded"]), (200, True))
        self.assertEqual(self.upload("metadata.json", self.metadata())[0], 200)
        code, reply = self.upload("metadata.json", self.metadata(recordingType="other____"))
        self.assertGreaterEqual(code, 400)
        self.assertTrue(reply["vp_uploaded"])
        self.assertEqual((self.root / "data/session-1/vp_recording/metadata.json").read_bytes(), self.metadata())

    def test_cross_session_board_and_equal_size_wrong_content_rejected(self):
        self.saved()
        self.assertEqual(self.upload("metadata.json", self.metadata("session-2"))[0], 409)
        self.assertEqual(self.upload("metadata.json", self.metadata(captureBoardID="board-2"))[0], 409)
        self.upload("metadata.json", self.metadata())
        self.upload("tracking_events.jsonl", self.events())
        changed = self.metadata().replace(b'"100"', b'"200"')
        self.assertEqual(len(changed), len(self.metadata()))
        self.assertEqual(self.upload("metadata.json", changed)[0], 409)
        self.assertEqual((self.root / "data/session-1/vp_recording/metadata.json").read_bytes(), self.metadata())

    def test_mismatched_events_do_not_replace_file_or_confirm_upload(self):
        self.saved()
        self.upload("metadata.json", self.metadata())
        code, reply = self.upload("tracking_events.jsonl", self.events(start=200))
        self.assertEqual((code, reply["vp_uploaded"]), (409, False))
        self.assertFalse((self.root / "data/session-1/vp_recording/tracking_events.jsonl").exists())
        self.assertEqual(self.upload("tracking_events.jsonl", self.events())[0], 200)

    def test_interrupted_upload_leaves_no_final_or_temporary_file(self):
        self.saved()
        data = self.metadata()
        with self.assertRaisesRegex(board.APIError, "上传中断"):
            self.service.upload("session-1", "metadata.json", io.BytesIO(data[:10]), len(data))
        folder = self.root / "data/session-1/vp_recording"
        self.assertFalse(list(folder.iterdir()))
        self.assertEqual(self.upload("metadata.json", data)[0], 200)

    def test_old_upload_does_not_hold_new_recording_control(self):
        self.saved()
        started, release = threading.Event(), threading.Event()
        class SlowStream(io.BytesIO):
            def read(self, size=-1):
                started.set()
                if not release.wait(3):
                    raise RuntimeError("test did not release upload")
                return super().read(size)
        errors = []
        def upload_old():
            try:
                data = self.metadata()
                self.service.upload("session-1", "metadata.json", SlowStream(data), len(data))
            except Exception as error:
                errors.append(error)
        worker = threading.Thread(target=upload_old)
        worker.start()
        self.assertTrue(started.wait(2))
        try:
            self.prepare("session-2")
            code, reply = self.request("POST", "/sessions/session-2/start")
            self.assertEqual((code, reply["session_id"], reply["state"]), (200, "session-2", "recording"))
        finally:
            release.set()
            worker.join(3)
        self.assertEqual(errors, [])
        self.assertEqual(self.request("GET", "/status")[1]["session_id"], "session-2")
        self.assertEqual(self.request("GET", "/status?session_id=session-1")[1]["state"], "saved")

    def test_recovery_marks_interrupted_round_failed_and_preserves_saved(self):
        self.saved()
        self.prepare("session-2")
        recovered = board.BoardService(self.root, "board-1", "采集板 01")
        self.assertEqual(recovered.status("session-1")["state"], "saved")
        self.assertEqual(recovered.status("session-2")["state"], "failed")
        self.assertTrue((self.root / "data/session-1/camera/video.mov").exists())

    def test_failed_stop_confirms_termination_and_allows_next_round(self):
        with patch.object(self.service, "measure", side_effect=ValueError("未收到 VP 对钟回复")):
            code, reply = self.request("POST", "/sessions", {"session_id": "session-1"})
        self.assertEqual((code, reply["state"]), (400, "failed"))
        for _ in range(2):
            code, stopped = self.request("POST", "/sessions/session-1/stop")
            self.assertEqual((code, stopped["state"]), (200, "failed"))
            self.assertEqual(stopped["error"], "未收到 VP 对钟回复")
        self.prepare("session-2")

    def test_failed_stop_finishes_any_remaining_managed_process(self):
        self.prepare()
        self.request("POST", "/sessions/session-1/start")
        process = self.service.processes["session-1"]
        self.service.change(self.service.record("session-1"), state="failed", error="联合采集已失败")
        code, reply = self.request("POST", "/sessions/session-1/stop")
        self.assertEqual((code, reply["state"], reply["error"]), (200, "failed", "联合采集已失败"))
        self.assertIsNotNone(process.poll())
        self.assertFalse(self.service.processes)

    def test_wrong_board_header_rejected_before_mutation(self):
        code, reply = self.request("POST", "/sessions", {"session_id": "wrong-board"}, {"X-Capture-Board-ID": "board-2"})
        self.assertEqual(code, 409)
        self.assertFalse(self.service.records)
        self.assertEqual(reply["board_id"], "board-1")

    def test_invalid_routes_ids_and_json_return_json_errors(self):
        for method, path, body in [("POST", "/sessions", {"session_id": "../outside"}),
                                   ("POST", "/sessions", {"session_id": []}),
                                   ("POST", "/sync", b"{bad"),
                                   ("GET", "/unknown", None),
                                   ("PUT", "/sessions/unknown/files/run.sh", b"echo bad")]:
            code, reply = self.request(method, path, body)
            self.assertGreaterEqual(code, 400)
            self.assertIn("board_id", reply)
            self.assertTrue(reply["error"])
        self.assertFalse(list((self.root / "data").iterdir()))

    def test_missing_camera_and_clock_failure_are_explicit(self):
        with patch.object(self.service, "choose_camera", side_effect=ValueError("没有发现相机")):
            code, reply = self.request("POST", "/sessions", {"session_id": "session-1"})
        self.assertEqual(code, 400)
        self.assertEqual(reply["error"], "没有发现相机")
        with patch.object(self.service, "measure", side_effect=ValueError("未收到 VP 对钟回复")):
            code, reply = self.request("POST", "/sessions", {"session_id": "session-1"})
        self.assertEqual((code, reply["state"]), (400, "failed"))
        self.assertTrue((self.root / "data/session-1/calibration").exists())


if __name__ == "__main__":
    unittest.main()
