"""日常 CLI 的离线流程验收；不访问网络，不启动相机。"""
import contextlib
from datetime import datetime
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ego_cli
import sync_clocks


class EgoCliTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve() / "kit"
        self.root.mkdir()
        self.root_patch = mock.patch.object(ego_cli, "ROOT", self.root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)
        self.base = datetime(2026, 9, 16, 18, 23).timestamp()
        self.ip = "192.168.1.23"
        self.command = ["sync", "--vp", self.ip]
        self.install_calibration()

    def write_json(self, path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data), encoding="utf-8")

    def install_calibration(self):
        identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        eye = {"fx": 1000, "fy": 1000, "cx": 960, "cy": 600, "dist": [0] * 5}
        self.write_json(self.root / "calibration/stereo_calibration.json", {
            "image_size": [1920, 1200], "convert_meta": {"convention": "opencv_raw"},
            "recording_geometry": {"sbs_order": "imu_right_left", "rotation_deg": 0,
                                   "left_crop": [2080, 0, 1920, 1200], "right_crop": [160, 0, 1920, 1200]},
            "left_intrinsics": eye, "right_intrinsics": eye,
            "stereo_extrinsics": {"R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "T": [60, 0, 0]},
        })
        self.write_json(self.root / "calibration/ego_extrinsics.json", {
            "T_left_from_vp": identity, "T_right_from_vp": identity,
            "intrinsics_source": "/old-machine/calibration/stereo_calibration.json",
        })
        self.write_json(self.root / "ego_config.json", {
            "intrinsics": "calibration/stereo_calibration.json",
            "calibration": "calibration/ego_extrinsics.json", "camera_unique_id": None,
        })

    def measurement(self, seconds):
        started = self.base + seconds
        exchanges = []
        for seq in range(6):
            t1 = started + seq * .01
            exchanges.append({"seq": seq, "clock": "unix", "t1": t1,
                              "t2": t1 + .251, "t3": t1 + .252, "t4": t1 + .003})
        return {"schema_version": 1, "kind": "clock_sync", "clock": "unix", "status": "completed",
                "vp_ip": self.ip, "started_system_time": started, "finished_system_time": started + .1,
                "exchanges": exchanges, "summary": sync_clocks.summarize(exchanges)}

    def invoke(self, argv):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return ego_cli.main(argv)

    def before(self, seconds=1):
        with mock.patch.object(sync_clocks, "measure", return_value=self.measurement(seconds)):
            self.assertEqual(self.invoke(self.command), 0)
        return ego_cli.current_recording()

    def finish_camera(self, recording, first=10, last=20):
        self.write_json(recording / "camera/session.json", {
            "status": "completed", "schema_version": 2, "alignment_clock": "unix",
            "frame_unix_field": "systemTime",
        })
        frames = [{"frame_index": index, "pts_seconds": seconds - first,
                   "capture_host_seconds": 1000 + seconds, "received_host_seconds": 1000 + seconds + .01,
                   "systemTime": self.base + seconds} for index, seconds in enumerate((first, last))]
        (recording / "camera/frames.jsonl").write_text(
            "".join(json.dumps(frame) + "\n" for frame in frames), encoding="utf-8")

    def devices(self):
        return [
            {"name": "Built-in Camera", "unique_id": "builtin", "external": False,
             "supports_4000x1200": True},
            {"name": "DECXIN Camera", "unique_id": "usb-a", "external": True,
             "supports_4000x1200": True},
        ]

    def test_identical_command_completes_two_rounds_in_one_minute_without_overwrite(self):
        with mock.patch.object(sync_clocks, "measure", side_effect=[self.measurement(s) for s in (1, 30, 35, 50)]):
            self.assertEqual(self.invoke(self.command), 0)
            first = ego_cli.current_recording()
            self.finish_camera(first)
            self.assertEqual(self.invoke(self.command), 0)
            self.assertEqual(ego_cli.current_recording(), first)
            first_files = {p.name: p.read_bytes() for p in first.glob("*sync*.json")}
            self.assertEqual(set(first_files), {"2609161823sync.json", "2609161823sync_02.json"})
            self.assertEqual(ego_cli.manifest(first)["sync_before"], "2609161823sync.json")
            self.assertEqual(ego_cli.manifest(first)["sync_after"], "2609161823sync_02.json")
            self.assertEqual(self.invoke(self.command), 0)
            second = ego_cli.current_recording()
            self.assertNotEqual(first, second)
            self.assertEqual(second.name, "2609161823_02")
            self.finish_camera(second, 40, 45)
            self.assertEqual(self.invoke(self.command), 0)
        self.assertEqual({p.name: p.read_bytes() for p in first.glob("*sync*.json")}, first_files)
        self.assertEqual(len(list(second.glob("*sync*.json"))), 2)
        self.assertTrue((first / "vp_recording").is_dir())
        self.assertTrue((second / "vp_recording").is_dir())

    def test_repeating_before_capture_keeps_round_and_uses_latest_earlier_measurement(self):
        first = self.before(1)
        original = (first / "2609161823sync.json").read_bytes()
        self.assertEqual(self.before(2), first)
        self.assertEqual((first / "2609161823sync.json").read_bytes(), original)
        self.finish_camera(first)
        with mock.patch.object(sync_clocks, "measure", return_value=self.measurement(30)):
            self.assertEqual(self.invoke(self.command), 0)
        self.assertEqual(ego_cli.manifest(first)["sync_before"], "2609161823sync_02.json")
        self.assertEqual(ego_cli.manifest(first)["sync_after"], "2609161823sync_03.json")

    def test_failed_initial_measurement_does_not_create_round_or_pointer(self):
        with mock.patch.object(sync_clocks, "measure", side_effect=ValueError("no reply")):
            self.assertEqual(self.invoke(self.command), 1)
        self.assertFalse((self.root / "data").exists())
        self.assertFalse((self.root / ".ego_current").exists())

    def test_failed_post_measurement_keeps_manifest_pointer_and_files(self):
        recording = self.before()
        self.finish_camera(recording)
        files = {p.relative_to(self.root): p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        with mock.patch.object(sync_clocks, "measure", side_effect=ValueError("unstable clock")):
            self.assertEqual(self.invoke(self.command), 1)
        self.assertEqual({p.relative_to(self.root): p.read_bytes() for p in self.root.rglob("*") if p.is_file()}, files)
        self.assertNotIn("sync_after", ego_cli.manifest(recording))

    def test_failed_next_round_does_not_replace_completed_current_round(self):
        recording = self.before()
        self.finish_camera(recording)
        with mock.patch.object(sync_clocks, "measure", return_value=self.measurement(30)):
            self.assertEqual(self.invoke(self.command), 0)
        with mock.patch.object(sync_clocks, "measure", side_effect=OSError("network failed")):
            self.assertEqual(self.invoke(self.command), 1)
        self.assertEqual(ego_cli.current_recording(), recording)
        self.assertEqual(len(list((self.root / "data").iterdir())), 1)

    def test_unfinished_capture_fails_before_network_and_explicit_new_preserves_old_round(self):
        old = self.before()
        self.write_json(old / "camera/session.json", {"status": "recording"})
        with mock.patch.object(sync_clocks, "measure") as measure:
            self.assertEqual(self.invoke(self.command), 1)
            measure.assert_not_called()
        with mock.patch.object(sync_clocks, "measure", return_value=self.measurement(2)):
            self.assertEqual(self.invoke(self.command + ["--new"]), 0)
        self.assertNotEqual(ego_cli.current_recording(), old)
        self.assertEqual(ego_cli.read_json(old / "camera/session.json")["status"], "recording")

    def test_clock_pair_uses_contents_for_legacy_and_timestamp_names(self):
        directory = self.root / "legacy"
        # 故意让文件名字典序与实际时间完全无关。
        for filename, seconds in (("sync_before.json", 2), ("2601010000sync.json", 30),
                                  ("9912312359sync.json", 1), ("sync_after.json", 35),
                                  ("unrelated.json", 15)):
            data = self.measurement(seconds) if filename != "unrelated.json" else {"note": "ignore me"}
            self.write_json(directory / filename, data)
        before, after = ego_cli.select_clock_pair(directory, self.base + 10, self.base + 20)
        self.assertEqual(before.name, "sync_before.json")
        self.assertEqual(after.name, "2601010000sync.json")

    def test_clock_pair_rejects_recording_outside_measurement_coverage(self):
        directory = self.root / "legacy"
        self.write_json(directory / "sync_before.json", self.measurement(1))
        self.write_json(directory / "sync_after.json", self.measurement(15))
        with self.assertRaisesRegex(ValueError, "录前/录后"):
            ego_cli.select_clock_pair(directory, self.base + 10, self.base + 20)

    def test_saved_camera_identity_survives_reordering_and_never_chooses_builtin(self):
        devices = self.devices()
        self.assertEqual(ego_cli.choose_camera(devices, None, None)["unique_id"], "usb-a")
        self.assertEqual(ego_cli.choose_camera(list(reversed(devices)), None, "usb-a")["unique_id"], "usb-a")
        with self.assertRaises(ValueError):
            ego_cli.choose_camera(devices, "0", None)
        with self.assertRaises(ValueError):
            ego_cli.choose_camera(devices, "builtin", None)

    def test_missing_saved_camera_does_not_silently_switch_to_another_external_camera(self):
        devices = self.devices()
        with self.assertRaisesRegex(ValueError, "未找到相机，可尝试重启开发板"):
            ego_cli.choose_camera(devices, None, "disconnected-camera")
        self.assertEqual(ego_cli.choose_camera(devices, "usb-a", "disconnected-camera")["unique_id"], "usb-a")

    def test_multiple_decxin_cameras_require_explicit_choice(self):
        devices = self.devices()
        devices.append({**devices[1], "unique_id": "usb-b"})
        with self.assertRaisesRegex(ValueError, "2 台"):
            ego_cli.choose_camera(devices, None, None)
        self.assertEqual(ego_cli.choose_camera(devices, "2", None)["unique_id"], "usb-b")

    def test_record_creates_camera_next_to_sync_and_remembers_unique_id(self):
        recording = self.before()
        commands = []

        def run(command, **kwargs):
            commands.append(command)
            if command[-1] == "--list-json":
                return types.SimpleNamespace(stdout=json.dumps(self.devices()))
            self.assertEqual(command, [str(self.root / ("start_capture_linux.sh" if sys.platform.startswith("linux") else "start_capture.command")), "--camera", "usb-a",
                                       "--out", str(recording / "camera")])
            self.finish_camera(recording)
            return types.SimpleNamespace(returncode=0)

        with mock.patch.object(ego_cli.subprocess, "run", side_effect=run):
            self.assertEqual(self.invoke(["record"]), 0)
        self.assertEqual(len(commands), 2)
        self.assertEqual(ego_cli.config()["camera_unique_id"], "usb-a")
        self.assertTrue((recording / "camera/session.json").is_file())
        self.assertTrue((recording / ego_cli.manifest(recording)["sync_before"]).is_file())

    def test_record_list_selects_native_backend(self):
        for platform, script in (("linux", "start_capture_linux.sh"), ("darwin", "start_capture.command")):
            with self.subTest(platform=platform), mock.patch.object(ego_cli.sys, "platform", platform), \
                    mock.patch.object(ego_cli.subprocess, "run") as run:
                self.assertEqual(self.invoke(["record", "--list"]), 0)
                run.assert_called_once_with([str(self.root / script), "--list"], check=True)

    def test_record_refuses_existing_camera_data(self):
        recording = self.before()
        self.finish_camera(recording)
        with mock.patch.object(ego_cli.subprocess, "run") as run:
            self.assertEqual(self.invoke(["record"]), 1)
            run.assert_not_called()

    def test_process_requires_kept_review_but_accepts_legacy_recordings(self):
        recording = self.before()
        self.finish_camera(recording)
        with mock.patch.object(sync_clocks, "measure", return_value=self.measurement(30)):
            self.assertEqual(self.invoke(self.command), 0)
        self.write_json(recording / "vp_recording/metadata.json", {})
        (recording / "vp_recording/tracking_events.jsonl").write_text("{}\n", encoding="utf-8")
        original_manifest = ego_cli.manifest(recording)
        originals = {path.relative_to(recording): path.read_bytes() for path in recording.rglob("*")
                     if path.is_file() and path.name != "recording.json"}
        for decision in ("pending", "discarded", "kept", None):
            with self.subTest(review_state=decision):
                info = dict(original_manifest)
                if decision is not None:
                    info["review_state"] = decision
                self.write_json(recording / "recording.json", info)
                processor = mock.Mock()
                with mock.patch.dict(sys.modules, {"process_recording": types.SimpleNamespace(process=processor)}):
                    self.assertEqual(self.invoke(["process"]), 1 if decision in {"pending", "discarded"} else 0)
                if decision in {"pending", "discarded"}:
                    processor.assert_not_called()
                else:
                    processor.assert_called_once()
        self.assertEqual(originals, {path.relative_to(recording): path.read_bytes() for path in recording.rglob("*")
                                     if path.is_file() and path.name != "recording.json"})

    def test_moved_kit_process_uses_relative_pointer_and_original_per_round_calibration(self):
        recording = self.before()
        self.finish_camera(recording)
        with mock.patch.object(sync_clocks, "measure", return_value=self.measurement(30)):
            self.assertEqual(self.invoke(self.command), 0)
        self.write_json(recording / "vp_recording/metadata.json", {})
        (recording / "vp_recording/tracking_events.jsonl").write_text("{}\n", encoding="utf-8")
        relative_recording = recording.relative_to(self.root)
        saved_i = (recording / "calibration/stereo_calibration.json").read_bytes()
        saved_e = (recording / "calibration/ego_extrinsics.json").read_bytes()
        self.assertFalse(Path((self.root / ".ego_current").read_text().strip()).is_absolute())
        self.assertEqual(ego_cli.read_json(recording / "calibration/ego_extrinsics.json")["intrinsics_source"],
                         "stereo_calibration.json")
        # 整包搬家后，默认参数再被修改或移走，也不能改变已完成轮次。
        moved = Path(self.temporary.name).resolve() / "moved kit"
        shutil.move(str(self.root), moved)
        self.root = moved
        self.write_json(moved / "ego_config.json", {"intrinsics": "missing.json", "calibration": "missing.json"})
        shutil.rmtree(moved / "calibration")
        processor = mock.Mock()
        with mock.patch.object(ego_cli, "ROOT", moved), mock.patch.dict(
                sys.modules, {"process_recording": types.SimpleNamespace(process=processor)}):
            self.assertEqual(ego_cli.current_recording(), moved / relative_recording)
            self.assertEqual(self.invoke(["process"]), 0)
        relocated = moved / relative_recording
        args, kwargs = processor.call_args
        self.assertEqual(args[:3], (relocated, relocated / "calibration/ego_extrinsics.json",
                                   relocated / "calibration/stereo_calibration.json"))
        self.assertEqual(kwargs, {"output_dir": None})
        self.assertEqual((relocated / "calibration/stereo_calibration.json").read_bytes(), saved_i)
        self.assertEqual((relocated / "calibration/ego_extrinsics.json").read_bytes(), saved_e)


if __name__ == "__main__":
    unittest.main()
