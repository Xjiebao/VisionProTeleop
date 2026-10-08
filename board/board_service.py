#!/usr/bin/env python3
"""VP 控制 Linux 相机、对钟和接收原始文件；不执行离线对齐。"""
import argparse
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
from pathlib import Path
import re
import select
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
from urllib.parse import parse_qs, urlsplit

import ego_cli
import sync_clocks

SESSION_ID = re.compile(r"[A-Za-z0-9_-]{1,100}\Z")
UPLOAD_NAMES = {"metadata.json", "tracking_events.jsonl"}
ACTIVE_STATES = {"prepared", "starting", "recording"}
CANCELLED = "采集已取消，尚未开始相机录像"
STORAGE_WARNING_BYTES = 15_000_000_000


class APIError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


class BoardService:
    def __init__(self, root, board_id=None, board_name=None):
        self.root = Path(root).resolve()
        self.board_id = board_id or Path("/etc/machine-id").read_text().strip()
        self.board_name = board_name or socket.gethostname()
        self.control_lock = threading.Lock()
        self.manifest_lock = threading.RLock()
        self.upload_locks = {}
        self.processes = {}
        self.calibration_processes = {}
        self.stopping = set()
        self.records = {}
        self.current = None
        self.start_timeout = 30
        self.stop_timeout = 60
        self.preview_enabled = False
        self.preview_directory = None
        self.preview_camera = None
        self.preview_worker = None
        self.preview_session = None
        self.preview_last_request = 0
        self.preview_shutdown = threading.Event()
        self.preview_watchdog = None
        (self.root / "data").mkdir(exist_ok=True)
        # 中断的录制不自动续录，旧版工具生成的轮次不改动。
        for path in (self.root / "data").glob("*/recording.json"):
            info = read_json(path)
            if info.get("board_service_version") != 1:
                continue
            session_id = info["session_id"]
            self.validate_id(session_id)
            if path.parent.name != session_id or info.get("board_id") != self.board_id:
                continue
            self.records[session_id] = info
            if info["state"] in ACTIVE_STATES:
                info.update(state="failed", error="板子服务曾中断；本轮未确认完成，原始文件已保留，请新建一轮")
                self.persist(info)
            if info.get("calibration", {}).get("state") == "running":
                info["calibration"].update(state="failed", error="板子服务曾中断，外参计算未确认完成；可重新计算", active=False)
                self.persist(info)
            if self.current is None or info["created_system_time"] > self.records[self.current]["created_system_time"]:
                self.current = session_id

    @staticmethod
    def validate_id(session_id):
        if not isinstance(session_id, str) or not SESSION_ID.fullmatch(session_id):
            raise APIError(400, "session_id 只能包含 1–100 个字母、数字、下划线或连字符")

    def record(self, session_id):
        self.validate_id(session_id)
        with self.manifest_lock:
            if session_id not in self.records:
                raise APIError(404, "找不到本服务的指定采集轮次")
            return self.records[session_id]

    def folder(self, session_id):
        self.validate_id(session_id)
        return self.root / "data" / session_id

    def persist(self, info):
        with self.manifest_lock:
            write_json(self.folder(info["session_id"]) / "recording.json", info)

    def change(self, info, **values):
        with self.manifest_lock:
            info.update(values)
            self.persist(info)

    def refresh(self):
        with self.manifest_lock:
            for session_id, process in list(self.processes.items()):
                info = self.records[session_id]
                if info["state"] == "recording" and session_id not in self.stopping and process.poll() is not None:
                    self.change(info, state="failed", error=f"相机进程意外退出（代码 {process.returncode}）；请检查本轮 capture.log 和 camera/session.json")
                    self.close_process(session_id)
            for session_id, process in list(self.calibration_processes.items()):
                if process.poll() is None:
                    continue
                info = self.records[session_id]
                calibration = dict(info["calibration"])
                try:
                    if process.returncode != 0:
                        log = self.root / calibration["log_path"]
                        detail = log.read_text(encoding="utf-8", errors="replace")[-2000:].strip()
                        raise ValueError(f"外参计算失败（退出代码 {process.returncode}）；{detail}")
                    result = read_json(self.root / calibration["output_path"])
                    validation = result["quality"]["validation"]
                    translations = [float(validation[eye]["translation_mm"]["rms"]) for eye in ("left", "right")]
                    rotations = [float(validation[eye]["rotation_deg"]["rms"]) for eye in ("left", "right")]
                    if not all(math.isfinite(value) and value >= 0 for value in translations + rotations):
                        raise ValueError("外参结果的验证残差不是有效数值")
                    calibration.update(state="completed", error=None,
                                       sample_count=len(result["training_ids"]) + len(result["validation_ids"]),
                                       validation_count=len(result["validation_ids"]), warnings=result.get("warnings", []),
                                       validation_translation_mm=max(translations), validation_rotation_deg=max(rotations))
                except (OSError, ValueError, KeyError, TypeError) as error:
                    calibration.update(state="failed", error=str(error), active=False)
                self.change(info, calibration=calibration)
                self.calibration_processes.pop(session_id, None)

    def status(self, session_id=None):
        self.refresh()
        free_bytes = shutil.disk_usage(self.root / "data").free
        with self.manifest_lock:
            info = self.record(session_id) if session_id is not None else self.records.get(self.current)
            result = {"board_id": self.board_id, "board_name": self.board_name,
                      "state": info["state"] if info else "idle",
                      "session_id": info["session_id"] if info else None,
                      "error": info.get("error") if info else None,
                      "review_state": info.get("review_state", "kept") if info else None,
                      "vp_uploaded": bool(info and info.get("vp_uploaded")),
                      "sync_after": info.get("sync_after") if info else None,
                      "storage_free_bytes": free_bytes,
                      "storage_warning": (f"采集板仅剩 {free_bytes / 1_000_000_000:.1f} GB（提醒线 15 GB）。"
                                          "请结束本轮后导出并清理板端数据，避免录制写满。"
                                          if free_bytes < STORAGE_WARNING_BYTES else None)}
            if info and info.get("sync_summary"):
                result["sync_summary"] = info["sync_summary"]
            if info and info.get("calibration"):
                calibration = dict(info["calibration"])
                settings = read_json(self.root / "ego_config.json")
                calibration["active"] = (calibration["state"] == "completed"
                                         and (self.root / settings["calibration"]).resolve() == (self.root / calibration["output_path"]).resolve()
                                         and (self.root / settings["intrinsics"]).resolve() == (self.root / calibration["intrinsics_path"]).resolve())
                if calibration != info["calibration"]:
                    self.change(info, calibration=calibration)
                result["calibration"] = calibration
            return result

    def measure(self, peer):
        return sync_clocks.measure(str(ipaddress.IPv4Address(peer)))

    def sync(self, peer):
        with self.control_lock:
            self.refresh()
            if any(info["state"] in {"starting", "recording"} for info in self.records.values()):
                raise APIError(409, "录像期间不能单独检测延迟；请结束后重试")
            result = self.measure(peer)
            directory = self.root / "work" / "clock_checks"
            directory.mkdir(parents=True, exist_ok=True)
            output = sync_clocks.timestamped_path(directory, result["started_system_time"])
            sync_clocks.save_measurement(result, output)
            return dict(self.status(), sync_summary=result["summary"])

    def choose_camera(self):
        result = subprocess.run([str(self.root / "start_capture_linux.sh"), "--list-json"],
                                check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
        settings = read_json(self.root / "ego_config.json")
        device = ego_cli.choose_camera(json.loads(result.stdout), None, settings.get("camera_unique_id"))
        return device["unique_id"]

    def release_preview_processes(self):
        # The recording process is deliberately not part of this list.
        for name in ("preview_camera", "preview_worker"):
            process = getattr(self, name)
            if process is not None:
                self.terminate(process)
                setattr(self, name, None)

    def preview_expiry_loop(self):
        while not self.preview_shutdown.wait(1):
            with self.control_lock:
                if self.preview_enabled and time.monotonic() - self.preview_last_request > 15:
                    self.preview_enabled = False
                    self.release_preview_processes()

    def launch_preview_worker(self, session_id=None):
        folder = Path(self.preview_directory.name)
        for name in ("latest.jpg", "live.json"):
            (folder / name).unlink(missing_ok=True)
        settings = read_json(self.root / "ego_config.json")
        calibration = (self.folder(session_id) / "calibration/ego_extrinsics.json"
                       if session_id else self.root / settings["calibration"])
        intrinsics = (self.folder(session_id) / "calibration/stereo_calibration.json"
                      if session_id else self.root / settings["intrinsics"])
        with (folder / "preview.log").open("ab", buffering=0) as log:
            self.preview_worker = subprocess.Popen([
                str(self.root / ".venv/bin/python"), str(self.root / "calibration_preview.py"),
                "--source", str(folder), "--intrinsics", str(intrinsics),
                "--calibration", str(calibration)], stdin=subprocess.DEVNULL, stdout=log, stderr=log)

    def start_standalone_preview(self):
        self.release_preview_processes()
        camera_id = self.choose_camera()
        self.preview_session = None
        self.launch_preview_worker()
        folder = Path(self.preview_directory.name)
        try:
            with (folder / "preview.log").open("ab", buffering=0) as log:
                self.preview_camera = subprocess.Popen([
                    str(self.root / "start_capture_linux.sh"), "--camera", camera_id,
                    "--preview-only", "--preview", str(folder)],
                    stdin=subprocess.DEVNULL, stdout=log, stderr=log)
        except OSError:
            self.release_preview_processes()
            raise

    def live_preview(self, action=None):
        with self.control_lock:
            self.refresh()
            self.preview_last_request = time.monotonic()
            active = next((info for info in self.records.values() if info["state"] in ACTIVE_STATES), None)
            recording = bool(active and active["state"] == "recording")
            if action == "stop":
                self.preview_enabled = False
                self.release_preview_processes()
            elif action == "start":
                if self.calibration_processes:
                    raise APIError(409, "外参计算中，完成后可重新打开取景预览")
                if recording and active["session_id"] != self.preview_session:
                    raise APIError(409, "请结束当前录像，再打开标定预览并开始新一轮")
                if self.preview_directory is None:
                    self.preview_directory = tempfile.TemporaryDirectory(
                        prefix="egocapture-preview-", dir="/dev/shm" if Path("/dev/shm").is_dir() else None)
                was_enabled = self.preview_enabled
                self.preview_enabled = True
                if self.preview_watchdog is None:
                    self.preview_watchdog = threading.Thread(target=self.preview_expiry_loop, daemon=True)
                    self.preview_watchdog.start()
                if not was_enabled or any(process is not None and process.poll() is not None
                                          for process in (self.preview_camera, self.preview_worker)):
                    self.release_preview_processes()
                    if recording:
                        self.launch_preview_worker(active["session_id"])
                    elif active is None:
                        self.start_standalone_preview()
            # After recording, return to framing without retaining the previous recording's count.
            if (self.preview_enabled and active is None and
                    (self.preview_session is not None or (self.preview_camera is None and self.preview_worker is None))):
                self.start_standalone_preview()
            payload = {"board_id": self.board_id, "state": "starting" if self.preview_enabled else "stopped",
                       "recording": recording, "session_id": active["session_id"] if active else None,
                       "age_seconds": None, "frame_time": None, "image_base64": None,
                       "left_detected": False, "right_detected": False,
                       "stable_seconds": 0., "pose_count": 0, "error": None,
                       "guidance": "正在准备相机画面" if self.preview_enabled else "预览已停止，可点击重新预览"}
            if not self.preview_enabled:
                return payload
            if active and not recording:
                payload["guidance"] = "正在切换到正式录像，请稍候"
                return payload
            folder = Path(self.preview_directory.name)
            output = folder / "live.json"
            if output.is_file():
                payload.update(read_json(output))
                age = max(0., time.time() - payload["frame_time"]) if payload["frame_time"] is not None else None
                payload["age_seconds"] = age
                payload["state"] = "failed" if payload["error"] else "live" if age is not None and age <= 3 else "stale"
            for process in (self.preview_camera, self.preview_worker):
                if process is not None and process.poll() is not None:
                    payload.update(state="failed", error=payload["error"] or
                                   "相机预览进程已退出：" + (folder / "preview.log").read_text(errors="replace")[-400:])
            if payload["state"] != "live":
                payload.update(left_detected=False, right_detected=False, stable_seconds=0.)
                if payload["state"] == "stale":
                    payload["guidance"] = "画面已过期，请检查相机或重新预览"
            return payload

    def snapshot_calibration(self, folder):
        settings = read_json(self.root / "ego_config.json")
        source_i = (self.root / settings["intrinsics"]).resolve()
        source_e = (self.root / settings["calibration"]).resolve()
        read_json(source_i)
        extrinsics = read_json(source_e)
        destination = folder / "calibration"
        destination.mkdir()
        shutil.copyfile(source_i, destination / "stereo_calibration.json")
        extrinsics["intrinsics_source"] = "stereo_calibration.json"
        write_json(destination / "ego_extrinsics.json", extrinsics)

    def prepare(self, session_id, peer):
        self.validate_id(session_id)
        with self.control_lock:
            self.refresh()
            if session_id in self.records:
                return self.status(session_id)
            if self.calibration_processes:
                raise APIError(409, "外参计算尚未完成，暂不能开始新采集")
            if any(info["state"] in ACTIVE_STATES for info in self.records.values()):
                raise APIError(409, "已有未结束的轮次，请先在 VP 上结束该轮")
            self.release_preview_processes()
            camera_id = self.choose_camera()
            folder = self.folder(session_id)
            if folder.exists():
                raise APIError(409, "同名目录已存在，不能覆盖；请新建一轮")
            folder.mkdir()
            info = {"schema_version": 1, "board_service_version": 1,
                    "board_id": self.board_id, "session_id": session_id,
                    "created_system_time": time.time(), "vp_ip": str(ipaddress.IPv4Address(peer)),
                    "camera_unique_id": camera_id, "state": "prepared", "error": None, "vp_uploaded": False,
                    "review_state": "pending"}
            with self.manifest_lock:
                self.records[session_id] = info
                self.current = session_id
            self.persist(info)
            try:
                self.snapshot_calibration(folder)
                (folder / "vp_recording").mkdir()
                result = self.measure(peer)
                sync_clocks.save_measurement(result, folder / "sync_before.json")
                self.change(info, sync_before="sync_before.json", sync_summary=result["summary"])
            except Exception as error:
                self.change(info, state="failed", error=str(error))
                raise
            return self.status(session_id)

    def launch(self, info):
        folder = self.folder(info["session_id"])
        command = [str(self.root / "start_capture_linux.sh"), "--camera", info["camera_unique_id"],
                   "--out", str(folder / "camera")]
        if self.preview_enabled:
            self.release_preview_processes()
            self.preview_session = info["session_id"]
            self.launch_preview_worker(info["session_id"])
            command.extend(["--preview", self.preview_directory.name])
        return subprocess.Popen(command, stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)

    def close_process(self, session_id):
        process = self.processes.pop(session_id, None)
        if process:
            for stream in (process.stdin, process.stdout):
                if stream:
                    stream.close()

    def wait_ready(self, process, log):
        deadline = time.monotonic() + self.start_timeout
        pending = b""
        while time.monotonic() < deadline:
            readable, _, _ = select.select([process.stdout], [], [], min(.1, max(0., deadline - time.monotonic())))
            if readable:
                chunk = process.stdout.read(4096)
                if chunk:
                    log.write(chunk)
                    pending += chunk
                    if b"CAPTURE_READY\n" in pending:
                        if process.poll() is not None:
                            raise APIError(500, "相机已写首帧，但随后立即退出；请检查 capture.log")
                        return
                    pending = pending[-4096:]
            if process.poll() is not None:
                raise APIError(500, f"相机启动失败（退出代码 {process.returncode}）；请检查本轮 capture.log")
        raise APIError(504, "等待相机首帧超时；本轮已停止，请检查相机和 capture.log")

    def terminate(self, process):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)

    def start(self, session_id):
        with self.control_lock:
            self.refresh()
            info = self.record(session_id)
            if info["state"] == "recording":
                return self.status(session_id)
            if self.calibration_processes:
                raise APIError(409, "外参计算尚未完成，暂不能开始相机录像")
            if info["state"] != "prepared" or not info.get("sync_before"):
                raise APIError(409, "本轮不在可开始状态；已结束或失败的轮次不能重新录像")
            self.change(info, state="starting", error=None)
            process = None
            try:
                process = self.launch(info)
                self.processes[session_id] = process
                with (self.folder(session_id) / "capture.log").open("ab", buffering=0) as log:
                    self.wait_ready(process, log)
                self.change(info, state="recording")
            except Exception as error:
                if process:
                    self.terminate(process)
                    self.close_process(session_id)
                log_path = self.folder(session_id) / "capture.log"
                detail = log_path.read_text(encoding="utf-8", errors="replace")[-2000:].strip() if log_path.exists() else ""
                message = str(error) + ("；相机输出：" + detail if detail else "")
                self.change(info, state="failed", error=message)
                raise APIError(error.status if isinstance(error, APIError) else 500, message) from error
            return self.status(session_id)

    def camera_completed(self, session_id):
        camera = self.folder(session_id) / "camera"
        info = read_json(camera / "session.json")
        if info.get("status") != "completed" or info.get("frames_written", 0) < 1:
            raise APIError(500, f"相机未确认保存完成：{info.get('error') or info.get('status')}")
        if not (camera / "video.mov").is_file() or not (camera / "frames.jsonl").is_file():
            raise APIError(500, "相机完成记录存在，但缺少视频或逐帧时间文件")
        return info

    def stop(self, session_id):
        with self.control_lock:
            self.refresh()
            info = self.record(session_id)
            if info["state"] == "saved":
                self.camera_completed(session_id)
                return self.status(session_id)
            if info["state"] == "prepared" or info.get("error") == CANCELLED:
                self.change(info, state="failed", error=CANCELLED)
                return self.status(session_id)
            previously_failed = info["state"] == "failed"
            previous_error = info.get("error") if previously_failed else None
            process = self.processes.get(session_id)
            if info["state"] == "failed" and (process is None or process.poll() is not None):
                self.close_process(session_id)
                return self.status(session_id)
            if info["state"] not in {"recording", "failed"} or process is None:
                raise APIError(409, info.get("error") or "本轮没有正在录制的相机")
            self.stopping.add(session_id)
            try:
                process.stdin.write(b"\n")
                process.stdin.flush()
                output, _ = process.communicate(timeout=self.stop_timeout)
                with (self.folder(session_id) / "capture.log").open("ab") as log:
                    log.write(output or b"")
                if process.returncode != 0:
                    detail = (output or b"").decode("utf-8", errors="replace")[-2000:].strip()
                    raise APIError(500, f"相机停止失败（退出代码 {process.returncode}）；原文件已保留；{detail}")
                self.camera_completed(session_id)
                self.change(info, state="failed" if previously_failed else "saved",
                            error=previous_error, saved_system_time=time.time())
            except Exception as error:
                self.terminate(process)
                self.change(info, state="failed", error=str(error))
                raise
            finally:
                self.stopping.discard(session_id)
                self.close_process(session_id)
            return self.status(session_id)

    def sync_after(self, session_id, peer):
        with self.control_lock:
            info = self.record(session_id)
            if info["state"] != "saved":
                raise APIError(409, "必须等相机确认保存完成后再做录后对钟")
            if peer != info["vp_ip"]:
                raise APIError(409, "本轮 VP 地址已变化，无法确认仍是同一台 VP；请保留数据并核查")
            if info.get("sync_after"):
                return self.status(session_id)
            self.camera_completed(session_id)
            folder = self.folder(session_id)
            result = self.measure(peer)
            output = sync_clocks.timestamped_path(folder, result["started_system_time"])
            sync_clocks.save_measurement(result, output)
            first, last = None, None
            with (folder / "camera" / "frames.jsonl").open() as stream:
                for line in stream:
                    timestamp = json.loads(line)["systemTime"]
                    if first is None:
                        first = timestamp
                    last = timestamp
            before = read_json(folder / info["sync_before"])
            if (first is None or max(row["t4"] for row in before["exchanges"]) > first
                    or min(row["t1"] for row in result["exchanges"]) < last):
                raise APIError(409, "录前/录后对钟未覆盖实际视频时间；原始记录已保存，请核查")
            self.change(info, sync_after=output.name, sync_summary=result["summary"], error=None)
            return self.status(session_id)

    def review(self, session_id, decision):
        if decision not in {"kept", "discarded"}:
            raise APIError(400, "请选择保存本轮或放弃本轮")
        with self.control_lock:
            self.refresh()
            info = self.record(session_id)
            if info["state"] not in {"saved", "failed"}:
                raise APIError(409, "请先结束本轮采集，再确认保存或放弃")
            if decision == "discarded" and session_id in self.calibration_processes:
                raise APIError(409, "本轮外参正在计算，请等待完成")
            if decision == "discarded":
                settings = read_json(self.root / "ego_config.json")
                folder = self.folder(session_id).resolve()
                if any((self.root / settings[key]).resolve().is_relative_to(folder)
                       for key in ("calibration", "intrinsics")):
                    raise APIError(409, "本轮标定仍在使用，不能标记作废；请先启用另一组标定")
            self.change(info, review_state=decision)
            return self.status(session_id)

    def calibrate(self, session_id):
        with self.control_lock:
            self.refresh()
            info = self.record(session_id)
            if info.get("review_state", "kept") != "kept":
                raise APIError(409, "请先确认保存本轮；待确认或已作废的轮次不能计算外参")
            if any(record["state"] in ACTIVE_STATES for record in self.records.values()):
                raise APIError(409, "请先结束所有采集，再计算外参")
            if any(other != session_id for other in self.calibration_processes):
                raise APIError(409, "已有另一轮外参计算正在运行")
            if info.get("calibration", {}).get("state") in {"running", "completed"}:
                return self.status(session_id)
            if info["state"] != "saved" or not info.get("vp_uploaded"):
                raise APIError(409, "外参计算需要相机保存完成且 VP 两份原始文件已传输")
            if not info.get("sync_before") or not info.get("sync_after"):
                raise APIError(409, "外参计算需要本轮录前和录后对钟")
            self.preview_enabled = False
            self.release_preview_processes()
            self.camera_completed(session_id)
            folder = self.folder(session_id)
            intrinsics = folder / "calibration" / "stereo_calibration.json"
            snapshot = folder / "calibration" / "ego_extrinsics.json"
            inputs = [intrinsics, snapshot, folder / info["sync_before"], folder / info["sync_after"],
                      folder / "vp_recording" / "metadata.json", folder / "vp_recording" / "tracking_events.jsonl"]
            if any(not path.is_file() for path in inputs):
                raise APIError(409, "本轮标定副本、对钟或 VP 原始文件不完整")
            board = read_json(snapshot)["board"]
            cols, rows = board["inner_corners"]
            square_mm = board["square_mm"]
            python = self.root / ".venv" / "bin" / "python"
            solver = self.root / "video_extrinsics.py"
            if not python.is_file() or not solver.is_file():
                raise APIError(409, "板子缺少外参计算环境或 video_extrinsics.py，请先完成工具包安装")
            output_dir = folder / "calibration_result"
            attempt = 2
            while output_dir.exists():
                output_dir = folder / f"calibration_result_{attempt:02d}"
                attempt += 1
            output_dir.mkdir()
            output = output_dir / "ego_extrinsics.json"
            log_path = output_dir / "calibration.log"
            calibration = {"state": "running", "session_id": session_id, "error": None,
                           "sample_count": None, "validation_count": None, "validation_translation_mm": None,
                           "validation_rotation_deg": None, "warnings": [], "active": False,
                           "output_path": str(output.relative_to(self.root)),
                           "intrinsics_path": str(intrinsics.relative_to(self.root)),
                           "log_path": str(log_path.relative_to(self.root))}
            self.change(info, calibration=calibration)
            command = [str(python), str(solver), "--session", str(folder / "camera"),
                       "--vp-recording", str(folder / "vp_recording"),
                       "--sync-before", str(folder / info["sync_before"]),
                       "--sync-after", str(folder / info["sync_after"]),
                       "--intrinsics", str(intrinsics), "--output", str(output),
                       "--cols", str(cols), "--rows", str(rows), "--square-mm", str(square_mm)]
            try:
                with log_path.open("wb") as log:
                    process = subprocess.Popen(command, cwd=self.root, stdin=subprocess.DEVNULL,
                                               stdout=log, stderr=subprocess.STDOUT)
                with self.manifest_lock:
                    self.calibration_processes[session_id] = process
            except OSError as error:
                calibration.update(state="failed", error=str(error))
                self.change(info, calibration=calibration)
                raise
            return self.status(session_id)

    def calibration_preview(self, session_id):
        self.refresh()
        calibration = self.record(session_id).get("calibration")
        if not calibration or calibration["state"] != "completed":
            raise APIError(409, "外参计算尚未成功完成，暂不能查看角点检查图")
        output = self.root / calibration["output_path"]
        preview = output.with_name(output.stem + "_角点检查.jpg")
        if not preview.is_file():
            raise APIError(404, "本轮外参结果缺少角点检查图")
        return preview

    def apply_calibration(self, session_id):
        with self.control_lock:
            self.refresh()
            info = self.record(session_id)
            if info.get("review_state", "kept") != "kept":
                raise APIError(409, "待确认或已作废的轮次不能启用标定")
            if self.calibration_processes or any(record["state"] in ACTIVE_STATES for record in self.records.values()):
                raise APIError(409, "采集或外参计算进行中，暂不能启用新外参")
            calibration = info.get("calibration")
            if not calibration or calibration["state"] != "completed":
                raise APIError(409, "只能启用已经成功计算的外参")
            output, intrinsics = self.root / calibration["output_path"], self.root / calibration["intrinsics_path"]
            if not output.is_file() or not intrinsics.is_file():
                raise APIError(409, "外参结果或配套内参副本不存在，不能启用")
            settings = read_json(self.root / "ego_config.json")
            settings.update(calibration=str(output.relative_to(self.root)), intrinsics=str(intrinsics.relative_to(self.root)))
            write_json(self.root / "ego_config.json", settings)
            for record in self.records.values():
                if record.get("calibration"):
                    active = record["session_id"] == session_id
                    self.change(record, calibration=dict(record["calibration"], active=active))
            return self.status(session_id)

    def validate_upload(self, path, name, session_id):
        if name == "metadata.json":
            metadata = read_json(path)
            if not isinstance(metadata, dict) or metadata.get("captureSessionID") != session_id:
                raise APIError(409, "metadata.captureSessionID 与指定采集轮次不一致")
            if metadata.get("captureBoardID") != self.board_id:
                raise APIError(409, "metadata.captureBoardID 与当前板子不一致")
            if metadata.get("recordingType") != "egorecord" or type(metadata.get("frameCount")) is not int or metadata["frameCount"] < 1:
                raise APIError(400, "metadata 不是包含源事件的 EgoRecord 录制")
            pose_data = metadata.get("poseData")
            if not isinstance(pose_data, dict) or not math.isfinite(float(pose_data.get("recordingStartMonotonicTimestamp", "nan"))):
                raise APIError(400, "metadata 缺少有效的 VP 录制单调时钟起点")
            return metadata
        count, first = 0, None
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                record = json.loads(line)
                count += 1
                if (not isinstance(record, dict) or record.get("sequenceNumber") != count
                        or record.get("source") not in {"head", "leftHand", "rightHand"}):
                    raise APIError(400, "tracking_events.jsonl 不是连续编号的头手源事件")
                if first is None:
                    first = record
        if first is None:
            raise APIError(400, "tracking_events.jsonl 为空")
        return count, first

    def validate_pair(self, folder, session_id, replacement=None):
        metadata_path, events_path = folder / "metadata.json", folder / "tracking_events.jsonl"
        if replacement:
            if replacement[0] == "metadata.json":
                metadata_path = replacement[1]
            else:
                events_path = replacement[1]
        if not metadata_path.exists() or not events_path.exists():
            return False
        metadata = self.validate_upload(metadata_path, "metadata.json", session_id)
        count, first = self.validate_upload(events_path, "tracking_events.jsonl", session_id)
        if count != metadata["frameCount"]:
            raise APIError(409, "VP 源事件数量与 metadata.frameCount 不一致，不能确认属于同一轮")
        start = float(metadata.get("poseData", {}).get("recordingStartMonotonicTimestamp", "nan"))
        event_start = float(first.get("receivedTimestamp", "nan")) - float(first.get("recordingTimestamp", "nan"))
        if not math.isfinite(start) or not math.isfinite(event_start) or abs(start - event_start) > .0001:
            raise APIError(409, "VP 源事件的录制起点与 metadata 不一致，不能确认属于同一轮")
        return True

    def upload(self, session_id, name, stream, length):
        info = self.record(session_id)
        if name not in UPLOAD_NAMES:
            raise APIError(404, "只接收 metadata.json 和 tracking_events.jsonl")
        if length <= 0:
            raise APIError(400, "上传文件不能为空")
        with self.manifest_lock:
            upload_lock = self.upload_locks.setdefault(session_id, threading.Lock())
        with upload_lock:
            if info.get("review_state", "kept") != "kept":
                raise APIError(409, "请先确认保存本轮；待确认或已作废的轮次不接收上传")
            if info["state"] not in {"saved", "failed"}:
                raise APIError(409, "请先停止本轮相机后再上传 VP 文件")
            folder = self.folder(session_id) / "vp_recording"
            folder.mkdir(exist_ok=True)
            final, temporary = folder / name, folder / (name + ".uploading")
            try:
                remaining = length
                with temporary.open("wb") as output:
                    while remaining:
                        chunk = stream.read(min(1024 * 1024, remaining))
                        if not chunk:
                            raise APIError(400, "上传中断，文件长度不足；可重试上传")
                        output.write(chunk)
                        remaining -= len(chunk)
                self.validate_upload(temporary, name, session_id)
                if final.exists():
                    if final.stat().st_size != length:
                        raise APIError(409, "本轮已有不同大小的同名原件，不能覆盖")
                    # 直接比较内容，既不覆盖同名原件，也不引入哈希文件。
                    with final.open("rb") as original, temporary.open("rb") as incoming:
                        while True:
                            old, new = original.read(1024 * 1024), incoming.read(1024 * 1024)
                            if old != new:
                                raise APIError(409, "本轮已有不同内容的同名原件，不能覆盖")
                            if not old:
                                break
                    uploaded = self.validate_pair(folder, session_id)
                else:
                    uploaded = self.validate_pair(folder, session_id, (name, temporary))
                    temporary.replace(final)
                self.change(info, vp_uploaded=uploaded)
            finally:
                temporary.unlink(missing_ok=True)
            return self.status(session_id)

    def shutdown(self):
        self.preview_shutdown.set()
        with self.control_lock:
            self.preview_enabled = False
            self.release_preview_processes()
            for session_id, process in list(self.calibration_processes.items()):
                self.terminate(process)
                info = self.records[session_id]
                self.change(info, calibration=dict(info["calibration"], state="failed",
                                                  error="板子服务停止，外参计算已中断；可重新计算", active=False))
                self.calibration_processes.pop(session_id, None)
            for session_id, process in list(self.processes.items()):
                try:
                    self.terminate(process)
                finally:
                    self.change(self.records[session_id], state="failed", error="板子服务停止，本轮未完成联合采集；原始文件已保留")
                    self.close_process(session_id)
            if self.preview_directory is not None:
                self.preview_directory.cleanup()
        if self.preview_watchdog is not None:
            self.preview_watchdog.join(timeout=2)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def send_json(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)
        self.close_connection = True

    def body_length(self):
        if self.headers.get("Transfer-Encoding"):
            raise APIError(400, "请使用 Content-Length 上传，不支持 Transfer-Encoding")
        try:
            length = int(self.headers["Content-Length"])
        except (TypeError, ValueError):
            raise APIError(411, "需要有效的 Content-Length")
        if length < 0:
            raise APIError(400, "Content-Length 不能为负数")
        return length

    def json_body(self):
        length = self.body_length()
        if length > 4096:
            raise APIError(413, "控制请求过大")
        data = self.rfile.read(length)
        if len(data) != length:
            raise APIError(400, "控制请求未完整接收")
        body = json.loads(data or b"{}")
        if not isinstance(body, dict):
            raise APIError(400, "请求内容必须是 JSON 对象")
        return body

    def dispatch(self):
        service = self.server.service
        parts = urlsplit(self.path)
        session_id = None
        self.connection.settimeout(60)
        try:
            board_id = self.headers.get("X-Capture-Board-ID")
            if board_id is not None and board_id != service.board_id:
                raise APIError(409, "设备编号不匹配，请重新发现并连接原来的采集板")
            if self.command == "GET" and parts.path == "/status":
                session_id = parse_qs(parts.query).get("session_id", [None])[0]
                result = service.status(session_id)
            elif self.command == "POST" and parts.path == "/sync":
                self.json_body()
                result = service.sync(self.client_address[0])
            elif self.command == "POST" and parts.path == "/sessions":
                session_id = self.json_body().get("session_id")
                result = service.prepare(session_id, self.client_address[0])
            elif parts.path == "/calibration/preview" and self.command in {"GET", "POST"}:
                if self.command == "POST":
                    self.json_body()
                result = service.live_preview("start" if self.command == "POST" else None)
            elif parts.path == "/calibration/preview/stop" and self.command == "POST":
                self.json_body()
                result = service.live_preview("stop")
            else:
                route = parts.path.strip("/").split("/")
                if len(route) < 3 or route[0] != "sessions":
                    raise APIError(404, "未知接口")
                session_id = route[1]
                service.validate_id(session_id)
                if self.command == "POST" and len(route) == 3:
                    body = self.json_body()
                    if route[2] == "start":
                        result = service.start(session_id)
                    elif route[2] == "stop":
                        result = service.stop(session_id)
                    elif route[2] == "sync-after":
                        result = service.sync_after(session_id, self.client_address[0])
                    elif route[2] == "calibrate":
                        result = service.calibrate(session_id)
                    elif route[2] == "apply-calibration":
                        result = service.apply_calibration(session_id)
                    elif route[2] == "review":
                        result = service.review(session_id, body.get("decision"))
                    else:
                        raise APIError(404, "未知接口")
                elif self.command == "GET" and len(route) == 3 and route[2] == "calibration-preview":
                    data = service.calibration_preview(session_id).read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Connection", "close")
                    self.end_headers()
                    self.wfile.write(data)
                    self.close_connection = True
                    return
                elif self.command == "PUT" and len(route) == 4 and route[2] == "files":
                    result = service.upload(session_id, route[3], self.rfile, self.body_length())
                else:
                    raise APIError(404, "未知接口")
            self.send_json(200, result)
        except (APIError, OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
            status = error.status if isinstance(error, APIError) else 400 if isinstance(error, (ValueError, KeyError, TypeError)) else 500
            try:
                if isinstance(error, OSError) and error.errno in {errno.ENOSPC, errno.EDQUOT}:
                    message = ("传输失败：采集板空间不足。请先导出并清理板端数据，再重试；VP 原件仍保留。"
                               if self.command == "PUT" else
                               "采集板空间不足，无法继续写入。请先导出并清理板端数据，再重试。")
                    self.send_json(507, {"error": message})
                    return
                known = isinstance(session_id, str) and session_id in service.records
                payload = service.status(session_id) if known else service.status()
                if isinstance(session_id, str):
                    payload["session_id"] = session_id
                payload["error"] = str(error)
                self.send_json(status, payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

    do_GET = dispatch
    do_POST = dispatch
    do_PUT = dispatch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    args = parser.parse_args()
    service = BoardService(args.root)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.service = service
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    print(f"采集服务 {service.board_name} ({service.board_id})，HTTP {args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        service.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
