"""标定取景反馈。候选计数只用于操作提示，正式样本仍由完整视频算法选择。"""
from __future__ import annotations

import argparse
import base64
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np

import ego_extrinsics as ego


STALE_SECONDS = 2.0
MAX_FRAME_GAP = 1.5


class PoseCounter:
    def __init__(self):
        self.accepted = []
        self.reference = None
        self.started = None
        self.last_time = None

    def reset_window(self):
        self.reference = None
        self.started = None

    @staticmethod
    def close(first, second, translation_mm, rotation_deg):
        return all((error := ego.pose_error(a, b))["translation_mm"] <= translation_mm
                   and error["rotation_deg"] <= rotation_deg
                   for a, b in zip(first, second))

    def update(self, frame_time, poses, now):
        if now - frame_time > STALE_SECONDS or frame_time > now:
            self.reset_window()
            return 0., "预览帧已过期，请等待相机更新"
        if self.last_time is not None and frame_time <= self.last_time:
            self.reset_window()
            return 0., "等待新的预览帧"
        if self.last_time is not None and frame_time - self.last_time > MAX_FRAME_GAP:
            self.reset_window()
        self.last_time = frame_time
        if len(poses) != 2 or any(pose is None for pose in poses):
            self.reset_window()
            return 0., "请让左右眼都拍到完整棋盘，并保持长边近水平"
        if self.reference is None or not self.close(self.reference, poses, 3., .5):
            self.reference = [pose.copy() for pose in poses]
            self.started = frame_time
        stable = frame_time - self.started
        if stable < 1.:
            return stable, "已看见完整棋盘，请停稳至少 1 秒"
        if any(self.close(previous, poses, 10., 3.) for previous in self.accepted):
            return stable, "此姿态已计入候选，请换一个位置和朝向"
        self.accepted.append([pose.copy() for pose in poses])
        return stable, "已记录候选姿态，请换一个位置和朝向"


class Preview:
    def __init__(self, intrinsics, calibration):
        self.cameras, _, _, _ = ego.load_intrinsics(intrinsics)
        board = json.loads(Path(calibration).read_text(encoding="utf-8-sig"))["board"]
        self.pattern = tuple(board["inner_corners"])
        self.points = ego.object_points(*self.pattern, board["square_mm"])
        self.counter = PoseCounter()

    def process(self, raw, frame_time, now=None):
        previews, poses = [], []
        for side, image in zip(("left", "right"), ego.split_sbs(raw)):
            image = cv2.resize(image, (960, 600), interpolation=cv2.INTER_AREA)
            matrix, distortion = self.cameras[side]
            matrix = matrix.copy()
            matrix[:2] *= .5
            try:
                pose, _, corners = ego.board_pose(image, self.pattern, self.points, matrix, distortion)
            except ValueError:
                pose = None
            else:
                cv2.drawChessboardCorners(image, self.pattern, corners, True)
                origin = tuple(np.rint(corners[0, 0]).astype(int))
                cv2.circle(image, origin, 7, (0, 0, 255), 2)
                cv2.putText(image, "O", (origin[0] + 10, origin[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)
            color = (0, 220, 0) if pose is not None else (0, 100, 255)
            cv2.putText(image, side.upper() + (": OK" if pose is not None else ": NO BOARD"),
                        (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 1., color, 2)
            previews.append(cv2.resize(image, (480, 300), interpolation=cv2.INTER_AREA))
            poses.append(pose)
        checked_time = time.time() if now is None else now
        stable, guidance = self.counter.update(frame_time, poses, checked_time)
        fresh = 0 <= checked_time - frame_time <= STALE_SECONDS
        ok, encoded = cv2.imencode(".jpg", np.concatenate(previews, axis=1),
                                   [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            raise RuntimeError("预览 JPEG 编码失败")
        return {"frame_time": frame_time, "image_base64": base64.b64encode(encoded).decode("ascii"),
                "left_detected": poses[0] is not None and fresh,
                "right_detected": poses[1] is not None and fresh,
                "stable_seconds": stable, "pose_count": len(self.counter.accepted),
                "guidance": guidance,
                "error": None if fresh else "相机预览帧已过期"}


def write_status(source, status):
    temporary = source / "live.json.tmp"
    temporary.write_text(json.dumps(status, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    temporary.replace(source / "live.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    status = {"frame_time": 0., "image_base64": "", "left_detected": False,
              "right_detected": False, "stable_seconds": 0., "pose_count": 0,
              "guidance": "等待相机预览帧", "error": None}
    try:
        preview = Preview(args.intrinsics, args.calibration)
        started = time.monotonic()
        last_mtime = None
        while True:
            tick = time.monotonic()
            try:
                stream = (args.source / "latest.jpg").open("rb")
            except FileNotFoundError:
                if tick - started > 5:
                    raise RuntimeError("相机 5 秒内没有提供预览帧")
            else:
                with stream:
                    stat = os.fstat(stream.fileno())
                    if stat.st_mtime_ns != last_mtime:
                        raw = cv2.imdecode(np.frombuffer(stream.read(), np.uint8), cv2.IMREAD_COLOR)
                        if raw is None:
                            raise RuntimeError("相机预览 JPEG 解码失败")
                        status = preview.process(raw, stat.st_mtime)
                        write_status(args.source, status)
                        last_mtime = stat.st_mtime_ns
            if status["frame_time"] and time.time() - status["frame_time"] > STALE_SECONDS:
                preview.counter.reset_window()
                if status["error"] is None:
                    status.update(left_detected=False, right_detected=False, stable_seconds=0.,
                                  guidance="预览帧已过期，请等待相机更新", error="相机预览帧已过期")
                    write_status(args.source, status)
            time.sleep(max(0., .5 - (time.monotonic() - tick)))
    except Exception as error:
        status.update(left_detected=False, right_detected=False, stable_seconds=0.,
                      guidance="预览失败，请重新打开标定页面", error=str(error))
        write_status(args.source, status)
        raise


if __name__ == "__main__":
    main()
