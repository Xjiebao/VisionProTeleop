"""将一轮相机 / VP 采集对齐为逐帧标签，并用同一份标签渲染骨架视频。"""
from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction
import json
from pathlib import Path

import cv2
import numpy as np

import ego_extrinsics as ego
import video_extrinsics as timing


JOINT_NAMES = ["wrist"] + ["thumb" + name for name in
    ("Knuckle", "IntermediateBase", "IntermediateTip", "Tip")]
JOINT_NAMES += [finger + "Finger" + name
    for finger in ("index", "middle", "ring", "little")
    for name in ("Knuckle", "IntermediateBase", "IntermediateTip", "Tip")]
EDGES = [(a, b) for start in (1, 5, 9, 13, 17)
         for a, b in ((0, start), (start, start + 1),
                      (start + 1, start + 2), (start + 2, start + 3))]
HANDS = ("leftHand", "rightHand")
COLORS = {"leftHand": (255, 230, 0), "rightHand": (0, 175, 255)}


def finite_list(values):
    """JSON 中缺失坐标写 null，禁止输出非标准 NaN / Infinity。"""
    values = np.asarray(values, dtype=float)
    return np.where(np.isfinite(values), values, None).tolist()


def load_streams(directory):
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8-sig"))
    contract = metadata.get("poseData", {})
    if (contract.get("matrixLayout") != "column_major"
            or contract.get("translationUnit") != "meters"
            or contract.get("headPoseSource") != "queryDeviceAnchor_current_time"):
        raise ValueError("VP metadata 需要当前 EgoRecord 的列主序、米、queryDeviceAnchor_current_time 头位姿")
    records = {source: [] for source in ("head", *HANDS)}
    with (directory / "tracking_events.jsonl").open(encoding="utf-8-sig") as handle:
        for line in handle:
            record = json.loads(line)
            records[record["source"]].append(record)
    sessions = {r["trackingSessionId"] for values in records.values() for r in values}
    if len(sessions) != 1 or not next(iter(sessions)):
        raise ValueError("VP 记录需要同一个非空 trackingSessionId；不能跨世界坐标重置进行对齐")
    streams = {}
    for source, values in records.items():
        times = np.asarray([r["systemTime"] for r in values], dtype=float)
        if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
            raise ValueError(f"{source} systemTime 必须有限且严格递增")
        matrices = np.asarray([np.asarray(r["originFromAnchorTransform"], dtype=float)
                               .reshape(4, 4, order="F") for r in values]).reshape(-1, 4, 4)
        tracked = np.asarray([r["isTracked"] is True and r.get("event") != "removed"
                              for r in values], dtype=bool)
        if source == "head":
            for index in np.flatnonzero(tracked):
                ego.check_rigid(matrices[index], f"head systemTime={times[index]}")
        stream = {"times": times, "matrices": matrices, "tracked": tracked}
        if source in HANDS:
            xyz = np.full((len(values), 21, 3), np.nan)
            present = np.zeros((len(values), 21), dtype=bool)
            mask = np.zeros((len(values), 21), dtype=bool)
            for index, record in enumerate(values):
                joints = {j["name"]: j for j in (record.get("joints") or [])}
                for joint_index, name in enumerate(JOINT_NAMES):
                    joint = joints.get(name)
                    if joint is None:
                        continue
                    transform = np.asarray(joint["anchorFromJointTransform"], dtype=float).reshape(4, 4, order="F")
                    xyz[index, joint_index] = (matrices[index, :3, :3] @ transform[:3, 3]
                                               + matrices[index, :3, 3])
                    present[index, joint_index] = True
                    mask[index, joint_index] = joint["isTracked"] is True and tracked[index]
            stream.update(xyz=xyz, present=present, mask=mask)
        streams[source] = stream
    heads = records["head"]
    if len(heads) < 2:
        raise ValueError("VP 记录至少需要两条 head，才能检查 Unix 时钟连续性")
    continuity = timing.check_unix_continuity(
        [r["systemTime"] for r in heads], [r["recordingTimestamp"] for r in heads],
        "VP systemTime / recordingTimestamp")
    return streams, continuity, next(iter(sessions))


def bracket(stream, target, max_gap):
    """只用接收时刻，保留所有 ARKit 记录；禁止按 anchor timestamp 去重或外推。"""
    times = stream["times"]
    upper = int(np.searchsorted(times, target))
    if upper < len(times) and times[upper] == target:
        if not stream["tracked"][upper]:
            return None, "untracked"
        return (upper, upper, 0.0, 0.0), "tracked"
    lower = upper - 1
    if lower < 0 or upper >= len(times):
        return None, "no_data"
    gap = times[upper] - times[lower]
    if gap > max_gap:
        return None, "gap_too_large"
    if not stream["tracked"][lower] or not stream["tracked"][upper]:
        return None, "untracked"
    return (lower, upper, float((target - times[lower]) / gap), float(gap * 1000)), "tracked"


def sample_head(stream, target):
    bounds, status = bracket(stream, target, 0.15)
    if bounds is None:
        return None, status, None
    lower, upper, alpha, gap = bounds
    first, last = stream["matrices"][lower], stream["matrices"][upper]
    vector = cv2.Rodrigues(first[:3, :3].T @ last[:3, :3])[0]
    matrix = ego.rigid_matrix(
        first[:3, :3] @ cv2.Rodrigues(vector * alpha)[0],
        (1 - alpha) * first[:3, 3] + alpha * last[:3, 3])
    return matrix, status, gap


def sample_hand(stream, target):
    bounds, status = bracket(stream, target, 0.05)
    if bounds is None:
        return (np.full((21, 3), np.nan), np.zeros(21, dtype=bool),
                np.zeros(21, dtype=bool), [status] * 21, status, None)
    lower, upper, alpha, gap = bounds
    xyz = (1 - alpha) * stream["xyz"][lower] + alpha * stream["xyz"][upper]
    tracked = stream["mask"][lower] & stream["mask"][upper]
    present = stream["present"][lower] & stream["present"][upper]
    finite = np.isfinite(xyz).all(axis=1)
    valid = tracked & finite
    reasons = [None if valid[index] else
               "missing_joint" if not present[index] else
               "nonfinite_coordinates" if not finite[index] else "joint_not_tracked"
               for index in range(21)]
    return xyz, tracked, valid, reasons, status, gap


def project_hand(xyz, world_valid, world_reasons, head, head_status, transform, camera):
    camera_xyz = np.full((21, 3), np.nan)
    pixels = np.full((21, 2), np.nan)
    valid_3d = np.zeros(21, dtype=bool)
    projectable = np.zeros(21, dtype=bool)
    if head is not None:
        camera_from_world = transform @ np.linalg.inv(head)
        camera_xyz = xyz @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
        finite = np.isfinite(camera_xyz).all(axis=1)
        valid_3d = world_valid & finite
        projectable = finite & (camera_xyz[:, 2] > 0.01)
        if projectable.any():
            pixels[projectable] = cv2.projectPoints(
                camera_xyz[projectable], np.zeros(3), np.zeros(3), *camera)[0].reshape(-1, 2)
        projectable &= np.isfinite(pixels).all(axis=1)
    in_frame = (projectable & (pixels[:, 0] >= 0) & (pixels[:, 0] < 1920)
                & (pixels[:, 1] >= 0) & (pixels[:, 1] < 1200))
    valid = world_valid & projectable
    reasons = []
    for index in range(21):
        reason = world_reasons[index]
        if reason is None and head is None:
            reason = "head_" + head_status
        elif reason is None and not np.isfinite(camera_xyz[index]).all():
            reason = "nonfinite_coordinates"
        elif reason is None and camera_xyz[index, 2] <= 0.01:
            reason = "behind_or_near_camera"
        elif reason is None and not projectable[index]:
            reason = "nonfinite_projection"
        reasons.append(reason)
    return {"joints_camera_m": finite_list(camera_xyz), "pixels": finite_list(pixels),
            "valid_3d": valid_3d.tolist(), "valid": valid.tolist(),
            "in_frame": in_frame.tolist(), "invalid_reasons": reasons}


def annotate_frame(frame, vp_time, streams, cameras, transforms):
    head, status, gap = sample_head(streams["head"], vp_time)
    row = {
        "frame_index": frame["frame_index"], "pts_seconds": frame["pts_seconds"],
        "mac_system_time": frame["systemTime"], "vp_system_time": float(vp_time),
        "head": {"status": status, "valid": head is not None,
                 "invalid_reason": None if head is not None else status,
                 "bracket_gap_ms": gap,
                 "T_world_from_vp": None if head is None else finite_list(head)},
        "hands": {},
    }
    for source in HANDS:
        xyz, tracked, valid, reasons, hand_status, hand_gap = sample_hand(streams[source], vp_time)
        row["hands"][source] = {
            "status": hand_status, "bracket_gap_ms": hand_gap,
            "joints_world_m": finite_list(xyz), "tracked": tracked.tolist(),
            "world_valid": valid.tolist(), "invalid_reasons": reasons,
            "cameras": {side: project_hand(xyz, valid, reasons, head, status,
                                           transforms[side], cameras[side])
                        for side in ("left", "right")},
        }
    return row


def text(image, message, position, color=(245, 245, 245), scale=0.55):
    cv2.putText(image, message, position, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def dashed_line(image, start, end, color):
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length == 0:
        return
    for distance in np.arange(0, length, 12):
        first = tuple(np.round(start + delta * (distance / length)).astype(int))
        last = tuple(np.round(start + delta * (min(distance + 7, length) / length)).astype(int))
        cv2.line(image, first, last, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.line(image, first, last, color, 2, cv2.LINE_AA)


def render_frame(raw, row, clock):
    """只读取已导出的标签；半尺寸和标题栏偏移仅用于预览视频。"""
    panels = []
    for side, eye in zip(("left", "right"), ego.split_sbs(raw)):
        panel = np.zeros((700, 960, 3), dtype=np.uint8)
        panel[100:] = cv2.resize(eye, (960, 600), interpolation=cv2.INTER_AREA)
        text(panel, f"WEARER {side.upper()} CAMERA  |  LH cyan / RH orange", (12, 22), scale=0.62)
        offset = (clock["offset_at_reference_seconds"] + clock["drift"]
                  * (row["mac_system_time"] - clock["mac_reference_system_time"])) * 1000
        text(panel, f"Frame {row['frame_index']:04d}   Video {row['pts_seconds']:6.3f}s   VP-Mac {offset:+.2f}ms", (12, 45))
        text(panel, "Solid / filled = tracked   |   Dashed / hollow = ARKit estimate",
             (12, 91), (200, 200, 200), 0.48)
        for hand_index, source in enumerate(HANDS):
            hand = row["hands"][source]
            camera = hand["cameras"][side]
            points = np.asarray(camera["pixels"], dtype=float) * 0.5
            points[:, 1] += 100
            inside = np.asarray(camera["in_frame"], dtype=bool)
            valid = np.asarray(camera["valid"], dtype=bool)
            color = COLORS[source]
            estimate_color = tuple(int(0.5 * c + 105) for c in color)
            for first, last in EDGES:
                if inside[first] and inside[last]:
                    if valid[first] and valid[last]:
                        cv2.line(panel, tuple(np.round(points[first]).astype(int)),
                                 tuple(np.round(points[last]).astype(int)), color, 2, cv2.LINE_AA)
                    else:
                        dashed_line(panel, points[first], points[last], estimate_color)
            for index in np.flatnonzero(inside):
                position = tuple(np.round(points[index]).astype(int))
                if valid[index]:
                    cv2.circle(panel, position, 5 if index == 0 else 3, (0, 0, 0), -1, cv2.LINE_AA)
                    cv2.circle(panel, position, 4 if index == 0 else 2, color, -1, cv2.LINE_AA)
                else:
                    radius = 5 if index == 0 else 3
                    cv2.circle(panel, position, radius, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.circle(panel, position, radius, estimate_color, 1, cv2.LINE_AA)
            short = "LH" if source == "leftHand" else "RH"
            status = hand["status"] if row["head"]["valid"] else "head_" + row["head"]["status"]
            message = (f"{short}: {int((inside & valid).sum())} tracked + "
                       f"{int((inside & ~valid).sum())} est. in view" if status == "tracked"
                       else f"{short}: {status}")
            text(panel, message, (12 + hand_index * 465, 69), color, 0.46)
        panels.append(panel)
    return np.hstack(panels)


def process(recording: Path, calibration: Path, intrinsics: Path,
            sync_before: Path, sync_after: Path, output_dir: Path = None):
    import av

    recording = Path(recording).resolve()
    calibration, intrinsics = Path(calibration).resolve(), Path(intrinsics).resolve()
    sync_before, sync_after = Path(sync_before).resolve(), Path(sync_after).resolve()
    output_dir = (Path(output_dir) if output_dir is not None else recording / "result").resolve()
    if output_dir.exists() and (not output_dir.is_dir() or any(output_dir.iterdir())):
        raise FileExistsError(f"输出目录已存在且非空，禁止覆盖：{output_dir}")
    frames, session, mac_continuity = timing.load_timeline(recording / "camera")
    clock = timing.load_clock_pair(sync_before, sync_after, frames[0]["systemTime"], frames[-1]["systemTime"])
    targets = timing.mac_to_vp(clock, [frame["systemTime"] for frame in frames])
    cameras, size, _, _ = ego.load_intrinsics(intrinsics)
    calibration_data = json.loads(calibration.read_text(encoding="utf-8-sig"))
    transforms = {side: np.asarray(calibration_data[f"T_{side}_from_vp"], dtype=float)
                  for side in ("left", "right")}
    for side, transform in transforms.items():
        ego.check_rigid(transform, f"T_{side}_from_vp")
    streams, vp_continuity, tracking_session = load_streams(recording / "vp_recording")
    cv2.setNumThreads(2)
    output_dir.mkdir(parents=True, exist_ok=True)
    counts = {source: Counter() for source in ("head", *HANDS)}
    valid_world_counts = Counter()
    frame_count = 0
    print(f"开始处理 {len(frames)} 帧：{output_dir}", flush=True)
    with av.open(str(recording / "camera" / "video.mov")) as source_container, \
            av.open(str(output_dir / "overlay.mp4"), "w", options={"movflags": "+faststart"}) as destination, \
            (output_dir / "annotations.jsonl").open("x", encoding="utf-8") as annotations:
        source = source_container.streams.video[0]
        output = destination.add_stream("libx264", rate=30)
        output.width, output.height, output.pix_fmt = 1920, 700, "yuv420p"
        output.time_base = Fraction(1, 1_000_000)
        output.codec_context.time_base = Fraction(1, 1_000_000)
        output.codec_context.max_b_frames = 0
        output.codec_context.thread_count = 2
        output.options = {"crf": "20", "preset": "fast"}
        for index, video_frame in enumerate(source_container.decode(source)):
            if index >= len(frames):
                raise ValueError("视频解码帧数多于 frames.jsonl")
            expected = frames[index]["pts_seconds"]
            if video_frame.time is None or abs(float(video_frame.time) - expected) > 1e-6:
                raise ValueError(f"第 {index} 帧视频 PTS 与 frames.jsonl 不一致")
            raw = video_frame.to_ndarray(format="bgr24")
            row = annotate_frame(frames[index], targets[index], streams, cameras, transforms)
            canvas = render_frame(raw, row, clock)
            annotations.write(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
            counts["head"][row["head"]["status"]] += 1
            for hand in HANDS:
                counts[hand][row["hands"][hand]["status"]] += 1
                valid_world_counts[hand] += sum(row["hands"][hand]["world_valid"])
            encoded = av.VideoFrame.from_ndarray(canvas, format="bgr24")
            encoded.pts = int(round(expected * 1_000_000))
            encoded.time_base = Fraction(1, 1_000_000)
            for packet in output.encode(encoded):
                destination.mux(packet)
            frame_count += 1
            if frame_count % 300 == 0:
                print(f"处理进度：{frame_count}/{len(frames)} 帧", flush=True)
        if frame_count != len(frames):
            raise ValueError("视频解码帧数少于 frames.jsonl")
        for packet in output.encode():
            destination.mux(packet)
    summary = {
        "schema_version": 1, "status": "completed", "frame_count": frame_count,
        "recording_source": str(recording), "calibration_source": str(calibration),
        "intrinsics_source": str(intrinsics), "tracking_session_id": tracking_session,
        "clock_alignment": clock,
        "clock_continuity": {"mac": mac_continuity, "vp": vp_continuity},
        "frame_pts_span_seconds": frames[-1]["pts_seconds"],
        "outputs": {"annotations": "annotations.jsonl", "overlay": "overlay.mp4"},
        "overlay_size": [1920, 700], "per_eye_annotation_size": list(size),
        "joint_names": JOINT_NAMES, "bones": [list(edge) for edge in EDGES],
        "source_counts": {source: dict(count) for source, count in counts.items()},
        "valid_world_joint_counts": dict(valid_world_counts),
        "coordinates": {
            "world": "本次 VP trackingSessionId 的 ARKit 世界坐标，单位米",
            "head": "T_world_from_vp 为嵌套行数组；列向量 p_world = T_world_from_vp @ p_vp",
            "camera": "佩戴者左 / 右眼 OpenCV 相机坐标，单位米，x 向右、y 向下、z 向前",
            "pixels": "每眼原始 1920×1200 像素，含原始镜头畸变；左上角为原点，无缩放 / 裁剪偏移 / 标题栏偏移",
            "raw_crops": {"left": [2080, 0, 1920, 1200], "right": [160, 0, 1920, 1200]},
        },
        "validity": {
            "tracked": "手 anchor 与关节在插值两端均 isTracked；removed anchor 不可用",
            "world_valid": "tracked 且世界三维坐标有限；未跟踪关节可以保留估计坐标，但此值为 false",
            "valid_3d": "world_valid 且头位姿可用且相机三维坐标有限",
            "valid": "valid_3d 且相机 z > 0.01 米且二维投影有限；不要求位于画面内",
            "in_frame": "坐标可投影且在原始单眼画面范围内，独立于 tracked；不代表无遮挡",
            "missing": "缺失 / 非有限坐标为 null；invalid_reasons 按 joint_names 顺序",
        },
        "matching": "视频每帧 systemTime 经录前 / 录后对钟映射到 VP systemTime；头 SE3 插值间隔 <=150ms，手世界坐标线性插值间隔 <=50ms；禁止外推，禁止 anchor timestamp 去重",
        "timing": "标签保留原始逐帧 PTS；视频编码使用微秒时间基，不重采样到固定帧率",
        "limitations": ["对钟不能验证相机曝光延迟、VP 位姿延迟或网络不对称误差",
                        "ARKit 跟踪和关节估计并非物理真值；in_frame 不检测遮挡"],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"处理完成：{frame_count} 帧，输出 {output_dir}", flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for argument in ("recording", "calibration", "intrinsics", "sync-before", "sync-after"):
        parser.add_argument("--" + argument, type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    process(args.recording, args.calibration, args.intrinsics,
            args.sync_before, args.sync_after, args.output_dir)


if __name__ == "__main__":
    main()
