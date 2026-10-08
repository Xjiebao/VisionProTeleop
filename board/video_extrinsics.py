"""连续录制标定：对齐 Mac / VP Unix 时间，从双方停稳片段选帧并求静态外参。

用法：python video_extrinsics.py --session capture_dir --vp-recording vp_dir
      --sync-before before.json --sync-after after.json
      --output extrinsics.json [--intrinsics stereo_calibration.json]
默认读取程序目录内 stereo_calibration.json，各轮共用。
固定棋盘，VP 与相机保持刚性连接，每换姿态停稳 1–2 秒，建议采 20 组以上。
复用 VP 的 systemTime（收到位姿时的 Unix 秒）；相机和位姿的物理延迟未验证。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

import ego_extrinsics as ego


MAX_NETWORK_RTT_SECONDS = .05
MAX_CLOCK_FIT_RESIDUAL_SECONDS = .02


def read_jsonl(path):
    records = []
    with Path(path).open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            if line.strip():
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as error:
                    raise ValueError(f"{path.name}:{line_number}: 无效 JSON") from error
    if not records:
        raise ValueError(f"{path.name}: 没有记录")
    return records


def fit_clock(records):
    """Unix 四时间戳：VP = Mac + offset + drift * (Mac - reference)。"""
    if any(record.get("clock") != "unix" for record in records):
        raise ValueError("对钟记录需要 clock=unix；旧版单调时钟记录不能混用，请重新录制")
    sequences = [record.get("seq") for record in records]
    if (any(type(seq) is not int for seq in sequences) or len(set(sequences)) != len(sequences)):
        raise ValueError("对钟 seq 必须是唯一整数")
    # UDP 回复可以乱序到达；按请求序号恢复顺序，不能按 Unix 时间排序掩盖时钟回退。
    records = sorted(records, key=lambda record: record["seq"])
    values = np.asarray([[r[k] for k in ("t1", "t2", "t3", "t4")] for r in records], float)
    if len(values) < 5 or not np.isfinite(values).all():
        raise ValueError("至少需要 5 次有效四时间戳对钟")
    t1, t2, t3, t4 = values.T
    if np.any(t4 < t1) or np.any(t3 < t2) or np.any(np.diff(t1) <= 0):
        raise ValueError("对钟时间顺序异常")
    rtt = (t4 - t1) - (t3 - t2)
    if np.any(rtt < -1e-6):
        raise ValueError("对钟网络往返时间为负，时钟或文件不一致")
    rtt = np.maximum(rtt, 0)
    x = t1 + (t4 - t1) / 2
    # 先相减再求平均，避免 Unix epoch 大数相加后损失小偏移的精度。
    all_offsets = ((t2 - t1) + (t3 - t4)) / 2
    q25 = float(np.percentile(rtt, 25))
    cutoff = q25 + max(.002, 3 * float(np.median(np.abs(rtt - np.median(rtt)))))
    keep = (rtt <= cutoff) & (rtt <= MAX_NETWORK_RTT_SECONDS)
    if np.count_nonzero(keep) < 5:
        raise ValueError("剔除高延迟后，网络往返不超过 50 ms 的对钟样本不足 5 次；请重新录制")
    reference = float(x[keep][0] + np.mean(x[keep] - x[keep][0]))
    span = float(np.ptp(x[keep]))
    fit_drift = bool(span >= 30 and np.count_nonzero(keep) >= 8)
    centered = x[keep] - reference
    offsets = all_offsets[keep]
    if fit_drift:
        design = np.column_stack((np.ones(len(centered)), centered))
        offset, drift = np.linalg.lstsq(design, offsets, rcond=None)[0]
    else:
        offset, drift = float(np.mean(offsets)), 0.
    residual = offsets - (offset + drift * centered)
    if np.max(np.abs(residual)) > MAX_CLOCK_FIT_RESIDUAL_SECONDS:
        raise ValueError("对钟拟合最大残差超过 20 ms，可能发生 Unix 时钟跳变或对钟不稳定；请重新录制")
    drift_se = (float(np.sqrt(np.sum(residual ** 2) / (len(centered) - 2)
                               / np.sum(centered ** 2))) if fit_drift else None)
    if 1 + drift <= 0:
        raise ValueError("对钟拟合得到非单调时间映射")
    return {
        "clock": "unix",
        "mapping": "vp_system_time = mac_system_time + offset_at_reference_seconds + drift * (mac_system_time - mac_reference_system_time)",
        "mac_reference_system_time": reference, "offset_at_reference_seconds": float(offset),
        "drift": float(drift), "drift_ppm": float(drift * 1e6),
        "drift_fitted": fit_drift, "drift_standard_error_ppm": None if drift_se is None else drift_se * 1e6,
        "drift_policy": "fit only with >=8 retained exchanges spanning >=30 seconds; otherwise offset only",
        "valid_mac_system_time": [float(x[keep].min()), float(x[keep].max())],
        "exchanges_total": len(records), "exchanges_retained": int(keep.sum()),
        "acceptance_limits": {"max_network_rtt_seconds": MAX_NETWORK_RTT_SECONDS,
                              "max_clock_fit_residual_seconds": MAX_CLOCK_FIT_RESIDUAL_SECONDS},
        "rejected_high_rtt_sequences": [r.get("seq", i) for i, r in enumerate(records) if not keep[i]],
        "network_rtt_ms": {"min": float(rtt.min() * 1000), "median": float(np.median(rtt) * 1000),
                           "p90": float(np.percentile(rtt, 90) * 1000), "max": float(rtt.max() * 1000),
                           "retained_cutoff": min(cutoff, MAX_NETWORK_RTT_SECONDS) * 1000},
        "fit_residual_ms": {"rms": float(np.sqrt(np.mean(residual ** 2)) * 1000),
                            "max_abs": float(np.max(np.abs(residual)) * 1000)},
        "limitation": "RTT and fit residual limits check clock exchanges only and do not prove exposure timing accuracy; network asymmetry and camera/pose physical latency are unverified",
    }


def mac_to_vp(clock, times):
    times = np.asarray(times, float)
    return times + (clock["offset_at_reference_seconds"]
                    + clock["drift"] * (times - clock["mac_reference_system_time"]))


def load_clock_pair(before_path, after_path, first_frame_time, last_frame_time):
    """独立验收前后两次测量，再在两个参考时刻之间线性插值钟差。"""
    bursts = []
    for path in (before_path, after_path):
        record = json.loads(path.read_text(encoding="utf-8-sig"))
        if (record.get("schema_version") != 1 or record.get("kind") != "clock_sync"
                or record.get("clock") != "unix" or record.get("status") != "completed"):
            raise ValueError(f"{path.name}: 需要已完成的独立 Unix 对钟文件（schema_version=1, kind=clock_sync）")
        if not isinstance(record.get("vp_ip"), str) or not record["vp_ip"]:
            raise ValueError(f"{path.name}: 对钟文件缺少 vp_ip")
        exchanges = record["exchanges"]
        model = fit_clock(exchanges)
        first_request = min(row["t1"] for row in exchanges)
        last_reply = max(row["t4"] for row in exchanges)
        bounds = np.asarray([record["started_system_time"], record["finished_system_time"]], float)
        if not np.isfinite(bounds).all() or not bounds[0] <= first_request <= last_reply <= bounds[1]:
            raise ValueError(f"{path.name}: 独立对钟文件的起止时间与交换记录不一致")
        bursts.append({"file": str(path.resolve()), "vp_ip": record["vp_ip"],
                       "first_request_system_time": first_request, "last_reply_system_time": last_reply,
                       **model})
    before, after = bursts
    if before["vp_ip"] != after["vp_ip"]:
        raise ValueError("前后对钟文件的 vp_ip 不同；必须使用同一台 VP")
    if before["last_reply_system_time"] > after["first_request_system_time"]:
        raise ValueError("前后对钟顺序错误或测量时段重叠")
    if (before["last_reply_system_time"] > first_frame_time
            or after["first_request_system_time"] < last_frame_time):
        raise ValueError("前后对钟覆盖不足：前次必须在首帧之前完成，后次必须在末帧之后开始；禁止外推")
    reference = before["mac_reference_system_time"]
    end = after["mac_reference_system_time"]
    if end <= reference:
        raise ValueError("前后对钟参考时刻必须严格递增")
    drift = (after["offset_at_reference_seconds"] - before["offset_at_reference_seconds"]) / (end - reference)
    if 1 + drift <= 0:
        raise ValueError("前后对钟得到非单调时间映射")
    return {
        "clock": "unix", "mode": "independent_before_after",
        "mapping": "vp_system_time = mac_system_time + before_offset + drift * (mac_system_time - before_reference_system_time)",
        "mac_reference_system_time": reference,
        "offset_at_reference_seconds": before["offset_at_reference_seconds"],
        "drift": float(drift), "drift_ppm": float(drift * 1e6),
        "valid_mac_system_time": [reference, end], "before": before, "after": after,
        "limitation": "Two clock measurements estimate linear offset drift only; physical latency and network asymmetry remain unverified",
    }


def check_unix_continuity(system_times, monotonic_times, label):
    """允许 Unix 对单调时间有线性漂移；拒绝偏离平滑趋势超过 20 ms。"""
    values = np.asarray([system_times, monotonic_times], float)
    if (values.shape[1] < 2 or not np.isfinite(values).all()
            or np.any(np.diff(values, axis=1) <= 0)):
        raise ValueError(f"{label} 时间需要至少两条有限且严格递增的记录")
    elapsed = values[1] - values[1, 0]
    delta = (values[0] - values[0, 0]) - elapsed
    centered = elapsed - np.mean(elapsed)
    design = np.column_stack((np.ones(len(centered)), centered))
    offset, drift = np.linalg.lstsq(design, delta, rcond=None)[0]
    residual = delta - (offset + drift * centered)
    maximum = float(np.max(np.abs(residual)))
    if maximum > MAX_CLOCK_FIT_RESIDUAL_SECONDS:
        raise ValueError(f"{label} Unix 时间连续性残差超过 20 ms，可能发生中途时钟跳变；请重新录制")
    return {"drift_ppm": float(drift * 1e6), "max_abs_residual_ms": maximum * 1000,
            "max_allowed_residual_ms": MAX_CLOCK_FIT_RESIDUAL_SECONDS * 1000}


def load_heads(directory, max_gap):
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8-sig"))
    contract = metadata.get("poseData", {})
    if (contract.get("matrixLayout") != "column_major" or contract.get("translationUnit") != "meters"
            or contract.get("headPoseSource") != "queryDeviceAnchor_current_time"):
        raise ValueError("VP metadata 需要当前 EgoRecord 的列主序、米、queryDeviceAnchor_current_time 头位姿")
    heads = [record for record in read_jsonl(directory / "tracking_events.jsonl") if record.get("source") == "head"]
    if not heads:
        raise ValueError("VP 记录没有 source=head")
    sessions = {head["trackingSessionId"] for head in heads}
    if len(sessions) != 1 or not next(iter(sessions)):
        raise ValueError("VP 头位姿混有多个 trackingSessionId；世界坐标可能重置，请重新采集")
    runs, current, gaps = [], [], []
    previous_time = None
    for head in heads:
        time = head.get("systemTime")
        if not isinstance(time, (int, float)) or not np.isfinite(time):
            raise ValueError("head 缺少有限数值 systemTime（Unix 秒），不能用 queryTimestamp 或相对时间代替")
        if previous_time is not None and time <= previous_time:
            raise ValueError("head systemTime 必须严格递增；Unix 时间回退或重复不能用于配帧")
        if previous_time is not None and time - previous_time > max_gap:
            if current:
                runs.append(current)
                current = []
            gaps.append({"vp_start_system_time": previous_time, "vp_end_system_time": time, "reason": "head_timestamp_gap"})
        if head.get("isTracked") is not True:
            if current:
                runs.append(current)
                current = []
            if gaps and gaps[-1]["reason"] == "head_not_tracked" and gaps[-1]["vp_end_system_time"] == previous_time:
                gaps[-1]["vp_end_system_time"] = time
            else:
                gaps.append({"vp_start_system_time": time, "vp_end_system_time": time, "reason": "head_not_tracked"})
        else:
            matrix = np.asarray(head["originFromAnchorTransform"], float).reshape(4, 4, order="F")
            ego.check_rigid(matrix, f"head systemTime={time}")
            current.append((time, matrix, head))
        previous_time = time
    if any(not isinstance(head.get("recordingTimestamp"), (int, float)) for head in heads):
        raise ValueError("VP head 缺少 recordingTimestamp，无法检查录制中途 Unix 时钟跳变")
    continuity = check_unix_continuity([head["systemTime"] for head in heads],
                                       [head["recordingTimestamp"] for head in heads],
                                       "VP systemTime / recordingTimestamp")
    if current:
        runs.append(current)
    if not runs:
        raise ValueError("VP 没有可用的 tracked 头位姿")
    return next(iter(sessions)), runs, gaps, continuity


def stable_segments(runs, min_duration, position_mm, rotation_deg):
    """逐段检查所有位姿之间的最大变化；不跨无跟踪记录或时间断档。"""
    stable, rejected = [], []
    for run in runs:
        start = 0
        for index in range(1, len(run) + 1):
            fits = False
            if index < len(run):
                previous = np.asarray([item[1] for item in run[start:index]])
                candidate = run[index][1]
                translations = np.linalg.norm(previous[:, :3, 3] - candidate[:3, 3], axis=1) * 1000
                traces = np.einsum("nij,ij->n", previous[:, :3, :3], candidate[:3, :3])
                angles = np.degrees(np.arccos(np.clip((traces - 1) / 2, -1, 1)))
                fits = translations.max() <= position_mm and angles.max() <= rotation_deg
            if fits:
                continue
            block = run[start:index]
            if block[-1][0] - block[0][0] >= min_duration - 1e-6:
                stable.append(block)
            else:
                info = {"vp_start_system_time": block[0][0], "vp_end_system_time": block[-1][0],
                        "reason": "head_stable_duration_too_short_or_moving"}
                if rejected and rejected[-1]["vp_end_system_time"] == run[max(0, start - 1)][0]:
                    rejected[-1]["vp_end_system_time"] = info["vp_end_system_time"]
                else:
                    rejected.append(info)
            start = index
    return stable, rejected


def interpolate_head(segment, time):
    times = np.asarray([item[0] for item in segment])
    if time < times[0] or time > times[-1]:
        raise ValueError("目标时间超出同一个连续 tracked 静止段；禁止跨断档插值")
    upper = int(np.searchsorted(times, time))
    if upper < len(times) and times[upper] == time:
        return segment[upper][1].copy(), {"before_system_time": float(time), "after_system_time": float(time),
                                         "nearest_system_time_delta_ms": 0.}
    lower = upper - 1
    ratio = (time - times[lower]) / (times[upper] - times[lower])
    first, second = segment[lower][1], segment[upper][1]
    vector = cv2.Rodrigues(first[:3, :3].T @ second[:3, :3])[0]
    rotation = first[:3, :3] @ cv2.Rodrigues(vector * ratio)[0]
    position = first[:3, 3] * (1 - ratio) + second[:3, 3] * ratio
    return ego.rigid_matrix(rotation, position), {
        "before_system_time": float(times[lower]), "after_system_time": float(times[upper]),
        "nearest_system_time_delta_ms": float(min(time - times[lower], times[upper] - time) * 1000),
    }


def load_timeline(directory):
    session = json.loads((directory / "session.json").read_text(encoding="utf-8-sig"))
    if session.get("status") != "completed":
        raise ValueError("Mac session.json 的 status 必须是 completed；录制未成功完成")
    if (session.get("schema_version") != 2 or session.get("alignment_clock") != "unix"
            or session.get("frame_unix_field") != "systemTime"):
        raise ValueError("需要 schema_version=2、alignment_clock=unix、frame_unix_field=systemTime；旧版采集请重新录制")
    frames = read_jsonl(directory / "frames.jsonl")
    for index, frame in enumerate(frames):
        if frame["frame_index"] != index:
            raise ValueError("frames.jsonl 的 frame_index 必须从 0 连续递增，对应实际写入视频的帧")
        if "systemTime" not in frame:
            raise ValueError("视频帧缺少 systemTime（Unix 秒）；不能回退到 PTS 或 host 时钟，请重新录制")
    values = np.asarray([[frame[key] for key in ("pts_seconds", "capture_host_seconds", "systemTime", "received_host_seconds")]
                         for frame in frames], float)
    if not np.isfinite(values).all() or np.any(np.diff(values[:, :3], axis=0) <= 0):
        raise ValueError("视频 PTS / capture_host_seconds / systemTime 必须有限且严格递增")
    if abs(values[0, 0]) > 1e-6:
        raise ValueError("视频 PTS 必须从 0 开始")
    continuity = check_unix_continuity(values[:, 2], values[:, 1], "Mac systemTime / capture_host_seconds")
    return frames, session, continuity


def solve(args):
    if min(args.still_seconds, args.position_mm, args.rotation_deg, args.max_head_gap, args.margin_seconds) <= 0:
        raise ValueError("静止窗口、变化门槛、跟踪断档及边距参数必须为正数")
    if args.still_seconds < 4 * args.margin_seconds:
        raise ValueError("静止窗口必须至少为安全边距的 4 倍，容纳前中后检查")
    frames, session_metadata, mac_continuity = load_timeline(args.session)
    clock = load_clock_pair(args.sync_before, args.sync_after, frames[0]["systemTime"], frames[-1]["systemTime"])
    tracking_session, runs, gaps, vp_continuity = load_heads(args.vp_recording, args.max_head_gap)
    segments, skipped = stable_segments(runs, args.still_seconds, args.position_mm, args.rotation_deg)
    skipped = gaps + skipped
    cameras, size, stereo, convention = ego.load_intrinsics(args.intrinsics)
    points = ego.object_points(args.cols, args.rows, args.square_mm)
    frame_times = mac_to_vp(clock, [frame["systemTime"] for frame in frames])
    valid_clock = mac_to_vp(clock, clock["valid_mac_system_time"])
    earliest, latest = max(frame_times[0], valid_clock[0]), min(frame_times[-1], valid_clock[1])
    if earliest >= latest or latest < runs[0][0][0] or earliest > runs[-1][-1][0]:
        raise ValueError("相机、VP 记录和对钟时间没有重叠；请确认来自同一次采集")
    candidates, needed = [], {}
    for segment_number, segment in enumerate(segments, 1):
        start, end = max(segment[0][0], earliest), min(segment[-1][0], latest)
        info = {"segment_id": f"{segment_number:03d}", "vp_start_system_time": segment[0][0], "vp_end_system_time": segment[-1][0]}
        if end - start < args.still_seconds - 1e-6:
            skipped.append({**info, "reason": "insufficient_overlap_with_video_and_clock_coverage"})
            continue
        target_times = [start + args.margin_seconds, start + (end - start) / 2, end - args.margin_seconds]
        indices = []
        for time in target_times:
            insertion = int(np.searchsorted(frame_times, time))
            options = [index for index in (insertion - 1, insertion) if 0 <= index < len(frames)]
            indices.append(min(options, key=lambda index: abs(frame_times[index] - time)))
        actual = frame_times[indices]
        if (len(set(indices)) != 3 or actual[0] < start + args.margin_seconds - 1e-6
                or actual[-1] > end - args.margin_seconds + 1e-6
                or min(actual[1] - actual[0], actual[2] - actual[1]) < args.margin_seconds - 1e-6):
            # 端点使用向内取帧，避免最近邻刚好落在边距之外。
            indices[0] = int(np.searchsorted(frame_times, target_times[0], side="left"))
            indices[2] = int(np.searchsorted(frame_times, target_times[2], side="right")) - 1
            actual = frame_times[indices]
        if (len(set(indices)) != 3 or min(actual[1] - actual[0], actual[2] - actual[1]) < args.margin_seconds - 1e-6):
            skipped.append({**info, "reason": "insufficient_video_frames_for_margin_and_before_center_after_checks"})
            continue
        candidate = {"info": info, "segment": segment, "indices": indices, "detections": {}, "errors": []}
        candidates.append(candidate)
        for index in indices:
            needed.setdefault(index, []).append(candidate)
    # 顺序解码完整视频；帧索引来自实际写入记录，绝不以平均 FPS 推算时间。
    capture = cv2.VideoCapture(str(args.session / "video.mov"))
    try:
        if not capture.isOpened():
            raise ValueError("无法打开 session/video.mov")
        for index in range(len(frames)):
            ok, raw = capture.read()
            if not ok or raw is None:
                raise ValueError(f"视频第 {index} 帧读取失败，frames.jsonl 记录有 {len(frames)} 帧")
            if raw.shape[:2] != (1200, 4000):
                raise ValueError(f"视频第 {index} 帧不是 4000×1200；不能缩放或旋转后配原始内参")
            if index not in needed:
                continue
            detections, thumbnails = {}, []
            try:
                for side, image in zip(("left", "right"), ego.split_sbs(raw)):
                    matrix, distortion = cameras[side]
                    transform, rms, corners = ego.board_pose(image, (args.cols, args.rows), points, matrix, distortion)
                    detections[side] = {"transform": transform, "reprojection_rms_px": rms}
                    preview = cv2.resize(image, (480, 300))
                    marked = corners * .25
                    cv2.drawChessboardCorners(preview, (args.cols, args.rows), marked, True)
                    origin = tuple(np.round(marked[0, 0]).astype(int))
                    cv2.circle(preview, origin, 9, (0, 0, 255), 3)
                    cv2.putText(preview, "O", origin, cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)
                    cv2.putText(preview, f"frame {index} {side} {rms:.3f}px", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                    thumbnails.append(preview)
            except ValueError as error:
                for candidate in needed[index]:
                    candidate["errors"].append(f"frame {index}: {error}")
            else:
                for candidate in needed[index]:
                    candidate["detections"][index] = (detections, thumbnails if index == candidate["indices"][1] else [])
        if capture.read()[0]:
            raise ValueError("视频实际帧数超过 frames.jsonl；记录和视频不一致")
    finally:
        capture.release()
    poses, samples, thumbnails, selected = {}, {}, [], []
    for candidate in candidates:
        info, indices = candidate["info"], candidate["indices"]
        if candidate["errors"]:
            skipped.append({**info, "reason": "chessboard_detection_failed", "details": candidate["errors"]})
            continue
        middle = indices[1]
        detected, previews = candidate["detections"][middle]
        stability = {}
        for side in ("left", "right"):
            transforms = [candidate["detections"][index][0][side]["transform"] for index in indices]
            errors = [ego.pose_error(transforms[i], transforms[j]) for i in range(3) for j in range(i + 1, 3)]
            stability[side] = {key: max(error[key] for error in errors) for key in ("translation_mm", "rotation_deg")}
        if any(error["translation_mm"] > args.position_mm or error["rotation_deg"] > args.rotation_deg for error in stability.values()):
            skipped.append({**info, "reason": "camera_board_pose_changed_in_before_center_after_checks", "camera_stability": stability})
            continue
        head, interpolation = interpolate_head(candidate["segment"], float(frame_times[middle]))
        if any(ego.pose_error(other, head)["translation_mm"] < 10 and ego.pose_error(other, head)["rotation_deg"] < 3
               for other in poses.values()):
            skipped.append({**info, "reason": "duplicate_pose_within_10mm_and_3deg"})
            continue
        sample_id = info["segment_id"]
        poses[sample_id], samples[sample_id] = head, detected
        thumbnails.extend(previews)
        selected.append({**info, "sample_id": sample_id, "frame_index": middle,
                         "video_pts_seconds": frames[middle]["pts_seconds"],
                         "capture_host_seconds": frames[middle]["capture_host_seconds"],
                         "received_host_seconds": frames[middle]["received_host_seconds"],
                         "mac_system_time": frames[middle]["systemTime"],
                         "mapped_vp_system_time": float(frame_times[middle]), "head_interpolation": interpolation,
                         "camera_check_frame_indices": indices,
                         "camera_check_vp_system_times": frame_times[indices].tolist(), "camera_stability": stability,
                         "sampled_stable_intersection_vp_system_times": [float(frame_times[indices[0]]), float(frame_times[indices[2]])]})
        print(f"静止段 {sample_id}: 视频帧 {middle}，双眼棋盘静止检查通过", flush=True)
    if len(poses) < 8:
        reasons = {}
        for item in skipped:
            reasons[item["reason"]] = reasons.get(item["reason"], 0) + 1
        raise ValueError(f"仅选出 {len(poses)} 个不同静止姿态，至少需要 8 个（6 求解 + 2 独立验证）；跳过原因：{json.dumps(reasons, ensure_ascii=False)}")
    ids = list(poses)
    validation_count = max(2, len(ids) // 5)
    held_out = {ids[index] for index in np.linspace(0, len(ids) - 1, validation_count, dtype=int)}
    for item in selected:
        item["split"] = "validation" if item["sample_id"] in held_out else "training"
    continuous = {
        "mode": "static_extrinsics_from_continuous_recordings", "mac_session": str(args.session.resolve()),
        "vp_recording": str(args.vp_recording.resolve()), "mac_session_metadata": session_metadata,
        "clock_alignment": clock, "mac_matching_timestamp_field": "systemTime", "vp_matching_timestamp_field": "systemTime",
        "unix_clock_continuity": {"mac": mac_continuity, "vp": vp_continuity},
        "vp_original_timestamp_fields_preserved": ["systemTime", "recordingTimestamp", "receivedTimestamp", "timestamp", "queryTimestamp"],
        "parameters": {"still_seconds": args.still_seconds, "position_mm": args.position_mm,
                       "rotation_deg": args.rotation_deg, "max_head_gap_seconds": args.max_head_gap,
                       "boundary_margin_seconds": args.margin_seconds, "duplicate_pose_translation_mm": 10, "duplicate_pose_rotation_deg": 3},
        "decoded_video_frames": len(frames), "stable_head_segments": len(segments),
        "selected_segments": selected, "skipped_segments": skipped,
        "limitations": [
            "VP systemTime 是程序收到位姿时的 Unix 秒；原始头部、手部轨迹和全部时间字段均不重写。",
            "Mac systemTime 来自每帧 PTS 转换到 Mac host 时钟，再通过回调时的 host / Unix 同时读数换算；不是已验证的曝光时间。",
            "四时间戳拟合的残差不包括网络不对称、相机驱动延迟及 ARKit 实际物理延迟。",
            "前后两次独立对钟只线性插值钟差；录制内 Unix / 单调时间连续性检查不证明曝光同步。",
            "0.2 秒默认边距是选帧策略，不是已验证的物理延迟上界；双眼棋盘只核查前中后三帧。",
            "仅求刚性安装的静态外参；不进行运动片段联合时差优化或宣称精确曝光同步。",
            "对钟协议未携带 VP 设备身份；必须人工确保两份记录来自同一次、同一台 VP。",
        ],
    }
    return ego.solve_detected(args, tracking_session, poses, samples, thumbnails, size, stereo, convention, held_out,
                              {"vp_frame": "original ARKit DeviceAnchor frame from tracking_events.jsonl; no manual axis flip",
                               "continuous_capture": continuous})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--vp-recording", type=Path, required=True)
    parser.add_argument("--sync-before", type=Path, required=True, help="录制前完成的独立对钟 JSON")
    parser.add_argument("--sync-after", type=Path, required=True, help="录制后开始的独立对钟 JSON")
    parser.add_argument("--intrinsics", type=Path, default=ego.DEFAULT_INTRINSICS,
                        help=f"共用双目内参，默认 {ego.DEFAULT_INTRINSICS}")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cols", type=int, default=11, help="横向内角点数，非方格数")
    parser.add_argument("--rows", type=int, default=8, help="纵向内角点数，非方格数")
    parser.add_argument("--square-mm", type=float, default=30)
    parser.add_argument("--still-seconds", type=float, default=.8)
    parser.add_argument("--position-mm", type=float, default=3)
    parser.add_argument("--rotation-deg", type=float, default=.5)
    parser.add_argument("--max-head-gap", type=float, default=.15)
    parser.add_argument("--margin-seconds", type=float, default=.2)
    args = parser.parse_args()
    try:
        solve(args)
    except (ValueError, KeyError, OSError, cv2.error) as error:
        parser.exit(1, f"失败：{error}\n")


if __name__ == "__main__":
    main()
