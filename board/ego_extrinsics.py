"""衔接 calib_kit_0911：固定棋盘格 + 同编号 VP 位姿/双目照片，求相机与 VP 的外参。

依赖：Python >= 3.10、numpy、opencv-python（原工具包的 4.11 可用）。
用法见同目录《外参使用说明.md》。所有 T_a_from_b 满足 p_a=T_a_from_b@p_b。
左右均指佩戴者左右；原始帧为 [160 像素码带][1920 右眼][1920 左眼]，不旋转。
"""
from __future__ import annotations

import argparse
import json
from itertools import combinations
from pathlib import Path
import sys

import cv2
import numpy as np


DEFAULT_INTRINSICS = Path(__file__).resolve().parent / "stereo_calibration.json"


def rigid_matrix(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation).reshape(3)
    return matrix


def check_rigid(matrix, name):
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name}: 需要有限数值的 4×4 矩阵")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-5)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-4)):
        raise ValueError(f"{name}: 矩阵不是合法刚体变换，请检查布局和方向")


def load_poses(path):
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    for key, expected in (("matrixLayout", "column_major"),
                          ("transformDirection", "device_to_arkit_world"),
                          ("translationUnit", "meters")):
        if data.get(key) != expected:
            raise ValueError(f"poses.json 的 {key} 必须是 {expected}")
    poses = {}
    for sample in data["samples"]:
        sample_id = sample["sampleId"]
        if not isinstance(sample_id, str) or not sample_id.isdecimal():
            raise ValueError("sampleId 必须是数字字符串，例如 001")
        if sample_id in poses or sample.get("isTracked") is not True:
            raise ValueError(f"{sample_id}: 编号重复或 isTracked 不为 true")
        matrix = np.asarray(sample["headMatrix"], dtype=float).reshape(4, 4, order="F")
        check_rigid(matrix, sample_id)
        poses[sample_id] = matrix
    if not poses:
        raise ValueError("poses.json 没有采样")
    return data["sessionId"], poses


def load_intrinsics(path):
    """读取 0911 完整双目结果，保留原始每眼像素坐标，平移统一为米。"""
    path = Path(path)
    cameras = {}
    if path.suffix.lower() == ".npz":
        with np.load(path, allow_pickle=False) as data:
            if "R" not in data or "T" not in data:
                raise ValueError("缺少双目 R/T；请用 calib_result.npz，不是仅内参的 intrinsics.npz")
            size = tuple(int(x) for x in data["img_size"])
            for side, suffix in (("left", "l"), ("right", "r")):
                cameras[side] = (np.array(data["m" + suffix], dtype=float),
                                 np.array(data["d" + suffix], dtype=float).reshape(-1))
            stereo = rigid_matrix(data["R"], data["T"])
        convention = "opencv_raw"
    else:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        if "stereo_extrinsics" not in data:
            raise ValueError("缺少双目 R/T；请用 stereo_calibration.json，不是仅内参的 intrinsics.json")
        size = tuple(int(x) for x in data["image_size"])
        convention = data.get("convert_meta", {}).get("convention")
        geometry = data.get("recording_geometry", {})
        if (convention != "opencv_raw" or geometry.get("sbs_order") != "imu_right_left"
                or geometry.get("left_crop") != [2080, 0, 1920, 1200]
                or geometry.get("right_crop") != [160, 0, 1920, 1200]
                or geometry.get("rotation_deg") != 0):
            raise ValueError("需要 0911 原始像素、右眼在前的完整 JSON；请勿混用旧版内参或旋转后的参数")
        for side in ("left", "right"):
            item = data[side + "_intrinsics"]
            matrix = np.array([[item["fx"], 0, item["cx"]],
                               [0, item["fy"], item["cy"]], [0, 0, 1]], dtype=float)
            cameras[side] = (matrix, np.array(item["dist"], dtype=float))
        item = data["stereo_extrinsics"]
        stereo = rigid_matrix(item["R"], np.asarray(item["T"], dtype=float) / 1000)
    if size != (1920, 1200):
        raise ValueError("0911 内参单眼尺寸必须为 1920×1200，请用本版原始分辨率标定结果")
    for side, (matrix, distortion) in cameras.items():
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                or not np.isfinite(distortion).all() or distortion.size != 5
                or min(matrix[0, 0], matrix[1, 1]) <= 0
                or not np.allclose(matrix[2], [0, 0, 1])):
            raise ValueError(f"{side}: 需要工具包的针孔 K 和五参数畸变")
    check_rigid(stereo, "T_right_from_left")
    return cameras, size, stereo, convention


def object_points(cols, rows, square_mm):
    if cols < 3 or rows < 3 or square_mm <= 0:
        raise ValueError("内角点行列至少为 3，方格边长必须为正数")
    points = np.zeros((cols * rows, 3), np.float64)
    points[:, :2] = np.mgrid[:cols, :rows].T.reshape(-1, 2) * square_mm / 1000
    return points


def board_pose(image, pattern, points, matrix, distortion, *, high_accuracy=False):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=(cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
                              | (cv2.CALIB_CB_ACCURACY if high_accuracy else 0)))
    if not found:
        raise ValueError(f"未检出完整 {pattern[0]}×{pattern[1]} 内角点")
    # 普通棋盘格没有 ID。要求整个采集期间板的同一边始终朝上，画面中不过度侧转。
    # 检测器可能整体反向编号；按首末行的平均高度统一为从上到下。
    grid = corners.reshape(pattern[1], pattern[0], 2)
    if grid[0, :, 1].mean() > grid[-1, :, 1].mean():
        corners = corners[::-1].copy()
        grid = corners.reshape(pattern[1], pattern[0], 2)
    horizontal = grid[0, -1] - grid[0, 0]
    if horizontal[0] <= 0 or abs(horizontal[1]) > horizontal[0]:
        raise ValueError("棋盘方向不符合约定：长边保持近水平，侧倾小于 45°")
    ok, rvec, tvec = cv2.solvePnP(points, corners, matrix, distortion,
                                flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError("棋盘 PnP 求解失败")
    rotation = cv2.Rodrigues(rvec)[0]
    transform = rigid_matrix(rotation, tvec)
    check_rigid(transform, "T_camera_from_board")
    if np.min((points @ rotation.T + tvec.reshape(3))[:, 2]) <= 0:
        raise ValueError("棋盘位姿落在相机后方")
    projected = cv2.projectPoints(points, rvec, tvec, matrix, distortion)[0]
    rms = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2))))
    return transform, rms, corners


def solve_hand_eye(heads, boards):
    if len(heads) < 6:
        raise ValueError("至少需要 6 组求解样本；建议 20 组求解、5 组独立验证")
    motions = [cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0].reshape(3)
               for a, b in combinations(heads, 2)]
    singular = np.linalg.svd(np.array(motions), compute_uv=False)
    if singular[0] < 1e-5 or singular[1] / singular[0] < 1e-3:
        raise ValueError("旋转退化：不能只平移或只绕同一轴旋转，请补采俯仰和侧倾")
    rotation, translation = cv2.calibrateHandEye(
        [x[:3, :3] for x in heads], [x[:3, 3] for x in heads],
        [x[:3, :3] for x in boards], [x[:3, 3] for x in boards],
        method=cv2.CALIB_HAND_EYE_PARK)
    result = rigid_matrix(rotation, translation)
    check_rigid(result, "T_vp_from_left")
    return result, {"rotation_axis_singular_values_rad": singular.tolist(),
                    "second_to_first_axis_ratio": float(singular[1] / singular[0]),
                    "max_relative_rotation_deg": float(np.degrees(np.max(np.linalg.norm(motions, axis=1))))}


def mean_transform(transforms):
    u, _, vt = np.linalg.svd(np.mean([x[:3, :3] for x in transforms], axis=0))
    rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    return rigid_matrix(rotation, np.mean([x[:3, 3] for x in transforms], axis=0))


def pose_error(reference, actual):
    angle = cv2.Rodrigues(reference[:3, :3].T @ actual[:3, :3])[0]
    return {"translation_mm": float(1000 * np.linalg.norm(reference[:3, 3] - actual[:3, 3])),
            "rotation_deg": float(np.degrees(np.linalg.norm(angle)))}


def summarize_errors(records):
    if not records:
        return None
    return {key: {"rms": float(np.sqrt(np.mean(np.square([r[key] for r in records])))),
                  "max": float(max(r[key] for r in records))}
            for key in ("translation_mm", "rotation_deg")}


def solve(args):
    session, poses = load_poses(args.poses)
    cameras, size, stereo, source_convention = load_intrinsics(args.intrinsics)
    held_out = set(args.validation_ids.split(",")) if args.validation_ids else set()
    if held_out - poses.keys():
        raise ValueError(f"验证编号不存在：{sorted(held_out - poses.keys())}")
    training_ids = [s for s in poses if s not in held_out]
    if len(training_ids) < 6:
        raise ValueError("扣除验证样本后，求解样本不足 6 组")
    points = object_points(args.cols, args.rows, args.square_mm)
    samples, thumbnails = {}, []
    for sample_id in poses:
        samples[sample_id] = {}
        for side, (matrix, distortion) in cameras.items():
            path = args.images / f"{sample_id}_{side}.png"
            if not path.is_file():
                raise ValueError(f"缺少同编号原始照片：{path}")
            image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None or (image.shape[1], image.shape[0]) != size:
                raise ValueError(f"{path.name}: 图像必须匹配内参单眼尺寸 {size}，不可裁剪/缩放/校正")
            try:
                transform, rms, corners = board_pose(image, (args.cols, args.rows), points,
                                                       matrix, distortion)
            except ValueError as error:
                raise ValueError(f"{path.name}: {error}") from error
            samples[sample_id][side] = {"transform": transform, "reprojection_rms_px": rms}
            preview = cv2.resize(image, (480, round(size[1] * 480 / size[0])))
            marked = corners * (480 / size[0])
            cv2.drawChessboardCorners(preview, (args.cols, args.rows), marked, True)
            origin = tuple(np.round(marked[0, 0]).astype(int))
            cv2.circle(preview, origin, 9, (0, 0, 255), 3)
            cv2.putText(preview, "O", origin, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(preview, f"{sample_id} {side} {rms:.3f}px", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            thumbnails.append(preview)
        print(f"{sample_id}: L={samples[sample_id]['left']['reprojection_rms_px']:.3f}px "
              f"R={samples[sample_id]['right']['reprojection_rms_px']:.3f}px")
    return solve_detected(args, session, poses, samples, thumbnails, size, stereo,
                          source_convention, held_out)


def solve_detected(args, session, poses, samples, thumbnails, size, stereo,
                   source_convention, held_out, result_updates=None):
    """复用已配对位姿和棋盘检测结果，输出双眼外参及整组留出验证。"""
    training_ids = [sample_id for sample_id in poses if sample_id not in held_out]
    left, motion = solve_hand_eye([poses[s] for s in training_ids],
                                 [samples[s]["left"]["transform"] for s in training_ids])
    right = left @ np.linalg.inv(stereo)
    reference = mean_transform([poses[s] @ left @ samples[s]["left"]["transform"]
                                for s in training_ids])
    checks = []
    for sample_id, head in poses.items():
        item = samples[sample_id]
        record = {"sample_id": sample_id, "split": "validation" if sample_id in held_out else "training"}
        for side, extrinsic in (("left", left), ("right", right)):
            record[side] = {"reprojection_rms_px": item[side]["reprojection_rms_px"],
                            "board_in_world_error": pose_error(reference, head @ extrinsic @ item[side]["transform"])}
        record["stereo_error"] = pose_error(stereo, item["right"]["transform"] @ np.linalg.inv(item["left"]["transform"]))
        checks.append(record)
    quality = {}
    for split in ("training", "validation"):
        quality[split] = {side: summarize_errors([x[side]["board_in_world_error"] for x in checks if x["split"] == split])
                          for side in ("left", "right")}
    quality["stereo_consistency"] = summarize_errors([x["stereo_error"] for x in checks])
    warnings = ["数值残差仅用于检查；本程序不自动认证真实标定精度。请检查角点总览中的 O 始终对应同一实体角点。"]
    if not held_out:
        warnings.append("没有独立验证样本；当前误差仅为参与求解样本的拟合误差。")
    if len(training_ids) < 15:
        warnings.append("求解样本少于 15 组，建议补采多方向姿态。")
    if motion["second_to_first_axis_ratio"] < 0.1 or motion["max_relative_rotation_deg"] < 15:
        warnings.append("旋转方向或幅度较弱，建议补采明显的抬头低头和侧倾。")
    result = {
        "transform_definition": "p_destination = T_destination_from_source @ p_source (homogeneous column vectors)",
        "matrix_layout": "nested_rows_4x4", "translation_unit": "meters",
        "camera_frame": "wearer left/right; raw_opencv_per_eye: x right, y down, z forward; no rotation, rectification or mirror",
        "recording_geometry": {"raw_size": [4000, 1200], "sbs_order": "imu_right_left",
                               "left_crop": [2080, 0, 1920, 1200], "right_crop": [160, 0, 1920, 1200],
                               "rotation_deg": 0},
        "vp_frame": "original ARKit DeviceAnchor frame from poses.json; no manual axis flip",
        "method": "OpenCV calibrateHandEye PARK; left solved; right derived from stereo R/T",
        "session_id": session, "intrinsics_source": str(args.intrinsics.resolve()),
        "intrinsics_source_convention": source_convention, "image_size": list(size),
        "board": {"inner_corners": [args.cols, args.rows], "square_mm": args.square_mm},
        "training_ids": training_ids, "validation_ids": [s for s in poses if s in held_out],
        "T_vp_from_left": left.tolist(), "T_left_from_vp": np.linalg.inv(left).tolist(),
        "T_vp_from_right": right.tolist(), "T_right_from_vp": np.linalg.inv(right).tolist(),
        "T_right_from_left": stereo.tolist(), "T_world_from_board_reference": reference.tolist(),
        "usage": {"T_world_from_left": "T_world_from_vp @ T_vp_from_left",
                  "point_world_to_left": "p_left = T_left_from_vp @ inverse(T_world_from_vp) @ p_world"},
        "motion": motion, "quality": quality, "warnings": warnings, "samples": checks,
    }
    if result_updates:
        result.update(result_updates)
    overview = np.vstack([np.hstack(thumbnails[i:i + 2]) for i in range(0, len(thumbnails), 2)])
    preview_path = args.output.with_name(args.output.stem + "_角点检查.jpg")
    if args.output.exists() or preview_path.exists():
        raise ValueError("结果文件已存在，请指定新的 --output，避免覆盖已有外参")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(".jpg", overview)
    if not ok:
        raise ValueError("角点总览编码失败")
    encoded.tofile(preview_path)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(quality, ensure_ascii=False, indent=2))
    for warning in warnings:
        print("注意：" + warning)
    print(f"外参：{args.output}\n角点检查：{preview_path}")
    return result


def split_sbs(frame):
    """与商家 0911 分配一致：返回佩戴者左、右眼，不旋转或缩放。"""
    if frame.shape[:2] != (1200, 4000):
        raise ValueError(f"需要 4000×1200 原始帧，实际为 {frame.shape[1]}×{frame.shape[0]}")
    return frame[:, 2080:4000], frame[:, 160:2080]


def capture(args):
    """手动保存同一帧拆分的两只眼；VP 由使用者在同一静止阶段记录。"""
    args.images.mkdir(parents=True, exist_ok=True)
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    cap = cv2.VideoCapture(args.camera, backend)
    try:
        if not cap.isOpened():
            raise ValueError(f"无法打开相机索引 {args.camera}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4000)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        number = args.start_id
        print("固定棋盘，VP 和相机停稳。同一静止阶段先在 VP 保存同编号位姿，再按空格保存照片。Q 退出。")
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise ValueError("相机读帧失败")
            left, right = split_sbs(frame)
            preview = cv2.resize(np.hstack([left, right]), (1200, 375))
            for label, x in (("WEARER LEFT", 10), ("WEARER RIGHT", 610)):
                cv2.putText(preview, label, (x, 365), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(preview, f"ID {number:03d} | KEEP STILL | SPACE save | Q quit", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.imshow("Raw SBS capture", preview)
            key = cv2.waitKey(1) & 0xff
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                paths = [args.images / f"{number:03d}_{side}.png" for side in ("left", "right")]
                if any(path.exists() for path in paths):
                    raise ValueError(f"编号 {number:03d} 已有照片，请核对 VP 编号并使用 --start-id")
                for path, eye in zip(paths, (left, right)):
                    ok, encoded = cv2.imencode(".png", eye)
                    if not ok:
                        raise ValueError(f"图片编码失败：{path}")
                    encoded.tofile(path)
                print(f"已保存 {number:03d} 左右原始照片；确认 VP 也已保存该编号，才移动到下一姿态。")
                number += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture", help="手动保存原始 SBS 双目照片")
    capture_parser.add_argument("--camera", type=int, default=0)
    capture_parser.add_argument("--images", type=Path, required=True)
    capture_parser.add_argument("--start-id", type=int, default=1)
    solve_parser = commands.add_parser("solve", help="读取现有内参、VP 位姿及同编号照片，求解固定外参")
    solve_parser.add_argument("--intrinsics", type=Path, default=DEFAULT_INTRINSICS,
                              help=f"共用双目内参，默认 {DEFAULT_INTRINSICS}")
    solve_parser.add_argument("--poses", type=Path, required=True)
    solve_parser.add_argument("--images", type=Path, required=True)
    solve_parser.add_argument("--output", type=Path, required=True)
    solve_parser.add_argument("--cols", type=int, default=11, help="横向内角点数，非方格数")
    solve_parser.add_argument("--rows", type=int, default=8, help="纵向内角点数，非方格数")
    solve_parser.add_argument("--square-mm", type=float, default=30)
    solve_parser.add_argument("--validation-ids", default="", help="不参与求解的编号，例如 021,022,023,024,025")
    args = parser.parse_args()
    try:
        if args.command == "capture":
            capture(args)
        else:
            solve(args)
    except (ValueError, KeyError, OSError, cv2.error) as error:
        parser.exit(1, f"失败：{error}\n")


if __name__ == "__main__":
    main()
