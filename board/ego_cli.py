#!/usr/bin/env python3
"""Mac / Linux 与 Vision Pro 日常采集入口。文件路径相对于工具包或明确指定的采集目录。"""
import argparse
import ipaddress
import json
import os
from datetime import datetime
from pathlib import Path
import shutil
import subprocess
import sys

import sync_clocks

ROOT = Path(__file__).resolve().parent


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def config():
    return read_json(ROOT / "ego_config.json")


def current_recording():
    pointer = ROOT / ".ego_current"
    if not pointer.exists():
        return None
    path = (ROOT / pointer.read_text(encoding="utf-8").strip()).resolve()
    if not path.is_dir():
        raise ValueError(f"当前采集目录不存在：{path}；用 sync --new 开始新一轮")
    return path


def set_current(recording):
    (ROOT / ".ego_current").write_text(os.path.relpath(recording, ROOT) + "\n", encoding="utf-8")


def selected_recording(argument):
    path = Path(argument).expanduser().resolve() if argument else current_recording()
    if path is None:
        raise ValueError("没有当前采集轮；请先运行 sync --vp IP，或用 --session 指定已有目录")
    if not path.is_dir():
        raise FileNotFoundError(f"采集目录不存在：{path}")
    return path


def manifest(recording):
    path = recording / "recording.json"
    return read_json(path) if path.exists() else {}


def snapshot_calibration(recording):
    """每轮仅固定一次内外参，后续不再读取可能变化的默认配置。"""
    folder = recording / "calibration"
    intrinsics = folder / "stereo_calibration.json"
    calibration = folder / "ego_extrinsics.json"
    if folder.exists():
        if not intrinsics.is_file() or not calibration.is_file():
            raise ValueError(f"本轮内外参副本不完整：{folder}")
        return calibration, intrinsics
    settings = config()
    source_i = (ROOT / settings["intrinsics"]).resolve()
    source_e = (ROOT / settings["calibration"]).resolve()
    import ego_extrinsics as ego
    import numpy as np
    ego.load_intrinsics(source_i)
    extrinsics = read_json(source_e)
    for eye in ("left", "right"):
        ego.check_rigid(np.asarray(extrinsics[f"T_{eye}_from_vp"], float), f"T_{eye}_from_vp")
    folder.mkdir()
    shutil.copyfile(source_i, intrinsics)
    extrinsics["intrinsics_source"] = "stereo_calibration.json"
    write_json(calibration, extrinsics)
    return calibration, intrinsics


def new_recording_path(timestamp):
    stem = datetime.fromtimestamp(timestamp).strftime("%y%m%d%H%M")
    directory = ROOT / "data" / stem
    number = 2
    while directory.exists():
        directory = ROOT / "data" / f"{stem}_{number:02d}"
        number += 1
    return directory


def select_clock_pair(recording, first_time, last_time):
    """按文件实际交换时刻，选最近的有效录前和录后测量。"""
    import video_extrinsics as timing
    candidates = []
    for path in sorted(recording.glob("*.json")):
        data = read_json(path)
        if not isinstance(data, dict) or data.get("kind") != "clock_sync":
            continue
        if data.get("status") != "completed":
            raise ValueError(f"未完成的对钟文件：{path.name}")
        timing.fit_clock(data["exchanges"])
        candidates.append((path, data, min(row["t1"] for row in data["exchanges"]),
                           max(row["t4"] for row in data["exchanges"])))
    before = [item for item in candidates if item[3] <= first_time]
    after = [item for item in candidates if item[2] >= last_time]
    if not before or not after:
        raise ValueError("缺少覆盖视频的录前/录后对钟；请先完成本轮两次对钟")
    ips = {item[1].get("vp_ip") for item in before + after}
    if len(ips) != 1:
        raise ValueError("本轮对钟包含不同 VP IP，不能自动配对；请核对本轮源文件")
    first = max(before, key=lambda item: item[3])[0]
    last = min(after, key=lambda item: item[2])[0]
    timing.load_clock_pair(first, last, first_time, last_time)
    return first, last


def sync(args):
    vp_ip = str(ipaddress.IPv4Address(args.vp))
    recording = (Path(args.session).expanduser().resolve() if args.session else
                 None if args.new else current_recording())
    info = manifest(recording) if recording and recording.exists() else {}
    if recording and info.get("sync_after") and not args.session:
        recording = None
        info = {}
    camera = recording / "camera" if recording else None
    camera_meta = camera / "session.json" if camera else None
    after_capture = False
    if camera_meta and camera_meta.exists():
        status = read_json(camera_meta).get("status")
        if status != "completed":
            raise ValueError(f"本轮相机状态为 {status}；等待正常停止，失败的轮次用 sync --new 重开")
        after_capture = True
    elif camera and camera.exists() and any(camera.iterdir()):
        raise ValueError("camera 已有数据但缺少 session.json；请检查，或用 sync --new 重开")
    if info.get("vp_ip") and info["vp_ip"] != vp_ip:
        raise ValueError("本轮 VP IP 与之前不同；请使用同一台 VP，或用 sync --new 重开")
    if recording and recording.exists() and not info and any(recording.iterdir()) and not after_capture:
        raise ValueError("指定目录已有数据但没有本轮记录；请用新的目录")
    print(f"正在向 {vp_ip}:{sync_clocks.PORT} 对钟，共 {sync_clocks.ROUNDS} 轮…", flush=True)
    result = sync_clocks.measure(vp_ip)
    if recording is None:
        recording = new_recording_path(result["started_system_time"])
    recording.mkdir(parents=True, exist_ok=True)
    snapshot_calibration(recording)
    (recording / "vp_recording").mkdir(exist_ok=True)
    output = sync_clocks.timestamped_path(recording, result["started_system_time"])
    sync_clocks.save_measurement(result, output)
    info.update(schema_version=1, vp_ip=vp_ip)
    if after_capture:
        import video_extrinsics as timing
        frames, _, _ = timing.load_timeline(recording / "camera")
        before, after = select_clock_pair(recording, frames[0]["systemTime"], frames[-1]["systemTime"])
        info.update(sync_before=before.name, sync_after=after.name)
        print("录后对钟完成。将 VP 的两个文件放入 vp_recording，再运行 process。")
    else:
        info["sync_before"] = output.name
        print("录前对钟完成。先启动 VP 录制，再运行 record。")
    write_json(recording / "recording.json", info)
    set_current(recording)
    print(f"本轮目录：{recording}")


def choose_camera(devices, requested, saved):
    eligible = [item for item in devices if item["external"] and item["supports_4000x1200"]]
    if requested:
        matches = [item for item in devices if item["unique_id"] == requested]
        if not matches and requested.isdecimal() and int(requested) < len(devices):
            matches = [devices[int(requested)]]
        if not matches:
            raise ValueError("未找到相机，可尝试重启开发板")
        if matches[0] not in eligible:
            raise ValueError("指定相机不是支持 4000×1200 的外置相机")
        return matches[0]
    if saved:
        matches = [item for item in eligible if item["unique_id"] == saved]
        if not matches:
            raise ValueError("未找到相机，可尝试重启开发板")
        return matches[0]
    matches = [item for item in eligible if "DECXIN" in item["name"].upper()]
    if not matches:
        raise ValueError("未找到相机，可尝试重启开发板")
    if len(matches) != 1:
        raise ValueError(f"发现 {len(matches)} 台符合要求的 DECXIN；用 record --list 查看，再用 --camera 指定")
    return matches[0]


def record(args):
    if sys.platform == "darwin":
        capture = ROOT / "start_capture.command"
    elif sys.platform.startswith("linux"):
        capture = ROOT / "start_capture_linux.sh"
    else:
        raise ValueError("相机采集仅支持 macOS 和 Linux")
    if args.list:
        subprocess.run([str(capture), "--list"], check=True)
        return
    recording = selected_recording(args.session)
    info = manifest(recording)
    if not info.get("sync_before"):
        raise ValueError("本轮尚未完成录前对钟；请先运行 sync --vp IP")
    if info.get("sync_after"):
        raise ValueError("本轮已完成；再次运行 sync --vp IP 开始新一轮")
    camera_dir = recording / "camera"
    if camera_dir.exists() and any(camera_dir.iterdir()):
        raise FileExistsError(f"相机目录已有数据，不覆盖：{camera_dir}；用 sync --new 开始新一轮")
    settings = config()
    result = subprocess.run([str(capture), "--list-json"], check=True, text=True, stdout=subprocess.PIPE)
    device = choose_camera(json.loads(result.stdout), args.camera, settings.get("camera_unique_id"))
    settings["camera_unique_id"] = device["unique_id"]
    write_json(ROOT / "ego_config.json", settings)
    set_current(recording)
    print(f"本轮目录：{recording}\n相机：{device['name']} ({device['unique_id']})", flush=True)
    subprocess.run([str(capture), "--camera", device["unique_id"], "--out", str(camera_dir)], check=True)
    if read_json(camera_dir / "session.json").get("status") != "completed":
        raise ValueError("相机未正常完成；请检查 camera/session.json")
    print("相机录像已完成。请停止 VP 录制，然后运行相同的 sync --vp IP 命令。")


def process(args):
    recording = selected_recording(args.session)
    info = manifest(recording)
    if info.get("review_state", "kept") != "kept":
        raise ValueError("本轮尚未确认保存或已作废，不生成正式标注；原始文件仍保留")
    import video_extrinsics as timing
    frames, _, _ = timing.load_timeline(recording / "camera")
    before, after = select_clock_pair(recording, frames[0]["systemTime"], frames[-1]["systemTime"])
    for name in ("tracking_events.jsonl", "metadata.json"):
        if not (recording / "vp_recording" / name).is_file():
            raise FileNotFoundError(f"请将本轮 VP 导出文件放在：{recording / 'vp_recording' / name}")
    calibration, intrinsics = snapshot_calibration(recording)
    from process_recording import process as process_recording
    output = Path(args.output_dir).expanduser().resolve() if args.output_dir else None
    process_recording(recording, calibration, intrinsics, before, after, output_dir=output)


def main(argv=None):
    parser = argparse.ArgumentParser(description="独立对钟 → 相机采集 → 独立对钟 → 对齐标注渲染")
    commands = parser.add_subparsers(dest="command", required=True)
    sync_parser = commands.add_parser("sync", help="录前/录后使用同一条命令；自动创建本轮目录与时间文件名")
    sync_parser.add_argument("--vp", required=True, help="Vision Pro IPv4 地址")
    scope = sync_parser.add_mutually_exclusive_group()
    scope.add_argument("--session", help="指定本轮目录；可用于补测或重试")
    scope.add_argument("--new", action="store_true", help="明确开始新一轮，保留未完成的旧数据")
    sync_parser.set_defaults(run=sync)
    record_parser = commands.add_parser("record", help="自动选择相机，录制到本轮 camera 子目录")
    record_parser.add_argument("--session", help="指定已有采集目录，默认当前轮")
    record_parser.add_argument("--camera", help="首次多相机时指定编号或 uniqueID，后续记住 uniqueID")
    record_parser.add_argument("--list", action="store_true", help="只列相机，不启动录像")
    record_parser.set_defaults(run=record)
    process_parser = commands.add_parser("process", help="导出逐帧标注、骨架视频和统计")
    process_parser.add_argument("--session", help="指定采集目录，默认当前轮；旧 capture01 等目录也支持")
    process_parser.add_argument("--output-dir", help="重新处理时指定新的输出目录，默认本轮 result")
    process_parser.set_defaults(run=process)
    args = parser.parse_args(argv)
    try:
        args.run(args)
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"失败：{error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("已中断；已生成的数据保留，请检查本轮状态。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
