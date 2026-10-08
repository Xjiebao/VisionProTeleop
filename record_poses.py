"""Record Tracking Streamer pose messages locally; no video or cloud connection."""

import argparse
from datetime import datetime, timezone
import ipaddress
import json
import math
from pathlib import Path
import socket
import sys
import time

import grpc

# Import only the upstream message schema, avoiding avp_stream's video imports.
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "avp_stream" / "grpc_msg"))
import handtracking_pb2 as pb

JOINT_NAMES = [
    "wrist", "thumbKnuckle", "thumbIntermediateBase", "thumbIntermediateTip", "thumbTip",
    "indexFingerMetacarpal", "indexFingerKnuckle", "indexFingerIntermediateBase", "indexFingerIntermediateTip", "indexFingerTip",
    "middleFingerMetacarpal", "middleFingerKnuckle", "middleFingerIntermediateBase", "middleFingerIntermediateTip", "middleFingerTip",
    "ringFingerMetacarpal", "ringFingerKnuckle", "ringFingerIntermediateBase", "ringFingerIntermediateTip", "ringFingerTip",
    "littleFingerMetacarpal", "littleFingerKnuckle", "littleFingerIntermediateBase", "littleFingerIntermediateTip", "littleFingerTip",
    "forearmWrist", "forearmArm",
]


def matrix(message):
    return [[getattr(message, f"m{row}{col}") for col in range(4)] for row in range(4)]


def hand_data(hand):
    joints = hand.skeleton.jointMatrices
    if not hand.HasField("wristMatrix") or not (len(joints) == 25 or len(joints) >= 27):
        raise ValueError("收到缺失或不支持的手部数据，请检查头显 App 版本及跟踪是否启动。")
    return {
        "world_from_hand": matrix(hand.wristMatrix),
        "hand_from_joints": {name: matrix(joint) for name, joint in zip(JOINT_NAMES, joints)},
        "is_tracked": None,
        "joint_is_tracked": None,
    }


def main():
    parser = argparse.ArgumentParser(description="从 Vision Pro 接收头部和双手位姿，保存为 JSONL。")
    parser.add_argument("--ip", type=ipaddress.IPv4Address, required=True, help="Tracking Streamer 显示的局域网 IPv4 地址")
    parser.add_argument("--seconds", type=float, default=30, help="连接成功后的接收时长，默认 30 秒")
    parser.add_argument("--output", type=Path, help="保存路径；默认 recordings/时间.jsonl，不覆盖已有文件")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds 必须是大于零的有限数值")

    ip = str(args.ip)
    output = args.output or ROOT / "recordings" / f"{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl"
    output = output.resolve()
    print(f"连接 {ip}:12345（最长等待 5 秒）…", flush=True)
    with grpc.insecure_channel(f"{ip}:12345") as channel:
        grpc.channel_ready_future(channel).result(timeout=5)
        # Same discovery/version handshake as upstream VisionProStreamer.stream().
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
            route.connect((ip, 12345))
            parts = [int(part) for part in route.getsockname()[0].split(".")]
        request = pb.HandUpdate()
        request.Head.m00 = 888.0
        request.Head.m01, request.Head.m02, request.Head.m03, request.Head.m10 = parts
        request.Head.m30 = 25000  # Upstream protocol/library version 2.50.0.
        stream = channel.unary_stream(
            "/handtracking.HandTrackingService/StreamHandUpdates",
            request_serializer=pb.HandUpdate.SerializeToString,
            response_deserializer=pb.HandUpdate.FromString,
        )

        output.parent.mkdir(parents=True, exist_ok=True)
        count = 0
        status = "error"
        started = time.monotonic_ns()
        last_progress = started
        with output.open("x", encoding="utf-8", buffering=1) as file:
            metadata = {
                "type": "session", "schema_version": 1,
                "started_utc": datetime.now(timezone.utc).isoformat(), "ip": ip,
                "requested_seconds": args.seconds,
                "source": "VisionProTeleop Tracking Streamer gRPC",
                "length_unit": "meter", "matrix_layout": "row-major, column-vector multiplication",
                "world_frame": "ARKit origin, Y-up; no upstream Python YUP2ZUP or head rotation applied",
                "source_timestamp_available": False, "tracking_flags_available": False,
                "prediction_offset_seconds": None,
                "note": "记录接收消息而非独立传感器帧；接收时间不是采样时间；跟踪有效性和预测偏移未知。",
            }
            file.write(json.dumps(metadata, ensure_ascii=False) + "\n")
            responses = stream(request, timeout=args.seconds)
            print(f"开始接收，保存到 {output}；Ctrl+C 可提前结束。", flush=True)
            try:
                for update in responses:
                    received = time.monotonic_ns()
                    received_unix = time.time_ns()
                    if update.Head.m00 == 777.0:  # Upstream benchmark reply, not a pose.
                        continue
                    if not update.HasField("Head"):
                        raise ValueError("收到空消息，可能是 App 拒绝了协议版本或尚未启动跟踪。")
                    sample = {
                        "type": "pose", "sequence": count,
                        "received_monotonic_ns": received, "received_unix_ns": received_unix,
                        "source_timestamp": None,
                        "world_from_device": matrix(update.Head),
                        "device_is_tracked": None,
                        "left": hand_data(update.left_hand), "right": hand_data(update.right_hand),
                    }
                    file.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + "\n")
                    count += 1
                    if count == 1 or received - last_progress >= 1_000_000_000:
                        head = [round(row[3], 3) for row in sample["world_from_device"][:3]]
                        print(f"已收到 {count} 条；头部位置（米）={head}", flush=True)
                        last_progress = received
                raise RuntimeError("头显提前结束了数据流；已收到的数据保留，但本次采集不完整。")
            except grpc.RpcError as error:
                if error.code() != grpc.StatusCode.DEADLINE_EXCEEDED:
                    raise
                if not count:
                    raise RuntimeError("在接收时段内未收到任何位姿。") from error
                status = "completed"
            except KeyboardInterrupt:
                status = "interrupted"
                if not count:
                    raise RuntimeError("已停止，但尚未收到位姿。")
            finally:
                responses.cancel()
                elapsed = (time.monotonic_ns() - started) / 1e9
                file.write(json.dumps({"type": "end", "status": status, "messages": count, "elapsed_seconds": elapsed}) + "\n")
        print(f"结束：{count} 条消息，状态 {status}，文件 {output}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except grpc.FutureTimeoutError:
        print("连接超时：确认 IP、同一局域网，以及头显 Tracking Streamer 已点击 Start。", file=sys.stderr)
        sys.exit(1)
    except (grpc.RpcError, RuntimeError, ValueError, OSError) as error:
        print(f"采集失败：{error}", file=sys.stderr)
        sys.exit(1)
