#!/usr/bin/env python3
"""独立测量 VP − Mac 的 Unix 时间差；录制前后各运行一次。"""

import argparse
from datetime import datetime
import ipaddress
import json
import math
from pathlib import Path
import socket
import statistics
import sys
import time


PORT = 8766
ROUNDS = 20
INTERVAL = .1
REPLY_TIMEOUT = .5


def summarize(exchanges):
    if len(exchanges) < 5:
        raise ValueError(f"有效对钟只有 {len(exchanges)} 次，至少需要 5 次；请检查 VP 对钟服务和网络后重试")
    rtts, offsets, midpoints = [], [], []
    previous = None
    for record in exchanges:
        t1, t2, t3, t4 = (record[key] for key in ("t1", "t2", "t3", "t4"))
        if not all(math.isfinite(t) for t in (t1, t2, t3, t4)):
            raise ValueError("对钟时间不是有限数值")
        if t4 < t1 or t3 < t2 or (previous is not None and t1 <= previous):
            raise ValueError("对钟期间 Unix 时间顺序异常，请重新对钟")
        previous = t1
        rtt = (t4 - t1) - (t3 - t2)
        if rtt < -1e-6:
            raise ValueError("对钟网络往返时间为负，可能发生系统校时；请重新对钟")
        rtts.append(max(0., rtt))
        offsets.append(((t2 - t1) + (t3 - t4)) / 2)
        midpoints.append(t1 + (t4 - t1) / 2)
    median = statistics.median(rtts)
    mad = statistics.median(abs(rtt - median) for rtt in rtts)
    cutoff = min(.05, statistics.quantiles(rtts, n=4, method="inclusive")[0] + max(.002, 3 * mad))
    retained = [i for i, rtt in enumerate(rtts) if rtt <= cutoff]
    if len(retained) < 5:
        raise ValueError("剔除高延迟后，网络往返不超过 50 ms 的样本不足 5 次；请改善网络后重试")
    offset = statistics.mean(offsets[i] for i in retained)
    residual = max(abs(offsets[i] - offset) for i in retained)
    if residual > .02:
        raise ValueError("对钟偏移最大残差超过 20 ms，可能发生系统校时或网络不稳定；请重新对钟")
    reference = midpoints[retained[0]]
    return {
        "mac_system_time": reference + statistics.mean(midpoints[i] - reference for i in retained),
        "offset_vp_minus_mac_seconds": offset,
        "network_rtt_ms": statistics.median(rtts[i] for i in retained) * 1000,
        "max_abs_offset_residual_ms": residual * 1000,
        "samples_retained": len(retained),
    }


def measure(vp_ip):
    # UDP connect 将接收来源限定为指定 VP 的 IP 和端口。
    started = time.time()
    exchanges = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
        connection.connect((vp_ip, PORT))
        for seq in range(ROUNDS):
            round_start = time.monotonic()
            deadline = round_start + REPLY_TIMEOUT
            t1 = time.time()
            request = {"type": "clock_probe", "clock": "unix", "seq": seq, "t1": t1}
            connection.send(json.dumps(request).encode("utf-8"))
            while time.monotonic() < deadline:
                connection.settimeout(max(.001, deadline - time.monotonic()))
                try:
                    data = connection.recv(65535)
                    t4 = time.time()
                except socket.timeout:
                    break
                try:
                    reply = json.loads(data)
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if (not isinstance(reply, dict) or reply.get("type") != "clock_reply"
                        or reply.get("clock") != "unix" or type(reply.get("seq")) is not int
                        or reply["seq"] != seq or type(reply.get("t1")) not in (float, int)
                        or reply["t1"] != t1
                        or any(type(reply.get(key)) not in (float, int)
                               or not math.isfinite(reply[key]) for key in ("t2", "t3"))):
                    continue
                exchanges.append({"seq": seq, "clock": "unix", "t1": t1,
                                  "t2": reply["t2"], "t3": reply["t3"], "t4": t4})
                break
            remaining = round_start + INTERVAL - time.monotonic()
            if seq + 1 < ROUNDS and remaining > 0:
                time.sleep(remaining)
    finished = time.time()
    if not exchanges:
        raise ValueError("未收到有效 VP 对钟回复；请先在 VP 开启电脑对钟，确认 IP 和局域网连接")
    return {
        "schema_version": 1, "kind": "clock_sync", "clock": "unix", "status": "completed",
        "vp_ip": vp_ip, "started_system_time": started, "finished_system_time": finished,
        "exchanges": exchanges, "summary": summarize(exchanges),
    }


def timestamped_path(directory, timestamp):
    stem = datetime.fromtimestamp(timestamp).strftime("%y%m%d%H%M") + "sync"
    path = Path(directory) / (stem + ".json")
    number = 2
    while path.exists():
        path = Path(directory) / f"{stem}_{number:02d}.json"
        number += 1
    return path


def save_measurement(result, output):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(f"已保存对钟：{output.resolve()}")
    summary = result["summary"]
    print(f"VP − Mac = {summary['offset_vp_minus_mac_seconds']:+.6f} 秒；"
          f"保留 {summary['samples_retained']} 次，RTT {summary['network_rtt_ms']:.2f} ms")


def main(argv=None):
    parser = argparse.ArgumentParser(description="独立测量 VP − Mac 钟差，自动按测量时间命名")
    parser.add_argument("--vp", required=True, help="VP 的 IPv4 地址")
    outputs = parser.add_mutually_exclusive_group()
    outputs.add_argument("--output", type=Path, help="指定新建 JSON 路径；已有文件不覆盖")
    outputs.add_argument("--output-dir", type=Path, default=Path.cwd(), help="自动命名的保存目录，默认当前目录")
    args = parser.parse_args(argv)
    try:
        vp_ip = str(ipaddress.IPv4Address(args.vp))
        if args.output is not None and args.output.expanduser().exists():
            raise FileExistsError(f"结果路径已存在，不会覆盖：{args.output}")
        print(f"正在向 {vp_ip}:{PORT} 对钟，共 {ROUNDS} 轮…", flush=True)
        result = measure(vp_ip)
        output = (args.output.expanduser() if args.output is not None else
                  timestamped_path(args.output_dir.expanduser(), result["started_system_time"]))
        save_measurement(result, output)
        return 0
    except (OSError, ValueError, EOFError) as error:
        print(f"对钟失败：{error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n对钟已取消。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
