#!/bin/sh
# 一次安装开机服务；不启动录像或离线处理。
set -eu
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi
service_user=${SUDO_USER:-$(stat -c '%U' "$script_dir")}
if [ "$service_user" = root ]; then
    printf '%s\n' '请由采集工具包所属的普通用户运行本脚本。' >&2
    exit 1
fi
if ! command -v avahi-daemon >/dev/null 2>&1; then
    apt-get update
    apt-get install -y avahi-daemon
fi
if ! command -v python3 >/dev/null 2>&1; then
    printf '%s\n' '缺少 python3，请先完成采集环境安装。' >&2
    exit 1
fi
if [ ! -r /etc/machine-id ] || [ ! -s /etc/machine-id ]; then
    printf '%s\n' '缺少 /etc/machine-id，无法获得稳定设备编号。' >&2
    exit 1
fi
usermod -a -G video "$service_user"
python3 - "$script_dir" "$service_user" <<'PY'
import pathlib
import socket
import sys
from xml.sax.saxutils import escape
root, user = sys.argv[1:]
board_id = pathlib.Path('/etc/machine-id').read_text().strip()
quoted_root = root.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
unit = f'''[Unit]
Description=EgoCapture board control and VP file receiver
After=network.target avahi-daemon.service
Wants=avahi-daemon.service

[Service]
Type=simple
User={user}
SupplementaryGroups=video
WorkingDirectory={root.replace('%', '%%')}
ExecStart=/usr/bin/python3 "{quoted_root}/board_service.py"
Restart=on-failure
RestartSec=3
TimeoutStopSec=30
KillMode=control-group

[Install]
WantedBy=multi-user.target
'''
pathlib.Path('/etc/systemd/system/egocapture-board.service').write_text(unit)
advertisement = f'''<?xml version="1.0" standalone="no"?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">EgoCapture %h</name>
  <service>
    <type>_egocapture._tcp</type>
    <port>8767</port>
    <txt-record>id={escape(board_id)}</txt-record>
    <txt-record>name={escape(socket.gethostname())}</txt-record>
  </service>
</service-group>
'''
pathlib.Path('/etc/avahi/services/egocapture.service').write_text(advertisement)
PY
systemctl daemon-reload
systemctl enable --now avahi-daemon.service
systemctl enable --now egocapture-board.service
systemctl is-active --quiet egocapture-board.service
printf '%s\n' '已安装并启动 EgoCapture 服务，下次板子上电自动运行；当前没有启动录像。'
