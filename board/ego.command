#!/bin/sh
set -eu
script_dir=$(CDPATH= cd -P "$(dirname "$0")" && pwd)
if [ ! -x "$script_dir/.venv/bin/python" ]; then
  printf '%s\n' "缺少工具包 Python 环境，请先运行 ./setup_ego.command" >&2
  exit 1
fi
exec "$script_dir/.venv/bin/python" "$script_dir/ego_cli.py" "$@"
