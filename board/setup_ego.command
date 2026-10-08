#!/bin/sh
set -eu
script_dir=$(CDPATH= cd -P "$(dirname "$0")" && pwd)
python3 -c 'import sys; assert (3, 9) <= sys.version_info[:2] <= (3, 12), "锁定的依赖版本使用 Python 3.9–3.12，请使用该版本创建环境"'
case "$(uname -s)" in
  Darwin)
    xcrun --find swiftc >/dev/null
    environment_module=venv
    capture="$script_dir/start_capture.command"
    ;;
  Linux)
    if ! command -v g++ >/dev/null 2>&1 || ! command -v pkg-config >/dev/null 2>&1 || ! command -v ffmpeg >/dev/null 2>&1 || ! pkg-config --exists libavformat libavcodec libavutil; then
      printf '%s\n' "缺少 Linux 采集编译依赖。Debian/Ubuntu 请安装："         "sudo apt install g++ pkg-config ffmpeg libavformat-dev libavcodec-dev libavutil-dev python3-virtualenv" >&2
      exit 1
    fi
    if ! python3 -c "import virtualenv" 2>/dev/null; then
      printf '%s\n' "缺少 Linux Python 环境工具。Debian/Ubuntu 请安装：sudo apt install python3-virtualenv" >&2
      exit 1
    fi
    environment_module=virtualenv
    capture="$script_dir/start_capture_linux.sh"
    ;;
  *)
    printf '%s\n' "采集工具仅支持 macOS 和 Linux。" >&2
    exit 1
    ;;
esac
python3 -m "$environment_module" "$script_dir/.venv"
"$script_dir/.venv/bin/python" -m pip install --only-binary=:all: -r "$script_dir/requirements-ego.txt"
"$capture" --build-only
printf '%s\n' "准备完成。使用 ./ego.command --help 查看命令。"
