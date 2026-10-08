#!/bin/sh
set -eu
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ "$#" -eq 0 ]; then
    exec "$script_dir/ego.command" record
fi
build_dir="$script_dir/work/linux_capture"
mkdir -p "$build_dir"
if [ ! -x "$build_dir/record_linux" ] || [ "$script_dir/record_linux.cpp" -nt "$build_dir/record_linux" ]; then
    printf '%s\n' '编译 Linux V4L2 采集程序……' >&2
    g++ -std=c++17 -O2 -Wall -Wextra "$script_dir/record_linux.cpp" -o "$build_dir/record_linux" \
        $(pkg-config --cflags --libs libavformat libavcodec libavutil) >&2
fi
if [ "$#" -eq 1 ] && [ "$1" = '--build-only' ]; then
    printf '编译完成：%s\n' "$build_dir/record_linux" >&2
    exit 0
fi
exec "$build_dir/record_linux" "$@"
