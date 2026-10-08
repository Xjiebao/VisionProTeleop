#!/bin/zsh
set -eu

script_dir="${0:A:h}"
cd "$script_dir"
if (( $# == 0 )); then
  exec "$script_dir/ego.command" record
fi
build_dir="$script_dir/work/mac_capture"
mkdir -p "$build_dir"

cat > "$build_dir/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
<key>CFBundleIdentifier</key><string>local.egocalibration.capture</string>
<key>CFBundleName</key><string>Ego Camera Capture</string>
<key>NSCameraUsageDescription</key><string>录制外置双目相机视频，并保存每帧时间以对齐 Vision Pro 位姿。</string>
</dict></plist>
PLIST

print -u2 "编译 Mac 采集程序……"
xcrun swiftc -swift-version 5 -O -target "$(uname -m)-apple-macosx14.0" \
  -module-cache-path "$build_dir/module_cache" \
  "$script_dir/record_calibration.swift" -o "$build_dir/record_calibration" \
  -Xlinker -sectcreate -Xlinker __TEXT -Xlinker __info_plist -Xlinker "$build_dir/Info.plist" >&2
codesign --force --sign - --identifier local.egocalibration.capture "$build_dir/record_calibration" >&2

if [[ "$#" -eq 1 && "$1" == "--build-only" ]]; then
  print -u2 "编译完成：$build_dir/record_calibration"
  exit 0
fi
exec "$build_dir/record_calibration" "$@"
