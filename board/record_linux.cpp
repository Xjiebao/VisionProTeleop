#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
}
namespace fs = std::filesystem;
static volatile sig_atomic_t stopping = 0;
static void stopSignal(int) { stopping = 1; }
static std::string quoted(const std::string &s) {
    std::string out = "\"";
    const char *hex = "0123456789abcdef";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c < 32) { out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15]; }
        else out += c;
    }
    return out + "\"";
}
static int xioctl(int fd, unsigned long request, void *arg) {
    int r;
    do { r = ioctl(fd, request, arg); } while (r < 0 && errno == EINTR);
    return r;
}
static void checkIO(int r, const std::string &operation) {
    if (r < 0) throw std::runtime_error(operation + ": " + std::strerror(errno));
}
static void checkAV(int r, const std::string &operation) {
    if (r < 0) {
        char message[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(r, message, sizeof(message));
        throw std::runtime_error(operation + ": " + message);
    }
}
static double clockSeconds(clockid_t clock) {
    timespec t{};
    checkIO(clock_gettime(clock, &t), "clock_gettime");
    return t.tv_sec + t.tv_nsec / 1e9;
}
struct Device { std::string id, name, node; bool supported; };
static bool supportsMode(int fd) {
    v4l2_fmtdesc format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (format.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &format) == 0; ++format.index) {
        if (format.pixelformat != V4L2_PIX_FMT_MJPEG) continue;
        v4l2_frmsizeenum size{};
        size.pixel_format = V4L2_PIX_FMT_MJPEG;
        for (size.index = 0; xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
            if (size.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
                size.discrete.width == 4000 && size.discrete.height == 1200) return true;
        }
    }
    return false;
}
static std::vector<Device> cameras() {
    std::map<std::string, std::string> stableIDs;
    for (const char *directory : {"/dev/v4l/by-id", "/dev/v4l/by-path"}) {
        if (!fs::exists(directory)) continue;
        std::vector<fs::path> entries;
        for (const auto &entry : fs::directory_iterator(directory)) {
            // udev USB stable paths contain "usb"; ignore unconnected SoC MIPI nodes.
            if (entry.path().filename().string().find("usb") != std::string::npos) entries.push_back(entry.path());
        }
        std::sort(entries.begin(), entries.end());
        for (const auto &path : entries) stableIDs.emplace(fs::canonical(path).string(), path.string());
    }
    std::vector<Device> result;
    for (const auto &item : stableIDs) {
        int fd = open(item.first.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            std::cerr << "无法检查设备 " << item.second << ": " << std::strerror(errno) << '\n';
            continue;
        }
        v4l2_capability cap{};
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            uint32_t flags = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
            std::string bus(reinterpret_cast<char *>(cap.bus_info));
            if (bus.rfind("usb-", 0) == 0 && (flags & V4L2_CAP_VIDEO_CAPTURE) && (flags & V4L2_CAP_STREAMING)) {
                result.push_back({item.second, reinterpret_cast<char *>(cap.card), item.first, supportsMode(fd)});
            }
        }
        close(fd);
    }
    std::sort(result.begin(), result.end(), [](const Device &a, const Device &b) { return a.id < b.id; });
    return result;
}
struct Recorder {
    struct Mapping { void *address; size_t length; };
    Device device;
    fs::path out, preview;
    bool previewOnly = false;
    int fd = -1;
    bool streaming = false, headerWritten = false, finalized = false;
    std::vector<Mapping> mappings;
    AVFormatContext *movie = nullptr;
    AVStream *stream = nullptr;
    AVPacket *packet = nullptr;
    std::ofstream frames;
    std::string status = "starting", error, timestampSource = "unverified";
    double selectedFPS = 0, startHost = 0, finishHost = 0;
    double maxClockPairSpan = 0, lastPreviewHost = -1;
    int64_t firstUS = -1, lastUS = -1;
    uint64_t received = 0, written = 0, sourceDrops = 0, startupDiscarded = 0;
    uint32_t firstSequence = 0, lastSequence = 0;

    Recorder(const Device &d, const fs::path &p, const fs::path &previewPath, bool onlyPreview)
        : device(d), out(p), preview(previewPath), previewOnly(onlyPreview) { startHost = clockSeconds(CLOCK_MONOTONIC); }
    ~Recorder() {
        if (streaming) { v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(fd, VIDIOC_STREAMOFF, &type); }
        for (const auto &m : mappings) munmap(m.address, m.length);
        if (fd >= 0) close(fd);
        av_packet_free(&packet);
        if (movie) { if (movie->pb) avio_closep(&movie->pb); avformat_free_context(movie); }
    }
    void metadata() {
        if (previewOnly) return;
        std::ofstream file(out / "session.json");
        file.exceptions(std::ios::failbit | std::ios::badbit);
        const double span = firstUS >= 0 && lastUS >= 0 ? (lastUS - firstUS) / 1e6 : 0;
        file << std::setprecision(17)
             << "{\n  \"schema_version\": 2,\n  \"status\": " << quoted(status)
             << ",\n  \"video\": \"video.mov\",\n  \"device_name\": " << quoted(device.name)
             << ",\n  \"device_unique_id\": " << quoted(device.id)
             << ",\n  \"device_node\": " << quoted(device.node)
             << ",\n  \"width\": 4000,\n  \"height\": 1200,\n  \"capture_mode\": \"independent\","
             << "\n  \"host_clock\": \"CLOCK_MONOTONIC\",\n  \"alignment_clock\": \"unix\","
             << "\n  \"frame_unix_field\": \"systemTime\","
             << "\n  \"systemTime\": \"V4L2 monotonic buffer timestamp + per-frame CLOCK_REALTIME minus midpoint of bracketing CLOCK_MONOTONIC reads\","
             << "\n  \"capture_host_seconds\": \"original V4L2 buffer timestamp, CLOCK_MONOTONIC seconds\","
             << "\n  \"pts_seconds\": \"original integer-microsecond V4L2 timestamp minus first written timestamp; MOV track time_base=1/1000000\","
             << "\n  \"timestamp_source\": " << quoted(timestampSource)
             << ",\n  \"timestamp_note\": \"Driver timestamp converted to Unix; timestamp source is reported by V4L2. Physical exposure timing and VP pose latency are not verified.\","
             << "\n  \"pixel_layout\": \"[160 code band][1920 right][1920 left]; no resize/rotation/mirroring\","
             << "\n  \"encoding\": \"Original UVC MJPEG packets remuxed into MOV; no software re-encoding; camera JPEG compression is lossy\","
             << "\n  \"source_media_subtype\": \"MJPG\",\n  \"requested_fps\": 30,"
             << "\n  \"selected_fps\": " << selectedFPS
             << ",\n  \"actual_frame_duration_seconds\": " << (selectedFPS > 0 ? 1 / selectedFPS : 0)
             << ",\n  \"started_host_seconds\": " << startHost
             << ",\n  \"finished_host_seconds\": " << finishHost
             << ",\n  \"capture_frames_received\": " << received
             << ",\n  \"startup_discarded_frames\": " << startupDiscarded
             << ",\n  \"frames_written\": " << written
             << ",\n  \"source_dropped_frames\": " << sourceDrops
             << ",\n  \"dropped_capture_frames\": " << sourceDrops
             << ",\n  \"dropped_writer_frames\": 0,"
             << "\n  \"first_sequence\": " << firstSequence
             << ",\n  \"last_sequence\": " << lastSequence
             << ",\n  \"written_pts_span_seconds\": " << span
             << ",\n  \"capture_pts_span_seconds\": " << span
             << ",\n  \"actual_capture_fps\": " << (span > 0 ? (received - startupDiscarded - 1) / span : 0)
             << ",\n  \"actual_written_fps\": " << (span > 0 ? (written - 1) / span : 0)
             << ",\n  \"max_clock_pair_span_seconds\": " << maxClockPairSpan;
        if (!error.empty()) file << ",\n  \"error\": " << quoted(error);
        file << "\n}\n";
        file.close();
    }
    void start() {
        metadata();
        fd = open(device.node.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        checkIO(fd, "打开相机");
        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = 4000; format.fmt.pix.height = 1200;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        format.fmt.pix.field = V4L2_FIELD_NONE;
        checkIO(xioctl(fd, VIDIOC_S_FMT, &format), "设置 MJPEG 模式");
        if (format.fmt.pix.width != 4000 || format.fmt.pix.height != 1200 ||
            format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
            throw std::runtime_error("驱动未接受 MJPEG 4000×1200；停止录制");
        v4l2_streamparm rate{};
        rate.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rate.parm.capture.timeperframe = {1, 30};
        checkIO(xioctl(fd, VIDIOC_S_PARM, &rate), "设置 30 FPS");
        checkIO(xioctl(fd, VIDIOC_G_PARM, &rate), "读取实际帧率设置");
        if (!rate.parm.capture.timeperframe.numerator || !rate.parm.capture.timeperframe.denominator)
            throw std::runtime_error("驱动未返回有效帧率");
        selectedFPS = double(rate.parm.capture.timeperframe.denominator) / rate.parm.capture.timeperframe.numerator;
        if (std::abs(selectedFPS - 30) > 0.5) throw std::runtime_error("相机未接受约 30 FPS 的模式");
        v4l2_requestbuffers request{};
        request.count = 8; request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; request.memory = V4L2_MEMORY_MMAP;
        checkIO(xioctl(fd, VIDIOC_REQBUFS, &request), "申请 V4L2 缓冲区");
        if (request.count < 2) throw std::runtime_error("V4L2 缓冲区不足");
        for (uint32_t i = 0; i < request.count; ++i) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
            checkIO(xioctl(fd, VIDIOC_QUERYBUF, &b), "查询 V4L2 缓冲区");
            void *address = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
            if (address == MAP_FAILED) throw std::runtime_error("映射 V4L2 缓冲区失败");
            mappings.push_back({address, b.length});
            checkIO(xioctl(fd, VIDIOC_QBUF, &b), "提交 V4L2 缓冲区");
        }
        if (!previewOnly) {
            frames.open(out / "frames.jsonl");
            frames.exceptions(std::ios::failbit | std::ios::badbit);
            frames << std::setprecision(17);
            checkAV(avformat_alloc_output_context2(&movie, nullptr, "mov", (out / "video.mov").c_str()), "创建 MOV");
            stream = avformat_new_stream(movie, nullptr);
            if (!stream) throw std::runtime_error("创建 MOV 轨道失败");
            stream->time_base = {1, 1000000};
            stream->avg_frame_rate = av_inv_q({int(rate.parm.capture.timeperframe.numerator), int(rate.parm.capture.timeperframe.denominator)});
            stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            stream->codecpar->codec_id = AV_CODEC_ID_MJPEG;
            stream->codecpar->width = 4000; stream->codecpar->height = 1200;
            checkAV(avio_open(&movie->pb, (out / "video.mov").c_str(), AVIO_FLAG_WRITE), "打开视频输出");
            AVDictionary *options = nullptr;
            av_dict_set(&options, "video_track_timescale", "1000000", 0);
            int headerResult = avformat_write_header(movie, &options);
            av_dict_free(&options);
            checkAV(headerResult, "写入 MOV 头");
            headerWritten = true;
            if (stream->time_base.num != 1 || stream->time_base.den != 1000000)
                throw std::runtime_error("MOV 未保留 1 微秒时间基");
            packet = av_packet_alloc();
            if (!packet) throw std::runtime_error("分配视频 packet 失败");
        }
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        checkIO(xioctl(fd, VIDIOC_STREAMON, &type), "启动视频流");
        streaming = true;
        status = "recording";
        metadata();
    }
    void capture(double duration) {
        double begun = clockSeconds(CLOCK_MONOTONIC), lastReceive = begun;
        bool readInput = true;
        while (!stopping) {
            double now = clockSeconds(CLOCK_MONOTONIC);
            if (duration > 0 && now - begun >= duration) break;
            if (now - lastReceive >= 10) throw std::runtime_error("连续 10 秒未收到相机帧");
            pollfd polls[2] = {{fd, POLLIN, 0}, {readInput ? STDIN_FILENO : -1, POLLIN, 0}};
            int result = poll(polls, 2, 200);
            if (result < 0 && errno == EINTR) continue;
            checkIO(result, "等待视频帧");
            if (polls[1].revents & (POLLIN | POLLHUP)) {
                char input[256];
                ssize_t count = read(STDIN_FILENO, input, sizeof(input));
                if (count > 0) break;
                if (count == 0) readInput = false;
                if (count < 0 && errno != EINTR) checkIO(-1, "读取停止指令");
            }
            if (polls[0].revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("相机连接或视频流中断");
            if (!(polls[0].revents & POLLIN)) continue;
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
            int dequeued = xioctl(fd, VIDIOC_DQBUF, &b);
            if (dequeued < 0 && errno == EAGAIN) continue;
            checkIO(dequeued, "读取相机帧");
            double before = clockSeconds(CLOCK_MONOTONIC);
            double unixNow = clockSeconds(CLOCK_REALTIME);
            double after = clockSeconds(CLOCK_MONOTONIC);
            lastReceive = before;
            maxClockPairSpan = std::max(maxClockPairSpan, after - before);
            ++received;
            if (b.index >= mappings.size() || b.bytesused == 0 || b.bytesused > mappings[b.index].length ||
                (b.flags & V4L2_BUF_FLAG_ERROR)) throw std::runtime_error("相机返回无效或损坏的视频帧");
            if ((b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                throw std::runtime_error("驱动时间戳不是明确的 CLOCK_MONOTONIC，无法可靠对齐");
            uint32_t source = b.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
            std::string sourceName;
            if (source == V4L2_BUF_FLAG_TSTAMP_SRC_SOE) sourceName = "SOE (start of exposure, driver reported)";
            else if (source == V4L2_BUF_FLAG_TSTAMP_SRC_EOF) sourceName = "EOF (end of frame, driver reported)";
            else throw std::runtime_error("未知的 V4L2 时间戳采样位置");
            if (written == 0) { timestampSource = sourceName; firstSequence = b.sequence; }
            else if (sourceName != timestampSource) throw std::runtime_error("驱动时间戳采样位置在录制中变化");
            if (written > 0) {
                uint32_t sequenceGap = b.sequence - lastSequence;
                if (sequenceGap == 0 || sequenceGap > 0x7fffffffU) throw std::runtime_error("相机序号未递增");
                sourceDrops += sequenceGap - 1;
            }
            lastSequence = b.sequence;
            if (b.timestamp.tv_sec < 0 || b.timestamp.tv_usec < 0 || b.timestamp.tv_usec >= 1000000)
                throw std::runtime_error("相机帧时间无效");
            int64_t stampUS = int64_t(b.timestamp.tv_sec) * 1000000 + b.timestamp.tv_usec;
            if (stampUS <= 0 || (lastUS >= 0 && stampUS <= lastUS)) throw std::runtime_error("相机帧时间戳未严格递增");
            // This camera's first timestamp precedes the steady stream by about 1.4 s.
            // Skip that frame in both outputs; preserve absolute times of all retained frames.
            if (received == 1) {
                ++startupDiscarded;
                checkIO(xioctl(fd, VIDIOC_QBUF, &b), "丢弃启动首帧并重新提交缓冲区");
                continue;
            }
            if (firstUS < 0) firstUS = stampUS;
            int64_t ptsUS = stampUS - firstUS;
            if (!previewOnly) {
                packet->data = static_cast<uint8_t *>(mappings[b.index].address);
                packet->size = int(b.bytesused);
                packet->stream_index = stream->index;
                packet->pts = packet->dts = ptsUS;
                packet->duration = std::llround(1000000 / selectedFPS);
                packet->flags = AV_PKT_FLAG_KEY;
                checkAV(av_write_frame(movie, packet), "写入视频帧");
                if (movie->pb->error < 0) checkAV(movie->pb->error, "写入视频文件");
                const double captureHost = stampUS / 1e6;
                const double systemTime = unixNow + (captureHost - (before + after) / 2);
                frames << "{\"frame_index\":" << written << ",\"pts_seconds\":" << ptsUS / 1e6
                       << ",\"systemTime\":" << systemTime << ",\"capture_host_seconds\":" << captureHost
                       << ",\"received_host_seconds\":" << before << ",\"source_sequence\":" << b.sequence
                       << ",\"v4l2_flags\":" << b.flags << "}\n";
            }
            if (!preview.empty() && (lastPreviewHost < 0 || before - lastPreviewHost >= 0.33)) {
                // Publish an untouched JPEG from this same stream, without exposing partial writes.
                std::ofstream snapshot(preview / "latest.tmp", std::ios::binary);
                snapshot.exceptions(std::ios::failbit | std::ios::badbit);
                snapshot.write(static_cast<const char *>(mappings[b.index].address), b.bytesused);
                snapshot.close();
                fs::rename(preview / "latest.tmp", preview / "latest.jpg");
                lastPreviewHost = before;
            }
            lastUS = stampUS;
            ++written;
            if (written == 1) {
                if (!previewOnly) {
                    frames.flush();
                    avio_flush(movie->pb);
                    checkAV(movie->pb->error, "确认首帧写入");
                }
                std::cout << "CAPTURE_READY" << std::endl;
            }
            checkIO(xioctl(fd, VIDIOC_QBUF, &b), "重新提交 V4L2 缓冲区");
        }
        if (!written) throw std::runtime_error(previewOnly ? "没有获取任何预览帧" : "没有保存任何视频帧");
    }
    void finish() {
        std::string stopError;
        if (streaming) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            int r = xioctl(fd, VIDIOC_STREAMOFF, &type);
            streaming = false;
            if (r < 0) stopError = "停止视频流: " + std::string(std::strerror(errno));
        }
        if (headerWritten && !finalized) {
            finalized = true;
            checkAV(av_write_trailer(movie), "完成 MOV 封装");
            avio_flush(movie->pb);
            checkAV(movie->pb->error, "完成视频写入");
            checkAV(avio_closep(&movie->pb), "关闭视频文件");
        }
        if (frames.is_open()) { frames.flush(); frames.close(); }
        if (!stopError.empty()) throw std::runtime_error(stopError);
    }
};
static int run(int argc, char **argv) {
    if (argc == 1 || (argc == 2 && std::string(argv[1]) == "--help")) {
        std::cout << "列出相机：record_linux --list 或 --list-json\n录制：record_linux --out 新空目录 [--camera 编号或稳定ID] [--duration 秒] [--preview 快照目录]\n仅预览：record_linux --preview-only --preview 快照目录 [--camera 编号或稳定ID] [--duration 秒]\n仅支持 USB MJPEG 4000×1200 约30FPS，要求 MONOTONIC 驱动时间戳。\n按 Enter 或 Ctrl-C 停止；预览快照保存为 latest.jpg，约每 0.33 秒更新。\n";
        return 0;
    }
    auto devices = cameras();
    if (argc == 2 && (std::string(argv[1]) == "--list" || std::string(argv[1]) == "--list-json")) {
        bool json = std::string(argv[1]) == "--list-json";
        if (json) std::cout << '[';
        for (size_t i = 0; i < devices.size(); ++i) {
            const auto &d = devices[i];
            if (json) {
                if (i) std::cout << ',';
                std::cout << "{\"name\":" << quoted(d.name) << ",\"unique_id\":" << quoted(d.id)
                          << ",\"external\":true,\"supports_4000x1200\":" << (d.supported ? "true" : "false") << '}';
            } else std::cout << '[' << i << "] " << d.name << "\n    uniqueID: " << d.id << "\n    MJPEG 4000×1200: " << (d.supported ? "支持" : "不支持") << '\n';
        }
        if (json) std::cout << "]\n";
        else if (devices.empty()) std::cout << "没有发现具有稳定标识的 USB 采集相机；检查连接、video 组权限及 /dev/v4l/by-id 或 by-path。\n";
        return 0;
    }
    std::map<std::string, std::string> options;
    bool previewOnly = false;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key == "--preview-only") {
            if (previewOnly) throw std::runtime_error("重复参数：" + key);
            previewOnly = true;
            continue;
        }
        if (i + 1 >= argc || (key != "--camera" && key != "--out" && key != "--duration" && key != "--preview") || options.count(key))
            throw std::runtime_error("未知、重复或缺少值的参数：" + key);
        options[key] = argv[++i];
    }
    if (previewOnly && !options.count("--preview")) throw std::runtime_error("--preview-only 必须指定 --preview 快照目录");
    if (!previewOnly && !options.count("--out")) throw std::runtime_error("必须指定 --out 输出目录");
    double duration = 0;
    if (options.count("--duration")) {
        size_t consumed = 0;
        duration = std::stod(options["--duration"], &consumed);
        if (!std::isfinite(duration) || duration <= 0 || consumed != options["--duration"].size())
            throw std::runtime_error("--duration 必须为正秒数");
    }
    int chosen = -1;
    for (size_t i = 0; i < devices.size(); ++i) {
        if (options.count("--camera")) {
            if (devices[i].id == options["--camera"] || std::to_string(i) == options["--camera"]) chosen = int(i);
        } else if (devices[i].supported && devices[i].name.find("DECXIN") != std::string::npos) {
            if (chosen >= 0) throw std::runtime_error("多台 DECXIN 相机，请用 --camera 指定稳定 ID");
            chosen = int(i);
        }
    }
    if (chosen < 0) throw std::runtime_error("未找到相机，可尝试重启开发板");
    if (!devices[chosen].supported) throw std::runtime_error("相机不支持 MJPEG 4000×1200");
    fs::path out, preview;
    if (!previewOnly) {
        out = fs::absolute(options["--out"]);
        if (fs::exists(out) && (!fs::is_directory(out) || !fs::is_empty(out)))
            throw std::runtime_error("输出目录已存在且非空；不会覆盖");
        fs::create_directories(out);
    }
    if (options.count("--preview")) {
        if (options["--preview"].empty()) throw std::runtime_error("--preview 快照目录不能为空");
        preview = fs::absolute(options["--preview"]);
        fs::create_directories(preview);
    }
    Recorder recorder(devices[chosen], out, preview, previewOnly);
    signal(SIGINT, stopSignal); signal(SIGTERM, stopSignal); signal(SIGHUP, stopSignal);
    try {
        recorder.start();
        std::cout << "相机：" << devices[chosen].name << "，MJPEG 4000×1200，目标 " << recorder.selectedFPS
                  << " FPS。\n" << (previewOnly ? "正在预览" : "正在录制")
                  << "，按 Enter 或 Ctrl-C 停止。\n输出：" << (previewOnly ? preview : out) << std::endl;
        recorder.capture(duration);
        recorder.finish();
        recorder.status = "completed";
        recorder.finishHost = clockSeconds(CLOCK_MONOTONIC);
        recorder.metadata();
        std::cout << (previewOnly ? "预览完成：" : "录制完成：") << recorder.written << " 帧；驱动序号间隙 " << recorder.sourceDrops
                  << " 帧。时间戳：" << recorder.timestampSource << '\n';
    } catch (const std::exception &e) {
        recorder.status = "failed"; recorder.error = e.what();
        try { recorder.finish(); }
        catch (const std::exception &finishError) { recorder.error += "; 收尾失败: "; recorder.error += finishError.what(); }
        recorder.finishHost = clockSeconds(CLOCK_MONOTONIC);
        recorder.metadata();
        throw std::runtime_error(recorder.error);
    }
    return 0;
}
int main(int argc, char **argv) {
    try { return run(argc, argv); }
    catch (const std::exception &e) { std::cerr << "失败：" << e.what() << '\n'; return 1; }
}
