#include "system/loggerd/encoder/cluster_h264_encoder.h"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

void usage(const char *prog) {
  std::cerr
      << "Usage: " << prog << " --width W --height H [options]\n"
      << "\n"
      << "Reads raw R8G8B8A8 frames from stdin and writes raw H264 bytes to stdout.\n"
      << "\n"
      << "Options:\n"
      << "  --fps N                 Input FPS. Default 30.\n"
      << "  --bitrate BPS           Target bitrate in bits/s; K/M suffixes are accepted. Default 1000000.\n"
      << "  --gop N                 Keyframe interval in frames. Default 30.\n"
      << "  --slice-max-bytes N     V4L2 multi-slice max bytes; 0 disables. Default 4096.\n"
      << "  --rate-control MODE     V4L2 rate control: off, vbr-vfr, vbr-cfr, cbr-vfr,\n"
      << "                          cbr-cfr, mbr-cfr, mbr-vfr, cq. Default vbr-cfr.\n"
      << "  --realtime-priority     Request realtime encoder priority.\n"
      << "  --device PATH           V4L2 encoder device path.\n"
      << "  --input-format auto|rgb4|nv12\n"
      << "                          Hardware input format. Default auto.\n"
      << "  --rgb4-layout axrgb|rgba|bgra\n"
      << "                          RGBA to RGB4 byte layout. Default bgra.\n"
      << "  --debug                 Enable verbose encoder logging.\n";
}

int parse_int(const std::string &name, const std::string &value) {
  char *end = nullptr;
  errno = 0;
  long parsed = strtol(value.c_str(), &end, 10);
  if (errno != 0 || end == value.c_str() || *end != '\0' || parsed <= 0 || parsed > INT_MAX) {
    throw std::runtime_error("invalid " + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

int parse_nonnegative_int(const std::string &name, const std::string &value) {
  char *end = nullptr;
  errno = 0;
  long parsed = strtol(value.c_str(), &end, 10);
  if (errno != 0 || end == value.c_str() || *end != '\0' || parsed < 0 || parsed > INT_MAX) {
    throw std::runtime_error("invalid " + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

int parse_bitrate(const std::string &value) {
  std::string text = value;
  double multiplier = 1.0;
  if (!text.empty()) {
    const char suffix = text.back();
    if (suffix == 'k' || suffix == 'K') {
      multiplier = 1000.0;
      text.pop_back();
    } else if (suffix == 'm' || suffix == 'M') {
      multiplier = 1000000.0;
      text.pop_back();
    }
  }

  char *end = nullptr;
  errno = 0;
  double parsed = strtod(text.c_str(), &end);
  const double scaled = parsed * multiplier;
  if (errno != 0 || end == text.c_str() || *end != '\0' || scaled <= 0.0 || scaled > INT_MAX) {
    throw std::runtime_error("invalid --bitrate: " + value);
  }
  return static_cast<int>(scaled + 0.5);
}

ClusterH264InputFormat parse_input_format(const std::string &value) {
  if (value == "auto") return ClusterH264InputFormat::Auto;
  if (value == "rgb4") return ClusterH264InputFormat::RGB4;
  if (value == "nv12") return ClusterH264InputFormat::NV12;
  throw std::runtime_error("invalid --input-format: " + value);
}

ClusterH264Rgb4Layout parse_rgb4_layout(const std::string &value) {
  if (value == "axrgb") return ClusterH264Rgb4Layout::AXRGB;
  if (value == "rgba") return ClusterH264Rgb4Layout::RGBA;
  if (value == "bgra") return ClusterH264Rgb4Layout::BGRA;
  throw std::runtime_error("invalid --rgb4-layout: " + value);
}

int parse_rate_control(const std::string &value) {
  if (value == "off") return 0;
  if (value == "vbr-vfr") return 1;
  if (value == "vbr-cfr") return 2;
  if (value == "cbr-vfr") return 3;
  if (value == "cbr-cfr") return 4;
  if (value == "mbr-cfr") return 5;
  if (value == "mbr-vfr") return 6;
  if (value == "cq") return 7;
  throw std::runtime_error("invalid --rate-control: " + value);
}

const char *rate_control_name(int value) {
  switch (value) {
    case 0: return "off";
    case 1: return "vbr-vfr";
    case 2: return "vbr-cfr";
    case 3: return "cbr-vfr";
    case 4: return "cbr-cfr";
    case 5: return "mbr-cfr";
    case 6: return "mbr-vfr";
    case 7: return "cq";
  }
  return "unknown";
}

bool read_exact(int fd, uint8_t *data, size_t size) {
  size_t offset = 0;
  while (offset < size) {
    ssize_t n = read(fd, data + offset, size - offset);
    if (n == 0) {
      if (offset == 0) return false;
      throw std::runtime_error("short RGBA frame on stdin");
    }
    if (n < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(std::string("stdin read failed: ") + strerror(errno));
    }
    offset += static_cast<size_t>(n);
  }
  return true;
}

void write_all(int fd, const uint8_t *data, size_t size) {
  size_t offset = 0;
  while (offset < size) {
    ssize_t n = write(fd, data + offset, size - offset);
    if (n < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(std::string("stdout write failed: ") + strerror(errno));
    }
    if (n == 0) {
      throw std::runtime_error("stdout write returned zero bytes");
    }
    offset += static_cast<size_t>(n);
  }
}

bool should_log_packet(uint64_t index) {
  return index <= 40 || (index % 30) == 0;
}

void log_packet_debug(uint64_t index, const ClusterH264PacketView &packet) {
  std::cerr << "cluster_h264_encoder_cli packet " << index
            << ": size=" << packet.size
            << " flags=0x" << std::hex << packet.flags << std::dec
            << " ts=" << packet.timestamp_us
            << " codec_config=" << (packet.codec_config ? 1 : 0)
            << " keyframe=" << (packet.keyframe ? 1 : 0)
            << " head=";
  const size_t head_size = std::min<size_t>(packet.size, 16);
  for (size_t i = 0; i < head_size; ++i) {
    if (i != 0) std::cerr << ' ';
    std::cerr << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(packet.data[i]);
  }
  std::cerr << std::dec << std::setfill(' ') << std::endl;
}

}  // namespace

int main(int argc, char **argv) {
  ClusterH264EncoderConfig config;

  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next_value = [&](const std::string &name) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("missing value for " + name);
        }
        return argv[++i];
      };

      if (arg == "--width") {
        config.width = parse_int(arg, next_value(arg));
      } else if (arg == "--height") {
        config.height = parse_int(arg, next_value(arg));
      } else if (arg == "--fps") {
        config.fps = parse_int(arg, next_value(arg));
      } else if (arg == "--bitrate") {
        config.bitrate = parse_bitrate(next_value(arg));
      } else if (arg == "--gop") {
        config.gop = parse_int(arg, next_value(arg));
      } else if (arg == "--slice-max-bytes") {
        config.slice_max_bytes = parse_nonnegative_int(arg, next_value(arg));
      } else if (arg == "--rate-control") {
        config.rate_control = parse_rate_control(next_value(arg));
      } else if (arg == "--realtime-priority") {
        config.realtime_priority = true;
      } else if (arg == "--device") {
        config.device_path = next_value(arg);
      } else if (arg == "--input-format") {
        config.input_format = parse_input_format(next_value(arg));
      } else if (arg == "--rgb4-layout") {
        config.rgb4_layout = parse_rgb4_layout(next_value(arg));
      } else if (arg == "--debug") {
        config.debug = true;
      } else if (arg == "--help" || arg == "-h") {
        usage(argv[0]);
        return 0;
      } else {
        throw std::runtime_error("unknown option: " + arg);
      }
    }

    if (config.width <= 0 || config.height <= 0) {
      usage(argv[0]);
      return 2;
    }

    const size_t frame_size = static_cast<size_t>(config.width) * static_cast<size_t>(config.height) * 4;
    std::vector<uint8_t> frame(frame_size);
    ClusterH264Encoder encoder(config);
    encoder.open();

    std::cerr << "cluster_h264_encoder_cli: "
              << config.width << "x" << config.height << "@" << config.fps
              << " bitrate=" << config.bitrate
              << " gop=" << config.gop
              << " slice_max_bytes=" << config.slice_max_bytes
              << " rate_control=" << rate_control_name(config.rate_control)
              << " realtime_priority=" << (config.realtime_priority ? 1 : 0)
              << " input=" << encoder.input_v4l_format_name()
              << " stride=" << encoder.input_stride()
              << " scanlines=" << encoder.input_y_scanlines() << "/" << encoder.input_uv_scanlines()
              << " input_size=" << encoder.input_sizeimage()
              << " input_bytes=" << encoder.input_bytesused()
              << " uv_offset=" << encoder.input_uv_offset()
              << " capture_size=" << encoder.capture_sizeimage()
              << " device=" << config.device_path
              << std::endl;

    uint64_t frame_index = 0;
    uint64_t packet_index = 0;
    auto write_packet = [&](const ClusterH264PacketView &packet) {
      ++packet_index;
      if (config.debug && should_log_packet(packet_index)) {
        log_packet_debug(packet_index, packet);
      }
      write_all(STDOUT_FILENO, packet.data, packet.size);
    };

    while (read_exact(STDIN_FILENO, frame.data(), frame.size())) {
      const uint64_t timestamp_us = frame_index * 1000000ULL / static_cast<uint64_t>(config.fps);
      encoder.encode_rgba(frame.data(), frame.size(), timestamp_us, write_packet);
      ++frame_index;
    }

    encoder.drain(250, write_packet);
    if (config.debug) {
      std::cerr << "cluster_h264_encoder_cli summary: frames=" << frame_index
                << " packets=" << packet_index
                << " bytes_per_frame=" << frame.size()
                << std::endl;
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "cluster_h264_encoder_cli error: " << e.what() << std::endl;
    return 1;
  }
}
