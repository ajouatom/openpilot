#include "system/loggerd/encoder/cluster_h264_encoder.h"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
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
      << "  --bitrate BPS           Target bitrate in bits/s; K/M suffixes are accepted. Default 6000000.\n"
      << "  --gop N                 Keyframe interval in frames. Default 30.\n"
      << "  --slice-max-bytes N     V4L2 multi-slice max bytes; 0 disables. Default 4096.\n"
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
              << " input=" << encoder.input_v4l_format_name()
              << " stride=" << encoder.input_stride()
              << " device=" << config.device_path
              << std::endl;

    uint64_t frame_index = 0;
    while (read_exact(STDIN_FILENO, frame.data(), frame.size())) {
      const uint64_t timestamp_us = frame_index * 1000000ULL / static_cast<uint64_t>(config.fps);
      encoder.encode_rgba(frame.data(), frame.size(), timestamp_us, [](const ClusterH264PacketView &packet) {
        write_all(STDOUT_FILENO, packet.data, packet.size);
      });
      ++frame_index;
    }

    encoder.drain(250, [](const ClusterH264PacketView &packet) {
      write_all(STDOUT_FILENO, packet.data, packet.size);
    });
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "cluster_h264_encoder_cli error: " << e.what() << std::endl;
    return 1;
  }
}
