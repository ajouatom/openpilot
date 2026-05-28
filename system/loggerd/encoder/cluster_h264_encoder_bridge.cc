#include "system/loggerd/encoder/cluster_h264_encoder.h"

#include <memory>
#include <stdexcept>
#include <string>

struct ClusterH264EncoderBridge {
  ClusterH264EncoderConfig config;
  std::unique_ptr<ClusterH264Encoder> encoder;
  std::string last_error;
};

namespace {

ClusterH264InputFormat input_format_from_int(int value) {
  switch (value) {
    case 0: return ClusterH264InputFormat::Auto;
    case 1: return ClusterH264InputFormat::RGB4;
    case 2: return ClusterH264InputFormat::NV12;
  }
  throw std::runtime_error("invalid input format");
}

ClusterH264Rgb4Layout rgb4_layout_from_int(int value) {
  switch (value) {
    case 0: return ClusterH264Rgb4Layout::AXRGB;
    case 1: return ClusterH264Rgb4Layout::RGBA;
    case 2: return ClusterH264Rgb4Layout::BGRA;
  }
  throw std::runtime_error("invalid RGB4 layout");
}

ClusterH264Profile h264_profile_from_int(int value) {
  switch (value) {
    case 0: return ClusterH264Profile::Baseline;
    case 1: return ClusterH264Profile::High;
  }
  throw std::runtime_error("invalid H264 profile");
}

void set_error(ClusterH264EncoderBridge *bridge, const std::exception &e) {
  if (bridge != nullptr) {
    bridge->last_error = e.what();
  }
}

}  // namespace

extern "C" {

typedef void (*cluster_h264_packet_callback)(
    const uint8_t *data,
    size_t size,
    uint32_t flags,
    uint64_t timestamp_us,
    int codec_config,
    int keyframe,
    void *opaque);

ClusterH264EncoderBridge *cluster_h264_encoder_bridge_create(
    int width,
    int height,
    int fps,
    int bitrate,
    int gop,
    const char *device_path,
    int input_format,
    int rgb4_layout,
    int debug) {
  ClusterH264EncoderBridge *bridge = new ClusterH264EncoderBridge();
  try {
    bridge->config.width = width;
    bridge->config.height = height;
    bridge->config.fps = fps;
    bridge->config.bitrate = bitrate;
    bridge->config.gop = gop;
    bridge->config.debug = debug != 0;
    bridge->config.device_path = device_path == nullptr ? "" : device_path;
    bridge->config.input_format = input_format_from_int(input_format);
    bridge->config.rgb4_layout = rgb4_layout_from_int(rgb4_layout);
    return bridge;
  } catch (const std::exception &e) {
    set_error(bridge, e);
    return bridge;
  }
}

int cluster_h264_encoder_bridge_set_slice_max_bytes(ClusterH264EncoderBridge *bridge, int slice_max_bytes) {
  if (bridge == nullptr) return -1;
  if (bridge->encoder != nullptr) {
    bridge->last_error = "cannot change slice max bytes after encoder open";
    return -1;
  }
  if (slice_max_bytes < 0) {
    bridge->last_error = "slice max bytes must be 0 or greater";
    return -1;
  }
  bridge->config.slice_max_bytes = slice_max_bytes;
  bridge->last_error.clear();
  return 0;
}

int cluster_h264_encoder_bridge_set_slice_max_mb(ClusterH264EncoderBridge *bridge, int slice_max_mb) {
  if (bridge == nullptr) return -1;
  if (bridge->encoder != nullptr) {
    bridge->last_error = "cannot change slice max MB after encoder open";
    return -1;
  }
  if (slice_max_mb != 0) {
    bridge->last_error = "slice max MB is disabled because it can stall the Qualcomm V4L2 encoder";
    return -1;
  }
  bridge->config.slice_max_mb = slice_max_mb;
  bridge->last_error.clear();
  return 0;
}

int cluster_h264_encoder_bridge_set_qp(ClusterH264EncoderBridge *bridge, int qp) {
  if (bridge == nullptr) return -1;
  if (bridge->encoder != nullptr) {
    bridge->last_error = "cannot change qp after encoder open";
    return -1;
  }
  if (qp < -1 || qp > 51) {
    bridge->last_error = "qp must be -1 or between 0 and 51";
    return -1;
  }
  bridge->config.qp = qp;
  bridge->last_error.clear();
  return 0;
}

int cluster_h264_encoder_bridge_set_h264_profile(ClusterH264EncoderBridge *bridge, int profile) {
  if (bridge == nullptr) return -1;
  if (bridge->encoder != nullptr) {
    bridge->last_error = "cannot change H264 profile after encoder open";
    return -1;
  }
  try {
    bridge->config.h264_profile = h264_profile_from_int(profile);
    bridge->last_error.clear();
    return 0;
  } catch (const std::exception &e) {
    set_error(bridge, e);
    return -1;
  }
}

int cluster_h264_encoder_bridge_open(ClusterH264EncoderBridge *bridge) {
  if (bridge == nullptr) return -1;
  try {
    bridge->encoder = std::make_unique<ClusterH264Encoder>(bridge->config);
    bridge->encoder->open();
    bridge->last_error.clear();
    return 0;
  } catch (const std::exception &e) {
    set_error(bridge, e);
    bridge->encoder.reset();
    return -1;
  }
}

int cluster_h264_encoder_bridge_encode_rgba(
    ClusterH264EncoderBridge *bridge,
    const uint8_t *rgba,
    size_t rgba_size,
    uint64_t timestamp_us,
    cluster_h264_packet_callback callback,
    void *opaque) {
  if (bridge == nullptr || bridge->encoder == nullptr) return -1;
  try {
    bridge->encoder->encode_rgba(rgba, rgba_size, timestamp_us, [callback, opaque](const ClusterH264PacketView &packet) {
      if (callback != nullptr) {
        callback(
            packet.data,
            packet.size,
            packet.flags,
            packet.timestamp_us,
            packet.codec_config ? 1 : 0,
            packet.keyframe ? 1 : 0,
            opaque);
      }
    });
    bridge->last_error.clear();
    return 0;
  } catch (const std::exception &e) {
    set_error(bridge, e);
    return -1;
  }
}

int cluster_h264_encoder_bridge_drain(
    ClusterH264EncoderBridge *bridge,
    int timeout_ms,
    cluster_h264_packet_callback callback,
    void *opaque) {
  if (bridge == nullptr || bridge->encoder == nullptr) return -1;
  try {
    bridge->encoder->drain(timeout_ms, [callback, opaque](const ClusterH264PacketView &packet) {
      if (callback != nullptr) {
        callback(
            packet.data,
            packet.size,
            packet.flags,
            packet.timestamp_us,
            packet.codec_config ? 1 : 0,
            packet.keyframe ? 1 : 0,
            opaque);
      }
    });
    bridge->last_error.clear();
    return 0;
  } catch (const std::exception &e) {
    set_error(bridge, e);
    return -1;
  }
}

void cluster_h264_encoder_bridge_close(ClusterH264EncoderBridge *bridge) {
  if (bridge != nullptr && bridge->encoder != nullptr) {
    bridge->encoder->close();
    bridge->encoder.reset();
  }
}

void cluster_h264_encoder_bridge_destroy(ClusterH264EncoderBridge *bridge) {
  if (bridge != nullptr) {
    cluster_h264_encoder_bridge_close(bridge);
    delete bridge;
  }
}

const char *cluster_h264_encoder_bridge_last_error(ClusterH264EncoderBridge *bridge) {
  if (bridge == nullptr) return "null cluster H264 bridge";
  return bridge->last_error.c_str();
}

const char *cluster_h264_encoder_bridge_input_format_name(ClusterH264EncoderBridge *bridge) {
  if (bridge == nullptr || bridge->encoder == nullptr) return "";
  return bridge->encoder->input_v4l_format_name().c_str();
}

size_t cluster_h264_encoder_bridge_input_stride(ClusterH264EncoderBridge *bridge) {
  if (bridge == nullptr || bridge->encoder == nullptr) return 0;
  return bridge->encoder->input_stride();
}

}  // extern "C"
