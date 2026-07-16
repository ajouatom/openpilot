#include "system/loggerd/decoder/cluster_h264_decoder.h"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

struct ClusterH264DecoderBridge {
  ClusterH264DecoderConfig config;
  std::unique_ptr<ClusterH264Decoder> decoder;
  std::mutex error_mutex;
  std::string last_error;
};

namespace {

void clear_error(ClusterH264DecoderBridge *bridge) {
  std::lock_guard lock(bridge->error_mutex);
  bridge->last_error.clear();
}

void set_error(ClusterH264DecoderBridge *bridge, const std::exception &error) {
  if (bridge != nullptr) {
    std::lock_guard lock(bridge->error_mutex);
    bridge->last_error = error.what();
  }
}

}  // namespace

extern "C" {

ClusterH264DecoderBridge *cluster_h264_decoder_bridge_create(
    int width,
    int height,
    int fps,
    const char *device_path,
    int debug) {
  auto *bridge = new ClusterH264DecoderBridge();
  bridge->config.width = width;
  bridge->config.height = height;
  bridge->config.fps = fps;
  bridge->config.device_path = device_path == nullptr ? "" : device_path;
  bridge->config.debug = debug != 0;
  return bridge;
}

int cluster_h264_decoder_bridge_open(ClusterH264DecoderBridge *bridge) {
  if (bridge == nullptr) return -1;
  try {
    bridge->decoder = std::make_unique<ClusterH264Decoder>(bridge->config);
    bridge->decoder->open();
    clear_error(bridge);
    return 0;
  } catch (const std::exception &error) {
    bridge->decoder.reset();
    set_error(bridge, error);
    return -1;
  }
}

int cluster_h264_decoder_bridge_decode(
    ClusterH264DecoderBridge *bridge,
    const uint8_t *data,
    size_t size,
    uint64_t sequence,
    int timeout_ms,
    unsigned int *index,
    int *fd,
    size_t *width,
    size_t *height,
    size_t *stride,
    size_t *uv_offset,
    uint64_t *decoded_sequence) {
  if (bridge == nullptr || bridge->decoder == nullptr || index == nullptr || fd == nullptr ||
      width == nullptr || height == nullptr || stride == nullptr || uv_offset == nullptr || decoded_sequence == nullptr) {
    return -1;
  }
  try {
    ClusterH264DecodedFrame frame;
    if (!bridge->decoder->decode(data, size, sequence, timeout_ms, &frame)) {
      clear_error(bridge);
      return 0;
    }
    *index = frame.index;
    *fd = frame.fd;
    *width = frame.width;
    *height = frame.height;
    *stride = frame.stride;
    *uv_offset = frame.uv_offset;
    *decoded_sequence = frame.sequence;
    clear_error(bridge);
    return 1;
  } catch (const std::exception &error) {
    set_error(bridge, error);
    return -1;
  }
}

int cluster_h264_decoder_bridge_release(ClusterH264DecoderBridge *bridge, unsigned int index) {
  if (bridge == nullptr || bridge->decoder == nullptr) return -1;
  try {
    bridge->decoder->release(index);
    clear_error(bridge);
    return 0;
  } catch (const std::exception &error) {
    set_error(bridge, error);
    return -1;
  }
}

void cluster_h264_decoder_bridge_close(ClusterH264DecoderBridge *bridge) {
  if (bridge == nullptr || bridge->decoder == nullptr) return;
  bridge->decoder->close();
  bridge->decoder.reset();
}

void cluster_h264_decoder_bridge_destroy(ClusterH264DecoderBridge *bridge) {
  if (bridge == nullptr) return;
  cluster_h264_decoder_bridge_close(bridge);
  delete bridge;
}

const char *cluster_h264_decoder_bridge_last_error(ClusterH264DecoderBridge *bridge) {
  if (bridge == nullptr) return "invalid decoder bridge";
  std::lock_guard lock(bridge->error_mutex);
  return bridge->last_error.c_str();
}

}  // extern "C"
