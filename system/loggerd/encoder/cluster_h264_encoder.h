#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "msgq/visionipc/visionbuf.h"

constexpr int CLUSTER_H264_INPUT_BUFFER_COUNT = 7;
constexpr int CLUSTER_H264_CAPTURE_BUFFER_COUNT = 6;

enum class ClusterH264InputFormat {
  Auto,
  RGB4,
  NV12,
};

enum class ClusterH264Rgb4Layout {
  AXRGB,
  RGBA,
  BGRA,
};

struct ClusterH264EncoderConfig {
  int width = 0;
  int height = 0;
  int fps = 30;
  int bitrate = 1000000;
  int gop = 30;
  int slice_max_bytes = 4096;
  int rate_control = 2;
  bool realtime_priority = false;
  bool debug = false;
  ClusterH264InputFormat input_format = ClusterH264InputFormat::Auto;
  ClusterH264Rgb4Layout rgb4_layout = ClusterH264Rgb4Layout::BGRA;
  bool rgb4_use_source_alpha = false;
  uint8_t rgb4_alpha = 0xff;
  std::string device_path = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1";
};

struct ClusterH264Packet {
  std::vector<uint8_t> data;
  uint32_t flags = 0;
  uint64_t timestamp_us = 0;
  bool codec_config = false;
  bool keyframe = false;
};

struct ClusterH264PacketView {
  const uint8_t *data = nullptr;
  size_t size = 0;
  uint32_t flags = 0;
  uint64_t timestamp_us = 0;
  bool codec_config = false;
  bool keyframe = false;
};

using ClusterH264PacketCallback = std::function<void(const ClusterH264PacketView&)>;

struct ClusterH264EncodeTimings {
  uint64_t pre_poll_us = 0;
  uint64_t wait_input_us = 0;
  uint64_t convert_us = 0;
  uint64_t sync_us = 0;
  uint64_t queue_us = 0;
  uint64_t post_poll_us = 0;
  uint64_t total_us = 0;
};

class ClusterH264Encoder {
public:
  explicit ClusterH264Encoder(const ClusterH264EncoderConfig &config);
  ~ClusterH264Encoder();

  ClusterH264Encoder(const ClusterH264Encoder&) = delete;
  ClusterH264Encoder& operator=(const ClusterH264Encoder&) = delete;

  void open();
  void close();
  bool is_open() const { return is_open_; }

  std::vector<ClusterH264Packet> encode_rgba(const uint8_t *rgba, size_t rgba_size, uint64_t timestamp_us);
  void encode_rgba(const uint8_t *rgba, size_t rgba_size, uint64_t timestamp_us, const ClusterH264PacketCallback &on_packet);
  std::vector<ClusterH264Packet> drain(int timeout_ms = 0);
  void drain(int timeout_ms, const ClusterH264PacketCallback &on_packet);

  uint32_t input_v4l_format() const { return input_v4l_format_; }
  const std::string& input_v4l_format_name() const { return input_v4l_format_name_; }
  size_t input_sizeimage() const { return input_sizeimage_; }
  size_t input_stride() const { return input_stride_; }
  size_t input_y_scanlines() const { return input_y_scanlines_; }
  size_t input_uv_scanlines() const { return input_uv_scanlines_; }
  size_t input_uv_offset() const { return input_uv_offset_; }
  size_t input_bytesused() const { return input_bytesused_; }
  size_t capture_sizeimage() const { return capture_sizeimage_; }
  const ClusterH264EncodeTimings& last_encode_timings() const { return last_encode_timings_; }
  bool input_is_rgb4() const;
  bool input_is_nv12() const;

private:
  struct DequeueResult {
    unsigned int index = 0;
    unsigned int bytesused = 0;
    unsigned int flags = 0;
    uint64_t timestamp_us = 0;
  };

  void query_capability();
  std::vector<uint32_t> enumerate_formats(uint32_t buffer_type) const;
  void configure_formats();
  void set_fps();
  void set_controls();
  void request_buffers(uint32_t buffer_type, unsigned int count);
  void stream_on(uint32_t buffer_type);
  void stream_off(uint32_t buffer_type);
  void allocate_buffers();
  void queue_capture_buffer(unsigned int index);
  void queue_output_buffer(unsigned int index, uint64_t timestamp_us);
  bool dequeue_buffer(uint32_t buffer_type, DequeueResult *result);
  std::vector<ClusterH264Packet> process_ready_events(int timeout_ms, bool stop_after_first_event);
  size_t process_ready_events(int timeout_ms, bool stop_after_first_event, const ClusterH264PacketCallback &on_packet);
  void copy_rgba_to_input(const uint8_t *rgba, size_t rgba_size, VisionBuf *dst) const;
  void rgba_to_rgb4(const uint8_t *rgba, size_t rgba_size, VisionBuf *dst) const;
  void rgba_to_nv12(const uint8_t *rgba, size_t rgba_size, VisionBuf *dst) const;
  void validate_config() const;

  ClusterH264EncoderConfig config_;
  int fd_ = -1;
  bool is_open_ = false;
  bool streams_on_ = false;

  uint32_t input_v4l_format_ = 0;
  std::string input_v4l_format_name_ = "unknown";
  size_t input_sizeimage_ = 0;
  size_t input_stride_ = 0;
  size_t input_y_scanlines_ = 0;
  size_t input_uv_scanlines_ = 0;
  size_t input_uv_offset_ = 0;
  size_t input_bytesused_ = 0;
  size_t capture_sizeimage_ = 0;

  VisionBuf input_buffers_[CLUSTER_H264_INPUT_BUFFER_COUNT];
  VisionBuf capture_buffers_[CLUSTER_H264_CAPTURE_BUFFER_COUNT];
  bool input_allocated_[CLUSTER_H264_INPUT_BUFFER_COUNT] = {};
  bool capture_allocated_[CLUSTER_H264_CAPTURE_BUFFER_COUNT] = {};
  std::deque<unsigned int> free_inputs_;
  std::vector<uint8_t> codec_config_;
  bool sent_video_packet_ = false;
  ClusterH264EncodeTimings last_encode_timings_;
};
