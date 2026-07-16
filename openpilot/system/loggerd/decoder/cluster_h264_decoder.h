#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include "msgq/visionipc/visionbuf.h"

constexpr unsigned int CLUSTER_H264_DECODER_INPUT_BUFFER_COUNT = 8;
constexpr unsigned int CLUSTER_H264_DECODER_DEFAULT_CAPTURE_BUFFER_COUNT = 10;
constexpr unsigned int CLUSTER_H264_DECODER_MAX_CAPTURE_BUFFER_COUNT = 32;

struct ClusterH264DecoderConfig {
  int width = 0;
  int height = 0;
  int fps = 30;
  std::string device_path;
  bool debug = false;
};

struct ClusterH264DecodedFrame {
  unsigned int index = 0;
  int fd = -1;
  size_t width = 0;
  size_t height = 0;
  size_t stride = 0;
  size_t uv_offset = 0;
  uint64_t sequence = 0;
};

class ClusterH264Decoder {
public:
  explicit ClusterH264Decoder(const ClusterH264DecoderConfig &config);
  ~ClusterH264Decoder();

  void open();
  void close();
  bool decode(const uint8_t *data, size_t size, uint64_t sequence, int timeout_ms, ClusterH264DecodedFrame *frame);
  void release(unsigned int index);

private:
  enum class CaptureState : uint8_t {
    Unallocated,
    Queued,
    Leased,
  };

  struct ReadyFrame {
    unsigned int index = 0;
    uint64_t sequence = 0;
  };

  void validate_config() const;
  void query_capability();
  void subscribe_events();
  void configure_output();
  void set_fps();
  void set_dpb_controls();
  void configure_capture();
  unsigned int required_capture_buffer_count();
  void restart_capture();
  void request_buffers(uint32_t type, unsigned int count, unsigned int *actual_count = nullptr);
  void stream_on(uint32_t type);
  void stream_off(uint32_t type);
  void allocate_output_buffers();
  void free_output_buffers();
  void allocate_capture_buffers();
  void free_capture_buffers();
  void queue_output(unsigned int index, const uint8_t *data, size_t size, uint64_t sequence);
  void queue_capture_locked(unsigned int index);
  bool dequeue_output();
  bool dequeue_capture();
  bool dequeue_event();
  bool process_ready_events(int timeout_ms);
  int free_output_index() const;
  bool has_leased_capture_locked() const;
  ClusterH264DecodedFrame take_ready_frame();

  ClusterH264DecoderConfig config_;
  int fd_ = -1;
  bool is_open_ = false;
  bool output_stream_on_ = false;
  bool capture_stream_on_ = false;
  bool reconfigure_pending_ = false;

  size_t output_sizeimage_ = 0;
  unsigned int output_buffer_count_ = 0;
  std::array<VisionBuf, CLUSTER_H264_DECODER_INPUT_BUFFER_COUNT> output_buffers_;
  std::array<bool, CLUSTER_H264_DECODER_INPUT_BUFFER_COUNT> output_allocated_ = {};
  std::array<bool, CLUSTER_H264_DECODER_INPUT_BUFFER_COUNT> output_queued_ = {};

  size_t capture_width_ = 0;
  size_t capture_height_ = 0;
  size_t capture_format_width_ = 0;
  size_t capture_format_height_ = 0;
  size_t capture_stride_ = 0;
  size_t capture_uv_offset_ = 0;
  size_t capture_sizeimage_ = 0;
  unsigned int capture_buffer_count_ = 0;
  std::array<VisionBuf, CLUSTER_H264_DECODER_MAX_CAPTURE_BUFFER_COUNT> capture_buffers_;
  std::array<bool, CLUSTER_H264_DECODER_MAX_CAPTURE_BUFFER_COUNT> capture_allocated_ = {};
  std::array<CaptureState, CLUSTER_H264_DECODER_MAX_CAPTURE_BUFFER_COUNT> capture_states_ = {};
  std::mutex capture_mutex_;

  std::deque<uint64_t> submitted_sequences_;
  std::deque<ReadyFrame> ready_frames_;
};
