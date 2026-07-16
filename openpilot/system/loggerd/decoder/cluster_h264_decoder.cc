#include "system/loggerd/decoder/cluster_h264_decoder.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include "common/swaglog.h"
#include "common/util.h"
#include "system/camerad/cameras/nv12_info.h"
#include "third_party/linux/include/v4l2-controls.h"
#include <linux/videodev2.h>

namespace {

void xioctl(int fd, unsigned long request, void *arg, const char *message) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && errno == EINTR);
  if (ret == -1) {
    throw std::runtime_error(util::string_format("%s: %s (%d)", message, strerror(errno), errno));
  }
}

bool try_dequeue(int fd, unsigned long request, void *arg, const char *message) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && errno == EINTR);
  if (ret == 0) {
    return true;
  }
  if (errno == EAGAIN || errno == ENOENT) {
    return false;
  }
  throw std::runtime_error(util::string_format("%s: %s (%d)", message, strerror(errno), errno));
}

void optional_ioctl(int fd, unsigned long request, void *arg) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && errno == EINTR);
}

int remaining_timeout_ms(const std::chrono::steady_clock::time_point &deadline) {
  const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
  return static_cast<int>(std::max<int64_t>(0, remaining));
}

}  // namespace

ClusterH264Decoder::ClusterH264Decoder(const ClusterH264DecoderConfig &config) : config_(config) {
  validate_config();
  capture_format_width_ = static_cast<size_t>(config_.width);
  capture_format_height_ = static_cast<size_t>(config_.height);
}

ClusterH264Decoder::~ClusterH264Decoder() {
  close();
}

void ClusterH264Decoder::validate_config() const {
  if (config_.width <= 0 || config_.height <= 0 || (config_.width & 1) != 0 || (config_.height & 1) != 0) {
    throw std::runtime_error("cluster H264 decoder requires positive even dimensions");
  }
  if (config_.fps <= 0) {
    throw std::runtime_error("cluster H264 decoder fps must be positive");
  }
  if (config_.device_path.empty()) {
    throw std::runtime_error("cluster H264 decoder device path must not be empty");
  }
}

void ClusterH264Decoder::open() {
  if (is_open_) {
    return;
  }
  fd_ = HANDLE_EINTR(::open(config_.device_path.c_str(), O_RDWR | O_NONBLOCK));
  if (fd_ < 0) {
    throw std::runtime_error(util::string_format(
        "failed to open V4L2 decoder %s: %s", config_.device_path.c_str(), strerror(errno)));
  }

  try {
    query_capability();
    subscribe_events();
    configure_output();
    set_fps();
    set_dpb_controls();
    request_buffers(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, CLUSTER_H264_DECODER_INPUT_BUFFER_COUNT, &output_buffer_count_);
    allocate_output_buffers();
    stream_on(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    output_stream_on_ = true;

    {
      std::lock_guard lock(capture_mutex_);
      configure_capture();
      request_buffers(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, required_capture_buffer_count(), &capture_buffer_count_);
      allocate_capture_buffers();
      stream_on(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
      capture_stream_on_ = true;
      for (unsigned int i = 0; i < capture_buffer_count_; ++i) {
        queue_capture_locked(i);
      }
    }
    is_open_ = true;
    if (config_.debug) {
      LOGD("cluster H264 decoder ready: %dx%d fps=%d input_size=%zu capture=%zux%zu stride=%zu uv_offset=%zu size=%zu buffers=%u",
           config_.width, config_.height, config_.fps, output_sizeimage_, capture_width_, capture_height_,
           capture_stride_, capture_uv_offset_, capture_sizeimage_, capture_buffer_count_);
    }
  } catch (...) {
    close();
    throw;
  }
}

void ClusterH264Decoder::close() {
  if (fd_ >= 0 && capture_stream_on_) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    optional_ioctl(fd_, VIDIOC_STREAMOFF, &type);
    capture_stream_on_ = false;
  }
  if (fd_ >= 0 && output_stream_on_) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    optional_ioctl(fd_, VIDIOC_STREAMOFF, &type);
    output_stream_on_ = false;
  }
  if (fd_ >= 0) {
    struct v4l2_requestbuffers reqbuf = {};
    reqbuf.memory = V4L2_MEMORY_USERPTR;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    optional_ioctl(fd_, VIDIOC_REQBUFS, &reqbuf);
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    optional_ioctl(fd_, VIDIOC_REQBUFS, &reqbuf);
    ::close(fd_);
    fd_ = -1;
  }

  {
    std::lock_guard lock(capture_mutex_);
    free_capture_buffers();
  }
  free_output_buffers();
  output_buffer_count_ = 0;
  capture_buffer_count_ = 0;
  submitted_sequences_.clear();
  ready_frames_.clear();
  reconfigure_pending_ = false;
  is_open_ = false;
}

void ClusterH264Decoder::query_capability() {
  struct v4l2_capability cap = {};
  xioctl(fd_, VIDIOC_QUERYCAP, &cap, "VIDIOC_QUERYCAP decoder failed");
  if (strcmp(reinterpret_cast<const char *>(cap.driver), "msm_vidc_driver") != 0 ||
      strcmp(reinterpret_cast<const char *>(cap.card), "msm_vidc_vdec") != 0) {
    LOGW("cluster H264 decoder is %s/%s, expected msm_vidc_driver/msm_vidc_vdec", cap.driver, cap.card);
  }
}

void ClusterH264Decoder::subscribe_events() {
  for (uint32_t event_type : {V4L2_EVENT_MSM_VIDC_FLUSH_DONE, V4L2_EVENT_MSM_VIDC_PORT_SETTINGS_CHANGED_INSUFFICIENT}) {
    struct v4l2_event_subscription subscription = {};
    subscription.type = event_type;
    xioctl(fd_, VIDIOC_SUBSCRIBE_EVENT, &subscription, "VIDIOC_SUBSCRIBE_EVENT decoder failed");
  }
}

void ClusterH264Decoder::configure_output() {
  struct v4l2_format format = {};
  format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  format.fmt.pix_mp.width = static_cast<uint32_t>(config_.width);
  format.fmt.pix_mp.height = static_cast<uint32_t>(config_.height);
  format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
  xioctl(fd_, VIDIOC_S_FMT, &format, "VIDIOC_S_FMT H264 decoder input failed");
  if (format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_H264) {
    throw std::runtime_error("V4L2 decoder rejected H264 input format");
  }
  output_sizeimage_ = format.fmt.pix_mp.plane_fmt[0].sizeimage;
  if (output_sizeimage_ == 0) {
    throw std::runtime_error("V4L2 decoder returned zero H264 input sizeimage");
  }
}

void ClusterH264Decoder::set_fps() {
  struct v4l2_streamparm streamparm = {};
  streamparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  streamparm.parm.output.timeperframe.numerator = 1;
  streamparm.parm.output.timeperframe.denominator = static_cast<unsigned int>(config_.fps);
  xioctl(fd_, VIDIOC_S_PARM, &streamparm, "VIDIOC_S_PARM decoder failed");
}

void ClusterH264Decoder::set_dpb_controls() {
  struct v4l2_ext_control controls_array[2] = {};
  controls_array[0].id = V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_MODE;
  controls_array[0].value = 1;
  controls_array[1].id = V4L2_CID_MPEG_VIDC_VIDEO_DPB_COLOR_FORMAT;
  controls_array[1].value = 0;

  struct v4l2_ext_controls controls = {};
  controls.count = 2;
  controls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
  controls.controls = controls_array;
  xioctl(fd_, VIDIOC_S_EXT_CTRLS, &controls, "VIDIOC_S_EXT_CTRLS decoder DPB failed");
}

void ClusterH264Decoder::configure_capture() {
  struct v4l2_format format = {};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  format.fmt.pix_mp.width = static_cast<uint32_t>(capture_format_width_);
  format.fmt.pix_mp.height = static_cast<uint32_t>(capture_format_height_);
  format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
  xioctl(fd_, VIDIOC_S_FMT, &format, "VIDIOC_S_FMT NV12 decoder capture failed");
  xioctl(fd_, VIDIOC_G_FMT, &format, "VIDIOC_G_FMT NV12 decoder capture failed");
  if (format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12) {
    throw std::runtime_error("V4L2 decoder rejected linear NV12 capture format");
  }

  const auto [venus_stride, venus_y_scanlines, venus_uv_scanlines, venus_size] =
      get_nv12_info(capture_format_width_, capture_format_height_);
  capture_width_ = static_cast<size_t>(config_.width);
  capture_height_ = static_cast<size_t>(config_.height);
  capture_stride_ = std::max<size_t>(format.fmt.pix_mp.plane_fmt[0].bytesperline, venus_stride);
  capture_uv_offset_ = capture_stride_ * venus_y_scanlines;
  const size_t active_size = capture_uv_offset_ + capture_stride_ * venus_uv_scanlines;
  capture_sizeimage_ = std::max<size_t>({
      format.fmt.pix_mp.plane_fmt[0].sizeimage,
      active_size,
      venus_size,
  });
}

unsigned int ClusterH264Decoder::required_capture_buffer_count() {
  struct v4l2_control control = {
    .id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
  };
  if (ioctl(fd_, VIDIOC_G_CTRL, &control) == 0 && control.value > 0) {
    return std::clamp(
        static_cast<unsigned int>(control.value),
        CLUSTER_H264_DECODER_DEFAULT_CAPTURE_BUFFER_COUNT,
        CLUSTER_H264_DECODER_MAX_CAPTURE_BUFFER_COUNT);
  }
  return CLUSTER_H264_DECODER_DEFAULT_CAPTURE_BUFFER_COUNT;
}

void ClusterH264Decoder::request_buffers(uint32_t type, unsigned int count, unsigned int *actual_count) {
  struct v4l2_requestbuffers reqbuf = {};
  reqbuf.count = count;
  reqbuf.type = type;
  reqbuf.memory = V4L2_MEMORY_USERPTR;
  xioctl(fd_, VIDIOC_REQBUFS, &reqbuf, "VIDIOC_REQBUFS decoder failed");
  if (count > 0 && reqbuf.count == 0) {
    throw std::runtime_error("V4L2 decoder allocated zero buffers");
  }
  if (actual_count != nullptr) {
    *actual_count = std::min(count, reqbuf.count);
  }
}

void ClusterH264Decoder::stream_on(uint32_t type_value) {
  enum v4l2_buf_type type = static_cast<enum v4l2_buf_type>(type_value);
  xioctl(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON decoder failed");
}

void ClusterH264Decoder::stream_off(uint32_t type_value) {
  enum v4l2_buf_type type = static_cast<enum v4l2_buf_type>(type_value);
  optional_ioctl(fd_, VIDIOC_STREAMOFF, &type);
}

void ClusterH264Decoder::allocate_output_buffers() {
  for (unsigned int i = 0; i < output_buffer_count_; ++i) {
    output_buffers_[i].allocate(output_sizeimage_);
    output_allocated_[i] = true;
    output_queued_[i] = false;
  }
}

void ClusterH264Decoder::free_output_buffers() {
  for (unsigned int i = 0; i < output_buffers_.size(); ++i) {
    output_queued_[i] = false;
    if (output_allocated_[i]) {
      output_buffers_[i].free();
      output_allocated_[i] = false;
    }
  }
}

void ClusterH264Decoder::allocate_capture_buffers() {
  for (unsigned int i = 0; i < capture_buffer_count_; ++i) {
    capture_buffers_[i].allocate(capture_sizeimage_);
    capture_buffers_[i].init_yuv(capture_width_, capture_height_, capture_stride_, capture_uv_offset_);
    capture_allocated_[i] = true;
    capture_states_[i] = CaptureState::Unallocated;
  }
}

void ClusterH264Decoder::free_capture_buffers() {
  for (unsigned int i = 0; i < capture_buffers_.size(); ++i) {
    if (capture_allocated_[i]) {
      capture_buffers_[i].free();
      capture_allocated_[i] = false;
    }
    capture_states_[i] = CaptureState::Unallocated;
  }
}

void ClusterH264Decoder::queue_output(
    unsigned int index, const uint8_t *data, size_t size, uint64_t sequence) {
  if (index >= output_buffer_count_ || !output_allocated_[index] || output_queued_[index]) {
    throw std::runtime_error("invalid V4L2 decoder input buffer state");
  }
  if (data == nullptr || size == 0 || size > output_buffers_[index].len) {
    throw std::runtime_error(util::string_format(
        "H264 access unit is %zu bytes, decoder input capacity is %zu", size, output_buffers_[index].len));
  }
  memcpy(output_buffers_[index].addr, data, size);
  output_buffers_[index].sync(VISIONBUF_SYNC_TO_DEVICE);

  struct timeval timestamp = {
    .tv_sec = static_cast<long>(sequence / 1000000ULL),
    .tv_usec = static_cast<long>(sequence % 1000000ULL),
  };
  struct v4l2_plane plane = {};
  plane.bytesused = static_cast<uint32_t>(size);
  plane.length = static_cast<uint32_t>(output_buffers_[index].len);
  plane.m.userptr = reinterpret_cast<unsigned long>(output_buffers_[index].addr);
  plane.reserved[0] = static_cast<unsigned int>(output_buffers_[index].fd);

  struct v4l2_buffer buffer = {};
  buffer.index = index;
  buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  buffer.flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
  buffer.timestamp = timestamp;
  buffer.memory = V4L2_MEMORY_USERPTR;
  buffer.m.planes = &plane;
  buffer.length = 1;
  xioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF H264 decoder input failed");
  output_queued_[index] = true;
  submitted_sequences_.push_back(sequence);
}

void ClusterH264Decoder::queue_capture_locked(unsigned int index) {
  if (index >= capture_buffer_count_ || !capture_allocated_[index] || capture_states_[index] == CaptureState::Queued) {
    throw std::runtime_error("invalid V4L2 decoder capture buffer state");
  }
  VisionBuf &capture = capture_buffers_[index];
  struct v4l2_plane plane = {};
  plane.bytesused = static_cast<uint32_t>(capture.len);
  plane.length = static_cast<uint32_t>(capture.len);
  plane.m.userptr = reinterpret_cast<unsigned long>(capture.addr);
  plane.reserved[0] = static_cast<unsigned int>(capture.fd);

  struct v4l2_buffer buffer = {};
  buffer.index = index;
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buffer.memory = V4L2_MEMORY_USERPTR;
  buffer.m.planes = &plane;
  buffer.length = 1;
  xioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF H264 decoder capture failed");
  capture_states_[index] = CaptureState::Queued;
}

bool ClusterH264Decoder::dequeue_output() {
  struct v4l2_plane plane = {};
  struct v4l2_buffer buffer = {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  buffer.memory = V4L2_MEMORY_USERPTR;
  buffer.m.planes = &plane;
  buffer.length = 1;
  if (!try_dequeue(fd_, VIDIOC_DQBUF, &buffer, "VIDIOC_DQBUF H264 decoder input failed")) {
    return false;
  }
  if (buffer.index >= output_buffer_count_) {
    throw std::runtime_error("V4L2 decoder returned invalid input buffer index");
  }
  output_queued_[buffer.index] = false;
  return true;
}

bool ClusterH264Decoder::dequeue_capture() {
  struct v4l2_plane plane = {};
  struct v4l2_buffer buffer = {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buffer.memory = V4L2_MEMORY_USERPTR;
  buffer.m.planes = &plane;
  buffer.length = 1;
  if (!try_dequeue(fd_, VIDIOC_DQBUF, &buffer, "VIDIOC_DQBUF H264 decoder capture failed")) {
    return false;
  }
  if (buffer.index >= capture_buffer_count_) {
    throw std::runtime_error("V4L2 decoder returned invalid capture buffer index");
  }
  if (reconfigure_pending_ || plane.bytesused == 0) {
    return true;
  }

  uint64_t sequence = static_cast<uint64_t>(buffer.timestamp.tv_sec) * 1000000ULL +
                      static_cast<uint64_t>(buffer.timestamp.tv_usec);
  if (!submitted_sequences_.empty()) {
    if (sequence == 0) {
      sequence = submitted_sequences_.front();
    }
    submitted_sequences_.pop_front();
  }
  capture_buffers_[buffer.index].sync(VISIONBUF_SYNC_FROM_DEVICE);
  {
    std::lock_guard lock(capture_mutex_);
    if (capture_states_[buffer.index] != CaptureState::Queued) {
      throw std::runtime_error("V4L2 decoder capture buffer was not queued");
    }
    capture_states_[buffer.index] = CaptureState::Leased;
  }
  ready_frames_.push_back({buffer.index, sequence});
  return true;
}

bool ClusterH264Decoder::dequeue_event() {
  struct v4l2_event event = {};
  if (!try_dequeue(fd_, VIDIOC_DQEVENT, &event, "VIDIOC_DQEVENT H264 decoder failed")) {
    return false;
  }
  if (event.type == V4L2_EVENT_MSM_VIDC_PORT_SETTINGS_CHANGED_INSUFFICIENT) {
    const auto *event_data = reinterpret_cast<const unsigned int *>(event.u.data);
    capture_format_height_ = std::max<size_t>(1, event_data[0]);
    capture_format_width_ = std::max<size_t>(1, event_data[1]);
    if (config_.debug) {
      LOGD("cluster H264 decoder port settings changed: coded=%zux%zu", capture_format_width_, capture_format_height_);
    }
    {
      std::lock_guard lock(capture_mutex_);
      if (has_leased_capture_locked()) {
        throw std::runtime_error("H264 decoder format changed while capture buffers were leased");
      }
    }
    struct v4l2_decoder_cmd command = {};
    command.flags = V4L2_QCOM_CMD_FLUSH_CAPTURE;
    command.cmd = V4L2_QCOM_CMD_FLUSH;
    xioctl(fd_, VIDIOC_DECODER_CMD, &command, "VIDIOC_DECODER_CMD capture flush failed");
    reconfigure_pending_ = true;
  } else if (event.type == V4L2_EVENT_MSM_VIDC_FLUSH_DONE) {
    const auto *event_data = reinterpret_cast<const unsigned int *>(event.u.data);
    if (reconfigure_pending_ && (event_data[0] & V4L2_QCOM_CMD_FLUSH_CAPTURE) != 0) {
      restart_capture();
      reconfigure_pending_ = false;
    }
  }
  return true;
}

void ClusterH264Decoder::restart_capture() {
  std::lock_guard lock(capture_mutex_);
  if (has_leased_capture_locked()) {
    throw std::runtime_error("cannot restart H264 decoder with leased capture buffers");
  }
  ready_frames_.clear();
  if (capture_stream_on_) {
    stream_off(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    capture_stream_on_ = false;
  }
  request_buffers(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 0);
  free_capture_buffers();
  set_dpb_controls();
  configure_capture();
  request_buffers(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, required_capture_buffer_count(), &capture_buffer_count_);
  allocate_capture_buffers();
  stream_on(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
  capture_stream_on_ = true;
  for (unsigned int i = 0; i < capture_buffer_count_; ++i) {
    queue_capture_locked(i);
  }
  if (config_.debug) {
    LOGD("cluster H264 decoder capture reconfigured: %zux%zu stride=%zu uv_offset=%zu size=%zu buffers=%u",
         capture_width_, capture_height_, capture_stride_, capture_uv_offset_, capture_sizeimage_, capture_buffer_count_);
  }
}

bool ClusterH264Decoder::process_ready_events(int timeout_ms) {
  struct pollfd pfd = {
    .fd = fd_,
    .events = POLLIN | POLLOUT | POLLPRI | POLLERR,
    .revents = 0,
  };
  int ret;
  do {
    ret = poll(&pfd, 1, timeout_ms);
  } while (ret < 0 && errno == EINTR);
  if (ret < 0) {
    throw std::runtime_error(util::string_format("cluster H264 decoder poll failed: %s (%d)", strerror(errno), errno));
  }
  if (ret == 0) {
    return false;
  }

  bool progress = false;
  if (pfd.revents & POLLPRI) {
    while (dequeue_event()) {
      progress = true;
    }
  }
  if (pfd.revents & POLLOUT) {
    while (dequeue_output()) {
      progress = true;
    }
  }
  if (pfd.revents & POLLIN) {
    while (dequeue_capture()) {
      progress = true;
    }
  }
  if ((pfd.revents & POLLERR) && !progress && (pfd.revents & POLLPRI) == 0) {
    throw std::runtime_error("cluster H264 decoder reported POLLERR");
  }
  return progress;
}

int ClusterH264Decoder::free_output_index() const {
  for (unsigned int i = 0; i < output_buffer_count_; ++i) {
    if (!output_queued_[i]) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool ClusterH264Decoder::has_leased_capture_locked() const {
  return std::any_of(capture_states_.begin(), capture_states_.end(), [](CaptureState state) {
    return state == CaptureState::Leased;
  });
}

ClusterH264DecodedFrame ClusterH264Decoder::take_ready_frame() {
  if (ready_frames_.empty()) {
    throw std::runtime_error("H264 decoder has no completed frame");
  }
  const ReadyFrame ready = ready_frames_.front();
  ready_frames_.pop_front();
  const VisionBuf &capture = capture_buffers_[ready.index];
  return {
    .index = ready.index,
    .fd = capture.fd,
    .width = capture_width_,
    .height = capture_height_,
    .stride = capture_stride_,
    .uv_offset = capture_uv_offset_,
    .sequence = ready.sequence,
  };
}

bool ClusterH264Decoder::decode(
    const uint8_t *data, size_t size, uint64_t sequence, int timeout_ms, ClusterH264DecodedFrame *frame) {
  if (!is_open_ || frame == nullptr) {
    throw std::runtime_error("cluster H264 decoder is not open");
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(0, timeout_ms));
  int input_index = free_output_index();
  while (input_index < 0) {
    if (!process_ready_events(remaining_timeout_ms(deadline))) {
      return false;
    }
    input_index = free_output_index();
  }

  queue_output(static_cast<unsigned int>(input_index), data, size, sequence);
  while (ready_frames_.empty()) {
    const int remaining = remaining_timeout_ms(deadline);
    if (remaining == 0 || !process_ready_events(remaining)) {
      return false;
    }
  }
  *frame = take_ready_frame();
  return true;
}

void ClusterH264Decoder::release(unsigned int index) {
  std::lock_guard lock(capture_mutex_);
  if (!is_open_ || index >= capture_buffer_count_) {
    return;
  }
  if (capture_states_[index] != CaptureState::Leased) {
    throw std::runtime_error("H264 decoder capture buffer was released twice");
  }
  if (reconfigure_pending_) {
    capture_states_[index] = CaptureState::Unallocated;
    return;
  }
  queue_capture_locked(index);
}
