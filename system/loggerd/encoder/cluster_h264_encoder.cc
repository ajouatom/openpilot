#include "system/loggerd/encoder/cluster_h264_encoder.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iterator>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <utility>
#include <unistd.h>

#include "common/swaglog.h"
#include "common/util.h"
#include "system/camerad/cameras/nv12_info.h"
#include "third_party/linux/include/v4l2-controls.h"
#include <linux/videodev2.h>

#define V4L2_QCOM_BUF_FLAG_CODECCONFIG 0x00020000
#define V4L2_QCOM_BUF_FLAG_EOS 0x02000000

namespace {

std::string fourcc_to_string(uint32_t value) {
  char text[5] = {
    static_cast<char>(value & 0xff),
    static_cast<char>((value >> 8) & 0xff),
    static_cast<char>((value >> 16) & 0xff),
    static_cast<char>((value >> 24) & 0xff),
    '\0',
  };
  return std::string(text);
}

uint64_t elapsed_us(std::chrono::steady_clock::time_point start) {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count());
}

const char *rate_control_name(int value) {
  switch (value) {
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF: return "rate-control-off";
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_VFR: return "rate-control-vbr-vfr";
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR: return "rate-control-vbr-cfr";
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_VFR: return "rate-control-cbr-vfr";
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR: return "rate-control-cbr-cfr";
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_MBR_CFR: return "rate-control-mbr-cfr";
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_MBR_VFR: return "rate-control-mbr-vfr";
    case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CQ: return "rate-control-cq";
  }
  return "rate-control-unknown";
}

void xioctl(int fd, unsigned long request, void *arg, const char *message) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && errno == EINTR);

  if (ret == -1) {
    throw std::runtime_error(util::string_format("%s: %s (%d)", message, strerror(errno), errno));
  }
}

void optional_ioctl(int fd, unsigned long request, void *arg, const char *message, bool debug) {
  try {
    xioctl(fd, request, arg, message);
  } catch (const std::exception &e) {
    if (debug) {
      LOGW("%s", e.what());
    }
  }
}

}  // namespace

ClusterH264Encoder::ClusterH264Encoder(const ClusterH264EncoderConfig &config) : config_(config) {
  validate_config();
}

ClusterH264Encoder::~ClusterH264Encoder() {
  close();
}

void ClusterH264Encoder::validate_config() const {
  if (config_.width <= 0 || config_.height <= 0) {
    throw std::runtime_error("cluster H264 encoder dimensions must be positive");
  }
  if ((config_.width % 2) != 0 || (config_.height % 2) != 0) {
    throw std::runtime_error("cluster H264 encoder requires even dimensions");
  }
  if (config_.fps <= 0) {
    throw std::runtime_error("cluster H264 encoder fps must be positive");
  }
  if (config_.bitrate <= 0) {
    throw std::runtime_error("cluster H264 encoder bitrate must be positive");
  }
  if (config_.gop <= 0) {
    throw std::runtime_error("cluster H264 encoder gop must be positive");
  }
  if (config_.slice_max_bytes < 0) {
    throw std::runtime_error("cluster H264 encoder slice max bytes must be 0 or greater");
  }
  if (config_.rate_control < V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF ||
      config_.rate_control > V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CQ) {
    throw std::runtime_error("cluster H264 encoder rate control is invalid");
  }
  if (config_.device_path.empty()) {
    throw std::runtime_error("cluster H264 encoder device path must not be empty");
  }
}

bool ClusterH264Encoder::input_is_nv12() const {
  return input_v4l_format_ == V4L2_PIX_FMT_NV12;
}

void ClusterH264Encoder::open() {
  if (is_open_) {
    return;
  }

  fd_ = HANDLE_EINTR(::open(config_.device_path.c_str(), O_RDWR | O_NONBLOCK));
  if (fd_ < 0) {
    throw std::runtime_error(util::string_format("failed to open V4L2 encoder %s: %s", config_.device_path.c_str(), strerror(errno)));
  }

  try {
    query_capability();
    configure_formats();
    set_fps();
    set_controls();
    request_buffers(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, CLUSTER_H264_CAPTURE_BUFFER_COUNT);
    request_buffers(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, CLUSTER_H264_INPUT_BUFFER_COUNT);
    allocate_buffers();
    stream_on(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    stream_on(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    streams_on_ = true;
    for (unsigned int i = 0; i < CLUSTER_H264_CAPTURE_BUFFER_COUNT; ++i) {
      queue_capture_buffer(i);
    }
    free_inputs_.clear();
    for (unsigned int i = 0; i < CLUSTER_H264_INPUT_BUFFER_COUNT; ++i) {
      free_inputs_.push_back(i);
    }
    codec_config_.clear();
    sent_video_packet_ = false;
    is_open_ = true;
  } catch (...) {
    close();
    throw;
  }
}

void ClusterH264Encoder::close() {
  if (fd_ >= 0 && is_open_) {
    struct v4l2_encoder_cmd encoder_cmd = {};
    encoder_cmd.cmd = V4L2_ENC_CMD_STOP;
    optional_ioctl(fd_, VIDIOC_ENCODER_CMD, &encoder_cmd, "VIDIOC_ENCODER_CMD failed", config_.debug);
    try {
      drain(250);
    } catch (...) {
    }
  }

  if (fd_ >= 0 && streams_on_) {
    stream_off(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    stream_off(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    streams_on_ = false;
  }

  if (fd_ >= 0) {
    try {
      request_buffers(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 0);
      request_buffers(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 0);
    } catch (...) {
    }
    ::close(fd_);
    fd_ = -1;
  }

  for (int i = 0; i < CLUSTER_H264_INPUT_BUFFER_COUNT; ++i) {
    if (input_allocated_[i]) {
      input_buffers_[i].free();
      input_allocated_[i] = false;
    }
  }
  for (int i = 0; i < CLUSTER_H264_CAPTURE_BUFFER_COUNT; ++i) {
    if (capture_allocated_[i]) {
      capture_buffers_[i].free();
      capture_allocated_[i] = false;
    }
  }

  free_inputs_.clear();
  codec_config_.clear();
  sent_video_packet_ = false;
  is_open_ = false;
}

void ClusterH264Encoder::query_capability() {
  struct v4l2_capability cap = {};
  xioctl(fd_, VIDIOC_QUERYCAP, &cap, "VIDIOC_QUERYCAP failed");
  if (config_.debug) {
    LOGD("cluster H264 V4L2 encoder device %s %s", cap.driver, cap.card);
  }
  if (strcmp(reinterpret_cast<const char*>(cap.driver), "msm_vidc_driver") != 0 ||
      strcmp(reinterpret_cast<const char*>(cap.card), "msm_vidc_venc") != 0) {
    LOGW("cluster H264 encoder is %s/%s, expected msm_vidc_driver/msm_vidc_venc", cap.driver, cap.card);
  }
}

std::vector<uint32_t> ClusterH264Encoder::enumerate_formats(uint32_t buffer_type) const {
  std::vector<uint32_t> formats;
  for (uint32_t index = 0; ; ++index) {
    struct v4l2_fmtdesc desc = {};
    desc.index = index;
    desc.type = buffer_type;
    int ret;
    do {
      ret = ioctl(fd_, VIDIOC_ENUM_FMT, &desc);
    } while (ret == -1 && errno == EINTR);
    if (ret == -1) {
      if (errno == EINVAL) {
        break;
      }
      throw std::runtime_error(util::string_format("VIDIOC_ENUM_FMT failed: %s (%d)", strerror(errno), errno));
    }
    formats.push_back(desc.pixelformat);
  }
  return formats;
}

void ClusterH264Encoder::configure_formats() {
  const uint32_t nv12 = V4L2_PIX_FMT_NV12;
  std::vector<uint32_t> input_formats = enumerate_formats(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
  const auto supports_format = [&input_formats](uint32_t format) {
    return input_formats.empty() || std::find(input_formats.begin(), input_formats.end(), format) != input_formats.end();
  };

  if (!supports_format(nv12)) {
    std::string found;
    for (uint32_t format : input_formats) {
      if (!found.empty()) found += ", ";
      found += fourcc_to_string(format);
    }
    throw std::runtime_error("V4L2 encoder does not report NV12 input support; found: " + found);
  }
  const uint32_t selected_input_format = nv12;

  struct v4l2_format fmt_out = {};
  fmt_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt_out.fmt.pix_mp.width = static_cast<unsigned int>(config_.width);
  fmt_out.fmt.pix_mp.height = static_cast<unsigned int>(config_.height);
  fmt_out.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
  fmt_out.fmt.pix_mp.field = V4L2_FIELD_ANY;
  fmt_out.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
  xioctl(fd_, VIDIOC_S_FMT, &fmt_out, "VIDIOC_S_FMT capture failed");

  struct v4l2_format fmt_in = {};
  fmt_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  fmt_in.fmt.pix_mp.width = static_cast<unsigned int>(config_.width);
  fmt_in.fmt.pix_mp.height = static_cast<unsigned int>(config_.height);
  fmt_in.fmt.pix_mp.pixelformat = selected_input_format;
  fmt_in.fmt.pix_mp.field = V4L2_FIELD_ANY;
  fmt_in.fmt.pix_mp.colorspace = V4L2_COLORSPACE_470_SYSTEM_BG;
  xioctl(fd_, VIDIOC_S_FMT, &fmt_in, "VIDIOC_S_FMT output failed");

  if (fmt_in.fmt.pix_mp.pixelformat != selected_input_format) {
    throw std::runtime_error("V4L2 encoder rejected " + fourcc_to_string(selected_input_format) +
                             " input, returned " + fourcc_to_string(fmt_in.fmt.pix_mp.pixelformat));
  }
  if (fmt_out.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_H264) {
    throw std::runtime_error("V4L2 encoder rejected H264 output, returned " + fourcc_to_string(fmt_out.fmt.pix_mp.pixelformat));
  }
  if (fmt_in.fmt.pix_mp.width != static_cast<unsigned int>(config_.width) ||
      fmt_in.fmt.pix_mp.height != static_cast<unsigned int>(config_.height) ||
      fmt_out.fmt.pix_mp.width != static_cast<unsigned int>(config_.width) ||
      fmt_out.fmt.pix_mp.height != static_cast<unsigned int>(config_.height)) {
    throw std::runtime_error("V4L2 encoder adjusted dimensions; cluster H264 wrapper requires exact dimensions");
  }

  input_v4l_format_ = selected_input_format;
  input_v4l_format_name_ = fourcc_to_string(selected_input_format);
  input_sizeimage_ = fmt_in.fmt.pix_mp.plane_fmt[0].sizeimage;
  const size_t driver_stride = fmt_in.fmt.pix_mp.plane_fmt[0].bytesperline;
  auto [venus_stride, venus_y_height, venus_uv_height, venus_size] = get_nv12_info(config_.width, config_.height);
  input_stride_ = venus_stride;
  input_y_scanlines_ = venus_y_height;
  input_uv_scanlines_ = venus_uv_height;
  input_uv_offset_ = input_stride_ * input_y_scanlines_;
  const size_t min_bytesused = input_uv_offset_ + input_stride_ * input_uv_scanlines_;
  input_bytesused_ = std::max({input_sizeimage_, min_bytesused, static_cast<size_t>(venus_size)});
  capture_sizeimage_ = fmt_out.fmt.pix_mp.plane_fmt[0].sizeimage;
  if (capture_sizeimage_ == 0) {
    throw std::runtime_error("V4L2 encoder returned zero H264 capture sizeimage");
  }

  LOGD("cluster H264 V4L2 formats: in=%s %dx%d driver_stride=%zu stride=%zu scanlines=%zu/%zu sizeimage=%zu bytesused=%zu uv_offset=%zu out=H264 sizeimage=%zu",
       input_v4l_format_name_.c_str(), config_.width, config_.height, driver_stride, input_stride_,
       input_y_scanlines_, input_uv_scanlines_, input_sizeimage_, input_bytesused_, input_uv_offset_,
       capture_sizeimage_);
}

void ClusterH264Encoder::set_fps() {
  struct v4l2_streamparm streamparm = {};
  streamparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  streamparm.parm.output.timeperframe.numerator = 1;
  streamparm.parm.output.timeperframe.denominator = static_cast<unsigned int>(config_.fps);
  xioctl(fd_, VIDIOC_S_PARM, &streamparm, "VIDIOC_S_PARM failed");
}

void ClusterH264Encoder::set_controls() {
  const int p_frames = std::max(0, config_.gop - 1);
  struct NamedControl {
    uint32_t id;
    int value;
    const char *name;
  };

  const auto set_control = [this](uint32_t id, int value, const char *name) {
    struct v4l2_control control = {
      .id = id,
      .value = value,
    };
    const std::string message = util::string_format("VIDIOC_S_CTRL %s failed", name);
    xioctl(fd_, VIDIOC_S_CTRL, &control, message.c_str());
    if (config_.debug) {
      LOGD("cluster H264 V4L2 ctrl %s=%d ok", name, value);
    }
  };

  const auto try_control = [this, &set_control](uint32_t id, int value, const char *name) {
    try {
      set_control(id, value, name);
      return true;
    } catch (const std::exception &e) {
      if (config_.debug) {
        LOGW("%s", e.what());
      }
      return false;
    }
  };

  const NamedControl controls[] = {
    { .id = V4L2_CID_MPEG_VIDEO_BITRATE, .value = config_.bitrate, .name = "bitrate" },
    { .id = V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES, .value = p_frames, .name = "num-p-frames" },
    { .id = V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES, .value = 0, .name = "num-b-frames" },
    { .id = V4L2_CID_MPEG_VIDEO_HEADER_MODE, .value = V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE, .name = "header-mode-separate" },
    { .id = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL, .value = config_.rate_control, .name = rate_control_name(config_.rate_control) },
    {
      .id = V4L2_CID_MPEG_VIDC_VIDEO_PRIORITY,
      .value = config_.realtime_priority ?
               V4L2_MPEG_VIDC_VIDEO_PRIORITY_REALTIME_ENABLE :
               V4L2_MPEG_VIDC_VIDEO_PRIORITY_REALTIME_DISABLE,
      .name = config_.realtime_priority ? "priority-realtime-enable" : "priority-realtime-disable",
    },
    { .id = V4L2_CID_MPEG_VIDC_VIDEO_IDR_PERIOD, .value = 1, .name = "idr-period" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LEVEL, .value = V4L2_MPEG_VIDEO_H264_LEVEL_UNKNOWN, .name = "h264-level-unknown" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE, .value = 0, .name = "h264-loop-filter-mode" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA, .value = 0, .name = "h264-loop-filter-alpha" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA, .value = 0, .name = "h264-loop-filter-beta" },
  };
  for (const NamedControl &control : controls) {
    set_control(control.id, control.value, control.name);
  }

  if (config_.slice_max_bytes > 0) {
    bool slice_mode_ok = try_control(
        V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
        V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES,
        "multi-slice-mode-max-bytes");
    bool slice_bytes_ok = try_control(
        V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES,
        config_.slice_max_bytes,
        "multi-slice-max-bytes");
    if (!slice_mode_ok || !slice_bytes_ok) {
      slice_bytes_ok = try_control(
          V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES,
          config_.slice_max_bytes,
          "multi-slice-max-bytes");
      slice_mode_ok = try_control(
          V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
          V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES,
          "multi-slice-mode-max-bytes");
    }
    if (slice_mode_ok && slice_bytes_ok) {
      try_control(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_DELIVERY_MODE, 1, "multi-slice-delivery-mode");
      if (config_.debug) {
        LOGD("cluster H264 V4L2 multi-slice max_bytes=%d", config_.slice_max_bytes);
      }
    } else {
      try_control(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE, V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE, "multi-slice-mode-single");
      if (config_.debug) {
        LOGW("cluster H264 V4L2 multi-slice max-bytes unavailable, using single-slice output");
      }
    }
  } else {
    try_control(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE, V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE, "multi-slice-mode-single");
    if (config_.debug) {
      LOGD("cluster H264 V4L2 multi-slice disabled");
    }
  }

  try_control(V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER, 1, "repeat-seq-header");
  try_control(
      V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO,
      V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_ENABLED,
      "h264-vui-timing-info");
  try_control(
      V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_BITSTREAM_RESTRICT,
      V4L2_MPEG_VIDC_VIDEO_H264_VUI_BITSTREAM_RESTRICT_ENABLED,
      "h264-vui-bitstream-restrict");

  bool low_complexity_h264 = try_control(
      V4L2_CID_MPEG_VIDEO_H264_PROFILE,
      V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
      "h264-profile-constrained-baseline");
  if (!low_complexity_h264) {
    low_complexity_h264 = try_control(
        V4L2_CID_MPEG_VIDEO_H264_PROFILE,
        V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
        "h264-profile-baseline");
  }
  if (low_complexity_h264) {
    low_complexity_h264 = try_control(
        V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
        V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
        "h264-entropy-cavlc");
  }
  if (!low_complexity_h264) {
    set_control(V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, "h264-profile-high");
    set_control(V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE, V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC, "h264-entropy-cabac");
    set_control(V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL, V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0, "h264-cabac-model-0");
  }
}

void ClusterH264Encoder::request_buffers(uint32_t buffer_type, unsigned int count) {
  struct v4l2_requestbuffers reqbuf = {};
  reqbuf.count = count;
  reqbuf.type = buffer_type;
  reqbuf.memory = V4L2_MEMORY_USERPTR;
  xioctl(fd_, VIDIOC_REQBUFS, &reqbuf, "VIDIOC_REQBUFS failed");
}

void ClusterH264Encoder::stream_on(uint32_t buffer_type) {
  enum v4l2_buf_type type = static_cast<enum v4l2_buf_type>(buffer_type);
  xioctl(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON failed");
}

void ClusterH264Encoder::stream_off(uint32_t buffer_type) {
  enum v4l2_buf_type type = static_cast<enum v4l2_buf_type>(buffer_type);
  optional_ioctl(fd_, VIDIOC_STREAMOFF, &type, "VIDIOC_STREAMOFF failed", config_.debug);
}

void ClusterH264Encoder::allocate_buffers() {
  for (int i = 0; i < CLUSTER_H264_INPUT_BUFFER_COUNT; ++i) {
    input_buffers_[i].allocate(input_bytesused_);
    memset(input_buffers_[i].addr, 16, std::min(input_uv_offset_, input_buffers_[i].len));
    if (input_uv_offset_ < input_buffers_[i].len) {
      memset(reinterpret_cast<uint8_t*>(input_buffers_[i].addr) + input_uv_offset_,
             128, input_buffers_[i].len - input_uv_offset_);
    }
    input_buffers_[i].init_yuv(config_.width, config_.height, input_stride_, input_uv_offset_);
    input_allocated_[i] = true;
  }
  for (int i = 0; i < CLUSTER_H264_CAPTURE_BUFFER_COUNT; ++i) {
    capture_buffers_[i].allocate(capture_sizeimage_);
    capture_allocated_[i] = true;
  }
}

void ClusterH264Encoder::queue_capture_buffer(unsigned int index) {
  VisionBuf *buf = &capture_buffers_[index];
  struct v4l2_plane plane = {};
  plane.bytesused = static_cast<uint32_t>(buf->len);
  plane.length = static_cast<uint32_t>(buf->len);
  plane.m.userptr = reinterpret_cast<unsigned long>(buf->addr);
  plane.reserved[0] = static_cast<unsigned int>(buf->fd);

  struct v4l2_buffer v4l_buf = {};
  v4l_buf.index = index;
  v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  v4l_buf.memory = V4L2_MEMORY_USERPTR;
  v4l_buf.m.planes = &plane;
  v4l_buf.length = 1;
  xioctl(fd_, VIDIOC_QBUF, &v4l_buf, "VIDIOC_QBUF capture failed");
}

void ClusterH264Encoder::queue_output_buffer(unsigned int index, uint64_t timestamp_us) {
  VisionBuf *buf = &input_buffers_[index];
  struct timeval timestamp = {
    .tv_sec = static_cast<long>(timestamp_us / 1000000ULL),
    .tv_usec = static_cast<long>(timestamp_us % 1000000ULL),
  };

  struct v4l2_plane plane = {};
  plane.bytesused = static_cast<uint32_t>(input_bytesused_);
  plane.length = static_cast<uint32_t>(buf->len);
  plane.m.userptr = reinterpret_cast<unsigned long>(buf->addr);
  plane.reserved[0] = static_cast<unsigned int>(buf->fd);

  struct v4l2_buffer v4l_buf = {};
  v4l_buf.index = index;
  v4l_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  v4l_buf.flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
  v4l_buf.timestamp = timestamp;
  v4l_buf.memory = V4L2_MEMORY_USERPTR;
  v4l_buf.m.planes = &plane;
  v4l_buf.length = 1;
  xioctl(fd_, VIDIOC_QBUF, &v4l_buf, "VIDIOC_QBUF output failed");
}

bool ClusterH264Encoder::dequeue_buffer(uint32_t buffer_type, DequeueResult *result) {
  struct v4l2_plane plane = {};
  struct v4l2_buffer v4l_buf = {};
  v4l_buf.type = buffer_type;
  v4l_buf.memory = V4L2_MEMORY_USERPTR;
  v4l_buf.m.planes = &plane;
  v4l_buf.length = 1;

  int ret;
  do {
    ret = ioctl(fd_, VIDIOC_DQBUF, &v4l_buf);
  } while (ret == -1 && errno == EINTR);
  if (ret == -1) {
    if (errno == EAGAIN) {
      return false;
    }
    throw std::runtime_error(util::string_format("VIDIOC_DQBUF failed: %s (%d)", strerror(errno), errno));
  }

  if (result != nullptr) {
    result->index = v4l_buf.index;
    result->bytesused = plane.bytesused;
    result->flags = v4l_buf.flags;
    result->timestamp_us = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec) * 1000000ULL + static_cast<uint64_t>(v4l_buf.timestamp.tv_usec);
  }
  return true;
}

std::vector<ClusterH264Packet> ClusterH264Encoder::process_ready_events(int timeout_ms, bool stop_after_first_event) {
  std::vector<ClusterH264Packet> packets;
  process_ready_events(timeout_ms, stop_after_first_event, [&packets](const ClusterH264PacketView &view) {
    ClusterH264Packet packet;
    packet.flags = view.flags;
    packet.timestamp_us = view.timestamp_us;
    packet.codec_config = view.codec_config;
    packet.keyframe = view.keyframe;
    packet.data.assign(view.data, view.data + view.size);
    packets.push_back(std::move(packet));
  });
  return packets;
}

size_t ClusterH264Encoder::process_ready_events(int timeout_ms, bool stop_after_first_event, const ClusterH264PacketCallback &on_packet) {
  size_t packet_count = 0;
  struct pollfd pfd = {
    .fd = fd_,
    .events = POLLIN | POLLOUT | POLLERR,
    .revents = 0,
  };

  while (true) {
    pfd.revents = 0;
    int ret;
    do {
      ret = poll(&pfd, 1, timeout_ms);
    } while (ret < 0 && errno == EINTR);
    if (ret < 0) {
      throw std::runtime_error(util::string_format("cluster H264 poll failed: %s (%d)", strerror(errno), errno));
    }
    if (ret == 0) {
      return packet_count;
    }
    if ((pfd.revents & POLLERR) && (pfd.revents & (POLLIN | POLLOUT)) == 0) {
      throw std::runtime_error("cluster H264 V4L2 encoder reported POLLERR");
    }

    bool made_progress = false;
    if (pfd.revents & POLLIN) {
      while (true) {
        DequeueResult result;
        if (!dequeue_buffer(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, &result)) {
          break;
        }
        made_progress = true;
        if ((result.flags & V4L2_QCOM_BUF_FLAG_EOS) == 0 && result.bytesused > 0) {
          VisionBuf *buf = &capture_buffers_[result.index];
          buf->sync(VISIONBUF_SYNC_FROM_DEVICE);
          const uint8_t *data = reinterpret_cast<const uint8_t*>(buf->addr);
          const bool codec_config = (result.flags & V4L2_QCOM_BUF_FLAG_CODECCONFIG) != 0;
          const bool keyframe = (result.flags & V4L2_BUF_FLAG_KEYFRAME) != 0;
          if (codec_config) {
            codec_config_.assign(data, data + result.bytesused);
            ++packet_count;
            queue_capture_buffer(result.index);
            continue;
          }

          ClusterH264PacketView packet;
          std::vector<uint8_t> joined_keyframe;
          const bool needs_codec_config = !codec_config_.empty() && (keyframe || !sent_video_packet_);
          if (needs_codec_config) {
            joined_keyframe.reserve(codec_config_.size() + result.bytesused);
            joined_keyframe.insert(joined_keyframe.end(), codec_config_.begin(), codec_config_.end());
            joined_keyframe.insert(joined_keyframe.end(), data, data + result.bytesused);
            packet.data = joined_keyframe.data();
            packet.size = joined_keyframe.size();
          } else {
            packet.data = data;
            packet.size = result.bytesused;
          }
          packet.flags = result.flags;
          packet.timestamp_us = result.timestamp_us;
          packet.codec_config = false;
          packet.keyframe = keyframe;
          if (on_packet) {
            on_packet(packet);
          }
          sent_video_packet_ = true;
          ++packet_count;
        }
        queue_capture_buffer(result.index);
      }
    }

    if (pfd.revents & POLLOUT) {
      while (true) {
        DequeueResult result;
        if (!dequeue_buffer(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, &result)) {
          break;
        }
        made_progress = true;
        if (std::find(free_inputs_.begin(), free_inputs_.end(), result.index) == free_inputs_.end()) {
          free_inputs_.push_back(result.index);
        }
      }
    }

    if (stop_after_first_event || !made_progress) {
      return packet_count;
    }
    timeout_ms = 0;
  }
}

std::vector<ClusterH264Packet> ClusterH264Encoder::drain(int timeout_ms) {
  if (!is_open_) {
    return {};
  }
  return process_ready_events(timeout_ms, false);
}

void ClusterH264Encoder::drain(int timeout_ms, const ClusterH264PacketCallback &on_packet) {
  if (!is_open_) {
    return;
  }
  process_ready_events(timeout_ms, false, on_packet);
}

std::vector<ClusterH264Packet> ClusterH264Encoder::encode_nv12(const uint8_t *nv12, size_t nv12_size, uint64_t timestamp_us) {
  std::vector<ClusterH264Packet> packets;
  encode_nv12(nv12, nv12_size, timestamp_us, [&packets](const ClusterH264PacketView &view) {
    ClusterH264Packet packet;
    packet.flags = view.flags;
    packet.timestamp_us = view.timestamp_us;
    packet.codec_config = view.codec_config;
    packet.keyframe = view.keyframe;
    packet.data.assign(view.data, view.data + view.size);
    packets.push_back(std::move(packet));
  });
  return packets;
}

void ClusterH264Encoder::encode_nv12(const uint8_t *nv12, size_t nv12_size, uint64_t timestamp_us, const ClusterH264PacketCallback &on_packet) {
  encode_input(nv12, nv12_size, timestamp_us, on_packet, &ClusterH264Encoder::copy_nv12_to_input, "NV12");
}

std::vector<ClusterH264Packet> ClusterH264Encoder::encode_nv12_active(const uint8_t *nv12, size_t nv12_size, uint64_t timestamp_us) {
  std::vector<ClusterH264Packet> packets;
  encode_nv12_active(nv12, nv12_size, timestamp_us, [&packets](const ClusterH264PacketView &view) {
    ClusterH264Packet packet;
    packet.flags = view.flags;
    packet.timestamp_us = view.timestamp_us;
    packet.codec_config = view.codec_config;
    packet.keyframe = view.keyframe;
    packet.data.assign(view.data, view.data + view.size);
    packets.push_back(std::move(packet));
  });
  return packets;
}

void ClusterH264Encoder::encode_nv12_active(const uint8_t *nv12, size_t nv12_size, uint64_t timestamp_us, const ClusterH264PacketCallback &on_packet) {
  encode_input(nv12, nv12_size, timestamp_us, on_packet, &ClusterH264Encoder::copy_nv12_active_to_input, "NV12 active");
}

void ClusterH264Encoder::encode_input(const uint8_t *data, size_t data_size, uint64_t timestamp_us,
                                      const ClusterH264PacketCallback &on_packet, InputCopyFn copy_input,
                                      const char *input_name) {
  if (!is_open_) {
    throw std::runtime_error("cluster H264 encoder is not open");
  }
  if (data == nullptr) {
    throw std::runtime_error(std::string("cluster H264 encoder received null ") + input_name + " input");
  }

  last_encode_timings_ = {};
  const auto total_start = std::chrono::steady_clock::now();
  auto stage_start = std::chrono::steady_clock::now();
  process_ready_events(0, false, on_packet);
  last_encode_timings_.pre_poll_us = elapsed_us(stage_start);
  while (free_inputs_.empty()) {
    stage_start = std::chrono::steady_clock::now();
    const size_t packet_count = process_ready_events(2000, true, on_packet);
    last_encode_timings_.wait_input_us += elapsed_us(stage_start);
    if (free_inputs_.empty() && packet_count == 0) {
      throw std::runtime_error("cluster H264 encoder timed out waiting for a free input buffer");
    }
  }

  unsigned int index = free_inputs_.front();
  free_inputs_.pop_front();
  stage_start = std::chrono::steady_clock::now();
  (this->*copy_input)(data, data_size, &input_buffers_[index]);
  last_encode_timings_.convert_us = elapsed_us(stage_start);
  stage_start = std::chrono::steady_clock::now();
  if (input_buffers_[index].sync(VISIONBUF_SYNC_TO_DEVICE) != 0) {
    throw std::runtime_error("cluster H264 encoder failed to sync input to device");
  }
  last_encode_timings_.sync_us = elapsed_us(stage_start);
  stage_start = std::chrono::steady_clock::now();
  queue_output_buffer(index, timestamp_us);
  last_encode_timings_.queue_us = elapsed_us(stage_start);

  stage_start = std::chrono::steady_clock::now();
  process_ready_events(0, false, on_packet);
  last_encode_timings_.post_poll_us = elapsed_us(stage_start);
  last_encode_timings_.total_us = elapsed_us(total_start);
}

void ClusterH264Encoder::copy_nv12_to_input(const uint8_t *nv12, size_t nv12_size, VisionBuf *dst) const {
  if (!input_is_nv12()) {
    throw std::runtime_error("cluster H264 encoder has unsupported input format " + input_v4l_format_name_);
  }
  if (nv12_size < input_bytesused_) {
    throw std::runtime_error("cluster H264 encoder NV12 input is smaller than the V4L2 input bytesused");
  }
  if (dst == nullptr || dst->addr == nullptr || dst->len < input_bytesused_) {
    throw std::runtime_error("cluster H264 encoder input buffer is not allocated");
  }
  memcpy(dst->addr, nv12, input_bytesused_);
}

void ClusterH264Encoder::copy_nv12_active_to_input(const uint8_t *nv12, size_t nv12_size, VisionBuf *dst) const {
  if (!input_is_nv12()) {
    throw std::runtime_error("cluster H264 encoder has unsupported input format " + input_v4l_format_name_);
  }
  const size_t active_bytes = input_active_bytes();
  if (nv12_size < active_bytes) {
    throw std::runtime_error("cluster H264 encoder active NV12 input is smaller than the Y+UV plane bytes");
  }
  if (dst == nullptr || dst->addr == nullptr || dst->len < input_bytesused_) {
    throw std::runtime_error("cluster H264 encoder input buffer is not allocated");
  }
  memcpy(dst->addr, nv12, active_bytes);
}
