#include "system/loggerd/encoder/cluster_h264_encoder.h"

#include <algorithm>
#include <cerrno>
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

uint8_t clamp_u8(int value) {
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

uint8_t rgba_to_y(uint8_t r, uint8_t g, uint8_t b) {
  return clamp_u8(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);
}

uint8_t rgba_to_u(int r, int g, int b) {
  return clamp_u8(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
}

uint8_t rgba_to_v(int r, int g, int b) {
  return clamp_u8(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
}

const char *rgb4_layout_name(ClusterH264Rgb4Layout layout) {
  switch (layout) {
    case ClusterH264Rgb4Layout::AXRGB: return "AXRGB";
    case ClusterH264Rgb4Layout::RGBA: return "RGBA";
    case ClusterH264Rgb4Layout::BGRA: return "BGRA";
  }
  return "unknown";
}

const char *h264_profile_name(ClusterH264Profile profile) {
  switch (profile) {
    case ClusterH264Profile::Baseline: return "Baseline/CAVLC";
    case ClusterH264Profile::High: return "High/CABAC";
  }
  return "unknown";
}

const char *rate_control_name(ClusterH264RateControl rate_control) {
  switch (rate_control) {
    case ClusterH264RateControl::VbrCfr: return "vbr-cfr";
    case ClusterH264RateControl::CbrCfr: return "cbr-cfr";
    case ClusterH264RateControl::Cq: return "cq";
    case ClusterH264RateControl::Off: return "off";
  }
  return "unknown";
}

int rate_control_value(ClusterH264RateControl rate_control) {
  switch (rate_control) {
    case ClusterH264RateControl::VbrCfr: return V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR;
    case ClusterH264RateControl::CbrCfr: return V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR;
    case ClusterH264RateControl::Cq: return V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CQ;
    case ClusterH264RateControl::Off: return V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF;
  }
  return V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR;
}

int pack_frame_qps(int i_qp, int p_qp, int b_qp) {
  return (i_qp & 0xff) | ((p_qp & 0xff) << 8) | ((b_qp & 0xff) << 16);
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
  if (config_.slice_max_mb != 0) {
    throw std::runtime_error("cluster H264 encoder slice max MB is disabled because it can stall the Qualcomm V4L2 encoder");
  }
  if (config_.qp < -1 || config_.qp > 51) {
    throw std::runtime_error("cluster H264 encoder qp must be -1 or between 0 and 51");
  }
  if (config_.device_path.empty()) {
    throw std::runtime_error("cluster H264 encoder device path must not be empty");
  }
}

bool ClusterH264Encoder::input_is_rgb4() const {
  return input_v4l_format_ == V4L2_PIX_FMT_RGB32;
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
  const uint32_t rgb4 = V4L2_PIX_FMT_RGB32;
  const uint32_t nv12 = V4L2_PIX_FMT_NV12;
  std::vector<uint32_t> input_formats = enumerate_formats(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
  const auto supports_format = [&input_formats](uint32_t format) {
    return input_formats.empty() || std::find(input_formats.begin(), input_formats.end(), format) != input_formats.end();
  };

  uint32_t selected_input_format = 0;
  if (config_.input_format == ClusterH264InputFormat::RGB4) {
    if (!supports_format(rgb4)) {
      throw std::runtime_error("V4L2 encoder does not report RGB4 input support");
    }
    selected_input_format = rgb4;
  } else if (config_.input_format == ClusterH264InputFormat::NV12) {
    if (!supports_format(nv12)) {
      throw std::runtime_error("V4L2 encoder does not report NV12 input support");
    }
    selected_input_format = nv12;
  } else if (supports_format(rgb4)) {
    selected_input_format = rgb4;
  } else if (supports_format(nv12)) {
    selected_input_format = nv12;
  } else {
    std::string found;
    for (uint32_t format : input_formats) {
      if (!found.empty()) found += ", ";
      found += fourcc_to_string(format);
    }
    throw std::runtime_error("V4L2 encoder does not report RGB4 or NV12 input support; found: " + found);
  }

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
  fmt_in.fmt.pix_mp.colorspace = selected_input_format == rgb4 ?
                                  V4L2_COLORSPACE_SRGB :
                                  V4L2_COLORSPACE_470_SYSTEM_BG;
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
  input_y_scanlines_ = 0;
  input_uv_scanlines_ = 0;
  if (selected_input_format == nv12) {
    auto [venus_stride, venus_y_height, venus_uv_height, venus_size] = get_nv12_info(config_.width, config_.height);
    input_stride_ = venus_stride;
    input_y_scanlines_ = venus_y_height;
    input_uv_scanlines_ = venus_uv_height;
    input_uv_offset_ = input_stride_ * input_y_scanlines_;
    const size_t min_bytesused = input_uv_offset_ + input_stride_ * input_uv_scanlines_;
    input_bytesused_ = std::max({input_sizeimage_, min_bytesused, static_cast<size_t>(venus_size)});
  } else {
    input_stride_ = driver_stride;
    const size_t min_stride = static_cast<size_t>(config_.width) * 4;
    if (input_stride_ < min_stride) {
      input_stride_ = min_stride;
    }
    input_uv_offset_ = 0;
    input_bytesused_ = std::max(input_sizeimage_, input_stride_ * static_cast<size_t>(config_.height));
  }
  capture_sizeimage_ = fmt_out.fmt.pix_mp.plane_fmt[0].sizeimage;
  if (capture_sizeimage_ == 0) {
    throw std::runtime_error("V4L2 encoder returned zero H264 capture sizeimage");
  }

  LOGD("cluster H264 V4L2 formats: in=%s %dx%d driver_stride=%zu stride=%zu scanlines=%zu/%zu sizeimage=%zu bytesused=%zu uv_offset=%zu rgb4_layout=%s profile=%s rate_control=%s out=H264 sizeimage=%zu",
       input_v4l_format_name_.c_str(), config_.width, config_.height, driver_stride, input_stride_,
       input_y_scanlines_, input_uv_scanlines_, input_sizeimage_, input_bytesused_, input_uv_offset_,
       input_is_rgb4() ? rgb4_layout_name(config_.rgb4_layout) : "n/a",
       h264_profile_name(config_.h264_profile), rate_control_name(config_.rate_control), capture_sizeimage_);
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
    { .id = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL, .value = rate_control_value(config_.rate_control), .name = rate_control_name(config_.rate_control) },
    { .id = V4L2_CID_MPEG_VIDC_VIDEO_PRIORITY, .value = V4L2_MPEG_VIDC_VIDEO_PRIORITY_REALTIME_DISABLE, .name = "priority-realtime-disable" },
    { .id = V4L2_CID_MPEG_VIDC_VIDEO_IDR_PERIOD, .value = 1, .name = "idr-period" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LEVEL, .value = V4L2_MPEG_VIDEO_H264_LEVEL_UNKNOWN, .name = "h264-level-unknown" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE, .value = 0, .name = "h264-loop-filter-mode" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA, .value = 0, .name = "h264-loop-filter-alpha" },
    { .id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA, .value = 0, .name = "h264-loop-filter-beta" },
  };
  for (const NamedControl &control : controls) {
    set_control(control.id, control.value, control.name);
  }

  if (config_.slice_max_mb > 0) {
    bool slice_mode_ok = try_control(
        V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
        V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB,
        "multi-slice-mode-max-mb");
    bool slice_mb_ok = try_control(
        V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
        config_.slice_max_mb,
        "multi-slice-max-mb");
    if (!slice_mode_ok || !slice_mb_ok) {
      slice_mb_ok = try_control(
          V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
          config_.slice_max_mb,
          "multi-slice-max-mb");
      slice_mode_ok = try_control(
          V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
          V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB,
          "multi-slice-mode-max-mb");
    }
    if (slice_mode_ok && slice_mb_ok) {
      try_control(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_DELIVERY_MODE, 1, "multi-slice-delivery-mode");
      if (config_.debug) {
        LOGD("cluster H264 V4L2 multi-slice max_mb=%d", config_.slice_max_mb);
      }
    } else {
      try_control(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE, V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE, "multi-slice-mode-single");
      if (config_.debug) {
        LOGW("cluster H264 V4L2 multi-slice max-mb unavailable, using single-slice output");
      }
    }
  } else if (config_.slice_max_bytes > 0) {
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

  if (config_.qp >= 0) {
    const int qp = config_.qp;
    const int packed_qp = pack_frame_qps(qp, qp, qp);
    try_control(V4L2_CID_MPEG_VIDEO_MIN_QP_PACKED, packed_qp, "qp-min-packed");
    try_control(V4L2_CID_MPEG_VIDEO_MAX_QP_PACKED, packed_qp, "qp-max-packed");
    try_control(V4L2_CID_MPEG_VIDEO_H264_MAX_QP, qp, "h264-max-qp");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_I_FRAME_QP_MAX, qp, "vidc-i-frame-qp-max");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_P_FRAME_QP_MAX, qp, "vidc-p-frame-qp-max");
    try_control(V4L2_CID_MPEG_VIDEO_H264_MIN_QP, qp, "h264-min-qp");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_I_FRAME_QP_MIN, qp, "vidc-i-frame-qp-min");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_P_FRAME_QP_MIN, qp, "vidc-p-frame-qp-min");
    try_control(V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, qp, "h264-i-frame-qp");
    try_control(V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, qp, "h264-p-frame-qp");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_I_FRAME_QP, qp, "vidc-i-frame-qp");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_P_FRAME_QP, qp, "vidc-p-frame-qp");
    try_control(
        V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP,
        V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP_IFRAME | V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP_PFRAME,
        "vidc-enable-initial-qp");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_INITIAL_I_FRAME_QP, qp, "vidc-initial-i-frame-qp");
    try_control(V4L2_CID_MPEG_VIDC_VIDEO_INITIAL_P_FRAME_QP, qp, "vidc-initial-p-frame-qp");
    if (config_.debug) {
      LOGD("cluster H264 V4L2 qp=%d", qp);
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

  if (config_.h264_profile == ClusterH264Profile::High) {
    set_control(V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, "h264-profile-high");
    set_control(V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE, V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC, "h264-entropy-cabac");
    set_control(V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL, V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0, "h264-cabac-model-0");
    return;
  }

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
    if (input_is_nv12()) {
      memset(input_buffers_[i].addr, 16, std::min(input_uv_offset_, input_buffers_[i].len));
      if (input_uv_offset_ < input_buffers_[i].len) {
        memset(reinterpret_cast<uint8_t*>(input_buffers_[i].addr) + input_uv_offset_,
               128, input_buffers_[i].len - input_uv_offset_);
      }
      input_buffers_[i].init_yuv(config_.width, config_.height, input_stride_, input_uv_offset_);
    } else {
      memset(input_buffers_[i].addr, 0, input_buffers_[i].len);
    }
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

std::vector<ClusterH264Packet> ClusterH264Encoder::encode_rgba(const uint8_t *rgba, size_t rgba_size, uint64_t timestamp_us) {
  std::vector<ClusterH264Packet> packets;
  encode_rgba(rgba, rgba_size, timestamp_us, [&packets](const ClusterH264PacketView &view) {
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

void ClusterH264Encoder::encode_rgba(const uint8_t *rgba, size_t rgba_size, uint64_t timestamp_us, const ClusterH264PacketCallback &on_packet) {
  if (!is_open_) {
    throw std::runtime_error("cluster H264 encoder is not open");
  }
  if (rgba == nullptr) {
    throw std::runtime_error("cluster H264 encoder received null RGBA input");
  }

  process_ready_events(0, false, on_packet);
  while (free_inputs_.empty()) {
    const size_t packet_count = process_ready_events(2000, true, on_packet);
    if (free_inputs_.empty() && packet_count == 0) {
      throw std::runtime_error("cluster H264 encoder timed out waiting for a free input buffer");
    }
  }

  unsigned int index = free_inputs_.front();
  free_inputs_.pop_front();
  copy_rgba_to_input(rgba, rgba_size, &input_buffers_[index]);
  if (input_buffers_[index].sync(VISIONBUF_SYNC_TO_DEVICE) != 0) {
    throw std::runtime_error("cluster H264 encoder failed to sync input to device");
  }
  queue_output_buffer(index, timestamp_us);

  process_ready_events(0, false, on_packet);
}

void ClusterH264Encoder::copy_rgba_to_input(const uint8_t *rgba, size_t rgba_size, VisionBuf *dst) const {
  if (input_is_rgb4()) {
    rgba_to_rgb4(rgba, rgba_size, dst);
    return;
  }
  if (input_is_nv12()) {
    rgba_to_nv12(rgba, rgba_size, dst);
    return;
  }
  throw std::runtime_error("cluster H264 encoder has unsupported input format " + input_v4l_format_name_);
}

void ClusterH264Encoder::rgba_to_rgb4(const uint8_t *rgba, size_t rgba_size, VisionBuf *dst) const {
  const size_t width = static_cast<size_t>(config_.width);
  const size_t height = static_cast<size_t>(config_.height);
  const size_t src_row_bytes = width * 4;
  const size_t expected_rgba_size = src_row_bytes * height;
  if (rgba_size < expected_rgba_size) {
    throw std::runtime_error("cluster H264 encoder RGBA input is smaller than width*height*4");
  }
  if (dst == nullptr || dst->addr == nullptr || dst->len < input_bytesused_) {
    throw std::runtime_error("cluster H264 encoder input buffer is not allocated");
  }

  uint8_t *base = reinterpret_cast<uint8_t*>(dst->addr);
  for (size_t y = 0; y < height; ++y) {
    const uint8_t *src = rgba + y * src_row_bytes;
    uint8_t *dst_row = base + y * input_stride_;
    if (config_.rgb4_layout == ClusterH264Rgb4Layout::RGBA) {
      memcpy(dst_row, src, src_row_bytes);
    } else if (config_.rgb4_layout == ClusterH264Rgb4Layout::BGRA) {
      for (size_t x = 0; x < width; ++x) {
        const uint8_t *src_px = src + x * 4;
        uint8_t *dst_px = dst_row + x * 4;
        dst_px[0] = src_px[2];
        dst_px[1] = src_px[1];
        dst_px[2] = src_px[0];
        dst_px[3] = config_.rgb4_use_source_alpha ? src_px[3] : config_.rgb4_alpha;
      }
    } else {
      for (size_t x = 0; x < width; ++x) {
        const uint8_t *src_px = src + x * 4;
        uint8_t *dst_px = dst_row + x * 4;
        // Fallback for drivers that want network-order A/X,R,G,B bytes.
        dst_px[0] = config_.rgb4_use_source_alpha ? src_px[3] : config_.rgb4_alpha;
        dst_px[1] = src_px[0];
        dst_px[2] = src_px[1];
        dst_px[3] = src_px[2];
      }
    }
  }
}

void ClusterH264Encoder::rgba_to_nv12(const uint8_t *rgba, size_t rgba_size, VisionBuf *dst) const {
  const size_t width = static_cast<size_t>(config_.width);
  const size_t height = static_cast<size_t>(config_.height);
  const size_t expected_rgba_size = width * height * 4;
  if (rgba_size < expected_rgba_size) {
    throw std::runtime_error("cluster H264 encoder RGBA input is smaller than width*height*4");
  }
  if (dst == nullptr || dst->addr == nullptr || dst->len < input_bytesused_) {
    throw std::runtime_error("cluster H264 encoder input buffer is not allocated");
  }

  uint8_t *base = reinterpret_cast<uint8_t*>(dst->addr);
  uint8_t *y_plane = base;
  uint8_t *uv_plane = base + input_uv_offset_;

  for (size_t y = 0; y < height; ++y) {
    uint8_t *dst_y = y_plane + y * input_stride_;
    const uint8_t *src = rgba + y * width * 4;
    for (size_t x = 0; x < width; ++x) {
      const uint8_t r = src[x * 4 + 0];
      const uint8_t g = src[x * 4 + 1];
      const uint8_t b = src[x * 4 + 2];
      dst_y[x] = rgba_to_y(r, g, b);
    }
  }

  for (size_t y = 0; y < height; y += 2) {
    uint8_t *dst_uv = uv_plane + (y / 2) * input_stride_;
    const uint8_t *row0 = rgba + y * width * 4;
    const uint8_t *row1 = rgba + (y + 1) * width * 4;
    for (size_t x = 0; x < width; x += 2) {
      const uint8_t *p00 = row0 + x * 4;
      const uint8_t *p01 = p00 + 4;
      const uint8_t *p10 = row1 + x * 4;
      const uint8_t *p11 = p10 + 4;
      const int r = (static_cast<int>(p00[0]) + p01[0] + p10[0] + p11[0] + 2) / 4;
      const int g = (static_cast<int>(p00[1]) + p01[1] + p10[1] + p11[1] + 2) / 4;
      const int b = (static_cast<int>(p00[2]) + p01[2] + p10[2] + p11[2] + 2) / 4;
      dst_uv[x + 0] = rgba_to_u(r, g, b);
      dst_uv[x + 1] = rgba_to_v(r, g, b);
    }
  }
}
