from __future__ import annotations

from dataclasses import dataclass
import ctypes
import errno
import os
import select
import time
from typing import Any


DEFAULT_V4L2_ENCODER_DEVICE = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1"
V4L2_INPUT_BUFFER_COUNT = 7
V4L2_CAPTURE_BUFFER_COUNT = 6
V4L2_CAPTURE_TIMEOUT_MS = 2000

VIDEO_MAX_PLANES = 8
V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9
V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE = 10
V4L2_MEMORY_USERPTR = 2
V4L2_FIELD_ANY = 0
V4L2_COLORSPACE_DEFAULT = 0
V4L2_COLORSPACE_SRGB = 8
V4L2_BUF_FLAG_KEYFRAME = 0x00000008
V4L2_BUF_FLAG_TIMESTAMP_COPY = 0x00004000
V4L2_QCOM_BUF_FLAG_CODECCONFIG = 0x00020000
V4L2_QCOM_BUF_FLAG_EOS = 0x02000000
V4L2_ENC_CMD_STOP = 1

V4L2_CTRL_CLASS_MPEG = 0x00990000
V4L2_CID_MPEG_BASE = V4L2_CTRL_CLASS_MPEG | 0x900
V4L2_CID_MPEG_MSM_VIDC_BASE = V4L2_CTRL_CLASS_MPEG | 0x2000
V4L2_CID_MPEG_VIDEO_BITRATE = V4L2_CID_MPEG_BASE + 207
V4L2_CID_MPEG_VIDEO_HEADER_MODE = V4L2_CID_MPEG_BASE + 216
V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE = V4L2_CID_MPEG_BASE + 221
V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE = V4L2_CID_MPEG_BASE + 357
V4L2_CID_MPEG_VIDEO_H264_LEVEL = V4L2_CID_MPEG_BASE + 359
V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA = V4L2_CID_MPEG_BASE + 360
V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA = V4L2_CID_MPEG_BASE + 361
V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE = V4L2_CID_MPEG_BASE + 362
V4L2_CID_MPEG_VIDEO_H264_PROFILE = V4L2_CID_MPEG_BASE + 363
V4L2_CID_MPEG_VIDC_VIDEO_IDR_PERIOD = V4L2_CID_MPEG_MSM_VIDC_BASE + 5
V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES = V4L2_CID_MPEG_MSM_VIDC_BASE + 6
V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES = V4L2_CID_MPEG_MSM_VIDC_BASE + 7
V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL = V4L2_CID_MPEG_MSM_VIDC_BASE + 9
V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL = V4L2_CID_MPEG_MSM_VIDC_BASE + 11
V4L2_CID_MPEG_VIDC_VIDEO_PRIORITY = V4L2_CID_MPEG_MSM_VIDC_BASE + 52

V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE = 0
V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC = 1
V4L2_MPEG_VIDEO_H264_LEVEL_UNKNOWN = 17
V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED = 0
V4L2_MPEG_VIDEO_H264_PROFILE_HIGH = 4
V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE = 0
V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR = 2
V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0 = 0
V4L2_MPEG_VIDC_VIDEO_PRIORITY_REALTIME_DISABLE = 1


def v4l2_fourcc(a: str, b: str, c: str, d: str) -> int:
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)


V4L2_PIX_FMT_H264 = v4l2_fourcc("H", "2", "6", "4")
V4L2_PIX_FMT_RGB32 = v4l2_fourcc("R", "G", "B", "4")


def fourcc_to_str(value: int) -> str:
    return "".join(chr((int(value) >> shift) & 0xFF) for shift in (0, 8, 16, 24))


def parse_h264_bitrate_to_bps(value: str) -> int:
    text = str(value).strip().lower()
    if not text:
        raise ValueError("bitrate must not be empty")
    multiplier = 1
    if text[-1] == "k":
        multiplier = 1000
        text = text[:-1]
    elif text[-1] == "m":
        multiplier = 1000 * 1000
        text = text[:-1]
    try:
        bitrate = int(float(text) * multiplier)
    except ValueError as exc:
        raise ValueError(f"invalid bitrate: {value}") from exc
    if bitrate <= 0:
        raise ValueError(f"invalid bitrate: {value}")
    return bitrate


def _ioc(direction: int, ioctl_type: str, number: int, size: int) -> int:
    return (direction << 30) | (size << 16) | (ord(ioctl_type) << 8) | number


def _iow(ioctl_type: str, number: int, ctype: type[ctypes.Structure] | type[ctypes.c_int]) -> int:
    return _ioc(1, ioctl_type, number, ctypes.sizeof(ctype))


def _ior(ioctl_type: str, number: int, ctype: type[ctypes.Structure]) -> int:
    return _ioc(2, ioctl_type, number, ctypes.sizeof(ctype))


def _iowr(ioctl_type: str, number: int, ctype: type[ctypes.Structure]) -> int:
    return _ioc(3, ioctl_type, number, ctypes.sizeof(ctype))


class _V4L2Capability(ctypes.Structure):
    _fields_ = [
        ("driver", ctypes.c_uint8 * 16),
        ("card", ctypes.c_uint8 * 32),
        ("bus_info", ctypes.c_uint8 * 32),
        ("version", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("device_caps", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


class _V4L2PlanePixFormat(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("sizeimage", ctypes.c_uint32),
        ("bytesperline", ctypes.c_uint32),
        ("reserved", ctypes.c_uint16 * 6),
    ]


class _V4L2PixFormatMPlane(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("pixelformat", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("colorspace", ctypes.c_uint32),
        ("plane_fmt", _V4L2PlanePixFormat * VIDEO_MAX_PLANES),
        ("num_planes", ctypes.c_uint8),
        ("flags", ctypes.c_uint8),
        ("ycbcr_enc", ctypes.c_uint8),
        ("quantization", ctypes.c_uint8),
        ("xfer_func", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 7),
    ]


class _V4L2FormatUnion(ctypes.Union):
    _fields_ = [
        ("pix_mp", _V4L2PixFormatMPlane),
        ("raw_data", ctypes.c_uint8 * 200),
        # The real videodev2.h union also contains pointer-bearing members
        # such as v4l2_window. Keep the union 8-byte aligned on 64-bit Linux
        # so VIDIOC_* request numbers encode the same sizeof(v4l2_format).
        ("_align", ctypes.c_void_p),
    ]


class _V4L2Format(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("fmt", _V4L2FormatUnion),
    ]


class _V4L2RequestBuffers(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class _V4L2Fract(ctypes.Structure):
    _fields_ = [
        ("numerator", ctypes.c_uint32),
        ("denominator", ctypes.c_uint32),
    ]


class _V4L2OutputParm(ctypes.Structure):
    _fields_ = [
        ("capability", ctypes.c_uint32),
        ("outputmode", ctypes.c_uint32),
        ("timeperframe", _V4L2Fract),
        ("extendedmode", ctypes.c_uint32),
        ("writebuffers", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 4),
    ]


class _V4L2StreamParmUnion(ctypes.Union):
    _fields_ = [
        ("output", _V4L2OutputParm),
        ("raw_data", ctypes.c_uint8 * 200),
    ]


class _V4L2StreamParm(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("parm", _V4L2StreamParmUnion),
    ]


class _V4L2Control(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("value", ctypes.c_int32),
    ]


class _TimeVal(ctypes.Structure):
    _fields_ = [
        ("tv_sec", ctypes.c_long),
        ("tv_usec", ctypes.c_long),
    ]


class _V4L2Timecode(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("frames", ctypes.c_uint8),
        ("seconds", ctypes.c_uint8),
        ("minutes", ctypes.c_uint8),
        ("hours", ctypes.c_uint8),
        ("userbits", ctypes.c_uint8 * 4),
    ]


class _V4L2PlaneMemory(ctypes.Union):
    _fields_ = [
        ("mem_offset", ctypes.c_uint32),
        ("userptr", ctypes.c_uint64),
        ("fd", ctypes.c_int32),
    ]


class _V4L2Plane(ctypes.Structure):
    _fields_ = [
        ("bytesused", ctypes.c_uint32),
        ("length", ctypes.c_uint32),
        ("m", _V4L2PlaneMemory),
        ("data_offset", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 11),
    ]


class _V4L2BufferMemory(ctypes.Union):
    _fields_ = [
        ("offset", ctypes.c_uint32),
        ("userptr", ctypes.c_uint64),
        ("planes", ctypes.POINTER(_V4L2Plane)),
        ("fd", ctypes.c_int32),
    ]


class _V4L2BufferRequest(ctypes.Union):
    _fields_ = [
        ("request_fd", ctypes.c_int32),
        ("reserved", ctypes.c_uint32),
    ]


class _V4L2Buffer(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("timestamp", _TimeVal),
        ("timecode", _V4L2Timecode),
        ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("m", _V4L2BufferMemory),
        ("length", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("request", _V4L2BufferRequest),
    ]


class _V4L2EncoderCmdRaw(ctypes.Structure):
    _fields_ = [("data", ctypes.c_uint32 * 8)]


class _V4L2EncoderCmdUnion(ctypes.Union):
    _fields_ = [("raw", _V4L2EncoderCmdRaw)]


class _V4L2EncoderCmd(ctypes.Structure):
    _fields_ = [
        ("cmd", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("u", _V4L2EncoderCmdUnion),
    ]


VIDIOC_QUERYCAP = _ior("V", 0, _V4L2Capability)
VIDIOC_S_FMT = _iowr("V", 5, _V4L2Format)
VIDIOC_REQBUFS = _iowr("V", 8, _V4L2RequestBuffers)
VIDIOC_QBUF = _iowr("V", 15, _V4L2Buffer)
VIDIOC_DQBUF = _iowr("V", 17, _V4L2Buffer)
VIDIOC_STREAMON = _iow("V", 18, ctypes.c_int)
VIDIOC_STREAMOFF = _iow("V", 19, ctypes.c_int)
VIDIOC_S_PARM = _iowr("V", 22, _V4L2StreamParm)
VIDIOC_S_CTRL = _iowr("V", 28, _V4L2Control)
VIDIOC_ENCODER_CMD = _iowr("V", 77, _V4L2EncoderCmd)


@dataclass
class V4L2H264Packet:
    data: bytes
    flags: int
    is_config: bool = False
    is_keyframe: bool = False


@dataclass
class _InputBufferRef:
    source: Any
    view: memoryview | None
    c_char: Any
    scratch: bytearray | None


class V4L2H264Encoder:
    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        bitrate: str,
        gop: int,
        *,
        device_path: str = DEFAULT_V4L2_ENCODER_DEVICE,
        rgb4_order: str = "rgba",
        debug: bool = False,
    ) -> None:
        self.requested_width = int(width)
        self.requested_height = int(height)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))
        self.bitrate_bps = parse_h264_bitrate_to_bps(bitrate)
        self.gop = max(1, int(gop))
        self.device_path = device_path
        self.rgb4_order = rgb4_order
        self.debug = debug
        self.fd = -1
        self._libc = None
        self._started = False
        self._poller = None
        self._capture_buffers: list[bytearray] = []
        self._capture_refs: list[Any] = []
        self._free_inputs: list[int] = []
        self._inflight_inputs: dict[int, _InputBufferRef] = {}
        self._frame_index = 0
        self._packet_index = 0
        self._input_sizeimage = 0
        self._input_bytesperline = 0
        self._capture_sizeimage = 0
        self._samples: list[tuple[str, float]] = []

    def start(self) -> None:
        if os.name != "posix":
            raise RuntimeError("V4L2 H264 RGB4 encoder is only available on Linux/openpilot devices")

        self._libc = ctypes.CDLL(None, use_errno=True)
        self.fd = os.open(self.device_path, os.O_RDWR | os.O_NONBLOCK)
        try:
            if self.debug:
                self._log_ioctl_abi()
            self._query_capability()
            fmt_out = self._set_format(
                V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                self.width,
                self.height,
                V4L2_PIX_FMT_H264,
                V4L2_COLORSPACE_DEFAULT,
            )
            self._set_fps()
            fmt_in = self._set_format(
                V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                self.width,
                self.height,
                V4L2_PIX_FMT_RGB32,
                V4L2_COLORSPACE_SRGB,
            )
            self._log_formats(fmt_in, fmt_out)
            self._validate_formats(fmt_in, fmt_out)
            self._set_controls()
            self._request_buffers(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_CAPTURE_BUFFER_COUNT)
            self._request_buffers(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_INPUT_BUFFER_COUNT)
            self._stream_on(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
            self._stream_on(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
            self._queue_capture_buffers()
            self._free_inputs = list(range(V4L2_INPUT_BUFFER_COUNT))
            self._poller = select.poll()
            self._poller.register(self.fd, select.POLLIN | select.POLLOUT | select.POLLERR)
            self._started = True
        except Exception:
            self.close()
            raise

    def encode_rgba(self, rgba: Any, width: int, height: int) -> list[V4L2H264Packet]:
        if not self._started:
            raise RuntimeError("V4L2 H264 encoder is not started")
        if int(width) != self.width or int(height) != self.height:
            raise RuntimeError(
                f"V4L2 H264 input size changed from {self.width}x{self.height} "
                f"to {width}x{height}"
            )
        if not self._free_inputs:
            self._drain_until(lambda _packets: bool(self._free_inputs))
        buffer_index = self._free_inputs.pop(0)
        input_ref, input_addr, input_length, bytesused = self._prepare_rgb4_input(rgba)
        timestamp = self._frame_timestamp()
        packets: list[V4L2H264Packet] = []

        profile_stage = self._profile_start()
        self._inflight_inputs[buffer_index] = input_ref
        self._queue_output(buffer_index, input_addr, input_length, bytesused, timestamp)
        self._profile_add("usb_h264.v4l2.queue_output", profile_stage)

        target_index = buffer_index
        saw_frame_packet = False

        def done(next_packets: list[V4L2H264Packet]) -> bool:
            nonlocal saw_frame_packet
            if any(not packet.is_config for packet in next_packets):
                saw_frame_packet = True
            return saw_frame_packet and target_index not in self._inflight_inputs

        packets.extend(self._drain_until(done))
        self._frame_index += 1
        return packets

    def profile_samples(self) -> tuple[tuple[str, float], ...]:
        samples = tuple(self._samples)
        self._samples.clear()
        return samples

    def close(self) -> None:
        if self.fd < 0:
            return
        if self._started:
            try:
                cmd = _V4L2EncoderCmd()
                cmd.cmd = V4L2_ENC_CMD_STOP
                self._xioctl(VIDIOC_ENCODER_CMD, cmd, "VIDIOC_ENCODER_CMD failed")
                self._drain_until(lambda _packets: False, timeout_ms=250, stop_on_eos=True)
            except Exception:
                pass
        for buf_type in (V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE):
            try:
                self._stream_off(buf_type)
            except Exception:
                pass
            try:
                self._request_buffers(buf_type, 0)
            except Exception:
                pass
        try:
            os.close(self.fd)
        finally:
            self.fd = -1
            self._started = False
            self._capture_buffers.clear()
            self._capture_refs.clear()
            self._free_inputs.clear()
            self._inflight_inputs.clear()

    def _query_capability(self) -> None:
        cap = _V4L2Capability()
        self._xioctl(VIDIOC_QUERYCAP, cap, "VIDIOC_QUERYCAP failed")
        driver = bytes(cap.driver).split(b"\0", 1)[0].decode("ascii", errors="replace")
        card = bytes(cap.card).split(b"\0", 1)[0].decode("ascii", errors="replace")
        if self.debug:
            print(f"V4L2 encoder device: driver={driver} card={card}", flush=True)
        if driver != "msm_vidc_driver" or card != "msm_vidc_venc":
            print(
                f"Warning: V4L2 encoder is {driver}/{card}, expected msm_vidc_driver/msm_vidc_venc",
                flush=True,
            )

    def _set_format(
        self,
        buf_type: int,
        width: int,
        height: int,
        pixelformat: int,
        colorspace: int,
    ) -> _V4L2Format:
        fmt = _V4L2Format()
        fmt.type = buf_type
        pix = fmt.fmt.pix_mp
        pix.width = int(width)
        pix.height = int(height)
        pix.pixelformat = int(pixelformat)
        pix.field = V4L2_FIELD_ANY
        pix.colorspace = int(colorspace)
        self._xioctl(VIDIOC_S_FMT, fmt, "VIDIOC_S_FMT failed")
        return fmt

    def _set_fps(self) -> None:
        streamparm = _V4L2StreamParm()
        streamparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
        streamparm.parm.output.timeperframe.numerator = 1
        streamparm.parm.output.timeperframe.denominator = self.fps
        self._xioctl(VIDIOC_S_PARM, streamparm, "VIDIOC_S_PARM failed")

    def _validate_formats(self, fmt_in: _V4L2Format, fmt_out: _V4L2Format) -> None:
        pix_in = fmt_in.fmt.pix_mp
        pix_out = fmt_out.fmt.pix_mp
        if int(pix_in.pixelformat) != V4L2_PIX_FMT_RGB32:
            raise RuntimeError(
                "V4L2 encoder rejected RGB4 input: "
                f"returned {fourcc_to_str(pix_in.pixelformat)}"
            )
        if int(pix_out.pixelformat) != V4L2_PIX_FMT_H264:
            raise RuntimeError(
                "V4L2 encoder rejected H264 output: "
                f"returned {fourcc_to_str(pix_out.pixelformat)}"
            )
        if int(pix_in.width) != self.width or int(pix_in.height) != self.height:
            raise RuntimeError(
                "V4L2 encoder adjusted RGB4 input size from "
                f"{self.width}x{self.height} to {pix_in.width}x{pix_in.height}; "
                "this first RGB4 path requires exact dimensions"
            )
        if int(pix_out.width) != self.width or int(pix_out.height) != self.height:
            raise RuntimeError(
                "V4L2 encoder adjusted H264 output size from "
                f"{self.width}x{self.height} to {pix_out.width}x{pix_out.height}; "
                "this first RGB4 path requires exact dimensions"
            )
        self._input_sizeimage = int(pix_in.plane_fmt[0].sizeimage)
        self._input_bytesperline = int(pix_in.plane_fmt[0].bytesperline)
        self._capture_sizeimage = int(pix_out.plane_fmt[0].sizeimage)
        if self._capture_sizeimage <= 0:
            raise RuntimeError("V4L2 encoder returned zero H264 capture sizeimage")

    def _set_controls(self) -> None:
        p_frames = max(0, self.gop - 1)
        controls = (
            (V4L2_CID_MPEG_VIDEO_BITRATE, self.bitrate_bps, True),
            (V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES, p_frames, True),
            (V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES, 0, True),
            (V4L2_CID_MPEG_VIDEO_HEADER_MODE, V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE, True),
            (V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL, V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR, True),
            (V4L2_CID_MPEG_VIDC_VIDEO_PRIORITY, V4L2_MPEG_VIDC_VIDEO_PRIORITY_REALTIME_DISABLE, False),
            (V4L2_CID_MPEG_VIDC_VIDEO_IDR_PERIOD, 1, True),
            (V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, True),
            (V4L2_CID_MPEG_VIDEO_H264_LEVEL, V4L2_MPEG_VIDEO_H264_LEVEL_UNKNOWN, False),
            (V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE, V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC, False),
            (V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL, V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0, False),
            (V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE, V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED, False),
            (V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA, 0, False),
            (V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA, 0, False),
            (V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE, V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE, False),
        )
        for ctrl_id, value, required in controls:
            control = _V4L2Control()
            control.id = ctrl_id
            control.value = int(value)
            try:
                self._xioctl(VIDIOC_S_CTRL, control, "VIDIOC_S_CTRL failed")
            except OSError:
                if required:
                    raise
                if self.debug:
                    print(f"V4L2 optional control 0x{ctrl_id:x} rejected", flush=True)

    def _request_buffers(self, buf_type: int, count: int) -> None:
        req = _V4L2RequestBuffers()
        req.count = int(count)
        req.type = int(buf_type)
        req.memory = V4L2_MEMORY_USERPTR
        self._xioctl(VIDIOC_REQBUFS, req, "VIDIOC_REQBUFS failed")

    def _stream_on(self, buf_type: int) -> None:
        value = ctypes.c_int(buf_type)
        self._xioctl(VIDIOC_STREAMON, value, "VIDIOC_STREAMON failed")

    def _stream_off(self, buf_type: int) -> None:
        value = ctypes.c_int(buf_type)
        self._xioctl(VIDIOC_STREAMOFF, value, "VIDIOC_STREAMOFF failed")

    def _queue_capture_buffers(self) -> None:
        for index in range(V4L2_CAPTURE_BUFFER_COUNT):
            buffer = bytearray(self._capture_sizeimage)
            c_char = ctypes.c_char.from_buffer(buffer)
            self._capture_buffers.append(buffer)
            self._capture_refs.append(c_char)
            self._queue_capture(index)

    def _queue_capture(self, index: int) -> None:
        buffer = self._capture_buffers[index]
        address = ctypes.addressof(self._capture_refs[index])
        plane = _V4L2Plane()
        plane.bytesused = 0
        plane.length = len(buffer)
        plane.m.userptr = address
        plane.data_offset = 0
        v4l_buf = _V4L2Buffer()
        v4l_buf.index = index
        v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
        v4l_buf.memory = V4L2_MEMORY_USERPTR
        v4l_buf.m.planes = ctypes.pointer(plane)
        v4l_buf.length = 1
        self._xioctl(VIDIOC_QBUF, v4l_buf, "VIDIOC_QBUF failed")

    def _queue_output(
        self,
        index: int,
        address: int,
        length: int,
        bytesused: int,
        timestamp: _TimeVal,
    ) -> None:
        plane = _V4L2Plane()
        plane.bytesused = int(bytesused)
        plane.length = int(length)
        plane.m.userptr = int(address)
        plane.data_offset = 0
        v4l_buf = _V4L2Buffer()
        v4l_buf.index = int(index)
        v4l_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
        v4l_buf.flags = V4L2_BUF_FLAG_TIMESTAMP_COPY
        v4l_buf.timestamp = timestamp
        v4l_buf.memory = V4L2_MEMORY_USERPTR
        v4l_buf.m.planes = ctypes.pointer(plane)
        v4l_buf.length = 1
        self._xioctl(VIDIOC_QBUF, v4l_buf, "VIDIOC_QBUF failed")

    def _dequeue(self, buf_type: int) -> tuple[int, int, int, _TimeVal]:
        plane = _V4L2Plane()
        v4l_buf = _V4L2Buffer()
        v4l_buf.type = int(buf_type)
        v4l_buf.memory = V4L2_MEMORY_USERPTR
        v4l_buf.m.planes = ctypes.pointer(plane)
        v4l_buf.length = 1
        self._xioctl(VIDIOC_DQBUF, v4l_buf, "VIDIOC_DQBUF failed")
        return int(v4l_buf.index), int(plane.bytesused), int(v4l_buf.flags), v4l_buf.timestamp

    def _drain_until(
        self,
        done,
        *,
        timeout_ms: int = V4L2_CAPTURE_TIMEOUT_MS,
        stop_on_eos: bool = False,
    ) -> list[V4L2H264Packet]:
        if self._poller is None:
            raise RuntimeError("V4L2 poller is not initialized")
        deadline = time.monotonic() + timeout_ms / 1000.0
        packets: list[V4L2H264Packet] = []
        while True:
            if done(packets):
                return packets
            remaining_ms = int(max(0.0, deadline - time.monotonic()) * 1000)
            if remaining_ms <= 0:
                if stop_on_eos:
                    return packets
                raise RuntimeError("V4L2 H264 encoder timed out waiting for an encoded frame")
            profile_stage = self._profile_start()
            events = self._poller.poll(remaining_ms)
            self._profile_add("usb_h264.v4l2.poll", profile_stage)
            if not events:
                continue
            for _fd, event_mask in events:
                if event_mask & select.POLLIN:
                    while True:
                        try:
                            packet = self._dequeue_capture_packet()
                        except OSError as exc:
                            if exc.errno == errno.EAGAIN:
                                break
                            raise
                        if packet is None:
                            if stop_on_eos:
                                return packets
                            continue
                        packets.append(packet)
                if event_mask & select.POLLOUT:
                    while True:
                        try:
                            index, _bytesused, _flags, _timestamp = self._dequeue(
                                V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
                            )
                        except OSError as exc:
                            if exc.errno == errno.EAGAIN:
                                break
                            raise
                        self._inflight_inputs.pop(index, None)
                        if index not in self._free_inputs:
                            self._free_inputs.append(index)
                if event_mask & select.POLLERR:
                    raise RuntimeError("V4L2 H264 encoder poll reported POLLERR")

    def _dequeue_capture_packet(self) -> V4L2H264Packet | None:
        profile_stage = self._profile_start()
        index, bytesused, flags, _timestamp = self._dequeue(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        self._profile_add("usb_h264.v4l2.dequeue_capture", profile_stage)
        try:
            if flags & V4L2_QCOM_BUF_FLAG_EOS:
                return None
            if bytesused <= 0:
                return None
            data = bytes(self._capture_buffers[index][:bytesused])
            is_config = bool(flags & V4L2_QCOM_BUF_FLAG_CODECCONFIG)
            is_keyframe = bool(flags & V4L2_BUF_FLAG_KEYFRAME)
            self._packet_index += 1
            if self.debug and (self._packet_index <= 8 or self._packet_index % 30 == 0):
                head = " ".join(f"{byte:02X}" for byte in data[:8])
                kind = "config" if is_config else "frame"
                print(
                    f"V4L2 H264 packet {self._packet_index}: {kind} "
                    f"{len(data)} bytes flags=0x{flags:08x} head={head}",
                    flush=True,
                )
            return V4L2H264Packet(data=data, flags=flags, is_config=is_config, is_keyframe=is_keyframe)
        finally:
            self._queue_capture(index)

    def _prepare_rgb4_input(self, rgba: Any) -> tuple[_InputBufferRef, int, int, int]:
        source_view = memoryview(rgba)
        expected_source_bytes = self.width * self.height * 4
        if len(source_view) < expected_source_bytes:
            raise RuntimeError(
                f"V4L2 RGB4 input buffer is too small: {len(source_view)} < {expected_source_bytes}"
            )
        row_bytes = self.width * 4
        stride = self._input_bytesperline or row_bytes
        required_length = max(self._input_sizeimage, stride * self.height)
        if self.rgb4_order == "rgba" and stride == row_bytes and required_length <= len(source_view):
            return self._buffer_ref(rgba, source_view, None, expected_source_bytes)

        profile_stage = self._profile_start()
        scratch = bytearray(required_length)
        if self.rgb4_order == "rgba":
            self._copy_rows_rgba(source_view, scratch, row_bytes, stride)
        elif self.rgb4_order == "bgra":
            self._copy_rows_bgra(source_view, scratch, row_bytes, stride)
        else:
            raise RuntimeError(f"unsupported RGB4 input order: {self.rgb4_order}")
        self._profile_add(f"usb_h264.v4l2.rgb4_{self.rgb4_order}_copy", profile_stage)
        return self._buffer_ref(scratch, None, scratch, required_length)

    def _buffer_ref(
        self,
        source: Any,
        source_view: memoryview | None,
        scratch: bytearray | None,
        bytesused: int,
    ) -> tuple[_InputBufferRef, int, int, int]:
        try:
            c_char = ctypes.c_char.from_buffer(source)
            view = source_view
        except TypeError:
            scratch = bytearray(source)
            c_char = ctypes.c_char.from_buffer(scratch)
            view = None
        address = ctypes.addressof(c_char)
        length = len(scratch) if scratch is not None else len(source)
        return _InputBufferRef(source=source, view=view, c_char=c_char, scratch=scratch), address, length, bytesused

    def _copy_rows_rgba(self, source: memoryview, target: bytearray, row_bytes: int, stride: int) -> None:
        for row in range(self.height):
            src_start = row * row_bytes
            dst_start = row * stride
            target[dst_start : dst_start + row_bytes] = source[src_start : src_start + row_bytes]

    def _copy_rows_bgra(self, source: memoryview, target: bytearray, row_bytes: int, stride: int) -> None:
        for row in range(self.height):
            src_start = row * row_bytes
            dst_start = row * stride
            dst_end = dst_start + row_bytes
            target[dst_start:dst_end:4] = source[src_start + 2 : src_start + row_bytes : 4]
            target[dst_start + 1 : dst_end : 4] = source[src_start + 1 : src_start + row_bytes : 4]
            target[dst_start + 2 : dst_end : 4] = source[src_start : src_start + row_bytes : 4]
            target[dst_start + 3 : dst_end : 4] = source[src_start + 3 : src_start + row_bytes : 4]

    def _frame_timestamp(self) -> _TimeVal:
        usec = int(self._frame_index * 1_000_000 / self.fps)
        timestamp = _TimeVal()
        timestamp.tv_sec = usec // 1_000_000
        timestamp.tv_usec = usec % 1_000_000
        return timestamp

    def _log_formats(self, fmt_in: _V4L2Format, fmt_out: _V4L2Format) -> None:
        pix_in = fmt_in.fmt.pix_mp
        pix_out = fmt_out.fmt.pix_mp
        print(
            "V4L2 H264 RGB4 format: "
            f"in={fourcc_to_str(pix_in.pixelformat)} "
            f"{pix_in.width}x{pix_in.height} "
            f"stride={pix_in.plane_fmt[0].bytesperline} "
            f"sizeimage={pix_in.plane_fmt[0].sizeimage}; "
            f"out={fourcc_to_str(pix_out.pixelformat)} "
            f"{pix_out.width}x{pix_out.height} "
            f"sizeimage={pix_out.plane_fmt[0].sizeimage}; "
            f"order={self.rgb4_order}",
            flush=True,
        )

    def _log_ioctl_abi(self) -> None:
        print(
            "V4L2 ctypes ABI: "
            f"sizeof(format)={ctypes.sizeof(_V4L2Format)} "
            f"format.fmt.offset={_V4L2Format.fmt.offset} "
            f"sizeof(buffer)={ctypes.sizeof(_V4L2Buffer)} "
            f"sizeof(plane)={ctypes.sizeof(_V4L2Plane)} "
            f"VIDIOC_S_FMT=0x{VIDIOC_S_FMT:08x}",
            flush=True,
        )

    def _xioctl(self, request: int, arg: Any, message: str) -> None:
        if self._libc is None:
            raise RuntimeError("libc is not initialized")
        while True:
            ret = self._libc.ioctl(ctypes.c_int(self.fd), ctypes.c_ulong(request), ctypes.byref(arg))
            if ret == 0:
                return
            err = ctypes.get_errno()
            if err == errno.EINTR:
                continue
            raise OSError(err, f"{message}: {os.strerror(err)}")

    def _profile_start(self) -> float:
        return time.perf_counter()

    def _profile_add(self, name: str, start_time: float) -> None:
        self._samples.append((name, (time.perf_counter() - start_time) * 1000.0))
