from __future__ import annotations

from collections import deque
import ctypes
import os
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time
from typing import Any

from cluster_usb_display import TuringUsbDisplay


OPENPILOT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_H264_LIBRARY = OPENPILOT_ROOT / "system" / "loggerd" / "libcluster_h264_encoder_bridge.so"
DEFAULT_H264_HELPER = OPENPILOT_ROOT / "system" / "loggerd" / "cluster_h264_encoder_cli"
DEFAULT_H264_DEVICE = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1"
DEFAULT_H264_FFMPEG = "ffmpeg"
DEFAULT_H264_FFMPEG_ENCODER = "libx264"
DEFAULT_H264_SLICE_MAX_BYTES = 4096
DEFAULT_H264_PACKETIZE = "auto"

NATIVE_INPUT_FORMATS = {"auto": 0, "rgb4": 1, "nv12": 2}
NATIVE_RGB4_LAYOUTS = {"axrgb": 0, "rgba": 1, "bgra": 2}
H264_AUD_NAL = b"\x00\x00\x00\x01\x09\xf0"
H264_NAL_NAMES = {
    1: "P",
    5: "IDR",
    6: "SEI",
    7: "SPS",
    8: "PPS",
    9: "AUD",
}


class _H264BitReader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.bitpos = 0

    def read_bit(self) -> int:
        if self.bitpos >= len(self.data) * 8:
            raise ValueError("SPS ended unexpectedly")
        value = (self.data[self.bitpos // 8] >> (7 - (self.bitpos % 8))) & 1
        self.bitpos += 1
        return value

    def read_bits(self, count: int) -> int:
        value = 0
        for _ in range(count):
            value = (value << 1) | self.read_bit()
        return value

    def read_ue(self) -> int:
        zeros = 0
        while self.read_bit() == 0:
            zeros += 1
            if zeros > 31:
                raise ValueError("SPS Exp-Golomb value is too large")
        value = 1
        for _ in range(zeros):
            value = (value << 1) | self.read_bit()
        return value - 1

    def read_se(self) -> int:
        value = self.read_ue()
        sign = -1 if (value & 1) == 0 else 1
        return sign * ((value + 1) // 2)


class _H264BitWriter:
    def __init__(self) -> None:
        self.bits: list[int] = []

    def write_bit(self, value: int) -> None:
        self.bits.append(1 if value else 0)

    def write_bits(self, value: int, count: int) -> None:
        for shift in range(count - 1, -1, -1):
            self.write_bit((value >> shift) & 1)

    def write_ue(self, value: int) -> None:
        code_num = value + 1
        bit_length = code_num.bit_length()
        for _ in range(bit_length - 1):
            self.write_bit(0)
        self.write_bits(code_num, bit_length)

    def copy_bits(self, data: bytes, start_bit: int, end_bit: int) -> None:
        for bitpos in range(start_bit, end_bit):
            self.write_bit((data[bitpos // 8] >> (7 - (bitpos % 8))) & 1)

    def to_bytes(self) -> bytes:
        while len(self.bits) % 8:
            self.bits.append(0)
        out = bytearray(len(self.bits) // 8)
        for bitpos, bit in enumerate(self.bits):
            if bit:
                out[bitpos // 8] |= 1 << (7 - (bitpos % 8))
        return bytes(out)


NativePacketCallback = ctypes.CFUNCTYPE(
    None,
    ctypes.c_void_p,
    ctypes.c_size_t,
    ctypes.c_uint32,
    ctypes.c_uint64,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_void_p,
)


def _h264_start_code_len(data: bytes, index: int) -> int:
    if index + 4 <= len(data) and data[index:index + 4] == b"\x00\x00\x00\x01":
        return 4
    if index + 3 <= len(data) and data[index:index + 3] == b"\x00\x00\x01":
        return 3
    return 0


def _h264_start_codes(data: bytes) -> list[tuple[int, int]]:
    positions: list[tuple[int, int]] = []
    index = 0
    while index + 3 <= len(data):
        start_len = _h264_start_code_len(data, index)
        if start_len:
            positions.append((index, start_len))
            index += start_len
        else:
            index += 1
    return positions


def _h264_nals(data: bytes) -> list[tuple[int, int, int]]:
    starts = _h264_start_codes(data)
    nals: list[tuple[int, int, int]] = []
    for index, (start, start_len) in enumerate(starts):
        nal_start = start + start_len
        nal_end = starts[index + 1][0] if index + 1 < len(starts) else len(data)
        while nal_end > nal_start and data[nal_end - 1] == 0:
            nal_end -= 1
        if nal_start < nal_end:
            nals.append((start, nal_start, nal_end))
    return nals


def _h264_byte_stream_units(data: bytes) -> list[bytes]:
    starts = _h264_start_codes(data)
    if not starts:
        return []
    units: list[bytes] = []
    for index, (start, _) in enumerate(starts):
        end = starts[index + 1][0] if index + 1 < len(starts) else len(data)
        if start < end:
            units.append(data[start:end])
    return units


def _h264_unescape_rbsp(data: bytes) -> bytes:
    out = bytearray()
    zeros = 0
    for value in data:
        if zeros >= 2 and value == 0x03:
            zeros = 0
            continue
        out.append(value)
        if value == 0:
            zeros += 1
        else:
            zeros = 0
    return bytes(out)


def _h264_escape_rbsp(data: bytes) -> bytes:
    out = bytearray()
    zeros = 0
    for value in data:
        if zeros >= 2 and value <= 0x03:
            out.append(0x03)
            zeros = 0
        out.append(value)
        if value == 0:
            zeros += 1
        else:
            zeros = 0
    return bytes(out)


def _h264_rbsp_stop_bitpos(data: bytes) -> int:
    for bitpos in range(len(data) * 8 - 1, -1, -1):
        if (data[bitpos // 8] >> (7 - (bitpos % 8))) & 1:
            return bitpos
    raise ValueError("SPS RBSP stop bit not found")


def _h264_skip_scaling_list(reader: _H264BitReader, size: int) -> None:
    last_scale = 8
    next_scale = 8
    for _ in range(size):
        if next_scale:
            delta_scale = reader.read_se()
            next_scale = (last_scale + delta_scale + 256) % 256
        if next_scale:
            last_scale = next_scale


def _h264_read_sps_to_crop(reader: _H264BitReader) -> dict[str, int]:
    profile_idc = reader.read_bits(8)
    constraint_flags = reader.read_bits(8)
    level_idc = reader.read_bits(8)
    reader.read_ue()

    chroma_format_idc = 1
    separate_colour_plane_flag = 0
    high_profiles = {
        100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135,
    }
    if profile_idc in high_profiles:
        chroma_format_idc = reader.read_ue()
        if chroma_format_idc == 3:
            separate_colour_plane_flag = reader.read_bit()
        reader.read_ue()
        reader.read_ue()
        reader.read_bit()
        if reader.read_bit():
            scaling_count = 8 if chroma_format_idc != 3 else 12
            for index in range(scaling_count):
                if reader.read_bit():
                    _h264_skip_scaling_list(reader, 16 if index < 6 else 64)

    reader.read_ue()
    pic_order_cnt_type = reader.read_ue()
    if pic_order_cnt_type == 0:
        reader.read_ue()
    elif pic_order_cnt_type == 1:
        reader.read_bit()
        reader.read_se()
        reader.read_se()
        for _ in range(reader.read_ue()):
            reader.read_se()

    reader.read_ue()
    reader.read_bit()
    pic_width_in_mbs_minus1 = reader.read_ue()
    pic_height_in_map_units_minus1 = reader.read_ue()
    frame_mbs_only_flag = reader.read_bit()
    if not frame_mbs_only_flag:
        reader.read_bit()
    reader.read_bit()
    crop_flag_bitpos = reader.bitpos
    frame_cropping_flag = reader.read_bit()
    crop_left = crop_right = crop_top = crop_bottom = 0
    if frame_cropping_flag:
        crop_left = reader.read_ue()
        crop_right = reader.read_ue()
        crop_top = reader.read_ue()
        crop_bottom = reader.read_ue()

    return {
        "profile_idc": profile_idc,
        "constraint_flags": constraint_flags,
        "level_idc": level_idc,
        "chroma_format_idc": 0 if separate_colour_plane_flag else chroma_format_idc,
        "pic_width_in_mbs_minus1": pic_width_in_mbs_minus1,
        "pic_height_in_map_units_minus1": pic_height_in_map_units_minus1,
        "frame_mbs_only_flag": frame_mbs_only_flag,
        "crop_flag_bitpos": crop_flag_bitpos,
        "after_crop_bitpos": reader.bitpos,
        "crop_left": crop_left,
        "crop_right": crop_right,
        "crop_top": crop_top,
        "crop_bottom": crop_bottom,
    }


def _h264_crop_units(chroma_format_idc: int, frame_mbs_only_flag: int) -> tuple[int, int]:
    if chroma_format_idc == 0:
        return 1, 2 - frame_mbs_only_flag
    if chroma_format_idc == 1:
        return 2, 2 * (2 - frame_mbs_only_flag)
    if chroma_format_idc == 2:
        return 2, 2 - frame_mbs_only_flag
    return 1, 2 - frame_mbs_only_flag


def _h264_sps_info(nal: bytes) -> str:
    if len(nal) < 5 or (nal[0] & 0x1F) != 7:
        return ""
    try:
        rbsp = _h264_unescape_rbsp(nal[1:])
        reader = _H264BitReader(rbsp)
        info = _h264_read_sps_to_crop(reader)
        coded_width = (info["pic_width_in_mbs_minus1"] + 1) * 16
        coded_height = (info["pic_height_in_map_units_minus1"] + 1) * 16 * (2 - info["frame_mbs_only_flag"])
        crop_unit_x, crop_unit_y = _h264_crop_units(info["chroma_format_idc"], info["frame_mbs_only_flag"])
        display_width = coded_width - (info["crop_left"] + info["crop_right"]) * crop_unit_x
        display_height = coded_height - (info["crop_top"] + info["crop_bottom"]) * crop_unit_y
        vui_text = ""
        if info["after_crop_bitpos"] < len(rbsp) * 8:
            vui_reader = _H264BitReader(rbsp)
            vui_reader.bitpos = info["after_crop_bitpos"]
            vui_text = " vui=1" if vui_reader.read_bit() else " vui=0"
        return (
            f"profile=0x{info['profile_idc']:02X} constraints=0x{info['constraint_flags']:02X} "
            f"level=0x{info['level_idc']:02X} coded={coded_width}x{coded_height} "
            f"display={display_width}x{display_height} "
            f"crop={info['crop_left']},{info['crop_right']},{info['crop_top']},{info['crop_bottom']}"
            f"{vui_text}"
        )
    except Exception:
        return ""


def _h264_packet_summary(data: bytes, max_nals: int = 6) -> str:
    parts: list[str] = []
    sps_parts: list[str] = []
    nals = _h264_nals(data)
    largest_name = ""
    largest_size = 0
    for _, nal_start, nal_end in nals[:max_nals]:
        nal = data[nal_start:nal_end]
        nal_type = nal[0] & 0x1F
        name = H264_NAL_NAMES.get(nal_type, f"NAL{nal_type}")
        parts.append(f"{name}:{len(nal)}")
        if nal_type == 7:
            sps_info = _h264_sps_info(nal)
            if sps_info:
                sps_parts.append(sps_info)
    for _, nal_start, nal_end in nals:
        nal = data[nal_start:nal_end]
        nal_size = len(nal)
        if nal_size > largest_size:
            largest_size = nal_size
            largest_name = H264_NAL_NAMES.get(nal[0] & 0x1F, f"NAL{nal[0] & 0x1F}")
    if len(nals) > max_nals:
        parts.append(f"+{len(nals) - max_nals}")
    if not parts:
        return "nals=none"
    summary = "nals=" + ",".join(parts)
    summary += f" nal_count={len(nals)} max={largest_name}:{largest_size}"
    if sps_parts:
        summary += " " + " ".join(f"sps[{part}]" for part in sps_parts)
    return summary


def _patch_h264_sps_constraints(data: bytes) -> tuple[bytes, bool]:
    patched = False
    mutable: bytearray | None = None
    for _, nal_start, nal_end in _h264_nals(data):
        nal = data[nal_start:nal_end]
        if len(nal) < 4 or (nal[0] & 0x1F) != 7 or nal[1] != 0x42:
            continue
        constraints_index = nal_start + 2
        constraints = data[constraints_index] if mutable is None else mutable[constraints_index]
        next_constraints = constraints | 0x40
        if next_constraints == constraints:
            continue
        if mutable is None:
            mutable = bytearray(data)
        mutable[constraints_index] = next_constraints
        patched = True
    if mutable is None:
        return data, False
    return bytes(mutable), patched


def _patch_h264_sps_crop(data: bytes, width: int, height: int) -> tuple[bytes, bool, str]:
    out = bytearray()
    last = 0
    patched = False
    patched_info = ""

    for _, nal_start, nal_end in _h264_nals(data):
        nal = data[nal_start:nal_end]
        if len(nal) < 5 or (nal[0] & 0x1F) != 7:
            continue

        try:
            rbsp = _h264_unescape_rbsp(nal[1:])
            reader = _H264BitReader(rbsp)
            info = _h264_read_sps_to_crop(reader)
            coded_width = (info["pic_width_in_mbs_minus1"] + 1) * 16
            coded_height = (info["pic_height_in_map_units_minus1"] + 1) * 16 * (2 - info["frame_mbs_only_flag"])
            crop_unit_x, crop_unit_y = _h264_crop_units(info["chroma_format_idc"], info["frame_mbs_only_flag"])
            if width > coded_width or height > coded_height:
                continue
            crop_right_pixels = coded_width - width
            crop_bottom_pixels = coded_height - height
            if crop_right_pixels % crop_unit_x or crop_bottom_pixels % crop_unit_y:
                continue
            crop_left = 0
            crop_top = 0
            crop_right = crop_right_pixels // crop_unit_x
            crop_bottom = crop_bottom_pixels // crop_unit_y
            if (
                info["crop_left"] == crop_left
                and info["crop_right"] == crop_right
                and info["crop_top"] == crop_top
                and info["crop_bottom"] == crop_bottom
            ):
                continue

            writer = _H264BitWriter()
            writer.copy_bits(rbsp, 0, info["crop_flag_bitpos"])
            if crop_left or crop_right or crop_top or crop_bottom:
                writer.write_bit(1)
                writer.write_ue(crop_left)
                writer.write_ue(crop_right)
                writer.write_ue(crop_top)
                writer.write_ue(crop_bottom)
            else:
                writer.write_bit(0)
            stop_bitpos = _h264_rbsp_stop_bitpos(rbsp)
            writer.copy_bits(rbsp, info["after_crop_bitpos"], stop_bitpos)
            writer.write_bit(1)
            patched_nal = bytes([nal[0]]) + _h264_escape_rbsp(writer.to_bytes())

            out.extend(data[last:nal_start])
            out.extend(patched_nal)
            last = nal_end
            patched = True
            patched_info = (
                f"coded={coded_width}x{coded_height} display={width}x{height} "
                f"crop={crop_left},{crop_right},{crop_top},{crop_bottom}"
            )
        except Exception:
            continue

    if not patched:
        return data, False, ""
    out.extend(data[last:])
    return bytes(out), True, patched_info


def _write_h264_vui_timing(writer: _H264BitWriter, fps: int) -> None:
    writer.write_bit(0)  # aspect_ratio_info_present_flag
    writer.write_bit(0)  # overscan_info_present_flag
    writer.write_bit(0)  # video_signal_type_present_flag
    writer.write_bit(0)  # chroma_loc_info_present_flag
    writer.write_bit(1)  # timing_info_present_flag
    writer.write_bits(1, 32)  # num_units_in_tick
    writer.write_bits(max(2, int(fps) * 2), 32)  # time_scale
    writer.write_bit(1)  # fixed_frame_rate_flag
    writer.write_bit(0)  # nal_hrd_parameters_present_flag
    writer.write_bit(0)  # vcl_hrd_parameters_present_flag
    writer.write_bit(0)  # pic_struct_present_flag
    writer.write_bit(0)  # bitstream_restriction_flag


def _patch_h264_sps_vui_timing(data: bytes, fps: int) -> tuple[bytes, bool, str]:
    out = bytearray()
    last = 0
    patched = False
    patched_info = ""

    for _, nal_start, nal_end in _h264_nals(data):
        nal = data[nal_start:nal_end]
        if len(nal) < 5 or (nal[0] & 0x1F) != 7:
            continue

        try:
            rbsp = _h264_unescape_rbsp(nal[1:])
            reader = _H264BitReader(rbsp)
            info = _h264_read_sps_to_crop(reader)
            vui_flag_bitpos = info["after_crop_bitpos"]
            if vui_flag_bitpos >= len(rbsp) * 8:
                continue
            existing_vui = (rbsp[vui_flag_bitpos // 8] >> (7 - (vui_flag_bitpos % 8))) & 1
            if existing_vui:
                continue

            writer = _H264BitWriter()
            writer.copy_bits(rbsp, 0, vui_flag_bitpos)
            writer.write_bit(1)
            _write_h264_vui_timing(writer, fps)
            writer.write_bit(1)
            patched_nal = bytes([nal[0]]) + _h264_escape_rbsp(writer.to_bytes())

            out.extend(data[last:nal_start])
            out.extend(patched_nal)
            last = nal_end
            patched = True
            patched_info = f"fps={fps} num_units_in_tick=1 time_scale={max(2, int(fps) * 2)}"
        except Exception:
            continue

    if not patched:
        return data, False, ""
    out.extend(data[last:])
    return bytes(out), True, patched_info


class H264UsbPipeline:
    def __init__(
        self,
        usb_display: TuringUsbDisplay,
        width: int,
        height: int,
        fps: int,
        bitrate: str,
        gop: int,
        backend: str,
        library_path: str,
        helper_path: str,
        ffmpeg_path: str,
        ffmpeg_encoder: str,
        device_path: str,
        input_format: str,
        rgb4_layout: str,
        slice_max_bytes: int,
        qp: int,
        packetize: str,
        requested_chunk_size: int,
        wait_for_ack: bool,
        soft_ack: bool,
        insert_aud: bool,
        patch_sps_constraints: bool,
        patch_sps_crop: bool,
        patch_sps_vui: bool,
        dump_path: str,
        debug: bool,
    ) -> None:
        self.usb_display = usb_display
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))
        self.bitrate = bitrate
        self.gop = max(1, int(gop))
        self.backend_request = backend
        self.backend_name = backend
        self.library_path = library_path
        self.helper_path = helper_path
        self.ffmpeg_path = ffmpeg_path
        self.ffmpeg_encoder_request = ffmpeg_encoder
        self.ffmpeg_encoder_name = ffmpeg_encoder
        self.ffmpeg_muxer_name = ""
        self.device_path = device_path
        self.input_format = input_format
        self.rgb4_layout = rgb4_layout
        self.slice_max_bytes = max(0, int(slice_max_bytes))
        self.qp = int(qp)
        self.packetize_request = packetize
        self.requested_chunk_size = max(0, int(requested_chunk_size))
        self.wait_for_ack = wait_for_ack
        self.soft_ack = soft_ack
        self.insert_aud = insert_aud
        self.patch_sps_constraints = patch_sps_constraints
        self.patch_sps_crop = patch_sps_crop
        self.patch_sps_vui = patch_sps_vui
        self.dump_path = dump_path
        self._dump_file = None
        self.debug = debug
        self._sps_patch_logged = False
        self._sps_crop_patch_logged = False
        self._sps_vui_patch_logged = False
        self.chunk_size = 0
        self._chunks_sent = 0
        self._proc: subprocess.Popen[bytes] | None = None
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._sender_thread: threading.Thread | None = None
        self._native_lib: ctypes.CDLL | None = None
        self._native_handle: int | None = None
        self._native_callback: Any = None
        self._native_frame_index = 0
        self._packet_queue: queue.Queue[Any] | None = None
        self._condition = threading.Condition()
        self._closing = False
        self._stream_started = False
        self._error: BaseException | None = None
        self._samples: list[tuple[str, float]] = []
        self._stderr_tail: deque[str] = deque(maxlen=20)

    def start(self) -> None:
        if self.backend_request == "ffmpeg":
            self._start_ffmpeg()
            return

        if self.backend_request in ("auto", "native"):
            try:
                self._start_native()
                return
            except Exception as exc:
                if self.backend_request == "native":
                    raise
                print(f"Warning: native H264 encoder unavailable, falling back to helper: {exc}", flush=True)
                self._close_native()

        if self.backend_request not in ("auto", "helper"):
            raise RuntimeError(f"unsupported H264 backend: {self.backend_request}")
        self._start_helper()

    def _start_native(self) -> None:
        library = self._resolve_library()
        lib = ctypes.CDLL(library)
        self._configure_native_library(lib)

        bitrate_bps = self._parse_bitrate_bps(self.bitrate)
        handle = lib.cluster_h264_encoder_bridge_create(
            self.width,
            self.height,
            self.fps,
            bitrate_bps,
            self.gop,
            self.device_path.encode("utf-8"),
            NATIVE_INPUT_FORMATS[self.input_format],
            NATIVE_RGB4_LAYOUTS[self.rgb4_layout],
            1 if self.debug else 0,
        )
        if not handle:
            raise RuntimeError("native H264 encoder bridge allocation failed")
        self._native_lib = lib
        self._native_handle = handle
        self._native_callback = NativePacketCallback(self._native_packet_callback)
        self._set_native_slice_max_bytes(lib, handle)
        self._set_native_qp(lib, handle)

        if lib.cluster_h264_encoder_bridge_open(handle) != 0:
            raise RuntimeError(self._native_error_text("native H264 encoder open failed"))

        try:
            self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
            self._stream_started = True
            self._open_dump_file()
        except Exception:
            self._close_native()
            raise

        self._packet_queue = queue.Queue(maxsize=64)
        self._sender_thread = threading.Thread(
            target=self._send_queued_packets,
            name="cluster-usb-h264-native-send",
            daemon=True,
        )
        self._sender_thread.start()

        self.backend_name = "native"
        input_name = self._native_input_format_name()
        input_stride = lib.cluster_h264_encoder_bridge_input_stride(handle)
        print(
            "Starting H264 USB native hardware encoder: "
            f"{self.width}x{self.height}@{self.fps} "
            f"bitrate={bitrate_bps} gop={self.gop} "
            f"slice_max={self.slice_max_bytes} qp={self.qp} "
            f"packetize={self._packetize_mode('native')} "
            f"input={input_name or self.input_format} stride={input_stride} "
            f"rgb4_layout={self.rgb4_layout} device={self.device_path} "
            f"chunk_ack={'soft' if self.wait_for_ack and self.soft_ack else ('on' if self.wait_for_ack else 'off')} "
            f"aud={'on' if self.insert_aud else 'off'} "
            f"sps_patch={'on' if self.patch_sps_constraints else 'off'} "
            f"sps_crop_patch={'on' if self.patch_sps_crop else 'off'} "
            f"sps_vui_patch={'on' if self.patch_sps_vui else 'off'}",
            flush=True,
        )

    def _start_helper(self) -> None:
        helper = self._resolve_helper_executable()
        command = self._helper_command(helper)
        self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
        self._stream_started = True
        self._open_dump_file()
        self.backend_name = "helper"
        print(
            "Starting H264 USB helper hardware encoder: "
            f"{self.width}x{self.height}@{self.fps} "
            f"bitrate={self.bitrate} gop={self.gop} "
            f"slice_max={self.slice_max_bytes} qp={self.qp} "
            f"packetize={self._packetize_mode('helper')} "
            f"input={self.input_format} rgb4_layout={self.rgb4_layout} "
            f"device={self.device_path} "
            f"chunk_ack={'soft' if self.wait_for_ack and self.soft_ack else ('on' if self.wait_for_ack else 'off')} "
            f"aud={'native-only' if self.insert_aud else 'off'} "
            f"sps_patch={'on' if self.patch_sps_constraints else 'off'} "
            f"sps_crop_patch={'on' if self.patch_sps_crop else 'off'} "
            f"sps_vui_patch={'on' if self.patch_sps_vui else 'off'}",
            flush=True,
        )
        if self.debug:
            print(f"H264 helper encoder command: {' '.join(command)}", flush=True)
        try:
            self._proc = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except Exception:
            self.usb_display.stop_h264_stream()
            self._stream_started = False
            self._close_dump_file()
            raise

        self._stdout_thread = threading.Thread(
            target=self._read_stdout,
            name="cluster-usb-h264-out",
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=self._read_stderr,
            name="cluster-usb-h264-err",
            daemon=True,
        )
        self._stdout_thread.start()
        self._stderr_thread.start()

    def _start_ffmpeg(self) -> None:
        ffmpeg = self._ffmpeg_executable()
        self.ffmpeg_encoder_name = self._resolve_ffmpeg_encoder(ffmpeg)
        self.ffmpeg_muxer_name = self._resolve_ffmpeg_output_muxer(ffmpeg)
        command = self._ffmpeg_command(ffmpeg, self.ffmpeg_encoder_name, self.ffmpeg_muxer_name)
        self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
        self._stream_started = True
        self._open_dump_file()
        self.backend_name = "ffmpeg"
        print(
            "Starting H264 USB ffmpeg encoder: "
            f"{self.ffmpeg_encoder_name} {self.width}x{self.height}@{self.fps} "
            f"bitrate={self.bitrate} gop={self.gop} muxer={self.ffmpeg_muxer_name} "
            f"packetize={self._packetize_mode('ffmpeg')} "
            f"chunk_ack={'soft' if self.wait_for_ack and self.soft_ack else ('on' if self.wait_for_ack else 'off')}",
            flush=True,
        )
        if self.debug:
            print(f"H264 ffmpeg command: {' '.join(command)}", flush=True)
        try:
            self._proc = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except Exception:
            self.usb_display.stop_h264_stream()
            self._stream_started = False
            self._close_dump_file()
            raise

        self._stdout_thread = threading.Thread(
            target=self._read_stdout,
            name="cluster-usb-h264-ffmpeg-out",
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=self._read_stderr,
            name="cluster-usb-h264-ffmpeg-err",
            daemon=True,
        )
        self._stdout_thread.start()
        self._stderr_thread.start()

    def submit_rgba(self, rgba: Any, width: int, height: int) -> None:
        self.check_error()
        if width != self.width or height != self.height:
            raise RuntimeError(
                f"H264 encoder input size changed from {self.width}x{self.height} "
                f"to {width}x{height}"
            )
        if self._closing:
            raise RuntimeError("H264 USB pipeline is closing")

        if self._native_handle is not None:
            self._submit_rgba_native(rgba)
            return

        proc = self._proc
        if proc is None or proc.stdin is None:
            raise RuntimeError("H264 USB pipeline is not started")

        profile_stage = time.perf_counter()
        try:
            self._write_all(proc.stdin.fileno(), rgba, self.width * self.height * 4)
        except BrokenPipeError as exc:
            self._set_error(exc)
            raise RuntimeError(self._error_text(f"H264 {self.backend_name} encoder pipe closed")) from exc
        except Exception as exc:
            self._set_error(exc)
            raise
        self._add_sample("usb_h264.write_rgba", profile_stage)
        self.check_error()

    def _submit_rgba_native(self, rgba: Any) -> None:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None or self._native_callback is None:
            raise RuntimeError("native H264 USB pipeline is not started")

        byte_count = self.width * self.height * 4
        view = memoryview(rgba)
        if view.nbytes < byte_count:
            raise RuntimeError(
                f"H264 encoder RGBA input is {view.nbytes} bytes, expected at least {byte_count}"
            )
        if not view.contiguous:
            raise RuntimeError("H264 encoder RGBA input must be contiguous")

        profile_stage = time.perf_counter()
        try:
            data_ptr = ctypes.addressof(ctypes.c_uint8.from_buffer(view))
        except TypeError as exc:
            raise RuntimeError("H264 encoder RGBA input must expose a writable buffer") from exc

        timestamp_us = self._native_frame_index * 1000000 // self.fps
        result = lib.cluster_h264_encoder_bridge_encode_rgba(
            handle,
            ctypes.c_void_p(data_ptr),
            byte_count,
            timestamp_us,
            self._native_callback,
            None,
        )
        if result != 0:
            raise RuntimeError(self._native_error_text("native H264 encode failed"))
        self._native_frame_index += 1
        self._add_sample("usb_h264.native_encode_rgba", profile_stage)
        self.check_error()

    def profile_samples(self) -> tuple[tuple[str, float], ...]:
        with self._condition:
            samples = tuple(self._samples)
            self._samples.clear()
        return samples

    def check_error(self) -> None:
        with self._condition:
            error = self._error
            closing = self._closing
        if error is not None:
            raise RuntimeError(self._error_text("H264 USB pipeline failed", error)) from error

        proc = self._proc
        if proc is not None and not closing:
            return_code = proc.poll()
            if return_code is not None:
                raise RuntimeError(self._error_text(f"H264 {self.backend_name} encoder exited with code {return_code}"))

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify_all()

        if self._native_handle is not None:
            self._drain_native()
            self._close_native()

        proc = self._proc
        if proc is not None:
            if proc.stdin is not None:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2.0)

        if self._stdout_thread is not None:
            self._stdout_thread.join(timeout=3.0)
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=1.0)

        packet_queue = self._packet_queue
        if packet_queue is not None:
            try:
                packet_queue.put(None, timeout=1.0)
            except queue.Full:
                pass
        if self._sender_thread is not None:
            self._sender_thread.join(timeout=3.0)

        if self._stream_started:
            try:
                self.usb_display.stop_h264_stream()
            except Exception as exc:
                print(f"Warning: TURZX H264 stop command skipped: {exc}", flush=True)
            self._stream_started = False
        self._close_dump_file()

    def _open_dump_file(self) -> None:
        if not self.dump_path:
            return
        dump_path = Path(self.dump_path)
        if not dump_path.is_absolute():
            dump_path = OPENPILOT_ROOT / dump_path
        dump_path.parent.mkdir(parents=True, exist_ok=True)
        self._dump_file = dump_path.open("wb")
        print(f"Dumping H264 USB bytestream to {dump_path}", flush=True)

    def _write_dump(self, chunk: bytes) -> None:
        if self._dump_file is None:
            return
        self._dump_file.write(chunk)
        self._dump_file.flush()

    def _prepare_hardware_packet(self, packet: bytes) -> bytes:
        if self.patch_sps_constraints:
            packet, patched = _patch_h264_sps_constraints(packet)
            if patched and self.debug and not self._sps_patch_logged:
                print(
                    "H264 hardware SPS patched: baseline constraint flags OR 0x40 to match libx264 constrained-baseline",
                    flush=True,
                )
                self._sps_patch_logged = True
        if self.patch_sps_crop:
            packet, patched, crop_info = _patch_h264_sps_crop(packet, self.width, self.height)
            if patched and self.debug and not self._sps_crop_patch_logged:
                print(
                    f"H264 hardware SPS crop patched: {crop_info}",
                    flush=True,
                )
                self._sps_crop_patch_logged = True
        if self.patch_sps_vui:
            packet, patched, vui_info = _patch_h264_sps_vui_timing(packet, self.fps)
            if patched and self.debug and not self._sps_vui_patch_logged:
                print(
                    f"H264 hardware SPS VUI timing patched: {vui_info}",
                    flush=True,
                )
                self._sps_vui_patch_logged = True
        if self.insert_aud:
            packet = H264_AUD_NAL + packet
        return packet

    def _packetize_mode(self, source: str) -> str:
        if self.packetize_request != "auto":
            return self.packetize_request
        if source in ("native", "helper"):
            return "nal-groups"
        return "access-unit"

    def _packetize_h264_for_usb(self, packet: bytes, chunk_size: int, *, source: str) -> list[bytes]:
        chunk_size = max(1, chunk_size)
        mode = self._packetize_mode(source)
        if mode == "access-unit":
            return [packet[offset:offset + chunk_size] for offset in range(0, len(packet), chunk_size)]

        units = _h264_byte_stream_units(packet)
        if not units:
            return [packet[offset:offset + chunk_size] for offset in range(0, len(packet), chunk_size)]

        if mode == "nal":
            chunks: list[bytes] = []
            for unit in units:
                chunks.extend(unit[offset:offset + chunk_size] for offset in range(0, len(unit), chunk_size))
            return chunks

        if mode != "nal-groups":
            raise RuntimeError(f"unsupported H264 USB packetize mode: {mode}")

        chunks: list[bytes] = []
        current = bytearray()
        for unit in units:
            if len(unit) > chunk_size:
                if current:
                    chunks.append(bytes(current))
                    current.clear()
                chunks.extend(unit[offset:offset + chunk_size] for offset in range(0, len(unit), chunk_size))
                continue
            if current and len(current) + len(unit) > chunk_size:
                chunks.append(bytes(current))
                current.clear()
            current.extend(unit)
        if current:
            chunks.append(bytes(current))
        return chunks

    def _close_dump_file(self) -> None:
        if self._dump_file is None:
            return
        try:
            self._dump_file.close()
        except Exception:
            pass
        self._dump_file = None

    def _resolve_library(self) -> str:
        path = Path(self.library_path)
        candidates = [path]
        if not path.is_absolute():
            candidates.append(OPENPILOT_ROOT / path)
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        raise RuntimeError(
            f"native H264 encoder library not found: {self.library_path}. "
            "Build system/loggerd/libcluster_h264_encoder_bridge.so first."
        )

    def _resolve_helper_executable(self) -> str:
        path = Path(self.helper_path)
        candidates = [path]
        if not path.is_absolute():
            candidates.append(OPENPILOT_ROOT / path)
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        raise RuntimeError(
            f"H264 hardware encoder helper not found: {self.helper_path}. "
            "Build system/loggerd/cluster_h264_encoder_cli first."
        )

    def _helper_command(self, helper: str) -> list[str]:
        command = [
            helper,
            "--width",
            str(self.width),
            "--height",
            str(self.height),
            "--fps",
            str(self.fps),
            "--bitrate",
            self.bitrate,
            "--gop",
            str(self.gop),
            "--device",
            self.device_path,
            "--input-format",
            self.input_format,
            "--rgb4-layout",
            self.rgb4_layout,
            "--slice-max-bytes",
            str(self.slice_max_bytes),
            "--qp",
            str(self.qp),
        ]
        if self.debug:
            command.append("--debug")
        return command

    def _ffmpeg_executable(self) -> str:
        path = Path(self.ffmpeg_path)
        if path.exists():
            return str(path)
        found = shutil.which(self.ffmpeg_path)
        if found is None:
            raise RuntimeError(f"ffmpeg executable not found: {self.ffmpeg_path}")
        return found

    def _available_ffmpeg_encoders(self, ffmpeg: str) -> set[str]:
        try:
            result = subprocess.run(
                [ffmpeg, "-hide_banner", "-encoders"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except Exception:
            return set()
        names: set[str] = set()
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 2 and fields[0].startswith("V"):
                names.add(fields[1])
        return names

    def _available_ffmpeg_muxers(self, ffmpeg: str) -> set[str]:
        try:
            result = subprocess.run(
                [ffmpeg, "-hide_banner", "-muxers"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except Exception:
            return set()
        names: set[str] = set()
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 2 and fields[0].endswith("E"):
                names.add(fields[1])
        return names

    def _resolve_ffmpeg_encoder(self, ffmpeg: str) -> str:
        if self.ffmpeg_encoder_request != "auto":
            return self.ffmpeg_encoder_request

        encoders = self._available_ffmpeg_encoders(ffmpeg)
        h264_encoders = sorted(name for name in encoders if "264" in name)
        print(f"ffmpeg H264 encoders visible: {', '.join(h264_encoders) or 'none'}", flush=True)
        for candidate in ("h264_v4l2m2m", "h264_omx", "libx264"):
            if candidate in encoders:
                return candidate
        return "libx264"

    def _resolve_ffmpeg_output_muxer(self, ffmpeg: str) -> str:
        muxers = self._available_ffmpeg_muxers(ffmpeg)
        if self.debug:
            h264_muxers = sorted(name for name in muxers if name in ("h264", "rawvideo"))
            print(f"ffmpeg H264 stdout muxers visible: {', '.join(h264_muxers) or 'none'}", flush=True)
        for candidate in ("h264", "rawvideo"):
            if candidate in muxers:
                return candidate
        raise RuntimeError("ffmpeg does not provide a usable raw H264 stdout muxer")

    def _ffmpeg_command(self, ffmpeg: str, encoder: str, output_muxer: str) -> list[str]:
        command = [
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgba",
            "-s:v",
            f"{self.width}x{self.height}",
            "-framerate",
            str(self.fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            encoder,
            "-b:v",
            self.bitrate,
            "-maxrate",
            self.bitrate,
            "-bufsize",
            self.bitrate,
            "-g",
            str(self.gop),
            "-bf",
            "0",
            "-flags",
            "+low_delay",
        ]
        if encoder == "libx264":
            command.extend(
                [
                    "-preset",
                    "ultrafast",
                    "-tune",
                    "zerolatency",
                    "-profile:v",
                    "baseline",
                    "-pix_fmt",
                    "yuv420p",
                    "-x264-params",
                    f"keyint={self.gop}:min-keyint={self.gop}:scenecut=0:repeat-headers=1",
                ]
            )
        elif encoder == "h264_v4l2m2m":
            command.extend(["-pix_fmt", "nv12"])
        else:
            command.extend(["-pix_fmt", "yuv420p"])

        command.extend(["-flush_packets", "1", "-f", output_muxer, "pipe:1"])
        return command

    def _configure_native_library(self, lib: ctypes.CDLL) -> None:
        lib.cluster_h264_encoder_bridge_create.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
        ]
        lib.cluster_h264_encoder_bridge_create.restype = ctypes.c_void_p
        lib.cluster_h264_encoder_bridge_open.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_open.restype = ctypes.c_int
        lib.cluster_h264_encoder_bridge_encode_rgba.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_uint64,
            NativePacketCallback,
            ctypes.c_void_p,
        ]
        lib.cluster_h264_encoder_bridge_encode_rgba.restype = ctypes.c_int
        lib.cluster_h264_encoder_bridge_drain.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            NativePacketCallback,
            ctypes.c_void_p,
        ]
        lib.cluster_h264_encoder_bridge_drain.restype = ctypes.c_int
        lib.cluster_h264_encoder_bridge_close.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_close.restype = None
        lib.cluster_h264_encoder_bridge_destroy.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_destroy.restype = None
        lib.cluster_h264_encoder_bridge_last_error.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_last_error.restype = ctypes.c_char_p
        lib.cluster_h264_encoder_bridge_input_format_name.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_input_format_name.restype = ctypes.c_char_p
        lib.cluster_h264_encoder_bridge_input_stride.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_input_stride.restype = ctypes.c_size_t
        try:
            set_slice_max = lib.cluster_h264_encoder_bridge_set_slice_max_bytes
        except AttributeError:
            return
        set_slice_max.argtypes = [ctypes.c_void_p, ctypes.c_int]
        set_slice_max.restype = ctypes.c_int
        try:
            set_qp = lib.cluster_h264_encoder_bridge_set_qp
        except AttributeError:
            return
        set_qp.argtypes = [ctypes.c_void_p, ctypes.c_int]
        set_qp.restype = ctypes.c_int

    def _set_native_slice_max_bytes(self, lib: ctypes.CDLL, handle: int) -> None:
        try:
            set_slice_max = lib.cluster_h264_encoder_bridge_set_slice_max_bytes
        except AttributeError:
            if self.slice_max_bytes and self.debug:
                print(
                    "Warning: native H264 library does not expose slice max-byte control; rebuild "
                    "system/loggerd/libcluster_h264_encoder_bridge.so",
                    flush=True,
                )
            return
        if set_slice_max(handle, self.slice_max_bytes) != 0:
            raise RuntimeError(self._native_error_text("native H264 slice max-byte setup failed"))

    def _set_native_qp(self, lib: ctypes.CDLL, handle: int) -> None:
        try:
            set_qp = lib.cluster_h264_encoder_bridge_set_qp
        except AttributeError:
            if self.qp >= 0 and self.debug:
                print(
                    "Warning: native H264 library does not expose QP control; rebuild "
                    "system/loggerd/libcluster_h264_encoder_bridge.so",
                    flush=True,
                )
            return
        if set_qp(handle, self.qp) != 0:
            raise RuntimeError(self._native_error_text("native H264 QP setup failed"))

    def _native_packet_callback(
        self,
        data: int,
        size: int,
        _flags: int,
        _timestamp_us: int,
        _codec_config: int,
        _keyframe: int,
        _opaque: int,
    ) -> None:
        if size <= 0 or not data:
            return
        packet_queue = self._packet_queue
        if packet_queue is None:
            return
        try:
            chunk_size = max(1, self.chunk_size)
            base = int(data)
            packet = ctypes.string_at(base, int(size))
            packet = self._prepare_hardware_packet(packet)
            self._write_dump(packet)
            for chunk in self._packetize_h264_for_usb(packet, chunk_size, source="native"):
                packet_queue.put(chunk, timeout=1.0)
        except queue.Full as exc:
            self._set_error(RuntimeError("native H264 USB sender queue is full"))
        except BaseException as exc:
            self._set_error(exc)

    def _drain_native(self) -> None:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None or self._native_callback is None:
            return
        lib.cluster_h264_encoder_bridge_drain(handle, 250, self._native_callback, None)

    def _close_native(self) -> None:
        lib = self._native_lib
        handle = self._native_handle
        if lib is not None and handle is not None:
            lib.cluster_h264_encoder_bridge_destroy(handle)
        self._native_handle = None
        self._native_lib = None
        self._native_callback = None

    def _native_error_text(self, message: str) -> str:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None:
            return message
        error = lib.cluster_h264_encoder_bridge_last_error(handle)
        if error:
            return f"{message}: {error.decode('utf-8', errors='replace')}"
        return message

    def _native_input_format_name(self) -> str:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None:
            return ""
        value = lib.cluster_h264_encoder_bridge_input_format_name(handle)
        return "" if not value else value.decode("utf-8", errors="replace")

    @staticmethod
    def _parse_bitrate_bps(value: str) -> int:
        text = value.strip()
        multiplier = 1.0
        if text[-1:].lower() == "k":
            multiplier = 1000.0
            text = text[:-1]
        elif text[-1:].lower() == "m":
            multiplier = 1000000.0
            text = text[:-1]
        parsed = int(float(text) * multiplier + 0.5)
        if parsed <= 0:
            raise ValueError(f"invalid H264 bitrate: {value}")
        return parsed

    def _write_all(self, fd: int, data: Any, byte_count: int) -> None:
        view = memoryview(data)
        if view.nbytes < byte_count:
            raise RuntimeError(
                f"H264 encoder RGBA input is {view.nbytes} bytes, expected at least {byte_count}"
            )

        offset = 0
        while offset < byte_count:
            written = os.write(fd, view[offset:byte_count])
            if written <= 0:
                raise BrokenPipeError("H264 helper encoder pipe wrote zero bytes")
            offset += written

    def _read_stdout(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return

        try:
            fd = proc.stdout.fileno()
            chunk_size = max(1, self.chunk_size)
            while True:
                chunk = os.read(fd, chunk_size)
                if not chunk:
                    return
                if self.backend_name == "helper":
                    chunk = self._prepare_hardware_packet(chunk)
                chunks = self._packetize_h264_for_usb(chunk, chunk_size, source=self.backend_name)
                self._write_dump(chunk)
                for packet_chunk in chunks:
                    self._send_h264_chunk(packet_chunk, chunk_size, source=self.backend_name)
        except BaseException as exc:
            with self._condition:
                if not self._closing:
                    self._error = exc
                    print(
                        f"H264 USB stdout worker failed after {self._chunks_sent} chunks: "
                        f"{type(exc).__name__}: {exc}",
                        flush=True,
                    )
                self._condition.notify_all()

    def _send_queued_packets(self) -> None:
        packet_queue = self._packet_queue
        if packet_queue is None:
            return
        try:
            chunk_size = max(1, self.chunk_size)
            while True:
                chunk = packet_queue.get()
                if chunk is None:
                    return
                self._send_h264_chunk(chunk, chunk_size, source="native")
        except BaseException as exc:
            with self._condition:
                if not self._closing:
                    self._error = exc
                self._condition.notify_all()

    def _send_h264_chunk(self, chunk: bytes, chunk_size: int, *, source: str) -> None:
        self._chunks_sent += 1
        if self.debug and (self._chunks_sent <= 5 or self._chunks_sent % 30 == 0):
            head = " ".join(f"{byte:02X}" for byte in chunk[:8])
            ack_mode = "soft" if self.wait_for_ack and self.soft_ack else ("on" if self.wait_for_ack else "off")
            summary = _h264_packet_summary(chunk)
            print(
                f"H264 chunk {self._chunks_sent}: {source} {len(chunk)} bytes "
                f"head={head} ack={ack_mode} {summary}",
                flush=True,
            )
        profile_stage = time.perf_counter()
        self.usb_display.send_h264_chunk(
            chunk,
            wait_for_ack=self.wait_for_ack,
            require_ack_response=not self.soft_ack,
        )
        self._add_sample("usb_h264.send_chunk", profile_stage)

    def _read_stderr(self) -> None:
        proc = self._proc
        if proc is None or proc.stderr is None:
            return

        try:
            while True:
                line = proc.stderr.readline()
                if not line:
                    return
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    with self._condition:
                        self._stderr_tail.append(text)
                    if self.debug:
                        print(f"H264 encoder: {text}", flush=True)
        except Exception:
            return

    def _add_sample(self, name: str, start_time: float) -> None:
        milliseconds = (time.perf_counter() - start_time) * 1000.0
        with self._condition:
            self._samples.append((name, milliseconds))

    def _set_error(self, error: BaseException) -> None:
        with self._condition:
            self._error = error
            self._condition.notify_all()

    def _error_text(self, message: str, error: BaseException | None = None) -> str:
        parts = [message]
        if error is not None:
            parts.append(f"cause: {type(error).__name__}: {error}")
        with self._condition:
            tail = "\n".join(self._stderr_tail)
        if tail:
            parts.append(f"H264 encoder stderr tail:\n{tail}")
        return "\n".join(parts)
