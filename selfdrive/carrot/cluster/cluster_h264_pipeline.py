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
DEFAULT_H264_DEVICE = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1"
DEFAULT_H264_FFMPEG = "ffmpeg"
DEFAULT_H264_FFMPEG_ENCODER = "libx264"
DEFAULT_H264_SLICE_MAX_BYTES = 4096
DEFAULT_H264_ENCODER_ALIGN = 16
DEFAULT_H264_RATE_CONTROL = "vbr-cfr"

NATIVE_INPUT_FORMATS = {
    "auto": 0,
    "nv12": 2,
}
NATIVE_RATE_CONTROLS = {
    "off": 0,
    "vbr-vfr": 1,
    "vbr-cfr": 2,
    "cbr-vfr": 3,
    "cbr-cfr": 4,
    "mbr-cfr": 5,
    "mbr-vfr": 6,
    "cq": 7,
}
NATIVE_TIMING_GETTERS = (
    ("usb_h264.native.pre_poll", "cluster_h264_encoder_bridge_last_pre_poll_us"),
    ("usb_h264.native.wait_input", "cluster_h264_encoder_bridge_last_wait_input_us"),
    ("usb_h264.native.convert", "cluster_h264_encoder_bridge_last_convert_us"),
    ("usb_h264.native.sync", "cluster_h264_encoder_bridge_last_sync_us"),
    ("usb_h264.native.queue", "cluster_h264_encoder_bridge_last_queue_us"),
    ("usb_h264.native.post_poll", "cluster_h264_encoder_bridge_last_post_poll_us"),
    ("usb_h264.native.total_inner", "cluster_h264_encoder_bridge_last_total_us"),
)
H264_NAL_NAMES = {
    1: "P",
    5: "IDR",
    6: "SEI",
    7: "SPS",
    8: "PPS",
    9: "AUD",
}
H264_DEBUG_PACKET_LIMIT = 40
H264_DEBUG_PACKET_INTERVAL = 30
H264_DEBUG_CHUNK_LIMIT = 60
H264_DEBUG_CHUNK_INTERVAL = 25
NATIVE_PACKET_QUEUE_MAX_CHUNKS = 8
NATIVE_PACKET_QUEUE_PUT_TIMEOUT_S = 0.05
V4L2_BUF_FLAG_KEYFRAME = 0x00000008
V4L2_BUF_FLAG_PFRAME = 0x00000010
V4L2_BUF_FLAG_BFRAME = 0x00000020
V4L2_QCOM_BUF_FLAG_CODECCONFIG = 0x00020000
V4L2_QCOM_BUF_FLAG_EOS = 0x02000000


def _align_dimension(value: int, alignment: int) -> int:
    alignment = max(1, int(alignment))
    return ((int(value) + alignment - 1) // alignment) * alignment


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


def _h264_first_unit_of_type(data: bytes, nal_type: int) -> bytes | None:
    for start, nal_start, nal_end in _h264_nals(data):
        if nal_start < nal_end and (data[nal_start] & 0x1F) == nal_type:
            return data[start:nal_end]
    return None


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


def _h264_skip_hrd_parameters(reader: _H264BitReader) -> None:
    cpb_count = reader.read_ue() + 1
    reader.read_bits(4)
    reader.read_bits(4)
    for _ in range(cpb_count):
        reader.read_ue()
        reader.read_ue()
        reader.read_bit()
    reader.read_bits(5)
    reader.read_bits(5)
    reader.read_bits(5)
    reader.read_bits(5)


def _h264_read_vui_info(reader: _H264BitReader) -> dict[str, int]:
    info = {
        "timing_info_present": 0,
        "num_units_in_tick": 0,
        "time_scale": 0,
        "fixed_frame_rate_flag": 0,
        "bitstream_restriction_flag": 0,
        "max_num_reorder_frames": 0,
        "max_dec_frame_buffering": 0,
    }

    if reader.read_bit():
        aspect_ratio_idc = reader.read_bits(8)
        if aspect_ratio_idc == 255:
            reader.read_bits(16)
            reader.read_bits(16)
    if reader.read_bit():
        reader.read_bit()
    if reader.read_bit():
        reader.read_bits(3)
        reader.read_bit()
        if reader.read_bit():
            reader.read_bits(8)
            reader.read_bits(8)
            reader.read_bits(8)
    if reader.read_bit():
        reader.read_ue()
        reader.read_ue()
    if reader.read_bit():
        info["timing_info_present"] = 1
        info["num_units_in_tick"] = reader.read_bits(32)
        info["time_scale"] = reader.read_bits(32)
        info["fixed_frame_rate_flag"] = reader.read_bit()
    nal_hrd_parameters_present = reader.read_bit()
    if nal_hrd_parameters_present:
        _h264_skip_hrd_parameters(reader)
    vcl_hrd_parameters_present = reader.read_bit()
    if vcl_hrd_parameters_present:
        _h264_skip_hrd_parameters(reader)
    if nal_hrd_parameters_present or vcl_hrd_parameters_present:
        reader.read_bit()
    reader.read_bit()
    if reader.read_bit():
        info["bitstream_restriction_flag"] = 1
        reader.read_bit()
        reader.read_ue()
        reader.read_ue()
        reader.read_ue()
        reader.read_ue()
        info["max_num_reorder_frames"] = reader.read_ue()
        info["max_dec_frame_buffering"] = reader.read_ue()
    return info


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
        vui_text = " vui=0"
        stop_bitpos = _h264_rbsp_stop_bitpos(rbsp)
        if info["after_crop_bitpos"] < stop_bitpos:
            vui_reader = _H264BitReader(rbsp)
            vui_reader.bitpos = info["after_crop_bitpos"]
            if vui_reader.read_bit():
                try:
                    vui_info = _h264_read_vui_info(vui_reader)
                    vui_text = (
                        f" vui=1 timing={vui_info['timing_info_present']}"
                        f" tick={vui_info['num_units_in_tick']}/{vui_info['time_scale']}"
                        f" fixed={vui_info['fixed_frame_rate_flag']}"
                        f" restrict={vui_info['bitstream_restriction_flag']}"
                    )
                    if vui_info["bitstream_restriction_flag"]:
                        vui_text += (
                            f" reorder={vui_info['max_num_reorder_frames']}"
                            f" max_dpb={vui_info['max_dec_frame_buffering']}"
                        )
                except Exception:
                    vui_text = " vui=1 timing=?"
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


def _h264_diag_next_start_code(data: bytes, index: int) -> tuple[int, int]:
    start3 = data.find(b"\x00\x00\x01", index)
    start4 = data.find(b"\x00\x00\x00\x01", index)
    if start4 >= 0 and (start3 < 0 or start4 <= start3):
        return start4, 4
    if start3 >= 0:
        return start3, 3
    return -1, 0


def _h264_diag_stats(data: bytes) -> tuple[int, int, bool]:
    nal_count = 0
    max_nal_bytes = 0
    has_idr = False
    start, start_len = _h264_diag_next_start_code(data, 0)
    while start >= 0:
        nal_start = start + start_len
        next_start, next_len = _h264_diag_next_start_code(data, nal_start)
        nal_end = next_start if next_start >= 0 else len(data)
        while nal_end > nal_start and data[nal_end - 1] == 0:
            nal_end -= 1
        if nal_start >= nal_end:
            start, start_len = next_start, next_len
            continue
        nal_count += 1
        nal_size = nal_end - nal_start
        if nal_size > max_nal_bytes:
            max_nal_bytes = nal_size
        if (data[nal_start] & 0x1F) == 5:
            has_idr = True
        start, start_len = next_start, next_len
    return nal_count, max_nal_bytes, has_idr


def _bytes_head(data: bytes, limit: int = 16) -> str:
    return " ".join(f"{byte:02X}" for byte in data[:limit])


def _h264_flags_text(flags: int, codec_config: bool = False, keyframe: bool = False) -> str:
    names: list[str] = []
    known_mask = 0
    for value, name in (
        (V4L2_BUF_FLAG_KEYFRAME, "KEYFRAME"),
        (V4L2_BUF_FLAG_PFRAME, "PFRAME"),
        (V4L2_BUF_FLAG_BFRAME, "BFRAME"),
        (V4L2_QCOM_BUF_FLAG_CODECCONFIG, "CODECCONFIG"),
        (V4L2_QCOM_BUF_FLAG_EOS, "EOS"),
    ):
        known_mask |= value
        if flags & value:
            names.append(name)
    if codec_config and "CODECCONFIG" not in names:
        names.append("codec_config_cb")
    if keyframe and "KEYFRAME" not in names:
        names.append("keyframe_cb")
    extra = flags & ~known_mask
    if extra:
        names.append(f"extra=0x{extra:X}")
    return "|".join(names) if names else "none"


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
    writer.write_bit(1)  # bitstream_restriction_flag
    writer.write_bit(1)  # motion_vectors_over_pic_boundaries_flag
    writer.write_ue(0)  # max_bytes_per_pic_denom
    writer.write_ue(0)  # max_bits_per_mb_denom
    writer.write_ue(10)  # log2_max_mv_length_horizontal
    writer.write_ue(10)  # log2_max_mv_length_vertical
    writer.write_ue(0)  # max_num_reorder_frames
    writer.write_ue(4)  # max_dec_frame_buffering


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
            stop_bitpos = _h264_rbsp_stop_bitpos(rbsp)
            if vui_flag_bitpos > stop_bitpos:
                continue
            existing_vui = 0
            existing_timing = 0
            if vui_flag_bitpos < stop_bitpos:
                vui_reader = _H264BitReader(rbsp)
                vui_reader.bitpos = vui_flag_bitpos
                existing_vui = vui_reader.read_bit()
                if existing_vui:
                    try:
                        existing_timing = _h264_read_vui_info(vui_reader)["timing_info_present"]
                    except Exception:
                        existing_timing = 0
                    if existing_timing:
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
            patched_info = (
                f"fps={fps} num_units_in_tick=1 time_scale={max(2, int(fps) * 2)} "
                f"replaced_vui={existing_vui} previous_timing={existing_timing}"
            )
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
        encoder_align: int,
        fps: int,
        bitrate: str,
        gop: int,
        backend: str,
        library_path: str,
        ffmpeg_path: str,
        ffmpeg_encoder: str,
        device_path: str,
        input_format: str,
        slice_max_bytes: int,
        rate_control: str,
        realtime_priority: bool,
        requested_chunk_size: int,
        wait_for_ack: bool,
        soft_ack: bool,
        dump_path: str,
        debug: bool,
        diagnose_interval_s: float = 0.0,
    ) -> None:
        self.usb_display = usb_display
        self.width = int(width)
        self.height = int(height)
        align_hardware_input = backend != "ffmpeg"
        self.encoder_width = _align_dimension(self.width, encoder_align) if align_hardware_input else self.width
        self.encoder_height = _align_dimension(self.height, encoder_align) if align_hardware_input else self.height
        self.fps = max(1, int(fps))
        self.bitrate = bitrate
        self.gop = max(1, int(gop))
        self.backend_request = backend
        self.backend_name = backend
        self.library_path = library_path
        self.ffmpeg_path = ffmpeg_path
        self.ffmpeg_encoder_request = ffmpeg_encoder
        self.ffmpeg_encoder_name = ffmpeg_encoder
        self.ffmpeg_muxer_name = ""
        self.device_path = device_path
        self.input_format = input_format
        self.slice_max_bytes = max(0, int(slice_max_bytes))
        self.rate_control = rate_control
        self.realtime_priority = realtime_priority
        self.requested_chunk_size = max(0, int(requested_chunk_size))
        self.wait_for_ack = wait_for_ack
        self.soft_ack = soft_ack
        self.dump_path = dump_path
        self._dump_file = None
        self._dump_write_count = 0
        self.debug = debug
        self.diagnose_interval_s = max(0.0, float(diagnose_interval_s))
        self._sps_patch_logged = False
        self._sps_crop_patch_logged = False
        self._sps_vui_patch_logged = False
        self._sps_patch_cache: tuple[bytes, bytes] | None = None
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
        self._native_input_stride = 0
        self._native_input_y_scanlines = 0
        self._native_input_uv_scanlines = 0
        self._native_input_uv_offset = 0
        self._native_input_bytesused = 0
        self._native_input_active_bytes = 0
        self._native_has_active_nv12 = False
        self._packet_queue: queue.Queue[Any] | None = None
        self._condition = threading.Condition()
        self._closing = False
        self._stream_started = False
        self._error: BaseException | None = None
        self._samples: list[tuple[str, float]] = []
        self._stderr_tail: deque[str] = deque(maxlen=20)
        self._padded_rgba: bytearray | None = None
        now = time.perf_counter()
        self._debug_started_at = now
        self._debug_encoder_packets = 0
        self._debug_encoder_bytes = 0
        self._debug_stdout_reads = 0
        self._debug_stdout_bytes = 0
        self._debug_packetize_events = 0
        self._debug_usb_bytes = 0
        self._debug_max_packet_bytes = 0
        self._debug_max_chunk_bytes = 0
        self._diag_window_started_at = now
        self._reset_h264_diag_window(now)

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
                print(f"Warning: native H264 encoder unavailable, falling back to ffmpeg: {exc}", flush=True)
                self._close_native()

        if self.backend_request not in ("auto", "ffmpeg"):
            raise RuntimeError(f"unsupported H264 backend: {self.backend_request}")
        if self.backend_request == "auto":
            self.encoder_width = self.width
            self.encoder_height = self.height
        self._start_ffmpeg()

    def _diagnose_text(self) -> str:
        if not self._diagnostics_enabled():
            return ""
        return f" diag={self.diagnose_interval_s:g}s"

    def _start_native(self) -> None:
        library = self._resolve_library()
        lib = ctypes.CDLL(library)
        self._configure_native_library(lib)

        bitrate_bps = self._parse_bitrate_bps(self.bitrate)
        handle = lib.cluster_h264_encoder_bridge_create(
            self.encoder_width,
            self.encoder_height,
            self.fps,
            bitrate_bps,
            self.gop,
            self.device_path.encode("utf-8"),
            NATIVE_INPUT_FORMATS[self.input_format],
            0,
            1 if self.debug else 0,
        )
        if not handle:
            raise RuntimeError("native H264 encoder bridge allocation failed")
        self._native_lib = lib
        self._native_handle = handle
        self._native_callback = NativePacketCallback(self._native_packet_callback)
        self._set_native_slice_max_bytes(lib, handle)
        self._set_native_rate_control(lib, handle)
        self._set_native_realtime_priority(lib, handle)

        if lib.cluster_h264_encoder_bridge_open(handle) != 0:
            raise RuntimeError(self._native_error_text("native H264 encoder open failed"))

        try:
            self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
            self._stream_started = True
            self._open_dump_file()
        except Exception:
            self._close_native()
            raise

        self._packet_queue = queue.Queue(maxsize=NATIVE_PACKET_QUEUE_MAX_CHUNKS)
        self._sender_thread = threading.Thread(
            target=self._send_queued_packets,
            name="cluster-usb-h264-native-send",
            daemon=True,
        )
        self._sender_thread.start()

        self.backend_name = "native"
        input_name = self._native_input_format_name()
        input_stride = lib.cluster_h264_encoder_bridge_input_stride(handle)
        input_y_scanlines = self._native_size_value("cluster_h264_encoder_bridge_input_y_scanlines")
        input_uv_scanlines = self._native_size_value("cluster_h264_encoder_bridge_input_uv_scanlines")
        input_sizeimage = self._native_size_value("cluster_h264_encoder_bridge_input_sizeimage")
        input_uv_offset = self._native_size_value("cluster_h264_encoder_bridge_input_uv_offset")
        input_bytesused = self._native_size_value("cluster_h264_encoder_bridge_input_bytesused")
        input_active_bytes = self._native_size_value("cluster_h264_encoder_bridge_input_active_bytes")
        if input_active_bytes <= 0:
            input_active_bytes = int(input_uv_offset) + int(input_stride) * int(input_uv_scanlines)
        capture_sizeimage = self._native_size_value("cluster_h264_encoder_bridge_capture_sizeimage")
        self._native_input_stride = int(input_stride)
        self._native_input_y_scanlines = int(input_y_scanlines)
        self._native_input_uv_scanlines = int(input_uv_scanlines)
        self._native_input_uv_offset = int(input_uv_offset)
        self._native_input_bytesused = int(input_bytesused)
        self._native_input_active_bytes = int(input_active_bytes)
        print(
            "Starting H264 USB native hardware encoder: "
            f"{self.width}x{self.height}@{self.fps} "
            f"encoder={self.encoder_width}x{self.encoder_height} "
            f"bitrate={bitrate_bps} gop={self.gop} "
            f"slice_max={self.slice_max_bytes} rate_control={self.rate_control} "
            f"realtime_priority={'on' if self.realtime_priority else 'off'} packetize=access-unit "
            f"input={input_name or self.input_format} stride={input_stride} "
            f"scanlines={input_y_scanlines}/{input_uv_scanlines} "
            f"input_size={input_sizeimage} input_bytes={input_bytesused} "
            f"active_bytes={input_active_bytes} uv_offset={input_uv_offset} "
            f"capture_size={capture_sizeimage} "
            f"device={self.device_path} "
            f"chunk_ack={'soft' if self.wait_for_ack and self.soft_ack else ('on' if self.wait_for_ack else 'off')}"
            f"{self._diagnose_text()} "
            "sps_patch=on sps_crop_patch=on sps_vui_patch=on",
            flush=True,
        )
        self._debug_log_session_config("native")

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
            f"{self.ffmpeg_encoder_name} {self.encoder_width}x{self.encoder_height}@{self.fps} "
            f"bitrate={self.bitrate} gop={self.gop} muxer={self.ffmpeg_muxer_name} "
            "packetize=access-unit "
            f"chunk_ack={'soft' if self.wait_for_ack and self.soft_ack else ('on' if self.wait_for_ack else 'off')}"
            f"{self._diagnose_text()}",
            flush=True,
        )
        if self.debug:
            print(f"H264 ffmpeg command: {' '.join(command)}", flush=True)
        self._debug_log_session_config("ffmpeg")
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
        input_size = (int(width), int(height))
        display_size = (self.width, self.height)
        encoder_size = (self.encoder_width, self.encoder_height)
        if input_size not in (display_size, encoder_size):
            raise RuntimeError(
                f"H264 encoder input size changed to {width}x{height}; expected "
                f"display {self.width}x{self.height} or encoder {self.encoder_width}x{self.encoder_height}"
            )
        if self._closing:
            raise RuntimeError("H264 USB pipeline is closing")
        if self._native_handle is not None:
            raise RuntimeError("native H264 USB pipeline requires NV12 input")

        if input_size == encoder_size:
            encoder_rgba = rgba
        else:
            profile_stage = time.perf_counter()
            encoder_rgba = self._encoder_rgba(rgba, width, height)
            self._add_sample("usb_h264.pad_rgba", profile_stage)
        proc = self._proc
        if proc is None or proc.stdin is None:
            raise RuntimeError("H264 USB pipeline is not started")

        profile_stage = time.perf_counter()
        try:
            self._write_all(proc.stdin.fileno(), encoder_rgba, self.encoder_width * self.encoder_height * 4)
        except BrokenPipeError as exc:
            self._set_error(exc)
            raise RuntimeError(self._error_text(f"H264 {self.backend_name} encoder pipe closed")) from exc
        except Exception as exc:
            self._set_error(exc)
            raise
        self._add_sample("usb_h264.write_rgba", profile_stage)
        self.check_error()

    def submit_nv12(self, nv12: Any, width: int, height: int) -> None:
        self.check_error()
        if (int(width), int(height)) != (self.encoder_width, self.encoder_height):
            raise RuntimeError(
                f"H264 encoder NV12 input size changed to {width}x{height}; expected "
                f"{self.encoder_width}x{self.encoder_height}"
            )
        if self._closing:
            raise RuntimeError("H264 USB pipeline is closing")
        self._submit_nv12_native(nv12)

    def _submit_nv12_native(self, nv12: Any) -> None:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None or self._native_callback is None:
            raise RuntimeError("native NV12 H264 USB pipeline is not started")
        try:
            encode_nv12 = lib.cluster_h264_encoder_bridge_encode_nv12
        except AttributeError as exc:
            raise RuntimeError(
                "native H264 library does not expose NV12 input; rebuild "
                "system/loggerd/libcluster_h264_encoder_bridge.so"
            ) from exc

        byte_count = self._native_input_bytesused
        active_count = self._native_input_active_bytes
        if byte_count <= 0:
            raise RuntimeError("native H264 encoder did not report NV12 input bytesused")
        view = memoryview(nv12)
        use_active = False
        if view.nbytes < byte_count:
            use_active = self._native_has_active_nv12 and active_count > 0 and view.nbytes >= active_count
        if view.nbytes < byte_count and not use_active:
            raise RuntimeError(
                f"H264 encoder NV12 input is {view.nbytes} bytes, expected at least {byte_count}"
                + (f" or active {active_count}" if self._native_has_active_nv12 and active_count > 0 else "")
            )
        if not view.contiguous:
            raise RuntimeError("H264 encoder NV12 input must be contiguous")

        profile_stage = time.perf_counter()
        try:
            data_ptr = ctypes.addressof(ctypes.c_uint8.from_buffer(view))
        except TypeError as exc:
            raise RuntimeError("H264 encoder NV12 input must expose a writable buffer") from exc

        timestamp_us = self._native_frame_index * 1000000 // self.fps
        encode_fn = encode_nv12
        encode_size = byte_count
        sample_name = "usb_h264.native_encode_nv12"
        if use_active:
            try:
                encode_fn = lib.cluster_h264_encoder_bridge_encode_nv12_active
            except AttributeError as exc:
                raise RuntimeError(
                    "native H264 library does not expose active NV12 input; rebuild "
                    "system/loggerd/libcluster_h264_encoder_bridge.so"
                ) from exc
            encode_size = active_count
            sample_name = "usb_h264.native_encode_nv12_active"
        result = encode_fn(
            handle,
            ctypes.c_void_p(data_ptr),
            encode_size,
            timestamp_us,
            self._native_callback,
            None,
        )
        if result != 0:
            raise RuntimeError(self._native_error_text("native H264 NV12 encode failed"))
        self._native_frame_index += 1
        self._add_native_timing_samples(lib, handle)
        self._add_sample(sample_name, profile_stage)
        self.check_error()

    def build_nv12_color_test_pattern(self) -> bytearray:
        if self._native_input_bytesused <= 0:
            raise RuntimeError("native H264 encoder must be started before building an NV12 test pattern")
        if self._native_input_stride <= 0 or self._native_input_uv_offset <= 0:
            raise RuntimeError("native H264 encoder did not report a usable NV12 layout")

        pattern = bytearray([16]) * self._native_input_bytesused
        uv_start = self._native_input_uv_offset
        uv_end = min(len(pattern), uv_start + self._native_input_stride * self._native_input_uv_scanlines)
        if uv_start < uv_end:
            pattern[uv_start:uv_end] = b"\x80" * (uv_end - uv_start)

        half_width = max(2, self.encoder_width // 2)
        half_height = max(2, self.encoder_height // 2)
        # BT.601 limited-range YUV values for red, green, blue, and white.
        quadrants = (
            (0, 0, half_width, half_height, 82, 90, 240),
            (half_width, 0, self.encoder_width, half_height, 145, 54, 34),
            (0, half_height, half_width, self.encoder_height, 41, 240, 110),
            (half_width, half_height, self.encoder_width, self.encoder_height, 235, 128, 128),
        )
        stride = self._native_input_stride
        for x0, y0, x1, y1, y_value, u_value, v_value in quadrants:
            for y in range(y0, y1):
                row = y * stride
                pattern[row + x0:row + x1] = bytes((y_value,)) * (x1 - x0)
            for y in range(y0 & ~1, y1, 2):
                uv_row = uv_start + (y // 2) * stride
                for x in range(x0 & ~1, x1, 2):
                    offset = uv_row + x
                    if offset + 1 < uv_end:
                        pattern[offset] = u_value
                        pattern[offset + 1] = v_value
        return pattern

    def native_nv12_layout(self) -> tuple[int, int, int, int, int]:
        if self._native_handle is None:
            raise RuntimeError("native NV12 layout is only available for the native H264 backend")
        if self._native_input_stride <= 0 or self._native_input_uv_offset <= 0 or self._native_input_bytesused <= 0:
            raise RuntimeError("native H264 encoder did not report a usable NV12 layout")
        return (
            self._native_input_stride,
            self._native_input_y_scanlines,
            self._native_input_uv_scanlines,
            self._native_input_uv_offset,
            self._native_input_bytesused,
        )

    def native_nv12_render_layout(self) -> tuple[int, int, int, int, int, int, bool]:
        stride, y_scanlines, uv_scanlines, uv_offset, input_bytes = self.native_nv12_layout()
        active_bytes = self._native_input_active_bytes or (uv_offset + stride * uv_scanlines)
        use_active = self._native_has_active_nv12 and 0 < active_bytes < input_bytes
        render_bytes = active_bytes if use_active else input_bytes
        return stride, y_scanlines, uv_scanlines, uv_offset, input_bytes, render_bytes, use_active

    def _encoder_rgba(self, rgba: Any, width: int, height: int) -> Any:
        if self.encoder_width == width and self.encoder_height == height:
            return rgba

        src_view = memoryview(rgba)
        src_row_bytes = int(width) * 4
        src_bytes = src_row_bytes * int(height)
        if src_view.nbytes < src_bytes:
            raise RuntimeError(
                f"H264 source RGBA input is {src_view.nbytes} bytes, expected at least {src_bytes}"
            )
        if not src_view.contiguous:
            raise RuntimeError("H264 source RGBA input must be contiguous")

        dst_row_bytes = self.encoder_width * 4
        dst_bytes = dst_row_bytes * self.encoder_height
        if self._padded_rgba is None or len(self._padded_rgba) != dst_bytes:
            self._padded_rgba = bytearray(dst_bytes)
            # Keep padding opaque black so the encoder sees stable macroblock-aligned edges.
            self._padded_rgba[3::4] = b"\xff" * (dst_bytes // 4)

        dst = self._padded_rgba
        for y in range(int(height)):
            src_start = y * src_row_bytes
            dst_start = y * dst_row_bytes
            dst[dst_start:dst_start + src_row_bytes] = src_view[src_start:src_start + src_row_bytes]
        return dst

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
        self._maybe_log_h264_diag(force=True)
        self._debug_log_close_summary()
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
        self._dump_write_count += 1
        if self._dump_write_count % self.fps == 0:
            self._dump_file.flush()

    @staticmethod
    def _debug_should_log(index: int, limit: int, interval: int) -> bool:
        return index <= limit or (interval > 0 and index % interval == 0)

    def _debug_log_session_config(self, source: str) -> None:
        if not self.debug:
            return
        print(
            "H264 debug session: "
            f"backend={source} requested_backend={self.backend_request} "
            f"display={self.width}x{self.height} encoder={self.encoder_width}x{self.encoder_height} "
            f"fps={self.fps} bitrate={self.bitrate} gop={self.gop} "
            f"chunk_size={self.chunk_size} requested_chunk={self.requested_chunk_size} "
            "packetize_mode=access-unit "
            f"wait_ack={1 if self.wait_for_ack else 0} soft_ack={1 if self.soft_ack else 0} "
            f"dump={self.dump_path or 'off'}",
            flush=True,
        )

    def _debug_log_encoder_packet(
        self,
        index: int,
        source: str,
        stage: str,
        packet: bytes,
        *,
        flags: int = 0,
        timestamp_us: int = 0,
        codec_config: bool = False,
        keyframe: bool = False,
    ) -> None:
        if not self.debug or not self._debug_should_log(index, H264_DEBUG_PACKET_LIMIT, H264_DEBUG_PACKET_INTERVAL):
            return
        print(
            f"H264 encoder packet {index}: {source}/{stage} size={len(packet)} "
            f"flags=0x{flags:08X}({_h264_flags_text(flags, codec_config, keyframe)}) "
            f"ts={timestamp_us} codec_config={1 if codec_config else 0} keyframe={1 if keyframe else 0} "
            f"head={_bytes_head(packet)} {_h264_packet_summary(packet, max_nals=10)}",
            flush=True,
        )

    def _debug_log_stdout_read(self, index: int, source: str, packet: bytes) -> None:
        if not self.debug or not self._debug_should_log(index, H264_DEBUG_PACKET_LIMIT, H264_DEBUG_PACKET_INTERVAL):
            return
        print(
            f"H264 stdout read {index}: {source} size={len(packet)} ffmpeg_read=1 "
            f"head={_bytes_head(packet)} {_h264_packet_summary(packet, max_nals=10)}",
            flush=True,
        )

    def _debug_log_packetize(self, source: str, index: int, packet: bytes, chunks: list[bytes], chunk_size: int) -> None:
        self._debug_packetize_events += 1
        if not self.debug or not self._debug_should_log(index, H264_DEBUG_PACKET_LIMIT, H264_DEBUG_PACKET_INTERVAL):
            return
        units = _h264_byte_stream_units(packet)
        sizes = [len(chunk) for chunk in chunks]
        min_size = min(sizes) if sizes else 0
        max_size = max(sizes) if sizes else 0
        total_size = sum(sizes)
        first_head = _bytes_head(chunks[0], 8) if chunks else ""
        last_head = _bytes_head(chunks[-1], 8) if chunks else ""
        print(
            f"H264 packetize {index}: {source} mode=access-unit "
            f"input={len(packet)} chunk_limit={chunk_size} chunks={len(chunks)} "
            f"sizes={min_size}/{max_size}/{total_size} start_units={len(units)} "
            f"first={first_head} last={last_head}",
            flush=True,
        )

    def _debug_log_close_summary(self) -> None:
        if not self.debug:
            return
        elapsed = max(0.001, time.perf_counter() - self._debug_started_at)
        print(
            "H264 debug summary: "
            f"backend={self.backend_name} elapsed={elapsed:.2f}s "
            f"encoder_packets={self._debug_encoder_packets} encoder_bytes={self._debug_encoder_bytes} "
            f"stdout_reads={self._debug_stdout_reads} stdout_bytes={self._debug_stdout_bytes} "
            f"packetize_events={self._debug_packetize_events} "
            f"usb_chunks={self._chunks_sent} usb_bytes={self._debug_usb_bytes} "
            f"max_packet={self._debug_max_packet_bytes} max_chunk={self._debug_max_chunk_bytes}",
            flush=True,
        )

    def _reset_h264_diag_window(self, now: float) -> None:
        self._diag_window_started_at = now
        self._diag_source = ""
        self._diag_units = 0
        self._diag_key_units = 0
        self._diag_unit_bytes = 0
        self._diag_max_unit_bytes = 0
        self._diag_chunks = 0
        self._diag_max_chunks = 0
        self._diag_nals = 0
        self._diag_max_nals = 0
        self._diag_max_nal_bytes = 0
        self._diag_queue_max = 0
        self._diag_send_chunks = 0
        self._diag_send_bytes = 0
        self._diag_send_ms = 0.0
        self._diag_send_ms_max = 0.0

    def _diagnostics_enabled(self) -> bool:
        return self.diagnose_interval_s > 0.0

    def _record_h264_unit(
        self,
        source: str,
        packet: bytes,
        chunks: list[bytes],
        *,
        keyframe: bool = False,
    ) -> None:
        if not self._diagnostics_enabled():
            return
        nal_count, max_nal_bytes, has_idr = _h264_diag_stats(packet)
        with self._condition:
            self._diag_source = source
            self._diag_units += 1
            self._diag_key_units += 1 if keyframe or has_idr else 0
            self._diag_unit_bytes += len(packet)
            self._diag_max_unit_bytes = max(self._diag_max_unit_bytes, len(packet))
            self._diag_chunks += len(chunks)
            self._diag_max_chunks = max(self._diag_max_chunks, len(chunks))
            self._diag_nals += nal_count
            self._diag_max_nals = max(self._diag_max_nals, nal_count)
            self._diag_max_nal_bytes = max(self._diag_max_nal_bytes, max_nal_bytes)

    def _record_h264_queue_depth(self, depth: int) -> None:
        if not self._diagnostics_enabled():
            return
        with self._condition:
            self._diag_queue_max = max(self._diag_queue_max, int(depth))

    def _record_h264_send(self, source: str, byte_count: int, milliseconds: float) -> None:
        if not self._diagnostics_enabled():
            return
        with self._condition:
            self._diag_source = source
            self._diag_send_chunks += 1
            self._diag_send_bytes += int(byte_count)
            self._diag_send_ms += float(milliseconds)
            self._diag_send_ms_max = max(self._diag_send_ms_max, float(milliseconds))
        self._maybe_log_h264_diag()

    def _maybe_log_h264_diag(self, *, force: bool = False) -> None:
        if not self._diagnostics_enabled():
            return

        line = ""
        now = time.perf_counter()
        with self._condition:
            span_s = max(0.001, now - self._diag_window_started_at)
            if not force and span_s < self.diagnose_interval_s:
                return
            if self._diag_units == 0 and self._diag_send_chunks == 0:
                self._reset_h264_diag_window(now)
                return

            unit_avg = self._diag_unit_bytes / self._diag_units if self._diag_units else 0.0
            chunks_avg = self._diag_chunks / self._diag_units if self._diag_units else 0.0
            nals_avg = self._diag_nals / self._diag_units if self._diag_units else 0.0
            unit_kbps = (self._diag_unit_bytes * 8.0) / span_s / 1000.0
            send_kbps = (self._diag_send_bytes * 8.0) / span_s / 1000.0
            send_avg_ms = self._diag_send_ms / self._diag_send_chunks if self._diag_send_chunks else 0.0
            source = self._diag_source or self.backend_name
            line = (
                f"H264 diag {span_s:.1f}s: backend={self.backend_name} source={source} "
                f"units={self._diag_units} key={self._diag_key_units} "
                f"unit_bytes_avg={unit_avg:.0f} max={self._diag_max_unit_bytes} "
                f"unit_kbps={unit_kbps:.0f} "
                f"chunks_avg={chunks_avg:.1f} max={self._diag_max_chunks} "
                f"nals_avg={nals_avg:.1f} max={self._diag_max_nals} "
                f"max_nal={self._diag_max_nal_bytes} qmax={self._diag_queue_max} "
                f"send_chunks={self._diag_send_chunks} send_kbps={send_kbps:.0f} "
                f"send_ms_avg={send_avg_ms:.2f} max={self._diag_send_ms_max:.2f}"
            )
            self._reset_h264_diag_window(now)

        if line:
            print(line, flush=True)

    def _prepare_hardware_packet(self, packet: bytes, *, may_have_sps: bool = True) -> bytes:
        if not may_have_sps:
            return packet
        cached_sps = self._sps_patch_cache
        if cached_sps is not None:
            raw_sps, patched_sps = cached_sps
            cached_packet = packet.replace(raw_sps, patched_sps, 1)
            if cached_packet != packet:
                return cached_packet

        raw_sps = _h264_first_unit_of_type(packet, 7)
        if raw_sps is None:
            return packet

        packet, patched = _patch_h264_sps_constraints(packet)
        if patched and self.debug and not self._sps_patch_logged:
            print(
                "H264 hardware SPS patched: baseline constraint flags OR 0x40 to match libx264 constrained-baseline",
                flush=True,
            )
            self._sps_patch_logged = True
        packet, patched, crop_info = _patch_h264_sps_crop(packet, self.width, self.height)
        if patched and self.debug and not self._sps_crop_patch_logged:
            print(
                f"H264 hardware SPS crop patched: {crop_info}",
                flush=True,
            )
            self._sps_crop_patch_logged = True
        packet, patched, vui_info = _patch_h264_sps_vui_timing(packet, self.fps)
        if patched and self.debug and not self._sps_vui_patch_logged:
            print(
                f"H264 hardware SPS VUI timing patched: {vui_info}",
                flush=True,
            )
            self._sps_vui_patch_logged = True
        patched_sps = _h264_first_unit_of_type(packet, 7)
        if patched_sps is not None:
            self._sps_patch_cache = (raw_sps, patched_sps)
        return packet

    def _packetize_h264_for_usb(self, packet: bytes, chunk_size: int) -> list[bytes]:
        chunk_size = max(1, chunk_size)
        if len(packet) <= chunk_size:
            return [packet]
        return [packet[offset:offset + chunk_size] for offset in range(0, len(packet), chunk_size)]

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
            f"{self.encoder_width}x{self.encoder_height}",
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
        try:
            encode_nv12 = lib.cluster_h264_encoder_bridge_encode_nv12
        except AttributeError:
            pass
        else:
            encode_nv12.argtypes = [
                ctypes.c_void_p,
                ctypes.c_void_p,
                ctypes.c_size_t,
                ctypes.c_uint64,
                NativePacketCallback,
                ctypes.c_void_p,
            ]
            encode_nv12.restype = ctypes.c_int
        try:
            encode_nv12_active = lib.cluster_h264_encoder_bridge_encode_nv12_active
        except AttributeError:
            self._native_has_active_nv12 = False
        else:
            encode_nv12_active.argtypes = [
                ctypes.c_void_p,
                ctypes.c_void_p,
                ctypes.c_size_t,
                ctypes.c_uint64,
                NativePacketCallback,
                ctypes.c_void_p,
            ]
            encode_nv12_active.restype = ctypes.c_int
            self._native_has_active_nv12 = True
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
        for getter_name in (
            "cluster_h264_encoder_bridge_input_y_scanlines",
            "cluster_h264_encoder_bridge_input_uv_scanlines",
            "cluster_h264_encoder_bridge_input_sizeimage",
            "cluster_h264_encoder_bridge_input_uv_offset",
            "cluster_h264_encoder_bridge_input_bytesused",
            "cluster_h264_encoder_bridge_input_active_bytes",
            "cluster_h264_encoder_bridge_capture_sizeimage",
        ):
            try:
                getter = getattr(lib, getter_name)
            except AttributeError:
                continue
            getter.argtypes = [ctypes.c_void_p]
            getter.restype = ctypes.c_size_t
        for _, getter_name in NATIVE_TIMING_GETTERS:
            try:
                getter = getattr(lib, getter_name)
            except AttributeError:
                continue
            getter.argtypes = [ctypes.c_void_p]
            getter.restype = ctypes.c_size_t
        try:
            set_slice_max = lib.cluster_h264_encoder_bridge_set_slice_max_bytes
        except AttributeError:
            pass
        else:
            set_slice_max.argtypes = [ctypes.c_void_p, ctypes.c_int]
            set_slice_max.restype = ctypes.c_int
        try:
            set_rate_control = lib.cluster_h264_encoder_bridge_set_rate_control
        except AttributeError:
            pass
        else:
            set_rate_control.argtypes = [ctypes.c_void_p, ctypes.c_int]
            set_rate_control.restype = ctypes.c_int
        try:
            set_realtime_priority = lib.cluster_h264_encoder_bridge_set_realtime_priority
        except AttributeError:
            pass
        else:
            set_realtime_priority.argtypes = [ctypes.c_void_p, ctypes.c_int]
            set_realtime_priority.restype = ctypes.c_int

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

    def _set_native_rate_control(self, lib: ctypes.CDLL, handle: int) -> None:
        try:
            set_rate_control = lib.cluster_h264_encoder_bridge_set_rate_control
        except AttributeError:
            if self.rate_control != DEFAULT_H264_RATE_CONTROL:
                raise RuntimeError(
                    "native H264 library does not expose rate-control setup; rebuild "
                    "system/loggerd/libcluster_h264_encoder_bridge.so"
                )
            return
        if set_rate_control(handle, NATIVE_RATE_CONTROLS[self.rate_control]) != 0:
            raise RuntimeError(self._native_error_text("native H264 rate-control setup failed"))

    def _set_native_realtime_priority(self, lib: ctypes.CDLL, handle: int) -> None:
        try:
            set_realtime_priority = lib.cluster_h264_encoder_bridge_set_realtime_priority
        except AttributeError:
            if self.realtime_priority:
                raise RuntimeError(
                    "native H264 library does not expose realtime-priority setup; rebuild "
                    "system/loggerd/libcluster_h264_encoder_bridge.so"
                )
            return
        if set_realtime_priority(handle, 1 if self.realtime_priority else 0) != 0:
            raise RuntimeError(self._native_error_text("native H264 realtime-priority setup failed"))

    def _native_packet_callback(
        self,
        data: int,
        size: int,
        flags: int,
        timestamp_us: int,
        codec_config: int,
        keyframe: int,
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
            profile_callback = self.usb_display.profile_enabled
            profile_total = time.perf_counter() if profile_callback else 0.0
            profile_stage = profile_total
            packet = ctypes.string_at(base, int(size))
            if profile_callback:
                self._add_sample("usb_h264.native.callback_copy", profile_stage)
            self._debug_encoder_packets += 1
            packet_index = self._debug_encoder_packets
            self._debug_encoder_bytes += len(packet)
            self._debug_max_packet_bytes = max(self._debug_max_packet_bytes, len(packet))
            self._debug_log_encoder_packet(
                packet_index,
                "native",
                "raw",
                packet,
                flags=int(flags),
                timestamp_us=int(timestamp_us),
                codec_config=bool(codec_config),
                keyframe=bool(keyframe),
            )
            profile_stage = time.perf_counter() if profile_callback else 0.0
            packet = self._prepare_hardware_packet(packet, may_have_sps=bool(codec_config) or bool(keyframe))
            if profile_callback:
                self._add_sample("usb_h264.native.callback_prepare", profile_stage)
            self._debug_max_packet_bytes = max(self._debug_max_packet_bytes, len(packet))
            self._debug_log_encoder_packet(
                packet_index,
                "native",
                "patched",
                packet,
                flags=int(flags),
                timestamp_us=int(timestamp_us),
                codec_config=bool(codec_config),
                keyframe=bool(keyframe),
            )
            self._write_dump(packet)
            profile_stage = time.perf_counter() if profile_callback else 0.0
            chunks = self._packetize_h264_for_usb(packet, chunk_size)
            if profile_callback:
                self._add_sample("usb_h264.native.callback_packetize", profile_stage)
            self._debug_log_packetize("native", packet_index, packet, chunks, chunk_size)
            self._record_h264_unit("native", packet, chunks, keyframe=bool(keyframe))
            profile_stage = time.perf_counter() if profile_callback else 0.0
            for chunk in chunks:
                packet_queue.put((chunk, False), timeout=NATIVE_PACKET_QUEUE_PUT_TIMEOUT_S)
                self._record_h264_queue_depth(packet_queue.qsize())
            if profile_callback:
                self._add_sample("usb_h264.native.callback_queue", profile_stage)
                self._add_sample("usb_h264.native.callback_total", profile_total)
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

    def _native_size_value(self, name: str) -> int:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None:
            return 0
        try:
            getter = getattr(lib, name)
        except AttributeError:
            return 0
        return int(getter(handle))

    def _add_native_timing_samples(self, lib: ctypes.CDLL, handle: int) -> None:
        for sample_name, getter_name in NATIVE_TIMING_GETTERS:
            try:
                getter = getattr(lib, getter_name)
            except AttributeError:
                return
            value_us = int(getter(handle))
            if value_us > 0:
                self._add_sample_value(sample_name, value_us / 1000.0)

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
                f"H264 ffmpeg RGBA input is {view.nbytes} bytes, expected at least {byte_count}"
            )

        offset = 0
        while offset < byte_count:
            written = os.write(fd, view[offset:byte_count])
            if written <= 0:
                raise BrokenPipeError("H264 ffmpeg encoder pipe wrote zero bytes")
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
                self._debug_stdout_reads += 1
                read_index = self._debug_stdout_reads
                self._debug_stdout_bytes += len(chunk)
                self._debug_max_packet_bytes = max(self._debug_max_packet_bytes, len(chunk))
                self._debug_log_stdout_read(read_index, self.backend_name, chunk)
                chunks = self._packetize_h264_for_usb(chunk, chunk_size)
                self._debug_log_packetize(self.backend_name, read_index, chunk, chunks, chunk_size)
                self._write_dump(chunk)
                self._record_h264_unit(self.backend_name, chunk, chunks)
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
                item = packet_queue.get()
                if item is None:
                    return
                chunk, is_last = item
                self._send_h264_chunk(chunk, chunk_size, source="native", is_last=is_last)
        except BaseException as exc:
            with self._condition:
                if not self._closing:
                    self._error = exc
                self._condition.notify_all()

    def _send_h264_chunk(self, chunk: bytes, chunk_size: int, *, source: str, is_last: bool = False) -> None:
        self._chunks_sent += 1
        self._debug_usb_bytes += len(chunk)
        self._debug_max_chunk_bytes = max(self._debug_max_chunk_bytes, len(chunk))
        if self.debug and (
            self._chunks_sent <= H264_DEBUG_CHUNK_LIMIT
            or self._chunks_sent % H264_DEBUG_CHUNK_INTERVAL == 0
            or is_last
        ):
            ack_mode = "soft" if self.wait_for_ack and self.soft_ack else ("on" if self.wait_for_ack else "off")
            summary = _h264_packet_summary(chunk)
            print(
                f"H264 chunk {self._chunks_sent}: {source} {len(chunk)} bytes "
                f"limit={chunk_size} head={_bytes_head(chunk, 8)} ack={ack_mode} "
                f"last={1 if is_last else 0} total_usb={self._debug_usb_bytes} {summary}",
                flush=True,
            )
        profile_stage = time.perf_counter()
        self.usb_display.send_h264_chunk(
            chunk,
            is_last=is_last,
            wait_for_ack=self.wait_for_ack,
            require_ack_response=not self.soft_ack,
        )
        elapsed_ms = (time.perf_counter() - profile_stage) * 1000.0
        self._add_sample_value("usb_h264.send_chunk", elapsed_ms)
        self._record_h264_send(source, len(chunk), elapsed_ms)

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
        self._add_sample_value(name, milliseconds)

    def _add_sample_value(self, name: str, milliseconds: float) -> None:
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
