from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass, replace
import json
import socket
import threading
import time
from typing import Any

from aiohttp import web

from openpilot.selfdrive.carrot.carrot_navi import (
    CATALOG,
    MAP_HZ_MAX,
    CarrotNaviReceiver,
    ItemRecord,
    create_app,
)
from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload

from cluster_h264_decoder import HardwareH264DecoderError, create_tici_h264_decoder
from cluster_models import (
    ClusterUiState,
    NaviDashboardState,
    NaviItemStatus,
    NaviMediaFrame,
    SimulatorInput,
)
from cluster_navi import parse_carrot_navi
from cluster_simulator import ClusterSimulator


DISCOVERY_PORT = 7705
DEFAULT_NAVI_HOST = "0.0.0.0"
DEFAULT_NAVI_PORT = 7714
MAP_FRAME_STALE_TIMEOUT_MS = 3000
NAVI_IPC_DISCONNECT_TIMEOUT_S = 3.0
H264_FLAG_KEYFRAME = 1
H264_DECODE_QUEUE_MAX = MAP_HZ_MAX


def detect_advertise_ip(bind_host: str) -> str:
    if bind_host not in ("", "0.0.0.0", "::"):
        return bind_host
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return str(probe.getsockname()[0])
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        probe.close()


class DiscoveryBeacon:
    def __init__(self, advertise_ip: str, interval_s: float = 1.0) -> None:
        self.advertise_ip = advertise_ip
        self.interval_s = max(0.2, float(interval_s))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="cluster-navi-discovery", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None

    def _run(self) -> None:
        body = json.dumps({"ip": self.advertise_ip, "navi_debug": 1}).encode("utf-8")
        while not self._stop.is_set():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.sendto(body, ("255.255.255.255", DISCOVERY_PORT))
            except OSError:
                pass
            finally:
                sock.close()
            self._stop.wait(self.interval_s)


class EmbeddedNaviReceiverServer:
    def __init__(self, receiver: CarrotNaviReceiver, host: str, port: int) -> None:
        self.receiver = receiver
        self.host = host
        self.port = int(port)
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._runner: web.AppRunner | None = None
        self._started = threading.Event()
        self._error: BaseException | None = None

    def start(self, timeout_s: float = 5.0) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="cluster-navi-receiver", daemon=True)
        self._thread.start()
        if not self._started.wait(timeout_s):
            raise RuntimeError(f"timed out starting navi receiver on {self.host}:{self.port}")
        if self._error is not None:
            raise RuntimeError(f"unable to start navi receiver on {self.host}:{self.port}: {self._error}")

    def close(self) -> None:
        loop = self._loop
        runner = self._runner
        if loop is not None and loop.is_running():
            if runner is not None:
                future = asyncio.run_coroutine_threadsafe(runner.cleanup(), loop)
                try:
                    future.result(timeout=2.0)
                except Exception:
                    pass
            loop.call_soon_threadsafe(loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    def _run(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            runner = web.AppRunner(create_app(self.receiver), access_log=None)
            self._runner = runner
            loop.run_until_complete(runner.setup())
            loop.run_until_complete(web.TCPSite(runner, self.host, self.port).start())
        except BaseException as exc:
            self._error = exc
            self._started.set()
            try:
                if self._runner is not None:
                    loop.run_until_complete(self._runner.cleanup())
            finally:
                loop.close()
            return
        self._started.set()
        try:
            loop.run_forever()
        finally:
            if self._runner is not None:
                try:
                    loop.run_until_complete(self._runner.cleanup())
                except Exception:
                    pass
            loop.close()


@dataclass(frozen=True)
class DecodedH264Frame:
    width: int
    height: int
    planes: tuple[bytes, bytes, bytes] | None = None
    strides: tuple[int, int, int] | None = None
    hardware_buffer: object | None = None


class H264Decoder:
    def __init__(self) -> None:
        self._av = None
        self._codec = None
        self._hardware = None
        self._hardware_disabled = False
        self._hardware_error_logged = False
        self._hardware_started_logged = False
        self._software_started_logged = False

    def _create_codec(self):
        if self._av is None:
            try:
                import av
            except ImportError as exc:
                raise RuntimeError("PyAV is required for cluster --input navi H.264 map frames") from exc
            self._av = av
        return self._av.CodecContext.create("h264", "r")

    def reset(self) -> None:
        if self._hardware is not None:
            self._hardware.close()
            self._hardware = None
        if self._codec is not None:
            self._codec = self._create_codec()

    def close(self) -> None:
        if self._hardware is not None:
            self._hardware.close()
            self._hardware = None
        self._codec = None

    def decode(
        self,
        access_unit: bytes,
        *,
        sequence: int = 0,
        width: int = 0,
        height: int = 0,
    ) -> DecodedH264Frame | None:
        hardware_disabled = getattr(self, "_hardware_disabled", True)
        if not hardware_disabled and width > 0 and height > 0:
            try:
                if self._hardware is None:
                    self._hardware = create_tici_h264_decoder(width, height)
                    if self._hardware is None:
                        self._hardware_disabled = True
                if self._hardware is not None:
                    if not self._hardware_started_logged:
                        print(
                            f"Navi H264 decoder: Qualcomm VIDC NV12 DMA-BUF tid={threading.get_native_id()}",
                            flush=True,
                        )
                        self._hardware_started_logged = True
                    hardware_buffer = self._hardware.decode(access_unit, sequence)
                    if hardware_buffer is None:
                        return None
                    return DecodedH264Frame(
                        width=hardware_buffer.width,
                        height=hardware_buffer.height,
                        hardware_buffer=hardware_buffer,
                    )
            except (HardwareH264DecoderError, OSError, ValueError) as exc:
                if self._hardware is not None:
                    self._hardware.close()
                    self._hardware = None
                self._hardware_disabled = True
                if not self._hardware_error_logged:
                    print(f"Qualcomm VIDC H264 decode disabled; using PyAV: {exc}", flush=True)
                    self._hardware_error_logged = True

        if self._codec is None:
            self._codec = self._create_codec()
            if not self._software_started_logged:
                print(f"Navi H264 decoder: PyAV software tid={threading.get_native_id()}", flush=True)
                self._software_started_logged = True
        try:
            frames = self._codec.decode(self._av.Packet(access_unit))
        except self._av.FFmpegError:
            self.reset()
            return None
        if not frames:
            return None
        frame = frames[-1]
        if frame.format.name != "yuv420p":
            frame = frame.reformat(format="yuv420p")
        if len(frame.planes) < 3:
            return None
        return DecodedH264Frame(
            width=int(frame.width),
            height=int(frame.height),
            planes=(bytes(frame.planes[0]), bytes(frame.planes[1]), bytes(frame.planes[2])),
            strides=(
                int(frame.planes[0].line_size),
                int(frame.planes[1].line_size),
                int(frame.planes[2].line_size),
            ),
        )


@dataclass(frozen=True)
class _H264DecodeRequest:
    epoch: int
    key: str
    sequence: int
    payload: bytes
    config_payload: bytes
    config_sequence: int
    keyframe: bool
    width: int = 0
    height: int = 0
    reset_decoder: bool = False


@dataclass(frozen=True)
class _H264DecodeResult:
    epoch: int
    frame: NaviMediaFrame


def _is_new_h264_result(
    result: _H264DecodeResult,
    media: dict[str, NaviMediaFrame],
    epoch: int,
) -> bool:
    if result.epoch != epoch:
        return False
    current = media.get(result.frame.key)
    return current is None or result.frame.sequence > current.sequence


class H264DecodeWorker:
    def __init__(self, decoder_factory=H264Decoder) -> None:
        self._decoder_factory = decoder_factory
        self._condition = threading.Condition()
        self._pending: deque[_H264DecodeRequest] = deque()
        self._results: dict[str, _H264DecodeResult] = {}
        self._waiting_for_keyframe: set[str] = set()
        self._thread: threading.Thread | None = None
        self._stopped = False
        self._dropped_requests = 0
        self._last_error = ""

    def submit(self, request: _H264DecodeRequest) -> None:
        with self._condition:
            if self._stopped:
                return
            self._ensure_thread_locked()
            if request.keyframe:
                reset_decoder = request.key in self._waiting_for_keyframe
                retained = deque(item for item in self._pending if item.key != request.key)
                self._dropped_requests += len(self._pending) - len(retained)
                self._pending = retained
                self._waiting_for_keyframe.discard(request.key)
                request = replace(request, reset_decoder=reset_decoder)
            elif request.key in self._waiting_for_keyframe:
                self._dropped_requests += 1
                return
            elif sum(item.key == request.key for item in self._pending) >= H264_DECODE_QUEUE_MAX:
                retained = deque(item for item in self._pending if item.key != request.key)
                self._dropped_requests += len(self._pending) - len(retained) + 1
                self._pending = retained
                self._waiting_for_keyframe.add(request.key)
                return
            self._pending.append(request)
            self._condition.notify()

    def discard(self, key: str) -> None:
        with self._condition:
            self._pending = deque(item for item in self._pending if item.key != key)
            self._results.pop(key, None)
            self._waiting_for_keyframe.discard(key)

    def reset(self) -> None:
        with self._condition:
            self._pending.clear()
            self._results.clear()
            self._waiting_for_keyframe.clear()

    def poll(self) -> tuple[_H264DecodeResult, ...]:
        with self._condition:
            results = tuple(self._results.values())
            self._results.clear()
            return results

    @property
    def dropped_requests(self) -> int:
        with self._condition:
            return self._dropped_requests

    def close(self) -> None:
        with self._condition:
            self._stopped = True
            self._pending.clear()
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)

    def _ensure_thread_locked(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="cluster-navi-h264-decode", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        decoders: dict[str, H264Decoder] = {}
        decoder_configs: dict[str, tuple[bytes, int, int]] = {}
        decoder_epochs: dict[str, int] = {}
        while True:
            with self._condition:
                while not self._pending and not self._stopped:
                    self._condition.wait()
                if self._stopped:
                    for decoder in decoders.values():
                        close_decoder = getattr(decoder, "close", None)
                        if close_decoder is not None:
                            close_decoder()
                    return
                request = self._pending.popleft()

            try:
                decoder = decoders.get(request.key)
                if decoder is None:
                    decoder = self._decoder_factory()
                    decoders[request.key] = decoder
                decoder_config = (request.config_payload, request.width, request.height)
                config_changed = decoder_configs.get(request.key) != decoder_config
                epoch_changed = decoder_epochs.get(request.key) != request.epoch
                if request.reset_decoder or config_changed or epoch_changed:
                    if request.key in decoder_epochs:
                        decoder.reset()
                    decoder_configs[request.key] = decoder_config
                    decoder_epochs[request.key] = request.epoch
                decoded = decoder.decode(
                    request.config_payload + request.payload,
                    sequence=request.sequence,
                    width=request.width,
                    height=request.height,
                )
                if decoded is None:
                    continue
                hardware_buffer = decoded.hardware_buffer
                decoded_sequence = int(getattr(hardware_buffer, "sequence", request.sequence))
                result = _H264DecodeResult(
                    epoch=request.epoch,
                    frame=NaviMediaFrame(
                        key=request.key,
                        sequence=decoded_sequence,
                        present=True,
                        mime="video/nv12-dmabuf" if hardware_buffer is not None else "image/yuv420p",
                        width=decoded.width,
                        height=decoded.height,
                        plane_data=decoded.planes,
                        plane_strides=decoded.strides,
                        hardware_buffer=hardware_buffer,
                    ),
                )
                with self._condition:
                    self._results[request.key] = result
                    self._last_error = ""
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                with self._condition:
                    should_log = error != self._last_error
                    self._last_error = error
                    self._waiting_for_keyframe.add(request.key)
                if should_log:
                    print(f"Navi H264 decode worker failed: {error}", flush=True)


def _bool_word(value: Any, true_text: str, false_text: str) -> str:
    return true_text if bool(value) else false_text


def _compact_app_status(record: ItemRecord | None) -> str:
    value = record.value if record is not None and record.present and isinstance(record.value, dict) else {}
    if not value:
        return "APP waiting"
    return "APP " + " ".join((
        _bool_word(value.get("foreground"), "FG", "BG"),
        _bool_word(value.get("window_focused"), "FOCUS", "NO-FOCUS"),
        _bool_word(value.get("main_map_visible"), "MAP", "NO-MAP"),
        _bool_word(value.get("ui_capture_available"), "CAPTURE", "NO-CAPTURE"),
    ))


def _compact_camera_status(record: ItemRecord | None) -> str:
    value = record.value if record is not None and record.present and isinstance(record.value, dict) else {}
    if not value:
        return "CAM waiting"
    mode = str(value.get("camera_mode", "-")).upper()
    level = value.get("view_level", "-")
    sub = value.get("view_sub_level", "-")
    tilt = value.get("tilt", "-")
    bearing = value.get("bearing", "-")
    return f"CAM {mode} L{level}.{sub} T{tilt} B{bearing}"


def _compact_composition_status(record: ItemRecord | None) -> str:
    value = record.value if record is not None and record.present and isinstance(record.value, dict) else {}
    if not value:
        return "UI waiting"
    active = [
        key.removesuffix("_active").replace("_", " ").upper()
        for key, enabled in value.items()
        if key != "generation" and bool(enabled)
    ]
    return "UI " + (" / ".join(active[:5]) if active else "MAP ONLY")


def _ipc_value(data: Any, name: str, default: Any = None) -> Any:
    if isinstance(data, dict):
        return data.get(name, default)
    try:
        return getattr(data, name)
    except Exception:
        return default


class NaviIpcMediaSource:
    """Projects standalone carrot_navi media messages into the cluster dashboard."""

    def __init__(self, messaging_module: Any | None = None) -> None:
        if messaging_module is None:
            import openpilot.cereal.messaging as messaging_module

        self.messaging = messaging_module
        self._socket = messaging_module.sub_sock("carrotNaviMedia", conflate=False)
        self._session_id = ""
        self._media: dict[str, NaviMediaFrame] = {}
        self._item_status: dict[str, NaviItemStatus] = {}
        self._media_epoch = 0
        self._h264_requested_sequences: dict[str, int] = {}
        self._h264_worker = H264DecodeWorker()
        self._h264_configs: dict[str, tuple[int, bytes]] = {}
        self._last_media_at_s = 0.0
        self._map_frame_at_s = 0.0
        self._received_count = 0
        self._error: str | None = None

    def update(self, navi_live: Any | None) -> NaviDashboardState:
        now_s = time.monotonic()
        for event in self.messaging.drain_sock(self._socket):
            try:
                self._consume(event.carrotNaviMedia, now_s)
            except Exception as exc:
                self._error = str(exc)[:256]
        self._apply_h264_results(now_s)

        navi_session = str(getattr(navi_live, "session_id", "") or "")
        if not self._session_id and navi_session:
            self._session_id = navi_session

        connected = bool(
            navi_live is not None
            or (self._last_media_at_s > 0.0 and now_s - self._last_media_at_s <= NAVI_IPC_DISCONNECT_TIMEOUT_S)
        )
        map_age_ms = int(max(0.0, now_s - self._map_frame_at_s) * 1000) if self._map_frame_at_s > 0.0 else None
        map_stalled = bool(connected and map_age_ms is not None and map_age_ms > MAP_FRAME_STALE_TIMEOUT_MS)
        if map_stalled:
            self._media.pop("render:map_main", None)
        if not connected and self._last_media_at_s > 0.0:
            self._clear_media()

        items = tuple(
            self._item_status.get(
                f"{kind}:{name}",
                NaviItemStatus(f"{kind}:{name}", 0, False, "waiting"),
            )
            for kind, name in CATALOG
        )
        return NaviDashboardState(
            connected=connected,
            endpoint="ipc://carrotNaviMedia",
            session_id=self._session_id,
            received_count=self._received_count,
            last_received_age_ms=(
                int(max(0.0, now_s - self._last_media_at_s) * 1000)
                if self._last_media_at_s > 0.0 else None
            ),
            map_frame_age_ms=map_age_ms,
            map_stream_stalled=map_stalled,
            error=self._error,
            media=tuple(self._media[key] for key in sorted(self._media)),
            items=items,
        )

    def close(self) -> None:
        self._clear_media()
        self._h264_worker.close()
        self._socket = None

    def _consume(self, data: Any, now_s: float) -> None:
        if int(_ipc_value(data, "schemaVersion", 0)) != 1:
            return
        session_id = str(_ipc_value(data, "sessionId", "") or "")
        if session_id and session_id != self._session_id:
            self._clear_media()
            self._session_id = session_id

        kind = str(_ipc_value(data, "kind", "") or "")
        name = str(_ipc_value(data, "name", "") or "")
        if kind not in ("image", "render") or not name:
            return
        key = f"{kind}:{name}"
        sequence = max(0, int(_ipc_value(data, "sequence", 0)))
        present = bool(_ipc_value(data, "present", False))
        reason = str(_ipc_value(data, "reason", "") or "") or None
        self._item_status[key] = NaviItemStatus(key, sequence, present, reason)
        self._last_media_at_s = now_s
        self._received_count += 1

        message_type = int(_ipc_value(data, "messageType", 0))
        payload = bytes(_ipc_value(data, "payload", b"") or b"")
        if not present or message_type == 4:
            self._h264_requested_sequences[key] = sequence
            self._h264_worker.discard(key)
            self._media[key] = NaviMediaFrame(key, sequence, False, reason=reason)
            if key == "render:map_main":
                self._map_frame_at_s = 0.0
            return

        width = max(0, int(_ipc_value(data, "width", 0)))
        height = max(0, int(_ipc_value(data, "height", 0)))
        format_or_reason = int(_ipc_value(data, "formatOrReason", 0))
        if message_type == 1:
            self._h264_requested_sequences[key] = sequence
            self._h264_worker.discard(key)
            mime = "image/png" if format_or_reason == 1 else "image/jpeg"
            self._media[key] = NaviMediaFrame(key, sequence, True, mime, width, height, payload)
        elif message_type == 2:
            self._h264_configs[key] = (sequence, payload)
            return
        elif message_type == 3:
            config_sequence, config_payload = self._h264_configs.get(key, (-1, b""))
            self._h264_requested_sequences[key] = sequence
            self._h264_worker.submit(
                _H264DecodeRequest(
                    epoch=self._media_epoch,
                    key=key,
                    sequence=sequence,
                    payload=payload,
                    config_payload=config_payload,
                    config_sequence=config_sequence,
                    keyframe=bool(int(_ipc_value(data, "flags", 0)) & H264_FLAG_KEYFRAME),
                    width=width,
                    height=height,
                )
            )
            return
        else:
            return

        if key == "render:map_main":
            self._map_frame_at_s = now_s

    def _apply_h264_results(self, now_s: float) -> None:
        for result in self._h264_worker.poll():
            frame = result.frame
            if not _is_new_h264_result(result, self._media, self._media_epoch):
                continue
            self._media[frame.key] = frame
            if frame.key == "render:map_main":
                self._map_frame_at_s = now_s

    def _clear_media(self) -> None:
        self._media.clear()
        self._media_epoch += 1
        self._h264_requested_sequences.clear()
        self._h264_worker.reset()
        self._h264_configs.clear()
        self._map_frame_at_s = 0.0


class NaviSimulatorSource:
    def __init__(
        self,
        host: str = DEFAULT_NAVI_HOST,
        port: int = DEFAULT_NAVI_PORT,
        advertise_ip: str | None = None,
        beacon_enabled: bool = True,
        publish_cereal: bool = False,
        map_theme: str = "dark",
    ) -> None:
        self.host = host
        self.port = int(port)
        self.advertise_ip = advertise_ip or detect_advertise_ip(host)
        self.endpoint = f"{self.advertise_ip}:{self.port}"
        self.receiver = CarrotNaviReceiver(port=self.port, map_theme=map_theme)
        self.server = EmbeddedNaviReceiverServer(self.receiver, host, self.port)
        self.beacon = DiscoveryBeacon(self.advertise_ip) if beacon_enabled else None
        self._simulator = ClusterSimulator()
        self._last_update_s = time.monotonic()
        self._last_state_generation = -1
        self._last_media_generation = -1
        self._last_session_id = ""
        self._navi_live = None
        self._media: dict[str, NaviMediaFrame] = {}
        self._media_epoch = 0
        self._h264_requested_sequences: dict[str, int] = {}
        self._h264_worker = H264DecodeWorker()
        self._map_frame_age_ms: int | None = None
        self._map_stream_stalled = False
        self._last_snapshot: dict[str, Any] = {}
        self._publisher = None
        try:
            self.server.start()
            if publish_cereal:
                from openpilot.selfdrive.carrot.carrot_navi_cereal import CarrotNaviCerealPublisher

                self._publisher = CarrotNaviCerealPublisher(self.receiver)
                self._publisher.start()
            if self.beacon is not None:
                self.beacon.start()
        except Exception:
            self.close()
            raise
        discovery = self.advertise_ip if self.beacon is not None else "off"
        print(f"Navi receiver listening on {self.host}:{self.port}; discovery={discovery}", flush=True)

    def close(self) -> None:
        if self.beacon is not None:
            self.beacon.close()
        if self._publisher is not None:
            self._publisher.stop()
            self._publisher = None
        self.server.close()
        self._h264_worker.close()

    def update(self) -> ClusterUiState:
        now_s = time.monotonic()
        dt = max(0.001, min(0.25, now_s - self._last_update_s))
        self._last_update_s = now_s
        snapshot = self.receiver.dashboard_snapshot()
        self._last_snapshot = snapshot
        session_id = str(snapshot.get("session_id", ""))
        connected = bool(snapshot.get("connected"))
        if session_id != self._last_session_id:
            self._last_session_id = session_id
            self._last_media_generation = -1
            self._clear_media()
        elif not connected and (self._media or self._h264_requested_sequences):
            self._clear_media()
        if int(snapshot["state_generation"]) != self._last_state_generation:
            self._last_state_generation = int(snapshot["state_generation"])
            payload = build_carrot_navi_payload(self.receiver.cereal_snapshot())
            self._navi_live = parse_carrot_navi(payload, now=now_s)
        if int(snapshot["media_generation"]) != self._last_media_generation:
            self._last_media_generation = int(snapshot["media_generation"])
            self._update_media(snapshot)
        self._apply_h264_results()
        self._update_map_liveness(snapshot)

        base = self._simulator.update(SimulatorInput(), dt)
        navi = self._navi_live
        speed_kph = 0.0
        speed_limit_kph = None
        if navi is not None:
            if navi.vehicle is not None:
                speed_kph = navi.vehicle.speed_kph
            elif navi.speed is not None:
                speed_kph = navi.speed.current_kph
            if navi.speed is not None:
                speed_limit_kph = navi.speed.road_limit_kph
        return replace(
            base,
            speed_kph=speed_kph,
            speed_limit_kph=speed_limit_kph,
            speed_limit_source="n" if speed_limit_kph is not None else None,
            onroad=bool(snapshot["connected"]),
            navi_live=navi,
            navi_dashboard=self._dashboard(snapshot),
            center_clock_text=time.strftime("%H:%M:%S"),
        )

    def status_text(self) -> str:
        snapshot = self._last_snapshot
        if not snapshot:
            return f"navi {self.endpoint} starting"
        records = snapshot.get("records", {})
        map_record = records.get("render:map_main")
        map_sequence = map_record.sequence if map_record is not None else 0
        connected = int(bool(snapshot.get("connected")))
        app_version = snapshot.get("app_version") or "-"
        received = snapshot.get("received_count", 0)
        map_age = "-" if self._map_frame_age_ms is None else f"{self._map_frame_age_ms}ms"
        map_status = "stalled" if self._map_stream_stalled else map_age
        decode_drops = self._h264_worker.dropped_requests
        decode_status = f" decode_drop={decode_drops}" if decode_drops else ""
        return (
            f"navi connected={connected} app={app_version} items={len(records)}/28 "
            + f"received={received} map={map_sequence}/{map_status}{decode_status}"
        )

    def _clear_media(self) -> None:
        self._media.clear()
        self._media_epoch += 1
        self._h264_requested_sequences.clear()
        self._h264_worker.reset()
        self._map_frame_age_ms = None
        self._map_stream_stalled = False

    def _update_map_liveness(self, snapshot: dict[str, Any]) -> None:
        if not bool(snapshot.get("connected")):
            self._map_frame_age_ms = None
            self._map_stream_stalled = False
            return

        record = snapshot.get("records", {}).get("render:map_main")
        if record is None or not record.present:
            self._map_frame_age_ms = None
            self._map_stream_stalled = False
            return

        self._map_frame_age_ms = max(0, (time.time_ns() // 1_000_000) - int(record.received_at_ms))
        self._map_stream_stalled = self._map_frame_age_ms > MAP_FRAME_STALE_TIMEOUT_MS
        if self._map_stream_stalled:
            self._media.pop("render:map_main", None)

    def _update_media(self, snapshot: dict[str, Any]) -> None:
        records: dict[str, ItemRecord] = snapshot["records"]
        configs: dict[str, ItemRecord] = snapshot["binary_configs"]
        for kind, name in CATALOG:
            if kind == "json":
                continue
            key = f"{kind}:{name}"
            record = records.get(key)
            if record is None:
                continue
            if not record.present:
                self._h264_requested_sequences[key] = record.sequence
                self._h264_worker.discard(key)
                self._media[key] = NaviMediaFrame(
                    key=key,
                    sequence=record.sequence,
                    present=False,
                    reason=record.reason,
                )
                continue
            if self._h264_requested_sequences.get(key) == record.sequence:
                continue
            frame = self._decode_media_record(key, record, configs.get(key))
            if frame is not None:
                self._media[key] = frame

    def _apply_h264_results(self) -> None:
        for result in self._h264_worker.poll():
            frame = result.frame
            if not _is_new_h264_result(result, self._media, self._media_epoch):
                continue
            self._media[frame.key] = frame

    def _decode_media_record(
        self,
        key: str,
        record: ItemRecord,
        config: ItemRecord | None,
    ) -> NaviMediaFrame | None:
        payload = record.payload or b""
        if record.message_type == 1:
            self._h264_requested_sequences[key] = record.sequence
            self._h264_worker.discard(key)
            mime = "image/png" if record.format_or_reason == 1 else "image/jpeg"
            return NaviMediaFrame(
                key=key,
                sequence=record.sequence,
                present=True,
                mime=mime,
                width=record.width,
                height=record.height,
                data=payload,
            )
        if record.message_type != 3 or not payload:
            return None
        self._h264_requested_sequences[key] = record.sequence
        self._h264_worker.submit(
            _H264DecodeRequest(
                epoch=self._media_epoch,
                key=key,
                sequence=record.sequence,
                payload=payload,
                config_payload=config.payload if config is not None and config.payload else b"",
                config_sequence=config.sequence if config is not None else -1,
                keyframe=bool(record.flags & H264_FLAG_KEYFRAME),
                width=record.width,
                height=record.height,
            )
        )
        return None

    def _dashboard(self, snapshot: dict[str, Any]) -> NaviDashboardState:
        records: dict[str, ItemRecord] = snapshot["records"]
        now_millis = time.time_ns() // 1_000_000
        last_received = int(snapshot.get("last_received_at_ms", 0))
        age_ms = max(0, now_millis - last_received) if last_received > 0 else None
        item_status = tuple(
            NaviItemStatus(
                key=f"{kind}:{name}",
                sequence=records[f"{kind}:{name}"].sequence if f"{kind}:{name}" in records else 0,
                present=records[f"{kind}:{name}"].present if f"{kind}:{name}" in records else False,
                reason=records[f"{kind}:{name}"].reason if f"{kind}:{name}" in records else "waiting",
            )
            for kind, name in CATALOG
        )
        return NaviDashboardState(
            connected=bool(snapshot["connected"]),
            endpoint=self.endpoint,
            session_id=str(snapshot["session_id"]),
            app_version=str(snapshot["app_version"]),
            manifest_revision=int(snapshot["manifest_revision"]),
            received_count=int(snapshot["received_count"]),
            last_received_age_ms=age_ms,
            map_frame_age_ms=self._map_frame_age_ms,
            map_stream_stalled=self._map_stream_stalled,
            peer=str(snapshot["peer"]),
            error=str(snapshot["error"]) if snapshot["error"] else None,
            media=tuple(self._media[key] for key in sorted(self._media)),
            items=item_status,
            app_status=_compact_app_status(records.get("json:app_status")),
            camera_status=_compact_camera_status(records.get("json:camera_state")),
            composition_status=_compact_composition_status(records.get("json:composition_state")),
        )
