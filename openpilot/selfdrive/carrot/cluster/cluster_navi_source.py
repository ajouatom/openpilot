from __future__ import annotations

import asyncio
from dataclasses import replace
import json
import socket
import threading
import time
from typing import Any

from aiohttp import web

from openpilot.selfdrive.carrot.carrot_navi import (
    CATALOG,
    CarrotNaviReceiver,
    ItemRecord,
    create_app,
)
from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload

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


class H264Decoder:
    def __init__(self) -> None:
        try:
            import av
        except ImportError as exc:
            raise RuntimeError("PyAV is required for cluster --input navi H.264 map frames") from exc
        self._av = av
        self._codec = self._create_codec()

    def _create_codec(self):
        return self._av.CodecContext.create("h264", "r")

    def reset(self) -> None:
        self._codec = self._create_codec()

    def decode(self, access_unit: bytes) -> tuple[bytes, int, int] | None:
        try:
            frames = self._codec.decode(self._av.Packet(access_unit))
        except self._av.FFmpegError:
            self.reset()
            return None
        if not frames:
            return None
        frame = frames[-1].reformat(format="rgba")
        plane = frame.planes[0]
        row_bytes = frame.width * 4
        raw = bytes(plane)
        if plane.line_size == row_bytes:
            pixels = raw[:row_bytes * frame.height]
        else:
            pixels = b"".join(
                raw[offset:offset + row_bytes]
                for offset in range(0, plane.line_size * frame.height, plane.line_size)
            )
        return pixels, int(frame.width), int(frame.height)


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


class NaviSimulatorSource:
    def __init__(
        self,
        host: str = DEFAULT_NAVI_HOST,
        port: int = DEFAULT_NAVI_PORT,
        advertise_ip: str | None = None,
        beacon_enabled: bool = True,
        publish_cereal: bool = False,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.advertise_ip = advertise_ip or detect_advertise_ip(host)
        self.endpoint = f"{self.advertise_ip}:{self.port}"
        self.receiver = CarrotNaviReceiver(port=self.port)
        self.server = EmbeddedNaviReceiverServer(self.receiver, host, self.port)
        self.beacon = DiscoveryBeacon(self.advertise_ip) if beacon_enabled else None
        self._simulator = ClusterSimulator()
        self._last_update_s = time.monotonic()
        self._last_state_generation = -1
        self._last_media_generation = -1
        self._navi_live = None
        self._media: dict[str, NaviMediaFrame] = {}
        self._h264_decoders: dict[str, H264Decoder] = {}
        self._h264_config_sequences: dict[str, int] = {}
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

    def update(self) -> ClusterUiState:
        now_s = time.monotonic()
        dt = max(0.001, min(0.25, now_s - self._last_update_s))
        self._last_update_s = now_s
        snapshot = self.receiver.dashboard_snapshot()
        self._last_snapshot = snapshot
        if int(snapshot["state_generation"]) != self._last_state_generation:
            self._last_state_generation = int(snapshot["state_generation"])
            payload = build_carrot_navi_payload(self.receiver.cereal_snapshot())
            self._navi_live = parse_carrot_navi(payload, now=now_s)
        if int(snapshot["media_generation"]) != self._last_media_generation:
            self._last_media_generation = int(snapshot["media_generation"])
            self._update_media(snapshot)

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
        return f"navi connected={connected} app={app_version} items={len(records)}/28 received={received} map={map_sequence}"

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
                self._media[key] = NaviMediaFrame(
                    key=key,
                    sequence=record.sequence,
                    present=False,
                    reason=record.reason,
                )
                continue
            previous = self._media.get(key)
            if previous is not None and previous.sequence == record.sequence:
                continue
            frame = self._decode_media_record(key, record, configs.get(key))
            if frame is not None:
                self._media[key] = frame

    def _decode_media_record(
        self,
        key: str,
        record: ItemRecord,
        config: ItemRecord | None,
    ) -> NaviMediaFrame | None:
        payload = record.payload or b""
        if record.message_type == 1:
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
        decoder = self._h264_decoders.get(key)
        if decoder is None:
            decoder = H264Decoder()
            self._h264_decoders[key] = decoder
        config_payload = b""
        if config is not None and config.payload:
            config_payload = config.payload
            if self._h264_config_sequences.get(key) != config.sequence:
                self._h264_config_sequences[key] = config.sequence
                decoder.reset()
        decoded = decoder.decode(config_payload + payload)
        if decoded is None:
            return None
        rgba, width, height = decoded
        return NaviMediaFrame(
            key=key,
            sequence=record.sequence,
            present=True,
            mime="image/rgba",
            width=width,
            height=height,
            data=rgba,
        )

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
            peer=str(snapshot["peer"]),
            error=str(snapshot["error"]) if snapshot["error"] else None,
            media=tuple(self._media[key] for key in sorted(self._media)),
            items=item_status,
            app_status=_compact_app_status(records.get("json:app_status")),
            camera_status=_compact_camera_status(records.get("json:camera_state")),
            composition_status=_compact_composition_status(records.get("json:composition_state")),
        )
