"""C4 renders the existing HUD; S3 forwards the resulting TURZX USB packets."""
from collections import deque
import time

from cluster_usb_display import TuringUsbDisplay, _set_cluster_hud_connected
from cluster_wifi_protocol import Connection, load_config


class S3WifiDisplay(TuringUsbDisplay):
    # The bridge expires idle TCP clients after two seconds. Graphics/font
    # initialization can exceed that, so connect only once the renderer is ready.
    open_after_renderer = True
    h264_preserve_reference_frames = True
    uses_host_usb = False

    def __init__(self, config: dict, **kwargs):
        super().__init__(**kwargs)
        self.config = config
        self.connection = None
        self.pending = deque()
        self._h264_active = False
        self.landscape_width, self.landscape_height = 1920, 462

    def open(self) -> None:
        if self.expected_product_id not in (None, 0x0092):
            raise RuntimeError('S3 bridge currently supports the TURZX 9.2-inch display')
        self._load_protocol_helpers()
        try:
            self.connection = Connection(self.config)
            self.dev = self.connection
            self.dev_pid = 0x0092
            self.landscape_width, self.landscape_height = 1920, 462
            self._disconnected = False
            self._send_command(10, 'sync')
            time.sleep(0.2)
            self._send_command(14, 'brightness', {8: int(self.brightness / 100 * 102)},
                               expect_response=self.brightness == 0, log=False)
            _set_cluster_hud_connected(True)
        except Exception as exc:
            self.close()
            raise RuntimeError('S3 display connection failed; waiting for rediscovery') from exc

    def _result(self) -> bytes:
        expected = self.pending.popleft()
        seq, response, _ = self.connection.result()
        if seq != expected:
            raise ConnectionError('S3 acknowledgement sequence mismatch')
        return response

    def _transaction(self, payload: bytes, *, frame: bool = False, expect_reply: bool = True) -> bytes:
        with self._usb_lock:
            try:
                if self.connection is None:
                    raise ConnectionError('S3 display is closed')
                if frame:
                    # Bound outstanding packets; H264 reference packets must stay ordered.
                    while len(self.pending) >= 3:
                        self._result()
                else:
                    while self.pending:
                        self._result()
                self.pending.append(self.connection.send(payload, expect_reply=expect_reply))
                return b'' if frame else self._result()
            except (OSError, ValueError) as exc:
                self._disconnected = True
                _set_cluster_hud_connected(False)
                if self.connection is not None:
                    self.connection.close()
                    self.connection = None
                raise RuntimeError('S3 display disconnected; reconnect required') from exc

    def send_jpeg(self, jpeg_data: bytes) -> None:
        self._transaction(self._build_frame_payload(self._cmd_upload_jpeg, jpeg_data), frame=True)

    def _send_frame_no_ack(self, command_id: int, frame: bytes, *, drain_input: bool) -> None:
        self._transaction(self._build_frame_payload(command_id, frame), expect_reply=False)

    def _h264_chunk_size(self, requested_chunk_size: int) -> int:
        # Wi-Fi forwarding does not share C4's USB bus with the eGPU.
        return max(512, min(requested_chunk_size or 65536, 65536))

    def start_h264_stream(self, requested_chunk_size: int = 0) -> int:
        self._h264_active = True
        super().start_h264_stream(requested_chunk_size)
        self._h264_chunk_gap_s = 0.0
        return self._h264_chunk_size(requested_chunk_size)

    def send_h264_chunk(self, chunk: bytes, *, is_last: bool = False,
                        wait_for_ack: bool = False, require_ack_response: bool = True) -> None:
        if chunk:
            self._transaction(self._build_h264_chunk_payload(chunk, is_last=is_last),
                              frame=not wait_for_ack, expect_reply=wait_for_ack and require_ack_response)

    def stop_h264_stream(self) -> None:
        if self._h264_active:
            try:
                super().stop_h264_stream()
            finally:
                self._h264_active = False

    def _write_payload_checked(self, payload: bytes, error_message: str, timeout_ms: int) -> bytes:
        return self._transaction(payload)

    def _write_payload_no_ack(self, payload: bytes, error_message: str, timeout_ms: int) -> None:
        self._transaction(payload, expect_reply=False)

    def _drain_input(self, **kwargs) -> None:
        pass  # Responses are sequence checked by the transport; never discard TCP bytes.

    def close(self) -> None:
        _set_cluster_hud_connected(False)
        if self.connection is not None:
            try:
                if not self._disconnected:
                    if self._h264_active:
                        self.stop_h264_stream()
                    self._send_command(10, 'sync-off', log=False)
                    time.sleep(0.2)
                    self._send_command(14, 'brightness-off', {8: 0}, log=False)
            except (RuntimeError, OSError):
                pass
            finally:
                if self.connection is not None:
                    self.connection.close()
                self.connection = None
        self.pending.clear()
        self._h264_active = False
        self.dev = self.dev_pid = None


def create_display(**kwargs) -> TuringUsbDisplay:
    config = load_config()
    return TuringUsbDisplay(**kwargs) if config is None else S3WifiDisplay(config, **kwargs)
