import asyncio
import json
import time
from types import SimpleNamespace
# for aiortc and its dependencies
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

from aiortc import RTCDataChannel
from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE
from aiohttp import web
import capnp
import pytest
from openpilot.cereal import messaging, log

import openpilot.system.webrtc.webrtcd as webrtcd
from openpilot.system.webrtc.webrtcd import CerealOutgoingMessageProxy, CerealIncomingMessageProxy
from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack


class TestStreamSession:
  def setup_method(self):
    self.loop = asyncio.new_event_loop()

  def teardown_method(self):
    self.loop.stop()
    self.loop.close()

  @staticmethod
  def _stream_request(mocker, streams, *, client_id: str, takeover: bool = False):
    request = mocker.Mock()
    request.remote = "192.168.0.2"
    request.app = {
      "streams": streams,
      "debug": False,
      "stream_lock": asyncio.Lock(),
    }
    request.json = mocker.AsyncMock(return_value={
      "sdp": "offer",
      "cameras": ["road"],
      "client_id": client_id,
      "takeover": takeover,
    })
    return request

  @staticmethod
  def _stream_session(mocker, identifier: str):
    session = mocker.Mock()
    session.identifier = identifier
    session.uses_road_camera = True
    session.get_answer = mocker.AsyncMock(return_value=SimpleNamespace(sdp="answer", type="answer"))
    session.post_run_cleanup = mocker.AsyncMock()
    session.stop = mocker.AsyncMock()
    return session

  def test_carrot_vision_rejects_foreign_road_without_takeover(self, mocker):
    owner = self._stream_session(mocker, "owner")
    owner.client_key = "client:owner"
    streams = {owner.identifier: owner}
    request = self._stream_request(mocker, streams, client_id="viewer")
    session_factory = mocker.patch.object(webrtcd, "StreamSession")
    mocker.patch.object(webrtcd, "_carrot_vision_mode", True)

    with pytest.raises(web.HTTPConflict) as exc_info:
      self.loop.run_until_complete(webrtcd.get_stream(request))

    payload = json.loads(exc_info.value.text)
    assert payload["code"] == webrtcd.CARROT_VISION_BUSY_CODE
    owner.stop.assert_not_awaited()
    session_factory.assert_not_called()

  def test_carrot_vision_takeover_replaces_foreign_road(self, mocker):
    owner = self._stream_session(mocker, "owner")
    owner.client_key = "client:owner"
    replacement = self._stream_session(mocker, "replacement")
    streams = {owner.identifier: owner}
    request = self._stream_request(mocker, streams, client_id="viewer", takeover=True)
    mocker.patch.object(webrtcd, "StreamSession", return_value=replacement)
    mocker.patch.object(webrtcd, "_carrot_vision_mode", True)

    response = self.loop.run_until_complete(webrtcd.get_stream(request))

    assert response.status == 200
    assert json.loads(response.body)["sdp"] == "answer"
    owner.stop.assert_awaited_once()
    replacement.start.assert_called_once()
    assert owner.identifier not in streams
    assert streams[replacement.identifier] is replacement

  def test_carrot_vision_reconnect_replaces_same_client_without_takeover(self, mocker):
    owner = self._stream_session(mocker, "owner")
    owner.client_key = "client:viewer"
    replacement = self._stream_session(mocker, "replacement")
    streams = {owner.identifier: owner}
    request = self._stream_request(mocker, streams, client_id="viewer")
    mocker.patch.object(webrtcd, "StreamSession", return_value=replacement)
    mocker.patch.object(webrtcd, "_carrot_vision_mode", True)

    response = self.loop.run_until_complete(webrtcd.get_stream(request))

    assert response.status == 200
    owner.stop.assert_awaited_once()
    replacement.start.assert_called_once()
    assert owner.identifier not in streams

  def test_carrot_vision_ignores_foreign_non_road_session(self, mocker):
    state_only = self._stream_session(mocker, "state-only")
    state_only.client_key = "client:owner"
    state_only.uses_road_camera = False
    replacement = self._stream_session(mocker, "replacement")
    streams = {state_only.identifier: state_only}
    request = self._stream_request(mocker, streams, client_id="viewer")
    mocker.patch.object(webrtcd, "StreamSession", return_value=replacement)
    mocker.patch.object(webrtcd, "_carrot_vision_mode", True)

    response = self.loop.run_until_complete(webrtcd.get_stream(request))

    assert response.status == 200
    state_only.stop.assert_not_awaited()
    replacement.start.assert_called_once()

  def test_outgoing_proxy(self, mocker):
    test_msg = log.Event.new_message()
    test_msg.logMonoTime = 123
    test_msg.valid = True
    test_msg.customReservedRawData0 = b"test"
    expected_dict = {"type": "customReservedRawData0", "logMonoTime": 123, "valid": True, "data": "test"}
    expected_json = json.dumps(expected_dict).encode()

    channel = mocker.Mock(spec=RTCDataChannel)
    mocked_submaster = messaging.SubMaster(["customReservedRawData0"])
    def mocked_update(t):
      mocked_submaster.update_msgs(0, [test_msg])

    mocker.patch.object(messaging.SubMaster, "update", side_effect=mocked_update)
    proxy = CerealOutgoingMessageProxy(mocked_submaster)
    proxy.add_channel(channel)

    proxy.update()

    channel.send.assert_called_once_with(expected_json)

  def test_incoming_proxy(self, mocker):
    tested_msgs = [
      {"type": "customReservedRawData0", "data": "test"}, # primitive
      {"type": "can", "data": [{"address": 0, "dat": "", "src": 0}]}, # list
      {"type": "testJoystick", "data": {"axes": [0, 0], "buttons": [False]}}, # dict
    ]

    mocked_pubmaster = mocker.MagicMock(spec=messaging.PubMaster)

    proxy = CerealIncomingMessageProxy(mocked_pubmaster)

    for msg in tested_msgs:
      proxy.send(json.dumps(msg).encode())

      mocked_pubmaster.send.assert_called_once()
      mt, md = mocked_pubmaster.send.call_args.args
      assert mt == msg["type"]
      assert isinstance(md, capnp._DynamicStructBuilder)
      assert hasattr(md, msg["type"])

      mocked_pubmaster.reset_mock()

  def test_livestream_track(self, mocker):
    fake_msg = messaging.new_message("livestreamDriverEncodeData")

    config = {"receive.return_value": fake_msg.to_bytes()}
    mocker.patch("msgq.SubSocket", spec=True, **config)
    track = LiveStreamVideoStreamTrack("driver")

    assert track.id.startswith("driver")
    assert track.codec_preference() == "H264"

    for i in range(5):
      packet = self.loop.run_until_complete(track.recv())
      assert packet.time_base == VIDEO_TIME_BASE
      if i == 0:
        start_ns = time.monotonic_ns()
        start_pts = packet.pts
      assert abs(i + packet.pts - (start_pts + (((time.monotonic_ns() - start_ns) * VIDEO_CLOCK_RATE) // 1_000_000_000))) < 450 #5ms
      assert packet.size == 0

