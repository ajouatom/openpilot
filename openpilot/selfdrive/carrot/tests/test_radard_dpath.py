import pytest
import math
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar import (
  effective_radar_track_mode,
)
from openpilot.selfdrive.carrot.radar_motion.coordinates import device_yaw_to_radar
from openpilot.selfdrive.carrot.radar_motion.predictor import project_to_model_path, radar_target_velocity_in_ego_frame
from openpilot.selfdrive.carrot.radar_motion.controller import _model_path
from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import _yaw_metadata
from openpilot.selfdrive.carrot.radar_motion.timing import front_radar_distance_delay_s, VOLKSWAGEN_MEB_FLAG


@pytest.mark.parametrize("side", (-1.0, 1.0))
def test_device_and_steering_yaw_match_radar_rotation_convention(side):
  yaw = side * 0.025
  pose = SimpleNamespace(angularVelocityDevice=SimpleNamespace(valid=True, z=-yaw), inputsOK=True, sensorsOK=True)
  measured, estimated, source = _yaw_metadata(10.0, 0.0, pose, 0.01, 14.0, 2.8)
  steering = math.degrees(math.atan(yaw * 2.8 / 10.0)) * 14.0
  fallback, _, _ = _yaw_metadata(10.0, steering, None, 1.0, 14.0, 2.8)
  assert measured == pytest.approx(yaw)
  assert fallback == pytest.approx(yaw)
  assert not estimated and source == "livePose"
  # A parallel target's apparent lateral motion comes entirely from ego
  # rotation. Correcting it must produce zero physical lateral velocity.
  _, lateral = radar_target_velocity_in_ego_frame(10.0, -yaw * 20.0, 20.0, side * 3.0, measured)
  assert lateral == pytest.approx(0.0)
  _, entering = radar_target_velocity_in_ego_frame(10.0, -side * 0.4 - yaw * 20.0, 20.0, side * 3.0, measured)
  assert entering == pytest.approx(-side * 0.4)


@pytest.mark.parametrize("value", (math.nan, math.inf, -math.inf))
def test_invalid_device_yaw_is_not_used(value):
  assert device_yaw_to_radar(value) == 0.0


@pytest.mark.parametrize("side", (-1.0, 1.0))
def test_model_path_is_converted_once_inside_projection(side):
  model = SimpleNamespace(position=SimpleNamespace(x=(0.0, 20.0), y=(0.0, -side * 2.0)))
  path = _model_path(model)
  assert path[-1][1] == -side * 2.0
  assert project_to_model_path(path, 20.0, side * 2.0).d_path == pytest.approx(0.0)


@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_hyundai_keeps_configured_radar_track_mode(configured_mode: int) -> None:
  assert effective_radar_track_mode(
    "hyundai", False, configured_mode,
  ) == configured_mode


@pytest.mark.parametrize(
  "brand", ("volkswagen", "honda", "toyota", "ford", "subaru"),
)
@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_other_brands_ignore_option_and_use_front_radar(
  brand: str,
  configured_mode: int,
) -> None:
  assert effective_radar_track_mode(
    brand, False, configured_mode,
  ) == 1


@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_other_brands_without_radar_use_vision(configured_mode: int) -> None:
  assert effective_radar_track_mode(
    "mazda", True, configured_mode,
  ) == -2


@pytest.mark.parametrize("flags", (VOLKSWAGEN_MEB_FLAG, VOLKSWAGEN_MEB_FLAG | 128, 16496))
def test_meb_distance_delay_does_not_change_ego_history(flags):
  from openpilot.cereal import car

  cp = car.CarParams.new_message(brand="volkswagen", flags=int(flags), radarDelay=0.8)
  assert front_radar_distance_delay_s(cp) == 0.0
  assert cp.radarDelay == pytest.approx(0.8)


@pytest.mark.parametrize("brand,flags", (("volkswagen", 0), ("hyundai", VOLKSWAGEN_MEB_FLAG), ("toyota", 0)))
def test_other_platforms_keep_distance_delay(brand, flags):
  cp = SimpleNamespace(brand=brand, flags=flags, radarDelay=0.1)
  assert front_radar_distance_delay_s(cp) == 0.1


@pytest.mark.parametrize("params_last", (False, True))
def test_meb_log_replay_uses_zero_distance_delay(tmp_path, params_last):
  from openpilot.cereal import log
  from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import load_frames

  params = log.Event.new_message(logMonoTime=4_000_000_000 if params_last else 1_000_000_000)
  cp = params.init("carParams")
  cp.brand = "volkswagen"
  cp.carFingerprint = "VOLKSWAGEN_ID4_MK1"
  cp.flags = VOLKSWAGEN_MEB_FLAG
  cp.radarDelay = 0.8
  tracks = log.Event.new_message(logMonoTime=2_000_000_000)
  tracks.init("liveTracks").init("points", 0)
  model = log.Event.new_message(logMonoTime=3_000_000_000)
  model.init("modelV2").timestampEof = 2_950_000_000
  path = tmp_path / "rlog"
  events = [tracks, model, params] if params_last else [params, tracks, model]
  path.write_bytes(b"".join(event.to_bytes() for event in events))
  frames = load_frames(path)
  assert len(frames) == 1
  assert frames[0].radar_delay_s == 0.0


def test_meb_flag_matches_persisted_vehicle_definition():
  import ast
  from pathlib import Path

  root = Path(__file__).resolve().parents[4]
  tree = ast.parse((root / "opendbc_repo/opendbc/car/volkswagen/values.py").read_text(encoding="utf-8"))
  flags = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "VolkswagenFlags")
  value = next(node.value for node in flags.body if isinstance(node, ast.Assign) and node.targets[0].id == "MEB")
  assert ast.literal_eval(value) == VOLKSWAGEN_MEB_FLAG
