import json
import sys
from unittest.mock import AsyncMock, MagicMock, patch

if "capnp" not in sys.modules:
  sys.modules["capnp"] = MagicMock()
if "serial" not in sys.modules:
  sys.modules["serial"] = MagicMock()

import pytest
import openpilot.selfdrive.carrot.server.features.autotune as autotune


def test_detect_key_episodes():
  # Create synthetic frames with a departure event, harsh acceleration, and harsh decel
  frames = [
    # 1. Standstill
    {
      "t": 0.0, "vEgo": 0.0, "aEgo": 0.0, "aTarget": 0.0, "aCmd": 0.0,
      "gas": False, "brake": False, "steer": False, "standstill": True,
      "hasLead": True, "dRel": 3.0, "vLead": 0.0, "aLead": 0.0, "jLead": 0.0,
      "steerAngle": 0.0, "desiredSteerAngle": 0.0, "steerTorque": 0.0
    },
    # 2. Departure start
    {
      "t": 0.5, "vEgo": 2.5, "aEgo": 0.8, "aTarget": 1.2, "aCmd": 1.4,
      "gas": False, "brake": False, "steer": False, "standstill": False,
      "hasLead": True, "dRel": 4.5, "vLead": 8.0, "aLead": 1.5, "jLead": 0.5,
      "steerAngle": 0.0, "desiredSteerAngle": 0.0, "steerTorque": 0.0
    },
    # 3. Harsh catch-up acceleration spike (aTarget > 1.4)
    {
      "t": 1.5, "vEgo": 12.0, "aEgo": 1.6, "aTarget": 1.95, "aCmd": 2.1,
      "gas": False, "brake": False, "steer": False, "standstill": False,
      "hasLead": True, "dRel": 10.0, "vLead": 22.0, "aLead": 1.8, "jLead": 1.0,
      "steerAngle": 0.0, "desiredSteerAngle": 0.0, "steerTorque": 0.0
    },
    # 4. Harsh braking (aTarget < -1.8 without driver brake pedal)
    {
      "t": 5.0, "vEgo": 35.0, "aEgo": -0.2, "aTarget": 0.0, "aCmd": 0.0,
      "gas": False, "brake": False, "steer": False, "standstill": False,
      "hasLead": True, "dRel": 15.0, "vLead": 35.0, "aLead": 0.0, "jLead": 0.0,
      "steerAngle": 0.0, "desiredSteerAngle": 0.0, "steerTorque": 0.0
    },
    {
      "t": 5.5, "vEgo": 30.0, "aEgo": -1.5, "aTarget": -2.1, "aCmd": -2.2,
      "gas": False, "brake": False, "steer": False, "standstill": False,
      "hasLead": True, "dRel": 11.0, "vLead": 20.0, "aLead": -2.0, "jLead": -0.5,
      "steerAngle": 0.0, "desiredSteerAngle": 0.0, "steerTorque": 0.0
    },
    # 5. Steering error (> 3.0 deg at > 30 km/h)
    {
      "t": 10.0, "vEgo": 60.0, "aEgo": 0.0, "aTarget": 0.0, "aCmd": 0.0,
      "gas": False, "brake": False, "steer": False, "standstill": False,
      "hasLead": False, "dRel": 0.0, "vLead": 0.0, "aLead": 0.0, "jLead": 0.0,
      "steerAngle": 2.0, "desiredSteerAngle": 6.5, "steerTorque": 0.8
    },
    # 6. Driver overrides
    {
      "t": 15.0, "vEgo": 45.0, "aEgo": -0.5, "aTarget": 1.2, "aCmd": 1.2,
      "gas": False, "brake": True, "steer": False, "standstill": False,
      "hasLead": True, "dRel": 8.0, "vLead": 30.0, "aLead": -1.0, "jLead": 0.0,
      "steerAngle": 0.0, "desiredSteerAngle": 0.0, "steerTorque": 0.0
    },
    {
      "t": 20.0, "vEgo": 10.0, "aEgo": 0.2, "aTarget": 0.4, "aCmd": 0.4,
      "gas": True, "brake": False, "steer": False, "standstill": False,
      "hasLead": True, "dRel": 18.0, "vLead": 25.0, "aLead": 1.0, "jLead": 0.0,
      "steerAngle": 0.0, "desiredSteerAngle": 0.0, "steerTorque": 0.0
    },
    {
      "t": 25.0, "vEgo": 55.0, "aEgo": 0.0, "aTarget": 0.0, "aCmd": 0.0,
      "gas": False, "brake": False, "steer": True, "standstill": False,
      "hasLead": False, "dRel": 0.0, "vLead": 0.0, "aLead": 0.0, "jLead": 0.0,
      "steerAngle": 5.0, "desiredSteerAngle": 1.0, "steerTorque": 0.3
    }
  ]

  # Test default (excluded)
  episodes_default = autotune._detect_key_episodes(frames, include_driver_override=False)
  types_default = [e["type"] for e in episodes_default]
  assert "departure_start" in types_default
  assert "harsh_catchup_accel" in types_default
  assert "harsh_decel" in types_default
  assert "steering_tracking_error" in types_default
  assert "driver_brake_override" not in types_default
  assert "driver_gas_override" not in types_default
  assert "driver_steer_override" not in types_default

  # Test included
  episodes_included = autotune._detect_key_episodes(frames, include_driver_override=True)
  types_included = [e["type"] for e in episodes_included]
  assert "driver_brake_override" in types_included
  assert "driver_gas_override" in types_included
  assert "driver_steer_override" in types_included


@pytest.mark.asyncio
async def test_apply_params_and_rollback():
  mock_params_store = {
    "CruiseMaxVals1": "200",
    "DynamicTFollow": "30",
  }

  def mock_get_param_values(keys, defaults):
    return {k: mock_params_store.get(k, defaults.get(k, 0)) for k in keys}

  def mock_set_param_value(k, v, p=None):
    mock_params_store[k] = str(v)

  with patch.object(autotune, "HAS_PARAMS", True), \
       patch.object(autotune, "get_param_values", side_effect=mock_get_param_values), \
       patch.object(autotune, "set_param_value", side_effect=mock_set_param_value):

    # Apply new parameters
    req_mock = MagicMock()
    req_mock.json = AsyncMock(return_value={
      "params": {
        "CruiseMaxVals1": "145",
        "DynamicTFollow": "12",
      }
    })

    resp = await autotune.api_autotune_apply_params(req_mock)
    payload = json.loads(resp.text)

    assert payload["ok"] is True
    assert str(payload["applied"]["CruiseMaxVals1"]) == "145"
    assert str(payload["applied"]["DynamicTFollow"]) == "12"
    assert payload["rollbackSnapshot"] == {"CruiseMaxVals1": "200", "DynamicTFollow": "30"}
    assert mock_params_store["CruiseMaxVals1"] == "145"
    assert mock_params_store["DynamicTFollow"] == "12"

    # Rollback to previous snapshot
    req_rollback = MagicMock()
    req_rollback.json = AsyncMock(return_value={
      "params": payload["rollbackSnapshot"]
    })

    resp_rb = await autotune.api_autotune_apply_params(req_rollback)
    payload_rb = json.loads(resp_rb.text)

    assert payload_rb["ok"] is True
    assert mock_params_store["CruiseMaxVals1"] == "200"
    assert mock_params_store["DynamicTFollow"] == "30"


@pytest.mark.asyncio
async def test_extract_telemetry_missing_route():
  req_mock = MagicMock()
  req_mock.json = AsyncMock(return_value={"route": ""})
  req_mock.query.get.return_value = ""

  resp = await autotune.api_autotune_extract_telemetry(req_mock)
  assert resp.status == 400
  payload = json.loads(resp.text)
  assert payload["ok"] is False
