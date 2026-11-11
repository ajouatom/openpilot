"""Startup settings switch with recalibration on a confirmed vehicle change."""
import base64
import copy

PORTER = "HYUNDAI_PORTER_II_EV"
IONIQ5 = "HYUNDAI_IONIQ_5"
CAR_LABELS = {
  PORTER: "Hyundai Porter II EV 2024",
  IONIQ5: "Hyundai Ioniq 5 (with HDA II) 2022-24",
}
IONIQ_SETTINGS = {
  "HyundaiCameraSCC": 2,
  "EnableRadarTracks": 3,
  "CanfdHDA2": 1,
  "CustomSR": 140,
  "PorterAutoSetMain": False,
  "PorterAutoCancelBelow30": False,
  "PorterGapUse": False,
}
RANGES = {"HyundaiCameraSCC": (0, 2), "EnableRadarTracks": (-2, 3), "CanfdHDA2": (0, 1), "CustomSR": (0, 300)}
BOOL_KEYS = frozenset(IONIQ_SETTINGS) - RANGES.keys()
RECALIBRATION_KEYS = ("CalibrationParams", "LiveTorqueParameters", "LiveParameters", "LiveParametersV2", "LiveDelay")
# These consumers must initialize after the seven settings are applied. Card
# identifies the car while Panda retains startup safety; no live profile switch.
WAIT_PROCESSES = frozenset({
  "modeld", "signalcolord", "dmonitoringmodeld", "dmonitoringd", "locationd", "calibrationd", "torqued",
  "controlsd", "selfdrived", "plannerd", "paramsd", "lagd", "radard", "radarcan", "joystickd",
  "maneuversd", "lateral_maneuversd", "xiaoge_data", "beep", "carrot_cluster",
})


def select_vehicle(candidate, fingerprint, exact_match):
  from opendbc.car.hyundai.porter_fingerprint import matches_porter
  if candidate == IONIQ5 and exact_match:
    hda2 = any(frames.get(0x50) == 16 or frames.get(0x110) == 32
               for bus, frames in fingerprint.items() if bus < 128)
    return IONIQ5 if hda2 else None
  if candidate is None and matches_porter(fingerprint):
    return PORTER
  return None


def _settings(values):
  if not isinstance(values, dict) or set(values) != set(IONIQ_SETTINGS):
    raise ValueError("Incomplete Porter settings baseline")
  result = {}
  for key, value in values.items():
    if not isinstance(value, (int, bool)):
      raise ValueError(f"Invalid vehicle setting type: {key}")
    low, high = RANGES.get(key, (0, 1))
    if not low <= value <= high:
      raise ValueError(f"Invalid vehicle setting range: {key}")
    result[key] = bool(value) if key in BOOL_KEYS else int(value)
  return result


def _snapshot(params):
  values = {}
  for key in IONIQ_SETTINGS:
    value = params.get(key)
    values[key] = params.get_default_value(key) if value is None else value
  return _settings(values)


def _write(params, values):
  for key, value in values.items():
    if params.get(key) != value:
      if value is None:
        params.remove(key)
      else:
        params.put(key, value)
    if params.get(key) != value:
      raise OSError(f"Vehicle setting did not persist: {key}")


def _store(params, data):
  _write(params, {"AutoVehiclePorterBase": data})


def _load(params):
  data = params.get("AutoVehiclePorterBase")
  if data is None or data == {}:
    data = {"version": 1, "porter": None, "active": None}
  if (not isinstance(data, dict) or not {"version", "porter", "active"} <= data.keys() or
      data["version"] != 1 or data["active"] not in (None, PORTER, IONIQ5)):
    raise ValueError("Invalid automatic vehicle settings store")
  if data.get("porter") is not None:
    _settings(data["porter"])
  return copy.deepcopy(data)


def _rollback(params, pending):
  if "recalibration" in pending:
    values = pending["recalibration"]
    if not isinstance(values, dict) or set(values) != set(RECALIBRATION_KEYS):
      raise ValueError("Invalid recalibration recovery record")
    restored = {}
    for key, value in values.items():
      restored[key] = (base64.b64decode(value, validate=True)
                       if value is not None and key != "LiveParameters" else value)
    _write(params, restored)
  _write(params, _settings(pending["values"]))
  _write(params, {"CarName": pending["car_name"], "CarSelected3": pending["car_selected"]})


def prepare_settings(params):
  """Recover an interrupted startup and preserve the owner's Porter baseline."""
  data = _load(params)
  if "pending" in data:
    _rollback(params, data["pending"])
    del data["pending"]
    _store(params, data)
  # Never capture Ioniq overrides as the Porter baseline. On first opt-in,
  # seed only when both previous detection and manual selection say Porter.
  previous_porter = (params.get("CarName") == PORTER and
                     params.get("CarSelected3") == CAR_LABELS[PORTER])
  if previous_porter and data["active"] in (None, PORTER):
    data["porter"] = _snapshot(params)
    _store(params, data)
  return data


def apply_vehicle_settings(params, candidate, data):
  if params.get_bool("AutoVehicleReady"):
    raise RuntimeError("Automatic vehicle settings cannot change during a ready session")
  if candidate not in CAR_LABELS:
    raise ValueError("Unsupported automatic vehicle")
  if data["porter"] is None:
    if candidate != PORTER:
      raise ValueError("Save the Porter baseline on the Porter before using automatic Ioniq settings")
    data["porter"] = _snapshot(params)
  target = _settings(IONIQ_SETTINGS if candidate == IONIQ5 else data["porter"])
  previous = params.get("CarName")
  if previous not in CAR_LABELS:
    previous = data["active"]
  changed = previous in CAR_LABELS and previous != candidate
  data["pending"] = {"values": _snapshot(params), "car_name": params.get("CarName"),
                     "car_selected": params.get("CarSelected3")}
  if changed:
    # Match the device's Reset Calibration action. Keep a temporary journal for
    # rollback only; returning to a car starts fresh rather than restoring its
    # old camera pose or learned steering values.
    values = {key: params.get(key) for key in RECALIBRATION_KEYS}
    data["pending"]["recalibration"] = {
      key: base64.b64encode(value).decode() if isinstance(value, bytes) else value for key, value in values.items()
    }
  _store(params, data)
  try:
    _write(params, target)
    if changed:
      _write(params, dict.fromkeys(RECALIBRATION_KEYS))
    # Keep the UI's selected vehicle consistent with the actual interface.
    _write(params, {"CarSelected3": CAR_LABELS[candidate], "CarName": candidate})
    del data["pending"]
    data["active"] = candidate
    _store(params, data)
  except Exception:
    recovery = _load(params)
    if "pending" in recovery:
      _rollback(params, recovery["pending"])
      del recovery["pending"]
      _store(params, recovery)
    raise
  return changed
