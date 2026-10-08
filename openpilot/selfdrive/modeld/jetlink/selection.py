"""Remember an offroad-approved App model; never adopt a new pick onroad."""
import json
import os
from pathlib import Path

from openpilot.selfdrive.modeld.jetlink.contracts import validate_model_spec
from jetlink.spec import ModelSpec


class ModelChanged(RuntimeError):
  pass


def approved_spec(cache):
  path = Path(cache) / 'selected.json'
  try:
    value = json.loads(path.read_text())
  except FileNotFoundError:
    return None
  return validate_model_spec(ModelSpec.from_dict(value))


def remember_spec(cache, spec):
  validate_model_spec(spec)
  cache = Path(cache)
  cache.mkdir(parents=True, exist_ok=True)
  temporary = cache / 'selected.tmp'
  with temporary.open('w') as stream:
    json.dump(spec.to_dict(), stream)
    stream.flush()
    os.fsync(stream.fileno())
  os.replace(temporary, cache / 'selected.json')


def check_loaded(client, telemetry):
  if isinstance(telemetry, dict) and 'loaded' in telemetry and telemetry['loaded'] != client.spec.sha256:
    raise ModelChanged('Jetlink App model changed or unloaded; reconnect and prepare offroad')
