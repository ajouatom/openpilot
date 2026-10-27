"""Model pin shared by the comma client and Jetson release tooling."""
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = Path('openpilot/selfdrive/modeld/jetlink')


def read_spec(root=None):
  if root is None and Path('/TICI').is_file():
    from openpilot.common.big_model_catalog import effective_id, model_spec
    return model_spec(effective_id())
  root = ROOT if root is None else root
  directory = Path(root) / MODEL_DIR
  pin = directory / 'jetson_model.json'
  # Older signed bundles predate the separate Jetson pin.
  return json.loads((pin if pin.is_file() else directory / 'cinque_v2.json').read_text())


def is_jetson(peer):
  return isinstance(peer, dict) and (peer.get('carrot_host') == 'jetson' or
                                   (peer.get('backend') == 'trt' and str(peer.get('device', '')).startswith('Orin')))


def matching_release(path, required=False):
  """Never advertise a different model's signed release for this client.

  This checks model compatibility only; the Jetson still verifies the signature
  and download trust policy. Nothing here grants trust to an unsigned manifest.
  """
  try:
    manifest = json.loads(Path(path).read_text())
    model, spec = manifest['model'], read_spec()
    if model['sha256'] == spec['sha256'] and model['size'] == spec['nbytes']:
      return manifest
  except (OSError, ValueError, KeyError, TypeError):
    pass
  if required:
    raise ValueError('Selected Jetson model requires a matching signed host_release.json; release deployment is not configured')
  return None


def selected_release(path, required=False):
  if Path('/TICI').is_file() and not any(Path(path).with_name(name).exists() for name in
      ('usb_host_sha256_release.json', 'local_host_release.json')):
    from openpilot.common.big_model_catalog import DIRECTORY, selected
    path = DIRECTORY / selected()['jetson_release']
  usb = Path(path).with_name('usb_host_sha256_release.json')
  if usb.is_file():
    from tools.jetlink.usb_release import validate, identity  # noqa: TID251 -- shared standalone updater
    value, spec = validate(json.loads(usb.read_text())), read_spec()
    if identity(value['model']) != {'sha256': spec['sha256'], 'size': spec['nbytes']}:
      raise ValueError('USB Jetson selection does not match this branch model')
    if Path(path).with_name('local_host_release.json').exists():
      raise ValueError('Choose one local or USB Jetson delivery method')
    return value
  local = Path(path).with_name('local_host_release.json')
  if not local.is_file():
    return matching_release(path, required)
  value, spec = json.loads(local.read_text()), read_spec()
  if (not isinstance(value, dict) or set(value) != {'format', 'delivery', 'profile', 'model'} or
      value['format'] != 1 or value['delivery'] != 'local' or
      re.fullmatch('[a-z][a-z0-9-]{0,63}', str(value['profile'])) is None or
      value['model'] != {'sha256': spec['sha256'], 'size': spec['nbytes']}):
    raise ValueError('Local Jetson selection does not match this branch model')
  return value
