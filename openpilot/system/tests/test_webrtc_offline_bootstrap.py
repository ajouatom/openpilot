import hashlib
import tomllib
from pathlib import Path

from openpilot.common.basedir import BASEDIR


WEBRTC_WHEELS = {
  "aioice": "0.10.2",
  "aiortc": "1.14.0",
  "av": "16.1.0",
  "cffi": "2.0.0",
  "cryptography": "46.0.5",
  "dnspython": "2.8.0",
  "google_crc32c": "1.8.0",
  "ifaddr": "0.2.0",
  "pycparser": "3.0",
  "pyee": "13.0.1",
  "pylibsrtp": "1.0.0",
  "pyopenssl": "26.0.0",
  "typing_extensions": "4.15.0",
}


def _locked_wheel_hashes() -> dict[str, str]:
  lock = tomllib.loads((Path(BASEDIR) / "uv.lock").read_text(encoding="utf-8"))
  return {
    wheel["url"].rsplit("/", 1)[-1]: wheel["hash"]
    for package in lock["package"]
    for wheel in package.get("wheels", [])
  }


def test_carrot_vision_runtime_is_available_offline() -> None:
  wheel_dir = Path(BASEDIR) / "third_party/wheels"
  locked_wheels = _locked_wheel_hashes()

  for package, version in WEBRTC_WHEELS.items():
    wheels = list(wheel_dir.glob(f"{package}-{version}-*.whl"))
    assert len(wheels) == 1, package
    wheel = wheels[0]
    assert locked_wheels[wheel.name] == f"sha256:{hashlib.sha256(wheel.read_bytes()).hexdigest()}"


def test_launcher_repairs_missing_aiortc_from_local_wheels() -> None:
  launcher = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")

  assert '"aiortc==1.14.0"' in launcher
  assert ('import aiortc; import av; import pylibsrtp' in launcher or
          '"aiortc; import av; import pylibsrtp"' in launcher)
  assert "--no-index" in launcher
  launch = launcher[launcher.index("function launch {"):]
  bootstrap = "ensure_webrtc_runtime" if "ensure_webrtc_runtime" in launch else "bootstrap_runtime_dependencies"
  assert launch.index(bootstrap) < launch.index("start_carrot_web")
