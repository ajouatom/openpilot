import hashlib
import tomllib
import zipfile
from pathlib import Path

from openpilot.common.basedir import BASEDIR


def _wheel_metadata(wheel: Path) -> str:
  with zipfile.ZipFile(wheel) as archive:
    metadata = next(name for name in archive.namelist() if name.endswith(".dist-info/METADATA"))
    return archive.read(metadata).decode("utf-8")


def _wheel_contains(wheel: Path, suffix: str) -> bool:
  with zipfile.ZipFile(wheel) as archive:
    return any(name.endswith(suffix) for name in archive.namelist())


def _locked_wheel_hashes() -> dict[str, str]:
  lock = tomllib.loads((Path(BASEDIR) / "uv.lock").read_text(encoding="utf-8"))
  return {
    wheel["url"].rsplit("/", 1)[-1]: wheel["hash"]
    for package in lock["package"]
    for wheel in package.get("wheels", [])
  }


def _assert_locked_wheels(expected: dict[str, str]) -> None:
  wheel_dir = Path(BASEDIR) / "third_party/wheels"
  locked_wheels = _locked_wheel_hashes()
  for package, version in expected.items():
    wheels = list(wheel_dir.glob(f"{package}-{version}-*.whl"))
    assert len(wheels) == 1
    wheel = wheels[0]
    assert locked_wheels[wheel.name] == f"sha256:{hashlib.sha256(wheel.read_bytes()).hexdigest()}"


def test_required_agnos_19_python_dependency_is_available_offline() -> None:
  wheel_dir = Path(BASEDIR) / "third_party/wheels"
  pyserial = wheel_dir / "pyserial-3.5-py2.py3-none-any.whl"

  assert pyserial.is_file()
  assert "Name: pyserial" in _wheel_metadata(pyserial)


def test_optional_shapely_dependency_is_available_for_agnos_19() -> None:
  wheel_dir = Path(BASEDIR) / "third_party/wheels"
  wheels = list(wheel_dir.glob("shapely-*-cp312-*aarch64.whl"))

  assert len(wheels) == 1
  assert "Name: shapely" in _wheel_metadata(wheels[0])


def test_legacy_native_build_dependencies_are_available_for_agnos_19() -> None:
  wheel_dir = Path(BASEDIR) / "third_party/wheels"
  expected = {
    "eigen-3.4.0-py3-none-linux_aarch64.whl": "Name: eigen",
    "libjpeg-3.1.0-py3-none-linux_aarch64.whl": "Name: libjpeg",
  }

  for filename, package_name in expected.items():
    wheel = wheel_dir / filename
    assert wheel.is_file()
    assert package_name in _wheel_metadata(wheel)

  assert _wheel_contains(wheel_dir / "eigen-3.4.0-py3-none-linux_aarch64.whl", "eigen/install/eigen3/Eigen/Dense")
  assert _wheel_contains(wheel_dir / "libjpeg-3.1.0-py3-none-linux_aarch64.whl", "libjpeg/install/include/jpeglib.h")
  assert _wheel_contains(wheel_dir / "libjpeg-3.1.0-py3-none-linux_aarch64.whl", "libjpeg/install/lib/libjpeg.a")


def test_carrot_aiohttp_runtime_is_available_offline() -> None:
  wheel_dir = Path(BASEDIR) / "third_party/wheels"
  expected = {
    "aiohttp": "3.13.3",
    "aiohappyeyeballs": "2.6.1",
    "aiosignal": "1.4.0",
    "attrs": "26.1.0",
    "frozenlist": "1.8.0",
    "multidict": "6.7.1",
    "propcache": "0.4.1",
    "yarl": "1.23.0",
    "idna": "3.11",
    "typing_extensions": "4.15.0",
  }

  _assert_locked_wheels(expected)

  assert _wheel_contains(next(wheel_dir.glob("aiohttp-3.13.3-*.whl")), "aiohttp/_http_parser.cpython-312-aarch64-linux-gnu.so")


def test_remaining_legacy_runtime_dependencies_are_available_offline() -> None:
  _assert_locked_wheels({
    "brotli": "1.2.0",
    "crcmod_plus": "2.3.1",
    "json_rpc": "1.15.0",
    "psutil": "7.2.2",
    "pyusb": "1.3.1",
    "qrcode": "8.2",
  })


def test_launcher_bootstraps_without_network_package_installs() -> None:
  launcher = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")

  assert "ensure_python_package serial pyserial 1" in launcher
  assert "ensure_python_package msgpack msgpack 1" in launcher
  assert "ensure_python_package aiohttp aiohttp 1 1" in launcher
  assert "ensure_python_package psutil psutil 1" in launcher
  assert "ensure_python_package crcmod crcmod-plus 1" in launcher
  assert "ensure_python_package jsonrpc json-rpc 1" in launcher
  assert "ensure_python_package qrcode qrcode 1" in launcher
  assert "ensure_python_package brotli brotli 0" in launcher
  assert "ensure_python_package usb pyusb 0" in launcher
  assert "ensure_python_package eigen eigen 1" in launcher
  assert "ensure_python_package libjpeg libjpeg 1" in launcher
  assert "[ -f /TICI ] || [ -f /AGNOS ]" in launcher
  assert "ensure_python_package shapely shapely 0" in launcher
  assert "pip install shapely" not in launcher
  assert "--no-index --no-deps" in launcher


def test_runtime_dependencies_precede_external_carrot_web() -> None:
  launcher = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")
  launch = launcher[launcher.index("function launch {"):]

  assert launch.index("bootstrap_runtime_dependencies") < launch.index("start_carrot_web")


def test_larch64_sconstruct_omits_dependencies_removed_from_agnos_19() -> None:
  sconstruct = (Path(BASEDIR) / "SConstruct").read_text(encoding="utf-8")

  assert 'if arch == "larch64":' in sconstruct
  assert "name not in ('bzip2', 'libyuv')" in sconstruct


def test_sconstruct_preserves_bootstrap_pythonpath_for_generated_code() -> None:
  sconstruct = (Path(BASEDIR) / "SConstruct").read_text(encoding="utf-8")

  assert 'os.environ.get("PYTHONPATH")' in sconstruct
  assert 'external_pythonpath.split(os.pathsep)' in sconstruct
  assert '"PYTHONPATH": os.pathsep.join(scons_python_paths)' in sconstruct


def test_shared_ffmpeg_runtime_path_matches_current_comma() -> None:
  sconstruct = (Path(BASEDIR) / "SConstruct").read_text(encoding="utf-8")

  assert "RPATH=[ffmpeg.LIB_DIR] if ffmpeg_shared else []" in sconstruct


def test_logger_xattrs_use_python_standard_library() -> None:
  source = (Path(BASEDIR) / "openpilot/system/loggerd/xattr_cache.py").read_text(encoding="utf-8")

  assert "import xattr" not in source
  assert "os.getxattr(path, attr_name)" in source
  assert "os.setxattr(path, attr_name, attr_value)" in source


def test_known_egpu_prefetch_does_not_block_startup_or_force_build_without_live_device() -> None:
  launcher = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")
  prepare = launcher[launcher.index("function prepare_big_model_if_needed {"):]
  prepare = prepare[:prepare.index("\n}")]
  update = launcher[launcher.index("function start_big_model_update {"):]
  update = update[:update.index("\n}")]

  egpu_check = prepare.index("usbgpu_present")
  active_sha = prepare.index("--active-sha")
  assert "--ensure-if-egpu" not in prepare
  assert egpu_check < active_sha
  assert "return" in prepare[egpu_check:active_sha]
  assert "check_usbgpu_power()" not in prepare
  assert "one-shot 12V check" in prepare
  assert "--ensure-if-egpu" in update
  assert "/tmp/big_model_update.log" in update
  assert ") >> \"$log_path\" 2>&1 &" in update

  launch = launcher[launcher.index("function launch {"):]
  build = launch.index("./build.py")
  update_start = launch.index("start_big_model_update", build)
  manager = launch.index("./manager.py", update_start)
  assert build < update_start < manager


def test_usbgpu_history_is_persistent_and_recorded() -> None:
  params = (Path(BASEDIR) / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  modeld = (Path(BASEDIR) / "openpilot/selfdrive/modeld/modeld.py").read_text(encoding="utf-8")

  assert '{"UsbGpuHardwareSeen", {PERSISTENT, BOOL}}' in params
  assert 'if _present or _compiled:' in modeld
  assert 'params.put_bool("UsbGpuHardwareSeen", True)' in modeld
  assert 'params.put_bool_nonblocking("UsbGpuHardwareSeen", True)' in modeld


def test_model_revalidation_preserves_compiled_artifacts() -> None:
  launcher = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")
  invalidate = launcher[launcher.index("function invalidate_modeld_build_if_needed {"):]
  invalidate = invalidate[:invalidate.index("\n}")]

  assert "rm -f" not in invalidate
  assert "revalidating with SCons" in invalidate
  assert "FORCE_REBUILD=1" in invalidate


def test_optional_egpu_scons_target_is_not_gated_by_second_usb_scan() -> None:
  source = (Path(BASEDIR) / "openpilot/selfdrive/modeld/SConscript").read_text(encoding="utf-8")

  assert "USBGPU = os.getenv('BUILD_USB_GPU_MODEL') == '1'" in source
  assert "and usbgpu_present()" not in source


def test_model_compiler_uses_isolated_agnos_core() -> None:
  source = (Path(BASEDIR) / "openpilot/selfdrive/modeld/SConscript").read_text(encoding="utf-8")

  assert "taskset = 'taskset -c 7 ' if arch == 'larch64' else ''" in source
  assert "{taskset}python3" in source


def test_optional_egpu_build_retries_transient_pcie_link_startup() -> None:
  source = (Path(BASEDIR) / "openpilot/system/manager/build.py").read_text(encoding="utf-8")

  assert "check_usbgpu(timeout=USBGPU_READINESS_TIMEOUT, require_clean_link=False)" in source
  assert "USB eGPU not ready for optional model compilation" in source
  assert "USBGPU_BUILD_ATTEMPTS = 6" in source
  assert "USBGPU_READINESS_ATTEMPTS = 6" in source
  assert "USBGPU_READINESS_RETRY_INTERVAL = 3.0" in source
  assert "USBGPU_READINESS_TIMEOUT = 30.0" in source
  assert 'USBGPU_TRANSIENT_READINESS_ERRORS = {"12V / PCIe not ready", "USB link errors", "GPU check timed out", "GPU incompatible"}' in source
  assert 'write_big_model_status(model_cache_dir(), "checking"' in source
  assert "USB eGPU transient readiness error" in source
  assert "USBGPU_ENUMERATION_WAIT_SECONDS = 20.0" in source
  assert "waiting for eGPU USB re-enumeration" in source
  assert "USB eGPU returned after" in source
  assert "usbgpu_pcie_not_ready(error_text)" in source
  assert "USB eGPU PCIe link not ready; retrying" in source

  helpers = (Path(BASEDIR) / "openpilot/selfdrive/modeld/helpers.py").read_text(encoding="utf-8")
  assert '"read(0xb450"' in helpers


def test_optional_egpu_build_preserves_generated_pkl_artifacts() -> None:
  source = (Path(BASEDIR) / "openpilot/system/manager/build.py").read_text(encoding="utf-8")
  build_body = source[source.index("def build_usbgpu_model"):source.index("def build(", source.index("def build_usbgpu_model"))]
  assert ".unlink()" not in build_body
  assert "Never delete generated PKL artifacts here" in build_body


def test_tinygrad_waits_for_custom_egpu_pcie_link_training() -> None:
  source = (Path(BASEDIR) / "tinygrad_repo/tinygrad/runtime/support/usb.py").read_text(encoding="utf-8")

  assert "PCIE_LINK_READY = 0x78" in source
  assert "PCIE_LINK_TIMEOUT_S = 2.0" in source
  assert "self.set_pcie_power(False)" not in source
  assert "self.set_pcie_power(True)" in source
  assert "self.reset_usb_bridge()" not in source
  assert "while ltssm != self.PCIE_LINK_READY" in source
  assert "time.sleep(self.PCIE_LINK_POLL_INTERVAL_S)" in source


def test_tinygrad_bounds_usbgpu_copy_staging_transfers() -> None:
  source = (Path(BASEDIR) / "tinygrad_repo/tinygrad/runtime/ops_amd.py").read_text(encoding="utf-8")

  assert "USBGPU_COPY_BUFFER_SIZE = 256 * 1024" in source
  assert "size=USBGPU_COPY_BUFFER_SIZE" in source
