"""Exercise both MPC build graphs without requiring a native compiler."""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from openpilot.common.basedir import BASEDIR


@pytest.mark.parametrize("kind, model", [("lateral", "lat"), ("longitudinal", "long")])
@pytest.mark.parametrize("arch", ["x86_64", "aarch64", "larch64", "Darwin"])
def test_mpc_uses_packaged_acados_and_deploys_runtime_libraries(tmp_path: Path, kind: str, model: str, arch: str) -> None:
  pytest.importorskip("SCons")
  mpc = tmp_path / "mpc"
  mpc.mkdir()
  shutil.copyfile(Path(BASEDIR) / f"openpilot/selfdrive/controls/lib/{kind}_mpc_lib/SConscript", mpc / "SConscript")
  (mpc / f"{model}_mpc.py").touch()
  constants = tmp_path / "openpilot/selfdrive/modeld/constants.py"
  constants.parent.mkdir(parents=True)
  constants.touch()

  # Deliberately keep the package outside the checkout and include a space.
  package = tmp_path.parent / f"{tmp_path.name} package"
  libraries = ["libacados.so", "libblasfeo.so", "libhpipm.so", "libqpOASES_e.so.3.1"]
  package_files = [
    "include/acados_c/ocp_nlp_interface.h",
    "templates/c_templates_tera/acados_solver.in.c",
    "templates/acados_ocp_solver_pyx.pyx",
    "templates/acados_solver_common.pxd",
    *[f"lib/{lib}" for lib in libraries],
  ]
  for relative in package_files:
    path = package / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(relative, encoding="utf-8")

  (tmp_path / "SConstruct").write_text(f'''
from types import SimpleNamespace
env = Environment(platform="posix", tools=["gcc", "g++", "gnulink"], LIBPATH=[], RPATH=[], CFLAGS=[], CXXFLAGS=[], CCFLAGS=[], LINKFLAGS=[])
envCython = env.Clone()
arch = {arch!r}
acados = SimpleNamespace(INCLUDE_DIR={str(package / 'include')!r}, TEMPLATE_DIR={str(package / 'templates')!r}, LIB_DIR={str(package / 'lib')!r})
msgq_python = env.Value("msgq")
common_python = env.Value("common")
np_version = env.Value("numpy")
Export("env", "envCython", "arch", "acados", "msgq_python", "common_python", "np_version")
SConscript("mpc/SConscript")
''', encoding="utf-8")

  def scons(*args: str) -> str:
    result = subprocess.run([sys.executable, "-m", "SCons", "-Q", *args], cwd=tmp_path,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout.replace("\\", "/")

  # An explicit extension target must pull in the solver, generator and runtime
  # copies; relying on SCons' default target traversal breaks partial builds.
  commands = scons("-n", "mpc/c_generated_code/acados_ocp_solver_pyx.so")
  assert f"{model}_mpc.py" in commands
  assert package.name in commands
  assert "third_party/acados" not in commands
  if arch == "Darwin":
    assert f"@loader_path/libacados_ocp_solver_{model}.dylib" in commands
    assert "-Wl,-rpath,@loader_path/" in commands
  else:
    assert "$$ORIGIN" in commands or "$ORIGIN" in commands
    assert "--disable-new-dtags" in commands
    for lib in libraries:
      assert lib in commands
    # Execute the real SCons copy actions and verify their bytes.
    scons(*[f"mpc/c_generated_code/{lib}" for lib in libraries])
    for lib in libraries:
      assert (mpc / "c_generated_code" / lib).read_bytes() == (package / "lib" / lib).read_bytes()
