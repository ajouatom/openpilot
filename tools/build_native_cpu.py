"""Build the optional CPU kernels in place (SCons builds them on the device).

Run from the repository/bundle root with Cython and setuptools installed:
  python tools/build_native_cpu.py build_ext --inplace
"""
from pathlib import Path
import sys

from Cython.Build import cythonize
from setuptools import Extension, setup

ROOT = Path(__file__).resolve().parents[1]
flags = ['/O2', '/fp:strict'] if sys.platform == 'win32' else ['-O2', '-fno-fast-math', '-ffp-contract=off']
modules = [
  ('openpilot.system.ui.lib._draw_native', 'openpilot/system/ui/lib/_draw_native.pyx'),
  ('openpilot.selfdrive.carrot.radar_motion._motion_native', 'openpilot/selfdrive/carrot/radar_motion/_motion_native.pyx'),
  ('opendbc.can._can_native', 'opendbc_repo/opendbc/can/_can_native.pyx'),
]
extensions = [Extension(name, [path], language='c++', extra_compile_args=flags) for name, path in modules if (ROOT / path).exists()]
setup(name='carrot-native-cpu', packages=[], package_dir={'opendbc': 'opendbc_repo/opendbc'},
      ext_modules=cythonize(extensions, build_dir=str(ROOT / 'build/native_cpu'), compiler_directives={'language_level': 3}))
