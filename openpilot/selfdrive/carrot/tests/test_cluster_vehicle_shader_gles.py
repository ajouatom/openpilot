"""Compile the P/R vehicle shaders in a real Linux EGL/GLES context."""
import ctypes as ct
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cluster"))
from cluster_renderer import rl, vehicle_shader_sources


@pytest.mark.skipif(sys.platform != "linux", reason="comma uses Linux GLES; Windows GL is not acceptance")
def test_vehicle_shader_compiles_and_links_on_gles(record_property):
  try:
    egl, gl = ct.CDLL("libEGL.so.1"), ct.CDLL("libGLESv2.so.2")
  except OSError as exc:
    pytest.skip(f"Linux EGL/GLES libraries unavailable: {exc}")

  def bind(lib, name, result, *args):
    function = getattr(lib, name)
    function.restype, function.argtypes = result, args
    return function

  ptr, integer, uint = ct.c_void_p, ct.c_int, ct.c_uint
  intp = ct.POINTER(integer)
  get_display = bind(egl, "eglGetPlatformDisplay", ptr, uint, ptr, ptr)
  initialize = bind(egl, "eglInitialize", uint, ptr, intp, intp)
  bind_api = bind(egl, "eglBindAPI", uint, uint)
  choose_config = bind(egl, "eglChooseConfig", uint, ptr, intp, ct.POINTER(ptr), integer, intp)
  create_surface = bind(egl, "eglCreatePbufferSurface", ptr, ptr, ptr, intp)
  create_context = bind(egl, "eglCreateContext", ptr, ptr, ptr, ptr, intp)
  make_current = bind(egl, "eglMakeCurrent", uint, ptr, ptr, ptr, ptr)
  destroy_context = bind(egl, "eglDestroyContext", uint, ptr, ptr)
  destroy_surface = bind(egl, "eglDestroySurface", uint, ptr, ptr)
  terminate = bind(egl, "eglTerminate", uint, ptr)
  get_string = bind(gl, "glGetString", ct.c_char_p, uint)
  create_shader = bind(gl, "glCreateShader", uint, uint)
  shader_source = bind(gl, "glShaderSource", None, uint, integer, ct.POINTER(ct.c_char_p), intp)
  compile_shader = bind(gl, "glCompileShader", None, uint)
  shader_status = bind(gl, "glGetShaderiv", None, uint, uint, intp)
  shader_log = bind(gl, "glGetShaderInfoLog", None, uint, integer, intp, ptr)
  create_program = bind(gl, "glCreateProgram", uint)
  attach_shader = bind(gl, "glAttachShader", None, uint, uint)
  link_program = bind(gl, "glLinkProgram", None, uint)
  program_status = bind(gl, "glGetProgramiv", None, uint, uint, intp)
  program_log = bind(gl, "glGetProgramInfoLog", None, uint, integer, intp, ptr)
  delete_shader = bind(gl, "glDeleteShader", None, uint)
  delete_program = bind(gl, "glDeleteProgram", None, uint)

  # EGL_MESA_platform_surfaceless: no display server or physical output needed.
  display = get_display(0x31DD, None, None)
  if not display or not initialize(display, None, None):
    pytest.skip("Linux driver does not provide surfaceless EGL")
  surface = context = program = None
  shaders = []
  try:
    assert bind_api(0x30A0)  # EGL_OPENGL_ES_API
    config, count = ptr(), integer()
    attrs = (integer * 9)(0x3033, 1, 0x3040, 4, 0x3024, 8, 0x3023, 8, 0x3038)
    assert choose_config(display, attrs, ct.byref(config), 1, ct.byref(count)) and count.value
    surface = create_surface(display, config, (integer * 5)(0x3057, 16, 0x3056, 16, 0x3038))
    context = create_context(display, config, None, (integer * 3)(0x3098, 2, 0x3038))
    assert surface and context and make_current(display, surface, surface, context)
    version = get_string(0x1F02).decode()
    assert "OpenGL ES" in version
    record_property("gles_version", version)
    sources = vehicle_shader_sources(rl.RL_OPENGL_ES_20)
    program = create_program()
    for kind, source in zip((0x8B31, 0x8B30), sources, strict=True):
      shader = create_shader(kind)
      shaders.append(shader)
      text = ct.c_char_p(source.encode())
      shader_source(shader, 1, ct.byref(text), None)
      compile_shader(shader)
      ok, log = integer(), ct.create_string_buffer(4096)
      shader_status(shader, 0x8B81, ct.byref(ok))
      shader_log(shader, len(log), None, log)
      assert ok.value, log.value.decode()
      attach_shader(program, shader)
    link_program(program)
    ok, log = integer(), ct.create_string_buffer(4096)
    program_status(program, 0x8B82, ct.byref(ok))
    program_log(program, len(log), None, log)
    assert ok.value, log.value.decode()
  finally:
    if program:
      delete_program(program)
    for shader in shaders:
      delete_shader(shader)
    make_current(display, None, None, None)
    if context:
      destroy_context(display, context)
    if surface:
      destroy_surface(display, surface)
    terminate(display)
