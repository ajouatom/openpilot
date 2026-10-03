"""Packed-NV12 GPU renderer used by hardware screen recording.

The shader source and the plane placement math are privately ported from the
cluster renderer (openpilot/selfdrive/carrot/cluster/cluster_renderer.py,
NV12_PACK_*_SHADER and _render_nv12_pack_plane). Keeping a copy here means the
cluster feature and its tuned code paths stay untouched and are not a runtime
dependency of screen recording.
"""
from __future__ import annotations

import pyray as rl


NV12_PACK_VERTEX_SHADER = """
attribute vec3 vertexPosition;
attribute vec2 vertexTexCoord;
attribute vec4 vertexColor;

varying vec2 fragTexCoord;
varying vec4 fragColor;

uniform mat4 mvp;

void main() {
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""

NV12_PACK_FRAGMENT_SHADER = """
#ifdef GL_ES
precision mediump float;
#endif

varying vec2 fragTexCoord;
varying vec4 fragColor;

uniform sampler2D texture0;
uniform vec2 srcSize;
uniform vec2 packedSize;
uniform int plane;
uniform int flipX;

const float Y_PAD = 0.062745;
const float UV_PAD = 0.501961;

vec3 sampleRgb(float x, float y) {
    if (flipX != 0) {
        // The portrait upload transform maps screen horizontal correction to source Y.
        y = srcSize.y - 1.0 - y;
    }
    vec2 clamped = clamp(vec2(x, y), vec2(0.0), srcSize - vec2(1.0));
    return texture2D(texture0, (clamped + vec2(0.5)) / srcSize).rgb;
}

float y601(vec3 rgb) {
    return clamp(0.062745 + 0.256788 * rgb.r + 0.504129 * rgb.g + 0.097906 * rgb.b, 0.0, 1.0);
}

float u601(vec3 rgb) {
    return clamp(0.501961 - 0.148223 * rgb.r - 0.290993 * rgb.g + 0.439216 * rgb.b, 0.0, 1.0);
}

float v601(vec3 rgb) {
    return clamp(0.501961 + 0.439216 * rgb.r - 0.367788 * rgb.g - 0.071427 * rgb.b, 0.0, 1.0);
}

vec3 sample2x2(float x, float y) {
    return (
        sampleRgb(x, y) +
        sampleRgb(x + 1.0, y) +
        sampleRgb(x, y + 1.0) +
        sampleRgb(x + 1.0, y + 1.0)
    ) * 0.25;
}

float packedY(float x, float y) {
    if (x >= srcSize.x || y >= srcSize.y) {
        return Y_PAD;
    }
    return y601(sampleRgb(x, y));
}

vec2 packedUV(float x, float y) {
    if (x >= srcSize.x || y >= srcSize.y) {
        return vec2(UV_PAD, UV_PAD);
    }
    vec3 rgb = sample2x2(x, y);
    return vec2(u601(rgb), v601(rgb));
}

void main() {
    vec2 packedCoord = min(floor(fragTexCoord * packedSize), packedSize - vec2(1.0));
    float baseX = packedCoord.x * 4.0;
    if (plane == 0) {
        float y = packedCoord.y;
        gl_FragColor = vec4(
            packedY(baseX, y),
            packedY(baseX + 1.0, y),
            packedY(baseX + 2.0, y),
            packedY(baseX + 3.0, y)
        );
    } else {
        float y = packedCoord.y * 2.0;
        vec2 left = packedUV(baseX, y);
        vec2 right = packedUV(baseX + 2.0, y);
        gl_FragColor = vec4(left.x, left.y, right.x, right.y);
    }
}
"""


class Nv12Packer:
  """Renders an RGBA texture into a Venus-style packed NV12 target."""

  def __init__(self):
    self._shader = rl.load_shader_from_memory(NV12_PACK_VERTEX_SHADER, NV12_PACK_FRAGMENT_SHADER)
    if not rl.is_shader_valid(self._shader):
      raise RuntimeError("failed to load NV12 pack shader")
    self._locations = {
      "srcSize": rl.get_shader_location(self._shader, "srcSize"),
      "packedSize": rl.get_shader_location(self._shader, "packedSize"),
      "plane": rl.get_shader_location(self._shader, "plane"),
      "flipX": rl.get_shader_location(self._shader, "flipX"),
    }

  def render_plane(
    self,
    source_texture: rl.Texture,
    target: rl.RenderTexture,
    source_width: int,
    source_height: int,
    plane: int,
    packed_width: int,
    packed_height: int,
    dest_y: int = 0,
    clear_target: bool = False,
    clear_color: tuple[int, int, int, int] = (0, 0, 0, 0),
  ) -> None:
    shader = self._shader
    locations = self._locations
    src_size = rl.ffi.new("float[]", [float(source_width), float(source_height)])
    packed_size = rl.ffi.new("float[]", [float(packed_width), float(packed_height)])
    plane_value = rl.ffi.new("int[]", [int(plane)])
    flip_x_value = rl.ffi.new("int[]", [0])
    rl.set_shader_value(shader, locations["srcSize"], src_size, rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2)
    rl.set_shader_value(shader, locations["packedSize"], packed_size, rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2)
    rl.set_shader_value(shader, locations["plane"], plane_value, rl.ShaderUniformDataType.SHADER_UNIFORM_INT)
    rl.set_shader_value(shader, locations["flipX"], flip_x_value, rl.ShaderUniformDataType.SHADER_UNIFORM_INT)

    rl.begin_texture_mode(target)
    if clear_target:
      rl.clear_background(rl.Color(clear_color[0], clear_color[1], clear_color[2], clear_color[3]))
    rl.begin_shader_mode(shader)
    rl.rl_set_blend_factors(rl.RL_ONE, rl.RL_ZERO, rl.RL_FUNC_ADD)
    rl.begin_blend_mode(rl.BlendMode.BLEND_CUSTOM)
    try:
      rl.draw_texture_pro(
        source_texture,
        rl.Rectangle(0.0, 0.0, float(source_width), float(source_height)),
        rl.Rectangle(0.0, float(dest_y), float(packed_width), float(packed_height)),
        rl.Vector2(0.0, 0.0),
        0.0,
        rl.WHITE,
      )
    finally:
      rl.end_blend_mode()
      rl.end_shader_mode()
      rl.end_texture_mode()

  def close(self) -> None:
    if self._shader is not None:
      rl.unload_shader(self._shader)
      self._shader = None
