from __future__ import annotations

import math
import os
import time
from pathlib import Path

import pyray as rl

from cluster_config import (
    AMBER,
    BG,
    BLUE,
    BLUE_SOFT,
    DESIGN_HEIGHT,
    DESIGN_WIDTH,
    FAINT,
    GREEN,
    MAX_ACCEL_MPS2,
    MAX_SPEED_KPH,
    MUTED,
    PANEL_BG,
    RED,
    TEXT,
    WHITE,
)
from cluster_models import ClusterUiState, RouteOverlay
from cluster_scene import ClusterScene, MeshStrip, RadarPointMarker, Vec3, VehicleBox, build_cluster_scene
from cluster_utils import clamp


VEHICLE_MODEL_PATH = Path(__file__).resolve().parent / "assets" / "models" / "cybertruck" / "cybertruck_cluster.obj"
VEHICLE_MATERIAL_COLORS: dict[str, tuple[int, int, int, int]] = {
    "body": (156, 166, 172, 255),
    "wheel": (18, 20, 22, 255),
    "besi_roda": (36, 38, 42, 255),
    "light": (184, 222, 255, 255),
    "stop_light": (226, 34, 28, 255),
    "riting": (255, 146, 20, 255),
    "Material": (136, 142, 148, 255),
    "Material.002": (68, 72, 78, 255),
    "Material.003": (18, 20, 22, 255),
    "Material.004": (18, 20, 22, 255),
    "Material.005": (18, 20, 22, 255),
    "Material.006": (18, 20, 22, 255),
}
DEFAULT_VEHICLE_MATERIAL_COLOR = (142, 150, 156, 255)
DESIRE_LABELS = ("NONE", "TURN L", "TURN R", "LC L", "LC R", "KEEP L", "KEEP R", "OTHER")
FXAA_FRAGMENT_SHADER_330 = """
#version 330

in vec2 fragTexCoord;
in vec4 fragColor;

uniform sampler2D texture0;
uniform vec2 resolution;

out vec4 finalColor;

void main()
{
    vec2 inverseResolution = 1.0 / resolution;
    vec3 rgbNW = texture(texture0, fragTexCoord + vec2(-1.0, -1.0) * inverseResolution).rgb;
    vec3 rgbNE = texture(texture0, fragTexCoord + vec2(1.0, -1.0) * inverseResolution).rgb;
    vec3 rgbSW = texture(texture0, fragTexCoord + vec2(-1.0, 1.0) * inverseResolution).rgb;
    vec3 rgbSE = texture(texture0, fragTexCoord + vec2(1.0, 1.0) * inverseResolution).rgb;
    vec4 center = texture(texture0, fragTexCoord);
    vec3 rgbM = center.rgb;
    vec3 luma = vec3(0.299, 0.587, 0.114);

    float lumaNW = dot(rgbNW, luma);
    float lumaNE = dot(rgbNE, luma);
    float lumaSW = dot(rgbSW, luma);
    float lumaSE = dot(rgbSE, luma);
    float lumaM = dot(rgbM, luma);
    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

    vec2 direction;
    direction.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    direction.y = ((lumaNW + lumaSW) - (lumaNE + lumaSE));
    float directionReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * 0.03125, 0.0078125);
    float inverseDirectionAdjustment = 1.0 / (min(abs(direction.x), abs(direction.y)) + directionReduce);
    direction = clamp(direction * inverseDirectionAdjustment, vec2(-8.0), vec2(8.0)) * inverseResolution;

    vec3 rgbA = 0.5 * (
        texture(texture0, fragTexCoord + direction * (1.0 / 3.0 - 0.5)).rgb +
        texture(texture0, fragTexCoord + direction * (2.0 / 3.0 - 0.5)).rgb);
    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture(texture0, fragTexCoord + direction * -0.5).rgb +
        texture(texture0, fragTexCoord + direction * 0.5).rgb);
    float lumaB = dot(rgbB, luma);

    vec3 color = ((lumaB < lumaMin) || (lumaB > lumaMax)) ? rgbA : rgbB;
    finalColor = vec4(color, center.a) * fragColor;
}
"""
FXAA_FRAGMENT_SHADER_100 = """
#version 100
precision mediump float;

varying vec2 fragTexCoord;
varying vec4 fragColor;

uniform sampler2D texture0;
uniform vec2 resolution;

void main()
{
    vec2 inverseResolution = 1.0 / resolution;
    vec3 rgbNW = texture2D(texture0, fragTexCoord + vec2(-1.0, -1.0) * inverseResolution).rgb;
    vec3 rgbNE = texture2D(texture0, fragTexCoord + vec2(1.0, -1.0) * inverseResolution).rgb;
    vec3 rgbSW = texture2D(texture0, fragTexCoord + vec2(-1.0, 1.0) * inverseResolution).rgb;
    vec3 rgbSE = texture2D(texture0, fragTexCoord + vec2(1.0, 1.0) * inverseResolution).rgb;
    vec4 center = texture2D(texture0, fragTexCoord);
    vec3 rgbM = center.rgb;
    vec3 luma = vec3(0.299, 0.587, 0.114);

    float lumaNW = dot(rgbNW, luma);
    float lumaNE = dot(rgbNE, luma);
    float lumaSW = dot(rgbSW, luma);
    float lumaSE = dot(rgbSE, luma);
    float lumaM = dot(rgbM, luma);
    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

    vec2 direction;
    direction.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    direction.y = ((lumaNW + lumaSW) - (lumaNE + lumaSE));
    float directionReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * 0.03125, 0.0078125);
    float inverseDirectionAdjustment = 1.0 / (min(abs(direction.x), abs(direction.y)) + directionReduce);
    direction = clamp(direction * inverseDirectionAdjustment, vec2(-8.0), vec2(8.0)) * inverseResolution;

    vec3 rgbA = 0.5 * (
        texture2D(texture0, fragTexCoord + direction * (1.0 / 3.0 - 0.5)).rgb +
        texture2D(texture0, fragTexCoord + direction * (2.0 / 3.0 - 0.5)).rgb);
    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture2D(texture0, fragTexCoord + direction * -0.5).rgb +
        texture2D(texture0, fragTexCoord + direction * 0.5).rgb);
    float lumaB = dot(rgbB, luma);

    vec3 color = ((lumaB < lumaMin) || (lumaB > lumaMax)) ? rgbA : rgbB;
    gl_FragColor = vec4(color, center.a) * fragColor;
}
"""


def rl_color(color: tuple[int, int, int] | tuple[int, int, int, int], alpha: int | None = None) -> rl.Color:
    if len(color) == 4:
        r, g, b, a = color
    else:
        r, g, b = color
        a = 255
    if alpha is not None:
        a = alpha
    return rl.Color(int(r), int(g), int(b), int(a))


def dominant_desire(
    desire_state: tuple[float, ...],
    desire_prediction: tuple[tuple[float, ...], ...],
) -> tuple[str | None, float]:
    candidates = desire_state
    if not candidates and desire_prediction:
        candidates = desire_prediction[0]
    if not candidates:
        return None, 0.0
    best_index = 0
    best_value = 0.0
    for index, value in enumerate(candidates[: len(DESIRE_LABELS)]):
        if index == 0 and len(candidates) > 1:
            continue
        value = clamp(value, 0.0, 1.0)
        if value > best_value:
            best_index = index
            best_value = value
    if best_value < 0.03:
        return "NONE", clamp(candidates[0] if candidates else 0.0, 0.0, 1.0)
    return DESIRE_LABELS[min(best_index, len(DESIRE_LABELS) - 1)], best_value


def model_risk_values(state: ClusterUiState) -> tuple[float, ...]:
    if state.risk_points:
        values = []
        for point in state.risk_points[:5]:
            values.append(
                max(
                    point.brake_disengage,
                    point.gas_disengage,
                    point.steer_override,
                    point.hard_brake_3,
                    point.hard_brake_4,
                    point.hard_brake_5,
                )
            )
        return tuple(values)
    values = (
        state.brake_disengage_risk,
        state.gas_disengage_risk,
        state.steer_override_risk,
        state.hard_brake_risk,
        max(state.gas_press_prob, state.brake_press_prob),
    )
    return tuple(value for value in values if value > 0.0)


def radar_point_distance_label(point: RadarPointMarker) -> str:
    return f"{point.longitudinal_m:.0f} m"


def radar_point_speed_label(point: RadarPointMarker) -> str:
    if point.absolute_speed_kph is None:
        return "-- km/h"
    return f"{point.absolute_speed_kph:.0f} km/h"


def vec3(point: Vec3) -> rl.Vector3:
    return rl.Vector3(point.x, point.y, point.z)


def rectangles_overlap(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> bool:
    lx, ly, lw, lh = left
    rx, ry, rw, rh = right
    return lx < rx + rw and lx + lw > rx and ly < ry + rh and ly + lh > ry


class ClusterUiRenderer:
    def __init__(
        self,
        width: int = DESIGN_WIDTH,
        height: int = DESIGN_HEIGHT,
        title: str = "carrotpilot cluster",
        target_fps: int = 30,
    ) -> None:
        self.width = width
        self.height = height
        self.title = title
        self.target_fps = target_fps
        self.hidden = False
        self._window_open = False
        self._font = None
        self._owns_font = False
        self._capture_target = None
        self._rotated_capture_target = None
        self._aa_source_target = None
        self._fxaa_shader = None
        self._fxaa_resolution_loc = -1
        self._fxaa_resolution_value = None
        self._vehicle_model = None
        self._vehicle_model_load_attempted = False
        self._route_video_texture = None
        self._route_video_size: tuple[int, int] | None = None
        self._route_video_frame_id: str | None = None

    def open(self, hidden: bool = False) -> None:
        if self._window_open:
            return
        self.hidden = hidden
        rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
        flags = rl.ConfigFlags.FLAG_MSAA_4X_HINT
        if hidden:
            flags |= rl.ConfigFlags.FLAG_WINDOW_HIDDEN
        rl.set_config_flags(flags)
        rl.init_window(self.width, self.height, self.title)
        if self.target_fps > 0:
            rl.set_target_fps(self.target_fps)
        self._font = self._load_font()
        self._load_vehicle_model()
        # self._load_fxaa_shader()
        self._window_open = True

    def close(self) -> None:
        if not self._window_open:
            return
        if self._aa_source_target is not None:
            rl.unload_render_texture(self._aa_source_target)
            self._aa_source_target = None
        if self._capture_target is not None:
            rl.unload_render_texture(self._capture_target)
            self._capture_target = None
        if self._rotated_capture_target is not None:
            rl.unload_render_texture(self._rotated_capture_target)
            self._rotated_capture_target = None
        if self._fxaa_shader is not None:
            rl.unload_shader(self._fxaa_shader)
            self._fxaa_shader = None
        if self._route_video_texture is not None:
            rl.unload_texture(self._route_video_texture)
            self._route_video_texture = None
        if self._owns_font and self._font is not None:
            rl.unload_font(self._font)
        self._font = None
        self._owns_font = False
        self._fxaa_resolution_loc = -1
        self._fxaa_resolution_value = None
        if self._vehicle_model is not None:
            rl.unload_model(self._vehicle_model)
            self._vehicle_model = None
        self._vehicle_model_load_attempted = False
        self._route_video_size = None
        self._route_video_frame_id = None
        rl.close_window()
        self._window_open = False

    def should_close(self) -> bool:
        return bool(self._window_open and rl.window_should_close())

    def render_frame(self, state: ClusterUiState) -> None:
        self.open()
        if self._fxaa_shader is None:
            rl.begin_drawing()
            self.render(state)
            rl.end_drawing()
            return

        scene_target = self._get_aa_source_target()
        rl.begin_texture_mode(scene_target)
        self._render_world(state)
        rl.end_texture_mode()

        rl.begin_drawing()
        self._draw_antialiased_texture(scene_target.texture)
        self._draw_hud(state)
        rl.end_drawing()

    def render(self, state: ClusterUiState) -> None:
        """Draw one frame into the currently active raylib render target."""
        self._render_world(state)
        self._draw_hud(state)

    def _render_world(self, state: ClusterUiState) -> None:
        scene = build_cluster_scene(state)
        rl.clear_background(rl_color(BG))
        self._draw_scene(scene)

    def render_to_file(self, state: ClusterUiState, output_path: str | Path) -> None:
        image = self._render_to_image(state)
        try:
            rl.export_image(image, str(output_path))
        finally:
            rl.unload_image(image)

    def render_to_png_bytes(self, state: ClusterUiState, rotate_clockwise: bool = False) -> bytes:
        image = self._render_to_image(state, rotate_clockwise=rotate_clockwise)
        try:
            size = rl.ffi.new("int *")
            data = rl.export_image_to_memory(image, ".png", size)
            try:
                if size[0] <= 0:
                    raise RuntimeError("raylib failed to encode frame as PNG")
                return bytes(rl.ffi.buffer(data, size[0]))
            finally:
                rl.mem_free(data)
        finally:
            rl.unload_image(image)

    def render_to_rgba_bytes(
        self,
        state: ClusterUiState,
        rotate_clockwise: bool = False,
    ) -> tuple[bytes, int, int]:
        profile = os.environ.get("CLUSTER_PROFILE_RGBA") == "1"

        t0 = time.perf_counter()
        image = self._render_to_image(state, rotate_clockwise=rotate_clockwise)
        t1 = time.perf_counter()

        try:
            rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
            t2 = time.perf_counter()

            byte_count = image.width * image.height * 4
            rgba = bytes(rl.ffi.buffer(image.data, byte_count))
            t3 = time.perf_counter()

            if profile and False:
                print(
                    f"rgba_detail render_to_image={(t1 - t0) * 1000:.1f}ms "
                    f"format={(t2 - t1) * 1000:.1f}ms "
                    f"copy={(t3 - t2) * 1000:.1f}ms "
                    f"image={image.width}x{image.height}",
                    flush=True,
                )

            return rgba, image.width, image.height
        finally:
            rl.unload_image(image)

    def _render_to_image(self, state: ClusterUiState, rotate_clockwise: bool = False):
        profile = os.environ.get("CLUSTER_PROFILE_RGBA") == "1"

        self.open(hidden=self.hidden)
        target = self._get_capture_target()

        t0 = time.perf_counter()

        if self._fxaa_shader is None:
            rl.begin_texture_mode(target)
            self.render(state)
            rl.end_texture_mode()
        else:
            scene_target = self._get_aa_source_target()
            rl.begin_texture_mode(scene_target)
            self._render_world(state)
            rl.end_texture_mode()

            rl.begin_texture_mode(target)
            self._draw_antialiased_texture(scene_target.texture)
            self._draw_hud(state)
            rl.end_texture_mode()

        t1 = time.perf_counter()

        if rotate_clockwise:
            rotated_target = self._get_rotated_capture_target()

            rl.begin_texture_mode(rotated_target)
            rl.clear_background(rl_color(BG))

            source = rl.Rectangle(
                0.0,
                0.0,
                float(target.texture.width),
                -float(target.texture.height),
            )
            dest = rl.Rectangle(
                float(self.height),
                0.0,
                float(self.width),
                float(self.height),
            )
            origin = rl.Vector2(0.0, 0.0)

            rl.draw_texture_pro(
                target.texture,
                source,
                dest,
                origin,
                90.0,
                rl_color(WHITE),
            )
            rl.end_texture_mode()

            t_rotate_gpu = time.perf_counter()

            image = rl.load_image_from_texture(rotated_target.texture)
            t2 = time.perf_counter()

            rl.image_flip_vertical(image)
            t3 = time.perf_counter()

            t4 = t3
        else:
            image = rl.load_image_from_texture(target.texture)
            t2 = time.perf_counter()

            rl.image_flip_vertical(image)
            t3 = time.perf_counter()

            t_rotate_gpu = t3
            t4 = t3

        if profile and False:
            print(
                f"render_image draw={(t1 - t0) * 1000:.1f}ms "
                f"gpu_rotate={(t_rotate_gpu - t1) * 1000:.1f}ms "
                f"readback={(t2 - t_rotate_gpu) * 1000:.1f}ms "
                f"flip={(t3 - t2) * 1000:.1f}ms "
                f"cpu_rotate={(t4 - t3) * 1000:.1f}ms",
                flush=True,
            )

        return image

    def _get_capture_target(self):
        if self._capture_target is None:
            self._capture_target = rl.load_render_texture(self.width, self.height)
            rl.set_texture_filter(self._capture_target.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        return self._capture_target
      
    def _get_rotated_capture_target(self):
        if self._rotated_capture_target is None:
            self._rotated_capture_target = rl.load_render_texture(self.height, self.width)
            rl.set_texture_filter(self._rotated_capture_target.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        return self._rotated_capture_target
      
    def _get_aa_source_target(self):
        if self._aa_source_target is None:
            self._aa_source_target = rl.load_render_texture(self.width, self.height)
            rl.set_texture_filter(self._aa_source_target.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        return self._aa_source_target

    def _load_fxaa_shader(self) -> None:
        gl_version = rl.rl_get_version()
        es_versions = (
            getattr(rl, "RL_OPENGL_ES_20", 5),
            getattr(rl, "RL_OPENGL_ES_30", 6),
        )
        shader_codes = (
            (FXAA_FRAGMENT_SHADER_100, FXAA_FRAGMENT_SHADER_330)
            if gl_version in es_versions
            else (FXAA_FRAGMENT_SHADER_330, FXAA_FRAGMENT_SHADER_100)
        )
        for shader_code in shader_codes:
            try:
                shader = rl.load_shader_from_memory(rl.ffi.NULL, shader_code)
            except Exception as exc:
                print(f"FXAA shader load failed: {exc}")
                continue
            if not rl.is_shader_valid(shader):
                rl.unload_shader(shader)
                continue
            self._fxaa_shader = shader
            self._fxaa_resolution_loc = rl.get_shader_location(shader, "resolution")
            self._fxaa_resolution_value = rl.ffi.new("float[2]", [float(self.width), float(self.height)])
            return
        self._fxaa_shader = None

    def _draw_antialiased_texture(self, texture) -> None:
        if self._fxaa_shader is None:
            return
        source = rl.Rectangle(0.0, 0.0, float(texture.width), -float(texture.height))
        dest = rl.Rectangle(0.0, 0.0, float(self.width), float(self.height))
        origin = rl.Vector2(0.0, 0.0)
        rl.clear_background(rl_color(BG))
        rl.begin_shader_mode(self._fxaa_shader)
        if self._fxaa_resolution_loc >= 0 and self._fxaa_resolution_value is not None:
            rl.set_shader_value(
                self._fxaa_shader,
                self._fxaa_resolution_loc,
                self._fxaa_resolution_value,
                rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2,
            )
        rl.draw_texture_pro(texture, source, dest, origin, 0.0, rl_color(WHITE))
        rl.end_shader_mode()

    def _load_font(self):
        for candidate in self._font_candidates():
            if candidate.exists():
                font = rl.load_font_ex(str(candidate), 160, None, 0)
                if font.texture.id > 0:
                    rl.gen_texture_mipmaps(font.texture)
                    rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_TRILINEAR)
                    self._owns_font = True
                    return font
        self._owns_font = False
        return rl.get_font_default()

    def _font_candidates(self) -> list[Path]:
        windir = Path(os.environ.get("WINDIR", "C:/Windows"))
        return [
            windir / "Fonts" / "segoeuib.ttf",
            windir / "Fonts" / "arialbd.ttf",
            windir / "Fonts" / "arial.ttf",
        ]

    def _load_vehicle_model(self) -> None:
        if self._vehicle_model_load_attempted:
            return
        self._vehicle_model_load_attempted = True
        if not VEHICLE_MODEL_PATH.exists():
            return
        try:
            mesh = self._load_obj_mesh(VEHICLE_MODEL_PATH)
            rl.upload_mesh(rl.ffi.addressof(mesh), False)
            model = rl.load_model_from_mesh(mesh)
            if not rl.is_model_valid(model):
                rl.unload_model(model)
                return
            self._vehicle_model = model
        except Exception as exc:
            print(f"Cybertruck vehicle model load failed: {exc}")
            self._vehicle_model = None

    def _load_obj_mesh(self, path: Path):
        vertices: list[tuple[float, float, float]] = []
        normals: list[tuple[float, float, float]] = []
        mesh_vertices: list[float] = []
        mesh_normals: list[float] = []
        mesh_colors: list[int] = []
        material_color = DEFAULT_VEHICLE_MATERIAL_COLOR

        def resolve_index(index_text: str, count: int) -> int:
            index = int(index_text)
            if index < 0:
                index = count + index + 1
            return index - 1

        def parse_face_token(token: str) -> tuple[int, int | None]:
            parts = token.split("/")
            vertex_index = resolve_index(parts[0], len(vertices))
            normal_index = None
            if len(parts) >= 3 and parts[2]:
                normal_index = resolve_index(parts[2], len(normals))
            return vertex_index, normal_index

        def face_normal(points: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float]:
            ax, ay, az = points[0]
            bx, by, bz = points[1]
            cx, cy, cz = points[2]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            if length <= 0.000001:
                return 0.0, 0.0, 1.0
            return nx / length, ny / length, nz / length

        for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
            parts = raw.split()
            if not parts or parts[0].startswith("#"):
                continue
            tag = parts[0]
            if tag == "v" and len(parts) >= 4:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif tag == "vn" and len(parts) >= 4:
                normals.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif tag == "usemtl" and len(parts) >= 2:
                material_color = VEHICLE_MATERIAL_COLORS.get(parts[1], DEFAULT_VEHICLE_MATERIAL_COLOR)
            elif tag == "f" and len(parts) >= 4:
                face = [parse_face_token(token) for token in parts[1:]]
                for index in range(1, len(face) - 1):
                    triangle = (face[0], face[index], face[index + 1])
                    points = tuple(vertices[vertex_index] for vertex_index, _ in triangle)
                    fallback_normal = face_normal(points)
                    for vertex_index, normal_index in triangle:
                        vertex = vertices[vertex_index]
                        normal = normals[normal_index] if normal_index is not None else fallback_normal
                        mesh_vertices.extend(vertex)
                        mesh_normals.extend(normal)
                        mesh_colors.extend(material_color)

        vertex_count = len(mesh_vertices) // 3
        if vertex_count < 3 or vertex_count % 3 != 0:
            raise RuntimeError(f"invalid vehicle mesh vertex count: {vertex_count}")

        mesh = rl.Mesh()
        mesh.vertexCount = vertex_count
        mesh.triangleCount = vertex_count // 3
        mesh.vertices = self._alloc_float_array(mesh_vertices)
        mesh.normals = self._alloc_float_array(mesh_normals)
        mesh.colors = self._alloc_uchar_array(mesh_colors)
        return mesh

    def _alloc_float_array(self, values: list[float]):
        data = rl.ffi.cast("float *", rl.mem_alloc(len(values) * rl.ffi.sizeof("float")))
        for index, value in enumerate(values):
            data[index] = value
        return data

    def _alloc_uchar_array(self, values: list[int]):
        data = rl.ffi.cast("unsigned char *", rl.mem_alloc(len(values) * rl.ffi.sizeof("unsigned char")))
        for index, value in enumerate(values):
            data[index] = int(value)
        return data

    def _draw_scene(self, scene: ClusterScene) -> None:
        camera = rl.Camera3D(
            vec3(scene.camera.position),
            vec3(scene.camera.target),
            rl.Vector3(0.0, 0.0, 1.0),
            scene.camera.fovy_deg,
            rl.CameraProjection.CAMERA_PERSPECTIVE,
        )
        rl.begin_mode_3d(camera)
        for strip in scene.highlight_lanes:
            self._draw_strip(strip)
        for strip in scene.road_edges:
            self._draw_strip(strip)
        for strip in scene.lane_markings:
            self._draw_strip(strip)
        for strip in scene.planned_path:
            self._draw_strip(strip)
        for point in scene.radar_points:
            self._draw_radar_point(point)
        for vehicle in scene.vehicles:
            self._draw_vehicle(vehicle)
        rl.end_mode_3d()
        self._draw_radar_point_labels(scene.radar_points, camera)
        self._draw_vehicle_badges(scene.vehicles, camera)

    def _draw_strip(self, strip: MeshStrip) -> None:
        count = min(len(strip.left), len(strip.right))
        if count < 2:
            return

        color = rl_color(strip.color)

        if hasattr(rl, "draw_triangle_strip_3d"):
            points = rl.ffi.new("struct Vector3[]", count * 2)

            for index in range(count):
                left = strip.left[index]
                right = strip.right[index]

                points[index * 2].x = left.x
                points[index * 2].y = left.y
                points[index * 2].z = left.z

                points[index * 2 + 1].x = right.x
                points[index * 2 + 1].y = right.y
                points[index * 2 + 1].z = right.z

            rl.draw_triangle_strip_3d(
                rl.ffi.cast("struct Vector3 *", points),
                count * 2,
                color,
            )
            return

        for index in range(count - 1):
            left_near = vec3(strip.left[index])
            right_near = vec3(strip.right[index])
            left_far = vec3(strip.left[index + 1])
            right_far = vec3(strip.right[index + 1])
            rl.draw_triangle_3d(left_near, right_near, right_far, color)
            rl.draw_triangle_3d(left_near, right_far, left_far, color)

    def _draw_vehicle(self, vehicle: VehicleBox) -> None:
        if self._vehicle_model is not None:
            self._draw_vehicle_shadow(vehicle)
            self._draw_vehicle_model(vehicle)
            return
        self._draw_vehicle_box(vehicle)

    def _draw_radar_point(self, point: RadarPointMarker) -> None:
        rl.draw_sphere(vec3(point.center), point.radius_m, rl_color(point.color))
        rl.draw_sphere_wires(vec3(point.center), point.radius_m * 1.08, 8, 8, rl_color((255, 255, 255, 150)))

    def _draw_radar_point_labels(self, points: tuple[RadarPointMarker, ...], camera) -> None:
        occupied: list[tuple[float, float, float, float]] = []
        ordered = sorted(points, key=lambda point: (point.longitudinal_m, abs(point.lateral_m), point.label))
        for point in ordered[:32]:
            anchor = rl.Vector3(point.center.x, point.center.y, point.center.z + 0.46)
            screen = rl.get_world_to_screen(anchor, camera)
            if screen.x < 430 or screen.x > self.width - 40 or screen.y < 76 or screen.y > self.height - 26:
                continue
            distance = radar_point_distance_label(point)
            speed = radar_point_speed_label(point)
            text_width = max(
                int(rl.measure_text_ex(self._font or rl.get_font_default(), distance, 18, 1).x),
                int(rl.measure_text_ex(self._font or rl.get_font_default(), speed, 16, 1).x),
            )
            width = max(74, text_width + 18)
            height = 40
            x = screen.x - width * 0.5
            y = screen.y - height - 4
            rect_tuple = (x, y, width, height)
            if any(rectangles_overlap(rect_tuple, taken) for taken in occupied):
                continue
            occupied.append(rect_tuple)
            center_x = x + width * 0.5
            shadow = (245, 248, 252)
            text = (8, 10, 12)
            self._draw_text(distance, center_x + 1, y + 10 + 1, 18, shadow, anchor="center")
            self._draw_text(distance, center_x, y + 10, 18, text, anchor="center")
            self._draw_text(speed, center_x + 1, y + 29 + 1, 16, shadow, anchor="center")
            self._draw_text(speed, center_x, y + 29, 16, text, anchor="center")

    def _draw_vehicle_shadow(self, vehicle: VehicleBox) -> None:
        half_width = vehicle.width_m * 0.5
        half_length = vehicle.length_m * 0.5

        def corner(local_x: float, local_y: float, z: float) -> Vec3:
            return Vec3(
                vehicle.center.x + vehicle.right_x * local_x + vehicle.forward_x * local_y,
                vehicle.center.y + vehicle.right_y * local_x + vehicle.forward_y * local_y,
                z,
            )

        shadow = (
            corner(-half_width * 1.12, -half_length * 1.08, 0.018),
            corner(half_width * 1.12, -half_length * 1.08, 0.018),
            corner(half_width * 1.12, half_length * 1.08, 0.018),
            corner(-half_width * 1.12, half_length * 1.08, 0.018),
        )
        self._draw_quad(
            shadow[0],
            shadow[1],
            shadow[2],
            shadow[3],
            (0, 0, 0, int(18 + 34 * clamp(vehicle.confidence, 0.0, 1.0))),
        )

    def _draw_vehicle_model(self, vehicle: VehicleBox) -> None:
        if self._vehicle_model is None:
            return
        yaw_deg = math.degrees(math.atan2(-vehicle.forward_x, vehicle.forward_y))
        position = rl.Vector3(vehicle.center.x, vehicle.center.y, 0.035)
        rotation_axis = rl.Vector3(0.0, 0.0, 1.0)
        scale = rl.Vector3(vehicle.width_m, vehicle.length_m, vehicle.height_m)
        try:
            rl.rl_disable_backface_culling()
            alpha = int(92 + 163 * clamp(vehicle.confidence, 0.0, 1.0))
            tint = rl_color(vehicle.body_color) if vehicle.source == "radarPoint" else rl_color(WHITE, alpha)
            rl.draw_model_ex(self._vehicle_model, position, rotation_axis, yaw_deg, scale, tint)
        finally:
            rl.rl_enable_backface_culling()

    def _draw_vehicle_badges(self, vehicles: tuple[VehicleBox, ...], camera) -> None:
        occupied: list[tuple[float, float, float, float]] = []
        ordered = sorted(
            (vehicle for vehicle in vehicles if vehicle.annotate and vehicle.label),
            key=lambda vehicle: (0 if vehicle.primary else 1 if vehicle.cut_in else 2, -vehicle.confidence),
        )
        for vehicle in ordered:
            anchor = rl.Vector3(vehicle.center.x, vehicle.center.y, vehicle.height_m + 0.55)
            screen = rl.get_world_to_screen(anchor, camera)
            if screen.x < 430 or screen.x > self.width - 40 or screen.y < 88 or screen.y > self.height - 28:
                continue

            distance_m = max(0.0, vehicle.center.y - 4.18)
            rel_text = "" if vehicle.relative_speed_mps is None else f" {vehicle.relative_speed_mps:+.1f}"
            text = f"{vehicle.label} {distance_m:.0f}m{rel_text}"
            if vehicle.ttc_s is not None and vehicle.ttc_s < 9.9:
                sub = f"TTC {vehicle.ttc_s:.1f}s"
            elif vehicle.cut_in:
                sub = "CUT-IN"
            elif vehicle.acceleration_mps2 is not None and abs(vehicle.acceleration_mps2) > 0.2:
                sub = f"{vehicle.confidence:.0%}  a {vehicle.acceleration_mps2:+.1f}"
            else:
                sub = f"{vehicle.confidence:.0%}"
            color = RED if vehicle.ttc_s is not None and vehicle.ttc_s < 3.0 else AMBER if vehicle.cut_in else BLUE if vehicle.confidence >= 0.55 else MUTED
            bg = (254, 250, 238) if vehicle.cut_in else (246, 249, 252)
            border = AMBER if vehicle.cut_in else FAINT
            width = max(90, int(rl.measure_text_ex(self._font or rl.get_font_default(), text, 16, 1).x) + 24)
            height = 42
            x = screen.x - width * 0.5
            y = screen.y - height * 0.5
            rect_tuple = (x, y, width, height)
            if any(rectangles_overlap(rect_tuple, taken) for taken in occupied):
                if not vehicle.primary and not vehicle.cut_in:
                    continue
                for _ in range(3):
                    y -= height + 4
                    rect_tuple = (x, y, width, height)
                    if not any(rectangles_overlap(rect_tuple, taken) for taken in occupied):
                        break
                else:
                    continue
            occupied.append(rect_tuple)
            rect = rl.Rectangle(x, y, width, height)
            rl.draw_rectangle_rounded(rect, 0.18, 8, rl_color(bg, int(178 + 62 * vehicle.confidence)))
            rl.draw_rectangle_rounded_lines_ex(rect, 0.18, 8, 1.4, rl_color(border, int(130 + 80 * vehicle.confidence)))
            self._draw_text(text, x + width * 0.5, y + 14, 16, TEXT, anchor="center")
            self._draw_text(sub, x + width * 0.5, y + 31, 12, color, anchor="center")

    def _draw_vehicle_box(self, vehicle: VehicleBox) -> None:
        half_width = vehicle.width_m * 0.5
        half_length = vehicle.length_m * 0.5
        z0 = 0.035
        z1 = vehicle.height_m + z0

        def corner(local_x: float, local_y: float, z: float) -> Vec3:
            return Vec3(
                vehicle.center.x + vehicle.right_x * local_x + vehicle.forward_x * local_y,
                vehicle.center.y + vehicle.right_y * local_x + vehicle.forward_y * local_y,
                z,
            )

        base = (
            corner(-half_width, -half_length, z0),
            corner(half_width, -half_length, z0),
            corner(half_width, half_length, z0),
            corner(-half_width, half_length, z0),
        )
        top = (
            corner(-half_width, -half_length, z1),
            corner(half_width, -half_length, z1),
            corner(half_width, half_length, z1),
            corner(-half_width, half_length, z1),
        )
        self._draw_vehicle_shadow(vehicle)
        self._draw_quad(base[0], base[1], top[1], top[0], vehicle.rear_color)
        self._draw_quad(base[1], base[2], top[2], top[1], vehicle.side_color)
        self._draw_quad(base[2], base[3], top[3], top[2], vehicle.body_color)
        self._draw_quad(base[3], base[0], top[0], top[3], vehicle.side_color)
        self._draw_quad(top[0], top[1], top[2], top[3], vehicle.body_color)

        inset = 0.22
        highlight = tuple(
            Vec3(
                point.x + (vehicle.center.x - point.x) * inset,
                point.y + (vehicle.center.y - point.y) * inset,
                point.z + 0.006,
            )
            for point in top
        )
        self._draw_quad(highlight[0], highlight[1], highlight[2], highlight[3], vehicle.top_highlight)

        outline = rl_color(vehicle.outline_color)
        edge_points = base + top
        edges = (
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        )
        for start, end in edges:
            rl.draw_line_3d(vec3(edge_points[start]), vec3(edge_points[end]), outline)

    def _draw_quad(
        self,
        p0: Vec3,
        p1: Vec3,
        p2: Vec3,
        p3: Vec3,
        color: tuple[int, int, int, int],
    ) -> None:
        draw_color = rl_color(color)
        rl.draw_triangle_3d(vec3(p0), vec3(p1), vec3(p2), draw_color)
        rl.draw_triangle_3d(vec3(p0), vec3(p2), vec3(p3), draw_color)

    def _draw_hud(self, state: ClusterUiState) -> None:
        sx = self.width / DESIGN_WIDTH
        sy = self.height / DESIGN_HEIGHT
        rl.rl_push_matrix()
        rl.rl_scalef(sx, sy, 1.0)
        self._draw_speed_block(state)
        self._draw_accel_block(state)
        self._draw_turn_signal("left", state.left_signal)
        self._draw_turn_signal("right", state.right_signal)
        self._draw_center_clock(state)
        self._draw_route_overlay(state.route_overlay)
        rl.rl_pop_matrix()

    def _draw_center_clock(self, state: ClusterUiState) -> None:
        if not state.center_clock_text:
            return

        text = state.center_clock_text
        x = DESIGN_WIDTH * 0.5
        y = 58
        size = 54
        spacing = max(1.0, size * 0.02)
        font = self._font or rl.get_font_default()
        measured = rl.measure_text_ex(font, text, size, spacing)

        pad_x = 28
        pad_y = 14
        rect = rl.Rectangle(
            x - measured.x * 0.5 - pad_x,
            y - measured.y * 0.5 - pad_y,
            measured.x + pad_x * 2,
            measured.y + pad_y * 2,
        )

        rl.draw_rectangle_rounded(rect, 0.28, 12, rl_color((8, 10, 12, 150)))
        rl.draw_rectangle_rounded_lines_ex(rect, 0.28, 12, 2.0, rl_color((255, 255, 255, 72)))
        self._draw_text(text, x, y, size, WHITE, anchor="center")
  
    def _draw_route_overlay(self, overlay: RouteOverlay | None) -> None:
        if overlay is None:
            return
        panel_x = 1416
        panel_y = 34
        panel_w = 476
        video_h = 244
        data_y = 300
        self._rounded_rect(panel_x, panel_y, panel_w, 410, 18, (248, 250, 252), FAINT, 2)
        self._draw_route_video(overlay, panel_x + 10, panel_y + 10, panel_w - 20, video_h)
        self._draw_route_data(overlay, panel_x + 18, data_y, panel_w - 36)

    def _draw_route_video(self, overlay: RouteOverlay, x: float, y: float, width: float, height: float) -> None:
        video_rect = rl.Rectangle(x, y, width, height)
        rl.draw_rectangle_rounded(video_rect, 0.04, 10, rl_color((18, 20, 22)))
        if overlay.video_rgba is None or overlay.video_width <= 0 or overlay.video_height <= 0:
            status = overlay.video_status or "qcamera unavailable"
            self._draw_text(status, x + width * 0.5, y + height * 0.5, 20, (212, 218, 224), anchor="center")
            return

        texture = self._route_video_texture_for_overlay(overlay)
        if texture is None:
            return
        source = rl.Rectangle(0.0, 0.0, float(overlay.video_width), float(overlay.video_height))
        scale = min(width / overlay.video_width, height / overlay.video_height)
        draw_w = overlay.video_width * scale
        draw_h = overlay.video_height * scale
        dest = rl.Rectangle(x + (width - draw_w) * 0.5, y + (height - draw_h) * 0.5, draw_w, draw_h)
        rl.draw_texture_pro(texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))

    def _route_video_texture_for_overlay(self, overlay: RouteOverlay):
        size = (overlay.video_width, overlay.video_height)
        if self._route_video_texture is None or self._route_video_size != size:
            if self._route_video_texture is not None:
                rl.unload_texture(self._route_video_texture)
            image = rl.gen_image_color(overlay.video_width, overlay.video_height, rl_color((0, 0, 0)))
            self._route_video_texture = rl.load_texture_from_image(image)
            rl.unload_image(image)
            rl.set_texture_filter(self._route_video_texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            self._route_video_size = size
            self._route_video_frame_id = None

        if overlay.video_frame_id != self._route_video_frame_id:
            expected = overlay.video_width * overlay.video_height * 4
            if len(overlay.video_rgba or b"") != expected:
                return self._route_video_texture
            pixels = rl.ffi.new("unsigned char[]", overlay.video_rgba)
            rl.update_texture(self._route_video_texture, pixels)
            self._route_video_frame_id = overlay.video_frame_id
        return self._route_video_texture

    def _draw_route_data(self, overlay: RouteOverlay, x: float, y: float, width: float) -> None:
        self._draw_text("ROUTE DATA", x, y, 16, MUTED)
        for index, line in enumerate(overlay.data_lines[:10]):
            self._draw_text(line, x, y + 22 + index * 14, 12, TEXT)

    def _draw_speed_block(self, state: ClusterUiState) -> None:
        self._rounded_rect(28, 36, 377, 408, 26, PANEL_BG, FAINT, 2)
        speed_value = int(round(clamp(state.speed_kph, 0.0, MAX_SPEED_KPH)))
        self._draw_text(str(speed_value), 214, 160, 156, TEXT, anchor="center")
        self._draw_text("km/h", 214, 260, 34, MUTED, anchor="center")

        if state.speed_limit_kph is None:
            self._rounded_rect(80, 316, 98, 82, 16, (247, 248, 249), FAINT, 3)
            self._draw_text("--", 129, 346, 30, MUTED, anchor="center")
            self._draw_text("LIMIT", 129, 376, 18, MUTED, anchor="center")
        else:
            center = rl.Vector2(130, 360)
            rl.draw_circle_v(center, 56, rl_color(RED))
            rl.draw_circle_v(center, 47, rl_color(WHITE))
            self._draw_text(str(state.speed_limit_kph), 130, 359, 42, TEXT, anchor="center")

        if state.cruise_kph is not None:
            self._rounded_rect(224, 320, 130, 84, 18, (232, 241, 255), BLUE_SOFT, 2)
            self._draw_text(str(state.cruise_kph), 289, 348, 40, BLUE, anchor="center")
            self._draw_text("SET", 289, 381, 18, BLUE, anchor="center")

    def _draw_accel_block(self, state: ClusterUiState) -> None:
        top = 80
        bottom = 400
        center = (top + bottom) // 2
        self._rounded_rect(440, top, 56, bottom - top, 18, (232, 236, 240), FAINT, 2)
        rl.draw_line_ex(rl.Vector2(424, center), rl.Vector2(512, center), 3, rl_color((88, 96, 104)))
        value = clamp(state.accel_mps2, -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2)
        fill_color = GREEN if value > 0 else RED if value < 0 else MUTED
        if value != 0.0:
            fill_height = int(abs(value) / MAX_ACCEL_MPS2 * ((bottom - top) / 2 - 8))
            if value > 0:
                self._rounded_rect(448, center - fill_height, 40, fill_height, 13, fill_color)
            else:
                self._rounded_rect(448, center, 40, fill_height, 13, fill_color)
        self._draw_text(f"{state.accel_mps2:+.1f}", 468, 48, 38, fill_color, anchor="center")
        self._draw_text("m/s^2", 468, 424, 21, MUTED, anchor="center")

    def _draw_model_status_block(self, state: ClusterUiState) -> None:
        if state.model_confidence is None and state.disengage_risk <= 0.0 and not state.hard_brake_predicted:
            return

        x = 704
        y = 26
        confidence_color = self._model_confidence_color(state)
        confidence_label = "RED" if confidence_color == RED else (state.model_confidence or "--").upper()
        rl.draw_circle_v(rl.Vector2(x + 488, y + 18), 8, rl_color(confidence_color))
        self._draw_text(confidence_label, x + 488, y + 39, 12, confidence_color, anchor="center")

    def _model_confidence_color(self, state: ClusterUiState) -> tuple[int, int, int]:
        if state.hard_brake_predicted or state.disengage_risk > 0.55:
            return RED
        confidence = (state.model_confidence or "").lower()
        if confidence == "green":
            return GREEN
        if confidence == "yellow" or state.disengage_risk > 0.22:
            return AMBER
        if confidence == "red":
            return RED
        return MUTED

    def _draw_model_insight_block(self, state: ClusterUiState) -> None:
        has_risk = bool(state.risk_points) or state.hard_brake_risk > 0.0 or state.brake_press_prob > 0.0
        has_desire = bool(state.desire_state or state.desire_prediction)
        if not (has_risk or has_desire):
            return

        x = 704
        y = 84

        desire_label, desire_prob = dominant_desire(state.desire_state, state.desire_prediction)
        if desire_label:
            desire_color = BLUE if "LC" in desire_label or "KEEP" in desire_label else AMBER if "TURN" in desire_label else MUTED
            self._draw_text("DES", x + 320, y + 14, 11, MUTED, anchor="center")
            self._draw_text(desire_label, x + 320, y + 34, 18, desire_color, anchor="center")
            rl.draw_line_ex(
                rl.Vector2(x + 288, y + 51),
                rl.Vector2(x + 288 + 64 * clamp(desire_prob, 0.0, 1.0), y + 51),
                4,
                rl_color(desire_color, 220),
            )

        risk_values = model_risk_values(state)
        if risk_values:
            self._draw_text("RISK", x + 438, y + 14, 11, MUTED, anchor="center")
            self._draw_tiny_bars(risk_values, x + 404, y + 24, 72, 24)
            peak_risk = max(risk_values)
            risk_color = RED if peak_risk > 0.55 else AMBER if peak_risk > 0.22 else BLUE
            self._draw_text(f"{peak_risk:.0%}", x + 438, y + 57, 11, risk_color, anchor="center")

    def _draw_sparkline(
        self,
        values: tuple[float, ...],
        x: float,
        y: float,
        width: float,
        height: float,
        minimum: float,
        maximum: float,
        color: tuple[int, int, int],
    ) -> None:
        if len(values) < 2:
            return
        rl.draw_line_ex(rl.Vector2(x, y + height * 0.5), rl.Vector2(x + width, y + height * 0.5), 1, rl_color(FAINT))
        span = max(0.001, maximum - minimum)
        previous: rl.Vector2 | None = None
        for index, value in enumerate(values):
            px = x + width * index / max(1, len(values) - 1)
            amount = clamp((value - minimum) / span, 0.0, 1.0)
            py = y + height * (1.0 - amount)
            current = rl.Vector2(px, py)
            if previous is not None:
                rl.draw_line_ex(previous, current, 2, rl_color(color, 210))
            previous = current

    def _draw_tiny_bars(self, values: tuple[float, ...], x: float, y: float, width: float, height: float) -> None:
        count = len(values)
        if count == 0:
            return
        gap = 3.0
        bar_w = max(3.0, (width - gap * (count - 1)) / count)
        for index, value in enumerate(values):
            value = clamp(value, 0.0, 1.0)
            bar_h = max(2.0, height * value)
            color = RED if value > 0.55 else AMBER if value > 0.22 else BLUE
            self._rounded_rect(x + index * (bar_w + gap), y + height - bar_h, bar_w, bar_h, 2, color)

    def _draw_turn_signal(self, side: str, active: bool) -> None:
        cx = 610 if side == "left" else 1310
        cy = 72
        direction = -1 if side == "left" else 1
        fill = GREEN if active else (195, 202, 209, 92)
        outline = (8, 118, 65) if active else (168, 176, 184)

        def point(local_x: float, local_y: float) -> rl.Vector2:
            return rl.Vector2(cx + direction * local_x, cy + local_y)

        tail_rect = rl.Rectangle(cx + direction * -58, cy - 13, direction * 66, 26)
        if tail_rect.width < 0:
            tail_rect.x += tail_rect.width
            tail_rect.width = -tail_rect.width

        head_top = point(8, -34)
        head_tip = point(68, 0)
        head_bottom = point(8, 34)
        if direction < 0:
            head_vertices = (head_top, head_tip, head_bottom)
        else:
            head_vertices = (head_top, head_bottom, head_tip)

        rl.draw_rectangle_rec(tail_rect, rl_color(fill))
        rl.draw_triangle(*head_vertices, rl_color(fill))

        outline_points = [
            point(-58, -13),
            point(8, -13),
            head_top,
            head_tip,
            head_bottom,
            point(8, 13),
            point(-58, 13),
        ]
        line_color = rl_color(outline)
        for index, start in enumerate(outline_points):
            end = outline_points[(index + 1) % len(outline_points)]
            rl.draw_line_ex(start, end, 3, line_color)

    def _rounded_rect(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        radius: float,
        fill: tuple[int, int, int],
        outline: tuple[int, int, int] | None = None,
        outline_width: float = 1.0,
    ) -> None:
        rect = rl.Rectangle(x, y, width, height)
        roundness = max(0.0, min(1.0, radius / max(1.0, min(width, height))))
        rl.draw_rectangle_rounded(rect, roundness, 12, rl_color(fill))
        if outline is not None and outline_width > 0:
            rl.draw_rectangle_rounded_lines_ex(rect, roundness, 12, outline_width, rl_color(outline))

    def _draw_text(
        self,
        text: str,
        x: float,
        y: float,
        size: float,
        color: tuple[int, int, int],
        anchor: str = "left",
    ) -> None:
        if self._font is None:
            self._font = rl.get_font_default()
        spacing = max(1.0, size * 0.02)
        measured = rl.measure_text_ex(self._font, text, size, spacing)
        draw_x = x
        draw_y = y
        if anchor == "center":
            draw_x = x - measured.x * 0.5
            draw_y = y - measured.y * 0.5
        elif anchor == "left":
            draw_y = y - measured.y * 0.5
        rl.draw_text_ex(self._font, text, rl.Vector2(draw_x, draw_y), size, spacing, rl_color(color))
