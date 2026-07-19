from __future__ import annotations

from dataclasses import dataclass
import multiprocessing
from pathlib import Path
from queue import Empty, Full
import time
from typing import Any

from cluster_models import RouteOverlay


WINDOW_WIDTH = 1100
WINDOW_HEIGHT = 520
UPDATE_INTERVAL_S = 0.1
LAST_FOLDER_FILE = Path.home() / ".cluster_replay_usb_last_folder"
OPTION_LABELS = (
    ("front_radar_only", "front-radar-only"),
    ("route_loop", "route-loop"),
    ("pause_on_cutin", "pause-on-cutin"),
    ("show_route_overlay", "route-overlay"),
    ("road_camera", "road-camera"),
)


@dataclass(frozen=True, slots=True)
class RouteReplayToolAction:
    seek_s: float | None = None
    toggle_pause: bool = False
    closed: bool = False
    open_path: str | None = None
    previous_log: bool = False
    next_log: bool = False
    option_updates: tuple[tuple[str, bool], ...] = ()


class RouteReplayToolsWindow:
    def __init__(self) -> None:
        context = multiprocessing.get_context("spawn")
        self._state_queue = context.Queue(maxsize=1)
        self._command_queue = context.Queue(maxsize=16)
        self._process = context.Process(
            target=_run_replay_tools_window,
            args=(self._state_queue, self._command_queue),
            name="cluster-replay-tools",
            daemon=True,
        )
        self._process.start()
        self._last_update_s = 0.0

    def update(
        self,
        playback_s: float,
        duration_s: float,
        paused: bool,
        status_text: str,
        overlay: RouteOverlay | None,
        current_log_folder: str,
        options: dict[str, bool],
    ) -> None:
        if not self.is_alive:
            return
        now = time.monotonic()
        if now - self._last_update_s < UPDATE_INTERVAL_S:
            return
        self._last_update_s = now
        state = {
            "type": "state",
            "playback_s": float(playback_s),
            "duration_s": float(duration_s),
            "paused": bool(paused),
            "status_text": str(status_text),
            "current_log_folder": str(current_log_folder),
            "options": dict(options),
            "video_rgba": overlay.video_rgba if overlay is not None else None,
            "video_width": overlay.video_width if overlay is not None else 0,
            "video_height": overlay.video_height if overlay is not None else 0,
            "video_status": overlay.video_status if overlay is not None else None,
            "cutin_status": overlay.cutin_status if overlay is not None else None,
            "data_lines": overlay.data_lines if overlay is not None else (),
        }
        self._put_latest(state)

    def poll(self) -> RouteReplayToolAction | None:
        seek_s = None
        toggle_pause = False
        closed = False
        open_path = None
        previous_log = False
        next_log = False
        option_updates: dict[str, bool] = {}
        received = False
        while True:
            try:
                command = self._command_queue.get_nowait()
            except Empty:
                break
            received = True
            command_type = command.get("type")
            if command_type == "seek":
                seek_s = float(command.get("value", 0.0))
            elif command_type == "toggle_pause":
                toggle_pause = not toggle_pause
            elif command_type == "closed":
                closed = True
            elif command_type == "open_path":
                open_path = str(command.get("value", "")).strip() or None
            elif command_type == "previous_log":
                previous_log = True
            elif command_type == "next_log":
                next_log = True
            elif command_type == "option":
                name = str(command.get("name", ""))
                if name:
                    option_updates[name] = bool(command.get("value"))
        if not received:
            return None
        return RouteReplayToolAction(
            seek_s=seek_s,
            toggle_pause=toggle_pause,
            closed=closed,
            open_path=open_path,
            previous_log=previous_log,
            next_log=next_log,
            option_updates=tuple(option_updates.items()),
        )

    @property
    def is_alive(self) -> bool:
        return self._process.is_alive()

    def close(self) -> None:
        if self._process.is_alive():
            self._put_latest({"type": "stop"})
            self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        self._state_queue.close()
        self._command_queue.close()

    def _put_latest(self, value: dict[str, Any]) -> None:
        try:
            self._state_queue.put_nowait(value)
            return
        except Full:
            pass
        try:
            self._state_queue.get_nowait()
        except Empty:
            pass
        try:
            self._state_queue.put_nowait(value)
        except Full:
            pass


def _run_replay_tools_window(state_queue: Any, command_queue: Any) -> None:
    import os

    os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "20,510")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    try:
        import pygame
    except ImportError:
        _put_command(command_queue, {"type": "closed"})
        return

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("Carrot Cluster Replay Tools")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)
    small_font = pygame.font.SysFont("consolas", 14)
    title_font = pygame.font.SysFont("segoeui", 19, bold=True)
    state: dict[str, Any] = {
        "playback_s": 0.0,
        "duration_s": 0.0,
        "paused": False,
        "status_text": "Waiting for replay",
        "current_log_folder": "--",
        "options": {},
        "data_lines": (),
    }
    last_folder = _load_last_folder()
    video_surface = None
    dragging_seek = False
    running = True

    while running:
        while True:
            try:
                update = state_queue.get_nowait()
            except Empty:
                break
            if update.get("type") == "stop":
                running = False
                break
            state = update
            video_surface = _video_surface(pygame, update)

        width, height = screen.get_size()
        open_rect = pygame.Rect(24, 72, 116, 36)
        previous_rect = pygame.Rect(150, 72, 92, 36)
        pause_rect = pygame.Rect(252, 72, 92, 36)
        next_rect = pygame.Rect(354, 72, 92, 36)
        seek_rect = pygame.Rect(24, height - 42, max(120, width - 48), 16)
        option_rects = _option_rects(pygame, width, height)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    _put_command(command_queue, {"type": "toggle_pause"})
                elif event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                    step = -5.0 if event.key == pygame.K_LEFT else 5.0
                    value = max(0.0, min(float(state.get("duration_s", 0.0)), float(state.get("playback_s", 0.0)) + step))
                    _put_command(command_queue, {"type": "seek", "value": value})
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if open_rect.collidepoint(event.pos):
                    current_folder = str(state.get("current_log_folder") or "").strip()
                    selected = _choose_log_file(last_folder or current_folder)
                    if selected is not None:
                        last_folder = str(Path(selected).parent)
                        _save_last_folder(last_folder)
                        _put_command(command_queue, {"type": "open_path", "value": selected})
                elif previous_rect.collidepoint(event.pos):
                    _put_command(command_queue, {"type": "previous_log"})
                elif pause_rect.collidepoint(event.pos):
                    _put_command(command_queue, {"type": "toggle_pause"})
                elif next_rect.collidepoint(event.pos):
                    _put_command(command_queue, {"type": "next_log"})
                elif seek_rect.inflate(0, 20).collidepoint(event.pos):
                    dragging_seek = True
                    _send_seek(command_queue, event.pos[0], seek_rect, state)
                else:
                    option_name = next(
                        (name for name, rect in option_rects.items() if rect.collidepoint(event.pos)),
                        None,
                    )
                    if option_name is not None:
                        options = dict(state.get("options") or {})
                        value = not bool(options.get(option_name, False))
                        options[option_name] = value
                        state["options"] = options
                        _put_command(command_queue, {"type": "option", "name": option_name, "value": value})
                    else:
                        _put_command(command_queue, {"type": "toggle_pause"})
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if dragging_seek:
                    _send_seek(command_queue, event.pos[0], seek_rect, state)
                dragging_seek = False
            elif event.type == pygame.MOUSEMOTION and dragging_seek:
                _send_seek(command_queue, event.pos[0], seek_rect, state)

        _draw_replay_tools(
            pygame,
            screen,
            state,
            video_surface,
            font,
            small_font,
            title_font,
            open_rect,
            previous_rect,
            pause_rect,
            next_rect,
            option_rects,
            seek_rect,
        )
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    _put_command(command_queue, {"type": "closed"})


def _video_surface(pygame: Any, state: dict[str, Any]):
    rgba = state.get("video_rgba")
    width = int(state.get("video_width", 0))
    height = int(state.get("video_height", 0))
    if not rgba or width <= 0 or height <= 0 or len(rgba) != width * height * 4:
        return None
    try:
        return pygame.image.frombytes(rgba, (width, height), "RGBA")
    except ValueError:
        return None


def _draw_replay_tools(
    pygame: Any,
    screen: Any,
    state: dict[str, Any],
    video_surface: Any,
    font: Any,
    small_font: Any,
    title_font: Any,
    open_rect: Any,
    previous_rect: Any,
    pause_rect: Any,
    next_rect: Any,
    option_rects: dict[str, Any],
    seek_rect: Any,
) -> None:
    width, height = screen.get_size()
    bg = (18, 21, 25)
    panel = (28, 32, 37)
    border = (62, 68, 76)
    text = (232, 235, 239)
    muted = (151, 158, 168)
    blue = (91, 165, 255)
    amber = (255, 179, 62)
    screen.fill(bg)

    screen.blit(title_font.render("REPLAY TOOLS", True, text), (24, 17))
    status = str(state.get("status_text", ""))
    screen.blit(small_font.render(status[:110], True, muted), (184, 21))
    current_folder = str(state.get("current_log_folder") or "--")
    screen.blit(small_font.render(f"LOG: {current_folder}"[:135], True, text), (24, 46))

    for rect, label in (
        (open_rect, "OPEN LOG"),
        (previous_rect, "< PREV"),
        (pause_rect, "PLAY" if bool(state.get("paused")) else "PAUSE"),
        (next_rect, "NEXT >"),
    ):
        pygame.draw.rect(screen, (38, 43, 49), rect)
        pygame.draw.rect(screen, border, rect, 1)
        rendered = small_font.render(label, True, text)
        screen.blit(rendered, rendered.get_rect(center=rect.center))

    content_bottom = height - 139
    debug_width = min(380, max(300, int(width * 0.35)))
    video_rect = pygame.Rect(24, 120, max(160, width - debug_width - 60), max(120, content_bottom - 120))
    debug_rect = pygame.Rect(video_rect.right + 18, 120, width - video_rect.right - 42, video_rect.height)
    pygame.draw.rect(screen, panel, video_rect)
    pygame.draw.rect(screen, border, video_rect, 1)
    pygame.draw.rect(screen, panel, debug_rect)
    pygame.draw.rect(screen, border, debug_rect, 1)

    if video_surface is not None:
        scale = min(video_rect.width / video_surface.get_width(), video_rect.height / video_surface.get_height())
        target = (
            max(1, int(video_surface.get_width() * scale)),
            max(1, int(video_surface.get_height() * scale)),
        )
        scaled = pygame.transform.smoothscale(video_surface, target)
        screen.blit(scaled, (video_rect.centerx - target[0] // 2, video_rect.centery - target[1] // 2))
    else:
        video_status = str(state.get("video_status") or "ROUTE VIDEO WAITING")
        label = font.render(video_status[:70], True, muted)
        screen.blit(label, label.get_rect(center=video_rect.center))

    x = debug_rect.x + 16
    y = debug_rect.y + 14
    screen.blit(small_font.render("LOG RADARSTATE CUT-IN", True, muted), (x, y))
    y += 25
    cutin = str(state.get("cutin_status") or "waiting")
    cutin_color = amber if ": YES" in cutin else text
    screen.blit(font.render(cutin[:45], True, cutin_color), (x, y))
    y += 34
    for line in tuple(state.get("data_lines") or ())[:12]:
        screen.blit(small_font.render(str(line)[:54], True, text), (x, y))
        y += 24
        if y > debug_rect.bottom - 20:
            break

    options = dict(state.get("options") or {})
    for name, label in OPTION_LABELS:
        rect = option_rects[name]
        checked = bool(options.get(name, False))
        box = pygame.Rect(rect.x, rect.y + 2, 16, 16)
        pygame.draw.rect(screen, blue if checked else panel, box)
        pygame.draw.rect(screen, border, box, 1)
        if checked:
            pygame.draw.line(screen, (255, 255, 255), (box.x + 3, box.y + 8), (box.x + 7, box.y + 12), 2)
            pygame.draw.line(screen, (255, 255, 255), (box.x + 7, box.y + 12), (box.x + 14, box.y + 4), 2)
        screen.blit(small_font.render(label, True, text), (rect.x + 23, rect.y))

    duration = max(0.001, float(state.get("duration_s", 0.0)))
    playback = max(0.0, min(duration, float(state.get("playback_s", 0.0))))
    ratio = playback / duration
    pygame.draw.rect(screen, (64, 70, 78), seek_rect)
    if ratio > 0.0:
        pygame.draw.rect(screen, blue, (seek_rect.x, seek_rect.y, max(2, int(seek_rect.width * ratio)), seek_rect.height))
    knob_x = seek_rect.x + int(seek_rect.width * ratio)
    pygame.draw.circle(screen, text, (knob_x, seek_rect.centery), 9)
    time_text = f"{_format_time(playback)} / {_format_time(duration)}"
    rendered_time = font.render(time_text, True, text)
    screen.blit(rendered_time, (seek_rect.x, seek_rect.y - 27))


def _option_rects(pygame: Any, width: int, height: int) -> dict[str, Any]:
    available_width = max(1, width - 48)
    columns = 3
    item_width = max(180, available_width // columns)
    return {
        name: pygame.Rect(
            24 + (index % columns) * item_width,
            height - 123 + (index // columns) * 25,
            item_width - 8,
            24,
        )
        for index, (name, _label) in enumerate(OPTION_LABELS)
    }


def _choose_log_file(initial_folder: str) -> str | None:
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        selected = filedialog.askopenfilename(
            title="Open cluster replay log",
            initialdir=initial_folder if Path(initial_folder).is_dir() else None,
            filetypes=(
                ("Openpilot logs", "rlog.zst qlog.zst rlog.bz2 qlog.bz2"),
                ("All files", "*.*"),
            ),
        )
        root.destroy()
        return selected or None
    except Exception:
        return None


def _load_last_folder() -> str:
    try:
        folder = LAST_FOLDER_FILE.read_text(encoding="utf-8").strip()
        return folder if Path(folder).is_dir() else ""
    except OSError:
        return ""


def _save_last_folder(folder: str) -> None:
    try:
        LAST_FOLDER_FILE.write_text(folder, encoding="utf-8")
    except OSError:
        pass


def _send_seek(command_queue: Any, mouse_x: int, seek_rect: Any, state: dict[str, Any]) -> None:
    ratio = max(0.0, min(1.0, (mouse_x - seek_rect.x) / max(1, seek_rect.width)))
    duration = max(0.0, float(state.get("duration_s", 0.0)))
    _put_command(command_queue, {"type": "seek", "value": duration * ratio})


def _put_command(command_queue: Any, command: dict[str, Any]) -> None:
    try:
        command_queue.put_nowait(command)
    except Full:
        pass


def _format_time(seconds: float) -> str:
    total = max(0, int(seconds))
    return f"{total // 60}:{total % 60:02d}"
