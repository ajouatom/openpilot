#!/usr/bin/env python3
# ruff: noqa: TID251 -- bootstrap display must not import the regular UI/native dependency graph
"""Small recovery display that works without any SCons-built Python modules."""
import argparse
import os
from pathlib import Path
import struct
import time

import pyray as rl

from openpilot.common.network_info import label_with_port, start_ip_monitor
from openpilot.common.startup_recovery import BUSY_STATES, RecoveryUpdate, read_startup_error

ROOT = Path(__file__).resolve().parents[3]
EVENT = struct.Struct('llHHi')
TOUCH_DEVICE = '/dev/input/by-path/platform-894000.i2c-event'
LABELS = {
  'idle': ('시작 실패 · 업데이트 대기', 'Startup failed · waiting for update'),
  'waiting': ('새 수정본을 기다리고 있습니다', 'Waiting for a new fix'),
  'updating': ('업데이트 중 · 전원을 유지하세요', 'Updating · keep power connected'),
  'failed': ('복구 작업 실패 · 다시 시도하세요', 'Recovery action failed · tap to retry'),
  'rebooting': ('업데이트 완료 · 재부팅 중', 'Update complete · rebooting'),
  'cleaning': ('빌드 정리 중 · 전원을 유지하세요', 'Cleaning build · keep power connected'),
  'rebuild_rebooting': ('재부팅 후 다시 빌드합니다', 'Rebooting to rebuild'),
}
AUTO_KO = '30초마다 수정본 확인 · 리빌드는 재부팅 후 수 분 소요'
BUTTON_KO = 'Git pull 후 재부팅'
REBUILD_KO = '리빌드 · 재부팅'
ERROR_KO = '시작 오류 · 누르면 다음 내용'


def device_size():
  try:
    model = Path('/sys/firmware/devicetree/base/model').read_text().strip('\x00\n').lower()
  except OSError:
    model = ''
  return (2160, 1080) if model in ('comma tici', 'comma tizi', 'comma c3', 'comma c3x') else (536, 240)


def wake_display():
  panel = Path('/sys/class/backlight/panel0-backlight')
  try:
    (panel / 'bl_power').write_text('0')
    maximum = int((panel / 'max_brightness').read_text())
    (panel / 'brightness').write_text(str(maximum // 2))
  except (OSError, ValueError):
    pass


class TouchPress:
  """Track actual slot-0 presses; Raylib supplies transformed coordinates."""
  def __init__(self):
    self.fd = None
    self.slot = 0
    self.down = False
    self.previous = False
    self.saw_mt = False
    if os.name != 'posix':
      return
    try:
      self.fd = os.open(TOUCH_DEVICE, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
      pass

  def pressed(self):
    if self.fd is None:
      return rl.is_mouse_button_pressed(0)
    try:
      data = os.read(self.fd, EVENT.size * 128)
    except BlockingIOError:
      return False
    except OSError:
      self.close()
      return False
    pressed = False
    for offset in range(0, len(data) - EVENT.size + 1, EVENT.size):
      _, _, kind, code, value = EVENT.unpack_from(data, offset)
      if kind == 3 and code == 0x2f:
        self.slot = value
      elif kind == 3 and code == 0x39:
        self.saw_mt = True
        if self.slot == 0:
          self.down = value != -1
      elif kind == 1 and code == 0x14a and not self.saw_mt:
        self.down = value != 0
      elif kind == 0 and code == 0:
        pressed |= self.down and not self.previous
        self.previous = self.down
    return pressed

  def close(self):
    if self.fd is not None:
      os.close(self.fd)
      self.fd = None


def fit_text(font, text, size, width):
  text = ' '.join(text.split())
  if rl.measure_text_ex(font, text, size, 0).x <= width:
    return text
  while text and rl.measure_text_ex(font, text + '...', size, 0).x > width:
    text = text[:-1]
  return text + '...'


def error_pages(font, text, scale):
  """Wrap long compiler paths/messages instead of silently clipping the cause."""
  lines = []
  for line in text.splitlines():
    line = line.expandtabs(2)
    while line:
      low, high = 1, len(line)
      while low < high:
        mid = (low + high + 1) // 2
        if rl.measure_text_ex(font, line[:mid], 12 * scale, 0).x <= 504 * scale:
          low = mid
        else:
          high = mid - 1
      lines.append(line[:low])
      line = line[low:]
  return [lines[i:i + 4] for i in range(0, len(lines), 4)] or [['No error output was captured.']]


def draw_screen(font, width, height, state, reason, address, pages, page=0):
  scale = min(width / 536, height / 240)
  x = (width - 536 * scale) / 2
  y = (height - 240 * scale) / 2
  status, detail = state

  def line(text, top, size, color=rl.WHITE):
    fitted = fit_text(font, text, size * scale, 504 * scale)
    rl.draw_text_ex(font, fitted, rl.Vector2(x + 16 * scale, y + top * scale), size * scale, 0, color)

  rl.clear_background(rl.Color(17, 22, 29, 255))
  ko, en = LABELS[status]
  line(ko, 7, 21)
  line(en, 31, 12, rl.LIGHTGRAY)
  line(address + ' · ' + reason, 47, 12, rl.Color(120, 195, 255, 255))
  line(f'{ERROR_KO} ({page + 1}/{len(pages)})', 65, 12, rl.Color(255, 190, 120, 255))
  for i, text in enumerate(pages[page]):
    line(text, 81 + i * 15, 12)
  # Update status never replaces the original startup error.
  line(detail or en, 145, 11, rl.LIGHTGRAY)
  line(AUTO_KO, 160, 11, rl.LIGHTGRAY)
  line('Auto-check every 30s · reboot only after a new update', 173, 10, rl.LIGHTGRAY)
  buttons = []
  busy = status in BUSY_STATES
  for left, ko, en in ((16, REBUILD_KO, 'Clean build & reboot'), (274, BUTTON_KO, 'Git pull & reboot')):
    button = rl.Rectangle(x + left * scale, y + 188 * scale, 246 * scale, 44 * scale)
    buttons.append(button)
    rl.draw_rectangle_rounded(button, .15, 8, rl.DARKGRAY if busy else rl.Color(33, 104, 200, 255))
    for text, top, size in ((ko, 192, 18), (en, 215, 11)):
      size *= scale
      tw = rl.measure_text_ex(font, text, size, 0).x
      rl.draw_text_ex(font, text, rl.Vector2(button.x + (button.width - tw) / 2, y + top * scale), size, 0, rl.WHITE)
  return *buttons, rl.Rectangle(x + 16 * scale, y + 63 * scale, 504 * scale, 79 * scale)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--reason', default='Startup failed')
  parser.add_argument('--log', type=Path)
  parser.add_argument('--preview', type=Path, help='Save a desktop preview; never update or reboot')
  parser.add_argument('--big', action='store_true', help='Preview the C3 layout')
  parser.add_argument('--preview-state', choices=LABELS, default='waiting')
  args = parser.parse_args()
  if not args.preview and os.environ.get('CARROT_STARTUP_RECOVERY') != '1':
    raise SystemExit('Recovery actions are available only from the failed-startup launcher.')
  reason = args.reason
  error = read_startup_error(args.log) or reason
  width, height = (2160, 1080) if args.big else device_size()
  if args.preview:
    rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  rl.init_window(width, height, 'Startup recovery')
  rl.set_target_fps(20)
  rl.set_exit_key(0)
  characters = ''.join(ko + en for ko, en in LABELS.values()) + AUTO_KO + BUTTON_KO + REBUILD_KO + ERROR_KO + reason + error
  codepoints = sorted(set(range(32, 127)) | {ord(c) for c in characters if ord(c) >= 32})
  glyph_buffer = rl.ffi.new('int[]', codepoints)
  font = rl.load_font_ex(str(ROOT / 'openpilot/selfdrive/assets/fonts/Pretendard-Medium.ttf'),
                         round(28 * min(width / 536, height / 240)), rl.ffi.cast('int *', glyph_buffer), len(codepoints))
  rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  pages = error_pages(font, error, min(width / 536, height / 240))
  page = 0
  if args.preview:
    target = rl.load_render_texture(width, height)
    rl.begin_texture_mode(target)
    detail = 'No new commit. Waiting for a fix or network connection.' if args.preview_state == 'waiting' else ''
    draw_screen(font, width, height, (args.preview_state, detail), reason, '192.168.0.10:6999', pages)
    rl.end_texture_mode()
    preview = rl.load_image_from_texture(target.texture)
    rl.image_flip_vertical(preview)
    rl.export_image(preview, str(args.preview.resolve()))
    rl.unload_image(preview)
    rl.unload_render_texture(target)
    rl.unload_font(font)
    rl.close_window()
    return
  touch = TouchPress()
  update = RecoveryUpdate(ROOT)
  next_check = time.monotonic() + 5
  was_busy = False
  wake_display()
  start_ip_monitor()
  try:
    while not rl.window_should_close():
      busy = update.state[0] in BUSY_STATES
      if was_busy and not busy:
        next_check = time.monotonic() + 30
      was_busy = busy
      if not busy and time.monotonic() >= next_check:
        update.start(automatic=True)
        next_check = time.monotonic() + 30
      rl.begin_drawing()
      rebuild_button, update_button, error_area = draw_screen(font, width, height, update.state, reason, label_with_port(6999), pages, page)
      rl.end_drawing()
      if touch.pressed():
        position = rl.get_touch_position(0) if touch.fd is not None else rl.get_mouse_position()
        if rl.check_collision_point_rec(position, rebuild_button):
          update.start_rebuild()
        elif rl.check_collision_point_rec(position, update_button):
          update.start()
        elif rl.check_collision_point_rec(position, error_area):
          page = (page + 1) % len(pages)
      time.sleep(.001)
  finally:
    touch.close()
    rl.unload_font(font)
    rl.close_window()


if __name__ == '__main__':
  main()
