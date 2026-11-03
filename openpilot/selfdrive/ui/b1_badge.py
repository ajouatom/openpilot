"""Persistent B1 link indicator on the C3 and C4 home and driving screens."""
import json
from pathlib import Path
import time

ENROLLED = Path('/data/carrot/b1-monitor.json')
STATUS = Path('/dev/shm/carrot-b1-link.json')
LABELS = {
  'updating': ('S3 · 업데이트', (96, 218, 243)),
  'offline': ('S3 · 연결 대기', (170, 178, 188)),
  'auth': ('S3 · 인증 중', (255, 197, 89)),
  'missing': ('S3 · CAN 대기', (202, 147, 255)),
  'ready': ('S3 · CAN 연결', (96, 232, 162)),
  'waiting': ('S3 · CAN 연결', (96, 232, 162)),
  'sending': ('S3 · 송신', (96, 218, 243)),
  'driver_override': ('S3 · 레버 우선', (255, 197, 89)),
  'error': ('S3 · CAN 오류', (252, 128, 110)),
}


def status(now, enrolled=ENROLLED, path=STATUS):
  if not enrolled.is_file():
    return None
  try:
    with path.open('rb') as stream:
      record = json.loads(stream.read(256))
    if 0 <= now-record['time'] < 2 and record['state'] in LABELS:
      return record['state']
  except (OSError, KeyError, ValueError, TypeError):
    pass
  return 'offline'


_next = 0.
_state = None


def draw(rect):
  global _next, _state
  now = time.monotonic()
  if now >= _next:
    _state = status(now)
    _next = now + .35
  if _state is None:
    return
  import pyray as rl
  from openpilot.system.ui.lib.application import FontWeight, gui_app
  label, rgb = LABELS[_state]
  size = 24 if rect.width >= 1200 else 16
  width = size*8
  x, y = rect.x + rect.width*.72 - width/2, rect.y + size*.55
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, width, size*1.55), .7, 8, rl.Color(10, 18, 25, 220))
  rl.draw_circle(int(x+size*.62), int(y+size*.77), size*.19, rl.Color(*rgb, 255))
  rl.draw_text_ex(gui_app.font(FontWeight.DISPLAY), label, rl.Vector2(x+size*1.05, y+size*.24), size, 0, rl.WHITE)
