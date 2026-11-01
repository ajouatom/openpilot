"""Read-only, bounded automatic-indicator presentation shared by all displays."""
import json
import math
from pathlib import Path
import time

PATH = Path('/dev/shm/carrot-indicator-display.json')
MAX_BYTES = 1024
MAX_AGE = .6
ORIGINS = {'turn': 'TBT 턴', 'lane': 'ATC 차선변경', 'tbt': '250m 우측',
           'manual': '운전자 차선접근', 'model_path': '모델 경로',
           'model_desire': '모델 차선변경', 'model_turn': '모델 턴', 'bluetooth': 'BT 리모컨'}
REASONS = {'confirming': '조건 확인 중', 'approach': '턴 지점 접근', 'turning': '턴 진행 중',
           'boundary': '차선 경계 접근 중', 'waiting_conditions': '차선·주변 조건 대기',
           'front': '전방 거리·상대속도 조건 대기', 'bsd': '우측 BSD 감지',
           'admission': '송신 조건 확인 중', 'inputs': '입력 상태 대기', 'peer': '수신 장치 확인 대기',
           'completed': '완료', 'returned': '차선 안으로 복귀', 'geometry': '차선 위치 불확실',
           'relabel': '차선 위치 변경', 'driver_opposite': '운전자 반대 조향',
           'driver': '운전자 조작', 'driver_override': '운전자 조작',
           'route_or_distance': '경로·거리 조건 종료', 'route': '경로 조건 종료',
           'timeout': '유지 시간 종료', 'cancelled': '요청 종료', 'ended': '요청 종료',
           'disabled': '기능 꺼짐', 'start': '시작 조건 종료', 'atc': '다른 ATC 동작',
           'turn_started': '턴 시작', 'maneuver': '차선변경 조건 종료',
           'interrupted': '차선변경 중단', 'route_jump': '경로 거리 변경',
           'invalid': '입력 무효', 'active': '유지 중', 'waiting': '시작 대기',
           'changing': '차선변경 중', 'finishing': '차선변경 마무리', 'trigger': '조건 충족'}
STATES = {'prepare': '준비', 'request': '점등 요청', 'confirmed': '점등 확인', 'ended': '종료'}


def clean(value, now):
  try:
    stamp = value['stamp']
    if (type(stamp) not in (int, float) or not math.isfinite(stamp) or not 0 <= now-stamp < MAX_AGE or
        value.get('version') != 1 or value['state'] not in STATES or value['origin'] not in ORIGINS or
        value['direction'] not in ('left', 'right') or value['reason'] not in REASONS or
        value['lamp'] not in ('left', 'right', 'both', 'off', 'unknown')):
      return None
    return {k: value[k] for k in ('version', 'stamp', 'state', 'origin', 'direction', 'reason', 'lamp')}
  except (KeyError, TypeError, ValueError):
    return None


class Reader:
  def __init__(self, path=PATH):
    self.path, self.next_read, self.value = path, 0., None

  def read(self, now=None):
    now = time.monotonic() if now is None else now
    if now >= self.next_read:
      self.next_read = now+.2
      try:
        with self.path.open('rb') as f:
          raw = f.read(MAX_BYTES+1)
        self.value = json.loads(raw) if len(raw) <= MAX_BYTES else None
      except (OSError, ValueError):
        self.value = None
    return clean(self.value, now)


_reader = Reader()


def current():
  return _reader.read()


def lines(value):
  direction = '좌' if value['direction'] == 'left' else '우'
  lamp = {'left': '좌 ON', 'right': '우 ON', 'both': '양쪽 ON', 'off': 'OFF', 'unknown': '미확인'}[value['lamp']]
  return (f"{direction} · {ORIGINS[value['origin']]} · {STATES[value['state']]}",
          f"{REASONS[value['reason']]} / 차량 램프 {lamp}")


_widths = {}


def draw(x, y, width, font, size):
  value = current()
  if value is None:
    return
  import pyray as rl
  colors = {'prepare': (255, 200, 80), 'request': (120, 200, 255),
            'confirmed': (100, 240, 150), 'ended': (190, 190, 190)}
  color = rl.Color(*colors[value['state']], 255)
  rl.draw_rectangle(int(x), int(y), int(width), int(size*2.7), rl.Color(0, 0, 0, 180))
  for i, text in enumerate(lines(value)):
    # Keep the same bounded two-line badge on C3, C4 and the narrow USB panel.
    key = (id(font), text, size)
    if key not in _widths:
      if len(_widths) >= 128:
        _widths.clear()
      # Standalone Jetson HUD does not ship the system.ui runtime.
      _widths[key] = rl.measure_text_ex(font, text, size, 0).x  # noqa: TID251
    measured = _widths[key]
    fitted = min(size, size*(width-16)/max(measured, 1))
    rl.draw_text_ex(font, text, rl.Vector2(x+8, y+4+i*size*1.25), fitted, 0, color if i == 0 else rl.WHITE)
