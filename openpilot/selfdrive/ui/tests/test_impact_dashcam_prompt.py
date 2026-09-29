import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.common.impact_dashcam import FEEDBACK_KEY, NOTICE_KEY, REBOOT_KEY, read_object


class Params(dict):
  def get_bool(self, key):
    return bool(self.get(key))

  def put(self, key, value):
    self[key] = value


def prompt(width=536):
  memory, params = Params(), Params()
  memory[NOTICE_KEY] = {'token': '123', 'created': 1.0}
  sm = Params(selfdriveState=NS(alertSize=NS(raw=0), alertStatus=NS(raw=0)))
  sm.alive = sm.valid = {'selfdriveState': True}
  sm.recv_time = {'selfdriveState': 1.0}
  ui = NS(params_memory=memory, params=params, started=True, sm=sm)
  app = NS(mouse_events=[], width=width, add_nav_stack_tick=lambda cb: None)
  draw = []
  namespace = {'ui_state': ui, 'gui_app': app, 'read_object': read_object,
    'FEEDBACK_KEY': FEEDBACK_KEY, 'NOTICE_KEY': NOTICE_KEY, 'REBOOT_KEY': REBOOT_KEY, 'COUNTDOWN_SECONDS': 10,
    'time': NS(monotonic=lambda: 1.0), 'math': __import__('math'), 'tr': lambda s: s,
    'rl': NS(Rectangle=lambda *args: args, Color=lambda *args: args, WHITE='white',
             draw_rectangle_rounded=lambda *args: None),
    'gui_label': lambda *args, **kwargs: draw.append(args),
    'FontWeight': NS(BOLD='bold', NORMAL='normal'), 'TextAlignment': NS(CENTER=1)}
  namespace['rl'].Rectangle = lambda x, y, width, height: NS(x=x, y=y, width=width, height=height)
  path = Path(__file__).parents[1] / 'impact_dashcam.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
  exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), namespace)
  instance = namespace['ImpactDashcamPrompt']()
  instance.handle_touch()
  return instance, ui, app, draw, namespace


@pytest.mark.parametrize('width', [536, 2160])
def test_notice_draws_and_acknowledges_visibility_on_both_screens(width):
  instance, ui, _, draw, _ = prompt(width)
  instance.render()
  assert len(draw) == 3
  assert '10s' in draw[1][1]
  assert ui.params_memory[FEEDBACK_KEY] == {'token': '123', 'visible': 1.0}
  assert all(d[0].x >= 0 and d[0].x + d[0].width <= width for d in draw)


@pytest.mark.parametrize('touch', ['left_pressed', 'left_down', 'left_released'])
def test_any_touch_cancels_and_is_consumed(touch):
  instance, ui, app, draw, _ = prompt()
  event = {'left_pressed': False, 'left_down': False, 'left_released': False}
  event[touch] = True
  app.mouse_events.append(NS(**event))
  instance.handle_touch()
  instance.render()
  assert ui.params_memory[FEEDBACK_KEY] == {'token': '123', 'cancel': True}
  assert not draw and not app.mouse_events


@pytest.mark.parametrize('cause', ['full', 'critical', 'stale', 'invalid', 'offroad'])
def test_hidden_or_critical_notice_cannot_acknowledge_countdown(cause):
  instance, ui, _, draw, _ = prompt()
  if cause == 'full':
    ui.sm['selfdriveState'].alertSize.raw = 3
  elif cause == 'critical':
    ui.sm['selfdriveState'].alertStatus.raw = 2
  elif cause == 'stale':
    ui.sm.recv_time['selfdriveState'] = 0
  elif cause == 'invalid':
    ui.sm.valid['selfdriveState'] = False
  elif cause == 'offroad':
    ui.started = False
  instance.render()
  assert not draw and FEEDBACK_KEY not in ui.params_memory


def test_cancelled_notice_cannot_restart_countdown_but_new_token_can():
  instance, ui, app, draw, _ = prompt()
  app.mouse_events.append(NS(left_down=True))
  instance.handle_touch()
  instance.render()
  assert not draw
  ui.params_memory[NOTICE_KEY] = {'token': '456', 'created': 1.0}
  instance.handle_touch()
  instance.render()
  assert ui.params_memory[FEEDBACK_KEY]['token'] == '456'
  assert draw
