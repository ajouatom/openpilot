"""A global, touch-cancellable notice shared by the C3 and C4 UI."""
import math
import time

import pyray as rl

from openpilot.common.impact_dashcam import COUNTDOWN_SECONDS, FEEDBACK_KEY, NOTICE_KEY, REBOOT_KEY, read_object
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, TextAlignment, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.label import gui_label


class ImpactDashcamPrompt:
  def __init__(self):
    self.notice = {}
    self.shown_since = None
    self.cancelled_token = ""
    gui_app.add_nav_stack_tick(self.handle_touch)

  def handle_touch(self):
    notice = read_object(ui_state.params_memory, NOTICE_KEY)
    if notice.get('token') != self.notice.get('token'):
      self.shown_since = None
    self.notice = notice
    if not notice or not ui_state.started or ui_state.params.get_bool(REBOOT_KEY):
      return
    if any(ev.left_down or ev.left_pressed or ev.left_released for ev in gui_app.mouse_events):
      self.cancelled_token = notice['token']
      ui_state.params_memory.put(FEEDBACK_KEY, {'token': notice['token'], 'cancel': True})
      # The cancelling touch must not activate a control underneath the notice.
      gui_app.mouse_events.clear()

  def render(self):
    if not self.notice or not ui_state.started or self.notice.get('token') == self.cancelled_token:
      return
    sm = ui_state.sm
    now = time.monotonic()
    # Preserve full/critical driving alerts. The controller cancels its timer
    # if the UI cannot keep presenting this notice, or stops responding.
    ss = sm['selfdriveState']
    if (not sm.alive['selfdriveState'] or not sm.valid['selfdriveState']
        or now - sm.recv_time['selfdriveState'] > 0.5 or ss.alertSize.raw == 3 or ss.alertStatus.raw == 2):
      return
    if self.shown_since is None:
      self.shown_since = now
    rebooting = ui_state.params.get_bool(REBOOT_KEY)
    remaining = max(0, math.ceil(COUNTDOWN_SECONDS - (now - self.shown_since)))
    title = tr("Impact suspected")
    line = tr("Dashcam mode: reboot in {seconds}s").format(seconds=remaining)
    hint = tr("Touch anywhere to cancel")
    if rebooting:
      line, hint = tr("Rebooting into Dashcam Mode"), tr("openpilot disabled")
    scale = gui_app.width / 536.0
    rect = rl.Rectangle(8 * scale, 8 * scale, gui_app.width - 16 * scale, 100 * scale)
    rl.draw_rectangle_rounded(rect, 0.15, 8, rl.Color(165, 68, 0, 245))
    for index, (text, size) in enumerate(((title, 27), (line, 23), (hint, 21))):
      gui_label(rl.Rectangle(rect.x + 8 * scale, rect.y + (5 + index * 31) * scale,
                             rect.width - 16 * scale, 30 * scale), text, int(size * scale), rl.WHITE,
                font_weight=FontWeight.BOLD if index == 0 else FontWeight.NORMAL, alignment=TextAlignment.CENTER)
    if not rebooting:
      # Only a rendered frame acknowledges visibility. The reboot decision is
      # made by selfdrived after a full ten seconds of these acknowledgements.
      ui_state.params_memory.put(FEEDBACK_KEY, {'token': self.notice['token'], 'visible': now})
