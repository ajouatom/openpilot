"""Display the planner's actual signal suspension, not a separate UI road guess."""
import pyray as rl


def signal_detection_paused(sm):
  try:
    return bool(sm.valid['longitudinalPlan'] and sm.alive['longitudinalPlan']
                and getattr(sm['longitudinalPlan'], 'signalDetectionSuspended', False))
  except (AttributeError, KeyError, TypeError):
    return False


def draw_signal_pause(cx, cy, radius):
  cx, cy = int(cx), int(cy)
  rl.draw_circle(cx, cy, radius, rl.Color(75, 80, 85, 245))
  rl.draw_circle_lines(cx, cy, radius, rl.Color(215, 220, 225, 255))
  for side in (-1, 1):
    x = cx + side * radius * .27
    rl.draw_line_ex(rl.Vector2(x, cy - radius * .45), rl.Vector2(x, cy + radius * .45),
                    max(3., radius * .18), rl.Color(235, 235, 235, 255))
