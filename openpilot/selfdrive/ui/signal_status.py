"""Display the planner's actual signal suspension, not a separate UI road guess."""
import pyray as rl


def signal_assist_status(sm):
  try:
    if not sm.valid['longitudinalPlan'] or not sm.alive['longitudinalPlan'] or signal_detection_paused(sm):
      return 0, 0
    plan = sm['longitudinalPlan']
    color, action = int(getattr(plan, 'signalObservationColor', 0)), int(getattr(plan, 'signalAssistAction', 0))
    return (color if color in (1, 2, 3) else 0, action if action in (1, 2, 3) else 0)
  except (AttributeError, KeyError, TypeError, ValueError):
    return 0, 0


def draw_signal_assist(sm, cx, cy, radius):
  """Outer ring = fresh evidence; centre dot = actual planner intervention.

  A grey ring/red dot explicitly shows a retained veto without fresh color.
  Mixed evidence is amber, never a green departure indication.
  """
  color, action = signal_assist_status(sm)
  if not (color or action):
    return False
  palette = [(160, 165, 170, 255), (255, 75, 75, 255), (60, 255, 120, 255), (255, 185, 40, 255)]
  cx, cy = int(cx), int(cy)
  for offset in (-3, -2, -1):
    rl.draw_circle_lines(cx, cy, radius + offset, rl.Color(*palette[color]))
  if action:
    dot = {1: palette[1], 2: palette[3], 3: palette[2]}[action]
    rl.draw_circle(cx, cy, radius * .32, rl.Color(20, 25, 30, 255))
    rl.draw_circle(cx, cy, radius * .23, rl.Color(*dot))
  return True


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
