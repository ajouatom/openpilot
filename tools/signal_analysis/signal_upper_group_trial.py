"""Offline upper/lower grouping trial. Never imported by the vehicle worker.

The Santa Fe image replay still fails departure with this rule: colour/identity
qualification fails upstream. Keep this candidate out of live control. This
geometry separates image rows; it does not classify a rear lamp or assign lanes.
"""
from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard


class UpperSignalGroupTrial(SignalStartGuard):
  @staticmethod
  def separate_lower_red(red, a, b):
    """Only a paired overhead transition may exclude a separate lower row.

    This is an image grouping hypothesis, not metric distance or lane assignment.
    Nearby rear lamps can be larger than distant overhead lamps. Such candidates
    need substantially more vertical separation than the small/lower exception.
    A single green never grants either exception at the final release gate.
    """
    width = min(a[2]-a[0], b[2]-b[0])
    height = min(a[3]-a[1], b[3]-b[1])
    rw, rh = red[2]-red[0], red[3]-red[1]
    separation = red[1] - max(a[3], b[3])
    upper_height = max(a[3]-a[1], b[3]-b[1])
    small_lower = rw <= .7*width and rh <= .7*height and separation >= 2*upper_height
    separate_row = rw <= 2*width and rh <= 2*height and separation >= 6*max(upper_height, rh)
    return small_lower or separate_row


def compare_recorded_inputs(rows):
  """Compare identical, chronological recorded inputs; never simulate motion.

  Each row has `now`, `observation`, and the explicit keyword `context` accepted
  by SignalStartGuard.update. Optional `truth` is a reviewed scene colour, not a
  detector prediction. Callers must preserve capture timestamps and arrivals.
  """
  from dataclasses import asdict

  guards = (SignalStartGuard(), UpperSignalGroupTrial())
  counts = [dict(frames=0, hold=0, green=0, releases=0, reviewed_red_releases=0) for _ in guards]
  changes, releases = [], []
  for index, row in enumerate(rows):
    decisions = [asdict(g.update(row['now'], row['observation'], **row['context'])) for g in guards]
    for variant, decision in enumerate(decisions):
      count = counts[variant]
      count['frames'] += 1
      count['hold'] += decision['hold']
      count['green'] += decision['green_confirmed']
      count['releases'] += decision['released']
      count['reviewed_red_releases'] += decision['released'] and row.get('truth') == 'red'
      if decision['released']:
        releases.append(dict(index=index, now=row['now'], variant=variant, truth=row.get('truth'), **decision))
    if decisions[0] != decisions[1]:
      changes.append(dict(index=index, now=row['now'], baseline=decisions[0], trial=decisions[1]))
  return dict(baseline=counts[0], trial=counts[1], changes=changes, releases=releases)


def main():
  import argparse
  import json
  from pathlib import Path

  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('inputs', type=Path, help='private JSONL recorded guard inputs')
  parser.add_argument('output', type=Path, help='private comparison JSON')
  args = parser.parse_args()
  with args.inputs.open(encoding='utf-8') as stream:
    result = compare_recorded_inputs(json.loads(line) for line in stream if line.strip())
  args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
  print(json.dumps({key: result[key] for key in ('baseline', 'trial')}))


if __name__ == '__main__':
  main()

