"""Recorded-input Carrot state-machine replay, NOT MPC or vehicle simulation.

Only load trusted local decoded.pkl files. Camera observer timestamps are causal;
labels are consulted after decisions for scoring. Parameter polling, lane-change
bookkeeping and driving-mode inference are replaced with recorded/default context.
"""
import argparse
import ast
import bisect
import collections
from enum import Enum
import importlib.util
import json
from pathlib import Path
import pickle
import sys
from types import SimpleNamespace as NS
import time

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from openpilot.selfdrive.carrot.signal_assist import SignalAssist


def load_file(path, name):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  sys.modules[name] = mod
  spec.loader.exec_module(mod)
  return mod


class EventNames:
  def __getattr__(self, name):
    return name


class Events:
  def __init__(self):
    self.events = []

  def add(self, name):
    self.events.append(name)


def planner_class(source):
  helpers = load_file(REPO / 'openpilot/selfdrive/carrot/t_follow.py', 'replay_tf')
  stop = load_file(REPO / 'openpilot/selfdrive/carrot/traffic_stop.py', 'replay_stop')
  filters = load_file(REPO / 'openpilot/common/filter_simple.py', 'replay_filters')
  driving = Enum('DrivingMode', dict(Eco=1, Safe=2, Normal=3, High=4))
  ns = dict(time=time, np=np, Enum=Enum, DT_MDL=.05, CV=NS(MS_TO_KPH=3.6, KPH_TO_MS=1/3.6),
            DrivingMode=driving, DrivingModeDetector=NS, LaneChangeGapPlan=NS, LaneChangeGapTracker=NS,
            Params=lambda: NS(get_int=lambda name: 2 if name == 'MyDrivingMode' else 0),
            Events=Events, EventName=EventNames(), log=NS(LongitudinalPersonality=NS(standard=1)),
            get_carrot_man=lambda sm: None, get_mode_lead_response=lambda requested, mode: requested,
            RADAR_TO_CAMERA=1.52, MyMovingAverage=filters.MyMovingAverage,
            TrafficStopModelLeadMatcher=stop.TrafficStopModelLeadMatcher,
            is_traffic_stop_entry_allowed=stop.is_traffic_stop_entry_allowed)
  ns.update({n: getattr(helpers, n) for n in dir(helpers) if not n.startswith('_')})
  nodes = [n for n in ast.parse(source).body if isinstance(n, (ast.ClassDef, ast.FunctionDef))]
  exec(compile(ast.Module(nodes, type_ignores=[]), 'carrot_source_replay', 'exec'), ns)
  cls = ns['CarrotPlanner']
  cls._params_update = lambda self: None
  cls._update_driving_mode = lambda self, sm: None
  cls._update_model_desire = lambda self, sm: None
  return cls, ns


def nested(x):
  if isinstance(x, dict):
    return NS(**{k: nested(v) for k, v in x.items()})
  if isinstance(x, list):
    return [nested(v) for v in x]
  return x


def run(root, out, reference, cadence, latency):
  source = (REPO / 'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8')
  cls, ns = planner_class(source)
  old_cls, old_ns = planner_class(reference)
  planners = [old_cls(), cls(), cls(SignalAssist())]
  plans = [json.loads(s) for s in (root / 'aligned.jsonl').read_text().splitlines()]
  labels = json.loads((root / 'reviewed_intervals.json').read_text())['intervals']
  trace = []; previous_model = None; disabled_equal = 0; compared = 0
  logged_matches = collections.Counter()
  for segment in dict.fromkeys(r['segment'] for r in plans):
    num = int(segment.rsplit('--', 1)[1])
    directory, mode = {'full': ('candidate_optimized', 'full'),
                       'recorded': ('candidate_optimized', 'sampled'),
                       'parked': ('candidate_trial_cadence', 'sampled')}[cadence]
    path = root / directory / f'{num}.jsonl'
    observations = [json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []
    observations = [r for r in observations if r['mode'] == mode]
    times = [r['timestamp'] for r in observations]
    d = pickle.load((root / 'routes' / segment / 'decoded.pkl').open('rb'))
    ts = {k: [x[0] for x in d[k]] for k in ('carState', 'radarState', 'selfdriveState', 'carControl')}
    models = {t: (valid, m) for t, valid, m in d['modelV2']}
    log_plans = {t: p for t, valid, p in d['longitudinalPlan']}

    def before(k, t):
      idx = bisect.bisect_right(ts[k], t) - 1
      if idx < 0:
        return None
      return d[k][idx]

    for r in [r for r in plans if r['segment'] == segment]:
      t = r['t']
      if r['model_t'] == previous_model or r['model_t'] not in models:
        continue
      previous_model = r['model_t']
      inputs = {k: before(k, t) for k in ts}
      if any(v is None for v in inputs.values()):
        continue
      mv, m = models[r['model_t']]
      cs = nested(inputs['carState'][2]); sd = nested(inputs['selfdriveState'][2])
      radar = nested(inputs['radarState'][2]); control = nested(inputs['carControl'][2])
      sd.personality = getattr(sd, 'personality', 1)
      if isinstance(sd.personality, str):
        sd.personality = {'aggressive': 0, 'standard': 1, 'relaxed': 2, 'moreRelaxed': 3}[sd.personality]
      sm = dict(carState=cs, selfdriveState=sd, radarState=radar,
                modelV2=NS(position=NS(x=m['x'], y=m['y']), velocity=NS(x=m['v']), leadsV3=nested(m['leadsV3'])))
      idx = bisect.bisect_right(times, t - latency) - 1
      obs = None if idx < 0 else {**observations[idx], 'session': segment + ':' + cadence}
      valid = bool(mv and 0 <= t - r['model_t'] < .25 and cs.canValid
                   and all(v[1] and 0 <= t - v[0] < .25 for v in inputs.values()))
      context = dict(now=t, enabled=sd.enabled and control.longActive, valid=valid, drive=str(cs.gearShifter) == 'drive')
      for i, p in enumerate(planners):
        enum = old_ns['DrivingMode'] if i == 0 else ns['DrivingMode']
        p.myDrivingMode = enum(r['mode'])
        kwargs = dict(signal_observation=obs, signal_context=context) if i == 2 else {}
        p.update(sm, cs.vCruise, 'acc', **kwargs)
      outputs = [(p.xState.value, p.trafficState.value, p.v_cruise, p.stop_dist, p.actual_stop_distance,
                  p.fakeCruiseDistance, p.comfort_brake, p.trafficStopModelLeadOffset) for p in planners]
      compared += 1
      assert outputs[0] == outputs[1], (segment, r['seconds'], outputs[:2])
      disabled_equal += 1
      logged_matches['state'] += outputs[0][0] == r['x_state']
      logged_matches['traffic'] += outputs[0][1] == r['traffic']
      dec = planners[2].signal_decision
      frame = r['frame']['index'] if r['frame'] and r['frame']['segment'] == segment else None
      truth = next((state for lo, hi, state in labels.get(str(num), []) if frame is not None and lo <= frame <= hi), None)
      trace.append(dict(segment=segment, seconds=r['seconds'], t=t, frame=frame, truth=truth,
                        speed=cs.vEgo, gas=cs.gasPressed, long_active=control.longActive, valid=valid,
                        base_state=outputs[0][0], state=outputs[2][0], base_traffic=outputs[0][1], traffic=outputs[2][1],
                        base_stop=outputs[0][3], stop=outputs[2][3], base_v_cruise=outputs[0][2], v_cruise=outputs[2][2],
                        logged_state=r['x_state'], **dec.__dict__))
  incidents = {}
  for num, (lo, hi) in {12: (4, 11), 16: (19, 25), 20: (40, 43)}.items():
    q = [r for r in trace if r['segment'].endswith('--'+str(num)) and lo <= r['seconds'] <= hi]
    incidents[num] = dict(frames=len(q), base_go=sum(r['base_state'] not in (3, 5) for r in q),
                         candidate_go=sum(r['state'] not in (3, 5) for r in q), holds=sum(r['hold'] for r in q),
                         red_signs=sum(r['red_sign'] for r in q), valid=sum(r['valid'] for r in q),
                         enabled=sum(r['long_active'] for r in q),
                         reasons=dict(collections.Counter(r['reason'] for r in q)))
  red = [r for r in trace if r['truth'] == 'red' and r['long_active']]
  green = [r for r in trace if r['truth'] == 'green' and r['long_active']]
  moving = [r for r in red if r['speed'] > .3 and r['red_sign']]
  summary = dict(cadence=cadence, delay=latency, frames=compared, disabled_equal=disabled_equal,
                 baseline_log_matches=dict(logged_matches), incidents=incidents,
                 green_release_during_red=sum(r['released'] for r in red),
                 green_hold_frames=sum(r['hold'] for r in green), green_frames=len(green),
                 moving_red_sign_frames=len(moving),
                 moving_changed_stop_frames=sum(r['base_state'] not in (3, 5) and r['state'] in (3, 5) for r in moving),
                 moving_min_distance=min((r['stop'] for r in moving), default=None),
                 reason_counts=dict(collections.Counter(r['reason'] for r in trace)),
                 limits='Recorded-input Carrot state machine only. No actuator/MPC/changed camera response. Defaults for saved Params; recorded mode, no carrotMan navigation. Forward ROI is not ego-lane proof. Reused reviewed video.')
  stem = f'{cadence}_{round(latency*1000)}'
  (out / (stem+'.json')).write_text(json.dumps(summary, indent=2))
  (out / (stem+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in trace))
  print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--data', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--reference-source', type=Path, required=True)
  parser.add_argument('--cadence', choices=['full', 'recorded', 'parked'], default='full')
  parser.add_argument('--delay', type=float, default=.125)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  run(args.data, args.output, args.reference_source.read_text(encoding='utf-8'), args.cadence, args.delay)
