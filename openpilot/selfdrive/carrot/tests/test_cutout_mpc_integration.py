"""Execute the production MPC update with a recording solver on Windows too."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_cutout import cutout_obstacle_relief
from openpilot.selfdrive.controls.lib.longitudinal_preview import get_lead_accel_mpc_request
from openpilot.selfdrive.carrot.traffic_stop import get_traffic_stop_distance_adjust, get_traffic_stop_obstacle_distance


def run_update(*, confidence=0., mode="acc", reset=False, enabled=True, second_distance=100., stop_x=1000.):
  path = Path(__file__).resolve().parents[2] / "controls/lib/longitudinal_mpc_lib/long_mpc.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  mpc = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "LongitudinalMpc")
  methods = [node for node in mpc.body if isinstance(node, ast.FunctionDef)
             and node.name in ("update", "update_predicted_danger_margin")]
  helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
             and node.name in ("get_stopped_equivalence_factor", "get_safe_obstacle_distance", "desired_follow_distance")]
  times = np.array([10.*(i/12)**2 for i in range(13)])
  namespace = {"np": np, "log": NS(LongitudinalPersonality=NS(standard=1)),
                   "COMFORT_BRAKE": 2.5, "STOP_DISTANCE": 6., "ACCEL_MIN": -3.5, "N": 12,
                   "T_IDXS": times, "T_DIFFS": np.diff(times, prepend=0.),
                   "FCW_IDXS": times<5., "PRED_DANGER_IDXS": (times>.2)&(times<3.),
                   "SOURCES": ["lead0", "lead1", "cruise", "e2e"], "LEAD_DANGER_FACTOR": .8,
                   "A_CHANGE_COST_STARTING": 10., "COST_E_DIM": 5, "CRASH_DISTANCE": .25,
                   "LEAD_ACCEL_MIN_TRACK_FRAMES": 3,
                   "get_lead_accel_mpc_request": get_lead_accel_mpc_request,
                   "get_traffic_stop_distance_adjust": get_traffic_stop_distance_adjust,
                   "get_traffic_stop_obstacle_distance": get_traffic_stop_obstacle_distance,
                   "cutout_obstacle_relief": cutout_obstacle_relief}
  exec(compile(ast.Module(body=helpers+methods, type_ignores=[]), str(path), "exec"), namespace)
  lead = NS(status=True, radar=True, radarTrackId=50, dRel=25., vRel=-1., vLead=14.,
            aLeadK=-.5, aLeadTau=1.5, modelProb=.99, cutOutTime=1., cutOutConfidence=confidence)
  second = NS(**(vars(lead) | {"dRel": second_distance, "radarTrackId": 51, "cutOutConfidence": 0.}))
  carrot = NS(comfort_brake=2.5, stop_distance=6., v_cruise=20., stop_dist=stop_x, mode=mode,
              trafficStopDistanceAdjust=0., trafficStopModelLeadOffset=0., leadAccelResponse=0,
              get_T_FOLLOW=lambda *a, **kw:1.45, dynamic_t_follow=lambda tf,*a:tf)
  self = NS(x0=np.array([0., 15., 0.]), source="lead0", mode=mode, max_a=1.5, cruise_min_a=-1.2,
            params=np.zeros((13,8)), prev_a=np.zeros(13), yref=np.zeros((13,6)),
            solver=NS(set=lambda *a:None), set_weights=lambda *a,**kw:None, crash_cnt=0,
            x_sol=np.column_stack([15.*times, np.full(13,15.), np.zeros(13)]), run=lambda:None)
  self.process_lead=lambda l:(np.column_stack([l.dRel+l.vLead*times, np.full(13,l.vLead)]), l.vLead)
  self.update_predicted_danger_margin=lambda *a:namespace["update_predicted_danger_margin"](self,*a)
  arrays=[np.zeros(13) for _ in range(4)]
  namespace["update"](self, carrot, reset, NS(leadOne=lead, leadTwo=second), 20., *arrays, cutout_relief_enabled=enabled)
  return self, times


def test_acc_changes_only_post_clearance_obstacle_and_keeps_collision_diagnostics():
  baseline, times = run_update()
  changed, _ = run_update(confidence=1.)
  np.testing.assert_array_equal(changed.params[times<=1.3], baseline.params[times<=1.3])
  assert np.any(changed.params[:,2] > baseline.params[:,2])
  np.testing.assert_array_equal(changed.params[:,[0,1,3,4,5,6,7]], baseline.params[:,[0,1,3,4,5,6,7]])
  assert changed.predicted_danger_margin == baseline.predicted_danger_margin
  assert changed.crash_cnt == baseline.crash_cnt


@pytest.mark.parametrize("kwargs", ({"mode": "blended"}, {"reset": True}, {"enabled": False}, {"second_distance": 10.}, {"stop_x": 5.}))
def test_other_obstacles_modes_and_disengagement_keep_control(kwargs):
  baseline, _ = run_update(**kwargs)
  changed, _ = run_update(confidence=1., **kwargs)
  np.testing.assert_array_equal(changed.params, baseline.params)


@pytest.mark.parametrize("gas,reset,force,stop,expected", ((False, False, False, False, True),
  (True, False, False, False, False), (False, True, False, False, False),
  (False, False, True, False, False), (False, False, False, True, False)))
def test_planner_blocks_relief_on_override_or_stopping(gas, reset, force, stop, expected):
  path = Path(__file__).resolve().parents[2] / "controls/lib/longitudinal_planner.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  expression = next(node.value for node in ast.walk(tree) if isinstance(node, ast.keyword) and node.arg == "cutout_relief_enabled")
  assert eval(compile(ast.Expression(expression), str(path), "eval"), {
    "reset_state": reset, "sm": {"carState": NS(gasPressed=gas)}, "force_slow_decel": force,
    "self": NS(output_should_stop=stop)}) == expected
