import ast
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai.lane_line_time import lane_line_time


@pytest.mark.parametrize('name', ['CAM_0x2a4', 'CAM_0x362'])
def test_actual_parser_cache_timestamp_and_carstate_publication(name):
  parser = CANParser('hyundai_canfd_generated', [(name, 20)], 2)
  cached = parser.vl[name]
  packer = CANPacker('hyundai_canfd_generated')
  values = {'LEFT_LANE_TYPE':1, 'LEFT_LANE_COLOR':2, 'RIGHT_LANE_TYPE':0, 'RIGHT_LANE_COLOR':1}
  now = 10_000_000_000
  assert lane_line_time((parser,), cached) == 0
  parser.update([(now, [packer.make_can_msg(name, 2, values)])])
  assert parser.vl[name] is cached
  assert lane_line_time((None, parser), cached) == now
  parser.update([(now + 300_000_000, [packer.make_can_msg(name, 0, values)])])
  assert lane_line_time((parser,), cached) == now  # wrong bus cannot refresh it

  tree = ast.parse(Path('opendbc_repo/opendbc/car/hyundai/carstate.py').read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'update_canfd')
  start = next(i for i, n in enumerate(method.body) if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == 'lane_info')
  ret = structs.CarState()
  cs = N(cam_0x2a4=cached if name == 'CAM_0x2a4' else None,
         cam_0x362=cached if name == 'CAM_0x362' else None, cp_cam=parser, cp_alt=None)
  exec(compile(ast.Module(body=method.body[start:start+3], type_ignores=[]), '<actual-lane-publication>', 'exec'),
       {'self':cs, 'ret':ret, 'lane_line_time':lane_line_time})
  assert ret.laneLineInfoMonoTime == now
  assert ret.leftLaneLine == 21 and ret.rightLaneLine == 10
