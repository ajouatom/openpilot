"""수신기 단독 검사의 경로와 미빌드 Params 입출력만 준비한다.

스키마·CAN 디코더·Base·MyTrack과 판단 함수는 대체하지 않는다.
실제 Params가 빌드되어 있으면 그대로 두며, 없을 때의 생성자는 접근을 거절한다.
"""

import importlib
import sys
import types
from pathlib import Path


class BlockedParams:
  def __init__(self, *args, **kwargs):
    raise RuntimeError("수신기 오프라인 검사에서 운영 Params 접근은 허용하지 않습니다")


def prepare():
  root = Path(__file__).resolve().parents[2]
  for path in (root, root / "opendbc_repo"):
    if str(path) not in sys.path:
      sys.path.insert(0, str(path))
  try:
    importlib.import_module("openpilot.common.params")
  except ModuleNotFoundError as error:
    if error.name != "openpilot.common.params_pyx":
      raise
    params = types.ModuleType("openpilot.common.params")
    params.Params = BlockedParams
    sys.modules[params.__name__] = params
    return {"params_io": "미빌드 생성자 차단", "algorithm_mock": False}
  return {"params_io": "기존 모듈 보존", "algorithm_mock": False}
