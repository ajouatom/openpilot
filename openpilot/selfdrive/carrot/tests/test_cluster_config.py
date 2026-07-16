from pathlib import Path
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import normalize_cluster_live_fps


def test_cluster_live_fps_modes_remain_independent_from_map_fps():
  assert [normalize_cluster_live_fps(mode) for mode in range(7)] == [
    0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0,
  ]
  assert normalize_cluster_live_fps(7) == 0.0
  assert normalize_cluster_live_fps("invalid") == 0.0
