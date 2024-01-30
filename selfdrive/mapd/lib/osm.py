import os
import sys
import subprocess
import numpy as np
from openpilot.selfdrive.mapd.lib.geo import R

from openpilot.common.basedir import PYEXTRADIR
sys.path.append(os.path.join(PYEXTRADIR, "pyextra"))

import overpy


def create_way(way_id, node_ids, from_way):
  """
  Creates and OSM Way with the given `way_id` and list of `node_ids`, copying attributes and tags from `from_way`
  """
  return overpy.Way(way_id, node_ids=node_ids, attributes={}, result=from_way._result,
                    tags=from_way.tags)


class OSM():
  def __init__(self):
    self.api = overpy.Overpass()
    # self.api = overpy.Overpass(url='https://z.overpass-api.de/api/interpreter')

  def fetch_road_ways_around_location(self, lat, lon, radius):
    # Calculate the bounding box coordinates for the bbox containing the circle around location.
    bbox_angle = np.degrees(radius / R)
    # fetch all ways and nodes on this ways in bbox
    bbox_str = f'{str(lat - bbox_angle)},{str(lon - bbox_angle)},{str(lat + bbox_angle)},{str(lon + bbox_angle)}'
    q = """
        way(""" + bbox_str + """)
          [highway]
          [highway!~"^(footway|path|corridor|bridleway|steps|cycleway|construction|bus_guideway|escape|service|track)$"];
        (._;>;);
        out;
        """
    try:
      print("Query OSM from remote Server")
      ways = self.api.query(q).ways
    except Exception as e:
      print(f'Exception while querying OSM:\n{e}')
      ways = []

    return ways
