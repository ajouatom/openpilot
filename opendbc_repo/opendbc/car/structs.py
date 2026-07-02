import os
import capnp
from opendbc.car.common.basedir import BASEDIR

try:
  from openpilot.cereal import car
except ImportError:
  capnp.remove_import_hook()
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"), "/car.capnp", imports=[BASEDIR, os.path.join(BASEDIR, "include")])

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule
