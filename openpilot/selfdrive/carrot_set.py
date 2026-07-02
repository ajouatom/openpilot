#!/usr/bin/env python3
import sys
from openpilot.common.params import Params

def set_params():
  key = sys.argv[1]
  value = sys.argv[2]
  print(f"Setting {key} to {value}")
  print(f"Current value: {Params().get(key)}")
  Params().put(key, value)
  print(f"New value: {Params().get(key)}")

if __name__ == "__main__":
  set_params()
