#!/usr/bin/env python3
import os
import runpy
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
TARGET = os.path.join(ROOT, "openpilot", "tools", "plotjuggler", "juggle.py")

sys.argv[0] = TARGET
runpy.run_path(TARGET, run_name="__main__")
