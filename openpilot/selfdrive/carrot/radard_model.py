#!/usr/bin/env python3
"""Stable compatibility entry point for the model-based radard."""

from openpilot.selfdrive.carrot.radar.radard_model import main

__all__ = ["main"]


if __name__ == "__main__":
  main()
