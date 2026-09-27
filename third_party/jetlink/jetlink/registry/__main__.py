"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

`python -m jetlink.registry`, the same thing as `jetlink-models`, for a machine
where the entry point was never installed.
"""
from __future__ import annotations

import sys

from jetlink.registry.cli import main

sys.exit(main())
