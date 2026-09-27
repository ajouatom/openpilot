#!/usr/bin/env python3
"""Jetson devkit Xorg: expose a virtual monitor for the existing GLFW renderer."""
from pathlib import Path
import shutil

path = Path('/etc/X11/xorg.conf')
text = path.read_text()
marker = '    # Carrot Jetlink USB display (no physical DisplayPort monitor)\n'
if marker not in text:
  anchor = '    Option      "AllowEmptyInitialConfiguration" "true"'
  if 'Identifier  "Tegra0"' not in text or anchor not in text:
    raise SystemExit('Unrecognized Xorg configuration; automatic editing refused')
  backup = path.with_name('xorg.conf.before-carrot-jetlink')
  if not backup.exists():
    shutil.copy2(path, backup)
  text = text.replace(anchor, anchor + '\n' + marker +
                      '    Option "ConnectedMonitor" "DFP-0"\n'
                      '    Option "UseEDID" "false"\n')
  path.write_text(text)
