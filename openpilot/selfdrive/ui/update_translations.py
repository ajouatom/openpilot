#!/usr/bin/env python3
from itertools import chain
import os
from openpilot.common.basedir import BASEDIR
from openpilot.system.ui.lib.multilang import SYSTEM_UI_DIR, UI_DIR, TRANSLATIONS_DIR, multilang
from openpilot.selfdrive.ui.translations.potools import extract_strings, generate_pot, merge_po, init_po

LANGUAGES_FILE = os.path.join(str(TRANSLATIONS_DIR), "languages.json")
POT_FILE = os.path.join(str(TRANSLATIONS_DIR), "app.pot")
ALERTS_FILE = "openpilot/selfdrive/selfdrived/events.py"

# Alert text is translated when Events.create_alerts() returns an alert. These
# constructor arguments are no-op source markers for the PO extractor.
ALERT_TRANSLATION_CALL_ARGS = {
  "Alert": ((0, 1), ("alert_text_1", "alert_text_2")),
  "NoEntryAlert": ((0,), ("alert_text_1", "alert_text_2")),
  "SoftDisableAlert": ((0,), ()),
  "UserSoftDisableAlert": ((0,), ()),
  "ImmediateDisableAlert": ((0,), ()),
  "NormalPermanentAlert": ((0, 1), ("alert_text_1", "alert_text_2")),
  "StartupAlert": ((0, 1), ("alert_text_1", "alert_text_2")),
  "soft_disable_alert": ((0,), ()),
  "user_soft_disable_alert": ((0,), ()),
}


def update_translations():
  files = []
  for root, _, filenames in chain(os.walk(SYSTEM_UI_DIR),
                                  os.walk(os.path.join(UI_DIR, "widgets")),
                                  os.walk(os.path.join(UI_DIR, "layouts")),
                                  os.walk(os.path.join(UI_DIR, "onroad"))):
    for filename in filenames:
      if filename.endswith(".py"):
        files.append(os.path.relpath(os.path.join(root, filename), BASEDIR))
  # Extract translatable strings and generate .pot template
  entries_by_msgid = {entry.msgid: entry for entry in extract_strings(files, BASEDIR)}
  for entry in extract_strings([ALERTS_FILE], BASEDIR, ALERT_TRANSLATION_CALL_ARGS):
    if entry.msgid in entries_by_msgid:
      current = entries_by_msgid[entry.msgid]
      current.source_refs = sorted(set(current.source_refs + entry.source_refs))
    else:
      entries_by_msgid[entry.msgid] = entry
  entries = list(entries_by_msgid.values())
  generate_pot(entries, POT_FILE)

  # Generate/update translation files for each language
  for name in multilang.languages.values():
    po_file = os.path.join(TRANSLATIONS_DIR, f"app_{name}.po")
    if os.path.exists(po_file):
      merge_po(po_file, POT_FILE)
    else:
      init_po(POT_FILE, po_file, name)


if __name__ == "__main__":
  update_translations()
