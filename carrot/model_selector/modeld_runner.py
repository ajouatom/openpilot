"""Dispatcher entrypoint registered in place of `selfdrive.modeld.modeld`.

Decides at startup whether to run the carrot model-selector's own 3-model
aware engine (`carrot_modeld`) or the upstream carrot-wip `modeld` unchanged:

* `/data/models` holds a valid custom model set  → `carrot_modeld.main()`
* otherwise                                       → upstream `modeld.main()`

There is no runtime patching of the upstream module — the two engines are
fully independent so upstream can evolve (2-model today, 3-model tomorrow)
without us having to re-merge.
"""
from __future__ import annotations

import argparse

from openpilot.common.swaglog import cloudlog

from .config import MODELS_DIR as CUSTOM_MODELS_DIR
from .validator import describe, is_valid_model_dir


def _use_custom_model() -> bool:
    if is_valid_model_dir(CUSTOM_MODELS_DIR):
        cloudlog.warning(
            f"model_selector: running carrot_modeld — {describe(CUSTOM_MODELS_DIR)}"
        )
        return True
    cloudlog.info(
        f"model_selector: running upstream modeld — {describe(CUSTOM_MODELS_DIR)}"
    )
    return False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true", help="A boolean for demo mode.")
    args, _ = parser.parse_known_args()

    if _use_custom_model():
        from openpilot.carrot.model_selector import carrot_modeld
        carrot_modeld.main(demo=args.demo)
    else:
        from openpilot.selfdrive.modeld import modeld as upstream_modeld
        upstream_modeld.main(demo=args.demo)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cloudlog.warning("got SIGINT")
