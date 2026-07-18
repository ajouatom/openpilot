from __future__ import annotations

import logging

from carrotbot.bot import run_bot
from carrotbot.config import Config


def main() -> None:
  logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
  )
  run_bot(Config.from_env())


if __name__ == "__main__":
  main()
