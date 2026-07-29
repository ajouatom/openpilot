from collections.abc import Callable
from typing import Protocol, TypeVar


class PandaLike(Protocol):
  def get_usb_serial(self) -> str: ...
  def get_type(self) -> bytes: ...
  def is_internal(self) -> bool: ...
  def close(self) -> None: ...


PandaT = TypeVar("PandaT", bound=PandaLike)


def connect_all_pandas(panda_serials: list[str], connector: Callable[[str], PandaT]) -> list[PandaT]:
  pandas: list[PandaT] = []
  try:
    for serial in panda_serials:
      pandas.append(connector(serial))

    # Keep the peripheral/internal panda first and assign deterministic bus ranges
    # to any additional pandas.
    pandas.sort(key=lambda p: (not p.is_internal(), p.get_type(), p.get_usb_serial()))
    return pandas
  except Exception:
    for panda in pandas:
      panda.close()
    raise


def pandas_include_internal(pandas: list[PandaT]) -> bool:
  return any(panda.is_internal() for panda in pandas)
