import os
from collections.abc import Callable
from typing import Protocol, TypeVar


PANDA_SPI_SPEED_ENV = "PANDA_SPI_SPEED_HZ"
PANDA_SPI_SPEEDS_HZ = (50_000_000, 40_000_000, 30_000_000, 25_000_000)


class PandaLike(Protocol):
  def get_usb_serial(self) -> str: ...
  def get_type(self) -> bytes: ...
  def is_internal(self) -> bool: ...
  def close(self) -> None: ...


PandaT = TypeVar("PandaT", bound=PandaLike)


def get_panda_spi_speed_hz(setting: int) -> int:
  if 0 <= setting < len(PANDA_SPI_SPEEDS_HZ):
    return PANDA_SPI_SPEEDS_HZ[setting]
  return PANDA_SPI_SPEEDS_HZ[0]


def configure_panda_spi_speed(setting: int) -> int:
  speed_hz = get_panda_spi_speed_hz(setting)
  os.environ[PANDA_SPI_SPEED_ENV] = str(speed_hz)
  return speed_hz


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
