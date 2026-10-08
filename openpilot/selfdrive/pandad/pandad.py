#!/usr/bin/env python3
# simple pandad wrapper that updates the panda first
import os
import usb1
import time
import signal
import subprocess

from panda import Panda, PandaDFU, PandaProtocolMismatch, FW_PATH
from openpilot.cereal import car
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.pandad.panda_helpers import connect_all_pandas, pandas_include_internal

TESLA_WAKE_CAR_FINGERPRINTS = {"TESLA_MODEL_3", "TESLA_MODEL_Y"}

def tesla_wake_on_can_enabled(params: Params) -> bool:
  if not params.get_bool("TeslaWakeOnCAN"):
    return False

  try:
    selected_car = params.get("CarSelected3")
    if isinstance(selected_car, bytes):
      selected_car = selected_car.decode("utf-8", errors="strict")

    car_params = params.get("CarParamsPersistent") or params.get("CarParams")
    if car_params is None:
      return selected_car in TESLA_WAKE_CAR_FINGERPRINTS
    with car.CarParams.from_bytes(car_params) as CP:
      return CP.brand == "tesla" and CP.carFingerprint in TESLA_WAKE_CAR_FINGERPRINTS
  except (UnicodeDecodeError, ValueError, TypeError):
    cloudlog.exception("Invalid CarParams while selecting Tesla wake firmware")
    return False


def get_firmware_path(panda: Panda, params: Params) -> str:
  app_fn = panda.get_mcu_type().config.app_fn
  if not tesla_wake_on_can_enabled(params):
    return os.path.join(FW_PATH, app_fn)

  wake_fn = app_fn.removesuffix(".bin.signed") + "_tesla_wake.bin.signed"
  fn = os.path.join(FW_PATH, wake_fn)
  if not os.path.isfile(fn):
    raise FileNotFoundError(f"Tesla wake firmware is missing: {fn}")
  return fn


def configure_boardd_firmware_check(params: Params) -> None:
  # Python has already verified the exact selected image. The C++ check only
  # knows the stock filenames, so it must not reject the Tesla variant.
  if tesla_wake_on_can_enabled(params):
    os.environ["BOARDD_SKIP_FW_CHECK"] = "1"
  else:
    os.environ.pop("BOARDD_SKIP_FW_CHECK", None)


def get_expected_signature(panda: Panda, params: Params) -> bytes:
  fn = get_firmware_path(panda, params)
  return Panda.get_signature_from_firmware(fn)

def flash_panda(panda_serial: str, params: Params) -> Panda:
  try:
    panda = Panda(panda_serial)
  except PandaProtocolMismatch:
    cloudlog.warning("detected protocol mismatch, reflashing panda")
    HARDWARE.recover_internal_panda()
    raise

  try:
    fw_path = get_firmware_path(panda, params)
    fw_signature = get_expected_signature(panda, params)
    internal_panda = panda.is_internal()

    panda_version = "bootstub" if panda.bootstub else panda.get_version()
    panda_signature = b"" if panda.bootstub else panda.get_signature()
    cloudlog.warning(f"Panda {panda_serial} connected, version: {panda_version}, signature {panda_signature.hex()[:16]}, expected {fw_signature.hex()[:16]}")

    if panda.bootstub or panda_signature != fw_signature:
      cloudlog.info("Panda firmware out of date, update required")
      panda.flash(fn=fw_path)
      cloudlog.info("Done flashing")

    if panda.bootstub:
      bootstub_version = panda.get_version()
      cloudlog.info(f"Flashed firmware not booting, flashing development bootloader. {bootstub_version=}, {internal_panda=}")
      if internal_panda:
        HARDWARE.recover_internal_panda()
      panda.recover(reset=(not internal_panda))
      cloudlog.info("Done flashing bootstub")

    if panda.bootstub:
      cloudlog.info("Panda still not booting, exiting")
      raise AssertionError

    panda_signature = panda.get_signature()
    if panda_signature != fw_signature:
      cloudlog.info("Version mismatch after flashing, exiting")
      raise AssertionError

    return panda
  except Exception:
    panda.close()
    raise


def flash_all_pandas(panda_serials: list[str], params: Params) -> list[Panda]:
  return connect_all_pandas(panda_serials, lambda serial: flash_panda(serial, params))


def main() -> None:
  # signal pandad to close the relay and exit
  def signal_handler(signum, frame):
    cloudlog.info(f"Caught signal {signum}, exiting")
    nonlocal do_exit
    do_exit = True
    if process is not None:
      process.send_signal(signal.SIGINT)

  process = None
  do_exit = False
  signal.signal(signal.SIGINT, signal_handler)

  count = 0
  first_run = True
  params = Params()
  no_internal_panda_count = 0

  while not do_exit:
    pandas: list[Panda] = []
    try:
      count += 1
      cloudlog.event("pandad.flash_and_connect", count=count)
      params.remove("PandaSignatures")

      # Handle missing internal panda
      if no_internal_panda_count > 0:
        if no_internal_panda_count == 3:
          cloudlog.info("No pandas found, putting internal panda into DFU")
          HARDWARE.recover_internal_panda()
        else:
          cloudlog.info("No pandas found, resetting internal panda")
          HARDWARE.reset_internal_panda()
        time.sleep(3)  # wait to come back up

      # Flash all Pandas in DFU mode
      dfu_serials = PandaDFU.list()
      if len(dfu_serials) > 0:
        for serial in dfu_serials:
          cloudlog.info(f"Panda in DFU mode found, flashing recovery {serial}")
          PandaDFU(serial).recover()
        time.sleep(1)

      panda_serials = Panda.list()
      if len(panda_serials) == 0:
        no_internal_panda_count += 1
        continue

      cloudlog.info(f"{len(panda_serials)} panda(s) found, connecting - {panda_serials}")

      # Flash every panda. C3 uses an internal DOS plus a USB red panda, while
      # C3X/C4 normally have a single internal panda.
      pandas = flash_all_pandas(panda_serials, params)

      # Ensure internal panda is present if expected
      if HARDWARE.has_internal_panda() and not pandas_include_internal(pandas):
        cloudlog.error("Internal panda is missing, trying again")
        no_internal_panda_count += 1
        continue
      no_internal_panda_count = 0

      panda_serials = [panda.get_usb_serial() for panda in pandas]

      # log panda fw versions
      params.put("PandaSignatures", b','.join(panda.get_signature() for panda in pandas))

      # check health for lost heartbeat
      for panda in pandas:
        health = panda.health()
        if health["heartbeat_lost"]:
          params.put_bool("PandaHeartbeatLost", True)
          cloudlog.event("heartbeat lost", deviceState=health, serial=panda.get_usb_serial())
        if health["som_reset_triggered"]:
          params.put_bool("PandaSomResetTriggered", True)
          cloudlog.event("panda.som_reset_triggered", health=health, serial=panda.get_usb_serial())

        if first_run:
          # reset pandas to ensure they're in a good state
          cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
          panda.reset(reconnect=True)
    # TODO: wrap all panda exceptions in a base panda exception
    except (usb1.USBErrorNoDevice, usb1.USBErrorPipe):
      # a panda was disconnected while setting everything up. let's try again
      cloudlog.exception("Panda USB exception while setting up")
      continue
    except PandaProtocolMismatch:
      cloudlog.exception("pandad.protocol_mismatch")
      continue
    except Exception:
      cloudlog.exception("pandad.uncaught_exception")
      continue
    finally:
      for panda in pandas:
        panda.close()

    first_run = False

    # run pandad with all connected serials as arguments
    configure_boardd_firmware_check(params)
    os.environ['MANAGER_DAEMON'] = 'pandad'
    process = subprocess.Popen(["./pandad", *panda_serials], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"))
    process.wait()


if __name__ == "__main__":
  main()
