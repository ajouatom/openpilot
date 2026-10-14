"""Cooperative completion waits for the verified isolated USB runtime.

The pinned runtime calls USBIface.sleep after its spin window, but implements
it as a no-op. Keep device completion/error/timeout checks intact and yield the
CPU while the external GPU is working, without slowing modeld's dispatch.
"""
import time

RUNTIME_SHA = '90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2'
SPIN_MS = 1
SLEEP_SECONDS = .0002


def usb_sleep(self, timeout):
  time.sleep(SLEEP_SECONDS)


def configure_usb_wait(manifest):
  if manifest.get('runtime', {}).get('sha256') != RUNTIME_SHA:
    return False
  from tinygrad.runtime.ops_amd import AMDDevice, USBIface
  # This interpreter explicitly selects USB+AMD. Do not modify an installed
  # runtime file, the compiled graph, synchronization deadlines or model state.
  AMDDevice.sleep_timeout_ms = SPIN_MS
  USBIface.sleep = usb_sleep
  return True
