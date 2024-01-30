#!/usr/bin/env python3
import os
import sys
import errno
import shutil
import time
from glob import glob
import subprocess
import importlib.util
from urllib.request import urlopen


# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.spinner import Spinner

LIB_INSTALLED = os.path.exists('/data/pyextra/overpy')
OPSPLINE_SPEC = importlib.util.find_spec('scipy')
OVERPY_SPEC = importlib.util.find_spec('overpy')
MAX_BUILD_PROGRESS = 100
TMP_DIR = '/data/tmp'
PYEXTRA_DIR = '/data/pyextra'

def createFolder(directory):
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
    except OSError:
        print ('Error: Creating directory. ' +  directory)

def wait_for_internet_connection(return_on_failure=False):
  retries = 0
  while True:
    try:
      _ = urlopen('https://www.google.com/', timeout=10)
      return True
    except Exception as e:
      print(f'Wait for internet failed: {e}')
      if return_on_failure and retries == 15:
        return False
      retries += 1
      time.sleep(2)  # Wait for 2 seconds before retrying


def install_dep(spinner):
  wait_for_internet_connection()

  TOTAL_PIP_STEPS = 37

  try:
    os.makedirs(TMP_DIR)
  except OSError as e:
    if e.errno != errno.EEXIST:
      raise
  my_env = os.environ.copy()
  my_env['TMPDIR'] = TMP_DIR

  pip_target = [f'--target={PYEXTRA_DIR}']
  packages = []
  if OPSPLINE_SPEC is None:
    packages.append('scipy==1.11.1')
  if OVERPY_SPEC is None:
    packages.append('overpy==0.6')

  pip = subprocess.Popen([sys.executable, "-m", "pip", "install", "-v"] + pip_target + packages,
                          stdout=subprocess.PIPE, env=my_env)

  # Read progress from pip and update spinner
  steps = 0
  while True:
    output = pip.stdout.readline()
    if pip.poll() is not None:
      break
    if output:
      steps += 1
      spinner.update(f"Downloading... {round(min(100, MAX_BUILD_PROGRESS * (steps / TOTAL_PIP_STEPS)))}%")
      print(output.decode('utf8', 'replace'))

  shutil.rmtree(TMP_DIR)
  os.unsetenv('TMPDIR')

  # remove numpy installed to PYEXTRA_DIR since numpy is already present in the AGNOS image
  if OPSPLINE_SPEC is None:
    for directory in glob(f'{PYEXTRA_DIR}/numpy*'):
      shutil.rmtree(directory)
    if os.path.exists(f'{PYEXTRA_DIR}/bin'):
      shutil.rmtree(f'{PYEXTRA_DIR}/bin')


if __name__ == "__main__" and not LIB_INSTALLED:
  createFolder(PYEXTRA_DIR)
  spinner = Spinner()
  spinner.update_progress(0, 100)
  install_dep(spinner)