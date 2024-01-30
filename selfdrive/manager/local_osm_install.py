#!/usr/bin/env python3
import os
import subprocess
import shutil

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.mapd.lib.helpers import is_local_osm_installed, timestamp_local_osm_db, is_osm_db_up_to_date
from openpilot.selfdrive.manager.custom_dep import wait_for_internet_connection


MAX_BUILD_PROGRESS = 100
INSTALL_OSM_STEPS = 1021
INSTALL_DB_STEPS = 58


def install_local_osm(spinner, steps=0, total_steps=INSTALL_OSM_STEPS + INSTALL_DB_STEPS):
  osm_install = subprocess.Popen(['sh', './install_osm.sh'], cwd=os.path.join(BASEDIR, 'installer/custom/'),
                                 stdout=subprocess.PIPE)
  # Read progress from install script and update spinner
  while True:
    output = osm_install.stdout.readline()
    if osm_install.poll() is not None:
      break
    if output:
      steps += 1
      spinner.update_progress(MAX_BUILD_PROGRESS * min(1., steps / total_steps), 100.)
      print(output.decode('utf8', 'replace'))


def install_osm_db(spinner, steps=0, total_steps=INSTALL_DB_STEPS):
  # Fetch and get db
  fetch_osm_db = subprocess.Popen(['sh', './install_osm_db.sh'], cwd=os.path.join(BASEDIR, 'installer/custom/'),
                                  stdout=subprocess.PIPE)
  # Read progress from install script and update spinner
  while True:
    output = fetch_osm_db.stdout.readline()
    if fetch_osm_db.poll() is not None:
      break
    if output:
      steps += 1
      spinner.update_progress(MAX_BUILD_PROGRESS * min(1., steps / total_steps), 100.)
      print(output.decode('utf8', 'replace'))


if __name__ == "__main__":
  if wait_for_internet_connection(return_on_failure=True):
    is_osm_installed = is_local_osm_installed()
    is_db_updated = is_osm_db_up_to_date()
    print(f'Local OSM Installer:\nOSM currently installed: {is_osm_installed}\nDB up to date: {is_db_updated}')

    if is_osm_installed and is_db_updated:
      pass
    else:
      spinner = Spinner()
      spinner.update_progress(0, 100)

      if is_osm_installed:
        install_osm_db(spinner)
      else:
        install_local_osm(spinner)
        install_osm_db(spinner, steps=INSTALL_OSM_STEPS, total_steps=INSTALL_OSM_STEPS + INSTALL_DB_STEPS)

      timestamp_local_osm_db()

      if not is_local_osm_installed():
        print("Local OSM was not setup succesfully. Cleaning up.")
        shutil.rmtree("/data/osm")
