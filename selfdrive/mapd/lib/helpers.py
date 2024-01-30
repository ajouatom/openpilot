import os
import sys
import subprocess
import requests
import email.utils as eut
import time

from openpilot.common.basedir import PYEXTRADIR
sys.path.append(os.path.join(PYEXTRADIR, "pyextra"))

import overpy


S3_LOCAL_OSM_URL = "https://kisapilot.kro.kr/osm/db/kor/db.tar.gz"
OSM_DB_STAMP_FILE = "/data/osm/db_stamp"


def get_current_s3_osm_db_timestamp():
  r = requests.head(S3_LOCAL_OSM_URL)
  if r.status_code != 200:
    print(f'Failed to fetch HEAD for S3 OSM db.\n\n{r.status_code}')
    return None

  timestamp_string = r.headers.get('Last-Modified', None)
  if timestamp_string is None:
    print(f'HEAD for S3 OSM db contained no "Last-Modified" value.\n\n{r.headers}')
    return None

  try:
    parsed_date = eut.parsedate(timestamp_string)
    return time.mktime(parsed_date)
  except Exception as e:
    print(f'Could not parse last modified timestamp for S3 local osm db.\n\n{e}')
    return None


def persist_s3_osm_db_timestamp(timestamp):
  try:
    with open(OSM_DB_STAMP_FILE, 'w') as file:
      file.write(f'{timestamp}')
  except Exception as e:
    print(f'Failed to timestamp local OSM db.\n\n{e}')


def get_local_osm_timestamp():
  try:
    with open(OSM_DB_STAMP_FILE, 'r') as file:
      return float(file.readline())
  except Exception as e:
    print(f'Failed to read timestamp for local OSM db.\n\n{e}')
    return None


def is_osm_db_up_to_date():
  current_osm_ts = get_local_osm_timestamp()
  if current_osm_ts is None:
    return False

  current_s3_osm_ts = get_current_s3_osm_db_timestamp()
  if current_s3_osm_ts is None:
    return True

  return current_osm_ts == current_s3_osm_ts


def timestamp_local_osm_db():
  current_s3_osm_ts = get_current_s3_osm_db_timestamp()
  if current_s3_osm_ts is not None:
    persist_s3_osm_db_timestamp(current_s3_osm_ts)


def is_local_osm_installed():
  api = overpy.Overpass()
  q = """
      way(616074250);
      (._;>;);
      out;
      """

  try:
    if not os.path.isfile('/data/osm/v0.7.56/bin/osm3s_query'):
      return False
    else:
      completion = subprocess.run(["/data/osm/v0.7.56/bin/osm3s_query", "--db-dir=/data/osm/db", f'--request={q}'],
                                  check=True, capture_output=True)
      print(f'OSM local query returned with exit code: {completion.returncode}')

    if completion.returncode != 0:
      return False

    print(f'OSM Local query returned:\n\n{completion.stdout}')

    ways = api.parse_xml(completion.stdout).ways
    success = len(ways) == 1
    print(f"Test osm script returned {len(ways)} ways")
    print(f'OSM local server query {"succeded" if success else "failed"}')

    return success

  except Exception as e:
    print(e)
    return False
