#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    echo "Waiting for internet..."

    timeout=0
    while [ $timeout -lt 120 ]; do
        if getent hosts pypi.org >/dev/null 2>&1; then
            break
        fi
        echo "Waiting for internet... (${timeout})"
        sleep 5
        timeout=$((timeout+5))

    done

    if python3 -c "import jeepney" > /dev/null 2>&1; then
      echo "jeepney already installed."
    else
      echo "jeepney installing to pydeps."
      python3 -m pip install --target "$PYDEPS" --upgrade jeepney
    fi

    AGNOS_PY="$DIR/openpilot/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/openpilot/system/hardware/tici/agnos.json"
    MODEL="$(tr -d '\000\r\n' 2>/dev/null < /sys/firmware/devicetree/base/model | tr '[:upper:]' '[:lower:]')"
    MODEL="${MODEL#comma }"
    if [ "$MODEL" = "c3" ] || [ "$MODEL" = "tici" ]; then
      MANIFEST="$DIR/openpilot/system/hardware/tici/agnos-tici.json"
    fi
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    echo "AGNOS_PY=${AGNOS_PY}"
    echo "MANIFEST=${MANIFEST}"
    echo "MODEL=${MODEL}"
    if ! python3 $DIR/openpilot/system/ui/updater.py $AGNOS_PY $MANIFEST; then
      echo "python updater failed, falling back to bundled updater"
      $DIR/openpilot/system/hardware/tici/updater $AGNOS_PY $MANIFEST
    fi
    echo "end updater $AGNOS_PY $MANIFEST"
  fi
}

function start_carrot_recovery {
  local recovery_script="$DIR/openpilot/selfdrive/carrot/recovery/server.py"
  local py_bin

  [ -f "$recovery_script" ] || return
  py_bin="$(command -v python3 || command -v python || true)"
  [ -n "$py_bin" ] || return

  if command -v pgrep >/dev/null 2>&1 && pgrep -f "openpilot/selfdrive/carrot/recovery/server.py" >/dev/null 2>&1; then
    return
  fi

  echo "Starting carrot recovery server on 6999."
  (cd "$DIR" && "$py_bin" "$recovery_script" --port 6999 >> /tmp/carrot_recovery.log 2>&1 &)
}

function start_carrot_web {
  export CARROT_WEB_EXTERNAL="${CARROT_WEB_EXTERNAL:-1}"
  [ "$CARROT_WEB_EXTERNAL" = "1" ] || return

  local watchdog_script="$DIR/scripts/carrot_web_watchdog.sh"
  local pid_file="${CARROT_WEB_PID_FILE:-/tmp/carrot_web_watchdog.pid}"
  local py_bin

  [ -f "$watchdog_script" ] || return

  if [ -f "$pid_file" ]; then
    local old_pid
    old_pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [ -n "$old_pid" ] && kill -0 "$old_pid" >/dev/null 2>&1; then
      return
    fi
  fi

  py_bin="$(command -v python3 || command -v python || true)"
  [ -n "$py_bin" ] || return

  echo "Starting external carrot web server on 7000."
  if command -v setsid >/dev/null 2>&1; then
    setsid bash "$watchdog_script" "$DIR" "$py_bin" >> /tmp/carrot_server.log 2>&1 &
  else
    bash "$watchdog_script" "$DIR" "$py_bin" >> /tmp/carrot_server.log 2>&1 &
  fi
}

function invalidate_modeld_build_if_needed {
  local stamp_path="$DIR/openpilot/selfdrive/modeld/models/.build_stamp"
  local tg_devices_path="$DIR/openpilot/selfdrive/modeld/models/tg_input_devices.json"
  local driving_pkl_path="$DIR/openpilot/selfdrive/modeld/models/driving_tinygrad.pkl"
  local old_stamp

  MODEL_BUILD_STAMP_VALUE="$(git rev-parse HEAD:openpilot/selfdrive/modeld HEAD:tinygrad_repo HEAD:openpilot/common/file_chunker.py 2>/dev/null | tr '\n' ':')"
  if [ -z "$MODEL_BUILD_STAMP_VALUE" ]; then
    MODEL_BUILD_STAMP_VALUE="$(git rev-parse HEAD 2>/dev/null || true)"
  fi

  old_stamp="$(cat "$stamp_path" 2>/dev/null || true)"
  if [ "$MODEL_BUILD_STAMP_VALUE" != "$old_stamp" ] || [ ! -f "$tg_devices_path" ] || { [ ! -f "$driving_pkl_path" ] && [ ! -f "$driving_pkl_path.chunkmanifest" ]; }; then
    echo "Model/tinygrad inputs changed, invalidating generated modeld artifacts."
    rm -f "$DIR"/openpilot/selfdrive/modeld/models/*_tinygrad.pkl*
    rm -f "$DIR"/openpilot/selfdrive/modeld/models/*_metadata.pkl
    rm -f "$DIR"/openpilot/selfdrive/modeld/models/tg_input_devices.json
    FORCE_REBUILD=1
  fi
}

function invalidate_native_build_if_needed {
  local missing=0
  local path

  for path in "$DIR/openpilot/system/loggerd/loggerd" "$DIR/openpilot/system/loggerd/encoderd" "$DIR/openpilot/system/camerad/camerad"; do
    if [ ! -x "$path" ]; then
      echo "Missing native binary: $path"
      missing=1
    fi
  done

  if [ "$missing" = "1" ]; then
    FORCE_REBUILD=1
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  PYDEPS="$DIR/pydeps"
  mkdir -p "$PYDEPS"
  export PYTHONPATH="$PYDEPS:$PWD${PYTHONPATH:+:$PYTHONPATH}"

  # Keep recovery access available even when an OS update or source build fails.
  if [ ! -f /data/params/d/GithubSshKeys ]; then
    echo -n openpilot > /data/params/d/GithubUsername
    cat /usr/comma/setup_keys > /data/params/d/GithubSshKeys
  fi
  if [ "$(cat /data/params/d/SshEnabled 2>/dev/null)" != "1" ]; then
    echo -n 1 > /data/params/d/SshEnabled
  fi
  start_carrot_recovery

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # AGNOS must be current before SCons loads its Python and native build
  # dependencies. Build Params before any long-running carrot service imports it.
  if ! bash "$DIR/scripts/ensure_params_build.sh"; then
    echo "Params registry build failed, not starting openpilot."
    while true; do sleep 1; done
  fi

  start_carrot_web


  FORCE_REBUILD=0
  invalidate_modeld_build_if_needed
  invalidate_native_build_if_needed

  rm openpilot/selfdrive/pandad/*.so
  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1500 > /tmp/launch_log
  if python -c "import flask" > /dev/null 2>&1; then
    echo "Flask already installed."
  else
    echo "Flask installing."
    pip install flask
  fi
  if python -c "import shapely" > /dev/null 2>&1; then
    echo "shapely already installed."
  else
    echo "shapely installing."
    pip install shapely
  fi
  if python -c "import kaitaistruct" > /dev/null 2>&1; then
    echo "kaitaistruct already installed."
  else
    echo "kaitaistruct installing."
    pip install kaitaistruct
  fi
  if python3 -c "import msgpack" > /dev/null 2>&1; then
    echo "msgpack already installed."
  else
    MSGPACK_WHEEL_DIR="$DIR/third_party/wheels"
    if ls "$MSGPACK_WHEEL_DIR"/msgpack-*.whl > /dev/null 2>&1; then
      echo "msgpack installing from local wheel to pydeps."
      python3 -m pip install --no-index --find-links "$MSGPACK_WHEEL_DIR" --target "$PYDEPS" --upgrade msgpack || python3 -m pip install --target "$PYDEPS" --upgrade msgpack
    else
      echo "msgpack local wheel missing, installing to pydeps from network."
      python3 -m pip install --target "$PYDEPS" --upgrade msgpack
    fi

    if python3 -c "import msgpack" > /dev/null 2>&1; then
      echo "msgpack installed for python3."
    else
      echo "msgpack install failed for python3."
    fi
  fi

  # start manager
  cd openpilot/system/manager
  if [ "$FORCE_REBUILD" = "1" ] || [ ! -f $DIR/prebuilt ]; then
    if ! ./build.py; then
      echo "openpilot build failed, not starting manager."
      while true; do sleep 1; done
    fi
    if [ "$FORCE_REBUILD" = "1" ]; then
      mkdir -p "$DIR/openpilot/selfdrive/modeld/models"
      echo -n "$MODEL_BUILD_STAMP_VALUE" > "$DIR/openpilot/selfdrive/modeld/models/.build_stamp"
    fi
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
