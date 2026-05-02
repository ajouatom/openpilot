#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
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

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

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

  # events language init
  #LANG=$(cat ${PARAMS_ROOT}/d/LanguageSetting)
  LANG=$(cat /data/params/d/LanguageSetting)
  GITSTAT=$(git status)

  # events.py 한글로 변경 및 파일이 교체된 상태인지 확인
  if [ "${LANG}" = "ko" ] && [[ ! "${GITSTAT}" == *"modified:   selfdrive/selfdrived/events.py"* ]]; then
    cp -f $DIR/selfdrive/selfdrived/events.py $DIR/scripts/add/events_en.py
    cp -f $DIR/scripts/add/events_ko.py $DIR/selfdrive/selfdrived/events.py
  elif [ "${LANG}" = "zh-CHS" ] && [[ ! "${GITSTAT}" == *"modified:   selfdrive/selfdrived/events.py"* ]]; then
    # Backup current events.py (assumed English) and install Simplified Chinese events
    cp -f $DIR/selfdrive/selfdrived/events.py $DIR/scripts/add/events_en.py
    cp -f $DIR/scripts/add/events_zh.py $DIR/selfdrive/selfdrived/events.py
  elif [ "${LANG}" = "en" ] && [[ "${GITSTAT}" == *"modified:   selfdrive/selfdrived/events.py"* ]]; then
    cp -f $DIR/scripts/add/events_en.py $DIR/selfdrive/selfdrived/events.py
  fi

  # c3xl amplifier file change
  C3XL=$(cat /data/params/d/HardwareC3xLite)

  if [ "${C3XL}" = "1" ] && [[ ! "${EVENTSTAT}" == *"modified:   system/hardware/tici/amplifier.py"* ]]; then
    cp -f $DIR/system/hardware/tici/amplifier.py $DIR/scripts/add/amplifier_org.py
    cp -f $DIR/scripts/add/amplifier_c3xl.py $DIR/system/hardware/tici/amplifier.py
  elif [ "${C3XL}" = "0" ] && [[ "${EVENTSTAT}" == *"modified:   system/hardware/tici/amplifier.py"* ]]; then
    cp -f $DIR/scripts/add/amplifier_org.py $DIR/system/hardware/tici/amplifier.py
  fi

  # openpilot default ssh key installer
  if [ ! -f /data/params/d/GithubSshKeys ]; then
    echo -n openpilot > /data/params/d/GithubUsername
    cat /usr/comma/setup_keys > /data/params/d/GithubSshKeys
  fi

  # always ssh enable
  if [ "$(cat /data/params/d/SshEnabled 2>/dev/null)" = "0" ]; then
    echo -n 1 > /data/params/d/SshEnabled
  fi

  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
