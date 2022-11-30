#!/usr/bin/bash

if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi

source "$BASEDIR/launch_env.sh"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

function two_init {

  # mount -o remount,rw /system
  if [ ! -f /ONEPLUS ] && ! $(grep -q "letv" /proc/cmdline); then
    mount -o remount,rw /system
    sed -i -e 's#/dev/input/event1#/dev/input/event2#g' ~/.bash_profile
    touch /ONEPLUS
    mount -o remount,r /system
  else
    if [ ! -f /LEECO ]; then
      mount -o remount,rw /system
      touch /LEECO
      mount -o remount,r /system 
    fi
  fi
  # mount -o remount,r /system  
  
  neos=`cat /VERSION`
  if [ -f /ONEPLUS ] && [ $neos != 20 ] ; then
    mount -o remount,rw /system
    echo -n 20 > /VERSION
    mount -o remount,r /system
  fi

  # set IO scheduler
  setprop sys.io.scheduler noop
  for f in /sys/block/*/queue/scheduler; do
    echo noop > $f
  done

  # android gets two cores
  echo 0-1 > /dev/cpuset/background/cpus
  echo 0-1 > /dev/cpuset/system-background/cpus
  echo 0-1 > /dev/cpuset/foreground/cpus
  echo 0-1 > /dev/cpuset/foreground/boost/cpus
  echo 0-1 > /dev/cpuset/android/cpus

  # openpilot gets all the cores
  echo 0-3 > /dev/cpuset/app/cpus

  # mask off 2-3 from RPS and XPS - Receive/Transmit Packet Steering
  echo 3 | tee  /sys/class/net/*/queues/*/rps_cpus
  echo 3 | tee  /sys/class/net/*/queues/*/xps_cpus

  # *** set up governors ***

  # +50mW offroad, +500mW onroad for 30% more RAM bandwidth
#  echo "performance" > /sys/class/devfreq/soc:qcom,cpubw/governor
  echo 1056000 > /sys/class/devfreq/soc:qcom,m4m/max_freq

  # *** set up IRQ affinities ***

  # Collect RIL and other possibly long-running I/O interrupts onto CPU 1
  echo 1 > /proc/irq/78/smp_affinity_list # qcom,smd-modem (LTE radio)
  echo 1 > /proc/irq/33/smp_affinity_list # ufshcd (flash storage)
  echo 1 > /proc/irq/35/smp_affinity_list # wifi (wlan_pci)
  echo 1 > /proc/irq/6/smp_affinity_list  # MDSS

  # USB traffic needs realtime handling on cpu 3
  [ -d "/proc/irq/733" ] && echo 3 > /proc/irq/733/smp_affinity_list

  # GPU and camera get cpu 2
  #CAM_IRQS="177 178 179 180 181 182 183 184 185 186 192"
  CAM_IRQS="177 178 179 180 181 182 183 192"
  for irq in $CAM_IRQS; do
    echo 2 > /proc/irq/$irq/smp_affinity_list
  done
  echo 2 > /proc/irq/193/smp_affinity_list # GPU

  # give GPU threads RT priority
  for pid in $(pgrep "kgsl"); do
    chrt -f -p 52 $pid
  done

  # the flippening!
  #LD_LIBRARY_PATH="" content insert --uri content://settings/system --bind name:s:user_rotation --bind value:i:1

  # disable bluetooth
  service call bluetooth_manager 8

  # wifi scan
  wpa_cli IFNAME=wlan0 SCAN

  # install missing libs
  LIB_PATH="/data/openpilot/selfdrive/hardware/eon/libs"
  PY_LIB_DEST="/system/comma/usr/lib/python3.8/site-packages"
  mount -o remount,rw /system
  # mapd
  if [ ! -f "/system/comma/usr/lib/libgfortran.so.5.0.0" ]; then
    echo "Installing libgfortran..."
    tar -zxvf "$LIB_PATH/libgfortran.tar.gz" -C /system/comma/usr/lib/
  fi
  # mapd
  MODULE="opspline"
  if [ ! -d "$PY_LIB_DEST/$MODULE" ]; then
    echo "Installing $MODULE..."
    tar -zxvf "$LIB_PATH/$MODULE.tar.gz" -C "$PY_LIB_DEST/"
  fi
  MODULE="overpy"
  if [ ! -d "$PY_LIB_DEST/$MODULE" ]; then
    echo "Installing $MODULE..."
    tar -zxvf "$LIB_PATH/$MODULE.tar.gz" -C "$PY_LIB_DEST/"
  fi
  # laika
  MODULE="hatanaka"
  if [ ! -d "$PY_LIB_DEST/$MODULE" ]; then
    echo "Installing $MODULE..."
    tar -zxvf "$LIB_PATH/$MODULE.tar.gz" -C "$PY_LIB_DEST/"
  fi
  if [ ! -f "$PY_LIB_DEST/ncompress.cpython-38.so" ]; then
    echo "Installing ncompress.cpython-38.so..."
    cp -f "$LIB_PATH/ncompress.cpython-38.so" "$PY_LIB_DEST/"
  fi
  MODULE="importlib_resources"
  if [ ! -d "$PY_LIB_DEST/$MODULE" ]; then
    echo "Installing $MODULE..."
    tar -zxvf "$LIB_PATH/$MODULE.tar.gz" -C "$PY_LIB_DEST/"
  fi
  if [ ! -f "$PY_LIB_DEST/zipp.py" ]; then
    echo "Installing zipp.py..."
    cp -f "$LIB_PATH/zipp.py" "$PY_LIB_DEST/"
  fi
  # updated
  MODULE="markdown_it"
  if [ ! -d "$PY_LIB_DEST/$MODULE" ]; then
    echo "Installing $MODULE..."
    tar -zxvf "$LIB_PATH/$MODULE.tar.gz" -C "$PY_LIB_DEST/"
  fi
  MODULE="mdurl"
  if [ ! -d "$PY_LIB_DEST/$MODULE" ]; then
    echo "Installing $MODULE..."
    tar -zxvf "$LIB_PATH/$MODULE.tar.gz" -C "$PY_LIB_DEST/"
  fi
  mount -o remount,r /system


  # One-time fix for a subset of OP3T with gyro orientation offsets.
  # Remove and regenerate qcom sensor registry. Only done on OP3T mainboards.
  # Performed exactly once. The old registry is preserved just-in-case, and
  # doubles as a flag denoting we've already done the reset.
  if [ -f /ONEPLUS ] && [ ! -f "/persist/comma/op3t-sns-reg-backup" ]; then
    echo "Performing OP3T sensor registry reset"
    mv /persist/sensors/sns.reg /persist/comma/op3t-sns-reg-backup &&
      rm -f /persist/sensors/sensors_settings /persist/sensors/error_log /persist/sensors/gyro_sensitity_cal &&
      echo "restart" > /sys/kernel/debug/msm_subsys/slpi &&
      sleep 5  # Give Android sensor subsystem a moment to recover
  fi

  if [ ! -f "/data/ntune/common.json" ]; then
    mkdir /data/ntune
  fi

  if [ ! -f "/data/ntune/lat_torque_v4.json" ]; then
    cp /data/openpilot/ntune/lat_torque_v4.json /data/ntune/lat_torque_v4.json
  fi

  if [ ! -f "/data/ntune/common.json" ]; then
    cp /data/openpilot/ntune/common.json /data/ntune/common.json
  fi

  if [ ! -f "/data/ntune/scc.json" ]; then
    cp /data/openpilot/ntune/scc.json /data/ntune/scc.json
  fi

  if [ ! -f "/data/ntune/option.json" ]; then
    cp /data/openpilot/ntune/option.json /data/ntune/option.json
  fi

  if [ ! -f "/data/ntune/lat_lqr.json" ]; then
    cp /data/openpilot/ntune/lat_lqr.json /data/ntune/lat_lqr.json
  fi

}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Pull time from panda
  $DIR/selfdrive/boardd/set_time.py

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD:$PWD/pyextra"

  two_init

  # write tmux scrollback to a file
  #tmux capture-pane -pq -S-1000 > /tmp/launch_log

  if [ -f "$BASEDIR/prebuilt" ]; then
    python /data/openpilot/common/spinner.py &
  fi

  # start manager
  cd selfdrive/manager
  ./build.py && ./manager.py
}

launch
