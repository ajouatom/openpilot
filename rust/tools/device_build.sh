#!/usr/bin/env bash
# Follows carrot's system/manager/build.py: show the spinner, compile in place,
# retry with fewer jobs when memory runs out, show an error window on failure.
# Everything needed is received with the Git branch, so no network is used here.
set -uo pipefail

root=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)
cd "$root" || exit 1
head=$(git rev-parse HEAD) || exit 1
stamp=rust/.device-built
if [[ -x rust/bin/openpilot-launch && -f $stamp && $(<"$stamp") == "$head" ]]; then
  exit 0
fi

# Earlier builds hid the unpacked models with skip-worktree. Git then refused
# to switch to any branch where those files differ, such as the original
# carrot branch ("not uptodate. Cannot merge"), so the device stayed here.
hidden=$(git ls-files -v | sed -n 's/^S //p')
if [[ -n $hidden ]]; then
  printf '%s\n' "$hidden" | xargs -d '\n' git update-index --no-skip-worktree --
  git update-index -q --refresh || true
fi

store=${OPENPILOT_BUILD_STORE:-/data/openpilot-build}
mkdir -p "$store" || exit 1
log=$store/build.log
: > "$log"
started=$SECONDS
export LD_LIBRARY_PATH="$root/rust/bin/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

ui=
feeder=
recovery=
# The spinner reads its text only between frames, after it has the display.
# Writing to it directly stopped the whole build where the display never came
# up or stalled (C3X lite: comma logo, idle CPUs, 6999 answering): the pipe
# filled and cargo's output with it. Lines go to a file and tail feeds them,
# so only tail waits on the spinner.
status_file=$store/spinner.status
stop_ui() {
  [[ -n $feeder ]] && kill "$feeder" 2>/dev/null
  [[ -n $ui ]] && kill "$ui" 2>/dev/null
  [[ -n $recovery ]] && kill "$recovery" 2>/dev/null
  ui=
  feeder=
  recovery=
}
start_ui() {
  [[ -z $ui && -x rust/bin/openpilot-spinner ]] || return 0
  local fifo=$store/spinner.fifo
  rm -f "$fifo" && mkfifo "$fifo" || return 0
  : > "$status_file"
  rust/bin/openpilot-spinner --source-root "$root" < "$fifo" >> "$log" 2>&1 &
  ui=$!
  tail -n +1 -F "$status_file" 2>/dev/null > "$fifo" &
  feeder=$!
  if [[ -x rust/bin/openpilot-carrot-recovery ]]; then
    rust/bin/openpilot-carrot-recovery --port 6999 >> "$log" 2>&1 &
    recovery=$!
  fi
}
status() {
  echo "[$((SECONDS - started))s] $1" >> "$log"
  [[ -n $ui ]] && echo "$1" >> "$status_file"
}
fail() {
  status "$1"
  [[ -n $feeder ]] && kill "$feeder" 2>/dev/null
  [[ -n $ui ]] && kill "$ui" 2>/dev/null
  ui=
  feeder=
  if [[ -x rust/bin/openpilot-text-window ]]; then
    rust/bin/openpilot-text-window --source-root "$root" \
      --text "openpilot failed to build

$1

Log: $log
Recovery: port 6999

$(tail -n 40 "$log")"
  fi
  stop_ui
  exit 1
}
trap stop_ui EXIT

set_cpu_limit() {
  # Compile at the full clock. Normal driving keeps its 1689 MHz cap.
  [[ -f /AGNOS ]] || return 0
  local policy limit
  for policy in 0 4; do
    policy=/sys/devices/system/cpu/cpufreq/policy$policy
    limit=$1
    [[ $limit == max ]] && limit=$(<"$policy/cpuinfo_max_freq")
    echo "$limit" | sudo -n tee "$policy/scaling_max_freq" > /dev/null
  done
}

start_ui
for core in 4 5 6 7; do
  [[ -f /AGNOS ]] && echo 1 | sudo -n tee "/sys/devices/system/cpu/cpu$core/online" > /dev/null
done
set_cpu_limit max
trap 'set_cpu_limit 1689600; stop_ui' EXIT

unpack() {  # bundle destination
  local name=$1 destination=$2
  mkdir -p "$destination" || return 1
  cat device-inputs/"$name".tar.zst.* | zstd -dc -T0 | tar -x -C "$destination"
}
bundle_id() { grep " $1\.tar\.zst\." device-inputs/MANIFEST | sha256sum | cut -d' ' -f1; }
# The path stays the same across SDK updates so cargo reuses what did not change.
sdk=$store/sdk
sdk_id=$(bundle_id sdk)
if [[ ! -f $sdk/.id || $(<"$sdk/.id") != "$sdk_id" ]]; then
  status 'Preparing build tools'
  rm -rf "$sdk"
  unpack sdk "$sdk" || fail 'Could not unpack build tools'
  echo "$sdk_id" > "$sdk/.id"
fi
assets_id=$(bundle_id assets)
assets_stamp=rust/bin/.assets
if [[ -L rust/bin || ! -f $assets_stamp || $(<"$assets_stamp") != "$assets_id" ]]; then
  status 'Preparing runtime files'
  if [[ -L rust/bin ]]; then
    rm rust/bin || fail 'Could not replace the packaged runtime link'
  fi
  unpack assets . || fail 'Could not unpack runtime files'
  echo "$assets_id" > "$assets_stamp"
  # The unpacked models equal their LFS pointers through the clean filter;
  # refreshing the index is enough for Git to see the checkout as unchanged.
  git update-index -q --refresh || true
  start_ui
fi

export PATH="$sdk/rust/bin:$sdk/capnp/usr/bin:$PATH"
export CARGO_HOME=$store/cargo-home CARGO_TARGET_DIR=$store/target CARGO_INCREMENTAL=0
export CARGO_PROFILE_RELEASE_LTO=false CARGO_PROFILE_RELEASE_CODEGEN_UNITS=16 CARGO_PROFILE_RELEASE_DEBUG=0
export OPENPILOT_BUILD_STORE=$store
export CC=/usr/bin/gcc CXX=$root/rust/tools/device_cxx.sh AR=/usr/bin/ar RANLIB=/usr/bin/ranlib
# Isolated CPUs (isolcpus) take no compile unless one is pinned there.
pinned=
if [[ -f /AGNOS ]]; then
  pinned=$(python3 -I -c '
cores = []
try:
  text = open("/sys/devices/system/cpu/isolated").read().strip()
  isolated = {c for part in text.split(",") if part
              for c in range(int(part.split("-")[0]), int(part.split("-")[-1]) + 1)}
  for core in (6, 7):
    if core in isolated and open(f"/sys/devices/system/cpu/cpu{core}/online").read().strip() == "1":
      cores.append(str(core))
except (OSError, ValueError):
  pass
print(" ".join(cores))
')
fi
if [[ -n $pinned ]]; then
  export DEVICE_BUILD_PINNED_CORES=$pinned RUSTC_WRAPPER=$root/rust/tools/device_rustc.sh
  status "Using isolated CPUs $pinned for compiles"
fi
# Collect more often; generated code does not change.
export CXXFLAGS='--param ggc-min-expand=20 --param ggc-min-heapsize=32768'
export HOST_CC=$CC HOST_CXX=$CXX HOST_AR=$AR HOST_RANLIB=$RANLIB
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=/usr/bin/aarch64-linux-gnu-gcc-13
export LIBCLANG_PATH=/usr/lib/llvm-18/lib OPENPILOT_CAPNP=$sdk/capnp/usr/bin/capnp
export FFMPEG_DIR=$sdk/ffmpeg STARTUP_UI_RAYLIB_ROOT=$sdk/raylib OPENPILOT_ORT_ROOT=$sdk/ort
export ENCODER_LIBYUV_LIB=$sdk/libyuv/lib LOCATIOND_EIGEN_INCLUDE=$sdk/eigen PARAMSD_EIGEN_INCLUDE=$sdk/eigen
export OPENPILOT_OPENCV_ROOT=$sdk/opencv OPENPILOT_WEBCAM_OPENCV_ROOT=$sdk/opencv
link_dirs=("$sdk/capnp/usr/lib/aarch64-linux-gnu" "$sdk/ffmpeg/lib" "$sdk/opencv/install/lib")
LD_LIBRARY_PATH="$(IFS=:; echo "${link_dirs[*]}"):$LD_LIBRARY_PATH"
RUSTFLAGS=
for directory in "${link_dirs[@]}"; do
  RUSTFLAGS+=" -C link-arg=-Wl,-rpath-link,$directory"
done
export RUSTFLAGS

mapfile -t arguments < <(python3 -I -c '
import json
args = json.load(open("rust/ci/runtime/build-args.json"))
for i, value in enumerate(args):
  if value in ("--bin", "-p"):
    if args[i + 1] not in ("openpilot-runtime-install", "openpilot-runtime-installer"):
      print(value); print(args[i + 1])
  elif value == "--features":
    print(value); print(args[i + 1])
')
binaries=()
for ((i = 0; i < ${#arguments[@]}; i++)); do
  [[ ${arguments[i]} == --bin ]] && binaries+=("${arguments[i + 1]}")
done

compile() {
  local jobs=$1 done=0 line
  status "Compiling with $jobs jobs"
  # The compiler, not the display or recovery server, is what the kernel should
  # stop when memory runs out; a smaller job count then continues its results.
  while IFS= read -r line; do
    echo "$line" >> "$log"
    case $line in
      *Compiling*)
        done=$((done + 1))
        [[ -n $ui ]] && echo "Compiling ${line##*Compiling } · $done" >> "$status_file"
        ;;
    esac
  done < <(
    echo 1000 > /proc/self/oom_score_adj
    exec taskset -c 0-7 cargo --config 'source.crates-io.replace-with="device-vendor"' \
      --config "source.device-vendor.directory=\"$sdk/vendor\"" \
      build --manifest-path rust/Cargo.toml --locked --offline --release \
      --target aarch64-unknown-linux-gnu -j "$jobs" "${arguments[@]}" 2>&1
    )
  wait $!
}

built=
for jobs in 6 4 3 2; do
  if compile "$jobs"; then
    built=1
    break
  fi
  status "Compile stopped with $jobs jobs"
done
[[ -n $built ]] || fail 'Compilation failed'

status 'Installing'
output=$store/target/aarch64-unknown-linux-gnu/release
for binary in "${binaries[@]}"; do
  cp "$output/$binary" "rust/bin/.$binary.tmp" && mv "rust/bin/.$binary.tmp" "rust/bin/$binary" ||
    fail "Could not install $binary"
done
install -m 755 rust/tools/device_stage_update.sh rust/bin/openpilot-runtime-install ||
  fail 'Could not install the update helper'

# A previous packaged runtime is no longer referenced by anything after this point.
rm -rf rust/.runtime-packages rust/.runtime-active rust/.runtime-bootstrap \
  rust/.runtime-install-rollback rust/.runtime-install-journal.json rust/runtime-manifest.json
echo "$head" > "$stamp"
status "Build complete in $((SECONDS - started))s"
