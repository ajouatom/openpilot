#!/usr/bin/env bash
# cargo's RUSTC_WRAPPER during the device build. The boot arguments isolate
# CPUs 6-7 (isolcpus, kept for the cameras and the UI onroad), so the scheduler
# never moves a compile there even under taskset -c 0-7: the build ran on four
# little cores and two big ones while two big cores sat idle. Nothing else runs
# on them before openpilot starts, so one compile is pinned to each.
#
# Only while at least three other compiles run: a pinned compile keeps its
# codegen threads on that one core, and near the end, with one large crate
# left, it does better spread over CPUs 0-5.
if (( $(pgrep -c -x rustc 2>/dev/null || echo 0) >= 3 )); then
  for core in ${DEVICE_BUILD_PINNED_CORES-}; do
    exec {lock}> "${OPENPILOT_BUILD_STORE:-/data/openpilot-build}/core$core.lock" || continue
    # The lock stays held through exec until this compile and its linker exit.
    flock -n "$lock" && exec taskset -c "$core" "$@"
    exec {lock}>&-
  done
fi
exec "$@"
