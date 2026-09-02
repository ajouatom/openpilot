#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  # Keep fresh C3X/C4 installs on comma's production AGNOS. The optional
  # carrot image contains an experimental USB-PD kernel and must not be rolled
  # out implicitly from the custom software installer.
  export AGNOS_VERSION="19.6"
  export AGNOS_COMPATIBLE_VERSIONS="19.6 19.6.3-carrot"
else
  export AGNOS_COMPATIBLE_VERSIONS="${AGNOS_COMPATIBLE_VERSIONS:-$AGNOS_VERSION}"
fi

export STAGING_ROOT="/data/safe_staging"
