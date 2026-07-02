#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
OPENPILOT_ROOT="${SCRIPT_DIR}/../.."

exec "${OPENPILOT_ROOT}/openpilot/tools/sim/launch_openpilot.sh" "$@"
