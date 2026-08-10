#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
TMP_DIR="$(mktemp -d)"
SOURCE_DIR="$TMP_DIR/source"
BACKUP_DIR="$TMP_DIR/source-backup"
REMOTE_DIR="$TMP_DIR/remote.git"
OUTPUT_FILE="$TMP_DIR/build.log"
VERSION="carrot_test_batch_push"
if python3 -c 'pass' >/dev/null 2>&1; then
  TEST_PYTHON=python3
else
  TEST_PYTHON=python
fi

cleanup() {
  rm -rf -- "$TMP_DIR"
}
trap cleanup EXIT

git -c init.templateDir= init "$SOURCE_DIR" >/dev/null
git -C "$SOURCE_DIR" config user.name "Carrot Release Test"
git -C "$SOURCE_DIR" config user.email "release-test@example.com"
touch "$SOURCE_DIR/README"
printf '*.bin\n' >"$SOURCE_DIR/.gitignore"
mkdir -p "$SOURCE_DIR/carrot/model_selector"
mkdir -p "$SOURCE_DIR/openpilot/system/manager"
mkdir -p "$SOURCE_DIR/openpilot/selfdrive/modeld/models"
touch "$SOURCE_DIR/carrot/__init__.py" "$SOURCE_DIR/carrot/model_selector/__init__.py"
printf '%s\n' \
  'def compile_env_tag():' \
  '  return "tg-env:release-test"' \
  >"$SOURCE_DIR/carrot/model_selector/config.py"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -e' \
  'MODEL_DIR="$(cd "$(dirname "$0")/../../selfdrive/modeld/models" && pwd -P)"' \
  'printf "driving" >"$MODEL_DIR/driving_tinygrad.pkl"' \
  'printf "monitoring" >"$MODEL_DIR/dmonitoring_model_tinygrad.pkl"' \
  'printf "metadata" >"$MODEL_DIR/dmonitoring_model_metadata.pkl"' \
  'printf "warp-tici" >"$MODEL_DIR/dm_warp_1928x1208_tinygrad.pkl"' \
  'printf "warp-mici" >"$MODEL_DIR/dm_warp_1344x760_tinygrad.pkl"' \
  'printf "{}\n" >"$MODEL_DIR/tg_input_devices.json"' \
  >"$SOURCE_DIR/openpilot/system/manager/build.py"
chmod +x "$SOURCE_DIR/openpilot/system/manager/build.py"
printf 'driving-onnx' >"$SOURCE_DIR/openpilot/selfdrive/modeld/models/driving_supercombo.onnx"
printf 'dm-onnx' >"$SOURCE_DIR/openpilot/selfdrive/modeld/models/dmonitoring_model.onnx"
git -C "$SOURCE_DIR" add .
git -C "$SOURCE_DIR" commit -m base >/dev/null
BASE_SHA="$(git -C "$SOURCE_DIR" rev-parse HEAD)"

git -c init.templateDir= init --bare "$REMOTE_DIR" >/dev/null
git -C "$SOURCE_DIR" remote add origin "$REMOTE_DIR"
git -C "$SOURCE_DIR" push origin HEAD:refs/heads/main >/dev/null

# Three incompressible files force multiple 1 MiB release batches.
dd if=/dev/urandom of="$SOURCE_DIR/artifact-1.bin" bs=1024 count=700 status=none
dd if=/dev/urandom of="$SOURCE_DIR/artifact-2.bin" bs=1024 count=700 status=none
dd if=/dev/urandom of="$SOURCE_DIR/artifact-3.bin" bs=1024 count=700 status=none

if ! RELEASE_PUSH_BATCH_MB=1 RELEASE_PUSH_RETRIES=2 PYTHON_BIN="$TEST_PYTHON" \
    "$SCRIPT_DIR/build_carrot.sh" \
      --source "$SOURCE_DIR" \
      --backup "$BACKUP_DIR" \
      --remote-url "$REMOTE_DIR" \
      --branch "$VERSION" \
      --no-prompt-token >"$OUTPUT_FILE" 2>&1; then
  cat "$OUTPUT_FILE" >&2
  exit 1
fi

grep -q "Pushing release batch 2" "$OUTPUT_FILE"
grep -q "Release pushed: $VERSION" "$OUTPUT_FILE"
grep -q "Rebuilding model artifacts for the current tinygrad compiler" "$OUTPUT_FILE"

FINAL_SHA="$(git --git-dir="$REMOTE_DIR" rev-parse "refs/heads/$VERSION")"
[[ "$(git --git-dir="$REMOTE_DIR" rev-list --count "$BASE_SHA..$FINAL_SHA")" == "1" ]]
[[ "$(git --git-dir="$REMOTE_DIR" rev-parse "$FINAL_SHA^")" == "$BASE_SHA" ]]
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:artifact-1.bin"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:artifact-2.bin"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:artifact-3.bin"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:prebuilt"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:openpilot/selfdrive/modeld/models/driving_tinygrad.pkl"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:openpilot/selfdrive/modeld/models/dmonitoring_model_tinygrad.pkl"
[[ "$(git --git-dir="$REMOTE_DIR" show "$FINAL_SHA:openpilot/selfdrive/modeld/models/.build_stamp")" == "tg-env:release-test" ]]
if git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:openpilot/selfdrive/modeld/models/driving_supercombo.onnx" 2>/dev/null; then
  echo "release unexpectedly retained model ONNX" >&2
  exit 1
fi

[[ -d "$SOURCE_DIR/.git" ]]
[[ ! -e "$BACKUP_DIR" ]]
[[ -f "$SOURCE_DIR/artifact-1.bin" ]]

printf 'build_carrot batch push test passed\n'
