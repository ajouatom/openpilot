#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
TMP_DIR="$(mktemp -d)"
SOURCE_DIR="$TMP_DIR/source"
BACKUP_DIR="$TMP_DIR/source-backup"
REMOTE_DIR="$TMP_DIR/remote.git"
OUTPUT_FILE="$TMP_DIR/build.log"
VERSION="carrot_test_batch_push"

cleanup() {
  rm -rf -- "$TMP_DIR"
}
trap cleanup EXIT

git -c init.templateDir= init "$SOURCE_DIR" >/dev/null
git -C "$SOURCE_DIR" config user.name "Carrot Release Test"
git -C "$SOURCE_DIR" config user.email "release-test@example.com"
touch "$SOURCE_DIR/README"
printf '*.bin\n' >"$SOURCE_DIR/.gitignore"
git -C "$SOURCE_DIR" add README .gitignore
git -C "$SOURCE_DIR" commit -m base >/dev/null
BASE_SHA="$(git -C "$SOURCE_DIR" rev-parse HEAD)"

git -c init.templateDir= init --bare "$REMOTE_DIR" >/dev/null
git -C "$SOURCE_DIR" remote add origin "$REMOTE_DIR"
git -C "$SOURCE_DIR" push origin HEAD:refs/heads/main >/dev/null

# Three incompressible files force multiple 1 MiB release batches.
dd if=/dev/urandom of="$SOURCE_DIR/artifact-1.bin" bs=1024 count=700 status=none
dd if=/dev/urandom of="$SOURCE_DIR/artifact-2.bin" bs=1024 count=700 status=none
dd if=/dev/urandom of="$SOURCE_DIR/artifact-3.bin" bs=1024 count=700 status=none

if ! RELEASE_PUSH_BATCH_MB=1 RELEASE_PUSH_RETRIES=2 \
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

FINAL_SHA="$(git --git-dir="$REMOTE_DIR" rev-parse "refs/heads/$VERSION")"
[[ "$(git --git-dir="$REMOTE_DIR" rev-list --count "$BASE_SHA..$FINAL_SHA")" == "1" ]]
[[ "$(git --git-dir="$REMOTE_DIR" rev-parse "$FINAL_SHA^")" == "$BASE_SHA" ]]
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:artifact-1.bin"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:artifact-2.bin"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:artifact-3.bin"
git --git-dir="$REMOTE_DIR" cat-file -e "$FINAL_SHA:prebuilt"

[[ -d "$SOURCE_DIR/.git" ]]
[[ ! -e "$BACKUP_DIR" ]]
[[ -f "$SOURCE_DIR/artifact-1.bin" ]]

printf 'build_carrot batch push test passed\n'
