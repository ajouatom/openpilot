#!/usr/bin/env bash
set -Eeuo pipefail

SOURCE_DIR_RAW="${SOURCE_DIR:-/data/openpilot}"
BACKUP_DIR_RAW="${BACKUP_DIR:-/data/openpilot_bak}"
REMOTE_NAME="${REMOTE_NAME:-origin}"
RELEASE_REMOTE_URL="${RELEASE_REMOTE_URL:-}"
VERSION="${VERSION:-carrot_v$(date +%y%m%d)}"
MIN_FREE_AFTER_BACKUP_KB="${MIN_FREE_AFTER_BACKUP_KB:-524288}"
RELEASE_PUSH_BATCH_MB="${RELEASE_PUSH_BATCH_MB:-32}"
RELEASE_PUSH_RETRIES="${RELEASE_PUSH_RETRIES:-3}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

DRY_RUN=0
RECOVER_ONLY=0
PROMPT_GITHUB_TOKEN=1
BACKUP_READY=0
BACKUP_TMP=""
RECOVERY_SCRIPT_COPY="${BUILD_CARROT_RECOVERY_COPY:-}"
ASKPASS_FILE=""
GIT_AUTH_CONFIG=()

log() {
  printf '[build_carrot] %s\n' "$*"
}

die() {
  log "ERROR: $*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: release/build_carrot.sh [options]

Creates and pushes a dated prebuilt release branch, then restores the original
/data/openpilot tree from an automatic cp -a backup.

Options:
  --dry-run          Validate paths, branch name, and GitHub access only
  --recover          Restore SOURCE_DIR from BACKUP_DIR after an interrupted run
  --branch NAME      Release branch name (default: carrot_vYYMMDD)
  --source PATH      Source tree (default: /data/openpilot)
  --backup PATH      Backup tree (default: /data/openpilot_bak)
  --remote NAME      Git remote name (default: origin)
  --remote-url URL   Push to URL instead of the configured remote
  --prompt-token     Securely prompt for a GitHub token (default)
  --no-prompt-token  Use existing Git credentials without prompting
  -h, --help         Show this help

Environment variables with the same names can also be used:
SOURCE_DIR, BACKUP_DIR, REMOTE_NAME, RELEASE_REMOTE_URL, VERSION, and
MIN_FREE_AFTER_BACKUP_KB, RELEASE_PUSH_BATCH_MB, and RELEASE_PUSH_RETRIES.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --recover)
      RECOVER_ONLY=1
      shift
      ;;
    --branch)
      [[ $# -ge 2 ]] || die "--branch requires a value"
      VERSION="$2"
      shift 2
      ;;
    --source)
      [[ $# -ge 2 ]] || die "--source requires a value"
      SOURCE_DIR_RAW="$2"
      shift 2
      ;;
    --backup)
      [[ $# -ge 2 ]] || die "--backup requires a value"
      BACKUP_DIR_RAW="$2"
      shift 2
      ;;
    --remote)
      [[ $# -ge 2 ]] || die "--remote requires a value"
      REMOTE_NAME="$2"
      shift 2
      ;;
    --remote-url)
      [[ $# -ge 2 ]] || die "--remote-url requires a value"
      RELEASE_REMOTE_URL="$2"
      shift 2
      ;;
    --prompt-token)
      PROMPT_GITHUB_TOKEN=1
      shift
      ;;
    --no-prompt-token)
      PROMPT_GITHUB_TOKEN=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

canonical_path() {
  local raw_path="$1"
  local parent
  local name

  parent="$(dirname -- "$raw_path")"
  name="$(basename -- "$raw_path")"
  parent="$(cd "$parent" 2>/dev/null && pwd -P)" || die "parent directory does not exist: $parent"
  printf '%s/%s\n' "$parent" "$name"
}

SOURCE_DIR="$(canonical_path "$SOURCE_DIR_RAW")"
BACKUP_DIR="$(canonical_path "$BACKUP_DIR_RAW")"

[[ "$SOURCE_DIR" != "/" && "$BACKUP_DIR" != "/" ]] || die "refusing to use the filesystem root"
[[ "$SOURCE_DIR" != "$BACKUP_DIR" ]] || die "source and backup paths must differ"

case "$BACKUP_DIR/" in
  "$SOURCE_DIR/"*) die "backup directory must not be inside source directory" ;;
esac
case "$SOURCE_DIR/" in
  "$BACKUP_DIR/"*) die "source directory must not be inside backup directory" ;;
esac

# Recovery is normally launched from the backup tree itself. Re-exec a copy
# from /tmp so restoring can move the entire backup directory atomically even
# on filesystems that lock the currently executing script.
if [[ "$RECOVER_ONLY" -eq 1 && -z "$RECOVERY_SCRIPT_COPY" ]]; then
  RECOVERY_SCRIPT_COPY="$(mktemp /tmp/build_carrot_recover.XXXXXX)"
  cp -- "${BASH_SOURCE[0]}" "$RECOVERY_SCRIPT_COPY"
  chmod +x "$RECOVERY_SCRIPT_COPY"
  exec env \
    BUILD_CARROT_RECOVERY_COPY="$RECOVERY_SCRIPT_COPY" \
    SOURCE_DIR="$SOURCE_DIR" \
    BACKUP_DIR="$BACKUP_DIR" \
    "$RECOVERY_SCRIPT_COPY" --recover
fi

safe_remove_source() {
  [[ -e "$SOURCE_DIR" ]] || return 0
  [[ -d "$SOURCE_DIR" && ! -L "$SOURCE_DIR" ]] || {
    log "Refusing to remove unexpected source path: $SOURCE_DIR" >&2
    return 1
  }
  [[ -d "$SOURCE_DIR/.git" ]] || {
    log "Refusing to remove source without .git marker: $SOURCE_DIR" >&2
    return 1
  }
  rm -rf -- "$SOURCE_DIR"
}

restore_original() {
  [[ -d "$BACKUP_DIR/.git" ]] || {
    log "Backup is missing or invalid: $BACKUP_DIR" >&2
    return 1
  }

  log "Restoring original tree: $BACKUP_DIR -> $SOURCE_DIR"
  safe_remove_source || return 1
  mv -- "$BACKUP_DIR" "$SOURCE_DIR" || return 1
  sync
  log "Original source tree restored: $SOURCE_DIR"
}

on_exit() {
  local rc=$?
  trap - EXIT INT TERM HUP
  set +e

  if [[ "$BACKUP_READY" -eq 1 ]]; then
    if restore_original; then
      BACKUP_READY=0
    else
      log "AUTOMATIC RESTORE FAILED. Preserve $BACKUP_DIR and run with --recover." >&2
      rc=1
    fi
  fi

  if [[ -n "$BACKUP_TMP" && -e "$BACKUP_TMP" ]]; then
    rm -rf -- "$BACKUP_TMP"
  fi

  if [[ -n "$RECOVERY_SCRIPT_COPY" && -f "$RECOVERY_SCRIPT_COPY" ]]; then
    rm -f -- "$RECOVERY_SCRIPT_COPY"
  fi

  if [[ -n "$ASKPASS_FILE" && -f "$ASKPASS_FILE" ]]; then
    rm -f -- "$ASKPASS_FILE"
  fi
  unset BUILD_CARROT_GITHUB_TOKEN GIT_ASKPASS GITHUB_TOKEN

  exit "$rc"
}

trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

if [[ "$RECOVER_ONLY" -eq 1 ]]; then
  [[ -d "$BACKUP_DIR/.git" ]] || die "no recoverable backup found at $BACKUP_DIR"
  BACKUP_READY=1
  if restore_original; then
    BACKUP_READY=0
  else
    BACKUP_READY=0
    die "recovery failed; backup remains at $BACKUP_DIR"
  fi
  exit 0
fi

[[ -d "$SOURCE_DIR/.git" && ! -L "$SOURCE_DIR" ]] || die "source is not a normal Git checkout: $SOURCE_DIR"
[[ ! -e "$BACKUP_DIR" ]] || die "backup already exists at $BACKUP_DIR; inspect it and run --recover if needed"

if [[ -e /data/params/d/IsOnroad ]]; then
  IS_ONROAD="$(tr -d '\000[:space:]' </data/params/d/IsOnroad 2>/dev/null || true)"
  [[ "$IS_ONROAD" != "1" && "$IS_ONROAD" != "true" ]] || die "vehicle is onroad; release build is refused"
fi

git -C "$SOURCE_DIR" check-ref-format --branch "$VERSION" >/dev/null || die "invalid release branch: $VERSION"
[[ "$RELEASE_PUSH_BATCH_MB" =~ ^[1-9][0-9]*$ ]] || die "RELEASE_PUSH_BATCH_MB must be a positive integer"
[[ "$RELEASE_PUSH_RETRIES" =~ ^[1-9][0-9]*$ ]] || die "RELEASE_PUSH_RETRIES must be a positive integer"
RELEASE_PUSH_BATCH_BYTES=$((RELEASE_PUSH_BATCH_MB * 1024 * 1024))

if [[ "$PROMPT_GITHUB_TOKEN" -eq 1 && -z "${GITHUB_TOKEN:-}" ]]; then
  read -r -s -p "GitHub token: " GITHUB_TOKEN
  printf '\n'
fi

if [[ -n "${GITHUB_TOKEN:-}" ]]; then
  ASKPASS_FILE="$(mktemp /tmp/build_carrot_askpass.XXXXXX)"
  cat >"$ASKPASS_FILE" <<'EOF'
#!/usr/bin/env bash
case "$1" in
  *Username*) printf '%s\n' 'x-access-token' ;;
  *) printf '%s\n' "$BUILD_CARROT_GITHUB_TOKEN" ;;
esac
EOF
  chmod 700 "$ASKPASS_FILE"
  export BUILD_CARROT_GITHUB_TOKEN="$GITHUB_TOKEN"
  export GIT_ASKPASS="$ASKPASS_FILE"
  export GIT_TERMINAL_PROMPT=0
  GIT_AUTH_CONFIG=(-c credential.helper=)
  unset GITHUB_TOKEN
fi

git_release() {
  # Release refs never introduce new LFS content, and this repository doesn't
  # use LFS file locking. Keep the LFS hook enabled for object verification,
  # but skip the lock API which requires a separate authentication round-trip.
  git "${GIT_AUTH_CONFIG[@]}" -c lfs.locksverify=false "$@"
}

model_compile_tag() {
  PYTHONPATH="$SOURCE_DIR${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" -c \
    'from carrot.model_selector.config import compile_env_tag; print(compile_env_tag(), end="")'
}

require_model_artifact() {
  local artifact="$1"
  local manifest="${artifact}.chunkmanifest"
  local chunk_count
  local chunk_index
  local chunk

  [[ -s "$artifact" ]] && return 0
  [[ -s "$manifest" ]] || die "missing generated model artifact: $artifact"
  chunk_count="$(tr -d '[:space:]' <"$manifest")"
  [[ "$chunk_count" =~ ^[1-9][0-9]*$ ]] || die "invalid model chunk manifest: $manifest"
  for ((chunk_index = 1; chunk_index <= chunk_count; chunk_index++)); do
    printf -v chunk '%s.chunk%02dof%02d' "$artifact" "$chunk_index" "$chunk_count"
    [[ -s "$chunk" ]] || die "missing generated model chunk: $chunk"
  done
}

validate_prebuilt_models() {
  local model_dir="$SOURCE_DIR/openpilot/selfdrive/modeld/models"
  local expected_tag="$1"
  local packaged_tag

  require_model_artifact "$model_dir/driving_tinygrad.pkl"
  require_model_artifact "$model_dir/dmonitoring_model_tinygrad.pkl"
  [[ -s "$model_dir/dmonitoring_model_metadata.pkl" ]] || die "missing dmonitoring metadata"
  [[ -s "$model_dir/dm_warp_1928x1208_tinygrad.pkl" ]] || die "missing TICI DM warp"
  [[ -s "$model_dir/dm_warp_1344x760_tinygrad.pkl" ]] || die "missing mici DM warp"
  [[ -s "$model_dir/tg_input_devices.json" ]] || die "missing tinygrad device map"
  packaged_tag="$(tr -d '[:space:]' <"$model_dir/.build_stamp" 2>/dev/null || true)"
  [[ "$packaged_tag" == "$expected_tag" ]] || die "model build stamp does not match compiler fingerprint"
}

prepare_prebuilt_models() {
  local model_dir="$SOURCE_DIR/openpilot/selfdrive/modeld/models"
  local expected_tag

  # The small release-script fixture has no openpilot model build. Real device
  # releases always do, and must compile before their ONNX inputs are removed.
  if [[ ! -f "$SOURCE_DIR/carrot/model_selector/config.py" || ! -f "$SOURCE_DIR/openpilot/system/manager/build.py" ]]; then
    return 0
  fi

  log "Rebuilding model artifacts for the current tinygrad compiler"
  rm -f -- "$model_dir"/*_tinygrad.pkl* "$model_dir"/*_metadata.pkl
  rm -f -- "$model_dir/tg_input_devices.json" "$model_dir/.build_stamp"
  (cd "$SOURCE_DIR/openpilot/system/manager" && ./build.py) || die "openpilot prebuilt compilation failed"

  expected_tag="$(model_compile_tag)" || die "could not calculate model compiler fingerprint"
  [[ -n "$expected_tag" ]] || die "empty model compiler fingerprint"
  printf '%s' "$expected_tag" >"$model_dir/.build_stamp"
  validate_prebuilt_models "$expected_tag"
}

if [[ -n "$RELEASE_REMOTE_URL" ]]; then
  PUSH_TARGET="$RELEASE_REMOTE_URL"
  REMOTE_LABEL="custom release URL"
else
  git -C "$SOURCE_DIR" remote get-url "$REMOTE_NAME" >/dev/null || die "Git remote not found: $REMOTE_NAME"
  PUSH_TARGET="$REMOTE_NAME"
  REMOTE_LABEL="$REMOTE_NAME"
fi

git_release -C "$SOURCE_DIR" ls-remote "$PUSH_TARGET" HEAD >/dev/null || die "cannot access release remote: $REMOTE_LABEL"

SOURCE_COMMIT="$(git -C "$SOURCE_DIR" rev-parse HEAD)"
PREFLIGHT_REFS="$(git_release -C "$SOURCE_DIR" ls-remote --heads "$PUSH_TARGET" "refs/heads/$VERSION")"
PREFLIGHT_SHA="$(printf '%s\n' "$PREFLIGHT_REFS" | awk 'NR == 1 {print $1}')"
if [[ -n "$PREFLIGHT_SHA" ]]; then
  git_release -C "$SOURCE_DIR" push --dry-run \
    --force-with-lease="refs/heads/$VERSION:$PREFLIGHT_SHA" \
    "$PUSH_TARGET" "$SOURCE_COMMIT:refs/heads/$VERSION" >/dev/null || die "release remote rejected the preflight push"
else
  git_release -C "$SOURCE_DIR" push --dry-run \
    "$PUSH_TARGET" "$SOURCE_COMMIT:refs/heads/$VERSION" >/dev/null || die "release remote rejected the preflight push"
fi

log "Source:  $SOURCE_DIR"
log "Backup:  $BACKUP_DIR"
log "Branch:  $VERSION"
log "Remote:  $REMOTE_LABEL"

if [[ "$DRY_RUN" -eq 1 ]]; then
  log "Dry run complete; no files or refs were changed"
  exit 0
fi

SOURCE_KB="$(du -sk "$SOURCE_DIR" | awk '{print $1}')"
AVAILABLE_KB="$(df -Pk "$(dirname "$BACKUP_DIR")" | awk 'NR == 2 {print $4}')"
REQUIRED_KB=$((SOURCE_KB + MIN_FREE_AFTER_BACKUP_KB))
if (( AVAILABLE_KB < REQUIRED_KB )); then
  die "not enough free space for backup: need ${REQUIRED_KB} KiB, available ${AVAILABLE_KB} KiB"
fi

BACKUP_TMP="${BACKUP_DIR}.tmp.$$"
[[ ! -e "$BACKUP_TMP" ]] || die "temporary backup path already exists: $BACKUP_TMP"

log "Backing up the complete source tree with cp -a"
cp -a -- "$SOURCE_DIR" "$BACKUP_TMP"
[[ -d "$BACKUP_TMP/.git" ]] || die "backup verification failed: .git is missing"
mv -- "$BACKUP_TMP" "$BACKUP_DIR"
BACKUP_TMP=""
sync
BACKUP_READY=1
log "Backup complete"

cd "$SOURCE_DIR"
BUILD_BRANCH="release-build/${VERSION}-$(date +%s)-$$"
git switch -c "$BUILD_BRANCH" "$SOURCE_COMMIT"

prepare_prebuilt_models

log "Cleaning build-only files"
find . -type f \( -name '*.a' -o -name '*.o' -o -name '*.os' -o -name '*.pyc' -o -name 'moc_*' \) -delete
find . -type d -name '__pycache__' -prune -exec rm -rf -- {} +
rm -rf -- .sconsign.dblite Jenkinsfile release/
rm -f -- openpilot/selfdrive/modeld/models/*.onnx
touch prebuilt

BIG_FILES="$(find . -type f -not -path './.git/*' -size +95M -print)"
[[ -z "$BIG_FILES" ]] || die "files exceeding the GitHub size limit were found:\n$BIG_FILES"

REMOTE_SHA="$PREFLIGHT_SHA"

push_release_head() {
  local label="$1"
  local current_sha attempt remote_after

  current_sha="$(git rev-parse HEAD)"
  for ((attempt = 1; attempt <= RELEASE_PUSH_RETRIES; attempt++)); do
    log "Pushing $label (attempt $attempt/$RELEASE_PUSH_RETRIES)"
    if [[ -n "$REMOTE_SHA" ]]; then
      if git_release push --force-with-lease="refs/heads/$VERSION:$REMOTE_SHA" \
          "$PUSH_TARGET" "HEAD:refs/heads/$VERSION"; then
        REMOTE_SHA="$current_sha"
        return 0
      fi
    elif git_release push "$PUSH_TARGET" "HEAD:refs/heads/$VERSION"; then
      REMOTE_SHA="$current_sha"
      return 0
    fi

    # The server can update the ref even when the client loses the HTTP
    # response. Treat that case as success before retrying.
    remote_after="$(git_release ls-remote --heads "$PUSH_TARGET" "refs/heads/$VERSION" 2>/dev/null | awk 'NR == 1 {print $1}' || true)"
    if [[ "$remote_after" == "$current_sha" ]]; then
      log "$label reached the remote despite the interrupted response"
      REMOTE_SHA="$current_sha"
      return 0
    fi

    if (( attempt < RELEASE_PUSH_RETRIES )); then
      log "$label push failed; retrying"
      sleep $((attempt * 2))
    fi
  done
  die "failed to push $label after $RELEASE_PUSH_RETRIES attempts"
}

BATCH_INDEX=0
BATCH_BYTES=0
BATCH_PATHS=()

commit_release_batch() {
  ((${#BATCH_PATHS[@]} > 0)) || return 0

  git add -f -A -- "${BATCH_PATHS[@]}"
  BATCH_PATHS=()
  BATCH_BYTES=0
  git diff --cached --quiet && return 0

  BATCH_INDEX=$((BATCH_INDEX + 1))
  git commit -m "$VERSION build batch $BATCH_INDEX"
  push_release_head "release batch $BATCH_INDEX"
}

# Stage build products in bounded batches. Each intermediate push seeds the
# remote with the next group of blobs, avoiding one large HTTP request. The
# commits are squashed below, so the final release branch still has one commit.
mapfile -d '' -t RELEASE_PATHS < <(
  {
    git diff --name-only -z
    git ls-files --others -z
  } | sort -zu
)
((${#RELEASE_PATHS[@]} > 0)) || die "release cleanup produced no changes"

for path in "${RELEASE_PATHS[@]}"; do
  path_bytes=0
  if [[ -e "$path" || -L "$path" ]]; then
    path_bytes="$(stat -c '%s' -- "$path")"
  fi

  if ((${#BATCH_PATHS[@]} > 0 && BATCH_BYTES + path_bytes > RELEASE_PUSH_BATCH_BYTES)); then
    commit_release_batch
  fi
  BATCH_PATHS+=("$path")
  BATCH_BYTES=$((BATCH_BYTES + path_bytes))
done
commit_release_batch

# Catch files created or changed while the batches were being committed.
git add -f -A
if ! git diff --cached --quiet; then
  BATCH_INDEX=$((BATCH_INDEX + 1))
  git commit -m "$VERSION build batch $BATCH_INDEX"
  push_release_head "release batch $BATCH_INDEX"
fi

# Replace the temporary batch history with the original single release commit.
# All large blobs already exist on the remote, so this final force-with-lease
# push only transfers the final commit and tree metadata.
git reset --soft "$SOURCE_COMMIT"
git commit -m "$VERSION"
RELEASE_COMMIT="$(git rev-parse HEAD)"
if [[ -f "$SOURCE_DIR/carrot/model_selector/config.py" && -f "$SOURCE_DIR/openpilot/system/manager/build.py" ]]; then
  FINAL_MODEL_TAG="$(model_compile_tag)" || die "could not verify final model compiler fingerprint"
  validate_prebuilt_models "$FINAL_MODEL_TAG"
fi
push_release_head "final release commit"

log "Release pushed: $VERSION @ $RELEASE_COMMIT"
log "Automatic source restoration will run now"
