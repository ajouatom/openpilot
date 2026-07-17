#!/usr/bin/env bash
set -Eeuo pipefail

SOURCE_DIR_RAW="${SOURCE_DIR:-/data/openpilot}"
BACKUP_DIR_RAW="${BACKUP_DIR:-/data/openpilot_bak}"
REMOTE_NAME="${REMOTE_NAME:-origin}"
RELEASE_REMOTE_URL="${RELEASE_REMOTE_URL:-}"
VERSION="${VERSION:-carrot_v$(date +%y%m%d)}"
MIN_FREE_AFTER_BACKUP_KB="${MIN_FREE_AFTER_BACKUP_KB:-524288}"

DRY_RUN=0
RECOVER_ONLY=0
PROMPT_GITHUB_TOKEN=1
BACKUP_READY=0
BACKUP_TMP=""
RECOVERY_SCRIPT_COPY="${BUILD_CARROT_RECOVERY_COPY:-}"
ASKPASS_FILE=""

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
MIN_FREE_AFTER_BACKUP_KB.
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
  unset GITHUB_TOKEN
fi

if [[ -n "$RELEASE_REMOTE_URL" ]]; then
  PUSH_TARGET="$RELEASE_REMOTE_URL"
  REMOTE_LABEL="custom release URL"
else
  git -C "$SOURCE_DIR" remote get-url "$REMOTE_NAME" >/dev/null || die "Git remote not found: $REMOTE_NAME"
  PUSH_TARGET="$REMOTE_NAME"
  REMOTE_LABEL="$REMOTE_NAME"
fi

git -C "$SOURCE_DIR" ls-remote "$PUSH_TARGET" HEAD >/dev/null || die "cannot access release remote: $REMOTE_LABEL"

SOURCE_COMMIT="$(git -C "$SOURCE_DIR" rev-parse HEAD)"
PREFLIGHT_REFS="$(git -C "$SOURCE_DIR" ls-remote --heads "$PUSH_TARGET" "refs/heads/$VERSION")"
PREFLIGHT_SHA="$(printf '%s\n' "$PREFLIGHT_REFS" | awk 'NR == 1 {print $1}')"
if [[ -n "$PREFLIGHT_SHA" ]]; then
  git -C "$SOURCE_DIR" push --dry-run \
    --force-with-lease="refs/heads/$VERSION:$PREFLIGHT_SHA" \
    "$PUSH_TARGET" "$SOURCE_COMMIT:refs/heads/$VERSION" >/dev/null 2>&1 || die "GitHub token does not have push access"
else
  git -C "$SOURCE_DIR" push --dry-run \
    "$PUSH_TARGET" "$SOURCE_COMMIT:refs/heads/$VERSION" >/dev/null 2>&1 || die "GitHub token does not have push access"
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

log "Cleaning build-only files"
find . -type f \( -name '*.a' -o -name '*.o' -o -name '*.os' -o -name '*.pyc' -o -name 'moc_*' \) -delete
find . -type d -name '__pycache__' -prune -exec rm -rf -- {} +
rm -rf -- .sconsign.dblite Jenkinsfile release/
rm -f -- openpilot/selfdrive/modeld/models/*.onnx
touch prebuilt

git add -f -A
git diff --cached --quiet && die "release cleanup produced no changes"
git commit -m "$VERSION"

BIG_FILES="$(find . -type f -not -path './.git/*' -size +95M -print)"
[[ -z "$BIG_FILES" ]] || die "files exceeding the GitHub size limit were found:\n$BIG_FILES"

RELEASE_COMMIT="$(git rev-parse HEAD)"
REMOTE_REFS="$(git ls-remote --heads "$PUSH_TARGET" "refs/heads/$VERSION")"
REMOTE_SHA="$(printf '%s\n' "$REMOTE_REFS" | awk 'NR == 1 {print $1}')"

if [[ -n "$REMOTE_SHA" ]]; then
  log "Updating existing branch $VERSION with force-with-lease"
  git push --force-with-lease="refs/heads/$VERSION:$REMOTE_SHA" "$PUSH_TARGET" "HEAD:refs/heads/$VERSION"
else
  log "Pushing new branch $VERSION"
  git push "$PUSH_TARGET" "HEAD:refs/heads/$VERSION"
fi

log "Release pushed: $VERSION @ $RELEASE_COMMIT"
log "Automatic source restoration will run now"
