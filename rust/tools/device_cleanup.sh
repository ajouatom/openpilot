#!/usr/bin/env bash
# After an update is built (device_build.sh), keep only what this version
# needs. A broken update is repaired from port 6999 or by choosing another
# branch, so no older checkout, build output or history is kept for a
# rollback. Measured on the C3X Lite (K5, ~18 GB /data), these leftovers
# pushed free space under the deleter's floor and drive logs were deleted.
#
#   device_cleanup.sh now ROOT STORE   before openpilot starts (seconds):
#       cargo outputs this build did not use, the updater's old checkouts
#       and copies in safe_staging; then starts the part below.
#   device_cleanup.sh git ROOT STORE   in the background, little cores, idle
#       I/O, under the repository lock: refs and history other than this
#       branch, old reflogs and LFS objects, then a repack.
set -uo pipefail
mode=${1:-} root=${2:-} store=${3:-}
[[ -n $mode && -d $root && -d $store ]] || { echo 'usage: device_cleanup.sh now|git ROOT STORE' >&2; exit 2; }
log=$store/cleanup.log
free_mb() { df -Pm "$root" | awk 'NR == 2 {print $4}'; }
note() { echo "[$(date '+%F %T')] $*" >> "$log"; }
remove() {
  [[ -d $1 && ! -L $1 ]] || return 0
  note "removing $1 ($(du -sm "$1" 2>/dev/null | cut -f1) MB)"
  rm -rf -- "$1" 2>/dev/null || sudo -n rm -rf -- "$1"
}

clean_staging() {
  local staging=${UPDATER_STAGING_ROOT:-/data/safe_staging} path
  [[ -d $staging ]] || return 0
  if [[ -e $staging/.runtime-update-journal.json ]]; then
    note 'safe_staging: an activation is pending; left for startup recovery'
    return 0
  fi
  # Rollback checkouts of earlier installs (launcher and native installer).
  for path in "$staging"/old_openpilot "$staging"/old_openpilot.*; do
    remove "$path"
  done
  # A native update the user downloaded is applied by the launcher after this
  # build, so it stays. The shell launcher installs an overlay copy before the
  # build; one still here was refused ("has been modified") and is stale.
  [[ -e $staging/finalized/.runtime-update-ready.json ]] || remove "$staging/finalized"
  if ! mountpoint -q "$staging/merged" 2>/dev/null; then
    for path in "$staging"/upper "$staging"/metadata "$staging"/merged; do
      remove "$path"
    done
  fi
}

trim_git() {
  local lock_path=${CARROT_REPO_LOCK_PATH:-/tmp/carrot_repo_update.lock} lock
  exec {lock}>"$lock_path" || return 0
  # The launcher holds this lock until the manager is up; web updates take it too.
  if ! flock -w 1800 "$lock"; then
    note 'git: repository busy for 30 min; trim skipped'
    return 0
  fi
  cd "$root" || return 0
  local head branch upstream ref before gitdir
  head=$(git rev-parse -q --verify 'HEAD^{commit}') || return 0
  gitdir=$(git rev-parse --absolute-git-dir) || return 0
  branch=$(git symbolic-ref -q --short HEAD || true)
  upstream=
  [[ -n $branch ]] && upstream=$(git rev-parse -q --symbolic-full-name '@{upstream}' 2>/dev/null || true)
  before=$(du -sm "$gitdir" | cut -f1)
  # Only this branch and what it tracks. The web branch list and port 6999
  # fetch the others again when they are asked for.
  git for-each-ref --format='%(refname)' | while IFS= read -r ref; do
    case $ref in
      "refs/heads/$branch" | refs/stash) continue ;;
    esac
    [[ -n $upstream && $ref == "$upstream" ]] && continue
    git update-ref -d --no-deref "$ref"
  done
  # History stops at this version (and at the bases of saved stashes, so
  # they still apply). Later fetches add only the new commits.
  {
    echo "$head"
    [[ -n $upstream ]] && git rev-parse -q --verify "$upstream^{commit}"
    git log -g --format='%P' refs/stash 2>/dev/null | tr ' ' '\n'
  } | sed '/^$/d' | sort -u > "$gitdir/shallow.new" && mv "$gitdir/shallow.new" "$gitdir/shallow"
  { git for-each-ref --format='%(refname)' | grep -vx refs/stash; echo HEAD; } |
    while IFS= read -r ref; do
      # A reflog that cannot be walked any more (it names commits past the
      # cut) only holds old history: drop it.
      git reflog expire --expire=now --expire-unreachable=now "$ref" > /dev/null 2>&1 ||
        rm -f -- "$gitdir/logs/$ref"
    done
  if git lfs version > /dev/null 2>&1; then
    git -c lfs.fetchrecentrefsdays=0 -c lfs.fetchrecentcommitsdays=0 \
      -c lfs.fetchrecentremoterefs=false -c lfs.pruneoffsetdays=0 lfs prune >> "$log" 2>&1
  fi
  git -c pack.threads=1 -c pack.windowMemory=128m -c core.bigFileThreshold=64m \
    gc --prune=now --quiet >> "$log" 2>&1
  note "git: .git ${before} MB -> $(du -sm "$gitdir" | cut -f1) MB (kept ${branch:-detached HEAD} ${upstream})"
}

case $mode in
  now)
    started=$(free_mb)
    if [[ -f $store/artifacts.jsonl ]]; then
      python3 -I "$root/rust/tools/device_sweep_target.py" "$store/target" "$store/artifacts.jsonl" >> "$log" 2>&1
    fi
    clean_staging
    note "update cleanup: free ${started} MB -> $(free_mb) MB; history trim continues in the background"
    # Detached from the launcher: the inherited boot lock (fd 9) is closed so
    # the trim waits for that lock like any other repository user.
    setsid nohup taskset -c 0-3 nice -n 19 ionice -c 3 \
      bash "$0" git "$root" "$store" 9>&- < /dev/null >> "$log" 2>&1 &
    ;;
  git)
    started=$(free_mb)
    trim_git
    note "history trim: free ${started} MB -> $(free_mb) MB"
    ;;
  *)
    echo "unknown mode: $mode" >&2
    exit 2
    ;;
esac
exit 0
