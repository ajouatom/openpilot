#!/usr/bin/env bash
# The next start compiles what changed, so an update is only the Git operation.
set -euo pipefail
root= target= branch= mode=
while [[ $# -gt 0 ]]; do
  case $1 in
    --stage-update) shift ;;
    --root) root=$2; shift 2 ;;
    --target) target=$2; shift 2 ;;
    --branch) branch=$2; shift 2 ;;
    --detach) mode=detach; shift ;;
    --ff-only) mode=ff; shift ;;
    *) echo "unsupported update option: $1" >&2; exit 2 ;;
  esac
done
[[ -n $root && -n $target ]] || { echo 'update needs --root and --target' >&2; exit 2; }
cd "$root"
git cat-file -e "$target^{commit}" 2>/dev/null || git fetch origin "$target"
# A branch picked by name follows the remote, as carrot's own branch change
# does (checkout -B b origin/b); the local copy of a branch left earlier is old.
if [[ -n $branch && $target != HEAD ]] &&
  remote=$(git rev-parse -q --verify "refs/remotes/origin/$branch^{commit}") &&
  [[ $(git rev-parse "$target^{commit}") == $(git rev-parse -q --verify "refs/heads/$branch^{commit}") ]]; then
  target=$remote
fi
if [[ -n $branch ]]; then
  git switch -C "$branch" "$target"
elif [[ $mode == detach ]]; then
  git switch --detach "$target"
else
  git merge --ff-only "$target"
fi
