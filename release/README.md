# openpilot releases

## Carrot device release branch

`build_carrot.sh` creates a dated prebuilt branch from the current device tree.
Before changing anything, it copies the complete `/data/openpilot` directory to
`/data/openpilot_bak` with `cp -a`. The original tree is restored automatically
after a successful push, a build error, or Ctrl-C.

```bash
cd /data/openpilot

# Check paths, branch name, and GitHub authentication without changing anything.
# A GitHub token is requested without displaying the input.
release/build_carrot.sh --dry-run

# Prompt for a token, push carrot_vYYMMDD, and restore automatically.
release/build_carrot.sh

# Use an already configured Git credential without prompting.
release/build_carrot.sh --no-prompt-token

# Use an explicit name when more than one release is needed on the same day.
release/build_carrot.sh --branch carrot_v260717_2
```

The configured `origin` push URL is used by default. It can be overridden for a
single run without modifying the restored checkout:

```bash
RELEASE_REMOTE_URL=https://github.com/ajouatom/openpilot.git release/build_carrot.sh
```

Token prompting is enabled by default and uses a temporary `GIT_ASKPASS` helper.
`--prompt-token` remains available as an explicit alias. The
token is not written to the repository, backup, Git configuration, remote URL,
or shell history, and the helper is deleted on exit. For unattended use,
`GITHUB_TOKEN` is also accepted, but exporting it separately is safer than
placing it inline in the command:

```bash
read -rsp "GitHub token: " GITHUB_TOKEN; echo
export GITHUB_TOKEN
release/build_carrot.sh
unset GITHUB_TOKEN
```

Use a fine-grained token with repository `Contents: Read and write`, or a
classic token with `repo` scope.

If the device loses power or the process receives `SIGKILL`, the exit trap cannot
run. The backup is intentionally left untouched. Recover it with:

```bash
/data/openpilot_bak/release/build_carrot.sh --recover
```

The script refuses to overwrite an existing backup and refuses to run while the
vehicle is onroad. An existing same-day release branch is updated only with
`--force-with-lease`, so a concurrent remote update is not overwritten.

```
## release checklist

### Go to staging
- [ ] make a GitHub issue to track release with this checklist
- [ ] create release master branch
  - [ ] create a branch from upstream master named `zerotentwo` for release `v0.10.2`
  - [ ] revert risky commits (double check with autonomy team)
  - [ ] push the new branch
- [ ] push to staging:
  - [ ] make sure you are on the newly created release master branch (`zerotentwo`)
  - [ ] run `BRANCH=devel-staging release/build_stripped.sh`. Jenkins will then automatically build staging on device, run `test_onroad` and update the staging branch
- [ ] bump version on master: `common/version.h` and `RELEASES.md`
- [ ] post on Discord, tag `@release crew`

### Go to release
- [ ] before going to release, test the following:
  - [ ] update from previous release -> new release
  - [ ] update from new release -> previous release
  - [ ] fresh install with `openpilot-test.comma.ai`
  - [ ] drive on fresh install
  - [ ] no submodules or LFS
  - [ ] check sentry, MTBF, etc.
  - [ ] stress test passes in production
- [ ] publish the blog post
- [ ] `git reset --hard origin/release-mici-staging`
- [ ] tag the release: `git tag v0.X.X <commit-hash> && git push origin v0.X.X`
- [ ] create GitHub release
- [ ] final test install on `openpilot.comma.ai`
- [ ] update factory provisioning
- [ ] close out milestone and issue
- [ ] post on Discord, X, etc.
```
