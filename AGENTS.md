# Repository memory

- Whenever radar detection or lead-selection code changes, update the NAS Carrot Routes
  radar replay service in the same task. The `Carrot Routes image` GitHub workflow builds
  committed shared code using `tools/carrot_route_vault/build_bundle.py`; the NAS scheduled
  updater pulls, validates and deploys the tested image to the existing
  `/volume1/docker/carrot-route-vault` project. Use this automatic path for routine updates;
  do not manually copy sources or rebuild on the NAS. Verify the workflow and NAS updater
  report the intended commit, and verify an actual upload-result page and
  its recalculated radar data before reporting completion. This includes replay adapters and
  shared detection dependencies such as `radar_motion`, `cluster`, cut-in helpers, and required
  cereal/DBC compatibility changes. Keep the server's code fingerprint/cache invalidation and
  deployed `SOURCE_COMMIT` current; a vehicle-branch push alone does not complete this work.
- For long-running work, treat user questions, status checks, clarifications, and added in-scope
  requests as interruptions to answer while continuing the active work. Stop an active process or
  abandon the task only when the user explicitly asks to stop, cancel, pause, or replace it.
- Apply model selection/artifacts, model and branch names, required model compatibility changes,
  and branch-specific features (such as YOLO2) only to their relevant branches. Preserve these
  differences when synchronizing shared code; do not spread an experiment to other branches.
  The user will explicitly identify new feature experiments and their target branches.
- As of 2026-09-13, `carrot-wip` is the sole maintained top-level `carrot-*` branch.
  It incorporates the complete `carrot-cinque_v2` history and uses its pinned Cinque v2
  eGPU big model from commaai/openpilot#38823. The internal-GPU fallback model is unchanged.
  Commit and push common changes, including radar processing and Carrot Web, to `carrot-wip`;
  verify it matches `origin/carrot-wip` with no unpushed commits before completion.
  Do not recreate retired branches or synchronize changes to their archive tags or detached
  worktrees. Retired local branch tips are preserved under `archive/2026-09-13/<branch>`.
  Namespaced contributor branches such as `thftgr/carrot-*` are outside this consolidation.
- Create new model or feature experiment branches from current `carrot-wip` only when the user
  explicitly requests them. Keep their model selections, generated display assets, compatibility
  changes and dedicated features scoped to those experiments; agree their maintenance scope
  with the user instead of automatically restoring the retired multi-branch synchronization rule.
- On this Windows workstation, vehicle tmux session captures are stored under
  `\\DS1821P\openpilot\<branch>`. When tmux is mentioned, search the directory for the known
  branch for a vehicle folder whose name ends with the exact dongle ID. If the branch is unknown,
  search `\\DS1821P\openpilot` across branch directories for the exact dongle ID first. Do not
  start by looking for a local Windows or WSL tmux installation.
- Big-model ONNX files and manifests are hosted on the user's NAS under
  `\\DS1821P\openpilot\models\<model-directory>`. Vehicles download the same files through
  `https://upload.shind0.synology.me/models/<model-directory>/`. For a new big-model branch,
  create a distinct model directory, place the verified ONNX and `manifest.json` there, and point
  the branch at that NAS manifest. Do not use GitHub LFS as the vehicle download source.
- Navigation deceleration behavior for the `origin/thftgr/navi-stream` branch is documented in
  `docs/carrot_navi_7713_7714_deceleration.md`.
- The 7714-only control comparison between `origin/carrot-wip` and `origin/thftgr/navi-stream` is
  documented in `docs/carrot_navi_7714_branch_comparison.md`.
- When changing the UDP 7713 legacy navigation path, TCP 7714 Carrot Navi v2 path, shared
  `CarrotServ` speed selection, or the on-road/cluster speed-source UI, update that document and
  the focused tests together.
- Both `origin/carrot-wip` and `origin/thftgr/navi-stream` evaluate a new 7714 speed item before
  assigning the current `lane_current.road_category`. A present lane item without that key becomes
  category 0. This can suppress primary SDI type 22 (`xSpdType=-1`) even though raw 7714 UI shows it;
  7713 assigns `roadcate` before `_update_sdi()` and does not have this ordering failure.
- The `origin/thftgr/navi-stream` cluster road-camera/map flicker analysis is documented in
  `docs/cluster_road_camera_map_flicker_analysis.md`. The hardware H.264 map and TICI road camera
  both use `samplerExternalOES` on texture unit 0; each external image must be rebound immediately
  before every draw, not only when its source frame changes.

# Vehicle settings snapshots

- On this Windows workstation, uploaded vehicle settings are stored under
  `W:\<branch>\<car-fingerprint> <dongle-id>\toggles-YYYYMMDD-HHMMSS.json`.
- To find a vehicle's most recent settings, first search all of `W:\` for directories whose names
  end with the exact dongle ID. Gather `toggles-*.json` from every matching directory and select the
  file with the newest timestamp encoded in its filename.
- A dongle can appear under several branch or fingerprint directories. For incident analysis,
  narrow the matches using the branch and car fingerprint from the route/upload metadata, then
  inspect the newest snapshot at or before the incident time and compare it with the newest later
  snapshot.
- Treat the JSON values as raw Params values; for example, `StoppingAccel` is stored in hundredths
  of m/s^2.

# Vehicle route log lookup

- On this Windows workstation, vehicle route logs are stored under `\\DS1821P\openpilot\routes`. Whenever
  the user mentions an `rlog`, a route log, or a vehicle log, start by searching that share for a
  vehicle directory whose name ends with the exact dongle ID or device ID. The vehicle directory
  name identifies the car fingerprint; its child directories identify the route/segment numbers.
  Do not start by searching the repository, tmux captures, or another route root.

- A `Carrot Dashcam Upload` result is always uploaded under `\\DS1821P\openpilot\routes`. When the user
  provides a `Carrot Dashcam Upload` block, resolve that local route first and do not start from
  the remote result link, the repository, tmux captures, or another route root. Build the exact
  preferred log path as
  `\\DS1821P\openpilot\routes\<Car name> <DongleId>\<Result segment>\rlog.zst`.
- Decode presentation escaping before building the path: Markdown `\_` is `_`, URL `%20` is a
  space, and the result link's final directory name is the route segment.
- If the exact path is absent, search `\\DS1821P\openpilot\routes` for a vehicle directory ending
  in the exact dongle ID and then the exact result segment. Prefer `rlog.zst`; use `qlog.zst` only
  when the full log is unavailable.
- Treat Upload Time, Branch, and Commit as incident-analysis metadata, not as path components.

# Vehicle rlog decoding

- For full rlog analysis, use the full OpenPilot cereal schema. Prefer
  `openpilot.tools.lib.logreader.LogReader`; do not use
  `opendbc.car.logreader.LogReader(..., only_union_types=True)` to determine which services are
  present. The opendbc reader loads the reduced `opendbc/car/rlog.capnp` schema and silently drops
  services unknown to it, which can make `carState`, `carrotMan`, `navInstruction`,
  `navInstructionCarrot`, `controlsState`, and `carControl` appear absent.
- On Windows, if importing `openpilot.tools.lib.logreader` fails because of platform-only
  dependencies such as `fcntl`, decompress the `.zst` file with `zstandard` and parse it directly
  with `openpilot.cereal.log.Event.read_multiple_bytes`. Use `opendbc.car.logreader` only for an
  intentionally CAN-only inspection.
- Before reporting that a service is missing, count message types with the full cereal schema and
  inspect at least one expected service value. When calculating segment-relative time, use the
  first message of the analyzed service (for example `can` or `carState`) rather than `initData`,
  whose boot-time timestamp can make later segments appear cumulatively longer.

# User documentation policy

- Do not create or edit files under `docs/user/ko/` or `docs/user/en/` unless the user explicitly
  requests user-documentation work. User-visible code changes alone do not authorize guide edits.
- The user has explicitly requested that every user-visible setting addition, removal, or behavior
  change update the relevant Korean and English user guides in the same change. Treat setting work
  as user-documentation work, keep the catalog summary and detailed guide synchronized with the
  implementation, and run the user-docs validator.
- Also keep setting-level explanations in the generated GitHub Wiki workflow. Keep explanations
  specific to web-only features in the localized UI instead of duplicating web internals into
  `docs/user/`.
- `docs/user/docs_map.json` and `tools/docs/check_user_docs.py` are validation aids, not instructions
  to generate documentation. For an ordinary code pull request without explicitly requested docs,
  record a concrete `Docs-Not-Needed: <reason>` in the PR body when the workflow requires it.
  For direct pushes, put the reason in each affected commit message; it only exempts that
  commit. Settings-related rules remain required; other mapped changes produce review advice
  and do not authorize unsolicited guide edits. Settings behavior changes still require the
  relevant Korean/English guides and Wiki explanations, even when outside mapped paths.
- Do not place private, internal-only, credential-bearing, or non-public feature documentation in
  `docs/user/` or link it from the public Wiki.

# Settings Wiki authoring

- Before editing generated settings Wiki content or its generator, read
  `tools/docs/wiki_settings/AUTHORING_GUIDE.md` completely.
- In an existing generated Wiki setting page, edit only the matching `CARROT:MANUAL` region.
  Preserve every `CARROT:*` marker and never hand-edit `CARROT:AUTO` content.
- Verify behavior against the current `carrot-wip` implementation instead of inferring it from the
  parameter name. Run the Wiki validator and focused generator tests after editing.
- Generated setting pages carry the same authoring-guide URL in a hidden `CARROT:AUTHORING` marker
  so an agent working from the Wiki checkout alone can discover the canonical rules.
