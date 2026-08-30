# Repository memory

- For long-running work, treat user questions, status checks, clarifications, and added in-scope
  requests as interruptions to answer while continuing the active work. Stop an active process or
  abandon the task only when the user explicitly asks to stop, cancel, pause, or replace it.
- While the remote branches exist, apply and push each user-requested `carrot-wip` code change to
  all three branches: `carrot-wip`, `carrot-egpu`, and `carrot-egpu-tg`. Stop mirroring a branch
  when it is removed from the remote.
- Apply and push every shared big-model or eGPU infrastructure/fix commit to every existing remote
  branch whose name starts with `carrot-egpu`. Preserve each experimental branch's intentionally
  different model selection or payload unless the user explicitly asks to change it. Stop
  mirroring only branches that have been removed from the remote.
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

# Carrot Dashcam Upload route lookup

- When the user provides a `Carrot Dashcam Upload` block, resolve the local route before doing a
  broad search. Build the preferred log path as
  `\\DS1821P\openpilot\routes\<Car name> <DongleId>\<Result segment>\rlog.zst`.
- Decode presentation escaping before building the path: Markdown `\_` is `_`, URL `%20` is a
  space, and the result link's final directory name is the route segment.
- If the exact path is absent, search `\\DS1821P\openpilot\routes` for a vehicle directory ending
  in the exact dongle ID and then the exact result segment. Prefer `rlog.zst`; use `qlog.zst` only
  when the full log is unavailable.
- Treat Upload Time, Branch, and Commit as incident-analysis metadata, not as path components.

# User documentation policy

- Do not create or edit files under `docs/user/ko/` or `docs/user/en/` unless the user explicitly
  requests user-documentation work. User-visible code changes alone do not authorize guide edits.
- Keep setting-level explanations in the generated GitHub Wiki workflow and web-only explanations
  in the localized UI instead of duplicating them into `docs/user/` by default.
- `docs/user/docs_map.json` and `tools/docs/check_user_docs.py` are validation aids, not instructions
  to generate documentation. For an ordinary code pull request without explicitly requested docs,
  record a concrete `Docs-Not-Needed: <reason>` in the PR body when the workflow requires it.
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
