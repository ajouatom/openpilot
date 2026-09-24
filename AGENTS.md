# Repository memory

- On 2026-09-24, the user requested AGNOS updates without per-update approval:
  automatically download/install, wait and retry transient network failures,
  then reboot and continue normal startup. Both startup UIs now start the
  updater without consulting saved confirmation. Keep Wi-Fi setup accessible
  during retries, prevent duplicate workers, and retain image verification,
  inactive-slot installation and fatal-error handling. This does not change
  the required OS image/version or authorize weakening startup compatibility.
  See docs/cinque_v3_integration_20260919.md for behavior and validation limits.

- On 2026-09-24, the user requested cleanup of accumulated root `.tmp_*`
  analysis work. Local archives and an index are under
  `.analysis/archive/2026-09-24/`; they are private, ignored working data,
  not Git-tracked documentation or a remote backup. Use
  `.analysis/scratch/<date>-<task>/` for new temporary analysis, captures,
  dependency installs and Wiki staging instead of new root `.tmp_*` paths.
  At task completion, retain useful findings/reproduction evidence with an
  index in `.analysis/archive/`, then remove reproducible caches and scratch.
  Keep durable conclusions in the relevant tracked investigation document.
  Archived scripts may contain old relative paths; restore their original
  layout in scratch and adjust paths before running them. For radar lead
  validation, pass `--cache-dir .analysis/scratch/radar-validation-cache`
  explicitly to avoid the tool's legacy root cache default. Never include
  local captures, settings snapshots or credentials in commits.

- On 2026-09-24, parked C4 exposure A/B/A on original bt1 reproduced isolated
  driver-camera gaps of 95.671/95.653 ms when switching OS04C10 exposure
  2298 -> 2309. CSID hardware timestamps also gap by 95.677/95.656 ms;
  both other streams stay near 50 ms. Four historical isolated wide gaps have
  the same near-limit exposure-byte crossing and three-frame command offset.
  Grouping exposure/gain writes with manual delayed group-0 launch completed
  600 s / 12,000 frames per camera without a >75 ms gap (maximum57.112 ms).
  A separate brightness trial verified commands affect actual image statistics.
  Camera-only A/B/A had stale model IPC and does not establish model validity;
  a separate 600 s normal-AE/DM/eGPU run after reboot had 12,000 camera/model/
  pose/DM messages each, no invalidity or model skips, camera max57.540 ms and
  accelerometer/gyro max ages34.973/35.796 ms. DisableDM restored to2. Parked C4
  validation does not establish loaded driving or C3 behavior.
  No thresholds, exposure limits, priorities or model behavior are weakened.
  This hazard predates September19; the recent frequency change and BT causality
  remain unproved. f8d--3 is a distinct low-exposure IFE error-signalled fence,
  not a wait timeout, and is not shown fixed by grouping. Radar work did not
  spike before either incident type. The bt2 OS trial remains separate and has
  not been installed. See docs/os04c10_exposure_investigation.md.

- On 2026-09-23, Ioniq 5 C4 `00000594--abf5912e57--7` on 1fbfe331 reproduced
  a 102.044 ms wide-camera BOOT_TS gap with consecutive raw-derived frame and
  request IDs, one skipped model input and about 304 ms invalid pose inputs.
  BOOT_TS is sampled in kernel SOF handling, not an independent sensor clock;
  do not claim a physical sensor or UI cause from this log. Separately, startup
  expected a 25 ms driver offset although bundled Panda still drives all FSIN
  channels in phase from TIM1. Driver staggered_sof is now false, retaining
  the strict startup tolerance and all runtime validity/scheduling policies.
  Passive SOF/receive timing logs are bounded to one per second per camera.
  The startup fix is not a demonstrated fix for the later driving gap; C3/C4
  target validation remains required. See docs/camera_sof_gap_20260923.md.

- On 2026-09-23, Group1 video/CAN comparisons showed reversed front lateral
  coordinates on one Tucson and one Sportage, but normal left/right on a
  Staria; another Sportage was inconclusive. Do not infer upside-down mounting
  or automatically invert a whole model/group. The user approved RadarTrackFlip:
  default normal, manually invert frontRadar yRel/yvRel per vehicle at the next
  onroad start. Preserve SCC/corner/vision and scheduling. liveTracks records
  radarTrackFlipped; replay must avoid double inversion and preserve recorded
  leads. NAS recorded/normal/flipped choices are analysis-only. See
  docs/radar_track_flip.md for offline verification and vehicle-validation limits.

- On 2026-09-23, the user approved the parked display-placement candidate:
  onroad main UI core6 and USB cluster core7, both SCHED_OTHER/nice19;
  offroad both return to cores0..3 before big-core power saving. This supersedes
  the UI-little placement below. Keep camera/control/model/radar priorities and
  placement unchanged. Apply the display policy to all workers, including the
  software encoder child. USB render/encode/controller rate is fixed at10 FPS,
  or5 while UsbGpuActive; removed ClusterHudLiveFps/ClusterHudCoreMode and legacy
  FPS/core environment overrides must not restore custom placement/rates.
  DM-enabled parked UI6/cluster7 measured core2/6/7 means61.7/77.6/85.2%, UI19.77Hz,
  road/wide max48.73/49.02ms, no camera/model/DM gaps or pose/CAN invalidity.
  Core7 still briefly reached100%; C3, loaded driving and actual ignition-off
  hotplug remain unvalidated. Isolated device syscalls verified nice19 and
  core6/7-to-little restoration for threads and a child under comma credentials.
  See docs/camera_core5_trial.md for comparisons and limits.

- On 2026-09-22, the user reported near-idle core6 and busy cores4/5 after
  camera reservation and requested balanced placement. Whole-core /proc/stat
  and deviceState, including background work, confirmed core4 about91%.
  Earlier selected-process CPU sums did not establish total core headroom.
  New parked C4 A/B/A trials place controlsd/selfdrived together on core6
  FIFO53 with camerad SCHED_OTHER. Keep planner/radarcan core4 FIFO51,
  card core5 FIFO53, radard core5 FIFO51, model/DM core7, and UI little/normal.
  This supersedes the camera-exclusive and control-core4 placements below.
  Both-controls trial core4/5/6/7 means were59/63/54/48%; camera road/wide max
  rose to47.261/49.151ms while planner work max fell to8.613ms and radar input
  age max to13.360ms. No model/pose/CAN failure occurred. Core4 still briefly
  reached100%; parked timing is not proof of loaded driving or C3 behavior.
  Follow-up temporarily enabled DM:90s yielded1804 valid DM frames with no
  DM/driving-model skips or pose/CAN failures, whole-core means61/71/46/69%,
  road/wide max51.370/52.838ms. DisableDM was restored to2 afterwards.
  Preserve priorities and validity thresholds. See docs/camera_core5_trial.md.

- On 2026-09-22, after three parked C4 grouping comparisons, the user approved
  leaving camerad/camera IRQ on core6, moving card to core5 FIFO53 with radard
  FIFO51, and moving planner to core4 FIFO51 with radarcan below the unchanged
  controlsd/selfdrived FIFO53. This supersedes the card/planner placements in
  earlier entries. Camera/UI remain SCHED_OTHER; model/DM stay on core7.
  The selected 60-second trial reduced road/wide maximum ages to 36.889/37.335ms
  but increased planner maximum work from about 8ms to 19ms and radar input
  maximum age from about 15ms to 25ms. No CAN/pose/model-gap failure occurred.
  DM was disabled; loaded driving, DM-enabled behavior and C3 are unvalidated.
  Core6 is reserved by application placement, not free of all kernel/IRQ work.
  Preserve pose limits and radar semantics. See docs/camera_core5_trial.md.

- On 2026-09-22, parked Ioniq 5 C4 follow-up reproduced a 91.961ms road-camera
  delay, model input frame gap and invalid odometry/pose inputs with ftrace off;
  IMU ages stayed below 34ms. Live boot args isolate only cores6..7. Kernel
  tracing on core5 verified ready-camera scheduling delays behind normal
  proclogd/kswapd work and FIFO planner/radard. The high-reclaim trace also had
  diagnostic tmpfs overhead; do not treat its frequency as an unperturbed result.
  The camera/IRQ core5 trial below is rolled back to core6, retaining UI on
  cores0..3 and camera/UI SCHED_OTHER. A short 5/6/5 parked comparison reduced
  core6's observed tail/runqueue wait but added about 3ms mean camera age.
  A nice=-10 trial did not materially improve mean/p99; keep nice0. Preserve
  other process placements and pose limits. This is a measured mitigation, not
  proof of a driving fix, C3 benefit or the cause of earlier SOF/IFE faults.
  See docs/camera_core5_trial.md for evidence and limitations.

- On 2026-09-22, PV5 follow-up `0000022a--99e06b0cc0--17` disproved
  interpreting A-CAN 0x380 bit 6 falling as camera passage: roughly 4.7 s
  notification pulses ended while MapSource=2 and a 30 km/h camera still
  had 139 m of tracked distance. Do not use that edge or byte value 0x04 as
  proof of passage. Retain a matched PV5 camera only while fresh messages
  confirm the same MapSource=2 enforcement limit. End/change/invalidity or
  more than one second of message loss clears current and queued cameras;
  cached profiles cannot reinsert them. Queued previews alone never authorize
  PV5 camera control. Consume a completed camera's unchanged map warning;
  a new warning without a new matched profile uses a separate virtual distance.
  PV5 does not decode current-route/position 0x4B9/0x4B4: do not claim that
  generic route-reset code detects its departure. If stock enforcement remains
  unchanged after a turn, this cancellation cannot detect that turn independently.
  Follow-up replay reconstructs initial targets from logged distances because
  the preceding segment is unavailable; current cancellation drops those old
  previews and uses the warning's virtual distance at the reported pulse end.
  Cancellation/recovery/completion tests are synthetic, not vehicle validation.
  Earlier bit-transition tests did not establish physical passage semantics.

- On 2026-09-21, after Ioniq 5 C4 `00000f90--96d7dcd525--4` reproduced a
  101 ms wide-camera SOF gap, the user authorized a CPU-placement trial:
  main UI uses cores0..3 with SCHED_OTHER (core0 bootstrap), camerad and its
  camera IRQ targets move from core6 to core5. card remains core6 FIFO53;
  planner/radard remain core5 FIFO51 and camera keeps normal scheduling.
  Preserve the UI's verified SCHED_OTHER contract and pose validity limits.
  This supersedes the camera/UI placements described in older observations,
  not radar isolation or cluster affinity. No C3/C4 vehicle benefit is yet
  validated; do not claim same-core contention caused the camera fault.
  See docs/camera_core5_trial.md for scope, trade-offs and validation.

- On 2026-09-21, ID.4 replay showed that adding CP.radarDelay (0.8 s) to
  distance alignment could switch the selected lead to a farther CAN object.
  The user approved zero extra distance projection for VW MEB. Use the shared
  radar_motion/timing.py policy in runtime and NAS replay; preserve measured
  camera/publication skew. Do not also zero CP.radarDelay: its ego-history
  compensation and velocity/acceleration effects have not been recalibrated.
  Other platforms retain their existing delay. See
  docs/meb_radar_distance_alignment.md for scope and regression evidence.

- On 2026-09-21, K9 C4 logs reproduced locationd timing-check invalidity from
  repeated IMU timestamps over 100 ms old. Historical captures first showed
  these failures after the September 19 update, despite unchanged HUD 10 FPS,
  cores 1..4 and FIFO 10. The user authorized normal SCHED_OTHER scheduling for
  cluster autorun/render workers so sensord FIFO 1 and other realtime work
  take precedence. Keep legacy ClusterHudPriority/environment overrides from
  restoring FIFO; retain FPS and core selection. This is a contention mitigation,
  not a proved fix for the OS/runtime regression. Do not weaken pose validity
  thresholds or claim vehicle validation from desktop tests. Official 521db4c
  changes initial gyro-bias covariance, not the observed sensor timestamp delays.

- On 2026-09-21 the user authorized radar optimization and preprocessing
  isolation to reduce card/camerad contention on core6, with mandatory radar
  regression validation. RadarInterface/liveTracks now belong to radarcan on
  core4 FIFO51 (below controlsd/selfdrived FIFO53); card remains core6 FIFO53,
  model-driven radard/planner core5. Preserve carState.radarInput batch metadata
  and non-conflated CAN/ego joining: using an arbitrary latest ego sample breaks
  delay/filter cadence. Keep planner's existing fast liveTracks path during this
  first isolation step. See docs/radar_process_isolation.md for equivalence,
  corpus failures and limits. C3/C4 device timing/camera improvements are NOT
  yet validated. Never present same-core contention as a proved IFE root cause
  or desktop speedup as a vehicle result. Radar changes also require NAS replay
  deployment and actual result verification below.

- On 2026-09-20, EV9 `3eef70e8fb92485c` (tizi/C3 family) reproduced Cinque v3
  dropped-frame odometry invalidity even with `xiaoge_data` stopped. Raw-image
  upload averaged 24.75 ms and model execution 50.69 ms; C4 `07b62e389ed26c81`
  used the same artifact at 13.94 ms upload / 39.73 ms execution. Old TG code
  warped on QCOM before transferring model-sized images, whereas generic v3
  uploads full NV12 images before AMD warp. The user approved C3-only QCOM
  pre-upload warp while retaining the existing C4 path. See
  `docs/c3_preupload_warp.md` for implementation, validation limits and evidence.
  EV9 segment `000002c9--15d447d91b--0` on 9a349b60 failed the QCOM/AMD pixel
  comparison and fell back to AMD (7,471,616 USB bytes, about 24.8 ms upload).
  At that stage the optimization was NOT confirmed active. Diagnose the
  per-probe mismatch details before changing warp math or acceptance criteria.
  Follow-up `000002ca--50469cb155--0` on 820f82ea found 16 repeat-stable
  projective-only mismatches; all eight logged samples reconstruct as adjacent
  source pixels at half-pixel rounding boundaries. Validation now checks each
  mismatch against correct-camera/plane NV12 source values within 0.00025
  source pixels of a rounding boundary. Do not replace this with a percentage
  or intensity tolerance. On 2026-09-21, EV9 `000002cc--03d0a44f7d--10`
  on 2723a8eb confirmed QCOM active: 393,728 USB bytes, 5.36 ms upload,
  34.98 ms mean model execution. Four remaining warnings matched complete
  camera streams with 12.6-13.2 ms SOF skew: Carrot's strict 10 ms pairing
  discarded four main frames. Current pairing allows at most 20 ms skew;
  metadata replay retains all 1,200 EV9 pairs while preserving real Ioniq
  phase-slip/IFE-loss gaps. This is not on-device validation of the pairing fix.
  Official v3 also publishes invalid odometry after a real main-frame gap;
  do not describe this policy as a Carrot-only regression. Official pairing
  logs >10 ms skew but proceeds; Carrot still bounds large/stale pairs.
  Preserve official model input/outputs and recurrent state; never hide overload
  by weakening pose validity. Evaluate model/runtime updates per device family;
  do not assume C4 validation covers C3, or automatically freeze all C3 models.
  Keep World Model experiments local and apply its separate validation rules.

- As of 2026-09-20, the user requested deletion of the remote `carrot-worldmodel`
  branch to prevent others from installing an unfinished experiment. Keep this
  experiment local only; do not recreate or push its remote branch unless the
  user explicitly authorizes publication again. Continue applying common
  `carrot-wip` changes locally while preserving World Model-specific artifacts
  and runtime work. Only `carrot-wip` must be pushed for shared changes; this
  exception does not restore any retired branch. World Model has passed isolated
  synthetic inference, but vehicle control integration remains unvalidated.

- As of 2026-09-19, the user requests full integration of `carrot-cinque_v3` into
  `carrot-wip`, including the pinned Cinque v3 eGPU model/runtime, AGNOS
  `19.8-carrot-bt1`, and Bluetooth remote features. This supersedes the earlier
  Cinque v2/OS separation below. Keep the internal-GPU driving model unchanged;
  driver monitoring uses official Super Leicht (#38942). After successful
  integration the user explicitly retired `carrot-cinque_v3`; `carrot-wip` is
  the sole maintained top-level `carrot-*` branch. Do not recreate v3 or push
  changes to its detached worktree. Its complete history is merged into wip.

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
  It incorporates the complete `carrot-cinque_v2` history. Its former Cinque v2
  selection was superseded by the 2026-09-19 integration above.
  Commit and push common changes, including radar processing and Carrot Web, to `carrot-wip`;
  verify it matches `origin/carrot-wip` with no unpushed commits before completion.
  Do not recreate retired branches or synchronize changes to their archive tags or detached
  worktrees. Retired local branch tips are preserved under `archive/2026-09-13/<branch>`.
  Namespaced contributor branches such as `thftgr/carrot-*` are outside this consolidation.
- Create new model or feature experiment branches from current `carrot-wip` only when the user
  explicitly requests them. Keep their model selections, generated display assets, compatibility
  changes and dedicated features scoped to those experiments; agree their maintenance scope
  with the user instead of automatically restoring the retired multi-branch synchronization rule.
- The 2026-09-17 exception maintaining `carrot-cinque_v3` separately ended on
  2026-09-19 after its complete integration and the user's explicit deletion request.
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
