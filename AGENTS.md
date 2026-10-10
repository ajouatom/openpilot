# Repository memory

- On 2026-10-10, user requested trying and verifying actual image-layer driving
  ONNX fine-tuning. Trained original final vision conv/SE/projection (2,241,600
  weights), optionally original temporal encoders/attention/MLP; early CNN and
  final heads frozen. Four400-step and four4000-step candidates; all fail
  development. Regenerate each candidate's full consumed image history; x/v/a
  gradients reach camera weights. Original graph/IO unchanged, auxiliary color
  head discarded. 44,400-frame original numerical parity and four full ONNXs
  on259 real feeds each pass calculation checks. Selected failed807f758c keeps
  major incident Go counts88->89 and19->20; brief third1->0. One trained green
  predicate response improves1.575->0.868s, not physical launch validation.
  Reused reviewed data, no metric stop-line targets, full-controller/vehicle
  simulation or QCOM compilation. No vehicle/model/control/observer deployment.
  Do not say original vision fine-tuning has never been tried, impossible, or
  these failed models are ready. See docs/signal_vision_finetune_20261010.md;
  retain private artifacts under the matching local analysis archive.

- On 2026-10-10, user correctly noted earlier ONNX failures predated the new
  night drive and requested retry. Replayed37 segments/44,400 road+wide pairs,
  trained on new1091 night plus old day data, and exported six original driving
  ONNX candidates. Compare final75 future x/v/a rows with existing nonlinear
  plan-branch fine-tuning plus discarded auxiliary color head; image/shared
  temporal backbone stays frozen, graph/IO unchanged. All six fail development.
  One night red error improves0.563->0.495m/s in broader fit, but another red
  worsens0.364->0.753 and plan lateral outputs drift. Selected failed candidate
  a68a29bf retains two incident Go-frame counts88->88 and19->19; third1->0 is
  not vehicle-response proof. Two full ONNXs verified on259 feeds each; nonplan
  outputs exact. Recorded stopping method matches318/318 incident states.
  No surveyed stop-line targets, full-controller simulation, QCOM compilation
  or vehicle installation. Keep original controller/model and private evidence
  local. Do not call this new data untested or model fine-tuning impossible.
  See docs/signal_night_finetune_20261010.md.

- On 2026-10-10, user asked to use the improved signal version now. Fresh vehicle
  read confirms b57b81835d observer already installed and parked/inactive. A new
  decision-availability comparison does not validate direct control promotion:
  125ms result delay plus measured frame gaps can expire red at one stopped
  incident; requiring scene green has about5.2s confirmation in one encounter
  and no green over its reviewed6.4s window with the parked gap pattern. This is
  not full-controller/closed-loop simulation or actual departure-delay proof.
  Retain improved observer; no control hook/hold latch or new ONNX was installed.
  See follow-up in docs/signal_night_observer_20261010.md; keep incident data local.

- On 2026-10-10, new night logs exposed all11,061 observer outputs unknown.
  Added saturated-core/color-halo night proposals and125ms tracking continuity.
  Reflection rejection and image-only processing; original ONNX/x/v/a, departure
  thresholds and control unchanged. Reused night scoring:0/7427 red-as-green,
  520/688 correct green; recorded cadence0/5704 and339/540. Side red plus frontal
  green still abstains; ego-lane association remains unresolved. Prior1447
  day/dusk decisions unchanged;42 desktop tests pass. Optimized parked trial:
  42.7ms median work,302/302 fresh observations,640/640 valid model frames/no gaps.
  Three red acceleration windows include lead-mode exit while still red and
  x[-1] about1.4m, followed by a model-speed surge. Exact stopping method matches
  318/318 traffic states, not whole-controller or network causal proof. Logging
  only, not false-departure prevention. See docs/signal_night_observer_20261010.md;
  installed b57b81835d on parked C4:600/600 valid model frames/no gaps,395/395
  fresh observer events at11.98Hz, little/nice19 threads and98 saved rlog events
  joined to camera frames. Retain private incident evidence locally.

- On 2026-10-10, user requested live installation of the automatic signal
  tracker. Existing opt-in signalcolord dispatches to signal_tracking_shadow
  only with the additional local tracking_enabled flag. Same causal tracker,
  road NV12, diagnostic logs only; no control or departure-threshold changes.
  Little CPUs0..3/nice19, OpenCV single-thread, max20Hz and50% one-CPU duty
  target. Actual parked trial averages15.5 observations/s:388 fresh unknown
  observations, model640/640 valid at20Hz without gaps. 36 desktop tests and
  OpenCV4.13 equality on1447 recorded frames pass. Do not claim stopping or
  false-departure prevention. Installed/enabled b83c003d73 on the parked C4;
  automatic worker verified600/600 valid model frames at20Hz/no gaps,518 fresh
  tracking records at15.67Hz, all3 threads little/nice19;26 saved rlog entries
  match recorded camera IDs. No reboot needed. Old color latest.json is history;
  use tracking_latest.json/events for the selected tracker. See docs/signal_tracking_live_20261010.md;
  retain private installation and validation evidence locally.

- On 2026-10-10, completed an offline automatic horizontal-signal detector,
  causal tracker and bounded lamp-state observer after the user requested a
  concrete result. School development then frozen six-clip replay gives
  red-as-green 0/1043, correct green361/404, unknown58/1447; four first-green
  delays0.155..0.205s. Clips were previously reviewed, not unseen validation.
  Eight temporal tests and a20-frame generic CLI replay pass. Private report
  and six annotated videos remain local. This iteration is image-processing
  engineering, not new ONNX training. No ego-lane/arrow assignment or stop-line
  distance; startup green is unarmed. User also wants distant-red approach
  learning: document sequence/lane/stop-line labels and separate stopping
  evaluation, without claiming these are implemented. Original x/v/a,
  departure thresholds and vehicle configuration remain unchanged. User
  prioritizes red stopping and preventing false departures: report red-as-green
  errors separately; unknown is not confirmed green. Do not claim offline
  observer success prevents actual vehicle departures. See
  docs/signal_tracking_trial_20261010.md.

- On 2026-10-10, user requested object-location learning and asked about lamp
  brightness/history. Added75 manual housing boxes (73 eligible:21/17/35),
  trained two tile detectors with class+box losses and exported private ONNXs
  bac2f128/41d3a430; each matches Torch on3456 augmented inputs. Scratch detector
  fails; frozen-color-plus-learned-residual improves development green16/33 to
  23/33 but adds road-heldout false greens2/66 and wide26/34. Top5 correct-color
  target recall atIoU0.3 is5/17 dev,13/35 reused evaluation; reject promotion.
  User's grayscale inner-lamp/3-frame transition prototype finds four first
  changes in0.099..0.400s with manually interpolated boxes (future labels assist;
  not automatic causal tracking). School green remains unstable47/145 confirmed;
  separate static red20 has2 false greens. Do not claim solved signal recognition
  or departure improvement. No original x/v/a, thresholds or vehicle changes.
  Keep models/data local. See docs/signal_object_brightness_20261010.md.

- On 2026-10-10, the user requested actual retraining on added drives. Refit the
  39-parameter camera color model and trained a 2,215-parameter color+CNN model;
  new private ONNXs 5146a59c/db3f511c match Torch decisions on all 370 inputs.
  Whole-encounter splits are 129 train/88 development/153 reused evaluation;
  all daylight/dusk school data stay outside fitting/selection. Hybrid full-scan
  evaluation improves pooled cameras134/153 to143/153, but road-only108/109
  regresses to107/109 and development green16/33 to7/33. Wide false green7/34
  drops to1/34 while green9/10 drops to6/10. Five training sign crops clear0.8
  false-green5/5 to0/5, yet their full frames still have1/5 false green (old2/5).
  School full-scan first-green sample is unchanged; ROI gains are not departure
  gains. Reject both candidates for promotion. Original x/v/a, thresholds and
  vehicle files/config are unchanged. Device replay was not run because fresh
  valid parked state was unavailable. Keep artifacts local. See
  docs/signal_dusk_training_20261010.md.

- On 2026-10-10, dusk route1090 segments64..83 were collected with 100/100
  source SHA256 matches. Segment81 school signal is green in both cameras;
  Go takes4.254..4.298s, briefly reverts, and persists at4.908..4.952s. Ego
  slows to0.182m/s, not full standstill. Segment68 waits1.690..1.740s after
  green until driver gas/disengagement; do not count it as automatic departure.
  Stop trajectories persist, then the low-speed x<20/terminal-v<10 gate can
  override an already-true start candidate. Segment66 repeats red-light Go
  with brake interventions. Replay matches1380/1380 traffic decisions in six
  windows; no thresholds/weights/device changes. Logging-only color model
  also falsely selects green electronic text/signs; it is not a solved detector.
  Added123 reviewed rows from five encounters, including five hard negatives
  and one excluded mixed transition; no new fitting. Keep overlapping-location
  and adjacent-frame splits honest. See docs/ioniq5_signal_dusk_20261010.md.

- On 2026-10-10, latest Ioniq 5 PE route 1090 segment 30 shows red-light
  reacceleration at 47.108s with x[-1]=34.458m: averaged terminal speed 4.967
  exceeds v[0]+2=4.303; five consecutive start candidates change state to Go.
  traffic off already removes the stop-model obstacle, and positive acceleration
  precedes green by 102ms. Brake disengages, then RES reenables after a second
  false Go at 51.108s. Exact stopping-function replay matches 220/220 decisions.
  Logging-only color ONNX reports red on all six sampled incident inputs and
  cannot affect control. Later actual green is visible in wide at segment31 23s
  while road remains visually dark until about26.75s; do not label that departure
  false based only on road footage. Added 82 reviewed images (50 red/12 green/20
  unknown), one encounter. Frozen color ONNX on supplied visible ROIs scores
  16/18 road, 3/44 wide at0.8; this is not blind detection or new model training.
  Need camera/scale-aware learning, localization and temporal evidence, plus
  separate encounters for validation. No weights, start thresholds or vehicle
  configuration changed. See docs/ioniq5_signal_1090_20261010.md; data stay private.

- On 2026-10-10, the user authorized connecting the color classifier to live
  driving footage. `signalcolord` is opt-in, separate, logging-only, reading road
  VisionIPC; no control/modelV2 return path. Pinned private c091fad2 color ONNX,
  667-window search, top-three candidates, frame/time/age/CPU logging. Little
  cores0..3/nice19/single-thread, max1Hz and adaptive 25%-of-one-CPU duty target;
  ordinary failures latch until disable/restart. Preserve actual wrong selections.
  Parked preinstall trial: 1301/1301 valid model frames, 20Hz, no gaps; 26 color
  events at median1.751s with six false greens in a no-signal doorway scene.
  Installed/enabled df7cc14549 on the parked C4 and verified automatic startup.
  Include launcher-created IPC threads in scheduling; all three verified on
  cores0..3/nice19. Final 700/700 valid model frames at20Hz/no gaps, 26 desktop
  and 26 device tests pass; 90 saved rlog color entries join recorded camera IDs.
  These verify recording, not detector accuracy or loaded driving performance.
  See docs/signal_color_live_trial_20261010.md for install status and limitations.

- On 2026-10-10, the user requested actual follow-through beyond failed signal
  training. Trained a 2,179-parameter RGB ROI CNN (poor generalization), then a
  39-parameter classifier on fixed pooled RGB/chroma features. Given reviewed
  ROIs, the latter scores 57/57 development and 44/44 reused heldout frames;
  separate medium-confidence school greens improve from 1/11 to 10/11. Blind
  667-window search falls to 44/57 development and 7/11 school greens, selecting
  brake lamps or other objects. Do not claim a solved detector or original x/v/a
  improvement. Both ONNXs match PC decisions on 252 recorded device inputs in
  Park; color ROI median 0.395 ms, full scan inference median 129 ms/p95 487 ms.
  Finite low-priority device test ended; no live service/control/model replacement.
  Keep artifacts private and follow up on signal location/lane association plus
  unseen encounters. See docs/signal_roi_trial_20261010.md.

- On 2026-10-10, the user requested continuing from visual labels into training.
  Trained new binary signal heads on frozen internal vision512 and policy256
  features; only the former has camera-only graph ancestry. Vision gives 46/57
  development and 44/44 high-confidence heldout correctness, but misses 11/23
  development greens and 10/11 separate dim school greens. Do not advertise the
  small heldout 100% as resolving the incident. Saved private camera classifier
  ONNX dba6728b and tiny head 033fa90e; 237-head/27-full-feed checks pass. No x/v/a
  weights, device models or start thresholds changed. Reconstructed school signal
  ROI is outside the road crop, with a small signal in wide input; this is not
  causal or bit-exact device proof. See docs/signal_visual_training_20261010.md.

- On 2026-10-10, the user requested assistant-created visual signal labels and
  questions only for unresolved scenes. The first local batch directly reviews
  237 frames from nine encounters; 158 high-confidence color/shape labels are
  exported with existing encounter splits (57/57/44). Vehicle state and model
  predictions are not label targets. Preserve uncertainty, camera frame/time
  provenance and separate user testimony from pixel observations. Two dark
  return intersections await user clarification. Data/HTML/crops remain private
  in the signal-labels archive; this is not whole-corpus labeling, model training
  or independent accuracy validation. See docs/signal_visual_labels_20261010.md.

- On 2026-10-10, the user requested collection and retraining on all recordings
  after the first observer drive. Collected 79 segments/395 files (7.209 GB) and
  replayed 67,845 dual-camera pairs from 57 segments. Balanced final-head fitting
  used 829 red, 55 recorded green-departure and 1,118 preservation samples. All
  12 development candidates failed; the saved diagnostic ONNX 91d47a89 increases
  red-wait speed error and does not resolve heldout school green delay. Full ONNX
  validation on 114 feeds preserves outputs outside 75 future longitudinal mean
  rows exactly. No new artifact was installed or enabled for control. Keep data
  and models private/local; do not promote or change signal-start thresholds on
  this evidence. See docs/ioniq5_signal_retrain_20261010.md.

- On 2026-10-10, first internal observer drive `0000108c--e6597f4447` was collected
  directly (segments 11..21, 3,300 comparison entries). Video confirms green-to-Go
  delays of 0.51..0.56 s and 2.61..2.66 s; driver gas precedes the latter Go state.
  The candidate does not resolve the delayed green response, suppresses progress
  and produces negative future velocities despite reduced pooled red-wait error.
  Keep comparison-only; do not promote or alter signal-start conditions on this
  evidence. Two return-junction stop forecasts have unresolved applicable signal
  color. Preserve private recordings locally and event-level validation separation.
  See docs/ioniq5_signal_first_drive_20261010.md.

- On 2026-10-10, the user requested a separate internal signal-model trial branch
  and installation on their connected parked C4. `carrot-signal-shadow` retains
  original control and runs a policy-only CPU observer at 5 Hz, using original
  camera features and matching temporal inputs. Candidate outputs only enter
  logMessage/rlog. The initial combined QCOM graph failed original-output parity
  and was never enabled; version 2 rejects it. The candidate still fails prior
  green-progress preservation criteria. Device tests (39), parked 40-second
  20 Hz/zero-drop observation and saved-rlog comparison entries were verified;
  recognition gains and vehicle-response validation remain unproven. Keep wip's
  model selection unchanged. See docs/signal_shadow_trial_20261010.md.

- On 2026-10-09, after trying Mountain Dew 870a4823, the user requested
  returning carrot-wip's eGPU model to Cinque v3 (892fc3a1, AMD e758b96d,
  isolated tinygrad d5e17c93). This supersedes the MDM selection below, not
  the completed branch consolidation. Retain shared HUD/camera/startup/UI
  fixes, MDM format compatibility, internal/DM models and Jetson Cinque v2.
  Do not recreate retired branches. A user-facing model selector was discussed
  as a possible follow-up, not implemented. See docs/mdm2_20261008.md.

- On 2026-10-09, the user requested full integration of `carrot-mdm2` into
  `carrot-wip` and deletion of the local and remote `carrot-mdm2`/`carrot-mdm`
  branches. `carrot-wip` now selects Mountain Dew v1 checkpoint 870a4823,
  AMD model e20cde17, with the existing pinned runtime and NAS mdm2 package URL.
  This supersedes the branch-only restriction below; do not recreate the retired
  branches. Keep the newer wip HUD/camera recovery, DM notices and GV70 fixes.
  Internal/DM models, Jetson Cinque v2 and control/validity policies remain
  unchanged. Model compatibility CI follows carrot-wip. See docs/mdm2_20261008.md.

- On 2026-10-08, the user confirmed PR #39047 commit 4bfb534063 for `carrot-mdm2`,
  starting from carrot-wip ca8f553d1e. The previous bea3fd4 remains carrot-mdm.
  Pin the new e20cde17 AMD model and 870a4823/12864 metadata checkpoint on this
  branch only; upstream's commit subject says 1284 but the file says 12864.
  Keep the 9d0446a4 tinygrad runtime and MDM compatibility adapter. The matching
  870a4823 ONNX is absent from the public export catalog; Jetson stays Cinque v2.
  Do not label this a new official MDM v2 or claim Jetson MDM support. Preserve
  internal/DM models, control, validity and C3 warp policy. Include the inherited
  boot selected-model delivery gate and core4 USB cluster placement.
  See docs/mdm2_20261008.md for artifact identity and validation limits.

- On 2026-10-05, the user clarified that instructions such as "작업해" or
  "진행해" include committing and pushing the completed task unless explicitly
  instructed otherwise. Commit only the task's changes; preserve unrelated work.

- On 2026-10-05, the user requested an offline clock floor after pull/reboot,
  before building. AGNOS startup reads local HEAD's committer timestamp before
  dependency/Params/main builds; only an earlier clock advances to commit+1s.
  Log and verify correction; failures enter existing startup recovery. Preserve
  later clocks, NTP/GPS, file mtimes, caches and Cython/SCons behavior. 26 focused
  desktop tests pass; device clock setting and native builds remain unvalidated.
  This does not establish the cause or cure of intermittent native build errors.
  See docs/build_time_floor_20261005.md.

- On 2026-10-05, the user approved a distance range for holding extra following
  headroom. Define D as ego speed times base TF plus configured stop distance,
  excluding extra TF. Preserve existing hold at <=1.2D; smoothly reduce its
  envelope/rise rate and restore recovery strength through 1.2-1.5D; at >=1.5D
  recover at the existing level rate even for a stopped/opening lead. Use the
  same rule in live state and MPC prediction. Preserve new-lead entry, level-5
  bypass, physical obstacles, base TF, stopping logic/limits and the J20 trial.
  299 focused tests pass; recorded-input reference replay reduces one approach's
  extra margin from 1.916 to 0.142 m, not a vehicle stopping-time/clearance result.
  Closed-loop comfort and clearance remain unvalidated. Keep incident data local.
  See docs/longitudinal_gap_hold_band_20261005.md.

- On 2026-10-03, the user requested a manual compatibility option for intermittent
  cluster warnings: HyundaiCanfdClusterDirectTx defaults OFF on every vehicle,
  including EV6. In CAN-FD CAMERA_SCC only, enabling it at startup selects the
  legacy direct host-TX path for 0x161/162/1e0/1ea/200 via Hyundai flag bit 27
  and Panda safetyParam 2048. Preserve other control FIFOs/reuse, allowlists,
  relay protection and the default RX-paced path. This explicitly permits
  independent cluster TX only when selected; no automatic RX timeout fallback,
  vehicle-specific default or live switching. Reboot and updated Panda firmware
  are required. 543 focused/settings/Wiki/firmware-identity tests and F4/H7
  builds pass; actual warning resolution and physical timing remain unvalidated.
  See docs/canfd_cluster_rx_forwarding.md. Keep incident data local.

- On 2026-10-03, the user explicitly approved adding core7 to C3/C3X main UI
  onroad affinity: cores0,1,2,3,6,7 with SCHED_OTHER/nice19 for all UI threads.
  This supersedes the earlier core7 exclusion for this UI. C4 stays core6;
  offroad returns to cores0..3. Check each big core independently; preserve
  model/DM, control/camera, IRQ and USB cluster policies. An allowed mask is
  not a CPU quota or parallel rendering and may concentrate UI work on core7.
  Device affinity/FPS and model/DM impact remain unvalidated. See
  docs/camera_core5_trial.md.

- On 2026-10-03, Tucson `0000030c--adf522a321--4` showed unnecessary left
  steering while passing a transporter. Actual speed stayed near 104 km/h
  while the model velocity trajectory fell to about 36 km/h; lane MPC remained
  active because the old end/start 70% test missed whole-trajectory collapse.
  The user requested diagnosis through correction. LaneModelSpeedGuard now
  also requires model starting speed >=70% of measured speed, retaining the
  original future-deceleration gate and continuous one-second reacquisition.
  Preserve model paths/speeds, MPC tuning, actuator limits and angle handover.
  26 focused tests pass; same-input target replay reduces initial left peak
  about 79% with fallback around 6.33 s. Small-angle MPC reconstruction matches
  logged mode decisions and incident curvature closely; this is not native
  acados or vehicle-response validation. Shadow versus transporter influence
  on the original model output remains unresolved. See
  docs/tucson_30c_left_steering_20261003.md.

- On 2026-10-03, the user selected existing combined handover mode 3 as
  standard for Hyundai/Kia/Genesis angle control and removed the selector.
  CarController always uses mode 3 inside ANGLE_CONTROL; SteerHandoverMode
  registration, catalog/menu and runtime reads are removed. Saved values no
  longer affect behavior. Preserve the combined algorithm, thresholds, targets,
  angle/CAN limits, torque-control paths and touch/DM. Internal helper variants
  remain for comparisons; diagnostics still identify mode 3. 83 steering tests,
  45 settings tests, 25 Wiki tests and 6,000-frame old-mode-3/new CAN equality
  pass on desktop with Windows Params storage substituted. This promotion is
  not a new retry fix or vehicle-response validation. See
  docs/steering_handover_20260930.md.

- On 2026-10-02, after the native CPU experiment and Ioniq 5 PE before/after
  logs, the user explicitly approved promotion to `carrot-wip` and deletion of
  the remote `carrot-native-cpu` branch. Keep the tested Cython radar statistics/
  path projection and CAN extraction/packing kernels, Python comparison/fallback,
  strict floating-point build flags and backend timing diagnostics. Preserve
  radar algorithms, history, thresholds, validity, counters and CPU placement.
  The discussed trajectory prefilter is deferred and must not be included.
  Ioniq 5 logs confirm native activation and core5 mean 77.7 -> 70.9%, but input
  workload differs; same-input replay matches all 2,400 radar frames with 30-31%
  lower PC compute time. EV9 overloaded mode-3 native vehicle behavior is still
  unvalidated. Maintain native x86/ARM CI and the shared NAS replay build.
  See docs/native_cpu_experiment_20261002.md.

- On 2026-10-01, Casper EV `00001e75--ace5ac2325--9` confirmed SCC-only mode 0
  with every SCC lateral measurement zero. The user requested always using the
  measured SCC object in SCC-only modes and ignoring unreliable SCC lateral
  position. Modes 0/-1 now use SCC longitudinal range/speed directly, then the
  existing probability-qualified vision lead without an extra dPath gate when
  SCC is absent. All SCC matching excludes its lateral coordinate; it cannot
  establish geometric path occupancy. Mode 2 retains low-speed longitudinal
  vision matching and independently corroborated SCC L2; mode 1 excludes SCC
  and mode 3 retains front-first/SCC fallback. Web replay restores the recorded
  source policy instead of forcing mode 2. Original mode-0 replay loses L1 on
  30/1,199 frames with SCC and strong vision; corrected replay loses none and
  uses vision on all 45 SCC-absent frames. Replay is not vehicle-response
  validation. See docs/scc_longitudinal_lead_20261001.md.

- On 2026-09-30, the user approved the handover revision and then explicitly
  selected torque-ceiling-only rapid recovery: keep target angles and existing
  angle limits unchanged, raise the ceiling faster for small error and slower
  for large error, with no angle-error entry gate. This supersedes the earlier
  captured-angle/offset-blending design proposal below. Mode 1 combines effort
  and error levels/trends with tolerance, paused increases and gradual error-only
  withdrawal; strong renewed force still yields quickly. Modes 2/3 use limited
  early capture then low-force confirmation and continuous error-dependent rise.
  Active experimental transitions own the total ceiling, so legacy max() cannot
  bypass their rate; preserve independent legacy history, mode 0 and live polling.
  88 focused tests and 6,000-frame mode-0 CAN/angle equivalence pass. Recorded-input
  schedules are not vehicle response or steering-feel validation. See
  docs/steering_handover_20260930.md for constants, replay and limitations.

- On 2026-09-30, the handover follow-up review uses both angle-error and driver-
  effort trends for mode 1, with tolerance and paused/gradual withdrawal for
  ambiguous error growth, retaining fast yield for strong renewed driver effort.
  For rapid release, onset angle error is an initial transition condition, not
  a delayed small-error permission gate. Offline prototypes avoid ff7's short
  46-to-25 withdrawal and schedule earlier limited offers, but unchanged-input
  replay cannot establish vehicle response. The existing max(legacy, extra)
  ceiling defeats bounded recovery; any implementation must coordinate total
  authority and reference transition while preserving mode 0 and actuator limits.
  This review changes no production code. See
  docs/steering_handover_ff6_ff7_20260930.md for evidence and unresolved cases.

- On 2026-09-30, Ioniq 5 PE ff6 segments 2/3 and ff7 segment 1 on b8a8a532
  confirmed live handover mode 2→3 but delayed recovery after torque release.
  Low-force confirmation can let angle error exceed the two-degree fast-recovery
  gate; other releases miss arming/deadline conditions. No rapid recovery is
  reconstructed in these windows. Legacy repeated-override ramps reach three
  seconds; ff7's last release waits 520 ms then ramps for three seconds. One
  short convergence offer is withdrawn 46→25 in about 12 ms on error growth
  despite decreasing force. This may explain a tactile discontinuity but is not
  proof of the user's exact felt moment. Touch-release edges arrive later and
  are not demonstrated to be a faster cue. No controller change was requested
  or made during this analysis. See docs/steering_handover_ff6_ff7_20260930.md.

- On 2026-09-30, the user requested original-RX-paced forwarding of Hyundai
  CAN-FD CAMERA_SCC cluster 0x161/162/1e0/1ea/200 from bus2 to bus0. Consume
  allowed host copies into independent latest-value caches; use each stock RX
  counter and recompute CRC, including byte-2 8-bit COUNTER for 8-byte 0x200.
  No independent send without RX. Missing/expired host (150 ms) returns stock;
  invalid original frames pass unchanged and invalidate the cache. Preserve
  allowlists, relay protection, non-camera paths and existing control FIFO/reuse.
  Latest-value sampling supports differing rates but can coalesce transient
  displays and delay changes until next RX; freshness bounds host arrival only.
  348 tests, 36,120-frame replay, 285,594 unchanged control comparisons and
  F4/H7 builds pass. Wire timing, vehicle warning resolution and display/chime
  behavior remain unvalidated. Requires updated Panda firmware. See
  docs/canfd_cluster_rx_forwarding.md.

- On 2026-09-30, the user requested live SteerHandoverMode for Hyundai/Kia/Genesis
  angle control: 0 preserves legacy/default, 1 offers bounded recovery using
  continuous driver effort and angle-error trends, 2 confirms abrupt force release
  before faster recovery, and 3 combines them with release priority and no summed
  gains. Poll every 0.5 seconds; only actual mode changes reset experimental history.
  Keep legacy recovery state independent, steeringPressed boolean, torque-control
  vehicles, touch/DM and angle/CAN limits unchanged. Effort is unbounded above 2;
  the offer ceiling 80 is not physical torque or a proven tactile notification.
  Reversal, rising force, error and invalidity withdraw added authority. Mode 0
  matches the prior controller in a 6,000-frame input replay; synthetic/CAN tests
  do not establish closed-loop driving or driver consent. A prolonged zero crossing
  remains ambiguous. See docs/steering_handover_20260930.md for tests and limits.

- On 2026-09-29, the user selected IMU-based suspected-impact detection at 1.5g
  horizontal acceleration, with a visible/audible warning, ten seconds to cancel
  by touching anywhere, then OpenpilotEnabledToggle=false and manager DoReboot.
  Compensate gravity and mounting angle using fresh valid pose/calibration;
  require two fresh samples within 30ms. aEgo is supporting context only.
  Unseen/frozen UI cancels the transition; preserve takeover alert precedence.
  Block control including AlwaysLateral during reboot, preserve normal volume,
  and use the existing bounded reboot sound helper. Saved OFF persists until
  manually enabled; reboot interrupts recording. No incident file protection or
  upload is implied. The 1.5g threshold, drop/rough-road rejection, physical
  display/audio and actual vehicle reboot remain unvalidated. See
  docs/impact_dashcam_20260929.md.

- On 2026-09-29, the user expanded the Carrot Web auto-update reboot sound request
  to ordinary reboots. Use the stdlib-parent common/reboot.py helper for hardware,
  manager, main Web tools and startup recovery: existing prompt.wav once before
  reboot, separate audio child, four-second timeout, saved volume/mute respected.
  Audio failure must not block reboot. Keep update eligibility and recovery policy
  unchanged. Raw OS/factory-reset/standalone-recovery-web commands are not hooked.
  Desktop tests do not establish physical speaker/reboot behavior. See
  docs/reboot_sound_20260929.md.

- On 2026-09-29, the user requested one onroad readiness sound at the first
  engageable state (no NO_ENTRY event), preferring an existing sound. Use
  prompt.wav once after 0.5 seconds of initialized, non-passive, onroad, healthy
  CAN/service readiness and after current alerts finish. systemReady is sound-only
  and lowest priority; preserve warning precedence and normal user/ambient volume.
  The latch lasts for selfdrived's onroad process lifetime. Desktop tests do not
  establish vehicle speaker/timing validation. See docs/system_ready_sound_20260929.md.

- On 2026-09-29, Ioniq 5 PE C4 ff1 segments 0/2 on 250f14ed showed startup
  DM inference/model readiness delay and a separate Jetlink 97.28 ms roundtrip
  causing one model input skip and transient downstream invalidity. Expected
  process PIDs and all camera frame-ID sequences remain continuous; this is
  not evidence of a process crash or sensor capture loss. The user requested
  hiding an absent Jetson: a fresh waiting report is now quiet before modeld's
  first report, with READY restored on healthy connection. Preserve fresh model
  errors, active-session conflicts and stale-link errors. Physical display and
  the underlying isolated latency remain unvalidated. See
  docs/jetlink_ff1_investigation_20260929.md.

- On 2026-09-29, after two GV70 camera-side warning recurrences with unknown
  cause, the user authorized blocking the observed stock-cluster popup and
  requested checking its sound. Scope suppression to GENESIS_GV70_1ST_GEN
  camera-SCC, lateral-only control, HDA_InfoPUDis=3 with the observed camera
  FCA_SYSWARN=1/VALUE63=15 signature and no decoded MDPS/SCC fault or separate
  popup/sound request. Modify only the outgoing cluster copy; retain raw CAN,
  camera state, actual control and other fault/hands-off alerts. Both logs have
  HDA_LFA_WrnSnd=0 and openpilot alertSound=none; popup-associated chime is an
  inference, not confirmed audio causality. Replay removes all four observed
  popup frames; physical display/sound suppression remains unvalidated.
  See docs/canfd_feedback_counters.md. This supersedes the earlier recommendation
  to leave this popup unchanged pending root-cause diagnosis.

- On 2026-09-29, the user clarified that DM's 20-second standard hold starts
  only when surrounding moving traffic appears after an absence. Additional
  vehicles during occupancy do not extend it. Camera monitoring during the hold
  uses stock timing/detection/inputs, expires prior grace and suspends experimental
  resets; camera-unavailable timing stays 15/30/45. Then experimental criteria
  resume, but occupied surroundings cannot earn the empty-road bonus. Retain
  accumulated warnings/lockout and the two-second observation dropout retention.
  No forced warning for attentive drivers. See docs/dm_traffic_hold_20260929.md.

- On 2026-09-29, the user approved the C4 DM inset immediately right of D:
  84x84 at (382,144), leaving 10px before the right strip. VISION moves above it;
  confidence-dot travel returns to full height. C3 placement is unchanged.
  DM event stage1 is visual-only; stage2 (first audible) has final PCM gain
  >=0.7, and stage3 (final) always uses 1.0 regardless of user/ambient volume.
  Match event identity and sound together so navigation sharing the WAV retains
  normal volume. Desktop PCM/UI tests and synthetic rendering do not establish
  physical-device loudness or readability. See docs/dm_onroad_preview_20260928.md.

- On 2026-09-28, the user requested live DriverMonitoringMode changes. Poll
  typed Params every 0.5 seconds in the existing DM dispatcher; ignore the retired
  CARROT_DM_MODE startup latch. Preserve elapsed awareness, calibration, traffic
  hold, warning counts and lockout. A real mode change ends previous interaction
  grace and the forward-attention streak; an unchanged read must preserve them.
  Shorter budgets may immediately trigger warnings; toggling is never attention
  or a lockout reset. See docs/driver_monitoring_dm2.md for desktop validation.

- On 2026-09-28, the user superseded the Ioniq 5 PE-only touch restriction:
  Hyundai/Kia/Genesis CAN-FD uses original ECAN STEER_TOUCH_2AF by received
  profile, without a vehicle-name whitelist. Require the named DBC/address/size,
  existing layout/checksum/status/counter and freshness checks. Discover late
  arrivals with optional registration only after reception; do not add missing-
  hardware CAN faults or populate/modify ADAS TX caches. Address 0x2AF alone
  is insufficient. All 37 configured CAN-FD platforms pass synthetic parser
  tests; physical evidence remains Ioniq 5 PE only. See docs/driver_monitoring_dm2.md.

- On 2026-09-28, the user authorized clearing DM lockout after confirmed parking:
  valid/fresh Park, raw zero speed, standstill and disengaged/inactive status for
  one continuous second, in both modes with or without camera. Filtered speed
  may have only <0.01 m/s settling residue. Speed-only or engage OFF/ON resets
  are excluded. Keep stock policy.py unchanged; selfdrived persists fresh DM
  lock/release transitions so a cleared saved flag cannot relock on DM restart.
  Desktop tests do not validate actual parking. See docs/driver_monitoring_dm2.md.

- On 2026-09-28, the user requested a manual DM switch triggered by three
  distinct physical vehicle CANCEL presses,
  each separated by a release, within three seconds. Held/repeated packets, BT
  CANCEL and automatic-control CANCEL echoes do not count. Another received
  non-CANCEL button event, invalid/stale state, input-stream gap or timeout resets
  progress. On 2026-09-29, the user removed all gear and speed restrictions:
  every gear and standstill are eligible, and gear or speed changes alone do not
  reset progress. Ambiguous stock-ACC speed-button echoes conservatively reset
  progress because they cannot be distinguished from a physical intervening
  press. The
  cancel-echo filter correlates carControl requests rather than confirmed CAN
  transmission, so a physical CANCEL overlapping that 150 ms window may be
  conservatively ignored and must be pressed again. Hyundai/Kia/Genesis
  openpilot-long bypasses this filter because its controller does not transmit
  CANCEL buttons from that path even though the internal request level can stay
  high; stock-long and other platforms retain the filter. Its paired-release
  suppression expires after 0.5 seconds; an interleaved physical press/release
  can therefore require one additional press without causing a false disable.
  The gesture turns DM off only for the current ignition session. The later
  2026-09-29 revision also retains DriverMonitoringEnabled as a persistent,
  default-on switch exposed only through Carrot Web search. It is intended for
  absent or failed DM cameras; recommend leaving it on, and explain that turning
  DM off may violate applicable laws or driving requirements without claiming
  universal illegality. The next ignition-on or manager/device restart clears
  only DriverMonitoringSessionDisabled; a saved Web OFF stays off until the
  user manually enables DriverMonitoringEnabled again. File/QR backups and
  file/QR/profile restore paths exclude DriverMonitoringEnabled, including
  values in older backups; old backup downloads are filtered too. Resetting all
  settings may restore the default ON value. Persistent OFF must be selected
  locally on each device. Disabled DM
  stops the model during normal onroad operation and gates alerts, monitoring
  force deceleration and lockout while retaining a neutral state heartbeat.
  Driver View may run the model only for face preview while enforcement remains
  neutral. Keep DriverMonitoringMode and CarrotVisionEnabled independent;
  DisableDM remains migration-only. Desktop tests do not establish vehicle
  validation. See docs/driver_monitoring_dm2.md and both localized DM guides.

- On 2026-09-28, the user authorized automatic Git update/reboot after failed
  builds or manager startup, waiting through network loss. The launcher owns a
  standalone recovery display and releases its build lock before recovery Git.
  Retry after 30 seconds; automatic reboot requires a newly applied commit, so
  the same broken revision cannot reboot-loop. Keep the manual Git pull/reboot
  button, current branch/upstream, dirty-file protection and shared repo lock.
  No hard reset, normal onroad update action or AGNOS-policy change is implied.
  Graphics failure has a stdlib-only update fallback. Desktop tests/renders do
  not validate physical C3/C4 touch or device reboot. See docs/startup_recovery.md.

- On 2026-09-28, the user requested original Ioniq 5 PE wheel touch in DM,
  explicitly preserving existing ADAS transmission. ECAN 0x2AF raw bytes now
  feed separate CarState.steeringTouch; torque-based steeringPressed and TX
  remain unchanged. Six historical segments verify the receive layout/checksum
  and counter, not physical-contact ground truth or current vehicle validation.
  Accept the lowest reported touch level 1; raw TOUCH1/2 ranges overlap and must
  not become an unvalidated baseline-plus-one threshold. No-camera modes accept
  fresh held contact; camera mode 1 accepts only release-to-contact edges, never
  indefinite grace from holding or reconnecting. Camera mode 0 stays stock.
  Unknown, stale, malformed or frozen-counter data grants no touch credit;
  terminal alerts remain. Scope this empirical profile to Ioniq 5 PE until other
  vehicles are verified. See docs/driver_monitoring_dm2.md.

- On 2026-09-28, the user revised DriverMonitoringMode after the initial DM2
  implementation. Mode 0 keeps stock camera behavior, but unavailable-camera
  interaction timing is now 15/30/45 seconds. Mode 1 uses the same interaction
  timing, doubled only on a verified empty straight road. New moving traffic
  removes the empty-road bonus for 20 seconds. Camera mode 1 uses 2x stock vision
  timing, 4x on a verified empty road, and 20% head-pose tolerance relaxation.
  The user explicitly selected a full interaction grace: fresh control/BT input
  resets monitoring and defers camera warnings for 45/90 seconds before its
  warning clock starts. This supersedes the earlier two-second credit and
  protected-distraction debt restriction; detection thresholds remain unchanged,
  but sleep/eye/phone warnings are also delayed. Confident forward attention for
  two seconds resets the camera clock without renewing interaction grace.
  Terminal alerts and lockout remain; no input or context change clears them.
  Camera absence AND failure automatically use interaction monitoring, with
  recovery preserving progress; do not add a manual camera-installation setting.
  Stock policy/dmonitoringd files stay unchanged. DisableDM is migration-only;
  CarrotVisionEnabled is independent. These are requested experimental timing
  choices, not statutory limits or device/driving validation. See
  docs/driver_monitoring_dm2.md and both localized DM guides.

- On 2026-09-28, the user requested ordinary Git storage wherever possible to
  eliminate this branch's Git LFS bandwidth dependency. All seven remaining
  LFS pointers were converted to byte-identical Git blobs; bundled models and
  the legacy updater are below GitHub's per-file limit. Do not reintroduce LFS
  tracking or setup pulls. Existing NAS model delivery stays unchanged, and
  historical refs are not rewritten. See docs/lfs_to_git_20260928.md.

- On 2026-09-28, the user approved C3/C3X main UI onroad affinity cores0,1,2,3,6
  with SCHED_OTHER/nice19, superseding core6-only for tici/tizi. C4/mici stays
  core6. Apply to all UI threads; offroad returns to little cores, and onroad
  C3 keeps nice19 during big-core unavailability. Cluster/core7, camera/control/
  model/radar and IRQ policies are unchanged. Casper logs on a3278c04 measured
  UI15.54/14.68Hz with camera20Hz and substantial UI runnable wait; this is
  pre-change evidence, not validation of the new mask. Affinity does not pin
  one whole frame or guarantee little-first placement. See docs/camera_core5_trial.md.

- On 2026-09-28, the user requested a single Windows installation ZIP and a
  minimal Korean guide: extract, run 01, run 02, insert the finished card.
  Follow-up requires bilingual stage introductions, approximate durations,
  exact response instructions and brief safety guidance; brevity must not
  remove backup/write-in-progress cautions or Jetson shutdown and power
  disconnection before card insertion. Label the link "설치파일 받기".
  Present Korean first with English underneath on a separate, visually secondary
  line. Use clear stage headings, spacing and a styled offline HTML guide; never
  interleave Korean and English with slash-separated sentences.
  Keep hashes, portable dependencies, USB-C patching and readback automatic;
  do not restore manual Python/Etcher/hash/hotfix steps to the default guide.
  The package prepares a patched file before writing, preserves the published
  base image/runtime/model and confirms the selected USB card before erasing.
  PC preparation and disk-guard tests do not establish physical-card writing
  or first-boot validation. See docs/jetson_windows_installer_20260928.md.

- On 2026-09-27, the user requested full integration of `carrot-jetlink` into
  `carrot-wip` and Korean-first public installation/release instructions. The
  complete Jetlink history through b9950442ca is merged; do not treat it as an
  independently maintained vehicle feature branch or recreate older experiments.
  Keep the existing internal model, AMD Cinque v3 selection, AGNOS and validity
  policies unchanged. Jetson uses its separately pinned Cinque v2 contract and
  signed f2b22dc host release; merging vehicle code does not promote a new host
  runtime/model or justify another image rebuild. Public host sources remain in
  ajouatom/carrot-jetson and images on NAS. PC offline SD patch first-boot and
  integrated vehicle driving/C3 checks remain distinct from prior parked C4
  trials. See docs/jetson_wip_integration_20260927.md and the linked Korean guide.

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
