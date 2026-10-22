# Repository memory

- On 2026-10-22, user reports red Jetson startup and requests latest-drive review.
  Rearm bounded HELLO grace only after confirmed offroad host absence; retain
  onroad/unknown-state failures and stop detach from extending grace forever.
  Connecting-only status lease8s covers3/5s HELLO+2s retry; other/model/health
  freshness3s unchanged.71 desktop checks pass on each branch; both Linux CIs
  pass (NAV1008/56skips, Wip801/2skips broad, plus update/nav/storage/network).
  Ten hash-verified rlogs: latest12:09-12:13 N16 active only final66.223s,
  1326 valid consecutive outputs, mean42.369/p9946.531/max78.483ms,5>50,
  no fallback or pose input failure. Prior132.230s2645 outputs,6>50 and one
  camera-sync frame skip0.95s after join matches odometry invalid/pose inputs
  bad/locationdTemporaryError; camera captures continuous. Latest60.713ms
  call has roundtrip55.08 vs server25.50ms, other warp/server tails remain.
  No complete latency/locationd cure or cable-root-cause claim. Raw private,
  vehicle read-only; normal Git Pull/reboot, no Jetson bundle. See
  docs/jetson_startup_drive_review_20261022.md. Both branches single-parent.

- On 2026-10-21, paired USB trace localizes50.092ms call to comma reply
  rearm:8.496ms userspace gap after first read, worker queue0.051ms,
  completion-return0.094ms; normal-priority watchdog wakes sleeping reader.
  GIL inversion inferred, lock not directly traced. Separate watchdog runnable
  wait25.533ms. FIFO1 trial stops at61.65s on observer camera/model gap
  (observer88.65ms gap), no causal camera-regression claim; priority restored.
  Coalesce watchdog wakes against earliest pending timer; preserve earlier-arm,
  close, expiry/cancel and outside-lock abort semantics. Old1000-write test
  fails2002notifications; new passes.73NAV/62Wip desktop checks and both CIs pass.
  GPU signed default activation9cfe verified via normal boot updater, no overlay.
  Normal Git Pull/reboot installs cd986d7e9d; two120s windows4800 calls,
  mean42.443/max48.327ms/zero>50. First observer omission recovered valid in
  original rlog; buffered repeat2400 consecutive valid model/DM/both cameras,
  pose healthy. NAV/HUD/preview240/240 ready/fresh. Watchdog~0.77 switches/s
  vs prior short trace106.9/s; original OTHER policy. No hard deadline/road or
  all-locationd cure claim. Wip source2ea4415ac2; no host update for this fix.
  See docs/jetlink_watchdog_latency_20261021.md; raw private.


- On 2026-10-21, user requests enabling GPU recurrence if beneficial, then
  tracing/fixing remaining >50ms calls. NAV defaults stateful TRT recurrence ON;
  explicit env0 retains diagnostic host path, Wip backend/model untouched.
  Same N16/live NAV/USB HUD/FIFO2 OFF/ON/ON/OFF120s: means44.294/42.414/42.439/
  44.314ms, no frame-ID gaps or invalid model/DM/pose; ON2 counts2399 not strict
  >=20Hz. Mean gain1.877ms/4.24%; both modes2>50ms in240s, tail NOT fixed.
  ON worst60.985ms has same-frame server27.89/roundtrip55.32ms; investigate
  outside-server path rather than call it GPU inference. HUD480/480 fresh,
  NAV preview479/480; NAV ready471/480 with position/route-position exceptions.
  67 focused desktop regressions pass. Signed runtime deployment/trace follow-up
  recorded in docs/jetson_gpu_state_activation_20261021.md. Raw private.


- On 2026-10-21, user requests review/integration of tjdgus200 7cbedfe/8f0cd37.
  NAV gains default-OFF GPU recurrent state with all-state finite scan, reply
  event and same-stream trailing feedback/reset; no async USB/model/deadline
  changes. Wip lacks N16 stateful infrastructure: common C3/C3X eligibility and
  record only, preserving model choices. Exact tici/tizi/mici + aarch644.9.103,
  dynamic owned DWC3 workers, FIFO2 lease/restoration unchanged; C3 unmeasured.
  Actual guarded offroad Orin N16 OFF/ON/ON/OFF,250frames each incl3 resets:
  all outputs exactly equal; means26.828/24.633/24.631/26.743ms (~2.153ms gain).
  Actual CUDA checks all65536 fp16 patterns plus signed fp32 finite/Inf/NaNs.
  Initial diagnostic child missing video/render groups failed before inference;
  corrected account groups pass; existing runtime691566 restored automatically.
  Original offroad parameter restored; final60s model/DM/cameras/odometry each
  1200 valid consecutive at20Hz, livePose1200 healthy; FIFO2/N16 ready.
  Native timings exclude USB/DM/live NAV,
  not a full-system/road/locationd fix.112 focused desktop tests pass; Linux
  socket/fcntl checks use CI. Signed runtime pin and live GPU opt-in unchanged.
  NAV a51dcb02d5/wip e925a3cce4 Linux integration and Windows installer CI pass.
  See docs/jetson_gpu_usb_review.md; private diagnostic scripts/logs excluded.


- On 2026-10-21, owner requests restoring comma-to-Jetson USB display traffic.
  Nav-only new receiver capability selects bounded snapshot plus at most two
  32KiB media fragments during GPU execution; old receivers retain Wi-Fi until
  updated. Preserve N16 and FIFO2 helper, no ESP32 return. Parked120s Wi-Fi/USB
  each2400 consecutive valid model frames, healthy pose; mean44.256/44.226ms,
  max63.704/63.361ms,3/2 above50. USB adds~0.644MB/s,7% of9.18MB/s model input;
  main-threadCPU+1.63ms/frame, wholeCPU78.38->77.13% is not causal attribution.
  Phone960x540 video and mode7 preview fresh120/120 USB samples. NAV validity
  route_position throughout USB: no valid-route driving or full deadline claim.
  Wi-Fi return60s1200 consecutive valid each,mean44.381/max52.838,two>50.
  Signed host691566f0fa published/HTTPS verified, N16 unchanged. Ordinary
  Git Pull/comma reboot682ceeb484 and normal Jetson boot update verified; no
  manual cache/SD patch. Final120s NAV ready120/120,2400 consecutive valid
  model/DM/both cameras,pose healthy;mean44.174/max57.566ms,two>50. HUD/preview
  fresh120/120,phone video no decode drops. Display~0.707MB/s(7.70%). USB stays
  applied; temporary offroad/overrides cleared. Wip alreadyUSB: models unchanged,
  Cinquev2 Jetson and MDv3 eGPU. See docs/jetson_usb_hud_restore_20261021.md.

- On 2026-10-21, owner rejects ending Jetson latency trials with diagnostics
  restored only. Retain FIFO2 USB completion scheduling through a session root
  helper on tested mici/aarch64 Linux4.9.103. Discover actual dwc3_bh_work
  workers in isolated filtered trace, not hard-coded u17; validate kernel task
  and starttime, bound leases, restore at owner EOF/signals, journal for next
  startup after SIGKILL. Shared workers remain a limitation. Preserve serial
  reads, user CPU placement, N16/other branch models,150ms IPC and validity.
  Manual normal-NAV120s/2400 calls max49.81ms with no>50; full logs consecutive.
  Automatic helper60s mean44.51/max50.94ms,one>50/1201 observed consecutive;
  ten-second reactivation/EOF cleanup pass, NAV route_position invalid. No hard
  deadline, complete locationd cure or road stability claim. Common Git Pull/
  comma reboot delivery; no Jetson image/bootstrap update. Raw evidence private.
  See docs/jetlink_usb_completion_priority_20261021.md.
  Published nav040f000e1b/wipc5a5876b68; both Linux/installer CI pass. Actual
  guarded comma Git Pull/reboot installs040f000e1b, N16 sameSHA active; automatic
  new-worker FIFO2 retained. Final60s model/DM1201 valid consecutive each,
  ~20.011Hz; model mean44.07/max48.57ms,no>50,pose healthy. NAV position invalid,
  so no full-NAV/road claim. No Jetson update or manual priority script needed.

- On 2026-10-20, PR559 review accepts independent default-off OnnxBsdEnabled:
  unload BSD network while lane recognition/snapshots continue; vehicle BSD stays.
  Existing ShareData users must opt into camera BSD. General TCP7711 telemetry
  removed, fresh Tesla DAS_road fields retained. Correct concurrent refresh order,
  clear results on transitions and reject old-generation camera work/results;
  Web lane-only readiness/OFF state and settings grouping fixed. Three new
  regressions fail original PR/pass corrected; 142 desktop Python tests with
  platform doubles, 261 web tests and25 Wiki tests pass. Both maintained branches,
  single-parent; Git Pull/reboot delivery, no vehicle install or measured-RSS claim.
  See docs/pr559_visual_bsd_20261020.md.

- On 2026-10-20, owner clarifies initial red errors are comma upper-right
  Jetson badge. Distinguish normal boot-runtime handoff; first-handshake
  timeout/explicit no-reader wait stays connecting for<=120s before first ready.
  Preserve model/protocol/signature/cleanup/runtime failures, deadlines,
  transport retry and actual control alerts. Korean waiting/preparing badges.
  Nav122/wip120 focused tests; actual C4 draw method desktop render checked.
  Further drive review finds missing model frame18031 present in both cameras,
  SOF skew11.302us; recorded metadata passes existing pairing. Exact reader
  delay remains unknown. Add bounded previous/current-cycle camera gap trace,
  <=1Hz on gaps only, no validity/model change. Native fake-input overhead
  ~11us/pair; not real camera timing or a locationd fix. Comma Git Pull/reboot;
  existing signed NAV host pin retained. No live install. Raw private.
  Nav CI39389864670:94root/923broad(56skip)/119nav/11storage/11NAT pass;
  wip CI39389916455:735broad(2skip)/119nav/11storage pass; both Windows pass.
  See docs/jetson_startup_badge_20261020.md.

- On 2026-10-20, owner clarifies missing map is Jetson USB screen mode7.
  C4 NAV256 preview is fresh but absent from host snapshot/files; received
  phone960x540 video is a different map. Forward bounded PNG only through
  explicit Wi-Fi publisher opt-in; validate source/transport age, digest,
  dimensions and decode in host HUD adapter. Preserve96KiB budget and hide
  stale/missing images. Desktop55pass/2skip, actual draw method with graphics
  double; physical activation pending Git Pull plus BOTH reboots. No live patch.
  DATA live40% used/9.53GiB free, fixed16GiB inside40GiB image on119.375GiB card;
  capacity adequate, no resize/cleanup. Three full drive segments show one
  camera frame gap18030->18032, invalid odometry, pose inputs false57.7ms,
  alert1.985s; sensors/posenet healthy, no locationd exit. Model loop max83.801ms
  is window evidence, not a unique delay cause. No alert/validity/model changes.
  CI39386479268 passes94root/868broad(56skip)/119nav/11storage/11NAT.
  Signed hostc8a95308b0 published, full HTTPS size/hash verified; N16 unchanged.
  Nav runtime only; wip record only. Raw private.
  See docs/jetson_nav_preview_drive_20261020.md.

- From 2026-10-20, user requires all normal delivery through comma Git Pull
  and restart/reboot. Design and package changes for that path; per-device
  SSH copies, Params edits, service manipulation and cache seeding are not
  normal user installation steps. Use Jetson USB patch/install only when
  the installed bootstrap cannot recover through the ordinary updater.
  Current boot-gated runtime selection requires Jetson reboot too; keep
  actual offroad and Internet throughout preparation. Do not claim a
  comma-only restart activates a new already-running Jetson release.
  See docs/jetson_update_policy_20261020.md for the delivery contract,
  AP Internet-sharing correction and outstanding end-to-end validation.

- On 2026-10-20, read-only owner checks reproduce Jetson NAS timeout while
  comma HTTPS works: AP/DNS/forwarding active, no NAT, default nft backend
  unsupported, legacy backend operational. Nav AP now owns one scoped
  masquerade rule with backend probing, offroad-only setup/retry and exact
  cleanup; Internet failure preserves local HUD Wi-Fi. No USB/OS/model change
  or live installation. Desktop70pass/4skip; CI39369739310 passes94 root,
  855 broad/56skip,119 nav,11 storage,11 NAT including real isolated packets.
  Do not claim an uncached physical Git Pull/reboot update has been validated.
  Wip receives delivery policy/docs only, preserving its runtime.


- On 2026-10-20, user supersedes earlier model deferral and requests contributor
  Jetson N16 port (reviewed cf036fa578/a1d723a6b4). Official ONNX01effc67,
  814591107B, shares eGPU N16 checkpoint96244b1f/12800 export family; embedded
  ONNX checkpoint omits step suffix. PKL/TRT equivalence is not established.
  Port identity/label only to nav; keep existing recurrence, USB inference,
  signed NAS trust and Wi-Fi/local-USB HUD. Wip model/runtime unchanged.
  Independent full ONNX checks and148 CPU raw/patched frames exactly equal;
  desktop91pass/1skip, Linux94root+845broad/55skip+119nav+11storage pass.
  Signed runtime abac23f644 and comma5a0e8158 applied under fresh P/inactive
  guards. NAS fetch timeout uses hash-verified WLAN cache transfer; normal
  boot probe builds45.3s new TRT10.3 plan and passes actual finite inference.
  Manager restart retains env/affinity; final offroad setup and daemon restart
  recover an existing disappearing-DHCP-PID cleanup race (not fixed here).
  Restore both temporary holds/onroad. Final300s model/DM each
  6000/6000 valid consecutive at19.99947/19.99927Hz,
  sender61/61 and receiver60/60 connected, USB renderer~10Hz.
  Approx20Hz only; no actual-map, road, PKL/TRT or long-stability claim.
  Raw private. See docs/jetson_n16_integration_20261020.md.


- On 2026-10-20, owner reports default-only Jetson panel with ignition/P and
  authorizes agent-controlled offroad. SD enrollment/cache succeeded, but prior
  boot gate still approved989386eb; comma a4e57bad rejects legacy USB HUD and
  dedicated AP waits for offroad. Fresh P/zero/standstill/inactive/panda guard,
  existing snapshot startup hold plus onroad cycle and30min restoration watchdog
  produce real offroad. AP starts; normal Jetson reboot/probe activates signed
  b7b463a20b and autojoins WLAN. Restore original absent snapshot hold/onroad.
  Physical USB H264 renderer reports10Hz/26 live services;12 receiver samples
  and13 sender samples connected over60s, final HUD connected. Native model/DM
  each1200 valid consecutive at19.99812/19.99857Hz (~20, not strict>=20).
  Old80eedcdd model unchanged. No map bytes, independent visual confirmation,
  road/RF recovery/long stability or new full-image claim. Nav hardware only;
  wip runtime unchanged. Raw private. See docs/jetson_wifi_usb_panel_20261020.md.


- On 2026-10-20, user requests comma-to-Jetson HUD/navigation over Wi-Fi,
  no USB fallback, and complete ESP32-S3 display retirement. S3 code, firmware,
  pairing tools and installer routes removed both branches; shared renderer
  opens USB directly, preserving Jetson panel hotplug. Nav-only Wi-Fi integrates
  tjdgus200 b09e20a1 with both-end USB display rejection and no Jetson display
  pumps in inference. Dedicated AP starts offroad; model/provisioning USB stays.
  Signed nav runtime b7b463a20b published and pinned, old80eedcdd ONNX unchanged;
  latest n16 Jetson model remains deferred. Owner SD patched6144 APP bytes with
  verbatim original setup retained; verified FAT payload/runtime pre-cache,
  independent ext4/GPT readback; DATA journal/model/key not written. Next boot
  performs one-time receiver enrollment/cache, then ordinary boot gate probes
  and activates selected runtime. SD prepared, NOT boot/hardware tested.
  Desktop52 integration and41 retirement tests/4Windows skips; Linux94 root,
  845 broad/55skips,119 nav,11 real-storage pass; eGPU and installer CI pass.
  No physical RF/display-rate/20Hz/road claim. Wip retains its separate runtime.
  Private backups excluded; single-parent. See docs/jetson_wifi_usb_panel_20261020.md.

- On 2026-10-19, offroad Mapbox review finds real navd allManeuvers failure:
  np.cumsum yields numpy.float64, rejected by device Capnp struct-list setter.
  Convert distance to Python float at serialization only. Six real-cereal cases
  fail before/pass after on nav; wip old sums already pass, same boundary guard
  ported without model/route changes. Nav93 pass/9 native-render skips; wip22 pass.
  Exact corrected source in isolated C4 /tmp trial:44 synthetic route positions,
  actual Mapbox tiles/Carrot bridge/native VisionIPC/MapInput pixels match;
  warm10.109-20.089ms, IPC+checks0.913-1.623ms. Host-shaped buffers only, no
  driving-model inference/20Hz/steering claim. Cold537ms (earlier885ms) still
  exceeds500ms gate; not fixed. Operational code/Params/processes unchanged,
  no vehicle installation. Both maintained branches single-parent. See
  docs/mapbox_instruction_serialization_20261019.md; tokens/evidence private.


- On 2026-10-19, Ioniq9 route417/19 on5032afca77 reproduces retained stopped-red
  memory after lead-follow departure: target-4.497/output-4/measured-3.616m/s2
  at road speed, no new signal tracks, gas intervention at6.395s. Retire both
  start/legacy stopped latches above5km/h; preserve low-speed hold and fresh
  moving-red approach. Track auxiliary ownership to clear orphan e2eStopped
  after mode/invalid reset; preserve user/soft/stay stops. Gate final output by
  speed too. Seeded recorded-input127-pair replay: logged56/old55/new0 holds;
  no native MPC or closed-loop claim. Prior segment/latch origin unavailable.
  Both branches704 focused tests pass including61 new cases. Raw private; no
  vehicle install. See docs/signal_departure_hold_20261019.md.


- On 2026-10-19, user selects pearl cartoon DM eyes for both C3/C4. Replace
  shared onroad camera inset with plain Widget: no video client/shader/textures
  or box. Bounded mirrored head pose is illustrative, not ocular gaze; eyelids
  use actual per-eye blink/confidence, no decorative random blinking. Preserve
  freshness/session gates plus 500ms repeated-frame guard; gray unknown eyes,
  warning/lockout and wheel fallback, alert hiding/AlwaysOnDM and RHD selection.
  No monitoring/model/control/settings/sound changes. 75 focused and 180 wider
  UI/sound tests pass (one Windows native-IPC exclusion); desktop actual-renderer
  captures cover both sizes with synthetic inputs/schematic HUD. No physical
  vehicle installation, readability or CPU/road validation. Both maintained
  branches single-parent. See docs/dm_pearl_eyes_20261019.md.

- On 2026-10-19, user confirms company/hotspot share SSID and explicitly requests
  disconnect/reconnect testing without waiting for another hotspot. App-only S3
  1.2.1 preserves pairing: separate association/IP states,30s/20s watchdogs,
  max5s failed-attempt backoff; no repeated connect during active association.
  Diagnostic suppressed-IP notification recovers in20s plus rejoin: the stack
  still had IP, so this is a state-machine fault test, not DHCP/root-cause proof.
  Sender sync protects setup from an observed local idle-off collision; local
  decoder reset invalidates TCP/reference queue. Keep2s visual idle-off.
  Production has UART S3RECONNECT, no private AP/fault hooks. Exact893088-byte app
  67c8222e passes5 forced reconnects without MCU reboot/USB errors: IP3.88-3.94s,
  authentication13.52-14.02s, forwarded stream verified17.34-20s. Host discovery
  retries within same deadline; restore1Mbps/QP28..51 while retaining bounded
  pacing/queues/CBR. Both branches60 tests+1skip; native C recovery policy and
  installer5 tests pass; download test1pass/4Windows skips. Initial100s complex
  test passes, worst full-colour~2fps then other phases~10: no universal10fps,
  hotspot/road or overnight-memory clearance. Preserve C4 model/DM processes.
  Exact final100s complex sequence passes too. Guarded C4 Git reconciliation
  retains model163840/DM163845; mixed180s includes two observed DM gaps, then
  untouched60s model1202/DM1201 consecutive valid at20.005802/20.000011Hz,
  HUD12/12 connected. Final120s UART has no disconnect/close/USB error.
  Both maintained branches single-parent; private backups/logs excluded.
  See docs/s3_reconnect_20261019.md for final observation/release status.

- On 2026-10-19, user rejects ending S3 work at parser-only correction. Native
  driver QP S_CTRL succeeds without applying ranges; pre-stream S_EXT_CTRLS
  with all-layer255/all six I/P/B limits works. S3 defaults500kbps CBR-CFR,
  QP44..51 reduce luma bursts831472B to~28KB, but full-colour noise still177KB:
  neither setting is a hard cap. Pace <=16KB chunks at125000B/s, keep three
  ordered slots/backpressure; reset3s dead-sender timer only on completed chunk
  progress. Fixed timer previously aborted active paced transmission. Retry
  authenticated UDP capabilities within existing1s deadline; legacyJPEG remains.
  Exact native/Python actual-S3 full100s sequence passes: colour noise90units/30s
  (~3fps/~982kbps), later luma/blocks~10fps, no transport error; earlier same-
  profile failures remain recorded. Guarded seven-source/native-library C4
  install and HUD-only restart, model49147/DM49149 unchanged. Actual HUD~10fps/
  280kbps/~8KB peaks, but180s observation has16/36 connected samples and ends
  disconnected. Independent568-byte sync retransmissions and S3~2s FIN persist;
  no universal radio/firmware/hotspot fix. Model/DM3601/3600 valid consecutive,
  ~20Hz; mixed HUD-state available638->704MiB does not clear overnight creep.
  Both branches59 tests/oneLinuxskip; no router/firmware change, raw private.
  Publish single-parent. See docs/s3_complex_scene_20261019.md follow-up.

- On 2026-10-19, user requests deliberately complex S3 traffic after motion-only
  dropouts on company Wi-Fi and hotspot. New boot/base68e7785c8b at~78% reproduces
  native1Mbps VBR-CFR silence:751 motion inputs/75s yield one residual unit,
  601 noise inputs/60s yield zero; raw no-network callbacks reproduce and recover
  on static. CBR gradual90s actual S3 passes~10fps, but abrupt noise produces
  831472-byte units and another fast-parser trial hits3s ordered-queue timeout.
  Withdraw CBR default candidate; no rate/firmware/router change. Post-open
  QP28/36/44 controls return success but identical output, not a validated cap.
  Retain byte-equivalent Annex-B bytes.find search: native100KB394-487ms ->
  0.212-0.228ms;831KB2.986-3.407s ->1.745-3.635ms, synthetic low-priority timings.
  Both branches52 tests/one Linux skip; guarded one-file C4 install restoresHUD1,
  model49147/DM49149 unchanged. Final180s model/DM3601 valid consecutive each
  at20.000236/20.000119Hz; first25s reconnect then31/36 connected samples,
  actual H264~10fps/~1Mbps; available692.37->689.74MiB. Bounded parked only,
  not a full disconnect/encoder/memory fix.
  Small-command/discovery failures also precede video; auto can fall back toJPEG
  after lost capability response. Old pending memory watch ended at reboot after
  322samples/1639.90s, not2h; mixed HUD states do not prove a plateau. See
  docs/s3_complex_scene_20261019.md. Raw evidence private; single-parent branches.

- On 2026-10-19, user requests HUD isolation and broad recent/native/signal
  memory review. Same C4 boot/base2d71d318: normal HUD stop90.22->84.12%; stop
  mode2 recognition and separate legacy policy observer reaches79.12%, without
  model/DM/eGPU restart. Legacy observer consumes104MiB despite idle internal
  standby, remains after SignalAssistMode0, original private flag1 backed up
  then set0 and identity-checked worker terminated. Gate its creation only for
  ready-eGPU standby; preserve warm fallback and active-internal opt-in behavior.
  This is footprint correction, not proven creeping-leak/full-warning cure.
  150 latest closed qlogs peak91%; user92% outside retained samples remains.
  Full warmed windows lose200MiB; proclogd~12.4MiB, model/eGPU broadly stable,
  Shmem+79.5MiB and smaller common-service growth, incomplete total attribution.
  Oct11 pre-WiFi109c90-91% includes377MiB signalcolord child plus103MiB legacy
  observer; current mode2 is~60MiB. Native CPU drawing60k iterations plateaus;
  not GPU/road proof. RAM logs~81MiB; do not remove watchdog heartbeat or hide
  growth by vacuuming. Observer adds bounded tmpfs/enable-state inventory.
  20min HUD-off and10min all-off comparisons include marked diagnostic windows;
  no long stability claim. Restore HUD1/mode2, retain legacy flag0:~87% snapshots,
  then180s model/DM3600 valid consecutive each20.00088/20.00138Hz; S3 still retries
  below90%. PC/C4 same H264 clips pass;8KiB chunks slower, no transport-policy fix.
  NAV42/wip43 lifecycle tests and20proclog/2Linuxskips pass. Publish both branches
  single-parent; model source not installed live. Restored-state low-priority
  observer408329 capped2h/32MiB is pending, not a stability result. See
  docs/memory_feature_isolation_20261019.md. Raw evidence private.

- On 2026-10-19, user requested applying Jetlink491cf1e USB-current protection
  after PV5 UVLO review. Port USB_ICL zero vote into Carrot's own daemon/helper,
  all transport modes, only after1.5s observed empty port and helper rechecks.
  Preserve across retries/exit; reboot clears. Omit upstream's attached-iOS
  exception because transport selection is not proof of independent power.
  Attached/unknown startup defers; missing/conflicting voter and failed helper
  are bounded/reported. No VM/role/model/deadline changes or vehicle action.
  Desktop fake-sysfs and daemon regression pass; Linux real-socket validation
  remains CI-only. UVLO+Jetson is a plausible mechanism, not confirmed PV5 cure;
  available900mA is not measured draw. Both maintained branches single-parent.
  See docs/jetlink_usb_power_20261019.md for activation and validation limits.

- On 2026-10-18, user again reports90% versus~80% after reboot. Completed2h
  capture has1412 samples/118 inventories: available421.45->375.51MiB,
  minimum326.65. Proclogd PSS14.64->52.81MiB; at10:21 private dirty rises38.62MiB
  with RSS only+0.36MiB, then plateaus. GC-induced shared-page copying reproduced
  in native isolated forks; exact live GC trigger remains inferred. Fix daemon-
  entry GC freeze before runtime resources, keeping new cycles collectible and
  parent unchanged. Separately prune exited metadata and validatePID/starttime:
  2000 native byte-identical publications retain2 entries versus old2001; PID
  reuse now refreshes metadata. Exact-main native GC private-dirty delta13.28->
  1.66MiB; desktop20pass/2Linuxskips. Nativepytest unavailable, standalone native
  assertions pass. No vehicle install/restart; old manager preimports module.
  Model/DM/UI GPU inventories unchanged, HUD lifecycle median+0.59MiB, S3 often
  disconnected. Does NOT explain all80->90%, prove Wi-Fi/eGPU cause, or establish
  long stability. Publish both branches single-parent; raw evidence private.
  See docs/proclog_memory_20261018.md. Supersedes pending2h status below.

- On 2026-10-18, user explicitly removed the Ioniq5PE-only CAN GPS restriction.
  Use actual DBC profile eligibility across Hyundai/Kia/Genesis: named0x374,
  eight bytes, two unsigned LE32 coordinates at0/32, scale1e-6/offset0. Optional
  registration covers CANFD and legacy parser paths; current legacy DBC lacks
  the profile. All CANFD configurations pass synthetic reception on configured
  bus, including late arrival; absent/incompatible profiles stay inactive.
  Preserve priority, original timestamps, motion/freeze/course checks. Nav155/
  wip134 focused tests pass with desktop OS doubles. Real driving evidence is
  still Ioniq5PE only; no vehicle install or new CPU claim. This supersedes the
  model-name restriction below. Both maintained branches, single-parent. See
  docs/vehicle_can_gps_20261018.md.


- On 2026-10-18, repeated S3 TCP failures persist after background downloads end;
  Omada has both clients associated, C4 5GHz/-67dBm and S3 2.4GHz/-71dBm.
  Remove only the Wi-Fi path's repeated10s shared-USB settling delay, preserving
  loading wait and wired policy.45 focused tests pass; selective guarded HUD
  install, no model/router/firmware change. Publish nav0adebf127c/wip2c479eaedf,
  single-parent. This improves recovery delay, not the original disconnect cause.
  User redirects C3/eGPU memory comparison to own C4:129qlogs show warmed evening
  82-83% versus morning88-89%; overall increase remains incompletely attributed.
  Native preimport comparison creates no sockets/threads; disabled HUD does not
  start a Wi-Fi sender. Same-boot600s live retry observation ends with unchanged
  ~393MiB available, HUD inactivePSS83.75-84.01MiB, active~182-186, stable oldION;
  model/eGPU PSS changes~0.1MiB, GPU inventories unchanged. No new leak reproduced
  or memory fix claimed. Some older95% samples include withdrawn diagnostic
  compilation; do not use this to dismiss earlier92%. Read-only low-priority
  observer PID273288 runs at most2h/32MiB under /data/carrot/diagnostics; result
  pending, do not claim completed stability. Concurrent60s model/DM1200/1199
  valid consecutive at19.99856/19.99491Hz, not strict>=20. See
  docs/s3_disconnect_followup_20261018.md and docs/memory_scope_followup_20261018.md.

- On 2026-10-18, user requested external-nav > phone > vehicle > device GPS.
  Ioniq5PE bus0/0x374 contains positive LE32 lat/lon*1e-6: 20 segments/1195
  original samples near1Hz. No OEM heading/fix-quality decoded; restrict profile
  to this model/validated hemisphere. Optional receive-only monitor requires
  three plausible samples, original timestamps, motion/freeze/freshness checks;
  derived >=5m course is separately valid, disabled in reverse, never north by
  default. Independent sources prevent lower-priority overwrite. Raw replay
  119551 CAN events accepts1166 fixes, rejects6 acquisition/23 moving repeats;
  829 have course. Nav188 tests pass/9 native-render skips, desktop OS doubles.
  No vehicle install/native CPU/closed-loop claim. Publish both maintained
  branches single-parent, preserving model architecture; wip also needs existing
  nav receipt/sequence handling. See docs/vehicle_can_gps_20261018.md. Raw private.


- On 2026-10-18, user reported worse signals and requested evening/morning review.
  Read-only/hash-verified213 files; actual drives10ea/10eb run087f862d81 with
  unchanged918654b8 tracker. Upper/central selectors remain OFFLINE, not the cause.
  Morning08:17:02 visible green is read, but box/ID mismatch repeatedly resets
  guard confirmation; right arrow+round green15x8 fails1.7 aspect gate. Actual
  hold persists until driver gas04.390, despite model stop distance rising~49m.
  Evening twilight near-red miss is proposal failure, not highest-row selection;
  later green confirms in~1.24–1.49s but lead bypass already removed veto.
  Measured evening9.29–12.87Hz, morning7.70–12.07Hz. New20-segment/12,492-observer
  central replay removes7 greens, changes0/20,955 nominal and0/20,951 +20ms guard
  updates, one release each; no solution or closed-loop claim. User proposes
  remembering central red housings and accepting strong same-housing transitions.
  Target whole-housing continuity/arrow separation, not brightness-only release.
  No vehicle/code/model change; publish records both branches, single-parent.
  See docs/signal_drive_review_20261018.md; raw evidence remains private.

- On 2026-10-17, user requested locationd CPU optimization, then rejected the
  small benefit and explicitly requested stopping, original code and records
  only. Trial native-result/noise-allocation optimization preserves32,625 replay
  inputs exactly and saves9.23% replay CPU; live38.63->35.24% of one core is
  only3.38 percentage points (~0.85 points averaged over cores0--3). Do NOT
  publish/reinstall the withdrawn implementation/tests. Restore all four C4
  source/native files to verified original hashes on same boot/base2ef833b2;
  guarded manager restart recreates the exited tmux session with original env.
  Original native mapping verified in locationd/paramsd. Final60s model1201/DM1200
  valid consecutive at20.000005/20.003386Hz; pose1200 healthy at20.001242Hz,
  same PIDs and eGPU active at both ends. No model/settings/reboot changes or
  road/memory/hard-deadline claim. Publish records only to both maintained
  branches, single-parent. See docs/locationd_cpu_20261017.md.

- On 2026-10-17, user requested central-priority signal improvement and historical
  validation. Offline 122 segments/56,612 observations: any-colour peripheral
  rejection removes1,035 reds; do not activate it. Preserve every red/red-history
  track and same-row green pairs: revised selector excludes503 green boxes,
  not503 labelled false positives. Development48-frame review removes33/34 clear
  backgrounds and retains7/7 visible overhead greens; seven exclusions remain
  ambiguous. These examples informed the rule, not independent accuracy tests.
  Road-only selection barely acts at low-confidence intersections; explicit
  image fallback is only a screen-position hypothesis. Paired stopped guards
  have0 changes over115,920 nominal and115,898 +20ms updates,18 releases each;
  cold per segment, not MPC/moving/closed-loop validation. Existing18.802s red
  colour gap remains.750 desktop tests pass with native Params lookup doubled.
  Keep CentralSignalSelector OFFLINE: no observer/display/control/vehicle change.
  Publish both maintained branches single-parent. See
  docs/signal_central_selection_20261017.md; raw evidence remains private.

- On 2026-10-17, user requested another live memory-leak investigation while
  ignition stays on. Same boot/base2ef833b2/selective HUD fixes: untouched580s
  retry observation returns HUD PSS75.55--75.74MiB, old ION4.75MiB unchanged.
  Then guarded HUD-only isolation runs real live renderer/native H264 into a
  packet sink (not S3) for~10min/5985 diagnostic units. Last5min PSS183.81--184.98
  MiB; bounded text cache resets, GPU240 objects/151.49MiB unchanged across140s.
  Normal close/collect drops to83.66MiB, display weakref gone, owned ION empty.
  No additional production leak reproduced or speculative code change. Earlier
  92% event remains incompletely attributed; no hours/road/S3 stability claim.
  Restore HUD1; final453.42MiB available/87.43%, S3 disconnected. Independent60s
  model/DM1200 valid consecutive each at20.0012/20.0069Hz, original PIDs/eGPU.
  Publish findings both branches single-parent. See docs/hud_memory_followup_20261017.md.

- On 2026-10-17, user reported >92% memory and clarified HUD power was cycled
  before the observed88%; user authorizes vehicle-log transfer for diagnosis,
  raw evidence stays private. Three routes/11,046 memory samples reach91%.
  Old510 graphics leaks are absent; current4.75MiB orphans do not explain it.
  Reproduce failed JPEG/H264 traceback retention until post-handler collection;
  gc alone barely returns RSS, glibc trim returns~25-33MiB after close and~9-10MiB
  after init. Collect/trim only at lifecycle boundaries, preflight authenticated
  S3 discovery before graphics, preserve TCP-after-init and concise retries.
  Twelve native injected failures reclaim all test-owned ION/weakrefs, but small
  PSS growth remains; no long plateau/full-warning fix claim. Selective two-HUD-
  file install on C4 2ef833b2, no model/control/reboot. Final model/DM60s1200/1201
  valid consecutive at20.0049/20.0025Hz; S3 still disconnected. 114 desktop tests,
  one skip. Both maintained branches single-parent. See
  docs/hud_memory_retry_20261017.md for remaining limits and measurements.

- On 2026-10-17, user requested muting signal notices with a lead and analysis
  of today's creeping/stopping. Gate all planner signal notices on current
  leadOne.status, any range, before navigation processing; preserve raw display,
  control, unrelated warnings and unconsumed notice cooldown. 724 tests pass;
  paired recorded-input replay20,384 updates changes no control outputs and
  removes42 notices (default context, not native/MPC replay). Read-only vehicle
  already runs2ef833b2/mode2. Collected/hash-verified99qlogs,17rlogs,3videos.
  11:05:20 red stop: transient vision lead32.6m opens the new2m exception for
  ~449ms; no pedal, peak0.627m/s and ~1.53m speed-integrated travel. Crossing
  traffic is visible; exact object identity unproven. This is a remaining
  regression in immediate lead bypass, NOT fixed by notice muting. 12:01 lead
  fixed4.15m but filtered stop crosses2.15m:16 hold/exception transitions.
  13:47 small0.141m/s/~0.20m movement follows actual gap opening with guard
  continuously bypassed. Do not call all creeping signal flicker or relax red
  further. Require separate validation of lead continuity/assignment and range
  stability. Publish both branches single-parent; no vehicle install. See
  docs/signal_drive_review_20261017.md; recordings remain private.

- On 2026-10-17, user explicitly requested reusing the existing stop-to-lead
  condition to bypass auxiliary red departure vetoes. Share finite positive
  leadOne range and dRel - filtered model stop distance <2m with both stopped
  handovers; 2m is relative to model stopping, not ego-to-lead spacing. Effective
  start/legacy decisions suppress hold/warning/red injection while eligible,
  including final longitudinal veto, without reporting green or granting accel.
  Preserve red memory for immediate reapplication on lead loss/range exit and
  preserve soft hold/user stop/MPC leads. Moving mode2 preparation is unchanged.
  715 desktop tests pass with native Params lookup doubled. Publish both maintained
  branches single-parent; no vehicle install or closed-loop road claim. See
  docs/signal_lead_handover_20261017.md. This supersedes prior blanket red-veto
  priority over an eligible lead, not signal recognition/green confirmation.

- On 2026-10-16, user reported repeated blank S3 H.264 output after initial pass.
  250ms queue-full shutdown magnified short stalls: retain three ordered slots,
  apply HUD-encoder backpressure with close/error checks and a3s dead-sender cap.
  This alone did NOT fix it: actual TCP timeouts and confirmed S3 idle-off remain.
  C4->S3 loses14/30 packets, PC11/15; router0/30. PC cannot connect even with HUD
  stopped. UART shows no association loss/reset/USB failures. Earlier successful
  boot selected channel6/-48dBm; failing boot another AP channel1/-68dBm. Replace
  first-match fast scan with all-channel RSSI-ranked connection; add BSSID status.
  App-only flash preserves pairing and selects channel6/-50dBm; PC detailed601
  frames/60.076s passes and user confirms restored actual C4 HUD stays normal.
  No router changes, AP lock, ongoing-roaming or general radio root-cause claim.
  Final180s UART has no close/off/USB failure;300s model/DM6000 consecutive valid
  each at19.9999977/19.9998398Hz (approximately20, not strict>=20).115 tests pass,
  one skip. See docs/s3_h264_release_20261016.md for observation/distribution.

- On 2026-10-16, user requested implementing and deploying H.264 for S3 Wi-Fi.
  Firmware1.2.0 authenticates capability discovery and keeps video chunks awake;
  C4 targets1Mbps/GOP10, preserves reference order in bounded queues, and retains
  legacy/auto-initialization JPEG fallback. PC physical simple601/60.06s and
  detailed589/60.11s pass; user confirms smooth test and normal native C4 HUD.
  AP channel/RSSI changed, so do not attribute every gain to codec. Two native
  HUD stops reclaim all six owned buffers; TCP loss reconnects in20.205s.
  Initial model/DM60s each1200 valid consecutive at20.007/20.008Hz. A separate
  300s memory watch stays connected, same PID/104FDs, PSS184.95->185.66MiB;
  synchronous smaps reads lose observer frames at exact30s boundaries and are
  not a model-rate pass. See docs/s3_h264_release_20261016.md for final clean
  frame observation, release checks and remaining Wi-Fi/long-duration limits.
  Preserve pairing; public firmware never includes NVS. Both branches single-parent.

- On 2026-10-16, user requested faster signals (20–50Hz target) and brake-lamp
  ON/OFF. Camera is20fps; no sustained20Hz signal pass. Remove packed NV12 copy
  with exact two-plane conversion; native conversion CPU drops~11–12%, wall
  varies. Count preemption/I/O in50% CPU-duty sleep, preserving unique-frame,
  freshness, detector/control and DM policies. Reject slower color prefilter
  and core-reassignment trial. Add default-OFF LeadBrakeLights, mode0 dominates:
  lazy context, bounded central lead crop, radar/vision match, turn rejection,
  display/log only, candidate/unknown (never confirmed OFF or go). Signal-first
  admission and overrun backoff; not a hard real-time deadline. Selected spatial
  screen8/10 ON and0/16 OFF proposals is development evidence, not accuracy or
  temporal validation.693 desktop tests with Params double pass. Native /tmp
  diagnostics only; no vehicle installation. Publish both maintained branches,
  single-parent. See docs/signal_speed_brake_lamps_20261016.md.

- On 2026-10-16, user reported live low-memory warning after keeping ignition on.
  Captured93.29%/241.89MiB available on the same14.5h boot; guarded HUD stop
  recovers88.68%/408.40MiB and reclaims its current6 buffers, but old510 remain.
  Comparable HUD-off snapshots2h44m apart lose24.29MiB, with no hundreds-of-MiB
  growth in surviving process PSS. Save evidence, verify selective HUD hashes,
  auto-pull off and no staged overlay, then authorized parked C4 reboot clears
  historical buffers:74.75%/910.88MiB at173s, same e9b33329e4/eGPU active.
  Recovery exceeds known GBM bytes; do not attribute all recovered memory to it.
  Initial postboot1199/1200 valid consecutive model/DM frames at19.9836/19.9852Hz
  is not a strict20Hz pass. Restore HUD with headroom guard:180s ends~82.3%/
  639.38MiB, same PID/six current buffers, S3 connected at all40–180s samples.
  HUD PSS188.00->191.19MiB is not a proven long plateau. Concurrent final60s
  model/DM1202/1202 valid consecutive at20.0340/20.0323Hz, eGPU active. HUD stays1.
  See docs/hud_low_memory_reboot_20261016.md for bounded
  HUD-restoration observation and remaining long-duration/Wi-Fi limits.

- On 2026-10-16, reproduced the remaining HUD native leak without S3: each
  post98 raylib window close retains6 ION objects/1658880bytes plus DRM/socket
  handles. Installed binary omits GBM surface/owned-FD and handoff-socket cleanup.
  Add HUD-only hash-pinned adapter for post98/post103; preserve borrowed DRM_FD.
  Native12-cycle minimal, actual renderer, post103 overlay and borrowed-FD tests
  each reclaim all test-owned buffers with stable handles. Manager preimports
  raylib: discard inherited graphics once by exec of only the HUD child before
  binding the adapter. Actual managed stop exposed another cleanup gap: mark
  context owned before asset loading and guarantee native close in finally,
  before blocking display close.54 focused tests pass; unknown binaries fail before render.
  Selective HUD-only install on vehicle e9b33329e4; no model/control/full pull.
  Final3 actual managed HUD stops each reclaim all6 objects; S3 reconnects and
  stays enabled at configured10fps (actual Wi-Fi refresh varies). Historical510 orphan objects/134.47MiB remain
  until a clean restart; no total-memory clearance or original Wi-Fi-cause claim.
  Recent S3 retries expose an older cleanup defect; introduction date unproven.
  Final model/DM60s1200/1200 consecutive valid frames at20.000010/20.000090Hz;
  S3 flag false at interval end, so Wi-Fi stability remains unresolved.
  See docs/hud_native_cleanup_20261016.md for actual-cycle verification and limits.

- On 2026-10-16, stop Wi-Fi HUD under fresh Park/inactive guard for memory/link
  diagnosis; ClusterHud stays0, networking stays on. Available memory recovers
  to501MiB, but510 orphan HUD ION objects/141004800bytes persist; not fixed.
  Isolated raylib headless init segfaults: never enable as workaround. Small USB
  commands and PC auth fail without rendering; channel6 temporarily refuses S3,
  channel1 attaches at-72dBm. After approved reset/app flash channel6 at-38dBm
  passes24 commands and150 JPEG frames/three10fps phases; later1200 frames in
  120.029s with0 disconnects and idle internal heap restored182104bytes. AP changed: no
  root-cause or long-duration fix claim. S3 firmware1.1.1 adds dim GPIO48 RGB
  states and secret-free S3STATUS UART diagnostics; GPIO38 revisions configurable.
  Pairing preserved, C policy tests/build pass; actual LED visual check pending.
  No C4 model/control install. See docs/s3_wifi_memory_20261016.md.

- On 2026-10-16, user clarified mode2 is moving gentle deceleration with no/remote
  lead; after stopping it uses mode1 exactly. Near means fixed distance, with50m
  used provisionally. Check both leads at entry/final cap; retain MPC leads and
  reject low speed/soft hold. User explicitly corrected display scope: mode0
  does NO auxiliary recognition/display/control; modes1/2 show raw recognition
  even with ACC inactive. Separate HUD ring from planner action, dim previous
  result at most2s from capture, gray unknown; never feed display memory to
  control. Do not restart the CPU tracker solely for1<->2. Collected144qlogs,
  20full segments/10,020 observer records. Evening green/lead departure is still
  held by existing guard; no blanket lead release or solved-recognition claim.
  Publish both maintained branches single-parent; no vehicle deployment. See
  docs/signal_modes_display_20261016.md for checks and limits.

- On 2026-10-15, user requested review/integration of Tesla PR551 c61d998ff8.
  Reproduced inactive phase-lead CAN/feedback and saturation defects; Tesla's
  encoded zero375 also regranted controls through the shared accel helper.
  Port the nine PR files unchanged: measured inactive angle, final feedback,
  cooperative torque blending only above25km/h with normal low-speed steering,
  output-limit-aware saturation and Tesla-specific longitudinal permission.
  Both maintained branches:199 focused tests/128 subtests pass. Three inherited
  generic disabled-angle safety cases still fail identically before/after;
  do not claim full safety suite success. Native C uses a desktop packet/forward
  ABI bridge, not MCU/vehicle validation. Publish single-parent on both branches;
  no vehicle installation. See docs/tesla_pr551_review_20261015.md.

- On 2026-10-15, user requested historical attribution of memory >80%. Read925
  retained local logs (916 own-device with memory) plus on-device qlogs. Earliest
  observed >80% is Oct10/108e, but Oct11/1095 stays73--77% with HUD+signals;
  not one proven regression date. Current DM warmup/plateau and logger sawtooth
  match earlier runs. GC retry fix reduces Python retention, but native HUD ION
  buffers persist after process death (89.6MB), and new PID accumulates again.
  Do not call memory normal/fixed or recommend process cycling as its fix.
  Expected S3 transport log cleanup published both branches; fresh-manager
  reboot verifies concise timeout lines (HUD-only restart kept cached code).
  Reboot clears old native buffers but four retries retain6.6MB again; current
  81%/685MiB available is not leak clearance. Preboot model/DM1200 frames/60s,
  postboot1199 valid consecutive each at19.974Hz, no missing IDs/fallback;
  do not claim a strict >=20Hz postboot pass. S3 still times out. See
  docs/hud_retry_memory_20261015.md; raw evidence private.

- On 2026-10-15, user's C4 Git Pull was blocked by two untracked manually
  installed eGPU helpers identical to incoming blobs; back up only those files,
  preserve unrelated build/model artifacts. Old93160a93 HUD repeatedly lost S3
  before renderer ready. Prior qlogs reach99%; same HUD PID grows168.62->528.55
  MiB RSS, +181.89MiB PSS across segments5->27. Actual failed JPEG pipelines
  remain frozen after close/collect; unfreeze releases them. Pair HUD session
  freeze with finally unfreeze/threshold restore, retaining active-frame policy.
  27 focused tests and native isolated12-cycle reclamation pass. This fixes a
  demonstrated retention path, not every system/native allocation or reboot
  cause. User authorizes parked system stop/update; verify fresh P/zero/inactive.
  Publish both maintained branches, single-parent. See docs/hud_retry_memory_20261015.md.

- On 2026-10-15, user requested public S3 firmware/install guide after confirming
  the panel wakes and turns off. S3 1.1.0 confirms sync/brightness-off, retries
  failed shutdown and reasserts idle-off; C4 offroad dimming includes registered
  S3. N16R8 / powered hub / TURZX 1cbe:0092 only. Browser installer and NAS ZIP
  preserve NVS; never distribute device flash backups or pairing keys. UART
  --auto requires a unique bridge and verifies identity. Native three-phase
  150-frame final test passes with preserved pairing; browser-native flashing
  is distinct from UART validation. Publish both maintained branches, single-parent.
  See docs/s3_release_20261015.md and tools/esp32s3_display/INSTALL_KO.md.

- On 2026-10-15, user requested MD v3 PR39087 on carrot-wip ONLY unless already
  installed. Prior selection was8d736683; new checkpoint1365126a/12864, AMDd4c7a3fc,
  pinned tinygrad02e69816/runtimec7c58622. PR changes only the big PKL; no matching
  ONNX, internal/DM and Jetson remain unchanged. Preserve retry/fallback policies;
  source-verified new runtime retains USB no-op sleep and +/-1 link-error meanings,
  so extend only the exact hash allowlist.411 focused tests/13 Web tests pass,
  8 Windows skips; Linux CI403 plus3 native-runtime tests and HTTPS empty-cache
  install/offline reuse pass on8864280c9c.
  No vehicle install or native20Hz/driving claim. See docs/md_v3_20261015.md.

- On 2026-10-15, user requested explicit tmux error/retry visibility after
  repeated eGPU investigations. Add bounded stderr [eGPU] lifecycle events with
  PID/monotonic time, saved failure cause/classification, attempt/limit/backoff,
  waiting reason, probe-ready/handover, actual first live publication and stop
  reason. Preserve all retry/validation/model policies; never write diagnostic
  text to the worker stdout protocol.179 focused tests pass; no vehicle action.
  See docs/egpu_persistent_rejection_20261015.md.

- On 2026-10-15, identified the persistent Ioniq 5 PE block from actual NAS
  diagnostics: pinned90970e01 reports AMD failed with +1 during input upload.
  Its usb_reap latches LIBUSB_TRANSFER_ERROR=1; old classifier recognized only-1.
  Both branches now classify exactly +/-1 only for that verified runtime, with
  previous-boot/hash/artifact/smoke validation for old rejections.175 tests and
  the exact recorded traceback probe pass. The failing734634 predates Oct15
  timing changes. Device wall clock was stale; do not assert recorded21:05 as
  incident time. Physical USB root cause remains unknown; no vehicle install.
  This supersedes the earlier unknown reporter cause below; numerical migration
  is a separate correction. See docs/egpu_persistent_rejection_20261015.md.

- On 2026-10-15, a Mountain Dew report remained rejected after reboot. Current
  wip lacked nav's previous-boot numerical-rejection migration; four positive
  replay cases fail before the port and pass afterward. Port only that migration,
  preserving branch model/runtime and per-frame policies. Keep identity, fresh
  boot, artifact verification and new smoke checks; unknown/integrity failures
  remain blocked. Both web branches deny ineffective compile-restart on rejection
  and offer a bounded matching historical failure download. Original reporter
  failure/commit is still unknown; no vehicle action or USB root-cause claim.
  See docs/egpu_persistent_rejection_20261015.md.

- On 2026-10-15, Carnival225-28 on0e647cd2 has a visible moving-to-stop corner
  born in-lane, excluded from L1 and outside-history/45m CUT-IN L2. Prior Oct14
  fallback is inactive, not a demonstrated regression. Add measured braking L2
  only with current confident lanes/vision,350ms lane/braking/physical history,
  actual speed decline and integrated range-rate agreement; preserve front/SCC
  primary and ordinary CUT-IN. Broad cruising association was rejected. All
  102 logs/502 labels: prior499 rows identical in modes1/2/3; failures11/10/13,
  predecel0/0/1 and208 unverified remain. Three new cases pass. All122,090 mode1
  frames have unchanged L1; only6 Carnival and7 reviewed Azera L2 samples differ.
  Carnival first66.35m/video16.602s, only6 samples:1.45s before replay front or
  about1.3s before recorded front. Tighter obstacle input is not MPC/closed-loop
  braking validation; the later evidence gap remains.1230 Windows tests pass,
  vault36 pass/4 skips. Publish both maintained branches, single-parent, verify
  automatic NAS image/page/replay; no vehicle action. See
  docs/carnival_in_lane_braking_20261015.md. Recordings remain private.

- On 2026-10-15, the user requested allowing S3 Wi-Fi HUD up to10fps while eGPU
  is active. Its live rate reader inherited the wired USB5fps cap. Give only the
  Wi-Fi reader a10fps cap, preserve lower configured rates and direct USB policy.
  Parked HUD-only restart reports9.8-10.0Hz; model/DM each1200 consecutive valid
  frames in60s, no process restart or fallback.32 focused tests pass on both
  branches. This is host refresh, not optical panel FPS. See
  docs/s3_wifi_egpu_investigation_20261015.md.

- On 2026-10-15, the user required normal model output at20Hz and requested
  upstream NAV n16 afbf2e547e / AMD52205044 (checkpoint96244b1f/12800). It changes
  only the PKL; runtime d3f09c9/90970e01 and old Jetson ONNX are unchanged. Pin
  the separate NAS n16 package on carrot-nav; preserve wip's model selection.
  Shared eGPU fixes yield200us after1ms in this exact runtime's previously no-op
  USB sleep, retaining FIFO54/core7 after lower-priority/core trials failed20Hz.
  Full-response deadline150ms, bounded failed-worker kill/reaping plus ownership
  lock, fresh-frame fallback and startup-only internal inference warmup/reset
  replace the ~3s stale fallback. Stopped-worker probe reaches internal output
  in718ms with106ms capture age; this is not fault-time20Hz or root-cause proof.
  Final native300s has6000/6000 consecutive valid model/DM frames at20.0002Hz;
  no fallback, S3 connected. Later USB link counter remains1->1, not zero.
  Nav298/wip257 focused tests pass. User authorized parked restarts/install. See
  docs/egpu_n16_stability_20261015.md for final validation and remaining limits.

- On 2026-10-15, deeper direct-eGPU analysis confirms all three fault snapshots
  were USB4-1/5Gbps, active with autosuspend off. Outstanding output_read used
  ~95–99% main-thread CPU; worker FIFO54 and DM FIFO5 share CPU7, with DM CPU
  sharply reduced during stalls. Pinned90970e01 has no-op USB sleep and zero-time
  libusb polling; older AMDSignal yield tests do not cover it. Polling/starvation
  is a supported failure-path hypothesis, not the initial GPU/bridge root cause.
  Parent close can add2s; no runtime/model change or eGPU restart performed.
  Parked retry permission pending. S3-only TCP disconnect recovers in9.217s
  without process restart, model/DM601/600 valid updates in30s. Offroad Wi-Fi
  association differs from HUD output gating. See docs/s3_wifi_egpu_investigation_20261015.md.

- On 2026-10-15, S3 home Wi-Fi delay improved after an approved single-AP
  channel1->6/20MHz change: 35% loss/217ms mean to no loss/6.7ms after removing
  the temporary AP lock. C4 had no pairing file; saved the existing key privately.
  Defer S3 TCP open until renderer readiness to avoid its2s idle timeout; notice
  new registration during USB wait.30 focused tests pass; native parked HUD runs
  typically9-13Hz, model/DM600/601 valid updates in30s with original PIDs. Physical
  panel/road validation is separate. Three new Oct14/15 NAV-RL eGPU output-read
  timeouts cause~3s model gaps and~3s-old first fallback; underlying cause remains
  unresolved. No eGPU/model/control change. Publish host fixes to both maintained
  branches, single-parent. See docs/s3_wifi_egpu_investigation_20261015.md.

- On 2026-10-14, the upper/lower signal-group trial fixes supplied qualified
  observations but not actual departure: 43,090 historical planner inputs at
  nominal/+20ms and 20 recent segments show no output changes; Santa Fe encoded
  165-frame replay still has zero releases. Keep UpperSignalGroupTrial OFFLINE;
  production guard/tracker remain unchanged. Fresh neural comparison gives
  68/73 box-eligible images versus rules47, tiles70; tiny green cores10/37 or
  tiles19/37 remain weak. Neural also fails identity/release at recorded timing.
  No retraining or vehicle change. Next target is learned whole-housing/lamp
  recognition with encounter-separated labels, not another live rule relaxation.
  Publish offline tools/tests/findings to both maintained branches, single-parent.
  See docs/signal_upper_group_trial_20261014.md; raw evidence stays private.

- On 2026-10-14, Carnival 222-1 on 0245e76f shows late front arrival and a
  stopping-path tail rejecting visible stationary corner 1193. Add missing-L1
  measured L2 only with confident actual inner lanes, stopping context, stable
  corner history and current visual agreement. Do not relax ordinary path/L1
  gates. Replay selects 11 frames at 74.32m, about0.57s earlier, not closed-loop
  stopping validation. User also requested screen + one short sound for an
  unselected, vision-missing, stable front/corner stationary pair: TTC<=4.8s,
  range<=120m, active longitudinal only, 500ms confirmation and encounter/cooldown
  suppression. Existing100 logs/497 labels are identical in modes1/2/3; existing
  failures11/10/13 and208 unverified remain. All119,689 prior mode1 outputs match;
  no advisory fires, so real positive performance is unproven.1176 wip tests pass;
  two new Carnival cases pass all modes. Publish both maintained branches and
  verify automatic NAS replay deployment; no vehicle action. See
  docs/carnival_stopped_car_20261014.md; recordings stay private.

- On 2026-10-14, Elantra route311 segments3/10 confirms86->88% memory use.
  DPathRadarD's lifetime Cap'n Proto builder leaks replaced list/error storage:
  restore a fresh builder each update, preserving controller history and fields.
  532 tests pass;2,399 recorded-input frames serialize identically.72,000 real-
  payload updates grow old RSS23.02MiB, fixed0.086MiB then flat (accelerated PC,
  not an hour of vehicle operation). Actual radar RSS grows3.31MiB across~8min;
  loggerd grows36.86MiB and remains unexplained. Do not claim the overall88%
  issue solved. Publish both maintained branches, single-parent; no vehicle action.
  See docs/elantra_memory_20261014.md; recordings remain private.

- On 2026-10-14, the user requested recovering uncommitted completed work and
  an explanation before publication. The pending October 3 changes are NNFF
  startup-notice retry, a default-OFF search-only NNFF setting, and a common
  120 ms temporary-steering warning presentation delay. Publish separately to
  carrot-nav and carrot-wip with single-parent commits after current checks.
  The user explicitly requested deleting the failed parked-blinker experiment:
  remove its uncommitted safety hooks, helper and tests; retain only historical
  findings. Do not reinstall or revive that experiment. See
  docs/uncommitted_work_recovery_20261014.md. No vehicle deployment is included.

- On 2026-10-12, 10a5 on nav ac1da01591/mode2 reproduces a real green held by
  a tiny sole red anchor and zero moving preparation after small night lamps
  repeatedly lose IDs. Add a two-housing red-to-green release: each anchor needs
  independent red history and fresh 700ms green, with no forward red conflict.
  Night-only pending-red templates and unique >=0.9 image correspondence retain
  small red identity through bounded motion; do not relax green/daytime rules.
  512 tests pass; prior 43,090-frame guard summaries match at nominal/+20ms;
  3,347 prior image comparisons have unchanged fresh colors/green boxes. New
  replay releases the reviewed green hold; image replay requests preparation
  5.60s earlier for only0.9s, cap about-0.19m/s2. Not sustained braking or a full
  stop; no vehicle install/native timing/closed-loop claim. Publish to BOTH
  carrot-nav and carrot-wip, single-parent only. See docs/signal_a5_followup_20261012.md.

- On 2026-10-12, the user reaffirmed that corrections must be committed and
  pushed to ALL maintained carrot-* branches unless explicitly excepted. This
  supersedes the earlier wip-only signal scope. Current remote targets are
  carrot-wip and carrot-nav; do not recreate retired branches or modify other
  contributors' namespaced branches. Port both signal preparation cadence and
  night red-group corrections to nav, preserving its navigation/model/runtime
  choices. Keep every new commit single-parent, preserve unrelated local work,
  and distinguish branch publication from vehicle installation and road validation.

- On 2026-10-12, the user requested solving the failed night-red preparation.
  A separate night contract now requires 2–4 persistent compact red lamps with
  compatible road-direction support, NOT a bounded distance or stop-line target.
  Same-ID current-red templates bridge a direct core <=500ms without renewing
  it. Night capture freshness is350ms; missing pair evidence is inactive with
  <=350ms confirmation retention, and fresh forward green cancels. Only night
  geometry erodes road edges by0.3+2*std (std<=1m); measured housing policy stays.
  Mode2 alone actuates; keep CPU/DM protection and original ONNX. 492 tests pass.
  Nine-minute replay enters once6.10s before original stopping, for1.1s to target
  -0.27m/s2; +20/+100ms still enter, +250ms rejects. Prior4329-input regression
  adds no entries. Not closed-loop improvement or guaranteed stop: identity loss
  and applicability remain limitations. No vehicle deployment; read-only vehicle
  remains carrot-nav135020d1. Work remains carrot-wip, single-parent only. See
  docs/signal_night_group_20261012.md and private2026-10-12-night-geometry archive.

- On 2026-10-12, the latest 10a4 evening drive has 1,063 fresh-red observer
  outputs but zero moving preparation: every confirmed red box is a night core
  or template, not a measured housing. Preserve confirmation across repeated
  accepted samples ageing beyond 200ms, for at most 450ms from capture; output
  remains inactive, and fresh new evidence/context/gap/session checks still win.
  448 focused tests pass. Recorded-input replay of 10,276 planner samples and
  +20ms delay still gives zero entries, as does an offline night lamp-ray/height
  experiment: road confidence and broad/truncated range remain blocking. Do not
  connect the failed night candidate, pretend inferred width is metric housing
  size, or claim better stopping. No vehicle deployment/model changes. Work only
  on carrot-wip with single-parent commits; preserve the separate nav checkout.
  See docs/signal_night_prepare_20261012.md and private evidence archives.

- On 2026-10-12, SignalAssistMode adds the requested live0/1/2 selection on
  carrot-wip only:0 auxiliary OFF,1 display/alerts and stopped-red departure veto,
  2 also conditional red preparation toward-0.5m/s2. Default0; old production
  enable files cannot bypass it. Mode2 uses CPU housing evidence alongside either
  driving model; preserve DM protection. Mode changes reject pre-activation
  observations;0 clears the auxiliary veto without issuing go. Native rebuild
  after pull is required for the new Params key. The user requested all signal
  trial content on wip and SINGLE-PARENT commits going forward, superseding the
  trial-only correction. Leave old topology/ref alone; no parallel trial changes.
  Range audit finds6 observations of a real red outside3.5m, all with blinker on;
  close signals mostly fail boundary confidence, not proven lateral exclusion.
  See docs/signal_color_experiment_20261012.md for validation and limits.

- On 2026-10-12, after correcting branch mistakes, the user superseded the
  earlier trial-only instruction and explicitly requested ALL carrot-signal-shadow
  contents integrated into carrot-wip. Preserve existing wip radar/font fixes.
  Continue ONLY on carrot-wip with linear, single-parent commits; no merge
  commits or parallel signal-branch development. Leave historical topology alone.
  This includes the retrained Mountain Dew eGPU selection8d736683 and its pinned
  runtime, while keeping internal/DM models and Jetson selection unchanged.
  Add SignalAssistMode:0 fully OFF,1 display/alerts and red departure veto,
  2 additionally gentle red stop preparation. This supersedes old per-file enables.

- On 2026-10-12, the user clarified that traffic-light geometry must allow
  space on both sides of the ego lane. The opt-in stop preparation now expands
  each inner lane boundary by3.5m, clipped by reliable road edges with0.3m inward
  clearance. Check original source agreement before expansion. This provisional
  search region is not signal-to-lane assignment. The3348-observation screen
  restores lateral/height support for3 observations of one real red signal;
  only1 passes the unchanged full25-140m distance-interval requirement, and none
  passes all color/context/temporal gates. Wider margins also widen distance
  uncertainty; do not claim measured range or earlier braking. 193 focused tests
  pass. Default OFF, no vehicle deployment. See docs/signal_color_experiment_20261012.md.

- On 2026-10-12, the user requested edge/lane XYZ geometry for red preparation.
  Remove position input/range constraints; use confident inner lanes or road
  edges, intersect when both reliable, and never extrapolate. Correct the new
  runtime pose reader to actual XYZMeasurement fields (previous adapter failed
  closed). 187 tests pass, including a real Cap'n Proto pose. Full-rate-context
  screening of3,348 retained observations gives9 new usable lane corridors but
  zero qualified candidates/entries. Exact frames confirm real red lamps among
  lateral rejects: do not call these false-positive removals or claim improved
  braking. Signal mounting need not align with the ego lane. Keep OFF; no vehicle
  deployment. See docs/signal_color_experiment_20261012.md and private road-geometry
  archive for assumptions and negative results.

- On 2026-10-12, the user requested red pre-deceleration before model stopping.
  Add default-OFF per-device stop_prepare_enabled and state6 e2eStopPrepare:
  confirmed measured red housing, calibrated pose/road-edge/path geometry,
  no lead or driver override; final acceleration cap ramps0.25m/s3 to-0.5m/s2.
  Release on missing/changed evidence, preserve model-stop handover and stronger
  braking, and reuse trafficSignChanged audio once with cooldown. Remove only
  visible signal track numbers. Geometry ranges are assumed, not metric truth.
  184 desktop tests and1,200 default-OFF target comparisons pass. Screening3,348
  retained night/morning observations yields zero qualified entries: do not claim
  better stopping or activate from this result. No vehicle deployment. See
  docs/signal_color_experiment_20261012.md and private stop-prepare archive.

- On 2026-10-12, during the color experiment, the user prioritized avoiding
  false green over missed detections, accepted rejecting turn-time readings,
  and suggested pose. Offline rotation variants now reject green before tracker
  confirmation, immediately remove old confirmed green and preserve red history.
  Existing red start latch holds through unknown; old green never releases it.
  Steering+speed avoids treating a stationary -39.5-degree wheel as rotation.
  Preferred pose experiment uses calibrated livePose yaw, required validity,
  uncertainty and separate arrival/sample freshness; no permissive fallback.
  Four-way3348-frame replay: pose gates898 observations with zero fresh/confirmed
  green outputs, retains the stationary real green;108 tests pass. Counts are
  not accuracy and LED/reflection false greens remain. Both variants stay
  offline, no vehicle deployment. See docs/signal_color_experiment_20261012.md.

- On 2026-10-12, the user requested a signal correction experiment. Add
  default-OFF offline conservative_green HSV38-90 consistently across housing,
  day/night seeds and templates. Exact-frame paired replay of13 full videos and
  3348 observations reduces morning green candidates5608->581, not an accuracy
  metric. Selected development boxes remove61 replayed background greens;
  21 newly reviewed frames keep37 visible green lamps and reject29/36 backgrounds.
  LED text/reflections remain; one motion-blurred real night green is lost.
  Only one of two live red miscrops reproduces green in encoded pixels.
  Guard color-confirmation probe differs from live release and is not control
  validation. 84 tests pass;21 default-OFF image results match prior code.
  No deployment, model training or live enable; keep DM GPU/start-guard policy.
  See docs/signal_color_experiment_20261012.md and its private evidence archive.

- On 2026-10-11, user requested the latest signal camera observer with both
  internal and retrained eGPU models, parked switching and performance checks.
  Integrate the already deployed 42760 signal/display revisions into this trial;
  retain its newer comparison/assist-OFF policy and preserve 7f3bfcd7 model/runtime
  selection. The old October 10 assist-ON snapshot is historical, not the current
  requested configuration. New camera neural output remains display/log only.

- On 2026-10-11, the user selected upstream 6f4c0faf eGPU on this
  carrot-signal-shadow branch only, for testing before a combined promotion.
  Wip publication 03142df861 was reverted by b09ae3ddad; wip keeps Cinque v3.
  Use retrained Mountain Dew 8d736683/12864, AMD 256f2178, isolated tinygrad
  d3f09c9b and verified NAS package comma4-big-mdm-8d736683. Adapt byte-addressed
  buffers while retaining prior dtype-bearing buffers. Empty-cache HTTPS
  install and offline reuse pass; preserve startup delivery before build/manager.
  Public ONNX catalog now has old 870a4823 but no matching 8d736683. Jetson
  stays Cinque v2; internal/DM models and control/validity/warp policies stay
  unchanged. No physical GPU, first-boot or driving validation is implied.
  See docs/mdm_retrained_20261011.md. Keep the existing signal experiment scope.

- On 2026-10-12, the user explicitly corrected the branch scope: current
  signal experiments belong ONLY on carrot-signal-shadow, not carrot-wip.
  Remove the five experimental commits5a80605d61,3f537bc13f,6df4f6296f,
  df51eed7fb,4bc5307d0b from wip by a normal corrective commit; preserve the
  trial branch, prior deployed helper/DM fixes and unrelated work. Future
  signal experiments/settings/validation are trial-only until explicitly promoted.
  Wip runtime/tools are restored byte-for-byte to85d97b3160. No vehicle action.

- On 2026-10-12, the user requested collecting unseen night/morning logs and
  evaluating signal size/height/road-edge filtering, analysis first. All52
  completed segments/169 source files are locally SHA256-verified. Morning CPU
  observer emits6792 fresh green candidate samples,6011 from day_color_core;
  reviewed branches/sky pass hue38-105. Two still-red signals receive green
  through tiny/cropped housings and adjacent cyan sky. This is not just UI hold.
  In73 selected labels, conservative geometry removes8/62 background candidates
  and neither red miscrop; an exploratory narrower-hue pixel test removes61/62
  and both miscrops but is not a validated detector or night regression test.
  No runtime/settings/model change or deployment. Prioritize sky rejection and
  complete housing/lamp alignment; road edges are supporting evidence, not lane
  assignment. Keep DM GPU protection. See docs/signal_morning_false_green_20261012.md
  and private .analysis/archive/2026-10-12-day-signal/ for evidence and limits.

- On 2026-10-11, the next drive exposed signal QCOM GPU contention with DM:
  parked A/B/A gives DM 10.6-10.8 Hz with neural GPU versus 20 Hz with CPU;
  priority15 also fails. This supersedes eGPU-implies-QCOM eligibility below.
  Auto AND explicit QCOM now use CPU comparison whenever DM or Driver View
  needs the internal GPU; never disable DM or use camera failure as permission.
  Preserve DM policies and external/internal driving-model selections. Latest
  drive neural observations at 2.33-2.67 Hz reset identity on every >250 ms gap.
  Display revision4 extends UI-only memory to two seconds from capture, unique
  same-housing unknown-ID carry without renewing old color, dim/~ previous and
  immediate new color replacement. Ten-phase replay reduces visibility dropouts
  146-162 to14, not recognition accuracy. Stopped red guard stays650 ms, warning
  ON and moving assist OFF. 163 desktop tests pass, one Linux-only skip.
  See docs/signal_dm_flicker_20261011.md for native verification and limitations.

- On 2026-10-11, the user explicitly requested red automatic-departure blocking
  AND one warning sound. Add separate default-OFF start_guard_enabled, fresh
  observation transport and stopped-only veto for both observer backends. Keep
  old moving assist OFF. Red needs two samples/150 ms; green same red-anchored
  housing needs three/700 ms with no competing forward red. Unknown/stale and
  observer restart never grant green release. Stop-only freshness is 650 ms
  for measured GPU latency; old moving-assist 200 ms stays unchanged. Gas
  overrides with 10 s cooldown. Final shouldStop/vTarget/aTarget and ACC mode
  enforce the veto; audioRefuse is once per blocked stop's departure request.
  264 checks pass, one skip. 43,090-frame nominal/+20 ms replays block both
  1095 red departures; reviewed-red green releases are zero, but green holds
  remain. Forward ROI is not lane association and detection can still fail.
  New MDM stays trial-only; auto chooses QCOM neural with eGPU, lightweight
  CPU tracker with internal model. Both sustain ~20 Hz parked; internal+QCOM
  co-load falls to ~8 Hz and is rejected. See docs/signal_start_guard_20261011.md
  and docs/signal_gpu_dual_model_20261011.md for evidence and device status.

- On 2026-10-11, the user redirected the new upstream 6f4c0faf eGPU model
  to carrot-signal-shadow for testing before a combined promotion. Revert the
  just-pushed 03142df861 model/runtime change from carrot-wip; wip keeps Cinque
  v3. Retrained Mountain Dew 8d736683 and its d3f09c9 runtime belong only on
  carrot-signal-shadow until the user approves promotion. Its verified NAS
  package is comma4-big-mdm-8d736683. The public ONNX catalog now includes old
  870a4823, but not 8d736683; keep Jetson Cinque v2 and internal/DM models.

- On 2026-10-11, the user explicitly requested connecting the new neural
  signal detector to the vehicle, engine-off camera testing and CPU/QCOM GPU
  timing comparison. Add opt-in neural_enabled dispatch in signalcolord, a
  display/log-only VisionIPC worker, hash-pinned artifacts and explicit CPU/QCOM
  backends. Preserve original ONNX/x/v and assist OFF; no control transport.
  Default remains existing observer unless explicitly selected. QCOM uses the
  existing tinygrad runtime, fixed-shape JIT and synchronous readback; no hidden
  CPU fallback. 130 desktop tests pass, one skip. Vehicle offroad/noOutput and
  runtime inventory were read, but intermittent SSH timeouts interrupted model
  upload. GPU speed/parity, new live inference and forced camera/model co-load
  remain unvalidated and must not be called installed or activated. Last checked
  vehicle HEAD fc7588d67a contains only the earlier display-continuity deployment.
  Resume private scripts/data in .analysis/scratch/2026-10-11-signal-live/ after
  connection recovery. See docs/signal_neural_live_20261011.md.

- On 2026-10-11, the user requested holding prior signal display results and
  combining upper ROI, neural detection, fast tracking and high-resolution crop.
  New 26-segment data has 5,714 observations at2.71--4.99 Hz, median gap250 ms
  and result age183 ms. Display revision3 remembers candidates up to800 ms from
  capture, marks them dim/~ previous, immediately replaces new red/green and
  never renews old color from unknown/repeated data. Camera/session failures reset
  it. This supersedes display-only no-hold policy; control freshness stays200 ms.
  Offline pinned neural+tracking on381 new frames reduces PC mean work100 to33 ms;
  faster search after loss removes a150 ms green reacquisition delay in that case.
  Dark-light loss/arrows remain unresolved. ROI NV12 copying is implemented, but
  native full-resolution camerad access, dynamic path ROI and GPU inference are
  not connected. New neural reference remains offline, original ONNX/assist OFF
  preserved. 123 related tests pass, one Linux-only skip. Vehicle install guards
  offroad/inactive Park; see private deployment.json for live activation status.
  See docs/signal_continuity_20261011.md. Keep recordings private.

- On 2026-10-11, the user requested the best distant/fast/accurate signal
  recognition approach, including upper-path ROI and overlapping frames. Offline
  pretrained Autoware YOLOX-s on 73 selected boxes improves IoU>=0.5 coverage
  from rules 49 to whole-frame 63, upper ROI 68 and two tiles 70; this is not
  exhaustive precision or vehicle timing. A separate MobileNet classifier still
  misses arrow combinations and returns unknown on dark real-green frames.
  Five-frame averaging retains old red across a green transition even after
  alignment. Prefer native-pixel ROI, neural detection, tracking and fresh lamp
  evidence; dynamic ROI/full-resolution/vehicle integration are not implemented.
  Logs confirm OS04C10; code downsamples 2688x1520 to1344x760 and has a full-res
  placeholder, whose safe per-frame access remains unverified. Preserve original
  driving ONNX and assist OFF; no vehicle changes in this investigation.
  See docs/signal_detection_strategy_20261011.md; keep all raw evidence private.

- On 2026-10-11, the post-overlay 1096/1097 drive contains 22 collected segments.
  Helper observations run 3.23--4.93 Hz; median result age is 178 ms and
  2,345/5,345 gaps exceed the 250 ms color-confirmation limit. Track 1267
  keeps identity but never confirms 13 raw green observations after red.
  Display expiry and red-history qualification also cause hiding/unknown labels.
  User requested installation before departure: display-only revision 2 uses
  current color R?/G? for unconfirmed candidates, R/G for matching confirmation.
  Preserve freshness/lost-track gates, original ONNX and assist OFF; no memory
  hold or control timing change. 68 focused tests pass, one Linux-only skip.
  See docs/signal_display_drive_20261011.md; keep all recordings private.

- On 2026-10-11, the user requested signal candidate boxes on the USB
  camera image and vehicle installation. display_enabled opts into a separate
  bounded tmpfs display endpoint; comparison's experimental observer supplies
  R/G/? boxes and track IDs. Use the road camera's actual image rectangle,
  reject wide/replay/3D overlays, expired/lost tracks and reused camera buffers.
  Display age is <=350 ms with <=250 ms/five-frame camera skew; control freshness
  stays 200 ms. Installed 64151b60ff with revalidated/comparison/display ON and
  assist OFF, original ONNX unchanged, existing camera mode 2. This supersedes
  the prior vehicle-code-unchanged status only for observation/visualization;
  the candidate remains unapproved for control. Parked verification saw 501
  valid consecutive model frames at 20.01 Hz, 501 plans, 25 renderer updates,
  an open USB device and unchanged process PIDs/control transport. 425 desktop
  tests pass, one Linux-only skip. Physical panel/road accuracy remain distinct.
  See docs/signal_usb_display_20261011.md; keep captures and telemetry private.

- On 2026-10-11, accumulated signal replay tested 37 segments / 15,422
  images and 43,090 planner updates. The opt-in candidate combines housing/core
  proposals, image-supported 500 ms identity retention with unchanged 250 ms
  color continuity and 200 ms freshness, and explicit producer-red-history reuse.
  Five nominal false-start windows are blocked, but added-latency trials still
  fail acquisition; 1094-10's new green release lacks visually verified green.
  Eight isolated parked camera-timed sequences improve nominal holds/releases
  but 1093-6 loses acquisition with +20 ms. Earlier dusk green coverage regresses.
  Do not enable this candidate for control. revalidated_enabled is only an
  experimental opt-in name, default OFF, with separate assist_enabled required.
  Vehicle remains c115ce4, original ONNX, comparison ON and assist OFF; no vehicle
  code/settings changes. 360 tests and production/prototype parity pass, not
  perception or closed-loop validation. This is hand-written OpenCV, not neural
  retraining. See docs/signal_revalidation_20261011.md. Keep raw evidence private.

- On 2026-10-11, Ioniq 5 PE 1095 segments 65/66 show two red-light
  departures without gas while original x/v changes produce trafficState green.
  Both legacy and daytime shadow detectors recorded red; current c115ce4 vehicle
  uses original internal ONNX with assist OFF and comparison-only publication.
  Same-recorded-input planner replay blocks both departures with either observer,
  but delaying daytime arrivals by 20 ms loses both holds: fresh observations
  miss stop-time acquisition and consumer confirmation resets across gaps.
  Sequential dual-engine timing penalizes the second result. No confirmed green
  tracks in this drive; manual overrides prevent continuous green-release proof.
  Do not treat replay as vehicle-response validation or enable failed candidate.
  No vehicle/model/control changes in this investigation; CPL is not supported
  as the remedy by these images. See docs/signal_1095_red_departure_20261011.md.
  Keep raw logs and captures private.

- On 2026-10-11, the user authorized installing the new signal detector for
  live comparison recording, with the same pushed code on the vehicle. Add
  explicit per-device daytime_comparison_enabled: one camera copy, independent
  legacy/daytime trackers and separate signalTrackingShadow/DaytimeShadow logs.
  In comparison mode never publish either engine to the control transport;
  install with assist_enabled=0. Missing opt-in preserves legacy operation.
  Keep original driving ONNX/x/v, CPU duty/affinity and freshness limits. The
  failed daytime candidate is authorized for observation only, not braking/go.
  329 related desktop tests pass, including actual worker logging/no-publish.
  Use guarded Park/inactive installation and verify commit/model/log identity.
  See docs/signal_live_comparison_20261011.md; keep vehicle evidence private.

- On 2026-10-11, the user demanded actual corrections and validation of the
  remaining daytime signal miss and latency failures. A compact colored-core
  plus dark-bar trial fixes the reviewed morning underpass red-to-green case,
  but full 23-segment replay exposes a new retained stop request at green in
  1094-8 after selected-track loss. Do NOT promote this daytime detector into
  the live worker: SignalTracker(daytime_cores=True) is analysis-only; default
  construction preserves legacy proposals with equivalent compute optimizations.
  10,631 trial observations match the clean port; 459 final default/trial image
  comparisons and 324 related tests pass. Park-only camera-timed stored-image
  workload reduces >200ms results from12/150 to0/107, but this is the rejected
  daytime candidate's timing, not a guarantee for the default/live pipeline.
  Keep 200ms freshness, same-track release, original ONNX/x/v and opt-in policy.
  Vehicle production remains b80db2c97d; only an isolated test folder was used.
  Validation was performed and partially failed, not left unperformed. Preserve
  failed evidence privately; see docs/signal_day_latency_fix_20261011.md.

- On 2026-10-11, the user requested latest-drive signal transition training and
  validation using lamp brightness/color changes plus original ONNX x/v. Three
  offline logistic probes were actually fitted; x/v reduced reviewed red abstention
  but did not improve held-out-encounter green detection. Original driving ONNX
  and vehicle files/settings remain unchanged; learned weights are analysis-only.
  Fix the helper's 125 ms confirmation/cadence mismatch, count consecutive color
  samples, retain bounded identity after a late result, and transport same-ID
  producer green history (>=400 ms, >=3 samples) to avoid a second missed streak.
  Keep 200 ms final freshness, 250 ms camera-gap expiry, default OFF, original
  x/v and normal departure gates. Recorded-input replay releases both known
  night holds and reaches normal departure states; 318 tests pass. Extra delay
  still loses red acquisition, and daytime housing proposals merge into dark
  background. No device deployment or vehicle-response validation is implied.
  Keep footage, labels and weights private. See docs/signal_transition_training_20261011.md.

- On 2026-10-11, first post-assist night/morning drives confirm a helper
  regression: two visible green transitions remain hold_through_unknown until
  gas/inactive override. Native stopping-function reconstruction is green on
  69/13 overlapping held frames; not a full vehicle counterfactual. Tracker
  confirmation restarts above125ms while44.1%/51.2% of real gaps exceed it.
  Same103 observations yield0 green at recorded times vs24 at artificial100ms;
  this diagnoses timing, not a deployable fix. A208ms stale result also wipes
  red identity before green; morning visible-red housing proposals are missed.
  23 driving segments,161 source files/2.21GB SHA256 verified;27,380 valid model
  results. Existing internal ONNX hash unchanged, eGPU and Jetlink inactive.
  Vehicle stays b80db2c97d with assist ON; analysis changed no vehicle settings.
  Recommend disabling assist pending timing/identity/day-detector corrections
  and both green-release/red-false-start regression validation. Parked readiness
  and old replay were insufficient; do not claim the helper is road-validated.
  Keep raw evidence private. See docs/signal_assist_followup_20261011.md.

- On 2026-10-10, after successful parked installation, user requested bringing
  the signal experiment into carrot-wip. Imported signal camera observation,
  causal tracking, optional stop assistance, offline tools and investigation
  documents from e3dc031730. Keep per-device opt-in default OFF; the user's
  installed b80db2c97d vehicle has it ON. CANFD zero-aReq/no-retry is already
  present here as2608792bdd and must not be applied twice. Original driving
  ONNX/x/v unchanged; no failed trained model is promoted. Red hold survives
  unknown, same-track green only removes the constraint, gas overrides with
  10s cooldown. Moving red uses plausible original model distance and existing
  lead/turn/speed gates. This is not lane/arrow association or guaranteed stop.
  All300 focused observer/tracker/assist/CANFD tests pass (Windows Params stub,
  OpenCV4.13). Prior parked checks belong to the signal branch; no carrot-wip
  vehicle installation or closed-loop validation is implied by this promotion.
  Preserve unrelated local edits. See docs/signal_assist_trial_20261010.md.

- On 2026-10-10, RAV4 route 0d segments 5-9 on e7987ed confirmed an
  onroadEvents cadence regression: healthy event-change bursts exceed the 1 Hz
  tracker's upper bound and the continuous readiness gate interrupts steering.
  Check event receipt/validity/liveness without fixed-rate rejection; retain
  periodic-input checks and the separately requested startup-only latch in
  7ceee11642, which includes this correction. Lateral selection alone still
  needs active cruise or AlwaysLateral. All five minutes have cruise inactive;
  no Panda TX rejection or new CAN errors. The long segment 6-7 inactivity is
  not fully explained by event cadence; live setting changes are not captured.
  166 focused tests pass; recorded-input readiness matches all 11,201 examined
  decisions in segments 5/8 after warmup. No physical steering validation.
  Keep incident data local. See docs/rav4_steering_20261010.md.

- On 2026-10-10, the user clarified that the Sonata startup fix must apply
  only until FIRST readiness, never reblock during operation. This supersedes
  e7987ed0fd's continuous readiness checks and repeated PID/curvature resets.
  controlsd/card now latch readiness once per process lifetime; disengagement,
  reengagement and later bad inputs do not rearm it. Nominal geometry is startup-
  only; measured curvature seeds the first lateral activation only. Restore
  existing torque reset behavior. Hyundai camera-SCC waits only for its first
  steering TX template, preserving the bounded first CAN command. Keep existing
  runtime safety, limits, longitudinal policy and firmware unchanged. 241 focused
  tests, 5,969 recorded-input frames and 6,000 post-startup control frames pass;
  the latter exactly match pre-fix 5479d1279a outputs/PID/ratio with input health
  changes and reengagement. Replay is not vehicle-response validation. Keep raw
  incident data local. See docs/sonata_92_startup_steering_20261010.md.

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

- On 2026-10-03, the user explicitly approved adding core7 to C3/C3X main UI
  onroad affinity: cores0,1,2,3,6,7 with SCHED_OTHER/nice19 for all UI threads.
  This supersedes the earlier core7 exclusion for this UI. C4 stays core6;
  offroad returns to cores0..3. Check each big core independently; preserve
  model/DM, control/camera, IRQ and USB cluster policies. An allowed mask is
  not a CPU quota or parallel rendering and may concentrate UI work on core7.
  Device affinity/FPS and model/DM impact remain unvalidated. See
  docs/camera_core5_trial.md.

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

- On 2026-10-01, Ioniq 5 PE `000005d3--54fb087f33--21` on eb58e6d7
  confirmed live handover mode 3. The reporter clarified first-curve inside
  positioning and insufficient turning at the street end after 35 s; this
  supersedes interpreting the complaint as vibration/catching. First-curve
  authority is full and angle tracking close; path/target and unwind timing
  need investigation, without declaring the driving model at fault. First
  0–10 s also has roughly 10 Hz original-MDPS
  column-torque oscillation with ceiling 250 and no pressed detection. In the
  tight left turn after 35 s, command reaches the existing 175-degree limit,
  then repeated torque-based override drops authority to 25; at 39.125 s actual
  wheel 51.7 versus target/command 147.1 gives 95.4 degrees error. Mode 3 stays
  waiting during those drops. Exit capture/recover then hits the raw-force veto;
  Panda TX echoes confirm 44-to-25 in 7.1 ms, followed by bounded return until
  42.79 s. No CAN/steering/model fault or decoded warning-vibration request.
  Torque/contact signals do not prove driver intent; the source of full-authority
  oscillation and physical remedy remain unresolved. Analysis only; do not infer
  that faster handover alone fixes it. See docs/ioniq5_5d3_steering_20261001.md.

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

- On 2026-10-08, the user explicitly gave standing approval to commit and push
  task-related Wiki updates to `https://github.com/ajouatom/openpilot.wiki.git`,
  including its default `master` branch. Do not ask for separate Wiki-push
  confirmation again for requested work. Continue following the authoring and
  validation rules below and preserve unrelated Wiki content.

- Before editing generated settings Wiki content or its generator, read
  `tools/docs/wiki_settings/AUTHORING_GUIDE.md` completely.
- In an existing generated Wiki setting page, edit only the matching `CARROT:MANUAL` region.
  Preserve every `CARROT:*` marker and never hand-edit `CARROT:AUTO` content.
- Verify behavior against the current `carrot-wip` implementation instead of inferring it from the
  parameter name. Run the Wiki validator and focused generator tests after editing.
- Generated setting pages carry the same authoring-guide URL in a hidden `CARROT:AUTHORING` marker
  so an agent working from the Wiki checkout alone can discover the canonical rules.
