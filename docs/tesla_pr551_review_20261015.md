# Tesla PR #551 review and integration

Reviewed https://github.com/ajouatom/openpilot/pull/551 at
`c61d998ff8fdda9cbb8a1a6d5b5d0b198085b9c4` against carrot-nav
`e9b33329e4` and carrot-wip `d329752ab0`. The affected production files
were identical on both maintained branches. The nine PR files are integrated
without additional production changes, preserving the contributor's code and
tests. Publication uses one-parent commits on both branches.

## Findings

- Tesla zero acceleration is encoded as 375. The existing shared
  `longitudinal_accel_checks` sets `controls_allowed` when its raw input is
  nonzero, including this inactive request. The Tesla-specific replacement
  checks both bounds and requires longitudinal permission for nonzero physical
  acceleration without mutating permission. Other brands' policies are unchanged.
  Stock AEB, cancel-only operation and the existing reverse check remain in place.
- Inactive cooperative steering previously returned the phase-lead estimate,
  which can differ from the EPS measurement and exceed the angle envelope.
  It now sends the measurement clamped to 360 degrees while preserving the
  separate phase-lead resume state.
- Controller actuator feedback now follows the final cooperative steering
  request and holds that value between 50 Hz steering transmissions. It remains
  a requested angle, not measured actuator motion; CAN quantization is 0.1 degree.
- Cooperative torque blending runs only above 25 km/h. At and below that speed,
  ordinary lateral control remains active, the final request receives normal
  limits referenced to the previous final output, and cooperative state resets.
  The boundary uses CarState Float32 precision. Crossing it does not disable
  normal lateral control.
- The common angle controller now passes the actual output-limit flag to the
  saturation timer, matching the timer's existing contract. This affects all
  angle-controlled cars; it suppresses saturation accumulation while the output
  is being limited, without changing angle commands or curvature limiting.
- The firmware source-version regression includes the Tesla safety header, so
  this safety change participates in the existing firmware rebuild mechanism.

## Local verification

Windows Python 3.12, actual Cap'n Proto schemas, DBC encoder/parser, vehicle
model and production controller code were used. The desktop runner substitutes
only default-false Params reads and the 0.01 s control-period import; it does not
replace the control algorithms. No real Params or vehicle is accessed.

1. Before the production patch, the 54 focused cooperative/feedback/saturation
   cases gave **29 failures and 25 passes**. This reproduces the reported
   inactive-angle, feedback and saturation defects as well as the new speed
   boundary behavior.
2. After the patch, the complete Tesla car test directory, angle saturation
   tests and firmware-version tests gave **102 passes and 23 passing subtests**.
3. Baseline and patched production C safety headers were separately compiled
   with Zig 0.13, optimization and warnings-as-errors. A temporary desktop bridge
   allocates the real native CANPacket structure and adapts the old test
   forwarding signature to the current packet API. The production safety
   predicates are not replaced. This avoids the existing Python/native packet
   ABI mismatch; the missing `parameterized` test dependency was also supplied.
4. Full patched Tesla safety execution gave **97 passes, 105 passing subtests,
   38 skips and 3 failures**. All three failures are inherited
   `test_angle_cmd_when_disabled` cases (stock, longitudinal and auto-speed).
   They also fail on the baseline, whose additional acceleration-limit tests
   and 62 permission subcases fail. The full baseline reports 67 failures
   including those subcases, 95 passes and 43 passing subtests.
5. Excluding only that pre-existing common test gives **97 passes, 105 passing
   subtests, 37 skips and 4 deselections** (three concrete cases plus the abstract
   base). This includes acceleration permission loss, both acceleration bounds,
   stock AEB blocking/passthrough, reverse prevention, inactive measured angles,
   speed-wheel policy and wrong-safety-mode checks. No new failure remains.

The focused passing total is **199 tests and 128 subtests**; this is not a claim
that the unfiltered safety suite passes. Raw logs and temporary Windows bridge
are retained privately under `.analysis/scratch/2026-10-15-pr551/`.

No vehicle installation, firmware flashing, physical EPS/MCU timing or road
validation was performed. The PR author's F4/H7 build result was not independently
repeated here; the native C check above is host execution, not an MCU build.

Docs-Not-Needed: This is a controller/safety bug-fix port with a fixed cooperative
speed boundary; no user-visible setting is added, removed or redefined, and no
user-guide or Wiki edits were requested.
