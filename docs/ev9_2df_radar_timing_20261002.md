# EV9 radar timing alerts, 2026-10-02

## Evidence and scope

Full cereal-schema analysis of `KIA_EV9 e81593da6ddcb8b2`, segment
`000002df--25c01be8dd--0`, device `mici` (C4), serial `b48fcbf2`, clean
commit `abe1a232d81fd5b0f8db4b7212587952514e0274`.
The incident is October 2 around 07:31 KST; October 1 in the report is
the commit date. Times below use the first carState, monotonic
52.208340447 s, as zero. The carState interval spans 59.815709 s.

Source: NAS route directory, full `rlog.zst`, 14,470,186 bytes;
SHA-256 `5f5edddf477dfecc038cd6acb61b1cc7cbbdd9011a07e7566c27389631794b1c`.
This investigation changes no runtime code, scheduling, thresholds, or vehicle settings.

## What the driver was shown

| Relative time | Recorded behavior |
| --- | --- |
| 30.065975 s | Engagement; selfdriveState becomes enabled. |
| 38.266750 s | `commIssueAvgFreq/softDisable`: `즉시 수동 조작하세요` / `장치 프로세스 통신속도 오류`. Approximately 07:31:48.75 KST, 17.45 km/h. |
| 38.316692 s | State recovers to enabled; the displayed warning persists through its minimum display duration. |
| 41.288068 s | Another communication-frequency soft-disable transition. |
| 41.351290 s | Display switches to `radarFault/softDisable`: `레이더 오류 : 차량을 재가동 하세요`, approximately 16.27 km/h. |
| 43–49 s | Further alternating communication-frequency/radar warnings and recoveries. |
| 52.203183–52.913890 s | Final group of brief soft-disable transitions and recoveries. |
| 54.759613 s | Last warning display clears. |

The longest continuous softDisabling interval is 44.118969–46.533458 s
(2.414489 s). After engagement, enabled and active remain true through the
recording; there is no completed disengagement. Warning sound requests are
`warningSoft`; logs do not establish physical speaker playback.

## Direct failure: radar output cadence, not a reported radar ECU fault

The first active-driving communication diagnostic identifies radarState:
long-window frequency 15.990 Hz, recent frequency 15.948 Hz, required
minimum 16 Hz. It is alive and valid at that instant, with receive age
2.466 ms. modelV2, driverAssistance and longitudinalPlan pass their checks
at approximately 20 Hz. alertDebug appears in the diagnostic but has all
three checks ignored and does not cause the warning.

In the 38–49 s window:

| Stream | Messages | Timestamp-based rate | Invalid messages |
| --- | ---: | ---: | ---: |
| modelV2 | 220 | 19.994 Hz | 0 |
| liveTracks | 220 | 20.010 Hz | 0 |
| radarState | 176 | 15.946 Hz | 78 |

Across the complete segment, all 813 radarState messages have every
radarErrors flag false (`canError`, `radarFault`, `wrongConfig`,
`radarUnavailableTemporary`). Nevertheless, 108 envelopes are invalid,
including 83 after 30 s. liveTracks has 1,197 messages: only its first,
at 0.104543 s during startup, is invalid with canError. The startup log
reports `radarcan input invalid: staleEgoState`; subsequent 1,196 inputs
are valid with all error flags false. This startup sample does not explain
the later driving warnings.

The incident revision's radard sets its envelope validity from
`sm.all_checks()` across subscribed inputs. That includes observed
receive frequency, not just the radar interface's error flags. selfdrived
maps an invalid radarState without canError or temporary-unavailable flags
to the generic radarFault event. Thus the displayed label does not
establish a defective radar sensor.

The slowing consumer also polls/samples its conflated input sockets less
frequently. This explains how healthy 20 Hz producers can fail radard's
own frequency checks. Exact per-input radard frequency-filter values were
not logged; the specific failing input check cannot be independently
reconstructed exactly from producer timestamps. The independently logged
selfdrived radarState frequency failure is direct evidence.

## Cause: insufficient CPU5 execution budget for card and radard together

Code at the incident revision places card on core5 at SCHED_FIFO 53 and
radard on the same core at SCHED_FIFO 51. procLog confirms processor 5,
priorities -54/-52, and stable PIDs 53043/53067. radarcan and plannerd are
on processor 4. Relevant code is unchanged between the incident commit
and this investigation's checkout.

Runtime summaries ending within 30–54 s show:

| Metric | card | radard |
| --- | ---: | ---: |
| Observed work-loop rate | 99.98 Hz | 16.11 Hz |
| Mean CPU time per work iteration | 6.153 ms | 22.553 ms |
| Mean elapsed work time | 6.082 ms | 61.387 ms |
| Maximum elapsed work time | 9.831 ms | 84.177 ms |
| Main-thread scheduler CPU / elapsed | 62.31% | 36.89% |
| Main-thread runnable wait / elapsed | 0.0095% | 62.91% |

The diagnostic spans are independently aggregated (24.145 s for card,
23.775 s for radard), rather than exactly identical interval boundaries.
card's CPU timer starts earlier than its elapsed work timer, so those two
card means must not be subtracted to estimate scheduling delay.

All 42 deviceState samples in 30–54 s report CPU5 at 100%. radard's
recorded runnable wait is about 0.63 seconds per elapsed second, while
card's wait is negligible. This is measured scheduling contention, not an
inference solely from CPU utilization. The higher-priority card consumes
most of the CPU budget; radard runs in the remaining time and cannot
sustain its required 50 ms period.

At the measured per-frame cost, radard at 20 Hz would need about 45.1%
of one core, plus card's approximately 62.3%, exceeding a core's capacity
before other work. Actual radard work stretches to approximately 61 ms
and its output crosses the 16 Hz alarm boundary. In one representative
card summary near 41.24 s, vehicle interface update costs 2.153 ms/frame
and control apply costs 2.243 ms/frame, within a 6.177 ms total stage
measurement. This identifies substantial consumers, not a proven new
regression within either function.

This evidence strongly supports core5 budget contention as the immediate
cause of the radar processing slowdown. It does not identify every
preemption (no scheduler trace), establish when the workload regressed,
or profile individual radard functions. Runtime log delivery is sometimes
delayed in batches: summaries are aligned using their embedded mono_time,
not logMessage envelope time.

## Exclusions and separate startup behavior

- All 5,983 carState messages have valid CAN, no CAN timeout, and no
  temporary/permanent steering fault. There is no reported radar fault
  flag during the driving incident.
- managerState has no expected process stopped; card, radard, radarcan,
  modeld and plannerd have unchanged PIDs. This is not a radard crash/restart.
- Startup includes 8.3 s model loading, camera-pair waits and initial model
  frame skipping. modelV2 first appears at 10.277106 s; its next frame jumps
  by 17, then all remaining frame IDs are consecutive. After 12 s livePose
  validity, inputsOK and sensorsOK all pass. That startup behavior is
  separate from the later radar output slowdown.
- Camera streams each contain 1,203 messages, with maximum SOF intervals
  under 62 ms. Thermal state remains green near the incident; reported
  CPU temperatures are around 56–61 C. No heat warning explains this event.
- Radar inputs contain frontRadar, corner235 and SCC objects (up to 39
  points/frame). Neither ECU replacement nor a radar alignment fault is
  established by this recording.

## Follow-up direction

### Follow-up: whole-core utilization and vehicle differences

These are arithmetic means of recorded deviceState.cpuUsagePercent samples,
which hardwared obtains from psutil.cpu_percent(percpu=True) and rounds to
integer percentages. They include all work on the core, not selected-process
CPU totals. The steady window excludes startup/model loading (12–59.7 s,
83 samples). The warning window spans the first driving warning through the
last display clearing (38.266750–54.759613 s, 29 samples), including brief
recoveries between warnings.

| Core | Steady mean % | Warning-window mean % | Warning-window maximum % |
| --- | ---: | ---: | ---: |
| 0 | 67.58 | 63.86 | 85 |
| 1 | 64.98 | 60.62 | 71 |
| 2 | 65.02 | 61.14 | 73 |
| 3 | 79.42 | 77.86 | 84 |
| 4 | 56.57 | 60.79 | 100 |
| 5 | 99.98 | 100.00 | 100 |
| 6 | 88.18 | 89.45 | 93 |
| 7 | 33.39 | 34.59 | 43 |

CPU5 is 100% in 82/83 steady samples and 98% in the remaining sample.
It is already 100% in all 14 engaged-before-warning samples (30–38.266750 s)
and remains 100% in all nine after-warning samples (54.759613–59.7 s).
The warning clearing therefore does not establish recovery of CPU headroom;
frequency filters can pass again near their lower boundary.

procLog CPU deltas over complete intervals within 30–54 s corroborate the
main consumers: card 62.50% and radard 36.91%, both sampled on processor 5;
radarcan 24.00% and plannerd 20.36% on processor 4; UI 33.18%, selfdrived
26.36%, controlsd 20.77% on processor 6; modeld 20.68% and
dmonitoringmodeld 12.91% on processor 7. pandad contributes 54.45% of one
core's CPU time and is sampled on processor 3. These process totals and
last-processor observations are supporting evidence, not a complete per-thread
core attribution. Other tasks and kernel work are included in deviceState.

Vehicle-dependent cost can vary with CAN message rate/layout, controller
branches and radar object count/source, even with identical core assignment.
In this EV9's 12–59.7 s window, liveTracks averages 25.93 frontRadar,
4.73 corner235 and 1.00 SCC points per update (maxima 32/8/1).
That is a measured workload; no matched log from the user's own car has
been compared, so it does not establish which vehicle-dependent difference
explains the absence of warnings there. Hardware/runtime and enabled features
also need to match for an informative comparison. A seemingly healthy car may
also remain just above the warning boundary.

Core7 has measured average headroom, but its model/DM deadlines and peak
workload still need evaluation before moving radar work there. No placement
change is made or validated by these measurements. CSV samples and the
reproduction script are retained in the private archive's `cores` directory.

Recover a sustainable 20 Hz radar pipeline by profiling/reducing card and
radard CPU cost and/or revisiting their shared-core placement. Check the
full per-core workload before choosing another core. Raising radard above
card would risk delaying vehicle CAN/control work and is not a validated
solution. Lowering frequency/validity thresholds would hide the recorded
overload rather than recover the missing processing rate.

Separate input/processing timing failures from sensor fault wording in
diagnostics, while preserving fault enforcement. Any implementation needs
recorded-input equivalence and on-device timing validation. No such fix
or vehicle validation is claimed by this analysis.

Private scripts and selected evidence are retained at
`.analysis/archive/2026-10-02/ev9-2df-radar/`; raw logs remain on NAS.
