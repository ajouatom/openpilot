# Ioniq 9 device-state frequency alert, 2026-09-30

Evidence: full rlog for `000003ab--48be46c5e6--21`, Ioniq 9 C4,
commit `1dfe513b1606f3bb3fc3243e5504f4e14ecf93b0`. Times are relative
to the first carState, monotonic 1352.917116289 s. Analysis uses the full
cereal schema. This investigation changes no runtime code or thresholds.

## Confirmed alert

- At 12.414550 s (approximately September 30, 17:29:01 KST), selfdriveState
  changes from enabled to softDisabling. The displayed text is
  `즉시 수동 조작하세요` / `장치 프로세스 통신속도 오류`, with
  `commIssueAvgFreq/softDisable` and sound request `warningSoft`.
- The selfdrived communication diagnostic identifies deviceState as the
  effective failing service: average 1.561 Hz, recent 0.823 Hz, allowed
  1.6–2.4 Hz. Its valid and alive flags are true. alertDebug and external GPS
  also appear in the diagnostic lists, but their ignore flags exclude them
  from this failure. modelV2, driverAssistance and longitudinalPlan pass.
- At 13.439737 s the event clears and the state returns to enabled, after
  1.025187 s. The alert display remains until 14.420466 s, about 2.006 s
  after onset. This is a soft-disable warning that recovered before complete
  disengagement; enabled and active remain true. Speed near onset is
  0.771 m/s (2.77 km/h), shortly after standstill.

## Timing evidence and limits

deviceState has 102 messages in the approximately 60-second segment,
about 1.699 Hz by timestamp intervals. Normal intervals are roughly
0.57–0.59 s. hardwared polls pandaStates and processes every fifth polling
iteration; pandaStates is about 8.66 Hz by intervals in this recording,
which explains much of the lower baseline relative to the nominal 2 Hz.
This is separate from the isolated delay below.

A deviceState stamped at 11.111817 s appears in logger order between
control messages stamped approximately 12.393–12.396 s, indicating about
1.28 s of generation-to-log-arrival delay. The next deviceState is stamped
12.832606 s, creating a 1.720790 s timestamp interval. These are generation
timestamps: do not equate that interval with an independently measured wire
or subscriber arrival gap. At alert onset, selfdrived reports a fresh
receive age of 1.94 ms despite failing its frequency averages.

hardwared constructs the message before its hardware reads and publishes
it near the end of the loop. The evidence points to a delayed device-status
update, but does not identify the blocking call, scheduling delay, GIL delay
or IPC contribution. Per-stage hardwared wall-time and thread scheduling
measurements would be needed to isolate that cause. CPU5 is heavily loaded
in deviceState, but that alone does not prove the cause of this delay.

## Other checks

- All three camera streams have 1,200 messages with consecutive frame IDs;
  modelV2 also has 1,200 consecutive frame IDs. Maximum camera SOF intervals
  are below 59 ms and maximum model publication interval is 67.655 ms.
- cameraOdometry, livePose, calibration, live parameters and driver assistance
  remain valid; livePose inputsOK and sensorsOK remain true.
- All 6,001 carState samples report valid CAN, no CAN timeout and no temporary
  or permanent steering fault. All 103 managerState samples have expected
  processes running, with unchanged PIDs. Nearby thermal status is green.
- Later driverDistracted1 visual notices at 31.934 s and 34.835 s are separate
  from the reported immediate-takeover warning and request no sound.

The log records sound requests, not microphone confirmation of playback.
No vehicle hardware fault or specific operating-system scheduling cause is
established. Private reproduction scripts and selected evidence are retained
under `.analysis/archive/2026-09-30/ioniq9-alert/`; raw logs stay on NAS.
