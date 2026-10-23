# DHCP cleanup deadline investigation — 2026-10-23

## Scope and evidence

A previous rapid USB recovery trial latched `MobileCable` cleanup blocked after
its owned dnsmasq did not stop within two seconds. The PID had disappeared by
the later inspection. That historical event has no syscall trace; do not claim
its precise internal execution path was recorded.

The installed comma runs dnsmasq 2.91 on AGNOS Linux 4.9.103. Its ordinary idle
SIGTERM exits in milliseconds. dnsmasq `icmp_ping` / `delay_dhcp` can instead
queue SIGTERM while checking that a newly offered DHCP address is unused.
During that bounded check it polls in 250 ms intervals without servicing the
queued termination event. See the versioned upstream implementation:
https://github.com/imp/dnsmasq/blob/v2.91/src/dnsmasq.c

We reproduced that specific failure mechanism with the *installed binary* in
an isolated network namespace on the comma. A private veth pair and synthetic
DHCP DISCOVER triggered an address probe. No production USB/Wi-Fi interface,
lease database, firewall or DHCP process was changed by this isolated test.
strace recorded SIGTERM followed by continued 250 ms polling and a normal
exit(0). The original helper rejected shutdown at 2.128 s; the owned dnsmasq
exited at 3.397 s (both measured from starting the stop helper).

This establishes a real deadline mismatch and matches the historical symptom.
It does not prove every earlier cleanup or USB reader failure had this cause.

## Correction

- Wait at most five seconds for the verified process, accommodating its normal
  three-second address check and scheduling overhead. Fast exits still return
  immediately; this is not an unconditional five-second sleep.
- Use the same bound on pidfd and AGNOS /proc fallback paths.
- Keep executable, exact arguments, saved PID/start-time validation and the
  immediate pre-signal recheck. Only SIGTERM is sent; no SIGKILL/name-based kill.
- Treat an already-validated process disappearing between checking/signaling or
  while waiting as completed cleanup. A reused PID is never signaled again.
- Keep ownership records and block reconnection if the exact process remains
  live at the deadline. No forced FunctionFS rebind, firewall weakening, DHCP
  duplicate-address-check removal or model-input freshness relaxation.
- Existing outer root-helper timeout remains 15 s. No model/GPU/USB ICL power,
  driving control or scheduling-priority change is made.

## Isolated device result

| Stop policy | Trials | Helper result | Stop-helper duration |
|---|---:|---|---:|
| Original two-second deadline | 1 | rejected despite subsequent normal exit | 2.128 s failure / 3.397 s exit |
| Candidate five-second deadline | 3 | all normal, no error | 3.463 / 3.217 / 3.208 s |

This correction restores recovery instead of permanently latching blocked in
this case. It does not accelerate GPU inference or reduce ordinary frame P99.
The shared-RAM input optimization is evaluated separately after reconnection.

## PC validation

214 focused tests pass: mobile helper/framing/auto transport, owned cleanup,
shared input and auxiliary USB input-power policy. Nineteen new cases execute
the embedded ownership/stop Python code with simulated process lifecycle:
2.1–4.9 s exits, zombie/reused PID completion, disappearing process, live
process beyond 5 s, wrong executable/arguments/start identity and pidfd paths.
Wrong-owner cases send no signal; timeouts preserve ownership records.

Initial Windows checkout CRLF caused three shell tests to fail; the modified
helper was normalized to LF. A fourth permissions test ran on Linux-native
/tmp rather than Windows-mounted storage to verify mode0700. Final combined
result: 213 passed in the main run plus that one native-permission test passed.
Ruff passes for the new test module.

## Device reconnect/P99 follow-up

The candidate's actual USB owner cleanup/reopen recovered from six injected
link losses (three before send, three after send), followed by one 200 ms late
local reply. All seven expected failed warm-up requests are retained in the
report. There were eight unique RAM sessions, all beginning with reset, and
1,200/1,200 sender/receiver payload digests matched. Six retired buffers stayed
unchanged. Same GPU-submission image equality passed 128 times; 126 success
acknowledgements allowed post-consumption checks (two deliberately interrupted).

After recovery, A/B/B/A blocks of 750 frames each compared existing socket IPC
against shared RAM on the same camera/model/map/cruise workload:

| Whole JoiningModel call | Socket A (1,500) | Shared RAM B (1,500) |
|---|---:|---:|
| Mean | 36.941302 ms | 36.350218 ms |
| P99 | 37.713840 ms | 37.072065 ms |
| Max | 38.439935 ms | 37.595089 ms |
| >50 ms | 0 | 0 |

All 3,000 timed calls succeeded with map/navigation; no USB failure, extra HELLO
or camera drop was recorded in those blocks. P99 improved by
0.641774 ms and mean by 0.591084 ms. This is shared-RAM benefit,
not DHCP-stop acceleration or GPU compute reduction. The longer post-recovery
observation collected 5,393 model-output samples; only the exact A/B blocks
above are used for the comparison. Device noOutput/harness-notConnected/TX0
and offroad guards remained active. Physical HUD panel rendering was absent.

All eight net-down calls in this reconnect run succeeded, taking 262–340 ms
including helper/firewall/address work; none needed the >2 s grace. Therefore
the prolonged-exit fix is validated specifically by the isolated reproduction,
while this run validates ordinary repeated reconnection and retained RAM gain.
An additional trace-instrumented old-helper run recovered its two link losses
and one late reply; it did not reproduce the random historical failure. Do
not present the added grace as a proven speed improvement across those runs.

These are software-injected losses through the real USB cleanup/reopen path,
not physical cable-unplug tests or a road-safety certification. Installed comma
HEAD91cb2392 and Jetson source13542cc7 were unchanged. Original manager was
resumed, temporary children exited, CPU0-3/parameters/calibration restored,
and final USB status was ready/error-free. No vehicle control process was run.

The code correction is prepared locally and device-tested in a private overlay;
no push, permanent installation or shared-RAM default activation occurred.
Shared RAM remains opt-in. The reproduced DHCP deadline defect is corrected;
other reader/driver/power failure causes are not declared resolved.

Private evidence: `jetson-nvme/dhcp-cleanup-20261023` (isolated reproduction),
`dhcp-baseline-20261023` (instrumented original helper), and
`dhcp-fixed-reconnect-20261023` (candidate reconnect/performance trial).
