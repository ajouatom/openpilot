# Ioniq 5 PE 07b parked blinker experiment, 2026-10-02

## Retired on 2026-10-14

At the user's request, the failed experiment's uncommitted safety extension,
production-header hooks, host tool and dedicated tests were deleted. The normal
Hyundai CAN-FD safety header is unchanged from the published version. The code
descriptions below record the historical experiment; these tools are no longer
available in the maintained branches. This cleanup performs no vehicle action.

## Result

The user requested a live, observed test and then authorized the necessary code
changes. On `07b62e389ed26c81`, a bounded experimental Panda build transmitted
the UDS extended-session request to **bus 0 / 0x7B1**, confirmed by Panda bus-128
TX echo. There was **no expected 0x7B9 response**. Consequently the tool did not
send the communication-disable request or any SPAS lamp-control frames. This
does not establish that the vehicle cannot control blinkers: the diagnostic
address, accessible bus/gateway and supported session remain unverified.

The original H7 firmware was restored and its signature and reported version
`DEV-14cc6268f895-DEBU` verified. Normal `hyundaiCanfd` safety parameter 157,
valid CAN, P, zero speed, parking brake, disabled/inactive control, valid model
output and running manager processes were confirmed after restoration. No
production source or saved user setting on the car was changed. Actual lamp
actuation, ECU silence and post-silence restoration were **not exercised**.

## Evidence

- Existing route segments `00001005--f5fbe6ee48--3` and
  `00001006--475cef72a1--3` each contain 3,000 bus-0 32-byte SPAS2 frames at
  approximately 50 Hz. DBC BLINKER_CONTROL is zero throughout, including a
  period with leftBlinker recorded true in the first segment. ENABLE_BLINKERS
  is false; these are not recordings of automatic lamp commands.
- Live preflight independently confirmed EV accelerator/gear 0x35 (P and no
  accelerator), shifter 0x130 (P), all four raw wheel speeds 0xA0 exactly zero,
  TCS 0x175 parking brake value 1 and neutral, CRC-valid SPAS1/SPAS2.
- Final trial echoes, bus 128: `0210030000000000`,
  `0328000100000000`, `0210010000000000`, all at address 0x7B1.
  The latter two are recovery requests to enable normal communication and
  return to the default session. No `280101` disable request was transmitted.
  No expected diagnostic response and zero SPAS TX echoes were recorded.
- An earlier attempt stopped at firmware-version comparison before any trial
  request. The existing version endpoint returns sizeof(gitversion)-1 although
  the generated array excludes a trailing NUL, so the reported DEBUG suffix is
  DEBU. This experiment did not change that endpoint or generation behavior.
- The next attempt's first diagnostic request was rejected by Panda. A host
  receive-queue timing discrepancy was a candidate, not a proved cause. The tool
  now discards queued traffic after mode selection and requires 300 ms of
  continuous new parked observations. The final trial was accepted by Panda.

## Code and isolation

`safety_hyundai_canfd_blinker_test.h` provides an opt-in service-bay parameter
bit 1024. Only debug builds with the exact tested configuration `157 | 1024`
can operate it. Normal vehicle initialization never selects it. With the bit
set, the host TX allowlist is replaced with only 0x7B1/8, 0x165/24 and 0x16A/32
on bus 0; steering, acceleration and all other host addresses are denied.
Forwarding follows the existing policy. No allOutput mode is used.

The firmware independently validates CRC, advancing counters, exact layout and
100 ms freshness for the parked inputs. Two consecutive CRC-valid samples are
required. Test start also requires fresh neutral stock SPAS. It allows one
extended-session request per mode initialization, and permits TX-only
communication disable (`28 01 01`) only after the positive session response.
The disable operation itself must receive `68 01`; both stock SPAS streams must
then be silent for at least 100 ms before host SPAS transmission is allowed.
Resumed stock traffic aborts instead of competing with it.

SPAS payloads preserve the last observed original body except CRC, sequential
counter and the three blinker bits. Only off/left/right are allowed. There is
a 20 ms minimum send interval, 100 ms host-gap abort, a two-second nonzero
request budget without renewal and a four-second overall session budget.
There is no tester-present, reset, security access, flashing service or arbitrary
IO-control request in the diagnostic allowlist. Explicit communication-enable
and default-session requests remain available after an abort.

`openpilot/selfdrive/debug/car/parked_blinker_test.py` defaults to a non-sending
plan. Execution requires the exact dongle, reviewed firmware version and
exclusive Panda ownership while the normal manager/card/pandad stack is stopped.
It attempts 1.5 seconds of one direction followed by zero requests, explicit
diagnostic cleanup and confirmation of resumed valid neutral SPAS. Ordinary
interrupts/disconnects trigger cleanup. After the live result, cleanup reporting
was refined: if communication disable was never attempted, existing stock SPAS
is reported separately from an unconfirmed diagnostic-session reset, preserving
the original session-request failure as the primary result.

The live supervisor temporarily paused the two managers, stopped their children,
verified the original firmware against the rollback file, flashed only the main
application, ran the trial, restored the original application and resumed the
managers. Bootstub, production checkout and saved configuration were preserved.

## Validation and limitations

- Production C TX/forward-hook tests plus the existing cluster-forwarding tests:
  **201 passed**. This includes a simulated ECU running the real Python trial
  against the real C safety hook, success and refusal/no-response/movement cases.
- ARM GCC 13.3.1, unchanged `-Werror`: F4/H7 main and bootstub builds and main
  development signing pass. Trial build source identity: `fb5591243078`.
- Python lint, compilation and default non-sending invocation pass.
- Timer and host-gap limits bound accepted requests, not the physical lamp-on
  duration or ECU restoration after power loss/SIGKILL. ECU S3 behavior has not
  been measured. A dead/disconnected ECU cannot be made to acknowledge cleanup.
- Do not enable this path for ordinary onroad automatic blinkers or infer that
  sending an observed status message can actuate a lamp. Investigate the correct
  diagnostic route before another actuation attempt.

Private reproduction scripts, build/validation evidence and vehicle diagnostics
are retained under `.analysis/archive/2026-10-02/parked-blinker/`. No raw settings
or credentials are included in tracked files.

## Public ECU-address research after the trial

Research on 2026-10-02 found useful discovery candidates, but no verified Korean
IONIQ 5 PE blinker IO-control identifier or actuation payload. No additional
vehicle diagnostic requests or firmware changes were made for this research.

| Request address | Evidence | Meaning for 07b |
| --- | --- | --- |
| 0x7B1 | Upstream opendbc identifies parkingAdas; EV Dashboard Build 24 reports parking-sensor polling here. | Supported discovery candidate; our bus-0 session request had no expected response. Neither identity nor lamp control is confirmed on 07b. |
| 0x770, expected reply 0x778 | OBDb IONIQ 5 definitions read light status with service 0x22 and DIDs BC03/BC06. | Body/light diagnostic candidate, not a verified PE mapping or write command. |
| 0x7A0, expected reply 0x7A8 | EV Dashboard Build 105 corrects the module label to IBU; OBDb reads tire data here. | Separate body-unit identity candidate; do not equate it with 0x770 or assume either directly drives the lamps. |
| 0x7C4 / 0x7D0 | Local PE firmware reference contains camera 99211-GI500 and radar 99110-GI500. | Useful discovery controls, not firmware identities measured from 07b; the reviewed 07b carFw records were empty. |

Upstream and local Hyundai firmware-query configuration use ReadDataByIdentifier
`22 F100` and the alternative `22 F110` on buses 0 and 1. The latter explicitly
includes CAN-FD HVAC/parking ADAS discovery. A missing reply to `10 03` on one
bus therefore does not prove a wrong address or absent ECU. Camera-harness access
and the diagnostic connector may expose different routes; this is a hypothesis
to test on 07b, not a demonstrated gateway restriction.

The next diagnostic experiment should first identify the known camera/radar and
the three candidate addresses using bounded default-session identity reads, with
request/reply address, physical connection, Panda bus, payload and negative
response code retained separately. Negative UDS replies establish a responding
endpoint but not support for the requested DID. An absent reply does not identify
the cause. If the camera connection cannot reach body diagnostics, evaluate the
OBD diagnostic route before any further actuation attempt. Finding an ECU is
separate from establishing its supported lamp command and release behavior.

The existing `openpilot/selfdrive/debug/car/ecu_addrs.py` provides address
discovery, but its wrapper changes IsOnroad/FirmwareQueryDone and its underlying
helper sends a batch over the whole 11-bit and 29-bit diagnostic ranges. It was
reviewed, not executed against the running vehicle. A targeted, paced identity
query is more appropriate for the current candidates.

Sources (public implementation/research, not an OEM PE actuation specification):

- [commaai Hyundai query configuration](https://github.com/commaai/opendbc/blob/master/opendbc/car/hyundai/values.py)
- [OBDb IONIQ 5 signal definitions](https://github.com/OBDb/Hyundai-IONIQ-5/blob/main/signalsets/v3/default.json)
- [EV Dashboard developer history, Builds 24 and 105](https://www.theburl.com/ioniq-app/versions)
- [Hyundai TSB 26-BE-009H](https://static.nhtsa.gov/odi/tsbs/2026/MC-11031396-0001.pdf)
  names a BDC on specified 2025 NEA EV vehicles. It supplies no applicable lamp
  diagnostic address and does not establish the body topology of Korean 07b.

## Live ECU discovery on the user's request

The user then explicitly requested continuing while parked and clarified the
connection is at ADAS/ECAN. Identity discovery was executed on the original
Panda firmware, with the normal managers temporarily paused and their managed
processes stopped. The original firmware signature was checked unchanged after
each pass. The diagnostic mode was ELM327 parameter 1 (normal harness routing),
not allOutput. A continuously fresh, CRC/counter-qualified raw CAN guard required
P, zero wheel speeds, parking brake and no accelerator. No ECU coding, security
access, DTC clearing, communication disable or lamp-control request was sent.

Actual 07b identity responses:

| Request / response | ECU's own F197 name | F187 part number | Observed normal-route buses |
| --- | --- | --- | --- |
| 0x730 / 0x738 | ADAS_DRV | 99810GI520 | 0, 2 |
| 0x7B3 / 0x7BB | ATCU | 97255GIAA0 | 0, 2 |
| 0x7B7 / 0x7BF | RR_C_RDR | 99140GI500 | 0, 2 |
| 0x7C4 / 0x7CC | FR_CMR | 99211GI520 | 0, 2 |
| 0x7C6 / 0x7CE | CLU | 940C3GI400 | 0, 2 |
| 0x7D0 / 0x7D8 | FR_RDR | 99110GI500 | 0, 1, 2 |

F197 names were read on bus 0 (also the radar on bus 1); bus-2 identity reads
were in the first pass. These are six distinct diagnostic request addresses,
not thirteen separate physical ECUs. Under the diagnostic mode's restored
harness relay, bus-0 and bus-2 captures include copies of the same requests and
responses. The camera's actual F100 software string is
`NE  MFC  AT KOR LHD 1.00 1.10 99211-GI520 260204`, different from the local PE
reference entry. This identifies the installed module, not a control capability.

The first pass completed 807 identity requests: targeted F100/F110 reads on
buses 0/1/2, a paced 0x700..0x7F7 F100 sweep on each, and F110/F187/F191 follow-up
for responding endpoints. The second normal-route pass completed 581 requests:
targeted TesterPresent `3E00` and F197/F100/F110/F187/F191 reads, followed by a
0x700..0x7F7 TesterPresent sweep on buses 0/1. Positive replies and NRC 0x31 were
distinguished. Neither pass found another diagnostic address. The three body/
parking candidates 0x7B1, 0x770 and 0x7A0 returned no response to those targeted
requests. Negative/absent identity responses must not be interpreted as proof
that a module is absent from the vehicle. This was not a 29-bit/subaddress or
all-DID scan.

An intervening ELM327 parameter-0 OBD-mux trial using classic CAN had no ECU
response. Its initial link health showed error-warning/error-passive and a TX
error counter of 128. The supervisor was interrupted and restored after 141
requests; 140 result rows completed before interruption. Its echo-only link
gate was insufficient: H7 `fdcan.h::process_can` emits TX echoes when placing
frames into the hardware FIFO, before confirmed wire delivery/acknowledgment.
Do not treat those echoes or `total_tx_cnt` as an ECU acknowledgment. A proposed
single-probe FD variant with a link-error gate was prepared locally but not run
after the user clarified the ADAS/ECAN connection. Switching the mux does not
create a physical OBD connection. The prototype scripts are retained as evidence,
not a general-purpose diagnostic utility.

Finally, the earlier 0x7B1 session check was repeated with explicit `fd=True`:
one `1003` followed in a finally block by `1001` on each of normal-route buses
0 and 1. All four were accepted into Panda's TX path; there was no ECU response.
No communication-disable or SPAS request followed. The original blinker trial's
diagnostic send did not explicitly set FD, so that earlier lack of reply alone
was not a sufficient protocol check. This follow-up still found no reachable
parking ECU through the tested method.

All passes ended with original firmware `DEV-14cc6268f895-DEBU` unchanged and
managers resumed. Final telemetry confirmed all five observed services alive
and valid (carState, selfdriveState, pandaStates, managerState, modelV2), valid
CAN, P/zero/parking brake, inactive control, hyundaiCanfd parameter 157, no Panda
faults or safety RX invalidity, and no unexpectedly stopped managed process.
All three current CAN receive/transmit error counters were zero; historical
error counters are not claimed cleared. Physical lamp actuation remains untested.

Current conclusion: known diagnostic services work on the connected ECAN
path, but the tested body/parking endpoints are unreachable by these requests.
The exact cause (routing, ECU organization, address/protocol or access condition)
is unresolved. Do not disable ADAS_DRV merely because it is the reachable ADAS
module, or assume its address substitutes for parkingAdas. Further progress
requires evidence from the vehicle's actual OBD diagnostic connection or a
verified PE-specific lamp-test transaction. Evidence, scripts and restoration
telemetry are in `.analysis/archive/2026-10-02/ecu-discovery/`.

## Further public search using actual module identity

Searching the measured 99810GI520/99810-GI520 identity did not locate a verified
PE lamp-control transaction. Two more relevant references were found:

- comma's [2022 Taco Bell drive report](https://blog.comma.ai/taco-bell/)
  explicitly documents enabling automatic blinkers on its Kia EV6. Its linked
  [commit 9ee80155](https://github.com/commaai/openpilot/commit/9ee80155e837ec82b0f4a1880e599bc705f3a550)
  enables the blinker flag for EV6 and changes state feedback to the stalk.
  This is first-party evidence of working control on that vehicle, not PE
  validation. The public Hyundai code still provides SPAS1/SPAS2 generation
  and a separate 0x7B1 communication-control path; merely enabling the flag
  cannot establish that those prerequisites are met on 07b.
- An [INPRONIC forum author's IONIQ 5 UDS investigation](https://inpronicusa.com/community/forum/topic/216128/reverse-engineering-of-uds-actuator-frames-in-the-hyundai-ioniq-5)
  describes ThinkDiag/trainer analysis of IO-control and mirror status/output.
  It omits a verified PE model year, target ECU address and left/right lamp
  control DID. Its illustrative request is not a validated blinker command
  and must not be copied into a live-vehicle test.

These sources support further investigation, not a claim of universal ECAN
inaccessibility or guaranteed success through OBD. The missing evidence remains
the PE-specific route and accepted lamp command/release transaction.

## GitHub issues, PRs and community follow-up

Public GitHub code, issues, PRs and selected discussion comments were searched
for SPAS, blinker control, IONIQ 5 PE and the measured ADAS part number. This was
not access to private Discord channel history or an exhaustive search of all
forks/branches. No message was posted to any community.

- [openpilot issue 27885](https://github.com/commaai/openpilot/issues/27885)
  reports parking-sensor/camera and parking-assist disruption on a 2022 EV6
  using experimental taco2 changes. In the
  [maintainer reply](https://github.com/commaai/openpilot/issues/27885#issuecomment-1506012706),
  sshane confirms that the experimental turn-signal method disables the parking
  ADAS ECU and has some of these side effects. This is direct evidence of a
  cost of that implementation, not evidence that every reported feature or
  every PE vehicle behaves identically. It does not authorize substituting
  the reachable 0x730 ADAS_DRV for the unresponsive 0x7B1 parking endpoint.
- [openpilot PR 26772](https://github.com/commaai/openpilot/pull/26772)
  distinguishes EV6 stalk input from lamps commanded by openpilot; a reviewer
  also notes that the selected message is not present on all cars.
- [opendbc PE port PR 1676](https://github.com/commaai/opendbc/pull/1676)
  and [sunnypilot/opendbc PR 119](https://github.com/sunnypilot/opendbc/pull/119)
  concern angle steering and identification. The reviewed descriptions and
  discussion comments did not establish PE exterior-lamp actuation.
- [OlamideD/sunnypilot issue 7](https://github.com/OlamideD/sunnypilot/issues/7)
  independently asks about exterior turn-signal requests on a 2025 Canadian
  non-HDA2 ccNC Sonata. It explicitly treats SPAS applicability and bus access
  as unresolved research, with no comments or verified actuation result at
  review time. It is not a PE solution.
- Default-branch code search finds the same SPAS2/BLINKER_CONTROL implementation
  in upstream and several forks. Repeated source copies are not independent
  vehicle validation. Exact code search for 99810GI520 returned no result.
- Public Sunnypilot lane-change settings describe reacting to the driver's
  blinker input; this must not be mistaken for commanding the exterior lamp.
  Public web searches, including Discord-related terms, yielded no verified
  PE command/response capture. Internal Discord conversations remain unread.

The research adds working EV6 precedent and a documented side effect, but still
does not supply 07b's accepted lamp command, release transaction or diagnostic
route. Live vehicle state was not changed during this follow-up research.

## Public diagnostic-capture owners and remaining gap

The user's follow-up asked whether anyone already has diagnostic measurements.
Public evidence does identify investigators, but not a verified PE blinker
actuation capture:

- INPRONIC author Alex.c (linked above) claims ThinkDiag actuator-frame analysis
  using an ECU trainer. The post has no downloadable timestamped trace or
  PE-specific blinker command/response/release sequence. It remains an unverified
  lead for requesting original data, not proof that the author has that sequence.
- [gburlingame/ioniq-app issue 14](https://github.com/gburlingame/ioniq-app/issues/14)
  includes Greg Burlingame's analysis of diagnostic reads across 2024/2025 Ioniq 5
  and 2026 Ioniq 9, with tester feedback and a brake-status decoding correction.
  These are status measurements, not lamp actuation. A comment separately
  identifies a stalk ECU at 0x7A6 with part 934B2-GI580, but does not establish
  that endpoint or a write capability on 07b. It is not a replacement control
  address. The earlier 07b presence sweeps already included 0x7A6 without finding
  an additional responder.
- [tylerharvey/Ioniq5_CAN](https://github.com/tylerharvey/Ioniq5_CAN) publishes
  real SavvyCAN CSV logs and names contributors Tyler, Roy/dragz and Liz/L1Z3.
  The documented working control is preconditioning on 2021-2024 E-GMP vehicles,
  not a diagnostic-tool PE lamp test. Its README links a Fluxer community and
  [Discord mirror](https://discord.gg/zKyNAfmtU), providing a specific community
  lead; neither internal channel was read or contacted.
- [dragz's UDS examples](https://github.com/dragz/explorationsincarhacking/blob/main/articles/obdb_and_udsoncan.md)
  provide diagnostic data-reading examples, without a PE blinker actuation trace.

The accurate search result is therefore "investigators and related measurements
exist; the required PE lamp actuation trace has not been located publicly."
It is not "nobody has measured this." No community contact was sent.

## Broader 2025+ Hyundai/Kia search

The follow-up widened the search to ccNC EV6/EV9, Sonata, Tucson and current
diagnostic-tool coverage. Model year alone does not establish a shared lamp
command, diagnostic endpoint or reachable network route.

- Bosch's [ADS X 6.8 announcement](https://www.boschdiagnostics.com/node/1835140)
  explicitly includes model-year 2025 Ioniq 5/5 N/6, EV6 and EV9 in its added
  diagnostic/data/bidirectional coverage. The manufacturer's
  [release PDF](https://www.boschdiagnostics.com/sites/default/files/2025-05/6.8%20Release%20Notes_Bosch_Final_0.pdf)
  lists front left/right turn-signal output tests on page 6 and rear lamp
  outputs on pages 9-10 in the Hyundai/Genesis section; Kia has corresponding
  items on pages 12-15. This is concrete vendor evidence of lamp actuation
  functions, but the document does not map each test to each vehicle/market
  or publish addresses, services, payloads, prerequisites and release requests.
  It does not prove that the Korean 07b PE supports a particular listed test.
- In an [EV9 CAN research discussion](https://www.reddit.com/r/KiaEV9/comments/1w6k74t/anyone_reverse_engineer_the_can_bus_codes_for_ev9/),
  TheGamingGallifreyan reports extracting roughly 4,000 lines of control items
  from an EV6 ccNC head-unit firmware, mainly HVAC and ambient lighting, and
  claims substantial similarity across ccNC Hyundai/Kia vehicles. This is a
  firsthand research lead, not a published PE exterior-blinker capture. The
  linked [ccNC forum thread](https://www.kiaevforums.com/threads/root-access-achieved-to-ccnc-infotainment-systems.16595/)
  could not be read through the available requests; its contents and private
  Discord history were not verified. No contact was sent.
- The [Zenith September 2025 vendor bulletin](https://upcdn.zenithdiag.com/web_PCUTIL/2025_September_Update_Distributor_Bulletin.pdf)
  adds USA Ioniq 5 NEA EV 2025-2026 and other recent Hyundai models. A coverage
  entry alone is not proof of a specific exterior-lamp test or protocol.
- A Speedsignal EV9 product initially looked promising, but its manufacturer's
  B-3450223-W installation guide, reproduced at
  [Manualzz](https://manualzz.com/doc/90241656/speedsignal-3450223-w-owner-s-manual),
  describes reading CAN lamp states and producing analog outputs. It is not
  evidence of commanding the factory indicators. Its PDC connection is not a
  verified wiring or routing recommendation for the Ioniq 5 PE.
- The examined HiCOM EV9 support listing has generic UDS ECU entries with zero
  listed actuators. The public canair capture repository examined here uses a
  2017 Ioniq profile; 2026 capture dates must not be mistaken for vehicle years.

The strongest new result is explicit commercial-tool lamp-actuation coverage
alongside 2025 vehicle support. The still-missing artifact is a model/module-
identified lamp ON/response/OFF capture, including its diagnostic connection.
Neither the coverage list nor a ccNC infotainment firmware similarity claim
supplies that artifact. No vehicle commands or production code changes were
made during this research follow-up.

## ccNC-specific follow-up: named table and access limitation

Further author-post tracing found a more concrete artifact than the earlier
generic reference to thousands of control items:

- In [the author's CarHacking post](https://www.reddit.com/r/CarHacking/comments/1tyiplk/rooted_my_2025_kia_ccnc_head_unit/),
  TheGamingGallifreyan specifically names `CANFD_MASTERTABLE` in the decrypted
  2025 Kia ccNC firmware, describing documented transmit/receive IDs available
  on the head-unit bus. This is the author's report; the actual file has not
  been obtained. Exact-name public web and GitHub code searches returned no
  downloadable table in this pass. A HU-bus table is not a vehicle-wide DBC or
  a UDS actuator-DID list.
- The author's [detailed EV6 progress post](https://www.reddit.com/r/KiaEV6/comments/1tvcnz3/progress_update_on_my_ccnc_head_unit_hack/)
  reports working ambient-light and front-window control, but explicitly says
  headlight/taillight control is on another bus that the HU cannot write to.
  This is evidence against treating HU access as unrestricted exterior-lamp
  access. It neither specifically resolves turn-signal actuation nor proves
  the same restriction on 07b's separate ADAS/ECAN connection.
- Both posts publish the project's [Discord invitation](https://discord.gg/BNvMhpAbrD).
  This is a newly identified, specific ccNC research community, distinct from
  the earlier Ioniq5_CAN mirror. Internal channels were not read and no message
  was sent. The author's [bench update](https://www.reddit.com/r/kia/comments/1vvkos1/got_my_ccnc_research_test_bench_set_up/)
  describes K4/K5/EV6 bench components and says documentation is still limited.

ccNC is therefore a useful firmware/message-definition research lead, not a
confirmed lamp-controlling ECU or proven explanation for 07b's absent diagnostic
responses. The next concrete source artifact is the reported master table;
its contents must distinguish lamp state/settings from an accepted actuation
request and must be checked against the connected bus. No rooting, firmware
change, community contact or vehicle transmission was performed in this pass.

## User-supplied SJB 0x771 lead and bounded live probe

The user supplied a community screenshot attributed to an Avante AD owner:
request 0x771, response 0x779, extended session `1003`, left IO-control
`2FBC1503` with return-control `2FBC1500`, and right `2FBC1603` with
return-control `2FBC1600`. The author says the values came from app reverse
engineering and match his vehicle's control requests. This does not establish
the ECU/DID semantics on Ioniq 5 PE. Return-to-ECU-control is not necessarily
an unconditional lamp-OFF command.

Before any new diagnostic transmission, the C4 was at `68657e2c` (core tune),
parked with zero raw speed, parking brake and inactive control. Panda reported
`interruptRateCan1` and historical bus-0 errors; current RX/TX error counters
were zero and the cumulative count did not increase over the observation.
The interrupt implementation can latch this fault. A normal, parked C4 reboot
cleared the fault and restarted the pulled revision; the cause of the earlier
fault was not determined or attributed to core tune/the new probe.

With fresh healthy telemetry, original firmware `DEV-14cc6268f895-DEBU` and
normal harness routing (ELM327 parameter 1), the bounded supervisor paused
the control stack and checked 0x771 on buses 0 and 1. Each bus received `3E00`,
`22F100`, `22F110`, `1003`, then `1001` in cleanup. The whole ten-request pass
was run once with explicit CAN FD and once with classical CAN. Both passes
had ten TX-path echoes and no 0x779 response. Echoes do not prove target ECU
acknowledgment. Raw CRC/counter/freshness-qualified parked inputs, Panda faults
and CAN error-counter growth were monitored; no probe error was reported.
No `2F`, communication-disable, security-access, coding or firmware-write
operation was sent. No physical lamp actuation occurred.

Final verification confirmed all five sampled services alive/valid, P/zero/
parking brake/inactive controls, no missing expected managed process, normal
hyundaiCanfd parameter 157, no Panda fault or safety RX invalidity and zero
current CAN RX/TX error counters. Firmware signatures were unchanged during
both probes. Evidence is in `.analysis/archive/2026-10-02/sjb771-probe/`.

This adds an explicit session/protocol comparison for the supplied address;
it does not identify the reason for absent replies or prove that the vehicle
lacks a matching SJB function. The screenshot alone is insufficient to move
past the unconfirmed endpoint into IO-control on this connection.

## Positive-control audit of the 0x771 test path

The user correctly requested known responding ECUs alongside 0x771 to check
the implementation. A fresh parked pass reused the same Reader, framing,
response parser, original firmware and normal-route ELM327 parameter 1.
It sent only `3E00` and `22F100` to 0x730, 0x771, 0x7C4 and 0x7D0 on buses
0/1, first CAN FD and then classical CAN: 32 requests total. No session change
or actuator request was included.

| Endpoint | Bus 0 CAN FD | Bus 0 classical | Bus 1, both formats |
| --- | --- | --- | --- |
| ADAS 0x730 / 0x738 | Both requests positive | Both positive | No reply |
| Camera 0x7C4 / 0x7CC | Both positive | Both positive | No reply |
| Front radar 0x7D0 / 0x7D8 | Both positive | Both positive | Both positive |
| Candidate SJB 0x771 / 0x779 | No reply | No reply | No reply |

There were 16 positive UDS responses. Presence responses were `7E00`; identity
responses began `62F100` and reconstructed full multi-frame strings, including
camera `99211-GI520`, radar `99110-GI500` and ADAS `NE1 FBL5 1.00 1.10 260204`.
The raw capture itself contained zero received 0x779 frames, not merely zero
successfully decoded replies. Forty TX-path echoes comprise the 32 initial
requests plus eight ISO-TP flow-control frames; these echoes are not the
evidence of ECU response, the positive received UDS payloads are.

This validates functioning transmit/receive, short-response parsing and
multi-frame reconstruction for the control ECUs under the same tested
conditions. It does not prove all possible protocol/addressing behavior for
the unresponsive ECU or establish the cause of 0x771 silence. The preliminary
0x771-only test should have included this simultaneous positive control.

Final restoration verified healthy services, original safety parameter 157,
no Panda faults/safety RX invalidity and zero cumulative CAN error counters
on all three buses. Original firmware/signature remained unchanged.
Private evidence: `.analysis/archive/2026-10-02/sjb771-controls/`.

## Vehicle ECU-list cross-check

On the user's request to prioritize the vehicle's own ECU list, a read-only
SSH check of `CarParams`, `CarParamsCache` and `CarParamsPersistent` found
`HYUNDAI_IONIQ_5_PE` with an empty `carFw` list in all three. These saved
openpilot identification records do not provide a complete vehicle inventory.
The six responding addresses in the earlier live discovery therefore remain
the measured list for this connection.

An offline check of both original discovery captures searched received raw
frames for normal-addressed single-frame positive/negative UDS responses,
without requiring the original request-plus-eight address mapping. It found
only 0x738, 0x7BB, 0x7BF, 0x7CC, 0x7CE and 0x7D8, consistent with the six
recorded endpoints. This limited check does not cover every framing format.

The repository's `get_all_ecu_addrs` is a TesterPresent discovery scan, not a
gateway command returning a stored master inventory. It additionally includes
29-bit physical diagnostic addresses, which the earlier bounded 11-bit sweeps
did not test. Neither an absent saved list nor the six observed responders
proves that other ECUs are absent. No additional diagnostic traffic, session
change or actuation was performed during this cross-check.
