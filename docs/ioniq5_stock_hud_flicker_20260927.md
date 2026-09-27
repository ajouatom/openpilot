# Ioniq 5 stock HUD / cluster lead-color flicker

## Evidence

Route `0000044e--029303fb58--10`, incident commit `7e1a8031`, was
stationary from 47.145 to 60.010 seconds relative to the first `carState`.
All 258 stopped `radarState` samples retained leadOne, track 33, at
4.199–4.254 m. Relative speed ranged from -0.15 to +0.03 m/s.

The transmitted SCC_CONTROL (0x1a0, bus 0) contained 644 stopped frames:
HUD_LEAD_INFO was 1 in 304 frames and 2 in 340 frames, with 77 transitions.
The relative-speed sign also changed 77 times. ACCMode remained 1 throughout.
Incoming SCC_CONTROL on bus 2 retained HUD_LEAD_INFO=2 in all 644 frames;
the bus-128 transmit echo reproduced the 77 transitions.

The user identified the affected displays as the OEM HUD and instrument
cluster, and confirmed white for approaching leads and gray for receding
leads. The former `lead.vRel > 0` test in `_apply_scc_lead` mapped tiny
stationary speed fluctuations directly to white/gray transitions. This is
distinct from USB cluster rendering or loss of lead detection.

## Change and validation

HUD_LEAD_INFO now selects receding gray (1) only above +0.3 m/s
(approximately 1.08 km/h relative speed); otherwise a valid lead stays
white (2). Missing/invalid leads still clear the indicator to 0.
This is a display threshold, not a stateful hysteresis filter: values
oscillating around +0.3 m/s can still change color.

Distance, lateral position, transmitted relative speed, lead selection,
acceleration and cruise-control fields are unchanged. No settings were
added or changed. The shared Hyundai CAN FD SCC display helper is affected;
radar detection/replay processing is unchanged.

The SCC lead, display lateral filter and CCNC lead suites passed (62 tests).
The regression test exercises the incident speed noise, threshold boundary,
receding and approaching values through the actual CAN packer/parser, and
checks that other SCC fields remain unchanged. Tests ran on Windows with a
local Params import stub and without the native root pytest fixtures.
This does not establish on-vehicle visual validation of the fix.

Private signal evidence and reproduction scripts are archived under
`.analysis/archive/2026-09-27/stock-hud/`; test setup is archived in
the sibling `stock-hud-fix/` directory. Raw vehicle data is not committed.
