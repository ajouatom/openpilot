# Hyundai Staria EV 2026 registration

## Scope

Add manual selection `Hyundai Staria EV 2026` (`HYUNDAI_STARIA_EV`) as a
separate Hyundai CAN-FD EV platform. Existing Staria selections, Params,
CAN-FD feature detection, safety limits, controllers and radar decoding remain
unchanged. No ECU firmware was collected, so no automatic firmware fingerprint
is added or copied from the combustion Staria.

The EV flag selects the existing `ACCELERATOR` (0x35, DLC32) pedal and gear
path, the 255 pedal divisor, and Panda `EV_GAS` safety bit. Panda's existing
EV pedal check reads byte 5 of the same frame. Ioniq 5, EV6 and PV5 use this
same path. No new counter/checksum exceptions are introduced.

## Specifications and assumptions

Hyundai's [Korean-market English catalog](https://www.hyundai.com/content/dam/hyundai/kr/ko/data/vehicles/catalog/en/staria-limousine-electric-catalog-eng.pdf)
lists a 3,275 mm wheelbase, 84 kWh battery, 160 kW motor, and 350 Nm torque.
The Lounge 7/11-seat variants have a 2,590 kg curb weight; the Limousine
6-seat variant is 2,695 kg. These are variant-specific values, not a common
weight for every cargo/passenger conversion.

The user approved registering the Lounge 2,590 kg baseline on September 23,
2026. This is the selected registration baseline, not confirmation of the
reported vehicle's exact seating/trim. Runtime adds the existing 136 kg
standard payload once.
Only mass and wheelbase override the original Staria specs. Steering ratio
11.94 and the torque fallback are inherited baselines, not measured EV
calibration. The observed 0xCB camera frame activates the existing angle
controller dynamically; no steering limits or tuning are relaxed. The
Hyundai K harness listing follows the existing Staria entry and still needs
the installation's connector confirmation.

## Supplied route

Owner confirmed that `2997785d30a15d76`, route
`0000027d--2d1a1493b3--0`, is a 2026 EV manually selected as
`HYUNDAI_STARIA_4TH_GEN`, running e78b3387.

Full cereal decoding finds 6,127 CAN batches, 5,995 carState records and
empty carFw. All recorded gears are `unknown`; gas/brake inputs are false.
Recorded carState CAN validity is false in 5,789 records and true in 206.

Recorded flags include HDA2, alternative buttons, HDA2 alternative steering
and angle control, but no EV flag. The HDA2=1 / HyundaiCameraSCC=0 mapping
expects ECAN on bus 1, whereas this capture carries vehicle/EV data on bus 0:

| Bus | Observed messages |
| --- | --- |
| 0 | 0x35 EV accelerator/gear (3,062 frames), 0x130 shifter, 0x175 TCS, wheel speeds, MDPS, alternative buttons |
| 1 | 0x100 DLC24, 0x110 DLC32, 0x3A5..0x3C4 radar frames |
| 2 | 0xCB LFA_ALT, 0x12A LFA, 0x1A0 SCC_CONTROL (3,062 frames) |

Do not interpret bus 1's 0x100/DLC24 as the combustion accelerator message
(which is DLC32), or infer Group1 radar from bus 0's unrelated 0x210 when
the bus mapping is incorrect.

The uploaded settings at 17:47:31 through 18:01:00 on September 23 show
HDA2=1 / camera SCC=0; 18:06:21 shows 1/1 and 18:12:38 returns to 1/0.
These later snapshots do not prove the wiring or physical equipment changed.

## Offline checks and limits

Replayed raw CAN through the current CarState/parser with isolated Params;
no vehicle settings were written. With EV selection, HDA2=0 and camera SCC=1,
all 5,927 batches after the first 200 startup batches report valid parsers
and Park gear, with zero accelerator/brake input. With EV selection but
camera SCC=0, bus 0 SCC data disappears after startup forwarding stops;
5,362 of the same batches are invalid. Original Staria decoding additionally
uses the counter-checked 0x130 gear path instead of the existing EV path.

This comparison supports the observed bus layout and parked signal decoding,
not a recommendation to force one setting on every Staria EV. Camera SCC=1
also enables the existing longitudinal control path; actual steering,
acceleration/braking, cancellation and cruise-button operation remain untested.
The capture contains no pedal press or D/R transition. The 18 focused tests
cover manual selection, EV safety selection, existing Staria preservation,
Ioniq 5/EV6/PV5 path comparison, and synthetic P/D/N/R plus pedal values
0/128/255. Synthetic transitions are not vehicle validation.

Windows execution substitutes only the unavailable Params binding; real
cereal, Hyundai interface/CarState, DBC, CAN parser and packer are used.
Native Panda safety execution and on-device control validation remain pending.
