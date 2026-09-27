# Automatic driving-mode recovery

On 2026-09-27 the user requested less conservative Safe release after a lead
starts moving. The acceleration threshold is now strictly above 1.0 m/s²
(previously 1.5), still requiring about 0.5 seconds of continuous evidence.
Flow recovery now requires 3 seconds rather than 6. All speed/distance gates,
stopping and slow-following entry conditions, lead-change/invalid-input resets,
and clear-road confirmation remain unchanged. This only selects the driving
character; it does not change radar selection or physical braking constraints.

For the reported Ioniq 5 segment, the recorded Safe-to-Normal transition was
26.567 seconds after the first carState (ego 53.45 km/h). Replaying the latest
logged carState/radarState at each plan publication with the new detector
releases at 12.898 seconds (ego 0.09, lead 7.58 km/h), without re-entry during
the remainder of the segment. This is an offline mode comparison using the
original vehicle trajectory, not a simulation of the changed acceleration or
a bit-exact reconstruction of SubMaster input ordering. Raw evidence and
reproduction scripts remain in the private analysis archive.

Validation: 58 detector tests cover moderate/strong acceleration, short spikes,
stopping priority, flow recovery, braking waves, track changes, invalid inputs,
and Normal/Eco selection. On Windows only the hardware import was stubbed as
PC; detector and timing code were unmodified. The 25 Wiki generator/validator
tests, Wiki candidate validation, and bilingual user-docs check pass. Actual
vehicle comfort and mode stability still require driving validation.
