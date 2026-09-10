# Radar lead acceleration filtering

The front/corner radar acceleration estimator again differentiates filtered
speed, using the two first-order filters present before `93bab17bca`:

```python
v_filtered += dt / (0.10 + dt) * (v_measured - v_filtered)
a_sample = (v_filtered - v_filtered_previous) / dt
a_filtered += dt / (0.15 + dt) * (a_sample - a_filtered)
```

The pseudocode omits the existing standstill handling, acceleration bounds,
innovation limit and acquisition reset. Those protections remain in `MyTrack`.
The current-frame publication correction in `380b5d9d1b` also remains. There is
no added output filter, prediction term, adaptive time constant, or setting.

## Why restore this estimator

The September 1 change replaced the filtered-speed derivative with a short raw
velocity difference and shortened the acceleration filter. The subsequent
four-sample linear slope reduced some of that noise, but still differentiated
short raw radar-speed fluctuations. A small speed error can become a large
acceleration error and enter the MPC's lead-motion prediction.

The restored filters attenuate those fluctuations before and after taking the
derivative. Both updates use the existing `FirstOrderFilter`; at 20 Hz their
weights are 1/3 and 1/4. The non-SCC four-sample slope history is no longer used.

Equal 0.12 s time constants were evaluated and rejected. Their white-noise RMS
was within about 4% of the historical filter, but a single 0.5 m/s speed spike
produced a 0.865 m/s² peak instead of 0.750 m/s². Restoring the original constants
preserves both steady-noise attenuation and the original spike bound without
adding special-case logic.

## Timing and validation

For a known acceleration transition from 0 to -3 m/s², sampled every 50 ms:

| Estimator | 10% response | 50% response | 90% response |
| --- | --- | --- | --- |
| Historical 0.10/0.15 s filters, including old publication lag | 150 ms | 300 ms | 650 ms |
| Four-sample slope before this change | 50 ms | 150 ms | 250 ms |
| Restored filters, current-frame publication | 100 ms | 250 ms | 600 ms |

These are estimator threshold crossings, not vehicle brake application times.
Stronger attenuation costs response speed relative to the recent slope-based
estimator. The restored estimator is one publication frame faster than the
historical version; it does not eliminate causal filtering delay.

With deterministic independent speed noise of standard deviation 0.10 m/s,
acceleration RMS was approximately 0.505 m/s² for the four-sample slope and
0.194 m/s² for both the restored and historical filters. In recorded launch
windows, the restored filter reduced the 95th-percentile absolute adjacent
acceleration change by approximately 53–58% relative to the four-sample slope.
These measures describe estimate variability, not error against ground truth.

Focused tests cover known braking at multiple sample periods and transition
phases, monotonic braking, constant-acceleration bias, isolated speed spikes,
steady-speed noise, reset/reacquisition, publication copies and SCC replacement.
The historical filter is an independent reference in the tests. Recorded-input
comparisons are offline estimator replays, not closed-loop vehicle validation.

## Boundaries

- SCC has a reusable target slot. Its existing three-sample acceleration
  discriminator and 0.05 s acceleration filter remain unchanged, so this change
  does not mask its target-replacement reset.
- Identified radar tracks keep their age during a large acceleration innovation;
  real braking must not cause the acceleration publication to reset to zero.
- Raw `vLead`, `vRel`, distance, target selection and the independent jerk
  estimator remain unchanged. The new filtering affects `aLead`/`aLeadK`; it does
  not make the raw speed display smooth or remove all noisy inputs to the MPC.
- The NAS viewer recalculates lead selection from recorded `liveTracks` inputs.
  Deploying new source invalidates its replay cache but does not rewrite old
  recorded accelerations or MPC plans. Comparing this filter on old velocity
  samples requires the separate offline estimator replay.
- No setting has been added or reinterpreted. TF, response-level mappings,
  braking limits and model/branch choices are unchanged.
