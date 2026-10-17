# Auxiliary red veto and existing lead handover — 2026-10-17

The user requested that a lead satisfying the existing stop-to-lead range
condition take priority over the auxiliary red departure veto. Previously the
veto ran before the existing handover and forced e2eStopped even when the model
wanted to follow a departing lead.

The shared condition is leadOne.status, a finite positive dRel, finite filtered
model stop distance, and `dRel - stop_model_x_raw < 2.0`. The model distance is
the existing `update_stop_dist(x[31])` result, before its closing-rate limiter.
Exactly 2m is excluded. This is not an absolute 2m following distance, nor an
independent lead-motion detector. A stationary qualifying lead remains an MPC
obstacle; enabling lead following does not command acceleration through it.

When eligible, the effective start decision has no hold, refusal warning,
green confirmation or green-release event. The final longitudinal veto sees
this same decision. The optional legacy SignalAssist decision also suppresses
its hold and red injection. Raw recognition remains available to the display;
the underlying red memory is retained. Losing the lead or leaving the boundary
reapplies any retained veto on the next planner update, including with unknown
camera evidence. There is no persistent bypass timer. Suppressed warnings do
not consume the warning needed when the veto resumes.

Soft hold and explicit user stop remain independent. Existing radar/MPC
obstacles, pedal handling, mode-0 shutdown, mode-2 moving preparation, detector
and green-confirmation policies are unchanged. Normal e2eStopped handover still
has its existing one-cycle zero cruise target before the lead state update.

Validation: 715 desktop signal, planner, geometry, timing and brake-display
tests pass (316 pre-existing NumPy deprecation warnings). The new 22 cases
exercise both ACC/blended paths, strict range boundaries, invalid distances,
both auxiliary vetoes, the real final-output veto method, continuous lead
following, lead disappearance/range exit, unknown evidence, refusal warning,
raw red display and independent driver holds. Existing tests requiring red
priority now explicitly place the lead outside the handover boundary. Tests
use the repository's production-method replay harness and a native Params
lookup double on Windows. They are not closed-loop MPC or road validation.

Publish as single-parent commits to carrot-nav and carrot-wip. No vehicle
installation, fresh log replay, model retraining or recognition improvement is
claimed by this change.
