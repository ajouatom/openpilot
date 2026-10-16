# Signal modes and continuous observation display — 2026-10-16

The requested contract is: mode 0 does no auxiliary recognition, display or
control; modes 1 and 2 show recognition during ordinary driving, including when
longitudinal control is inactive. Mode 2 adds moving gentle preparation only.
At standstill it uses the same red departure guard as mode 1.

## Changes

- Keep the mode-0 observer/display enable policy unchanged. The initial local
  interpretation that would have enabled observation in mode 0 was withdrawn
  before publication following the user's explicit correction.
- In modes 1/2 the HUD ring now reads the observation-only display transport,
  independently of planner intervention/ACC activation and the model lamp's
  fade. The centre dot still reports actual planner action. USB camera view
  retains boxes and adds current/previous/waiting status; the USB speed panel
  also has an observation ring. Wide-camera view can show the summary without
  placing road-camera boxes in the wrong image.
- Bright color means a current observation; dim color means a previous result
  held for at most two seconds **from capture**. A later unknown packet cannot
  renew that color. New color replaces it immediately. Unknown/no input is gray
  instead of making the entire auxiliary indicator disappear. This memory is
  never a control input. Highway suspension retains its existing pause mark.
- A CPU observer need not restart merely when changing 1 to 2 or back; backend
  supervision and mode-0 shutdown remain. Control consumers retain their own
  activation freshness checks. No GPU scheduling, DM, driving-model or detector
  algorithm changes are included.
- The user specified a **fixed** near-lead distance, rather than comparison with
  model stop distance. Use 50 m as a provisional trial value, not a measured
  optimal threshold: either lead at/below 50 m prevents preparation; both leads
  remain unmodified MPC obstacles. Invalid active-lead range also blocks it.
  Check the same rule at preparation entry and at the final acceleration cap.
- The final cap independently rejects standstill/low speed, soft hold, driver
  pedals and speed at/above 82 km/h. Preparation remains limited to speeds above
  5 km/h and does not set `shouldStop` or issue a departure command. Original
  model stopping and the existing stopped guard remain separate.

## Evidence and remaining failures

Read-only vehicle collection retained 144 completed qlogs and selected 20 full
rlogs/videos (local size checks and SHA256 manifests). The detailed set contains
21,427 model samples and 10,020 observer records, including parked tests. The
18 evening/night/morning segments have recorded observer rates of about
5.66–11.95 Hz, not the older 2–3 Hz neural cadence. These are recorded rates,
not a post-change native performance measurement. There are 846 raw-color
observations while longitudinal control was inactive; the old plan-derived
color could suppress this display despite the observer seeing candidates.

The Oct 15 19:46:44 evening incident is a real retained-veto failure: the lead
is about 4.17 m ahead and moving, the model's terminal speed is about 9.87 m/s,
but the auxiliary hold forces `e2eStopped`. The video shows an overhead green
transition and lower rear-light candidates. A raw green box is insufficient
for the current same-object/conflict-qualified release. This mode/display
change **does not claim to solve that recognition/identity failure** and does
not add a blanket lead-motion override of red. Earlier no-extra-release offline
trials remain offline. Some screened launch points are segment-boundary motion;
they must not be counted as verified red departures.

Of 3,759 observer records with a lead in the 18 drive segments, only 46 have all
active leads beyond 50 m. This is only a distance screen: it does not prove red
qualification, additional deceleration, or stopping performance. The new fixed
distance condition has not been validated in a closed-loop vehicle trial.

## Checks

673 focused signal/observer/geometry/planner/UI tests pass on carrot-nav.
Coverage includes both leads and the exact 50 m boundary, stronger/normal MPC
behavior, cancelling preparation when a lead becomes close, final-output
standstill/soft-hold rejection, and identical mode-1/mode-2 stopped red/unknown/
green traces. Display tests cover cruise-independent current/held/unknown
status, capture-based expiry, session reset, and zero observation-file reads
in mode 0. These tests do not establish recognition accuracy or road safety.

Raw video and diagnostic files remain private in the dated signal-drive
analysis directory. Publication to maintained branches is distinct from vehicle
installation; this task made no vehicle code, setting or process changes.
