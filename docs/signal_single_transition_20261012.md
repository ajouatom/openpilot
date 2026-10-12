# One central red-to-green transition removes only the stopped-car veto

The user requested that one previously red central signal changing to green
release the helper's departure block, leaving departure to the original model
and planner. The previous two-housing fallback could wait for a second signal
after a tiny primary red candidate disappeared.

## Change

`SignalStartGuard` now accepts one independently red-confirmed anchor whose
horizontal centre is in the middle 45–55% of the road image. This is a provisional
image-centre hypothesis, not verified signal-to-lane or arrow association.
The anchor needs two red observations spanning at least 150 ms, then at least
three fresh same-box green observations spanning 700 ms. The existing 650 ms
capture-age, 700 ms observation-gap, IoU 0.6, session, validity and duplicate-frame
checks remain. Green cannot move the red anchor. A fresh forward red conflict
still prevents release; this change does not resolve mixed lane/arrow signals.

The original primary-anchor release and the two-disjoint-housing fallback outside
the central band remain. A single central fallback release logs
`green_transition`. It only clears the helper veto: no synthetic model green,
velocity, acceleration or direct cruise-state command is added. Existing model
stop/go, departure delay, lead, left-blinker, soft-hold and detect-only behavior
remains in `CarrotPlanner`. The driving ONNX and moving-red preparation are unchanged.

## Validation

- 532 focused Windows tests pass with the existing native Params storage stub.
  Added single-anchor cases cover either central housing, insufficient red
  history, off-centre rejection and preservation of the two-housing fallback.
  Actual planner-method tests cover both ACC/blended requests with model stop/go,
  left blinker, soft hold and detect-only settings. Model stop remains stopped;
  model go with normal settings reaches `e2eCruise` after the existing delay.
- Same-recorded-input replay of six 10a5 segments releases the reviewed green
  hold at 20:49:51.837, versus 20:49:53.740 for the previous two-housing version
  (1.902 seconds earlier). The original planner subsequently leaves stopped state
  at 20:49:52.236. These are replay state transitions, not measured vehicle motion.
  Added 20/100/250 ms observation delays still release in this reviewed case.
- The earlier 43,090-input guard corpus has zero releases in reviewed active-red
  intervals at nominal timing and with 20 ms added delay. Release times,
  red/green hold counts and known incident outcomes match the previous version
  at each timing; diagnostic release reason labels can change. Both known 1095 red departure windows
  remain blocked. The earlier 1091 segment-20 missed acquisition remains unresolved.

Evidence and replay scripts are private in
`.analysis/scratch/2026-10-12-single-transition/`; original inputs and visual
review remain in the preceding 10a5 archive. This is recorded-input state-machine
validation with existing replay substitutions, not native MPC, perception accuracy
or closed-loop validation. No vehicle files, settings or processes were changed.
Publish the same correction to carrot-nav and carrot-wip using single-parent commits.
