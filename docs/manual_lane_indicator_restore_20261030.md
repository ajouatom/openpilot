# Separate manual lane-change indicator restored — 2026-10-30

The owner clarified that this is independent of ATC and explicitly requested its restoration. Restore only `ManualLaneChangeIndicator`, its source/model metadata and own-lamp isolation, using the reviewed implementation from `bd1ea4eb00`. Do not restore that commit's ATC audio expansion. SelfdriveD, navigation, the 250m helper and existing ATC entry conditions remain unchanged from `3be19f6332`.

The option appears under Lane Change, defaults OFF and supports the existing Ioniq 5 PE Jetson USB2CAN path with AutoBlinkerControl 1/2. It does not require an ATC mode, a TBT instruction or a 250m approach. Fresh reliable lane boundaries, sustained same-direction driver steering and at least 30 km/h are required. An existing maneuver prevents competing requests. It requests one lamp tap and never supplies a steering desire. Physical lamp acceptance and real crossing sensitivity remain unvalidated.

Own feedback is isolated before request publication and journaled before transmission. Keep the minimum 12-second plus lamp-extinction gate and restart recovery. Refine startup: without a journaled own request, only delay new detections; do not mask pre-existing driver/ATC lamps. Four regression cases cover both directions with/without an existing maneuver.

Validation: 482 desktop checks pass (26 native safety cases excluded on Windows); 25 Wiki generator checks pass. With this option enabled and fresh inputs, the same 14,558 recorded morning frames produce identical lane state/direction/desire/arming/ATC state to the pre-consolidation Wip helper. No own-trigger occurs in this recording; synthetic tests separately cover own-trigger steering isolation. This does not prove road performance or resolve the previously reported missed ATC maneuver.

Normal carrot-wip Git Pull and comma reboot applies the option; no Jetson runtime/model bundle is needed. No live vehicle setting, reboot or CAN actuation was performed. Native CI is associated with the published source commit.
