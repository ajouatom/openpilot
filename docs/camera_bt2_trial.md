# Camera / AGNOS Bluetooth diagnostic trial

Created from carrot-wip c67e0ad876 on 2026-09-24 at the user's request.
This branch is an isolated parked-device experiment, not a completed camera fix.

The builder branch is `ajouatom/agnos-builder:carrot-camera-bt2-19.8`.
Target OS: `19.8-carrot-bt2`. Do not change the stable branch's OS selection.
The trial branch now selects the verified C3/C4 release manifests. The C4 device
continues running bt1 while the separate sensor-exposure hypothesis is compared;
bt2 has not yet received device validation.
The independent [OS04C10 exposure correction](os04c10_exposure_investigation.md)
passed parked A/B/A and normal-runtime tests on bt1 and was also published to
carrot-wip as 93d8ab1fbf. It does not depend on installing this experimental OS.

Builder commit: `60b901ff2d9bad09403e745ef1bc6b2745b0c1d5`.
[Build 35942144886](https://github.com/ajouatom/agnos-builder/actions/runs/35942144886)
passed C3 and C4 compilation and frozen-userspace image checks.
[Release](https://github.com/ajouatom/agnos-builder/releases/tag/agnos-19.8-carrot-bt2)
manifest hashes and PROVENANCE were verified; all non-boot/system partition
entries are unchanged. The system image derives from the immutable bt1 image,
changing only VERSION/BUILD labels, so target apt packages and GPU firmware
are not silently updated during this kernel comparison.

The OS corrects Qualcomm diagnostic ACL routing (raw handle/flags 0x2edc),
preserving ordinary Bluetooth traffic and monitor diagnostics. It also provides
disabled-by-default `camera:cam_csid_sof_history` and
`camera:cam_ife_irq_payload` tracepoints. Only enabled history tracing performs
additional MMIO reads. Camera scheduling, frame acceptance, error recovery,
model/runtime, thresholds and recurrent state are preserved.

The kernel's frame ID is a software SOF counter; consecutive IDs do not prove
all physical frames or IRQs arrived. Existing `cam_isp_activated_irq` exposes
CSID hardware time, whereas rlog timestampSof is sampled in kernel processing.
The additional previous-SOF register can identify a hardware SOF between two
observed callbacks. A hardware gap does not by itself distinguish sensor, FSIN,
CSI or receiver faults. Verify register semantics on normal frames, reject
current/verification mismatches and check trace continuity before classifying.

Use bounded trace instances while parked, and restore prior settings after
each comparison. Do not add a second video0 reader: this kernel rejects a
second open. The source also has notification suppression, request recovery
and IRQ payload exhaustion paths; their existence is not incident evidence.

Tests: `python -m unittest discover -s tools/camera -p test_sof_history.py`.
These synthetic timestamp cases are not hardware validation. Record C3 and C4
results separately. The observed C4 device was on 209b3c0c and bt1 initially;
the branch base differs only in web settings navigation and CI/tests.

Docs-Not-Needed: Isolated internal OS/camera diagnostics, no new user setting.
