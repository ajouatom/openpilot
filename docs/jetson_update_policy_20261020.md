# Jetson updates through Git Pull and reboot

October 27 consolidation: carrot-wip now includes the NAV/model catalog runtime.
N16 Nav and Flip Flop are selected in Carrot Web and activated at reboot for both
eGPU and Jetson; internal CD210 remains fixed. Earlier branch-specific statements
below describe historical deployments, not the consolidated branch.

The owner's delivery requirement, effective 2026-10-20, is that routine work
must reach users through comma Git Pull and restart/reboot. A per-device SSH
copy, parameter edit, service restart, or manually prefilled cache is not an
acceptable normal installation procedure. Jetson USB patching or a separate
installation is a recovery exception only when the installed bootstrap cannot
consume the ordinary update. This applies to future changes as well.

## User procedure

1. Park, disengage, and turn ignition off so comma reaches its actual offroad
   screen. Keep both devices powered and give comma Internet access.
2. Run Git Pull on comma, then reboot comma and Jetson. A comma-only manager
   restart does not cause an already approved Jetson boot to check a new runtime
   pin. Keep ignition off throughout download and model preparation.
3. The installed boot updater reads the branch's selected release, verifies its
   signature and hashes, downloads missing files, and builds/probes a new engine
   when needed. Wait for the model-ready indication before starting a drive.
   First preparation can take substantially longer than a cached startup.

N16 selection is specific to carrot-nav; carrot-wip keeps its own model/runtime.
The eGPU PKL, Jetson ONNX and TensorRT engine are distinct artifacts. This
procedure does not promise automatic model equivalence or a road validation.

As of October 21, the owner requested restoring bounded USB HUD/navigation
transport on carrot-nav. The updated receiver explicitly negotiates USB;
older Wi-Fi-only receivers retain Wi-Fi until updated. Jetson still drives its
attached USB panel. See [the physical comparison and release](jetson_usb_hud_restore_20261021.md).
A normal Internet outage is first handled by restoring comma's
Internet access and allowing the updater to retry, not by switching to a
different USB release or repeatedly reinstalling the SD card.

## Internet-sharing correction

Read-only checks on the owner's devices reproduced the previous NAS timeout:
Jetson had a direct-AP default route and working DNS, but an HTTPS manifest
request timed out after 12 seconds. The identical comma request returned HTTP
200 in 0.388 seconds. IPv4 forwarding was already enabled. NetworkManager's
shared AP was active, but the legacy NAT table had no rules; the default
iptables nft backend failed with a kernel generation-ID error while the legacy
backend worked.

The comma AP worker now probes the available NAT backend and installs one
idempotent, commented masquerade rule for its private /24, excluding traffic
within that subnet. It removes only that exact owned rule on normal cleanup.
It does not flush tables, change firewall policies, configure USB, or alter
the Internet station profile. Installation/retry is offroad-only. A failure
is reported separately and leaves the local display AP running. The
`internet_sharing=configured` diagnostic describes rule setup, not proof of
Internet availability.

This correction applies to carrot-nav's direct AP; carrot-wip retains its
separate runtime. This is comma-side code delivered by Git Pull. The existing signed N16 host
release, Jetson bootstrap and image do not need replacing for this correction.
The immutable Jetson Wi-Fi worker is unchanged.

## Validation and remaining limits

The final desktop selection passed 70 tests, with four native/platform skips,
covering boot/update behavior, Wi-Fi transport,
provisioning, backend fallback, idempotence, ownership-preserving cleanup,
failed inspections and the offroad boundary. A Linux CI test additionally
creates isolated client/gateway/server namespaces and checks actual packets
before/after NAT installation and removal, preserving an unrelated rule.
[CI39369739310](https://github.com/ajouatom/openpilot/actions/runs/39369739310)
on `756a853307` passed: 94 root updater cases, 855 broad cases (56 skips),
119 navigation/cluster cases, 11 protected-storage cases and 11 NAT cases
including the actual isolated packet test. Counts overlap. Windows installer
checks passed too.

The previous N16 hardware installation used a manual verified WLAN cache
transfer and an offroad daemon restart; it is not evidence that an uncached
Git Pull/reboot update already passed end to end. This change does not fix
the separately observed optional cable DHCP process-exit race. A normal
offroad reboot is the current recovery for that latched cleanup error.
Do not report automatic-delivery acceptance until the complete path has
actually been observed. No vehicle settings or live services were changed
during the read-only diagnosis for this correction.
