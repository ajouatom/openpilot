# PR 568 integration in carrot-nav-blink — 2026-10-26

Reviewed source: https://github.com/ajouatom/openpilot/pull/568 at
`3615bf4bf40016f102e7d4af3745455a3791faa2`, including the hazards fix.
The PR adds default-off Tesla automatic indicators, received-idle-template CAN
requests, echoed-request and physical-lamp feedback, driver/hazards handoff,
bounded sessions/cancellation, and corresponding Panda restrictions. It also
corrects Tesla measurement/DBC interpretation and APP traffic-control decoding.

This branch already owns ModelDataV2.Meta fields 22/23 for Ioniq automatic lamps.
They retain their names/types/ordinals. Tesla fields are additive at 24/25.
The existing NAV MapInput and both Ioniq publisher fields are preserved. Tesla
CAN observation and its intent machine are skipped on non-Tesla vehicles.
No Tesla setting is enabled on the owner's Ioniq5PE.

Validation: Linux CI 40302082309 passes 1054 tests, 40 existing abstract/platform
skips and 362 subtests. These counts overlap the contributor suites; they are not
additive. Production CAN kernels and actual C safety hooks are built and used.
Settings are isolated test defaults, live IPC entry points reject access, and the
production byte decoder/Capnp schema is used. No physical CAN is sent by this suite.
The first CI attempts exposed missing test dependencies, subsequently corrected.
Jetson Linux/Windows CI 40301706739 passes; Wiki generation checks pass 25/25.

No Tesla physical acceptance is possible on the owner's Ioniq. Existing Jetson
bundles retain their signed source commits; the additional Tesla fields are not
consumed by the Jetson, and existing Ioniq ordinals are unchanged. This integration
is published only to carrot-nav-blink. It does not merge the PR into carrot-wip.

## Separate SPAS proposal

The owner subsequently supplied f8448120/04dd4119 patches and requests a CAN-FD
SPAS-based alternative after the model tests. The supplied left/right 0x16A
counter-zero bytes exactly match our existing Hyundai CAN-FD packer (db65/848c
checksums); SPAS1 is 24 bytes and SPAS2 is 32 bytes.

The supplied implementation is not adopted unchanged: it disables parking-ECU
communication without checking replies and offers unlimited test transmission at
any speed/gear. Its author explicitly records missing Panda/vehicle validation.
The requested implementation must select compatible SPAS buses/contracts rather
than an Ioniq-only fingerprint, remain opt-in, prevent simultaneous Jetson USB2CAN
operation, and validate bounded parked trials including ECU restoration. These
requirements are not a claim that all vehicles with matching CAN IDs accept it.
That was the initial review checkpoint. Implementation and subsequent guarded
vehicle trials are recorded in [the SPAS trial report](canfd_spas_blinkers_20261026.md);
they have not established reliable illumination or holding.
