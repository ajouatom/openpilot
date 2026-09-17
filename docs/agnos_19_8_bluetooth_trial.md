# AGNOS 19.8 native Bluetooth / Cinque v3 experiment

## Scope

The user requested this combined experiment on 2026-09-17, on the existing
`carrot-cinque_v3` branch. The earlier proposal for a separate openpilot Bluetooth
branch was superseded. `carrot-wip` remains on its existing OS and model.
The OS builder experiment is `ajouatom/agnos-builder:carrot-bluetooth-19.8`.

The new image is `19.8-carrot-bt1`. It preserves the existing Carrot USB-PD v3
patches, the C3-specific PCIe/NVMe patch, the legacy C3 boot firmware manifest,
and Carrot's Adreno library selection. Bluetooth support is shared by both
device boot images. Actual hardware validation starts with a C4; compiling
the C3 image does not establish C3/C3X runtime compatibility.

## AGNOS review

- [openpilot cab53438 / #38935](https://github.com/commaai/openpilot/commit/cab53438aefe5b4d48721da4af09efacd0d19451)
  selects AGNOS 19.8. Its PR describes the release as a prerequisite to the
  latest tinygrad update.
- AGNOS builder `bdbf1bc` is the 19.8 release source, merged into the existing
  Carrot builder. Both use kernel base
  `eccd146599f2e2f159d951092642689bede91632`.
- [#619](https://github.com/commaai/agnos-builder/pull/619) updates the AMD GPU
  firmware; [#626](https://github.com/commaai/agnos-builder/pull/626) records frame
  content sizes in Zstandard-compressed firmware. The build validates those
  headers. Unused ALSA state restoration is disabled as upstream does.
- The Bluetooth kernel changes and startup sequence are selectively adapted
  from StarPilot. Attribution and the exact source hashes are recorded in the
  builder's `patches/BLUETOOTH.md`. This does not replace the kernel with
  StarPilot's complete configuration or its userspace image.

## Model interface and tinygrad review

The pinned v3 model remains upstream PR #38932 commit `892fc3a1`, SHA-256
`e758b96df27858ea97122d18554930d04f9f8bda417417074edfb3a72b008d0b`.
Its isolated runtime remains tinygrad `d5e17c935daf11f6318e45aade9528f71b8fbdcc`.
The internal driving and driver-monitoring models are unchanged.

The recent upstream sequence is:

1. [#38916](https://github.com/commaai/openpilot/pull/38916): move image, desire
   and feature history queues into the ONNX graph.
2. [#38864](https://github.com/commaai/openpilot/pull/38864) and
   [#38922](https://github.com/commaai/openpilot/pull/38922): separate camera warp
   compilation and use tinygrad's ONNX/warp compilers.
3. [#38926](https://github.com/commaai/openpilot/pull/38926): generic artifacts
   with `run`, `input_specs`, `output_specs`, GPU-resident state and explicit
   output buffers.
4. [#38930](https://github.com/commaai/openpilot/pull/38930): ship the precompiled
   eGPU artifact, retain on-device camera warp compilation and an ARM submission
   helper.
5. [#38933](https://github.com/commaai/openpilot/pull/38933), inspected at
   `61bbab5e`: propose tinygrad `1e97630ff30793bd34f876656e82d611994be0b0` and
   remove `--device-input` from the generic ONNX compiler invocation. This PR
   was open at review; its reported upstream checks passed.

The current v3 adapter already implements the required generic contract:
two uint8 images `(2, 6, 128, 256)`, rising-edge desire pulses, traffic
convention, action timing, and three recurrent state buffers. Next-state
outputs alias the state inputs as upstream does. The graph advances history
once per invocation; the parent does not inject a second feature queue.
The parser consumes the model's declared slices from 18,452 values.

Direct comparison of the pinned and proposed tinygrad sources found identical
`device.py`, `helpers.py`, AMD backend/firmware loader and camera warp compiler.
The candidate changes JIT call handling, dependency tracking, USB argument-cache
conditions and compiler helper defaults. Therefore the AGNOS firmware update
does not itself require replacing the v3 runtime. Replacing the repository's
tinygrad would also affect the internal models, whose older compilation path
is deliberately retained here. The pinned PKL and matching runtime remain a
unit; no newer runtime is silently substituted underneath them.

Validation so far: 57 focused model/artifact/runner tests passed. The initial
offroad probe found approximately 0.13 V and PCIe LTSSM=0x00; USB bridge
enumeration alone is not GPU readiness. After the user enabled the 12 V supply,
the existing AGNOS 19.6.3-carrot system ran the pinned v3 worker with the C4
1344x760 camera input. A passive 10-second observation received 200 consecutive
model frames, with finite position/velocity values, effectively zero dropped
frames, and median model execution time 39.56 ms. The vehicle reported zero
speed. This is a stationary baseline, not road validation or a controlled
benchmark. Evidence is stored on the trial device at
`/data/carrot-bluetooth-observation-before.json`.

## Bluetooth trial operation

This phase adds native WCN3990 radio support, BlueZ, classic HID, BLE HID,
RFCOMM and pairing cryptography. GPS retains `ttyHS0`; Bluetooth uses `ttyHS1`.
The existing active-slot Bluetooth firmware partition is mounted read-only.
Pairing state is stored in `/data/bluetooth` across reboots.

The radio defaults to off. While offroad, explicitly enable it on a trial unit:

```sh
sudo install -d -m 700 /data/bluetooth
sudo touch /data/bluetooth/ENABLED
sudo systemctl start carrot-bluetooth-radio
bluetoothctl show
bluetoothctl --timeout 15 scan on
```

Pair only an identified, user-selected accessory with interactive
`bluetoothctl`. Test its input with `evtest`. To disable the trial:

```sh
sudo rm -f /data/bluetooth/ENABLED
sudo systemctl stop carrot-bluetooth-radio
```

This is OS-level bring-up. Phone application protocols, Carrot button mappings,
audio routing and Bluetooth settings UI are separate work. No vehicle control
is bound to a discovered or paired device by this change.

## Deployment and recovery

Use the generated device-specific manifest and existing AGNOS A/B updater.
Verify the inactive partitions before switching slots, preserve the previous
slot, and record the actual boot slot and original commit on the device.
The device's Cinque v3 artifact/runtime must remain unchanged.

The [dual-device build](https://github.com/ajouatom/agnos-builder/actions/runs/35178560903)
passed for builder commit `f5b3f77e59f1917f95e0752630ca08050463eede`.
The [trial release](https://github.com/ajouatom/agnos-builder/releases/tag/agnos-19.8-carrot-bt1)
contains the device manifests, compressed images, provenance and SHA256SUMS.
Manifest/version/provenance downloads were checked against SHA256SUMS. Only
boot and system differ from the previous manifests; other firmware entries
are byte-for-byte unchanged.

Raw image SHA-256 values:

- C3 boot: `9d1c81ef890edf349e0919260a850ab5d1f95162fbd4b262cfcd52d92c0ac0b8`
- C3X/C4 boot: `dccd7965346b0a87a9f64cb6be257f6bb5d3d0f368c8655085efc1e460527f5a`
- Common system: `375c5d22335770ac08750660bb5b29a3331c550d6a9875b35f36b88af792ea44`

The initial C4 slot was `_b`, running AGNOS `19.6.3-carrot`. Its existing
Bluetooth firmware partition contains `crbtfw11/20/21.tlv` and corresponding
`crnv11/20/21.bin` files. No failed system services were present at the baseline
check. The user confirmed the vehicle was stationary. A three-second live
check showed zero speed, valid car state, and neither enabled nor active control.
The comma service was stopped before flashing. All inactive A-slot partitions
passed a full raw-hash check before the standard updater activated that slot.
The device booted `19.8-carrot-bt1`, `/BUILD` matched `f5b3f77`, and the kernel
build timestamp was 2026-09-17 03:41:40 UTC. The previous B slot was preserved.

Hardware results:

- `/dev/btpower`, Bluetooth `ttyHS1` and GPS `ttyHS0` are present.
- Native `hci0` powers on and reports BR/EDR, LE and secure-connections support.
  Discovery found nearby devices over the air; no USB Bluetooth adapter was used.
- Radio service restart succeeds. Firmware is mounted read-only from the active
  Bluetooth partition, and `/var/lib/bluetooth` is backed by `/data/bluetooth`.
- A second reboot with the enable flag present automatically starts the radio;
  the controller remains powered and no system services are failed.
- With Bluetooth enabled, a stationary 10-second live model observation received
  201 consecutive frames, finite position/velocity values, zero frame drops and
  median model execution time 39.28 ms (baseline: 39.56 ms). This brief sample
  demonstrates operation, not a performance improvement or driving validation.
- With comma stopped, the pinned v3 artifact loaded for both 1928x1208 and
  1344x760 inputs and passed ten inference calls per format. Load times were
  21.40 and 21.05 seconds respectively. The first invocation includes warm-up;
  these synthetic IPC smoke timings are not the live modeld timing metric.
- A pre-existing 108 MB `/cache` partition was full from accumulated boot logs.
  This caused `agnos-debug` to fail after reboot. 61 older boot-log directories
  were preserved under `/data/carrot-bluetooth-trial/debug-cache-backup`, leaving
  the latest 40 in place. Restarting `agnos-debug` succeeded; no services remained
  failed. This did not require an image or model code change.

Evidence on the trial device: `/data/carrot-bluetooth-trial/before.json`,
`staged.json` in that directory, `/data/carrot-bluetooth-observation-after.json`,
and `/data/carrot-bluetooth-model-after.json`. The latter records both camera
formats and the unchanged model checkpoint. Four device-manifest tests also pass.

The user's remote appeared as `Yiser-J6`, advertising the BLE HID service and
keyboard appearance after being put in pairing mode. Pairing, bonding, trust
and connection succeeded; the battery service reported 90%. Linux created a
UHID input device named `Yiser-J6` with keyboard and absolute-pointer handlers.
Its descriptor exposes media keys plus `BTN_TOUCH`, `ABS_X` and `ABS_Y`, so it
must not be treated as a seven-arrow-key keyboard based on appearance alone.
The bond info exists under `/data/bluetooth` with mode 0600; its keys are not
included in logs or this document. After a radio/BlueZ service restart, the bond
and trust remained, and the remote reconnected automatically without pairing
again. No unidentified accessory was paired. Discovery and pairability were
turned off after pairing.

The user pressed all seven buttons in the requested order while `evtest --grab`
captured only this remote and comma was stopped. Recorded short-press behavior
in its current mode:

| Button | Observed input |
| --- | --- |
| Up | Touch swipe with increasing `ABS_Y` |
| Down | Touch swipe with decreasing `ABS_Y` |
| Left | Touch swipe with increasing `ABS_X` |
| Right | Touch swipe with decreasing `ABS_X` |
| Center | Touch tap at (300, 500) |
| 1 | `KEY_VOLUMEUP` (115), press and release |
| 2 | Touch tap at (420, 850) |

Touch events include `BTN_TOUCH` and `BTN_TOOL_PEN`. Coordinates are in the
descriptor's 0–1000 range, not physical display pixels. The raw capture is
`/data/carrot-bluetooth-yiser-keys.log`. Other remote modes have not been tested.

### Carrot Web mapping follow-up (Cinque v3 experiment only)

Tools → Bluetooth remotes provides radio control, 30-second discovery, explicit
device pairing (including PIN/passkey confirmation), connect/disconnect/forget,
and a per-device input profile and action map. Setup requires fresh stationary,
disengaged telemetry, or a confirmed offroad state. The HTTP API cannot execute
vehicle actions. Pairing is application-scoped; unsolicited pairing requests are
rejected. Smartphone pairing does not implement phone audio, calls or PAN.

`carrot_bluetooth` runs independently of the web server and exclusively grabs
only configured Bluetooth evdev nodes. The mapping is stored outside the repo
in `/data/carrot/bluetooth.json`; pairing keys remain in BlueZ's private store.
Disabling a device's mapping releases its original HID input. Disconnects and
process exit release the kernel grab; reconnects are discovered automatically.
Generic HID keyboard/media keys can be learned from a press/release. Relative
mouse wheels, gamepad axes and arbitrary vendor protocols are not implemented.

| Yiser-J6 button | Default Carrot action |
| --- | --- |
| Up | `accelCruise` |
| Down | `decelCruise` |
| Left / Right | Existing remote lane-change request, left / right |
| Center | Paddle deceleration (physical paddle setting unchanged) |
| 1 | Cycle cruise gap |
| 2 | No action |

The learn/test session lasts 120 seconds and suppresses commands for that device.
Closing the dialog does not end the session early. Save a profile before learning,
then save again to persist newly learned keys/actions. Unmapped events are swallowed
while the mapping is enabled. Key repeats are ignored; actions fire on release or
completion of a short touch gesture. Stale (>400 ms), startup and replayed commands
are discarded. Physical cruise-button events/held buttons have priority, and valid
CAN, cruise availability and Drive are required for cruise actions. Lane requests
retain lateral-active, speed, trailer, geometry, blind-spot and torque checks.
They do not synthesize CAN messages or physical vehicle turn-signal operation.

Validation: replay of the actual seven-button recording, repeat/drop/expiry tests,
HTTP origin/stationary/configuration checks, and cruise/desire regression tests:
75 passed in the C4 Python environment, with warnings treated as errors. Live
BlueZ enumeration and application agent registration also passed. The web dialog
was checked on the device, and a second EVIOCGRAB returned EBUSY while the daemon
owned the remote, confirming exclusive HID capture.

The follow-up live test decoded all four direction gestures to their requested
actions without emitting vehicle commands. Its remaining observed tokens were
`key:114` and `2`; center and button 1 were not independently confirmed in this
run. The earlier seven-button capture still passes replay, but it does not prove
that every button has the same behavior in the remote's current mode. Unknown
key 114 remains unassigned. No further physical testing was requested after the
user ended the session. Evidence: `/data/carrot-bluetooth-mapping-events.json`.

The requested Yiser profile and default mapping are saved on the trial device.
Test mode has ended; persistent vehicle-command activation remains off pending
explicit activation approval. Road behavior and other accessories are not yet
verified. Korean/English usage and scope explanations live in the web dialog;
no global Params setting or generated settings Wiki page is added.

For rollback while stationary, stop comma, restore the matching previous
openpilot OS manifest/version (the pre-trial commit is in `before.json`), select
the preserved B slot with `sudo abctl --set_active 1`, and reboot. Restoring the
matching code avoids immediately requesting the new OS again. Do not flash the
preserved slot as part of rollback.

Docs-Not-Needed: Developer-only OS/model compatibility experiment with no
user-facing setting addition or changed setting behavior.
