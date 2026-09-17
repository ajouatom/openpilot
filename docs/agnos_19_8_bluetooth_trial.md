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

The initial C4 slot is `_b`, running AGNOS `19.6.3-carrot`. Its existing
Bluetooth firmware partition contains `crbtfw11/20/21.tlv` and corresponding
`crnv11/20/21.bin` files. No failed system services were present at the baseline
check. New-OS boot, Bluetooth discovery/pairing and post-update inference are
pending an offroad installation window; build success is not hardware validation.

Docs-Not-Needed: Developer-only OS/model compatibility experiment with no
user-facing setting addition or changed setting behavior.
