# Carrot prebuilt distribution

Source: carrot-wip, 3bbc3c124360edf93815c2c08466206145071635.
Target: aarch64, Python 3.12, AGNOS 19.8-carrot-bt1.

This release ships native executables/bindings, Panda firmware, the internal-GPU
driving model, Super Leicht driver monitoring, and both camera-resolution warps.
The prebuilt JSON manifest records artifact/input hashes and platform requirements.
Startup validates these files and the native Params registry before skipping SCons.
Missing or changed artifacts fall back to the source build path.

An initial OS update/reboot and Python dependency downloads may still be needed.
The optional Cinque v3 eGPU model/runtime remains on the configured NAS and is
downloaded by the normal background updater. It is not duplicated in this branch.
The obsolete Qt updater LFS pointer is omitted; launch uses system/ui/updater.py.

Packaging used an isolated checkout. No device settings, routes, logs, credentials,
SCons cache, stale eGPU artifacts, or device-specific Python dependencies are included.
