# Orin Nano C-to-C host policy hotfix

Scope: NVIDIA P3768 reference carrier with L4T 36.4.7. Jetlink needs the
Jetson to be the USB host and the comma to be its USB device. The default
FUSB301 Try.SNK policy can instead connect Jetson as `0955:7020` under the
comma host. This hotfix uses the installed driver's sysfs policy interface;
it does not replace the kernel, firmware, model, or USB speed checks.

On a parked, disengaged installation, copy `usbc_host.py` and `install_usbc.py`
from this release into the same directory on the Jetson, then run:

```sh
sudo python3 install_usbc.py
sudo systemctl daemon-reload
sudo systemctl start carrot-jetlink-usbc-host.service
systemctl is-enabled carrot-jetlink-usbc-host.service
systemctl is-active carrot-jetlink-usbc-host.service
cat /sys/class/usb_role/usb2-0-role-switch/role
lsusb -t
```

The installer enables the policy before inference on future boots. When already
selected, it makes no writes that would detach the working connection. Selecting
the policy on a connected device-mode port briefly disconnects that USB session.
Linux USB-C is dedicated to host operation; USB-A host ports remain available.
Firmware recovery is a separate boot mode. Suspend/resume is not validated.

To restore the normal Linux dual-role preference:

```sh
sudo systemctl disable --now carrot-jetlink-usbc-host.service
sudo python3 /usr/local/lib/carrot-jetlink/usbc_host.py --restore-dual-role
```

`install_usbc.configure(mounted_root)` can also patch an offline mounted image
without rebuilding its OS/model. New image builders call it automatically.
The already-published v0.2.0 image does not run arbitrary files placed on
CARROTSETUP. For an existing installation, use authenticated SSH to install the
hotfix. If it has no management key, add your SSH **public** key to
CARROTSETUP/setup.json and boot once; no image rewrite is needed. Management
keys are owner-specific and are never included in the public image.

Source semantics:
https://github.com/OE4T/linux-nv-oot/blob/jetson_36.4.7/drivers/usb/typec/fusb301.c
(`fsw_trysnk_store`, `fmode_store`, `fusb301_detach`, `fusb301_snk_detected`).
Hardware host support:
https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html

Validation is recorded separately from installation: successful role negotiation
alone is not inference, driving, another carrier, or suspend/resume validation.
