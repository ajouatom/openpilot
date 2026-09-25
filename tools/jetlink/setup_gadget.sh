#!/usr/bin/env bash
#
# Copyright (c) 2026-, Zeph Leggett.
# This file is part of jetlink and is licensed under the MIT License.
#
# Bring up the jetlink USB gadget on the comma. Run as root, at boot, before
# anything opens the link. The comma is the USB device and the Jetson the host;
# docs/transport.md says why.
#
# Does not bind the UDC: a FunctionFS gadget cannot attach to a controller until
# its descriptors are written, and whoever opens ep0 writes them and binds.
#
#   sudo scripts/setup_gadget.sh
#   sudo scripts/setup_gadget.sh --teardown
#
# On failure the reason is left in $STATUS_FILE as well as on stderr, so the
# openpilot side can say why the link is unavailable.
set -euo pipefail

GADGET=/sys/kernel/config/usb_gadget/jetlink
FFS_MOUNT=${FFS_MOUNT:-/dev/ffs-jetlink}
FFS_NAME=jetlink
CONFIGFS=/sys/kernel/config
# tmpfs on purpose: per-boot state, and the comma's flash is precious
STATUS_FILE=${JETLINK_STATUS_FILE:-/dev/shm/jetlink-gadget}
# pid.codes test allocation; get a real PID before distributing this
VID=${JETLINK_VID:-0x1209}
PID=${JETLINK_PID:-0x0001}

status() {
  # best effort: a device with no /dev/shm still gets the stderr line
  { echo "$1" > "$STATUS_FILE" && chmod 0644 "$STATUS_FILE"; } 2>/dev/null || true
}

fail() {
  echo "jetlink: $1" >&2
  status "error: $1"
  exit 1
}

if [[ "${1:-}" == "--teardown" ]]; then
  if [[ -d "$GADGET" ]]; then
    echo "" > "$GADGET/UDC" 2>/dev/null || true
    rm -f "$GADGET/configs/c.1/ffs.$FFS_NAME" 2>/dev/null || true
    rmdir "$GADGET/configs/c.1/strings/0x409" 2>/dev/null || true
    rmdir "$GADGET/configs/c.1" 2>/dev/null || true
    rmdir "$GADGET/functions/ffs.$FFS_NAME" 2>/dev/null || true
    rmdir "$GADGET/strings/0x409" 2>/dev/null || true
    rmdir "$GADGET" 2>/dev/null || true
  fi
  # A plain umount can block or segfault on a FunctionFS instance whose owner died
  # with endpoints open; lazy-detach unhooks it now and lets the kernel finish.
  umount -l "$FFS_MOUNT" 2>/dev/null || umount "$FFS_MOUNT" 2>/dev/null || true
  rmdir "$FFS_MOUNT" 2>/dev/null || true
  status "error: gadget torn down"
  echo "jetlink gadget torn down"
  exit 0
fi

[[ $EUID -eq 0 ]] || fail "setup_gadget.sh must run as root"

# set -e alone exits without going through fail, leaving last boot's "ok" in
# $STATUS_FILE for the openpilot side to read.
trap 'fail "line $LINENO: $BASH_COMMAND failed"' ERR

# AGNOS builds these into the kernel and ships no /lib/modules, so modprobe is only
# worth trying where a module tree exists; the checks below test for the result.
if [[ -d "/lib/modules/$(uname -r)" ]]; then
  for m in configfs libcomposite usb_f_fs; do
    modprobe "$m" 2>/dev/null || true
  done
fi

mountpoint -q "$CONFIGFS" || mount -t configfs none "$CONFIGFS" 2>/dev/null || true
mountpoint -q "$CONFIGFS" || fail "no configfs at $CONFIGFS; this kernel cannot configure a USB gadget"

# absent means this AGNOS build has no CONFIG_USB_LIBCOMPOSITE, which nothing
# in userspace can fix
[[ -d "$CONFIGFS/usb_gadget" ]] || fail "kernel has no USB gadget support (CONFIG_USB_LIBCOMPOSITE); jetlink needs an AGNOS build that has it"

# No FunctionFS preflight on purpose: the kernel registers the functionfs
# filesystem only while some ffs.* function exists, so /proc/filesystems never
# lists it on a cold boot. The mkdir, mount and ep0 checks below test it in use.

# a gadget needs a device controller; a machine wired host-only has none
shopt -s nullglob
udcs=("/sys/class/udc"/*)
shopt -u nullglob
[[ ${#udcs[@]} -gt 0 ]] || fail "no USB device controller in /sys/class/udc; this device cannot act as a USB gadget"

# refuse to fight another gadget for the controller rather than unbinding it
for other in "$CONFIGFS"/usb_gadget/*/UDC; do
  if [[ -e "$other" ]]; then
    owner=$(basename "$(dirname "$other")")
    bound=$(cat "$other" 2>/dev/null || true)
    if [[ "$owner" != "jetlink" && -n "$bound" ]]; then
      fail "USB gadget '$owner' already holds the device controller ($bound); tear it down first"
    fi
  fi
done

mkdir -p "$GADGET" || fail "could not create the gadget at $GADGET"
cd "$GADGET"

echo "$VID"   > idVendor
echo "$PID"   > idProduct
echo 0x0100   > bcdDevice
echo 0x0320   > bcdUSB            # 3.2: advertise SuperSpeed
echo 0x00     > bDeviceClass      # class is per-interface (vendor specific)

mkdir -p strings/0x409
echo "zoompilot"                              > strings/0x409/manufacturer
echo "jetlink"                                > strings/0x409/product
echo "$(cat /proc/device-tree/serial-number 2>/dev/null | tr -d '\0' || echo 0001)" \
                                              > strings/0x409/serialnumber

mkdir -p configs/c.1/strings/0x409
echo "jetlink inference link" > configs/c.1/strings/0x409/configuration
# self-powered, and as little as the spec allows: the Jetson has its own 12 V feed
echo 0xC0 > configs/c.1/bmAttributes
echo 8    > configs/c.1/MaxPower

# the mkdir instantiates the function and registers functionfs, so a kernel
# genuinely without it fails here
mkdir -p "functions/ffs.$FFS_NAME" ||
  fail "kernel has no ffs gadget function (CONFIG_USB_CONFIGFS_F_FS); jetlink cannot present its endpoints"
# Link once: this is re-run on every deploy, and unlinking a function from a bound
# config force-unbinds the UDC, so an unconditional ln -sf drops a live link.
[[ -L "configs/c.1/ffs.$FFS_NAME" ]] ||
  ln -s "$GADGET/functions/ffs.$FFS_NAME" "configs/c.1/ffs.$FFS_NAME" ||
  fail "could not link ffs.$FFS_NAME into configs/c.1"

mkdir -p "$FFS_MOUNT"
# Owned by the user openpilot runs as: on a root-only mount modeld and jetlinkd
# cannot open the endpoints, and Path.exists() raises rather than returning False.
FFS_USER="${JETLINK_USER:-comma}"
if id -u "$FFS_USER" >/dev/null 2>&1; then
  FFS_OPTS="uid=$(id -u "$FFS_USER"),gid=$(id -g "$FFS_USER")"
else
  FFS_OPTS=""
fi
mountpoint -q "$FFS_MOUNT" || mount -t functionfs ${FFS_OPTS:+-o "$FFS_OPTS"} "$FFS_NAME" "$FFS_MOUNT" ||
  fail "could not mount functionfs at $FFS_MOUNT"

# the check that proves the chain: ep0 is what a client opens to write the
# descriptors and bind the controller
[[ -e "$FFS_MOUNT/ep0" ]] || fail "functionfs mounted at $FFS_MOUNT but has no ep0"

# the client binds the UDC as the openpilot user, so hand it that one attribute
if [ -n "$FFS_OPTS" ]; then
  chown "$FFS_USER" "$GADGET/UDC" 2>/dev/null || true
fi

status ok
echo "gadget ready at $GADGET"
echo "functionfs mounted at $FFS_MOUNT"
echo "available UDCs: $(ls /sys/class/udc | tr '\n' ' ')"
echo "now start the server; it writes the descriptors and binds the UDC"
