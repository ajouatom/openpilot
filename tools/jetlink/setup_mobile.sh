#!/usr/bin/env bash
# iOS-only composite gadget/network helper. Keep the ordinary USB setup intact.
# gadget: before ep0/UDC bind; net: after bind; net-down: stop owned DHCP/address;
# --teardown: after ep0 closes, detach our NCM link, retaining its kernel instance.
set -euo pipefail
umask 077

GADGET=${JETLINK_MOBILE_GADGET:-/sys/kernel/config/usb_gadget/jetlink}
NET_CLASS=${JETLINK_MOBILE_NET_CLASS:-/sys/class/net}
STATE=${JETLINK_MOBILE_STATE:-/dev/shm/jetlink-mobile}
LEASE_DIR=${JETLINK_MOBILE_LEASE_DIR:-${STATE}-leases}
FUNCTION=ncm.jetlink
ADDRESS=192.168.60.1
PORT=5599
FIREWALL_COMMENT=jetlink-mobile-owned-5599
PIDFILE=$STATE/dnsmasq.pid
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

fail() { echo "jetlink mobile: $*" >&2; exit 1; }
[[ $EUID -eq 0 ]] || fail "must run as root"
[[ ${JETLINK_TRANSPORT:-usb} == ios ]] || fail "requires explicit JETLINK_TRANSPORT=ios"
case "${1:-gadget}" in gadget|net|net-down|--teardown) ;; *) fail "usage: $0 {gadget|net|net-down|--teardown}";; esac
mkdir -p "$STATE"
[[ ! -L "$STATE" && $(stat -c %u "$STATE") == 0 ]] || fail "state directory must be root-owned and not a symlink"
chmod 0700 "$STATE"
exec 9>"$STATE/lock"
flock -x 9
trap 'fail "line $LINENO: $BASH_COMMAND failed"' ERR

# A PID file alone is not ownership. Check executable, exact helper-specific
# arguments and saved process start time. Prefer pidfd where supported;
# AGNOS 4.9 has no pidfds, so revalidate immediately before signaling its PID.
# Never fall back to name-based kills.
dhcp_process() {
  python3 - "$STATE" "$1" "$LEASE_DIR" <<'PY'
import errno
import json
import os
from pathlib import Path
import select
import signal
import sys
import time

state, action = Path(sys.argv[1]), sys.argv[2]
lease_dir = Path(sys.argv[3])
pidfile, record = state / 'dnsmasq.pid', state / 'dnsmasq.owner'
if not pidfile.exists() and not record.exists():
  sys.exit(1 if action == 'alive' else 0)
try:
  pid = int(pidfile.read_text().strip()) if pidfile.exists() else int(json.loads(record.read_text())['pid'])
  if pid <= 1:
    raise ValueError('invalid PID')
  proc = Path('/proc') / str(pid)
  if not proc.exists():
    pidfile.unlink(missing_ok=True)
    record.unlink(missing_ok=True)
    sys.exit(1 if action == 'alive' else 0)
  fd = None
  if hasattr(os, 'pidfd_open'):
    try:
      fd = os.pidfd_open(pid)
    except OSError as exc:
      if exc.errno not in (errno.ENOSYS, errno.EINVAL):
        raise
  try:
    args = (proc / 'cmdline').read_bytes().split(b'\0')
    required = [f'--pid-file={pidfile}', '--port=0', '--conf-file=/dev/null',
                f'--dhcp-leasefile={lease_dir / "dnsmasq.leases"}']
    if Path(os.readlink(proc / 'exe')).name != 'dnsmasq':
      raise ValueError('PID does not belong to helper dnsmasq')
    if not all(arg.encode() in args for arg in required):
      raise ValueError('dnsmasq arguments do not belong to helper')
    start = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()[19]
    if action == 'record':
      record.write_text(json.dumps({'pid': pid, 'start': start}))
    else:
      saved = json.loads(record.read_text())
      if saved != {'pid': pid, 'start': start}:
        raise ValueError('PID was reused or is not helper-owned')
    if action == 'stop':
      if fd is not None and hasattr(signal, 'pidfd_send_signal'):
        signal.pidfd_send_signal(fd, signal.SIGTERM)
        poll = select.poll()
        poll.register(fd, select.POLLIN)
        if not poll.poll(2000):
          raise ValueError('helper dnsmasq did not stop within two seconds')
      else:
        latest = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()[19]
        if latest != start or (proc / 'cmdline').read_bytes().split(b'\0') != args:
          raise ValueError('PID changed before signaling')
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 2
        while proc.exists():
          fields = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()
          if fields[19] != start or fields[0] == 'Z':
            break
          if time.monotonic() >= deadline:
            raise ValueError('helper dnsmasq did not stop within two seconds')
          time.sleep(.02)
      pidfile.unlink(missing_ok=True)
      record.unlink(missing_ok=True)
  finally:
    if fd is not None:
      os.close(fd)
except (OSError, ValueError, KeyError, AttributeError) as exc:
  raise SystemExit(f'jetlink mobile: refusing DHCP ownership/operation: {exc}')
PY
}

owned() {
  [[ -f "$STATE/ncm-owned" ]] || fail "NCM is not owned by this helper"
  [[ -d "$GADGET/functions/$FUNCTION" ]] || fail "owned NCM function is missing"
}

netdev() {
  owned
  dev=$(cat "$GADGET/functions/$FUNCTION/ifname")
  [[ "$dev" =~ ^[a-zA-Z0-9_.:-]{1,15}$ && -d "$NET_CLASS/$dev" ]] ||
    fail "NCM netdev not present; run net after the FunctionFS owner binds UDC"
  index=$(cat "$NET_CLASS/$dev/ifindex")
  [[ "$index" =~ ^[0-9]+$ && "$index" -gt 0 ]] || fail "invalid NCM interface index"
}

firewall_backend() {
  if [[ -f "$STATE/firewall-backend" ]]; then
    FIREWALL_BACKEND=$(cat "$STATE/firewall-backend")
    case "$FIREWALL_BACKEND" in iptables|iptables-legacy) ;; *) fail "invalid owned firewall backend";; esac
    command -v "$FIREWALL_BACKEND" >/dev/null || fail "owned firewall backend $FIREWALL_BACKEND is unavailable"
    "$FIREWALL_BACKEND" -w 2 -S INPUT >/dev/null ||
      fail "owned firewall backend $FIREWALL_BACKEND is not operational"
    return
  fi
  local candidate detail
  for candidate in iptables iptables-legacy; do
    if command -v "$candidate" >/dev/null; then
      if detail=$("$candidate" -w 2 -S INPUT 2>&1); then
        FIREWALL_BACKEND=$candidate
        printf '%s\n' "$candidate" > "$STATE/firewall-backend"
        echo "jetlink mobile: using $candidate INPUT isolation" >&2
        return
      fi
      echo "jetlink mobile: $candidate probe failed: $detail" >&2
    fi
  done
  fail "no operational iptables backend; refusing an unisolated cable listener"
}

firewall_rule() {
  "$FIREWALL_BACKEND" -w 2 "$1" INPUT "${@:3}" -d "$ADDRESS/32" -p tcp --dport "$PORT" \
    ! -i "$2" -m comment --comment "$FIREWALL_COMMENT" -j DROP
}

firewall_down() {
  if [[ -f "$STATE/firewall-interface" ]]; then
    local old_firewall_dev
    old_firewall_dev=$(cat "$STATE/firewall-interface")
    [[ "$old_firewall_dev" =~ ^[a-zA-Z0-9_.:-]{1,15}$ ]] || fail "invalid owned firewall interface"
    # Old deployments used the default backend without recording its name.
    # Never guess a different backend when an owned rule may already exist.
    if [[ ! -f "$STATE/firewall-backend" ]]; then
      echo iptables > "$STATE/firewall-backend"
    fi
    firewall_backend
    if firewall_rule -C "$old_firewall_dev"; then
      firewall_rule -D "$old_firewall_dev"
    else
      local result=$?
      [[ $result -eq 1 ]] || fail "could not inspect owned mobile INPUT rule (iptables exit $result)"
    fi
    rm "$STATE/firewall-interface"
    rm "$STATE/firewall-backend"
  fi
}

firewall_up() {
  firewall_down
  firewall_backend
  # Record before insertion so failed setup can safely remove this exact rule.
  # INPUT position 1 prevents a pre-existing broad ACCEPT bypassing isolation.
  echo "$dev" > "$STATE/firewall-interface"
  firewall_rule -I "$dev" 1
  firewall_rule -C "$dev" || fail "mobile INPUT interface isolation could not be verified"
}

leases_prepare() {
  # dnsmasq drops to nobody: keep leases outside root-only control state, in a
  # dedicated directory only that UID can access. PID/ownership files stay 0700.
  dhcp_user=nobody
  dhcp_uid=$(id -u "$dhcp_user") || fail "dnsmasq privilege-drop user nobody is unavailable"
  dhcp_group=$(id -gn "$dhcp_user")
  [[ "$LEASE_DIR" != "$STATE" && ! -L "$LEASE_DIR" ]] || fail "unsafe DHCP lease directory"
  if [[ -e "$LEASE_DIR" ]]; then
    [[ -d "$LEASE_DIR" && $(stat -c %u "$LEASE_DIR") == "$dhcp_uid" ]] ||
      fail "DHCP lease directory is not owned by nobody"
  fi
  install -d -o "$dhcp_user" -g "$dhcp_group" -m 0700 "$LEASE_DIR"
  local lease_file=$LEASE_DIR/dnsmasq.leases
  [[ ! -L "$lease_file" ]] || fail "DHCP lease file must not be a symlink"
  if [[ -e "$lease_file" ]]; then
    [[ -f "$lease_file" && $(stat -c %u "$lease_file") == "$dhcp_uid" ]] ||
      fail "DHCP lease file must be a regular file owned by nobody"
  else
    install -o "$dhcp_user" -g "$dhcp_group" -m 0600 /dev/null "$lease_file"
  fi
}

net_down() {
  dhcp_process stop
  firewall_down
  if [[ -f "$STATE/interface" ]]; then
    read -r old_dev old_index < "$STATE/interface"
    # Do not touch a modem/other device that reused a disappeared NCM's name.
    if [[ "$old_dev" =~ ^[a-zA-Z0-9_.:-]{1,15}$ && -e "$NET_CLASS/$old_dev/ifindex" &&
          $(cat "$NET_CLASS/$old_dev/ifindex") == "$old_index" ]]; then
      if ip -o -4 address show dev "$old_dev" | grep -Fq " $ADDRESS/24 "; then
        ip address del "$ADDRESS/24" dev "$old_dev"
      fi
    fi
    rm "$STATE/interface"
  fi
}

case "${1:-gadget}" in
  gadget)
    [[ ! -e "$GADGET/UDC" || -z "$(cat "$GADGET/UDC")" ]] ||
      fail "close the ep0 owner/unbind UDC before changing mobile descriptors"
    # setup_gadget.sh mounts FFS and links it FIRST, preserving interface 0.
    bash "$SCRIPT_DIR/setup_gadget.sh"
    if [[ -d "$GADGET/functions/$FUNCTION" ]]; then
      owned
    else
      mkdir "$GADGET/functions/$FUNCTION" ||
        fail "kernel lacks CDC-NCM gadget support (CONFIG_USB_CONFIGFS_NCM)"
      echo "$FUNCTION" > "$STATE/ncm-owned"
    fi
    echo 0x0101 > "$GADGET/bcdDevice"
    echo 0xEF > "$GADGET/bDeviceClass"
    echo 0x02 > "$GADGET/bDeviceSubClass"
    echo 0x01 > "$GADGET/bDeviceProtocol"
    # Some AGNOS 4.9 kernels reject configfs MAC writes. Preserve their generated
    # MACs and explicitly report that limitation; DHCP does not require fixed MACs.
    for entry in dev_addr=02:4a:4c:60:00:01 host_addr=02:4a:4c:60:00:02; do
      attribute=${entry%%=*}
      value=${entry#*=}
      if ! printf '%s\n' "$value" > "$GADGET/functions/$FUNCTION/$attribute"; then
        echo "jetlink mobile: kernel rejected fixed $attribute; retaining kernel-generated MAC" >&2
      fi
    done
    link=$GADGET/configs/c.1/$FUNCTION
    if [[ -L "$link" ]]; then
      [[ $(readlink -f "$link") == "$(readlink -f "$GADGET/functions/$FUNCTION")" ]] ||
        fail "unexpected NCM configuration link"
    else
      ln -s "$GADGET/functions/$FUNCTION" "$link"
    fi
    echo "jetlink mobile: composite FFS+NCM ready; bind ep0 owner, then run net"
    ;;
  net)
    netdev
    [[ -n "$(cat "$GADGET/UDC")" ]] || fail "UDC is not bound"
    if [[ -f "$STATE/interface" && $(cat "$STATE/interface") != "$dev $index" ]]; then
      net_down
    fi
    echo "$dev $index" > "$STATE/interface"
    if command -v nmcli >/dev/null; then
      nmcli device set "$dev" managed no
    fi
    ip address replace "$ADDRESS/24" dev "$dev"
    ip link set "$dev" up
    firewall_up
    if [[ -e "$PIDFILE" ]]; then
      dhcp_process alive || fail "existing DHCP state is not a live owned process; run net-down"
    else
      command -v dnsmasq >/dev/null || fail "dnsmasq is required for the iOS cable DHCP service"
      leases_prepare
      dnsmasq --conf-file=/dev/null --bind-interfaces --interface="$dev" \
        --listen-address="$ADDRESS" --except-interface=lo --port=0 \
        --dhcp-range=192.168.60.2,192.168.60.254,255.255.255.0,10m \
        --dhcp-option=3 --dhcp-option=6 --dhcp-leasefile="$LEASE_DIR/dnsmasq.leases" \
        --user="$dhcp_user" --group="$dhcp_group" --pid-file="$PIDFILE" 9>&-
      dhcp_process record
    fi
    echo "jetlink mobile: $dev at $ADDRESS/24; isolated TCP $PORT INPUT rule, scoped DHCP, no router or DNS"
    ;;
  net-down)
    net_down
    ;;
  --teardown)
    [[ ! -e "$GADGET/UDC" || -z "$(cat "$GADGET/UDC")" ]] ||
      fail "close the ep0 owner/unbind UDC before NCM teardown"
    net_down
    if [[ -d "$GADGET/functions/$FUNCTION" && ! -f "$STATE/ncm-owned" ]]; then
      fail "refusing to remove NCM not owned by this helper"
    fi
    if [[ -e "$STATE/ncm-owned" ]]; then
      owned
      link=$GADGET/configs/c.1/$FUNCTION
      if [[ -L "$link" ]]; then
        [[ $(readlink -f "$link") == "$(readlink -f "$GADGET/functions/$FUNCTION")" ]] ||
          fail "refusing to remove an unexpected NCM link"
        rm "$link"
      fi
      # AGNOS 4.9 has faulted in configfs rmdir after an NCM session. Detach
      # the unbound configuration link, but reuse the function until reboot.
      echo 0x0100 > "$GADGET/bcdDevice"
      echo 0x00 > "$GADGET/bDeviceClass"
      echo 0x00 > "$GADGET/bDeviceSubClass"
      echo 0x00 > "$GADGET/bDeviceProtocol"
    fi
    ;;
esac
