# Jetlink protocol 3 compatibility

The October 6 connection failure was a wire-version mismatch: the comma sent
protocol 2, and the current Mac app rejected it with `peer speaks protocol v2,
we speak v3`. The comma then timed out waiting for the 32-byte HELLO response
header. Changing cables or adding a hub cannot repair that mismatch.

## Reference and scope

Reviewed upstream zoompilot `develop` at
[`e8058c0b6707a40409dec39a9963e6ba26589ef8`](https://github.com/zoompilot/zoompilot/tree/e8058c0b6707a40409dec39a9963e6ba26589ef8)
and its Jetlink pin
[`24e8917673829cbe1d146735956e9038cc93f227`](https://github.com/zoompilot/jetlink/tree/24e8917673829cbe1d146735956e9038cc93f227).
The supplied `jetlink-main` checkout provides the protocol-3 reference.

This is a compatibility adapter for Carrot's existing daemon/local IPC, not a
wholesale migration to upstream's separately versioned openpilot API 2.
The internal model, generic eGPU path, camera warp validation, switching
conditions, finite-output checks, 150 ms local inference deadline and failure
notification are unchanged. Current protocol-3 Mac/iPhone/Android Apps own
external model selection: Carrot follows their prepared, loaded model while
offroad. The legacy protocol-2/Jetson path retains its exact Cinque v2 ONNX,
SHA256 `09d080f36965bb2a0790500452bd328aa03c484d0222aa79d1ad9f021a522aec`.
No internal driving model, Jetson image or signed host release is changed.

## App model selection

Upstream's **Use Model** loads the external engine; it does not change a comma
model-manager parameter. Stock zoompilot independently requests its own pick,
and the server can restore that pick after an App-requested build. Carrot's
App-following adapter is intentional additional behavior, not a claim that
copying upstream automatically enables remote selection.

With ignition off, select **Use Model** in the App and let preparation finish.
HELLO's `loaded` SHA selects the engine; ENGINE_REQ obtains its full spec. A
zero-byte request for a different App engine means use the resident/cached
engine, never upload an empty model. Carrot does not download Cinque v2 over
the App's choice. With no loaded model, it waits on the same physical cable,
polling STATE rather than choosing another model.

Before READY, Carrot validates the SHA, actual model size, context stride,
camera/scalar/state shapes and every driving output used by its parser and
publisher. The approved spec is saved atomically to
`/data/models/carrot-jetlink-mac/selected.json` (the old cache directory name is
retained for upgrades). A reconnect onroad may reuse that exact approval;
first adoption of a different model requires ignition off. A changed contract
under the same SHA or corrupt approval is an explicit error, not a default.

After warming, an idle offroad session sends HELLO to release its old engine
request without unplugging USB/NCM. Thus a later App **Use Model** is not
silently overwritten by the server's pending-request logic. A changed/unloaded
engine causes a fresh handshake and contract validation. Local inference
re-establishes the approved request before its IPC handshake.

Queued models (including Cinque v2) and graph-stateful models (`new_img` and
matching `next_state_*`, as in Cinque Terre v3 onward) use per-session specs
throughout daemon, local IPC and modeld. There is no hard-coded 18,452-element
reply or `prev_feat` access for stateful graphs. Older five-hypothesis plans
and multi-hypothesis leads are parsed into Carrot's existing message layout.

This does **not** accept arbitrary ONNX models just because the App lists or
loads them. The supported driving contract has narrow/wide uint8
`(2, 6, 128, 256)` frames, eight desires, two traffic-convention values, two
action times and context stride four. Driving slices must match Carrot's
33-step plan, lane/edge, lead, pose and meta semantics. Other resolutions,
control inputs or output formats fail explicitly. A compatible spec is not
evidence of inference accuracy or real-time performance on every device.

Never change models while assisting the driver. If the App replaces the
engine during inference, NOT_READY or loaded-SHA mismatch ends the session;
the existing local-model fallback and disengagement fault remain. No output
from a differently shaped model is parsed under the old spec.

## Wire compatibility

The comma tries protocol 3 first. If HELLO fails in automatic mode, it closes
that session and tries protocol 2 on a fresh connection. It never changes
framing halfway through a stream. The working version is retained for the
daemon's following sessions. `JETLINK_PROTOCOL=2` or `3` pins a version;
phone modes require 3.

The vendored server still defaults to protocol 2, preserving existing Jetson
host behavior. Protocol selection is per transport, not a process-global
constant. Both versions retain strict header-version checking.

For protocol 3 the adapter removes queued models' `prev_feat` from the request:
Cinque v2 sends 12 scalar floats rather than 16,396. Stateful models already
pack only those scalars and have no caller-owned recurrent features. The server owns its hidden
state and resets it with `RESET_QUEUES`. The adapter requests `WANT_HIDDEN`,
so its response contains the complete output specified by the selected graph
for IPC, parsing and raw-prediction logging. Hidden state is
never reconstructed as zero. This preserves the existing local contract,
but does not claim the upstream compact-response bandwidth optimization.
Protocol 2 keeps its original client-side recurrent inputs and refuses stateful
graphs before sending inference.

Protocol 3 uses the upstream 512-byte short-packet padding rule; gadget
transmissions retain 16 KiB alignment. Protocol 2 retains its own padding.
HELLO validates the selected version, and full model identity/metadata must
match before inference.

## Device setup

Only test while parked with assistance disengaged. First-time model download,
upload and engine preparation require ignition off and a powered, online
comma. Existing verified caches are reused.

| Host | Persistent comma mode | Physical/app path |
| --- | --- | --- |
| Automatic cable selection | `auto` (new default when no file exists) | Composite FFS + NCM; TCP grace then bounded sequential USB discovery |
| Mac Mini / Apple Silicon Mac | `usb` | Current Jetlink app, USB server, CoreML/ANE |
| Existing Jetson | `usb` | Existing vendor USB bulk server; protocol-2 fallback |
| Android | `android` | Current Jetlink app, USB-host permission; vendor bulk interface |
| iPhone / iPad | `ios` | Current app dials comma over USB CDC-NCM/TCP |

On the comma, select a phone mode while offroad, then reboot:

```sh
printf 'android\n' > /data/jetlink-transport
# For an iPhone/iPad use 'ios' instead of 'android'.
sudo reboot
```

Existing `/data/jetlink-transport` files remain overrides: a deployed `android`
file does **not** change automatically. To explicitly opt into the new default:

```sh
printf 'auto\n' > /data/jetlink-transport
sudo reboot
```

For Mac or Jetson, select the manual USB path:

```sh
printf 'usb\n' > /data/jetlink-transport
sudo reboot
```

`JETLINK_TRANSPORT` overrides that file for controlled development runs.
Invalid/unreadable configuration reports an error rather than choosing a
different transport silently. These are daemon startup choices, not live
onroad switches or new UI settings.

**Mac:** stop any second server competing for the USB interface. Use the
current native protocol-3 App and its model picker; a separate protocol-2
Python server is not needed. Mac, iOS and Android share the same App-following
model-contract path, but keep their distinct cable transports.

**Android:** grant the app permission for the comma's `1209:0001` vendor
interface. ORT/QNN (`htp`, `htp-whole`, `gpu`) and LiteRT (`gpu`, `npu`) tags
are recognized on the selected USB path. CPU and unknown peers do
not gain automatic provisioning. Android is not the iOS network path.

**iOS:** a composite gadget adds NCM after FunctionFS. The comma retains ep0
to keep it enumerated, but never sends or receives inference on the vendor
endpoints. The actual NCM interface name is read from configfs; it is not
assumed to be `usb0`, which the modem may own. DHCP gives the phone an address
in `192.168.60.0/24`, without a default gateway or DNS. The app dials
`192.168.60.1:5599`; the comma does not dial the phone. The listener is confined
to this cable, not a general Wi-Fi/LAN inference service.

The iOS setup needs kernel NCM support, `dnsmasq`, and the firewall tooling
used by the helper; missing facilities fail explicitly. This patch does not
install a new AGNOS image or take over another gadget/controller.

Use USB 3 data cables/ports throughout. For phones, a powered USB 3 hub is
recommended: direct C-to-C power/data-role negotiation is a separate hardware
issue. This patch does not force charger voters, USB-PD swaps, phone charging,
or system-wide VM/CPU policies. A phone must act as USB host and the comma as
USB device. In phone modes the daemon checks the data role (`ufp`), not merely
the power role. Direct-cable negotiation on every C3/C3X/C4 is not established.

The manual Mac/Jetson `usb` path still requires SuperSpeed. Explicit phone modes
also permit high-speed/USB 2 as current Jetlink does, with the same inference
deadlines and fallback checks; this is not a promise of 20 Hz performance.
Full/low/unknown speed is rejected.

### Automatic transport arbitration and current limitation

Auto prepares a composite FunctionFS/NCM gadget and keeps its cable listener
alive before sending any HELLO. An accepted, interface-isolated cable-subnet
TCP connection is decisive evidence of the network path, not authenticated
phone identity. The comma remains the Jetlink **client**; the App server
receives the comma's HELLO. Supported iOS HELLO assertions are checked before
preparation; an M-series iPad on TCP is labelled iOS, never Mac.

**There is no passive USB host-claim signal in the current protocol.**
Upstream's `jetlink/comma/owner.py` chooses USB or iOS
explicitly and lends either the bulk endpoints or the cable listener; it does
not implement a host-claim signal. FunctionFS ENABLE/configured and
SET_INTERFACE are not such signals: iOS can enumerate the vendor interface
too, and a host's local USB claim/read is not a distinct ep0 event. Both USB
and TCP servers are passive until the client sends HELLO. A synchronous FFS
write, even with O_NONBLOCK, can wait until its watchdog unbinds the entire
gadget. Repeated speculative HELLOs could therefore drop NCM every 15 seconds.
No concurrent dual HELLO, OS-descriptor fingerprint, power-role identity
inference, or unverified kernel-AIO cancellation is used here.

Instead, auto uses **sequential discovery**. The isolated NCM TCP listener gets
a five-second grace period (polled in one-second accept windows). At every
selection attempt TCP is checked first. If no App dials and the controller is
configured, the comma tries USB HELLO with a three-second discovery budget
covering endpoint-readiness checks, the FFS write watchdog and HELLO reception.
`transport-selection` and `usb-probe` status messages report this pending work.
No TCP HELLO is sent while the USB probe is running.

If that probe fails, closing/unbinding the owner and rebuilding the composite
gadget causes **one re-enumeration per failed probe**. A phone dialing late may lose its socket
and must redial; setup/DHCP and the two-second retry delay add latency beyond
the grace/probe budget. The watchdog's UDC cancellation is not guaranteed to
return on a broken native kernel. Cleanup verifies reader/watchdog termination
and controller unbind; uncertainty blocks further attempts. USB discovery is
limited to **two fresh probes** with automatic wire selection: protocol 3,
then protocol 2 on a freshly cleaned/rebound gadget, with TCP grace/checks
before each. Explicit `JETLINK_PROTOCOL` pinning permits only one discovery
probe. After exhaustion USB discovery is disabled until physical detach:
TCP-only waiting explicitly reports the fallback, so there is no repeated
15-second watchdog bounce loop. A decisive
TCP dial also disables USB probing for that attachment, including App retries.
If NCM setup is unavailable, auto blocks with an explicit error rather than
silently becoming USB-only.

Path-aware USB HELLO classification recognizes Apple Silicon Mac and Android
tags; recognized Apps permit USB 2 in auto with a throughput warning. Known
legacy Orin/Jetson USB peers retain SuperSpeed and the fixed legacy model path.
Automatic discovery therefore supports the legacy protocol-2 fallback without
changing framing in a live stream. A slow-starting USB App may still miss both
bounded attempts. Select manual `usb` or `android` for persistent USB-only
retries. These limitations and unavoidable
late-iOS re-enumeration mean auto is **not** seamless simultaneous arbitration
or a physical compatibility guarantee.

Auto checks `current_dr=ufp`, independently of USB-PD power role. Only systems
where that attribute is missing fall back to `Source attached`; a present
`dfp`, empty or unreadable attribute never takes over an eGPU's host role.
This does not force data-role negotiation or guarantee a direct cable works.

TCP disconnects/retries close only their session sockets; ep0, NCM and the
listener remain alive. A physical detach clears peer/wire-version history and
the next session re-enters the existing approval gates and recurrent resets.
During an active session, data-role loss or UDC configuration loss ends the
session and latches the detach through cleanup, even if the cable is quickly
replugged. This also treats a bus reset as a fresh session, not proof of a new
platform. Existing offroad adoption/exact onroad approval rules still apply.
The helper's NCM function is retained rather than deleted each disconnect.
This avoids the reported AGNOS configfs `rmdir ncm.jetlink` fault; it does not
prove the cause of an earlier whole-device reboot. The helper probes the
default `iptables` backend, then `iptables-legacy` if necessary, and records
the selected backend for exact-rule verification/deletion. An owned rule
never migrates silently to another backend during cleanup. If neither backend
works, the isolated TCP listener is not enabled.
Cleanup failures are logged and put the daemon in fail-closed `blocked` state
instead of escaping into a manager restart/rebind loop; resolve ownership and
restart/reboot offroad before retrying.

## Confirmation and limits

Check the comma's `/dev/shm/carrot-jetlink.json` and host app logs. A successful
session has `hello from carrot-jetlink/...` on the host and `Jetlink ready` on
the comma, followed by a READY badge. READY means preparation finished, not
that the external model is active. The existing guarded switch conditions
still apply. Phone labels use the configured transport so an M-series iPad
on the iOS cable is not displayed as a Mac.

Desktop tests cover actual framed protocol-3 HELLO/engine/inference against
the supplied latest runtime, legacy protocol-2 behavior, request/output sizes,
recurrent reset, queued/stateful layouts, App selection and persisted approval,
dynamic IPC/output parsing, version rejection, preparation guards and mobile
listener lifecycle. They cannot establish physical C3/Mac/iPhone/Android enumeration,
pixel parity, thermal behavior or sustained driving performance. Confirm
those separately while parked before considering driving evaluation.

Auto tests additionally cover TCP grace, bounded sequential v3/v2 USB attempts,
fresh gadget/TCP recovery without repeated probing, real framed TCP
HELLO and reconnect, path-aware classifications, configuration overrides,
data-role/eGPU exclusions, speed policy and cleanup failures. No desktop test
can establish physical USB-claim detection, cable negotiation, endpoint
cancellation, or the safety of an untested transport on a vehicle.

The current upstream source itself notes Android QNN/LiteRT device-validation
gaps and potential fp16 numerical errors. Keep iPhone foreground, use the
App's benchmark/parity facilities where available, and do not interpret
READY or passing desktop tests as a guarantee that every selectable model
or backend is safe or fast enough for driving.
