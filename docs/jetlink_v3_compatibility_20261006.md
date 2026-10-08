# Jetlink v2/v3 compatibility — 2026-10-06

The reported `peer speaks protocol v2, we speak v3` comes from the newer
upstream server rejecting Carrot's pinned v2 header. USB discovery alone
cannot resolve this. The existing deployed Carrot Jetson remains on v2;
this change does not require updating it or rebuilding its image.

## Scope

- Keep the vendored `194ff6dc` v2 runtime and its default wire behavior.
- Add the unmodified v3 protocol definitions from upstream
  `24e8917673829cbe1d146735956e9038cc93f227`; record the single transport hook
  in `third_party/jetlink/UPSTREAM.md`.
- Each fresh transport uses one strict protocol. Try v2 first, close a failed
  HELLO session, then try v3 on a new connection. A successful HELLO keeps that
  version for retries until physical detach. There is no upstream version
  negotiation, so the first v3 connection can log one v2 mismatch and incur
  the existing HELLO timeout/retry delay. An unsupported peer never becomes ready.
- No protocol switching after model or inference errors. Header/HELLO
  disagreement, corrupt streams and unsupported versions fail closed.
- The existing USB role guard still requires a host/source peer; AMD eGPU
  role ownership and the USB3 requirement remain unchanged.
- Mac and Android vendor-USB peers can use the new protocol path. iPhone/iPad
  require a separate CDC-NCM/TCP gadget lifecycle that is **not implemented by
  this change**. Protocol support alone does not provide iPhone connectivity.

## Model and wire contract

Protocol v3 and the Cinque v3 **model** are different version axes. Keep the
exact existing NAS Cinque v2 model hash, checkpoint, shapes, slices and frame
skip. A different preloaded app model does not authorize changing our model.

The local modeld IPC remains unchanged. v3 requests carry only desire[8],
traffic convention[2] and action_t[2] after the camera image: 48 scalar bytes.
The old prev_feat is not sent; the upstream v3 server maintains recurrent
state, resetting it on HELLO/engine load/RESET_QUEUES and retaining only finite
outputs. Carrot keeps its existing explicit reset on each modeld join.

Always request v3 WANT_HIDDEN. This returns the **actual full output vector**,
including hidden state, so existing parsing, IPC, raw prediction diagnostics
and validity checks are preserved without filling omitted outputs with zeros.
The cost is that this compatibility implementation does **not** take upstream's
approximately 74 KB to 8 KB response reduction. It still removes prev_feat
from the request. No upstream late-frame reuse, fallback, scheduling or driving
policy is imported.

Model provisioning uses the existing verified NAS download/upload mechanism
for v3 peers as well as the earlier Mac case. Downloads, uploads and pending
build waits require ignition off; an already loaded exact model may reconnect
without downloading. The cache keeps its historical `carrot-jetlink-mac` path
to reuse verified files. Deployed non-Mac v2 Jetsons retain their legacy call.

## Validation and remaining work

- Independent raw-header fixtures check v2/v3 framing, 512/1024-byte padding,
  16 KB gadget alignment, fragmented reads and consecutive messages.
- Socket tests cover v3 HELLO, verified-model upload, readiness, full outputs,
  telemetry and reset requests. Existing v2 socket/server and model guards run
  alongside them. The socket fixture substitutes model execution.
- Unmodified upstream v3 Python client/spec were executed separately against
  our pinned manifest. Serialized model metadata matches; the 393,272-byte
  request payload and complete output bytes match the committed reference hashes
  in `tools/jetlink/test_v3.py`.
- Reject wrong models, failed statuses, frame mismatch, truncated/compact
  responses despite WANT_HIDDEN, unexpected tails, invalid telemetry, NaNs and
  timeouts. Missing/preparing models cannot initiate onroad provisioning.
- Windows suite: 303 passed, 29 skipped (platform-dependent coverage).

Real Mac/Android application handshakes, USB re-enumeration, sustained inference
latency, numerical accelerator parity and thermal behavior remain unvalidated.
There is no claim of on-device or driving validation from desktop fixtures.
Initial tester acceptance is ignition-off preparation followed by a parked
model/latency/reconnect trial; preserve existing control and fault policy.

## Tester handoff

Update the comma to the commit containing this change and reboot. Keep the
existing Carrot Jetson installation unchanged. For a v3 Mac/Android app, use
USB3 with the phone/computer acting as USB host; stay offroad and online for
the first exact-model preparation. A first v2 mismatch followed by v3 HELLO
can be expected, but repeated v2/v3 mismatches are not success. Share the
comma commit, app version, host model, USB arrangement and both sides' logs.

An iPhone still needs the separate transport work described above. Do not
interpret this update as support for all three devices in the initial report.
