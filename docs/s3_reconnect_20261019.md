# S3 Wi-Fi recovery and picture quality, 2026-10-19

The user reports poor picture quality and failure to resume when a company AP and
phone hotspot use the same SSID/password. The user explicitly requests testing
Wi-Fi loss/reconnection on the available network rather than waiting for a
hotspot. Firmware flashing and modification tests are authorized; registration
and credentials must remain intact.

## Changes

- Restore the C4 Wi-Fi H.264 profile from 500 kbps / minimum QP44 to 1 Mbps /
  minimum QP28 (maximum51). Preserve CBR-CFR, the three ordered reference slots,
  <=16 KiB transport chunks, and 125000-byte/s pacing including display headers.
  Neither target bitrate nor QP is a hard encoded-frame size limit.
- Retry authenticated UDP discovery every250ms within the existing deadline,
  matching capability discovery. A lost broadcast no longer consumes the whole
  discovery attempt before the next HUD lifecycle retry.
- Firmware1.2.1 separates association and IP-ready event state. Do not repeatedly
  call connect while scanning/authenticating or already associated without IP.
  Bound association attempts to30s, IP wait to20s, and ordinary retry backoff to
  1/2/4/5s. Expiry disconnects/rejoins Wi-Fi, never restarts the MCU or erases NVS.
- UART S3RECONNECT retries saved credentials without rewriting NVS. S3STATUS
  includes recovery count; RX/TX termination logs include direction and errno
  (zero on orderly EOF). Public firmware has no test AP pinning or fault injection.
- A successful sender sync grants the existing two-second initialization window.
  Previously an expired idle timer could issue its own sync/off during the
  sender setup sequence; this was observed on UART during this investigation.
  If local idle-off resets a live panel decoder, invalidate its TCP session and
  queued reference packets so C4 performs fresh initialization. Preserve normal
  two-second visual idle-off and explicit brightness-zero shutdown.

## Physical evidence before the final production verification

The initial diagnostic flash preserves pairing and uses a strong AP around
-46dBm/channel6. Forcing the other company AP gives about-80dBm/channel1: IP
returns in4.5s and TCP authentication in28.5s. Returning to all-channel selection
gets the strong AP, IP in3.9s and authentication in13.5s, without MCU reboot.
An intentionally nonexistent BSSID yields AP-not-found/zero IP; removing the
restriction restores IP and actual C4 forwarding without reboot. This does not
prove a cross-subnet hotspot transition or an automatic roaming policy.

A diagnostic-only dropped IP-ready notification exercises the manager's
associated/not-ready state. The new policy rejoins after20.06s, gets IP about
4.5s later, and authenticates C4 about13.4s after recovery begins. The injection
suppresses notification only: the network stack had an address and even carried
traffic before recovery. It verifies the state-machine deadline, not a measured
DHCP-server failure or the original cause of the user's lost connection.
Native C policy tests additionally cover association timeout, healthy-state reset,
IP loss and1000 bounded failed attempts.

With restored quality and the original transport timing, a100s actual-S3 native
sequence passes without a transport exception. Random full-colour frames produce
one513072-byte unit and average about2.06 submitted frames/s over30s; moving luma,
random luma and changing blocks recover to approximately10fps. USB ACK latency
stays bounded; no physical USB failure is reported. This stress source is much
more complex than an ordinary dashboard. It demonstrates bounded backpressure,
not a promise of10fps for every possible image. Its combined model observer
misses1/2 IDs; it is not used as a model-rate pass. A separate60s per-topic
observer records1201 consecutive valid model and DM frames each at20.00028 and
20.00038Hz. The final socket snapshot overlaps the next intentional disconnect,
so that observation is not a disconnect-free end-of-test claim.

## Scope and remaining limits

The native encoder library is unchanged from the preceding verified build.
C4 guarded HUD-only installs preserve modeld163840 and dmonitoringmodeld163845
on bootccda54c7-c7f2-461e-a060-eba62ae9682c. No model, driving-control, AP/router,
or eGPU setting is changed. App-only S3 flashes preserve the existing pairing;
NVS and pre-test app backups remain private. Public artifacts never include them.

Strong-AP parked measurements cannot establish all mobile hotspot or road radio
behaviour. These changes do not establish the root cause of every historical
TCP timeout, and do not resolve the separate overnight system-memory increase.
Raw UART, transport and vehicle diagnostic records remain in private analysis.

## Production reconnect verification

The final app is893088 bytes, SHA256
`67c8222e9c2c57e4a03e6c1dc90373c5f9853c7ccccb6d17eec387f5957895d2`.
Five S3RECONNECT cycles on this exact image reacquire IP in3.875-3.938s,
authenticate in13.516-14.016s and verify new-session forwarded packets in
17.344-20.000s (USB statistics sampled every5s). Uptime increases throughout;
all five intentionally caused disconnections have reason8. USB failed count
remains0. Earlier candidate runs are separate and not used for this claim.
Both maintained branches pass60 focused Python tests with one Linux-only skip;
the native C policy assertions pass. Installer Node tests pass5/5; NAS release
file/resume/allowlist test passes, with four Windows symlink cases skipped.

The exact final firmware also passes a second100s physical sequence: colour
noise62 inputs/30.049s (2.063fps), motion201/20.007s, luma noise201/20.035s,
and blocks201/20.027s, with no transport exception. UART retains the same
association/disconnect count and reports0 failed USB transactions. HUD is restored.
The combined stress observer misses one model ID, so a separate independent
final observation is required for model-rate reporting.

## Final vehicle observation and deployment

The guarded Git reconciliation fast-forwards C4 to9af82818a6 and preserves
modeld163840 / dmonitoringmodeld163845. The three installed HUD source files now
match their committed blobs; only pre-existing generated DBC changes remain.
The180s observer spans this planned HUD restart:32/36 connected samples,
model3601 consecutive valid frames, DM3599 with2 observed ID gaps. It is not a
clean uninterrupted model/DM or HUD pass. A subsequent untouched60s interval has
1202/1201 consecutive valid model/DM frames at20.005802/20.000011Hz and12/12 HUD
connected samples, same active eGPU. Available memory564.70->562.82MiB is only a
short observation, not overnight leak clearance. Final independent120s UART has
no extra Wi-Fi disconnection, TCP close, decoder reset or USB error; its stream
counter continues increasing. Actual dashboard typically9.9-10fps/~626kbps.

Code is published to nav9af82818a6 / wip e93d46674e; firmware source manifest
points to the nav code commit. The package verifies17 firmware source hashes,
app size/hash, ZIP CRC, dependency hashes and absence of private registration.
GitHub Carrot Routes image run39197291072 passes its native replay/service tests.
The Korean Wiki installation page is updated separately, preserving pre-existing
local Wiki edits. Browser installer tests are distinct from physical UART flashing.

NAS automatic deployment confirms code e93d46674e. An independent public endpoint
check verifies health, the actual upload-result page and1196 replay frames against
the image-local expected hash. Public index/release.json/app/ZIP/SHA256SUMS/guide
bytes match the verified local1.2.1 package. The app SHA matches the board build.
The final optical picture-quality question has no reply yet; UART/transport
success is not an additional user visual confirmation.
