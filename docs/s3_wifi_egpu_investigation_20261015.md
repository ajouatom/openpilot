# S3 HUD connection and eGPU incident investigation — 2026-10-15

## Results

Three independent issues were found on the user's C4/S3 installation:

1. The S3's home-network 2.4 GHz link had large delays despite strong signal.
   Separating its AP from channel 1 to channel 6 at 20 MHz improved wired-host
   ping from 13/20 replies (35% loss), mean 217 ms / max 595 ms, to 20/20,
   mean 6.35 ms / max 28 ms. After removing the temporary client AP lock,
   another run remained 20/20, mean 6.7 ms / max 28 ms. The approved AP radio
   change is retained; the temporary client lock is removed and verified after
   reloading the controller. Other APs and 5 GHz settings were unchanged.
2. C4 had no S3 pairing registration. After authenticated connectivity recovered,
   the existing device/key was saved locally with mode 0600. The HUD then exposed
   a startup defect: it opened the bridge connection before slow graphics/font
   initialization, exceeding firmware's two-second TCP idle receive timeout.
   Deferring the S3 connection until the renderer is open allows continuous HUD
   rendering. Direct USB still opens before geometry selection; S3 supplies its
   known 1920x462 geometry before connecting. JPEG workers start after the renderer.
3. Three new eGPU incidents were output-read timeouts, independent of S3 Wi-Fi.
   The underlying GPU/driver/physical-link cause is not established by these logs.

## Wireless evidence and limits

Omada manages five EAP610-Outdoor APs; the LAN gateway is a separate ipTIME router.
The S3 initially reported RSSI -49 dBm, SNR 46 dB and power save disabled. This is
not evidence of a missing antenna or weak reception. The affected 2.4 GHz radio
reported about 40–46% utilization; utilization includes Wi-Fi traffic, not just
interference. A direct AP-to-S3 ping measured mean 151 ms / max 630 ms. Wired pings
to the gateway and both relevant APs remained about 1 ms, localizing the delay to
the wireless client path rather than the wired uplink or Internet.

Immediately after changing the AP channel, S3 roamed to another AP still on channel
1, so that initial post-change result is **not** a channel-6 comparison. A temporary
client AP lock provided a verified channel-6 association. The initial handover run
had losses; the subsequent stable and unlocked runs above did not. This is a short
field comparison, not an A/B/A interference survey or proof of a particular RF
interferer. Three PC TCP authentication attempts recovered to 31/16/16 ms.

C4 uses a different AP on 5 GHz, RSSI about -70 dBm, power save enabled. Its wired-
host ping still averaged 92–136 ms in these comparisons. Both a temporary
NetworkManager powersave reapply and Wireless Extensions power-off request were
rejected by the device. Neither changed the active setting or saved profile; no
network reconnection was forced. Power save and weaker reception are candidates,
not proven causes. Phones can use different bands/APs and do not validate S3's link.

## Host fixes and device verification

- S3 connection opens after graphics initialization, with dimensions available
  beforehand. No TCP timeout, firmware, authentication or eGPU policy was relaxed.
- The USB-device wait notices a newly saved S3 registration at the existing HUD
  check cadence, returns to transport selection, and retains the offroad output
  gate. Importing registration no longer requires an unrelated USB event/reboot.
- Thirty focused transport, registration, geometry and rate-policy tests pass.
  Tests cover registration appearing during USB wait, including output becoming
  disallowed, and preserve the live eGPU 5 FPS cap.
- Native C4 installation checked fresh Park/zero-speed/inactive controls and exact
  prior hashes; only three HUD Python files were replaced. The HUD catches SIGTERM
  inside its render loop, so it retried with already imported code. A guarded
  termination of **only** the HUD process was required for the manager to load the
  new files. Model and driver-monitoring process IDs remained unchanged.
- The new HUD process maintains a registered connection and renders the existing
  camera layout, typically 9–13 Hz in initial reports. This is renderer cadence,
  not measured optical panel FPS. Model and driver monitoring each produced 200
  valid updates in a ten-second check. Physical panel confirmation and sustained
  road operation are separate checks; a 15–20 FPS vehicle guarantee is not made.
  A later thirty-second check recorded 600/601 valid model/DM updates (maximum
  gaps 74/69 ms), the same process IDs and a connected HUD. Recent renderer reports
  ranged 6.9–13.2 Hz, mostly 10–13 Hz; occasional slower periods remain.

Phone setup and UART provisioning remain as documented in
[`tools/esp32s3_display/README.md`](../tools/esp32s3_display/README.md).
Offroad registration is supported; normal offroad display still requires the
existing `ClusterHudDebug` setting. No new firmware was flashed in this test.

## New eGPU incidents

Eight NAS reports and eleven completed vehicle rlog segments were copied and
SHA256 verified privately. Duplicate reports of the same incident were deduplicated.
The new failures use the NAV-RL a03bb1d4 / AMD 20cae5d0 model and pinned 90970e01
runtime, not the separate Mountain Dew branch configuration.

| Local time | Output-read timeout | Model publication gap | First fallback capture age | DM publication gap |
| --- | ---: | ---: | ---: | ---: |
| Oct 14 22:47:30 | 1159 ms | 3015 ms | 3045 ms | 1498 ms |
| Oct 14 22:52:50 | 1147 ms | 2930 ms | 2952 ms | 1395 ms |
| Oct 15 06:57:04 | 1192 ms | 3026 ms | 3042 ms | 1492 ms |

Normal preceding worker times averaged about 43 ms, with p99 about 44–45 ms.
The first internal fallback result was marked valid despite its old capture time;
the next fresh result followed about 298–308 ms later. This corroborates the
already documented fallback latency limitation, not safe continuous perception.
Camera publication gaps in these windows remained about 59–64 ms. Communication
and localization errors accompanied disengagement in the examined failures.

The current eGPU is directly attached at USB 4-1 / 5000 Mbps. Its link counter was
6 before and after a 20-second inactive check. No explicit USB reset/-71 evidence
was retained around the new timeouts; that absence does not exonerate hardware.
The worker's D state / `_do_fork` wait-channel snapshot does not identify the full
blocked operation. Current low free memory is not proof of memory pressure at the
historical fault. An earlier cable-reseat report is kept separate from spontaneous
drive incidents. No eGPU model, runtime, recovery policy or vehicle control code
was changed or restarted for this investigation.

Raw routes, device identities, captures, network details and keys remain in the
private `.analysis/scratch/2026-10-15-egpu-s3/` evidence directory, not in this document.
