# CarrotAssist Android public beta

In 0.2.10, tap the top device name and **Copy ChatGPT instructions** to copy the localized prompt and see how to append it to your existing ChatGPT custom instructions.

Version 0.2.9 sends balanced lane phrase hints to Android 13+ recognizers and explains rejected final transcripts. If speech-start timing is available, the freshness check excludes the wait before speech. Phone voice uses a device recognizer and command grammar. Mini voice uses the default assistant, which selects its own tools.

CarrotAssist 0.2.8 supports ordinal follow-ups in the same phone voice session. After “Find acceleration settings”, say “Set the first one to 240”. The selected row sets the target, and the spoken response names the setting that was saved.

Version 0.2.7 adds direct setting search and absolute integer writes. Say “Find acceleration settings” or “가속도 설정을 찾아줘”. For writes, enable setting access in Carrot Web Tools and say “가속도 설정 0을 240으로 바꿔줘”. The app requires one exact catalog match and consistent final recognition. The device validates the value and verifies storage. No ChatGPT subscription or MCP registration is required.

The app uses Android 12+ on-device speech recognition for direct Korean and English left/right lane requests and opening Automatic turn signals. The phone checks the visible Carrot Web page, greets and listens. No MCP connection is needed for this route. Motion uses only whole final imperatives, consistent alternatives, a high reported score or a fresh direction-specific confirmation when scores are absent, and the existing device drive grant and controller. Questions hand off to the default assistant, where the user repeats the question. Disable direct listening in device settings to use the default assistant first.

당근파일럿 AI 조수. Display name: CarrotAssist.

The launcher uses the approved Carrot mascot with CARROT above and ASSIST below.
WebView text zoom is 100% so Android's font scale does not additionally enlarge
the existing web page's CSS typography. Native controls retain system scaling.

Small native host for the existing device web service on port 7000. It does not
run an AI model or use an API key. The microphone button asks Android to show
the current default assistant with `Activity.showAssist`; it does not simulate
the side key, replace the default assistant or request microphone/accessibility
permissions. The assistant controls the shape and behavior of its own UI.

On launch, discover private IPv4 addresses on connected Wi-Fi and local Wi-Fi/AP
interfaces. Probe port 7000 with at most 32 workers. Confirm both the Carrot Navi
capability response and Carrot Web document markers before opening a device.
These markers identify the software; they are not cryptographic authentication.
Use a trusted LAN, as with Carrot Web itself.

- Wait through the initial five-second discovery window. One device opens
  automatically; two or more require selection. No result continues to 60 seconds.
- A remembered IP is probed first only when inside the current subnet. It does
  not bypass verification or override the multiple-device picker.
- Each subnet prioritizes the phone's /24, then checks up to 1024 addresses in
  that subnet. Larger/IPv6-only networks may require manual IPv4 entry.
- Repeat finite sweeps while waiting for a device to boot. Stop and cancel
  connections at the deadline, user cancellation, selection or backgrounding.
- Manual entry also verifies Carrot Web. HTTP redirects are rejected by probes.
- App WebView storage is separate from Samsung Internet/Chrome. Approve ChatGPT
  once from this app. In 0.2.2, discovery/selection transfers the most recently
  used saved ChatGPT connection to the selected Carrot Web, even a different
  device. IP changes do not require another code. No device-identity condition
  gates this connection. After backgrounding, an unreachable address triggers
  discovery on return. No outside browser storage is imported.
- An isolated, network-blocked WebView reads only the app's four connection keys
  from at most 16 local origins. Browser credentials go only to the fixed HTTPS
  relay. Relocation retains the OAuth session/scopes and discards pending work.
  Local permissions are separate: optional challenge-response restores only
  grants already stored on that device; other-device/old-firmware failure never
  blocks account connection. Existing destination grants are preserved. New
  settings/drive/Tools access still uses the existing explicit permission flow.
- External links open the user's browser. The WebView has no native JavaScript
  bridge, no file/content access and no microphone permission. Voice invocation
  uses a 68dp draggable native microphone inside CarrotAssist. Dragging stores a
  relative position and tapping checks the selected WebView session through the
  authenticated relay inspect endpoint before opening Android’s default assistant.
  Transient errors get at most three checks, each limited to four seconds; leaving
  the activity cancels the operation. Unlinked accounts skip retries. Enable
  auto-start in device-name settings to greet once when opening a selected web,
  with a spoken notice if the device reports an available Git update. The greeting ends without a question. With Tools access, “Open Naver Map” requests the installed Android app.
  The manual button skips the greeting. Startup performs no maintenance actions. A successful
  check provides optional short-lived Android assist content containing only
  connection status. The voice provider controls spoken preambles. Account connection
  settings remain in the web Tools menu.
- Target SDK 36 with installed SDK tools. Android 17 requires a local-network
  runtime permission for target SDK 37+; raise the target only together with
  that permission flow. Do not request the new permission for this target.

## Build a local test APK (Windows)

Requires installed JDK, Android SDK platform 36 and build-tools 35.0.0. No Gradle,
third-party app libraries, downloads or SDK-license changes are needed.

```powershell
./build.ps1 -OutputApk C:/work/CarrotWeb-debug.apk -DebugKeystore C:/work/carrot-android-debug.jks
```

The script tests discovery policy and real HTTP probing, compiles Java/resources,
creates a local debug signing key if missing, aligns, signs and verifies the APK.
The debug key's standard password is not a production signing secret. Keep the
key outside the repository; use a protected release key for public distribution.
Physical Android 17/Samsung testing confirms discovery and automatic opening of
the actual Carrot device, and the native microphone opens ChatGPT's small voice
overlay while keeping the WebView visible. Native insets are consumed after the
host reserves them, avoiding duplicated bottom space in WebView. OAuth was
reauthorized for the app's separate web session; a fresh ChatGPT conversation
opened AutoBlinkerControl in that WebView without another code. Multiple-device
selection and the 60-second timeout have desktop policy checks; a second live
device and other phone/Android versions still need acceptance testing. The
MCP 0.5 additionally supports individually requested setting reads and confirmed
changes after both OAuth `carrot.settings` and local device permission. Dynamic
driving commands and log uploads remain unsupported.

On 2026-11-06, the owner's parked device was tested through their ChatGPT account:
actual ShowCustomBrightness 90, prepare 95, confirmed apply/readback 95, then
confirmed restoration/readback 90. The Android app was updated through its own
GitHub download/verifier/Android installer from version code 6 to 7; the existing
account connection and local setting permission survived without entering a code.
The test used ChatGPT text tool calls with the phone WebView active; voice overlay
navigation was tested separately. It is not a driving or multi-user load test.

## Signed app updates

The 0.2.2 recovery fixes a reproduced 0.2.1 omission: WebStorage.getOrigins()
can omit localStorage-only origins, and last_address may already hold the new IP.
The app now records its own bounded origin history. For pre-history upgrades it
also reads bounded address hints from its own WebView local-storage files (up to
8 MiB/32 files/16 local origins). This is a best-effort compatibility fallback,
not a LevelDB reader or authority: it never writes database files or extracts
credentials from them. Actual tokens are read through WebView and authenticated
by the relay. No outside browser storage, raw database data or tokens are logged.
Future discovery does not depend on legacy-file hints. The owner's physical phone
recovered its previous hotspot connection to the office IP and completed an MCP
screen-open request without reauthorization or a device update. No live driving
command or setting write was performed. Native UI follows the phone language
(Korean or English fallback); the Carrot Web language remains independent. Run `test-recovery.ps1` on a disposable emulator
with `-Serial emulator-N -Apk ... -DebugKeystore ...`; it overwrites only test
WebView storage and rejects physical-device serials. No live vehicle commands
are executed by this harness.

The app checks the CarrotAssist-specific update manifest on launch, at most once
per six hours after a successful check. Tap the top device name for app information
and a manual check. A new version is announced without interrupting the page.
Downloading and installation are user initiated; Android may require allowing
this app as an installation source and always retains its installer checks.
GitHub and its named HTTPS release-asset hosts are the only download destinations.
The APK must match the manifest SHA-256, package, increasing version code and
the installed app's signing certificate. Downloads are bounded to 32 MiB/60s.
Only that private cache file is shared with the system installer by temporary URI
permission. There is no general file provider or background silent installation.

Release builds use `-SigningKeystore`, `-SigningAlias` (default `carrotassist`),
and `-SigningPasswordEnv` (default `CARROT_ASSIST_SIGNING_PASSWORD`). Never commit
the key or password. Preserve the signing identity across releases to retain
WebView account pairing. The existing unique prototype identity was retained
and re-encrypted for releases; the APK does not enable Android debugging.

Publish immutable APK assets under `carrot-assist-vX.Y.Z`, then update
`update.json` on the separate `carrot-assist-updates` release only after verifying
the downloadable APK. Schema: `package`, `version`, `version_code`, `apk_url`,
`sha256`. The APK name is `CarrotAssist-X.Y.Z.apk`. Do not point this app at the
repository's general latest release, which also contains unrelated tools.

## Assistant compatibility

The Android invocation is provider-neutral. Actual assistant availability and
overlay behavior need per-device testing. Tool integration is separate:

- ChatGPT: existing Carrot OAuth MCP connection; fresh-chat navigation verified.
- Gemini: consumer custom MCP apps are documented, currently US/English/personal
  accounts with Keep Activity enabled. Mobile availability does not by itself
  establish Gemini Live tool support. Our OAuth callback allowlist currently
  accepts ChatGPT only; Gemini requires a separately verified client integration.
- Legacy Google Assistant: Conversational Actions were retired June 13, 2023.
  App Actions are a separate integration, not arbitrary conversational MCP access.

References: https://developer.android.com/reference/android/app/Activity#showAssist(android.os.Bundle)
https://developer.android.com/privacy-and-security/local-network-permission
https://support.google.com/gemini/answer/17209137
https://developers.google.com/assistant/ca-sunset
