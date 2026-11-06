# CarrotAssist Android prototype

당근파일럿 AI 조수. Display name: CarrotAssist.

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
- App WebView storage is separate from Samsung Internet/Chrome. Each selected
  origin needs its own first OAuth device approval. Existing browser pairing is
  not imported or copied. One ChatGPT connection currently selects one approved
  web session; changing devices requires explicit reauthorization.
- External links open the user's browser. The WebView has no native JavaScript
  bridge, no file/content access and no microphone permission. Voice invocation
  is available only from the native toolbar microphone; the former floating web
  launcher and conversation-pairing UI have been removed. Account connection
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
existing MCP permits only search/explanation/screen navigation.

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
