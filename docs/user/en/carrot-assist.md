# CarrotAssist guide

[한국어](../ko/carrot-assist.md) · [Carrot Web](carrot-web.md)

CarrotAssist discovers a Carrot device on the local network, opens its Carrot Web interface, and invokes the phone's default Android assistant. Connecting the Carrot plugin to ChatGPT enables setting search, explanations and screen navigation. Additional permission enables individual live setting reads and changes after confirmation of the before/after values.

## Prepare and install

1. While parked, update the device to current `carrot-wip` and restart it.
2. Connect the phone and device to the same trusted Wi-Fi or hotspot. Never expose the device's port 7000 to the Internet.
3. Install the APK from the [CarrotAssist release](https://github.com/ajouatom/openpilot/releases/tag/carrot-assist-v0.2.2). Review Android's installation-source permission if prompted.
4. Install the official ChatGPT app and sign in with your own account. Select ChatGPT as Android's default digital assistant. Settings names vary by phone.

The app targets Android 8 and newer. Device connection and the small voice overlay were tested on Samsung Android 17; other phones may behave differently. Your ChatGPT account must offer **Add custom MCP server**. Carrot connectivity and mini voice mode on Free accounts remain unverified. Paid accounts are also subject to their available features and policies.

## Connect once

**Copy the code in CarrotAssist. Paste it into the device authorization WEBPAGE opened by ChatGPT. Never send it in a voice or mini-mode conversation.**

### 1. Prepare in CarrotAssist

1. Open CarrotAssist. One device found in the initial five seconds opens automatically; choose if several appear. With no result, scanning continues for about one minute. A device IPv4 address can also be entered manually.
2. Open **Tools → ChatGPT assistant**. While parked and disengaged, allow the features you want. Settings, driving commands and Tools each have separate permission.
3. Expand **First connection / reconnect**. **Do not create a code yet.** Open the authorization page using the appropriate path below.

### 2-A. Already installed in ChatGPT

Select **Open ChatGPT Plugins → Installed → CarrotAssist → Reconnect**. An older connection may be called ‘당근파일럿 미니 음성’. Reconnect may appear under Connected accounts. Do not create another plugin. If only categories such as Popular or Productivity appear, select Installed first.

When you see a code field and **이 기기 연결 승인** (Authorize this device), continue to step 3.

### 2-B. First-time installation

1. Expand **First registration only: show instructions**, select **First registration: copy Carrot server address**, then **Open ChatGPT Plugins**. If the add menu is missing on the phone, use [ChatGPT Plugins](https://chatgpt.com/plugins) on a computer signed into the same account.
2. Select **+ → Add custom MCP server** and enter:

   | Field | Value |
   |---|---|
   | Name | `CarrotAssist` |
   | Server URL | `https://carrot-mcp.shind0.synology.me/account/mcp` |
   | Authentication | Select **OAuth** |

   OAuth is the authentication option in this form. You do not need to set up a server or buy an API key.
3. Create/connect. When the page with the code field and **이 기기 연결 승인** appears, continue below. Availability varies by account; support is not promised for accounts without custom MCP registration.

### 3. Put a fresh code into that page

1. Leave the authorization page open. Return to **CarrotAssist**, then select **After opening authorization: create a fresh code → Copy code for the authorization page**.
2. Return to **the authorization webpage you just opened**, paste into its code field, select the desired settings/driving/Tools permissions, and press **이 기기 연결 승인**. Permission is needed both here and in CarrotAssist.
3. Check that ChatGPT returns successfully and CarrotAssist shows **Account connected**. Complete plugin installation if this is the first registration. If new tools are missing, use **Refresh tools** in plugin management and try a new conversation.
4. Keep CarrotAssist visible, invoke voice, and try “Find one turn-signal setting with Carrot tools and open it.” Start by verifying search and screen navigation.

**Five minutes is the code-entry window, not the connection lifetime.** A successful connection is reused in new conversations. Creating another code immediately invalidates the previous code. An approved code cannot be used again.

| Screen or error | What to do |
|---|---|
| ChatGPT conversation or small voice overlay | This is not the code-entry page. Go to Plugins → Installed → CarrotAssist → Reconnect. |
| Google Play | This is app installation. Return to CarrotAssist and choose **Open ChatGPT Plugins**. |
| Invalid or expired code | Open a fresh authorization page through **Reconnect**, then create one fresh code in CarrotAssist and paste it immediately. If it still fails, report the error text, not the code. |
| Connected but device offline | Check the device/phone network and Internet access, and keep CarrotAssist visible. Do not start by making another code. |
| Permission missing | Check both the relevant permission button in CarrotAssist and the checkbox during ChatGPT reconnection. |

CarrotAssist, Chrome and Samsung Internet use separate storage. **Create the code in the CarrotAssist app you will use.** App deletion or clearing data can require reconnection. Menu labels can vary by account, language and app version.

### Switching between office Wi-Fi and a vehicle hotspot

With app **0.2.2** and relay **0.7.1**, ChatGPT follows **the Carrot Web currently opened in the app** after the initial authorization. A changed IP or a different selected Carrot device does not require another code or matching the previous device's identity. Choose the intended device when discovery finds several. Returning to the app triggers discovery if the previous address is unreachable.

Settings, driving and Tools actions still follow the permissions allowed in the current Carrot Web. Updated device software can restore existing local permissions after an IP change. Features not yet allowed on a different device need permission there. Failure to restore local permissions never blocks account connection or screen navigation. Pending commands from the old address are discarded.

Existing users can tap the top device name to check for an app update; do not uninstall. Update and restart the parked Carrot device as well for local permission restoration. Version 0.2.2 fixes a 0.2.1 case where the app could not find stored information for an older IP. On the owner’s phone, updating the app restored the existing hotspot-address connection to the office address and successfully opened a settings screen, without a fresh code or device GitPull. No driving command or setting write was performed.

## Using English

App 0.2.2 displays native search, connection and update messages in English when the phone language is not Korean. Carrot Web keeps its own language preference; select English there as well. Relay 0.7.2 displays the authorization page in English or Korean according to browser language and instructs the AI to reply in the user’s language.

For example: “Find the screen brightness setting and open it.” English setting search, the English consent page and the native English search screen have been tested. Actual spoken-English recognition and accent accuracy have not been separately verified. Parameter IDs are never translated. Speed commands use km/h; do not treat an mph value as km/h.

## Example requests

| Purpose | Example |
|---|---|
| Find settings | “Use the Carrot tools to find turn-signal settings.” |
| Explain | “What does the first setting do?” |
| Open screen | “Open the following-gap level setting.” |
| Read value | “What is the Carrot device's current screen brightness?” |
| Change value | “Change that brightness to 60.” → “Changed to 60” after verified storage. |

Relay **0.7.3** treats a clear command specifying the setting and value or adjustment as authorization. It internally **checks the current value and bounds → saves → verifies readback**, without asking the same question again. “Set acceleration setting 0 to 240” applies 240 when valid and gives a short result. Ambiguous requests or AI-recommended values still need clarification. Out-of-range requests are rejected with the allowed range.

Internal preparations expire after 60 seconds; changed values or definitions prevent saving. Failed, expired or unverified requests are not automatically replayed. ChatGPT may independently show a tool approval such as **Allow once**. That is not a request for a Carrot connection code or new permission.

Reads do not need a change confirmation. Updated Carrot Web refreshes local permission from storage for each new request and does not block commands while a read-only code field has focus. Active editing blocks changes, but not reads. GitPull the device and reopen Carrot Web for these web fixes; keep phone APK 0.2.2.

“Increase by 10%” adds ten percent of the current value: 50 becomes 55; fractional calculated results are rounded to the nearest integer. “Add 10” adds ten units. If zero has a special meaning such as automatic mode, check the explanation and request an explicit value. Distinguish the Carrot display from the phone display.

Keep the CarrotAssist web interface active beneath the small voice overlay. A different fullscreen app or locked phone can suspend delivery until a request expires. A queued request is not a confirmed change.

## Supported scope

Voice changes support device-defined integer settings with valid bounds and increments. All supported settings, including driving settings, can be changed while driving on a clear change command. Only initial authorization requires stationary/disengaged state. Each setting retains its existing activation timing; some require a restart. An explicit integer such as “set it to 240” is preserved within the allowed range. A button increment of 5 and minimum of 1 do not force 240 to become 241. Relative calculations are rounded to the nearest integer without an extra confirmation question for a clear command. Restricted settings such as disabling driver monitoring are excluded.

Setting history identifies changes made by the voice assistant. Follow each setting's instructions when a restart is required. Direct indicator control, drive analysis and driving-record uploads remain unsupported. Separate Tools permission enables tmux diagnostic upload requests.

## Driving commands in the same conversation — experimental

Requires relay 0.6.0 and its matching device code. Keep the existing Android 0.2.0 APK. GitPull and fully restart the device while parked and disengaged; restarting only the web server does not reload the vehicle controllers. Controller and connection tests run on a PC; spoken on-road operation has not been validated.

1. In CarrotAssist, select **Tools > ChatGPT assistant > Allow voice lane-change and set-speed commands** while parked and disengaged.
2. Reauthorize the ChatGPT CarrotAssist connection with the new scope and select driving commands on the Carrot consent page. Settings access alone never grants driving access. If a code is needed, create a fresh one in the app and enter it on that consent page.
3. Start voice once with the existing microphone button. Discuss settings, then ask for a lane change within the same conversation. No button press or mode switch is needed for each utterance. Keep Carrot Web visible under the small voice overlay.

| Request | Behavior |
|---|---|
| “Change one lane left/right” | Request one lane through the existing lane-change controller |
| “Increase speed” | Same set-speed handling as one remote RES/+ press |
| “Reduce speed by 10” | Subtract 10 km/h from the current set speed |
| “Reduce speed” | Subtract 10 km/h, matching the legacy voice command |
| “Set speed to 100” | Set the cruise target to 100 km/h |

Clear execution requests do not add a separate conversational confirmation. Clear target/value commands for saved settings also apply after internal validation without another question. ChatGPT may independently require tool approval; fully hands-free operation is not guaranteed for every account. Speech recognition, model reasoning and network latency remain, so this is not an emergency control interface.

Lane commands respect active assistance, speed, lane geometry, measured blind spots, existing steering-torque requirements and driver priority. Conflicting maneuvers and stale state can reject them. Speed commands require already-active, supported openpilot longitudinal control and never engage assistance. Configurations following the PCM set speed can reject them. An applied set speed does not mean the vehicle has reached that speed.

Relay delivery expires after five seconds; controller consumption requires an intent no older than one second. Pending lane preparation expires after three seconds. Only one driving request runs at once. During ten seconds after a request, identical calls return its result and other calls can be rejected to prevent duplicate maneuvers. Expired, failed or uncertain requests never retry automatically. An ambiguous “go slowly, there is a car ahead” needs a specific target; it does not trigger indefinite braking until radar detects a lead.

Running and completed are distinct. Lane completion requires both controller completion and observed lane crossing. **Revoke driving commands** prevents new commands and cancels pending preparation; it does not force an ongoing steering maneuver to abort. A phone voice-start action has not been added to the existing Bluetooth remote.

## Use Tools buttons by voice

This requires relay 0.7.0 and matching device code. Android 0.2.0 remains compatible. GitPull and restart the device, then reload Carrot Web. Select **Tools > ChatGPT assistant > Allow Tools panels and actions**, then follow the existing-installation reconnection steps above and select **도구 화면·작업 실행 허용** (Tools access) on the authorization page. Existing settings/driving grants do not expand automatically.

| Example | Behavior |
|---|---|
| “Open Bluetooth settings”, “Show device info” | Opens the panel. Car selection, language, web settings, branches, Git history, setting display and QR backup are also supported. |
| “Run GitPull” | Explains the effect and starts the existing job after confirmation. Reboot is a separate request. |
| “Reboot the device”, “Rebuild everything” | Requests the operation after confirmation. Rebuild and calibration reset include reboot. |
| “Back up settings” | Creates a backup on the device after confirmation. Download it from Tools on the phone. |
| “Send the tmux diagnostic log to the server” | Confirms the transfer and requests upload through the existing Carrot upload service. This is not a driving-record upload. |
| “Save the diagnostic log” | Captures tmux to a local device file without server upload. |
| “Start/stop screen recording” | Confirms and sets the requested recording state. Does not claim a recording file has been created. |
| “Delete all screen recordings”, “Delete all driving logs” | Confirms the full deletion scope before starting. Deletion cannot be undone. |
| “Open settings restore” | Shows the corresponding button. File selection, QR scanning, clipboard copying and repository reset require direct phone interaction. |

Git Sync and hard/mixed/soft reset to HEAD are also available after confirmation. No arbitrary shell command, URL, file path, commit or branch is accepted by voice execution. Make branch/Git-history choices on the opened screen. The eGPU panel can be shown, but voice does not automatically change a model or restart compilation.

Execution other than screen recording requires a fresh parked/disengaged state during preparation and dispatch. **Prepared means not executed; running means not complete.** Reboot, rebuild, calibration reset and diagnostic transfer report request acceptance, not verified reboot/upload completion. Unknown results and disconnected sessions are never automatically retried. See the Tools job history for details. Revoking permission blocks new requests and uncommitted proposals; it does not stop a job already running.

ChatGPT receives only the action and a limited result, not diagnostic contents or backup files. Diagnostic upload may include vehicle identifiers and diagnostics on the existing upload server. Actual vehicle deletion, reboot and upload have not been exercised as acceptance tests.

## Updates and disconnection

The app checks for updates on launch, at most once per six hours after a successful check. Tap the top device name for app information and a manual check. Download the new version and approve installation in Android. Updates with the same signing identity preserve connection data; do not uninstall first.

The first in-app update may open Android's **Install unknown apps > CarrotAssist > Allow permission** screen. After allowing it, return and select **Package installer > Just once > Update** if an app chooser appears. Samsung devices may show an additional notice for apps distributed outside their stores; verify that you are installing CarrotAssist from the official release above.

App updates, device Git updates and relay updates are separate. Updating the APK does not add device features to old device software.

Use **Revoke setting access** to revoke live value access alone, or **Disconnect** to revoke this web session's connection. Disconnect the plugin in ChatGPT as needed.

## Costs and other assistants

AI conversations use each user's own ChatGPT account and its usage limits. The relay does not call a model API, so user conversations do not consume the developer's ChatGPT or Codex allowance. Relay hosting and network costs remain separate. This connection does not upload voice recordings or driving logs.

The microphone invokes Android's default assistant. Gemini can be an invocation target, but Carrot MCP integration is not advertised as supported. Google's current custom-MCP documentation specifies US, English and personal accounts; custom MCP tools in Gemini Live remain unverified. Legacy Google Assistant App Actions require a separate integration and are not implemented here.

Official references: [ChatGPT custom MCP](https://developers.openai.com/plugins/deploy/connect-chatgpt), [Gemini custom apps](https://support.google.com/gemini/answer/17209137), [Gemini Live connected apps](https://support.google.com/gemini/answer/15274899).
