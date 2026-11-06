# CarrotAssist guide

[한국어](../ko/carrot-assist.md) · [Carrot Web](carrot-web.md)

CarrotAssist discovers a Carrot device on the local network, opens its Carrot Web interface, and invokes the phone's default Android assistant. Connecting the Carrot plugin to ChatGPT enables setting search, explanations and screen navigation. Additional permission enables individual live setting reads and changes after confirmation of the before/after values.

## Prepare and install

1. While parked, update the device to current `carrot-wip` and restart it.
2. Connect the phone and device to the same trusted Wi-Fi or hotspot. Never expose the device's port 7000 to the Internet.
3. Install the APK from the [CarrotAssist release](https://github.com/ajouatom/openpilot/releases/tag/carrot-assist-v0.2.0). Review Android's installation-source permission if prompted.
4. Install the official ChatGPT app and sign in with your own account. Select ChatGPT as Android's default digital assistant. Settings names vary by phone.

The app targets Android 8 and newer. Device connection and the small voice overlay were tested on Samsung Android 17; other phones may behave differently. Your ChatGPT account must offer **Add custom MCP server**. Carrot connectivity and mini voice mode on Free accounts remain unverified. Paid accounts are also subject to their available features and policies.

## Connect once

1. Open CarrotAssist. One device found during the initial five-second window opens automatically; multiple devices require selection. With no result, scanning continues for about one minute. Enter a device's IPv4 address manually if necessary.
2. Open **Tools > ChatGPT assistant** in the web interface.
3. To use live values, select **Allow setting reads and changes** while stationary and disengaged. Requested current values and change results will be sent to ChatGPT and the Carrot relay.
4. Expand **First connection / reconnect**, then use **Copy Carrot connection address** and **Open ChatGPT connections**. If the add menu is unavailable on the phone, open [ChatGPT Plugins](https://chatgpt.com/plugins) in a desktop browser with the same account.
5. Select the add button and **Add custom MCP server**. Use `CarrotAssist` as its name and paste the copied address into the server URL field. Select **OAuth** from the authentication list and create/connect. OAuth is the connection method that lets you authorize access to your Carrot device from your own ChatGPT account.

   `https://carrot-mcp.shind0.synology.me/account/mcp`

6. When the Carrot authorization page opens, return to CarrotAssist and select **Create mini-mode connection code**, then enter that code on the authorization page. Select setting access as well if you want reads and changes, then approve. Existing screen-only connections need new authorization for the additional scope. If an existing plugin lacks new tools, select **Refresh tools** in ChatGPT's plugin management and test in a new chat.
7. Install the plugin and search for a Carrot setting in a new chat. Return to CarrotAssist and tap its top-right microphone to invoke the assistant.

The code's **five-minute expiry applies to initial authorization**, not the resulting connection. Codes are single-use; create another if expired. Enter codes only on the Carrot authorization page, never in a conversation. Once approved, new voice chats can use the same device without another connection request.

CarrotAssist has separate web storage from Chrome or Samsung Internet, so authorize the app once. Uninstalling, clearing app data or changing the device IP may require reconnection. One connection targets one explicitly approved web session.

## Example requests

| Purpose | Example |
|---|---|
| Find settings | “Use the Carrot tools to find turn-signal settings.” |
| Explain | “What does the first setting do?” |
| Open screen | “Open the following-gap level setting.” |
| Read value | “What is the Carrot device's current screen brightness?” |
| Propose change | “Change that brightness to 60.” |
| Confirm | After hearing the values: “Yes, apply that value.” |

Changes follow **read current value → propose → user confirms → verify application**. Catalog defaults are not treated as live device values. Bounds and increments are checked. Proposals expire after 60 seconds; a changed current value requires a new proposal and confirmation.

ChatGPT's plugin permissions may add a tool-approval prompt such as **Allow once**. If a proposal expires while waiting for approval, confirm a new proposal. Completely hands-free operation is not guaranteed in every account or conversation.

“Increase by 10%” adds ten percent of the current value: 50 becomes 55 before rounding to the setting's increment. “Add 10” adds ten units. If zero has a special meaning such as automatic mode, check the explanation and request an explicit value. Distinguish the Carrot display from the phone display.

Keep the CarrotAssist web interface active beneath the small voice overlay. A different fullscreen app or locked phone can suspend delivery until a request expires. A queued request is not a confirmed change.

## Supported scope

Voice changes support device-defined integer settings with valid bounds and increments. All supported settings, including driving settings, can be changed while driving after confirming the current and proposed values. Only initial authorization requires stationary/disengaged state. Each setting retains its existing activation timing; some require a restart. An explicit integer such as “set it to 240” is preserved within the allowed range. A button increment of 5 and minimum of 1 do not force 240 to become 241. Relative calculations are rounded to the nearest integer and the proposed value must be confirmed. Restricted settings such as disabling driver monitoring are excluded.

Setting history identifies changes made by the voice assistant. Follow each setting's instructions when a restart is required. Direct indicator control, drive analysis and log uploads remain unsupported.

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

Clear execution requests do not add a separate conversational confirmation. Saved tuning settings retain their value-confirmation procedure. ChatGPT may independently require tool approval; fully hands-free operation is not guaranteed for every account. Speech recognition, model reasoning and network latency remain, so this is not an emergency control interface.

Lane commands respect active assistance, speed, lane geometry, measured blind spots, existing steering-torque requirements and driver priority. Conflicting maneuvers and stale state can reject them. Speed commands require already-active, supported openpilot longitudinal control and never engage assistance. Configurations following the PCM set speed can reject them. An applied set speed does not mean the vehicle has reached that speed.

Relay delivery expires after five seconds; controller consumption requires an intent no older than one second. Pending lane preparation expires after three seconds. Only one driving request runs at once. During ten seconds after a request, identical calls return its result and other calls can be rejected to prevent duplicate maneuvers. Expired, failed or uncertain requests never retry automatically. An ambiguous “go slowly, there is a car ahead” needs a specific target; it does not trigger indefinite braking until radar detects a lead.

Running and completed are distinct. Lane completion requires both controller completion and observed lane crossing. **Revoke driving commands** prevents new commands and cancels pending preparation; it does not force an ongoing steering maneuver to abort. A phone voice-start action has not been added to the existing Bluetooth remote.

## Updates and disconnection

The app checks for updates on launch, at most once per six hours after a successful check. Tap the top device name for app information and a manual check. Download the new version and approve installation in Android. Updates with the same signing identity preserve connection data; do not uninstall first.

The first in-app update may open Android's **Install unknown apps > CarrotAssist > Allow permission** screen. After allowing it, return and select **Package installer > Just once > Update** if an app chooser appears. Samsung devices may show an additional notice for apps distributed outside their stores; verify that you are installing CarrotAssist from the official release above.

App updates, device Git updates and relay updates are separate. Updating the APK does not add device features to old device software.

Use **Revoke setting access** to revoke live value access alone, or **Disconnect** to revoke this web session's connection. Disconnect the plugin in ChatGPT as needed.

## Costs and other assistants

AI conversations use each user's own ChatGPT account and its usage limits. The relay does not call a model API, so user conversations do not consume the developer's ChatGPT or Codex allowance. Relay hosting and network costs remain separate. This connection does not upload voice recordings or driving logs.

The microphone invokes Android's default assistant. Gemini can be an invocation target, but Carrot MCP integration is not advertised as supported. Google's current custom-MCP documentation specifies US, English and personal accounts; custom MCP tools in Gemini Live remain unverified. Legacy Google Assistant App Actions require a separate integration and are not implemented here.

Official references: [ChatGPT custom MCP](https://developers.openai.com/plugins/deploy/connect-chatgpt), [Gemini custom apps](https://support.google.com/gemini/answer/17209137), [Gemini Live connected apps](https://support.google.com/gemini/answer/15274899).
