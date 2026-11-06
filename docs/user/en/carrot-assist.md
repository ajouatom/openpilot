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
4. Select **Create mini-mode connection code**.
5. Open [ChatGPT Plugins](https://chatgpt.com/plugins) in a desktop browser with the same account. Select the add button and **Add custom MCP server**. Use `CarrotAssist` as its name, **OAuth** authentication and this server URL:

   `https://carrot-mcp.shind0.synology.me/account/mcp`

6. Enter the code on the Carrot authorization page. Select setting access as well if you want reads and changes, then approve. Existing screen-only connections need new authorization for the additional scope.
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

“Increase by 10%” adds ten percent of the current value: 50 becomes 55 before rounding to the setting's increment. “Add 10” adds ten units. If zero has a special meaning such as automatic mode, check the explanation and request an explicit value. Distinguish the Carrot display from the phone display.

Keep the CarrotAssist web interface active beneath the small voice overlay. A different fullscreen app or locked phone can suspend delivery until a request expires. A queued request is not a confirmed change.

## Supported scope

Voice changes support device-defined integer settings with valid bounds and increments. Except for screen brightness (`ShowCustomBrightness`), cluster brightness (`ClusterHudBrightness`) and alert volume (`SoundVolumeAdjust`), changes require fresh stationary/disengaged state. Restricted settings such as disabling driver monitoring are excluded.

This version does not execute real-time cruise-speed commands, lane changes, direct indicator control, drive analysis or log uploads. Editing a tuning parameter is different from issuing an immediate driving command. Setting history identifies changes made by the voice assistant. Follow each setting's instructions when a restart is required.

## Updates and disconnection

The app checks for updates on launch, at most once per six hours after a successful check. Tap the top device name for app information and a manual check. Download the new version and approve installation in Android. Updates with the same signing identity preserve connection data; do not uninstall first.

App updates, device Git updates and relay updates are separate. Updating the APK does not add device features to old device software.

Use **Revoke setting access** to revoke live value access alone, or **Disconnect** to revoke this web session's connection. Disconnect the plugin in ChatGPT as needed.

## Costs and other assistants

AI conversations use each user's own ChatGPT account and its usage limits. The relay does not call a model API, so user conversations do not consume the developer's ChatGPT or Codex allowance. Relay hosting and network costs remain separate. This connection does not upload voice recordings or driving logs.

The microphone invokes Android's default assistant. Gemini can be an invocation target, but Carrot MCP integration is not advertised as supported. Google's current custom-MCP documentation specifies US, English and personal accounts; custom MCP tools in Gemini Live remain unverified. Legacy Google Assistant App Actions require a separate integration and are not implemented here.

Official references: [ChatGPT custom MCP](https://developers.openai.com/plugins/deploy/connect-chatgpt), [Gemini custom apps](https://support.google.com/gemini/answer/17209137), [Gemini Live connected apps](https://support.google.com/gemini/answer/15274899).
