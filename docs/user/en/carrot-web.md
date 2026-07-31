# Carrot Web User Guide

[한국어](../ko/carrot-web.md)

Carrot Web is a local web interface for viewing and managing carrotpilot from a phone, tablet, or PC connected to the same network as the device. It provides driving status, settings, logs, updates, and diagnostic tools.

> [!WARNING]
> Do not operate Carrot Web while driving. Change settings, select logs, enter terminal commands, and run tools only after parking safely.

## Connecting

1. Connect the carrotpilot device and your browser device to the same Wi-Fi network or hotspot.
2. Find the carrotpilot device IP address.
3. Open `http://device-IP:7000` in the browser.

Example: `http://192.168.0.25:7000`

Select the white carrot icon on the device to show a large QR code for its current address. On C3, use the bottom-left button; on C4, use the bottom status-icon row. Scan it with a phone on the same network to connect. If the device IP changes while the QR screen is open, both the QR code and displayed address update automatically. Long addresses scale to fit instead of being shortened. The last QR refresh time appears below the address as numeric `HH:MM:SS`, with the 30-second auto-close countdown on the right. Tap the QR screen to close it, or leave it open and it closes when the countdown reaches zero.

Carrot Web is a local device-management interface. Do not expose it directly to the internet or give its address, a remote-support link, or terminal access to an untrusted person.

## Pages at a glance

Navigation may appear as a left rail on a wide display or a bottom bar on a portrait phone.

| Page | Main purpose |
|---|---|
| Drive | Live road video, HUD and driving data, screen-recording control, and recorded-log replay |
| Settings | carrotpilot and device settings, search, favorites, profiles, and change history |
| Tools | Vehicle and language selection, web layout, Git management, backup/restore, and system actions |
| Logs | Dashcam routes, playback, analysis and upload, plus screen-recording playback and download |
| Terminal | Device terminal and time-limited remote-support sessions |

Some controls or content may be absent depending on device type, driving state, branch, and browser capabilities.

## Drive page

The Drive page displays live video and driving state. Its usual layout contains road video and an auxiliary information area. Use `Tools > Web Settings` to adjust split direction, visible areas, and content.

### Live video and HUD

- Select `Start Drive Vision` to begin connecting to the camera stream.
- The HUD may show vehicle speed, set speed, speed limit, following gap, lead vehicle, and driving alerts.
- Enable or disable AR in `Tools > Web Settings`.
- If another browser owns the video stream, the page may say that it is in use elsewhere. Select `Use here` only when you intend to move the stream to the current browser.
- A parked device or unavailable camera/driving data produces a waiting state instead of live video.

The web video and HUD are reference displays. Always prioritize the vehicle's actual warnings and the road ahead.

### AR display and diagnostics

- Enabling **Show AR** in `Tools > Web Settings` overlays Carrot Navi driving guidance on the Carrot Vision video and requests additional real-time pose and position data only while it is enabled.
- **AR debug** shows sign, anchor, and draw counts plus the current blocking reason, and can copy or save its diagnostic history.
- The replay event timeline distinguishes recorded Carrot Navi connection changes, current and next maneuvers, lane guidance, road-safety alerts, average-speed zones, traffic signals, and intersection guidance.

### Layout

`Tools > Web Settings > Layout` controls:

- horizontal or vertical split;
- split view, Area 1 full screen, or Area 2 full screen;
- road video or another supported content type in each area; and
- swapping the two areas.

Unavailable content reports waiting, recovering, unavailable, or unsupported. Changing the web layout does not change vehicle-control behavior.

### Starting and stopping a screen recording

The `Record` button on the Drive page toggles the device `ScreenRecord` state.

1. Select `Record` to start.
2. Confirm that a `REC` badge appears on the Drive navigation button.
3. Select `Record` again after capturing the required behavior.
4. Open `Logs > Screen Record` to play or download the saved file.

A screen recording helps explain what was visible, but it does not replace the raw driving log required for vehicle-control analysis. Use [Sending Dashcam Logs for Analysis](dashcam-log-sharing.md) for abnormal driving behavior.

### Replaying a recorded drive

From `Logs > Dashcam`, open a segment menu and select `Replay` to display the saved road video with recorded driving data on the Drive page. Previous/next segment navigation, playback speed, HUD, events, graphs, sensor views, and raw-log analysis depend on the data stored in that segment and browser support.

Recorded replay may stop automatically when real driving begins.

## Settings page

The Settings page is divided into `Device` and `carrotpilot` tabs.

- Select a group, then an item, to change its value.
- Search by setting name or description.
- Long-press a setting to add or remove it from Favorites.
- Profiles save a group of values and allow selected values to be applied later.
- A setting detail page can show popular values, recent change history, and a verified Wiki guide.
- Risky settings display a warning, and some settings take effect only after reboot.
- Resetting defaults changes many values; make a backup first.

See [Understanding Settings](settings.md) for ranges, units, and a safe tuning order.

## Tools page

Tools contains management actions that can immediately change device or repository state.

### Quick actions

- `Car Select`: choose the vehicle maker and model.
- `Language`: change the Carrot Web language.
- `Web Settings`: configure the Drive layout, video, HUD, AR, and other web-only display options.
- `Info`: view and copy the device type, branch, commit, identifiers, and network information.

### Git Commands

- `git pull`: fetch and integrate new commits for the current branch.
- `git sync`: synchronize with the remote branch; local work may be cleaned up.
- `git reset`: move or reset code changes to a selected reference.
- `git log`: inspect commits and choose a commit or branch.
- `change branch`: choose a local or remote branch.
- `reset repo`: forcibly reset repository remotes and branch state.

> [!CAUTION]
> `git sync`, `git reset`, and `reset repo` can discard local code changes. If you do not understand the action, do not run it. For a support request, read the current branch and commit from `Info` instead.

### User / System

- `Reset Calib`: clear camera calibration and start calibration again.
- `capture tmux`: capture the current tmux log and download it through the browser.
- `send tmux`: request a server-log upload.

CAN diagnostics are uploaded automatically only when a currently received `carState` or `radarState` from the present onroad session reports a real CAN error. A timeout left over from the previous drive is not used, and capture is delayed for five seconds after detection so the log includes the immediate aftermath. In an automatic diagnostic log, `CarrotException can_error queued` means the upload was queued; the preceding `current onroad CAN error detected from ...` line identifies the source used for the decision.
- `install required`: check and install packages used by optional Carrot Web features.
- `delete all videos`: delete screen recordings.
- `delete all logs`: delete stored driving logs.
- `Rebuild All`: rebuild and restart the software.
- `reboot`: reboot the device.

After deleting logs or videos, Carrot Web cannot replay or upload them for analysis. Complete any support upload first.

### Settings backup and restore

The page provides file backup/restore, QR backup/restore, setting-value copy, and a full value view. Compare current and backup values before restoring. A backup from another vehicle or an older branch can have different meaning or recommended ranges even when parameter names match.

### System command field

The command field at the bottom of Tools accepts only server-allowlisted Git management or inspection commands and diagnostics such as `df`, `free`, and `uptime`. Output appears in the panel below it. Prefer the prepared buttons for routine device operations.

## Logs page

Logs has `Dashcam` and `Screen Record` tabs.

### Dashcam

- View a card and time range for each drive.
- Expand a card to see segments of about one minute each.
- Select a segment to play its road video.
- The segment menu offers recorded-drive replay, log upload, and `qcamera`, `rlog`, or `qlog` download.
- `Drive Report` summarizes assisted/manual driving time, distance, speed, hard acceleration/deceleration, interventions, and warnings.
- Check segments and use `Upload selected`, Select all, or range selection for a multi-segment upload.
- The top-right Logs menu provides sorting and quick upload of the newest 2, 5, or 10 completed logs.

See [Sending Dashcam Logs for Analysis](dashcam-log-sharing.md) for the complete problem-selection and upload procedure.

### Screen Record

The newest files appear first with name, saved time, and size. Select a row to play it or use the download button on the right to save the original. This tab displays existing recordings; start and stop recording with `Record` on the Drive page.

## Terminal page

Terminal connects to the device's `carrot-terminal` tmux session. It provides reconnect, clear display, `Ctrl+C`, arrows, Tab, and other mobile-friendly helper keys.

Terminal commands run immediately on the device and can have a much wider impact than prepared Tools actions. Do not enter a command you do not understand. If disconnected, try `Reconnect` first. `Clear` only clears the visible terminal display; it does not delete device files.

### Remote support

`Support` creates a time-limited terminal session for an external helper.

1. Enter a short symptom note.
2. Choose only the required duration—normally 15 minutes, 30 minutes, or one hour.
3. Prefer `Approve each` for command permission.
4. Send the support link and PIN only to the designated helper.
5. Read every command shown on the device and approve only what is required.
6. Select `Stop` as soon as support is complete.

Use `Allow all` or an unlimited session only if you understand the risk and will monitor the entire session. Never post the link or PIN in a public channel.

## Connection and display troubleshooting

1. Confirm that the browser and device are on the same network.
2. Check whether the device IP address changed.
3. Confirm the `http://device-IP:7000` address format.
4. Check whether a public Wi-Fi network blocks traffic between clients.
5. Refresh the page. If needed, reboot the device and wait for its services to start.
6. Include device type, vehicle, branch, commit, occurrence time, and exact error text in a support request.

If only video is missing, check whether another browser owns the stream, whether the device is ready to provide camera/driving data, and whether the browser supports the required video codec and WebRTC.

## Related guides

- [Sending Dashcam Logs for Analysis](dashcam-log-sharing.md)
- [Understanding Settings](settings.md)
- [Buttons and Presets](buttons-presets.md)
- [Speed and Deceleration](speed-deceleration.md)
- [Cruise and Following Gap](cruise-gap.md)
- [Radar Tracks and Corner Radar](radar.md)
