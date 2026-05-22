# Carrot Web Reference

Structure reference for `selfdrive/carrot`.

## System Shape

```text
selfdrive/carrot/
  carrot_server.py                 main web server entry
  carrot_controls.py               carrot control helpers
  carrot_functions.py              carrot shared functions
  carrot_learning.py               carrot learning logic
  carrot_man.py                    carrot manager
  carrot_serv.py                   carrot service/runtime
  xiaoge_data.py                   car data
  server/                          aiohttp backend
  realtime/                        WebSocket realtime transports
  recovery/                        standalone recovery server
  web/                             static frontend
```

Runtime shape:

```text
carrot_server.py
  -> server.app.make_app()
    -> server.features.register_all(app)
    -> web/ static files

browser
  -> web/index.html
    -> web/js/shared/*
    -> web/js/pages/*
    -> web/js/realtime/*
    -> web/js/app.js
```

## Main Entry Points

| Path | Role |
|---|---|
| `carrot_server.py` | Starts aiohttp web server on port `7000`. |
| `server/app.py` | Backend composition root. Creates app, runtime hubs, background tasks, routes. |
| `server/features/__init__.py` | Registers backend feature modules. |
| `server/config.py` | Shared backend paths and constants. |
| `web/index.html` | Single HTML shell containing all page sections and script/style load order. |
| `web/js/app.js` | Final frontend bootstrap and browser history restore. |
| `recovery/server.py` | Standalone recovery server on port `6999`. |

## Backend Tree

```text
server/
  app.py
  config.py
  live_runtime/
    broker.py
    contract.py
    normalize.py
    runner.py
    services.py
    snapshot.py
  services/
    device_info.py
    git_state.py
    git_status.py
    heartbeat.py
    params.py
    setting_favorites.py
    setting_profiles.py
    settings.py
    ssh_keys.py
    time_sync.py
    tmux.py
    web_settings.py
  features/
    cars.py
    params.py
    setting_favorites.py
    setting_profiles.py
    settings.py
    ssh_keys.py
    static.py
    stream.py
    system.py
    terminal.py
    web_settings.py
    ws.py
    dashcam/
    screenrecord/
    tools/
```

## Backend Core Files

| Path | Role |
|---|---|
| `server/app.py` | Creates `aiohttp.Application`, `ClientSession`, `RealtimeBroker`, `CameraWsHub`, `RawWsHub`, heartbeat task, git status task, route registration, static serving. |
| `server/config.py` | `WEB_DIR`, settings paths, state paths, dashcam paths, WebRTC URL, tmux session, backup paths. |
| `server/features/__init__.py` | Calls each feature module's `register(app)`. |
| `server/live_runtime/broker.py` | Owns the realtime `SubMaster` broker used by `/api/live_runtime`. |
| `server/live_runtime/snapshot.py` | Builds live runtime snapshots for frontend polling. |
| `server/live_runtime/normalize.py` | Converts raw cereal/openpilot values into JSON-safe values. |
| `realtime/transports/camera_ws.py` | Camera frame WebSocket hub. |
| `realtime/transports/raw_ws.py` | Raw capnp WebSocket hub. |

## Backend Services

| Path | Used by | Role |
|---|---|---|
| `services/params.py` | `features/params.py`, tools backup/restore | Params read/write, bulk backup values. |
| `services/settings.py` | `features/settings.py`, static bootstrap | `carrot_settings.json` cache and parsing. |
| `services/git_state.py` | tools, metadata | Persisted git pull state such as last pull time. |
| `services/git_status.py` | `features/tools/routes.py`, startup loop | Cached git fetch/status info for update badges. |
| `services/heartbeat.py` | `server/app.py`, `features/system.py` | Heartbeat/register loop and status. |
| `services/device_info.py` | `features/system.py` | Device network and calibration helpers. |
| `services/time_sync.py` | `features/system.py` | Browser-to-device time sync. |
| `services/tmux.py` | `features/terminal.py` | tmux session create, capture, input, clear. |
| `services/ssh_keys.py` | `features/ssh_keys.py` | SSH key fetch/store helpers. |
| `services/setting_favorites.py` | `features/setting_favorites.py` | Favorite setting names. |
| `services/setting_profiles.py` | `features/setting_profiles.py` | Setting profile CRUD/import/export. |
| `services/web_settings.py` | `features/web_settings.py` | Server-backed web settings. |

## Backend Feature Routes

| Feature | Main paths | Files |
|---|---|---|
| Static/bootstrap | `/`, static fallback | `features/static.py` |
| WebRTC proxy | `/stream` | `features/stream.py` |
| Raw/camera WebSocket | `/ws/raw/{service}`, `/ws/raw_multiplex`, `/ws/camera/{camera}` | `features/ws.py`, `realtime/transports/*` |
| Settings | `/api/settings` | `features/settings.py`, `services/settings.py` |
| Params | `/api/params_bulk`, `/api/param_set`, restore/backup/download endpoints | `features/params.py`, `services/params.py` |
| Favorites | `/api/setting_favorites` | `features/setting_favorites.py` |
| Profiles | `/api/setting_profiles`, import/export endpoints | `features/setting_profiles.py` |
| Web settings | `/api/web_settings` | `features/web_settings.py`, `services/web_settings.py` |
| SSH keys | `/api/ssh_keys` | `features/ssh_keys.py`, `services/ssh_keys.py` |
| Cars | `/api/cars` | `features/cars.py` |
| System | heartbeat, live runtime, reboot, poweroff, recalibrate, network, calibration | `features/system.py`, `services/device_info.py`, `services/heartbeat.py` |
| Terminal | `/ws/terminal`, `/download/tmux.log` | `features/terminal.py`, `services/tmux.py` |
| Tools | `/api/tools`, `/api/tools/start`, `/api/tools/job`, `/api/tools/jobs`, `/api/tools/git_status` | `features/tools/*`, `services/git_status.py` |
| Dashcam | `/api/dashcam/*` | `features/dashcam/*` |
| Screenrecord | `/api/screenrecord/*` | `features/screenrecord/*` |

## Tools Backend

```text
server/features/tools/
  __init__.py
  actions.py
  dispatcher.py
  jobs.py
  routes.py
```

| File | Role |
|---|---|
| `actions.py` | Known tool action names and shell command allowlist. |
| `routes.py` | HTTP routes for sync actions, async jobs, job history, git status. |
| `jobs.py` | In-memory/persisted tool job state, log snapshots, async process helpers. |
| `dispatcher.py` | Actual action implementations: git pull/sync/reset, branch, logs, backup, reboot, install, shell command. |

## Dashcam Backend

```text
server/features/dashcam/
  __init__.py
  catalog.py
  ffmpeg.py
  paths.py
  routes.py
  upload.py
  upload_jobs.py
```

| File | Role |
|---|---|
| `paths.py` | Dashcam filesystem paths. |
| `catalog.py` | Route/segment listing and metadata. |
| `ffmpeg.py` | Thumbnail/preview/video processing helpers. |
| `routes.py` | HTTP route registration and handlers. |
| `upload.py` | Upload request handling. |
| `upload_jobs.py` | Upload job state and progress. |

## Screenrecord Backend

```text
server/features/screenrecord/
  __init__.py
  catalog.py
  routes.py
```

| File | Role |
|---|---|
| `catalog.py` | Screenrecord file listing and metadata. |
| `routes.py` | HTTP route registration and handlers. |

## Realtime Tree

```text
realtime/
  __init__.py
  raw_protocol.py
  raw_runner.py
  raw_services.py
  transports/
    __init__.py
    camera_ws.py
    raw_ws.py
```

| Path | Role |
|---|---|
| `raw_services.py` | Raw cereal service list definitions. |
| `raw_protocol.py` | Raw capnp packet/protocol helpers. |
| `raw_runner.py` | Async raw stream runner. |
| `transports/camera_ws.py` | Camera stream lifecycle and WebSocket broadcasting. |
| `transports/raw_ws.py` | Raw service and multiplex WebSocket broadcasting. |

## Frontend Tree

```text
web/
  index.html
  assets/
    img_chffr_wheel.png
    speed_bg.png
  css/
  js/
```

## Frontend CSS

```text
web/css/
  tokens.css
  layout_tokens.css
  hud_card.css
  base.css
  layout.css
  components.css
  responsive.css
  vendor/
    plyr.css
  pages/
    drive.css
    logs.css
    terminal.css
    settings/
      base.css
      panels.css
      device.css
    tools/
      base.css
      qr.css
      main.css
```

| Path | Scope |
|---|---|
| `tokens.css` | Color, typography, motion, elevation, z-index tokens. |
| `layout_tokens.css` | Layout sizing and spacing variables. |
| `hud_card.css` | Drive HUD card. |
| `base.css` | Base reset, nav, global app chrome. |
| `layout.css` | Page containers, headers, common layout blocks. |
| `components.css` | Shared dialogs, buttons, chips, toasts, generic components. |
| `responsive.css` | Cross-page responsive adjustments loaded near the end. |
| `pages/drive.css` | Drive stage, video, overlay canvas, vision controls. |
| `pages/logs.css` | Logs, dashcam, screenrecord views. |
| `pages/terminal.css` | Terminal page. |
| `pages/settings/*` | Setting page split by base/panels/device. |
| `pages/tools/*` | Tools page split by base/QR/main notification layout. |

## Frontend JavaScript

```text
web/js/
  app.js
  shared/
  pages/
  realtime/
  translations/
  vendor/
```

### Shared JS

```text
web/js/shared/
  activity.js
  api.js
  constants.js
  dom.js
  i18n.js
  setting_diff.js
  utils.js
  ui/
    dialog.js
    effects.js
    focus_trap.js
    navigation.js
    viewport.js
```

| Path | Scope |
|---|---|
| `shared/api.js` | JSON API helpers and Params helpers. |
| `shared/dom.js` | Shared DOM references from `index.html`. |
| `shared/i18n.js` | Language state and UI text rendering. |
| `shared/utils.js` | String/HTML/copy/math helpers. |
| `shared/activity.js` | Cross-page activity state and nav badges. |
| `shared/setting_diff.js` | Setting diff rendering helpers. |
| `shared/ui/dialog.js` | App dialogs, prompts, confirms, toasts. |
| `shared/ui/navigation.js` | Page switching, nav state, transitions. |
| `shared/ui/viewport.js` | Viewport variables and Drive HUD layout sync. |
| `shared/ui/focus_trap.js` | Focus trapping for overlays. |
| `shared/ui/effects.js` | Shared pointer/visual effects. |

### Page JS

```text
web/js/pages/
  branch.js
  car.js
  setting.js
  setting_device.js
  setting_device_actions.js
  setting_device_config.js
  setting_device_network.js
  setting_device_render.js
  tools.js
  tools_notifications.js
  tools_settings_qr.js
  tools_web_settings.js
  terminal.js
  vision_background.js
  logs/
    shared.js
    dashcam.js
    screenrecord.js
```

| Path | Scope |
|---|---|
| `pages/car.js` | Car picker, current car label, record state. |
| `pages/branch.js` | Branch page and branch picker modal. |
| `pages/setting.js` | CarrotPilot setting groups/items/search/favorites/profiles. |
| `pages/setting_device_config.js` | Device setting group definitions and option data. |
| `pages/setting_device_render.js` | Device setting row/panel rendering. |
| `pages/setting_device_network.js` | Device network refresh flow. |
| `pages/setting_device_actions.js` | Device setting action handlers. |
| `pages/setting_device.js` | Device tab coordinator and tab state. |
| `pages/tools.js` | Tools page state, metadata, action runner, button binding. |
| `pages/tools_notifications.js` | Tools log/notification card renderer. |
| `pages/tools_web_settings.js` | Web settings dialog and auto update setting. |
| `pages/tools_settings_qr.js` | Settings QR backup/restore UI. |
| `pages/logs/shared.js` | Logs tab state, player, lazy image helpers. |
| `pages/logs/dashcam.js` | Dashcam route/segment list, upload flow. |
| `pages/logs/screenrecord.js` | Screenrecord list and thumbnails. |
| `pages/terminal.js` | tmux WebSocket terminal UI. |
| `pages/vision_background.js` | Non-realtime page background state. |

### Realtime JS

```text
web/js/realtime/
  app_realtime.js
  home_drive.js
  hud_card.js
  raw_capnp.js
  raw_capnp_worker.js
  vision_raw.js
  vision_rtc.js
  vision_state.js
```

| Path | Scope |
|---|---|
| `realtime/app_realtime.js` | Live runtime polling/raw stream wiring and HUD bridge. |
| `realtime/home_drive.js` | Drive/Carrot Vision renderer and overlay canvas. |
| `realtime/hud_card.js` | HUD card data rendering. |
| `realtime/raw_capnp.js` | Raw capnp decode entry. |
| `realtime/raw_capnp_worker.js` | Worker-side decode entry. |
| `realtime/vision_state.js` | Shared vision/HUD state object. |
| `realtime/vision_rtc.js` | WebRTC stream client. |
| `realtime/vision_raw.js` | Raw WebSocket client and worker bridge. |

## HTML Page Sections

All primary pages live in `web/index.html`.

| Section ID | Page |
|---|---|
| `pageCarrot` | Drive/Home |
| `pageSetting` | Setting |
| `pageTools` | Tools |
| `pageLogs` | Logs |
| `pageTerminal` | Terminal |
| `pageCar` | Car Select |
| `pageBranch` | Branch Select |

Common overlay/dialog hosts in `index.html`:

| ID | Scope |
|---|---|
| `driveHudCard` | Shared Drive HUD card DOM. |
| `rtcVideo` | Shared RTC video element. |
| `appToastHost` | Toast host. |
| `appDialog` | Generic app dialog. |
| `appBranchPicker` | Branch picker modal. |
| `appCarPicker` | Car picker modal. |

## Load Order Reference

CSS is loaded in `web/index.html` in this shape:

```text
tokens.css
layout_tokens.css
hud_card.css
base.css
layout.css
components.css
page CSS
responsive.css
vendor CSS
```

JavaScript is loaded in this shape:

```text
vendor/*
translations/*
shared/*
shared/ui/*
pages/*
pages/logs/*
realtime/*
app.js
```

## Page Reference

| Page | HTML | JS | CSS | Backend |
|---|---|---|---|---|
| Drive/Home | `pageCarrot`, `driveHudCard`, `rtcVideo` | `realtime/*`, `vision_background.js` | `pages/drive.css`, `hud_card.css` | `features/system.py`, `features/ws.py`, `features/stream.py`, `realtime/transports/*` |
| Setting | `pageSetting` | `setting.js`, `setting_device*.js`, `car.js` | `pages/settings/*` | `features/settings.py`, `features/params.py`, `features/system.py`, `features/setting_*` |
| Tools | `pageTools` | `tools.js`, `tools_notifications.js`, `tools_web_settings.js`, `tools_settings_qr.js`, `branch.js` | `pages/tools/*` | `features/tools/*`, `features/system.py`, `features/params.py`, `features/web_settings.py` |
| Logs | `pageLogs` | `pages/logs/shared.js`, `dashcam.js`, `screenrecord.js` | `pages/logs.css` | `features/dashcam/*`, `features/screenrecord/*` |
| Terminal | `pageTerminal` | `terminal.js` | `pages/terminal.css` | `features/terminal.py`, `services/tmux.py` |
| Car Select | `pageCar`, `appCarPicker` | `car.js` | `layout.css`, `components.css` | `features/cars.py`, Params |
| Branch Select | `pageBranch`, `appBranchPicker` | `branch.js` | `components.css`, `pages/tools/*` | `features/tools/*` |

## Feature Flow Reference

### Drive / Home

```text
openpilot messaging/cereal
  -> server/live_runtime/*
  -> features/system.py /api/live_runtime
  -> web/js/realtime/app_realtime.js
  -> web/js/realtime/hud_card.js
  -> web/js/realtime/home_drive.js
```

Camera path:

```text
openpilot camera encode data
  -> realtime/transports/camera_ws.py
  -> features/ws.py /ws/camera/{camera}
  -> web/js/realtime/vision_rtc.js or vision_raw.js
  -> pageCarrot video/canvas
```

### Setting

```text
web/js/pages/setting.js
  -> /api/settings
  -> server/features/settings.py
  -> server/services/settings.py
```

Device tab:

```text
setting_device*.js
  -> /api/params_bulk, /api/param_set
  -> /api/device_network, /api/calibration_status
  -> features/params.py, features/system.py
```

### Tools

```text
web/js/pages/tools.js
  -> /api/tools/start
  -> features/tools/routes.py
  -> features/tools/jobs.py
  -> features/tools/dispatcher.py
```

Notification/log rendering:

```text
/api/tools/job or /api/tools/jobs
  -> tools.js
  -> tools_notifications.js
  -> css/pages/tools/main.css
```

Git status badge:

```text
services/git_status.py
  -> features/tools/routes.py /api/tools/git_status
  -> tools.js refreshGitPullStatus()
```

### Logs

```text
web/js/pages/logs/shared.js
  -> dashcam.js / screenrecord.js
  -> features/dashcam/routes.py
  -> features/screenrecord/routes.py
```

Dashcam upload:

```text
dashcam.js
  -> features/dashcam/upload.py
  -> features/dashcam/upload_jobs.py
```

### Terminal

```text
web/js/pages/terminal.js
  -> /ws/terminal
  -> features/terminal.py
  -> services/tmux.py
```

### Branch / Git

```text
web/js/pages/branch.js
  -> /api/tools/start or /api/tools
  -> features/tools/dispatcher.py
```

## Quick File Lookup

| Need | File |
|---|---|
| Server startup | `carrot_server.py` |
| App wiring | `server/app.py` |
| Route registration | `server/features/__init__.py` |
| Static/bootstrap payload | `server/features/static.py` |
| Page HTML | `web/index.html` |
| Page switching | `web/js/shared/ui/navigation.js` |
| Final frontend boot | `web/js/app.js` |
| API helpers | `web/js/shared/api.js` |
| Dialogs/toasts | `web/js/shared/ui/dialog.js` |
| Settings UI | `web/js/pages/setting.js` |
| Device settings UI | `web/js/pages/setting_device*.js` |
| Settings backend | `server/features/settings.py`, `server/features/params.py` |
| Tools UI | `web/js/pages/tools.js` |
| Tools notifications | `web/js/pages/tools_notifications.js` |
| Tools backend routes | `server/features/tools/routes.py` |
| Tools backend actions | `server/features/tools/dispatcher.py` |
| Tool job state | `server/features/tools/jobs.py` |
| Git status polling | `server/services/git_status.py` |
| Drive realtime UI | `web/js/realtime/*` |
| Drive backend data | `server/features/system.py`, `server/live_runtime/*` |
| Raw WebSocket | `server/features/ws.py`, `realtime/transports/raw_ws.py` |
| Camera WebSocket | `realtime/transports/camera_ws.py` |
| Logs UI | `web/js/pages/logs/*` |
| Dashcam backend | `server/features/dashcam/*` |
| Screenrecord backend | `server/features/screenrecord/*` |
| Terminal UI/backend | `web/js/pages/terminal.js`, `server/features/terminal.py` |
| Recovery server | `recovery/server.py` |
