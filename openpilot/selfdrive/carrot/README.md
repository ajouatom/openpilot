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
  cweb_push.py                     CWP push client (device IP / git update notify)
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
    -> on_startup background tasks (broker, hubs, heartbeat, git status, auto update, malloc trim)

browser
  -> GET / -> server/features/static.py injects window.__CARROT_BOOTSTRAP__
    -> web/index.html
      -> web/js/vendor/*
      -> web/js/translations/*
      -> web/js/shared/* + shared/ui/*
      -> web/js/pages/*
      -> web/js/realtime/*
      -> web/js/app.js
```

## Main Entry Points

| Path | Role |
|---|---|
| `carrot_server.py` | Starts aiohttp web server on port `7000`. Core affinity tunable via `CARROT_WEB_CORES`. |
| `server/app.py` | Backend composition root. Creates app, runtime hubs, background tasks, routes, static serving. |
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
    auto_update.py
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
    vision_diag.py
    vision_test.py
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
    vision_diag.py
    vision_test.py
    web_settings.py
    ws.py
    dashcam/
    screenrecord/
    tools/
  terminal_commands/
```

## Backend Core Files

| Path | Role |
|---|---|
| `server/app.py` | Creates `aiohttp.Application` (with request-log middleware + 16MB client max size), `ClientSession`, `RealtimeBroker(repo_flavor="c3")`, `CameraWsHub`, `RawWsHub`. Startup tasks: `heartbeat_loop` (when Params present), `git_status_loop`, `auto_update_loop`, periodic gc + `malloc_trim` loop. Holds `realtime_broker_poll_lock` to serialize SubMaster polls. Adds static `/` after explicit routes. |
| `server/config.py` | `WEB_DIR`/`CSS_DIR`/`JS_DIR`/`ASSETS_DIR`, settings path, state paths (`git.json`, `tool_jobs.json`, `web_settings.json`, `setting_favorites.json`, `setting_profiles.json`), dashcam/screenrecord paths, obfuscated Discord webhooks (dashcam + vision diag), `WEBRTCD_URL`, tmux session, params backup path. |
| `server/features/__init__.py` | Calls each feature module's `register(app)`. |
| `server/features/static.py` | `GET /` handler — injects `window.__CARROT_BOOTSTRAP__` (webSettings, deviceLanguage, deviceLanguages) into `index.html`, no-cache headers; serves `/training/` assets. |
| `server/live_runtime/broker.py` | Owns the realtime `SubMaster` broker used by `/api/live_runtime`. |
| `server/live_runtime/snapshot.py` | Builds live runtime snapshots for frontend polling. |
| `server/live_runtime/normalize.py` | Converts raw cereal/openpilot values into JSON-safe values. |
| `realtime/transports/camera_ws.py` | Camera frame WebSocket hub. |
| `realtime/transports/raw_ws.py` | Raw capnp WebSocket hub. |

## Backend Services

| Path | Used by | Role |
|---|---|---|
| `services/params.py` | `features/params.py`, tools backup/restore | Params read/write, bulk backup values (`HAS_PARAMS` guard). |
| `services/settings.py` | `features/settings.py`, static bootstrap | `carrot_settings.json` cache and parsing. |
| `services/git_state.py` | tools, auto update, metadata | Persisted git pull state such as last pull time. |
| `services/git_status.py` | `features/tools/routes.py`, auto update, startup loop | Cached git fetch/status info (ahead/behind) for update badges. |
| `services/auto_update.py` | `server/app.py` | Device-side auto `git pull` loop (when `auto_update_git_pull` web setting on); hard reset + pull, no reboot, best-effort CWP "what changed" notify. |
| `services/heartbeat.py` | `server/app.py`, `features/system.py` | Heartbeat/register loop and status. |
| `services/device_info.py` | `features/system.py` | Device network and calibration helpers. |
| `services/time_sync.py` | `features/system.py` | Browser-to-device time sync. |
| `services/tmux.py` | `features/terminal.py` | tmux session create, capture, input, clear. |
| `services/ssh_keys.py` | `features/ssh_keys.py` | SSH key fetch/store helpers. |
| `services/setting_favorites.py` | `features/setting_favorites.py` | Favorite setting names. |
| `services/setting_profiles.py` | `features/setting_profiles.py` | Setting profile CRUD/import/export/apply. |
| `services/web_settings.py` | `features/web_settings.py`, static bootstrap, auto update | Server-backed web settings (`web_settings.json`). |
| `services/vision_diag.py` | `features/vision_diag.py` | Server diagnostic snapshot (camerad/encoderd/stream proxy history) + Discord upload of diagnostic bundle. |
| `services/vision_test.py` | `features/vision_test.py`, `services/vision_diag.py` | Standalone camerad + stream encoderd test runner, status/log at `/tmp/carrot-vision-test*`. |

## Backend Feature Routes

| Feature | Main paths | Files |
|---|---|---|
| Static/bootstrap | `/`, `/training/*`, static fallback | `features/static.py` |
| WebRTC proxy | `POST /stream` | `features/stream.py` |
| Raw/camera WebSocket | `/ws/raw/{service}`, `/ws/raw_multiplex`, `/ws/camera/{camera}` | `features/ws.py`, `realtime/transports/*` |
| Settings | `GET /api/settings` | `features/settings.py`, `services/settings.py` |
| Params | `GET /api/params_bulk`, `POST /api/param_set`, `POST /api/params_restore`, `POST /api/params_restore_preview`, `POST /api/params_restore_json`, `GET/POST /api/params_qr_dependency(/ensure)`, `GET /api/params_qr_backup`, `GET /download/params_backup.json` | `features/params.py`, `services/params.py` |
| Favorites | `GET/POST /api/setting_favorites` | `features/setting_favorites.py` |
| Profiles | `GET/POST /api/setting_profiles`, `/update`, `/delete`, `/preview`, `/apply` | `features/setting_profiles.py` |
| Web settings | `GET/POST /api/web_settings` | `features/web_settings.py`, `services/web_settings.py` |
| SSH keys | `GET/POST /api/ssh_keys` | `features/ssh_keys.py`, `services/ssh_keys.py` |
| Cars | `GET /api/cars` | `features/cars.py` |
| System | `GET /api/heartbeat_status`, `/api/live_runtime`, `/api/device_network`, `/api/calibration_status`, `/api/regulatory`; `POST /api/reboot`, `/api/poweroff`, `/api/recalibrate`, `/api/set_default`, `/api/time_sync` | `features/system.py`, `services/device_info.py`, `services/heartbeat.py`, `services/time_sync.py` |
| Terminal | `GET /ws/terminal`, `GET /download/tmux.log` | `features/terminal.py`, `services/tmux.py` |
| Tools | `POST /api/tools`, `/api/tools/start`, `/api/tools/jobs/notice`; `GET /api/tools/job`, `/api/tools/jobs`, `/api/tools/git_status`; `DELETE /api/tools/jobs` | `features/tools/*`, `services/git_status.py` |
| Dashcam | `/api/dashcam/*` (routes, segments, thumbnail, preview, video, download, upload) | `features/dashcam/*` |
| Screenrecord | `/api/screenrecord/*` (videos, thumbnail, video, download) | `features/screenrecord/*` |
| Vision diag | `GET /api/vision_diag/server_snapshot`, `POST /api/vision_diag/upload_discord` | `features/vision_diag.py`, `services/vision_diag.py` |
| Vision test | `GET /api/vision_test/status` | `features/vision_test.py`, `services/vision_test.py` |

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
| `routes.py` | HTTP routes for sync actions, async jobs, job history/clear/notice, git status. |
| `jobs.py` | In-memory/persisted tool job state, log snapshots, async process helpers. |
| `dispatcher.py` | Actual action implementations: git pull/sync/reset, branch, logs, backup, reboot, install, shell command. |

## Terminal Commands Backend

```text
server/terminal_commands/
  README.md
  bridge.py
  cli.py
  registry.py
  custom_commands/
    help.py
    vision_test.py
```

Custom in-terminal commands surfaced through the tmux terminal (see `terminal_commands/README.md`). `vision_test.py` drives the camerad/encoderd test runner from the terminal.

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
| `__init__.py` | Re-exports raw protocol constants/builders and raw service helpers. |
| `raw_services.py` | Raw cereal service list definitions (core/optional, allowlist). |
| `raw_protocol.py` | Raw capnp packet/protocol helpers, hello builders, multiplex framing. |
| `raw_runner.py` | Async raw stream runner. |
| `transports/camera_ws.py` | Camera stream lifecycle and WebSocket broadcasting (`ws_camera`). |
| `transports/raw_ws.py` | Raw service and multiplex WebSocket broadcasting (service allowlist enforced). |

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
  components/
    nav_hud.css
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
| `components/nav_hud.css` | Carrot Nav HUD V2 top-center guidance card. |
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
| `shared/constants.js` | Global constants — lang storage key/emoji, unit cycle, page transition classes, debug flag. |
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
| `pages/vision_background.js` | Non-realtime page ambient canvas background. |

### Realtime JS

```text
web/js/realtime/
  app_realtime.js
  carrot_map.js
  home_drive.js
  hud_card.js
  nav_hud.js
  raw_capnp.js
  raw_capnp_worker.js
  vision_diag.js
  vision_raw.js
  vision_rtc.js
  vision_state.js
```

| Path | Scope |
|---|---|
| `realtime/app_realtime.js` | Live runtime polling/raw stream wiring and HUD bridge. |
| `realtime/home_drive.js` | Drive/Carrot Vision renderer and overlay canvas. |
| `realtime/hud_card.js` | HUD card data rendering. |
| `realtime/carrot_map.js` | Kakao minimap iframe bridge (kmap), `FRAME_VERSION` gated to limit SDK quota. |
| `realtime/nav_hud.js` | Carrot Nav HUD V2 — top-center guidance card from `carrotMan` state, no iframe/network. |
| `realtime/raw_capnp.js` | Raw capnp decode entry. |
| `realtime/raw_capnp_worker.js` | Worker-side decode entry. |
| `realtime/vision_state.js` | Shared vision/HUD state object. |
| `realtime/vision_rtc.js` | WebRTC stream client. |
| `realtime/vision_raw.js` | Raw WebSocket client and worker bridge. |
| `realtime/vision_diag.js` | Silent WebRTC/video diagnostic recorder (localStorage ring buffer) feeding the vision diag upload. |

### Translations JS

```text
web/js/translations/
  registry.js
  ko.js
  en.js
  zh.js
```

| Path | Scope |
|---|---|
| `translations/registry.js` | Merges per-language packs over an English/Korean fallback into the shared `CarrotTranslations` API. |
| `translations/{ko,en,zh}.js` | Per-language string/actionLabel/errorMessage/driveMode packs. |

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
| `carrotNavHud` | Nav HUD V2 top-center card host. |
| `rtcVideo` | Shared RTC video element. |
| `appToastHost` | Toast host. |
| `appDialog` | Generic app dialog. |
| `appBranchPicker` | Branch picker modal. |
| `appCarPicker` | Car picker modal. |

## Load Order Reference

CSS is loaded in `web/index.html` in this order:

```text
tokens.css
layout_tokens.css
hud_card.css
base.css
layout.css
components.css
pages/logs.css
pages/terminal.css
pages/settings/{base,panels,device}.css
pages/tools/{base,qr,main}.css
pages/drive.css
components/nav_hud.css
responsive.css
vendor/plyr.css
```

JavaScript is loaded in this order:

```text
vendor/{plyr.min,qrcode-generator,jsQR}.js
translations/{registry,ko,en,zh}.js
realtime/hud_card.js
shared/{constants,dom,utils,i18n,api,setting_diff,activity}.js
shared/ui/{focus_trap,dialog,viewport,effects,navigation}.js
pages/car.js
pages/setting*.js
pages/tools*.js
pages/branch.js
pages/logs/*.js
pages/terminal.js
realtime/{raw_capnp,vision_state,vision_rtc,vision_raw,app_realtime,vision_diag,carrot_map,nav_hud}.js
pages/vision_background.js
realtime/home_drive.js
app.js
```

Asset URLs carry `?v=` cache-busting query strings; bump the version when a file changes.

## Page Reference

| Page | HTML | JS | CSS | Backend |
|---|---|---|---|---|
| Drive/Home | `pageCarrot`, `driveHudCard`, `carrotNavHud`, `rtcVideo` | `realtime/*`, `vision_background.js` | `pages/drive.css`, `hud_card.css`, `components/nav_hud.css` | `features/system.py`, `features/ws.py`, `features/stream.py`, `features/vision_diag.py`, `realtime/transports/*` |
| Setting | `pageSetting` | `setting.js`, `setting_device*.js`, `car.js` | `pages/settings/*` | `features/settings.py`, `features/params.py`, `features/system.py`, `features/setting_*` |
| Tools | `pageTools` | `tools.js`, `tools_notifications.js`, `tools_web_settings.js`, `tools_settings_qr.js`, `branch.js` | `pages/tools/*` | `features/tools/*`, `features/system.py`, `features/params.py`, `features/web_settings.py` |
| Logs | `pageLogs` | `pages/logs/shared.js`, `dashcam.js`, `screenrecord.js` | `pages/logs.css` | `features/dashcam/*`, `features/screenrecord/*` |
| Terminal | `pageTerminal` | `terminal.js` | `pages/terminal.css` | `features/terminal.py`, `services/tmux.py`, `terminal_commands/*` |
| Car Select | `pageCar`, `appCarPicker` | `car.js` | `layout.css`, `components.css` | `features/cars.py`, Params |
| Branch Select | `pageBranch`, `appBranchPicker` | `branch.js` | `components.css`, `pages/tools/*` | `features/tools/*` |

## Feature Flow Reference

### Drive / Home

```text
openpilot messaging/cereal
  -> server/live_runtime/*
  -> features/system.py /api/live_runtime
  -> web/js/realtime/app_realtime.js
  -> web/js/realtime/hud_card.js + nav_hud.js
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

Vision diagnostics:

```text
web/js/realtime/vision_diag.js (records RTC/video stats)
  -> /api/vision_diag/server_snapshot (features/vision_diag.py)
  -> /api/vision_diag/upload_discord -> services/vision_diag.py
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

Auto update (no browser needed):

```text
server/app.py on_startup
  -> services/auto_update.py auto_update_loop
  -> git_status (behind?) -> hard reset + git pull
  -> cweb_push notify (best effort)
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
  -> services/tmux.py (+ terminal_commands/*)
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
| App wiring / background tasks | `server/app.py` |
| Route registration | `server/features/__init__.py` |
| Static/bootstrap payload | `server/features/static.py` |
| Backend paths/constants | `server/config.py` |
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
| Auto update loop | `server/services/auto_update.py` |
| Drive realtime UI | `web/js/realtime/*` |
| Nav HUD / minimap | `web/js/realtime/nav_hud.js`, `web/js/realtime/carrot_map.js` |
| Vision diagnostics | `web/js/realtime/vision_diag.js`, `server/features/vision_diag.py` |
| Drive backend data | `server/features/system.py`, `server/live_runtime/*` |
| Raw WebSocket | `server/features/ws.py`, `realtime/transports/raw_ws.py` |
| Camera WebSocket | `realtime/transports/camera_ws.py` |
| Logs UI | `web/js/pages/logs/*` |
| Dashcam backend | `server/features/dashcam/*` |
| Screenrecord backend | `server/features/screenrecord/*` |
| Terminal UI/backend | `web/js/pages/terminal.js`, `server/features/terminal.py`, `server/terminal_commands/*` |
| Translations | `web/js/translations/*` |
| Recovery server | `recovery/server.py` |
</content>
</invoke>
