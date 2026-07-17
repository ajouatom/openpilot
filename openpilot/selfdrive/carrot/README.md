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
| `web/js/intro/intro.js` | First-run intro shell. Gated in `bootstrapWebStartPage()`; server decides via bootstrap payload. |
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
    support_terminal.py
    terminal_pty.py
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
| `server/live_runtime/broker.py` | Owns the route metadata plus server-side engagement safety `SubMaster` used by `/api/live_runtime`; HUD/overlay delivery stays on the compact relay. |
| `server/live_runtime/snapshot.py` | Builds live runtime snapshots for frontend polling. |
| `server/live_runtime/normalize.py` | Converts raw cereal/openpilot values into JSON-safe values. |
| `realtime/transports/camera_ws.py` | Camera frame WebSocket hub. |
| `realtime/compact_state.py` | Display-field serializer used by the client-rendered Vision overlay. |
| `realtime/transports/raw_ws.py` | Shared single-poll cereal relay for the main compact state WebSocket and legacy diagnostic raw endpoints. |

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
| `services/terminal_pty.py` | `features/terminal.py`, `services/support_terminal.py` | Persistent login-shell PTY shared by the local xterm and authenticated support viewers. Owns raw output broadcast, history replay, input, and owner-controlled geometry. |
| `services/support_terminal.py` | `features/support_terminal.py` | PIN/TTL/tunnel lifecycle, owner approval, single-controller arbitration, and the restricted support guest page/assets. |
| `services/ssh_keys.py` | `features/ssh_keys.py` | SSH key fetch/store helpers. |
| `services/setting_favorites.py` | `features/setting_favorites.py` | Favorite setting names. |
| `services/setting_profiles.py` | `features/setting_profiles.py` | Setting profile CRUD/import/export/apply. |
| `services/web_settings.py` | `features/web_settings.py`, static bootstrap, auto update | Server-backed web settings (`web_settings.json`). |
| `services/vision_diag.py` | `features/vision_diag.py` | Server diagnostic snapshot (camerad/encoderd/stream proxy history) + Discord upload of diagnostic bundle. |
| `services/vision_test.py` | `features/vision_test.py`, `services/vision_diag.py` | Standalone camerad + stream encoderd test runner, status/log at `/tmp/carrot-vision-test*`. |

## Backend Feature Routes

| Feature | Main paths | Files |
|---|---|---|
| Static/bootstrap | `/`, `/training/*`, `/sound-assets/*`, static fallback | `features/static.py` |
| Intro (first run) | `GET /api/intro/state`; `POST /api/intro/complete`, `/api/intro/reset`, `/api/intro/apply_preset` | `features/intro/*` |
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
| Terminal | `GET /ws/terminal`, `/ws/terminal_pty`, `/api/terminal_pty/status`, `/download/tmux.log` | `features/terminal.py`, `services/terminal_pty.py`, `services/tmux.py` |
| Support terminal | `POST /api/support_terminal/start`, `/stop`, command approve/reject; `GET /api/support_terminal/status`, `/support/terminal/{session_id}`, `/support-terminal-assets/{asset_name}`; WS owner/guest endpoints | `features/support_terminal.py`, `services/support_terminal.py`, `services/terminal_pty.py` |
| Tools | `POST /api/tools`, `/api/tools/start`, `/api/tools/jobs/notice`; `GET /api/tools/job`, `/api/tools/jobs`, `/api/tools/git_status`; `DELETE /api/tools/jobs` | `features/tools/*`, `services/git_status.py` |
| Dashcam | `/api/dashcam/*` (routes, segments, thumbnail, preview, raw replay source, video, download, upload) | `features/dashcam/*` |
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
| `catalog.py` | Route/segment listing, metadata, and untouched rlog/video source selection. |
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
  compact_state.py
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
| `compact_state.py` | Packs only overlay fields required by the browser; it does not project or render graphics. |
| `raw_services.py` | Raw cereal service list definitions (core/optional, allowlist). |
| `raw_protocol.py` | Raw capnp packet/protocol helpers, hello builders, multiplex framing. |
| `raw_runner.py` | Async raw stream runner. |
| `transports/camera_ws.py` | Camera stream lifecycle and WebSocket broadcasting (`ws_camera`). |
| `transports/raw_ws.py` | One shared cereal poll loop with per-client latest-only queues; the main page uses one compact HUD/overlay connection without per-viewer subscriptions. |

## Frontend Tree

```text
web/
  index.html
  assets/
    img_chffr_wheel.png
    speed_bg.png
  support_terminal/
    guest.html
    guest.css
    guest.js
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
      drive_layout.css
```

| Path | Scope |
|---|---|
| `tokens.css` | Color, typography, motion, elevation, z-index tokens. |
| `layout_tokens.css` | Layout sizing and spacing variables. |
| `hud_card.css` | Drive HUD card. |
| `base.css` | Base reset, nav, global app chrome. |
| `layout.css` | Page containers, headers, common layout blocks. |
| `components.css` | Shared dialogs, buttons, chips, toasts, generic components. |
| `components/drive_content_intro.css` | Shared Drive Workspace idle/disabled content surface. |
| `responsive.css` | Cross-page responsive adjustments loaded near the end. |
| `pages/drive.css` | Drive stage, video, overlay canvas, vision controls. |
| `pages/logs.css` | Logs, dashcam, screenrecord views. |
| `pages/terminal.css` | Terminal page. |
| `pages/settings/*` | Setting page split by base/panels/device. |
| `pages/tools/*` | Tools page split by base/QR/main and drive-layout preferences. |

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
  drive_layout_spec.js
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
| `shared/drive_layout_spec.js` | Neutral Area 1/Area 2 layout modes, per-area starting content, orientation-specific setting keys, and normalization shared by settings and Drive Workspace. |
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
  tools_web_settings_schema.js
  tools_web_settings_state.js
  tools_web_settings_render.js
  tools_web_settings.js
  web_settings/
    components.js
    drive_layout.js
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
| `pages/tools_web_settings_schema.js` | Web settings UI schema (groups/rows/control widgets); value types/defaults come from the backend spec. |
| `pages/tools_web_settings_state.js` | Web settings state + API; ingests `bootstrap.webSettingsSpec`, exposes `CarrotWebSettingsState` and the public getters/setters + start-page. |
| `pages/tools_web_settings_render.js` | Web settings dialog row/control rendering and active-panel sync. |
| `pages/tools_web_settings.js` | Web settings dialog open/bind controller. |
| `pages/web_settings/components.js` | Registry for compound, multi-key web preference components. |
| `pages/web_settings/drive_layout.js` | Interactive landscape/portrait Area 1/Area 2 layout preview, per-area starting-content selectors, and setting bindings. |
| `pages/tools_settings_qr.js` | Settings QR backup/restore UI. |
| `pages/logs/shared.js` | Logs tab state, player, lazy image helpers. |
| `pages/logs/dashcam.js` | Dashcam route/segment list, upload flow. |
| `pages/logs/screenrecord.js` | Screenrecord list and thumbnails. |
| `pages/terminal.js` | Local xterm client for the persistent login-shell PTY, with fixed 100-column geometry and touch key controls. |
| `pages/support_terminal.js` | Local owner controls for starting/stopping support, guest presence, and command approval overlays. |
| `support_terminal/guest.js` | Standalone authenticated xterm guest client. Approval mode is read-only plus command requests; allow-all mode grants raw PTY input to one controller. |
| `pages/vision_background.js` | Non-realtime page ambient canvas background. |

### Realtime JS

```text
web/js/realtime/
  app_realtime.js
  carrot_map.js
  drive_content.js
  drive_content_intro.js
  drive_workspace.js
  home_drive.js
  hud_card.js
  mini_hud.js
  mini_hud_mode.js
  mini_hud_model.js
  raw_capnp.js
  raw_capnp_worker.js
  replay_client.js
  replay_log_worker.js
  replay_video_worker.js
  vision_replay.js
  vision_diag.js
  vision_raw.js
  vision_rtc.js
  vision_state.js
```

| Path | Scope |
|---|---|
| `realtime/app_realtime.js` | Live runtime polling/raw stream wiring and HUD bridge. |
| `realtime/drive_workspace.js` | Stable Area 1/Area 2 slot geometry, resizing, and content registration. |
| `realtime/drive_content.js` | Common mounted content lifecycle contract. |
| `realtime/drive_content_intro.js` | Shared idle/disabled slot intro DOM component; feature runtimes provide state and translated copy. |
| `realtime/home_drive.js` | Drive/Carrot Vision renderer and overlay canvas. |
| `realtime/hud_card.js` | HUD card data rendering. |
| `realtime/mini_hud_mode.js` | Compact HUD eligibility, viewport hysteresis, and activation events. |
| `realtime/mini_hud_model.js` | Pure STOCK/NAV/WAZE compact HUD data normalization. |
| `realtime/mini_hud.js` | Compact HUD DOM rendering and responsive typography. |
| `realtime/carrot_map.js` | Kakao minimap iframe bridge (kmap), `FRAME_VERSION` gated to limit SDK quota. |
| `realtime/raw_capnp.js` | Schema-focused raw-capnp decoder used by the replay log worker and legacy diagnostics; not executed on the main UI thread. |
| `realtime/raw_capnp_worker.js` | Legacy diagnostic worker-side decoder; not loaded by the main page. |
| `realtime/replay_client.js` | Client replay capability selection and log/video worker coordination. |
| `realtime/replay_log_worker.js` | Browser-side rlog download, zstd/bzip2 decompression, Cap'n Proto decoding, sampling, and camera timestamp matching. |
| `realtime/replay_video_worker.js` | Browser-side MPEG-TS to fragmented-MP4 repackaging; it does not decode or re-encode video. |
| `realtime/vision_replay.js` | Replay lifecycle, media transport integration, timeline application, and old-platform fallback entry. |
| `realtime/vision_webgl2.js` | Optional WebGL2 geometry renderer with Canvas compatibility fallback. |
| `realtime/vision_webgl2_worker.js` | OffscreenCanvas worker used by performance mode when the browser supports worker WebGL2. |
| `realtime/vision_state.js` | Shared vision/HUD state object. |
| `realtime/vision_rtc.js` | WebRTC stream client. |
| `realtime/vision_raw.js` | Single Compact WebSocket state owner, HUD normalization, and video/model frame matching. |
| `realtime/vision_diag.js` | Silent WebRTC/video diagnostic recorder (localStorage ring buffer) feeding the vision diag upload. |

### Compact HUD mode

Compact HUD is an adaptive CWeb surface for resizable launcher panels and
Android multi-window use. By default it enters at viewport widths up to `480px`
when the viewport is at least 22% smaller than the exposed screen or outer
window. Widths up to `280px` are treated as unambiguously compact. The mode
stays active through `539px` and exits at `540px`.

`?mini_hud=auto` bypasses the windowed-evidence check but keeps the width
thresholds. `?mini_hud=force` always enables the surface, and `?mini_hud=off`
disables adaptive detection. Browser JavaScript has no reliable equivalent of
Android `Activity.isInMultiWindowMode()`, so the default evidence check keeps
ordinary full-screen phones out while supporting launcher panels, popup
windows, desktop responsive windows, and multi-window layouts. While active,
raw HUD realtime remains connected and Vision/WebRTC is suspended until the
window leaves Compact HUD mode.

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
vendor/xterm.css
pages/terminal.css
pages/settings/{base,panels,device}.css
pages/tools/{base,qr,main,drive_layout}.css
pages/drive.css
responsive.css
vendor/plyr.css
```

JavaScript is loaded in this order:

```text
vendor/{plyr.min,qrcode-generator,jsQR}.js
translations/{registry,ko,en,zh}.js
realtime/hud_card.js
shared/{constants,dom,utils,i18n,api,setting_diff,activity,drive_layout_spec}.js
shared/ui/{focus_trap,dialog,viewport,effects,navigation}.js
pages/car.js
pages/setting*.js
pages/tools*.js + pages/web_settings/*.js
pages/branch.js
pages/logs/*.js
pages/terminal.js
pages/support_terminal.js
realtime/{vision_compact,vision_state,vision_rtc,vision_raw,app_realtime,replay_client,vision_replay,vision_diag,carrot_map}.js
pages/vision_background.js
realtime/home_drive.js
app.js
```

Asset URLs carry `?v=` cache-busting query strings; bump the version when a file changes.

## Page Reference

| Page | HTML | JS | CSS | Backend |
|---|---|---|---|---|
| Drive/Home | `pageCarrot`, `driveHudCard`, `rtcVideo` | `realtime/*`, `vision_background.js` | `pages/drive.css`, `hud_card.css` | `features/system.py`, `features/ws.py`, `features/stream.py`, `features/vision_diag.py`, `realtime/transports/*` |
| Setting | `pageSetting` | `setting.js`, `setting_device*.js`, `car.js` | `pages/settings/*` | `features/settings.py`, `features/params.py`, `features/system.py`, `features/setting_*` |
| Tools | `pageTools` | `tools.js`, `tools_notifications.js`, `tools_web_settings*.js`, `pages/web_settings/*`, `tools_settings_qr.js`, `branch.js` | `pages/tools/*` | `features/tools/*`, `features/system.py`, `features/params.py`, `features/web_settings.py` |
| Logs | `pageLogs` | `pages/logs/shared.js`, `dashcam.js`, `screenrecord.js` | `pages/logs.css` | `features/dashcam/*`, `features/screenrecord/*` |
| Terminal | `pageTerminal` | `terminal.js`, `support_terminal.js` | `pages/terminal.css`, `vendor/xterm.css` | `features/terminal.py`, `features/support_terminal.py`, `services/terminal_pty.py`, `services/support_terminal.py` |
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
camerad road VisionIPC
  -> carrot_vision_encoderd --carrot-vision-road (real car, onroad, DisableDM=2, active session only)
  -> livestreamRoadEncodeData (fixed H.264 settings)
  -> carrot_webrtcd /stream (always listening while real car is onroad with DisableDM=2)
     - an accepted road session toggles only the dedicated encoder through CarrotVisionActive
     - generic notCar webrtcd remains separate
  -> web/js/realtime/vision_rtc.js video + 12-byte RTP/frameId sync mapping

cereal display state (one shared latest-only poll loop)
  -> server/features/ws.py /ws/compact_state
  -> realtime/compact_state_pyx (native display-field serialization for all services)
  -> realtime/compact_state.py (wire schema, Python compatibility fallback, multi-service batch)
  -> web/js/realtime/vision_compact.js + vision_raw.js
  -> browser-side projection
  -> compatibility Canvas 2D or performance WebGL2/OffscreenCanvas drawing
```

Where `requestVideoFrameCallback.rtpTimestamp` is available, the browser uses
the peer connection's RTP/frameId mapping to select the matching `modelV2`
sample. Browsers without that optional metadata keep the guarded `mediaTime`
compatibility path; state transport and reconnect behavior do not fork.

The normal `encoderd`, camera recording, model/UI VisionIPC, Cluster, and YouTube
encoder paths remain separate. The generic three-camera `encoderd --stream` mode
is retained for `notCar`; a real C3/C3X/C4 Carrot Vision session uses only the
dedicated road publisher and activates frame feeding only while a road viewer is
registered. With no viewer, manager stops the dedicated encoder process; its V4L
session and road VisionIPC client are therefore closed rather than consuming and
discarding frames.

Vision diagnostics:

```text
web/js/realtime/vision_diag.js (records RTC/video stats)
  -> /api/vision_diag/server_snapshot (features/vision_diag.py)
  -> /api/vision_diag/upload_discord -> services/vision_diag.py
```

### First-run intro

Shown once on a fresh install, before the app. Server-side decision, no
external service — there is no comma/carrot API that can answer "is this a
fresh install", and deciding locally also works offline.

```text
server/features/intro/state.py intro_bootstrap()
  -> features/static.py injects window.__CARROT_BOOTSTRAP__.intro
    -> shared/ui/navigation.js bootstrapWebStartPage() gate
      -> web/js/intro/intro.js CarrotIntroShell.open()
        -> steps/step0_welcome -> step2_car -> [step3_hda] -> step4_control
           -> step5_legal -> step6_outro
          -> POST /api/intro/apply_preset  (features/intro/presets.py)
          -> POST /api/intro/complete      (/data/carrot/state/intro.json)
        -> bootstrapWebStartPage("intro") enters the app
```

Flag lives in `CARROT_STATE_DIR` (outside the git tree) so `git clean -xfd`
from the reset/sync tools cannot make the intro reappear. `state.py`
additionally treats a device that already has a car selected (or web state
files, or a completed training guide) as already onboarded, so existing users
never see the intro on upgrade.

Car selection reuses the existing web code rather than duplicating it:
`ensureCarsLoaded()` / `stripMaker()` / `applyCurrentCarLabel()` from
`pages/car.js`, sharing the same `CARS` cache and car label state.

Preset values live only in `server/features/intro/presets.py`; the client
sends a preset name. There is no bulk param write API, so applying them
client-side would mean seven round trips and a half-applied vehicle control
state on any mid-way failure.

`step3_hda` runs for Hyundai/Kia/Genesis only; other brands keep `CanfdHDA2`
untouched.

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

Recorded replay startup:

```text
GET /api/dashcam/replay-source/{segment}
  -> cheap file metadata only (no LogReader, decompression, or ffmpeg)
  -> rlog.zst/rlog.bz2/rlog + qcamera.ts/qcamera.mp4 streamed unchanged

phone/PC replay_log_worker.js
  -> streaming zstd, buffered legacy bzip2, or raw input
  -> raw_capnp.js selected display fields
  -> sampled timeline + qRoadEncodeIdx camera synchronization

phone/PC replay_video_worker.js (qcamera.ts only)
  -> mux.js MPEG-TS to fragmented MP4 repackaging, no re-encode
  -> MediaSource/ManagedMediaSource as fragments arrive
  -> complete in-memory MP4 Blob when MSE is unavailable
```

Replay data requests use `no-store` and no persistent replay cache is created.
Missing streaming fetch falls back to a full response buffer on the same client;
missing MSE falls back to a full Blob on the same client. Slow processing and
ordinary worker/media errors are reported and never rerouted to the comma device.
Only browsers missing the required worker/client primitives use the existing
server preparation path. Logger output and loggerd are unchanged.

### Terminal

```text
web/js/pages/terminal.js
  -> /ws/terminal_pty
  -> features/terminal.py
  -> services/terminal_pty.py

web/js/pages/support_terminal.js (owner approval/presence)
  -> features/support_terminal.py
  -> services/support_terminal.py

web/support_terminal/guest.js (authenticated remote xterm)
  -> /ws/support_terminal/{session_id}
  -> services/support_terminal.py
  -> shared services/terminal_pty.py
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
| Minimap | `web/js/realtime/carrot_map.js` |
| Vision diagnostics | `web/js/realtime/vision_diag.js`, `server/features/vision_diag.py` |
| Drive backend data | `server/features/system.py`, `server/live_runtime/*` |
| Raw WebSocket | `server/features/ws.py`, `realtime/transports/raw_ws.py` |
| Compact Vision state | `server/features/ws.py`, `realtime/compact_state.py`, `web/js/realtime/vision_compact.js` |
| Camera WebSocket | `realtime/transports/camera_ws.py` |
| Logs UI | `web/js/pages/logs/*` |
| Dashcam backend | `server/features/dashcam/*` |
| Recorded replay client | `web/js/realtime/replay_client.js`, `replay_log_worker.js`, `replay_video_worker.js`, `vision_replay.js`, `raw_capnp.js` |
| Screenrecord backend | `server/features/screenrecord/*` |
| Terminal UI/backend | `web/js/pages/terminal.js`, `web/js/pages/support_terminal.js`, `web/support_terminal/*`, `server/features/terminal.py`, `server/features/support_terminal.py`, `server/services/terminal_pty.py`, `server/terminal_commands/*` |
| First-run intro | `web/js/intro/*`, `web/css/pages/intro.css`, `server/features/intro/*` |
| Intro presets (driving control) | `server/features/intro/presets.py` |
| Intro first-run decision | `server/features/intro/state.py` |
| Translations | `web/js/translations/*` |
| Recovery server | `recovery/server.py` |
