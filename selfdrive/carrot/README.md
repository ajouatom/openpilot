# Carrot Web

Anyone can modify. Refer to the structure below.

## Entry

- `carrot_server.py` — starts the aiohttp server on port 7000.
- `recovery/server.py` — tiny standalone recovery server on port 6999.

## Server (Python, aiohttp)

```
server/
├── __init__.py
├── app.py                  composition root: middleware, lifecycle, make_app()
├── config.py               constants (paths, URLs, tmux session, etc.)
├── live_runtime/           cereal SubMaster broker for /api/live_runtime
│   ├── broker.py           RealtimeBroker — single SubMaster lifecycle
│   ├── contract.py         response schema contract
│   ├── normalize.py        raw cereal → JSON normalizer
│   ├── runner.py           async snapshot runner
│   ├── services.py         subscribed service list
│   └── snapshot.py         full snapshot builder
├── services/               shared logic, no HTTP
│   ├── params.py           typed get/set + bulk backup/restore
│   ├── settings.py         carrot_settings.json mtime cache
│   ├── git_state.py        /data/.../git.json read/write
│   ├── git_status.py       periodic cached git fetch/status comparison for update badges
│   ├── heartbeat.py        external IP register loop
│   ├── ssh_keys.py         GitHub SSH key fetch/store helpers for Device developer panel
│   ├── time_sync.py        browser → system time sync
│   ├── device_info.py      focused calibration + network helpers for Device tab
│   ├── setting_favorites.py  CarrotPilot setting favorites state
│   ├── setting_profiles.py   CarrotPilot setting profile CRUD + import/export
│   ├── web_settings.py     device/server-backed Web Settings state
│   └── tmux.py             tmux session helpers
└── features/               HTTP entry points (one feature per file/folder)
    ├── static.py           /, static fallback + initial bootstrap payload
    ├── stream.py           /stream → webrtcd proxy
    ├── ws.py               /ws/raw, /ws/raw_multiplex, /ws/camera
    ├── settings.py         /api/settings
    ├── params.py           /api/params_*, /download/params_backup.json
    ├── setting_favorites.py /api/setting_favorites
    ├── setting_profiles.py  /api/setting_profiles, profile import/export
    ├── web_settings.py     /api/web_settings
    ├── ssh_keys.py         /api/ssh_keys
    ├── cars.py             /api/cars
    ├── system.py           /api/heartbeat_status, /api/reboot, /api/time_sync,
    │                       /api/live_runtime, /api/poweroff, /api/recalibrate,
    │                       /api/set_default, /api/calibration_status,
    │                       /api/device_network
    ├── terminal.py         /ws/terminal, /download/tmux.log
    ├── dashcam/            /api/dashcam/* (paths, catalog, ffmpeg, upload, upload_jobs, routes)
    ├── screenrecord/       /api/screenrecord/* (catalog, routes)
    └── tools/              /api/tools, /api/tools/start, /api/tools/job, /api/tools/git_status
                            (actions, jobs, dispatcher, routes)
```

Rule: `features/` may import `services/`. `services/` must not import `features/`.

## Realtime (WebSocket transports)

```
realtime/
├── __init__.py             package init (exports Hub classes)
├── raw_protocol.py         raw capnp message protocol helpers
├── raw_runner.py           async raw stream runner
├── raw_services.py         raw service list definitions
└── transports/
    ├── __init__.py         exports CameraWsHub, RawWsHub
    ├── camera_ws.py        CameraWsHub — road camera WebSocket relay
    └── raw_ws.py           RawWsHub — multiplexed raw capnp WebSocket relay
```

## Web (no build, plain `<script>` tags)

```
web/
├── index.html              all pages live in one HTML, toggled by display:none
├── assets/                 wheel icon, speed bg
├── css/
│   ├── tokens.css          design tokens
│   ├── layout_tokens.css   layout design tokens (spacing, container sizes)
│   ├── hud_card.css        driving HUD card (realtime — do not touch)
│   ├── base.css            reset, nav bar, FAB, search panel
│   ├── layout.css          page container, swipe, headings, sections
│   ├── components.css      dialog, toast, buttons, setting items, transitions
│   ├── responsive.css      desktop + mobile media queries (loads last)
│   ├── vendor/
│   │   └── plyr.css        Plyr video player styles
│   └── pages/
│       ├── logs.css        Logs/Dashcam page
│       ├── drive.css       WebRTC video + Carrot stage
│       ├── terminal.css    Terminal page styles
│       ├── settings/       Settings page styles, split for readability
│       │   ├── base.css        page base, car entry, FAB menu (open/close anim)
│       │   ├── panels.css      search panel, group list, profile sections, toolbar
│       │   └── device.css      Device tab, settings-diff dialog, subnav, responsive
│       └── tools/          Tools page styles, split by feature
│           ├── base.css        page base, meta/lang, Web Settings dialog
│           ├── qr.css          QR code dialog
│           └── main.css        groups, progress, notifications/console, responsive
└── js/
    ├── app.js              bootstrap: popstate, initial showPage()
    ├── shared/             cross-page modules
    │   ├── constants.js    LANG_*, SWIPE_*, PAGE_TRANSITION_*
    │   ├── dom.js          all getElementById refs in one place
    │   ├── utils.js        escapeHtml, clamp, copyToClipboard, quick link
    │   ├── i18n.js         bootstrapped LANG, getUIText, renderUIText, setWebLanguage
    │   ├── api.js          bulkGet, setParam, postJson, getJson, waitMs
    │   ├── setting_diff.js setting-diff dialog helpers (used by settings + tools)
    │   ├── activity.js     cross-page activity badges + beforeunload guard
    │   └── ui/
    │       ├── dialog.js   appAlert/Confirm/Prompt + toast
    │       ├── effects.js  pointer-down confetti easter egg
    │       ├── viewport.js viewport metrics + drive HUD layout
    │       └── navigation.js  showPage, menu transitions, page state
    ├── pages/
    │   ├── car.js          car picker + record FAB + currentCar status
    │   ├── setting.js      settings groups/items/search/subnav + device tab switcher
    │   ├── setting_device_config.js   Device tab constants and option tables
    │   ├── setting_device_render.js   Device row/panel rendering helpers
    │   ├── setting_device_network.js  Device network refresh loop
    │   ├── setting_device_actions.js  Device action/dialog handlers
    │   ├── setting_device.js          Device tab coordinator and state
    │   ├── tools_web_settings.js      server-backed Web Settings dialog
    │   ├── tools_notifications.js     Tools-tab notification preview/composer
    │   ├── tools_settings_qr.js       Settings QR import/export
    │   ├── tools.js        tools page + initToolsPage + action runners
    │   ├── branch.js       branch picker modal + Branch page
    │   ├── logs/           Logs page, split by tab
    │   │   ├── shared.js       tab state, scroll persistence, lazy-image observer,
    │   │   │                   video player, bind/init
    │   │   ├── dashcam.js      Dashcam tab: virtual route+segment list, upload subsystem
    │   │   └── screenrecord.js Screen Recording tab: virtual list, lazy thumbs
    │   ├── terminal.js     tmux WebSocket client
    │   └── vision_background.js  static background for non-realtime pages
    ├── translations/       ko/en/zh/ja/fr + registry.js
    ├── realtime/           realtime stream stack (loaded together)
    │   ├── hud_card.js          adaptive driving HUD card
    │   ├── raw_capnp.js         raw capnp decoders for HUD/overlay state
    │   ├── raw_capnp_worker.js  worker entry for raw capnp decoding
    │   ├── vision_state.js      shared vision/HUD state
    │   ├── vision_rtc.js        WebRTC vision stream client
    │   ├── vision_raw.js        raw WebSocket vision client + decoder worker bridge
    │   ├── app_realtime.js      live runtime/raw stream wiring + HUD payload bridge
    │   └── home_drive.js        Carrot Vision renderer and overlay canvas
    └── vendor/             third-party libraries (Plyr, jsQR, qrcode-generator)
```

### Settings page tab structure

The Setting page has two top-level tabs:

| Tab | Content | Data source |
|---|---|---|
| **Device** | stock openpilot options (info, toggles, software, power) | Params API direct; network/calibration only use focused APIs |
| **CarrotPilot** | Carrot-specific tuning groups | carrot_settings.json |

Device tab adapts to hardware via `DeviceType` param (`tici`/`mici`/`tizi`).

Load order (set in `index.html`):

CSS:
```
tokens → layout_tokens → hud_card → base → layout → components
  → pages/logs → pages/terminal
  → pages/settings/base → pages/settings/panels → pages/settings/device
  → pages/tools/base → pages/tools/qr → pages/tools/main
  → pages/drive → responsive → vendor/plyr
```

JS:
```
vendor/* → translations → shared/* → shared/ui/* → pages/* → pages/logs/* → realtime/* → app
```

CSS files merge byte-identical with the previous single `settings.css` and `tools.css` if concatenated in the order above. JS scripts share the same global realm — top-level `let`/`const` are visible across files (so the logs split files all see the shared `logsActiveTab`, `dashcamState`, `screenrecordState`, etc.).

### Recovery server (standalone)

```
recovery/
├── __init__.py
└── server.py               port 6999, minimal self-contained recovery UI
```
