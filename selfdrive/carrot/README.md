# Carrot Web

Anyone can modify. Refer to the structure below.

## Entry

- `carrot_server.py` — starts the aiohttp server on port 7000.

## Server (Python, aiohttp)

```
server/
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
│   ├── device_info.py      device type, dongle, serial, calibration, language list
│   └── tmux.py             tmux session helpers
└── features/               HTTP entry points (one feature per file/folder)
    ├── static.py           /, static fallback
    ├── stream.py           /stream → webrtcd proxy
    ├── ws.py               /ws/raw, /ws/raw_multiplex, /ws/camera
    ├── settings.py         /api/settings
    ├── params.py           /api/params_*, /download/params_backup.json
    ├── ssh_keys.py         /api/ssh_keys
    ├── cars.py             /api/cars
    ├── system.py           /api/heartbeat_status, /api/reboot, /api/time_sync,
    │                       /api/live_runtime, /api/poweroff, /api/recalibrate,
    │                       /api/set_default, /api/device_info
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
│   └── pages/
│       ├── logs.css        Logs/Dashcam page
│       ├── drive.css       WebRTC video + Carrot stage
│       ├── settings.css    Settings page styles (includes device tab)
│       ├── terminal.css    Terminal page styles
│       └── tools.css       Tools page styles
└── js/
    ├── app.js              bootstrap: popstate, initial showPage()
    ├── shared/             cross-page modules
    │   ├── constants.js    LANG_*, SWIPE_*, PAGE_TRANSITION_*
    │   ├── dom.js          all getElementById refs in one place
    │   ├── utils.js        escapeHtml, clamp, copyToClipboard, quick link
    │   ├── i18n.js         LANG, getUIText, renderUIText, setWebLanguage
    │   ├── api.js          bulkGet, setParam, postJson, getJson, waitMs
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
    │   ├── tools_web_settings.js      Web-only settings dialog
    │   ├── tools.js        tools page + initToolsPage + action runners
    │   ├── branch.js       branch picker modal + Branch page
    │   ├── logs.js         Dashcam + Screen Recording lists
    │   └── terminal.js     tmux WebSocket client
    ├── translations/       ko/en/zh/ja/fr + registry.js
    └── realtime
        app_realtime.js       live runtime/raw stream wiring + HUD payload bridge
        home_drive.js         Carrot Vision renderer and overlay canvas
        hud_card.js           adaptive driving HUD card
        raw_capnp.js          raw capnp decoders for HUD/overlay state
        raw_capnp_worker.js   worker entry for raw capnp decoding
```

### Settings page tab structure

The Setting page has two top-level tabs:

| Tab | Content | Data source |
|---|---|---|
| **Device** | stock openpilot options (info, toggles, software, power) | Params API direct |
| **CarrotPilot** | Carrot-specific tuning groups | carrot_settings.json |

Device tab adapts to hardware via `DeviceType` param (`tici`/`mici`/`tizi`).

Load order (set in `index.html`):
`tokens → layout_tokens → hud_card → base → layout → pages/* → components → responsive`
then JS:
`translations → hud_card → shared/* → shared/ui/* → pages/* → realtime/* → app.js`

CSS files merge byte-identical with the previous single `app.css` if concatenated in the order above. JS scripts share the same global realm — top-level `let`/`const` are visible across files.
