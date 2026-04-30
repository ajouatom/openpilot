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
├── services/               shared logic, no HTTP
│   ├── params.py           typed get/set + bulk backup/restore
│   ├── settings.py         carrot_settings.json mtime cache
│   ├── git_state.py        /data/.../git.json read/write
│   ├── git_status.py       periodic cached git fetch/status comparison for update badges
│   ├── heartbeat.py        external IP register loop
│   ├── time_sync.py        browser → system time sync
│   └── tmux.py             tmux session helpers
└── features/               HTTP entry points (one feature per file/folder)
    ├── static.py           /, static fallback
    ├── stream.py           /stream → webrtcd proxy
    ├── ws.py               /ws/raw, /ws/raw_multiplex, /ws/camera
    ├── settings.py         /api/settings
    ├── params.py           /api/params_*, /download/params_backup.json
    ├── cars.py             /api/cars
    ├── system.py           /api/heartbeat_status, /api/reboot, /api/time_sync, /api/live_runtime
    ├── terminal.py         /ws/terminal, /download/tmux.log
    ├── dashcam/            /api/dashcam/* (paths, catalog, ffmpeg, upload, upload_jobs, routes)
    ├── screenrecord/       /api/screenrecord/* (catalog, routes)
    └── tools/              /api/tools, /api/tools/start, /api/tools/job, /api/tools/git_status (jobs, dispatcher, routes)
```

Rule: `features/` may import `services/`. `services/` must not import `features/`.

## Web (no build, plain `<script>` tags)

```
web/
├── index.html              all pages live in one HTML, toggled by display:none
├── assets/                 wheel icon, speed bg
├── css/
│   ├── tokens.css          design tokens
│   ├── hud_card.css        driving HUD card (realtime — do not touch)
│   ├── base.css            reset, nav bar, FAB, search panel
│   ├── layout.css          page container, swipe, headings, sections
│   ├── components.css      dialog, toast, buttons, setting items, transitions
│   ├── responsive.css      desktop + mobile media queries (loads last)
│   └── pages/
│       ├── logs.css        Logs/Dashcam page
│       └── drive.css       WebRTC video + Carrot stage
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
    │   ├── setting.js      settings groups/items/search/subnav
    │   ├── tools.js        tools page + initToolsPage + action runners
    │   ├── branch.js       branch picker modal + Branch page
    │   ├── logs.js         Dashcam + Screen Recording lists
    │   └── terminal.js     tmux WebSocket client
    ├── translations/       ko/en/zh/ja/fr + registry.js
    └── (realtime — do not touch)
        app_realtime.js, home_drive.js, hud_card.js,
        raw_capnp.js, raw_capnp_worker.js
```

Load order (set in `index.html`):
`tokens → hud_card → base → layout → pages/logs → components → pages/drive → responsive`
then JS:
`translations → hud_card → shared/* → shared/ui/* → pages/* → realtime/* → app.js`

CSS files merge byte-identical with the previous single `app.css` if concatenated in the order above. JS scripts share the same global realm — top-level `let`/`const` are visible across files.
