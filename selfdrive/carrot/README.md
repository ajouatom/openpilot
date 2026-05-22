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
<<<<<<< HEAD
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
=======
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
>>>>>>> dd1ae2e3ff3a4e0ede3564277f8d8d00d682b644
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

<<<<<<< HEAD
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
=======
| Path | Used by | Role |
>>>>>>> dd1ae2e3ff3a4e0ede3564277f8d8d00d682b644
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

<<<<<<< HEAD
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

---

## Design System Reference

Everything below is a working contract. **When in doubt, copy the pattern.**
Don't invent new motion durations, shadow stacks, z-index numbers, or focus
ring colors — use the tokens. New components should compose existing
primitives before adding their own.

All tokens live in [css/tokens.css](web/css/tokens.css) with usage comments next to each group.

### Color (Material 3 dark)

#### Surface & text

| Token | Use for |
|---|---|
| `--md-surface` | page background |
| `--md-surface-cont` | cards, list rows |
| `--md-surface-cont-l` | slightly recessed (input field background) |
| `--md-surface-cont-h` | raised surfaces (dialog sheet, popover) |
| `--md-surface-cont-hh` | nested raised (chip on a card) |
| `--md-surface-bright` | highlight surface (selected row hover) |
| `--md-on-surface` | primary text |
| `--md-on-surface-var` | secondary text, captions |
| `--md-outline` / `--md-outline-var` | borders, dividers |

#### Brand & state surfaces

| Token | Use for |
|---|---|
| `--md-primary` | accent (Carrot orange) |
| `--md-on-primary` | text on a primary-filled surface |
| `--md-action-filled` / `--md-on-action-filled` | primary action button (filled variant) |
| `--md-primary-state-soft` / `-state` / `-state-strong` | hover/pressed surfaces tinted by primary |

#### Semantic status

Use these — don't hardcode greens, ambers, blues. Each family has a base color, a `-strong` accent, a `-cont` (container surface for chips/badges), and an `-on-*-cont` (text on that container).

| Family | Base | When |
|---|---|---|
| Success | `--md-success` (`#8fdc9b`) | confirmation, "OK", restored states |
| Warning | `--md-warning` (`#ffc94a`) | non-critical alerts, slow network, "may take a while" |
| Info | `--md-info` (`#7dd3fc`) | informational hints, "did you know" |
| Error | `--md-error` (`#ff9d94`) | soft errors — form validation, failed toast |
| Danger | `--md-danger` (`#ff8a80`) | alarm-level — active hazard, irreversible destructive |

**Example — semantic chip:**
```html
<span class="chip chip--success">Saved</span>
<span class="chip chip--warning">Slow network</span>
<span class="chip chip--danger">Recording</span>
```

`prefers-contrast: more` shifts surface and outline tokens automatically — don't override per-component.

### Motion

| Duration | Value | Use for |
|---|---|---|
| `--motion-instant` | 80ms | state flicks (toggle on/off colour) |
| `--motion-quick` | 140ms | hover, small togglers, taps |
| `--motion-base` | 180ms | default for most things |
| `--motion-medium` | 240ms | FAB menus, sheets entering |
| `--motion-long` | 380ms | page transitions, large slides |

| Easing | Use for |
|---|---|
| `--ease-standard` | symmetric in/out (default) |
| `--ease-emphasized` | enter / open / expand (decelerates onto place) |
| `--ease-emphasized-accelerate` | exit / close / dismiss (accelerates away) |
| `--ease-linear` | crossfades, progress bars only |

**Pattern — asymmetric open/close (preferred):**
```css
.menu          { transition: opacity var(--motion-quick) var(--ease-emphasized-accelerate); }
.menu.is-open  { transition: opacity var(--motion-medium) var(--ease-emphasized); }
```

For a worked example see the Setting FAB menu in [css/pages/settings/base.css](web/css/pages/settings/base.css) (`.setting-fab-actions`).

### State layer (Material 3)

```css
--state-hover:   0.08;
--state-focus:   0.12;
--state-pressed: 0.12;
--state-dragged: 0.16;
```

**Pattern A — use the `.state-layer` helper:** position any interactive element relative, add the class, and it overlays a primary-tinted layer that intensifies on hover/focus/press.
```html
<button class="my-button state-layer">…</button>
```

**Pattern B — manual color-mix** (when you need control over which color tints):
```css
.row:hover {
  background: color-mix(in srgb, var(--md-primary) var(--state-hover-pct), transparent);
}
```

### Elevation

5 levels, dark-tuned, picked smallest-first.

| Token | Use for |
|---|---|
| `--shadow-1` | hovered/lifted controls |
| `--shadow-2` | default cards, FAB |
| `--shadow-3` | popovers, dropdowns |
| `--shadow-4` | dialogs, sheets |
| `--shadow-5` | fullscreen modals, video player, pickers |

For brand-tinted elevation (orange FAB) compose with the base shadow rather than re-encoding a coloured shadow inline:
```css
box-shadow: var(--shadow-3), 0 0 0 1px var(--md-primary);
```

### Z-index scale

Use these tokens for cross-component layering. Local stacking inside one component (1/2/3) can stay as raw numbers.

| Token | Value | Use for |
|---|---|---|
| `--z-base` | 1 | in-flow content |
| `--z-sticky` | 50 | sticky headers, subnav |
| `--z-rail` | 100 | side nav rail (landscape) |
| `--z-nav` | 120 | bottom nav bar |
| `--z-fab` | 130 | FAB / FAB menus |
| `--z-popover` | 150 | dropdowns, tooltips |
| `--z-modal` | 170 | dialogs, sheets, pickers |
| `--z-toast` | 200 | transient toast layer |
| `--z-overlay` | 220 | fullscreen overlays |

### Focus & reduced motion (global)

[base.css](web/css/base.css) sets:
- One global `:focus-visible` ring using `--focus-ring-*` tokens — covers every interactive element. Override only when shape requires it.
- `@media (prefers-reduced-motion: reduce)` collapses every animation/transition to 0.001ms so things still *snap* into state without movement.

Both rules are intentionally broad. Don't recreate them per component.

### Shared primitives ([components.css](web/css/components.css))

These exist so pages don't reinvent the same chip / icon button / loading
skeleton / empty state over and over. Compose them before writing new CSS.

| Class | Variants | Use for |
|---|---|---|
| `.btn` | `--filled`, `--danger`, `.smallBtn` | text buttons |
| `.icon-btn` | `--circle`, `--ghost`, `--sm`, `--lg` | icon-only buttons (36×36 default) |
| `.chip` | `--accent`, `--danger`, `--success`, `--warning`, `--info` | status tags, counts, labels |
| `.skeleton` | `--circle` | loading placeholders (shimmer) |
| `.empty-state` | `__title`, `__message`, `__action` | "no items" cards in lists |
| `.state-layer` | — | M3 state overlay on any interactive surface |
| `.ui-stagger-item` | (uses CSS var `--i`) | sequenced list reveal animation |
| `.ui-dropdown-menu` | `__button`, `__panel`, `__item`, `--primary`, `--danger` | dropdown menus |
| `.ui-action-grid` | `--quick` | button grids (Tools quick actions) |
| `.app-dialog` | `__sheet`, `__title`, `__body`, `__actions` | dialogs (use the JS API instead) |
| `.app-toast` | `is-error`, `is-success`, `is-hint` | toasts (use `showAppToast` instead) |
| `.visually-hidden` | — | screen-reader-only text |

#### Worked examples

**Icon-only button** — picks up the global focus ring automatically:
```html
<button class="icon-btn icon-btn--circle" type="button" aria-label="More">
  <svg viewBox="0 0 24 24" aria-hidden="true"><path .../></svg>
</button>
```

**Status chip:**
```html
<span class="chip">3 segments</span>
<span class="chip chip--success">Connected</span>
<span class="chip chip--danger">Recording</span>
```

**Loading skeleton** — animates a shimmer; respects reduced motion:
```html
<div class="skeleton" style="width:80%; height:14px;"></div>
<div class="skeleton skeleton--circle" style="width:36px; height:36px;"></div>
```

**Empty state:**
```html
<div class="empty-state" role="status">
  <div class="empty-state__title">No items</div>
  <div class="empty-state__message">Try changing filters.</div>
  <button class="btn empty-state__action">Refresh</button>
</div>
```

**Staggered list reveal** — the animation runs once on append. Set `--i`
per item; the delay caps at 420 ms so long lists don't drag:
```js
items.forEach((el, i) => el.style.setProperty('--i', i));
items.forEach((el) => el.classList.add('ui-stagger-item'));
```

**Accessible icon-only close (with hidden label):**
```html
<button class="icon-btn icon-btn--ghost" aria-label="Close">
  <svg>…</svg>
  <span class="visually-hidden">Close</span>
</button>
```

### Shared keyframes

Named animations available via `animation: <name> …`:

| Keyframe | Where | Used by |
|---|---|---|
| `uiStaggerIn` | components.css | `.ui-stagger-item` — slide-up + fade-in |
| `skeleton-shimmer` | components.css | `.skeleton::after` — horizontal sweep |

Per-feature animations (e.g. `dashcam-segment-append`, `tools-detail-open`,
`record-blink`) live in the relevant page CSS and use a
`<feature>-<verb>` name. Promote one to a shared keyframe only when a
new primitive will use it.

### Naming conventions

- **BEM-ish** is the working style.
  - Block: `.app-dialog`, `.dashcam-route-card`.
  - Element: `.app-dialog__title`, `.dashcam-route-card__head`.
  - Modifier: `.btn--filled`, `.chip--success`, `.icon-btn--circle`.
- **State classes** (toggled at runtime): `.is-open`, `.is-active`,
  `.is-loading`, `.is-error`, `.is-collapsed`, `.is-visible`. Always
  `is-` prefixed.
- **Behavior vs. style separation:**
  - `[data-action="play"]` → wired in JS via event delegation.
  - `.is-active` → read by CSS only.
  - Don't put `[data-action="…"]` in CSS selectors.
- **JS globals.** Top-level `let`/`const` are visible across every file
  in load order (no modules). For state, prefer namespaced names —
  `dashcamState`, `screenrecordState`, `settingFabMenuOpen`. Grep
  before claiming a new name; collisions are real.
- **Translations.** `getUIText("key", "English fallback", { count })`.
  Always provide the English fallback inline — it's the source string.
  When adding a new key, update all five locale files in
  [`web/js/translations/`](web/js/translations/).

### JS UI utilities

`shared/ui/` — call these instead of building your own modal/toast:

| Function | Source | Use for |
|---|---|---|
| `appAlert`, `appConfirm`, `appPrompt`, `openAppDialog` | [dialog.js](web/js/shared/ui/dialog.js) | All modal text dialogs and choice sheets |
| `showAppToast(message, { tone, duration })` | dialog.js | Transient feedback. Tones: `default`, `error`, `success`, `hint` |
| `syncModalBodyLock` | dialog.js | Call after manually showing/hiding a sheet to lock body scroll |
| `createFocusTrap(container, opts)` | [focus_trap.js](web/js/shared/ui/focus_trap.js) | Required for any new modal/overlay (a11y) |
| `showPage`, page transition helpers | [navigation.js](web/js/shared/ui/navigation.js) | Page-level navigation |
| Viewport metrics, `--app-vv-height` | [viewport.js](web/js/shared/ui/viewport.js) | Responsive math against the real viewport |
| `escapeHtml`, `clamp`, `copyToClipboard` | [utils.js](web/js/shared/utils.js) | Always escape interpolated text in template literals |
| `getJson`, `postJson`, `bulkGet`, `setParam` | [api.js](web/js/shared/api.js) | Backend access (don't use raw `fetch`) |

**Modal pattern with focus trap** — required for any new dialog/overlay
to remain keyboard-accessible:
```js
const overlay = document.createElement("div");
overlay.className = "my-overlay";
overlay.setAttribute("role", "dialog");
overlay.setAttribute("aria-modal", "true");
overlay.innerHTML = `<div class="my-overlay__sheet">…</div>`;
document.body.appendChild(overlay);

const trap = createFocusTrap(overlay, {
  initialFocus: ".my-overlay__primary",  // selector or element
  escape: () => close(),                  // optional Esc handler
});
trap.activate();

function close() {
  trap.deactivate();         // restores focus to whoever had it before open
  overlay.remove();
  syncModalBodyLock();
}
```

Page-change broadcasting:
```js
window.addEventListener("carrot:pagechange", (ev) => { /* ev.detail.page */ });
```

Language-change broadcasting (re-render translated strings):
```js
window.addEventListener("carrot:languagechange", () => { /* re-render */ });
```

### Page lifecycle

- The current page is on `body[data-page="…"]`. Listen for
  `carrot:pagechange` to **clean up everything you started**: timers,
  observers, WebSockets, scroll listeners. Mirror the
  [`handleLogsPageChange`](web/js/pages/logs/shared.js) pattern.
- For lists with more than ~30 items: build a virtual window with
  top/bottom spacers, not a flat render. The canonical pattern is in
  [`logs/dashcam.js`](web/js/pages/logs/dashcam.js) — `dashcamWindowFor`
  computes the visible slice and `patchDashcamWindow` patches the DOM.
- For "load more" sentinels at list ends: use `IntersectionObserver`
  with the scroll container as `root`. See
  `ensureDashcamSegmentLoaderObserver` in
  [`logs/dashcam.js`](web/js/pages/logs/dashcam.js). **Never poll
  `getBoundingClientRect()` from a scroll handler** — it forces a
  layout per frame and kills scroll smoothness.
- For lazy images: reuse `hydrateLogsLazyImages` /
  `loadLogsLazyImage` in [`logs/shared.js`](web/js/pages/logs/shared.js).

### Accessibility checklist for new UI

1. Every interactive element is focusable and reachable by keyboard (Tab / Shift+Tab).
2. Icon-only buttons have an `aria-label`.
3. Dialogs use `role="dialog" aria-modal="true"` and a `.app-dialog__title` for the accessible name.
4. The global `:focus-visible` ring is visible — don't `outline: none` without a replacement.
5. Color is never the *only* signal (status chips include a label, error rows have an icon).
6. Animations respect the global `prefers-reduced-motion` guard — don't bypass it.
7. Tap targets are ≥ 44×44px (Material 3 / WCAG).

### Anti-patterns (common mistakes)

| Don't | Do | Why |
|---|---|---|
| `transition: opacity 0.2s ease;` | `transition: opacity var(--motion-base) var(--ease-standard);` | Consistent motion + reduced-motion guard hooks in |
| `z-index: 170;` | `z-index: var(--z-modal);` | Magic numbers drift; tokens describe intent |
| `color: #8fdc9b;` | `color: var(--md-success);` | Hardcoded greens fragment the palette |
| `box-shadow: 0 18px 40px rgba(0,0,0,.34);` | `box-shadow: var(--shadow-4);` | Same elevation everywhere = clear hierarchy |
| `confirm("…")`, `alert("…")` | `await appConfirm("…")`, `showAppToast("…")` | Native dialogs ignore theming and block the page |
| `outline: none;` (then no replacement) | leave the global ring, or replace with an equivalent | Keyboard users lose all feedback |
| `<button><svg/></button>` (no label) | add `aria-label="…"` or a `.visually-hidden` text | Screen readers can't announce the button |
| `addEventListener("touchmove", h)` | `addEventListener("touchmove", h, { passive: true })` | Non-passive `touchmove` blocks the compositor — every scroll jumps |
| `setInterval(check, 16)` polling rects | `IntersectionObserver` on the sentinel | rAF-rate rect polling kills scrolling on mobile |
| `\`<div>${userText}</div>\`` | `\`<div>${escapeHtml(userText)}</div>\`` | Template literals are interpolated raw; XSS risk |
| `fetch("/api/...")` | `getJson("/api/...")` / `postJson(...)` | Consistent error handling, JSON parsing, auth headers |

### Failure modes specific to this codebase

- **Scroll jank from non-passive listeners.** Any `touchmove` listener
  that isn't `passive: true` makes the entire scroll path go through
  the main thread. Even if you never call `preventDefault()`, the
  browser must wait for JS to decide. The dashcam segment list had this
  bug and removing the guard fixed it.
- **`[hidden]` kills transitions.** `[hidden] { display: none }` removes
  the element from layout — there's nothing for a transition to animate
  from/to. To animate close, remove `is-open` first to start the
  transition, then set `hidden` after the duration completes (use
  `--motion-medium` as the timer).
- **`replaceChildren` resets nested `scrollTop`.** When a virtual list
  re-renders, child scroll containers lose their position. Capture
  before replacing, restore after — see
  `rememberVisibleDashcamSegmentScrolls` /
  `restoreVisibleDashcamSegmentScrolls` in
  [`logs/dashcam.js`](web/js/pages/logs/dashcam.js).
- **Asymmetric container padding.** Padding cascades to every child.
  If a list looks lopsided, check the parent's `padding` first. The
  fix in dashcam was `padding: 12px 28px 12px 12px` → `padding: 12px 14px`.
- **Hardcoded scrollbar gutter.** On touch, scrollbars overlay and
  don't take space — `padding-right: 2px` to "make room" creates
  visible asymmetry on devices that show the scrollbar. Either
  `scrollbar-width: none` (hide on touch) or symmetric
  `padding-inline`.
- **Loader height change.** A loader that grows from 16 px to 22 px on
  state change shoves the list above it. Keep the loader at a fixed
  height and fade in the indicator with opacity instead.

### When adding a new page

1. Add `<link>` for `css/pages/<page>.css` in load order (after `components.css`).
2. Add `<script>` for `js/pages/<page>.js` (after `shared/ui/*`).
3. Use existing tokens and primitives first. Promote to a new token
   only after the third repeat of the same value.
4. Listen for `carrot:pagechange` to clean up timers/observers when
   navigating away.
5. For modals, use `createFocusTrap` and restore body scroll with
   `syncModalBodyLock`.
6. Wire keyboard support: Esc closes overlays, Enter/Space activates
   buttons.
7. Add UI strings via `getUIText("key", "English fallback")` and
   translate in all five locale files.

### A11y checklist for every new UI

1. Keyboard reachable: Tab/Shift+Tab cycles, Enter/Space activates,
   Esc closes overlays.
2. Icon-only buttons have `aria-label`.
3. Modals: `role="dialog"`, `aria-modal="true"`, focus trap active,
   focus restored on close.
4. The global `:focus-visible` ring is visible — don't `outline: none`
   unless you provide an equivalent visual.
5. Color is never the only signal — pair status colors with a label
   or icon (the `.chip` variants do this for you).
6. Tap targets ≥ 44×44 (use `.icon-btn--lg` when targeting touch).
7. Animations honor the global `prefers-reduced-motion` guard. If
   movement is essential to meaning, swap it for an opacity/color
   change rather than bypassing the guard.

### Quick checklist before writing CSS

1. Is the color a semantic token (`--md-success`, etc.)?
2. Is the motion duration one of `--motion-*`?
3. Is the easing one of `--ease-*`?
4. Is the shadow one of `--shadow-*`?
5. Is the z-index one of `--z-*`?
6. Does a primitive (`.icon-btn`, `.chip`, `.skeleton`,
   `.empty-state`, `.state-layer`, `.ui-stagger-item`) cover most of
   this already?
7. Does `:focus-visible` still work? (No bare `outline: none`.)
8. Does the selector stay flat (single class)? Avoid `.a .b .c`
   chains — they fight the cascade.

### Quick checklist before writing JS

1. Are template literal interpolations escaped with `escapeHtml`?
2. Are you cleaning up on `carrot:pagechange` (timers, observers,
   WebSockets, scroll listeners)?
3. For touch interactions: are scroll-impacting listeners
   `passive: true`?
4. For DOM lists ≥ 30 items: are you using a virtual window?
5. For "load more" / lazy hydration: are you using
   `IntersectionObserver`, not scroll-event rect polling?
6. For a new modal: did you call `createFocusTrap`?
7. Are you using `getJson`/`postJson`, not raw `fetch`?
8. Did you add the `<script>` tag to `index.html` in the right load
   position?

=======
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
>>>>>>> dd1ae2e3ff3a4e0ede3564277f8d8d00d682b644
