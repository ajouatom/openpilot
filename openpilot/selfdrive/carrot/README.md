# Carrot Web

Structure map for `openpilot/selfdrive/carrot`.

## Entry Points

| Path | Role |
|---|---|
| `carrot_server.py` | Web server, port `7000` |
| `server/app.py` | Server composition and background tasks |
| `web/index.html` | Main HTML shell and asset order |
| `web/js/app.js` | Final browser bootstrap |
| `web/build.mjs` | Bundle, Worker, and asset manifest build |
| `recovery/server.py` | Recovery server, port `6999` |

## Structure

```text
openpilot/selfdrive/carrot/
  carrot_server.py
  server/
    app.py                    server composition
    features/                 HTTP, API, WebSocket routes
    services/                 state and feature logic
    live_runtime/             live driving snapshots
    terminal_commands/        terminal command bridge
  realtime/
    compact_state.py          compact driving state
    raw_protocol.py           raw message protocol
    transports/               camera and raw WebSockets
  web/
    index.html                main page shell
    build.mjs                 build definitions
    assets/                   static images
    src/
      entries/                generated bundle entry points
      foundation/tokens/      design and layout tokens
      ui/primitives/          base UI styles
      ui/components/          shared UI components
      shared/                 shared runtime, assets, and state
      features/               feature modules
    js/
      generated/              generated JavaScript
      pages/                  remaining page controllers
      realtime/               remaining realtime bridges and Workers
      shared/                 compatibility runtime
      translations/           language packs
      vendor/                 third-party libraries
    css/
      generated/              generated CSS
      components/             remaining shared styles
      pages/                  remaining page styles
      vendor/                 third-party styles
    generated/
      asset-manifest.json     generated asset map and hashes
    support_terminal/         standalone guest terminal shell
    tests/                    frontend tests
  recovery/                   standalone recovery server
```

## Feature Ownership

| Feature | Path |
|---|---|
| Drive workspace, layout, registry | `web/src/features/drive/core/` |
| Carrot Vision | `web/src/features/drive/contents/vision/` |
| Carrot Navi | `web/src/features/drive/contents/carrot_navi/` |
| Graph and forward driving data | `web/src/features/drive/contents/drive_insights/` |
| Replay | `web/src/features/replay/` |
| Settings | `web/src/features/settings/` |
| Tools | `web/src/features/tools/` |
| Logs, dashcam, screen recording | `web/src/features/logs/` |
| Local, support, and guest terminal | `web/src/features/terminal/` |
| First-run flow | `web/src/features/intro/` |
| Shared UI | `web/src/ui/` |
| Shared browser state and assets | `web/src/shared/` |
| Design tokens | `web/src/foundation/tokens/` |

## Runtime Paths

```text
GET /
  -> server/features/static.py
  -> web/index.html
  -> web/js/generated/* + web/css/generated/*

openpilot messaging
  -> realtime/ + server/live_runtime/
  -> server/features/
  -> web drive runtime
```

## Rules

- Feature code: `web/src/features/<feature>/`
- Reusable UI: `web/src/ui/`
- Cross-feature runtime: `web/src/shared/`
- Tokens: `web/src/foundation/tokens/`
- Bundle entry points: `web/src/entries/`
- Keep new feature modules out of legacy `web/js/` and `web/css/`.
- Do not edit generated files directly.
- Commit the asset manifest with its generated outputs.
- Do not add manual `?v=` values to generated assets.
- Preserve the load order in `web/index.html`.

## Build and Check

```bash
cd openpilot/selfdrive/carrot/web
npm run build
npm run check
```
