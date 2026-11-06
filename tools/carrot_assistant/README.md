# Carrot ChatGPT bridge

This standalone Node MCP service serves a pinned public settings catalog and
delivers **setting screen navigation only** to a user-paired Carrot Web tab.
It has no device network access, Params writes, current values, route files,
vehicle controls, or model API calls. ChatGPT supplies the conversational model
through the user's own account. Client availability and tool approval behavior
remain subject to that ChatGPT client/account.

## Flow

1. In Carrot Web, tap the top-right microphone icon > Create pairing code.
2. Copy the generated message into a conversation with the Carrot MCP enabled.
   `connect_carrot_web` exchanges the one-time code for a conversation capability.
3. ChatGPT searches the public catalog and calls `open_carrot_setting` with an
   exact setting name. Return to Carrot Web; its connection dialog closes when
   the request arrives (unless you are editing its conversation URL).
4. The visible web tab opens its own setting entry and acknowledges the result.
   `get_carrot_web_status` distinguishes queued/opened/unavailable/failed/expired.
   `cancel_carrot_screen_request` cancels an unconsumed request.

The launcher can remember an optional ChatGPT conversation URL. It does not
automatically select a plugin, send a message, start voice, or bring another
app/browser tab to the foreground automatically. On Android, a user-initiated
intent requests the installed ChatGPT app; unresolved intents fall back to its
official Google Play listing. App handling of conversation links needs phone testing.
Start voice in ChatGPT, then return to the visible Carrot Web. Background voice
and split/pop-up windows depend on the app and phone; the web cannot force them.
After saving a conversation URL, the corner icon opens it; holding the icon,
Shift-click, or the Tools gear opens connection settings. Browser capabilities
are retained in localStorage for the same origin. Old sessionStorage is migrated.
Closing the tab pauses delivery; reopening the same address resumes it.

## Boundaries

- Codes have 80 random bits, expire after 5 minutes, and are single use.
- Browser and conversation capabilities are distinct 256-bit random tokens.
  Only hashes are retained server-side. Never log tool arguments or tokens.
- Paired connections last until explicit disconnect. With MCP_STATE_PATH set,
  paired hashes are atomically saved in the dedicated connections volume and
  survive server restarts. Codes, raw tokens and commands are never persisted.
  Invalid storage fails startup; failed saves do not issue new capabilities.
  Without MCP_STATE_PATH, state is in memory for local development.
  This is not account identity: reuse the same ChatGPT conversation; if it loses
  its capability, or browser data/origin changes, pair again. The one-time 0.2 to
  0.3 deployment cannot preserve old RAM-only connections; pair once afterwards.
- Each session holds one request, expiring after 90 seconds. Different pending
  targets require cancellation first. Editing/dialogs defer navigation.
- Browser requests require their exact paired Origin and bearer capability.
  Accepted origins are loopback or LAN hostnames/private IPv4 on port 7000.
  Origin filtering is an additional browser restriction, not authentication.
- A pairing-code holder can open screens in that browser until revocation.
  Do not share codes. This capability is deliberately unsuitable for future
  setting writes or logs; those need separate authorization and device checks.
- Up to 1000 sessions, 60 creations/minute and 600 MCP requests/minute globally.
  Each visible paired tab polls every 3 seconds; hidden tabs do not poll.
  These are prototype limits, not a hundreds-of-users load-test result.
- Private LAN HTTP does not protect browser capabilities from an attacker on
  that LAN. Use only a trusted network; future privileged operations need a
  stronger transport and identity design.

## Run / deploy

Node 24, `npm ci --ignore-scripts`, `npm test`, `npm start` (loopback 18770).
For NAS deployment use `compose.yaml` behind the existing HTTPS reverse proxy
`carrot-mcp.shind0.synology.me` to `127.0.0.1:18770`. Set PUBLIC_HOSTNAME to the
exact public host. `/mcp` is Streamable HTTP; `/health` reports the version.
The independent container does not mount upload-service data or credentials.
The image is non-root/read-only, has no extra capabilities, and has a 256 MiB
memory limit. CPU shares are a scheduling preference, not a hard CPU limit.

After deploying a changed tool list, refresh the connected MCP in ChatGPT.
Rebuild the web bundles with `npm run build` in
`openpilot/selfdrive/carrot/web`. The relay URL is in assistant_policy.js.
The catalog's source metadata is in catalog.json; defaults are not live values.

Validation: `npm test` covers real SDK calls, capability separation, expiry,
revocation, pending/cancel/ack lifecycle, Origin filtering and HTTP delivery.
Web tests run separately using `npm run check` in the web directory.
