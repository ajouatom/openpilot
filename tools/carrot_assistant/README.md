# CarrotAssist MCP relay

This Node service exposes a pinned public settings catalog and delivers commands
to an explicitly paired, visible Carrot Web session. It does not call a model API
or connect directly to a device LAN. Each user uses their own ChatGPT account.
See the [Korean](../../docs/user/ko/carrot-assist.md) and
[English](../../docs/user/en/carrot-assist.md) onboarding guides.

## Account connection

Connect `https://carrot-mcp.shind0.synology.me/account/mcp` with OAuth. Generate a
mini-mode code in Carrot Web Tools and enter it only on the HTTPS consent page.
Codes are single-use, 80-bit secrets valid for five minutes; the connection does
not expire in five minutes. New chats resolve the approved web session from the
validated token, without a caller-selected user or device ID.

`carrot.screen` permits search/explanation/navigation. `carrot.settings` is an
additional, explicitly selected consent scope for live reads and changes. Old
grants and refresh tokens remain screen-only. The device must also issue a local
grant while stationary/disengaged. That grant stays in WebView storage and goes
only to same-origin device endpoints, never to the relay or ChatGPT.

Five setting tools read a selected value, prepare a change, commit a confirmed
proposal, inspect the request and cancel a prepared proposal. Instructions require
before/after values and a follow-up user confirmation. The device validates actual
values, bounds/increments, ownership, expiry and fresh stationary state, verifies
readback and records voice change history. Defaults are not live values.
Brightness and volume have a narrow onroad allowlist; other edits require
stationary/disengaged state. Driver-monitoring disabling is excluded.
Real-time driving commands and log uploads are unsupported.

Requests expire after 30 seconds; proposals after 60 seconds. Results are bounded
and ephemeral. Duplicate delivery cannot repeat a committed write. Queued/unknown
results are not success. Cancellation does not undo an applied setting; reverting
is a new confirmed change. Navigation failure does not invalidate a verified write.

## Identity and storage

Authorization codes last two minutes and bind client/redirect/resource/S256 PKCE.
Consent uses same-origin POST and a per-flow HttpOnly/SameSite cookie. DCR accepts
only allowlisted ChatGPT callback shapes. Access tokens last one hour; rotating
refresh tokens renew a one-year idle window. Recent replay revokes the grant.
Disconnect/revocation is checked on every authenticated request.

`MCP_OAUTH_STATE_PATH` atomically stores clients and hashed access/refresh tokens
in the private connections volume. Confidential-client secrets are needed for SDK
authentication: never expose or log this file. Raw device codes, access/refresh
credentials and consent cookies are not persisted. Corrupt stores fail startup.
`MCP_STATE_PATH` persists paired capability hashes. Commands and live values are
not persisted. Without these paths, development state is RAM-only.

Legacy `/mcp` conversation pairing remains read-only for compatibility. Its old
floating web launcher/pairing UI was removed. New users use account OAuth and the
native Android toolbar. Storage is origin-specific; app and ordinary browser
sessions need separate approval. Web Disconnect revokes the local setting grant
and all account grants associated with that web connection.

## Operational limits

- Up to 1,000 paired sessions, 60 bridge creations/minute and 600 MCP requests/minute
  globally. Each visible paired tab polls every three seconds; hidden tabs pause.
- These are limits, not proof of capacity for hundreds of simultaneous users.
  300 visible tabs imply about 100 polls/second plus CORS traffic. Monitor the NAS
  before expanding beyond a small beta.
- Browser calls need the paired bearer capability and exact Origin. Origins must
  be loopback or LAN names/private IPv4 on port 7000. Origin checks supplement auth.
- Device HTTP requires a trusted LAN. Never expose port 7000 or share pairing
  codes, local storage, state files or credentials.
- ChatGPT account/voice/plugin availability and limits apply. Free-account support
  is unverified. Gemini invocation is separate from MCP compatibility; current
  US/English restrictions and unverified Live tools prevent equivalent Korean voice
  support. Google Assistant App Actions are not implemented.

## Run and deploy

Node 24: `npm ci --ignore-scripts`, `npm test`, `npm start` (loopback 18770).
Deploy `compose.yaml` behind the existing HTTPS reverse proxy to
`127.0.0.1:18770`, with `PUBLIC_HOSTNAME` set to the exact hostname. Preserve the
connections volume. This independent non-root/read-only container does not mount
upload-service data or credentials and has a 256 MiB memory limit. CPU shares are
a scheduling preference, not a hard CPU limit.

`/health` reports the service version. Refresh ChatGPT plugin metadata after tool
changes and start a new chat. Setting access also needs new OAuth consent;
refreshing metadata does not expand old grants. Rebuild web bundles in
`openpilot/selfdrive/carrot/web`. Catalog metadata is pinned in `catalog.mjs` and
the public catalog is `catalog.json`.

Tests cover real SDK/OAuth/HTTP calls, scope separation, replay/revocation, restart
persistence, expiry, bounds and ownership. Device and web suites run separately.
Publishing this relay does not update existing device software.
