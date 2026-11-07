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

Relay0.6.1 fixes a fresh approval code expiring with an older unpaired browser
lease. Each newly issued account code now has its full five-minute window;
renewal invalidates its predecessor without extending the legacy chat code.
Existing authenticated connections and permissions are unchanged.

`carrot.screen` permits search/explanation/navigation. `carrot.settings` is an
additional, explicitly selected consent scope for live reads and changes. Existing
grants and refresh tokens retain their original scopes. The device must also issue a local
grant while stationary/disengaged. That grant stays in WebView storage and goes
only to same-origin device endpoints, never to the relay or ChatGPT.

Five setting tools read a selected value, prepare a change, commit a confirmed
proposal, inspect the request and cancel a prepared proposal. A clear user
setting/value command authorizes internal prepare/commit without another question. The device validates actual
values, bounds/increments, ownership and expiry, verifies
readback and records voice change history. Defaults are not live values.
Catalog unit/step is the web button increment, not an allowed-value lattice.
Absolute requests preserve any in-range integer (e.g. CruiseMaxVals0 250 -> 240,
with min=1 and unit=5); relative calculations round to the nearest integer,
with ties away from zero. Fractional absolute values are rejected.
All supported catalog edits, including driving settings, may be requested and
applied onroad. Initial authorization still requires stationary/disengaged state.
Saving does not change when each setting takes effect in the vehicle.
Driver-monitoring disabling is excluded.
Driving-record uploads are unsupported. Experimental driving requests use separate
`carrot.drive` consent and a separate stationary-issued local grant. Existing
tokens never gain this scope automatically. `VOICE_RULES.md` is loaded as MCP
instructions: one continuing conversation, no per-command button/mode switch,
no added conversational confirmation for clear current driving intents, no
invented deceleration target, and truthful acknowledgement versus completion.
Platform tool approvals still apply; driving tools are marked as state-changing.

`request_carrot_drive` supports lane_left/right, remote-equivalent speed_up,
explicit speed_down and speed_set (km/h). `get_carrot_drive_request` only observes.
Delivery expires in5s, local controller consumption in1s. Pending lane start
expires in3s, active observation in20s. The relay deduplicates identical calls
and rejects other calls within10s; one request at a time. No pending work survives
restart. Local request IDs prevent replay after ambiguous delivery. The existing
lane FSM owns motion; fresh state, BSD, lane geometry, driver priority and torque
requirements remain. Speed cannot engage assistance or impersonate attention input.
Terminal lane completion also requires observed crossing. Revocation cancels
pending preparation, not an already-running steering trajectory.
Full device restart is required; web-only restart is insufficient. No APK update.
PC controller/transport tests are not on-road voice validation.

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
  Driving-enabled tabs use a wakeable7.5s long poll followed by a1s pause; pending
  commands wake it immediately. This reduces idle requests and delivery wait,
  not speech recognition/model latency. Simultaneous-client load remains untested.
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

## Selected Carrot Web connection (0.7.1)

Android 0.2.1 reuses an app-owned approved browser capability when discovery or
selection opens another local Carrot Web address. `/bridge/inspect` is read-only
and does not mark a device online. `/bridge/relocate` requires that capability
and the previous/current local origin; it persists the destination on the same
session, preserving OAuth grants/scopes. It invalidates pending screen, settings,
driving and Tools requests and outstanding approval codes. No commands replay.
Destination selection does not require matching the old vehicle identity.
Local device permissions remain independently enforced by the selected web.
Optional permission restoration cannot create grants or block screen connection.

## Tools operations (0.7.0)

`carrot.tools` requires explicit OAuth consent and a separate stationary-issued
local grant. Existing refresh grants never acquire it. Fixed action/panel enums
exclude arbitrary shell commands, paths, commits and URLs. Prepare returns a
60-second owner-bound proposal; commit consumes it before dispatch and requires
a follow-up user confirmation. Maintenance rechecks parked/disengaged state at
dispatch. Screen recording uses explicit on/off targets, not a toggle.

Existing Tools jobs provide execution and history. Relay results contain only
action/status, never job logs or backup content. Reboot, rebuild, calibration reset
and tmux upload report request acceptance, not completion. Expired/unknown results
are never automatically retried. Job ownership is local and in-memory: after a
web-server restart, inspect local Tools history instead of reissuing a job.
File/QR/clipboard/repository-reset workflows require direct local input.
No live vehicle maintenance acceptance test is run.

## Direct setting commands (0.7.3)

An explicit target/value command authorizes a setting edit. Prepare/commit remain
internal preflight and single-use storage steps, with device bounds, expiry,
current-value rechecks and readback unchanged. The relay presents legacy device
`confirmation_required:true` as `confirmation_required:false, commit_required:true`
for prepared settings. This never commits automatically or changes OAuth/local
permissions. Ambiguous requests and AI recommendations still need clarification.
Tool maintenance confirmations and platform approvals remain separate. Refresh
plugin metadata for updated tool descriptions; no phone APK update is required.

## ChatGPT metadata and action approvals

After deploying changed tool descriptions/instructions, refresh the registered
CarrotAssist plugin's tools and test in a NEW ChatGPT conversation. Deployment
alone does not invalidate cached plugin metadata. ChatGPT action approval policy
is separate from OAuth scopes and local device grants. Users may explicitly
choose CarrotAssist > Permissions > Allow all actions in their own account; the
relay/app cannot make or verify that choice. Never mark writes as read-only to
avoid platform approval. In the 2026-11-07 live test, metadata refresh plus the
owner-selected plugin override allowed240->250 without another prompt, verified
by an independent device read. The full Android ChatGPT app's background voice
path also continued while the owner touched Carrot Web; overlay behavior differs.

## Voice request order (0.7.4)

Clear driving commands call request_carrot_drive before narration or status/catalog
lookups. Korean lane/road-lane and left/right variants share the same action.
Existing authorization, vehicle checks, expiry and no-retry rules are unchanged.
Refresh ChatGPT tool metadata after deployment; existing voice sessions may retain
old instructions. This reduces avoidable turns, not model/network latency guarantees.
