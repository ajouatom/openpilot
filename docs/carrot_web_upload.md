# Carrot web upload

Carrot diagnostics and dashcam logs use HTTP(S) only. The runtime contains no
FTP connection, account, password, port, or fallback path.

## Upload targets

In Carrot Web, open **Tools > Web settings > Log upload** and select:

- **Carrot server** (default): `https://upload.shind0.synology.me`.
  Normal users do not enter a token. The device requests a short-lived session
  using its Dongle ID; the Carrot server binds that session to the device and
  source IP.
- **Toss server** (optional): `https://op.wjcloud.kr`. Toss keeps an independent
  URL and access token. The client never requests an automatic Carrot session
  from Toss and never falls back from Toss to a Carrot endpoint.

**Test connection** uses the currently selected target. Carrot testing checks
public health and requests a temporary test session. Toss testing sends its
configured Bearer token to the Toss health endpoint.

When **Carrot server** is selected, tmux diagnostics are also sent independently
to `https://tmux.carrotpilot.app/upload`, which creates the Discord
`carrot_logs` forum entry. The automatic onroad report includes both `tmux.log`
and `toggle_values.json`; exception and manual reports include the tmux log. A
Carrot DSM failure does not suppress this independent Carrot Logs copy.

When **Toss server** is selected, tmux diagnostics are exclusive to Toss:
`carrot_logs` and the direct Discord webhook are both skipped, and a Toss
failure never falls back to a Carrot endpoint.

Operator overrides are target-specific:

- `CARROT_WEB_UPLOAD_URL`
- `CARROT_WEB_UPLOAD_TOKEN` (private Carrot deployment escape hatch)
- `CARROT_TOSS_UPLOAD_URL`
- `CARROT_TOSS_UPLOAD_TOKEN`
- `CARROT_TMUX_WEB_UPLOAD_URL` (independent Carrot Logs endpoint)
- `CARROT_WEB_UPLOAD_CONCURRENCY` (1-6, default 3)

## Shared upload API

Both targets use the same streamed file and completion API after authentication:

- `GET /api/v1/health`
- `PUT /api/v1/upload/{device}/{segment}/{filename}`
- `POST /api/v1/complete`
- `POST /api/v1/tmux/upload`

Carrot additionally provides:

- `POST /api/v1/session`, which validates a Dongle ID and returns a random,
  short-lived token bound to that device and source IP.

The client sends `X-File-Size`, streams each file, compares the returned size
with the bytes actually sent, and retries once on failure or mismatch. Every
path component is percent-encoded.

## Carrot server protection policy

The reference Carrot receiver in `tools/carrot_upload_server` defaults to:

- 1 GiB per Dongle ID per UTC day
- 8 GiB per source IP per UTC day
- 3 concurrent uploads per device and 16 globally
- 512 MiB maximum per dashcam file and 16 MiB per tmux request
- 10 GiB free-space floor
- strict device, segment, filename, file-type, and path-confinement checks

Uploaded content has no public download route and is never executed. The DSM
container runs without root privileges, with a read-only root filesystem and a
single writable data volume.

## DSM deployment

The Carrot receiver and hardened Container Manager configuration live in
`tools/carrot_upload_server`. DSM exposes it through an HTTPS reverse proxy from
`https://upload.shind0.synology.me` to loopback `127.0.0.1:18080`. Dashcam files
retain the original `/volume1/openpilot/routes/<CarName> <DongleID>/<segment>`
layout;
tmux files retain the FTP-era
`/volume1/openpilot/<GitBranch>/<CarName> <DongleID>/` layout. Private manifests,
sessions, and quota state remain under `/volume1/openpilot/tmux/.state`.

1. Run the container on loopback `127.0.0.1:18080`.
2. Proxy `https://upload.shind0.synology.me:443` to
   `http://127.0.0.1:18080` and assign a trusted certificate.
3. Verify public health, automatic session creation, a real segment upload,
   exact returned file sizes, completion manifest, and a tmux upload.
4. Then delete the old transfer account, disable the DSM FTP service, and
   remove its router/firewall rule if nothing else uses it.

The application does not need DSM FTP, WebDAV, or a shared user credential.
DSM keeps the original remote layout. Dashcam files go to
`/volume1/openpilot/routes/<CarName> <DongleID>/<segment>`, while tmux files go
to `/volume1/openpilot/<GitBranch>/<CarName> <DongleID>/`. The private
`/volume1/openpilot/tmux/.state` directory holds manifests, sessions, and quota
state. The web receiver never scans or deletes the existing Openpilot tree.
