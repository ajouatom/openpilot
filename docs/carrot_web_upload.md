# Carrot web upload

Carrot diagnostics and dashcam logs use HTTP(S) only. The runtime contains no
FTP connection, account, password, port, or fallback path.

## User experience

There is no account setup and no token field. A device requests a short-lived
upload session automatically using its Dongle ID, and the server binds that
session to the Dongle ID and source IP. The same server handles dashcam files
and tmux diagnostics.

Tmux diagnostics are also sent independently to
`https://tmux.carrotpilot.app/upload`, which creates the Discord `carrot_logs`
forum entry. The automatic onroad report includes both `tmux.log` and
`toggle_values.json`; exception and manual reports include the tmux log. A DSM
failure does not redirect or suppress this independent Carrot Logs copy.

In Carrot Web, **Tools > Web settings > Web upload** contains only:

- Upload server (default: `https://upload.shind0.synology.me`)
- Test connection

An operator may override the base URL with `CARROT_WEB_UPLOAD_URL`. The
`CARROT_WEB_UPLOAD_TOKEN` environment variable remains an operator-only escape
hatch for a private server; it is not stored or shown in Carrot Web.

Uploads use three concurrent HTTPS streams per device by default, configurable
from one to six with `CARROT_WEB_UPLOAD_CONCURRENCY`. There is no bandwidth
throttle.

## API and protection policy

- `GET /api/v1/health` is public and reports readiness and limits.
- `POST /api/v1/session` validates a Dongle ID and returns a random,
  short-lived session bound to that device and source IP.
- `PUT /api/v1/upload/{device}/{segment}/{filename}` streams one file. The
  client sends `X-File-Size`; the server writes to a temporary file, verifies
  the exact size, fsyncs it, and atomically renames it.
- `POST /api/v1/complete` stores the completed dashcam-upload manifest.
- `POST /api/v1/tmux/upload` accepts the tmux log and optional settings file as
  multipart data.

The deployed defaults are:

- 1 GiB per Dongle ID per UTC day
- 8 GiB per source IP per UTC day
- no transfer-speed limit
- 3 concurrent uploads per device and 16 globally
- 512 MiB maximum per dashcam file and 16 MiB per tmux request
- 10 GiB free-space floor; existing files are never deleted automatically
- strict device, segment, filename, file-type, and path-confinement checks

Uploaded content has no public download route and is never executed. The DSM
container runs without root privileges, with a read-only root filesystem and a
single writable data volume.

## DSM deployment and cutover

The receiver and hardened Container Manager configuration live in
`tools/carrot_upload_server`. DSM should expose it only through an HTTPS reverse
proxy:

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
