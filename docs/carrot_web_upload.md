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

Uploaded content has no public index or unrestricted download route and is
never executed. An owner can explicitly create a high-entropy URL for one
route; that URL is the sole capability for downloading that route's whitelisted
logs and camera files. The DSM container runs without root privileges, with a
read-only root filesystem and a single writable data volume.

## Private route viewer

The same service includes a route vault designed for Synology hosting:

- `/` shows a branded landing page without route or device metadata.
- `/admin` requires the server-side `CARROT_ROUTE_ADMIN_KEY`. The owner can see
  all uploaded routes and their segments, play video, download individual
  files, create links with an optional expiration, and revoke existing links.
- `/s/<token>` reveals exactly the route selected when that link was created.
  Requests for a different canonical route, segment path, device, or filename
  fail. Only known Openpilot log/camera filenames are served, and symlinks are
  rejected.
- Plain share tokens are not stored. SQLite keeps the random share ID and a
  SHA-256 hash, plus route scope, creation/expiration, and revocation times.
- Pages and responses use restrictive security/no-index headers. The route
  token remains a bearer secret, so forward it only to people who should see
  the selected GPS, CAN, and camera data.

For video playback, an existing `qcamera.mp4` is streamed directly. Otherwise
the container uses ffmpeg to remux `qcamera.ts` into a fast-start MP4 without
re-encoding, then keeps it in a size-bounded cache. HTTP Range support allows
seeking and avoids loading the complete file before playback.

Each share page also exposes the comma API subset already consumed by Cabana
and PlotJuggler. It provides copyable commands that set the share-specific
`API_HOST` and route argument; no changes are made to either tool. Because the
tools accept a route plus API host rather than a custom browser URL, the command
is the link bridge.

## DSM deployment and cutover

The receiver and hardened Container Manager configuration live in
`tools/carrot_upload_server`. DSM should expose it only through an HTTPS reverse
proxy:

1. Run the container on loopback `127.0.0.1:18080`.
2. Set a strong `CARROT_ROUTE_ADMIN_KEY` and
   `CARROT_PUBLIC_BASE_URL=https://upload.shind0.synology.me` in the Container
   Manager project's `.env`. A suitable key is produced by
   `openssl rand -hex 32`; never commit or paste it into compose YAML.
3. Proxy `https://upload.shind0.synology.me:443` to
   `http://127.0.0.1:18080` and assign a trusted certificate.
4. Verify public health, owner login, route-scoped sharing/revocation, video
   seeking, automatic session creation, a real segment upload, exact returned
   file sizes, completion manifest, and a tmux upload.
5. Then delete the old transfer account, disable the DSM FTP service, and
   remove its router/firewall rule if nothing else uses it.

The application does not need DSM FTP, WebDAV, or a shared user credential.
DSM keeps the original remote layout. Dashcam files go to
`/volume1/openpilot/routes/<CarName> <DongleID>/<segment>`, while tmux files go
to `/volume1/openpilot/<GitBranch>/<CarName> <DongleID>/`. The private
`/volume1/openpilot/tmux/.state` directory holds manifests, sessions, quota
state, hashed route-share records, and the bounded MP4 cache. The upload path
never deletes existing Openpilot route data; cache eviction removes only
generated MP4 cache entries.
