# Carrot upload server

This is the HTTP(S) receiver for Carrot dashcam and tmux uploads. Users
do not create or enter tokens. A client requests a short-lived session that is
bound to its Dongle ID and source IP.

Sessions last four hours so a slow 1 GiB upload can finish without user action.

The default policy is 1 GiB per Dongle ID per UTC day with no bandwidth
throttle. Abuse and storage safeguards are enforced independently: 8 GiB per
source IP/day, three concurrent uploads per device, sixteen globally, 512 MiB
per file, a 10 GiB free-space floor, and safe path validation. Existing files
are never removed automatically.

## DSM deployment

1. Copy this directory to `/volume1/docker/carrot-upload`.
2. Create a Container Manager project from `compose.dsm.yml`.
3. The compose file runs as DSM UID 1026/GID 100 (with the DSM administrators
   ACL group, GID 101) and bind-mounts the existing `/volume1/openpilot`
   folder so tmux diagnostics retain their original branch-based path. No DSM
   password or FTP login is passed to the container. Adjust the numeric UID
   only if this DSM account changes.
4. Create a project `.env` file with a strong owner key and the externally
   visible HTTPS origin:

   ```sh
   CARROT_ROUTE_ADMIN_KEY=<64-character-random-hex-key>
   CARROT_PUBLIC_BASE_URL=https://upload.shind0.synology.me
   ```

   Generate the key with `openssl rand -hex 32`. Keep `.env` readable only by
   the DSM administrator and do not commit it.
5. Add DSM reverse proxy `https://upload.shind0.synology.me:443` to
   `http://127.0.0.1:18080` and assign a trusted certificate for that hostname.
6. Keep port 18080 bound to loopback. Do not publish the upload directory as a
   DSM shared-folder website.

Verify `GET /api/v1/health`, automatic session creation, a test segment, and a
tmux upload before disabling the old transfer service and removing its account.

Dashcam files retain the original FTP-era layout at
`/volume1/openpilot/routes/<CarName> <DongleID>/<segment>`. Tmux files use
`/volume1/openpilot/<GitBranch>/<CarName> <DongleID>/<reason>-<time>-<branch>.txt`.
Strict server-side path validation confines web writes to the expected route
and branch/device layouts. There is no public route index or unrestricted
download API. Completion manifests, share records, and quota/session state stay in the hidden
`/volume1/openpilot/tmux/.state` directory. The receiver never scans or deletes
the existing Openpilot tree.

## Route vault and sharing

- `/` is a private landing page and exposes no route names.
- `/admin` requires `CARROT_ROUTE_ADMIN_KEY` and lets the owner search every
  stored route, inspect segments/files, play qcamera video, create expiring
  links, list links, and revoke them immediately.
- A URL under `/s/<token>` is scoped to exactly one route. It cannot enumerate
  another route or use another route's file URL. The database stores only a
  SHA-256 token hash, so the full URL is shown once when it is created.
- Share and admin pages send `noindex` headers. This is capability-link access,
  not DRM: anyone who receives the full URL can read that one route until it
  expires or is revoked.
- `qcamera.ts` is remuxed without re-encoding to browser-compatible MP4 on first
  playback. MP4 files are cached under
  `/volume1/openpilot/tmux/.state/route_video_cache`, capped at 10 GiB, and all
  file/video responses support HTTP Range requests.

The share page provides ready-to-copy Cabana and PlotJuggler commands. It gives
each existing tool a token-scoped comma-compatible `API_HOST`; neither Cabana
nor PlotJuggler source code needs modification. The web share URL itself opens
in a browser, while the displayed command supplies that URL to the desktop
tool.
