# Carrot Routes vault and radar review

This directory maintains the deployed `carrot-route-vault` service, including
its existing upload receiver, route sharing, video player, and model downloads.
The older standalone receiver remains under `tools/carrot_upload_server`.

Every shared or direct upload-result page automatically loads the selected
segment into a browser radar reviewer. Its server-side JSON adapter uses
`ProductionDPathSelector`, the same production controller replay as the desktop
reviewer. The page shows radar points, model paths/lanes, vision leads, recorded
and recalculated Lead 1/2, per-track dPath diagnostics, and a lead-distance graph.
Video and radar share the recorded qcamera timing; when timing or video is
unavailable, the radar timeline operates independently and says so explicitly.
Browser source/sensitivity controls only change replay analysis.

Replay runs in a separate Python process, with one active job and at most four
pending jobs per server. Requests poll for completion. Results are gzip-cached
by source path, size, nanosecond mtime, replay code fingerprint, and options;
cache storage is capped at 2 GiB. Every data request rechecks its route scope or
share token, including cache hits. `rlog` is preferred; `qlog` fallback is labelled.

## Build and update the existing DSM project

Run `python tools/carrot_route_vault/build_bundle.py <new-output-directory> --ref HEAD`
after committing changes. This creates a Docker build context using committed
replay Python sources, the full cereal schemas, and DBC data. Uncommitted vehicle
changes and model artifacts are excluded. It also copies this viewer's source.

Back up the existing `/volume1/docker/carrot-route-vault` code and compose files,
then copy the generated context there. Keep existing secrets, database paths,
volume mounts, loopback port, and owner settings. Build with `Dockerfile.vault`,
give the new image a distinct tag, and raise the container memory limit to 2 GiB
for full-log decoding. Use DSM Container Manager to build/recreate the existing
project. Verify health, an existing route link, radar preparation/seek, and the
existing `/models/<family>/manifest.json` endpoint before considering the rollout
complete. Rollback uses the saved compose file and previous image.

Local tests: `python -m pytest -n 0 --confcutdir=tools/carrot_route_vault tools/carrot_route_vault/tests`.

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

1. Build the context described above and copy it to `/volume1/docker/carrot-route-vault`.
2. Preserve the existing project's compose configuration when updating it.
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
- Successful Carrot Web uploads can publish a permanent direct URL under
  `/routes/<quoted CarName DongleID>/<route--segment>`. A consecutive range
  uses the normal end-exclusive openpilot slice, for example `--10:13` exposes
  segments 10, 11, and 12. These URLs intentionally require no login, expose
  only the named segment or range, and never provide a public directory index.
- Share and admin pages send `noindex` headers. This is capability-link access,
  not DRM: anyone who receives the full URL can read that one route until it
  expires or is revoked.
- `qcamera.ts` is remuxed without re-encoding to browser-compatible MP4 on first
  playback. Before playback, the shared page shows a cached JPEG preview made
  from the first seconds of the same qcamera video. MP4 and preview files are cached under
  `/volume1/openpilot/tmux/.state/route_video_cache`, capped at 10 GiB, and all
  file/video responses support HTTP Range requests.
- Driver-facing `dcamera` files are rejected by every manifest, file, and API
  endpoint.

The share page provides ready-to-copy Cabana, PlotJuggler, and JotPluggler
commands. It gives each existing tool a route- or segment-scoped
comma-compatible `API_HOST`; none of the desktop tool sources require a
modification. The web URL itself opens in a browser, while the displayed
command supplies its `/api` endpoint and the same single-segment or consecutive
slice to the desktop tool.
