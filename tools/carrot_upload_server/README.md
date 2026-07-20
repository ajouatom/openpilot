# Carrot upload server

This is the HTTP(S) receiver for public Carrot dashcam and tmux uploads. Users
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
   ACL group, GID 101) and bind-mounts only the
   existing `/volume1/openpilot/routes` folder and the dedicated
   `/volume1/openpilot/tmux` folder. No DSM password or FTP login is passed to
   the container. Adjust the numeric UID only if this DSM account changes.
4. Add DSM reverse proxy `https://shind0.synology.me:443` to
   `http://127.0.0.1:18080` and assign a trusted certificate for that hostname.
5. Keep port 18080 bound to loopback. Do not publish the upload directory.

Verify `GET /api/v1/health`, automatic session creation, a test segment, and a
tmux upload before disabling the old transfer service and removing its account.

Dashcam files retain the original FTP-era layout at
`/volume1/openpilot/routes/<CarName> <DongleID>/<segment>`. Tmux files use
`/volume1/openpilot/tmux/<GitBranch>/<CarName> <DongleID>/<reason>-<time>-<branch>.txt`;
the extra `tmux` boundary prevents the receiver from mounting source trees.
Completion manifests and quota/session state stay in the hidden
`/volume1/openpilot/tmux/.state` directory. The receiver never scans or deletes
the existing Openpilot tree.
