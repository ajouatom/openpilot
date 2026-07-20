# Carrot web upload

Carrot diagnostics and dashcam logs use HTTP(S) only. The default target is
**Carrot**, with **Toss** available as an additional selectable target. Both
paths use the same streamed Web API engine, so upstream Carrot upload fixes
also apply to Toss without mixing their URLs or credentials.

## Client configuration

In Carrot Web, open **Tools > Web settings > Web upload** and choose:

- **Carrot server** (default): uses `web_upload_url` and `web_upload_token`.
- **Toss server**: uses independent `toss_upload_url` and
  `toss_upload_token` values (default URL: `https://op.wjcloud.kr`).

Use **Test connection** after selecting a target. The test, dashcam files,
completion notification, and tmux diagnostics all use only the selected
target. Carrot retains its web-only tmux fallback when its token is absent;
Toss never falls back to a Carrot endpoint.

Deployments may override the saved settings with:

- `CARROT_WEB_UPLOAD_URL`
- `CARROT_WEB_UPLOAD_TOKEN`
- `CARROT_TOSS_UPLOAD_URL`
- `CARROT_TOSS_UPLOAD_TOKEN`
- `CARROT_TMUX_WEB_UPLOAD_URL` (Carrot tmux-only web fallback)
- `CARROT_WEB_UPLOAD_CONCURRENCY` (1-6, default 3)

## Required web API

Authenticated routes use `Authorization: Bearer <token>`.

- `GET /api/v1/health` returns HTTP 200 when the service and storage are ready.
- `PUT /api/v1/upload/{device}/{segment}/{filename}` accepts a streamed
  `application/octet-stream` body and returns `{"ok": true, "size": N}`.
  The client compares `size` with the bytes actually sent and retries once on
  failure or mismatch.
- `POST /api/v1/complete` accepts the completed dashcam-upload summary as JSON.
- `POST /api/v1/tmux/upload` accepts multipart fields `files[0]` (tmux log),
  optional `files[1]` (settings), and device metadata.

Every path component is percent-encoded by the client. The server must decode,
validate, and confine it to the intended storage root.

## DSM cutover

No DSM FTP option is required by the client. If the HTTPS upload service already
writes to DSM storage, DSM only needs the shared-folder permission used by that
service and the HTTPS reverse proxy/API route.

Before deleting the old DSM account:

1. Save the upload URL and token in Carrot Web.
2. Run **Test connection** and confirm success.
3. Upload one small completed segment and confirm every returned file size.
4. Trigger one tmux diagnostic and confirm it appears on the web server.
5. Delete the old FTP account, disable the DSM FTP service, and remove the 8021
   router/firewall rule if nothing else uses it.

For a self-hosted DSM reverse proxy, allow long-lived streamed PUT requests and
set the request-body limit above the largest camera file. A DSM WebDAV toggle by
itself does not implement the API contract above.
