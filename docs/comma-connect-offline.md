# comma Connect disabled by default

Device cloud services require the explicit environment variable `COMMA_CONNECT_ENABLED=1`.
With the variable absent (the default), or set to any other value:

| Area | Behavior |
| --- | --- |
| Device registration | Reuses the saved DongleId without contacting the server, waiting for a modem, or replacing an existing identity. A device without an ID starts as unregistered. |
| Authenticated comma API | Requests fail locally before reaching the HTTP transport, including when `API_HOST` is overridden. |
| Athena and uploader | Do not start, including direct module invocation. Connect remote access, remote SSH, requested file uploads and automatic route uploads are unavailable. |
| Error reports | The comma Sentry client is not initialized. Local crash logging remains. |
| Pairing and Prime | Pairing shows a disabled message without generating a token/QR code. Prime and firehose status are not fetched; saved Prime state is preserved but may become stale. |
| comma-hosted map tiles | Unavailable because acquiring their download URL requires authenticated device information. Without a personal Mapbox key, map rendering and the corresponding map input to navigation-aware models are unavailable. The existing mapsd unavailable-map handling remains active. |
| Driving and local data | Control code, Panda safety, calibration, saved settings, keys, local routes and pending upload queues are unchanged. Normal local log deletion still applies when storage fills. |
| Downloads and other services | Public software/model/OS downloads and Git updates remain available. Carrot services, user-configured map providers and independent SSH/Tailscale access are unchanged. |

## Applying or reverting the policy

After installing this change, **reboot the device**. An Athena daemon from the previous
version deliberately survives a manager restart, so Git Pull or a manager restart alone
does not stop an already running old daemon.

To restore the previous cloud behavior, explicitly export `COMMA_CONNECT_ENABLED=1` in
the device's launch environment and reboot. This does not enable an uploader that is
already disabled separately in `process_config.py`. Existing local routes and cached
upload requests are retained and may be uploaded after cloud access is re-enabled or
another software version is installed.

This is an application policy, not an OS firewall. Public downloads still make network
requests. Manually invoked utilities under `openpilot/tools` have separate clients and
are outside this device-service policy. It does not erase data already received by
comma, change safety compliance, or guarantee that a device will avoid future
account/upload restrictions.

## Validation

The focused tests execute production function bodies without native device dependencies
and prevent real cloud calls. They cover the API transport boundary, saved/missing
identity, service startup, cached Prime state, pairing, map tiles and error reporting. Existing
OS-download cache and Git-update tests cover the retained paths.

```sh
python -m pytest -q -c /dev/null --confcutdir=openpilot/system/tests \
  openpilot/system/tests/test_comma_connect.py \
  openpilot/system/tests/test_setup_download.py \
  openpilot/system/tests/test_agnos_download_cache.py \
  openpilot/common/tests/test_repo_update.py
```

Native integration, rendered device UI, cold boot and on-device network capture require
separate device validation before treating the policy as verified on hardware.
