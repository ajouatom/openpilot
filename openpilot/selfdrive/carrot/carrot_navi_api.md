# Carrot Navi App-to-Simulator API

Last verified against APK: 2026-07-21
Current protocol: WebSocket v2 (`protocol_version=2`, catalog revision `1`)

This document describes the protocol used by the current patched Android app to
send navigation data and media to `simulator/app.py`. It is based on the active
implementation in `target-smali/`, its Java source mirror under `carrotnavi/`,
and the simulator receiver. Legacy HTTP and WebSocket v1 endpoints are not used
by the current app.

APK verification baseline:

- Package: `com.skt.tmap.ku`
- Version: `11.2.3.3740` (`versionCode=3411`)
- Signed artifact: `target-smali/dist/CarrotNavi_v11.2.3.3740 260721.apk`
- APK SHA-256:
  `C41F7351E1C07B5669DC7284EF28C13A2605AE4F23B60C005B14E1D23B7EA4FB`
- Signature verification: APK Signature Scheme v3, one signer
- Embedded `classes21.dex` SHA-256:
  `01BA1E4DB8A3BD93E770F3E7DC53846A1EE08B36F968B2A86D4AC0D47683CE6B`.
  This is identical to the `classes21.dex` in the final unsigned
  `target-smali/dist/smali.apk`, so the active `target-smali/` protocol classes
  are the classes shipped in the verified signed APK.

## 1. Connection Overview

The normal connection sequence is:

1. The simulator broadcasts its receive address as UDP JSON on port `7705`.
2. While the control socket is unavailable, the app reads `ip` from that
   datagram and uses it for TCP port `7714`. Before the first valid datagram, the
   APK can attempt its compiled fallback host `192.168.1.33`.
3. The app opens the v2 control WebSocket and sends `requirements_query`.
4. The simulator returns a complete `subscription_manifest` containing all 28
   catalog entries, including disabled entries.
5. The app sanitizes the manifest, sends `manifest_applied`, and opens one
   WebSocket for every enabled item.
6. JSON items are sent as text frames. Images and map render output are sent as
   binary frames with a fixed 40-byte `CNV2` header.

All timestamps are Unix epoch milliseconds. Coordinates are WGS84 latitude and
longitude. The protocol uses plain `ws://` on the local network; it currently
has no TLS, authentication, pairing token, or payload encryption.

### Ports

| Port | Protocol | Direction | Purpose |
| --- | --- | --- | --- |
| `7705` | UDP | Simulator -> app | Receiver discovery only |
| `7714` | TCP/WebSocket | Bidirectional | Control, JSON, image, and render streams |

### Discovery datagram

The app listens for one JSON datagram at a time on UDP `7705`:

```json
{
  "ip": "192.168.1.33",
  "navi_debug": 0
}
```

`ip` becomes the WebSocket host. `navi_debug` is retained for legacy aggregate
state diagnostics and does not create a separate v2 item. A missing or empty
`ip` is rejected. Each discovery receive has a 5-second timeout. The process
maintenance loop requests discovery whenever the control socket is missing or
not open (and also after three counted control failures), then waits 1 second
before its next pass. A valid datagram replaces the compiled fallback host.

## 2. WebSocket Endpoints

| Endpoint | Frame type sent by app | Purpose |
| --- | --- | --- |
| `/api/navi/ws/v2/control/{app_version}` | JSON text | Negotiation, status, commands, diagnostics, and metrics |
| `/api/navi/ws/v2/json/{session_id}/{name}` | JSON text | One named structured-data item |
| `/api/navi/ws/v2/image/{session_id}/{name}` | Binary | One named PNG/CLEAR item |
| `/api/navi/ws/v2/render/{session_id}/{name}` | Binary | One named JPEG or H.264 render item |

The app is the WebSocket client. Client frames are RFC 6455 masked. The control
socket sends a WebSocket ping every 15 seconds and supports pong handling. A
failed socket is retried after 2 seconds.

## 3. Control Negotiation

Every app-to-simulator control JSON message starts with:

```json
{
  "type": "message_type",
  "protocol_version": 2,
  "timestamp_ms": 1783920000000
}
```

### 3.1 `requirements_query` (app -> simulator)

The app sends this immediately after the control socket opens and repeats it
every second until a manifest is accepted.

```json
{
  "type": "requirements_query",
  "protocol_version": 2,
  "timestamp_ms": 1783920000000,
  "app_version": "11.2.3.3740",
  "catalog_revision": 1,
  "limits": {
    "max_binary_frame_bytes": 8388608,
    "max_total_bitrate_kbps": 12000
  },
  "streams": [
    {
      "kind": "json",
      "name": "vehicle",
      "schema_version": 1,
      "nullable": true,
      "supported_params": {
        "delivery_mode": ["on_change", "periodic", "on_change_with_heartbeat"],
        "interval_ms": {"min": 200, "max": 5000, "default": 1000}
      }
    }
  ]
}
```

The actual `streams` array contains the complete 28-item catalog listed below.

### 3.2 `subscription_manifest` (simulator -> app)

The simulator must return every catalog item exactly once. Disabled items must
remain in the array with `enabled=false`.

```json
{
  "type": "subscription_manifest",
  "protocol_version": 2,
  "session_id": "9f73c7b8c3132f10",
  "revision": 12,
  "metrics_enabled": true,
  "streams": [
    {
      "kind": "json",
      "name": "vehicle",
      "schema_version": 1,
      "stream_handle": 1,
      "enabled": true,
      "params": {
        "delivery_mode": "on_change",
        "interval_ms": 1000,
        "stale_timeout_ms": 10000
      }
    }
  ]
}
```

Manifest validation rules:

- `session_id`: `[A-Za-z0-9_-]{8,64}`.
- `revision`: `1..4294967295`; it must increase within a session.
- `streams`: exactly 28 entries with no duplicate kind/name or handle.
- `stream_handle`: unique `1..4294967295`.
- `schema_version`: currently `1` for every item.
- Enabled render bitrate total: at most `12000` kbps.
- Total binary WebSocket message: at most 8 MiB, including the 40-byte header.
- Both crossroad image streams, including disabled catalog entries, must resolve
  to the same `theme`.

Unknown JSON fields are ignored. Values outside supported ranges are generally
clamped and returned in the effective configuration. An invalid, incomplete,
duplicate, stale, or excessive-bitrate manifest is rejected.

### 3.3 Manifest result (app -> simulator)

On success:

```json
{
  "type": "manifest_applied",
  "protocol_version": 2,
  "timestamp_ms": 1783920000100,
  "session_id": "9f73c7b8c3132f10",
  "revision": 12,
  "effective_config": [
    {
      "kind": "json",
      "name": "vehicle",
      "schema_version": 1,
      "stream_handle": 1,
      "enabled": true,
      "params": {
        "delivery_mode": "on_change",
        "interval_ms": 1000,
        "stale_timeout_ms": 10000
      }
    }
  ]
}
```

On failure:

```json
{
  "type": "manifest_rejected",
  "protocol_version": 2,
  "timestamp_ms": 1783920000100,
  "revision": 12,
  "error": "manifest must contain the full catalog"
}
```

After a successful manifest, the app also sends `resource_status`.

## 4. Catalog And Stream Parameters

Fresh simulator configuration currently enables all 28 items. Enablement is a
manifest policy, not a requirement of the wire protocol.

### JSON items (13)

| Name | `value` content |
| --- | --- |
| `vehicle` | Vehicle position and motion |
| `guidance_current` | Current turn-by-turn guide point |
| `guidance_next` | Next turn-by-turn guide point |
| `lane_current` | Current/bottom lane guidance |
| `lane_ahead` | Array of ahead/top lane guidance groups |
| `speed` | Current, road-limit, safety, and section speed data |
| `traffic_signal` | Signal visibility, countdown, lights, and movement states |
| `crossroad` | Crossroad visibility and image metadata |
| `route` | Route progress and compact route polyline |
| `navigation_status` | Derived guidance lifecycle state |
| `app_status` | Foreground, focus, map visibility, and capture availability |
| `camera_state` | Current app map camera; nullable when unavailable |
| `composition_state` | Availability of captured UI layers and crossroad images |

JSON parameters:

| Parameter | Values | Default | Current behavior |
| --- | --- | --- | --- |
| `delivery_mode` | `on_change`, `periodic`, `on_change_with_heartbeat` | `on_change` | `on_change` evaluates dirty events immediately and sends changed values only; `periodic` sends every interval; heartbeat mode sends changes immediately and repeats the current value every interval |
| `interval_ms` | `200..5000` | `1000` | Periodic/heartbeat send interval; for `on_change`, a maintenance comparison interval for TTL expiry and sources without complete callbacks |
| `stale_timeout_ms` | `1000..120000` | `10000` | Sanitized and echoed; source-specific expiry currently remains authoritative |

### Image items (14)

| Name | Content |
| --- | --- |
| `tbt_current_compact` | SHORT current TBT PNG rendered from the app layout |
| `tbt_current_full` | FULL current TBT PNG rendered from the app layout |
| `tbt_next` | Next TBT PNG |
| `traffic_signal` | Traffic-signal UI PNG |
| `lane_top` | Ahead/top lane UI PNG |
| `lane_bottom` | Current/bottom lane UI PNG |
| `safety_primary` | Primary safety UI PNG |
| `safety_secondary` | Secondary safety UI PNG |
| `safety_section` | Section-speed UI PNG |
| `crossroad_minimized` | Small crossroad PNG |
| `crossroad_expanded` | Large crossroad PNG |
| `center_tbt_icon` | Center guidance icon PNG |
| `center_tbt_text` | Center guidance text PNG |
| `center_tbt_fee` | Center toll/fee PNG |

There is no `image:tbt_current` APP-compatibility item.

Image parameters:

| Parameter | Values | Default | Current behavior |
| --- | --- | --- | --- |
| `format` | `png` | `png` | Fixed |
| `max_fps` | `1..30` | `5` | Upper send rate for changes/CLEAR |
| `stale_timeout_ms` | `1000..120000` | `15000` | Sanitized and echoed; the current image worker does not consult it |
| `theme` | `auto`, `light`, `dark` | `auto` | Only on both `crossroad_*` items |

Activity-backed images expire after 10 seconds in the current sender. Both
`crossroad_*` streams skip that age check entirely. They change or CLEAR only
when their producer explicitly updates or clears the image store; neither the
manifest `stale_timeout_ms` nor expiry of `json:crossroad` independently emits a
binary CLEAR.

The transport asks the main thread to refresh requested UI sources every 500
ms, and each Activity View capture is coalesced to at most once per 500 ms per
slot. Therefore `max_fps` is only a send-rate ceiling; setting it above 2 does
not increase the current Activity-backed PNG capture rate. Captured/downloaded
UI bitmaps are limited to 4,194,304 pixels and encoded PNGs to 1 MiB. The wire
protocol still permits a complete binary message up to 8 MiB.

### Render item (1)

| Name | Content |
| --- | --- |
| `map_main` | Independent VSM map, route, and native vehicle marker only |

`map_main` does not contain TBT, lane, traffic-signal, safety, center-TBT, or
crossroad overlays. Those are separate image streams.

| Parameter | Range/values | Default |
| --- | --- | --- |
| `composition` | Fixed `map_route_vehicle` | `map_route_vehicle` |
| `width` | `160..1280` | `960` |
| `height` | `120..720` | `540` |
| `dpi` | `120..640` | `360` |
| `fps` | `1..60` | `5` |
| `codec` | `h264`, `jpeg` | `h264` |
| `jpeg_quality` | `30..95` | `75` |
| `h264_bitrate_kbps` | `1..12000` | `3000` |
| `h264_keyframe_interval_sec` | `1..10` | `2` |
| `camera_mode` | `app_sync`, `custom` | `app_sync` |
| `map_theme` | `auto`, `light`, `dark` | `auto` |
| `map_type` | `normal`, `satellite` | `normal` |
| `zoom` | `0..18.999` | `11.0` |
| `tilt` | `0..100` | `50.0` |
| `bearing` | Input is normalized to `0..<360` | `0.0` |
| `follow_vehicle_bearing` | Boolean | `true` |
| `fov` | `20..90` | `40.0` |
| `screen_center_y_ratio` | `0.1..0.95` | `0.8` |
| `follow_vehicle` | Boolean | `true` |
| `center_latitude` | `-90..90` or `null` | `null` |
| `center_longitude` | `-180..180` or `null` | `null` |
| `stale_timeout_ms` | `1000..30000` | `5000` |

H.264 dimensions are rounded down to even values. `stale_timeout_ms` is retained
in effective configuration but currently does not stop the render worker.
For `map_type=normal`, `map_theme` selects the TMAP driving day/night style.
MAP MAIN also loads the app's `theme_navi_day.json` / `theme_navi_night.json`
object themes. Traffic no-data route segments therefore use the app's blue
`#3673ee` by day or `#00a2ff` by night instead of the renderer's gray default,
while known traffic segments retain their green/yellow/red congestion colors.
For `map_type=satellite`, MAP MAIN uses `SAM_DRIVE:DEFAULT`; `map_theme` still
selects the route object palette and the independent CROSSROAD image theme.
The route palette is already rasterized into JPEG/H.264 MAP MAIN pixels. There
is no route-color field in `json:route` or the `CNV2` header, and receivers must
not replace the decoded blue/traffic colors with their own fallback palette.

### Known capability-advertisement difference

The current `requirements_query.supported_params` is narrower than the manifest
sanitizer: it does not advertise every accepted stale, quality, bitrate, camera,
or crossroad-theme parameter. It also still advertises old render overlay
selection names. Those old selectors are ignored and are absent from the
effective `map_route_vehicle` render configuration. Implement receivers against
the effective manifest contract above, not those legacy advertised selectors.

The APK accepts `fps=1..60` and `h264_bitrate_kbps=1..12000` as shown above.
The bundled simulator currently narrows its own generated configuration to
`fps=1..30` and `h264_bitrate_kbps=256..12000`; those receiver-side policies do
not change the APK sanitizer's accepted ranges.

## 5. JSON Item Messages

Each enabled JSON item has its own WebSocket and sends one `item_update` object
per message.

```json
{
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "9f73c7b8c3132f10",
  "manifest_revision": 12,
  "schema_version": 1,
  "kind": "json",
  "name": "vehicle",
  "stream_handle": 1,
  "sequence": 37,
  "source_timestamp_ms": 1783920000200,
  "sent_at_ms": 1783920000201,
  "present": true,
  "value": {
    "lat": 37.56652,
    "lon": 126.97802,
    "heading_deg": 92,
    "speed_kph": 48,
    "road_name": "Sejong-daero",
    "virtual_gps": false
  }
}
```

Absent value:

```json
{
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "9f73c7b8c3132f10",
  "manifest_revision": 12,
  "schema_version": 1,
  "kind": "json",
  "name": "guidance_next",
  "stream_handle": 3,
  "sequence": 8,
  "source_timestamp_ms": 1783920000200,
  "sent_at_ms": 1783920000201,
  "present": false,
  "value": null,
  "reason": "source_absent"
}
```

`sequence` is per item worker and increments only after a successful send. A
worker replaced by a manifest parameter/handle change may restart at `1`.
Structured `value` objects are extensible: receivers must ignore unknown fields.
The nullable `speed.sdi_secondary` addition remains schema version `1` and does
not change the meaning of the existing `speed.sdi` field.

### 5.1 Delivery and timestamp semantics

For `on_change` and `on_change_with_heartbeat`, known APK source callbacks mark
their dependent item names dirty and wake the shared supervisor immediately.
The supervisor's 50 ms wait is a fallback maintenance wait, not a mandatory
event delay. Core dirty sources include RGData, traffic-signal object/state/UI
counter updates, route-vertex updates, main-map registration, and composition
changes. A dirty evaluation that serializes to the same value is acknowledged
without sending or advancing `sequence`.

Not every TMAP-derived value exposes a complete callback. The `interval_ms`
maintenance evaluation remains necessary for time-driven TTL/countdown changes,
`app_status`, and manual `camera_state` changes. It sends nothing in
`on_change` mode when the serialized value is unchanged.

For JSON items, `source_timestamp_ms` is the APK's snapshot/evaluation time
passed to the worker. It is not an upstream TMAP event timestamp.
`sent_at_ms` is captured while constructing the outgoing envelope.

### 5.2 Structured value fields

All fields below are nullable unless naturally Boolean/numeric in the derived
status maps.

| Value | Fields |
| --- | --- |
| `vehicle` | `lat`, `lon`, `heading_deg`, `speed_kph`, `road_name`, `virtual_gps` |
| Guide point | `distance_m`, `time_sec`, `turn_type`, `road_name`, `main_text`, `near_direction`, `mid_direction`, `far_direction`, `point.{lat,lon}` |
| Lane | `count`, `distance_m`, `visible`, `lane_play`, `current_lane`, `turn_code`, `turn_info[]`, `etc_info[]`, `available[]`, `guide_line_color`, `road_category`, `voice_code` |
| `speed` | `current_kph`, `road_limit_kph`, `sdi`, `sdi_secondary`, `section` |
| `speed.sdi`, `speed.sdi_secondary` | `type`, `distance_m`, `speed_limit_kph`, `section_type`, `block_type`, `block_speed_kph`, `block_distance_m`, `block_average_kph`, `block_time_sec`, `point.{lat,lon}`. `sdi` is the valid first-normal projection or its `nSdiPlus*` fallback. `sdi_secondary` is the valid second-normal projection, or the Plus projection when the first normal occupies `sdi`. |
| `speed.section` | `active`, `speed_limit_kph`, `average_kph`, `overall_average_kph`, `remaining_distance_m`, `remaining_time_sec`, `progress`, `suspended`, `off_route` |
| `traffic_signal` | `visible`, `distance_m`, `last_update_age_ms`, `source`, `point`, `lights`, `movements`, `ui_counter` |
| Signal light | `on`, `remain_sec`; keys are `red`, `left`, `green`, `right`, `uturn` |
| Signal movement | `state`, `code`, `remain_sec`; keys are `bicycle`, `bus`, `left`, `pedestrian`, `right`, `straight`, `uturn` |
| `crossroad` | `visible`, `distance_m`, `image_code`, `image_url` |
| `route` | `remain_distance_m`, `remain_time_sec`, `moved_distance_m`, `moved_time_sec`, `total_distance_m`, `polyline[].{lat,lon}`; polyline is capped at 256 points |
| `navigation_status` | `mode` (`idle`, `guiding`, `off_route`), `guidance_active`, `off_route`, `route_present` |
| `app_status` | `foreground`, `window_focused`, `main_map_visible`, `ui_capture_available` |
| `camera_state` | `camera_mode` (`app_sync` or `background`), `center_latitude`, `center_longitude`, `view_level`, `view_sub_level`, `tilt`, `bearing`, `screen_center_x`, `screen_center_y`, `fov` |
| `composition_state` | `generation`, `tbt_current`, `tbt_next`, `traffic_signal`, `lane_top`, `lane_bottom`, `safety_primary`, `safety_secondary`, `safety_section`, `center_tbt_icon`, `center_tbt_text`, `center_tbt_fee`, `crossroad_active`, `crossroad_minimized`, `crossroad_expanded`, `vehicle`, `upper_lane_available`, `upper_lane_composited` |

Do not use numeric truthiness to decide whether an enum-like field exists.
Current-lane `road_category=0` and SDI `type=0` are both valid APK values.

`speed.road_limit_kph`, when present, is already a validated kph value emitted
by the APK. A raw value in `1..200` is accepted only when it is a multiple of 10.
For a raw value above 200, the APK attempts `(raw - 20) / 10`, requires exact
integer division, then applies the same `10..200` and multiple-of-10 checks.
Examples are `320 -> 30`, `520 -> 50`, and `1020 -> 100`; invalid values such as
`300` are omitted.

### 5.3 SDI projection and fallback

The sender chooses SDI events in this order. A normal slot counts as present
only when its projection contains at least one emitted field:

| Output field | First choice | Fallback |
| --- | --- | --- |
| `speed.sdi` | Valid projection of normal `sdiInfo[0]` | Valid `nSdiPlus*` projection when slot 0 is absent or projects to no fields |
| `speed.sdi_secondary` | Valid projection of normal `sdiInfo[1]` | Valid `nSdiPlus*` projection when slot 0 occupies `speed.sdi` and slot 1 is absent or projects to no fields |

This fallback is required because the app can display a safety icon from
`nSdiPlus*` even when a normal slot is absent or empty. The APK emits normal and
Plus `type` values when they are nonnegative: `type=0` is a valid signal-speed
camera and must not be interpreted as missing, while negative type sentinels
are omitted. Other optional SDI numeric members are emitted only when positive.
Raw `type=22` is the current TMAP speed-bump code. A live speed-bump update has
this shape:

```json
{
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "9f73c7b8c3132f10",
  "manifest_revision": 12,
  "schema_version": 1,
  "kind": "json",
  "name": "speed",
  "stream_handle": 6,
  "sequence": 91,
  "source_timestamp_ms": 1783920000200,
  "sent_at_ms": 1783920000201,
  "present": true,
  "value": {
    "current_kph": 0,
    "road_limit_kph": 30,
    "sdi": {
      "type": 22,
      "distance_m": 93,
      "point": {
        "lat": 37.432582119865074,
        "lon": 127.15066060330007
      }
    }
  }
}
```

Unused SDI members and `sdi_secondary` are omitted rather than serialized as
`null`. When the event disappears, the next `speed` value omits that event; if
the complete speed group becomes absent, the item sends the normal
`present=false` tombstone.

### 5.4 Freshness and background behavior

Source cache expiry is currently 6 seconds for live vehicle/guidance/speed/
crossroad groups, 30 seconds for route, and 60 seconds for traffic signal.

The v2 control and item workers are process-owned rather than Activity-owned.
While the Android process remains alive and the control socket stays connected,
stopping `TmapNaviActivity` does not close the session or enabled item sockets:

- Refreshed JSON sources, including `speed.sdi` speed-bump data, remain
  available. With `delivery_mode=on_change`, an unchanged value does not advance
  its sequence; `periodic` and `on_change_with_heartbeat` continue interval sends.
- `app_status.foreground` becomes `false`; consumers must use that field instead
  of inferring foreground state from socket activity.
- `map_main` continues from the independent render worker and retains the
  selected map/object theme in the background.
- Activity-backed PNG items that are not refreshed can emit CLEAR `expired`
  after 10 seconds. The two `crossroad_*` images are exempt from this fixed
  timeout and remain cached until an explicit producer update or clear. The
  6-second structured `crossroad` expiry can therefore occur without a binary
  CLEAR; consumers that compose a navigation UI should use `json:crossroad`
  (and, where relevant, `composition_state.crossroad_active`) as the visibility
  gate for the cached PNG.

Android process termination and control-socket loss are outside this continuity
guarantee; normal discovery and WebSocket reconnection rules then apply.

## 6. Binary Image And Render Messages

Every binary WebSocket message is:

```text
40-byte big-endian CNV2 header || payload bytes
```

The payload is never Base64 and has no per-frame JSON metadata.

### 6.1 Header layout

| Offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| `0` | 4 | ASCII | Magic `CNV2` |
| `4` | 1 | uint8 | Protocol version, `2` |
| `5` | 1 | uint8 | Message type |
| `6` | 1 | uint8 | Format code or CLEAR reason code |
| `7` | 1 | uint8 | Flags bit field |
| `8` | 4 | uint32 | `stream_handle` |
| `12` | 4 | uint32 | `manifest_revision` |
| `16` | 8 | int64 | Per-stream `sequence` |
| `24` | 8 | int64 | `source_timestamp_ms` |
| `32` | 4 | uint32 | Payload length in bytes |
| `36` | 2 | uint16 | Width |
| `38` | 2 | uint16 | Height |

Equivalent Python format: `struct.Struct(">4sBBBBIIQQIHH")`.

Binary `sequence` advances once per successfully sent binary message, not once
per rendered video frame. An H.264 callback containing both codec config and an
access unit therefore consumes two consecutive sequence values. For an image or
render frame, `source_timestamp_ms` is the producer capture timestamp. A CLEAR
uses the stored source timestamp when one exists, otherwise the worker's current
time.

### 6.2 Message and format codes

| Message type | Value | Format/reason | Payload |
| --- | ---: | --- | --- |
| `IMAGE_FRAME` | `1` | `1=PNG`, `2=JPEG` | Complete image bytes |
| `VIDEO_CONFIG` | `2` | `3=H264_ANNEX_B` | Annex-B SPS/PPS NAL units |
| `VIDEO_ACCESS_UNIT` | `3` | `3=H264_ANNEX_B` | Annex-B access unit |
| `CLEAR` | `4` | CLEAR reason below | Empty |

Flags: `1=KEYFRAME`, `2=DISCONTINUITY`. The current sender uses KEYFRAME for
PNG/JPEG, H.264 config, and H.264 keyframes. DISCONTINUITY is reserved but is not
currently emitted.

CLEAR reason codes:

| Code | Name |
| ---: | --- |
| `1` | `source_absent` |
| `2` | `cleared` |
| `3` | `expired` |
| `4` | `passed` |
| `5` | `invalid` |

A CLEAR packet must have a zero payload length and `width=height=0`.

### 6.3 Render codec behavior

- JPEG mode sends one `IMAGE_FRAME` with format `JPEG` per frame.
- H.264 mode sends SPS/PPS as `VIDEO_CONFIG` when present, followed by each
  access unit as `VIDEO_ACCESS_UNIT` on the same item socket.
- H.264 payloads use Annex-B 3-byte or 4-byte start codes.
- The receiver must retain the latest codec config and prepend/use it when
  initializing or resynchronizing its decoder.
- H.264 access units are delivered serially. The sender may drop raw captures
  before encoding under load, but it does not drop already encoded P-frames.

## 7. Other App-to-Simulator Control Messages

| `type` | Additional fields | When sent |
| --- | --- | --- |
| `resource_status` | `heap_used_bytes`, `heap_max_bytes`, optional `thermal_status` | After manifest acceptance |
| `effective_config` | `session_id`, `revision`, `streams` | Reply to `effective_config_query` |
| `time_sync_response` | `request_id`, `simulator_timestamp_ms`, `app_received_at_ms`, `app_sent_at_ms` | Reply to `time_sync_request` |
| `diagnostics_response` | `request_id`, `session_id`, `revision`, `streams[]` metrics | Reply to `diagnostics_query` |
| `command_result` | `request_id`, `command`, `success` | Reply to `keyframe_request` or `resync_request` |
| `stream_status` | `kind`, `name`, `stream_handle`, `revision`, `state`, `reason`, optional `encoder` | Stream opening/running/error transition |
| `stream_end` | `kind`, `name`, `stream_handle`, `revision`, `reason` | Disable, replacement, or session/end-point teardown |
| `stream_metrics` | Per-stream metric fields below | Every 5 seconds when `metrics_enabled=true` |
| `protocol_error` | `code`, `recoverable`, `message`, optional stream identity | Invalid control or item failure |

Per-stream metrics fields are `kind`, `name`, `stream_handle`, `revision`,
`sent`, `dropped`, `sent_bytes`, `errors`, `queue_depth` (currently `0`), and
`last_sent_at_ms`. Render metrics additionally contain `encoder`, `codec`, and
`requested_fps`.

Error messages are truncated to 256 characters. A received `protocol_error` is
recorded without replying with another error, preventing a feedback loop.

## 8. Simulator-to-App Control Messages

These messages are not navigation payloads, but a compatible receiver needs
them to control the app:

| `type` | Required/used fields |
| --- | --- |
| `subscription_manifest` | Full manifest described above |
| `effective_config_query` | No additional field |
| `time_sync_request` | `request_id`, `simulator_timestamp_ms` |
| `keyframe_request` | `request_id`, optional `name` (defaults to `map_main`) |
| `resync_request` | `request_id`, either `name` or `names[]`; matching is by name across kinds, and an empty name matches all enabled workers |
| `diagnostics_query` | `request_id` |
| `catalog_query` | No additional field; app replies with `requirements_query` |
| `protocol_error` | Error report; deliberately receives no response |

Any other control `type` produces a recoverable
`protocol_error(code="unsupported_control_message")`.

## 9. Receiver Validation Checklist

A compatible simulator should:

1. Validate the control frame as JSON text and negotiate the exact 28-item
   catalog.
2. Keep session ID, manifest revision, stream handle, kind, name, and path
   consistent for every item.
3. Reject duplicate/non-increasing sequences within the same worker revision,
   while allowing a replaced worker to restart its sequence.
4. Validate `CNV2`, payload length, dimensions, message type, format, PNG/JPEG
   signatures, and H.264 Annex-B start codes before decoding.
5. Treat JSON `present=false` and binary CLEAR as tombstones and remove stale
   display data immediately.
6. Gate cached `crossroad_*` PNG display with the structured crossroad state;
   its 6-second expiry does not itself generate a binary CLEAR.
7. Apply new manifest revisions atomically enough that old sockets cannot write
   into the new item identity.
8. Reply to WebSocket pings and tolerate reconnects without assuming the prior
   session survives.

## 10. Legacy Interfaces

`simulator/app.py` still contains HTTP diagnostics and WebSocket v1 compatibility
routes. The current app transport is `CarrotV2Transport`; it does not call those
HTTP or v1 WebSocket routes. They are outside this API contract.

## 11. Implementation References

- Active generated target protocol:
  `target-smali/smali_classes21/com/skt/tmap/engine/navigation/util/CarrotV2Protocol*.smali`
- Active generated target transport:
  `target-smali/smali_classes21/com/skt/tmap/engine/navigation/util/CarrotV2Transport*.smali`
- Java protocol source:
  `carrotnavi/app/src/main/java/com/skt/tmap/engine/navigation/util/CarrotV2Protocol.java`
- Java transport source:
  `carrotnavi/app/src/main/java/com/skt/tmap/engine/navigation/util/CarrotV2Transport.java`
- Structured value source:
  `carrotnavi/app/src/main/java/com/skt/tmap/engine/navigation/util/CarrotUiStateData.java`
- Discovery and item source bridge:
  `carrotnavi/app/src/main/java/com/skt/tmap/engine/navigation/util/CarrotUtilj.java`
- Simulator receiver and manifest authority: `simulator/app.py`
- Receiver tests: `simulator/tests/test_protocol.py`
- Compatible OpenPilot raw receiver:
  `openpilot/openpilot/selfdrive/carrot/carrot_navi.py`
- OpenPilot receive-boundary tests:
  `openpilot/openpilot/selfdrive/carrot/tests/test_carrot_navi.py`
