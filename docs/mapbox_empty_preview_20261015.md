# Empty navigation preview after saving Mapbox keys

The reporter showed saved Public/Secret keys and a black Carrot Navigation
preview. The user confirmed that only keys had been registered; no destination
guidance had been started.

This page displays the shared route-rendering preview. Saving credentials does
not start a route or provide a general-purpose browsable map. `mapsd` runs onroad
and requires a route, usable position and fresh instructions before rendering.
Without a route it publishes a black image with status `route`. The previous web
page displayed that image, with the explanation below it (outside the supplied
crop). A fresh black image also incorrectly made the destination API report
`map_ready: true`.

The Public key is used for tiles, search and routing. A saved Secret key does not
satisfy these route/position prerequisites. The settings' Applied state indicates
storage, not successful authentication against every Mapbox API. We have not
tested the reporter's account or token; additional API/network errors after
selecting a destination remain possible.

## Correction

- Show localized waiting instructions inside the map area, including selecting a
  destination or starting external guidance, GPS wait and unavailable map data.
- Show the preview image only when its status is `ready`; release the previous
  browser image URL when returning to a waiting state.
- Distinguish offroad from waiting when no fresh preview exists.
- Report `map_ready` only for a fresh preview whose status is `ready`.

Rendering/model eligibility, map inputs and the existing 500 ms web polling
interval are unchanged. This is a web/API correction on `carrot-nav`, with no
vehicle installation or process restart.

## Validation

- Nine Python navigation API tests passed, covering destination/search behavior,
  freshness versus readiness, and absent-preview status/demand handling.
- Seven Node tests passed: four new page behavior tests and three existing
  navigation UI tests. They cover route absence, offroad, ready-to-GPS-wait image
  cleanup, and the Korean connection-error message.
- The Windows Python run bypassed only the unrelated eager `features` package
  initializer, which imports unavailable native `msgq`; the navigation module
  and its dependencies were loaded normally. This is not a full vehicle server
  integration test or a physical-browser visual check.
