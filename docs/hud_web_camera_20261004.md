# External HUD and Carrot Web camera concurrency

On 2026-10-04, the user requested removing the external-HUD exclusion from
Carrot Web's live camera view.

- Manager now permits `carrot_webrtcd` and `carrot_vision_encoderd` with
  `ClusterHud=1`, subject to the existing CarrotVisionEnabled and onroad/car gates.
- The `/stream` proxy forwards requests with HUD enabled. Upstream ownership
  conflicts, including HTTP 409, remain intact.
- Web availability and translated hints no longer forbid HUD concurrency.
  CarrotVisionEnabled still defaults off and starting video remains explicit.
- The setting descriptions feed the existing generated Wiki workflow; Korean
  and English guides describe concurrent use and the possible extra load.

The existing road-only hardware encoder prewarms one frame and remains idle
without CarrotVisionActive. Its process/session lifetime, CPU placement and
encoder settings are unchanged, as are HUD FPS/placement, on-device camera
suppression, control, model and driver-monitoring behavior. Concurrent viewing
adds encoding, memory and networking work; existing load can therefore cause
stuttering, heat or timing pressure. This change does not establish C3/C4
performance or driving validation.

Validation: 20 browser availability/lifecycle tests, four stream proxy cases,
25 Wiki tests, user-docs and Wiki candidate validation, and web build pass.
Two actual manager policy test methods pass with extracted production predicates
and registration expressions plus in-memory Params. The full native manager
suite could not collect on Windows because params_pyx is unavailable. Proxy
tests isolate diagnostic logging and use a mocked upstream HTTP session.
