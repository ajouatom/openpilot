# External model catalog — carrot-nav-blink trial, 2026-10-26

The internal CD210 model/artifacts remain unchanged. A boot-frozen selection shared by
AMD eGPU and Jetson adds N16 Nav, MD v3 and Flip Flop. Saving changes only the next-boot
preference; worker recovery and manager restarts keep the current boot selection.
The launch path freezes it before model preparation. Normal delivery remains Git Pull,
then both devices reboot with ignition off and Internet access.

## Official provenance

- N16: comma `afbf2e547e9319e2eb9f515e1e4330af63705cec`, PKL `52205044…`,
  ONNX `01effc67…`; existing artifacts and exact-N16 optimization gates retained.
- MD v3: [comma PR39087](https://github.com/commaai/openpilot/pull/39087),
  source `e86b58dc7b9fb499dd84ac6dbe514a03fa2bc428`, PKL `d4c7a3fc…` (799941965 bytes).
- Flip Flop: [comma PR39156](https://github.com/commaai/openpilot/pull/39156),
  source `85476c4bceeea005357be283f23dce90b390e724`, PKL `f0667cf6…` (775663355 bytes).
  Full official Git LFS SHA-256 verified. Official history selects `1365126a…/12864`
  and recompiles. Its input/output metadata and checkpoint match MD v3; the PKLs differ.
- Both non-NAV entries reference official ONNX
  `1365126a-e71e-4778-be00-8ab92dc82c09/12864/big_driving_supercombo.onnx`
  from [comma's ONNX repository](https://huggingface.co/commaai/openpilot_driving_models).
  SHA-256 `35976b11be3904eb78e32f7dc55a6abf8e9433ac5aa391a1998f1e9ce001b199`,
  768750198 bytes, full checksum/ONNX checker and independently parsed ModelSpec match.
  They share a Jetson engine; selecting Flip Flop does not invent a different ONNX.
  This establishes checkpoint/contract identity, not bitwise PKL/TRT execution equivalence.
- MD v3 and Flip Flop use tinygrad `02e698160eed1751738212144a7ea2cfb160b3f1`,
  the already verified MD v3 runtime archive `c7c58622…`. Official PKLs copied verbatim;
  no model-kernel compilation. Existing camera-warp/dispatch setup may still run locally.
  Runtime-specific USB error/wait handling carried from carrot-wip for this exact archive.

## Selection and storage

Reviewed catalog pins map one choice to official PKL, ONNX contract and signed Jetson
runtime. Both nav inputs are required for the NOO label. N16 supports them; MD v3 and
Flip Flop do not. NOO capability is distinct from current route validity/performance.
New compatible releases are added through reviewed Git Pull catalog updates, never
arbitrary unverified runtime downloads. Only one engine is loaded into GPU memory.

Offroad deletion protects current-boot, next-boot and installed-active eGPU identities.
Local deletion locks against delivery. Jetson deletes only allowlisted identities and
protects its signed runtime pin, loaded engine and active build; requests run only in
jetlinkd's idle/offroad path, with no work added to the inference transaction.
Shared ONNX identities remain if another protected selection needs them. Inventory is
explicitly stale after reboot/disconnection; hidden models can be restored/redownloaded.

The web-only setting is documented in localized UI and the Korean/English Carrot Web
guides. It does not introduce a Params setting or a generated setting Wiki page.
All source, releases and tests belong only to carrot-nav-blink; no carrot-wip merge.

## Validation

Full official artifact and HTTPS mirror checks pass. Initial desktop regressions: 199
passed; follow-up selection/API/corruption/alias and USB-wait checks: 54 passed (overlap).
Existing web status/name checks: 13 passed; new real-render/action structure check: 1.
Linux CI, signed bundles and physical switching results are recorded below when completed.
No road-performance or 50 ms hard-deadline claim follows from metadata or desktop tests.
