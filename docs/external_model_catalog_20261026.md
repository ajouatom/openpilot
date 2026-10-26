# External model catalog — carrot-nav-blink trial, 2026-10-26

## Correction and retirement, 2026-10-27

MD v3 is retired. The selectable list is N16 Nav and Flip Flop. Its metadata
remains only for old-boot compatibility and exact unused PKL cleanup. A retained
Flip Flop ONNX is never deleted through the retired alias, even when N16 is selected.
Earlier statements below about an identical official MD/Flip ONNX were incorrect:
the same Hugging Face path was overwritten on 2026-10-24. The 2026-10-16 file was
792542056 bytes/SHA256 1cfd2c45353d876b5b257b24d77537de58ab90a44ae595f15b1f64b6ea3fd770;
the replacement is 768750198 bytes/SHA256 35976b11be3904eb78e32f7dc55a6abf8e9433ac5aa391a1998f1e9ce001b199.
Our old MD selection referenced the replacement, not the historical file. The
same checkpoint label does not establish weight/graph or inference equivalence.
The remaining sections describe the historical integration and trials.


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
Final Linux/Windows CI 40304926502 passes: 1630 broad checks/56 skips, 62 codec,
94 updater, 119 NAV, 11 storage and 11 NAT checks. The counts overlap earlier runs.
No road-performance or 50 ms hard-deadline claim follows from metadata or desktop tests.

## Signed delivery and parked switching

Normal Git Pull and both-device reboots installed comma `1e80fbe2f8`. The final
signed MD/Flip runtime is `559e560e8529956c4029f53592f00832d735155b`
(64822428 bytes); N16 is `0d4947499931f42b94157129a847706fd2f5b637`
(64822535 bytes). Both complete HTTPS payloads and signatures were verified.

An initial non-NAV release passed its engine probe but failed standalone service
startup because its new model-library import lacked the repository search path.
The corrected server establishes that path. The release probe now launches the
actual server's import check from outside the checkout with PYTHONPATH removed;
the bundle regression tests exercise this too. Recovery used the normal signed
updater, with no source overlay or manually seeded model cache.

The real web API deferred selection until reboot, downloaded the official Flip
PKL, rejected deletion of running/pending entries, hid and restored the MD alias,
and retained the shared protected MD/Flip engine. All three choices were restored.
Jetson DATA had approximately 6.6 GB free; no partition resize was required.

Physical parked observations, with fresh P/zero/inactive-control guards:

| Selected model | Model frames | Mean | P99 | Maximum | >50 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| Flip Flop (shared MD/Flip ONNX) | 1200 valid, consecutive | 36.841 ms | 41.110 ms | 43.009 ms | 0 |
| Restored N16 | 1200 valid, consecutive | 40.255 ms | 44.562 ms | 45.968 ms | 0 |

These are separate roughly 60-second windows, not an A/B comparison of equivalent
models. Both cameras and camera odometry were valid and consecutive; pose stayed
healthy. N16's DM observer collected 1199 valid messages with one frame-ID gap;
this observation alone does not distinguish a missed observer sample from a
producer gap. N16 link/model/HUD were ready in all 60 one-second samples.

During N16 validation, 143 route coordinates and two maneuvers arrived, but every
map sample reported `position`: vehicle GPS had no fix and fused heading was
invalid. The owner confirmed the navigation device was indoors. Valid NAV image
application remains unverified in this window; no location/freshness gate was
relaxed. N16 was left selected and running, with the temporary offroad snapshot
hold restored. eGPU hardware inference and road behavior were not tested.


## Curated choices and automatic retirement (owner revision)

The owner's final UI has one dropdown: changing it saves the next-boot selection.
There are no user hide, delete, restore or separate save buttons. The server accepts
only `select`; legacy hidden preferences no longer remove reviewed choices. Failed
selection restores the displayed saved value. N16, MD v3 and Flip Flop remain
available; this revision does not retire any real model.

To retire a model, maintainers set `retired: true` on its catalog entry and retain
its exact artifact metadata. Do not delete its metadata/spec/release files: those
identities are needed to protect an already-running boot and to remove only known
cache paths. The default must remain available. A retired selection falls back to
the default next boot; the currently frozen boot is unchanged.

A background maintenance pass checks actual offroad state every 30 seconds, then
uses the existing library/delivery locks and active/boot/pending protections before
removing the retired eGPU pickle, partial download and compiled cache. The exact
Jetson identity is queued for the existing offroad USB cache protocol; shared
protected identities stay queued. The host retains its pinned/loaded/building
protections. Jetson deletion covers the ONNX and current-backend engine/metadata
known to that host, not arbitrary files or historical backend variants.

These client/UI changes need normal comma Git Pull/reboot. The existing signed
Jetson host already implements this cache request; its runtime and N16 are unchanged.
Temporary-cache tests cover retirement, boot fallback, active protection, actual
file removal and delayed shared-identity deletion. No installed model was deleted
as part of this revision's validation.


Revision validation: 56 focused catalog/API checks, 34 web model/settings checks,
25 Wiki checks, web production build and lint pass. Jetson integration CI
40325554976 passes Linux 1634 broad/56 skips, 62 codec, 94 updater, 119 NAV,
11 storage and 11 NAT checks, plus Windows installer checks. Counts overlap.
Published source is 49512be286 on carrot-nav-blink only.
