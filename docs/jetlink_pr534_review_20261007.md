# PR #534: existing Jetson and eGPU compatibility

Reviewed contributor commit `3f2fe40ac104fac6ceacd2918fd40f42914ce3a0`
against `carrot-wip` `dee0cf91c737233135a936297d321af969d8aae0`.
The user requested compatibility review and integration. The merge retains
the destination's later explicit QCOM initialization and early NPY buffers.

## Problems corrected before integration

1. **Jetson boot gate rejected by auto discovery.** Its real HELLO includes
   `carrot_host=jetson` and `carrot_boot_update_v1=true`, but intentionally has
   no inference `backend` or `device`. The PR accepted only TRT/Orin for legacy
   USB, preventing the gate from reaching Wi-Fi and signed-release delivery.
   Recognize this explicit gate identity before preparation. This does not
   relax signature verification or let the gate perform inference.
2. **Late USB servers permanently missed.** Two failed probes left auto in
   TCP-only discovery until unplug. A Jetson still booting could remain absent
   after its server started. Keep the v3/v2 probe group, then a 30-second quiet
   NCM grace before retrying. TCP always has priority and a selected TCP path
   is never interrupted by later USB probes until detach.
3. **Optional iOS dependencies blocked existing USB.** Missing NCM, DHCP or
   working firewall support made default auto block all transports. Auto now
   reports the error and retries plain USB after resource cleanup. Detach
   helper-owned NCM even if setup failed before linking the function. Never
   bypass TCP isolation; unresolved cleanup still blocks. Explicit iOS mode
   keeps its own error and does not silently switch transports.
4. The contributor's model-join test predates destination NPY preallocation.
   Keep that GPU-independent test's camera preparation substituted, alongside
   the existing dedicated tests for real initialization decisions.

## Preserved behavior

- Existing Jetson v2 framing, pinned Cinque v2 model and exact contract.
- Signed boot-update gate/release pin, USB Wi-Fi provisioning, HUD/navigation,
  temperature diagnostics and existing session failure policy.
- Jetson still requires SuperSpeed. App USB2 allowance does not apply to it.
- Existing AMD eGPU selection/loading/control and internal fallback. Auto
  requires the comma's data role to be `ufp`; `dfp` and unreadable/present
  role attributes cannot start gadget discovery. Older systems only fall back
  to the existing CC policy if the data-role attribute is absent.
- The recent explicit QCOM adapter startup fixes, deferred warp preparation,
  native internal-model warmup, join permissions and 150 ms local deadline.

No Jetson image, host package, signed release, NAS model, eGPU model or OS
was changed or installed by this task. App model selection applies to recognized
protocol-3 apps, with existing offroad adoption and exact onroad approval checks.
The PR's iOS transport code is included; physical iPhone stability is unvalidated.

## Validation

- Windows-compatible core suite: 365 passed, 29 platform skips.
- Separate desktop lifecycle/contract harness: 136 passed, four Linux socket
  cases deselected. It supplies missing Windows syscall names; test fixtures
  replace those calls. This is not native Linux/USB/scheduling validation.
- Regression coverage uses the actual `BootstrapSession` HELLO, slow USB
  startup past both first probes, failed NCM startup, unsafe cleanup blocking,
  30-second retry grace, eGPU data-role exclusion and queued/stateful parsing.
- Python compilation and mobile helper shell syntax passed. Focused lint
  passes apart from the destination's pre-existing multiline log literal in
  `model.py` (ISC002); that log was retained.
- Native Linux CI is required after push for the real socket/IPC, helper and
  storage coverage. Results are recorded in the task's local evidence archive.
  Its first pass exposed two old expectations that detached NCM needed no
  cleanup verification, plus a pre-existing refresh-image fixture missing
  `wifi_protocol.py` and `boot_update.py`. Updated those expectations and the
  fixture, and assert that the stable updater receives both files and the
  next-boot gate marker. No image test is excluded from native CI.

Desktop and CI tests do not establish physical cable negotiation, Jetson boot
timing, accelerator output parity, thermal behavior or driving performance.
Contributor-reported Android/Mac trials are not a test of this final merged
revision. Hardware validation remains a parked-device follow-up.
