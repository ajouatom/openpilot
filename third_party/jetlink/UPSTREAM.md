# Jetlink vendored runtime

Source: https://github.com/zoompilot/jetlink
Revision: `194ff6dc71cd27282378fff155e94daa7770152b` (0.3.0a1).
The v2 runtime is retained. `transport/base.py` adds a per-connection
`wire_protocol` selection hook; its default remains v2. See LICENSE.
`jetlink/protocol_v3.py` is an unmodified copy of `jetlink/protocol.py` from
zoompilot/jetlink `24e8917673829cbe1d146735956e9038cc93f227`.
PR #534 extends `protocol.py`, `client.py`, `spec.py` and `transport/base.py`
with strict per-stream v2/v3 selection, HELLO link metadata, queued/stateful
model contracts and validated full-output replies. `transport/ffs.py` reports
reader scheduling failures to the waiting consumer. The vendored server and
default protocol remain v2; these changes do not publish a new Jetson release.
`compat.py` retains the earlier pinned-model adapter for regression coverage;
the production daemon now uses the extended `JetlinkClient`.
Carrot's adapter, device-role policy, process placement and deployment tools are
outside this directory. Updating the protocol requires testing both peers.

This review adds GPU-resident recurrent state, adapted from the
tjdgus200/openpilot 7cbedfe03757c52482d9088dda49d89a94263ca4 and
zoompilot/jetlink d26001b4161f33bab28d806de6a99b96325a429b
(`JetlinkKit/Sources/JetlinkTRT/TrtEngine.swift`), under its MIT license.
After the owner-requested Orin comparison, CARROT_TRT_DEVICE_STATE defaults ON
for stateful TensorRT models; explicit 0 retains host state for diagnosis.
The GPU scans every float16/float32 state
element before its reply event; the original CPU driving-output validation,
reset, protocol and failure rules remain. This patch does not add asynchronous
USB writes, retained-frame output or a model change. Activation uses the
normal signed host-runtime pin and boot updater.
See docs/jetson_gpu_usb_review.md for the initial review and
docs/jetson_gpu_state_activation_20261021.md for activation evidence and limits.

Carrot coalesces `transport/watchdog.py` notifications: a pending earlier wake
rechecks a cancelled or later-rearmed write; an earlier deadline and close still
wake immediately. Expiry/cancellation locking, abort callback, time budgets and
thread scheduling stay unchanged. This removes per-write background Python
wakeups from the comma FunctionFS path. See docs/jetlink_watchdog_latency_20261021.md.

Carrot는 TensorRT 생성 중 할당 실패의 자원 회수와 캐시 파일의 실패 처리를
보강한다. 기존 엔진을 훼손할 수 있는 복사 fallback을 제거하고, 메타데이터는
임시 파일을 완성한 뒤 교체하며, 손상된 preload 힌트를 보존한 채 무시한다.
정상 추론·상태 피드백·모델·프로토콜과 원래 MIT 고지는 유지한다.
plan과 sidecar 두 파일을 함께 commit하는 거래는 아직 아니다.
실행 범위와 Konik A1M/Orin NX16 검증 한계는
docs/jetlink_konik_nx16_recovery_20261024.md에 기록한다.
