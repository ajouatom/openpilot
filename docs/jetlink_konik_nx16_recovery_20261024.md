# Konik A1M / Orin NX16: TensorRT 실패 복구와 캐시 보존

아래 2026-10-24 실기 결과는 [PR 563 기여자](https://github.com/ajouatom/openpilot/pull/563)의 보고이며, maintainer 차량에서 다시 측정한 결과가 아니다.

공식 carrot-nav `810e14aaf51176f1a50a113939dc2e70907b394b`를 기준으로 검토했다. 이 버전이 선택하는 서명 호스트는 `54e706fa0573e91dc3b4a0d4f625b9d81a6b68cc`, 모델은 기존 N16이다. 이전 기여의 USB 인계·DHCP 종료 경합·입력 준비 최적화는 이미 공식 소스에 반영됐으므로 다시 포함하지 않는다.

## 문제와 수정 범위

| 경로 | 실패 조건 | 수정 |
|---|---|---|
| TRT 엔진 생성 | 중간 device/pinned-host 할당이나 binding 설정 실패 | 획득 직후 정리 책임을 등록하고 실패 시 역순 회수한다. 정상 생성 뒤에는 기존 close 계약을 사용한다. |
| 메타데이터 저장 | 쓰기 중 공간 부족·부분 기록 | 같은 디렉터리의 고유 임시 파일을 완성·동기화한 뒤 교체한다. 교체 전 실패 시 이전 JSON을 보존한다. |
| 엔진 저장 | rename 실패 뒤 shutil.move의 copy 대체 경로도 실패 | os.replace만 사용해 이전 정상 엔진을 복사 대상으로 열지 않는다. |
| 캐시 자동 로드 | JSON 문법은 맞지만 루트·spec·마지막 모델 식별자의 형식이 잘못됨 | 손상된 파일을 삭제하지 않고 힌트를 무시한다. 정상 요청에서 기존 ONNX 재검증 또는 unavailable 경로를 사용한다. |

소프트웨어가 보고한 정리 성공과 실제 드라이버의 free 성공은 다르다. 정리 함수가 실패하면 원래 생성 오류를 보존하고 남은 해제도 시도한다. 실패한 자원까지 회수됐다고 표시하지 않는다.

이 수정은 플랫폼 중립적인 TensorRT 호스트 경로에 적용된다. NX 전용 텐서 크기·RAM 상수·모델 선택이나 다른 GPU의 계산 경로를 추가하지 않는다. 정상 run/state/graph 계산, 프로토콜, 캐시 키, 모델·정밀도, USB 속도·마감·서명 검사와 차량 제어는 유지한다. Nano의 RAM 용량이나 CPU 하나로 NX 연산을 제한한다는 결함을 확인한 것은 아니다.

## 검증 방법

새 회귀 검사는 실제 생성자·저장/복구 메서드와 작은 Linux 임시 파일을 사용한다. CUDA 할당기는 대역이며 host 메모리 뷰는 실제 ctypes·NumPy를 사용한다. 저장 시험의 TensorRT 빌더는 고정 바이트를 반환하는 대역이다. 실제 GPU 메모리 부족이나 실제 NVMe 공간 고갈을 유발하지 않는다.

```bash
python -m pytest -c /dev/null --rootdir=. --confcutdir=tools/jetlink -Werror -q \
  tools/jetlink/test_trt_constructor_cleanup.py \
  tools/jetlink/test_cache_storage.py \
  tools/jetlink/test_cache_recovery.py
```

설치된 사용자 외부 패치와 대조했을 때 저장·복구 세 파일은 SHA-256이 같다. 엔진 파일은 저장소 lint 규칙에 맞춰 불필요한 noqa 주석 한 줄을 정리했으며 전체 실행 구문(AST)이 같다. 아래 실기 결과는 이 동일한 실행 본문의 증거다. 새 테스트 소스나 PR 전체를 실차에서 검증했다는 뜻은 아니다.

## 실기 증거와 성능 한계

2026-10-24 Konik A1M + Orin NX 16GB/J4012, L4T 36.4.7·TensorRT 10.3 환경의 주차 상태에서 사용자 승인으로 검사했다. 기존 공식 서명 호스트를 보존하고 외부 패치로 네 파일을 불러오는 방식이었다. 이는 새 서명 호스트 번들을 발행하거나 PR 브랜치를 차량에 설치한 결과가 아니다.

- 대상 ARM의 합성 실패 회귀 20개가 통과했다.
- 같은 기존 엔진을 별도 프로세스에서 로드해 수정 전후 40프레임의 전체 주행 출력과 모든 재귀 상태 텐서 바이트 SHA가 같았다. 초기 상태·유한 값·CUDA 그래프·device-state·direct/native 준비 경로를 확인했다.
- 실제 서비스의 네 모듈 로드와 프로세스 세대, 공식 부팅 READY의 source·부팅 일치, 두 서비스 active/재시작 횟수0, 신선한 USB ready를 확인했다.
- 아래는 같은 모델·입력·profile·projection, 합성 영점/사전 변환 이미지, 공유 입력·20Hz 목표·별도 워밍업0회 조건의 USB 왕복 측정이다. 추가 상태 D2H·해시를 수행한 정확성 검사의 시간을 포함하지 않는다.

| 항목 | 수정 전 200회 | 수정 후 200회 |
|---|---:|---:|
| 평균 | 32.282ms | 31.648ms |
| P99 | 39.259ms | 42.446ms |
| 최대 | 49.985ms | 42.791ms |
| 50ms 초과 | 0/200 | 0/200 |

평균은 낮아졌지만 P99는 높아졌다. 속도 향상이나 지연 상한 준수를 확정하는 결과가 아니다. 실제 카메라·주행 입력, 장기 부하, 실제 OOM·전원 단절 검증은 남아 있다. 하드웨어별 BSP·전원 정격·모든 J4012 보드 리비전의 호환성도 이 패치로 입증하지 않는다.

## 저장 계약과 배포

각 파일의 교체는 보강했지만 **plan과 sidecar 두 파일을 하나의 세대로 함께 확정하는 거래는 아니다.** plan 교체 뒤 메타데이터 저장이 실패하면 새 plan/이전 메타데이터가 공존할 수 있다. 이 한계를 확인하는 검사는 해결됐음을 증명하는 검사가 아니다. 세대·해시 결합과 기존 캐시 마이그레이션은 별도 설계가 필요하다. fsync도 물리 저장장치의 무손상을 보증하지 않는다.

이번 PR은 공식 host_release pin을 바꾸거나 새 런타임을 배포하지 않는다. 호스트에서 실제로 사용하려면 maintainer가 통합 코드를 검토하고 공식 빌드·서명·릴리즈 절차를 거쳐야 한다. 기존 사용자 외부 패치의 설치기·service override·관리자 인증 도구나 사용자 전력 설정은 PR에 포함하지 않는다.

기존 Jetson integration workflow에 pull_request 트리거를 추가해 관련 수정과 새 회귀 검사가 PR에서도 실행되도록 한다. 원래 push·수동 실행과 job 내용은 유지한다. 실제 CI 상태는 해당 PR의 Checks에서 별도로 확인하며 로컬 통과로 대체하지 않는다.

## Maintainer integration review — 2026-10-26

Reviewed PR 563 at `31c8b1898a1a84a04b3d7a10c86729a0660b9422` against
`carrot-nav-blink`. The four runtime changes were not present. Applied the
failure cleanup, atomic individual-file publication and malformed-cache fallback;
kept current inference/state handling and all model/runtime ABI choices.
The PR event now covers all three existing integration branches, preserving the
current workflow jobs and push paths rather than copying the older workflow.

Independent Windows regression run: before the patch, 26 failed / 8 passed /
1 POSIX-mode skip; after it, 34 passed / 1 POSIX-mode skip. CUDA allocation and
TRT computation are test doubles; these are failure-path software checks, not
physical GPU OOM, latency or road acceptance. The original PR's hardware timings
are not a measured gain for this vehicle. The expanded local run passes 47 with one POSIX-mode skip; nine socket tests
require Linux sendmsg and cannot run on Windows. Linux validation below covers
those tests without changing their implementation.

Also reviewed PR 567 at `e2fac438b8eb148beb70ce863851f1e825991f67`.
It provides an optional root-owned USB Internet installer for operator-confirmed
Konik A1M and Orin NX16/J4012, including an exact
`5.15.148-tegra-j401-r3647` host gate and module hashes. It is not an inference
optimization or a prerequisite for PR 563. Its installer/lifecycle integration
has no reported physical installation acceptance. Our existing Jetson deployment
uses a different kernel profile and signed boot updater. Do not install or enable
this profile here; no files or networking behavior from PR 567 were integrated.
The ordinary Git Pull/restart delivery contract remains in force.

### Independent integration and delivery status

Source `c14c2cab0311d1d5316802416b07b2c6429ae721` is published on
`carrot-nav-blink`. [CI 40293651197](https://github.com/ajouatom/openpilot/actions/runs/40293651197)
passes: Linux broad 1,612 / 56 skips, codec 62, updater 94, navigation 119,
real-storage 11, isolated-network 11; Windows installer also passes.
These include the 35 new failure regressions; counts are not additive unique
coverage claims. Production engine and new-test Ruff checks pass.

Prepared bundle source `032cde486b3b95887dd41b5c70b1c42c7cd5d011` differs from
the tested commit only in repository memory and unrelated review documentation;
all exported source paths are identical. All 1,136 archive files match committed
bytes, including the bootstrap-compatible selected-model alias. Bundle size is
64,813,444 bytes, SHA-256
`9ddfbc429b016f2585b98f88a36f2627732162a1db4e155e06ba6efa5e180bcb`.
N16 hash/size are unchanged; no PR 567 installer or private evidence is included.

Owner explicitly approved looking up the existing signing-key path and signing/
publishing this exact package after the automatic-review block. The existing
trust-root-matching key signed it; no key content was printed. The immutable
NAS release was published and the complete HTTPS bundle download matched the
expected size and SHA-256. The branch now selects signed host `032cde486b`.
N16 remains unchanged; no stable/global release alias was promoted.

Local bundle/failure checks pass 36 / 2 platform skips. Source CI above covers
the identical exported runtime. Final pin commit `ed6723245b` also passes
[Linux and Windows CI 40294246490](https://github.com/ajouatom/openpilot/actions/runs/40294246490).
The 24 model-pin/NAV checks pass locally, and the public HTTPS manifest matches
the selected pin with a verified signature. No live vehicle mutation, installation, GPU benchmark or
restart was performed for this review. Users apply through normal comma Git
Pull and both comma/Jetson restarts, with offroad power and Internet available
for the updater. A comma-only restart cannot switch an already-running host.
