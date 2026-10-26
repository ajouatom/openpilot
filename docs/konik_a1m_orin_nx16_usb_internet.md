# Konik A1M·Orin NX 16GB USB 인터넷 공유

2026-10-26. `carrot-nav-blink` 기준의 명시적 개발자 선택 기능이다. 기본 CarrotPilot 설정과 다른 장치의 동작은 바꾸지 않는다. 콤마가 받은 인터넷을 기존 복합 USB 링크의 NCM 인터페이스로 Jetson에 전달한다. 추론은 기존 bulk USB 경로를 사용한다.

## 지원 범위

- 운영자가 실제 Konik A1M·reComputer J4012·Orin NX 16GB 조합임을 확인해야 한다. Konik 제품명과 캐리어보드 리비전을 소프트웨어만으로 확정하지 않는다.
- 콤마는 ARM64·4.9 계열 커널을 검사한다. 이 조건만으로 다른 Qualcomm 장치를 Konik이라고 판정하지 않는다.
- Jetson은 ARM64·`5.15.148-tegra-j401-r3647`과 Device Tree의 `p3767-0000` 선언을 검사한다. NVIDIA는 이 모듈 식별자를 [Orin NX 16GB](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/)로 문서화한다. Device Tree 선언은 하드웨어 진품 인증이 아니다.
- 네트워크 모듈 경로·바이트·vermagic·의존성과 NetworkManager 1.36도 원래 실기 검증 범위에 맞게 제한한다. 다른 BSP에 맞추려고 이 검사를 끄거나 커널 파일을 덮지 않는다.

## 기본값과 실제 변경

설치기 기본 실행은 읽기 진단/계획 출력이다. `--apply`와 정확한 프로필, 하드웨어·정차 확인이 있어야 설치한다. 자동 재부팅이나 추론 서비스 재시작은 하지 않는다.

콤마에는 DATA에 공유 프로그램·실행 단위·원복용 원장을 보관하고 기존 시작 스크립트의 본문을 보존하며 검증된 optional 블록만 추가한다. 읽기 전용 루트를 다시 마운트하지 않는다. 기존 전용 USB 모드를 명시적으로 `auto`로 선택해 FFS와 NCM이 함께 생성되도록 한다. 이 선택은 phone-only/iOS/Android 모드에 적용하지 않는다.

`profile.json`은 root 소유·0644·단일 링크·정확한 JSON 값이며 그 부모도 root 소유·0755여야 한다. 이 검사를 통과한 Konik 프로필에서만 `jetlinkd`를 `spawn`으로 시작하고 SIGTERM이 기존 `finally` 정리를 거치게 한다. 파일의 존재만 확인하거나 일반 TICI 전체에 이 동작을 적용하지 않는다.

공유 서비스는 기존 USB 소유권, USB 3, WAN·DNS와 기존 IP forwarding을 확인한다. 본인 태그의 NAT/FORWARD/DNS 규칙과 본인 DNS 중계만 관리한다. 전역 forwarding, DNS, 방화벽 정책, 다른 AP나 프로필을 바꾸지 않는다. 전제 조건이 없으면 준비 상태로 대기한다.

Jetson 클라이언트는 해당 VID/PID·`cdc_ncm`·유일한 인터페이스·현재 ifindex/MAC을 확인한다. 랜덤하게 바뀐 USB MAC은 본인 네트워크 프로필만 재정합한다. 프로필 이름은 공식 Wi-Fi 관리 접두어와 겹치지 않는 `carrot-ncm-internet`이다. 기존 LAN 경로와 다른 프로필은 보존하고 USB 경로 우선순위는 700을 사용한다. udev `add|move`는 짧은 비동기 systemd 작업만 요청하고 실제 네트워크 작업은 별도 서비스에서 수행한다.

서명된 공식 호스트 업데이트, 모델, 엔진, 커널·DTB, 전력, 전압·전류, CAN·Panda·차량 제어 코드는 변경하지 않는다. 호스트 네트워크 도구는 서명 릴리즈 밖에 별도로 선택 설치한다. 새 모델이나 임의의 별도 추론 프로토콜을 설치하는 기능이 아니다.

## 실행 예시

각 장치의 로컬 체크아웃에서, 벤치 또는 안전하게 정차된 상태로 실행한다. 비밀번호나 SSH 키를 명령에 넣지 않는다.

```bash
# 콤마: 계획만 출력
python3 tools/jetlink/konik_nx16_usb/install.py konik --profile konik-a1m-orin-nx16-j4012

# Jetson: 계획만 출력
python3 tools/jetlink/konik_nx16_usb/install.py jetson --profile konik-a1m-orin-nx16-j4012

# 하드웨어와 정차 상태를 직접 확인한 경우에만 선택 설치
sudo python3 tools/jetlink/konik_nx16_usb/install.py konik --profile konik-a1m-orin-nx16-j4012 --apply --confirm-hardware-match --confirm-offroad
sudo python3 tools/jetlink/konik_nx16_usb/install.py jetson --profile konik-a1m-orin-nx16-j4012 --apply --confirm-hardware-match --confirm-offroad
```

콤마 설치는 다음 정상 시작 때 반영된다. 이 도구가 재부팅을 요청하지 않는다. 정상 시작/서비스 전환은 운영자가 별도로 승인하고 진행한다. Jetson 선택 설치는 본인 네트워크 작업을 등록하고 현재 연결된 해당 장치가 있으면 본인 작업만 큐에 넣을 수 있다. 이미 검증된 프로그램·실행 단위·시작 블록이 정확히 일치하면 보존한다. 다른 내용이나 중단 원장은 거부하며 지우고 반복 실행하지 않는다.

콤마 설치와 원복은 root 소유의 고정 runtime 잠금 파일을 공유한다. 단일 inode·0600·비 symlink·부모 디렉터리 소유권을 검사하고, DATA 생성부터 전체 작업을 직렬화한다. 일부 실패 때도 잠금을 해제하며 설치 완료를 기록하기 전에 모든 파일을 다시 대조한다.

## 검증 근거와 한계

이 패키지의 `sharing.py`, `host_client.py`, `boot.py`, `sharing.service`는 기존 실기 시험에서 사용한 바이트와 일치한다. 기존 실기에서는 USB 재연결, LAN 없는 일반 인터넷 요청, USB에 묶은 DNS·TLS HTTPS 200, LAN→USB 자동 전환, 정상 재부팅과 두 장치의 완전 전원 분리 후 자동 연결을 확인했다. 완전 전원 분리는 사용자 물리 확인을 근거로 했고 전압계 측정은 아니다. 복원 순서는 콤마 먼저→Jetson 다음 1회였다. 자동 연결 준비 뒤 별도 합성 입력 5개의 실제 USB 추론 응답도 확인했다. 원시 식별 로그는 공개하지 않는다.

**새 opt-in 설치기·선택 게이트를 실장치에 새로 배포한 결과는 아니다.** 이 새 연결부는 임시 파일·모의 네트워크와 무해한 자식 프로세스에서 검사한다. 기존 실기 결과를 새 설치기 전체의 실물 인증으로 바꾸지 않는다. 다른 부팅 순서, 장시간 이동 통신·주행, 모든 향후 업데이트, 지연 시간 1,000/6,000회는 이 결과 범위 밖이다.

```bash
python3 -m pytest -q -c tools/jetlink/konik_nx16_usb/pytest.ini --rootdir=tools/jetlink/konik_nx16_usb --noconftest tools/jetlink/konik_nx16_usb/tests
```

검사는 실제 네트워크를 조작하지 않는다. USB 전용 TLS 바인딩, 잘못된 VID/PID/드라이버 식별, 랜덤 MAC·ifindex 변경, DNS 정착·실패 원복, 외부 프로필 보존, root opt-in 권한, 기존 파일 보존·일부 설치 중단·원복, 선택한 SIGTERM 정리를 확인한다. POSIX 전용 경계는 Windows에서 명시적으로 건너뛴다. 실기 원문 세 모듈의 SHA를 지키기 위해 그 원문과 이식한 회귀의 기존 스타일 진단은 패키지 안에서만 한정한다. 새 설치기와 선택 게이트의 일반 코드 검사는 유지한다.

## 원복

LAN 등의 별도 관리 접속을 확보하고 정차를 확인한 뒤 같은 도구에 `--rollback --apply --confirm-hardware-match --confirm-offroad`를 사용한다. 원복도 정확히 소유 확인한 파일과 프로필만 대상으로 한다. 새로 등록한 공유 runtime 단위가 실행 중이면 원래 정리 경로로 정지한 뒤 DATA 원문을 복원한다. 설치 전부터 존재하던 같은 실행 단위는 보존한다. 외부 변경은 덮지 않고 중단한다. 원복 원장·백업은 보존하고 재부팅은 자동 수행하지 않는다.

필요한 모든 백업의 타입·소유권·모드·SHA와 파일 inode를 서비스 정지나 첫 파일 변경 전에 검증한다. 확인한 바이트를 보관하고 실제 복원 중에도 대상·백업·원장의 교체를 다시 확인한다. 미리 손상된 백업이 있으면 원복을 시작하지 않는다. 설치 마지막 검사에서 외부 변경이 발견되면 완료를 기록하지 않고, 이번 설치가 만든 선택 마커만 본인 inode가 맞을 때 회수한다.
