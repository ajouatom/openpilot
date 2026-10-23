# Konik USB gadget 인식·재연결 수정

기본 ADB+NCM gadget `g1`이 `a600000.dwc3`를 점유한 Konik에서는
Jetlink의 기존 설정 재시도가 `already holds the device controller`로 거절된다.
개인 부팅 스크립트를 추가하는 대신 기존 `setup_gadget.sh` 설정 경계에서
명시적으로 허용한 기본 gadget만 인계한다. 기존 daemon의 역할 확인·분리·재시도를 재사용한다.

## 명시적 인계

기본값은 비활성이다. 개발자가 정차 상태에서 다음 파일을 직접 만들 때만 허용한다.

```bash
sudo install -o root -g root -m 0600 /dev/null /data/jetlink-adb-handoff
```

이는 개발자용 파일 표식이며 웹 설정이나 Params 키를 추가하지 않는다.
USB 역할 `ufp`는 상대가 호스트임을 뜻할 뿐, PC와 Jetson을 구별하지 않는다.
**허용한 상태에서 PC에 연결해도 USB ADB·NCM이 해제될 수 있다.**
따라서 표식을 만들기 전에 USB ADB 이외의 장치 접근·복구 방법을 확보해야 한다.
개인 SSH 계정이나 특정 네트워크 프로그램에 의존하지 않는다.

다음 조건을 모두 다시 확인한 경우에만 `g1/UDC`에 빈 값을 쓴다.

- `/TICI` 플랫폼 표식과 실제 `ufp` 데이터 역할
- `IsOffroad=1`, `IsOnroad=0`
- 이름 `g1`, VID `04d8`, PID `1234`, 함수 `ffs.adb`와 `ncm.0`
- 알려진 제어기 `a600000.dwc3` 점유, 다른 bound gadget 없음
- 일반 파일인 허용 표식과 단일 실행 잠금

조건이 없거나 변했으면 다른 gadget을 보존하고 원래 설정 실패를 반환한다.
이미 연결된 Jetlink는 해제하지 않는다. 쓰기 직전에 조건을 다시 읽지만
서로 다른 sysfs 속성들의 검사·쓰기가 원자적이라는 주장은 하지 않는다.

허용 해제는 다음과 같다.

```bash
rm /data/jetlink-adb-handoff
```

표식 삭제는 앞으로의 인계를 막을 뿐 이미 해제한 ADB를 즉시 복구하지 않는다.
ADB를 다시 사용하려면 표식을 해제하고 정차 상태에서 원래 OS의 부팅 절차 등으로 복원한다.
FunctionFS가 살아 있는데 ADB를 자동 재바인딩하거나, 30초 후 복원하고
감시를 영구 종료하는 타이머를 추가하지 않는다.

## DHCP 종료 경합

모바일 USB 정리 중 확인한 dnsmasq 프로세스가 `/proc` 재검사 직전에 종료되면
원래 코드는 정상 종료를 소유권 오류로 처리해 연결 정리를 막을 수 있다.
`stop` 동작에서만 PID 완전 소멸과 자기 원장의 불변을 확인한 뒤 정리한다.
살아 있는 다른 프로세스·재사용 PID·바뀐 원장·그 밖의 실제 정리 오류는 계속 거절한다.
방화벽·주소·USB 전송량이나 기존 위험 정리 오류의 차단 정책은 바꾸지 않는다.

## 검증 범위

합성 configfs/PID fixture로 인계·거절·재시도와 정리 경합을 검증한다.
별도 장치 도우미로 동일 계열의 UDC 점유를 해제한 뒤 USB 3 연결·재부팅 복구를
관측했지만, 이 PR의 새 공용 helper를 장치에 설치한 결과와는 구분한다.
수정된 공용 경로의 실물 재검증은 남아 있다.

Linux PC에서 `test_adb_handoff.py`, `test_mobile_cleanup_race.py`,
`test_mobile_helper.py`, `test_mobile_link.py`의 120개 검사가 통과했다.
새 인계 검사는 29개, 종료 경합 검사는 6개이며 나머지는 기존 모바일 경로 회귀 검사다.
최초 Windows 공유 폴더 실행은 119개 통과·1개 실패였고, 실패는 `chmod 0700`을
보존하지 않는 파일시스템이었다. Linux 파일시스템에서 같은 120개를 다시 실행해 통과했다.
종료 경합 검사의 원형 재현에서는 2개 실패·4개 통과였고 수정 후 6개가 통과했다.
수정한 셸 2개의 Bash 문법 검사, Python 3.10 문법 호환성 검사와
새 Python 파일 3개의 Ruff 검사도 통과했다.

모델·추론 최적화·클럭·차량 제어·USB 속도 검사·서명된 호스트 런타임 선택은
이 변경에 포함하지 않는다. 개인 경로·주소·인증정보나 원본 장치 로그는 게시하지 않는다.


Maintainer integration: the opt-in marker must additionally belong to the
privileged caller (root for the CLI), and must not be group/world writable.
No marker is installed or enabled by ordinary Git Pull/reboot. See
[PR review](jetlink_pr560_562_review_20261023.md) for final integration scope.
