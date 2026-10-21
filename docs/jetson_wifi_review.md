# Integration status: 2026-10-20

The Wi-Fi-only display policy below is superseded by the October 21
[bounded USB restoration](jetson_usb_hud_restore_20261021.md). ESP32-S3 remains
retired; this page preserves the earlier investigation.

The source below is the contributor's review record, not a hardware validation
of this integration. The owner requests Wi-Fi-only comma-to-Jetson display
transport and removal of ESP32-S3 display support. Jetson keeps its local USB
panel. Old Jetson peers no longer get USB HUD or navigation fallback; the
receiver rejects these USB messages even before WLAN negotiation. The inference
loop skips all Jetson display USB pumps, including old advertised capabilities.
Model selection remains 80eedcdd/12864; the n16 ONNX change stays deferred.

Desktop checks exercise real loopback authenticated/encrypted records, tamper and
sequence rejection, both receiver states, missing/failed WLAN negotiation,
private-AP route preference and offroad admission. Inference-loop tests verify
unchanged model replies with no HUD or navigation pump calls. This is not a
physical C3/C4/Jetson RF, display-rate or driving validation.

# 콤마–Jetson 직접 Wi-Fi 연결 검토

기준: 공식 `ajouatom/openpilot`의 `carrot-nav` **74955c98**.
이 브랜치는 그 기준 위의 단일 커밋이며 추론 검토 브랜치와 독립적이다.
개인 배포 선택·서명 키·모델·배터리 화면·밝기·BSD·추론 최적화는 포함하지 않는다.

## 구성과 코드

```
휴대폰/공유기 ← 선택적 인터넷 → comma wlan0
comma wljetson0 AP ← 암호화된 Wi-Fi → Jetson → USB HUD
comma ← 기존 USB → Jetson 모델 추론
```

- `wifi_direct.py` / `wifi_direct_netlink.py`: 실제 station/AP 동시 조합을
  확인한 뒤 오프로드에서만 독립 AP를 생성한다. 서브넷 중복을 피하고 기존
  wlan0 연결과 일반 저장 프로필을 유지한다. AP 감독은 nice19·일반 스케줄링·
  little CPU에서 동작하며 세션 재연결과 분리된다. AP만 power-save를 해제하고
  종료 시 원래 값으로 복원한다. 권한 있는 USB 프로비저닝 경로로만 전용 AP를
  클라이언트 프로필로 전달한다. 비밀번호는 root 전용 상태에 보관한다.
- `wifi_publisher.py`: 소유한 활성 AP만 내보내며 일반 테더링 AP는 제외한다.
  AP 부팅/재생성 시에만 클라이언트 UUID를 바꿔 구형 Jetson importer가 재시도하게
  한다. 휴대폰 연결·해제 때문에 프로필을 주기적으로 교체하지 않는다.
- `hud_wifi.py` / `hud_wifi_worker.py`: USB 세션으로 임시 키를 협상한 뒤 별도
  HUD/media TCP 스트림을 Wi-Fi에서 상호 인증하고 AES-GCM으로 보호한다.
  키는 상속한 로컬 소켓으로만 전달한다. 크기·순서·나이·처리 시간을 제한하며
  Wi-Fi 대기는 모델의 USB 소유 프로세스 밖에서 처리한다.
- `hud_wifi_route.py`: 신선한 USB 상태가 알려준 Jetson WLAN 주소에 실제
  WLAN 경로가 있는지 확인한다. 전용 AP 경로를 우선하며 선택한 경로가 그대로면
  무관한 인터넷 인터페이스 소실 때문에 HUD를 재연결하지 않는다.
- `wifi_coexistence.py`, UI `wifi_manager.py`, web `device_info.py`: AP가 활성이고
  선택 인터넷 연결이 없을 때 불필요한 백그라운드 스캔을 줄인다. 사용자가 네트워크
  화면에서 명시적으로 요청하는 스캔은 유지하고 UI는 wlan0를 선택한다.
- daemon/server 변경은 HUD 협상과 정리 지점만 연결한다. 추론 전송·큐·모델·
  제어·전원 처리·서명 업데이트·차량 유효성 정책은 수정하지 않는다.

## 적용 전 알아둘 점

AP 연결/프로필 전달은 기존 Jetson 네트워크 importer를 사용하며 QSPI나
JetPack 업데이트를 요구하지 않는다. **HUD를 Wi-Fi로 전환하려면 제작자가
수정 server/worker를 포함한 Jetson 런타임을 자신의 기존 배포 절차로 빌드·서명해야
한다. 콤마 브랜치만 설치한다고 Jetson의 실행 파일까지 바뀌지는 않는다.**
개인키를 전달할 필요가 없고 공식 신뢰 키를 교체하지 않는다.

새 capability가 없는 기존 host는 기존 HUD 방식으로 동작한다. 새 Wi-Fi 방식으로
협상이 끝난 뒤 오류가 나면 HUD를 USB로 몰래 되돌리지 않는다. 모델 USB 경로는
계속 독립적이다. 반환한 공식 코드에서 helper가 사라지면 감독기가 자신이 만든
AP와 프로필만 정리한다. 소유자 전용 root identity는 재설치용으로 남을 수 있다.
AP는 드라이버 동시 동작 지원과 채널/RF 제약을 받으며 모든 하드웨어에 보장되지 않는다.

## 검증 결과와 한계

최신 공식 기준으로 추출한 이 브랜치에서 PC 회귀 **104 통과, 실패·건너뜀 없음**.
AP 지원 조합, 오프로드 활성화, 프로필 선택, phone-loss 경로 유지, 소유권·정리,
AP 전력 상태 복원, UI/web 스캔, 상호 인증과 변조 거부, 기존 daemon/HUD 계약을
검사했다. 기존 pytest 설정 경고3개가 있다. 요청에 따라 추가 자동 테스트 파일과
임시 장치 진단·개인 배포 파일은 커밋에서 제외했다.

기존 개인 통합 버전의 탁상 C3–Orin 측정에서는 35초 인터넷 station 분리 동안
전용 AP의 기존 HUD TCP 두 연결이 유지되고 약2.25MB가 전송됐다. ICMP300/300,
평균6.703ms, p95 33.420ms, 최대110ms였다. AP 감독은 관찰된 30초에 코어 하나의
약0.133%, RSS 약13MiB였으며 드라이버/IRQ 비용은 포함하지 않는다.
이 수치는 **과거 통합 버전**의 관찰이고 최신 공식 위 추출 브랜치의 장치 재검증이
아니다. 동시 A/B 또는 RF 최악 조건·실제 휴대폰 전원 OFF·상시20Hz 보장도 아니다.
초기 power-save 활성 시험에서는 수초 지연이 있었으며 스캔 억제만으로 해결되지
않았다. 전용 AP만 awake로 유지하는 좁은 보완이 해당 장치의 제한 시험을 통과했다.

실차 적용 전 제작자 서명 패키지, 기본/전용 채널 재연결, 휴대폰 상실, 재부팅,
대역 간섭, 모델/DM/pose 신선도와 주행 부하에서 추가 확인해야 한다.
다음 단계의 ONNX 차선 검증·영상 Wi-Fi 업로드·Jetson CPU 실행은 포함하지 않는다.
