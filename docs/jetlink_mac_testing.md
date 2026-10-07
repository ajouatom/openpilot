# Mac Jetlink 연결 시험

Protocol 3 compatibility and current phone setup are documented in
[Jetlink protocol 3 compatibility](jetlink_protocol3_compatibility.md).
The older protocol-2-only app assumption below no longer applies to the
updated comma adapter; model identity and parked-test restrictions still do.

The comma now defaults to automatic Mac/Android/iOS transport discovery when
no transport override exists. If you previously selected a phone mode, opt in
once with `printf 'auto\n' > /data/jetlink-transport` and restart the comma.
Explicit `usb`, `android` and `ios` modes remain available for diagnosis.
Automatic discovery does not fix USB host/device role negotiation; retain a
working data cable/hub topology and perform device changes while parked.

기존 **zoompilot Jetlink Mac 앱**과 Carrot를 연결하는 시험용 기능입니다.
Mac 앱을 수정하거나 Carrot 전용 앱을 설치할 필요가 없습니다. 현재 실제 Mac USB 연결과
CoreML 추론 성능은 검증 전이며, PC 통신 시험을 통과한 상태입니다.

## 준비

- Apple Silicon Mac, macOS 15 이상, 메모리 16GB 권장.
- [원본 Jetlink Mac 앱](https://github.com/zoompilot/jetlink/releases)을 설치합니다.
- 콤마는 이 기능이 포함된 최신 `carrot-wip`으로 업데이트합니다.
- Mac과 콤마의 전원을 확보하고 USB 3 데이터 케이블을 준비합니다.
- 최신 앱의 기본 네이티브 서버와 **Automatic**, **USB**를 사용합니다.
  별도의 구형 Python 서버는 필요하지 않습니다.

## 1. 최초 모델 준비 — 시동 끈 상태

1. **차량 시동은 끄고 콤마는 켜 둡니다.** 콤마의 대기 화면과 인터넷 연결을 확인합니다.
   시동을 켠 P단은 이 단계의 대기 상태가 아닙니다.
2. Mac에서 Jetlink 앱을 열고 USB 3 케이블로 콤마에 연결합니다.
   연결되지 않으면 USB 3 허브/어댑터의 USB-A 포트와 A-to-C 데이터 케이블로 확인합니다.
3. 앱에서 호환되는 주행 모델을 선택하고 **Use Model**로 준비합니다.
   콤마는 앱에 로드된 모델의 SHA와 입력·출력 규격을 검증한 뒤 사용합니다.
   Cinque v2로 고정되지 않으며, 호환되는 queued/stateful 모델을 지원합니다.
4. 앱의 모델 준비가 끝나고 콤마에 **MAC READY**가 나타날 때까지 기다립니다.
   다운로드는 회선 속도에 따라 수 분 이상 걸릴 수 있습니다. 앱 Logs에서 준비 진행을 확인하세요.

준비 도중 시동을 켜면 콤마는 새 모델 승인을 중단합니다. 앱에서 이미 시작한 다운로드나
빌드는 계속될 수 있습니다. 다시 시동을 끈 상태로 연결하면 준비된 Mac 엔진을 검증하고
재사용합니다. 콤마가 앱의 선택을 다른 고정 모델로 바꾸지는 않습니다.

## 2. 정차 상태에서 연결 확인

시동을 켜고 정차 상태에서 기존 전환 조건이 충족되면 **MAC** 표시로 바뀌는지 확인합니다.
`MAC READY`는 준비 완료이며 아직 외부 모델로 전환하지 않은 상태입니다.
Mac은 전원에 연결하고 절전되지 않게 유지하세요. 시험 중 앱의 모델이나 실행기를 바꾸지 마세요.

- 수 분 동안 모델 실행 속도와 지연, 연결 오류 여부를 확인합니다.
- 정차·제어 해제 상태에서 USB 재연결 후 이미 준비된 모델을 재사용하는지 확인합니다.
- 대기 상태에서 앱 종료/재실행 후 재연결을 확인합니다.
- 주행 성능과 안전성은 이 정차 시험만으로 검증되지 않습니다.

`MAC WAIT`는 준비/대기, `MAC RETRY` 또는 `MAC ERROR`는 연결·준비·추론 오류일 수 있습니다.
모델 규격과 기존 전환·오류 검사는 유지됩니다. iPhone/iPad와 Android의 연결 방법은
위 protocol 3 문서에 설명되어 있습니다. 모델 변경은 시동을 끈 상태에서만 준비하세요.
Jetson 전용 지도 스트리밍·Wi-Fi 설정 전달·온도 진단은 원본 Mac 앱에서 제공되지 않습니다.

## 결과를 알려주실 때

Mac 모델/메모리/macOS 버전, Jetlink 앱 버전과 서버 선택, 콤마 C3X/C4 및 커밋,
케이블/허브, 최초 준비 소요시간, 앱에 표시된 속도·지연과 오류 내용을 함께 알려주세요.
앱 **Logs** 및 콤마 오류 화면을 포함하면 연결 문제와 성능 문제를 구분하는 데 도움이 됩니다.
실제 Mac 연결과 지속 추론 시험은 아직 남아 있습니다.

---

# Mac Jetlink connection test

This experimental integration uses the **unmodified zoompilot Jetlink Mac app**.
Desktop protocol tests passed; actual Mac USB and CoreML performance remain unvalidated.

## Preparation

Use an Apple Silicon Mac with macOS15+ (16GB recommended), the
[upstream app](https://github.com/zoompilot/jetlink/releases), an updated `carrot-wip`
comma, separate power and a USB3 data cable. Use the current native App server,
**Automatic**, and **USB**; a separate old Python server is not needed.

## 1. First model setup — ignition off

1. Leave the comma powered, on its offroad screen and connected to the internet.
   Park with ignition on is not offroad for this setup.
2. Open the Mac app and connect USB3. If needed, try a USB3 hub/adapter's USB-A
   port with an A-to-C data cable.
3. Select a compatible driving model in the App and choose **Use Model**.
   The comma validates the loaded model's SHA and complete driving contract.
   Current Apps are not pinned to Cinque v2; compatible queued and stateful
   graphs use their own input/output metadata.
4. Wait for preparation and **MAC READY**. Download time depends on the connection;
   inspect the App's Logs for preparation progress.

Ignition-on cancels comma approval of a different model. An App download/build
already in progress can continue. Reconnect offroad to validate and reuse the
prepared Mac engine. The comma does not replace the App's pick with a fixed model.

## 2. Parked connection check

Start the car while parked and check **MAC** after the existing switch conditions
are met. **MAC READY** means prepared but not yet active. Keep the Mac powered and
awake, and do not change its model/backend during the test. Observe rate, latency
and errors for several minutes. While parked/disengaged, check cable reconnection
and cached model reuse; test app restart offroad. This does not validate driving.

**MAC WAIT** means waiting/preparing; **MAC RETRY/ERROR** can indicate setup, link
or inference failures. Existing model/switch/fault checks remain. Prepare model
changes with ignition off. iPhone/iPad and Android setup is in the protocol-3
document above; Jetson's navigation, Wi-Fi provisioning and temperature
extensions are outside this Mac test.

## Report

Include Mac model/RAM/macOS, Jetlink version/server choice, comma C3X/C4/commit,
cable/hub, first setup duration, displayed rate/latency, and app **Logs** or comma
error details. Real Mac connectivity and sustained inference remain to be tested.
