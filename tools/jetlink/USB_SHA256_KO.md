# 콤마 USB를 통한 선택적 Jetson 업데이트

검토 기준은 공식 `carrot-nav`의 `c4568c30346bc6412237f3fc1e1cfcf74c9ceb9d`입니다.
이 변경은 콤마가 호스트 번들과 ONNX를 내려받아 USB로 전달하는 경로만 추가합니다.
HUD, 무선 연결, ONNX 추론, 차량 제어, 모델 선택 및 펌웨어는 변경하지 않습니다.
개인 배포 주소·키·장치 로그·특정 패키지 선택도 포함하지 않습니다.

## 경로와 검증

1. 제작자가 공개 `host.tar.gz`와 `model.onnx`의 SHA-256·크기를 콤마 브랜치에 고정합니다.
2. Jetson 부팅 수신기가 로컬 정책을 확인한 뒤 `carrot_usb_sha256_v1` 기능을 알립니다.
3. 콤마가 오프로드에서 파일을 HTTPS로 내려받고 해시·크기를 검사해 USB로 전송합니다.
   캐시가 있거나 Jetson에 같은 패키지가 등록됐다면 다시 내려받지 않습니다.
   현재 커밋이 같을 때도 읽기 전용 확인으로 실제 카탈로그 등록·해시를 확인합니다.
   이 확인은 설치나 파일 할당을 하지 않으며, 정상 등록된 같은 버전의 온로드 부팅을 막지 않습니다.
4. Jetson이 파일별 해시·크기·순서·공간·소유권을 확인한 뒤에만 로컬 카탈로그에 등록합니다.
   Jetson은 이 경로에서 인터넷 다운로드를 하지 않습니다.
5. 기존 부팅 게이트가 등록 패키지를 선택해 ABI·모델·보호 저장소·실행 검사를 수행합니다.
   검사 실패 시 기존 버전을 보존합니다. 전송 완료만으로 실행을 승인하지 않습니다.

새 패키지 전송과 활성화 요청은 콤마의 `IsOffroad=true`, `IsOnroad=false`일 때만 합니다.
전송 중 온로드 전환 또는 USB 단절이 감지되면 중단하고, 미완성 파일은 설치하지 않습니다.
온로드 P단 설치, 기어 판정, 실행 중인 추론의 교체는 이 제안에 포함하지 않습니다.
패키지 설치는 Jetson의 부팅 게이트에서만 가능하며, 런타임 USB 서버에는 설치 명령을 추가하지 않습니다.

SHA-256은 손상 검출이며 제작자 신원 인증이 아닙니다. 패키지와 고정 해시를 함께 바꿀 수 있는
브랜치 또는 USB 상대를 신뢰한다는 전제입니다. 파일은 평문이며 별도의 암호화·서명 키가 필요 없습니다.
공식 NAS 경로의 서명·공개키·HTTPS 제한은 그대로 유지합니다.

## Jetson에서 한 번 필요한 허용

기존 이미지에 수신기와 정책이 없으면 콤마의 브랜치 변경만으로 활성화되지 않습니다.
신뢰하는 이 소스의 작업 복사본에서, Jetson을 차량에서 분리한 뒤 직접 설치합니다.
`<SOURCE>`는 그 복사본의 절대 경로입니다.

```bash
sudo systemctl stop carrot-jetlink carrot-jetlink-hud carrot-jetlink-boot-gate carrot-jetlink-update-stage
sudo /opt/carrot-jetlink/venv/bin/python <SOURCE>/tools/jetlink/install_usb_receiver.py --runtime /opt/carrot-jetlink --sha256
sudo reboot
```

복구 환경에서 설치할 때는 `--runtime`에 마운트된 DATA의 `runtime` 절대 경로를 지정합니다.
이 경우 호스트 PC의 systemd를 조작하지 않습니다. 설치기는 공식 공개키를 보존하고 수신 코드와
root 소유 `updater/usb-sha256-policy.json`을 DATA에 기록합니다. 직전 수신 코드 한 벌만
`updater/before-usb-experiments/`에 보관합니다. USB 데이터로 이 정책을 켤 수는 없습니다.
기존 이미지에 이미 부팅 업데이트와 보호 저장소가 설정돼 있어야 합니다.

## 제작자 배포 절차

소스를 먼저 커밋하고 기존 `build_host_bundle.py`로 번들을 만듭니다. 이후 다음 도구로
그 번들의 소스 커밋과 모델 고정값에 맞는 매니페스트를 생성합니다.

```bash
python tools/jetlink/build_host_bundle.py /tmp/host.tar.gz
python tools/jetlink/prepare_usb_release.py \
  --bundle /tmp/host.tar.gz --model /path/to/model.onnx \
  --profile jetson-usb-example-1 \
  --bundle-url https://example.org/releases/host.tar.gz \
  --model-url https://example.org/releases/model.onnx \
  --output /tmp/jetson-usb-selection
```

실제 제작자 배포 URL로 바꿔 두 파일을 게시하고 생성된 `usb_host_sha256_release.json`만
`openpilot/selfdrive/modeld/jetlink/`에 커밋합니다. 이 검토 브랜치에는 선택 파일이 없으므로
설치만으로 업데이트 대상이 바뀌지 않습니다. `local_host_release.json`과 동시에 선택하면 거부합니다.
수신기 기능이 없거나 SHA 패키지가 거부되면 다른 검증 방식으로 자동 우회하지 않습니다.

선택 파일을 제거한 공식 브랜치로 돌아가면 기존 `host_release.json`의 서명/NAS 경로를 사용합니다.
Jetson USB 수신 허용 자체를 해제하려면 정지/복구 환경에서 로컬 정책 파일을 제거합니다.

## 검증 결과와 한계

PC Linux에서 실제 파일·SHA-256·root 소유권·심볼릭 링크와 메모리 USB 전송을 사용했습니다.
기존 업데이트, 로컬 등록, 부팅 게이트, 오프라인 패치, 모델 선택 회귀 검사까지
**115개 통과, 실패 0개, 건너뜀 0개**입니다.

확인한 실패 경로는 해시/크기/오프셋 불일치, 미완성·중단 전송, 공간 부족, 프로필 충돌,
잘못된 로컬 정책, 온로드 상태, 동시 활성화, ABI 불일치, 실행 검사 실패입니다.
공식 NAS 복귀 시 기존 서명 검사 호출과 HTTPS 경로 사용도 확인했습니다.
새 USB 경로는 Crypto 모듈을 가져오지 않고 동작합니다.

검사는 `systemctl`, 실제 OS/TensorRT 검사와 추론 실행을 대체한 임시 환경에서 수행했습니다.
이 공식 기준 포트 자체의 실물 USB·전원 차단·TensorRT 실행·실차 주행은 검증하지 않았습니다.
기존 개인 브랜치에서의 실물 설치 결과를 이 포트의 실물 검증으로 간주하지 않습니다.

재현 예시(개발 의존성이 설치된 Linux, root 소유권 검사는 격리된 root 테스트 환경 필요):

```bash
PYTHONPATH=.:tools/jetlink:third_party/jetlink python -m pytest --noconftest -o addopts= \
  tools/jetlink/test_usb_sha256.py tools/jetlink/test_update_host.py \
  tools/jetlink/test_local_experiment.py tools/jetlink/test_boot_update.py \
  tools/jetlink/test_offline_boot_patch.py tools/jetlink/test_nav_rl.py
```
