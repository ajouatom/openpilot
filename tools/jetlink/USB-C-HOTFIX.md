# Orin Nano C-to-C 연결 수정 안내

**처음 설치하는 분은 [Windows 초보자 설치 안내](../../docs/INSTALL-WINDOWS-KO.md)부터 읽으세요.**
이 문서는 이미 사용 중인 Jetson에 SSH로 접속하여 USB-C 연결 수정을 적용하는 방법입니다.
SD를 기록한 직후, 아직 Jetson에서 부팅하지 않은 카드라면 [PC 오프라인 핫픽스](OFFLINE-HOTFIX.md)를 사용할 수 있습니다.

## 어떤 문제를 수정하나요?

대상은 **NVIDIA P3768 기본 보드 / L4T 36.4.7**입니다.
Jetlink는 Jetson이 USB 호스트, 콤마가 USB 장치로 연결되어야 합니다.
기본 FUSB301 정책은 반대 역할을 선택하여 콤마 아래에 Jetson이 `0955:7020`으로 나타날 수 있습니다.
이 수정은 설치된 드라이버의 정책 설정으로 Jetson을 호스트 역할로 고정합니다.
커널·펌웨어·모델·USB 속도 검사 기준은 바꾸지 않습니다.

## 이미 사용 중인 Jetson에 설치

차량을 안전하게 주차하고 주행 보조를 해제한 상태에서 진행합니다.
자신의 SSH 공개키를 등록하고 Jetson에 접속하는 방법은 위 초보자 안내의 부록에 있습니다.
공용 이미지에는 개인 로그인 키가 없으며, Wi-Fi 자동 연결만으로 SSH 인증까지 설정되지는 않습니다.

같은 릴리스의 `usbc_host.py`와 `install_usbc.py`를 Jetson의 같은 폴더에 복사합니다.
**다음은 Windows PowerShell이 아니라 Jetson에 SSH로 접속한 뒤 실행하는 명령입니다.**

```sh
sudo python3 install_usbc.py
sudo systemctl daemon-reload
sudo systemctl start carrot-jetlink-usbc-host.service
systemctl is-enabled carrot-jetlink-usbc-host.service
systemctl is-active carrot-jetlink-usbc-host.service
cat /sys/class/usb_role/usb2-0-role-switch/role
lsusb -t
```

- `enabled`: 다음 부팅에도 자동 적용하도록 등록되었습니다.
- `active`: 정책 서비스가 실행 중입니다.
- 역할 출력 `host`: Jetson이 USB 호스트 역할입니다.
- `lsusb -t`: USB 연결 트리를 보여 줍니다. 역할이 맞아도 추론 성공까지 증명하는 것은 아닙니다.

다음 부팅에는 추론 서버가 시작되기 전에 정책이 적용됩니다.
이미 올바른 정책이면 연결을 끊는 재설정을 하지 않습니다.
반대 역할로 연결된 포트의 정책을 바꿀 때는 해당 USB 연결이 잠시 끊깁니다.
Linux 실행 중 USB-C는 호스트용으로 사용하며 USB-A 호스트 포트도 계속 사용할 수 있습니다.
펌웨어 복구는 별도 부팅 모드입니다. 절전·복귀 동작은 검증하지 않았습니다.

## 원래 USB 역할 자동 선택으로 되돌리기

필요할 때만 Jetson에서 실행합니다. 이후 C-to-C 연결 문제가 다시 발생할 수 있습니다.

```sh
sudo systemctl disable --now carrot-jetlink-usbc-host.service
sudo python3 /usr/local/lib/carrot-jetlink/usbc_host.py --restore-dual-role
```

## SD 기록 후 PC에서 적용할 때

기존 v0.2.0 이미지는 `CARROTSETUP`에 임의의 파일을 복사하는 것만으로 핫픽스를 실행하지 않습니다.
**기록 직후 한 번도 부팅하지 않은 v0.2.0 원본 카드**는 전용 [오프라인 도구](OFFLINE-HOTFIX.md)로 적용합니다.
이미 사용 중인 카드는 위 SSH 경로를 이용합니다. 전체 OS·모델 이미지를 다시 만들 필요는 없습니다.
관리키가 없으면 `CARROTSETUP/setup.json`에 자신의 **공개키**를 추가한 뒤 부팅하여 SSH 경로를 준비합니다.
개인키를 공용 이미지나 GitHub에 넣지 않습니다.

개발자용 이미지 제작기는 `install_usbc.configure(mounted_root)`를 호출하여 수정 내용을 포함합니다.
PC 오프라인 도구의 실제 카드 첫 부팅 시험은 아직 남아 있으며,
SSH로 설치한 수정의 정차 검증과 구분합니다. 다른 보드·주행·절전 복귀까지 검증한 것은 아닙니다.

## 구현 근거

- [FUSB301 드라이버 원문](https://github.com/OE4T/linux-nv-oot/blob/jetson_36.4.7/drivers/usb/typec/fusb301.c):
  `fsw_trysnk_store`, `fmode_store`, `fusb301_detach`, `fusb301_snk_detected`
- [NVIDIA 보드 USB 포트 설명](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html)
