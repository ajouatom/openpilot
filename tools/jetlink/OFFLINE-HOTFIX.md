# PC에서 첫 부팅 전 USB-C 핫픽스 적용

프로그램 설치나 디스크 번호 확인이 익숙하지 않다면
[Windows 초보자 설치 안내](../../docs/INSTALL-WINDOWS-KO.md)를 먼저 보세요.
실제 다운로드 링크, Etcher 기록 방법과 그대로 따라 할 명령이 있습니다.

기존 이미지를 SD에 기록한 뒤, **Jetson에서 한 번도 부팅하기 전에** PC에서
이 핫픽스를 적용할 수 있습니다. 전체 이미지 재생성·재기록, WSL, Jetson SSH,
Wi-Fi 연결은 필요 없습니다. Windows 관리자 PowerShell과 Python 3.10 이상이 필요합니다.

대상은 v0.2.0-preview 원본 하나입니다. 원본 이미지 SHA256:
`5a1e7a3ba6156c621d8a01412f062b6c16ecb8ef2274b82b6acdaadf4b19a516`.
P3768 / L4T 36.4.7용이며 다른 버전에는 사용할 수 없습니다.

## 적용

1. 기존 v0.2.0 이미지를 SD에 기록합니다. 이미 기록한 미부팅 카드라면 이 단계는 생략합니다.
2. 오프라인 핫픽스 패키지를 PC 폴더에 풉니다. CARROTSETUP에 ZIP만 복사하는 방식이 아닙니다.
3. 관리자 PowerShell에서 `Get-Disk`로 대상 USB 카드의 번호·일련번호·바이트 크기를 확인합니다.
4. 아래 명령의 값을 해당 카드와 패키지에 맞게 지정합니다.

```powershell
.\apply_offline_hotfix_windows.ps1 `
  -DiskNumber <카드번호> -SerialNumber '<일련번호>' -DiskBytes <바이트크기> `
  -Python 'C:\Python312\python.exe' `
  -ManifestSha256 '<릴리스에 명시한 offline-usbc.json SHA256>' `
  -Log "$PWD\offline-hotfix.log"
```

`-VerifyOnly`를 추가하면 검사만 합니다. 기록 전 Linux 파티션 전체를 읽어 검증하므로
카드 속도에 따라 수 분 걸립니다. 실제 변경량은 5,632바이트이며 파일 내용은 5,196바이트입니다.
성공 시 `APPLIED_AND_READ_BACK` 또는 `ALREADY_APPLIED`와 `.success` 파일을 확인합니다.
오류가 나면 성공으로 취급하지 마세요. 다른 OS/이미지이거나 이미 부팅해 확장된 카드에는 기록하지 않습니다.
중간에 카드가 빠졌다면 같은 패키지를 다시 실행할 수 있습니다. 수정 영역 이외의 손상이 있으면 거부합니다.

5. 안전하게 제거한 뒤 Jetson에 넣어 부팅합니다. 첫 부팅 코드가 USB-C 정책 서비스와
   기존 검증된 helper를 설치하고 시작합니다. 이후 기존 기기별 초기 설정을 그대로 실행합니다.
   이후 부팅에서도 서비스가 유지됩니다. C-to-C 연결을 통해 콤마 Wi-Fi 전달을 받을 수 있습니다.
   개인 SSH 공개키 등록은 관리 접속이 필요한 경우에만 별도로 합니다.

이미 사용 중이고 SSH로 핫픽스를 설치한 카드는 다시 기록할 필요가 없습니다.
사용한 카드에는 [온라인 설치 방법](USB-C-HOTFIX.md)을 사용합니다.
일반 런타임·모델 업데이트는 기존 서명된 업데이트 경로를 계속 사용합니다.
이 패키지는 최초 USB 연결을 위한 특정 버전용 부트스트랩이며 범용 OS 업데이트 도구가 아닙니다.

## 구현과 검증 범위

Windows에서 임의의 ext4 파일을 추가하는 대신, 검증된 이미지의 기존 첫 부팅 파일에
같은 크기의 부트스트랩을 넣습니다. 디렉터리·inode·할당·journal·GPT·모델은 변경하지 않습니다.
압축한 내용은 원래 `image_first_boot.py`, `install_usbc.py`, `usbc_host.py` 세 파일입니다.
생성 코드는 `build_offline_hotfix.py`, 기록/검증 코드는 `offline_hotfix.py`에 있습니다.
패키지의 `bootstrap-review.py`는 기록될 내용을 검토하는 용도이며 PC에서 실행하지 않습니다.
원본의 개인 정보 없는 초기 설정과 공개키 처리는 그대로 보존됩니다. 정책 시작 실패 시에도
기존 초기 설정을 실행하여 관리 접속 준비를 막지 않고 오류를 보고합니다.

패키지 manifest의 SHA256, USB 디스크 ID·용량·시스템 디스크 여부, 파티션 정보,
전체 Linux 파티션 SHA256을 기록 전에 확인합니다. 이미 패치된 영역만 원본으로 정규화하여
중단 후 재시도를 허용합니다. 이 영역 밖의 변경은 허용하지 않습니다. 기록 후 해당 바이트를 다시 읽습니다.
관리자 래퍼는 Windows 볼륨을 잠그고 해제한 상태에서 실행합니다.

Windows 단위 시험과 원본 이미지의 read-only overlay/ext4 재읽기로 패치 위치를 검증합니다.
Linux CI는 실제 ext4 파일을 수정하고 e2fsck로 검사합니다. 이것은 Windows 실제 카드 쓰기,
해당 카드 첫 부팅 또는 차량 검증을 대신하지 않습니다. 패키지 VALIDATION.json에 구분하여 기록합니다.
USB-C 정책 자체의 기존 차량 시험 결과와 이번 PC 설치 경로의 검증 결과는 별개입니다.
