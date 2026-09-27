# Jetson Windows 설치 패키지 검증 기록

사용자용 기본 안내는 [네 단계 설치 안내](INSTALL-WINDOWS-KO.md)입니다.
이 문서는 배포 유지보수와 검증 근거이며 설치자가 읽을 필요는 없습니다.

## 배포 구성

- 버전: `v0.3.0-windows-preview`, NAS의 `carrot-jetson-windows.zip` 하나.
- 크기: 8,274,603,186바이트.
- ZIP SHA256: `b06c1e19757ce8b75600e98fca265abb069fddebfd93b1691bfd8c2f36ac91f9`.
- 기존 v0.2.0 압축 이미지, 원본에 맞는 USB-C 수정 manifest, 휴대용 Python,
  01 준비·02 기록 실행 파일을 포함합니다. 개인 설정·SSH 키는 포함하지 않습니다.
- 기존 이미지와 안정 런타임·모델은 변경하지 않습니다. 01이 PC 작업 폴더에 이미지를 풀고
  검증된 5,632바이트 수정을 적용합니다. 02는 준비된 파일을 기록하고 전체 내용을 다시 읽어 검사합니다.
- 준비 완료 이미지 SHA256: `423cf57a837d7a6d5dfec60bc28fd7721613b0e4bf8c3428d57b6661a893ce52`.
- 일반 런타임·모델 업데이트는 기존 콤마 지정/서명 경로를 유지합니다.

Python은 [공식 3.14.7 Windows embedded x64 배포](https://www.python.org/downloads/release/python-3147/)를
공식 SHA256과 PSF Authenticode 서명으로 확인하여 포함했습니다. 별도 설치·pip·WSL은 사용하지 않습니다.
Python 라이선스와 Carrot 라이선스를 함께 넣습니다. 원본 URL·해시는 패키지의 support에 기록합니다.
PowerShell은 UTF-8 BOM으로 배포하여 영문 Windows에서도 한글 소스가 깨지지 않게 합니다.

## 확인한 내용

- Windows에서 배포 ZIP을 공백·한글 경로에 풀며 전체 ZIP CRC를 검사했습니다.
- 포함된 휴대용 Python으로 원본 검사 → 압축 해제 → USB-C 수정 → 최종 전체 해시 검사를 완료했습니다.
- 실제 `01_설치준비.cmd` 재실행도 성공했습니다. 설치한 시스템 Python이나 전역 pip에 의존하지 않습니다.
- Windows 도구 시험: 22 통과, Linux ext4 시험 1 제외. 도구 전체: 130 통과, 7 제외.
- [차량 저장소 CI](https://github.com/ajouatom/openpilot/actions/runs/36354657282):
  Windows 디스크 보호 조건 15개와 구문 검사, Linux 165 + 119개 통과.
- [호스트 저장소 CI](https://github.com/ajouatom/carrot-jetson/actions/runs/36354659289):
  Windows 디스크 보호 조건과 Linux 91개 통과.
- 첫 Windows CI에서 BOM 없는 소스를 영문 Windows가 잘못 해석했습니다.
  소스에도 BOM을 보존하여 해결했습니다. 배포 ZIP은 처음부터 BOM을 포함했고 바이트가 동일합니다.

## 보호와 미확인 범위

시스템·부팅·읽기 전용·오프라인·비USB 디스크와 설치 파일이 있는 디스크를 선택 대상에서 제외합니다.
사용자는 카드 번호와 삭제를 확인합니다. 기록 직전에 번호·용량·고유 ID·일련번호를 다시 확인하고
볼륨 잠금과 전체 이미지 읽기 검증을 유지합니다. 일련번호가 비어 있으면 고유 ID가 필수입니다.

이번 작업은 실제 SD카드를 지우거나 기록하지 않았습니다. 실제 02 기록, UAC 클릭,
그 카드의 첫 Jetson 부팅·차량 주행은 아직 이 패키지로 검증하지 않았습니다.
이전 SSH 핫픽스의 차량 시험과 구분하여 시험판으로 배포합니다.
