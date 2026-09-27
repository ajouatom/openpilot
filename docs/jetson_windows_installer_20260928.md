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
- NAS 업로드 후 ZIP 전체를 다시 읽어 SHA256을 확인했습니다. 공개 HEAD·앞뒤 Range 응답·메타데이터도
  원본과 일치합니다. NAS는 `5d7ebff7b8`을 자동 배포했으며 실제 결과 페이지와 1,196프레임 레이더 재계산 결과도 확인했습니다.
- [간편 설치 시험 릴리스](https://github.com/ajouatom/carrot-jetson/releases/tag/v0.3.0-windows-preview)를 게시했습니다.

## 보호와 미확인 범위

시스템·부팅·읽기 전용·오프라인·비USB 디스크와 설치 파일이 있는 디스크를 선택 대상에서 제외합니다.
사용자는 카드 번호와 삭제를 확인합니다. 기록 직전에 번호·용량·고유 ID·일련번호를 다시 확인하고
볼륨 잠금과 전체 이미지 읽기 검증을 유지합니다. 일련번호가 비어 있으면 고유 ID가 필수입니다.

이번 작업은 실제 SD카드를 지우거나 기록하지 않았습니다. 실제 02 기록, UAC 클릭,
그 카드의 첫 Jetson 부팅·차량 주행은 아직 이 패키지로 검증하지 않았습니다.
이전 SSH 핫픽스의 차량 시험과 구분하여 시험판으로 배포합니다.

## v0.3.1 — 한·영 실행 안내와 연결 주의사항

- 다운로드 문구를 `설치파일 받기 / Download installer`로 변경했습니다.
- 실행 직후 하는 일·대략적인 소요 시간·답변 방법을 한글과 영어로 보여 줍니다.
  01의 5~15분, 02의 15~60분은 사용 환경을 고려한 안내용 예상이며 실물 기록 시간 측정값이 아닙니다.
  느린 카드/리더에서는 더 걸릴 수 있음을 함께 표시합니다.
- 02는 관리자 권한 요청 전에 안내를 읽고 Enter로 진행하게 하며, 카드 번호와
  명시적인 삭제 확인(`설치` 또는 대문자 `INSTALL`)을 유지합니다. 애매한 입력은 취소합니다.
- 백업·기록 중 분리/전원 차단 금지와 Jetson 정상 종료·전원 분리 후 카드 삽입 순서를
  안내문·실행 창·완료 화면에 반영했습니다. 카드 삽입/제거는 Jetson 전원이 꺼진 상태에서 합니다.
- ZIP SHA256: `4cf6b508f284e613e1dd6bf1e21543056f10868b7fe0afa18942b98d0a62f8e0`, 8,274,609,553바이트.
  기존 원본 이미지·USB-C 수정·준비 완료 이미지 해시는 모두 그대로입니다.
- [차량 CI](https://github.com/ajouatom/openpilot/actions/runs/36356252459)와
  [호스트 CI](https://github.com/ajouatom/carrot-jetson/actions/runs/36356255075)가 성공했습니다.
  Windows 디스크 보호·한영 확인 입력·안내 문구를 검사했고 Linux 165 + 119개가 통과했습니다.
- 실제 ZIP의 CMD·휴대용 Python을 한글/공백 경로에서 **작은 합성 이미지**로 실행하여
  시작 안내·준비 과정·한영 완료 안내를 확인했습니다. 이전 버전의 전체 원본 준비 시험과 구분합니다.
  실제 SD 기록·UAC 클릭·Jetson 첫 부팅은 이번 안내 개정에서도 수행하지 않았습니다.
