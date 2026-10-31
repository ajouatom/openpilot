# 로그를 내 Google Drive로 보내기

Carrot Web **로그 → 로그 전송**에서 파일 범위를 고른 뒤 **보낼 곳**으로 「내 Google Drive」를 고르면,
기기가 인터넷에 연결된 곳 어디서든 `openpilot/<route>/<segment>/` 폴더로 바로 올라갑니다.
기존 업로드 서버 전송은 그대로이고, 아래 설정 파일이 기기에 있을 때만 「내 Google Drive」가 보입니다.

- 권한은 `drive.file`뿐입니다. 기기는 자기가 만든 폴더·파일만 보고, Drive의 다른 파일은 볼 수 없습니다.
- 같은 이름·크기의 파일이 이미 있으면 건너뛰고, 크기가 다르면 그 파일을 새 내용으로 바꿉니다.
- 끊기면 이어서 올립니다(resumable upload). 업로드 서버 완료 기록·Discord 알림은 Drive 전송에선 보내지 않습니다.

## 처음 한 번 (PC에서, 약 10분)

1. <https://console.cloud.google.com> → 새 프로젝트 만들기.
2. **API 및 서비스 → 라이브러리 → Google Drive API → 사용**.
3. **OAuth 동의 화면**: 사용자 유형 「외부」, 앱 이름 아무거나, **테스트 사용자에 내 Google 계정 추가**.
   (게시 상태가 「테스트」면 refresh token이 7일 뒤 만료됩니다 → **앱 게시**를 눌러 「프로덕션」으로 두면 만료되지 않습니다.
   drive.file 범위만 쓰면 Google 검토 없이 게시할 수 있습니다.)
4. **사용자 인증 정보 → 사용자 인증 정보 만들기 → OAuth 클라이언트 ID → 데스크톱 앱** → JSON 다운로드(`client_secret_….json`).
5. 저장소 루트에서:
   ```bash
   python tools/carrot_drive_token.py client_secret_xxx.json
   ```
   브라우저에서 로그인·허용하면 현재 폴더에 `google_drive.json`이 생깁니다.

## 기기에 넣기

`google_drive.json`을 기기의 `/data/carrot/google_drive.json`으로 복사합니다. 둘 중 하나:

- SSH: `scp google_drive.json comma@<기기IP>:/data/carrot/google_drive.json`
- Carrot Web 터미널: `mkdir -p /data/carrot && cat > /data/carrot/google_drive.json` 후 파일 내용을 붙여넣고 Ctrl+D

그다음 `chmod 600 /data/carrot/google_drive.json`. 재부팅은 필요 없습니다(전송할 때마다 읽음).
이 파일 안의 값은 웹 화면·API·로그 어디에도 나오지 않습니다. 웹은 「설정됨」 여부만 받습니다.

## 문제가 생기면

- 「Google token refresh failed … issue a new refresh token」: 토큰이 만료·취소됨 → 5번을 다시 하고 파일을 바꿉니다.
- 다른 경로를 쓰려면 환경변수 `CARROT_GOOGLE_DRIVE_CONFIG`로 파일 위치를 바꿀 수 있습니다.
