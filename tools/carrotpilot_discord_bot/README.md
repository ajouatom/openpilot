# CarrotPilot Discord 지원 봇 — Synology 설치

이 봇은 지정한 Discord 채널의 한국어 질문을 받고, Synology에 복제한
`ajouatom/openpilot`의 `carrot-wip` 브랜치에서 관련 설정과 코드를 읽은 뒤 답합니다.
저장소 수정, 임의 셸 실행, 외부 포트 개방 기능은 없습니다.

## 준비물

- DSM 7과 Container Manager가 설치된 Synology
- Discord Bot Token
- OpenAI Platform API Key (ChatGPT Pro 구독과 별도 과금)
- Discord의 `#carrotpilot-질문` 채널 ID

토큰과 API Key를 README, Discord, Git 저장소 또는 다른 사람에게 보내지 마세요.

## 1. 파일 업로드

DSM `File Station`에서 다음 폴더가 있는지 확인합니다.

```text
docker/carrotpilot-bot/
├── config/
├── data/
└── repos/
```

이 패키지의 아래 항목을 `docker/carrotpilot-bot`에 업로드합니다.

```text
carrotbot/
config/bot.env.example
.dockerignore
Dockerfile
docker-compose.yml
requirements.txt
```

업로드 후 Synology의 최종 구조는 다음과 같습니다.

```text
docker/carrotpilot-bot/
├── carrotbot/
├── config/
│   └── bot.env.example
├── data/
├── repos/
├── .dockerignore
├── Dockerfile
├── docker-compose.yml
└── requirements.txt
```

## 2. Discord 채널 ID 복사

Discord에서 `사용자 설정 → 고급 → 개발자 모드`를 켭니다.
`#carrotpilot-질문` 채널을 마우스 오른쪽 버튼으로 클릭하고 `채널 ID 복사`를 누릅니다.

Discord Developer Portal의 Bot 페이지에서는 `Message Content Intent`를 켭니다.
봇 권한은 `View Channel`, `Send Messages`, `Read Message History`, `Embed Links`,
`Create Public Threads`, `Send Messages in Threads`가 필요합니다.

## 3. 환경설정 파일 만들기

PC에서 `config/bot.env.example`을 복사해 파일명을 `bot.env`로 바꾸고 다음 세 값을 입력합니다.

```env
DISCORD_BOT_TOKEN=Discord에서_복사한_토큰
OPENAI_API_KEY=OpenAI_Platform에서_만든_API_Key
DISCORD_CHANNEL_ID=복사한_숫자_채널_ID
```

나머지 값은 처음에는 그대로 둡니다. 저장한 `bot.env`를 Synology의
`docker/carrotpilot-bot/config` 폴더에 업로드합니다. `bot.env.example`은 남겨도 됩니다.

## 4. Container Manager 프로젝트 생성

1. DSM에서 `Container Manager → 프로젝트 → 생성`으로 이동합니다.
2. 프로젝트 이름을 `carrotpilot-bot`으로 입력합니다.
3. 경로에서 `docker/carrotpilot-bot`을 선택합니다.
4. 원본은 `docker-compose.yml` 업로드를 선택하고 이 패키지의 파일을 지정합니다.
5. Web Station 웹 포털은 활성화하지 않습니다.
6. 설정을 확인하고 `완료`를 눌러 빌드 및 시작합니다.

첫 실행에는 Python 이미지 다운로드와 carrotpilot 저장소 복제로 수 분이 걸릴 수 있습니다.
이 프로젝트는 외부 수신 포트를 사용하지 않습니다.

## 5. 정상 동작 확인

`Container Manager → 프로젝트 → carrotpilot-bot → 컨테이너 → 로그`를 엽니다.
다음과 비슷한 로그가 나타나면 정상입니다.

```text
Discord connected as CarrotPilot Helper
Repository ready: carrot-wip 0123456789ab
```

Discord 질문 채널에 다음 메시지를 보냅니다.

```text
!상태
```

브랜치, 커밋, 모델명이 반환되면 설치가 완료된 것입니다. 이어서 예시 질문을 보냅니다.

```text
브레이크를 밟으면 크루즈가 꺼지고 떼면 다시 켜지는데 끄는 설정이 있나요?
```

새 질문을 질문 채널에 올리면 봇이 공개 쓰레드를 자동으로 만들고 그 안에서 답합니다.
그 질문의 추가 내용은 같은 쓰레드에 작성하세요. 쓰레드마다 대화 문맥이 따로 유지되어
여러 사람이 동시에 질문해도 서로 섞이지 않습니다. `!상태`는 쓰레드를 만들지 않습니다.

질문자의 Discord 서버 닉네임과 역할도 참고 정보로 전달합니다. 예를 들어
`CarrotMaster/Ioniq5PE` 닉네임과 `C4` 역할이 있으면 등록 차종과 디바이스 힌트로 사용합니다.
사용자가 설정할 수 있는 정보이므로 봇은 확정 정보로 단정하지 않습니다.

## 운영 설정

- `REQUIRE_MENTION=false`: 전용 채널의 모든 메시지에 답합니다.
- `REQUIRE_MENTION=true`: 질문 채널의 첫 메시지는 봇을 멘션해야 합니다. 생성된 쓰레드의
  후속 질문에는 멘션이 필요 없습니다.
- `DAILY_QUESTION_LIMIT=100`: 사용자 한 명의 하루 질문 한도입니다.
- `GIT_UPDATE_MINUTES=60`: 소스 갱신 주기입니다.
- `OPENAI_MODEL=gpt-5-mini`: 비용과 품질에 따라 변경할 수 있습니다.

동일 질문과 동일 커밋의 답변은 SQLite 캐시를 재사용해 API 비용을 줄입니다.

## 장치 로그 진단

`openpilot/carrot-wip` 공유폴더에 `차종 16자리동글ID` 형식의 장치 폴더와
`toggles-*.json`, `onroad-*.txt`가 업로드되면 봇이 최신 세션을 읽어 오류를 분석할 수 있습니다.
질문에 동글 ID가 없으면 봇이 먼저 요청합니다. 오류 발생일과 같은 날짜의 로그만 사용하며,
날짜를 말하지 않으면 한국시간 기준 오늘 로그만 확인합니다. 해당 날짜 로그가 없으면
과거 로그를 대신 사용하지 않고 로그가 없다고 답합니다.

컨테이너에는 `/volume1/openpilot/carrot-wip`을 `/device-logs`로 읽기 전용 연결합니다.
다른 볼륨에 공유폴더를 만들었다면 `docker-compose.yml`의 왼쪽 경로만 실제 경로로 바꾸세요.
`bot.env`에는 `DEVICE_LOGS_PATH=/device-logs`를 추가합니다. IP, IMEI, ICCID, 토큰 등
민감정보는 OpenAI로 보내기 전에 마스킹하며, 장치 로그 기반 답변은 캐시하지 않습니다.

## 업데이트와 문제 해결

0.3.1 이상에서는 봇 소스가 컨테이너에 읽기 전용으로 연결됩니다. `carrotbot` 소스 파일을
교체한 뒤에는 이미지를 다시 빌드하지 않고 컨테이너를 재시작하면 적용됩니다.

- `필수 환경변수 ... 설정되지 않았습니다`: `config/bot.env`의 값을 확인합니다.
- Discord 로그인 오류: Bot Token을 재발급하고 `bot.env`를 수정합니다.
- `401` OpenAI 오류: API Key와 OpenAI Platform 결제 상태를 확인합니다.
- 채널 메시지가 무시됨: 채널 ID와 Message Content Intent를 확인합니다.
- 쓰레드 생성 오류: 봇 역할과 질문 채널에서 `Create Public Threads`,
  `Send Messages in Threads` 권한을 허용합니다.
- `NanoCPUs can not be set`: 구형 패키지의 `docker-compose.yml`에서 `cpus:` 줄을 삭제하거나
  이 패키지로 교체한 뒤 프로젝트를 다시 빌드합니다.
- Git 복제 오류: Synology가 `github.com`에 외부 접속 가능한지 확인합니다.
- 권한 오류: `data`와 `repos` 폴더에 관리자 읽기/쓰기 권한이 있는지 확인합니다.

토큰을 로그나 화면 캡처에 노출했다면 즉시 Discord Token과 OpenAI API Key를 폐기하고 새로 발급하세요.
