// Only fixed, reviewed UI copy is translated. Form actions, scopes, callbacks and codes are untouched.
export const englishConsent = req => {
  const language = String(req?.headers?.['accept-language'] || 'ko').split(',')[0].trim().toLowerCase();
  return language !== '*' && !language.startsWith('ko');
};
export const consentText = (req, ko, en) => englishConsent(req) ? en : ko;
const copy = [
  ['ChatGPT에 당근 기기 연결', 'Connect Carrot to ChatGPT'],
  ['당근 기기 연결', 'Connect Carrot'],
  ['한 번 연결하면 새 대화와 미니 음성창에서도 같은 당근웹을 사용할 수 있습니다.', 'Connect once to use Carrot Web in new conversations and the small voice window.'],
  ['당근웹의 ChatGPT 연결 설정에서 <b>미니모드 연결 코드</b>를 만든 뒤 아래에 입력하세요. 이 코드는 대화에 보내지 마세요.', 'Create a <b>mini-mode connection code</b> in Carrot Web’s ChatGPT connection settings, then enter it below. Do not send the code in a chat.'],
  ['미니모드 연결 코드 (5분 유효)', 'Connection code (valid for 5 minutes)'],
  ['허용: 설정 검색·설명 및 해당 웹의 설정 화면 열기.', 'Allows setting search, explanations and opening settings in this Carrot Web.'],
  ['추가 권한은 아래에서 별도로 선택합니다. 도구 실행 권한에는 확인 후 진단 로그 전송이 포함됩니다.', 'Choose additional permissions separately below. Tools permission includes diagnostic log uploads after confirmation.'],
  ['설정값 조회·변경도 허용', 'Allow reading and changing setting values'],
  ['선택한 설정의 현재값과 변경 결과를 ChatGPT 및 당근 연결 서버에 전달합니다. 당근웹에서도 별도로 허용해야 합니다. 대상과 값을 명확히 말한 변경 명령은 범위 검사 후 재질문 없이 적용합니다. 모호한 요청만 되묻습니다. 지원하는 주행 설정도 같은 방식으로 주행 중 변경할 수 있습니다. 최초 권한 허용은 정차 상태에서 진행합니다.', 'The requested setting’s current value and change result are shared with ChatGPT and the Carrot relay. Permission is also required in Carrot Web. A clear command specifying the setting and value is applied after bounds checks without another question. Ambiguous requests need clarification. Supported driving settings may be changed while driving in the same way. Enable permission initially while parked.'],
  ['현재 연결은 화면 열기 전용입니다. 설정값 변경을 원하면 새 권한으로 다시 연결하세요.', 'This connection only opens screens. Reconnect with additional permission to change setting values.'],
  ['음성 차선변경·설정속도 명령 허용', 'Allow voice lane-change and cruise set-speed commands'],
  ['명확한 실행 명령은 재확인 대화 없이 차량에 요청합니다. 실행 여부는 기존 차량 조건에 따르며, 명령과 실행 결과가 ChatGPT와 당근 중계 서버에 전달됩니다. 당근웹에서도 별도로 허용해야 합니다.', 'Clear execution commands are requested without an extra conversational confirmation. Existing vehicle conditions determine execution. Commands and results are shared with ChatGPT and the Carrot relay. Permission is also required in Carrot Web.'],
  ['도구 화면·작업 실행 허용', 'Allow Tools screens and actions'],
  ['Git 업데이트, 재시작, 초기화, 삭제, 백업, 화면 녹화 및 진단 로그 전송을 확인 후 요청할 수 있습니다. 유지보수 작업은 정차·보조 기능 해제 상태에서만 처리합니다. 진단 로그는 기존 당근 업로드 서버로 전송하며 ChatGPT에는 작업 종류와 결과만 전달합니다. 당근웹에서도 별도로 허용해야 합니다.', 'After confirmation, request Git updates, restarts, resets, deletion, backups, screen recording and diagnostic log uploads. Maintenance requires the vehicle to be parked and assistance disengaged. Logs go to the existing Carrot upload server; ChatGPT receives only the action type and result. Permission is also required in Carrot Web.'],
  ['이 기기 연결 승인', 'Authorize this connection'],
  ['연결은 당근웹에서 해제할 수 있습니다. 인증은 자동 갱신되며, 1년 동안 사용하지 않으면 다시 연결해야 합니다. 다른 사람이 보낸 코드는 입력하지 마세요.', 'Disconnect in Carrot Web. Authorization refreshes automatically; reconnect after one year of inactivity. Do not enter codes sent by other people.'],
];
export function consentPage(html, req) {
  if (!englishConsent(req)) return html;
  for (const [ko, en] of copy) html = html.replaceAll(ko, en);
  return html.replace('<html lang="ko">', '<html lang="en">');
}
