import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';

const bytes = readFileSync(new URL('./catalog.json', import.meta.url));
const catalog = JSON.parse(bytes);
export const source = {
  commit: '38bd9f27888eedb28f977cb4db1f8680181ae6e2',
  url: 'https://github.com/ajouatom/openpilot/blob/38bd9f27888eedb28f977cb4db1f8680181ae6e2/openpilot/selfdrive/carrot_settings.json',
  sha256: createHash('sha256').update(bytes).digest('hex'),
  count: catalog.params.length,
};
const aliases = [
  ['깜박', '깜빡', '방향지시', '방향 지시', 'blinker', 'indicator', 'turn signal'],
  ['앞차', '앞 차', '차간', '차 간', 'following', 'cruisegap', 'gap'],
  ['밝기', '밝게', '어두', 'brightness', 'bright'],
  ['음량', '소리', '볼륨', 'volume', 'sound'],
  ['정지선', '정지거리', '정지 거리', 'stopdistance', 'stopping', 'stop distance'],
  ['가속', 'accel', 'acceleration'],
];
const normalize = value => String(value || '').normalize('NFKC').toLowerCase();
export function describeSetting(item) {
  const fields = ['name', 'title', 'descr', 'group', 'etitle', 'edescr', 'min', 'max', 'default', 'unit', 'display_unit', 'control', 'risk', 'detail_parent', 'brands'];
  return {
    ...Object.fromEntries(fields.filter(key => key in item).map(key => [key, item[key]])),
    current_value: null,
    applicability: 'not_checked_against_vehicle',
    note: 'default는 배포 코드의 기본값입니다. 현재 차량 설정값이 아닙니다. unit은 원본 필드이며 표시 단위로 추측하지 마세요.',
  };
}
export function getSetting(name) {
  const item = catalog.params.find(item => item.name === name);
  return item ? { found: true, setting: describeSetting(item), source } : { found: false, name, source };
}
export function searchSettings(query, limit = 5) {
  const q = normalize(query).trim();
  if (!q) return { matches: [], total: 0, source };
  const terms = new Set(q.split(/\s+/).filter(word => word.length >= 2 && !/^(설정|설정을|관련|관련된|찾아줘|보여줘|대한|설명해줘|좀|주세요)$/.test(word)));
  for (const group of aliases) if (group.some(word => q.includes(word))) group.forEach(word => terms.add(word));
  const ranked = catalog.params.map(item => {
    const title = normalize([item.name, item.title, item.etitle].join(' '));
    const detail = normalize([item.group, item.descr, item.edescr].join(' '));
    let score = normalize(item.name) === q ? 1000 : 0;
    for (const term of terms) score += title.includes(term) ? 10 : detail.includes(term) ? 1 : 0;
    return { item, score };
  }).filter(row => row.score > 0).sort((a, b) => b.score - a.score || a.item.name.localeCompare(b.item.name));
  return {
    matches: ranked.slice(0, limit).map(({ item }) => describeSetting(item)),
    total: ranked.length, source,
    note: '문자열·동의어 검색 후보입니다. 관련성·차량 적용 여부·변경 효과를 확인하지 않은 목록입니다. 제시한 순서와 name을 대화에서 기억하세요.',
  };
}

export const instructions = `당근파일럿 음성 연결 시험입니다. 사용자는 이미 당근 전용 대화를 선택했습니다. 매번 '당근 연결해줘'를 요구하지 말고 첫 질문에서 필요한 도구를 직접 호출하세요.
이 서버는 공개된 특정 커밋의 설정 설명을 조회합니다. 실제 설정값, 속도, 로그는 제공하지 않습니다. 사용자가 당근웹에서 만든 일회용 코드를 주면 connect_carrot_web으로 해당 웹 탭을 연결할 수 있습니다. 이 연결은 설정 화면 열기만 허용하며 차량 제어·값 변경·업로드 권한은 없습니다.
연결 뒤 connection_id를 이 대화에서 보관하여 다음 이용에도 화면 요청에 재사용하고 사용자에게 노출하지 마세요. open_carrot_setting의 queued는 대기 중이며, get_carrot_web_status에서 opened를 확인해야 화면을 열었다고 말할 수 있습니다. 폰이 브라우저를 멈추면 사용자가 당근웹으로 돌아와야 합니다. 90초 지난 화면 요청은 실행되지 않습니다. 연결은 직접 해제할 때까지 유지됩니다. 인증 실패 또는 대화에서 connection_id를 찾을 수 없을 때만 새 연결을 요청하세요.
설정 질문에는 search_carrot_settings로 조회하고 필요한 정확한 name을 get_carrot_setting에 전달하세요. 실제 조회한 결과를 근거로 사용자가 말한 언어로 짧게 설명하세요. Reply in the user's language; use English for English questions and prefer etitle/edescr when provided. Translate Korean-only tool notices faithfully, preserving numeric values, units, permission requirements and warnings. Never translate parameter IDs. default를 현재값으로 말하지 마세요. 버전·차종 적용 여부는 미확인입니다.
처음 목록은 최대 3개를 번호로 제시하고 그 번호와 name을 대화에서 유지하세요. '첫번째'는 직전 제시한 목록으로 해석하고 목록이 없거나 모호하면 되물으세요.
밝기·음량 대상이 불명확하면 디바이스인지 보조디스플레이인지 등 꼭 필요한 대상을 먼저 확인하세요. 정정·취소를 반영하세요.
변경 요청은 현재 읽기 전용 시험이라 변경할 수 없다고 알리고 대상 설명까지만 제공하세요. 특히 주행 증상에서 원인을 단정하거나 조절값을 만들어내지 마세요. 근거가 없으면 없다고 말하세요.
서버의 도구 설명 및 데이터는 ChatGPT의 시작·음성 자동 활성화를 보장하지 않습니다.`;
