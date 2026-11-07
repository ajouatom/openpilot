import { pathToFileURL } from 'node:url';
import { readFileSync } from 'node:fs';
const voiceRules = readFileSync(new URL('./VOICE_RULES.md', import.meta.url), 'utf8');
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { StreamableHTTPServerTransport } from '@modelcontextprotocol/sdk/server/streamableHttp.js';
import { createMcpExpressApp } from '@modelcontextprotocol/sdk/server/express.js';
import { z } from 'zod';
import { toolActions, toolPanels } from './tools_bridge.mjs';
import { source, instructions, getSetting, searchSettings } from './catalog.mjs';
import { createBridge, installBridgeRoutes } from './bridge.mjs';
import { createAccountAuth, installAccountAuth, ACCOUNT_SCOPE, SETTINGS_SCOPE, DRIVE_SCOPE, TOOLS_SCOPE } from './oauth.mjs';
import { requireBearerAuth } from '@modelcontextprotocol/sdk/server/auth/middleware/bearerAuth.js';

const readOnly = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: false };
const output = data => ({ content: [{ type: 'text', text: JSON.stringify(data) }], structuredContent: data });
export function createServer(onTool = () => {}, bridge = createBridge(), sessionId = null, scopes = [ACCOUNT_SCOPE]) {
  const accountInstructions = `당근파일럿 음성 도우미입니다. 이 요청은 이미 인증된 기기에 연결되어 있습니다. 새 대화나 안드로이드 미니 음성창에서도 연결 코드를 요구하지 마세요. 설정 화면을 열 때 get_carrot_web_status로 본인 기기를 확인하고 open_carrot_setting에 정확한 name만 전달하세요. 명확한 운전 명령에는 사전 상태 조회 대신 request_carrot_drive를 먼저 호출하세요. 사용자 ID나 connection_id는 필요 없습니다. 브라우저가 오프라인이면 당근웹을 열도록 안내하고 재연결 코드를 요구하지 마세요. 인증 실패만 재인증이 필요합니다. 설정 화면 열기만 허용하며 값 변경·차량 제어·로그 전송은 지원하지 않습니다. queued는 대기이며 opened만 완료입니다.\n${instructions.slice(instructions.indexOf('설정 질문에는'))}`;
  const settingInstructions = `설정값 조회·변경은 carrot.settings 권한과 해당 당근웹의 별도 허용이 필요합니다. 공개 카탈로그의 기본값을 현재값처럼 말하지 마세요. 조회 질문에는 read_carrot_setting_value를 사용하세요. 사용자가 대상과 값 또는 증감량을 명확히 지정한 실행 명령에는 prepare_carrot_setting_change로 내부 검증한 뒤 commit_carrot_setting_change를 이어서 호출하세요. 별도 read 호출이나 중간 승인 대화는 필요 없습니다. 대상이 모호하면 먼저 되물으세요. 예: 디바이스 밝기와 외부 HUD 밝기는 다릅니다. percent는 현재값의 절댓값을 기준으로 한 상대 변화이며 percent와 퍼센트포인트를 혼동하지 마세요. 값은 웹에 표시되는 저장 정수이고 제목의 x0.01 등 배율을 확인하세요. step은 웹 버튼의 증감 단위이며 허용값의 배수나 min 기준 격자가 아닙니다. 범위 안의 정수 absolute는 요청값 그대로 지정하세요. 예: CruiseMaxVals0의 min=1, step=5여도 240을 지정할 수 있으며 241로 바꾸지 마세요. 상대 변화의 소수 결과는 기기에서 정수로 반올림합니다. 명확한 사용자 변경 명령 자체가 승인입니다. 범위 안이면 prepared의 대상과 계산값이 원래 명령에 맞는지 내부 확인하고 재질문 없이 commit_carrot_setting_change(confirmed=true)를 호출하세요. commit_required는 내부 저장 단계이며 사용자가 누르는 승인 버튼이 아닙니다. 대상·값이 모호하거나 AI가 추천한 값이면 먼저 필요한 내용만 물으세요. 범위 밖이면 변경하지 말고 범위만 짧게 알려주세요. 허용 범위를 매번 길게 낭독하지 마세요. 성공은 “가속도설정 0을 240으로 변경했어요”처럼 한 문장으로 안내하세요. 실패는 실제 이유와 필요한 조치만 말하고 권한·코드 문제로 추측하지 마세요. queued이면 결과를 확인하되 실패·만료·저장 결과 미확인 후 자동 재실행하지 마세요. 정정하면 새 제안을 만들고 취소하면 cancel_carrot_setting_change를 호출하세요. completed만으로 성공이라 말하지 말고 result.status가 applied이며 current가 확인된 경우만 변경 완료로 말하세요. queued는 대기, expired는 미확인입니다. 막연한 주행 증상만으로 설정이나 원인을 추측하지 마세요. 지원하는 주행 관련 설정도 같은 명확한 명령으로 주행 중 변경할 수 있습니다. 저장 완료와 실제 제어 반영 시점은 다를 수 있으므로 즉시 주행 동작이 바뀐다고 단정하지 마세요. 운전 명령은 별도의 carrot.drive 권한과 도구를 사용합니다. 진단 로그 전송은 별도의 carrot.tools 도구 권한으로만 요청할 수 있습니다.`;
  const activeInstructions = sessionId ? accountInstructions.replace(/설정 화면 열기만 허용하며 값 변경·차량 제어·로그 전송은 지원하지 않습니다\./, '').replace(/변경 요청은 현재 읽기 전용 시험이라 변경할 수 없다고 알리고 대상 설명까지만 제공하세요\./, '') + '\n' + settingInstructions + '\n' + voiceRules : instructions;
  const security = sessionId ? { securitySchemes: [{ type: 'oauth2', scopes: [ACCOUNT_SCOPE] }] } : {};
  const server = new McpServer({ name: sessionId ? 'carrot-account-assistant' : 'carrot-voice-probe', version: '0.7.8' }, { instructions: activeInstructions });
  const register = (name, description, inputSchema, fn) => server.registerTool(name, {
    description, inputSchema, annotations: readOnly, _meta: security,
  }, async args => { onTool(name); return output(await fn(args)); });
  register('get_carrot_context', '당근 음성 도우미의 권한과 지원 범위를 조회합니다. 설정값·운전 명령·도구 작업은 각각 별도 계정 권한과 기기 허용이 필요합니다. 도구 작업에는 확인 후 진단 로그 전송 요청이 포함됩니다.', {}, () => ({
    mode: sessionId ? 'account_bound_web_navigation' : 'catalog_and_paired_web_navigation', vehicle_connected: sessionId ? null : false,
    can_drive: Boolean(sessionId && scopes.includes(DRIVE_SCOPE)), can_write: Boolean(sessionId && scopes.includes(SETTINGS_SCOPE)), can_navigate: true, can_use_tools: Boolean(sessionId && scopes.includes(TOOLS_SCOPE)), can_upload: Boolean(sessionId && scopes.includes(TOOLS_SCOPE)), navigation_requires_pairing: !sessionId,
    ...(sessionId ? { web: bridge.accountStatus(sessionId), vehicle_state: 'not_observed' } : {}), instructions: activeInstructions, source,
  }));
  register('search_carrot_settings', '깜박이·앞차간격·밝기·음량·정지거리·가속 등 당근 설정 검색. 자연어 질문을 받아 실제 설정 ID와 원본 설명 후보를 반환합니다. 현재 차량값은 없습니다.', {
    query: z.string().trim().min(1).max(300), limit: z.number().int().min(1).max(10).default(5),
  }, ({ query, limit }) => searchSettings(query, limit));
  register('get_carrot_setting', '검색 결과의 정확한 name으로 당근 설정 정의를 조회합니다. 첫 번째 등 지시어는 직전 대화 목록의 name으로 먼저 해석하세요.', {
    name: z.string().regex(/^[A-Za-z][A-Za-z0-9_]{0,100}$/),
  }, ({ name }) => getSetting(name));
  const connectionSchema = sessionId ? {} : { connection_id: z.string().regex(/^[A-Za-z0-9_-]{43}$/) };
  const action = (name, description, schema, fn, read = false) => server.registerTool(name, {
    description, inputSchema: schema, _meta: security,
    annotations: { ...readOnly, readOnlyHint: read, idempotentHint: false },
  }, async args => {
    onTool(name);
    try { return output(fn(args)); }
    catch (e) { return { isError: true, content: [{ type: 'text', text: e.message }] }; }
  });
  if (!sessionId) action('connect_carrot_web', '사용자가 자신의 당근웹에서 생성한 5분 유효 일회용 코드를 제공했을 때만 연결합니다. connection_id는 이 대화에서 보관하고 다음 이용에도 재사용하세요. 연결 해제 또는 인증 실패 전까지 새 코드를 요구하지 마세요. 설정값 조회·변경 권한은 없습니다.',
    { pairing_code: z.string().min(20).max(30) }, ({ pairing_code }) => bridge.pair(pairing_code));
  action('open_carrot_setting', '연결된 당근웹 탭에 설정 화면 열기를 요청합니다. 먼저 검색하여 정확한 name을 고르세요. queued는 대기 중이고 opened가 실제 완료입니다. 폰 브라우저가 멈춘 경우 사용자가 당근웹으로 돌아와야 합니다. 요청은 90초 후 만료됩니다. 설정값을 바꾸지 않습니다.',
    { ...connectionSchema, name: z.string().regex(/^[A-Za-z][A-Za-z0-9_]{0,100}$/) }, ({ connection_id, name }) => {
      if (!getSetting(name).found) throw new Error('Unknown setting name. Search first.');
      return sessionId ? bridge.accountAction(sessionId, 'open', name) : bridge.open(connection_id, name);
    });
  action('get_carrot_web_status', '연결과 마지막 화면 요청 결과를 확인합니다. opened만 완료로 말하세요. unavailable은 해당 차량 웹에 설정이 없는 경우입니다. 현재 차량값은 반환하지 않습니다.',
    connectionSchema, ({ connection_id }) => sessionId ? bridge.accountStatus(sessionId) : bridge.status(connection_id), true);
  action('cancel_carrot_screen_request', '사용자가 화면 요청을 취소하거나 다른 설정으로 정정했을 때 대기 중인 화면 요청을 취소합니다.',
    connectionSchema, ({ connection_id }) => sessionId ? bridge.accountAction(sessionId, 'cancel') : bridge.cancel(connection_id));
  if (sessionId) {
    const toolsTool = (name, description, inputSchema, fn, readOnlyHint = false) => server.registerTool(name, {
      description, inputSchema, annotations: { readOnlyHint, destructiveHint: !readOnlyHint, idempotentHint: readOnlyHint, openWorldHint: true },
      _meta: { securitySchemes: [{ type: 'oauth2', scopes: [ACCOUNT_SCOPE, TOOLS_SCOPE] }] },
    }, async args => {
      onTool(name);
      if (!scopes.includes(TOOLS_SCOPE)) return { isError: true, content: [{ type: 'text', text: '도구 작업 권한이 없습니다. 당근웹에서 도구 실행을 허용하고, ChatGPT > 플러그인 > 설치됨 > CarrotAssist > 다시 연결에서 도구 권한을 선택하세요. 기존 설정·운전 권한은 자동 확장되지 않습니다.' }] };
      try { return output(await fn(args)); }
      catch (e) { return { isError: true, content: [{ type: 'text', text: e.message }] }; }
    });
    toolsTool('list_carrot_tool_actions', '도구 화면에서 열 수 있는 창과 실행 가능한 작업 목록. 임의 버튼 ID·셸·파일 경로는 받지 않습니다.', {}, () => ({
      actions: toolActions, panels: toolPanels,
      rules: '작업은 prepare → 제목·영향 안내 → 후속 사용자 확인 → commit 순서. GitPull 후 재시작은 별개 요청. send_tmux_log는 기기에 저장, server_tmux_log는 기존 서버로 진단 로그 전송 요청이며 주행기록 업로드가 아닙니다. record_start/stop은 화면 녹화. git_reset_*는 HEAD 고정. 파일 복원·QR·클립보드·저장소 초기화는 화면 안내 후 사용자가 선택해야 합니다.',
    }), true);
    toolsTool('open_carrot_tool_panel', '도구 화면이나 지정한 선택창을 엽니다. opened만 화면 열기 완료. needs_input은 해당 버튼을 표시한 상태이며 폰에서 파일·QR·초기화 등을 직접 선택해야 합니다. 작업 실행 완료가 아닙니다.',
      { action: z.enum(toolPanels) }, args => bridge.accountToolsAction(sessionId, 'open', args));
    toolsTool('open_carrot_phone_app', '네이버지도, 네이버맵 또는 Naver Map을 열어 달라는 현재 명령에 사용합니다. CarrotAssist 폰에서 네이버지도 실행을 한 번 요청하고 결과만 말하세요. 미설치와 실행 요청을 구분합니다.',
      { app: z.literal('naver_map') }, () => bridge.accountToolsAction(sessionId, 'open', { action: 'naver_map' }));
    toolsTool('prepare_carrot_tool_action', 'GitPull·재시작·초기화·삭제·백업·진단 로그 전송·화면 녹화 작업을 준비합니다. 아직 실행하지 않습니다. prepared의 title/effect를 안내하고 후속 사용자 확인을 받은 뒤 commit하세요. Git reset 모드나 로그 종류가 모호하면 먼저 물으세요. 화면 녹화 외 작업은 정차·보조 해제가 필요합니다.',
      { action: z.enum(toolActions) }, args => bridge.accountToolsAction(sessionId, 'prepare', args));
    toolsTool('commit_carrot_tool_action', '준비된 작업의 영향에 대해 사용자가 후속 확인했을 때 한 번 실행합니다. 불확실한 결과를 자동 재실행하지 않습니다. running은 job_id로 상태를 조회하세요. requested는 재시작·전송·녹화 등의 요청 접수이며 실제 완료가 아닙니다.',
      { proposal_id: z.string().regex(/^[A-Za-z0-9_-]{43}$/) }, args => bridge.accountToolsAction(sessionId, 'commit', args));
    toolsTool('get_carrot_tool_request', '마지막 요청 전달 상태를 확인합니다. 바깥 status completed는 응답 수신일 뿐이며 실제 작업은 result.status를 따릅니다. queued/expired는 성공이 아닙니다.', {}, () => bridge.accountToolsStatus(sessionId), true);
    toolsTool('get_carrot_tool_job', '이 연결에서 시작한 job_id의 실제 작업 상태를 읽습니다. requested는 요청 접수만 확인한 것입니다. 완료된 재시작이나 업로드로 과장하지 마세요. unknown/failed나 연결 끊김 뒤 자동 재실행하지 않습니다.',
      { job_id: z.string().regex(/^[a-f0-9]{12}$/) }, args => bridge.accountToolsAction(sessionId, 'status', args), true);
    toolsTool('cancel_carrot_tool_proposal', '아직 실행하지 않은 도구 작업 제안을 취소합니다. 이미 시작된 작업을 중단하지 않습니다.', {}, () => bridge.accountToolsAction(sessionId, 'cancel'));
    const driveTool = (name, description, schema, fn, readOnlyHint = false) => server.registerTool(name, {
      description, inputSchema: schema, annotations: { readOnlyHint, destructiveHint: !readOnlyHint, idempotentHint: readOnlyHint, openWorldHint: true },
      _meta: { securitySchemes: [{ type: 'oauth2', scopes: [ACCOUNT_SCOPE, DRIVE_SCOPE] }] },
    }, async args => {
      onTool(name);
      if (!scopes.includes(DRIVE_SCOPE)) return { isError: true, content: [{ type: 'text', text: '운전 명령은 별도 권한이 필요합니다. 기존 설정 권한으로 실행하지 않습니다.' }] };
      try { return output(await fn(args)); }
      catch (e) { return { isError: true, content: [{ type: 'text', text: e.message }] }; }
    });
    driveTool('request_carrot_drive', '“좌측 차로 변경해”, “우측 차로 변경해” 등 명확한 현재 명령에서 바로 한 번 호출합니다. 시동·권한·운전 가능 여부를 미리 물을 필요가 없습니다. 첫 출력은 말 없이 이 도구 호출입니다. “응”, “바로 요청할게”, “연결부터 확인할게”를 먼저 출력하지 마세요. get_carrot_context/get_carrot_web_status/설정 검색/prepare/commit을 선행하지 마세요. 좌측·왼쪽 차로변경은 lane_left, 우측·오른쪽은 lane_right입니다. 방향·목표가 모호하면 필요한 값만 물으세요. lane_left/right는 한 차선, speed_up은 리모컨 RES/+ 1회, speed_down은 value km/h 감소(그냥 내려는 10), speed_set은 value km/h 지정입니다. 첫 응답의 queued는 중계 접수, waiting/running은 차량 대기/진행입니다. 받은 상태를 한 문장으로 답하고 사용자가 결과를 물을 때만 상태를 조회하세요. blocked 또는 duplicate는 반환된 message만 말하고 끝냅니다. 차량 확인, 재명령, 재시도 여부를 묻지 마세요. 만료·실패·응답 미확인 시 자동 재실행하지 마세요.',
      { action: z.enum(['lane_left', 'lane_right', 'speed_up', 'speed_down', 'speed_set']), value: z.number().int().min(1).max(199).optional() }, args => {
        if (['speed_down', 'speed_set'].includes(args.action) !== (args.value !== undefined)) throw new Error('A value is required only for speed_down and speed_set');
        return bridge.accountDriveAction(sessionId, args);
      });
    driveTool('get_carrot_drive_request', '사용자가 이미 보낸 명령의 결과를 물을 때만 현재 결과를 조회합니다. 명령 전 사전 검사나 자동 후속 조회에는 사용하지 마세요. 차선변경 completed만 완료이며, 속도 applied/unchanged는 current 설정속도를 안내합니다. 실제 차량 속도가 도달했다고 말하지 마세요. 실행 도구를 재호출하지 마세요.', {}, () => bridge.accountDriveStatus(sessionId), true);
    const settingTool = (name, description, inputSchema, fn, readOnlyHint = false) => server.registerTool(name, {
      description, inputSchema, annotations: { ...readOnly, readOnlyHint, idempotentHint: false },
      _meta: { securitySchemes: [{ type: 'oauth2', scopes: [ACCOUNT_SCOPE, SETTINGS_SCOPE] }] },
    }, async args => {
      onTool(name);
      if (!scopes.includes(SETTINGS_SCOPE)) return { isError: true, content: [{ type: 'text', text: '설정값 권한이 없습니다. ChatGPT 연결을 다시 승인하여 설정값 조회·변경 권한을 추가하세요. 기존 화면 열기 권한은 변경되지 않습니다.' }] };
      try { return output(await fn(args)); }
      catch (e) { return { isError: true, content: [{ type: 'text', text: e.message }] }; }
    });
    const name = z.string().regex(/^[A-Za-z][A-Za-z0-9_]{0,100}$/);
    const identified = (operation, args) => {
      if (!getSetting(args.name).found) throw new Error('Search the public catalog for an exact setting name first');
      return bridge.accountSettingAction(sessionId, operation, args);
    };
    settingTool('read_carrot_setting_value', '연결된 기기의 한 설정 현재값과 범위를 읽습니다. 카탈로그 기본값이 아닙니다. 결과가 queued이면 get_carrot_setting_request로 확인하세요.', { name }, args => identified('read', args), true);
    settingTool('prepare_carrot_setting_change', '값을 저장하지 않고 현재값·범위·변경단위를 확인한 제안을 만듭니다. absolute는 범위 안의 저장 정수를 그대로 지정합니다(step 배수 제한 없음). delta는 정수 증감, percent는 current + abs(current)*value/100이며 계산 결과를 정수로 반올림합니다. 사용자가 대상과 값·증감량을 명확히 지정했다면 이것이 승인입니다. prepared 결과를 내부 확인하고 추가 질문 없이 commit을 이어서 호출하세요. 준비 완료는 저장 완료가 아닙니다. 모호한 요청이나 AI 추천값은 먼저 확인하세요.',
      { name, mode: z.enum(['absolute', 'delta', 'percent']), value: z.number().finite().min(-1000000).max(1000000) }, args => identified('prepare', args));
    settingTool('commit_carrot_setting_change', '사용자의 명확한 변경 명령과 일치하는 직전 prepared 결과를 저장합니다. 원래 명령이 승인이므로 같은 값을 다시 묻지 말고 confirmed=true로 호출하세요. AI 추천값이나 모호한 요청은 사용자 확인이 먼저 필요합니다. 60초 제안 만료·현재값 변화·범위·변경단위를 기기에서 재검사합니다. 지원하는 주행 관련 설정도 주행 중 적용할 수 있습니다. applied 결과만 완료입니다.',
      { proposal_id: z.string().regex(/^[A-Za-z0-9_-]{43}$/), confirmed: z.literal(true) }, ({ proposal_id }) => bridge.accountSettingAction(sessionId, 'commit', { proposal_id }));
    settingTool('get_carrot_setting_request', '마지막 설정 조회·변경 요청의 상태와 기기 응답을 확인합니다. queued/expired는 변경 완료가 아닙니다. result.status applied와 current 확인 후에만 완료로 안내하세요.', {}, () => bridge.accountSettingStatus(sessionId), true);
    settingTool('cancel_carrot_setting_change', '미적용 설정 제안을 취소합니다. 이미 적용한 값을 원복하라는 명확한 명령은 원래 값으로 새 준비·저장 절차를 실행합니다.', {}, () => bridge.accountSettingAction(sessionId, 'cancel'));
  }
  return server;
}

export function createApp({ onTool = () => {}, host = '127.0.0.1', publicHostname, requestsPerMinute = 600, statePath, oauthStatePath, oauthIssuer } = {}) {
  const allowedHosts = ['127.0.0.1', 'localhost', '[::1]'];
  if (publicHostname) allowedHosts.push(publicHostname);
  const app = createMcpExpressApp({ host, allowedHosts });
  let accountAuth;
  const bridge = createBridge({ statePath, isAccountLinked: id => accountAuth?.hasConnection(id) || false,
    accountActivity: id => accountAuth?.accountActivity(id) || 0 });
  const issuer = oauthIssuer || (publicHostname ? `https://${publicHostname}` : null);
  if (issuer) accountAuth = createAccountAuth({ issuer, bridge, statePath: oauthStatePath });
  let windowStart = Date.now(), requests = 0;
  app.disable('x-powered-by');
  app.use((req, res, next) => {
    res.setHeader('Cache-Control', 'no-store');
    const origin = req.headers.origin;
    if (['/mcp', '/account/mcp'].includes(req.path) && origin && !/^https?:\/\/(localhost|127\.0\.0\.1)(:\d+)?$/.test(origin)) return res.status(403).end();
    if (['/mcp', '/account/mcp'].includes(req.path)) {
      if (Date.now() - windowStart >= 60000) { windowStart = Date.now(); requests = 0; }
      if (++requests > requestsPerMinute) return res.status(429).set('Retry-After', '60').end();
    }
    next();
  });
  installBridgeRoutes(app, bridge);
  if (accountAuth) installAccountAuth(app, accountAuth, issuer);
  app.get('/health', (_req, res) => res.json({ ok: true, version: '0.7.8', mode: 'catalog_and_paired_web_navigation', account_auth: Boolean(accountAuth), count: source.count, commit: source.commit }));
  app.get('/', (_req, res) => res.type('html').send('<!doctype html><html lang="ko"><meta charset="utf-8"><title>CarrotAssist 연결</title><h1>CarrotAssist 연결</h1><p>MCP 주소: /account/mcp · 인증: OAuth</p><p>설정 검색·설명·화면 열기와 별도로 허용한 설정값 조회·변경을 지원합니다. 별도 허용 시 음성 차선변경·설정속도 요청도 지원합니다. 도구 작업은 별도 허용과 확인 후 실행하며 진단 로그 전송 요청도 포함됩니다.</p><p><a href="https://github.com/ajouatom/openpilot/wiki/Guide-CarrotAssist">설치 및 사용 설명서</a></p>'));
  const handleMcp = async (req, res) => {
    const server = createServer(onTool, bridge, req.auth?.extra?.sessionId || null, req.auth?.scopes || []);
    const transport = new StreamableHTTPServerTransport({ sessionIdGenerator: undefined, enableJsonResponse: true });
    res.on('close', () => { void transport.close(); void server.close(); });
    try {
      await server.connect(transport);
      await transport.handleRequest(req, res, req.body);
    } catch {
      if (!res.headersSent) res.status(500).json({ jsonrpc: '2.0', id: null, error: { code: -32603, message: 'MCP request failed' } });
    }
  };
  app.post('/mcp', handleMcp);
  if (accountAuth) {
    app.use('/account/mcp', requireBearerAuth({ verifier: accountAuth, requiredScopes: [ACCOUNT_SCOPE], expectedResource: accountAuth.resource,
      resourceMetadataUrl: `${issuer}/.well-known/oauth-protected-resource/account/mcp` }));
    app.post('/account/mcp', handleMcp);
    app.all('/account/mcp', (_req, res) => res.status(405).set('Allow', 'POST').end());
  }
  app.all('/mcp', (_req, res) => res.status(405).set('Allow', 'POST').end());
  return app;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  const port = Number(process.env.PORT || 18770);
  const host = process.env.HOST || '127.0.0.1';
  const publicHostname = process.env.PUBLIC_HOSTNAME;
  if (host !== '127.0.0.1' && !publicHostname) throw new Error('PUBLIC_HOSTNAME required for non-local binding');
  const app = createApp({ host, publicHostname, statePath: process.env.MCP_STATE_PATH, oauthStatePath: process.env.MCP_OAUTH_STATE_PATH, onTool: name => console.log(JSON.stringify({ time: new Date().toISOString(), tool: name })) });
  const http = app.listen(port, host, () => console.log(`Carrot MCP ready on ${host}:${port}/mcp`));
  http.requestTimeout = 15000;
  http.headersTimeout = 10000;
}
