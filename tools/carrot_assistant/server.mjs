import { pathToFileURL } from 'node:url';
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { StreamableHTTPServerTransport } from '@modelcontextprotocol/sdk/server/streamableHttp.js';
import { createMcpExpressApp } from '@modelcontextprotocol/sdk/server/express.js';
import { z } from 'zod';
import { source, instructions, getSetting, searchSettings } from './catalog.mjs';
import { createBridge, installBridgeRoutes } from './bridge.mjs';
import { createAccountAuth, installAccountAuth, ACCOUNT_SCOPE } from './oauth.mjs';
import { requireBearerAuth } from '@modelcontextprotocol/sdk/server/auth/middleware/bearerAuth.js';

const readOnly = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: false };
const output = data => ({ content: [{ type: 'text', text: JSON.stringify(data) }], structuredContent: data });
export function createServer(onTool = () => {}, bridge = createBridge(), sessionId = null) {
  const accountInstructions = `당근파일럿 음성 도우미입니다. 이 요청은 이미 인증된 기기에 연결되어 있습니다. 새 대화나 안드로이드 미니 음성창에서도 연결 코드를 요구하지 마세요. get_carrot_web_status로 본인 기기를 확인하고 open_carrot_setting에 정확한 name만 전달하세요. 사용자 ID나 connection_id는 필요 없습니다. 브라우저가 오프라인이면 당근웹을 열도록 안내하고 재연결 코드를 요구하지 마세요. 인증 실패만 재인증이 필요합니다. 설정 화면 열기만 허용하며 값 변경·차량 제어·로그 전송은 지원하지 않습니다. queued는 대기이며 opened만 완료입니다.\n${instructions.slice(instructions.indexOf('설정 질문에는'))}`;
  const activeInstructions = sessionId ? accountInstructions : instructions;
  const security = sessionId ? { securitySchemes: [{ type: 'oauth2', scopes: [ACCOUNT_SCOPE] }] } : {};
  const server = new McpServer({ name: sessionId ? 'carrot-account-assistant' : 'carrot-voice-probe', version: '0.4.1' }, { instructions: activeInstructions });
  const register = (name, description, inputSchema, fn) => server.registerTool(name, {
    description, inputSchema, annotations: readOnly, _meta: security,
  }, async args => { onTool(name); return output(await fn(args)); });
  register('get_carrot_context', '당근 음성시험 지침을 조회합니다. 설정 설명 조회와 코드로 연결한 웹의 설정 화면 열기를 지원합니다. 현재값·차량 제어·업로드는 없습니다.', {}, () => ({
    mode: sessionId ? 'account_bound_web_navigation' : 'catalog_and_paired_web_navigation', vehicle_connected: false,
    can_write: false, can_navigate: true, can_upload: false, navigation_requires_pairing: !sessionId,
    ...(sessionId ? { web: bridge.accountStatus(sessionId) } : {}), instructions: activeInstructions, source,
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
  return server;
}

export function createApp({ onTool = () => {}, host = '127.0.0.1', publicHostname, requestsPerMinute = 600, statePath, oauthStatePath, oauthIssuer } = {}) {
  const allowedHosts = ['127.0.0.1', 'localhost', '[::1]'];
  if (publicHostname) allowedHosts.push(publicHostname);
  const app = createMcpExpressApp({ host, allowedHosts });
  let accountAuth;
  const bridge = createBridge({ statePath, isAccountLinked: id => accountAuth?.hasConnection(id) || false });
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
  app.get('/health', (_req, res) => res.json({ ok: true, version: '0.4.1', mode: 'catalog_and_paired_web_navigation', account_auth: Boolean(accountAuth), count: source.count, commit: source.commit }));
  app.get('/', (_req, res) => res.type('html').send('<!doctype html><html lang="ko"><meta charset="utf-8"><title>당근 ChatGPT 연결</title><h1>당근 ChatGPT 연결</h1><p>MCP 주소: /mcp</p><p>공개 설정 설명 조회와 코드로 연결한 당근웹의 설정 화면 열기를 지원합니다. 실제 설정값 변경·로그 업로드는 지원하지 않습니다.</p>'));
  const handleMcp = async (req, res) => {
    const server = createServer(onTool, bridge, req.auth?.extra?.sessionId || null);
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
  const http = app.listen(port, host, () => console.log(`Carrot read-only MCP ready on ${host}:${port}/mcp`));
  http.requestTimeout = 15000;
  http.headersTimeout = 10000;
}
