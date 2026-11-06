import { randomBytes, createHash } from 'node:crypto';
import { readFileSync, writeFileSync, renameSync } from 'node:fs';
import express from 'express';
import { z } from 'zod';
import { OAuthClientInformationFullSchema } from '@modelcontextprotocol/sdk/shared/auth.js';
import { mcpAuthRouter } from '@modelcontextprotocol/sdk/server/auth/router.js';
import { InvalidClientMetadataError, InvalidGrantError, InvalidRequestError, InvalidScopeError, InvalidTokenError } from '@modelcontextprotocol/sdk/server/auth/errors.js';

export const ACCOUNT_SCOPE = 'carrot.screen';
const random = () => randomBytes(32).toString('base64url');
const hash = value => createHash('sha256').update(String(value)).digest('hex');
const hex = z.string().regex(/^[a-f0-9]{64}$/);
const id = z.string().regex(/^[A-Za-z0-9_-]{43}$/);
const grantSchema = z.object({ id, sessionId: id, clientId: z.string(), refreshHash: hex,
  refreshExpiresAt: z.number().finite(), used: z.array(hex).max(64),
  access: z.array(z.object({ hash: hex, expiresAt: z.number().finite() })).max(8) });
const storeSchema = z.object({ version: z.literal(1), clients: z.array(OAuthClientInformationFullSchema).max(2000), grants: z.array(grantSchema).max(2000) });
const escape = value => String(value).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]);

// DCR only: no fetching client-controlled URLs or trusting ChatGPT-supplied user IDs.
export function allowedCallback(value) {
  try {
    const u = new URL(value);
    return u.origin === 'https://chatgpt.com' && !u.username && !u.password && !u.search && !u.hash
      && (u.pathname === '/connector_platform_oauth_redirect' || /^\/connector\/oauth\/[A-Za-z0-9_-]+$/.test(u.pathname));
  } catch { return false; }
}

export function createAccountAuth({ issuer, bridge, statePath, now = Date.now }) {
  issuer = new URL(issuer).origin;
  const resource = `${issuer}/account/mcp`;
  let clients = new Map(), grants = new Map();
  const flows = new Map(), codes = new Map();
  function save(nextClients = clients, nextGrants = grants) {
    if (statePath) {
      writeFileSync(`${statePath}.tmp`, JSON.stringify({ version: 1, clients: [...nextClients.values()], grants: [...nextGrants.values()] }), { mode: 0o600 });
      renameSync(`${statePath}.tmp`, statePath);
    }
    clients = nextClients; grants = nextGrants;
  }
  if (statePath) {
    let saved;
    try { saved = storeSchema.parse(JSON.parse(readFileSync(statePath, 'utf8'))); }
    catch (e) { if (e.code !== 'ENOENT') throw e; }
    if (saved) {
      clients = new Map(saved.clients.map(c => [c.client_id, c]));
      grants = new Map(saved.grants.map(g => [g.id, g]));
      const credentials = saved.grants.flatMap(g => [g.refreshHash, ...g.used, ...g.access.map(a => a.hash)]);
      if (new Set(credentials).size !== credentials.length || clients.size !== saved.clients.length || grants.size !== saved.grants.length
        || saved.clients.some(c => !c.redirect_uris.every(allowedCallback))
        || saved.grants.some(g => !clients.has(g.clientId))) throw new Error('Invalid OAuth store');
    }
  }
  const alive = g => {
    if (g.refreshExpiresAt <= now()) return false;
    try { return bridge.accountStatus(g.sessionId).connected; } catch { return false; }
  };
  const scopeCheck = scopes => { if (scopes?.some(s => s !== ACCOUNT_SCOPE)) throw new InvalidScopeError('Only setting screen navigation is supported.'); };
  const resourceCheck = value => { if (value?.href !== resource) throw new InvalidRequestError('Incorrect resource.'); };
  function sweep() {
    for (const map of [flows, codes]) for (const [key, value] of map) if (value.expiresAt <= now()) map.delete(key);
  }
  function codeFor(client, value) {
    sweep(); const c = codes.get(hash(value));
    if (!c || c.clientId !== client.client_id) throw new InvalidGrantError('Invalid or expired authorization code.');
    return c;
  }
  function issue(clientId, sessionId, previous) {
    const accessToken = random(), refreshToken = random();
    const grant = { id: previous?.id || random(), clientId, sessionId, refreshHash: hash(refreshToken),
      refreshExpiresAt: now() + 365 * 86400000,
      used: previous ? [...previous.used, previous.refreshHash].slice(-64) : [],
      access: [...(previous?.access || []).filter(a => a.expiresAt > now()).slice(-7), { hash: hash(accessToken), expiresAt: now() + 3600000 }] };
    const next = new Map([...grants].filter(([, g]) => g.refreshExpiresAt > now()));
    if (!next.has(grant.id) && next.size >= 2000) throw new InvalidRequestError('Connection capacity reached.');
    next.set(grant.id, grant); save(clients, next);
    return { access_token: accessToken, refresh_token: refreshToken, token_type: 'Bearer', expires_in: 3600, scope: ACCOUNT_SCOPE };
  }
  const provider = {
    resource,
    // Does not call bridge.accountStatus (which includes this display flag).
    hasConnection: sessionId => [...grants.values()].some(g => g.sessionId === sessionId && g.refreshExpiresAt > now()),
    clientsStore: {
      getClient: clientId => clients.get(clientId),
      registerClient: async client => {
        if (clients.size >= 2000 || !client.redirect_uris?.length || !client.redirect_uris.every(allowedCallback)
          || !['none', 'client_secret_post'].includes(client.token_endpoint_auth_method)
          || client.grant_types?.some(g => !['authorization_code', 'refresh_token'].includes(g))) {
          throw new InvalidClientMetadataError('Only ChatGPT authorization-code clients are supported.');
        }
        const next = new Map(clients); next.set(client.client_id, client); save(next); return client;
      },
    },
    async authorize(client, params, res) {
      scopeCheck(params.scopes); resourceCheck(params.resource); sweep();
      if (!allowedCallback(params.redirectUri)) throw new InvalidRequestError('Invalid authorization callback.');
      if (!/^[A-Za-z0-9_-]{43}$/.test(params.codeChallenge) || (params.state?.length || 0) > 2048 || flows.size >= 1000) throw new InvalidRequestError('Invalid authorization request.');
      const flow = random(), cookie = random();
      flows.set(flow, { clientId: client.client_id, redirectUri: params.redirectUri, state: params.state,
        challenge: params.codeChallenge, cookieHash: hash(cookie), expiresAt: now() + 600000, attempts: 0 });
      res.cookie('carrot_oauth', cookie, { httpOnly: true, secure: issuer.startsWith('https:'), sameSite: 'lax', path: '/oauth/approve', maxAge: 600000 });
      // Chromium checks form-action again after the POST's 303 redirect. Permit
      // only this validated ChatGPT callback as well as our own consent POST.
      // no-referrer also makes a browser form POST's Origin null. same-origin
      // preserves our CSRF origin check without disclosing the URL to ChatGPT.
      res.set({ 'Content-Security-Policy': `default-src 'none'; style-src 'unsafe-inline'; form-action 'self' ${params.redirectUri}; frame-ancestors 'none'; base-uri 'none'`, 'Referrer-Policy': 'same-origin', 'X-Content-Type-Options': 'nosniff' });
      res.type('html').send(`<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>당근 기기 연결</title><style>body{font:18px system-ui;background:#10151c;color:#eee;max-width:520px;margin:40px auto;padding:20px;line-height:1.6}input,button{font:inherit;box-sizing:border-box;padding:12px;width:100%;margin:12px 0}button{background:#ed9144;border:0;border-radius:10px}small{color:#bbb}</style><h1>ChatGPT에 당근 기기 연결</h1><p>한 번 연결하면 새 대화와 미니 음성창에서도 같은 당근웹을 사용할 수 있습니다.</p><p>당근웹의 ChatGPT 연결 설정에서 <b>미니모드 연결 코드</b>를 만든 뒤 아래에 입력하세요. 이 코드는 대화에 보내지 마세요.</p><form method="post" action="/oauth/approve"><input type="hidden" name="flow" value="${escape(flow)}"><label for="code">미니모드 연결 코드 (5분 유효)</label><input id="code" name="code" required maxlength="30" autocomplete="off" autocapitalize="characters" spellcheck="false"><p>허용: 설정 검색·설명 및 해당 웹의 설정 화면 열기.<br>설정값 변경·차량 제어·로그 전송은 허용하지 않습니다.</p><button type="submit">이 기기 연결 승인</button></form><small>연결은 당근웹에서 해제할 수 있습니다. 인증은 자동 갱신되며, 1년 동안 사용하지 않으면 다시 연결해야 합니다. 다른 사람이 보낸 코드는 입력하지 마세요.</small></html>`);
    },
    async challengeForAuthorizationCode(client, code) { return codeFor(client, code).challenge; },
    async exchangeAuthorizationCode(client, code, _verifier, redirectUri, target) {
      resourceCheck(target); const c = codeFor(client, code);
      if (redirectUri !== c.redirectUri) throw new InvalidGrantError('Redirect URI mismatch.');
      bridge.accountStatus(c.sessionId);
      const result = issue(client.client_id, c.sessionId); codes.delete(hash(code)); return result;
    },
    async exchangeRefreshToken(client, refreshToken, scopes, target) {
      resourceCheck(target); scopeCheck(scopes); const digest = hash(refreshToken);
      for (const g of grants.values()) {
        if (g.clientId !== client.client_id) continue;
        if (g.used.includes(digest)) { const next = new Map(grants); next.delete(g.id); save(clients, next); throw new InvalidGrantError('Refresh token reuse. Reauthorize this connection.'); }
        if (g.refreshHash === digest && alive(g)) return issue(g.clientId, g.sessionId, g);
      }
      throw new InvalidGrantError('Connection expired or revoked.');
    },
    async verifyAccessToken(token) {
      const digest = hash(token);
      for (const g of grants.values()) {
        const access = g.access.find(a => a.hash === digest && a.expiresAt > now());
        if (access && alive(g)) return { token, clientId: g.clientId, scopes: [ACCOUNT_SCOPE], expiresAt: access.expiresAt / 1000, resource: new URL(resource), extra: { sessionId: g.sessionId } };
      }
      throw new InvalidTokenError('Connection expired or revoked.');
    },
    async revokeToken(client, request) {
      const digest = hash(request.token), next = new Map(grants);
      for (const g of grants.values()) if (g.clientId === client.client_id && (g.refreshHash === digest || g.used.includes(digest) || g.access.some(a => a.hash === digest))) next.delete(g.id);
      save(clients, next);
    },
    approve(req, res) {
      sweep(); const flow = flows.get(req.body?.flow);
      const cookie = String(req.headers.cookie || '').split(';').map(s => s.trim()).find(s => s.startsWith('carrot_oauth='))?.slice('carrot_oauth='.length);
      if (!flow) return res.status(403).type('text').send('연결 요청이 만료됐습니다. ChatGPT에서 연결을 다시 시작하세요.');
      if (req.headers.origin !== issuer || hash(cookie || '') !== flow.cookieHash) return res.status(403).type('text').send('연결 요청의 보안 확인에 실패했습니다. 승인 화면을 연 브라우저에서 진행하거나 ChatGPT에서 연결을 다시 시작하세요.');
      if (++flow.attempts > 5) return res.status(403).type('text').send('입력 시도 횟수를 초과했습니다. ChatGPT에서 연결을 다시 시작하세요.');
      try {
        const linked = bridge.claimAccountCode(req.body.code);
        const code = random();
        codes.set(hash(code), { ...flow, sessionId: linked.id, expiresAt: now() + 120000 });
        flows.delete(req.body.flow);
        const redirect = new URL(flow.redirectUri); redirect.searchParams.set('code', code);
        if (flow.state) redirect.searchParams.set('state', flow.state);
        res.set('Referrer-Policy', 'no-referrer').redirect(303, redirect.href);
      } catch {
        res.status(400).type('text').send('코드가 만료됐거나 올바르지 않습니다. 당근웹에서 미니모드 코드를 새로 만든 뒤 뒤로 돌아가 다시 입력하세요.');
      }
    },
  };
  return provider;
}

export function installAccountAuth(app, provider, issuer) {
  // DSM is the sole reverse proxy. Keep global proxy-IP limits; never trust a caller's X-Forwarded-For.
  const validate = { xForwardedForHeader: false };
  app.use(mcpAuthRouter({ provider, issuerUrl: new URL(issuer), resourceServerUrl: new URL(provider.resource),
    scopesSupported: [ACCOUNT_SCOPE], resourceName: '당근파일럿 기기 연결',
    authorizationOptions: { rateLimit: { validate } }, revocationOptions: { rateLimit: { validate } },
    clientRegistrationOptions: { clientSecretExpirySeconds: 0, rateLimit: { max: 200, validate } }, tokenOptions: { rateLimit: { max: 1200, validate } } }));
  app.post('/oauth/approve', express.urlencoded({ extended: false, limit: '4kb' }), (req, res) => provider.approve(req, res));
}
