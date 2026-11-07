import { randomBytes, createHash } from 'node:crypto';
import { readFileSync, writeFileSync, renameSync } from 'node:fs';
import { createSettingBridge } from './setting_bridge.mjs';
import { createDriveBridge } from './drive_bridge.mjs';
import { createToolsBridge } from './tools_bridge.mjs';

const digest = value => createHash('sha256').update(String(value)).digest('hex');
const token = () => randomBytes(32).toString('base64url');
const fail = (message, status = 400) => { throw Object.assign(new Error(message), { status }); };
export function allowedWebOrigin(origin) {
  try {
    const u = new URL(origin);
    const local = u.hostname === 'localhost' || u.hostname === '127.0.0.1' || u.hostname === '[::1]';
    const ipv4 = /^\d{1,3}(\.\d{1,3}){3}$/.test(u.hostname) && u.hostname.split('.').every(n => Number(n) <= 255);
    const lan = (ipv4 && /^(10\.|192\.168\.|172\.(1[6-9]|2\d|3[01])\.)/.test(u.hostname))
      || u.hostname.endsWith('.local') || /^[a-z0-9-]+$/i.test(u.hostname);
    return u.origin === origin && ['http:', 'https:'].includes(u.protocol) && !u.username && !u.password
      && (local || (lan && u.port === '7000'));
  } catch { return false; }
}

// Only screen navigation is supported. No values, device credentials or log files.
// Persist only paired capability hashes. Pending commands never survive restart.
export function createBridge({ now = Date.now, maxSessions = 1000, statePath, isAccountLinked = () => false, accountActivity = () => 0 } = {}) {
  const settings = createSettingBridge(now);
  const toolJobs = createToolsBridge(now);
  const driving = createDriveBridge(now), waiters = new Map();
  const wake = id => waiters.get(id)?.();
  const sessions = new Map(), browsers = new Map(), controls = new Map(), codes = new Map();
  const accountCodes = new Map();
  const record = s => ({ id: s.id, origin: s.origin, label: s.label, browserHash: s.browserHash, controlHash: s.controlHash });
  function persist(replacement, removed) {
    if (!statePath) return;
    const rows = [...sessions.values()].filter(s => s.controlHash && s.id !== removed && s.id !== replacement?.id).map(record);
    if (replacement) rows.push(record(replacement));
    writeFileSync(`${statePath}.tmp`, JSON.stringify({ version: 1, connections: rows }), { mode: 0o600 });
    renameSync(`${statePath}.tmp`, statePath);
  }
  if (statePath) {
    let saved;
    try { saved = JSON.parse(readFileSync(statePath, 'utf8')); }
    catch (e) { if (e.code !== 'ENOENT') throw e; }
    if (saved) {
      if (saved.version !== 1 || !Array.isArray(saved.connections) || saved.connections.length > maxSessions) throw new Error('Invalid connection store');
      for (const row of saved.connections) {
        if (!allowedWebOrigin(row.origin) || !/^[A-Za-z0-9_-]{43}$/.test(row.id)
          || !/^[a-f0-9]{64}$/.test(row.browserHash) || !/^[a-f0-9]{64}$/.test(row.controlHash)
          || typeof row.label !== 'string' || row.label.length > 80
          || sessions.has(row.id) || browsers.has(row.browserHash) || controls.has(row.controlHash)) throw new Error('Invalid saved connection');
        const s = { ...record(row), expiresAt: null, lastSeen: 0, command: null, pollAt: -Infinity };
        sessions.set(s.id, s); browsers.set(s.browserHash, s); controls.set(s.controlHash, s);
      }
    }
  }
  const remove = s => {
    settings.remove(s.id); driving.remove(s.id); toolJobs.remove(s.id); wake(s.id);
    if (s.controlHash) persist(null, s.id);
    sessions.delete(s.id); browsers.delete(s.browserHash); codes.delete(s.codeHash);
    if (s.controlHash) controls.delete(s.controlHash);
  };
  function sweep() {
    for (const [hash, link] of accountCodes) if (link.expiresAt <= now() || !sessions.has(link.id)) accountCodes.delete(hash);
    for (const s of sessions.values()) {
      if (!s.controlHash && s.expiresAt <= now()) { remove(s); continue; }
      if (s.command?.status === 'queued' && s.command.expiresAt <= now()) s.command.status = 'expired';
    }
  }
  function lookup(map, key) {
    sweep();
    const s = map.get(digest(key));
    if (!s) fail('Connection expired or invalid. Pair again in Carrot Web.', 401);
    return s;
  }
  function status(s) {
    return { connected: Boolean(s.controlHash), account_linked: isAccountLinked(s.id), label: s.label, expires_at: s.expiresAt,
      browser_online: s.lastSeen > 0 && now() - s.lastSeen < 15000,
      command: s.command ? { id: s.command.id, name: s.command.name, status: s.command.status,
        expires_at: s.command.expiresAt } : null };
  }
  return {
    async waitForBrowser(key, origin) {
      const s = lookup(browsers, key);
      if (s.origin !== origin) fail('Browser origin mismatch.', 403);
      if (waiters.has(s.id)) fail('A browser poll is already pending.', 409);
      s.lastSeen = now();
      if (!settings.poll(s.id) && !driving.poll(s.id) && !toolJobs.poll(s.id) && s.command?.status !== 'queued') {
        await new Promise(resolve => {
          const timer = setTimeout(() => { waiters.delete(s.id); resolve(); }, 7500);
          waiters.set(s.id, () => { clearTimeout(timer); waiters.delete(s.id); resolve(); });
        });
      }
      return this.browser(key, origin, 'poll');
    },
    accountDriveStatus(id) {
      this.accountStatus(id);
      return driving.status(id);
    },
    async accountDriveAction(id, fields) {
      const state = this.accountStatus(id);
      if (!state.browser_online) fail('Carrot Web is offline. Do not retry automatically.', 409);
      const queued = driving.queue(id, fields);
      if (queued.submitted === false) return queued;
      wake(id);
      // Give a fast local acknowledgement a chance to arrive. A voice reply
      // need not wait for the vehicle to finish a maneuver.
      const deadline = performance.now() + 400;
      let status = queued;
      while (status.status === 'queued' && performance.now() < deadline) {
        await new Promise(resolve => setTimeout(resolve, 25));
        status = this.accountDriveStatus(id);
        if (status.request_id !== queued.request_id) return { status: 'none' };
      }
      return status;
    },
    accountSettingStatus(id) { this.accountStatus(id); return settings.status(id); },
    accountToolsStatus(id) { this.accountStatus(id); return toolJobs.status(id); },
    async accountToolsAction(id, operation, fields = {}) {
      if (!this.accountStatus(id).browser_online) fail('Carrot Web is offline. Open the app; do not retry automatically.', 409);
      toolJobs.queue(id, operation, fields); wake(id);
      const deadline = Date.now() + 8000;
      while (Date.now() < deadline && toolJobs.status(id).status === 'queued') await new Promise(resolve => setTimeout(resolve, 100));
      this.accountStatus(id);
      return toolJobs.status(id);
    },
    async accountSettingAction(id, operation, fields = {}) {
      this.accountStatus(id);
      settings.queue(id, operation, fields); wake(id);
      const deadline = Date.now() + 8000;
      while (Date.now() < deadline && settings.status(id).status === 'queued') await new Promise(resolve => setTimeout(resolve, 100));
      this.accountStatus(id);
      return settings.status(id);
    },
    // Internal OAuth interface: the identity comes only from validated credentials.
    accountStatus(id) {
      sweep(); const s = sessions.get(id);
      if (!s?.controlHash) fail('Carrot Web disconnected. Authorize this device again.', 401);
      return status(s);
    },
    accountAction(id, operation, name) {
      this.accountStatus(id);
      const s = sessions.get(id);
      if (operation === 'cancel') { if (s.command?.status === 'queued') s.command.status = 'cancelled'; return status(s); }
      if (operation !== 'open') fail('Unknown account operation.');
      if (s.command?.status === 'queued' && s.command.name !== name) fail('Another screen request is pending. Cancel it first.', 409);
      if (s.command?.status !== 'queued') s.command = { id: token(), name, status: 'queued', expiresAt: now() + 90000 };
      wake(id); return status(s);
    },
    claimAccountCode(code) {
      sweep(); const hash = digest(String(code).replace(/[\s-]/g, '').toUpperCase());
      const link = accountCodes.get(hash), s = link && sessions.get(link.id);
      if (!s || link.expiresAt <= now()) fail('Invalid or expired device code. Create a new code in Carrot Web.', 401);
      // Preserve the old conversation capability if already paired.
      const paired = { ...s, controlHash: s.controlHash || digest(token()), expiresAt: null };
      persist(paired); Object.assign(s, paired); controls.set(s.controlHash, s);
      accountCodes.delete(hash); codes.delete(s.codeHash);
      return { id: s.id, label: s.label };
    },
    create(origin, label = 'Carrot Web') {
      sweep();
      if (!allowedWebOrigin(origin)) fail('Open this from your local Carrot Web.', 403);
      if (sessions.size >= maxSessions) fail('Pairing capacity reached. Try later.', 429);
      const browserToken = token(), code = randomBytes(10).toString('hex').toUpperCase();
      const s = { id: token(), origin, label: String(label).slice(0, 80), browserHash: digest(browserToken),
        codeHash: digest(code), pairExpiresAt: now() + 300000, expiresAt: now() + 300000,
        lastSeen: 0, command: null, pollAt: -Infinity };
      sessions.set(s.id, s); browsers.set(s.browserHash, s); codes.set(s.codeHash, s);
      return { browser_token: browserToken, pairing_code: code, pairing_expires_at: s.pairExpiresAt,
        expires_at: s.expiresAt };
    },
    pair(code) {
      const s = lookup(codes, String(code).replace(/[\s-]/g, '').toUpperCase());
      if (s.controlHash || s.pairExpiresAt <= now()) fail('Pairing code expired or already used.', 401);
      const connection = token(), paired = { ...s, controlHash: digest(connection), expiresAt: null };
      persist(paired);
      Object.assign(s, paired); controls.set(s.controlHash, s);
      codes.delete(s.codeHash);
      return { connection_id: connection, ...status(s), scope: 'open_setting_screen_only',
        note: 'Keep connection_id in this conversation. Never display it or put it in URLs. Reuse it until disconnected; do not request a new code unless authentication fails.' };
    },
    status(connection) { return status(lookup(controls, connection)); },
    open(connection, name) {
      const s = lookup(controls, connection);
      if (s.command?.status === 'queued') {
        if (s.command.name === name) return status(s);
        fail('Another screen request is pending. Check status or cancel it first.', 409);
      }
      s.command = { id: token(), name, status: 'queued', expiresAt: now() + 90000 };
      wake(s.id); return { ...status(s), note: 'Queued, not yet opened. Return to the paired Carrot Web tab. Only status opened confirms completion.' };
    },
    cancel(connection) {
      const s = lookup(controls, connection);
      if (s.command?.status === 'queued') s.command.status = 'cancelled';
      return status(s);
    },
    browser(key, origin, operation, body = {}) {
      const s = lookup(browsers, key);
      if (operation === 'relocate') {
        if (!s.controlHash || !allowedWebOrigin(origin) || !allowedWebOrigin(body.previous_origin)
          || ![origin, body.previous_origin].includes(s.origin)) fail('Connection move is not authorized.', 403);
        if (s.origin !== origin) {
          const next = { ...s, origin, label: new URL(origin).host };
          persist(next); Object.assign(s, next);
          // A new address is not permission to replay old commands.
          settings.remove(s.id); driving.remove(s.id); toolJobs.remove(s.id);
          s.command = null; s.lastSeen = 0; s.pollAt = -Infinity; wake(s.id);
          for (const [hash, code] of accountCodes) if (code.id === s.id) accountCodes.delete(hash);
        }
        return status(s);
      }
      if (s.origin !== origin) fail('Browser origin mismatch.', 403);
      if (operation === 'inspect') return { ...status(s), account_activity_at: accountActivity(s.id) };
      if (operation === 'disconnect') { remove(s); return { disconnected: true }; }
      if (operation === 'drive_ack') return driving.ack(s.id, body.id, body.result);
      if (operation === 'settings_ack') return settings.ack(s.id, body.id, body.result);
      if (operation === 'tools_ack') return toolJobs.ack(s.id, body.id, body.result);
      if (operation === 'account_code') {
        for (const [hash, link] of accountCodes) if (link.id === s.id) accountCodes.delete(hash);
        const code = randomBytes(10).toString('hex').toUpperCase();
        const expiresAt = now() + 300000;
        // A freshly issued code must not die with the older unpaired browser lease.
        if (!s.controlHash) s.expiresAt = expiresAt;
        accountCodes.set(digest(code), { id: s.id, expiresAt });
        return { account_code: code, expires_at: expiresAt };
      }
      if (operation === 'ack') {
        if (!s.command || body.id !== s.command.id) fail('Unknown request.', 409);
        if (s.command.status === 'queued') {
          if (!['opened', 'unavailable', 'failed'].includes(body.status)) fail('Invalid result.');
          s.command.status = body.status;
        }
        return status(s);
      }
      if (operation !== 'poll') fail('Unknown operation.');
      if (now() - s.pollAt < 900) fail('Poll too fast.', 429);
      s.pollAt = now(); s.lastSeen = now();
      return { ...status(s), settings_command: settings.poll(s.id), drive_command: driving.poll(s.id), tools_command: toolJobs.poll(s.id) };
    },
  };
}

export function installBridgeRoutes(app, bridge) {
  let minute = 0, creations = 0;
  app.use('/bridge', (req, res, next) => {
    const origin = req.headers.origin;
    if (!allowedWebOrigin(origin)) return res.status(403).end();
    res.set('Access-Control-Allow-Origin', origin).set('Vary', 'Origin')
      .set('Access-Control-Allow-Headers', 'Authorization, Content-Type').set('Access-Control-Allow-Methods', 'POST, OPTIONS');
    if (req.method === 'OPTIONS') return res.status(204).end();
    next();
  });
  app.post('/bridge/:operation', async (req, res) => {
    try {
      const { operation } = req.params;
      if (operation === 'create') {
        const current = Math.floor(Date.now() / 60000);
        if (current !== minute) { minute = current; creations = 0; }
        if (++creations > 60) return res.status(429).end();
        return res.json(bridge.create(req.headers.origin, req.body?.label));
      }
      const auth = req.headers.authorization || '';
      if (!/^Bearer [A-Za-z0-9_-]{43}$/.test(auth)) return res.status(401).end();
      res.json(operation === 'poll_wait' ? await bridge.waitForBrowser(auth.slice(7), req.headers.origin) : bridge.browser(auth.slice(7), req.headers.origin, operation, req.body));
    } catch (e) { res.status(e.status || 400).json({ error: e.message }); }
  });
}
