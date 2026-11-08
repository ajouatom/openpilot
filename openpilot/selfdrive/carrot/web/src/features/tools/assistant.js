import { ASSISTANT_RELAY, ASSISTANT_SESSION_KEY, CHATGPT_INSTALL_URL, screenRequest, sessionValid } from './assistant_policy.js';
import { copyPairingMessage } from './assistant_clipboard.js';
import { executeDriveCommand, mountDriveAccess, revokeDriveAccess, driveAccessEnabled } from './assistant_drive.js';
import { executeSettingCommand, mountSettingAccess, revokeSettingAccess, settingCommandBlocked } from './assistant_settings.js';
import { executeToolsCommand, mountToolsAccess, revokeToolsAccess } from './assistant_tools.js';

const text = (key, fallback) => globalThis.getUIText?.(`assistant_${key}`, fallback) || fallback;
let session = null, completed = null, timer = null, polling = false, generation = 0;
let state = '';
try { session = JSON.parse(localStorage.getItem(ASSISTANT_SESSION_KEY) || sessionStorage.getItem(ASSISTANT_SESSION_KEY)); } catch {}
if (!sessionValid(session)) session = null;

function storeSession() {
  try {
    if (session) localStorage.setItem(ASSISTANT_SESSION_KEY, JSON.stringify(session));
    else localStorage.removeItem(ASSISTANT_SESSION_KEY);
    sessionStorage.removeItem(ASSISTANT_SESSION_KEY);
  } catch { /* In-memory pairing still works until this page is closed. */ }
}
function status(message) {
  state = message;
  const node = document.getElementById('assistantStatus');
  if (node) node.textContent = message;
}
async function request(operation, body = {}, current = session) {
  const response = await fetch(`${ASSISTANT_RELAY}/bridge/${operation}`, {
    method: 'POST', credentials: 'omit', referrerPolicy: 'no-referrer',
    headers: { 'Content-Type': 'application/json', ...(current ? { Authorization: `Bearer ${current.browser_token}` } : {}) },
    body: JSON.stringify(body), signal: AbortSignal.timeout(10000),
  });
  if (!response.ok) throw Object.assign(new Error(text('error', 'Connection failed. Check internet access and try again.')), { status: response.status });
  return response.json();
}
async function revoke() {
  await revokeToolsAccess();
  await revokeDriveAccess();
  await revokeSettingAccess();
  if (!session) return;
  try { await request('disconnect'); }
  catch (error) { if (error.status !== 401) throw error; }
}
function schedule() {
  clearTimeout(timer);
  if (session) timer = setTimeout(poll, document.hidden ? 10000 : driveAccessEnabled() ? 1000 : 3000);
}
async function poll() {
  if (polling || !session) return;
  if (!sessionValid(session)) {
    session = null; storeSession(); status(text('expired', 'Connection expired. Create a new pairing code.')); return;
  }
  if (document.hidden) { schedule(); return; }
  polling = true;
  const current = session, epoch = generation;
  try {
    const result = await request(driveAccessEnabled() ? 'poll_wait' : 'poll', {}, current);
    if (epoch !== generation) return;
    const newlyLinked = result.account_linked && !session.account_linked;
    if (session.expires_at !== result.expires_at || session.connected !== result.connected || session.account_linked !== result.account_linked) {
      session.expires_at = result.expires_at; session.connected = result.connected; session.account_linked = result.account_linked; storeSession();
    }
    if (result.connected && session.pairing_code) {
      delete session.pairing_code; storeSession();
    }
    status(result.account_linked ? text('account_connected', 'Account connected · reusable in new voice chats') : result.connected ? text('connected', 'Connected · screen navigation only') : text('waiting', 'Waiting for ChatGPT to pair'));
    if (newlyLinked) {
      const field = document.getElementById('assistantAccountCode');
      if (field) field.value = '';
      const feedback = document.getElementById('assistantAccountFeedback');
      if (feedback) feedback.textContent = text('account_connected', 'Account connected · reusable in new voice chats');
    }
    if (result.drive_command) {
      const driveResult = await executeDriveCommand(result.drive_command);
      if (driveResult && epoch === generation) await request('drive_ack', driveResult, current);
    }
    if (result.settings_command && !settingCommandBlocked(result.settings_command, document.activeElement)) {
      const settingResult = await executeSettingCommand(result.settings_command);
      if (settingResult && epoch === generation) await request('settings_ack', settingResult, current);
    }
    if (result.tools_command && !/^(INPUT|TEXTAREA|SELECT)$/.test(document.activeElement?.tagName)) {
      const toolsResult = await executeToolsCommand(result.tools_command);
      if (toolsResult && epoch === generation) await request('tools_ack', toolsResult, current);
    }
    const command = result.command;
    if (!screenRequest(command)) return;
    // Do not interrupt editing or another dialog. A pending request expires on the server.
    const active = document.activeElement;
    if (/^(INPUT|TEXTAREA|SELECT)$/.test(active?.tagName) && !active.readOnly) return;
    if (document.querySelector('.app-dialog:not([hidden])')) {
      // The connection panel itself must not block the first requested screen.
      if (document.getElementById('assistantPanel')) document.getElementById('appDialogConfirm')?.click();
      return;
    }
    if (completed?.id !== command.id) {
      let outcome = 'failed';
      try { outcome = await globalThis.openAssistantSetting(command.name); } catch {}
      completed = { id: command.id, status: ['opened', 'unavailable'].includes(outcome) ? outcome : 'failed' };
    }
    if (epoch !== generation) return;
    await request('ack', completed, current);
  } catch (error) {
    if (epoch !== generation) return;
    if (error.status === 401) { session = null; storeSession(); status(text('expired', 'Connection expired. Create a new pairing code.')); }
    else status(text('retry', 'Connection interrupted · retrying'));
  } finally { polling = false; schedule(); }
}

export function openAssistantDialog() {
  // Static structure only. User content, URLs and server responses are assigned via DOM properties.
  void globalThis.appAlert('', { title: text('title', 'ChatGPT assistant'), html: true,
    messageHtml: '<div id="assistantPanel"><p id="assistantHelp"></p><p id="assistantStatus" role="status"></p><button id="assistantDisconnect" class="btn" type="button"></button></div>' });
  const el = id => document.getElementById(`assistant${id}`);
  if (!el('Panel')) return;
  const label = (id, key, fallback) => { el(id).textContent = text(key, fallback); };
  const install = document.createElement('a');
  install.className = 'btn'; install.href = CHATGPT_INSTALL_URL;
  install.target = '_blank'; install.rel = 'noopener noreferrer';
  install.textContent = text('install', 'Install ChatGPT · Google Play');
  const voiceHelp = document.createElement('p');
  voiceHelp.textContent = text('voice_help', 'Phone voice works without a ChatGPT account or pairing code. Under the device name in CarrotAssist, enable “Listen for commands on this phone first” and tap the microphone. Say “Find acceleration settings.” With setting access enabled below, say “Set acceleration setting zero to 240.”');
  el('Panel').prepend(install, voiceHelp);
  const account = document.createElement('details');
  account.open = !session?.account_linked;
  account.innerHTML = '<p id="assistantAccountHelp"></p><button id="assistantAccountPair" class="btn" type="button"></button><label id="assistantAccountCodeLabel" for="assistantAccountCode"></label><input id="assistantAccountCode" class="input-text" readonly autocomplete="off" style="width:100%;box-sizing:border-box;user-select:text;-webkit-user-select:text"><button id="assistantAccountCopy" class="btn" type="button"></button><p id="assistantAccountFeedback" role="status"></p><hr>';
  const summary = document.createElement('summary');
  summary.textContent = text('setup_steps', 'First connection / reconnect');
  const setupLink = document.createElement('a');
  setupLink.className = 'btn'; setupLink.href = 'https://chatgpt.com/plugins';
  setupLink.target = '_blank'; setupLink.rel = 'noopener noreferrer';
  setupLink.textContent = text('setup_open', '1. Open ChatGPT connection page');
  const address = document.createElement('input');
  address.className = 'input-text'; address.readOnly = true; address.value = `${ASSISTANT_RELAY}/account/mcp`;
  address.setAttribute('aria-label', text('setup_address', 'Carrot connection address'));
  address.style.cssText = 'display:block;width:100%;box-sizing:border-box;margin:8px 0';
  const copyAddress = document.createElement('button');
  copyAddress.type = 'button'; copyAddress.className = 'btn';
  copyAddress.textContent = text('setup_copy_address', '2. Copy Carrot connection address');
  copyAddress.onclick = async () => {
    const copied = await copyPairingMessage(address);
    if (el('AccountFeedback')) el('AccountFeedback').textContent = copied ? text('copied', 'Copied') : text('copy_manual', 'Select the address and copy it manually.');
  };
  const reconnect = document.createElement('p'), codeWarning = document.createElement('strong');
  reconnect.textContent = text('setup_reconnect', 'Already installed: ChatGPT > Plugins > Installed > CarrotAssist > Reconnect. Open the authorization page first, then return here for a fresh code.');
  codeWarning.textContent = text('setup_code_warning', 'Paste codes only into the device authorization page, never a voice conversation. Google Play is only for installation.');
  const firstSetup = document.createElement('details'), firstSummary = document.createElement('summary');
  firstSummary.textContent = text('setup_first', 'First registration only: show instructions');
  firstSetup.append(firstSummary, address, copyAddress, account.querySelector('#assistantAccountHelp'));
  account.prepend(summary, codeWarning, reconnect, setupLink, firstSetup);
  el('Panel').prepend(account);
  const approval = document.createElement('details');
  const approvalSummary = document.createElement('summary');
  approvalSummary.textContent = text('approval_title', 'After connecting: repeated ChatGPT approvals');
  const approvalHelp = document.createElement('p');
  approvalHelp.textContent = text('approval_help', 'Carrot device access and ChatGPT action approvals are separate. To stop repeated prompts, choose CarrotAssist > Permissions > Allow all actions in ChatGPT plugin settings. This applies to all actions allowed for this plugin, not just one setting. Each user chooses this in their own account; CarrotAssist cannot enable it automatically or check the selection. It does not grant new device permissions.');
  const approvalLink = document.createElement('a');
  approvalLink.className = 'btn'; approvalLink.href = 'https://chatgpt.com/settings/plugins-settings';
  approvalLink.target = '_blank'; approvalLink.rel = 'noopener noreferrer';
  approvalLink.textContent = text('approval_open', 'Open ChatGPT plugin permissions');
  const approvalRefresh = document.createElement('p');
  approvalRefresh.textContent = text('approval_refresh', 'If old confirmation instructions remain, refresh tools in CarrotAssist plugin settings and start a new conversation. Do not create another connection code for an action approval.');
  approval.append(approvalSummary, approvalHelp, approvalLink, approvalRefresh);
  account.after(approval);
  mountSettingAccess(el('Panel'));
  mountDriveAccess(el('Panel'));
  mountToolsAccess(el('Panel'));
  label('AccountHelp', 'account_help', 'In ChatGPT, select + > Add custom MCP server. Name it CarrotAssist, paste the copied address into Server URL and choose OAuth under Authentication. Create and connect it, then enter the code below on the Carrot approval page. The five-minute limit is only for this code; your connection stays available. Do not send the code in chat.');
  label('AccountPair', 'account_pair', 'Create mini-mode connection code');
  label('AccountCodeLabel', 'account_code', 'Device authorization code · expires in 5 minutes');
  el('AccountCodeLabel').style.cssText = 'display:block;margin-top:8px';
  label('AccountCopy', 'account_copy', 'Copy authorization code');
  el('AccountCopy').onclick = async () => {
    const field = el('AccountCode');
    if (!field.value) return;
    const copied = await copyPairingMessage(field);
    if (el('AccountCode') === field) el('AccountFeedback').textContent = copied ? text('copied', 'Copied') : text('copy_manual', 'Select the code and copy it manually.');
  };
  el('AccountPair').onclick = async () => {
    el('AccountPair').disabled = true;
    try {
      if (!session) { session = await request('create', { label: location.host }, null); generation++; storeSession(); render(); schedule(); }
      const result = await request('account_code');
      if (el('AccountCode')) { el('AccountCode').value = result.account_code; el('AccountFeedback').textContent = text('account_ready', 'Enter this code only on the Carrot authorization page. Existing connections remain active.'); }
    } catch (e) { status(e.message); }
    finally { if (el('AccountPair')) el('AccountPair').disabled = false; }
  };
  label('Help', 'help', 'Connect your account once to use new voice chats. Setting reads and changes additionally require permission here and in the ChatGPT connection.');
  label('Disconnect', 'disconnect', 'Disconnect');
  const render = () => {
    if (!el('Panel')) return;
    el('Disconnect').disabled = !session;
    el('Status').textContent = state || text('not_connected', 'Not connected');
  };
  render();
  el('Disconnect').onclick = async () => {
    try {
      await revoke();
      generation++; session = null; completed = null; clearTimeout(timer); storeSession();
      status(text('not_connected', 'Not connected')); render();
    } catch (e) { status(e.message); }
  };
}

document.addEventListener('visibilitychange', () => { if (!document.hidden) { clearTimeout(timer); void poll(); } });
schedule();
