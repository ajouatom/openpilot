import { ASSISTANT_RELAY, ASSISTANT_SESSION_KEY, ASSISTANT_CHAT_KEY, CHATGPT_INSTALL_URL, chatGptLaunchUrl, chatGptUrl, screenRequest, sessionValid } from './assistant_policy.js';
import { copyPairingMessage } from './assistant_clipboard.js';

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
  updateLauncher();
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
  if (!session) return;
  try { await request('disconnect'); }
  catch (error) { if (error.status !== 401) throw error; }
}
function schedule() {
  clearTimeout(timer);
  if (session) timer = setTimeout(poll, document.hidden ? 10000 : 3000);
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
    const result = await request('poll', {}, current);
    if (epoch !== generation) return;
    if (session.expires_at !== result.expires_at || session.connected !== result.connected) {
      session.expires_at = result.expires_at; session.connected = result.connected; storeSession();
    }
    if (result.connected && session.pairing_code) {
      delete session.pairing_code; storeSession();
      const prompt = document.getElementById('assistantPrompt');
      const copy = document.getElementById('assistantCopy');
      if (prompt) prompt.value = '';
      if (copy) copy.disabled = true;
      const message = document.getElementById('assistantPairMessage');
      if (message) message.hidden = true;
    }
    status(result.connected ? text('connected', 'Connected · screen navigation only') : text('waiting', 'Waiting for ChatGPT to pair'));
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

function savedChat() {
  try { return chatGptUrl(localStorage.getItem(ASSISTANT_CHAT_KEY)) || 'https://chatgpt.com/'; }
  catch { return 'https://chatgpt.com/'; }
}

const isAndroid = () => /Android/i.test(navigator.userAgent);
const launchUrl = () => chatGptLaunchUrl(savedChat(), isAndroid());
function updateLauncher() {
  const launcher = document.getElementById('assistantLauncher');
  if (!launcher) return;
  launcher.href = launchUrl();
  launcher.target = isAndroid() ? '_self' : '_blank';
  launcher.title = launcher.ariaLabel = text('icon_help', 'ChatGPT · hold for connection settings');
  launcher.classList.toggle('is-connected', Boolean(session?.connected));
}
function installLauncher() {
  const launcher = document.getElementById('assistantLauncher');
  if (!launcher) return;
  updateLauncher();
  let hold, held = false;
  launcher.addEventListener('pointerdown', () => {
    held = false;
    hold = setTimeout(() => { held = true; openAssistantDialog(); }, 650);
  });
  for (const event of ['pointerup', 'pointercancel', 'pointerleave']) launcher.addEventListener(event, () => clearTimeout(hold));
  launcher.addEventListener('contextmenu', event => { event.preventDefault(); clearTimeout(hold); held = true; openAssistantDialog(); });
  launcher.addEventListener('click', event => {
    if (held) { event.preventDefault(); held = false; return; }
    if (!session?.connected || event.shiftKey) {
      event.preventDefault(); openAssistantDialog();
    }
  });
}

export function openAssistantDialog() {
  // Static structure only. User content, URLs and server responses are assigned via DOM properties.
  void globalThis.appAlert('', { title: text('title', 'ChatGPT assistant'), html: true,
    messageHtml: '<div id="assistantPanel"><p id="assistantHelp"></p><p id="assistantStatus" role="status"></p><div class="ui-action-grid"><button id="assistantPair" class="btn" type="button"></button><button id="assistantDisconnect" class="btn" type="button"></button></div><div id="assistantPairMessage"><label id="assistantPromptLabel" for="assistantPrompt"></label><textarea id="assistantPrompt" class="input-text" rows="3" readonly style="width:100%;box-sizing:border-box;resize:vertical;user-select:text;-webkit-user-select:text"></textarea><button id="assistantCopy" class="btn" type="button"></button><p id="assistantCopyStatus" role="status"></p></div><p id="assistantLaunchHelp"></p><label id="assistantChatLabel" for="assistantChat"></label><input id="assistantChat" class="input-text" type="url" style="width:100%;box-sizing:border-box" autocomplete="off"><button id="assistantSave" class="btn" type="button"></button><p><a id="assistantLaunch" class="btn" target="_blank" rel="noopener noreferrer"></a></p><p id="assistantSetup"></p></div>' });
  const el = id => document.getElementById(`assistant${id}`);
  if (!el('Panel')) return;
  const label = (id, key, fallback) => { el(id).textContent = text(key, fallback); };
  const install = document.createElement('a');
  install.className = 'btn'; install.href = CHATGPT_INSTALL_URL;
  install.target = '_blank'; install.rel = 'noopener noreferrer';
  install.textContent = text('install', 'Install ChatGPT · Google Play');
  const voiceHelp = document.createElement('p');
  voiceHelp.textContent = text('voice_help', 'Install ChatGPT and sign in. If available, enable Background conversations in its voice settings. Start voice in the paired conversation, then return to Carrot Web. Split screen or pop-up view may also be available on your phone. Start voice and choose window mode in the app/phone.');
  el('Panel').prepend(install, voiceHelp);
  label('Help', 'help', 'Pair once to open setting screens. Codes expire in 5 minutes; paired connections are saved until disconnected. Use the same browser, Carrot Web address and ChatGPT conversation. Clearing browser data or losing the conversation connection may require pairing again. Setting values cannot be changed.');
  label('Pair', 'pair', 'Create pairing code');
  label('Disconnect', 'disconnect', 'Disconnect');
  label('PromptLabel', 'prompt', 'Copy this message to your ChatGPT conversation once');
  label('Copy', 'copy', 'Copy pairing message');
  label('LaunchHelp', 'launch_help', 'After pairing, the corner icon opens ChatGPT without setup. Save the paired conversation URL to reopen that conversation, then tap its voice button. Voice does not start automatically. Keep Carrot Web visible to receive screen requests. Hold the icon for connection settings.');
  label('ChatLabel', 'chat_url', 'Your ChatGPT conversation URL (optional, saved in this browser)');
  label('Save', 'save', 'Save conversation URL');
  label('Launch', 'launch', 'Open ChatGPT');
  el('Setup').textContent = `${text('setup', 'First connect the Carrot MCP in your own ChatGPT account. Server URL:')} ${ASSISTANT_RELAY}/mcp`;
  el('Chat').value = savedChat(); el('Launch').href = launchUrl();
  el('Launch').target = isAndroid() ? '_self' : '_blank';
  const render = () => {
    if (!el('Panel')) return;
    el('Disconnect').disabled = !session;
    el('Copy').disabled = !session?.pairing_code;
    el('CopyStatus').textContent = '';
    el('PairMessage').hidden = !session?.pairing_code;
    el('Prompt').value = session?.pairing_code
      ? `${text('pair_message', 'Use the Carrot tool connect_carrot_web to connect this pairing code:')} ${session.pairing_code}` : '';
    el('Status').textContent = state || text('not_connected', 'Not connected');
  };
  render();
  el('Pair').onclick = async () => {
    el('Pair').disabled = true;
    try {
      // Revoke the previous pairing before replacing it, including a code not yet claimed.
      await revoke();
      generation++; session = null; completed = null; storeSession();
      session = await request('create', { label: location.host }, null);
      storeSession(); status(text('waiting', 'Waiting for ChatGPT to pair')); render(); schedule();
    } catch (e) { status(e.message); }
    finally { if (el('Pair')) el('Pair').disabled = false; }
  };
  el('Disconnect').onclick = async () => {
    try {
      await revoke();
      generation++; session = null; completed = null; clearTimeout(timer); storeSession();
      status(text('not_connected', 'Not connected')); render();
    } catch (e) { status(e.message); }
  };
  el('Copy').onclick = async () => {
    const field = el('Prompt');
    const copied = await copyPairingMessage(field);
    if (el('Prompt') !== field || !session?.pairing_code) return;
    // Connection polls update the main status; keep copy feedback next to its button.
    el('CopyStatus').textContent = copied ? text('copied', 'Copied')
      : text('copy_manual', 'Automatic copy was blocked. Long-press the message, select all, then Copy.');
  };
  el('Save').onclick = () => {
    const url = chatGptUrl(el('Chat').value);
    if (!url) { status(text('invalid_url', 'Use an https://chatgpt.com/c/... conversation URL without query parameters.')); return; }
    try { localStorage.setItem(ASSISTANT_CHAT_KEY, url); el('Launch').href = chatGptLaunchUrl(url, isAndroid()); updateLauncher(); status(text('saved', 'Saved')); }
    catch { status(text('error', 'Could not save in this browser.')); }
  };
}

document.addEventListener('visibilitychange', () => { if (!document.hidden) { clearTimeout(timer); void poll(); } });
schedule();
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', installLauncher, { once: true });
else installLauncher();
