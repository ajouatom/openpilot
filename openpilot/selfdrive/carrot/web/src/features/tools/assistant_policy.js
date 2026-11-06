export const ASSISTANT_RELAY = 'https://carrot-mcp.shind0.synology.me';
export const ASSISTANT_SESSION_KEY = 'carrot.assistant.session.v1';
export const ASSISTANT_CHAT_KEY = 'carrot.assistant.chat.v1';
export const CHATGPT_INSTALL_URL = 'https://play.google.com/store/apps/details?id=com.openai.chatgpt';

export function chatGptLaunchUrl(value, android = false) {
  const url = chatGptUrl(value);
  if (!url) return null;
  if (!android) return url;
  return `intent://${url.slice(8)}#Intent;scheme=https;package=com.openai.chatgpt;S.browser_fallback_url=${encodeURIComponent(CHATGPT_INSTALL_URL)};end`;
}

export function chatGptUrl(value) {
  if (!String(value || '').trim()) return 'https://chatgpt.com/';
  try {
    const url = new URL(String(value).trim());
    if (url.origin !== 'https://chatgpt.com' || url.username || url.password
      || url.search || url.hash || !/^\/(?:c\/[a-zA-Z0-9-]+|plugins(?:\/[^/]+)?)?\/?$/.test(url.pathname)) return null;
    return url.href;
  } catch { return null; }
}

export function screenRequest(command, now = Date.now()) {
  return command?.status === 'queued' && /^[A-Za-z0-9_-]{43}$/.test(command.id || '')
    && /^[A-Za-z][A-Za-z0-9_]{0,100}$/.test(command.name || '')
    && Number.isFinite(command.expires_at) && command.expires_at > now;
}

// The server is authoritative: pairing may finish while this tab is backgrounded.
// Do not discard its browser capability using the stale pre-pairing deadline.
export function sessionValid(session) {
  return /^[A-Za-z0-9_-]{43}$/.test(session?.browser_token || '')
    && (session?.expires_at === null || Number.isFinite(session?.expires_at));
}
