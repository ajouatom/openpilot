export const ASSISTANT_RELAY = 'https://carrot-mcp.shind0.synology.me';
export const ASSISTANT_SESSION_KEY = 'carrot.assistant.session.v1';
export const ASSISTANT_CHAT_KEY = 'carrot.assistant.chat.v1';

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

export function sessionValid(session, now = Date.now()) {
  return /^[A-Za-z0-9_-]{43}$/.test(session?.browser_token || '')
    && Number.isFinite(session?.expires_at) && session.expires_at > now;
}
