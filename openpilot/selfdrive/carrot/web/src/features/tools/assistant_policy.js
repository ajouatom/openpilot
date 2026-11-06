export const ASSISTANT_RELAY = 'https://carrot-mcp.shind0.synology.me';
export const ASSISTANT_SESSION_KEY = 'carrot.assistant.session.v1';
export const CHATGPT_INSTALL_URL = 'https://play.google.com/store/apps/details?id=com.openai.chatgpt';

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
