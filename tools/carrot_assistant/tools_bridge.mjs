import { randomBytes } from 'node:crypto';
import { z } from 'zod';

export const toolActions = ['git_pull', 'git_sync', 'git_reset_hard', 'git_reset_mixed', 'git_reset_soft',
  'reboot', 'rebuild_all', 'reset_calib', 'delete_all_videos', 'delete_all_logs', 'send_tmux_log',
  'server_tmux_log', 'backup_settings', 'record_start', 'record_stop'];
export const toolPanels = ['tools', 'car_select', 'language', 'web_settings', 'device_info', 'bluetooth', 'assistant',
  'git_branch', 'git_log', 'git_reset_repo', 'restore_settings', 'qr_backup_settings', 'qr_restore_settings',
  'copy_settings', 'view_settings', 'egpu', 'naver_map'];
const id = z.string().regex(/^[A-Za-z0-9_-]{43}$/);
export const toolResult = z.object({
  status: z.enum(['prepared', 'running', 'completed', 'requested', 'opened', 'needs_input', 'cancelled', 'failed', 'unknown']),
  action: z.enum([...toolActions, ...toolPanels]).optional(), title: z.string().max(160).optional(),
  effect: z.string().max(500).optional(), proposal_id: id.optional(), confirmation_required: z.boolean().optional(),
  expires_in_seconds: z.number().int().min(0).max(60).optional(), job_id: z.string().regex(/^[a-f0-9]{12}$/).optional(),
  error: z.string().max(300).optional(),
}).strict();

export function createToolsBridge(now = Date.now) {
  const requests = new Map();
  const status = owner => {
    for (const [key, value] of requests) if (now() - value.expires_at > 120000) requests.delete(key);
    const r = requests.get(owner);
    if (!r) return { status: 'none' };
    if (r.status === 'queued' && now() >= r.expires_at) r.status = 'expired';
    return { status: r.status, request_id: r.id, ...(r.result ? { result: r.result } : {}) };
  };
  return {
    status,
    queue(owner, operation, fields) {
      const fingerprint = JSON.stringify({ operation, ...fields }), previous = status(owner);
      if (previous.status === 'queued') {
        if (requests.get(owner).fingerprint !== fingerprint) throw new Error('A Tools request is pending; check its result first');
        return previous;
      }
      if (!requests.has(owner) && requests.size >= 1000) throw new Error('Tools capacity reached');
      requests.set(owner, { ...fields, operation, id: randomBytes(32).toString('base64url'),
        expires_at: now() + 30000, status: 'queued', fingerprint });
      return status(owner);
    },
    poll(owner) {
      if (status(owner).status !== 'queued') return null;
      const { status: ignored, fingerprint, result, ...command } = requests.get(owner);
      return command;
    },
    ack(owner, identity, raw) {
      const state = status(owner), r = requests.get(owner);
      if (!r || r.id !== identity) throw new Error('Unknown Tools request');
      if (state.status !== 'queued') return state;
      const result = toolResult.parse(raw);
      const expected = { open: ['opened', 'needs_input', ...(r.action === 'naver_map' ? ['requested'] : [])], prepare: ['prepared'], commit: ['running', 'requested', 'unknown'],
        status: ['running', 'completed', 'requested', 'unknown'], cancel: ['cancelled'] }[r.operation] || [];
      if (result.status !== 'failed' && (!expected.includes(result.status) || (r.action && result.action !== r.action))) throw new Error('Unexpected Tools result');
      if (result.status === 'prepared' && (!result.proposal_id || !result.title || !result.effect || result.confirmation_required !== true)) throw new Error('Incomplete Tools proposal');
      if (result.status === 'running' && !result.job_id) throw new Error('Job identity required');
      if (r.job_id && result.status !== 'failed' && r.job_id !== result.job_id) throw new Error('Job identity mismatch');
      r.result = result; r.status = 'completed'; return status(owner);
    },
    remove(owner) { requests.delete(owner); },
  };
}
