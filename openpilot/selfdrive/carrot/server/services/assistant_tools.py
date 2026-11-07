"""Explicitly granted Tools actions; fixed payloads, no remote shell or file paths."""
import re
import secrets
import time

from .assistant_settings import AssistantSettings

# These are the existing Tools dispatcher actions, with no model-supplied payload.
JOBS = {
  'git_pull': ('GitPull', '현재 브랜치를 업데이트합니다. 재시작은 별도 요청입니다.', {'action': 'git_pull'}),
  'git_sync': ('Git Sync', '현재 브랜치를 동기화합니다. 로컬 변경을 잃을 수 있습니다.', {'action': 'git_sync'}),
  'git_reset_hard': ('Git Reset Hard', 'HEAD 기준으로 로컬 파일 변경을 버립니다.', {'action': 'git_reset', 'mode': 'hard', 'target': 'HEAD'}),
  'git_reset_mixed': ('Git Reset Mixed', 'HEAD 기준으로 스테이징을 해제하고 파일 변경은 남깁니다.', {'action': 'git_reset', 'mode': 'mixed', 'target': 'HEAD'}),
  'git_reset_soft': ('Git Reset Soft', 'HEAD 기준 soft reset을 실행합니다.', {'action': 'git_reset', 'mode': 'soft', 'target': 'HEAD'}),
  'reboot': ('기기 재시작', '기기를 재시작하며 연결이 잠시 끊깁니다.', {'action': 'reboot'}),
  'rebuild_all': ('전체 재빌드', '재시작 후 전체 빌드를 진행합니다. 완료까지 기기를 사용할 수 없습니다.', {'action': 'rebuild_all'}),
  'reset_calib': ('캘리브레이션 초기화', '보정값을 지우고 자동 재시작합니다. 다시 보정해야 합니다.', {'action': 'reset_calib'}),
  'delete_all_videos': ('모든 화면 녹화 삭제', '기기의 화면 녹화 파일 전체를 삭제합니다. 되돌릴 수 없습니다.', {'action': 'delete_all_videos'}),
  'delete_all_logs': ('모든 주행 로그 삭제', '기기의 주행 로그 전체를 삭제합니다. 되돌릴 수 없습니다.', {'action': 'delete_all_logs'}),
  'send_tmux_log': ('진단 로그 저장', 'tmux 진단 로그를 기기에 저장합니다. 서버 업로드와는 다릅니다.', {'action': 'send_tmux_log'}),
  'server_tmux_log': ('진단 로그 서버 전송', 'tmux 진단 로그를 기존 당근 업로드 서버로 전송 요청합니다. 차량 식별정보와 진단 내용이 포함될 수 있습니다.',
                      {'action': 'server_tmux_log'}),
  'backup_settings': ('설정 백업', '기기에 설정 백업 파일을 만듭니다. 폰 다운로드는 도구 화면에서 진행합니다.', {'action': 'backup_settings'}),
}
PANELS = {
  'tools', 'car_select', 'language', 'web_settings', 'device_info', 'bluetooth', 'assistant',
  'git_branch', 'git_log', 'git_reset_repo', 'restore_settings', 'qr_backup_settings',
  'qr_restore_settings', 'copy_settings', 'view_settings', 'egpu', 'naver_map',
}
RECORD = {'record_start': True, 'record_stop': False}
REQUESTED = {'reboot', 'rebuild_all', 'reset_calib', 'server_tmux_log'}


class AssistantTools:
  def __init__(self, path, stationary, start, snapshot, record, clock=time.monotonic):
    self.permissions = AssistantSettings(path, dict, lambda _: None, lambda *_: None, stationary, lambda *_: None, clock)
    self.stationary, self.start, self.snapshot, self.record, self.clock = stationary, start, snapshot, record, clock
    self.proposals, self.results, self.owned_jobs = {}, {}, {}

  def enable(self, origin):
    return self.permissions.enable(origin)

  def revoke(self, token, origin):
    owner = self.permissions.authorize(token, origin)
    self.permissions.revoke(token, origin)
    self.proposals = {k: v for k, v in self.proposals.items() if v['owner'] != owner}

  def guard(self, token, origin, action):
    self.permissions.authorize(token, origin)
    if action in JOBS and not self.stationary():
      raise ValueError('정차하고 주행 보조를 해제한 뒤 도구 작업을 요청하세요.')

  def execute(self, token, origin, command):
    owner = self.permissions.authorize(token, origin)
    now = self.clock()
    self.proposals = {k: v for k, v in self.proposals.items() if v['until'] > now}
    self.results = {k: v for k, v in self.results.items() if v[0] > now}
    identity = command.get('id')
    if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9_-]{43}', identity):
      raise ValueError('Invalid request identity')
    key = (owner, identity)
    if key in self.results:
      return self.results[key][1]
    if len(self.results) >= 1000:
      raise ValueError('Too many requests')
    operation, action = command.get('operation'), command.get('action')
    if operation == 'open':
      if action not in PANELS:
        raise ValueError('Unknown Tools panel')
      result = {'status': 'open_requested', 'action': action}
    elif operation == 'prepare':
      if action not in JOBS and action not in RECORD:
        raise ValueError('Unsupported Tools action; choose a listed action')
      self.guard(token, origin, action)
      # One outstanding proposal per local permission. Corrections replace it.
      self.proposals = {k: v for k, v in self.proposals.items() if v['owner'] != owner}
      proposal_id = secrets.token_urlsafe(32)
      self.proposals[proposal_id] = {'owner': owner, 'action': action, 'until': now + 60}
      title, effect = JOBS[action][:2] if action in JOBS else (
        '화면 녹화 켜기' if RECORD[action] else '화면 녹화 끄기', '화면 녹화 설정을 변경합니다. 주행 로그 기록과는 별개입니다.')
      result = {'status': 'prepared', 'action': action, 'title': title, 'effect': effect,
                'proposal_id': proposal_id, 'confirmation_required': True, 'expires_in_seconds': 60}
    elif operation == 'commit':
      proposal = self.proposals.get(command.get('proposal_id'))
      if not proposal or proposal['owner'] != owner:
        raise ValueError('제안이 만료됐습니다. 다시 내용을 확인한 뒤 요청하세요.')
      action = proposal['action']
      self.guard(token, origin, action)
      del self.proposals[command['proposal_id']]
      # Consume before side effects; an uncertain delivery is never replayed.
      self.results[key] = (now + 120, {'status': 'unknown', 'action': action})
      if action in RECORD:
        self.record(RECORD[action])
        result = {'status': 'requested', 'action': action}
      else:
        self.owned_jobs = {k: v for k, v in self.owned_jobs.items() if self.snapshot(k) is not None}
        if len(self.owned_jobs) >= 1000:
          raise ValueError('Too many jobs')
        job_id = self.start(dict(JOBS[action][2]), lambda: self.guard(token, origin, action))
        self.owned_jobs[job_id] = (owner, action)
        result = {'status': 'running', 'action': action, 'job_id': job_id}
    elif operation == 'status':
      job_id = command.get('job_id')
      owned = self.owned_jobs.get(job_id)
      if not owned or owned[0] != owner:
        raise ValueError('이 연결에서 시작한 작업을 확인할 수 없습니다. 도구 작업 이력을 확인하고 자동 재실행하지 마세요.')
      action = owned[1]
      job = self.snapshot(job_id)
      state = 'unknown'
      if job:
        if job.get('status') == 'running':
          state = 'running'
        elif job.get('status') == 'failed' or (job.get('result') or {}).get('ok') is not True:
          state = 'failed'
        elif job.get('status') == 'done':
          state = 'requested' if action in REQUESTED else 'completed'
      # Never forward shell output, file contents, device IDs or diagnostic logs.
      result = {'status': state, 'action': action, 'job_id': job_id}
    elif operation == 'cancel':
      self.proposals = {k: v for k, v in self.proposals.items() if v['owner'] != owner}
      result = {'status': 'cancelled'}
    else:
      raise ValueError('Unknown Tools operation')
    self.results[key] = (now + 120, result)
    return result
