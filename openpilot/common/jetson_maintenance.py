"""Persistent, explicitly requested first-update hold for deployed Jetsons."""
import json
from pathlib import Path
import time

from openpilot.common.jetlink_status import LINK_STATUS, _fresh, host_label

PENDING = 'JetsonLegacyUpdatePending'
ALERT = 'Offroad_JetsonLegacyUpdate'
RELEASE = Path(__file__).resolve().parents[1] / 'selfdrive/modeld/jetlink/host_release.json'


def migrated(peer):
  """Only a normal runtime's receipt confirms both release and installation."""
  try:
    return (peer.get('carrot_host') == 'jetson' and peer.get('carrot_boot_update_installed') is True and
            peer.get('carrot_boot_update_v1') is not True and
            peer.get('carrot_source_commit') == json.loads(RELEASE.read_text())['source_commit'])
  except (OSError, ValueError, KeyError, TypeError):
    return False


def status(params):
  link = _fresh(LINK_STATUS, time.monotonic())
  peer = link.get('peer') or {}
  connected = link.get('state') in ('ready', 'loading', 'maintenance', 'updating') and host_label(peer) == 'jetSON'
  return {'pending': params.get_bool(PENDING), 'connected': connected,
          'migrated': connected and migrated(peer)}


def parked(sm):
  """Require live, unfiltered standstill and inactive controls for entry."""
  services = ('carState', 'selfdriveState', 'carControl')
  try:
    if not all(sm.valid[s] and sm.alive[s] for s in services):
      return False
    cs, sd, cc = (sm[s] for s in services)
    return (str(cs.gearShifter) == 'park' and cs.standstill and cs.vEgoRaw == 0 and
            not sd.enabled and not sd.active and not cc.enabled and not cc.latActive and not cc.longActive)
  except (AttributeError, KeyError, TypeError):
    return False


def alert_text(language):
  if str(language).startswith('ko'):
    return ('Jetson 최초 업데이트 대기 — 주행 보조가 중지되었습니다.\n' +
            '정차한 상태에서 시동과 인터넷 연결을 유지하세요. 구형 Jetson은 다운로드 완료 여부를 알려주지 않습니다. ' +
            '시동을 껐다 켠 뒤 새 프로그램 적용이 확인되면 자동 해제됩니다. ' +
            '취소: Carrot Web → 도구 → Jetson 최초 업데이트 대기 취소')
  return ('Jetson first-update wait — driving assistance is stopped.\n' +
          'Remain parked with ignition and Internet on. Older Jetsons cannot report download completion. ' +
          'After an ignition power cycle, this hold clears when the new runtime is confirmed. ' +
          'Cancel in Carrot Web → Tools → Cancel Jetson first-update wait.')
