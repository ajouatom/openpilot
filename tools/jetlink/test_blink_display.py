"""Display-only telemetry, actual lamp distinction and expiry across host clocks."""
import json
from types import SimpleNamespace as N

import pytest

from blink_display import Publisher, Remote
from openpilot.common import indicator_display as d


def inputs():
  return dict(mode=2, ok=True, cs=N(leftBlinker=False, rightBlinker=False, rightBlindspot=False),
              meta=N(indicatorPrepareDirection='none', indicatorPrepareSource='', laneChangeState='off'),
              hold=None, activated=False, peer='ready', automatic=N(reason='approach'),
              tbt=N(reason='route', phase='idle'), tbt_enabled=True, front_clear=True,
              observed_source='', auto_valid=True, atc_type='')


def record(stamp=10):
  return dict(version=1, stamp=stamp, state='confirmed', origin='turn', direction='left', reason='approach', lamp='left')


def test_prepare_request_confirmed_blink_dark_phase_and_end(tmp_path):
  path = tmp_path/'display'; p = Publisher(path); args = inputs()
  args['meta'].indicatorPrepareDirection='left'; args['meta'].indicatorPrepareSource='model_path'
  p.update(10, **args)
  assert json.loads(path.read_text())['state']=='prepare'
  args['hold']={'source':'observation', 'direction':'left'}; args['observed_source']='model_path'
  p.update(10.3, **args)
  value=json.loads(path.read_text()); assert (value['state'],value['lamp'])==('request','off')
  args['activated']=True; args['cs'].leftBlinker=True
  p.update(10.6, **args); assert json.loads(path.read_text())['state']=='confirmed'
  args['cs'].leftBlinker=False
  p.update(10.9, **args)
  value=json.loads(path.read_text()); assert value['state']=='confirmed' and value['lamp']=='off'
  assert 'OFF' in d.lines(value)[1]  # current CAN phase is distinct from historical acknowledgement
  args['hold']=None; args['meta'].indicatorPrepareDirection='none'; args['observed_source']='returned'
  p.update(11.2, **args)
  value=json.loads(path.read_text()); assert (value['state'],value['reason'])==('ended','returned')
  p.update(13.3, **args); assert json.loads(path.read_text()) is None


@pytest.mark.parametrize('reason,expected', [('confirming','confirming'),('waiting_conditions','front')])
def test_250_preparation_and_block_reason(tmp_path, reason, expected):
  path=tmp_path/'display';p=Publisher(path);args=inputs()
  args['tbt'].reason=reason; args['front_clear']=False
  p.update(10,**args);value=json.loads(path.read_text())
  assert value['origin']=='tbt' and value['reason']==expected
  args['cs'].rightBlindspot=True; args['tbt'].reason='waiting_conditions'
  p.update(10.3,**args); assert json.loads(path.read_text())['reason']=='bsd'
  args['tbt'].phase='cancelled';args['tbt'].reason='driver'
  p.update(10.6,**args);assert json.loads(path.read_text())['state']=='ended'


def test_idle_hidden_disabled_or_invalid_never_claim_lamp(tmp_path):
  path=tmp_path/'display';p=Publisher(path);args=inputs()
  p.update(10,**args);assert json.loads(path.read_text()) is None
  args['hold']={'source':'turn','direction':'left'}
  p.update(10.3,**args)
  args['hold']=None;args['ok']=False
  p.update(10.6,**args);value=json.loads(path.read_text())
  assert value['reason']=='inputs' and value['lamp']=='unknown'


@pytest.mark.parametrize('fault',['nan','future','old','state','direction','reason','origin','lamp','oversize','missing'])
def test_reader_rejects_malformed_and_expired(tmp_path,fault):
  path=tmp_path/'display';value=record()
  if fault in ('nan','future','old'):value['stamp']={'nan':float('nan'),'future':11,'old':9}[fault]
  elif fault not in ('oversize','missing'):value[fault]='unrecognised'
  if fault!='missing':path.write_text('x'*1025 if fault=='oversize' else json.dumps(value))
  assert d.Reader(path).read(10.1) is None


def test_remote_does_not_renew_old_source_or_old_snapshot():
  clock=[100.1];packet={'sent':10.1,'indicator_display':record()};snap=[(100.1,packet)]
  remote=Remote(lambda:snap[0],lambda:clock[0])
  assert remote.read()['stamp']==100
  clock[0]=100.4; snap[0]=(100.4,packet)
  assert remote.read()['stamp']==100  # same source repeated with new receive time
  clock[0]=100.7;snap[0]=(100.7,packet)
  assert remote.read() is None
  snap[0]=None;assert remote.read() is None


def test_packet_extra_is_allowlisted_and_bounded():
  value=record();value['token']='never forward'
  assert 'token' not in d.clean(value,10)
  assert len(json.dumps(value))<d.MAX_BYTES


def test_manual_turn_reason_and_existing_hud_compatibility(tmp_path):
  from hud_protocol import encode_snapshot
  path = tmp_path / 'display'
  args = inputs()
  args.update(hold={'source': 'observation', 'direction': 'left'}, observed_source='manual_turn')
  Publisher(path).update(10., **args)
  value = json.loads(path.read_text())
  assert value['origin'] == 'manual_turn' and value['reason'] == 'turning'
  assert '수동 턴' in d.lines(value)[0]
  packet = json.loads(encode_snapshot({'indicator_display': value}))
  assert packet['indicator_display']['origin'] == 'manual'
  assert packet['indicator_display']['reason'] == 'turning'
  assert value['origin'] == 'manual_turn'


def test_prepare_schema_round_trip():
  from openpilot.cereal import log
  md=log.ModelDataV2.new_message()
  md.meta.indicatorPrepareDirection='right';md.meta.indicatorPrepareSource='manual'
  with log.ModelDataV2.from_bytes(md.to_bytes()) as decoded:
    assert str(decoded.meta.indicatorPrepareDirection)=='right'
    assert decoded.meta.indicatorPrepareSource=='manual'


def test_source_display_failure_is_nonfatal(tmp_path):
  p=Publisher(tmp_path/'absent'/'display'); args=inputs()
  args['hold']={'source':'turn','direction':'left'}
  p.update(10,**args)
  assert args['hold']=={'source':'turn','direction':'left'}


def test_snapshot_pressure_drops_optional_indicator_before_existing_media():
  from hud_protocol import encode_snapshot
  value={'indicator_display':record(), 'events':{}, 'cameras':{'image':'x'*(96*1024-100)}}
  result=json.loads(encode_snapshot(value))
  assert 'indicator_display' not in result and result['cameras']['image']
