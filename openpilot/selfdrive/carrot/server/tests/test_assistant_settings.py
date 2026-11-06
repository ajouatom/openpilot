import math
import pytest

from openpilot.selfdrive.carrot.server.services.assistant_settings import AssistantSettings


class TestAssistantSettings:
  @pytest.fixture(autouse=True)
  def setup(self, tmp_path):
    self.now, self.parked = 100.0, True
    self.values = {'Gap': 100, 'ShowCustomBrightness': 60, 'Negative': -50}
    self.catalog = {name: {'title': name, 'min': (-100 if name == 'Negative' else 0), 'max': 200, 'unit': 5} for name in self.values}
    self.writes, self.history = [], []
    self.path = tmp_path / 'grants.json'
    self.service = self.make_service()
    self.token = self.service.enable('http://carrot:7000')
    self.counter = 0

  def make_service(self):
    def write(name, value):
      self.writes.append((name, value))
      self.values[name] = value

    return AssistantSettings(
      self.path, lambda: self.catalog, self.values.get, write, lambda: self.parked, lambda *args: self.history.append(args), lambda: self.now
    )

  def command(self, operation, **fields):
    self.counter += 1
    return {'id': str(self.counter).zfill(43), 'operation': operation, **fields}

  def execute(self, command, token=None):
    return self.service.execute(token or self.token, 'http://carrot:7000', command)

  def prepare(self, **fields):
    return self.execute(self.command('prepare', **{'name': 'Gap', 'mode': 'percent', 'value': 10, **fields}))

  def test_prepare_confirm_readback_and_duplicate_delivery(self):
    proposed = self.prepare()
    assert proposed['proposed'] == 110
    assert self.writes == []
    commit = self.command('commit', proposal_id=proposed['proposal_id'])
    result = self.execute(commit)
    assert result['current'] == 110
    assert self.execute(commit) == result
    assert self.writes == [('Gap', 110)]
    with pytest.raises(ValueError):
      self.execute(self.command('commit', proposal_id=proposed['proposal_id']))

  def test_scope_origin_and_cross_connection_proposal(self):
    proposed = self.prepare()
    with pytest.raises(PermissionError):
      self.execute(self.command('read', name='Gap'), 'wrong')
    with pytest.raises(PermissionError):
      self.service.execute(self.token, 'http://other:7000', self.command('read', name='Gap'))
    another = self.service.enable('http://carrot:7000')
    with pytest.raises(ValueError):
      self.execute(self.command('commit', proposal_id=proposed['proposal_id']), another)
    assert self.writes == []

  def test_expiry_changed_current_and_definition(self):
    for change in ('time', 'value', 'definition'):
      proposal = self.prepare()
      if change == 'time':
        self.now += 60
      if change == 'value':
        self.values['Gap'] += 5
      if change == 'definition':
        self.catalog['Gap']['unit'] = 10
      with pytest.raises(ValueError):
        self.execute(self.command('commit', proposal_id=proposal['proposal_id']))
    assert self.writes == []

  def test_stationary_rechecked_but_display_allowed_onroad(self):
    proposal = self.prepare()
    self.parked = False
    with pytest.raises(ValueError):
      self.execute(self.command('commit', proposal_id=proposal['proposal_id']))
    with pytest.raises(ValueError):
      self.prepare()
    with pytest.raises(ValueError):
      self.service.enable('http://carrot:7000')
    proposal = self.prepare(name='ShowCustomBrightness', mode='absolute', value=70)
    assert self.execute(self.command('commit', proposal_id=proposal['proposal_id']))['status'] == 'applied'

  def test_no_defaults_unknown_internal_or_invalid_values(self):
    for name in ('Missing', 'DriverMonitoringEnabled'):
      with pytest.raises(ValueError):
        self.execute(self.command('read', name=name))
    self.values['Gap'] = None
    with pytest.raises(ValueError):
      self.execute(self.command('read', name='Gap'))
    self.values['Gap'] = 100
    for value in (True, math.nan, math.inf, '110', 201, -1):
      with pytest.raises(ValueError):
        self.prepare(mode='absolute', value=value)
    assert self.writes == []

  def test_step_rounding_negative_percent_and_noop(self):
    assert self.prepare(mode='absolute', value=112.5)['proposed'] == 115
    assert self.prepare(name='Negative')['proposed'] == -45
    with pytest.raises(ValueError):
      self.prepare(value=1)

  def test_correction_and_cancel_invalidate_previous_proposals(self):
    old = self.prepare()
    self.prepare(value=20)
    with pytest.raises(ValueError):
      self.execute(self.command('commit', proposal_id=old['proposal_id']))
    current = self.prepare()
    self.execute(self.command('cancel'))
    with pytest.raises(ValueError):
      self.execute(self.command('commit', proposal_id=current['proposal_id']))

  def test_persistent_permission_but_no_persistent_commands(self):
    proposed = self.prepare()
    self.service = self.make_service()
    assert self.execute(self.command('read', name='Gap'))['current'] == 100
    with pytest.raises(ValueError):
      self.execute(self.command('commit', proposal_id=proposed['proposal_id']))
    self.service.revoke(self.token, 'http://carrot:7000')
    self.service = self.make_service()
    with pytest.raises(PermissionError):
      self.execute(self.command('read', name='Gap'))

  def test_bad_readback_is_never_success_and_not_replayed(self):
    proposed = self.prepare()
    self.service.write = lambda name, value: self.writes.append((name, value))
    command = self.command('commit', proposal_id=proposed['proposal_id'])
    with pytest.raises(ValueError):
      self.execute(command)
    assert self.execute(command)['status'] == 'failed'
    assert len(self.writes) == 1
