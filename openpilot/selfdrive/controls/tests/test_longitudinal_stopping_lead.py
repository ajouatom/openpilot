import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_stopping_lead import StoppingLeadFilter


def radar_state(*, distance=5.5, v_lead=0.05, v_rel=0.0, a_lead=0.0, j_lead=0.0, track_id=42, role='leadOne'):
  state = log.RadarState.new_message()
  lead = getattr(state, role)
  lead.status = lead.radar = True
  lead.radarTrackId = track_id
  lead.dRel = distance
  lead.vRel = v_rel
  lead.vLead = lead.vLeadK = v_lead
  lead.aLead = lead.aLeadK = a_lead
  lead.jLead = j_lead
  lead.aLeadTau = 1.5
  return state


def update(guard, state=None, *, t=1.0, stopping=True, v_ego=0.05, valid=True):
  return guard.update(
    radar_state() if state is None else state,
    stopping=stopping, v_ego=v_ego, mono_time_ns=round(t * 1e9), valid=valid,
  )


def establish_stop(guard, **kwargs):
  for index in range(5):
    output = update(guard, radar_state(**kwargs), t=1.0 + index * 0.05)
  return output


def test_stopped_lead_requires_a_history_of_distinct_stable_observations():
  guard = StoppingLeadFilter()
  for index in range(4):
    source = radar_state()
    assert update(guard, source, t=1.0 + index * 0.05) is source
  assert update(guard, t=1.2).leadOne.vLead == 0.0


def test_small_range_step_and_rising_speed_do_not_predict_departure():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  # Measured speed/range pairs from a stop where ego stayed around 0.04 m/s.
  samples = ((5.5, 0.05, 0.093946), (5.5, 0.08, 0.12146), (5.5, 0.09, 0.12999),
             (5.55, 0.11, 0.149906), (5.55, 0.14, 0.18026), (5.55, 0.16, 0.202281), (5.55, 0.17, 0.217369))
  for index, (distance, v_rel, v_lead) in enumerate(samples):
    source = radar_state(distance=distance, v_rel=v_rel, v_lead=v_lead, a_lead=0.25, j_lead=0.4)
    output = update(guard, source.as_reader(), t=1.25 + index * 0.05, v_ego=0.043)
    lead = output.leadOne
    assert (lead.vLead, lead.vLeadK, lead.aLead, lead.aLeadK, lead.jLead) == (0.0,) * 5
    assert lead.dRel == pytest.approx(distance)
    assert lead.vRel == pytest.approx(v_rel)
    assert source.leadOne.vLead == pytest.approx(v_lead)
    assert source.leadOne.aLeadK == pytest.approx(0.25)
    assert guard.held_mask == 1


def test_fast_real_departure_restores_measured_kinematics_after_range_confirmation():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t, distance in ((1.25, 5.6), (1.3, 5.7), (1.35, 5.75)):
    assert update(guard, radar_state(distance=distance, v_lead=0.8, v_rel=0.75), t=t).leadOne.vLead == 0.0
  source = radar_state(distance=5.8, v_lead=1.0, v_rel=0.95, a_lead=1.0, j_lead=0.5)
  assert update(guard, source, t=1.4) is source
  assert guard.held_mask == 0


def test_slow_departure_accumulates_range_from_fixed_stop_position():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  first_release = None
  for index in range(1, 41):
    elapsed = index * 0.05
    # 0.10 m/s crawl, quantized at 5 cm. No individual frame grows by 15 cm.
    distance = 5.5 + int((elapsed * 0.1 + 1e-9) / 0.05) * 0.05
    output = update(guard, radar_state(distance=distance, v_lead=0.1, v_rel=0.1), t=1.2 + elapsed, v_ego=0.0)
    if output.leadOne.vLead > 0.0 and first_release is None:
      first_release = elapsed
  assert first_release == pytest.approx(1.6)


def test_one_large_range_spike_does_not_confirm_departure():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t, distance in ((1.25, 5.75), (1.3, 5.5), (1.35, 5.55)):
    assert update(guard, radar_state(distance=distance, v_lead=0.3, v_rel=0.25), t=t).leadOne.vLead == 0.0


def test_repeated_or_older_fallback_measurement_does_not_accumulate_departure_time():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t in (1.25, 1.25, 1.20, 1.25, 1.30):
    assert update(guard, radar_state(distance=5.75, v_lead=0.5, v_rel=0.45), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.8, v_lead=0.5, v_rel=0.45), t=1.35).leadOne.vLead > 0.0


@pytest.mark.parametrize('kwargs', ({'stopping': False}, {'valid': False}))
def test_disabled_or_invalid_input_clears_stop_history(kwargs):
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state(v_lead=0.2)
  assert update(guard, source, t=1.25, **kwargs) is source
  assert update(guard, source, t=1.3) is source


@pytest.mark.parametrize('time', (0.5, 2.0))
def test_timestamp_discontinuity_requires_new_stop_evidence(time):
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state(v_lead=0.2)
  assert update(guard, source, t=time) is source


@pytest.mark.parametrize('role', ('leadOne', 'leadTwo'))
def test_stationary_classification_belongs_to_the_same_track_and_role(role):
  guard = StoppingLeadFilter()
  output = establish_stop(guard, role=role)
  assert getattr(output, role).vLead == 0.0
  assert guard.held_mask == (1 if role == 'leadOne' else 2)
  replacement = radar_state(track_id=43, role=role, v_lead=0.2)
  assert update(guard, replacement, t=1.25) is replacement


@pytest.mark.parametrize('changes', ({'v_lead': -0.5}, {'a_lead': -2.0}, {'distance': 5.2}))
def test_approaching_or_decelerating_lead_is_not_overwritten(changes):
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state(**changes)
  assert update(guard, source, t=1.25) is source


@pytest.mark.parametrize('changes', ({'v_lead': 2.0}, {'v_lead': -2.0}, {'distance': float('nan')}))
def test_nonstationary_or_nonfinite_lead_never_acquires_stop_hold(changes):
  guard = StoppingLeadFilter()
  for index in range(10):
    source = radar_state(**changes)
    assert update(guard, source, t=1.0 + index * 0.05) is source


def test_matching_speed_and_constant_range_while_driving_is_not_stationary():
  guard = StoppingLeadFilter()
  for index in range(10):
    source = radar_state(v_lead=5.0, v_rel=0.0)
    assert update(guard, source, t=1.0 + index * 0.05, v_ego=5.0) is source


def test_missing_or_vision_only_track_does_not_inherit_stopped_radar_history():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state()
  source.leadOne.radar = False
  assert update(guard, source, t=1.25) is source
  source.leadOne.radar = True
  assert update(guard, source, t=1.3) is source
  source.leadOne.status = False
  assert update(guard, source, t=1.35) is source
