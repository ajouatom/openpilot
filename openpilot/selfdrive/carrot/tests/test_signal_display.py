import json
import pytest

from openpilot.selfdrive.carrot.signal_display import DisplayContinuity, SignalDisplayReader, publish_display, visible_tracks, project_box


def record():
  return dict(version=1, stream='road', size=[1344, 760], frame_id=20, timestamp=10., session='worker',
              tracks=[dict(id=3, box=[600, 200, 640, 215], state='red', raw='red', age=0.)])


CAMERA = (22, 10.1, 1344, 760, False)


def test_red_green_unknown_are_observations_not_permissions():
  r = record()
  assert visible_tracks(r, 10.2, CAMERA)[0]['state'] == 'red'
  r['tracks'][0].update(raw='green', state='green')
  assert visible_tracks(r, 10.2, CAMERA)[0]['state'] == 'green'
  r['tracks'][0]['raw'] = 'unknown'
  assert visible_tracks(r, 10.2, CAMERA)[0]['state'] == 'unknown'


def test_green_first_candidate_is_visible_without_claiming_confirmation():
  r = record()
  r['tracks'][0].update(raw='green', state='unknown')
  track = visible_tracks(r, 10.2, CAMERA)[0]
  assert track['state'] == 'green' and not track['confirmed']
  r['tracks'][0]['state'] = 'red'
  assert not visible_tracks(r, 10.2, CAMERA)[0]['confirmed']
  r['tracks'][0]['state'] = 'green'
  assert visible_tracks(r, 10.2, CAMERA)[0]['confirmed']


def test_remembered_green_does_not_override_missing_or_red_current_evidence():
  r = record()
  r['tracks'][0].update(raw='unknown', state='green')
  assert visible_tracks(r, 10.2, CAMERA)[0]['state'] == 'unknown'
  r['tracks'][0]['raw'] = 'red'
  track = visible_tracks(r, 10.2, CAMERA)[0]
  assert track['state'] == 'red' and not track['confirmed']


@pytest.mark.parametrize('change', [dict(timestamp=9.8), dict(timestamp=11), dict(timestamp=float('nan')),
                                  dict(stream='wide'), dict(size=[1928, 1208]), dict(frame_id=5),
                                  dict(version=2), dict(tracks=None), dict(session=None)])
def test_stale_wrong_camera_or_invalid_records_are_hidden(change):
  r = record(); r.update(change)
  assert visible_tracks(r, 10.2, CAMERA) == ()


@pytest.mark.parametrize('camera', [None, (22, 9.8, 1344, 760, False), (22, 11., 1344, 760, False),
                                  (22, 10.1, 1344, 760, True), (22, 10.1, 1928, 1208, False)])
def test_wrong_or_stale_displayed_image_hides_boxes(camera):
  assert visible_tracks(record(), 10.2, camera) == ()


@pytest.mark.parametrize('change', [dict(age=.05), dict(box=[-1, 0, 30, 10]), dict(box=[0, 0, 10, 800]),
                                  dict(box=[2, 0, 1, 10]), dict(box=[0, 0, float('nan'), 10]), dict(id='oops')])
def test_lost_invalid_or_out_of_frame_boxes_are_hidden(change):
  r = record(); r['tracks'][0].update(change)
  assert visible_tracks(r, 10.2, CAMERA) == ()


def test_mapping_uses_actual_camera_rectangle_with_crop_offsets():
  assert project_box([0, 0, 1344, 760], (-50, -20, 672, 380)) == (-50, -20, 672, 380)
  assert project_box([672, 380, 1344, 760], (-50, -20, 672, 380)) == (286, 170, 336, 190)


def test_reader_expires_even_when_file_does_not_change_and_handles_invalid_data(tmp_path):
  flag, path = tmp_path/'display_enabled', tmp_path/'display.json'
  path.write_text(json.dumps(record()))
  reader = SignalDisplayReader(flag, path)
  assert reader.read(CAMERA, 10.2) == () and not reader.enabled
  flag.write_text('1')
  reader = SignalDisplayReader(flag, path)
  assert len(reader.read(CAMERA, 10.2)) == 1
  assert reader.read(CAMERA, 10.4)[0]['held']
  assert reader.read((62, 12.08, 1344, 760, False), 12.1) == ()
  for bad in ['x'*9000, '[]', 'null', '{']:
    path.write_text(bad)
    reader.next_read = 0
    assert reader.read(CAMERA, 10.2) == ()
  flag.write_text('0')
  assert reader.read(CAMERA, 13.5) == () and not reader.enabled


def test_display_has_separate_endpoint_and_no_control_fields(tmp_path):
  r = record(); t = r['tracks'][0]
  source = dict(tracks=[dict(t, evidence={'raw':'red'})], frame_id=20, timestamp_eof=10_000_000_000,
                session='worker', revalidated=True, daytime_comparison=True)
  path = tmp_path/'display.json'
  publish_display(source, path=path)
  out = json.loads(path.read_text())
  assert out['engine'] == 'trial' and len(visible_tracks(out, 10.2, CAMERA)) == 1
  assert not any(k in out for k in ['hold','red_sign','released','control_permission'])
  assert not (tmp_path/'carrot_signal_observation.json').exists()


def test_renderer_status_is_bounded_and_display_only(tmp_path):
  reader = SignalDisplayReader(tmp_path/'flag', tmp_path/'data')
  path = tmp_path/'status'
  reader.record_draw(CAMERA, 2, path=path, now=10.)
  assert not path.exists()
  reader.enabled = True; reader.record = record()
  reader.record_draw(CAMERA, 2, path=path, now=10.)
  before = path.read_bytes()
  reader.record_draw(CAMERA, 0, path=path, now=10.1)
  assert path.read_bytes() == before
  reader.record_draw(CAMERA, 0, path=path, now=11.)
  assert json.loads(path.read_text())['boxes_drawn'] == 0


def camera(now):
  return (20 + round((now - 10.) * 20), now - .01, 1344, 760, False)


def test_continuity_bridges_pending_result_without_refreshing_its_timestamp():
  memory = DisplayContinuity(); r = record()
  assert not memory.update(r, 10.2, camera(10.2))[0]['held']
  held = memory.update(r, 10.5, camera(10.5))[0]
  assert held['held'] and held['state'] == 'red' and not held['confirmed']
  assert held['age'] == pytest.approx(.5)
  assert memory.update(r, 11.8, camera(11.8))[0]['held']
  assert memory.update(r, 12.01, camera(12.01)) == ()


def test_new_opposite_color_replaces_previous_immediately():
  memory = DisplayContinuity(); r = record()
  r['tracks'][0].update(raw='green', state='green')
  memory.update(r, 10.2, camera(10.2))
  r.update(frame_id=26, timestamp=10.3)
  r['tracks'][0].update(raw='red', state='unknown')
  track = memory.update(r, 10.4, camera(10.4))[0]
  assert track['state'] == 'red' and not track['held'] and not track['confirmed']


def test_unknown_frames_move_box_but_cannot_renew_old_green():
  memory = DisplayContinuity(); r = record()
  r['tracks'][0].update(raw='green', state='green')
  memory.update(r, 10.1, camera(10.1))
  for timestamp in [10.3, 10.6, 11.5, 12.1]:
    r.update(frame_id=20 + round((timestamp-10)*20), timestamp=timestamp)
    r['tracks'][0].update(raw='unknown', box=[610, 210, 650, 225])
    track = memory.update(r, timestamp+.05, camera(timestamp+.05))[0]
    assert track['box'] == (610, 210, 650, 225)
    assert track['state'] == ('green' if timestamp < 12.0 else 'unknown')
    if timestamp < 12.0:
      assert track['held'] and not track['confirmed']


def test_temporarily_missing_object_expires_despite_new_empty_observations():
  memory = DisplayContinuity(); r = record()
  memory.update(r, 10.1, camera(10.1))
  for timestamp in [10.3, 10.6, 11.5, 12.1]:
    r.update(timestamp=timestamp, frame_id=20+round((timestamp-10)*20), tracks=[])
    tracks = memory.update(r, timestamp+.05, camera(timestamp+.05))
    assert bool(tracks) == (timestamp < 12.0)
    if tracks:
      assert tracks[0]['held']


def test_replacement_track_removes_overlapping_old_color():
  memory = DisplayContinuity(); r = record()
  memory.update(r, 10.1, camera(10.1))
  r.update(timestamp=10.3, frame_id=26)
  r['tracks'][0].update(id=4, raw='green')
  tracks = memory.update(r, 10.4, camera(10.4))
  assert len(tracks) == 1 and tracks[0]['id'] == 4 and tracks[0]['state'] == 'green'


def test_same_housing_unknown_new_ids_retain_color_without_renewing_age():
  memory = DisplayContinuity(); r = record()
  r['tracks'][0].update(raw='green', state='unknown')
  memory.update(r, 10.4, camera(10.4))
  for tid, timestamp in [(4, 10.45), (5, 10.9), (6, 11.35), (7, 11.8)]:
    r.update(timestamp=timestamp, frame_id=20+round((timestamp-10)*20))
    r['tracks'][0].update(id=tid, raw='unknown')
    tracks = memory.update(r, timestamp+.1, camera(timestamp+.1))
    assert len(tracks) == 1 and tracks[0]['id'] == tid
    assert tracks[0]['state'] == 'green' and tracks[0]['held']
  r.update(timestamp=12.05, frame_id=61)
  r['tracks'][0]['id'] = 8
  assert memory.update(r, 12.1, camera(12.1))[0]['state'] == 'unknown'


def test_ambiguous_housing_does_not_transfer_previous_color():
  memory = DisplayContinuity(); r = record()
  r['tracks'].append(dict(r['tracks'][0], id=4, box=[602, 200, 642, 215], raw='green'))
  memory.update(r, 10.1, camera(10.1))
  r.update(timestamp=10.4, frame_id=28, tracks=[dict(r['tracks'][0], id=5, raw='unknown')])
  tracks = memory.update(r, 10.5, camera(10.5))
  assert len(tracks) == 1 and tracks[0]['state'] == 'unknown'


@pytest.mark.parametrize('failure', ['wide', 'frozen', 'malformed', 'disabled', 'session'])
def test_continuity_cannot_cross_camera_failure_disable_or_worker_session(tmp_path, failure):
  flag, path = tmp_path/'flag', tmp_path/'data'
  flag.write_text('1'); r = record(); path.write_text(json.dumps(r))
  reader = SignalDisplayReader(flag, path)
  assert reader.read(camera(10.1), 10.1)
  cam = camera(10.7)
  if failure == 'wide': cam = (*cam[:4], True)
  elif failure == 'frozen': cam = CAMERA
  elif failure == 'malformed': path.write_text('invalid')
  elif failure == 'disabled': flag.write_text('0')
  else:
    r.update(session='new worker', timestamp=10.6, frame_id=32, tracks=[])
    path.write_text(json.dumps(r))
  assert reader.read(cam, 10.7) == ()
  assert not reader.continuity.tracks
