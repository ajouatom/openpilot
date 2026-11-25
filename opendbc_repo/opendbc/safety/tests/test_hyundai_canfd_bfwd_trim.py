# LFA (0x12A) camera-SCC forwarding queue: the standing-backlog trim in safety_hyundai_canfd.h.
# Each stock LFA frame from the camera carries the oldest queued host command. A host that runs
# slightly faster than the camera can leave the queue near capacity indefinitely.
import pytest

from opendbc.safety.tests.test_hyundai_canfd_cluster import Hooks, native as native_fixture

native = native_fixture  # reuse the session fixture that compiles the production hooks

LFA = 0x12A
PARAM = 29  # camera SCC + HDA2, as test_hyundai_canfd_fwd_timer
T = 10000   # stock LFA period, us


def lfa(marker, torque=0, req=True):
  d = bytearray(16)
  raw = (torque + 1024) & 0x7FF
  d[5] = (raw & 0x7F) << 1
  d[6] = ((raw >> 7) & 0xF) | (0x10 if req else 0)
  d[10:14] = marker.to_bytes(4, "little")
  return bytes(d)


def marker(out):
  return int.from_bytes(out[1][10:14], "little")


def torque(out):
  return (((out[1][6] & 0xF) << 7) | (out[1][5] >> 1)) - 1024


def req(out):
  return bool(out[1][6] & 0x10)


class Stream:
  def __init__(self, native):
    self.h = Hooks(native, PARAM)
    self.now = 1000
    self.n = 0

  def push(self, tq=0, rq=True):
    self.n += 1
    assert self.h.tx(LFA, lfa(self.n, tq, rq), self.now, bus=0) == 3
    self.now += 50
    return self.n

  def stock(self):
    self.now += T
    return self.h.rx(LFA, bytes(16), self.now, bus=2)


def backlog(s, pops, depth=3, tq=0):
  # keep `depth` commands queued before every stock frame
  for _ in range(depth):
    s.push(tq)
  out = []
  for _ in range(pops):
    out.append(s.stock())
    s.push(tq)
  return out


def test_standing_backlog_is_trimmed_to_target(native):
  s = Stream(native)
  out = backlog(s, 400, depth=5)
  # 5 queued -> trimmed one at a time down to 2 before a pop: newest output lags the newest push by 1
  assert s.n - marker(out[-1]) == 2
  assert all(marker(b) - marker(a) in (1, 2) for a, b in zip(out[:-1], out[1:], strict=True))


def test_target_depth_is_left_alone(native):
  s = Stream(native)
  out = backlog(s, 400, depth=2)
  assert all(marker(b) - marker(a) == 1 for a, b in zip(out[:-1], out[1:], strict=True))


@pytest.mark.parametrize("n", [3, 4, 5])
def test_burst_is_sent_in_order(native, n):
  s = Stream(native)
  s.push()
  s.push()
  first = [marker(s.stock()), marker(s.stock())]
  burst = [s.push() for _ in range(n)]
  seen = [marker(s.stock()) for _ in range(n)]
  assert first == [1, 2] and seen == burst


def _arm_with(s, tail):
  # Keep 3 commands queued before each stock frame. Commands c1..c3 are queued first; stock frame
  # i sends c_i and is followed by c_(i+3). Frames 1..99 are deep pops, so frame 100 is the first
  # that may trim, and it sees c100, c101, c102 queued with c99 as the last output.
  # `tail` = (torque, request) for c100, c101, c102; everything before is (0, request on).
  plan = [(0, True)] * 99 + list(tail)
  for k in range(3):
    s.push(*plan[k])
  outs = []
  for i in range(1, 100):
    outs.append(s.stock())
    s.push(*plan[i + 2])
  assert marker(outs[-1]) == 99 and s.n == 102
  return outs


def test_trim_happens_when_safe(native):
  s = Stream(native)
  _arm_with(s, [(0, True)] * 3)
  assert marker(s.stock()) == 101  # c100 trimmed


def test_no_trim_across_a_large_torque_step(native):
  s = Stream(native)
  _arm_with(s, [(10, True), (20, True), (30, True)])
  out = s.stock()
  assert marker(out) == 100 and torque(out) == 10  # skipping c100 would step 0 -> 20


@pytest.mark.parametrize("pattern", [(False, False, True), (False, True, True), (True, False, True)])
def test_request_off_frames_are_never_trimmed(native, pattern):
  s = Stream(native)
  _arm_with(s, [(0, rq) for rq in pattern])
  sent = [marker(s.stock()) for _ in range(3)]
  assert sent == [100, 101, 102]


def test_overflow_restarts_the_trim_count(native):
  s = Stream(native)
  _arm_with(s, [(0, True)] * 3)
  for _ in range(4):  # c103..c106: the 6-entry queue drops c100
    s.push()
  assert marker(s.stock()) == 101  # no extra trim on top of the overflow


@pytest.mark.parametrize("first,second", [(5, 10), (-5, -10), (0, 5)])
def test_trim_does_not_amplify_the_host_torque_step(native, first, second):
  s = Stream(native)
  _arm_with(s, [(first, True), (second, True), (second, True)])
  out = s.stock()
  assert marker(out) == 100 and torque(out) == first


def test_trim_preserves_the_valid_request_run_between_fault_avoidance_cuts(native):
  s = Stream(native)
  def push():
    s.push(0, s.n % 91 < 89)  # valid host sequence: 89 ON, 2 OFF
  for _ in range(3):
    push()
  runs, active = [], 0
  for _ in range(600):
    if req(s.stock()):
      active += 1
    elif active:
      runs.append(active)
      active = 0
    push()
  assert len(runs) >= 6 and min(runs) >= 89


@pytest.mark.parametrize("fault", ["length", "relay"])
def test_rejected_host_frame_cannot_enter_the_control_queue(native, fault):
  h = Hooks(native, PARAM)
  bad = lfa(1)[:12] if fault == "length" else lfa(1)
  assert h.tx(LFA, bad, 1000, bus=0, relay=fault == "relay") == 0
  assert h.tx(LFA, lfa(2), 2000, bus=0) == 3
  # Just one accepted packet: prefill is not met, so the stock frame survives.
  stock = lfa(999)
  assert h.rx(LFA, stock, 3000, bus=2) == (0, stock)


@pytest.mark.parametrize("address,bus,length", [(0x1A0, 0, 32), (0x0CB, 0, 24), (0x0EA, 2, 24),
                                              (0x1AA, 2, 16), (0x175, 2, 24)])
def test_other_control_fifos_keep_every_command(native, address, bus, length):
  h = Hooks(native, PARAM)
  def push(n):
    data = bytearray(length)
    # SCC raw/val acceleration = 0; request ON in byte 6 must not enable LFA trimming on another ID.
    if address == 0x1A0:
      data[16:19] = bytes((255, 0xF3, 63))
    data[6] = 0x10
    data[10:14] = n.to_bytes(4, 'little')
    assert h.tx(address, bytes(data), n * T, bus=bus) == 3
  for n in (1, 2, 3):
    push(n)
  for n in range(1, 401):
    out = h.rx(address, bytes(length), (n + 3) * T, bus=2 - bus)
    assert marker(out) == n
    push(n + 3)


def test_angle_mode_request_off_keeps_every_lfa_command(native):
  s = Stream(native)
  for _ in range(5):
    s.push(rq=False)
  for n in range(1, 401):
    assert marker(s.stock()) == n
    s.push(rq=False)


def test_low_depth_and_mode_reset_restart_deep_confirmation(native):
  s = Stream(native)
  _arm_with(s, [(0, True)] * 3)
  # Disabling the safety mode resets the FIFO and its pending trim confirmation.
  s.h.lib.alt2_test_init(PARAM)
  s.n = 0
  out = backlog(s, 99, depth=3)
  assert marker(out[-1]) == 99
  assert marker(s.stock()) == 101
  # A shallow pop resets confirmation, even after previously reaching the threshold.
  assert marker(s.stock()) == 102
  s.push()
  s.push()
  s.push()
  assert marker(s.stock()) == 103


@pytest.mark.parametrize("fault", ["length", "relay"])
def test_rejected_host_frame_cannot_overflow_or_reset_a_pending_trim(native, fault):
  s = Stream(native)
  _arm_with(s, [(0, True)] * 3)
  for n in range(4):
    bad = lfa(900 + n)[:12] if fault == "length" else lfa(900 + n)
    assert s.h.tx(LFA, bad, s.now, bus=0, relay=fault == "relay") == 0
  assert marker(s.stock()) == 101
