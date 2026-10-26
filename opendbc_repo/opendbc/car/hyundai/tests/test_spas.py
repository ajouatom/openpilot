import pytest
from opendbc.car.hyundai.spas import Session, SPAS, RESPONSE, EXTENDED, DISABLE, ENABLE, DEFAULT, frame, compatible, checksum


def stock(s, t=10.0, count=10):
  for i in range(count):
    for addr in SPAS:
      m = frame(addr, i, None, s.bus)
      s.observe(m.address, m.src, m.dat, t + i * 0.02)
  return t + (count - 1) * 0.02


def step(s, t, **kw):
  values = dict(healthy=True, request="a", direction="left")
  values.update(kw)
  return s.update(t, **values)


def reply(s, t, body):
  s.observe(RESPONSE, s.bus, bytes([len(body)]) + body + bytes(7 - len(body)), t)


def opened(bus=0):
  s = Session(bus)
  t = stock(s)
  step(s, t, request=None)
  assert step(s, t + 0.01)[0].dat == EXTENDED
  reply(s, t + 0.02, b"\x50\x03")
  assert step(s, t + 0.03)[0].dat == DISABLE
  reply(s, t + 0.04, b"\x68\x03")
  assert step(s, t + 0.05) == []
  assert step(s, t + 0.16)
  assert s.state == "active"
  return s, t + 0.16


def test_supplied_vectors_match():
  assert frame(0x16A, 0, "left", 0).dat.hex() == "db65000000000000000000000000000060000000000000000000000000000000"
  assert frame(0x16A, 0, "right", 0).dat.hex() == "848c000000000000000000000000000080000000000000000000000000000000"


@pytest.mark.parametrize("bus", [0, 1, 4, 5])
def test_bus_contract_and_native_frames(bus):
  assert compatible({bus: SPAS}, bus)
  assert not compatible({bus: {0x165: 16, 0x16A: 32}}, bus)
  assert not compatible({bus + 1: SPAS}, bus)
  s, t = opened(bus)
  messages = step(s, t + 0.021)
  assert [m.address for m in messages] == list(SPAS)
  assert all(m.src == bus and int.from_bytes(m.dat[:2], "little") == checksum(m.address, m.dat) for m in messages)


@pytest.mark.parametrize("failure", ["wrong_bus", "wrong_dlc", "bad_crc", "repeated_counter", "stale", "insufficient"])
def test_incompatible_stock_never_requests_disable(failure):
  s = Session(0)
  for i in range(10):
    for a in SPAS:
      m = frame(a, 0 if failure == "repeated_counter" else i, None, 1 if failure == "wrong_bus" else 0)
      data = m.dat
      if failure == "wrong_dlc":
        data = data[:-1]
      if failure == "bad_crc":
        data = bytes([data[0] ^ 1]) + data[1:]
      if failure == "insufficient" and i > 7:
        continue
      s.observe(a, m.src, data, 10 + i * 0.02)
  t = 11.0 if failure == "stale" else 10.19
  step(s, t, request=None)
  assert step(s, t + 0.01) == []


@pytest.mark.parametrize("body", [b"\x7f\x10\x22", b"\x50\x01", b"\x68\x03"])
def test_missing_or_negative_extended_ack_never_sends_spas(body):
  s = Session(0)
  t = stock(s)
  step(s, t, request=None)
  step(s, t + 0.01)
  reply(s, t + 0.02, body)
  messages = []
  for i in range(1, 50):
    messages += step(s, t + 0.02 + i * 0.01)
  assert not any(m.address in SPAS or m.dat == DISABLE for m in messages)
  assert s.state.startswith("restore_")


def test_stock_sender_must_be_quiet():
  s = Session(0)
  t = stock(s)
  step(s, t, request=None)
  step(s, t + 0.01)
  reply(s, t + 0.02, b"\x50\x03")
  step(s, t + 0.03)
  reply(s, t + 0.04, b"\x68\x03")
  step(s, t + 0.05)
  for i in range(1, 45):
    for a in SPAS:
      m = frame(a, (9 + i) % 256, None, 0)
      s.observe(a, 0, m.dat, t + 0.05 + i * 0.01)
    assert not any(m.address in SPAS for m in step(s, t + 0.05 + i * 0.01))
  assert s.reason == "ecu_not_quiet"


@pytest.mark.parametrize("cause", ["unhealthy", "gap", "request", "direction", "driver", "stock", "deadline"])
def test_active_session_revoked_and_restores_on_every_failure(cause):
  s, t = opened()
  kw = {}
  dt = 0.01
  if cause == "unhealthy":
    kw["healthy"] = False
  elif cause == "gap":
    dt = 0.21
  elif cause == "request":
    kw["request"] = "new"
  elif cause == "direction":
    kw["direction"] = "right"
  elif cause == "driver":
    kw["driver_override"] = True
  elif cause == "deadline":
    s.started = t - 301
  else:
    m = frame(0x16A, 10, None, 0)
    s.observe(0x16A, 0, m.dat, t + 0.001)
  assert step(s, t + dt, **kw)[0].dat == ENABLE
  assert s.state == "restore_enable"


def test_recovery_requires_reply_and_resumed_stock_and_no_replay():
  s, t = opened()
  step(s, t + 0.01, request=None)
  reply(s, t + 0.02, b"\x68\x00")
  assert step(s, t + 0.03, request=None)[0].dat == DEFAULT
  reply(s, t + 0.04, b"\x50\x01")
  step(s, t + 0.05, request=None)
  assert s.state == "restore_stock"
  for i in range(12):
    for a in SPAS:
      m = frame(a, 20 + i, None, 0)
      s.observe(a, 0, m.dat, t + 0.06 + i * 0.02)
    step(s, t + 0.06 + i * 0.02, request=None)
  assert s.state == "idle" and s.stock_resumed and s.restore_ack
  # A request consumed during failed conditions does not join later.
  step(s, t + 0.30, healthy=False)
  assert step(s, t + 0.31) == []


def test_restore_is_bounded_and_cannot_rearm_after_failure():
  s, t = opened()
  step(s, t + 0.01, request=None)
  sent = []
  for i in range(1, 900):
    sent += step(s, t + 0.01 + i * 0.01, request=None)
  assert s.state == "failed" and s.reason == "restore_unconfirmed"
  assert sum(m.dat == ENABLE for m in sent) == 2
  stock(s, t + 10)
  assert step(s, t + 10.2, request="new") == []


def test_fifty_hz_no_burst_after_delayed_update():
  s, t = opened()
  out = []
  for i in range(1, 100):
    msgs = step(s, t + i * 0.01)
    out += [(t + i * 0.01, m) for m in msgs if m.address == 0x16A]
  assert all(b[0] - a[0] >= 0.019999 for a, b in zip(out, out[1:], strict=False))
  assert all(b[1].dat[2] == (a[1].dat[2] + 1) % 256 for a, b in zip(out, out[1:], strict=False))
