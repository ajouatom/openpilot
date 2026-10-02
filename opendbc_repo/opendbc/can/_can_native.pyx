# cython: language_level=3
# distutils: language=c++
"""CAN integer loops. Keep Python arbitrary-width integers and rounding semantics."""
import math
from opendbc.car.carlog import carlog
from opendbc.can.dbc import SignalType

cdef object raw_value(object dat, object sig):
  cdef int i, bits, lsb, msb, size
  cdef int sig_lsb = sig.lsb, sig_msb = sig.msb
  cdef bint little = sig.is_little_endian
  cdef object ret, d
  ret = 0
  i = sig_msb // 8
  bits = sig.size
  while 0 <= i < len(dat) and bits > 0:
    lsb = sig_lsb if (sig_lsb // 8) == i else i * 8
    msb = sig_msb if (sig_msb // 8) == i else (i + 1) * 8 - 1
    size = msb - lsb + 1
    d = (dat[i] >> (lsb - (i * 8))) & (((<object>1) << size) - 1)
    ret |= d << (bits - size)
    bits -= size
    i = i - 1 if little else i + 1
  return ret


def raw_values(dat, signals):
  cdef object sig, value
  cdef list result = []
  for sig in signals:
    value = raw_value(dat, sig)
    if sig.is_signed:
      value -= ((value >> (sig.size - 1)) & 1) * (1 << sig.size)
    result.append(value)
  return result


cpdef set_value(bytearray msg, object sig, object ival):
  cdef int i, bits, shift, size, mask, lsb = sig.lsb
  cdef bint little = sig.is_little_endian
  i = lsb // 8
  bits = sig.size
  if sig.size < 64:
    ival &= (1 << sig.size) - 1
  while 0 <= i < len(msg) and bits > 0:
    shift = lsb % 8 if (lsb // 8) == i else 0
    size = min(bits, 8 - shift)
    mask = (((<object>1) << size) - 1) << shift
    msg[i] &= ~mask
    msg[i] |= (ival & (((<object>1) << size) - 1)) << shift
    bits -= size
    ival >>= size
    i = i + 1 if little else i - 1


def pack(dbc, dict counters, address, dict values, rx_counter=None):
  msg = dbc.addr_to_msg.get(address)
  if msg is None:
    carlog.error(f"msg not found for {address=}")
    return bytearray()
  dat = bytearray(msg.size)
  counter_set = False
  for name, value in values.items():
    sig = msg.sigs.get(name)
    if sig is None:
      carlog.error(f"unknown signal {name=} in {msg.name}")
      continue
    ival = int(math.floor((value - sig.offset) / sig.factor + 0.5))
    if ival < 0:
      ival = (1 << sig.size) + ival
    set_value(dat, sig, ival)
    if sig.type == SignalType.COUNTER or sig.name == "COUNTER":
      counters[address] = int(value)
      counter_set = True
  sig_counter = next((s for s in msg.sigs.values() if s.type == SignalType.COUNTER or s.name == "COUNTER"), None)
  if sig_counter and not counter_set:
    if address not in counters:
      counters[address] = 0 if rx_counter is None else (int(rx_counter) + 1) % (1 << sig_counter.size)
    set_value(dat, sig_counter, counters[address])
    counters[address] = (counters[address] + 1) % (1 << sig_counter.size)
  sig_checksum = next((s for s in msg.sigs.values() if s.type > SignalType.COUNTER), None)
  if sig_checksum and sig_checksum.calc_checksum:
    checksum = sig_checksum.calc_checksum(address, sig_checksum, dat)
    set_value(dat, sig_checksum, checksum)
  return dat
