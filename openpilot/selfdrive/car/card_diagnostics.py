CARD_DIAG_LOOP_WARN_US = 50_000
CARD_DIAG_PROCESS_WARN_US = 20_000


def should_log_card_diagnostics(loop_max_us: int, process_max_us: int, can_timeouts: int) -> bool:
  """Only surface diagnostics for a CAN timeout or a severe card-loop stall."""
  return (can_timeouts > 0 or
          loop_max_us >= CARD_DIAG_LOOP_WARN_US or
          process_max_us >= CARD_DIAG_PROCESS_WARN_US)


def invalid_can_messages(can_parsers) -> str:
  """Names of timed-out CAN messages for the canError alert text2 (at most 2, one line)."""
  # shortcut: names timed-out messages only (valid()); counter/checksum faults leave it blank, add counter_fail if those show up
  names = []
  for cp in can_parsers:
    bus_timeout = cp.bus_timeout  # property loops over every message; read once per parser
    names += [f"{s.name} 0x{s.address:X}" for s in cp.message_states.values() if not s.valid(cp._last_update_nanos, bus_timeout)]
  return ", ".join(names[:2])
