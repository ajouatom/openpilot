CARD_DIAG_LOOP_WARN_US = 50_000
CARD_DIAG_PROCESS_WARN_US = 20_000


def should_log_card_diagnostics(loop_max_us: int, process_max_us: int, can_timeouts: int) -> bool:
  """Only surface diagnostics for a CAN timeout or a severe card-loop stall."""
  return (can_timeouts > 0 or
          loop_max_us >= CARD_DIAG_LOOP_WARN_US or
          process_max_us >= CARD_DIAG_PROCESS_WARN_US)
