"""Interpret passive CSID timestamp history, without changing runtime validity.

Callers must first verify trace continuity and normal-frame register semantics.
The hardware records a CSI SOF, not an independently observed sensor exposure.
"""


def classify_sof_history(last_seen_ns: int, current_ns: int, previous_ns: int,
                         verify_ns: int, gap_ns: int = 75_000_000) -> str:
  if current_ns != verify_ns:
    return "unstable_register_sample"
  if min(last_seen_ns, current_ns, previous_ns) <= 0:
    return "insufficient_history"
  if current_ns <= last_seen_ns:
    return "nonadvancing_hardware_timestamp"
  if previous_ns >= current_ns:
    return "invalid_hardware_history"
  if current_ns - last_seen_ns <= gap_ns:
    return "normal_observed_interval"
  if last_seen_ns < previous_ns < current_ns:
    return "intervening_hardware_sof"
  if previous_ns == last_seen_ns:
    return "hardware_sof_gap"
  return "insufficient_history"
