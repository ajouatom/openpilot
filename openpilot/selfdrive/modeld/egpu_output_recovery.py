"""One missing eGPU publication with rollback; never relabel an old prediction."""
import time

from openpilot.selfdrive.modeld.output_validation import NonFiniteModelOutput, validate_compiled_output


MAX_HOLD_SECONDS = .100


class RepeatedInvalidModelOutput(ValueError):
  pass


class OutputRecovery:
  def __init__(self, runtime, count, report, clock=time.monotonic):
    self.runtime, self.count, self.report, self.clock = runtime, count, report, clock
    self.last_good = None
    self.failures = 0
    runtime.init_recovery_state()

  def run(self):
    # Device errors/timeouts and shape mismatches are never swallowed.
    result = self.runtime.run()
    try:
      validate_compiled_output(result, self.count)
    except NonFiniteModelOutput as exc:
      self.failures += 1
      if self.failures >= 2:
        raise RepeatedInvalidModelOutput(f'two consecutive invalid eGPU outputs: {exc}') from exc
      if self.last_good is None or not 0 <= self.clock() - self.last_good <= MAX_HOLD_SECONDS:
        raise
      # The graph writes recurrent inputs in place. A dropped publication alone
      # would leave corrupted history in the next inference.
      self.runtime.restore_recovery_state()
      if not 0 <= self.clock() - self.last_good <= MAX_HOLD_SECONDS:
        raise
      self.report('precompiledOutputHeld', consecutive=self.failures, error=str(exc))
      return None
    self.runtime.save_recovery_state()
    self.last_good = self.clock()
    if self.failures:
      self.report('precompiledOutputRecovered', consecutive=self.failures)
    self.failures = 0
    return result
