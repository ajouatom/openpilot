from panda.tests.libpanda.libpanda_py import libpanda


BOOT_STANDBY = 0
BOOT_BOOTKICK = 1


def test_wake_rising_edge_and_high_wake_do_not_block_ignition_bootkick():
  libpanda.bootkick_test_reset()

  libpanda.bootkick_test_tick(False, True, False)
  assert libpanda.bootkick_test_get_state() == BOOT_STANDBY

  libpanda.bootkick_test_tick(False, False, True)
  assert libpanda.bootkick_test_get_state() == BOOT_BOOTKICK

  libpanda.bootkick_test_tick(False, True, True)
  assert libpanda.bootkick_test_get_state() == BOOT_STANDBY

  libpanda.bootkick_test_tick(True, False, True)
  assert libpanda.bootkick_test_get_state() == BOOT_BOOTKICK
