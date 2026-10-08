#include <stdbool.h>
#include <stdint.h>

#include "config.h"

typedef uint32_t GPIO_TypeDef;
typedef struct {} USART_TypeDef;
typedef struct {} TIM_TypeDef;
typedef struct harness_configuration harness_configuration;

#include "drivers/uart_declarations.h"
#include "drivers/harness_declarations.h"
#include "boards/board_declarations.h"
#include "main_declarations.h"

extern struct harness_t harness;
extern uart_ring uart_ring_som_debug;

#include "drivers/bootkick.h"

struct harness_t harness;
uart_ring uart_ring_som_debug;

static bool bootkick_test_som_gpio = false;
static BootState bootkick_test_state = BOOT_BOOTKICK;

static void bootkick_test_set_bootkick(BootState state) {
  bootkick_test_state = state;
}

static bool bootkick_test_read_som_gpio(void) {
  return bootkick_test_som_gpio;
}

static board bootkick_test_board = {
  .set_bootkick = bootkick_test_set_bootkick,
  .read_som_gpio = bootkick_test_read_som_gpio,
};

void bootkick_test_reset(void) {
  current_board = &bootkick_test_board;
  harness.status = HARNESS_STATUS_NC;
  harness.sbu1_voltage_mV = 0U;
  harness.sbu2_voltage_mV = 0U;
  harness.relay_driven = false;
  harness.sbu_adc_lock = false;
  uart_ring_som_debug.w_ptr_tx = 0U;
  bootkick_test_som_gpio = false;
  bootkick_test_state = BOOT_BOOTKICK;
  bootkick_reset_triggered = false;
}

void bootkick_test_tick(bool ignition, bool recent_heartbeat, bool wake) {
  bootkick_tick(ignition, recent_heartbeat, wake);
}

uint8_t bootkick_test_get_state(void) {
  return (uint8_t)bootkick_test_state;
}
