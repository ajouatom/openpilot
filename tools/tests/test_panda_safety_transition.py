"""Exercise production transition/configuration functions with mock HW registers.

These are control-flow tests, not MCU timing or physical relay/CAN validation.
CC can select a native compiler, e.g. 'zig cc'. No Panda is connected.
"""
import os
from pathlib import Path
import shlex
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]


def function(path, signature):
  source = (ROOT / path).read_text(encoding="utf-8")
  start = source.index(signature)
  end = source.index("{", start) + 1
  depth = 1
  while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
  return source[start:end]


@pytest.mark.parametrize("h7", [True, False])
@pytest.mark.parametrize("hyundai_mode", [8, 23, 28])
def test_production_safety_transition(tmp_path, h7, hyundai_mode):
  common = "panda/board/drivers/can_common.h"
  fdcan = "panda/board/drivers/fdcan.h"
  prefix = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#undef NDEBUG
#include <assert.h>
#define STM32H7
#define PANDA_CAN_CNT 3
#define FDCAN_CCCR_INIT 1U
#define FDCAN_CCCR_CSR 2U
#define FDCAN_CCCR_CSA 4U
#define FDCAN_CCCR_MON 8U
#define FDCAN_CCCR_TEST 16U
#define FDCAN_PSR_BO 1U
#define FDCAN_PSR_EP 2U
#define FDCAN_PSR_EW 4U
#define FDCAN_IR_BO 1U
#define FDCAN_IR_EP 2U
#define FDCAN_IR_PEA 4U
#define FDCAN_IR_PED 8U
#define FDCAN_IR_RF0L 16U
#define FDCAN_RXF0S_F0FL 127U
#define FDCAN_RXF0S_F0GI 0x3f00U
#define FDCAN_RXF0S_F0GI_Pos 8U
#define FDCAN_RX_FIFO_0_EL_CNT 46U
#define MIN(a,b) ((a)<(b)?(a):(b))
#define REGISTER_INTERRUPT(...)
enum { SAFETY_SILENT=0, SAFETY_ELM327=3, SAFETY_HYUNDAI=8, SAFETY_NOOUTPUT=19,
       SAFETY_HYUNDAI_LEGACY=23, SAFETY_HYUNDAI_CANFD=28,
       CAN_MODE_NORMAL=0, CAN_MODE_OBD_CAN2=1, ALL_CAN_LIVE=0, ALL_CAN_SILENT=255,
       POWER_SAVE_STATUS_DISABLED=0, HARNESS_STATUS_NC=0 };
typedef struct { uint32_t CCCR, PSR, IR, TXBRP, RXF0S, RXF0A; } FDCAN_GlobalTypeDef;
static FDCAN_GlobalTypeDef regs[3];
#define CANIF_FROM_CAN_NUM(i) (&regs[i])
typedef struct { bool initialized; uint8_t bus; uint32_t speed, data_speed; bool non_iso, loopback; int silent; } fdcan_config_t;
static fdcan_config_t initialized_can_config[3];
static struct { uint8_t bus_lookup; uint32_t can_speed, can_data_speed; bool canfd_non_iso; } bus_config[3];
#define BUS_NUM_FROM_CAN_NUM(i) bus_config[i].bus_lookup
static struct { uint8_t status; } harness;
static struct { bool has_harness; } hc;
static int mux_calls, init_calls, process_calls, clear_send_calls, critical;
static void mux(uint8_t mode) { (void)mode; mux_calls++; }
static struct { bool has_canfd; typeof(hc) *harness_config; void (*set_can_mode)(uint8_t); } board = {true, &hc, mux};
#define current_board (&board)
static uint8_t applied_can_mode, applied_can_harness_status;
static uint16_t current_safety_mode, current_safety_param;
static int can_silent, power_save_status, safety_tx_blocked, safety_rx_invalid, heartbeat_counter;
static bool can_loopback, heartbeat_lost, relay, hook_fail, speed_ok, init_ok;
static int queue[3], host_rx;
static int *can_queues[3] = {&queue[0], &queue[1], &queue[2]};
#define ENTER_CRITICAL() do { critical++; } while (0)
#define EXIT_CRITICAL() do { assert(critical>0); critical--; } while (0)
static void can_clear(int *q) { *q=0; }
static uint32_t microsecond_timer_get(void) { return 123; }
static uint32_t get_ts_elapsed(uint32_t now, uint32_t before) { return now-before; }
static void print(const char *s) { (void)s; }
static void puth(uint32_t x) { (void)x; }
static void assert_fatal(bool ok, const char *s) { (void)s; assert(ok); }
static int set_safety_hooks(uint16_t mode, uint16_t param) {
  if (mode==65535 || (hook_fail && mode==HYUNDAI_MODE)) return -1;
  current_safety_mode=mode; current_safety_param=param; return 0;
}
static void set_intercept_relay(bool on, bool ignition) { (void)ignition; relay=on; }
static void can_clear_send(FDCAN_GlobalTypeDef *can, int n) { (void)can; (void)n; clear_send_calls++; }
static bool can_set_speed(uint8_t n) { (void)n; return speed_ok; }
static bool llcan_init(FDCAN_GlobalTypeDef *can) { (void)can; init_calls++; return init_ok; }
static void process_can(uint8_t n) { (void)n; process_calls++; }
'''
  functions = "\n".join([function(common, "void can_set_mode(")] + ([
    function(fdcan, "static bool can_preserve_configuration("),
    function(fdcan, "static void can_clear_safety_transition_queues("),
  ] if h7 else []) + [
    function(fdcan, "bool can_init("),
    function(common, "void can_init_all("),
    function("panda/board/main.c", "void set_safety_mode("),
  ])
  body = r'''
static void setup(void) {
  harness.status=1; hc.has_harness=true; board.has_canfd=true;
  current_safety_mode=SAFETY_ELM327; current_safety_param=1;
  can_silent=ALL_CAN_LIVE; can_loopback=false; power_save_status=0;
  critical=0; hook_fail=false; speed_ok=true; init_ok=true; relay=false;
  for(int i=0;i<3;i++) {
    regs[i]=(FDCAN_GlobalTypeDef){0}; bus_config[i].bus_lookup=i;
    bus_config[i].can_speed=5000; bus_config[i].can_data_speed=20000;
    bus_config[i].canfd_non_iso=false; assert(can_init(i));
    queue[i]=4;
  }
  can_set_mode(CAN_MODE_NORMAL); host_rx=7;
  init_calls=mux_calls=process_calls=clear_send_calls=0;
}
int main(void) {
  const bool fast_supported = FAST_SUPPORTED;
  setup(); regs[0].RXF0S=3U | (5U<<8); regs[1].RXF0S=127U | (9U<<8);
  set_safety_mode(HYUNDAI_MODE,2077);
  assert(init_calls==(fast_supported?0:3) && mux_calls==(fast_supported?0:1) && relay && critical==0);
  assert(current_safety_mode==HYUNDAI_MODE && current_safety_param==2077);
  assert(queue[0]==0 && queue[1]==0 && queue[2]==0 && host_rx==7);
  if (fast_supported) assert(regs[0].RXF0A==5 && regs[1].RXF0A==9);
  // Every unhealthy/changed configuration must retain full reinitialization.
  for (int cause=0;cause<20;cause++) {
    setup();
    switch(cause) {
      case 0: initialized_can_config[0].initialized=false; break;
      case 1: bus_config[1].can_speed++; break;
      case 2: bus_config[2].can_data_speed++; break;
      case 3: bus_config[0].canfd_non_iso=true; break;
      case 4: can_loopback=true; break;
      case 5: can_silent=ALL_CAN_SILENT; break;
      case 6: regs[0].CCCR=FDCAN_CCCR_INIT; break;
      case 7: regs[1].CCCR=FDCAN_CCCR_CSA; break;
      case 8: regs[2].PSR=FDCAN_PSR_BO; break;
      case 9: regs[1].IR=FDCAN_IR_PEA; break;
      case 10: regs[2].TXBRP=1; break;
      case 11: can_set_mode(CAN_MODE_OBD_CAN2); break;
      case 12: harness.status=2; break;
      case 13: power_save_status=1; break;
      case 14: hc.has_harness=false; break;
      case 15: current_safety_param=0; break;
      case 16: current_safety_mode=SAFETY_HYUNDAI_CANFD; break;
      case 17: bus_config[0].bus_lookup=2; break;
      case 18: hook_fail=true; break;
      case 19: harness.status=0; applied_can_harness_status=0; break;
    }
    set_safety_mode(HYUNDAI_MODE,29);
    assert(init_calls==3 && critical==0);
    assert(queue[0]==0 && queue[1]==0 && queue[2]==0 && host_rx==7);
    if (cause==18) assert(!relay && can_silent==ALL_CAN_SILENT);
  }
  const uint16_t modes[]={0,19,3,65535,1,8,23,28};
  for(unsigned int i=0;i<sizeof(modes)/sizeof(modes[0]);i++) {
    for(unsigned int j=0;j<sizeof(modes)/sizeof(modes[0]);j++) {
      setup(); current_safety_mode=modes[i];
      set_safety_mode(modes[j],1);
      assert(critical==0);
      const bool hyundai = modes[j]==8 || modes[j]==23 || modes[j]==28;
      assert(init_calls==((fast_supported && modes[i]==3 && hyundai)?0:3));
      if(modes[j]==0 || modes[j]==65535) assert(can_silent==ALL_CAN_SILENT && !relay);
    }
  }
  for(int speed=0;speed<2;speed++) for(int init=0;init<2;init++) {
    setup(); speed_ok=speed; init_ok=init;
    assert(can_init(0)==(speed && init));
    assert(initialized_can_config[0].initialized==(speed && init));
  }
  puts("transition/configuration/queue/failure tests passed");
  return 0;
}
'''
  source = tmp_path / "transition.c"
  executable = tmp_path / ("transition.exe" if os.name == "nt" else "transition")
  if not h7:
    prefix = prefix.replace("#define STM32H7", "")
  source.write_text((prefix + functions + body.replace("FAST_SUPPORTED", "true" if h7 else "false"))
                    .replace("HYUNDAI_MODE", str(hyundai_mode)), encoding="utf-8")
  compiler = shlex.split(os.environ.get("CC", "cc"))
  subprocess.run([*compiler, "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(executable)], check=True)
  subprocess.run([str(executable)], check=True, timeout=10)


def test_fdcan_sleep_exit_is_bounded(tmp_path):
  source = tmp_path / "init.c"
  executable = tmp_path / ("init.exe" if os.name == "nt" else "init")
  source.write_text(r'''
#include <stdbool.h>
#include <stdint.h>
#undef NDEBUG
#include <assert.h>
#define CAN_INIT_TIMEOUT_MS 500U
#define FDCAN_CCCR_CSR 1U
#define FDCAN_CCCR_CSA 2U
#define FDCAN_CCCR_INIT 4U
typedef struct { uint32_t CCCR; } FDCAN_GlobalTypeDef;
static FDCAN_GlobalTypeDef can;
static uint32_t calls, release_after;
static void delay(uint32_t cycles) {
  assert(cycles==10000); calls++;
  if (calls==release_after) can.CCCR &= ~FDCAN_CCCR_CSA;
}
''' + function("panda/board/stm32h7/llfdcan.h", "static bool fdcan_request_init(") + r'''
int main(void) {
  can.CCCR=FDCAN_CCCR_CSR | FDCAN_CCCR_CSA;
  calls=0; release_after=0;
  assert(!fdcan_request_init(&can)); assert(calls==500);
  assert(!(can.CCCR & FDCAN_CCCR_INIT));
  can.CCCR=FDCAN_CCCR_CSR | FDCAN_CCCR_CSA;
  calls=0; release_after=3;
  assert(fdcan_request_init(&can)); assert(calls==3);
  assert(can.CCCR & FDCAN_CCCR_INIT);
  can.CCCR=0; calls=0;
  assert(fdcan_request_init(&can)); assert(calls==0);
  return 0;
}
''', encoding="utf-8")
  compiler = shlex.split(os.environ.get("CC", "cc"))
  subprocess.run([*compiler, "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(executable)], check=True)
  subprocess.run([str(executable)], check=True, timeout=10)
