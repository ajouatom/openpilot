extern int _app_start[0xc000]; // Only first 3 sectors of size 0x4000 are used

// Prototypes
void set_safety_mode(uint16_t mode, uint16_t param);
bool is_car_safety_mode(uint16_t mode);

static int get_health_pkt(void *dat) {
  COMPILE_TIME_ASSERT(sizeof(struct health_t) <= USBPACKET_MAX_SIZE);
  struct health_t * health = (struct health_t*)dat;

  health->uptime_pkt = uptime_cnt;
  health->voltage_pkt = current_board->read_voltage_mV();
  health->current_pkt = current_board->read_current_mA();

  // Use the GPIO pin to determine ignition or use a CAN based logic
  health->ignition_line_pkt = (uint8_t)(current_board->check_ignition());
  health->ignition_can_pkt = ignition_can;

  health->controls_allowed_pkt = controls_allowed;
  health->safety_tx_blocked_pkt = safety_tx_blocked;
  health->safety_rx_invalid_pkt = safety_rx_invalid;
  health->tx_buffer_overflow_pkt = tx_buffer_overflow;
  health->rx_buffer_overflow_pkt = rx_buffer_overflow;
  health->car_harness_status_pkt = harness.status;
  health->safety_mode_pkt = (uint8_t)(current_safety_mode);
  health->safety_param_pkt = current_safety_param;
  health->alternative_experience_pkt = alternative_experience;
  health->power_save_enabled_pkt = power_save_status == POWER_SAVE_STATUS_ENABLED;
  health->heartbeat_lost_pkt = heartbeat_lost;
  health->safety_rx_checks_invalid_pkt = safety_rx_checks_invalid;

  health->spi_checksum_error_count_pkt = spi_checksum_error_count;

  health->fault_status_pkt = fault_status;
  health->faults_pkt = faults;

  health->interrupt_load_pkt = interrupt_load;

  health->fan_power = fan_state.power;
  health->fan_stall_count = fan_state.total_stall_count;

  health->sbu1_voltage_mV = harness.sbu1_voltage_mV;
  health->sbu2_voltage_mV = harness.sbu2_voltage_mV;

  health->som_reset_triggered = bootkick_reset_triggered;

  return sizeof(*health);
}

static bool clear_diagnostics(void) {
  const bool ignition = current_board->check_ignition() || ignition_can;
  if (ignition || is_car_safety_mode(current_safety_mode)) {
    return false;
  }

  ENTER_CRITICAL();
  spi_header_sync_nack_count = 0U;
  spi_header_checksum_nack_count = 0U;
  spi_data_checksum_nack_count = 0U;
  spi_endpoint3_checksum_nack_count = 0U;
  spi_endpoint3_backpressure_nack_count = 0U;
  spi_endpoint3_ack_count = 0U;
  spi_last_endpoint3_checksum_time_us = 0U;
  spi_last_endpoint3_backpressure_time_us = 0U;
  spi_last_endpoint3_checksum_len = 0U;
  spi_last_endpoint3_backpressure_len = 0U;
  (void)memset(spi_last_backpressure_tx_free, 0, sizeof(spi_last_backpressure_tx_free));

  (void)memset(tx_buffer_overflow_by_bus, 0, sizeof(tx_buffer_overflow_by_bus));
  (void)memset(can_error_hist, 0, sizeof(can_error_hist));
  (void)memset(can_error_diag, 0, sizeof(can_error_diag));
  (void)memset(can_diag_last_tx_addr, 0, sizeof(can_diag_last_tx_addr));
  (void)memset(can_diag_last_tx_time_us, 0, sizeof(can_diag_last_tx_time_us));
  (void)memset(can_diag_last_rx_addr, 0, sizeof(can_diag_last_rx_addr));
  (void)memset(can_diag_last_rx_time_us, 0, sizeof(can_diag_last_rx_time_us));
  (void)memset(can_diag_last_host_addr, 0, sizeof(can_diag_last_host_addr));
  (void)memset(can_diag_last_host_time_us, 0, sizeof(can_diag_last_host_time_us));
  (void)memset(can_diag_last_fwd_addr, 0, sizeof(can_diag_last_fwd_addr));
  (void)memset(can_diag_last_fwd_time_us, 0, sizeof(can_diag_last_fwd_time_us));

  #ifdef CANFD
    (void)memset(&hyundai_canfd_fwd_diag, 0, sizeof(hyundai_canfd_fwd_diag));
    for (uint8_t i = 0U; i < PANDA_DIAGNOSTICS_HYUNDAI_BUFFER_COUNT; i++) {
      CanfdBufferedFwd *buffer = &canfd_bfwd[i];
      buffer->push_attempt_cnt = 0U;
      buffer->push_accepted_cnt = 0U;
      buffer->push_full_drop_cnt = 0U;
      buffer->pop_attempt_cnt = 0U;
      buffer->pop_success_cnt = 0U;
      buffer->reuse_attempt_cnt = 0U;
      buffer->reuse_success_cnt = 0U;
      buffer->empty_fallback_cnt = 0U;
      buffer->reset_cnt = 0U;
      buffer->last_push_time_us = 0U;
      buffer->last_pop_time_us = 0U;
      buffer->last_reuse_time_us = 0U;
      buffer->last_empty_time_us = 0U;
    }
  #endif
  EXIT_CRITICAL();

  return true;
}

static int get_diagnostics_pkt(uint16_t page, void *dat) {
  COMPILE_TIME_ASSERT(sizeof(panda_spi_diag_t) == USBPACKET_MAX_SIZE);
  COMPILE_TIME_ASSERT(sizeof(panda_can_error_hist_t) == USBPACKET_MAX_SIZE);
  COMPILE_TIME_ASSERT(sizeof(panda_can_error_diag_t) == USBPACKET_MAX_SIZE);
  COMPILE_TIME_ASSERT(sizeof(panda_hyundai_fwd_diag_t) == USBPACKET_MAX_SIZE);
  COMPILE_TIME_ASSERT(sizeof(panda_hyundai_buffer_diag_t) == USBPACKET_MAX_SIZE);
  #ifdef CANFD
    COMPILE_TIME_ASSERT((sizeof(canfd_bfwd) / sizeof(canfd_bfwd[0])) == (PANDA_DIAGNOSTICS_HYUNDAI_BUFFER_COUNT + 1U));
  #endif

  int resp_len = 0;
  (void)memset(dat, 0, USBPACKET_MAX_SIZE);

  if (page == PANDA_DIAGNOSTICS_PAGE_SPI) {
    panda_spi_diag_t *diag = (panda_spi_diag_t *)dat;
    diag->version = PANDA_DIAGNOSTICS_VERSION;
    diag->page = PANDA_DIAGNOSTICS_PAGE_SPI;
    #ifdef ENABLE_SPI
      diag->spi_state = spi_state;
      diag->can_tx_ready = spi_can_tx_ready;
    #else
      diag->spi_state = UINT8_MAX;
      diag->can_tx_ready = 0U;
    #endif
    diag->header_sync_nack_cnt = spi_header_sync_nack_count;
    diag->header_checksum_nack_cnt = spi_header_checksum_nack_count;
    diag->data_checksum_nack_cnt = spi_data_checksum_nack_count;
    diag->endpoint3_checksum_nack_cnt = spi_endpoint3_checksum_nack_count;
    diag->endpoint3_backpressure_nack_cnt = spi_endpoint3_backpressure_nack_count;
    diag->endpoint3_ack_cnt = spi_endpoint3_ack_count;
    diag->last_endpoint3_checksum_time_us = spi_last_endpoint3_checksum_time_us;
    diag->last_endpoint3_backpressure_time_us = spi_last_endpoint3_backpressure_time_us;
    diag->last_endpoint3_checksum_len = spi_last_endpoint3_checksum_len;
    diag->last_endpoint3_backpressure_len = spi_last_endpoint3_backpressure_len;
    uint16_t current_tx_free[3] = {0U, 0U, 0U};
    comms_can_get_tx_queue_free(current_tx_free);
    for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
      diag->current_tx_free[i] = current_tx_free[i];
      diag->last_backpressure_tx_free[i] = spi_last_backpressure_tx_free[i];
      diag->tx_overflow_by_bus[i] = tx_buffer_overflow_by_bus[i];
    }
    resp_len = sizeof(*diag);
  } else if ((page >= PANDA_DIAGNOSTICS_PAGE_CAN_HIST_BASE) &&
             (page < (PANDA_DIAGNOSTICS_PAGE_CAN_HIST_BASE + PANDA_CAN_CNT))) {
    uint8_t can_number = page - PANDA_DIAGNOSTICS_PAGE_CAN_HIST_BASE;
    panda_can_error_hist_t *diag = (panda_can_error_hist_t *)dat;
    (void)memcpy(diag, &can_error_hist[can_number], sizeof(*diag));
    diag->version = PANDA_DIAGNOSTICS_VERSION;
    diag->page = (uint8_t)page;
    diag->can_number = can_number;
    diag->bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
    resp_len = sizeof(*diag);
  } else if ((page >= PANDA_DIAGNOSTICS_PAGE_CAN_SNAPSHOT_BASE) &&
             (page < (PANDA_DIAGNOSTICS_PAGE_CAN_SNAPSHOT_BASE + (PANDA_CAN_CNT * PANDA_CAN_ERROR_STAGE_COUNT)))) {
    uint8_t offset = page - PANDA_DIAGNOSTICS_PAGE_CAN_SNAPSHOT_BASE;
    uint8_t can_number = offset / PANDA_CAN_ERROR_STAGE_COUNT;
    uint8_t stage = offset % PANDA_CAN_ERROR_STAGE_COUNT;
    panda_can_error_diag_t *diag = (panda_can_error_diag_t *)dat;
    (void)memcpy(diag, &can_error_diag[can_number][stage], sizeof(*diag));
    diag->version = PANDA_DIAGNOSTICS_VERSION;
    diag->page = (uint8_t)page;
    diag->can_number = can_number;
    if (diag->valid == 0U) {
      diag->bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
    }
    resp_len = sizeof(*diag);
  }
  #ifdef CANFD
  else if (page == PANDA_DIAGNOSTICS_PAGE_HYUNDAI_SUMMARY) {
    panda_hyundai_fwd_diag_t *diag = (panda_hyundai_fwd_diag_t *)dat;
    diag->version = PANDA_DIAGNOSTICS_VERSION;
    diag->page = PANDA_DIAGNOSTICS_PAGE_HYUNDAI_SUMMARY;
    diag->enabled = hyundai_canfd_buffered_fwd;
    diag->safety_mode = (uint8_t)current_safety_mode;
    diag->safety_param = current_safety_param;
    diag->buffer_count = PANDA_DIAGNOSTICS_HYUNDAI_BUFFER_COUNT;
    diag->tx_buffered_cnt = hyundai_canfd_fwd_diag.tx_buffered_cnt;
    diag->fwd_call_cnt = hyundai_canfd_fwd_diag.fwd_call_cnt;
    diag->fwd_from_bus0_cnt = hyundai_canfd_fwd_diag.fwd_from_bus0_cnt;
    diag->fwd_from_bus2_cnt = hyundai_canfd_fwd_diag.fwd_from_bus2_cnt;
    diag->replace_to_bus0_cnt = hyundai_canfd_fwd_diag.replace_to_bus0_cnt;
    diag->replace_to_bus2_cnt = hyundai_canfd_fwd_diag.replace_to_bus2_cnt;
    diag->pass_to_bus0_cnt = hyundai_canfd_fwd_diag.pass_to_bus0_cnt;
    diag->pass_to_bus2_cnt = hyundai_canfd_fwd_diag.pass_to_bus2_cnt;
    diag->dynamic_block_to_bus0_cnt = hyundai_canfd_fwd_diag.dynamic_block_to_bus0_cnt;
    diag->dynamic_block_to_bus2_cnt = hyundai_canfd_fwd_diag.dynamic_block_to_bus2_cnt;
    diag->empty_to_bus0_cnt = hyundai_canfd_fwd_diag.empty_to_bus0_cnt;
    diag->empty_to_bus2_cnt = hyundai_canfd_fwd_diag.empty_to_bus2_cnt;
    diag->block_4b9_cnt = hyundai_canfd_fwd_diag.block_4b9_cnt;
    diag->invalid_bus_cnt = hyundai_canfd_fwd_diag.invalid_bus_cnt;
    resp_len = sizeof(*diag);
  } else if ((page >= PANDA_DIAGNOSTICS_PAGE_HYUNDAI_BUFFER_BASE) &&
             (page < (PANDA_DIAGNOSTICS_PAGE_HYUNDAI_BUFFER_BASE + PANDA_DIAGNOSTICS_HYUNDAI_BUFFER_COUNT))) {
    uint8_t index = page - PANDA_DIAGNOSTICS_PAGE_HYUNDAI_BUFFER_BASE;
    CanfdBufferedFwd *buffer = &canfd_bfwd[index];
    panda_hyundai_buffer_diag_t *diag = (panda_hyundai_buffer_diag_t *)dat;
    diag->version = PANDA_DIAGNOSTICS_VERSION;
    diag->page = (uint8_t)page;
    diag->index = index;
    diag->enabled = buffer->enabled;
    diag->addr = (uint16_t)buffer->addr;
    diag->dst_bus = (uint8_t)buffer->dst_bus;
    diag->count = buffer->count;
    diag->reuse_left = buffer->reuse_left;
    diag->started = buffer->started;
    diag->has_last_pkt = buffer->has_last_pkt;
    diag->push_attempt_cnt = buffer->push_attempt_cnt;
    diag->push_accepted_cnt = buffer->push_accepted_cnt;
    diag->push_full_drop_cnt = buffer->push_full_drop_cnt;
    diag->pop_attempt_cnt = buffer->pop_attempt_cnt;
    diag->pop_success_cnt = buffer->pop_success_cnt;
    diag->reuse_attempt_cnt = buffer->reuse_attempt_cnt;
    diag->reuse_success_cnt = buffer->reuse_success_cnt;
    diag->empty_fallback_cnt = buffer->empty_fallback_cnt;
    diag->reset_cnt = buffer->reset_cnt;
    diag->last_push_time_us = buffer->last_push_time_us;
    diag->last_pop_time_us = buffer->last_pop_time_us;
    diag->last_reuse_time_us = buffer->last_reuse_time_us;
    diag->last_empty_time_us = buffer->last_empty_time_us;
    resp_len = sizeof(*diag);
  }
  #endif

  return resp_len;
}

// send on serial, first byte to select the ring
void comms_endpoint2_write(const uint8_t *data, uint32_t len) {
  uart_ring *ur = get_ring_by_number(data[0]);
  if ((len != 0U) && (ur != NULL)) {
    if ((data[0] < 2U) || (data[0] >= 4U)) {
      for (uint32_t i = 1; i < len; i++) {
        while (!put_char(ur, data[i])) {
          // wait
        }
      }
    }
  }
}

int comms_control_handler(ControlPacket_t *req, uint8_t *resp) {
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  uint32_t time;

#ifdef DEBUG_COMMS
  print("raw control request: "); hexdump(req, sizeof(ControlPacket_t)); print("\n");
  print("- request "); puth(req->request); print("\n");
  print("- param1 "); puth(req->param1); print("\n");
  print("- param2 "); puth(req->param2); print("\n");
#endif

  switch (req->request) {
    // **** 0xa8: get microsecond timer
    case 0xa8:
      time = microsecond_timer_get();
      resp[0] = (time & 0x000000FFU);
      resp[1] = ((time & 0x0000FF00U) >> 8U);
      resp[2] = ((time & 0x00FF0000U) >> 16U);
      resp[3] = ((time & 0xFF000000U) >> 24U);
      resp_len = 4U;
      break;
    // **** 0xb0: set IR power
    case 0xb0:
      current_board->set_ir_power(req->param1);
      break;
    // **** 0xb1: set fan power
    case 0xb1:
      fan_set_power(req->param1);
      break;
    // **** 0xb2: get fan rpm
    case 0xb2:
      resp[0] = (fan_state.rpm & 0x00FFU);
      resp[1] = ((fan_state.rpm & 0xFF00U) >> 8U);
      resp_len = 2;
      break;
    // **** 0xc0: reset communications
    case 0xc0:
      comms_can_reset();
      break;
    // **** 0xc1: get hardware type
    case 0xc1:
      resp[0] = hw_type;
      resp_len = 1;
      break;
    // **** 0xc2: CAN health stats
    case 0xc2:
      COMPILE_TIME_ASSERT(sizeof(can_health_t) <= USBPACKET_MAX_SIZE);
      if (req->param1 < 3U) {
        update_can_health_pkt(req->param1, 0U);
        can_health[req->param1].can_speed = (bus_config[req->param1].can_speed / 10U);
        can_health[req->param1].can_data_speed = (bus_config[req->param1].can_data_speed / 10U);
        can_health[req->param1].canfd_enabled = bus_config[req->param1].canfd_enabled;
        can_health[req->param1].brs_enabled = bus_config[req->param1].brs_enabled;
        can_health[req->param1].canfd_non_iso = bus_config[req->param1].canfd_non_iso;
        resp_len = sizeof(can_health[req->param1]);
        (void)memcpy(resp, (uint8_t*)(&can_health[req->param1]), resp_len);
      }
      break;
    // **** 0xc3: fetch MCU UID
    case 0xc3:
      (void)memcpy(resp, ((uint8_t *)UID_BASE), 12);
      resp_len = 12;
      break;
    // **** 0xc4: get interrupt call rate
    case 0xc4:
      if (req->param1 < NUM_INTERRUPTS) {
        uint32_t load = interrupts[req->param1].call_rate;
        resp[0] = (load & 0x000000FFU);
        resp[1] = ((load & 0x0000FF00U) >> 8U);
        resp[2] = ((load & 0x00FF0000U) >> 16U);
        resp[3] = ((load & 0xFF000000U) >> 24U);
        resp_len = 4U;
      }
      break;
    // **** 0xc5: DEBUG: drive relay
    case 0xc5:
      set_intercept_relay((req->param1 & 0x1U), (req->param1 & 0x2U));
      break;
    // **** 0xc6: DEBUG: read SOM GPIO
    case 0xc6:
      resp[0] = current_board->read_som_gpio();
      resp_len = 1;
      break;
    // **** 0xc7: read diagnostic page
    case 0xc7:
      resp_len = get_diagnostics_pkt(req->param1, resp);
      break;
    // **** 0xc8: clear diagnostic-only counters and retained snapshots while offroad
    case 0xc8:
      resp[0] = ((req->param1 == PANDA_DIAGNOSTICS_VERSION) &&
                 (req->param2 == PANDA_DIAGNOSTICS_RESET_MAGIC) && clear_diagnostics()) ? 1U : 0U;
      resp_len = 1U;
      break;
    // **** 0xd0: fetch serial (aka the provisioned dongle ID)
    case 0xd0:
      // addresses are OTP
      if (req->param1 == 1U) {
        (void)memcpy(resp, (uint8_t *)DEVICE_SERIAL_NUMBER_ADDRESS, 0x10);
        resp_len = 0x10;
      } else {
        get_provision_chunk(resp);
        resp_len = PROVISION_CHUNK_LEN;
      }
      break;
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      switch (req->param1) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            print("-> entering bootloader\n");
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          #endif
          break;
        case 1:
          print("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          print("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd2: get health packet
    case 0xd2:
      resp_len = get_health_pkt(resp);
      break;
    // **** 0xd3: get first 64 bytes of signature
    case 0xd3:
      {
        resp_len = 64;
        char * code = (char*)_app_start;
        int code_len = _app_start[0];
        (void)memcpy(resp, &code[code_len], resp_len);
      }
      break;
    // **** 0xd4: get second 64 bytes of signature
    case 0xd4:
      {
        resp_len = 64;
        char * code = (char*)_app_start;
        int code_len = _app_start[0];
        (void)memcpy(resp, &code[code_len + 64], resp_len);
      }
      break;
    // **** 0xd6: get version
    case 0xd6:
      COMPILE_TIME_ASSERT(sizeof(gitversion) <= USBPACKET_MAX_SIZE);
      (void)memcpy(resp, gitversion, sizeof(gitversion));
      resp_len = sizeof(gitversion) - 1U;
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xdb: set OBD CAN multiplexing mode
    case 0xdb:
      if (current_board->harness_config->has_harness) {
        if (req->param1 == 1U) {
          // Enable OBD CAN
          current_board->set_can_mode(CAN_MODE_OBD_CAN2);
        } else {
          // Disable OBD CAN
          current_board->set_can_mode(CAN_MODE_NORMAL);
        }
      }
      break;

    // **** 0xdc: set safety mode
    case 0xdc:
      set_safety_mode(req->param1, (uint16_t)req->param2);
      break;
    // **** 0xdd: get healthpacket and CANPacket versions
    case 0xdd:
      resp[0] = HEALTH_PACKET_VERSION;
      resp[1] = CAN_PACKET_VERSION;
      resp[2] = CAN_HEALTH_PACKET_VERSION;
      resp_len = 3;
      break;
    // **** 0xde: set can bitrate
    case 0xde:
      if ((req->param1 < PANDA_BUS_CNT) && is_speed_valid(req->param2, speeds, sizeof(speeds)/sizeof(speeds[0]))) {
        bus_config[req->param1].can_speed = req->param2;
        bool ret = can_init(CAN_NUM_FROM_BUS_NUM(req->param1));
        UNUSED(ret);
      }
      break;
    // **** 0xdf: set alternative experience
    case 0xdf:
      // you can only set this if you are in a non car safety mode
      if (!is_car_safety_mode(current_safety_mode)) {
        alternative_experience = req->param1;
      }
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(req->param1);
      if (!ur) {
        break;
      }

      // read
      uint16_t req_length = MIN(req->length, USBPACKET_MAX_SIZE);
      while ((resp_len < req_length) &&
                         get_char(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xe1: uart set baud rate
    case 0xe1:
      ur = get_ring_by_number(req->param1);
      if (!ur) {
        break;
      }
      uart_set_baud(ur->uart, req->param2);
      break;
    // **** 0xe2: uart set parity
    case 0xe2:
      ur = get_ring_by_number(req->param1);
      if (!ur) {
        break;
      }
      switch (req->param2) {
        case 0:
          // disable parity, 8-bit
          ur->uart->CR1 &= ~(USART_CR1_PCE | USART_CR1_M);
          break;
        case 1:
          // even parity, 9-bit
          ur->uart->CR1 &= ~USART_CR1_PS;
          ur->uart->CR1 |= USART_CR1_PCE | USART_CR1_M;
          break;
        case 2:
          // odd parity, 9-bit
          ur->uart->CR1 |= USART_CR1_PS;
          ur->uart->CR1 |= USART_CR1_PCE | USART_CR1_M;
          break;
        default:
          break;
      }
      break;
    // **** 0xe4: uart set baud rate extended
    case 0xe4:
      ur = get_ring_by_number(req->param1);
      if (!ur) {
        break;
      }
      uart_set_baud(ur->uart, (int)req->param2*300);
      break;
    // **** 0xe5: set CAN loopback (for testing)
    case 0xe5:
      can_loopback = req->param1 > 0U;
      can_init_all();
      break;
    // **** 0xe6: set custom clock source period
    case 0xe6:
      clock_source_set_period(req->param1);
      break;
    // **** 0xe7: set power save state
    case 0xe7:
      set_power_save_state(req->param1);
      break;
    // **** 0xe8: set can-fd auto swithing mode
    case 0xe8:
      bus_config[req->param1].canfd_auto = req->param2 > 0U;
      break;
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (req->param1 == 0xFFFFU) {
        print("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (req->param1 < PANDA_BUS_CNT) {
        print("Clearing CAN Tx queue\n");
        can_clear(can_queues[req->param1]);
      } else {
        print("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2:
      {
        uart_ring * rb = get_ring_by_number(req->param1);
        if (rb != NULL) {
          print("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    // **** 0xf3: Heartbeat. Resets heartbeat counter.
    case 0xf3:
      {
        heartbeat_counter = 0U;
        heartbeat_lost = false;
        heartbeat_disabled = false;
        heartbeat_engaged = (req->param1 == 1U);
        break;
      }
    // **** 0xf6: set siren enabled
    case 0xf6:
      siren_enabled = (req->param1 != 0U);
      break;
    // **** 0xf7: set green led enabled
    case 0xf7:
      green_led_enabled = (req->param1 != 0U);
      break;
    // **** 0xf8: disable heartbeat checks
    case 0xf8:
      if (!is_car_safety_mode(current_safety_mode)) {
        heartbeat_disabled = true;
      }
      break;
    // **** 0xf9: set CAN FD data bitrate
    case 0xf9:
      if ((req->param1 < PANDA_CAN_CNT) &&
           current_board->has_canfd &&
           is_speed_valid(req->param2, data_speeds, sizeof(data_speeds)/sizeof(data_speeds[0]))) {
        bus_config[req->param1].can_data_speed = req->param2;
        bus_config[req->param1].canfd_enabled = (req->param2 >= bus_config[req->param1].can_speed);
        bus_config[req->param1].brs_enabled = (req->param2 > bus_config[req->param1].can_speed);
        bool ret = can_init(CAN_NUM_FROM_BUS_NUM(req->param1));
        UNUSED(ret);
      }
      break;
    // **** 0xfc: set CAN FD non-ISO mode
    case 0xfc:
      if ((req->param1 < PANDA_CAN_CNT) && current_board->has_canfd) {
        bus_config[req->param1].canfd_non_iso = (req->param2 != 0U);
        bool ret = can_init(CAN_NUM_FROM_BUS_NUM(req->param1));
        UNUSED(ret);
      }
      break;
    default:
      print("NO HANDLER ");
      puth(req->request);
      print("\n");
      break;
  }
  return resp_len;
}
