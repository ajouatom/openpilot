#include "selfdrive/pandad/pandad.h"

#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <cerrno>
#include <memory>
#include <thread>
#include <utility>

#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/messaging/messaging.h"
#include "cereal/services.h"
#include "common/ratekeeper.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"
#include "system/hardware/hw.h"

#define MAX_IR_PANDA_VAL 50
#define CUTOFF_IL 400
#define SATURATE_IL 1000

ExitHandler do_exit;

bool check_all_connected(const std::vector<Panda *> &pandas) {
  for (Panda *panda : pandas) {
    if (!panda->connected()) {
      do_exit = true;
      return false;
    }
  }
  return true;
}

Panda *connect(std::string serial, uint32_t index) {
  std::unique_ptr<Panda> panda;
  try {
    panda = std::make_unique<Panda>(serial, index * PANDA_BUS_OFFSET);
  } catch (std::exception &e) {
    return nullptr;
  }

  // common panda config
  if (getenv("BOARDD_LOOPBACK")) {
    panda->set_loopback(true);
  }
  //panda->enable_deepsleep();

  for (int i = 0; i < PANDA_CAN_CNT; i++) {
    panda->set_can_fd_auto(i, true);
  }

  if (!panda->up_to_date() && !getenv("BOARDD_SKIP_FW_CHECK")) {
    throw std::runtime_error("Panda firmware out of date. Run pandad.py to update.");
  }

  return panda.release();
}

void can_send_thread(std::vector<Panda *> pandas, bool fake_send) {
  util::set_thread_name("pandad_can_send");

  AlignedBuffer aligned_buf;
  std::unique_ptr<Context> context(Context::create());
  std::unique_ptr<SubSocket> subscriber(SubSocket::create(context.get(), "sendcan", "127.0.0.1", false, true, services.at("sendcan").queue_size));
  assert(subscriber != NULL);
  subscriber->setTimeout(100);

  // run as fast as messages come in
  while (!do_exit && check_all_connected(pandas)) {
    std::unique_ptr<Message> msg(subscriber->receive());
    if (!msg) {
      continue;
    }

    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(msg.get()));
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

    // Don't send if older than 1 second
    if ((nanos_since_boot() - event.getLogMonoTime() < 1e9) && !fake_send) {
      for (Panda *panda : pandas) {
        LOGT("sending sendcan to panda: %s", (panda->hw_serial()).c_str());
        panda->can_send(event.getSendcan());
        LOGT("sendcan sent to panda: %s", (panda->hw_serial()).c_str());
      }
    } else {
      LOGE("sendcan too old to send: %" PRIu64 ", %" PRIu64, nanos_since_boot(), event.getLogMonoTime());
    }
  }
}

void can_recv(const std::vector<Panda *> &pandas, PubMaster *pm) {
  static std::vector<can_frame> raw_can_data;
  {
    raw_can_data.clear();
    bool comms_healthy = true;
    for (Panda *panda : pandas) {
      comms_healthy &= panda->can_receive(raw_can_data);
    }

    MessageBuilder msg;
    auto evt = msg.initEvent();
    evt.setValid(comms_healthy);
    auto canData = evt.initCan(raw_can_data.size());
    for (size_t i = 0; i < raw_can_data.size(); ++i) {
      canData[i].setAddress(raw_can_data[i].address);
      canData[i].setDat(kj::arrayPtr((uint8_t*)raw_can_data[i].dat.data(), raw_can_data[i].dat.size()));
      canData[i].setSrc(raw_can_data[i].src);
    }
    pm->send("can", msg);
  }
}

void fill_panda_state(cereal::PandaState::Builder &ps, cereal::PandaState::PandaType hw_type, const health_t &health) {
  ps.setVoltage(health.voltage_pkt);
  ps.setCurrent(health.current_pkt);
  ps.setUptime(health.uptime_pkt);
  ps.setSafetyTxBlocked(health.safety_tx_blocked_pkt);
  ps.setSafetyRxInvalid(health.safety_rx_invalid_pkt);
  ps.setIgnitionLine(health.ignition_line_pkt);
  ps.setIgnitionCan(health.ignition_can_pkt);
  ps.setControlsAllowed(health.controls_allowed_pkt);
  ps.setTxBufferOverflow(health.tx_buffer_overflow_pkt);
  ps.setRxBufferOverflow(health.rx_buffer_overflow_pkt);
  ps.setPandaType(hw_type);
  ps.setSafetyModel(cereal::CarParams::SafetyModel(health.safety_mode_pkt));
  ps.setSafetyParam(health.safety_param_pkt);
  ps.setFaultStatus(cereal::PandaState::FaultStatus(health.fault_status_pkt));
  ps.setPowerSaveEnabled((bool)(health.power_save_enabled_pkt));
  ps.setHeartbeatLost((bool)(health.heartbeat_lost_pkt));
  ps.setAlternativeExperience(health.alternative_experience_pkt);
  ps.setHarnessStatus(cereal::PandaState::HarnessStatus(health.car_harness_status_pkt));
  ps.setInterruptLoad(health.interrupt_load_pkt);
  ps.setFanPower(health.fan_power);
  ps.setFanStallCount(health.fan_stall_count);
  ps.setSafetyRxChecksInvalid((bool)(health.safety_rx_checks_invalid_pkt));
  ps.setSpiChecksumErrorCount(health.spi_checksum_error_count_pkt);
  ps.setSbu1Voltage(health.sbu1_voltage_mV / 1000.0f);
  ps.setSbu2Voltage(health.sbu2_voltage_mV / 1000.0f);
}

void fill_panda_can_state(cereal::PandaState::PandaCanState::Builder &cs, const can_health_t &can_health) {
  cs.setBusOff((bool)can_health.bus_off);
  cs.setBusOffCnt(can_health.bus_off_cnt);
  cs.setErrorWarning((bool)can_health.error_warning);
  cs.setErrorPassive((bool)can_health.error_passive);
  cs.setLastError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_error));
  cs.setLastStoredError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_stored_error));
  cs.setLastDataError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_data_error));
  cs.setLastDataStoredError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_data_stored_error));
  cs.setReceiveErrorCnt(can_health.receive_error_cnt);
  cs.setTransmitErrorCnt(can_health.transmit_error_cnt);
  cs.setTotalErrorCnt(can_health.total_error_cnt);
  cs.setTotalTxLostCnt(can_health.total_tx_lost_cnt);
  cs.setTotalRxLostCnt(can_health.total_rx_lost_cnt);
  cs.setTotalTxCnt(can_health.total_tx_cnt);
  cs.setTotalRxCnt(can_health.total_rx_cnt);
  cs.setTotalFwdCnt(can_health.total_fwd_cnt);
  cs.setCanSpeed(can_health.can_speed);
  cs.setCanDataSpeed(can_health.can_data_speed);
  cs.setCanfdEnabled(can_health.canfd_enabled);
  cs.setBrsEnabled(can_health.brs_enabled);
  cs.setCanfdNonIso(can_health.canfd_non_iso);
  cs.setIrq0CallRate(can_health.irq0_call_rate);
  cs.setIrq1CallRate(can_health.irq1_call_rate);
  cs.setIrq2CallRate(can_health.irq2_call_rate);
  cs.setCanCoreResetCnt(can_health.can_core_reset_cnt);
}

template <typename T>
std::optional<T> read_panda_diagnostic(Panda *panda, uint16_t page) {
  T diag{};
  if (!panda->get_diagnostics(page, &diag, sizeof(diag)) ||
      (diag.version != PANDA_DIAGNOSTICS_VERSION) || (diag.page != page)) {
    return std::nullopt;
  }
  return diag;
}

void log_panda_fast_diagnostics(Panda *panda) {
  const std::string serial = panda->hw_serial();
  if (auto diag = read_panda_diagnostic<panda_spi_diag_t>(panda, PANDA_DIAGNOSTICS_PAGE_SPI)) {
    LOGW("Panda SPI diagnostic: serial=%s state=%u ready=%u header_sync_nack=%u header_checksum_nack=%u data_checksum_nack=%u "
         "ep3_checksum_nack=%u ep3_backpressure_nack=%u ep3_ack=%u "
         "last_ep3_checksum=%u@%u last_ep3_backpressure=%u@%u tx_free=%u/%u/%u "
         "last_backpressure_free=%u/%u/%u tx_overflow_bus=%u/%u/%u",
         serial.c_str(), (unsigned)diag->spi_state, (unsigned)diag->can_tx_ready,
         diag->header_sync_nack_cnt, diag->header_checksum_nack_cnt, diag->data_checksum_nack_cnt,
         diag->endpoint3_checksum_nack_cnt,
         diag->endpoint3_backpressure_nack_cnt, diag->endpoint3_ack_cnt,
         (unsigned)diag->last_endpoint3_checksum_len, diag->last_endpoint3_checksum_time_us,
         (unsigned)diag->last_endpoint3_backpressure_len, diag->last_endpoint3_backpressure_time_us,
         (unsigned)diag->current_tx_free[0], (unsigned)diag->current_tx_free[1],
         (unsigned)diag->current_tx_free[2], (unsigned)diag->last_backpressure_tx_free[0],
         (unsigned)diag->last_backpressure_tx_free[1], (unsigned)diag->last_backpressure_tx_free[2],
         diag->tx_overflow_by_bus[0], diag->tx_overflow_by_bus[1], diag->tx_overflow_by_bus[2]);
  } else {
    LOGW("Panda SPI diagnostic page unavailable: serial=%s", serial.c_str());
  }

  for (uint16_t can_number = 0U; can_number < PANDA_CAN_CNT; can_number++) {
    uint16_t page = PANDA_DIAGNOSTICS_PAGE_CAN_HIST_BASE + can_number;
    if (auto diag = read_panda_diagnostic<panda_can_error_hist_t>(panda, page)) {
      LOGW("Panda CAN IRQ diagnostic: serial=%s bus=%u can=%u "
           "nominal(stuff/form/ack/bit1/bit0/crc)=%u/%u/%u/%u/%u/%u "
           "data(stuff/form/ack/bit1/bit0/crc)=%u/%u/%u/%u/%u/%u warning=%u passive=%u bus_off=%u",
           serial.c_str(), panda->bus_offset + diag->bus_number, (unsigned)diag->can_number,
           diag->nominal_error_cnt[0], diag->nominal_error_cnt[1], diag->nominal_error_cnt[2],
           diag->nominal_error_cnt[3], diag->nominal_error_cnt[4], diag->nominal_error_cnt[5],
           diag->data_error_cnt[0], diag->data_error_cnt[1], diag->data_error_cnt[2],
           diag->data_error_cnt[3], diag->data_error_cnt[4], diag->data_error_cnt[5],
           diag->error_warning_irq_cnt, diag->error_passive_irq_cnt, diag->bus_off_irq_cnt);
    } else {
      LOGW("Panda CAN IRQ diagnostic page unavailable: serial=%s can=%u", serial.c_str(), can_number);
    }
  }
}

void log_panda_deep_diagnostics(Panda *panda) {
  const std::string serial = panda->hw_serial();
  const char *stage_names[PANDA_CAN_ERROR_STAGE_COUNT] = {"first", "warning", "passive", "bus_off"};
  for (uint16_t can_number = 0U; can_number < PANDA_CAN_CNT; can_number++) {
    for (uint16_t stage = 0U; stage < PANDA_CAN_ERROR_STAGE_COUNT; stage++) {
      uint16_t page = PANDA_DIAGNOSTICS_PAGE_CAN_SNAPSHOT_BASE +
                      (can_number * PANDA_CAN_ERROR_STAGE_COUNT) + stage;
      auto diag = read_panda_diagnostic<panda_can_error_diag_t>(panda, page);
      if (diag && (diag->valid != 0U)) {
        LOGW("Panda CAN first-state: serial=%s stage=%s bus=%u can=%u fw_time=%u safety=%u/%u "
             "ir=0x%08x psr=0x%08x ecr=0x%08x txfqs=0x%08x rxf0s=0x%08x "
             "last_tx=0x%x@%u last_rx=0x%x@%u last_host=0x%x@%u last_fwd=0x%x@%u",
             serial.c_str(), stage_names[stage], panda->bus_offset + diag->bus_number,
             (unsigned)diag->can_number, diag->timestamp_us, (unsigned)diag->safety_mode,
             (unsigned)diag->safety_param, diag->ir_reg, diag->psr_reg, diag->ecr_reg,
             diag->txfqs_reg, diag->rxf0s_reg, diag->last_tx_addr, diag->last_tx_time_us,
             diag->last_rx_addr, diag->last_rx_time_us, diag->last_host_addr,
             diag->last_host_time_us, diag->last_fwd_addr, diag->last_fwd_time_us);
      }
    }
  }

  if (auto diag = read_panda_diagnostic<panda_hyundai_fwd_diag_t>(panda, PANDA_DIAGNOSTICS_PAGE_HYUNDAI_SUMMARY)) {
    LOGW("Panda Hyundai fwd diagnostic: serial=%s enabled=%u safety=%u/%u buffered=%u calls=%u from=0:%u/2:%u "
         "replace_to=0:%u/2:%u pass_to=0:%u/2:%u dynamic_block_to=0:%u/2:%u "
         "empty_to=0:%u/2:%u block_4b9=%u invalid_bus=%u",
         serial.c_str(), (unsigned)diag->enabled, (unsigned)diag->safety_mode,
         (unsigned)diag->safety_param, diag->tx_buffered_cnt, diag->fwd_call_cnt,
         diag->fwd_from_bus0_cnt, diag->fwd_from_bus2_cnt, diag->replace_to_bus0_cnt,
         diag->replace_to_bus2_cnt, diag->pass_to_bus0_cnt, diag->pass_to_bus2_cnt,
         diag->dynamic_block_to_bus0_cnt, diag->dynamic_block_to_bus2_cnt,
         diag->empty_to_bus0_cnt, diag->empty_to_bus2_cnt, diag->block_4b9_cnt,
         diag->invalid_bus_cnt);
  } else {
    LOGW("Panda Hyundai forwarding diagnostic page unavailable: serial=%s", serial.c_str());
  }

  for (uint16_t index = 0U; index < PANDA_DIAGNOSTICS_HYUNDAI_BUFFER_COUNT; index++) {
    uint16_t page = PANDA_DIAGNOSTICS_PAGE_HYUNDAI_BUFFER_BASE + index;
    if (auto diag = read_panda_diagnostic<panda_hyundai_buffer_diag_t>(panda, page)) {
      LOGW("Panda Hyundai buffer diagnostic: serial=%s index=%u addr=0x%x dst=%u enabled=%u "
           "queue=%u started=%u reuse_left=%u has_last=%u push=%u/%u full_drop=%u "
           "pop=%u/%u reuse=%u/%u empty=%u resets=%u times=%u/%u/%u/%u",
           serial.c_str(), (unsigned)diag->index, (unsigned)diag->addr, (unsigned)diag->dst_bus,
           (unsigned)diag->enabled, (unsigned)diag->count, (unsigned)diag->started,
           (unsigned)diag->reuse_left, (unsigned)diag->has_last_pkt, diag->push_attempt_cnt,
           diag->push_accepted_cnt, diag->push_full_drop_cnt, diag->pop_attempt_cnt,
           diag->pop_success_cnt, diag->reuse_attempt_cnt, diag->reuse_success_cnt,
           diag->empty_fallback_cnt, diag->reset_cnt, diag->last_push_time_us,
           diag->last_pop_time_us, diag->last_reuse_time_us, diag->last_empty_time_us);
    } else {
      LOGW("Panda Hyundai buffer diagnostic page unavailable: serial=%s index=%u", serial.c_str(), index);
    }
  }
}

std::optional<bool> send_panda_states(PubMaster *pm, const std::vector<Panda *> &pandas, bool is_onroad, bool spoofing_started) {
  bool ignition_local = false;
  const uint32_t pandas_cnt = pandas.size();

  // build msg
  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto pss = evt.initPandaStates(pandas_cnt);

  std::vector<health_t> panda_states;
  panda_states.reserve(pandas_cnt);

  std::vector<std::array<can_health_t, PANDA_CAN_CNT>> panda_can_states;
  panda_can_states.reserve(pandas_cnt);

  struct CanTxDiagnosticState {
    bool initialized = false;
    bool pre_drive_reset_pending = true;
    bool was_onroad = false;
    uint8_t onroad_checkpoint_index = 0U;
    std::string serial;
    uint64_t onroad_start_ns = 0U;
    uint64_t last_reset_attempt_ns = 0U;
    uint64_t last_log_ns = 0;
    uint64_t last_detail_ns = 0;
    uint64_t drops = 0;
    health_t health{};
    std::array<can_health_t, PANDA_CAN_CNT> can{};
  };
  static std::vector<CanTxDiagnosticState> diagnostic_states;
  diagnostic_states.resize(pandas_cnt);

  const bool red_panda_comma_three = (pandas.size() == 2) &&
                                     (pandas[0]->hw_type == cereal::PandaState::PandaType::DOS) &&
                                     (pandas[1]->hw_type == cereal::PandaState::PandaType::RED_PANDA);

  for (Panda *panda : pandas) {
    auto health_opt = panda->get_state();
    if (!health_opt) {
      return std::nullopt;
    }

    health_t health = *health_opt;

    std::array<can_health_t, PANDA_CAN_CNT> can_health{};
    for (uint32_t i = 0; i < PANDA_CAN_CNT; i++) {
      auto can_health_opt = panda->get_can_state(i);
      if (!can_health_opt) {
        return std::nullopt;
      }
      can_health[i] = *can_health_opt;
    }
    panda_can_states.push_back(can_health);

    if (spoofing_started) {
      health.ignition_line_pkt = 1;
    }

    // A C3's DOS can report false ignition from the harness box. The red panda
    // is the vehicle-facing ignition source in this two-panda configuration.
    if (red_panda_comma_three && (panda->hw_type == cereal::PandaState::PandaType::DOS)) {
      health.ignition_line_pkt = 0;
    }

    ignition_local |= ((health.ignition_line_pkt != 0) || (health.ignition_can_pkt != 0));
    panda_states.push_back(health);
  }

  for (uint32_t i = 0; i < pandas_cnt; i++) {
    Panda *panda = pandas[i];
    const health_t &health = panda_states[i];
    CanTxDiagnosticState &diagnostic = diagnostic_states[i];
    const uint64_t now = nanos_since_boot();
    const uint64_t drops = panda->can_tx_drop_count();
    const std::string serial = panda->hw_serial();
    if (diagnostic.serial != serial) {
      diagnostic = CanTxDiagnosticState{};
      diagnostic.serial = serial;
    }

    const bool onroad_started = is_onroad && !diagnostic.was_onroad;
    const bool onroad_stopped = !is_onroad && diagnostic.was_onroad;
    if (onroad_started) {
      diagnostic.onroad_start_ns = now;
      diagnostic.onroad_checkpoint_index = 0U;
    } else if (onroad_stopped) {
      diagnostic.onroad_start_ns = 0U;
      diagnostic.onroad_checkpoint_index = 0U;
      diagnostic.pre_drive_reset_pending = true;
      diagnostic.last_reset_attempt_ns = 0U;
    }

    const bool reset_rate_ready = (diagnostic.last_reset_attempt_ns == 0U) ||
                                  ((now - diagnostic.last_reset_attempt_ns) >= 1'000'000'000ULL);
    if (!is_onroad && !ignition_local && diagnostic.pre_drive_reset_pending && reset_rate_ready) {
      diagnostic.last_reset_attempt_ns = now;
      const bool accepted = panda->clear_diagnostics();
      LOGW("Panda diagnostic pre-drive reset: serial=%s accepted=%u uptime=%u safety=%u/%u",
           serial.c_str(), (unsigned)accepted, health.uptime_pkt, (unsigned)health.safety_mode_pkt,
           (unsigned)health.safety_param_pkt);
      if (accepted) {
        diagnostic.pre_drive_reset_pending = false;
        diagnostic.initialized = false;
      }
    }

    static constexpr std::array<uint64_t, 2> ONROAD_DIAGNOSTIC_CHECKPOINT_NS = {
      5'000'000'000ULL, 15'000'000'000ULL,
    };
    const uint64_t onroad_elapsed_ns = (is_onroad && (diagnostic.onroad_start_ns != 0U)) ?
                                       now - diagnostic.onroad_start_ns : 0U;
    const bool onroad_checkpoint_due = is_onroad &&
                                       (diagnostic.onroad_checkpoint_index < ONROAD_DIAGNOSTIC_CHECKPOINT_NS.size()) &&
                                       (onroad_elapsed_ns >= ONROAD_DIAGNOSTIC_CHECKPOINT_NS[diagnostic.onroad_checkpoint_index]);

    bool diagnostic_changed = !diagnostic.initialized ||
                              (drops != diagnostic.drops) ||
                              (health.spi_checksum_error_count_pkt != diagnostic.health.spi_checksum_error_count_pkt) ||
                              (health.tx_buffer_overflow_pkt != diagnostic.health.tx_buffer_overflow_pkt) ||
                              (health.rx_buffer_overflow_pkt != diagnostic.health.rx_buffer_overflow_pkt) ||
                              (health.safety_mode_pkt != diagnostic.health.safety_mode_pkt) ||
                              (health.safety_param_pkt != diagnostic.health.safety_param_pkt) ||
                              (health.faults_pkt != diagnostic.health.faults_pkt);
    bool critical_transition = !diagnostic.initialized ||
                               ((diagnostic.drops == 0U) && (drops != 0U)) ||
                               ((diagnostic.health.tx_buffer_overflow_pkt == 0U) && (health.tx_buffer_overflow_pkt != 0U)) ||
                               ((diagnostic.health.rx_buffer_overflow_pkt == 0U) && (health.rx_buffer_overflow_pkt != 0U));
    const bool safety_changed = !diagnostic.initialized ||
                                (health.safety_mode_pkt != diagnostic.health.safety_mode_pkt) ||
                                (health.safety_param_pkt != diagnostic.health.safety_param_pkt);
    bool diagnostic_active = (drops != 0U) || (health.spi_checksum_error_count_pkt != 0U) ||
                             (health.tx_buffer_overflow_pkt != 0U) || (health.rx_buffer_overflow_pkt != 0U);

    for (uint32_t j = 0; j < PANDA_CAN_CNT; j++) {
      const can_health_t &can = panda_can_states[i][j];
      const can_health_t &previous = diagnostic.can[j];
      diagnostic_changed |= (can.bus_off != previous.bus_off) ||
                            (can.bus_off_cnt != previous.bus_off_cnt) ||
                            (can.error_warning != previous.error_warning) ||
                            (can.error_passive != previous.error_passive) ||
                            (can.last_stored_error != previous.last_stored_error) ||
                            (can.last_data_stored_error != previous.last_data_stored_error) ||
                            (can.receive_error_cnt != previous.receive_error_cnt) ||
                            (can.transmit_error_cnt != previous.transmit_error_cnt) ||
                            (can.total_error_cnt != previous.total_error_cnt) ||
                            (can.total_tx_lost_cnt != previous.total_tx_lost_cnt) ||
                            (can.total_rx_lost_cnt != previous.total_rx_lost_cnt) ||
                            (can.total_tx_checksum_error_cnt != previous.total_tx_checksum_error_cnt) ||
                            (can.can_core_reset_cnt != previous.can_core_reset_cnt);
      critical_transition |= ((previous.total_error_cnt == 0U) && (can.total_error_cnt != 0U)) ||
                             ((previous.can_core_reset_cnt == 0U) && (can.can_core_reset_cnt != 0U)) ||
                             ((previous.bus_off == 0U) && (can.bus_off != 0U)) ||
                             ((previous.error_warning == 0U) && (can.error_warning != 0U)) ||
                             ((previous.error_passive == 0U) && (can.error_passive != 0U));
      diagnostic_active |= (can.bus_off != 0U) || (can.bus_off_cnt != 0U) ||
                           (can.error_warning != 0U) || (can.error_passive != 0U) ||
                           (can.total_error_cnt != 0U) || (can.total_tx_lost_cnt != 0U) ||
                           (can.total_rx_lost_cnt != 0U) || (can.total_tx_checksum_error_cnt != 0U) ||
                           (can.can_core_reset_cnt != 0U);
    }

    const bool log_rate_ready = !diagnostic.initialized ||
                                ((now - diagnostic.last_log_ns) >= 1'000'000'000ULL);
    const bool periodic_due = diagnostic_active &&
                              (!diagnostic.initialized || ((now - diagnostic.last_log_ns) >= 10'000'000'000ULL));
    const bool log_can_health = onroad_checkpoint_due || (log_rate_ready && (diagnostic_changed || periodic_due));
    const bool log_deep_diagnostics = log_can_health &&
                                      (onroad_checkpoint_due || !diagnostic.initialized || safety_changed || critical_transition ||
                                       ((now - diagnostic.last_detail_ns) >= 10'000'000'000ULL));
    const uint64_t drops_delta = diagnostic.initialized ? drops - diagnostic.drops : 0U;
    if (onroad_checkpoint_due) {
      LOGW("Panda diagnostic route checkpoint: serial=%s checkpoint=%u/2 elapsed_ms=%" PRIu64 " pre_drive_reset=%u",
           serial.c_str(), (unsigned)diagnostic.onroad_checkpoint_index + 1U,
           static_cast<uint64_t>(onroad_elapsed_ns / 1'000'000ULL),
           (unsigned)!diagnostic.pre_drive_reset_pending);
    }
    if (log_can_health) {
      LOGW("Panda CAN TX diagnostic: index=%u/%u serial=%s type=%d buses=%u-%u drops=%" PRIu64 "(+%" PRIu64 ") "
           "uptime=%u safety=%u/%u spi_checksum=%u tx_overflow=%u rx_overflow=%u faults=0x%x",
           i, pandas_cnt, serial.c_str(), (int)panda->hw_type, panda->bus_offset,
           panda->bus_offset + PANDA_BUS_OFFSET - 1, drops, drops_delta,
           health.uptime_pkt, (unsigned)health.safety_mode_pkt, (unsigned)health.safety_param_pkt,
           (unsigned)health.spi_checksum_error_count_pkt, health.tx_buffer_overflow_pkt,
           health.rx_buffer_overflow_pkt, health.faults_pkt);
    }

    // Make sure CAN buses are live: safety_setter_thread does not work if Panda CAN are silent and there is only one other CAN node
    if (health.safety_mode_pkt == (uint8_t)(cereal::CarParams::SafetyModel::SILENT)) {
      panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
    }

    bool power_save_desired = !ignition_local;
    if (health.power_save_enabled_pkt != power_save_desired) {
      panda->set_power_saving(power_save_desired);
    }

    // Set safety mode to NO_OUTPUT when the car is off or we're not onroad.
    bool should_close_relay = !ignition_local || !is_onroad;
    if (should_close_relay && (health.safety_mode_pkt != (uint8_t)(cereal::CarParams::SafetyModel::NO_OUTPUT))) {
      panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
    }

    if (!panda->comms_healthy()) {
      evt.setValid(false);
    }

    auto ps = pss[i];
    fill_panda_state(ps, panda->hw_type, health);

    auto cs = std::array{ps.initCanState0(), ps.initCanState1(), ps.initCanState2()};
    for (uint32_t j = 0; j < PANDA_CAN_CNT; j++) {
      const can_health_t &can = panda_can_states[i][j];
      fill_panda_can_state(cs[j], can);
      if (log_can_health) {
        LOGW("Panda CAN health: serial=%s bus=%u off=%u off_cnt=%u warning=%u passive=%u "
             "lec=%u/%u dlec=%u/%u rec=%u tec=%u errors=%u tx_lost=%u tx_checksum=%u "
             "rx_lost=%u tx=%u rx=%u fwd=%u irq=%u/%u/%u reset=%u speed=%u/%u fd=%u brs=%u",
             serial.c_str(), panda->bus_offset + j, (unsigned)can.bus_off, can.bus_off_cnt,
             (unsigned)can.error_warning, (unsigned)can.error_passive, (unsigned)can.last_error,
             (unsigned)can.last_stored_error, (unsigned)can.last_data_error,
             (unsigned)can.last_data_stored_error, (unsigned)can.receive_error_cnt,
             (unsigned)can.transmit_error_cnt, can.total_error_cnt, can.total_tx_lost_cnt,
             can.total_tx_checksum_error_cnt, can.total_rx_lost_cnt, can.total_tx_cnt, can.total_rx_cnt,
             can.total_fwd_cnt, can.irq0_call_rate, can.irq1_call_rate, can.irq2_call_rate,
             can.can_core_reset_cnt,
             (unsigned)can.can_speed, (unsigned)can.can_data_speed,
             (unsigned)can.canfd_enabled, (unsigned)can.brs_enabled);
      }
    }
    if (log_can_health) {
      log_panda_fast_diagnostics(panda);
      if (log_deep_diagnostics) {
        log_panda_deep_diagnostics(panda);
        diagnostic.last_detail_ns = now;
      }
      diagnostic.initialized = true;
      diagnostic.last_log_ns = now;
      diagnostic.drops = drops;
      diagnostic.health = health;
      diagnostic.can = panda_can_states[i];
      if (onroad_checkpoint_due) {
        diagnostic.onroad_checkpoint_index += 1U;
      }
    }
    diagnostic.was_onroad = is_onroad;

    // Convert faults bitset to capnp list
    std::bitset<sizeof(health.faults_pkt) * 8> fault_bits(health.faults_pkt);
    auto faults = ps.initFaults(fault_bits.count());

    size_t j = 0;
    for (size_t f = size_t(cereal::PandaState::FaultType::RELAY_MALFUNCTION);
         f <= size_t(cereal::PandaState::FaultType::HEARTBEAT_LOOP_WATCHDOG); f++) {
      if (fault_bits.test(f)) {
        faults.set(j, cereal::PandaState::FaultType(f));
        j++;
      }
    }
  }

  pm->send("pandaStates", msg);
  return ignition_local;
}

void send_peripheral_state(Panda *panda, PubMaster *pm) {
  // build msg
  MessageBuilder msg;
  auto evt = msg.initEvent();
  evt.setValid(panda->comms_healthy());

  auto ps = evt.initPeripheralState();
  ps.setPandaType(panda->hw_type);

  double read_time = millis_since_boot();
  ps.setVoltage(Hardware::get_voltage());
  ps.setCurrent(Hardware::get_current());
  read_time = millis_since_boot() - read_time;
  if (read_time > 50) {
    LOGW("reading hwmon took %lfms", read_time);
  }

  // fall back to panda's voltage and current measurement
  if (ps.getVoltage() == 0 && ps.getCurrent() == 0) {
    auto health_opt = panda->get_state();
    if (health_opt) {
      health_t health = *health_opt;
      ps.setVoltage(health.voltage_pkt);
      ps.setCurrent(health.current_pkt);
    }
  }

  uint16_t fan_speed_rpm = panda->get_fan_speed();
  ps.setFanSpeedRpm(fan_speed_rpm);

  pm->send("peripheralState", msg);
}

void process_panda_state(const std::vector<Panda *> &pandas, PubMaster *pm, bool engaged, bool is_onroad, bool spoofing_started) {
  std::vector<std::string> connected_serials;
  connected_serials.reserve(pandas.size());
  for (Panda *panda : pandas) {
    connected_serials.push_back(panda->hw_serial());
  }

  auto ignition_opt = send_panda_states(pm, pandas, is_onroad, spoofing_started);
  if (!ignition_opt) {
    LOGE("Failed to get ignition_opt");
    return;
  }

  // check if we should have pandad reconnect
  if (!ignition_opt.value()) {
    bool comms_healthy = true;
    for (Panda *panda : pandas) {
      comms_healthy &= panda->comms_healthy();
    }

    if (!comms_healthy) {
      LOGE("Reconnecting, communication to pandas not healthy");
      do_exit = true;
    } else if (!is_onroad) {
      // A C3 red panda can enumerate after its internal DOS. Restart the
      // wrapper so it can sort and launch the complete set.
      for (const std::string &serial : Panda::list(true)) {
        if (!std::count(connected_serials.begin(), connected_serials.end(), serial)) {
          LOGW("Reconnecting to new panda: %s", serial.c_str());
          do_exit = true;
          break;
        }
      }
    }
  }

  for (Panda *panda : pandas) {
    panda->send_heartbeat(engaged);
  }
}

void process_peripheral_state(Panda *panda, PubMaster *pm, bool no_fan_control) {
  static Params params;
  static SubMaster sm({"deviceState", "driverCameraState"});

  static uint64_t last_driver_camera_t = 0;
  static uint16_t prev_fan_speed = 999;
  static int ir_pwr = 0;
  static int prev_ir_pwr = 999;
  static uint32_t prev_frame_id = UINT32_MAX;
  static bool driver_view = false;

  // TODO: can we merge these?
  static FirstOrderFilter integ_lines_filter(0, 30.0, 0.05);
  static FirstOrderFilter integ_lines_filter_driver_view(0, 5.0, 0.05);

  {
    sm.update(0);
    if (sm.updated("deviceState") && !no_fan_control) {
      // Fan speed
      uint16_t fan_speed = sm["deviceState"].getDeviceState().getFanSpeedPercentDesired();
      if (fan_speed != prev_fan_speed || sm.frame % 100 == 0) {
        panda->set_fan_speed(fan_speed);
        prev_fan_speed = fan_speed;
      }
    }

    if (sm.updated("driverCameraState")) {
      auto event = sm["driverCameraState"];
      int cur_integ_lines = event.getDriverCameraState().getIntegLines();

      // reset the filter when camerad restarts
      if (event.getDriverCameraState().getFrameId() < prev_frame_id) {
        integ_lines_filter.reset(0);
        integ_lines_filter_driver_view.reset(0);
        driver_view = params.getBool("IsDriverViewEnabled");
      }
      prev_frame_id = event.getDriverCameraState().getFrameId();

      cur_integ_lines = (driver_view ? integ_lines_filter_driver_view : integ_lines_filter).update(cur_integ_lines);
      last_driver_camera_t = event.getLogMonoTime();

      if (cur_integ_lines <= CUTOFF_IL) {
        ir_pwr = 0;
      } else if (cur_integ_lines > SATURATE_IL) {
        ir_pwr = 100;
      } else {
        ir_pwr = 100 * (cur_integ_lines - CUTOFF_IL) / (SATURATE_IL - CUTOFF_IL);
      }
    }

    // Disable IR on input timeout
    if (nanos_since_boot() - last_driver_camera_t > 1e9) {
      ir_pwr = 0;
    }

    if (ir_pwr != prev_ir_pwr || sm.frame % 100 == 0) {
      int16_t ir_panda = util::map_val(ir_pwr, 0, 100, 0, MAX_IR_PANDA_VAL); 
      panda->set_ir_pwr(ir_panda);
      Hardware::set_ir_power(ir_pwr); 
      prev_ir_pwr = ir_pwr;
    }
  }
}

void pandad_run(std::vector<Panda *> &pandas) {
  const bool no_fan_control = getenv("NO_FAN_CONTROL") != nullptr;
  const bool spoofing_started = getenv("STARTED") != nullptr;
  const bool fake_send = getenv("FAKESEND") != nullptr;

  // Start the CAN send thread
  std::thread send_thread(can_send_thread, pandas, fake_send);

  Params params;
  RateKeeper rk("pandad", 100);
  SubMaster sm({"selfdriveState"});
  PubMaster pm({"can", "pandaStates", "peripheralState"});
  PandaSafety panda_safety(pandas);
  Panda *peripheral_panda = pandas[0];
  bool engaged = false;
  bool is_onroad = false;

  // Main loop: receive CAN data and process states
  while (!do_exit && check_all_connected(pandas)) {
    can_recv(pandas, &pm);

    // Process peripheral state at 20 Hz
    if (rk.frame() % 5 == 0) {
      process_peripheral_state(peripheral_panda, &pm, no_fan_control);
    }

    // Process panda state at 10 Hz
    if (rk.frame() % 10 == 0) {
      sm.update(0);
      engaged = sm.allAliveAndValid({"selfdriveState"}) && sm["selfdriveState"].getSelfdriveState().getEnabled();
      is_onroad = params.getBool("IsOnroad");
      process_panda_state(pandas, &pm, engaged, is_onroad, spoofing_started);
      panda_safety.configureSafetyMode(is_onroad);
    }

    // Send out peripheralState at 2Hz
    if (rk.frame() % 50 == 0) {
      send_peripheral_state(peripheral_panda, &pm);
    }

    // Forward logs from pandas to cloudlog if available
    for (Panda *panda : pandas) {
      std::string log = panda->serial_read();
      if (!log.empty()) {
        if (log.find("Register 0x") != std::string::npos) {
          // Log register divergent faults as errors
          LOGE("%s", log.c_str());
        } else {
          LOGD("%s", log.c_str());
        }
      }
    }

    rk.keepTime();
  }

  // Close relay on exit to prevent a fault
  if (is_onroad && !engaged) {
    for (Panda *panda : pandas) {
      if (panda->connected()) {
        panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
      }
    }
  }

  send_thread.join();
}

void pandad_main_thread(std::vector<std::string> serials) {
  if (serials.empty()) {
    serials = Panda::list();
    if (serials.empty()) {
      LOGW("no pandas found, exiting");
      return;
    }
  }

  std::string serials_str;
  for (size_t i = 0; i < serials.size(); i++) {
    serials_str += serials[i];
    if (i < serials.size() - 1) {
      serials_str += ", ";
    }
  }
  LOGW("connecting to pandas: %s", serials_str.c_str());

  std::vector<Panda *> pandas;
  for (size_t i = 0; i < serials.size() && !do_exit; /**/) {
    Panda *panda = connect(serials[i], i);
    if (!panda) {
      util::sleep_for(100);
      continue;
    }
    pandas.push_back(panda);
    ++i;
  }

  if (!do_exit) {
    LOGW("connected to all pandas");
    pandad_run(pandas);
  }

  for (Panda *panda : pandas) {
    delete panda;
  }
}
