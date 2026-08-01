// When changing these structs, python/__init__.py needs to be kept up to date!

#define HEALTH_PACKET_VERSION 16
struct __attribute__((packed)) health_t {
  uint32_t uptime_pkt;
  uint32_t voltage_pkt;
  uint32_t current_pkt;
  uint32_t safety_tx_blocked_pkt;
  uint32_t safety_rx_invalid_pkt;
  uint32_t tx_buffer_overflow_pkt;
  uint32_t rx_buffer_overflow_pkt;
  uint32_t faults_pkt;
  uint8_t ignition_line_pkt;
  uint8_t ignition_can_pkt;
  uint8_t controls_allowed_pkt;
  uint8_t car_harness_status_pkt;
  uint8_t safety_mode_pkt;
  uint16_t safety_param_pkt;
  uint8_t fault_status_pkt;
  uint8_t power_save_enabled_pkt;
  uint8_t heartbeat_lost_pkt;
  uint16_t alternative_experience_pkt;
  float interrupt_load_pkt;
  uint8_t fan_power;
  uint8_t safety_rx_checks_invalid_pkt;
  uint16_t spi_checksum_error_count_pkt;
  uint8_t fan_stall_count;
  uint16_t sbu1_voltage_mV;
  uint16_t sbu2_voltage_mV;
  uint8_t som_reset_triggered;
};

#define CAN_HEALTH_PACKET_VERSION 5
typedef struct __attribute__((packed)) {
  uint8_t bus_off;
  uint32_t bus_off_cnt;
  uint8_t error_warning;
  uint8_t error_passive;
  uint8_t last_error; // real time LEC value
  uint8_t last_stored_error; // last LEC positive error code stored
  uint8_t last_data_error; // DLEC (for CANFD only)
  uint8_t last_data_stored_error; // last DLEC positive error code stored (for CANFD only)
  uint8_t receive_error_cnt; // Actual state of the receive error counter, values between 0 and 127. FDCAN_ECR.REC
  uint8_t transmit_error_cnt; // Actual state of the transmit error counter, values between 0 and 255. FDCAN_ECR.TEC
  uint32_t total_error_cnt; // How many times any error interrupt was invoked
  uint32_t total_tx_lost_cnt; // Tx event FIFO element lost
  uint32_t total_rx_lost_cnt; // Rx FIFO 0 message lost due to FIFO full condition
  uint32_t total_tx_cnt;
  uint32_t total_rx_cnt;
  uint32_t total_fwd_cnt; // Messages forwarded from one bus to another
  uint32_t total_tx_checksum_error_cnt;
  uint16_t can_speed;
  uint16_t can_data_speed;
  uint8_t canfd_enabled;
  uint8_t brs_enabled;
  uint8_t canfd_non_iso;
  uint32_t irq0_call_rate;
  uint32_t irq1_call_rate;
  uint32_t irq2_call_rate;
  uint32_t can_core_reset_cnt;
} can_health_t;

#define PANDA_DIAGNOSTICS_VERSION 1U
#define PANDA_DIAGNOSTICS_RESET_MAGIC 0xD1A6U
#define PANDA_DIAGNOSTICS_PAGE_SPI 0U
#define PANDA_DIAGNOSTICS_PAGE_CAN_HIST_BASE 1U
#define PANDA_DIAGNOSTICS_PAGE_CAN_SNAPSHOT_BASE 4U
#define PANDA_DIAGNOSTICS_PAGE_HYUNDAI_SUMMARY 16U
#define PANDA_DIAGNOSTICS_PAGE_HYUNDAI_BUFFER_BASE 17U

#define PANDA_CAN_ERROR_STAGE_FIRST 0U
#define PANDA_CAN_ERROR_STAGE_WARNING 1U
#define PANDA_CAN_ERROR_STAGE_PASSIVE 2U
#define PANDA_CAN_ERROR_STAGE_BUS_OFF 3U
#define PANDA_CAN_ERROR_STAGE_COUNT 4U
#define PANDA_DIAGNOSTICS_HYUNDAI_BUFFER_COUNT 6U

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t page;
  uint8_t spi_state;
  uint8_t can_tx_ready;
  uint32_t header_sync_nack_cnt;
  uint32_t header_checksum_nack_cnt;
  uint32_t data_checksum_nack_cnt;
  uint32_t endpoint3_checksum_nack_cnt;
  uint32_t endpoint3_backpressure_nack_cnt;
  uint32_t endpoint3_ack_cnt;
  uint32_t last_endpoint3_checksum_time_us;
  uint32_t last_endpoint3_backpressure_time_us;
  uint16_t last_endpoint3_checksum_len;
  uint16_t last_endpoint3_backpressure_len;
  uint16_t current_tx_free[3];
  uint16_t last_backpressure_tx_free[3];
  uint32_t tx_overflow_by_bus[3];
} panda_spi_diag_t;

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t page;
  uint8_t can_number;
  uint8_t bus_number;
  uint32_t nominal_error_cnt[6];
  uint32_t data_error_cnt[6];
  uint32_t error_warning_irq_cnt;
  uint32_t error_passive_irq_cnt;
  uint32_t bus_off_irq_cnt;
} panda_can_error_hist_t;

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t page;
  uint8_t can_number;
  uint8_t bus_number;
  uint8_t valid;
  uint8_t safety_mode;
  uint16_t safety_param;
  uint32_t timestamp_us;
  uint32_t ir_reg;
  uint32_t psr_reg;
  uint32_t ecr_reg;
  uint32_t txfqs_reg;
  uint32_t rxf0s_reg;
  uint32_t last_tx_addr;
  uint32_t last_tx_time_us;
  uint32_t last_rx_addr;
  uint32_t last_rx_time_us;
  uint32_t last_host_addr;
  uint32_t last_host_time_us;
  uint32_t last_fwd_addr;
  uint32_t last_fwd_time_us;
} panda_can_error_diag_t;

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t page;
  uint8_t enabled;
  uint8_t safety_mode;
  uint16_t safety_param;
  uint8_t buffer_count;
  uint8_t reserved;
  uint32_t tx_buffered_cnt;
  uint32_t fwd_call_cnt;
  uint32_t fwd_from_bus0_cnt;
  uint32_t fwd_from_bus2_cnt;
  uint32_t replace_to_bus0_cnt;
  uint32_t replace_to_bus2_cnt;
  uint32_t pass_to_bus0_cnt;
  uint32_t pass_to_bus2_cnt;
  uint32_t dynamic_block_to_bus0_cnt;
  uint32_t dynamic_block_to_bus2_cnt;
  uint32_t empty_to_bus0_cnt;
  uint32_t empty_to_bus2_cnt;
  uint32_t block_4b9_cnt;
  uint32_t invalid_bus_cnt;
} panda_hyundai_fwd_diag_t;

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t page;
  uint8_t index;
  uint8_t enabled;
  uint16_t addr;
  uint8_t dst_bus;
  uint8_t count;
  uint8_t reuse_left;
  uint8_t started;
  uint8_t has_last_pkt;
  uint8_t reserved;
  uint32_t push_attempt_cnt;
  uint32_t push_accepted_cnt;
  uint32_t push_full_drop_cnt;
  uint32_t pop_attempt_cnt;
  uint32_t pop_success_cnt;
  uint32_t reuse_attempt_cnt;
  uint32_t reuse_success_cnt;
  uint32_t empty_fallback_cnt;
  uint32_t reset_cnt;
  uint32_t last_push_time_us;
  uint32_t last_pop_time_us;
  uint32_t last_reuse_time_us;
  uint32_t last_empty_time_us;
} panda_hyundai_buffer_diag_t;
