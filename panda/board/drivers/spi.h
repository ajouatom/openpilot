#pragma once

#include "spi_declarations.h"
#include "crc.h"

#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
#define SPI_V3_DMA_ACQUIRE_BARRIER() __DMB()
#include "spi_v3.h"
#endif

uint16_t spi_checksum_error_count = 0;

#if defined(ENABLE_SPI) || defined(BOOTSTUB)
static const unsigned char version_text[] = "VERSION";

static uint16_t spi_version_packet(uint8_t *out) {
  // This request and response deliberately stay compatible across protocol
  // generations so the host can enumerate an app or bootstub before choosing
  // the transport codec.
  (void)memcpy(out, version_text, 7);

  uint16_t data_len = 0U;
  const uint16_t data_pos = 7U + 2U;
  (void)memcpy(&out[data_pos], ((uint8_t *)UID_BASE), 12);
  data_len += 12U;
  out[data_pos + data_len] = hw_type;
  data_len += 1U;
  out[data_pos + data_len] = USB_PID & 0xFFU;
  data_len += 1U;
#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
  out[data_pos + data_len] = SPI_V3_VERSION;
#else
  out[data_pos + data_len] = 0x2U;
#endif
  data_len += 1U;

  out[7] = data_len & 0xFFU;
  out[8] = (data_len >> 8U) & 0xFFU;
  uint16_t resp_len = data_pos + data_len;
  out[resp_len] = crc_checksum(out, resp_len, 0xD5U);
  resp_len += 1U;
  return resp_len;
}
#endif

#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)

// DMA2 can access D2 SRAM but not DTCM. Keep the RX ring continuously armed;
// response buffers used by TX DMA live in the same non-cached SRAM region.
__attribute__((section(".sram12"), aligned(SPI_V3_RX_RING_SIZE)))
uint8_t spi_v3_dma_rx_ring[SPI_V3_RX_RING_SIZE];
__attribute__((section(".sram12"), aligned(2048)))
static spi_v3_replay_cache_t spi_v3_replay_cache;
__attribute__((section(".sram12"), aligned(32)))
static uint8_t spi_v3_scratch_response[SPI_V3_WIRE_BUFFER_SIZE];

static spi_v3_rx_ring_t spi_v3_rx_ring;
static spi_v3_stream_parser_t spi_v3_parser;
static uint8_t spi_v3_endpoint_response[SPI_V3_MAX_PAYLOAD_SIZE];
static uint32_t spi_v3_last_crc_error_count = 0U;
static uint32_t spi_v3_last_ring_overrun_count = 0U;

static void spi_v3_note_diagnostics(void) {
  const uint32_t crc_errors = spi_v3_parser.stats.bad_header_crc +
                              spi_v3_parser.stats.bad_frame_crc;
  const uint32_t new_crc_errors = crc_errors - spi_v3_last_crc_error_count;
  const uint32_t new_overruns = spi_v3_rx_ring.overruns - spi_v3_last_ring_overrun_count;
  const uint32_t new_errors = new_crc_errors + new_overruns;
  const uint32_t counter_space = UINT16_MAX - spi_checksum_error_count;
  spi_checksum_error_count += (uint16_t)MIN(new_errors, counter_space);
  spi_v3_last_crc_error_count = crc_errors;
  spi_v3_last_ring_overrun_count = spi_v3_rx_ring.overruns;
}

static bool spi_v3_send_status(const spi_v3_frame_t *request, spi_v3_status_t status) {
  uint16_t response_length = 0U;
  if (!spi_v3_encode_response(spi_v3_scratch_response, SPI_V3_WIRE_BUFFER_SIZE,
                              request, status, NULL, 0U, &response_length)) {
    return false;
  }
  return llspi_v3_send(spi_v3_scratch_response, response_length);
}

static bool spi_v3_terminal_response(const spi_v3_frame_t *request,
                                     spi_v3_status_t status,
                                     const uint8_t *payload, uint16_t payload_length) {
  if (!spi_v3_replay_cache_commit(&spi_v3_replay_cache, request, status,
                                  payload, payload_length)) {
    return spi_v3_send_status(request, SPI_V3_STATUS_INTERNAL_ERROR);
  }
  return llspi_v3_send(spi_v3_replay_cache.response,
                       spi_v3_replay_cache.response_length);
}

static void spi_v3_handle_request(const spi_v3_frame_t *request) {
  const spi_v3_request_disposition_t disposition =
    spi_v3_replay_cache_classify(&spi_v3_replay_cache, request);
  if (disposition == SPI_V3_REQUEST_REPLAY) {
    (void)llspi_v3_send(spi_v3_replay_cache.response,
                        spi_v3_replay_cache.response_length);
    return;
  }
  if (disposition == SPI_V3_REQUEST_CONFLICT) {
    (void)spi_v3_send_status(request, SPI_V3_STATUS_SEQUENCE_CONFLICT);
    return;
  }
  if (disposition != SPI_V3_REQUEST_NEW) {
    (void)spi_v3_send_status(request, SPI_V3_STATUS_BAD_FRAME);
    return;
  }

  spi_v3_status_t status = SPI_V3_STATUS_OK;
  uint16_t response_length = 0U;
  bool endpoint2_write = false;

  if (request->endpoint == 0U) {
    if (request->payload_length < sizeof(ControlPacket_t)) {
      status = SPI_V3_STATUS_BAD_LENGTH;
    } else {
      ControlPacket_t ctrl = {0};
      (void)memcpy((uint8_t *)&ctrl, request->payload, sizeof(ControlPacket_t));
      const int handler_length = comms_control_handler(&ctrl, spi_v3_endpoint_response);
      if ((handler_length < 0) ||
          ((uint32_t)handler_length > request->max_response_length) ||
          ((uint32_t)handler_length > SPI_V3_MAX_PAYLOAD_SIZE)) {
        status = SPI_V3_STATUS_INTERNAL_ERROR;
      } else {
        response_length = (uint16_t)handler_length;
      }
    }
  } else if ((request->endpoint == 1U) || (request->endpoint == 0x81U)) {
    if (request->payload_length != 0U) {
      status = SPI_V3_STATUS_BAD_LENGTH;
    } else {
      const int handler_length = comms_can_read(spi_v3_endpoint_response,
                                                request->max_response_length);
      if ((handler_length < 0) ||
          ((uint32_t)handler_length > request->max_response_length)) {
        status = SPI_V3_STATUS_INTERNAL_ERROR;
      } else {
        response_length = (uint16_t)handler_length;
      }
    }
  } else if (request->endpoint == 2U) {
    if (request->payload_length == 0U) {
      status = SPI_V3_STATUS_BAD_LENGTH;
    } else {
      endpoint2_write = true;
    }
  } else if (request->endpoint == 3U) {
    // Reject oversized batches before touching the partial-packet assembler,
    // reserving TX slots, or running any safety hooks.
    if (!spi_v3_can_write_length_valid(request->payload_length)) {
      status = SPI_V3_STATUS_BAD_LENGTH;
    } else {
      const comms_can_write_result_t write_result =
        comms_can_write_v3(request->payload, request->payload_length);
      if (write_result == COMMS_CAN_WRITE_BUSY) {
        // BUSY is non-terminal. No assembly-buffer or queue state changed, so
        // the byte-identical request can safely retry when capacity returns.
        (void)spi_v3_send_status(request, SPI_V3_STATUS_BUSY);
        return;
      } else if (write_result != COMMS_CAN_WRITE_OK) {
        status = SPI_V3_STATUS_BAD_LENGTH;
      }
    }
  } else if (request->endpoint == 0xABU) {
    if (request->payload_length != 0U) {
      status = SPI_V3_STATUS_BAD_LENGTH;
    } else {
      response_length = request->max_response_length;
      (void)memset(spi_v3_endpoint_response, 0xAB, response_length);
    }
  } else {
    status = SPI_V3_STATUS_BAD_ENDPOINT;
  }

  // Bulk write effects run in this single main-loop context. Complete them
  // before publishing/caching OK: the host cannot observe success early, and
  // a retry after llspi_v3_send() fails will replay the post-effect cache
  // without executing the write again.
  if ((status == SPI_V3_STATUS_OK) && endpoint2_write) {
    comms_endpoint2_write(request->payload, request->payload_length);
  }
  (void)spi_v3_terminal_response(request, status, spi_v3_endpoint_response,
                                 response_length);
}

void spi_init(void) {
  spi_v3_rx_ring_init(&spi_v3_rx_ring);
  spi_v3_stream_parser_init(&spi_v3_parser);
  spi_v3_replay_cache_init(&spi_v3_replay_cache);
  llspi_init();
}

void spi_process(void) {
  // VERSION is intentionally handled before framed requests and only while no
  // response is in flight. Its response retains the legacy enumeration shape.
  if (llspi_v3_tx_idle() && llspi_v3_take_version_request()) {
    const uint16_t version_length = spi_version_packet(spi_v3_scratch_response);
    (void)llspi_v3_send(spi_v3_scratch_response, version_length);
  }

  if (llspi_v3_tx_idle()) {
    bool rx_was_reset = false;
    const uint32_t rx_epoch_before = llspi_v3_rx_begin(&rx_was_reset);
    if (rx_was_reset) {
      spi_v3_stream_parser_reset(&spi_v3_parser);
    }
    const spi_v3_parse_result_t parse_result =
      spi_v3_rx_ring_drain(&spi_v3_rx_ring, spi_v3_dma_rx_ring,
                           SPI_V3_RX_RING_SIZE, &spi_v3_parser);
    const uint32_t rx_epoch_after = llspi_v3_rx_epoch();
    spi_v3_note_diagnostics();
    if (!spi_v3_rx_frame_epoch_valid(rx_epoch_before, rx_epoch_after)) {
      // DMA reset preempted the drain. Never execute a frame assembled across
      // epochs; retire everything published by the new epoch and await retry.
      (void)llspi_v3_rx_begin(NULL);
      spi_v3_stream_parser_reset(&spi_v3_parser);
    } else if (parse_result == SPI_V3_PARSE_FRAME_READY) {
      const spi_v3_frame_t *request = spi_v3_stream_parser_frame(&spi_v3_parser);
      if ((request != NULL) && (request->type == SPI_V3_FRAME_REQUEST)) {
        spi_v3_handle_request(request);
      }
      spi_v3_stream_parser_consume(&spi_v3_parser);
    }
  }
}

void can_tx_comms_resume_spi(void) {
  // v3 reserves the exact per-bus batch requirements immediately before the
  // write, so it does not use the legacy coarse ready/NACK latch.
}

#else

#ifdef STM32H7
#define SPI_BUF_SIZE 2048U
// H7 DMA2 located in D2 domain, so we need to use SRAM1/SRAM2
__attribute__((section(".sram12"))) uint8_t spi_buf_rx[SPI_BUF_SIZE];
__attribute__((section(".sram12"))) uint8_t spi_buf_tx[SPI_BUF_SIZE];
#else
#define SPI_BUF_SIZE 1024U
uint8_t spi_buf_rx[SPI_BUF_SIZE];
uint8_t spi_buf_tx[SPI_BUF_SIZE];
#endif

#if defined(ENABLE_SPI) || defined(BOOTSTUB)
static uint8_t spi_state = SPI_STATE_HEADER;
static uint16_t spi_data_len_mosi;
static bool spi_can_tx_ready = false;

void spi_init(void) {
  // platform init
  llspi_init();

  // Start the first packet!
  spi_state = SPI_STATE_HEADER;
  llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
}

static bool validate_checksum(const uint8_t *data, uint16_t len) {
  // TODO: can speed this up by casting the bulk to uint32_t and xor-ing the bytes afterwards
  uint8_t checksum = SPI_CHECKSUM_START;
  for(uint16_t i = 0U; i < len; i++){
    checksum ^= data[i];
  }
  return checksum == 0U;
}

void spi_rx_done(void) {
  uint16_t response_len = 0U;
  uint8_t next_rx_state = SPI_STATE_HEADER_NACK;
  bool checksum_valid = false;
  static uint8_t spi_endpoint;
  static uint16_t spi_data_len_miso;

  // parse header
  spi_endpoint = spi_buf_rx[1];
  spi_data_len_mosi = (spi_buf_rx[3] << 8) | spi_buf_rx[2];
  spi_data_len_miso = (spi_buf_rx[5] << 8) | spi_buf_rx[4];

  if (memcmp(spi_buf_rx, version_text, 7) == 0) {
    response_len = spi_version_packet(spi_buf_tx);
    next_rx_state = SPI_STATE_HEADER_NACK;
    // VERSION is an intentional stable probe, not a malformed v2 header.
    checksum_valid = true;
  } else if (spi_state == SPI_STATE_HEADER) {
    checksum_valid = validate_checksum(spi_buf_rx, SPI_HEADER_SIZE);
    if ((spi_buf_rx[0] == SPI_SYNC_BYTE) && checksum_valid) {
      // response: ACK and start receiving data portion
      spi_buf_tx[0] = SPI_HACK;
      next_rx_state = SPI_STATE_HEADER_ACK;
      response_len = 1U;
    } else {
      // response: NACK and reset state machine
      #ifdef DEBUG_SPI
        print("- incorrect header sync or checksum "); hexdump(spi_buf_rx, SPI_HEADER_SIZE);
      #endif
      spi_buf_tx[0] = SPI_NACK;
      next_rx_state = SPI_STATE_HEADER_NACK;
      response_len = 1U;
    }
  } else if (spi_state == SPI_STATE_DATA_RX) {
    // We got everything! Based on the endpoint specified, call the appropriate handler
    bool response_ack = false;
    checksum_valid = validate_checksum(&(spi_buf_rx[SPI_HEADER_SIZE]), spi_data_len_mosi + 1U);
    if (checksum_valid) {
      if (spi_endpoint == 0U) {
        if (spi_data_len_mosi >= sizeof(ControlPacket_t)) {
          ControlPacket_t ctrl = {0};
          (void)memcpy((uint8_t*)&ctrl, &spi_buf_rx[SPI_HEADER_SIZE], sizeof(ControlPacket_t));
          response_len = comms_control_handler(&ctrl, &spi_buf_tx[3]);
          response_ack = true;
        } else {
          print("SPI: insufficient data for control handler\n");
        }
      } else if ((spi_endpoint == 1U) || (spi_endpoint == 0x81U)) {
        if (spi_data_len_mosi == 0U) {
          response_len = comms_can_read(&(spi_buf_tx[3]), spi_data_len_miso);
          response_ack = true;
        } else {
          print("SPI: did not expect data for can_read\n");
        }
      } else if (spi_endpoint == 2U) {
        if (spi_data_len_mosi > 0U) {
          comms_endpoint2_write(&spi_buf_rx[SPI_HEADER_SIZE], spi_data_len_mosi);
          response_ack = true;
        } else {
          print("SPI: insufficient data for endpoint 2\n");
        }
      } else if (spi_endpoint == 3U) {
        if (spi_data_len_mosi > 0U) {
          if (spi_can_tx_ready) {
            spi_can_tx_ready = false;
            comms_can_write(&spi_buf_rx[SPI_HEADER_SIZE], spi_data_len_mosi);
            response_ack = true;
          } else {
            response_ack = false;
            print("SPI: CAN NACK\n");
          }
        } else {
          print("SPI: did expect data for can_write\n");
        }
      } else if (spi_endpoint == 0xABU) {
        // test endpoint, send max response length
        response_len = spi_data_len_miso;
        response_ack = true;
      } else {
        print("SPI: unexpected endpoint"); puth(spi_endpoint); print("\n");
      }
    } else {
      // Checksum was incorrect
      response_ack = false;
      #ifdef DEBUG_SPI
        print("- incorrect data checksum ");
        puth4(spi_data_len_mosi);
        print("\n");
        hexdump(spi_buf_rx, SPI_HEADER_SIZE);
        hexdump(&(spi_buf_rx[SPI_HEADER_SIZE]), MIN(spi_data_len_mosi, 64));
        print("\n");
      #endif
    }

    if (!response_ack) {
      spi_buf_tx[0] = SPI_NACK;
      next_rx_state = SPI_STATE_HEADER_NACK;
      response_len = 1U;
    } else {
      // Setup response header
      spi_buf_tx[0] = SPI_DACK;
      spi_buf_tx[1] = response_len & 0xFFU;
      spi_buf_tx[2] = (response_len >> 8) & 0xFFU;

      // Add checksum
      uint8_t checksum = SPI_CHECKSUM_START;
      for(uint16_t i = 0U; i < (response_len + 3U); i++) {
        checksum ^= spi_buf_tx[i];
      }
      spi_buf_tx[response_len + 3U] = checksum;
      response_len += 4U;

      next_rx_state = SPI_STATE_DATA_TX;
    }
  } else {
    print("SPI: RX unexpected state: "); puth(spi_state); print("\n");
  }

  // send out response
  if (response_len == 0U) {
    print("SPI: no response\n");
    spi_buf_tx[0] = SPI_NACK;
    spi_state = SPI_STATE_HEADER_NACK;
    response_len = 1U;
  }
  llspi_miso_dma(spi_buf_tx, response_len);

  spi_state = next_rx_state;
  if (!checksum_valid && (spi_checksum_error_count < UINT16_MAX)) {
    spi_checksum_error_count += 1U;
  }
}

void spi_tx_done(bool reset) {
  if ((spi_state == SPI_STATE_HEADER_NACK) || reset) {
    // Reset state
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
  } else if (spi_state == SPI_STATE_HEADER_ACK) {
    // ACK was sent, queue up the RX buf for the data + checksum
    spi_state = SPI_STATE_DATA_RX;
    llspi_mosi_dma(&spi_buf_rx[SPI_HEADER_SIZE], spi_data_len_mosi + 1U);
  } else if (spi_state == SPI_STATE_DATA_TX) {
    // Reset state
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
  } else {
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
    print("SPI: TX unexpected state: "); puth(spi_state); print("\n");
  }
}

void can_tx_comms_resume_spi(void) {
  spi_can_tx_ready = true;
}
#else
void can_tx_comms_resume_spi(void) {
  return;
}
#endif

#endif  // STM32H7 application v3 / legacy v2
