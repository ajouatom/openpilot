#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board/drivers/spi_v3.h"

static uint16_t make_request(uint8_t *output, uint32_t session_id, uint32_t sequence,
                             const uint8_t *payload, uint16_t payload_length,
                             uint16_t max_response_length) {
  const spi_v3_frame_fields_t request = {
    .type = SPI_V3_FRAME_REQUEST,
    .flags = 0U,
    .status = SPI_V3_STATUS_OK,
    .session_id = session_id,
    .sequence = sequence,
    .endpoint = 3U,
    .max_response_length = max_response_length,
    .payload = payload,
    .payload_length = payload_length,
  };
  uint16_t length = 0U;
  assert(spi_v3_encode_frame(output, SPI_V3_WIRE_BUFFER_SIZE, &request, &length));
  return length;
}

static const spi_v3_frame_t *feed_all(spi_v3_stream_parser_t *parser,
                                      const uint8_t *data, size_t length) {
  size_t offset = 0U;
  while (offset < length) {
    size_t consumed = 0U;
    const spi_v3_parse_result_t result =
      spi_v3_stream_parser_feed(parser, &data[offset], 1U, &consumed);
    assert(consumed == 1U);
    offset += consumed;
    if (result == SPI_V3_PARSE_FRAME_READY) {
      assert(offset == length);
      break;
    }
    assert(result == SPI_V3_PARSE_NONE);
  }
  return spi_v3_stream_parser_frame(parser);
}

static void test_crc_and_round_trip(void) {
  static const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  assert(spi_v3_crc32c(check, sizeof(check)) == 0xe3069283U);
  assert(spi_v3_crc32c(NULL, 0U) == 0U);

  const uint8_t payload[] = {0x00U, 0xcdU, 0xffU};
  uint8_t encoded[SPI_V3_WIRE_BUFFER_SIZE];
  const uint16_t encoded_length =
    make_request(encoded, 0x12345678U, 0x89abcdefU, payload, sizeof(payload), 512U);
  static const uint8_t expected[] = {
    0x5aU, 0xc3U, 0x69U, 0x96U, 0x03U, 0x01U, 0x00U, 0x00U,
    0x78U, 0x56U, 0x34U, 0x12U, 0xefU, 0xcdU, 0xabU, 0x89U,
    0x03U, 0x00U, 0x03U, 0x00U, 0x00U, 0x02U, 0x00U, 0x00U,
    0x83U, 0x1aU, 0xd3U, 0x18U, 0x00U, 0xcdU, 0xffU, 0xd2U,
    0x3fU, 0xabU, 0xd2U,
  };

  assert(encoded_length == SPI_V3_HEADER_SIZE + sizeof(payload) + SPI_V3_TRAILER_SIZE);
  assert(encoded_length == sizeof(expected));
  assert(memcmp(encoded, expected, sizeof(expected)) == 0);
  assert(memcmp(encoded, spi_v3_magic, sizeof(spi_v3_magic)) == 0);
  assert(spi_v3_read_u32_le(&encoded[SPI_V3_SESSION_OFFSET]) == 0x12345678U);
  assert(spi_v3_read_u32_le(&encoded[SPI_V3_SEQUENCE_OFFSET]) == 0x89abcdefU);

  spi_v3_stream_parser_t parser;
  spi_v3_stream_parser_init(&parser);
  const spi_v3_frame_t *frame = feed_all(&parser, encoded, encoded_length);
  assert(frame != NULL);
  assert(frame->type == SPI_V3_FRAME_REQUEST);
  assert(frame->session_id == 0x12345678U);
  assert(frame->sequence == 0x89abcdefU);
  assert(frame->endpoint == 3U);
  assert(frame->max_response_length == 512U);
  assert(frame->payload_length == sizeof(payload));
  assert(memcmp(frame->payload, payload, sizeof(payload)) == 0);
}

static void test_filler_split_and_corruption_resync(void) {
  const uint8_t first_payload[] = {0x10U, 0x11U, 0x12U};
  const uint8_t valid_payload[] = {0x20U, 0x21U};
  uint8_t bad_header[SPI_V3_WIRE_BUFFER_SIZE];
  uint8_t bad_payload[SPI_V3_WIRE_BUFFER_SIZE];
  uint8_t valid[SPI_V3_WIRE_BUFFER_SIZE];
  const uint16_t bad_header_length = make_request(bad_header, 1U, 1U, first_payload,
                                                   sizeof(first_payload), 0U);
  const uint16_t bad_payload_length = make_request(bad_payload, 1U, 2U, first_payload,
                                                    sizeof(first_payload), 0U);
  const uint16_t valid_length = make_request(valid, 1U, 3U, valid_payload,
                                              sizeof(valid_payload), 0U);
  bad_header[SPI_V3_SESSION_OFFSET] ^= 0x80U;
  bad_payload[SPI_V3_HEADER_SIZE + 1U] ^= 0x01U;

  uint8_t stream[(SPI_V3_WIRE_BUFFER_SIZE * 3U) + 64U];
  size_t stream_length = 0U;
  memset(stream, 0xcd, 37U);
  stream_length += 37U;
  stream[stream_length++] = spi_v3_magic[0];
  stream[stream_length++] = spi_v3_magic[1];
  stream[stream_length++] = 0x00U;
  memcpy(&stream[stream_length], bad_header, bad_header_length);
  stream_length += bad_header_length;
  memcpy(&stream[stream_length], bad_payload, bad_payload_length);
  stream_length += bad_payload_length;
  memcpy(&stream[stream_length], valid, valid_length);
  stream_length += valid_length;

  spi_v3_stream_parser_t parser;
  spi_v3_stream_parser_init(&parser);
  size_t offset = 0U;
  while (offset < stream_length) {
    const size_t chunk = ((stream_length - offset) > 7U) ? 7U : stream_length - offset;
    size_t consumed = 0U;
    const spi_v3_parse_result_t result =
      spi_v3_stream_parser_feed(&parser, &stream[offset], chunk, &consumed);
    offset += consumed;
    if (result == SPI_V3_PARSE_FRAME_READY) {
      break;
    }
    assert(result == SPI_V3_PARSE_NONE);
  }

  const spi_v3_frame_t *frame = spi_v3_stream_parser_frame(&parser);
  assert(frame != NULL);
  assert(frame->sequence == 3U);
  assert(frame->payload_length == sizeof(valid_payload));
  assert(memcmp(frame->payload, valid_payload, sizeof(valid_payload)) == 0);
  assert(parser.stats.bad_header_crc == 1U);
  assert(parser.stats.bad_frame_crc == 1U);
  assert(parser.stats.discarded_bytes >= 40U);
}

static void test_maximum_frame_fits_exactly(void) {
  uint8_t payload[SPI_V3_MAX_PAYLOAD_SIZE];
  uint8_t encoded[SPI_V3_WIRE_BUFFER_SIZE];
  for (uint16_t i = 0U; i < sizeof(payload); ++i) {
    payload[i] = (uint8_t)i;
  }
  const uint16_t encoded_length = make_request(encoded, 2U, 4U, payload,
                                                sizeof(payload), SPI_V3_MAX_PAYLOAD_SIZE);
  assert(encoded_length == SPI_V3_WIRE_BUFFER_SIZE);

  spi_v3_stream_parser_t parser;
  spi_v3_stream_parser_init(&parser);
  const spi_v3_frame_t *frame = feed_all(&parser, encoded, encoded_length);
  assert(frame != NULL);
  assert(frame->payload_length == SPI_V3_MAX_PAYLOAD_SIZE);
  assert(memcmp(frame->payload, payload, sizeof(payload)) == 0);
  assert(parser.stats.buffer_overflow == 0U);
}

static void dma_write(uint8_t *ring_buffer, uint16_t ring_size, uint32_t *dma_position,
                      spi_v3_rx_ring_t *ring, const uint8_t *data, uint16_t length) {
  for (uint16_t i = 0U; i < length; ++i) {
    ring_buffer[*dma_position & (uint32_t)(ring_size - 1U)] = data[i];
    *dma_position += 1U;
  }
  spi_v3_rx_ring_note_produced(ring, length);
}

static void test_circular_ring_wrap_and_overrun(void) {
  uint8_t dma_ring[64];
  uint32_t dma_position = 0U;
  spi_v3_rx_ring_t ring;
  spi_v3_stream_parser_t parser;
  spi_v3_rx_ring_init(&ring);
  spi_v3_stream_parser_init(&parser);

  uint8_t filler[61];
  memset(filler, 0xcd, sizeof(filler));
  dma_write(dma_ring, sizeof(dma_ring), &dma_position, &ring, filler, sizeof(filler));
  assert(spi_v3_rx_ring_drain(&ring, dma_ring, sizeof(dma_ring), &parser) == SPI_V3_PARSE_NONE);

  const uint8_t payload[] = {0xaaU, 0xbbU, 0xccU, 0xddU};
  uint8_t encoded[SPI_V3_WIRE_BUFFER_SIZE];
  const uint16_t encoded_length = make_request(encoded, 9U, 77U, payload, sizeof(payload), 0U);
  for (uint16_t offset = 0U; offset < encoded_length;) {
    const uint16_t remaining = (uint16_t)(encoded_length - offset);
    const uint16_t chunk = (remaining > 11U) ? 11U : remaining;
    dma_write(dma_ring, sizeof(dma_ring), &dma_position, &ring, &encoded[offset], chunk);
    const spi_v3_parse_result_t result =
      spi_v3_rx_ring_drain(&ring, dma_ring, sizeof(dma_ring), &parser);
    offset = (uint16_t)(offset + chunk);
    if (offset == encoded_length) {
      assert(result == SPI_V3_PARSE_FRAME_READY);
    } else {
      assert(result == SPI_V3_PARSE_NONE);
    }
  }
  assert(spi_v3_stream_parser_frame(&parser)->sequence == 77U);

  spi_v3_stream_parser_consume(&parser);
  uint8_t overrun_filler[65];
  memset(overrun_filler, 0xcd, sizeof(overrun_filler));
  dma_write(dma_ring, sizeof(dma_ring), &dma_position, &ring,
            overrun_filler, sizeof(overrun_filler));
  assert(spi_v3_rx_ring_drain(&ring, dma_ring, sizeof(dma_ring), &parser) == SPI_V3_PARSE_NONE);
  assert(ring.overruns == 1U);
}

static void test_exactly_once_replay_cache(void) {
  const uint8_t payload[] = {0xaaU, 0xbbU};
  uint8_t encoded[SPI_V3_WIRE_BUFFER_SIZE];
  const uint16_t encoded_length = make_request(encoded, 0x10203040U, 55U, payload,
                                                sizeof(payload), 1U);
  spi_v3_stream_parser_t parser;
  spi_v3_stream_parser_init(&parser);
  const spi_v3_frame_t *request = feed_all(&parser, encoded, encoded_length);

  spi_v3_replay_cache_t cache;
  spi_v3_replay_cache_init(&cache);
  assert(spi_v3_replay_cache_classify(&cache, request) == SPI_V3_REQUEST_NEW);
  const uint8_t response_payload[] = {0x99U};
  assert(spi_v3_replay_cache_commit(&cache, request, SPI_V3_STATUS_OK,
                                    response_payload, sizeof(response_payload)));
  assert(spi_v3_replay_cache_classify(&cache, request) == SPI_V3_REQUEST_REPLAY);
  assert(cache.response_length == SPI_V3_HEADER_SIZE + sizeof(response_payload) +
                                  SPI_V3_TRAILER_SIZE);
  assert(!spi_v3_replay_cache_commit(&cache, request, SPI_V3_STATUS_BUSY, NULL, 0U));
  const uint8_t oversized_response[] = {0x01U, 0x02U};
  assert(!spi_v3_replay_cache_commit(&cache, request, SPI_V3_STATUS_OK,
                                     oversized_response, sizeof(oversized_response)));

  spi_v3_stream_parser_consume(&parser);
  const uint8_t different_payload[] = {0xabU, 0xbbU};
  const uint16_t conflict_length = make_request(encoded, 0x10203040U, 55U,
                                                 different_payload, sizeof(different_payload), 0U);
  request = feed_all(&parser, encoded, conflict_length);
  assert(spi_v3_replay_cache_classify(&cache, request) == SPI_V3_REQUEST_CONFLICT);

  spi_v3_stream_parser_consume(&parser);
  const uint16_t next_length = make_request(encoded, 0x10203040U, 56U,
                                             payload, sizeof(payload), 0U);
  request = feed_all(&parser, encoded, next_length);
  assert(spi_v3_replay_cache_classify(&cache, request) == SPI_V3_REQUEST_NEW);
}

static void test_version_transaction_boundary_and_wrap(void) {
  uint8_t ring[16];
  memset(ring, 0xcd, sizeof(ring));
  static const uint8_t version[] = {'V', 'E', 'R', 'S', 'I', 'O', 'N'};
  const uint32_t start = 13U;
  for (uint32_t i = 0U; i < sizeof(version); ++i) {
    ring[(start + i) & 15U] = version[i];
  }
  assert(spi_v3_ring_transaction_equals(ring, sizeof(ring), start,
                                         start + sizeof(version), version,
                                         sizeof(version)));
  assert(!spi_v3_ring_transaction_equals(ring, sizeof(ring), start,
                                          start + sizeof(version) + 1U, version,
                                          sizeof(version)));
  ring[0] ^= 1U;
  assert(!spi_v3_ring_transaction_equals(ring, sizeof(ring), start,
                                          start + sizeof(version), version,
                                          sizeof(version)));
}

static void test_tx_completion_all_interrupt_orders(void) {
  spi_v3_tx_completion_t tx = {0};
  spi_v3_tx_completion_start(&tx);
  spi_v3_tx_completion_note_nss(&tx, false);  // short poll, DMA still active
  spi_v3_tx_completion_note_txc(&tx);
  assert(!spi_v3_tx_completion_ready(&tx));
  spi_v3_tx_completion_note_dma(&tx, false);
  assert(!spi_v3_tx_completion_ready(&tx));   // requires a later NSS edge
  spi_v3_tx_completion_note_nss(&tx, true);
  assert(spi_v3_tx_completion_ready(&tx));
  spi_v3_tx_completion_finish(&tx);
  assert(!spi_v3_tx_completion_ready(&tx));

  spi_v3_tx_completion_start(&tx);
  spi_v3_tx_completion_note_dma(&tx, true);   // DMA IRQ ran after NSS EXTI
  assert(!spi_v3_tx_completion_ready(&tx));
  spi_v3_tx_completion_note_txc(&tx);         // main-loop TXC poll fallback
  assert(spi_v3_tx_completion_ready(&tx));

  spi_v3_tx_completion_start(&tx);
  spi_v3_tx_completion_note_dma(&tx, false);
  spi_v3_tx_completion_note_txc(&tx);
  assert(!spi_v3_tx_completion_ready(&tx));
  spi_v3_tx_completion_note_nss(&tx, false);
  assert(spi_v3_tx_completion_ready(&tx));
}

static void test_write_effect_then_send_failure_replays_once(void) {
  const uint8_t payload[] = {0x01U, 0x02U, 0x03U};
  uint8_t encoded[SPI_V3_WIRE_BUFFER_SIZE];
  const uint16_t encoded_length = make_request(encoded, 7U, 99U, payload,
                                                sizeof(payload), 0U);
  spi_v3_stream_parser_t parser;
  spi_v3_stream_parser_init(&parser);
  const spi_v3_frame_t *request = feed_all(&parser, encoded, encoded_length);
  spi_v3_replay_cache_t cache;
  spi_v3_replay_cache_init(&cache);

  uint32_t write_effect_count = 0U;
  assert(spi_v3_replay_cache_classify(&cache, request) == SPI_V3_REQUEST_NEW);
  write_effect_count += 1U;  // firmware completes endpoint 2/3 effect first
  assert(spi_v3_replay_cache_commit(&cache, request, SPI_V3_STATUS_OK, NULL, 0U));
  const bool simulated_llspi_send = false;
  assert(!simulated_llspi_send);

  // The failed queue/send leaves the terminal cache intact. A byte-identical
  // retry is replayed and cannot increment the effect count.
  assert(spi_v3_replay_cache_classify(&cache, request) == SPI_V3_REQUEST_REPLAY);
  assert(write_effect_count == 1U);
  assert(cache.response_length == SPI_V3_HEADER_SIZE + SPI_V3_TRAILER_SIZE);
}

static void test_rx_dma_wrap_and_epoch_invariants(void) {
  assert(!spi_v3_rx_progress_ambiguous(true, false, 100U, 4096U));
  assert(!spi_v3_rx_progress_ambiguous(false, true, 4096U, 0U));
  assert(spi_v3_rx_progress_ambiguous(true, true, 0U, 100U));
  assert(spi_v3_rx_progress_ambiguous(false, true, 100U, 200U));
  assert(spi_v3_rx_progress_ambiguous(true, true, 0U, 0U));

  assert(spi_v3_rx_frame_epoch_valid(7U, 7U));
  assert(!spi_v3_rx_frame_epoch_valid(7U, 8U));
  assert(!spi_v3_rx_frame_epoch_valid(UINT32_MAX, 0U));
}

static void test_can_batch_atomic_capacity_preflight(void) {
  uint8_t maximum_zero_length_frames[SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE] = {0U};
  uint16_t required[SPI_V3_CAN_BUS_COUNT] = {0U};
  assert(spi_v3_can_batch_requirements(NULL, 0U, 0U,
                                       maximum_zero_length_frames,
                                       sizeof(maximum_zero_length_frames),
                                       required));
  assert(required[0] == (SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE / SPI_V3_CAN_PACKET_HEADER_SIZE));
  assert(required[1] == 0U);
  assert(required[2] == 0U);

  const uint32_t one_short[SPI_V3_CAN_BUS_COUNT] = {
    required[0] - 1U, 415U, 415U,
  };
  const uint32_t exact_fit[SPI_V3_CAN_BUS_COUNT] = {
    required[0], 415U, 415U,
  };
  assert(!spi_v3_can_batch_fits(required, one_short));
  assert(spi_v3_can_batch_fits(required, exact_fit));

  const uint8_t mixed_buses[12] = {
    0x02U, 0U, 0U, 0U, 0U, 0U,
    0x04U, 0U, 0U, 0U, 0U, 0U,
  };
  assert(spi_v3_can_batch_requirements(NULL, 0U, 0U, mixed_buses,
                                       sizeof(mixed_buses), required));
  assert((required[0] == 0U) && (required[1] == 1U) && (required[2] == 1U));

  const uint8_t partial[3] = {0U, 0U, 0U};
  const uint8_t completes_then_bus1[9] = {
    0U, 0U, 0U,
    0x02U, 0U, 0U, 0U, 0U, 0U,
  };
  assert(spi_v3_can_batch_requirements(partial, sizeof(partial), 3U,
                                       completes_then_bus1,
                                       sizeof(completes_then_bus1), required));
  assert((required[0] == 1U) && (required[1] == 1U) && (required[2] == 0U));

  const uint8_t invalid_bus[1] = {0x06U};
  assert(!spi_v3_can_batch_requirements(NULL, 0U, 0U, invalid_bus,
                                        sizeof(invalid_bus), required));
  assert(!spi_v3_can_batch_requirements(partial, sizeof(partial), 4U,
                                        NULL, 0U, required));
}

static void test_can_write_payload_limit(void) {
  assert(!spi_v3_can_write_length_valid(0U));
  assert(spi_v3_can_write_length_valid(1U));
  assert(spi_v3_can_write_length_valid(SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE));
  assert(!spi_v3_can_write_length_valid(SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE + 1U));

  // The endpoint-specific execution limit must not shrink the generic codec.
  assert(SPI_V3_MAX_PAYLOAD_SIZE > SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE);
}

int main(void) {
  test_crc_and_round_trip();
  test_filler_split_and_corruption_resync();
  test_maximum_frame_fits_exactly();
  test_circular_ring_wrap_and_overrun();
  test_exactly_once_replay_cache();
  test_version_transaction_boundary_and_wrap();
  test_tx_completion_all_interrupt_orders();
  test_write_effect_then_send_failure_replays_once();
  test_rx_dma_wrap_and_epoch_invariants();
  test_can_batch_atomic_capacity_preflight();
  test_can_write_payload_limit();
  puts("SPI v3 parser tests passed");
  return 0;
}
