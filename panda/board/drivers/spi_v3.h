#pragma once

// SPI protocol v3 wire codec and stream parser.
//
// This file deliberately has no Panda or STM32 dependencies. The H7 SPI
// driver can keep RX DMA running in circular mode and publish completed byte
// counts to spi_v3_rx_ring_note_produced(). The parser then tolerates idle
// bytes, split transactions, and arbitrary alignment without changing SPI
// phases or stopping RX DMA.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SPI_V3_VERSION 3U
#define SPI_V3_HEADER_SIZE 28U
#define SPI_V3_TRAILER_SIZE 4U
#define SPI_V3_WIRE_BUFFER_SIZE 2048U
#define SPI_V3_MAX_PAYLOAD_SIZE (SPI_V3_WIRE_BUFFER_SIZE - SPI_V3_HEADER_SIZE - SPI_V3_TRAILER_SIZE)
// Keep one side-effecting CAN write small enough to bound queue reservation and
// safety-hook work. The generic v3 wire codec still accepts 2016-byte payloads
// for other endpoints.
#define SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE 512U

#define SPI_V3_VERSION_OFFSET 4U
#define SPI_V3_TYPE_OFFSET 5U
#define SPI_V3_FLAGS_OFFSET 6U
#define SPI_V3_STATUS_OFFSET 7U
#define SPI_V3_SESSION_OFFSET 8U
#define SPI_V3_SEQUENCE_OFFSET 12U
#define SPI_V3_ENDPOINT_OFFSET 16U
#define SPI_V3_PAYLOAD_LENGTH_OFFSET 18U
#define SPI_V3_MAX_RESPONSE_LENGTH_OFFSET 20U
#define SPI_V3_RESERVED16_OFFSET 22U
#define SPI_V3_HEADER_CRC_OFFSET 24U

// These hooks are intentionally empty in today's Panda firmware: D-cache is
// not enabled and .sram12 is already the DMA-accessible D2 SRAM section. If
// D-cache is enabled later, the RX ring should preferably be mapped as an MPU
// non-cacheable region. A cache invalidation hook is only safe when the ring is
// cache-line aligned and contains no unrelated CPU-owned data.
#ifndef SPI_V3_DMA_ACQUIRE_BARRIER
#define SPI_V3_DMA_ACQUIRE_BARRIER() do { } while (0)
#endif

#ifndef SPI_V3_DMA_INVALIDATE
#define SPI_V3_DMA_INVALIDATE(addr, len) do { (void)(addr); (void)(len); } while (0)
#endif

static const uint8_t spi_v3_magic[4] = {0x5aU, 0xc3U, 0x69U, 0x96U};

typedef enum {
  SPI_V3_FRAME_REQUEST = 1U,
  SPI_V3_FRAME_RESPONSE = 2U,
} spi_v3_frame_type_t;

typedef enum {
  SPI_V3_STATUS_OK = 0U,
  SPI_V3_STATUS_BAD_FRAME = 1U,
  SPI_V3_STATUS_UNSUPPORTED_VERSION = 2U,
  SPI_V3_STATUS_BAD_LENGTH = 3U,
  SPI_V3_STATUS_BAD_ENDPOINT = 4U,
  SPI_V3_STATUS_BUSY = 5U,
  SPI_V3_STATUS_SEQUENCE_CONFLICT = 6U,
  SPI_V3_STATUS_INTERNAL_ERROR = 7U,
} spi_v3_status_t;

typedef enum {
  SPI_V3_PARSE_NONE = 0U,
  SPI_V3_PARSE_FRAME_READY = 1U,
  SPI_V3_PARSE_INVALID_ARGUMENT = 2U,
} spi_v3_parse_result_t;

typedef enum {
  SPI_V3_REQUEST_NEW = 0U,
  SPI_V3_REQUEST_REPLAY = 1U,
  SPI_V3_REQUEST_CONFLICT = 2U,
  SPI_V3_REQUEST_INVALID = 3U,
} spi_v3_request_disposition_t;

static inline bool spi_v3_can_write_length_valid(uint16_t payload_length) {
  return (payload_length > 0U) &&
         (payload_length <= SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE);
}

typedef struct {
  uint32_t discarded_bytes;
  uint32_t bad_header_crc;
  uint32_t bad_frame_crc;
  uint32_t bad_version;
  uint32_t bad_length;
  uint32_t bad_fields;
  uint32_t buffer_overflow;
} spi_v3_parser_stats_t;

// Pointers in this view refer to the parser's internal buffer and remain valid
// until spi_v3_stream_parser_consume() or spi_v3_stream_parser_reset().
typedef struct {
  spi_v3_frame_type_t type;
  uint8_t flags;
  spi_v3_status_t status;
  uint32_t session_id;
  uint32_t sequence;
  uint8_t endpoint;
  uint16_t payload_length;
  uint16_t max_response_length;
  const uint8_t *payload;
  const uint8_t *encoded;
  uint16_t encoded_length;
} spi_v3_frame_t;

typedef struct {
  spi_v3_frame_type_t type;
  uint8_t flags;
  spi_v3_status_t status;
  uint32_t session_id;
  uint32_t sequence;
  uint8_t endpoint;
  uint16_t max_response_length;
  const uint8_t *payload;
  uint16_t payload_length;
} spi_v3_frame_fields_t;

typedef struct {
  uint8_t buffer[SPI_V3_WIRE_BUFFER_SIZE];
  uint16_t used;
  bool frame_ready;
  spi_v3_frame_t frame;
  spi_v3_parser_stats_t stats;
} spi_v3_stream_parser_t;

// The low-level DMA driver publishes exact byte deltas. Both counters are
// monotonic modulo 2^32, so wrapping is harmless while the outstanding amount
// stays below 2^31 bytes. ring_size passed to drain must be a power of two.
typedef struct {
  volatile uint32_t produced;
  volatile uint32_t consumed;
  volatile uint32_t overruns;
} spi_v3_rx_ring_t;

// Full encoded requests are retained instead of relying on a collision-prone
// digest. This makes endpoint 3 exactly-once for retransmissions within one
// Panda boot: a byte-identical duplicate replays the cached response, while a
// different request reusing (session, sequence) is rejected.
//
// Place an instance used directly by TX DMA in aligned .sram12 storage. The
// two arrays begin at 2048-byte boundaries relative to an aligned instance.
typedef struct {
  uint8_t request[SPI_V3_WIRE_BUFFER_SIZE];
  uint8_t response[SPI_V3_WIRE_BUFFER_SIZE];
  uint32_t session_id;
  uint32_t sequence;
  uint16_t request_length;
  uint16_t response_length;
  bool valid;
} spi_v3_replay_cache_t;

// Low-level H7 completion is deliberately a three-condition join. DMA TC can
// precede the last shifted byte, TXC can become visible just after NSS rises,
// and a short poll can raise NSS before DMA is done. Keeping the rule in this
// hardware-independent helper makes every interrupt ordering testable.
typedef struct {
  volatile bool active;
  volatile bool dma_done;
  volatile bool txc_done;
  volatile bool nss_done;
} spi_v3_tx_completion_t;

static inline void spi_v3_tx_completion_start(spi_v3_tx_completion_t *state) {
  if (state != NULL) {
    state->dma_done = false;
    state->txc_done = false;
    state->nss_done = false;
    state->active = true;
  }
}

static inline void spi_v3_tx_completion_note_dma(spi_v3_tx_completion_t *state,
                                                  bool nss_is_high) {
  if ((state != NULL) && state->active) {
    state->dma_done = true;
    state->nss_done = state->nss_done || nss_is_high;
  }
}

static inline void spi_v3_tx_completion_note_txc(spi_v3_tx_completion_t *state) {
  if ((state != NULL) && state->active) {
    state->txc_done = true;
  }
}

static inline void spi_v3_tx_completion_note_nss(spi_v3_tx_completion_t *state,
                                                  bool dma_reached_zero) {
  if ((state != NULL) && state->active && (state->dma_done || dma_reached_zero)) {
    state->nss_done = true;
  }
}

static inline bool spi_v3_tx_completion_ready(const spi_v3_tx_completion_t *state) {
  return (state != NULL) && state->active && state->dma_done &&
         state->txc_done && state->nss_done;
}

static inline void spi_v3_tx_completion_finish(spi_v3_tx_completion_t *state) {
  if (state != NULL) {
    state->active = false;
  }
}

static inline uint16_t spi_v3_read_u16_le(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static inline uint32_t spi_v3_read_u32_le(const uint8_t *data) {
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static inline void spi_v3_write_u16_le(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static inline void spi_v3_write_u32_le(uint8_t *data, uint32_t value) {
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

// CRC-32C (Castagnoli), reflected polynomial, conventional initial/final xor.
// The standard "123456789" check vector is 0xe3069283.
static inline uint32_t spi_v3_crc32c(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffffU;
  for (size_t i = 0U; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0x82f63b78U & mask);
    }
  }
  return ~crc;
}

static inline bool spi_v3_valid_status(spi_v3_status_t status) {
  return (uint32_t)status <= (uint32_t)SPI_V3_STATUS_INTERNAL_ERROR;
}

static inline bool spi_v3_valid_fields(const spi_v3_frame_fields_t *fields) {
  bool valid = (fields != NULL) && (fields->flags == 0U) &&
               spi_v3_valid_status(fields->status) &&
               (fields->payload_length <= SPI_V3_MAX_PAYLOAD_SIZE) &&
               (fields->max_response_length <= SPI_V3_MAX_PAYLOAD_SIZE) &&
               ((fields->payload_length == 0U) || (fields->payload != NULL));
  if (valid) {
    if (fields->type == SPI_V3_FRAME_REQUEST) {
      valid = fields->status == SPI_V3_STATUS_OK;
    } else if (fields->type == SPI_V3_FRAME_RESPONSE) {
      valid = fields->max_response_length == 0U;
    } else {
      valid = false;
    }
  }
  return valid;
}

static inline bool spi_v3_encode_frame(uint8_t *output, uint16_t output_capacity,
                                       const spi_v3_frame_fields_t *fields,
                                       uint16_t *encoded_length) {
  if ((output == NULL) || (encoded_length == NULL) || !spi_v3_valid_fields(fields)) {
    return false;
  }

  const uint16_t frame_length = (uint16_t)(SPI_V3_HEADER_SIZE + fields->payload_length +
                                            SPI_V3_TRAILER_SIZE);
  if (output_capacity < frame_length) {
    return false;
  }

  for (uint8_t i = 0U; i < 4U; ++i) {
    output[i] = spi_v3_magic[i];
  }
  output[SPI_V3_VERSION_OFFSET] = SPI_V3_VERSION;
  output[SPI_V3_TYPE_OFFSET] = (uint8_t)fields->type;
  output[SPI_V3_FLAGS_OFFSET] = fields->flags;
  output[SPI_V3_STATUS_OFFSET] = (uint8_t)fields->status;
  spi_v3_write_u32_le(&output[SPI_V3_SESSION_OFFSET], fields->session_id);
  spi_v3_write_u32_le(&output[SPI_V3_SEQUENCE_OFFSET], fields->sequence);
  output[SPI_V3_ENDPOINT_OFFSET] = fields->endpoint;
  output[SPI_V3_ENDPOINT_OFFSET + 1U] = 0U;
  spi_v3_write_u16_le(&output[SPI_V3_PAYLOAD_LENGTH_OFFSET], fields->payload_length);
  spi_v3_write_u16_le(&output[SPI_V3_MAX_RESPONSE_LENGTH_OFFSET], fields->max_response_length);
  spi_v3_write_u16_le(&output[SPI_V3_RESERVED16_OFFSET], 0U);
  spi_v3_write_u32_le(&output[SPI_V3_HEADER_CRC_OFFSET],
                      spi_v3_crc32c(output, SPI_V3_HEADER_CRC_OFFSET));

  for (uint16_t i = 0U; i < fields->payload_length; ++i) {
    output[SPI_V3_HEADER_SIZE + i] = fields->payload[i];
  }
  spi_v3_write_u32_le(&output[frame_length - SPI_V3_TRAILER_SIZE],
                      spi_v3_crc32c(output, frame_length - SPI_V3_TRAILER_SIZE));
  *encoded_length = frame_length;
  return true;
}

static inline bool spi_v3_encode_response(uint8_t *output, uint16_t output_capacity,
                                          const spi_v3_frame_t *request,
                                          spi_v3_status_t status,
                                          const uint8_t *payload, uint16_t payload_length,
                                          uint16_t *encoded_length) {
  if ((request == NULL) || (request->type != SPI_V3_FRAME_REQUEST) ||
      (request->status != SPI_V3_STATUS_OK) ||
      (payload_length > request->max_response_length)) {
    return false;
  }
  const spi_v3_frame_fields_t response = {
    .type = SPI_V3_FRAME_RESPONSE,
    .flags = 0U,
    .status = status,
    .session_id = request->session_id,
    .sequence = request->sequence,
    .endpoint = request->endpoint,
    .max_response_length = 0U,
    .payload = payload,
    .payload_length = payload_length,
  };
  return spi_v3_encode_frame(output, output_capacity, &response, encoded_length);
}

static inline void spi_v3_stream_parser_init(spi_v3_stream_parser_t *parser) {
  if (parser != NULL) {
    parser->used = 0U;
    parser->frame_ready = false;
    parser->frame = (spi_v3_frame_t){0};
    parser->stats = (spi_v3_parser_stats_t){0};
  }
}

// Reset partial framing after an RX overrun, but retain diagnostics.
static inline void spi_v3_stream_parser_reset(spi_v3_stream_parser_t *parser) {
  if (parser != NULL) {
    parser->used = 0U;
    parser->frame_ready = false;
    parser->frame = (spi_v3_frame_t){0};
  }
}

static inline void spi_v3_parser_discard_prefix(spi_v3_stream_parser_t *parser,
                                                 uint16_t count) {
  if (count > parser->used) {
    count = parser->used;
  }
  const uint16_t remaining = (uint16_t)(parser->used - count);
  for (uint16_t i = 0U; i < remaining; ++i) {
    parser->buffer[i] = parser->buffer[count + i];
  }
  parser->used = remaining;
  parser->stats.discarded_bytes += count;
}

static inline bool spi_v3_magic_at(const uint8_t *data, uint16_t offset) {
  bool matches = true;
  for (uint8_t i = 0U; i < 4U; ++i) {
    matches = matches && (data[offset + i] == spi_v3_magic[i]);
  }
  return matches;
}

static inline uint16_t spi_v3_magic_suffix_length(const uint8_t *data, uint16_t length) {
  uint16_t keep = 0U;
  const uint16_t maximum = (length < 3U) ? length : 3U;
  for (uint16_t candidate = maximum; candidate > 0U; --candidate) {
    bool matches = true;
    for (uint16_t i = 0U; i < candidate; ++i) {
      matches = matches && (data[length - candidate + i] == spi_v3_magic[i]);
    }
    if (matches) {
      keep = candidate;
      break;
    }
  }
  return keep;
}

static inline bool spi_v3_decode_header(const uint8_t *data, spi_v3_frame_t *frame,
                                        spi_v3_parser_stats_t *stats) {
  const uint32_t received_header_crc = spi_v3_read_u32_le(&data[SPI_V3_HEADER_CRC_OFFSET]);
  if (spi_v3_crc32c(data, SPI_V3_HEADER_CRC_OFFSET) != received_header_crc) {
    stats->bad_header_crc += 1U;
    return false;
  }
  if (data[SPI_V3_VERSION_OFFSET] != SPI_V3_VERSION) {
    stats->bad_version += 1U;
    return false;
  }

  const uint16_t payload_length = spi_v3_read_u16_le(&data[SPI_V3_PAYLOAD_LENGTH_OFFSET]);
  const uint16_t max_response_length =
    spi_v3_read_u16_le(&data[SPI_V3_MAX_RESPONSE_LENGTH_OFFSET]);
  if ((payload_length > SPI_V3_MAX_PAYLOAD_SIZE) ||
      (max_response_length > SPI_V3_MAX_PAYLOAD_SIZE)) {
    stats->bad_length += 1U;
    return false;
  }

  const uint8_t raw_type = data[SPI_V3_TYPE_OFFSET];
  const uint8_t raw_status = data[SPI_V3_STATUS_OFFSET];
  const bool known_type = (raw_type == (uint8_t)SPI_V3_FRAME_REQUEST) ||
                          (raw_type == (uint8_t)SPI_V3_FRAME_RESPONSE);
  const bool known_status = raw_status <= (uint8_t)SPI_V3_STATUS_INTERNAL_ERROR;
  const bool reserved_clear = (data[SPI_V3_ENDPOINT_OFFSET + 1U] == 0U) &&
                              (spi_v3_read_u16_le(&data[SPI_V3_RESERVED16_OFFSET]) == 0U);
  bool semantic_fields = false;
  if (known_type && known_status && reserved_clear && (data[SPI_V3_FLAGS_OFFSET] == 0U)) {
    semantic_fields = (raw_type == (uint8_t)SPI_V3_FRAME_REQUEST)
                        ? (raw_status == (uint8_t)SPI_V3_STATUS_OK)
                        : (max_response_length == 0U);
  }
  if (!semantic_fields) {
    stats->bad_fields += 1U;
    return false;
  }

  frame->type = (spi_v3_frame_type_t)raw_type;
  frame->flags = data[SPI_V3_FLAGS_OFFSET];
  frame->status = (spi_v3_status_t)raw_status;
  frame->session_id = spi_v3_read_u32_le(&data[SPI_V3_SESSION_OFFSET]);
  frame->sequence = spi_v3_read_u32_le(&data[SPI_V3_SEQUENCE_OFFSET]);
  frame->endpoint = data[SPI_V3_ENDPOINT_OFFSET];
  frame->payload_length = payload_length;
  frame->max_response_length = max_response_length;
  return true;
}

static inline spi_v3_parse_result_t spi_v3_stream_parser_process(spi_v3_stream_parser_t *parser) {
  while (!parser->frame_ready) {
    if (parser->used < 4U) {
      break;
    }

    uint16_t magic_offset = parser->used;
    for (uint16_t i = 0U; i <= (uint16_t)(parser->used - 4U); ++i) {
      if (spi_v3_magic_at(parser->buffer, i)) {
        magic_offset = i;
        break;
      }
    }
    if (magic_offset == parser->used) {
      const uint16_t keep = spi_v3_magic_suffix_length(parser->buffer, parser->used);
      spi_v3_parser_discard_prefix(parser, (uint16_t)(parser->used - keep));
      break;
    }
    if (magic_offset != 0U) {
      spi_v3_parser_discard_prefix(parser, magic_offset);
    }
    if (parser->used < SPI_V3_HEADER_SIZE) {
      break;
    }

    spi_v3_frame_t candidate = {0};
    if (!spi_v3_decode_header(parser->buffer, &candidate, &parser->stats)) {
      spi_v3_parser_discard_prefix(parser, 1U);
      continue;
    }

    const uint16_t frame_length =
      (uint16_t)(SPI_V3_HEADER_SIZE + candidate.payload_length + SPI_V3_TRAILER_SIZE);
    if (parser->used < frame_length) {
      break;
    }
    const uint32_t received_frame_crc =
      spi_v3_read_u32_le(&parser->buffer[frame_length - SPI_V3_TRAILER_SIZE]);
    if (spi_v3_crc32c(parser->buffer, frame_length - SPI_V3_TRAILER_SIZE) !=
        received_frame_crc) {
      parser->stats.bad_frame_crc += 1U;
      spi_v3_parser_discard_prefix(parser, 1U);
      continue;
    }

    candidate.payload = &parser->buffer[SPI_V3_HEADER_SIZE];
    candidate.encoded = parser->buffer;
    candidate.encoded_length = frame_length;
    parser->frame = candidate;
    parser->frame_ready = true;
  }
  return parser->frame_ready ? SPI_V3_PARSE_FRAME_READY : SPI_V3_PARSE_NONE;
}

// Consumes at most one complete frame. *consumed tells a circular-RX caller
// exactly how many source bytes can be retired; when a frame becomes ready,
// remaining source bytes must be passed again after consume().
static inline spi_v3_parse_result_t spi_v3_stream_parser_feed(spi_v3_stream_parser_t *parser,
                                                               const uint8_t *data, size_t length,
                                                               size_t *consumed) {
  if ((parser == NULL) || (consumed == NULL) || ((data == NULL) && (length != 0U))) {
    return SPI_V3_PARSE_INVALID_ARGUMENT;
  }
  *consumed = 0U;
  if (parser->frame_ready) {
    return SPI_V3_PARSE_FRAME_READY;
  }

  size_t i = 0U;
  while (i < length) {
    if (parser->used >= SPI_V3_WIRE_BUFFER_SIZE) {
      parser->stats.buffer_overflow += 1U;
      spi_v3_stream_parser_reset(parser);
    }

    // process() leaves one of two useful states: less than a full header, or
    // a validated header at offset zero waiting for its remaining payload.
    // In the latter case copy directly to the known frame boundary instead of
    // rescanning magic and CRC for every payload byte (quadratic at 2 KiB).
    size_t copy_length = 1U;
    if ((parser->used >= SPI_V3_HEADER_SIZE) && spi_v3_magic_at(parser->buffer, 0U)) {
      const uint16_t payload_length =
        spi_v3_read_u16_le(&parser->buffer[SPI_V3_PAYLOAD_LENGTH_OFFSET]);
      const uint16_t frame_length =
        (uint16_t)(SPI_V3_HEADER_SIZE + payload_length + SPI_V3_TRAILER_SIZE);
      const size_t needed = (size_t)frame_length - parser->used;
      const size_t available = length - i;
      copy_length = (needed < available) ? needed : available;
    }
    for (size_t j = 0U; j < copy_length; ++j) {
      parser->buffer[parser->used + j] = data[i + j];
    }
    parser->used = (uint16_t)(parser->used + copy_length);
    i += copy_length;
    *consumed = i;
    const spi_v3_parse_result_t result = spi_v3_stream_parser_process(parser);
    if (result == SPI_V3_PARSE_FRAME_READY) {
      return result;
    }
  }
  return SPI_V3_PARSE_NONE;
}

static inline const spi_v3_frame_t *spi_v3_stream_parser_frame(
    const spi_v3_stream_parser_t *parser) {
  return ((parser != NULL) && parser->frame_ready) ? &parser->frame : NULL;
}

static inline void spi_v3_stream_parser_consume(spi_v3_stream_parser_t *parser) {
  if ((parser != NULL) && parser->frame_ready) {
    // feed() stops exactly at the completed frame, so no trailing bytes have
    // entered the parser buffer yet.
    parser->used = 0U;
    parser->frame_ready = false;
    parser->frame = (spi_v3_frame_t){0};
  }
}

static inline void spi_v3_rx_ring_init(spi_v3_rx_ring_t *ring) {
  if (ring != NULL) {
    ring->produced = 0U;
    ring->consumed = 0U;
    ring->overruns = 0U;
  }
}

// byte_count must be an exact delta since the previous publish. On H7 this is
// derived from the circular DMA NDTR position at every HT, TC, and SPI EOT.
static inline void spi_v3_rx_ring_note_produced(spi_v3_rx_ring_t *ring,
                                                uint32_t byte_count) {
  if (ring != NULL) {
    ring->produced += byte_count;
  }
}

static inline bool spi_v3_is_power_of_two(uint16_t value) {
  return (value != 0U) && ((value & (uint16_t)(value - 1U)) == 0U);
}

// Circular DMA exposes only sticky half/full flags plus NDTR. These cases
// prove that at least one complete ring may be hidden, so the exact producer
// count is unknowable and the firmware must start a new RX epoch.
static inline bool spi_v3_rx_progress_ambiguous(bool half_pending, bool full_pending,
                                                 uint16_t previous_position,
                                                 uint16_t current_position) {
  return (half_pending && full_pending) ||
         (full_pending && (previous_position <= current_position));
}

static inline bool spi_v3_rx_frame_epoch_valid(uint32_t before, uint32_t after) {
  return before == after;
}

#define SPI_V3_CAN_BUS_COUNT 3U
#define SPI_V3_CAN_PACKET_HEADER_SIZE 6U
#define SPI_V3_CAN_PACKET_MAX_SIZE 70U

// Count every complete CAN packet a write would enqueue before changing the
// shared assembly buffer or a TX queue. The existing partial packet is part of
// this calculation because endpoint 3 transfers may split a packet boundary.
static inline bool spi_v3_can_batch_requirements(
    const uint8_t *partial, uint16_t partial_length, uint16_t partial_tail,
    const uint8_t *data, uint16_t length,
    uint16_t required[SPI_V3_CAN_BUS_COUNT]) {
  static const uint8_t dlc_lengths[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U,
  };
  if ((required == NULL) || ((length > 0U) && (data == NULL)) ||
      ((partial_length > 0U) && (partial == NULL)) ||
      ((partial_length == 0U) != (partial_tail == 0U))) {
    return false;
  }
  for (uint8_t bus = 0U; bus < SPI_V3_CAN_BUS_COUNT; ++bus) {
    required[bus] = 0U;
  }

  uint16_t position = 0U;
  if (partial_length > 0U) {
    const uint16_t packet_length = (uint16_t)(SPI_V3_CAN_PACKET_HEADER_SIZE +
      dlc_lengths[partial[0] >> 4U]);
    const uint8_t bus = (partial[0] >> 1U) & 0x7U;
    if ((partial_length >= SPI_V3_CAN_PACKET_MAX_SIZE) ||
        (((uint32_t)partial_length + partial_tail) != packet_length) ||
        (bus >= SPI_V3_CAN_BUS_COUNT)) {
      return false;
    }
    if (partial_tail > length) {
      return true;
    }
    required[bus] += 1U;
    position = partial_tail;
  }

  while (position < length) {
    const uint8_t first = data[position];
    const uint8_t bus = (first >> 1U) & 0x7U;
    const uint16_t packet_length = (uint16_t)(SPI_V3_CAN_PACKET_HEADER_SIZE +
      dlc_lengths[first >> 4U]);
    if (bus >= SPI_V3_CAN_BUS_COUNT) {
      return false;
    }
    const uint16_t remaining = length - position;
    if (packet_length > remaining) {
      break;
    }
    if (required[bus] == UINT16_MAX) {
      return false;
    }
    required[bus] += 1U;
    position = (uint16_t)(position + packet_length);
  }
  return true;
}

static inline bool spi_v3_can_batch_fits(
    const uint16_t required[SPI_V3_CAN_BUS_COUNT],
    const uint32_t free_slots[SPI_V3_CAN_BUS_COUNT]) {
  if ((required == NULL) || (free_slots == NULL)) {
    return false;
  }
  bool fits = true;
  for (uint8_t bus = 0U; bus < SPI_V3_CAN_BUS_COUNT; ++bus) {
    fits = fits && ((uint32_t)required[bus] <= free_slots[bus]);
  }
  return fits;
}

static inline bool spi_v3_ring_transaction_equals(const uint8_t *dma_ring,
                                                   uint16_t ring_size,
                                                   uint32_t start, uint32_t end,
                                                   const uint8_t *expected,
                                                   uint16_t expected_length) {
  if ((dma_ring == NULL) || (expected == NULL) ||
      !spi_v3_is_power_of_two(ring_size) ||
      ((end - start) != expected_length)) {
    return false;
  }
  bool matches = true;
  for (uint16_t i = 0U; (i < expected_length) && matches; ++i) {
    matches = dma_ring[(start + i) & (uint32_t)(ring_size - 1U)] == expected[i];
  }
  return matches;
}

static inline spi_v3_parse_result_t spi_v3_rx_ring_drain(spi_v3_rx_ring_t *state,
                                                          uint8_t *dma_ring,
                                                          uint16_t ring_size,
                                                          spi_v3_stream_parser_t *parser) {
  if ((state == NULL) || (dma_ring == NULL) || (parser == NULL) ||
      !spi_v3_is_power_of_two(ring_size)) {
    return SPI_V3_PARSE_INVALID_ARGUMENT;
  }
  if (parser->frame_ready) {
    return SPI_V3_PARSE_FRAME_READY;
  }

  SPI_V3_DMA_ACQUIRE_BARRIER();
  const uint32_t produced = state->produced;
  uint32_t available = produced - state->consumed;
  if (available > ring_size) {
    state->consumed = produced - ring_size;
    available = ring_size;
    state->overruns += 1U;
    spi_v3_stream_parser_reset(parser);
  }

  while (available > 0U) {
    const uint16_t offset = (uint16_t)(state->consumed & (uint32_t)(ring_size - 1U));
    uint32_t contiguous = (uint32_t)ring_size - offset;
    if (contiguous > available) {
      contiguous = available;
    }
    SPI_V3_DMA_INVALIDATE(&dma_ring[offset], contiguous);
    size_t consumed = 0U;
    const spi_v3_parse_result_t result =
      spi_v3_stream_parser_feed(parser, &dma_ring[offset], contiguous, &consumed);
    state->consumed += (uint32_t)consumed;
    available -= (uint32_t)consumed;
    if (result != SPI_V3_PARSE_NONE) {
      return result;
    }
    if (consumed == 0U) {
      break;
    }
  }
  return SPI_V3_PARSE_NONE;
}

static inline void spi_v3_replay_cache_init(spi_v3_replay_cache_t *cache) {
  if (cache != NULL) {
    cache->session_id = 0U;
    cache->sequence = 0U;
    cache->request_length = 0U;
    cache->response_length = 0U;
    cache->valid = false;
  }
}

static inline spi_v3_request_disposition_t spi_v3_replay_cache_classify(
    const spi_v3_replay_cache_t *cache, const spi_v3_frame_t *request) {
  if ((cache == NULL) || (request == NULL) ||
      (request->type != SPI_V3_FRAME_REQUEST) || (request->encoded == NULL) ||
      (request->encoded_length > SPI_V3_WIRE_BUFFER_SIZE)) {
    return SPI_V3_REQUEST_INVALID;
  }
  if (!cache->valid || (cache->session_id != request->session_id) ||
      (cache->sequence != request->sequence)) {
    return SPI_V3_REQUEST_NEW;
  }
  if (cache->request_length != request->encoded_length) {
    return SPI_V3_REQUEST_CONFLICT;
  }
  for (uint16_t i = 0U; i < request->encoded_length; ++i) {
    if (cache->request[i] != request->encoded[i]) {
      return SPI_V3_REQUEST_CONFLICT;
    }
  }
  return SPI_V3_REQUEST_REPLAY;
}

// Busy is deliberately non-terminal and must be encoded into a scratch buffer,
// not committed. Commit copies the exact request before the parser is consumed
// and builds a DMA-ready response in cache->response.
static inline bool spi_v3_replay_cache_commit(spi_v3_replay_cache_t *cache,
                                               const spi_v3_frame_t *request,
                                               spi_v3_status_t status,
                                               const uint8_t *payload,
                                               uint16_t payload_length) {
  if ((cache == NULL) || (request == NULL) || (request->encoded == NULL) ||
      (request->encoded_length > SPI_V3_WIRE_BUFFER_SIZE) ||
      (status == SPI_V3_STATUS_BUSY)) {
    return false;
  }

  uint16_t response_length = 0U;
  if (!spi_v3_encode_response(cache->response, SPI_V3_WIRE_BUFFER_SIZE, request,
                              status, payload, payload_length, &response_length)) {
    return false;
  }
  for (uint16_t i = 0U; i < request->encoded_length; ++i) {
    cache->request[i] = request->encoded[i];
  }
  cache->session_id = request->session_id;
  cache->sequence = request->sequence;
  cache->request_length = request->encoded_length;
  cache->response_length = response_length;
  cache->valid = true;
  return true;
}
