#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "selfdrive/pandad/spi_protocol_v3.h"

namespace panda::spi_v3 {

// Result codes are intentionally separate from protocol Status values. A
// non-OK Status is a valid, matching response from Panda, while these errors
// describe failures of the host transport itself.
enum class TransportError : int {
  None = 0,
  InvalidArgument = -1,
  Io = -2,
  Timeout = -3,
  ResponseMismatch = -4,
  ResponseTooLong = -5,
  RemoteError = -6,
};

struct TransferResult {
  TransportError error = TransportError::None;
  Status remote_status = Status::Ok;
  std::vector<uint8_t> payload;

  bool ok() const { return error == TransportError::None; }
};

struct TransportConfig {
  // A response can be split across any number of SPI transactions. Keeping
  // polls small bounds the time spent holding spidev for an empty response.
  size_t poll_chunk_size = 256U;
  // A complete maximum-sized response plus this much leading filler is
  // clocked per attempt. The bound prevents a missing response from filling
  // Panda's 8 KiB circular RX ring indefinitely.
  size_t max_filler_prefix_bytes = 512U;
  uint32_t poll_interval_us = 250U;
  uint32_t response_attempt_timeout_ms = 8U;
  uint32_t transfer_timeout_ms = 100U;
  uint8_t filler_byte = 0xcdU;
};

// One call clocks one SPI transaction/CS assertion. Linux SPI_IOC_MESSAGE(1)
// returns the number of messages, not the number of bytes, so every nonnegative
// return value is success.
using ClockTransfer = std::function<int(const uint8_t *tx, uint8_t *rx, size_t size)>;
using MonotonicMillis = std::function<uint64_t()>;
using PauseMicros = std::function<void(uint32_t)>;

// Stop-and-wait host transport for Panda SPI v3.
//
// Every logical request receives one (session, sequence) pair. Timeout and BUSY
// retries retransmit the byte-identical encoded request. This is what gives
// side-effecting endpoint 3 exactly-once behavior when paired with Panda's
// replay cache: a response lost on MISO does not enqueue the CAN batch twice.
class Transport {
public:
  Transport(ClockTransfer clock_transfer, uint32_t session_id,
            TransportConfig config = {}, MonotonicMillis monotonic_millis = {},
            PauseMicros pause_micros = {});

  TransferResult transfer(uint8_t endpoint, const uint8_t *tx_data, uint16_t tx_length,
                          uint16_t max_response_length, unsigned int timeout_ms = 0U);

  uint32_t session_id() const { return session_id_; }
  uint32_t next_sequence() const { return next_sequence_; }

private:
  enum class ResponseAction {
    Continue,
    Retry,
    Complete,
  };

  ResponseAction inspect_frames(const std::vector<Frame> &frames, const Frame &request,
                                TransferResult &result) const;
  bool clock_and_decode(const uint8_t *tx, size_t size, StreamDecoder &decoder,
                        std::vector<Frame> &frames);

  ClockTransfer clock_transfer_;
  MonotonicMillis monotonic_millis_;
  PauseMicros pause_micros_;
  TransportConfig config_;
  uint32_t session_id_;
  uint32_t next_sequence_ = 1U;
};

}  // namespace panda::spi_v3
