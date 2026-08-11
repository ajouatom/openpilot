#include "selfdrive/pandad/spi_v3_transport.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace panda::spi_v3 {
namespace {

uint64_t steady_millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

Transport::Transport(ClockTransfer clock_transfer, uint32_t session_id,
                     TransportConfig config, MonotonicMillis monotonic_millis,
                     PauseMicros pause_micros)
    : clock_transfer_(std::move(clock_transfer)),
      monotonic_millis_(std::move(monotonic_millis)),
      pause_micros_(std::move(pause_micros)),
      config_(config),
      session_id_(session_id) {
  if (!clock_transfer_ || config_.poll_chunk_size == 0U ||
      config_.poll_chunk_size > kWireBufferSize ||
      config_.max_filler_prefix_bytes > kWireBufferSize ||
      config_.response_attempt_timeout_ms == 0U || config_.transfer_timeout_ms == 0U) {
    throw std::invalid_argument("invalid SPI v3 transport configuration");
  }
  if (!monotonic_millis_) {
    monotonic_millis_ = steady_millis;
  }
  if (!pause_micros_) {
    pause_micros_ = [](uint32_t delay_us) {
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    };
  }
}

Transport::ResponseAction Transport::inspect_frames(const std::vector<Frame> &frames,
                                                    const Frame &request,
                                                    TransferResult &result) const {
  for (const Frame &frame : frames) {
    if (frame.type != FrameType::Response || frame.session_id != request.session_id ||
        frame.sequence != request.sequence) {
      // A delayed response from an earlier request/session is harmless. The
      // self-framed stream lets us consume it and keep looking for our reply.
      continue;
    }

    if (frame.endpoint != request.endpoint) {
      result.error = TransportError::ResponseMismatch;
      return ResponseAction::Complete;
    }
    if (frame.payload.size() > request.max_response_length) {
      result.error = TransportError::ResponseTooLong;
      return ResponseAction::Complete;
    }
    if (frame.status == Status::Busy) {
      return ResponseAction::Retry;
    }
    if (frame.status != Status::Ok) {
      result.error = TransportError::RemoteError;
      result.remote_status = frame.status;
      return ResponseAction::Complete;
    }

    result.payload = frame.payload;
    return ResponseAction::Complete;
  }
  return ResponseAction::Continue;
}

bool Transport::clock_and_decode(const uint8_t *tx, size_t size, StreamDecoder &decoder,
                                 std::vector<Frame> &frames) {
  std::vector<uint8_t> rx(size, config_.filler_byte);
  if (clock_transfer_(tx, rx.data(), size) < 0) {
    return false;
  }
  frames = decoder.feed(rx);
  return true;
}

TransferResult Transport::transfer(uint8_t endpoint, const uint8_t *tx_data,
                                   uint16_t tx_length, uint16_t max_response_length,
                                   unsigned int timeout_ms) {
  TransferResult result;
  if ((tx_length != 0U && tx_data == nullptr) || tx_length > kMaxPayloadSize ||
      (endpoint == 3U && tx_length > kCanWriteMaxPayloadSize) ||
      max_response_length > kMaxPayloadSize) {
    result.error = TransportError::InvalidArgument;
    return result;
  }

  Frame request;
  request.type = FrameType::Request;
  request.status = Status::Ok;
  request.session_id = session_id_;
  request.sequence = next_sequence_++;
  request.endpoint = endpoint;
  request.max_response_length = max_response_length;
  if (tx_length != 0U) {
    request.payload.assign(tx_data, tx_data + tx_length);
  }

  std::vector<uint8_t> encoded_request;
  try {
    encoded_request = encode_frame(request);
  } catch (const std::invalid_argument &) {
    result.error = TransportError::InvalidArgument;
    return result;
  }

  // Preserve pandad's existing bounded transfer behavior. A caller-provided
  // timeout may extend it, but a USB-style 5 ms endpoint timeout must not make
  // a single lost response drop a sendcan batch.
  const uint64_t overall_timeout = std::max<uint64_t>(config_.transfer_timeout_ms, timeout_ms);
  const uint64_t start = monotonic_millis_();
  const uint64_t deadline = start + overall_timeout;
  const uint64_t maximum_attempts = std::max<uint64_t>(
    1U, (overall_timeout + config_.response_attempt_timeout_ms - 1U) /
          config_.response_attempt_timeout_ms);
  StreamDecoder decoder;
  std::vector<uint8_t> filler(config_.poll_chunk_size, config_.filler_byte);

  for (uint64_t attempt = 0U; attempt < maximum_attempts; ++attempt) {
    if (attempt != 0U && monotonic_millis_() >= deadline) {
      break;
    }
    const uint64_t attempt_start = monotonic_millis_();

    // A request is always one complete frame under one CS assertion. Feed MISO
    // from this transaction too: after a timeout it can contain the tail of the
    // previously generated, still-valid response.
    std::vector<Frame> frames;
    if (!clock_and_decode(encoded_request.data(), encoded_request.size(), decoder, frames)) {
      result.error = TransportError::Io;
      return result;
    }
    ResponseAction action = inspect_frames(frames, request, result);
    if (action == ResponseAction::Complete) {
      return result;
    }

    const size_t response_scan_bytes = kHeaderSize + kTrailerSize +
                                       max_response_length + config_.max_filler_prefix_bytes;
    const size_t maximum_polls = std::max<size_t>(
      1U, (response_scan_bytes + config_.poll_chunk_size - 1U) / config_.poll_chunk_size);
    for (size_t poll = 0U; action == ResponseAction::Continue && poll < maximum_polls; ++poll) {
      if (monotonic_millis_() >= deadline) {
        break;
      }
      if (!clock_and_decode(filler.data(), filler.size(), decoder, frames)) {
        result.error = TransportError::Io;
        return result;
      }
      action = inspect_frames(frames, request, result);
      if (action == ResponseAction::Continue && poll + 1U < maximum_polls &&
          config_.poll_interval_us != 0U) {
        pause_micros_(config_.poll_interval_us);
      }
    }

    if (action == ResponseAction::Complete) {
      return result;
    }
    const uint64_t now = monotonic_millis_();
    if (now >= deadline) {
      break;
    }
    if (attempt + 1U >= maximum_attempts) {
      break;
    }

    // BUSY and an attempt timeout both retry the exact same encoded bytes.
    // Pace those retries to keep request+filler traffic below the capacity of
    // Panda's circular RX ring even if its main loop is briefly delayed.
    const uint64_t next_attempt = std::min<uint64_t>(
      deadline, attempt_start + config_.response_attempt_timeout_ms);
    if (now < next_attempt) {
      pause_micros_(static_cast<uint32_t>((next_attempt - now) * 1000U));
    }
  }

  result.error = TransportError::Timeout;
  return result;
}

}  // namespace panda::spi_v3
