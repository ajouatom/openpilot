#define CATCH_CONFIG_MAIN

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <utility>
#include <vector>

#include "catch2/catch.hpp"
#include "selfdrive/pandad/spi_protocol_v3.h"
#include "selfdrive/pandad/spi_v3_transport.h"

using namespace panda::spi_v3;

namespace {

Frame response_for(const Frame &request, Status status, std::vector<uint8_t> payload = {}) {
  Frame response;
  response.type = FrameType::Response;
  response.status = status;
  response.session_id = request.session_id;
  response.sequence = request.sequence;
  response.endpoint = request.endpoint;
  response.payload = std::move(payload);
  return response;
}

class FakeLink {
public:
  using RequestHandler = std::function<void(const Frame &, const std::vector<uint8_t> &)>;

  explicit FakeLink(RequestHandler handler) : handler_(std::move(handler)) {}

  int clock(const uint8_t *tx, uint8_t *rx, size_t size) {
    std::fill(rx, rx + size, static_cast<uint8_t>(0xcdU));

    // Bytes queued by a previous CS assertion appear before any response that
    // processing the current MOSI transaction may generate.
    for (size_t i = 0U; i < size && !miso_.empty(); ++i) {
      rx[i] = miso_.front();
      miso_.pop_front();
    }

    for (const Frame &frame : request_decoder_.feed(tx, size)) {
      if (frame.type == FrameType::Request) {
        handler_(frame, encode_frame(frame));
      }
    }
    return 1;  // SPI_IOC_MESSAGE(1) success convention
  }

  void queue(const std::vector<uint8_t> &bytes, size_t filler_prefix = 0U) {
    miso_.insert(miso_.end(), filler_prefix, static_cast<uint8_t>(0xcdU));
    miso_.insert(miso_.end(), bytes.begin(), bytes.end());
  }

private:
  RequestHandler handler_;
  StreamDecoder request_decoder_;
  std::deque<uint8_t> miso_;
};

TransportConfig fast_config() {
  TransportConfig config;
  config.poll_chunk_size = 17U;
  config.max_filler_prefix_bytes = 64U;
  config.poll_interval_us = 0U;
  config.response_attempt_timeout_ms = 3U;
  config.transfer_timeout_ms = 40U;
  return config;
}

}  // namespace

TEST_CASE("SPI v3 transport sends one request frame and scans a split filler response") {
  std::vector<Frame> requests;
  FakeLink *link_ptr = nullptr;
  FakeLink link([&](const Frame &request, const std::vector<uint8_t> &) {
    requests.push_back(request);
    link_ptr->queue(encode_frame(response_for(request, Status::Ok,
                                              {0x10U, 0xcdU, 0x20U})), 23U);
  });
  link_ptr = &link;

  Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
    return link.clock(tx, rx, size);
  }, 0x12345678U, fast_config());

  const std::vector<uint8_t> payload = {0xaaU, 0xbbU, 0xccU};
  const TransferResult result = transport.transfer(
    0x81U, payload.data(), static_cast<uint16_t>(payload.size()), 8U);

  REQUIRE(result.ok());
  REQUIRE((result.payload == std::vector<uint8_t>{0x10U, 0xcdU, 0x20U}));
  REQUIRE(requests.size() == 1U);
  REQUIRE(requests[0].session_id == 0x12345678U);
  REQUIRE(requests[0].sequence == 1U);
  REQUIRE(requests[0].endpoint == 0x81U);
  REQUIRE(requests[0].max_response_length == 8U);
  REQUIRE(requests[0].payload == payload);
  REQUIRE(transport.next_sequence() == 2U);
}

TEST_CASE("SPI v3 timeout retransmits byte-identical endpoint 3 request exactly once") {
  uint64_t now = 0U;
  int executions = 0;
  int request_count = 0;
  std::vector<std::vector<uint8_t>> encoded_requests;
  RequestReplayCache replay_cache;
  FakeLink *link_ptr = nullptr;
  FakeLink link([&](const Frame &request, const std::vector<uint8_t> &encoded) {
    ++request_count;
    encoded_requests.push_back(encoded);
    switch (replay_cache.classify(request)) {
      case RequestDisposition::New: {
        ++executions;
        const Frame response = response_for(request, Status::Ok);
        replay_cache.remember(request, response);
        // Model a lost first MISO response after Panda has already enqueued the
        // CAN batch. The host must retry, not create a new logical request.
        break;
      }
      case RequestDisposition::Replay:
        link_ptr->queue(replay_cache.encoded_response());
        break;
      default:
        FAIL("unexpected replay-cache disposition");
    }
  });
  link_ptr = &link;

  TransportConfig config = fast_config();
  config.poll_chunk_size = 64U;
  config.max_filler_prefix_bytes = 0U;
  Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
    return link.clock(tx, rx, size);
  }, 0xabcdef01U, config, [&]() { return now++; }, [](uint32_t) {});

  const std::vector<uint8_t> can_batch = {0x01U, 0x02U, 0x03U, 0x04U};
  const TransferResult result = transport.transfer(
    3U, can_batch.data(), static_cast<uint16_t>(can_batch.size()), 0U);

  REQUIRE(result.ok());
  REQUIRE(executions == 1);
  REQUIRE(request_count == 2);
  REQUIRE(encoded_requests[0] == encoded_requests[1]);
}

TEST_CASE("SPI v3 BUSY is nonterminal and retries the same sequence") {
  std::vector<std::vector<uint8_t>> encoded_requests;
  FakeLink *link_ptr = nullptr;
  FakeLink link([&](const Frame &request, const std::vector<uint8_t> &encoded) {
    encoded_requests.push_back(encoded);
    const Status status = encoded_requests.size() == 1U ? Status::Busy : Status::Ok;
    link_ptr->queue(encode_frame(response_for(request, status)));
  });
  link_ptr = &link;

  Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
    return link.clock(tx, rx, size);
  }, 77U, fast_config());
  const TransferResult result = transport.transfer(3U, nullptr, 0U, 0U);

  REQUIRE(result.ok());
  REQUIRE(encoded_requests.size() == 2U);
  REQUIRE(encoded_requests[0] == encoded_requests[1]);
}

TEST_CASE("SPI v3 transport rejects matching oversized and mismatched responses") {
  SECTION("response exceeds request limit") {
    FakeLink *link_ptr = nullptr;
    FakeLink link([&](const Frame &request, const std::vector<uint8_t> &) {
      link_ptr->queue(encode_frame(response_for(request, Status::Ok,
                                                {0x01U, 0x02U, 0x03U})));
    });
    link_ptr = &link;
    Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
      return link.clock(tx, rx, size);
    }, 1U, fast_config());

    const TransferResult result = transport.transfer(0x81U, nullptr, 0U, 2U);
    REQUIRE(result.error == TransportError::ResponseTooLong);
  }

  SECTION("matching sequence has wrong endpoint") {
    FakeLink *link_ptr = nullptr;
    FakeLink link([&](const Frame &request, const std::vector<uint8_t> &) {
      Frame response = response_for(request, Status::Ok);
      response.endpoint ^= 1U;
      link_ptr->queue(encode_frame(response));
    });
    link_ptr = &link;
    Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
      return link.clock(tx, rx, size);
    }, 2U, fast_config());

    const TransferResult result = transport.transfer(0x81U, nullptr, 0U, 2U);
    REQUIRE(result.error == TransportError::ResponseMismatch);
  }
}

TEST_CASE("SPI v3 transport ignores stale responses and reports terminal status") {
  FakeLink *link_ptr = nullptr;
  FakeLink link([&](const Frame &request, const std::vector<uint8_t> &) {
    Frame stale = response_for(request, Status::Ok, {0x55U});
    --stale.sequence;
    link_ptr->queue(encode_frame(stale));
    link_ptr->queue(encode_frame(response_for(request, Status::BadEndpoint)));
  });
  link_ptr = &link;
  Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
    return link.clock(tx, rx, size);
  }, 3U, fast_config());

  const TransferResult result = transport.transfer(0U, nullptr, 0U, 4U);
  REQUIRE(result.error == TransportError::RemoteError);
  REQUIRE(result.remote_status == Status::BadEndpoint);
}

TEST_CASE("SPI v3 transport validates arguments and propagates ioctl failure") {
  Transport transport([](const uint8_t *, uint8_t *, size_t) { return -1; },
                      4U, fast_config());
  const uint8_t byte = 0U;
  const std::vector<uint8_t> oversized_can_write(kCanWriteMaxPayloadSize + 1U, 0U);

  REQUIRE(transport.transfer(0U, nullptr, 1U, 0U).error == TransportError::InvalidArgument);
  REQUIRE(transport.transfer(3U, oversized_can_write.data(),
                             static_cast<uint16_t>(oversized_can_write.size()), 0U).error ==
          TransportError::InvalidArgument);
  REQUIRE(transport.transfer(2U, oversized_can_write.data(),
                             static_cast<uint16_t>(oversized_can_write.size()), 0U).error ==
          TransportError::Io);
  REQUIRE(transport.transfer(0U, &byte, 1U, 0U).error == TransportError::Io);
}

TEST_CASE("SPI v3 timeout is bounded even when the injected clock does not advance") {
  int request_count = 0;
  FakeLink link([&](const Frame &, const std::vector<uint8_t> &) {
    ++request_count;
  });
  TransportConfig config = fast_config();
  config.poll_chunk_size = 64U;
  config.max_filler_prefix_bytes = 0U;
  config.response_attempt_timeout_ms = 4U;
  config.transfer_timeout_ms = 16U;
  Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
    return link.clock(tx, rx, size);
  }, 5U, config, []() { return 0U; }, [](uint32_t) {});

  const TransferResult result = transport.transfer(0U, nullptr, 0U, 0U);
  REQUIRE(result.error == TransportError::Timeout);
  REQUIRE(request_count == 4);
}

TEST_CASE("SPI v3 decoder survives stale response plus current response across retry") {
  std::vector<std::vector<uint8_t>> second_request_attempts;
  FakeLink *link_ptr = nullptr;
  FakeLink link([&](const Frame &request, const std::vector<uint8_t> &encoded) {
    const std::vector<uint8_t> payload(64U, static_cast<uint8_t>(request.sequence));
    const auto response = encode_frame(response_for(request, Status::Ok, payload));
    if (request.sequence == 1U) {
      // Leave one full duplicate response on MISO after request 1 completes.
      link_ptr->queue(response);
      link_ptr->queue(response);
    } else {
      second_request_attempts.push_back(encoded);
      link_ptr->queue(response);
    }
  });
  link_ptr = &link;

  TransportConfig config = fast_config();
  config.poll_chunk_size = 96U;  // exactly one 64-byte-payload response frame
  config.max_filler_prefix_bytes = 0U;
  config.poll_interval_us = 0U;
  config.response_attempt_timeout_ms = 1U;
  Transport transport([&](const uint8_t *tx, uint8_t *rx, size_t size) {
    return link.clock(tx, rx, size);
  }, 6U, config);

  const TransferResult first = transport.transfer(0x81U, nullptr, 0U, 64U);
  const TransferResult second = transport.transfer(0x81U, nullptr, 0U, 64U);

  REQUIRE(first.ok());
  REQUIRE(second.ok());
  REQUIRE(second.payload == std::vector<uint8_t>(64U, 2U));
  REQUIRE(second_request_attempts.size() == 2U);
  REQUIRE(second_request_attempts[0] == second_request_attempts[1]);
}
