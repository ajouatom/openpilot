/*
  CAN transactions to and from the host come in the form of
  a certain number of CANPacket_t. The transaction is split
  into multiple transfers or chunks.

  * comms_can_read outputs this buffer in chunks of a specified length.
    chunks are always the given length, except the last one.
  * comms_can_write reads in this buffer in chunks.
  * both functions maintain an overflow buffer for a partial CANPacket_t that
    spans multiple transfers/chunks.
  * the overflow buffers are reset by a dedicated control transfer handler,
    which is sent by the host on each start of a connection.
*/

typedef struct {
  uint32_t ptr;
  uint32_t tail_size;
  uint8_t data[72];
} asm_buffer;

static asm_buffer can_read_buffer = {.ptr = 0U, .tail_size = 0U};

int comms_can_read(uint8_t *data, uint32_t max_len) {
  uint32_t pos = 0U;

  // Send tail of previous message if it is in buffer
  if (can_read_buffer.ptr > 0U) {
    uint32_t overflow_len = MIN(max_len - pos, can_read_buffer.ptr);
    (void)memcpy(&data[pos], can_read_buffer.data, overflow_len);
    pos += overflow_len;
    (void)memcpy(can_read_buffer.data, &can_read_buffer.data[overflow_len], can_read_buffer.ptr - overflow_len);
    can_read_buffer.ptr -= overflow_len;
  }

  if (can_read_buffer.ptr == 0U) {
    // Fill rest of buffer with new data
    CANPacket_t can_packet;
    while ((pos < max_len) && can_pop(&can_rx_q, &can_packet)) {
      uint32_t pckt_len = CANPACKET_HEAD_SIZE + dlc_to_len[can_packet.data_len_code];
      if ((pos + pckt_len) <= max_len) {
        (void)memcpy(&data[pos], (uint8_t*)&can_packet, pckt_len);
        pos += pckt_len;
      } else {
        (void)memcpy(&data[pos], (uint8_t*)&can_packet, max_len - pos);
        can_read_buffer.ptr += pckt_len - (max_len - pos);
        // cppcheck-suppress objectIndex
        (void)memcpy(can_read_buffer.data, &((uint8_t*)&can_packet)[(max_len - pos)], can_read_buffer.ptr);
        pos = max_len;
      }
    }
  }

  return pos;
}

static asm_buffer can_write_buffer = {.ptr = 0U, .tail_size = 0U};
#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
// USB and SPI v3 can be serviced from different contexts. Keep their partial
// CAN packet assembly independent so an optional USB host cannot alter the
// bytes that SPI preflighted before its main-loop dispatch.
static asm_buffer spi_v3_can_write_buffer = {.ptr = 0U, .tail_size = 0U};
#endif

// send on CAN
static void comms_can_write_internal(asm_buffer *write_buffer,
                                     const uint8_t *data, uint32_t len,
                                     bool use_reservations) {
  uint32_t pos = 0U;

  // Assembling can message with data from buffer
  if (write_buffer->ptr != 0U) {
    if (write_buffer->tail_size <= (len - pos)) {
      // we have enough data to complete the buffer
      CANPacket_t to_push = {0};
      (void)memcpy(&write_buffer->data[write_buffer->ptr], &data[pos], write_buffer->tail_size);
      write_buffer->ptr += write_buffer->tail_size;
      pos += write_buffer->tail_size;

      // send out
      (void)memcpy((uint8_t*)&to_push, write_buffer->data, write_buffer->ptr);
      #if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
        if (use_reservations) {
          can_send_reserved(&to_push, to_push.bus, false);
        } else {
          can_send(&to_push, to_push.bus, false);
        }
      #else
        (void)use_reservations;
        can_send(&to_push, to_push.bus, false);
      #endif

      // reset overflow buffer
      write_buffer->ptr = 0U;
      write_buffer->tail_size = 0U;
    } else {
      // maybe next time
      uint32_t data_size = len - pos;
      (void) memcpy(&write_buffer->data[write_buffer->ptr], &data[pos], data_size);
      write_buffer->tail_size -= data_size;
      write_buffer->ptr += data_size;
      pos += data_size;
    }
  }

  // rest of the message
  while (pos < len) {
    uint32_t pckt_len = CANPACKET_HEAD_SIZE + dlc_to_len[(data[pos] >> 4U)];
    if ((pos + pckt_len) <= len) {
      CANPacket_t to_push = {0};
      (void)memcpy((uint8_t*)&to_push, &data[pos], pckt_len);
      #if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
        if (use_reservations) {
          can_send_reserved(&to_push, to_push.bus, false);
        } else {
          can_send(&to_push, to_push.bus, false);
        }
      #else
        (void)use_reservations;
        can_send(&to_push, to_push.bus, false);
      #endif
      pos += pckt_len;
    } else {
      (void)memcpy(write_buffer->data, &data[pos], len - pos);
      write_buffer->ptr = len - pos;
      write_buffer->tail_size = pckt_len - write_buffer->ptr;
      pos += write_buffer->ptr;
    }
  }

  refresh_can_tx_slots_available();
}

void comms_can_write(const uint8_t *data, uint32_t len) {
  comms_can_write_internal(&can_write_buffer, data, len, false);
}

#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
comms_can_write_result_t comms_can_write_v3(const uint8_t *data, uint32_t len) {
  COMPILE_TIME_ASSERT(SPI_V3_CAN_BUS_COUNT == CAN_QUEUES_ARRAY_SIZE);
  uint16_t required[SPI_V3_CAN_BUS_COUNT] = {0U};
  if ((len > UINT16_MAX) ||
      !spi_v3_can_batch_requirements(spi_v3_can_write_buffer.data,
                                     (uint16_t)spi_v3_can_write_buffer.ptr,
                                     (uint16_t)spi_v3_can_write_buffer.tail_size,
                                     data, (uint16_t)len, required)) {
    return COMMS_CAN_WRITE_INVALID;
  }
  if (!can_tx_reserve_slots(required)) {
    return COMMS_CAN_WRITE_BUSY;
  }

  // Reservations prevent concurrent forwarding or USB writers from consuming
  // this batch's queue capacity. Per-transport assembly buffers protect its
  // partial packet state without holding interrupts off for the whole batch.
  comms_can_write_internal(&spi_v3_can_write_buffer, data, len, true);
  return COMMS_CAN_WRITE_OK;
}
#endif

void comms_can_reset(void) {
  can_write_buffer.ptr = 0U;
  can_write_buffer.tail_size = 0U;
#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
  spi_v3_can_write_buffer.ptr = 0U;
  spi_v3_can_write_buffer.tail_size = 0U;
#endif
  can_read_buffer.ptr = 0U;
  can_read_buffer.tail_size = 0U;
}

// TODO: make this more general!
void refresh_can_tx_slots_available(void) {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_USB_BULK_TRANSFER)) {
    can_tx_comms_resume_usb();
  }
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_SPI_BULK_TRANSFER)) {
    can_tx_comms_resume_spi();
  }
}
