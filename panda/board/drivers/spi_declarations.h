#pragma once

#include "crc.h"

#define SPI_TIMEOUT_US 10000U

// got max rate from hitting a non-existent endpoint
// in a tight loop, plus some buffer
#define SPI_IRQ_RATE  16000U

#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
#define SPI_V3_RX_RING_SIZE 8192U
#else
  #ifdef STM32H7
  #define SPI_BUF_SIZE 2048U
  // H7 DMA2 located in D2 domain, so we need to use SRAM1/SRAM2
  __attribute__((section(".sram12"))) extern uint8_t spi_buf_rx[SPI_BUF_SIZE];
  __attribute__((section(".sram12"))) extern uint8_t spi_buf_tx[SPI_BUF_SIZE];
  #else
#define SPI_BUF_SIZE 1024U
extern uint8_t spi_buf_rx[SPI_BUF_SIZE];
extern uint8_t spi_buf_tx[SPI_BUF_SIZE];
  #endif
#endif

#define SPI_CHECKSUM_START 0xABU
#define SPI_SYNC_BYTE 0x5AU
#define SPI_HACK 0x79U
#define SPI_DACK 0x85U
#define SPI_NACK 0x1FU

// SPI states
enum {
  SPI_STATE_HEADER,
  SPI_STATE_HEADER_ACK,
  SPI_STATE_HEADER_NACK,
  SPI_STATE_DATA_RX,
  SPI_STATE_DATA_RX_ACK,
  SPI_STATE_DATA_TX
};

extern uint16_t spi_checksum_error_count;

#define SPI_HEADER_SIZE 7U

// Low-level SPI prototypes. F4 and every bootstub keep protocol v2. The H7
// application uses a continuously armed circular RX DMA and self-framed v3.
void llspi_init(void);
#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
bool llspi_v3_send(const uint8_t *data, uint16_t length);
bool llspi_v3_tx_idle(void);
bool llspi_v3_take_version_request(void);
uint32_t llspi_v3_rx_begin(bool *was_reset);
uint32_t llspi_v3_rx_epoch(void);
#else
void llspi_mosi_dma(uint8_t *addr, int len);
void llspi_miso_dma(uint8_t *addr, int len);
#endif

void can_tx_comms_resume_spi(void);
#if defined(ENABLE_SPI) || defined(BOOTSTUB)
void spi_init(void);
#if defined(STM32H7) && defined(ENABLE_SPI) && !defined(BOOTSTUB)
void spi_process(void);
#else
void spi_rx_done(void);
void spi_tx_done(bool reset);
#endif
#endif
