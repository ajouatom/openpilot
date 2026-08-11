#if defined(ENABLE_SPI) && !defined(BOOTSTUB)

// SPI v3 keeps RX DMA and SPI4 enabled for the lifetime of the application.
// Request boundaries no longer depend on an interrupt re-arming a short DMA,
// eliminating the header/data race in protocol v2.
static uint16_t spi_v3_rx_dma_position = 0U;
static uint32_t spi_v3_last_nss_produced = 0U;
static volatile bool spi_v3_version_request_pending = false;
static volatile bool spi_v3_rx_reset_pending = false;
static volatile bool spi_v3_rx_recovery_pending = false;
static volatile uint32_t spi_v3_rx_dma_epoch = 0U;

static spi_v3_tx_completion_t spi_v3_tx_completion = {0};

// These waits cover many AHB/APB cycles while remaining bounded in an IRQ.
// A stream normally clears EN after its current bus beat, and two consecutive
// NDTR reads normally stabilize well inside these limits.
#define SPI_V3_DMA_DISABLE_SPIN_LIMIT 1024U
#define SPI_V3_NDTR_STABLE_READ_LIMIT 32U

static bool llspi_v3_dma_disable(volatile uint32_t *stream_cr) {
  *stream_cr &= ~DMA_SxCR_EN;
  for (uint16_t spin = 0U; spin < SPI_V3_DMA_DISABLE_SPIN_LIMIT; ++spin) {
    if ((*stream_cr & DMA_SxCR_EN) == 0U) {
      return true;
    }
  }
  return false;
}

static void llspi_v3_note_transport_error(void) {
  if (spi_checksum_error_count < UINT16_MAX) {
    spi_checksum_error_count += 1U;
  }
}

static void llspi_v3_mark_rx_epoch_reset(void) {
  if (!spi_v3_rx_recovery_pending) {
    const uint32_t aligned_produced =
      (spi_v3_rx_ring.produced + (SPI_V3_RX_RING_SIZE - 1U)) &
      ~(SPI_V3_RX_RING_SIZE - 1U);
    spi_v3_rx_ring.produced = aligned_produced;
    spi_v3_rx_ring.overruns += 1U;
    spi_v3_rx_dma_position = 0U;
    spi_v3_last_nss_produced = aligned_produced;
    spi_v3_rx_reset_pending = true;
    spi_v3_rx_recovery_pending = true;
    spi_v3_rx_dma_epoch += 1U;
  }
}

static bool llspi_v3_rx_dma_restart(void) {
  register_clear_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);
  const bool disabled = llspi_v3_dma_disable(&(DMA2_Stream2->CR));
  llspi_v3_mark_rx_epoch_reset();
  if (!disabled) {
    // Leave RXDMA disconnected and retry the bounded recovery from the main
    // service loop. Never write M0AR/NDTR while hardware still owns them.
    return false;
  }

  DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
                 DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
  register_set(&(DMA2_Stream2->M0AR), (uint32_t)spi_v3_dma_rx_ring, 0xFFFFFFFFU);
  DMA2_Stream2->NDTR = SPI_V3_RX_RING_SIZE;
  __DMB();
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  register_set_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);
  spi_v3_rx_recovery_pending = false;
  return true;
}

// Publish an exact monotonic byte delta from circular DMA. Reading and
// clearing HT/TC flags here also makes an NSS snapshot and the DMA IRQ safe in
// either ordering. A pending TC disambiguates a complete wrap when NDTR alone
// returns to the same position.
static void llspi_v3_rx_snapshot(void) {
  const uint32_t dma_flags = DMA2->LISR;
  const bool transfer_error =
    (dma_flags & (DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_TEIF2)) != 0U;
  if (transfer_error) {
    (void)llspi_v3_rx_dma_restart();
    return;
  }

  uint32_t ndtr_first = DMA2_Stream2->NDTR;
  uint32_t ndtr_second = DMA2_Stream2->NDTR;
  uint8_t stable_reads = 0U;
  while ((ndtr_first != ndtr_second) &&
         (stable_reads < SPI_V3_NDTR_STABLE_READ_LIMIT)) {
    ndtr_first = ndtr_second;
    ndtr_second = DMA2_Stream2->NDTR;
    stable_reads += 1U;
  }
  if (ndtr_first != ndtr_second) {
    // Continuous progress prevented an atomic producer snapshot. Dropping the
    // epoch is safer than publishing a guessed byte count.
    (void)llspi_v3_rx_dma_restart();
    return;
  }
  const uint16_t position =
    (uint16_t)((SPI_V3_RX_RING_SIZE - ndtr_second) & (SPI_V3_RX_RING_SIZE - 1U));
  const bool ambiguous_wrap = spi_v3_rx_progress_ambiguous(
    (dma_flags & DMA_LISR_HTIF2) != 0U,
    (dma_flags & DMA_LISR_TCIF2) != 0U,
    spi_v3_rx_dma_position, position);
  if (ambiguous_wrap) {
    // At 50 MHz an 8 KiB ring spans 1.31 ms. Panda's measured worst blocking
    // IRQs are below 200 us; observing both half/full flags (or an otherwise
    // indistinguishable whole wrap) violates that invariant. Exact byte count
    // is no longer recoverable from sticky flags, so drop the epoch and retry.
    (void)llspi_v3_rx_dma_restart();
    return;
  }
  uint32_t delta =
    ((uint32_t)position - spi_v3_rx_dma_position) & (SPI_V3_RX_RING_SIZE - 1U);
  if (delta > 0U) {
    __DMB();
    spi_v3_rx_ring_note_produced(&spi_v3_rx_ring, delta);
    spi_v3_rx_dma_position = position;
  }
  DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
                 DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
}

static bool llspi_v3_exact_version_transaction(uint32_t start, uint32_t end) {
  __DMB();
  return spi_v3_ring_transaction_equals(spi_v3_dma_rx_ring,
                                        SPI_V3_RX_RING_SIZE, start, end,
                                        version_text, 7U);
}

static void llspi_v3_wait_rx_quiescent(void) {
  // NSS can reach EXTI a few APB cycles before the final RX DMA write updates
  // NDTR. Wait at most ~1 us at 240 MHz for the FIFO to drain. This bounded
  // hardware handoff is needed for the exact 7-byte VERSION transaction; no
  // endpoint or other blocking work runs in the interrupt.
  for (uint16_t spin = 0U;
       (spin < 256U) && ((SPI4->SR & (SPI_SR_RXP | SPI_SR_RXPLVL)) != 0U);
       ++spin) {
    __NOP();
  }
  __DMB();
}

static void llspi_v3_tx_try_complete(void) {
  // Require NSS to be high *now*, not merely an earlier short poll edge. This
  // prevents stale nss_done from completing while a continuation transaction
  // is still shifting the tail left in the SPI FIFO.
  if (spi_v3_tx_completion_ready(&spi_v3_tx_completion) &&
      ((GPIOE->IDR & (1U << 11U)) != 0U)) {
    register_clear_bits(&(SPI4->IER), SPI_IER_EOTIE);
    register_clear_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
    spi_v3_tx_completion_finish(&spi_v3_tx_completion);
  }
}

bool llspi_v3_send(const uint8_t *data, uint16_t length) {
  if ((data == NULL) || (length == 0U) || (length > SPI_V3_WIRE_BUFFER_SIZE)) {
    return false;
  }

  bool queued = false;
  ENTER_CRITICAL();
  if (!spi_v3_tx_completion.active) {
    register_clear_bits(&(SPI4->IER), SPI_IER_EOTIE);
    register_clear_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
    if (!llspi_v3_dma_disable(&(DMA2_Stream3->CR))) {
      // Do not touch the live stream registers. A byte-identical host retry
      // can queue after hardware eventually releases EN.
      llspi_v3_note_transport_error();
    } else {
      DMA2->LIFCR = DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 |
                     DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3;
      SPI4->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC |
                   SPI_IFCR_OVRC | SPI_IFCR_SUSPC;

      register_set(&(DMA2_Stream3->M0AR), (uint32_t)data, 0xFFFFFFFFU);
      DMA2_Stream3->NDTR = length;
      spi_v3_tx_completion_start(&spi_v3_tx_completion);
      __DMB();
      register_set_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
      DMA2_Stream3->CR |= DMA_SxCR_EN;
      queued = true;
    }
  }
  EXIT_CRITICAL();
  return queued;
}

bool llspi_v3_tx_idle(void) {
  ENTER_CRITICAL();
  // TXC can become visible a few APB cycles after NSS rises. Polling here is
  // the final completion guarantee if EXTI ran before TXC asserted and the
  // grouped EOT/TXC interrupt was coalesced.
  if (spi_v3_tx_completion.active && spi_v3_tx_completion.dma_done &&
      spi_v3_tx_completion.nss_done &&
      ((SPI4->SR & SPI_SR_TXC) != 0U)) {
    spi_v3_tx_completion_note_txc(&spi_v3_tx_completion);
    register_clear_bits(&(SPI4->IER), SPI_IER_EOTIE);
    llspi_v3_tx_try_complete();
  }
  __DMB();
  const bool idle = !spi_v3_tx_completion.active;
  EXIT_CRITICAL();
  return idle;
}

bool llspi_v3_take_version_request(void) {
  bool pending = false;
  ENTER_CRITICAL();
  pending = spi_v3_version_request_pending;
  spi_v3_version_request_pending = false;
  EXIT_CRITICAL();
  return pending;
}

uint32_t llspi_v3_rx_begin(bool *was_reset) {
  ENTER_CRITICAL();
  if (spi_v3_rx_recovery_pending) {
    // A previous IRQ timed out waiting for DMA EN to clear. Retry here so a
    // transient hardware handoff cannot permanently disable the always-on RX.
    (void)llspi_v3_rx_dma_restart();
  }
  const bool pending = spi_v3_rx_reset_pending;
  if (pending) {
    // Apply the consumer reset in main context after any interrupted drain has
    // returned. The new DMA epoch starts at this produced ring boundary.
    spi_v3_rx_ring.consumed = spi_v3_rx_ring.produced;
  }
  spi_v3_rx_reset_pending = false;
  if (was_reset != NULL) {
    *was_reset = pending;
  }
  const uint32_t epoch = spi_v3_rx_dma_epoch;
  EXIT_CRITICAL();
  return epoch;
}

uint32_t llspi_v3_rx_epoch(void) {
  __DMB();
  return spi_v3_rx_dma_epoch;
}

static void DMA2_Stream2_IRQ_Handler(void) {
  llspi_v3_rx_snapshot();
}

static void DMA2_Stream3_IRQ_Handler(void) {
  const uint32_t dma_flags = DMA2->LISR;
  DMA2->LIFCR = DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 |
                 DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3;
  if ((dma_flags & (DMA_LISR_FEIF3 | DMA_LISR_DMEIF3 | DMA_LISR_TEIF3)) != 0U) {
    register_clear_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
    register_clear_bits(&(SPI4->IER), SPI_IER_EOTIE);
    SPI4->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC |
                 SPI_IFCR_OVRC | SPI_IFCR_SUSPC;
    llspi_v3_note_transport_error();
    // Do not disable SPE: doing so would reopen the RX race. At most a few
    // FIFO bytes can prefix the next retry, and v3 resynchronizes on magic.
    spi_v3_tx_completion_finish(&spi_v3_tx_completion);
  } else if ((dma_flags & DMA_LISR_TCIF3) != 0U) {
    const bool nss_is_high = (GPIOE->IDR & (1U << 11U)) != 0U;
    // NSS EXTI may have run before this lower-priority DMA handler. NDTR is
    // already zero, so a currently high NSS is the completed transaction.
    spi_v3_tx_completion_note_dma(&spi_v3_tx_completion, nss_is_high);
    // EOTIE also covers TXC on H7. It is armed only after DMA TC and disabled
    // as soon as TXC is observed, avoiding an interrupt storm while idle.
    register_set_bits(&(SPI4->IER), SPI_IER_EOTIE);
    if ((SPI4->SR & SPI_SR_TXC) != 0U) {
      spi_v3_tx_completion_note_txc(&spi_v3_tx_completion);
      register_clear_bits(&(SPI4->IER), SPI_IER_EOTIE);
    }
    llspi_v3_tx_try_complete();
  }
}

static void SPI4_IRQ_Handler(void) {
  const uint32_t status = SPI4->SR;
  SPI4->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC |
               SPI_IFCR_OVRC | SPI_IFCR_SUSPC;
  if (spi_v3_tx_completion.active && spi_v3_tx_completion.dma_done &&
      ((status & SPI_SR_TXC) != 0U)) {
    spi_v3_tx_completion_note_txc(&spi_v3_tx_completion);
    register_clear_bits(&(SPI4->IER), SPI_IER_EOTIE);
    llspi_v3_tx_try_complete();
  }
}

static void EXTI15_10_IRQ_Handler(void) {
  const uint32_t pending = EXTI->PR1 & (1U << 11U);
  if (pending != 0U) {
    llspi_v3_wait_rx_quiescent();
    llspi_v3_rx_snapshot();
    const uint32_t produced = spi_v3_rx_ring.produced;
    if (llspi_v3_exact_version_transaction(spi_v3_last_nss_produced, produced)) {
      spi_v3_version_request_pending = true;
    }
    spi_v3_last_nss_produced = produced;
    if (spi_v3_tx_completion.active) {
      // A short response poll must not satisfy NSS completion before all bytes
      // have left memory. It may continue the same self-framed response in a
      // later CS transaction; only the rising edge at/after NDTR zero counts.
      spi_v3_tx_completion_note_nss(&spi_v3_tx_completion,
                                    DMA2_Stream3->NDTR == 0U);
      if (spi_v3_tx_completion.dma_done && ((SPI4->SR & SPI_SR_TXC) != 0U)) {
        spi_v3_tx_completion_note_txc(&spi_v3_tx_completion);
        register_clear_bits(&(SPI4->IER), SPI_IER_EOTIE);
      }
      llspi_v3_tx_try_complete();
    }
  }
  EXTI->PR1 = (1U << 11U);
}

void llspi_init(void) {
  REGISTER_INTERRUPT(SPI4_IRQn, SPI4_IRQ_Handler, SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI)
  REGISTER_INTERRUPT(DMA2_Stream2_IRQn, DMA2_Stream2_IRQ_Handler,
                     SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_DMA)
  REGISTER_INTERRUPT(DMA2_Stream3_IRQn, DMA2_Stream3_IRQ_Handler,
                     SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_DMA)
  REGISTER_INTERRUPT(EXTI15_10_IRQn, EXTI15_10_IRQ_Handler,
                     SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_CS)

  register_set(&(DMAMUX1_Channel10->CCR), 83U, 0xFFFFFFFFU);
  register_set(&(DMA2_Stream2->CR),
               DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE |
               DMA_SxCR_TEIE | DMA_SxCR_DMEIE,
               0x1E077EFEU);
  register_set(&(DMA2_Stream2->PAR), (uint32_t)&(SPI4->RXDR), 0xFFFFFFFFU);
  register_set(&(DMA2_Stream2->M0AR), (uint32_t)spi_v3_dma_rx_ring, 0xFFFFFFFFU);
  DMA2_Stream2->NDTR = SPI_V3_RX_RING_SIZE;
  DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
                 DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;

  register_set(&(DMAMUX1_Channel11->CCR), 84U, 0xFFFFFFFFU);
  register_set(&(DMA2_Stream3->CR),
               DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE |
               DMA_SxCR_TEIE | DMA_SxCR_DMEIE,
               0x1E077EFEU);
  register_set(&(DMA2_Stream3->PAR), (uint32_t)&(SPI4->TXDR), 0xFFFFFFFFU);
  DMA2->LIFCR = DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 |
                 DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3;

  register_set(&(SPI4->IER), 0U, 0x3FFU);
  register_set(&(SPI4->CFG1),
               (7U << SPI_CFG1_DSIZE_Pos) | SPI_CFG1_RXDMAEN,
               SPI_CFG1_DSIZE_Msk | SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
  register_set(&(SPI4->UDRDR), 0xcdU, 0xFFFFU);
  register_set(&(SPI4->CR2), 0U, 0xFFFFU);
  SPI4->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC |
               SPI_IFCR_OVRC | SPI_IFCR_SUSPC;

  __DMB();
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  register_set_bits(&(SPI4->CR1), SPI_CR1_SPE);

  // PE11 remains SPI4_NSS in alternate-function mode; EXTI can independently
  // observe its rising edge and provides both transaction accounting and the
  // final TX completion condition.
  register_set(&(SYSCFG->EXTICR[2]), SYSCFG_EXTICR3_EXTI11_PE, 0xF000U);
  EXTI->PR1 = (1U << 11U);
  register_set_bits(&(EXTI->IMR1), (1U << 11U));
  register_set_bits(&(EXTI->RTSR1), (1U << 11U));
  register_clear_bits(&(EXTI->FTSR1), (1U << 11U));

  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  NVIC_EnableIRQ(SPI4_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

#elif defined(ENABLE_SPI) || defined(BOOTSTUB)
// master -> panda DMA start
void llspi_mosi_dma(uint8_t *addr, int len) {
  // disable DMA + SPI
  register_clear_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;
  register_clear_bits(&(SPI4->CR1), SPI_CR1_SPE);

  // drain the bus
  while ((SPI4->SR & SPI_SR_RXP) != 0U) {
    volatile uint8_t dat = SPI4->RXDR;
    (void)dat;
  }

  // clear all pending
  SPI4->IFCR |= (0x1FFU << 3U);
  register_set(&(SPI4->IER), 0, 0x3FFU);

  // setup destination and length
  register_set(&(DMA2_Stream2->M0AR), (uint32_t)addr, 0xFFFFFFFFU);
  DMA2_Stream2->NDTR = len;

  // enable DMA + SPI
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  register_set_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);
  register_set_bits(&(SPI4->CR1), SPI_CR1_SPE);
}

// panda -> master DMA start
void llspi_miso_dma(uint8_t *addr, int len) {
  // disable DMA + SPI
  DMA2_Stream3->CR &= ~DMA_SxCR_EN;
  register_clear_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
  register_clear_bits(&(SPI4->CR1), SPI_CR1_SPE);

  // setup source and length
  register_set(&(DMA2_Stream3->M0AR), (uint32_t)addr, 0xFFFFFFFFU);
  DMA2_Stream3->NDTR = len;

  // clear under-run while we were reading
  SPI4->IFCR |= (0x1FFU << 3U);

  // setup interrupt on TXC
  register_set(&(SPI4->IER), (1U << SPI_IER_EOTIE_Pos), 0x3FFU);

  // enable DMA + SPI
  register_set_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
  DMA2_Stream3->CR |= DMA_SxCR_EN;
  register_set_bits(&(SPI4->CR1), SPI_CR1_SPE);
}

static bool spi_tx_dma_done = false;
// master -> panda DMA finished
static void DMA2_Stream2_IRQ_Handler(void) {
  // Clear interrupt flag
  DMA2->LIFCR = DMA_LIFCR_CTCIF2;

  spi_rx_done();
}

// panda -> master DMA finished
static void DMA2_Stream3_IRQ_Handler(void) {
  ENTER_CRITICAL();

  DMA2->LIFCR = DMA_LIFCR_CTCIF3;
  spi_tx_dma_done = true;

  EXIT_CRITICAL();
}

// panda TX finished
static void SPI4_IRQ_Handler(void) {
  // clear flag
  SPI4->IFCR |= (0x1FFU << 3U);

  if (spi_tx_dma_done && ((SPI4->SR & SPI_SR_TXC) != 0U)) {
    spi_tx_dma_done = false;
    spi_tx_done(false);
  }
}


void llspi_init(void) {
  REGISTER_INTERRUPT(SPI4_IRQn, SPI4_IRQ_Handler, (SPI_IRQ_RATE * 2U), FAULT_INTERRUPT_RATE_SPI)
  REGISTER_INTERRUPT(DMA2_Stream2_IRQn, DMA2_Stream2_IRQ_Handler, SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_DMA)
  REGISTER_INTERRUPT(DMA2_Stream3_IRQn, DMA2_Stream3_IRQ_Handler, SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_DMA)

  // Setup MOSI DMA
  register_set(&(DMAMUX1_Channel10->CCR), 83U, 0xFFFFFFFFU);
  register_set(&(DMA2_Stream2->CR), (DMA_SxCR_MINC | DMA_SxCR_TCIE), 0x1E077EFEU);
  register_set(&(DMA2_Stream2->PAR), (uint32_t)&(SPI4->RXDR), 0xFFFFFFFFU);

  // Setup MISO DMA, memory -> peripheral
  register_set(&(DMAMUX1_Channel11->CCR), 84U, 0xFFFFFFFFU);
  register_set(&(DMA2_Stream3->CR), (DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE), 0x1E077EFEU);
  register_set(&(DMA2_Stream3->PAR), (uint32_t)&(SPI4->TXDR), 0xFFFFFFFFU);

  // Enable SPI
  register_set(&(SPI4->IER), 0, 0x3FFU);
  register_set(&(SPI4->CFG1), (7U << SPI_CFG1_DSIZE_Pos), SPI_CFG1_DSIZE_Msk);
  register_set(&(SPI4->UDRDR), 0xcd, 0xFFFFU);  // set under-run value for debugging
  register_set(&(SPI4->CR1), SPI_CR1_SPE, 0xFFFFU);
  register_set(&(SPI4->CR2), 0, 0xFFFFU);

  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  NVIC_EnableIRQ(SPI4_IRQn);
}
#endif
