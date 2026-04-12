/* SPI transport for DuetIAP
 * Receives firmware data from an SBC via SPI slave interface with DMA
 */

#include "iap_spi.h"

#ifdef IAP_SBC_SPI

#if SAME5x
# include <hri_sercom_e54.h>
# include <Serial.h>
#endif

#if SAM4E || SAM4S || SAME70
# include <asf/sam/drivers/pmc/pmc.h>
#endif

#if SAM4E || SAME70
# include <asf/sam/drivers/spi/spi.h>
#endif

#if USE_DMAC
# include <asf/sam/drivers/dmac/dmac.h>
# include <asf/sam/drivers/matrix/matrix.h>
#endif

#if USE_XDMAC
# include <asf/sam/drivers/xdmac/xdmac.h>
#endif

#if USE_DMAC_MANAGER
# include <DmacManager.h>
#endif

#include <cstring>

// SPI transfer state
alignas(4) static char writeData[blockReadSize];
uint32_t spiTransferStartTime;

static volatile bool dataReceived = false;
static bool transferPending = false;
static bool transferReadyHigh = false;

#if USE_XDMAC
static xdmac_channel_config_t xdmac_tx_cfg, xdmac_rx_cfg;
#endif

#if SAME5x
static void SbcSpiIrqHandler(void *) noexcept;
#endif

// Set up an SPI DMA transfer of the specified size
static void setupSpi(size_t bytesToTransfer) noexcept
{
# if !SAME5x
	// Reset SPI
	spi_reset(SBC_SPI);
	spi_set_slave_mode(SBC_SPI);
	spi_disable_mode_fault_detect(SBC_SPI);
	spi_set_peripheral_chip_select_value(SBC_SPI, spi_get_pcs(0));
	spi_set_clock_polarity(SBC_SPI, 0, 0);
	spi_set_clock_phase(SBC_SPI, 0, 1);
	spi_set_bits_per_transfer(SBC_SPI, 0, SPI_CSR_BITS_8_BIT);
# endif

# if USE_DMAC
	dmac_channel_disable(DMAC, DmacChanSbcRx);
	dmac_channel_disable(DMAC, DmacChanSbcTx);

	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	// Initialize channel config for transmitter
	dmac_channel_set_source_addr(DMAC, DmacChanSbcTx, reinterpret_cast<uint32_t>(writeData));
	dmac_channel_set_destination_addr(DMAC, DmacChanSbcTx, reinterpret_cast<uint32_t>(&(SBC_SPI->SPI_TDR)));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanSbcTx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanSbcTx,
			bytesToTransfer |
			DMAC_CTRLA_SRC_WIDTH_WORD |
			DMAC_CTRLA_DST_WIDTH_BYTE);
	dmac_channel_set_ctrlB(DMAC, DmacChanSbcTx,
		DMAC_CTRLB_SRC_DSCR |
		DMAC_CTRLB_DST_DSCR |
		DMAC_CTRLB_FC_MEM2PER_DMA_FC |
		DMAC_CTRLB_SRC_INCR_INCREMENTING |
		DMAC_CTRLB_DST_INCR_FIXED);

	// Initialize channel config for receiver
	dmac_channel_set_source_addr(DMAC, DmacChanSbcRx, reinterpret_cast<uint32_t>(&(SBC_SPI->SPI_RDR)));
	dmac_channel_set_destination_addr(DMAC, DmacChanSbcRx, reinterpret_cast<uint32_t>(readData));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanSbcRx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanSbcRx,
			bytesToTransfer |
			DMAC_CTRLA_SRC_WIDTH_BYTE |
			DMAC_CTRLA_DST_WIDTH_WORD);
	dmac_channel_set_ctrlB(DMAC, DmacChanSbcRx,
		DMAC_CTRLB_SRC_DSCR |
		DMAC_CTRLB_DST_DSCR |
		DMAC_CTRLB_FC_PER2MEM_DMA_FC |
		DMAC_CTRLB_SRC_INCR_FIXED |
		DMAC_CTRLB_DST_INCR_INCREMENTING);

	dmac_channel_enable(DMAC, DmacChanSbcRx);
	dmac_channel_enable(DMAC, DmacChanSbcTx);

	// Configure DMA RX channel
	dmac_channel_set_configuration(DMAC, DmacChanSbcRx,
			DMAC_CFG_SRC_PER(SBC_SPI_RX_DMA_HW_ID) |
			DMAC_CFG_SRC_H2SEL |
			DMAC_CFG_SOD |
			DMAC_CFG_FIFOCFG_ASAP_CFG);

	// Configure DMA TX channel
	dmac_channel_set_configuration(DMAC, DmacChanSbcTx,
			DMAC_CFG_DST_PER(SBC_SPI_TX_DMA_HW_ID) |
			DMAC_CFG_DST_H2SEL |
			DMAC_CFG_SOD |
			DMAC_CFG_FIFOCFG_ASAP_CFG);
# elif USE_XDMAC
	// Initialize channel config for transmitter
	xdmac_tx_cfg.mbr_ubc = bytesToTransfer;
	xdmac_tx_cfg.mbr_sa = (uint32_t)writeData;
	xdmac_tx_cfg.mbr_da = (uint32_t)&(SBC_SPI->SPI_TDR);
	xdmac_tx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_MEM2PER |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF0 |
		XDMAC_CC_DIF_AHB_IF1 |
		XDMAC_CC_SAM_INCREMENTED_AM |
		XDMAC_CC_DAM_FIXED_AM |
		XDMAC_CC_PERID(SBC_SPI_TX_PERID);
	xdmac_tx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_tx_cfg.mbr_sus = 0;
	xdmac_tx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanSbcTx, &xdmac_tx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanSbcTx, 0);
	xdmac_channel_enable(XDMAC, DmacChanSbcTx);
	xdmac_disable_interrupt(XDMAC, DmacChanSbcTx);

	// Initialize channel config for receiver
	xdmac_rx_cfg.mbr_ubc = bytesToTransfer;
	xdmac_rx_cfg.mbr_da = (uint32_t)readData;
	xdmac_rx_cfg.mbr_sa = (uint32_t)&(SBC_SPI->SPI_RDR);
	xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE|
		XDMAC_CC_SIF_AHB_IF1 |
		XDMAC_CC_DIF_AHB_IF0 |
		XDMAC_CC_SAM_FIXED_AM |
		XDMAC_CC_DAM_INCREMENTED_AM |
		XDMAC_CC_PERID(SBC_SPI_RX_PERID);
	xdmac_rx_cfg.mbr_bc = 0;
	xdmac_rx_cfg.mbr_ds = 0;
	xdmac_rx_cfg.mbr_sus = 0;
	xdmac_rx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanSbcRx, &xdmac_rx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanSbcRx, 0);
	xdmac_channel_enable(XDMAC, DmacChanSbcRx);
	xdmac_disable_interrupt(XDMAC, DmacChanSbcRx);
# elif USE_DMAC_MANAGER
	DmacManager::DisableChannel(DmacChanSbcRx);
	DmacManager::DisableChannel(DmacChanSbcTx);

	DmacManager::SetSourceAddress(DmacChanSbcTx, writeData);
	DmacManager::SetDestinationAddress(DmacChanSbcTx, &(SbcSpiSercom->SPI.DATA.reg));
#  if USE_32BIT_TRANSFERS
	DmacManager::SetBtctrl(DmacChanSbcTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetDataLength(DmacChanSbcTx, (bytesToTransfer + 3) >> 2);			// must do this one last
#  else
	DmacManager::SetBtctrl(DmacChanSbcTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetDataLength(DmacChanSbcTx, bytesToTransfer);						// must do this one last
#  endif
	DmacManager::SetTriggerSourceSercomTx(DmacChanSbcTx, SbcSpiSercomNumber);

	DmacManager::SetSourceAddress(DmacChanSbcRx, &(SbcSpiSercom->SPI.DATA.reg));
	DmacManager::SetDestinationAddress(DmacChanSbcRx, readData);
#  if USE_32BIT_TRANSFERS
	DmacManager::SetBtctrl(DmacChanSbcRx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_INT);
	DmacManager::SetDataLength(DmacChanSbcRx, (bytesToTransfer + 3) >> 2);			// must do this one last
#  else
	DmacManager::SetBtctrl(DmacChanSbcRx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_INT);
	DmacManager::SetDataLength(DmacChanSbcRx, bytesToTransfer);						// must do this one last
#  endif
	DmacManager::SetTriggerSourceSercomRx(DmacChanSbcRx, SbcSpiSercomNumber);

	DmacManager::EnableChannel(DmacChanSbcRx, DmacPrioSbc);
	DmacManager::EnableChannel(DmacChanSbcTx, DmacPrioSbc);
# endif

	// Enable SPI and notify the SBC we are ready
#if SAME5x
	SbcSpiSercom->SPI.INTFLAG.reg = 0xFF;			// clear any pending interrupts
	SbcSpiSercom->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_SSL;	// enable the start of transfer (SS low) interrupt
	hri_sercomspi_set_CTRLA_ENABLE_bit(SbcSpiSercom);
#else
	spi_enable(SBC_SPI);

	// Enable end-of-transfer interrupt
	(void)SBC_SPI->SPI_SR;							// clear any pending interrupt
	SBC_SPI->SPI_IER = SPI_IER_NSSR;				// enable the NSS rising interrupt
#endif

#if SAME5x
	// Register the SBC SPI interrupt handler via the CoreN2G SERCOM callback mechanism.
	// Only the IRQ3 (SS low) handler is needed.
	Serial::SetSercomVector(SbcSpiSercomNumber, nullptr, nullptr, nullptr, SbcSpiIrqHandler, nullptr);
#endif
	NVIC_SetPriority(SBC_SPI_IRQn, NvicPrioritySpi);
	NVIC_EnableIRQ(SBC_SPI_IRQn);

	// Begin transfer
	dataReceived = false;
	transferPending = true;
	spiTransferStartTime = millis();

	transferReadyHigh = !transferReadyHigh;
	digitalWrite(SbcTfrReadyPin, transferReadyHigh);
}

static void disableSpi() noexcept
{
# if USE_DMAC
	dmac_channel_disable(DMAC, DmacChanSbcRx);
	dmac_channel_disable(DMAC, DmacChanSbcTx);
# endif

# if USE_XDMAC
	xdmac_channel_disable(XDMAC, DmacChanSbcRx);
	xdmac_channel_disable(XDMAC, DmacChanSbcTx);
# endif

# if USE_DMAC_MANAGER
	DmacManager::DisableChannel(DmacChanSbcRx);
	DmacManager::DisableChannel(DmacChanSbcTx);
# endif

	// Disable SPI
#if SAME5x
	hri_sercomspi_clear_CTRLA_ENABLE_bit(SbcSpiSercom);
#else
	spi_disable(SBC_SPI);
#endif
}

// SPI interrupt handlers

#if SAME5x

static void SbcSpiIrqHandler(void *) noexcept
{
	// On the SAM5x we can't get an end-of-transfer interrupt, only a start-of-transfer interrupt.
	// So we can't disable SPI or DMA in this ISR.
	const uint8_t status = SbcSpiSercom->SPI.INTFLAG.reg;
	if ((status & SERCOM_SPI_INTENSET_SSL) != 0)
	{
		SbcSpiSercom->SPI.INTENCLR.reg = SERCOM_SPI_INTENSET_SSL;		// disable the interrupt
		SbcSpiSercom->SPI.INTFLAG.reg = SERCOM_SPI_INTENSET_SSL;		// clear the status
		dataReceived = true;
	}
}

#else

# ifndef SBC_SPI_HANDLER
#  error SBC_SPI_HANDLER undefined
# endif

extern "C" void SBC_SPI_HANDLER(void) noexcept
{
	const uint32_t status = SBC_SPI->SPI_SR;							// read status and clear interrupt
	SBC_SPI->SPI_IDR = SPI_IER_NSSR;									// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{
		// Data has been transferred, disable transfer ready pin and XDMAC channels
		disableSpi();
		dataReceived = true;
	}
}

#endif

static bool isSpiTransferComplete() noexcept
{
# if USE_DMAC
	const uint32_t status = DMAC->DMAC_CHSR;
	if (dataReceived &&
		  (((status & (DMAC_CHSR_ENA0 << DmacChanSbcRx)) == 0)	// controller is not enabled, perhaps because it finished a full buffer transfer
		|| ((status & (DMAC_CHSR_EMPT0 << DmacChanSbcRx)) != 0))	// controller is enabled, probably suspended, and the FIFO is empty
	   )
	{
		// Disable the channel.
		// We also need to set the resume bit, otherwise it remains suspended when we re-enable it.
		DMAC->DMAC_CHDR = (DMAC_CHDR_DIS0 << DmacChanSbcRx) | (DMAC_CHDR_RES0 << DmacChanSbcRx);
		transferPending = false;
		return true;
	}
	return false;
# elif USE_XDMAC
	if (dataReceived && (xdmac_channel_get_status(XDMAC) & ((1 << DmacChanSbcRx) | (1 << DmacChanSbcTx))) == 0)
	{
		transferPending = false;
		return true;
	}
	return false;
# elif USE_DMAC_MANAGER
	if (dataReceived && digitalRead(SbcSSPin))	// transfer is complete if SS is high
	{
		if (transferPending)
		{
#  if SAME5x
			// We cannot disable SPI in the ISR, do it here instead
			disableSpi();
#  endif
			transferPending = false;
		}
		return true;
	}
	return false;
# endif
}

// Public functions

void SpiInit() noexcept
{
	SetPinMode(SbcTfrReadyPin, OUTPUT_LOW);

# if SAME5x
	for (Pin p : SbcSpiSercomPins)
	{
		SetPinFunction(p, SbcSpiSercomPinsMode);
	}

	Serial::EnableSercomClock(SbcSpiSercomNumber);
	DmacManager::DisableChannel(DmacChanSbcRx);
	DmacManager::DisableChannel(DmacChanSbcTx);

	hri_sercomspi_set_CTRLA_SWRST_bit(SbcSpiSercom);
	SbcSpiSercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_MODE(2);
	hri_sercomspi_write_CTRLB_reg(SbcSpiSercom, SERCOM_SPI_CTRLB_RXEN | SERCOM_SPI_CTRLB_SSDE | SERCOM_SPI_CTRLB_PLOADEN);
#  if USE_32BIT_TRANSFERS
	hri_sercomspi_write_CTRLC_reg(SbcSpiSercom, SERCOM_SPI_CTRLC_DATA32B);
#  else
	hri_sercomspi_write_CTRLC_reg(SbcSpiSercom, 0);
#  endif
# else
	SetPinFunction(APIN_SBC_SPI_MOSI, SpiPinsFunction);
	SetPinFunction(APIN_SBC_SPI_MISO, SpiPinsFunction);
	SetPinFunction(APIN_SBC_SPI_SCK, SpiPinsFunction);
	SetPinFunction(APIN_SBC_SPI_SS0, SpiPinsFunction);

	spi_enable_clock(SBC_SPI);
	spi_disable(SBC_SPI);
# endif

# if USE_DMAC
	pmc_enable_periph_clk(ID_DMAC);
	NVIC_DisableIRQ(DMAC_IRQn);
# elif USE_XDMAC
	pmc_enable_periph_clk(ID_XDMAC);
	NVIC_DisableIRQ(XDMAC_IRQn);
# endif

# if USE_DMAC
	dmac_init(DMAC);
	dmac_set_priority_mode(DMAC, DMAC_PRIORITY_ROUND_ROBIN);
	dmac_enable(DMAC);

	// The DMAC is master 4 and the SRAM is slave 0. Give the DMAC the highest priority.
	matrix_set_slave_default_master_type(0, MATRIX_DEFMSTR_LAST_DEFAULT_MASTER);
	matrix_set_slave_priority(0, (3 << MATRIX_PRAS0_M4PR_Pos));
	// Set the slave slot cycle limit.
	// If we leave it at the default value of 511 clock cycles, we get transmit underruns due to the HSMCI using the bus for too long.
	// A value of 8 seems to work. I haven't tried other values yet.
	matrix_set_slave_slot_cycle(0, 8);
# endif

	memset(writeData, 0x1A, blockReadSize);
}

bool SpiReadBlock() noexcept
{
	if (transferPending)
	{
		if (isSpiTransferComplete())
		{
			// Got another flash block to write. The block size is fixed
			bytesRead = blockReadSize;
			return true;
		}
		else if (flashPos != FirmwareFlashStart && millis() - spiTransferStartTime > SpiTransferCompleteDelay)
		{
			// If anything could be written before, check for the delay indicating the flashing process has finished
			bytesRead = 0;
			disableSpi();
			memset(readData, 0xFF, blockReadSize);
			return true;
		}
		else if (millis() - spiTransferStartTime > TransferTimeout)
		{
			// Timeout while waiting for new data
			MessageF("ERROR: Timeout while waiting for response");
			Reset(false);
		}
	}
	else
	{
		// The last block has been written to Flash. Start the next SPI transfer
		setupSpi(blockReadSize);
	}
	return false;
}

void SpiSetupVerifyTransfer() noexcept
{
	setupSpi(sizeof(FlashVerifyRequest));
}

bool SpiIsTransferComplete() noexcept
{
	return isSpiTransferComplete();
}

void SpiSendVerifyResponse(uint8_t response) noexcept
{
	writeData[0] = static_cast<char>(response);
	setupSpi(1);
}

void SpiShutdown() noexcept
{
	digitalWrite(SbcTfrReadyPin, false);
}

#endif // IAP_SBC_SPI
