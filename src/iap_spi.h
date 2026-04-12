/* SPI transport for DuetIAP
 * Receives firmware data from an SBC via SPI slave interface
 */

#ifndef IAP_SPI_H_INCLUDED
#define IAP_SPI_H_INCLUDED

#include "iap.h"

#ifdef IAP_SBC_SPI

// SPI-specific constants
constexpr uint32_t NvicPrioritySpi = 1;
constexpr uint32_t SpiTransferCompleteDelay = 400;	// time for DCS to wait when the firmware image has been transferred (SPI end-of-transfer heuristic)

// Initialize the SPI slave interface and DMA
void SpiInit() noexcept;

// Read a 2048-byte block from SPI into readData[].
// Returns true when a block is ready with bytesRead set.
// bytesRead == 0 means end of firmware transfer (TransferCompleteDelay elapsed).
bool SpiReadBlock() noexcept;

// Set up an SPI DMA transfer to receive the FlashVerifyRequest
void SpiSetupVerifyTransfer() noexcept;

// Check if the current SPI transfer has completed
bool SpiIsTransferComplete() noexcept;

// Send a 1-byte verify response via SPI
void SpiSendVerifyResponse(uint8_t response) noexcept;

// Drive the transfer-ready pin low on shutdown
void SpiShutdown() noexcept;

// Transfer start time, needed by verify timeout checks in writeBinary()
extern uint32_t spiTransferStartTime;

#endif // IAP_SBC_SPI

#endif // IAP_SPI_H_INCLUDED
