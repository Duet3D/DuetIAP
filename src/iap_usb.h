/* USB CDC transport for DuetIAP using CoreN2G SerialCDC
 * Receives firmware data from an SBC via USB CDC
 */

#ifndef IAP_USB_H_INCLUDED
#define IAP_USB_H_INCLUDED

#include "iap.h"

#ifdef IAP_SBC_USB

// Firmware length received from DSF during handshake - used for end-of-transfer detection.
// Set in UsbWaitReady(), consumed by UsbReadBlock().
extern uint32_t expectedFirmwareLength;

// Reset block-transfer state so a new firmware transfer can start cleanly (used on verify retry)
void UsbResetTransferState() noexcept;

// Initialize USB clocks/pins and start the CDC device
void UsbInit() noexcept;

// Wait for USB connection and complete the IAPR handshake. Call after UsbInit().
void UsbWaitReady() noexcept;

// Process USB events (no-op for interrupt-driven SerialCDC, but keeps API consistent)
void UsbPoll() noexcept;

// Read a 2048-byte block from USB into readData[].
// Returns true when a block is ready with bytesRead set.
bool UsbReadBlock() noexcept;

// True once all expectedFirmwareLength bytes have been received
bool UsbTransferComplete() noexcept;

// Set up to receive the FlashVerifyRequest over USB
void UsbSetupVerifyTransfer() noexcept;

// Check if verify data has been received
bool UsbIsVerifyDataReady() noexcept;

// Send a 1-byte verify response over USB
void UsbSendVerifyResponse(uint8_t response) noexcept;

// Get the transfer start time for timeout checks
uint32_t UsbGetTransferStartTime() noexcept;

#endif // IAP_SBC_USB

#endif // IAP_USB_H_INCLUDED
