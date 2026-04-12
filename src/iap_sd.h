/* SD card transport for DuetIAP
 * Reads firmware data from a file on the SD card
 */

#ifndef IAP_SD_H_INCLUDED
#define IAP_SD_H_INCLUDED

#include "iap.h"

#ifndef IAP_VIA_SBC

// Initialize the SD card filesystem
void SdInit(const IapInfo *iapInfo) noexcept;

// Read a 2048-byte block from the firmware file into readData[].
// Returns true when a block is ready with bytesRead set.
bool SdReadBlock() noexcept;

// Close the firmware binary file
void SdClose() noexcept;

// SD-mode variables needed by writeBinary()
extern uint32_t firmwareFileSize;
extern bool isUf2File;
extern unsigned int eraseRetryCount;
extern uint32_t lastEraseRetryPos;

#endif // !IAP_VIA_SBC

#endif // IAP_SD_H_INCLUDED
