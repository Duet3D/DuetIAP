/* In-Application Programming application for RepRap Duet (Atmel SAM3X8E)
 *
 * This application is first written by RepRapFirmware to the end of the second
 * Flash bank and then started by RepRapFirmware.
 * 
 * Once this program is loaded, it performs in-application programming by
 * reading the new firmware binary from the SD card and replaces the corresponding
 * Flash content sector by sector.
 *
 * This application was written by Christian Hammacher (2016) and is
 * licensed under the terms of the GPL v2.
 */

#include "Arduino.h"

#include "DueFlashStorage.h"
#include "SD_HSMCI.h"


const size_t baudRate = 115200;						// For USB diagnostics

const char *fwFile = "0:/sys/RepRapFirmware.bin";	// Which file shall be used for IAP?

const uint32_t iapFirmwareSize = 0x10000;			// 64K max
const uint32_t lastSectorAddress = IFLASH_ADDR + IFLASH_SIZE - iapFirmwareSize;

const size_t blockReadSize = 2048;					// Read and write 2KB of data at once (must be multiple of IFLASH_PAGE_SIZE)

enum ProcessState
{
	UnlockingFlash,
	WritingUpgrade,
	FillingZeros,
	LockingFlash
};

void initFilesystem();
void openBinary();
void writeBinary();
void closeAndDeleteBinary();

void sendUSB(uint32_t ep, const void* d, uint32_t len);
void Reset();

// vim: ts=4:sw=4
