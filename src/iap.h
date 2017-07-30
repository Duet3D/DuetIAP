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

#include "Core.h"

#if SAM3XA
# define IFLASH_ADDR			(IFLASH0_ADDR)
# define IFLASH_PAGE_SIZE	(IFLASH1_PAGE_SIZE)
#endif

const size_t baudRate = 115200;						// For USB diagnostics

const uint32_t iapFirmwareSize = 0x10000;			// 64 KiB max

#if SAM3XA
# ifdef __RADDS__
#  define SERIAL_AUX_DEVICE Serial1
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = { 14, 14 };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[2] = { 87, 77 };
const char * const defaultFwFile = "0:/sys/RepRapFirmware-RADDS.bin";	// Which file shall be used for IAP?
const char * const fwFilePrefix = "0:/sys/RepRap";
# elif defined(__ALLIGATOR__)
#  define SERIAL_AUX_DEVICE Serial1
const size_t NumSdCards = 1;
const Pin SdCardDetectPins[NumSdCards] = { 87 };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin };
const Pin SdSpiCSPins[2] = { 77 };
const char * const defaultFwFile = "0:/sys/RepRapFirmware-Alligator.bin";	// Which file shall be used for IAP?
const char * const fwFilePrefix = "0:/sys/RepRap";

# else
#  define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {13, NoPin};
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {67};
const char * const defaultFwFile = "0:/sys/RepRapFirmware.bin";			// Which file shall be used for IAP?
const char * const fwFilePrefix = "0:/sys/RepRap";
# endif
#endif

#if SAM4E
# define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {53, NoPin};
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {56};
const char * const defaultFwFile = "0:/sys/DuetWiFiFirmware.bin";		// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/Duet";
#endif

const uint32_t firmwareFlashEnd = IFLASH_ADDR + IFLASH_SIZE - iapFirmwareSize;

// Read and write only 2 KiB of data at once (must be multiple of IFLASH_PAGE_SIZE).
// Unfortunately we cannot increase this value further, because f_read() would mess up data
const size_t blockReadSize = 2048;

const size_t maxRetries = 5;						// Allow 5 retries max if anything goes wrong


enum ProcessState
{
	Initializing,
	UnlockingFlash,
#if SAM4E || SAM4S
	ErasingFlash,
#endif
	WritingUpgrade,
	FillingZeros,
	LockingFlash
};


void initFilesystem();
void getFirmwareFileName();
void openBinary();
void writeBinary();
void closeAndDeleteBinary();
void Reset(bool success);

void sendUSB(uint32_t ep, const void* d, uint32_t len);

// vim: ts=4:sw=4
