/* In-Application Programming application for Duet3D platforms
 *
 * This application is first written by RepRapFirmware to the end of RAM
 * then started by RepRapFirmware.
 *
 * Once this program is launched, it performs in-application programming by
 * reading the new firmware binary from the SD card or designated SBC
 * transport method and replaces the corresponding Flash content sector by sector.
 *
 * This application was written by Christian Hammacher (2016-2026) and is
 * licensed under the terms of the GPL v2.
 */

#include "iap.h"
#include "Devices.h"
#include "Version.h"
#include "led.h"

#ifdef IAP_SBC_SPI
# include "iap_spi.h"
#endif

#ifdef IAP_SBC_USB
# include "iap_usb.h"
#endif

#ifndef IAP_VIA_SBC
# include "iap_sd.h"
#endif

#include <Flash.h>

#include <General/SafeVsnprintf.h>
#include <General/StringFunctions.h>

#include <cstdarg>
#include <cstring>

// CoreN2G requires a version string
extern const char VersionText[] =
#ifdef IAP_VIA_SBC
	"In-application programmer (SBC version) version " VERSION_TEXT;
#else
	"In-application programmer (SD version) version " VERSION_TEXT;
#endif

// Shared global variables
alignas(4) char readData[blockReadSize];	// use aligned memory so DMA works well

ProcessState state = Initializing;
uint32_t pageSize;
uint32_t flashPos = FirmwareFlashStart;

unsigned int retry = 0;

size_t bytesRead, bytesWritten;
bool haveDataInBuffer;
const size_t reportPercentIncrement = 20;
size_t reportNextPercent = reportPercentIncrement;

#ifdef IAP_VIA_SBC
static bool usingUsb = false;
static unsigned int crcRetryCount = 0;		// number of whole-image reflash attempts after a checksum mismatch
#endif

// Our own version of delay() that keeps the LED and USB up to date
void delayMs(uint32_t ms) noexcept
{
	const uint32_t startTime = millis();
	do
	{
		checkLed();
#if defined(IAP_SBC_USB)
		if (usingUsb)
		{
			UsbPoll();
		}
#endif
	} while (millis() - startTime < ms);
}

extern "C" void UrgentInit() noexcept { }

extern "C" void SysTick_Handler(void) noexcept
{
	CoreSysTick();
	WatchdogReset();

#if SAM4E || SAME70
	WatchdogResetSecondary();
#endif
}

extern "C" void SVC_Handler() noexcept { for (;;) {} }
extern "C" void PendSV_Handler() noexcept { for (;;) {} }

void AppMain() noexcept
{
	CoreInit();
	DeviceInit();

	// Initialise systick (needed for delay calls) - CoreN2G initialises it in non-interrupt mode
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1u) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1ul << SysTick_CTRL_ENABLE_Pos) | (1ul << SysTick_CTRL_TICKINT_Pos) | (1ul << SysTick_CTRL_CLKSOURCE_Pos);
	NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);	// set Priority for Systick Interrupt

	initLed();

	// Read IAP info from RAM above the stack (written by RRF before jumping to IAP)
	const uint32_t vtab = SCB->VTOR & SCB_VTOR_TBLOFF_Msk;
	const uint32_t stackTop = *reinterpret_cast<const uint32_t*>(vtab);
	const IapInfo * const iapInfo = reinterpret_cast<const IapInfo*>(stackTop);

	// Initialise AUX serial if RRF passed a baud rate, otherwise use default
	if (iapInfo->magic == IapInfo::MagicValue && iapInfo->auxBaudRate != 0)
	{
		serialUart0.begin(iapInfo->auxBaudRate);
	}
	else
	{
		serialUart0.begin(57600);			// fallback for older RRF versions
	}
#if SAME5x
	if (!Flash::Init())
	{
		MessageF("Failed to initialize flash controller");
		Reset(false);
	}

	pageSize = Flash::GetPageSize();
#else
	pageSize = IFLASH_PAGE_SIZE;
#endif

	// Initialize the appropriate transport
#ifdef IAP_VIA_SBC
# ifdef IAP_SBC_USB
	if (iapInfo->magic == IapInfo::MagicValue && iapInfo->transport == IapInfo::TransportUsb)
	{
		usingUsb = true;
		MessageF("IAP started, using USB");
		UsbInit();
		UsbWaitReady();
	}
	else
# endif
	{
# ifdef IAP_SBC_SPI
		usingUsb = false;
		MessageF("IAP started, using SPI");
		SpiInit();
# else
		MessageF("ERROR: SPI transport not available");
		Reset(false);
# endif
	}
#else
	MessageF("IAP started, using SD");
	SdInit(iapInfo);
#endif

	for (;;)
	{
		checkLed();
#if defined(IAP_SBC_USB)
		if (usingUsb)
		{
			UsbPoll();
		}
#endif
		writeBinary();
	}
}

#ifdef IAP_VIA_SBC

uint16_t CRC16(const char *buffer, size_t length) noexcept
{
	static const uint16_t crc16_table[] =
	{
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    uint16_t Crc = 65535;
    uint16_t x;
    for (size_t i = 0; i < length; i++)
    {
        x = (uint16_t)(Crc ^ buffer[i]);
        Crc = (uint16_t)((Crc >> 8) ^ crc16_table[x & 0x00FF]);
    }

    return Crc;
}

#endif // IAP_VIA_SBC

void ShowProgress() noexcept
{
#ifdef IAP_VIA_SBC
	const uint32_t totalSize = FirmwareFlashEnd - FirmwareFlashStart;		//TODO is there a way of knowing the total file size?
#else
	const uint32_t totalSize = (isUf2File) ? firmwareFileSize/2 : firmwareFileSize;
#endif
	const size_t percentDone = (100 * (flashPos - FirmwareFlashStart))/totalSize;
	if (percentDone >= reportNextPercent)
	{
		MessageF("Flashing firmware, %u%% completed", percentDone);
		reportNextPercent += reportPercentIncrement;
	}
}

// Return the size of the sector that starts at startPos
uint32_t GetFlashSectorSize(uint32_t startPos)
{
#if SAME5x
	return Flash::GetEraseRegionSize();
#elif SAM4E || SAM4S
	// Deal with varying size sectors on the SAM4E and SAM4S
	// There are two 8K sectors, then one 48K sector, then seven 64K sectors
	return (startPos - IFLASH_ADDR < 16 * 1024) ? 8 * 1024
			: (startPos - IFLASH_ADDR == 16 * 1024) ? 48 * 1024
				: 64 * 1024;
#elif SAME70
	// Deal with varying size sectors on the SAME70
	// There are two 8K sectors, then one 112K sector, then the rest are 128K sectors
	return (startPos - IFLASH_ADDR < 16 * 1024) ? 8 * 1024
			: (startPos - IFLASH_ADDR == 16 * 1024) ? 112 * 1024
				: 128 * 1024;
#endif
}

// Check whether an area of flash is erased
bool IsSectorErased(uint32_t addr, uint32_t sectorSize)
{
	for (uint32_t p = addr; p < addr + sectorSize; p += sizeof(uint32_t))
	{
		if (*reinterpret_cast<const uint32_t*>(p) != 0xFFFFFFFF)
		{
			return false;
		}
	}
	return true;
}

#ifndef IAP_VIA_SBC
// Find the start address of the sector that holds a particular flash address
uint32_t FindSectorStart(uint32_t addr)
{
	uint32_t sectorStart = IFLASH_ADDR;
	while (true)
	{
		const uint32_t sectorSize = GetFlashSectorSize(sectorStart);
		if (addr < sectorStart + sectorSize)
		{
			return sectorStart;
		}
		sectorStart += sectorSize;
	}
}

// Compare a block of memory we just wrote with the data we wrote.
// Return 0 if all OK, 1 if it contains 1s that should be 0s, 2 if it contains 0s that should be ones, 3 if it contains both.
int CompareMemory(const uint32_t *readBack, const uint32_t *written, size_t numWords)
{
	uint32_t bad1s = 0;
	uint32_t bad0s = 0;
	while (numWords != 0)
	{
		const uint32_t dataReadBack = *readBack++;
		const uint32_t dataWritten = *written++;
		bad1s |= dataReadBack & (~dataWritten);
		bad0s |= dataWritten & (~dataReadBack);
		--numWords;
	}
	int ret = (bad1s == 0) ? 0 : 1;
	if (bad0s != 0) { ret |= 2; }
	return ret;
}
#endif

// Helper to call the appropriate ReadBlock for the active transport
static bool ReadBlockDispatch() noexcept
{
#ifdef IAP_VIA_SBC
# ifdef IAP_SBC_USB
	if (usingUsb)
	{
		return UsbReadBlock();
	}
# endif
# ifdef IAP_SBC_SPI
	return SpiReadBlock();
# else
	return false;
# endif
#else
	return SdReadBlock();
#endif
}

// This implements the actual functionality of this program
void writeBinary() noexcept
{
	if (retry > MaxRetries)
	{
		MessageF("ERROR: Operation %d failed after %d retries", (int)state, MaxRetries);
		Reset(false);
	}
	else if (retry > 0)
	{
		debugPrintf("WARNING: Retry %d of %d at pos %08x", retry, MaxRetries, flashPos);
	}

	switch (state)
	{
	case Initializing:
		MessageF("Unlocking flash");
		state = UnlockingFlash;
		// no break
	case UnlockingFlash:
		{
			debugPrintf("Unlocking 0x%08x - 0x%08x", flashPos, flashPos + pageSize - 1);

			// We can unlock all the flash in one call. We may have to unlock from before the firmware start. The bootloader is protected separately.
			const uint32_t unlockStart = FirmwareFlashStart & ~(Flash::GetLockRegionSize() - 1);
			if (Flash::Unlock(unlockStart, FirmwareFlashEnd - unlockStart))
			{
				flashPos = FirmwareFlashStart;
				MessageF("Erasing flash");
				state = ErasingFlash;
			}
			else
			{
				++retry;
			}
		}
		break;

	case ErasingFlash:
		debugPrintf("Erasing 0x%08x", flashPos);
		if (retry != 0)
		{
			MessageF("Erase retry #%u at offset %08" PRIx32, retry, flashPos - IFLASH_ADDR);
			delayMs(RetryMessageDelay);
		}

		{
			const uint32_t sectorSize = GetFlashSectorSize(flashPos);
			// No need to erase a sector that is already erased
# if SAME5x
			if (IsSectorErased(flashPos, sectorSize) || Flash::Erase(flashPos, sectorSize))
#else
			if (IsSectorErased(flashPos, sectorSize) || Flash::EraseSector(flashPos))
#endif
			{
				// Check that the sector really is erased
				if (IsSectorErased(flashPos, sectorSize))
				{
					retry = 0;
					flashPos += sectorSize;
				}
				else
				{
					++retry;
				}
			}
			else
			{
				++retry;
			}

			if (flashPos >= FirmwareFlashEnd)
			{
				flashPos = FirmwareFlashStart;
				haveDataInBuffer = false;
				MessageF("Writing data");
				state = WritingUpgrade;
			}
		}
		break;

#ifndef IAP_VIA_SBC
	case EraseRetry:
		if (retry != 0)
		{
			MessageF("Erase sector retry #%u at offset %08" PRIx32, retry, flashPos - IFLASH_ADDR);
			delayMs(RetryMessageDelay);
		}

		{
			const uint32_t sectorSize = GetFlashSectorSize(flashPos);
# if SAME5x
			if (Flash::Erase(flashPos, sectorSize))
# else
			if (Flash::EraseSector(flashPos) && IsSectorErased(flashPos, sectorSize))
# endif
			{
				haveDataInBuffer = false;
				state = WritingUpgrade;
			}
			++retry;
		}
		break;
#endif

	case WritingUpgrade:
		// Attempt to read a chunk from the firmware file or SBC
		if (!haveDataInBuffer)
		{
			if (!ReadBlockDispatch())
			{
				break;
			}
			haveDataInBuffer = true;
			retry = 0;
			bytesWritten = 0;
		}

		// Write another page, unless the firmware region is already full
		{
			static bool writeSuceeded = false;			// static so that the retry message can say whether the write or the verify failed
			bool endOfTransfer = false;

			if (flashPos >= FirmwareFlashEnd)
			{
				// The firmware region is full. A further block is either the transport's trailing end-of-transfer
				// padding (an image that exactly fills the region) or a sender streaming more than the region can
				// hold; either way there is nothing left to program, so finish rather than writing past FirmwareFlashEnd
				endOfTransfer = true;
			}
			else
			{
				debugPrintf("Writing 0x%08x - 0x%08x", flashPos, flashPos + pageSize - 1);
				if (retry != 0)
				{
					MessageF("Flash write%s retry #%u at address %08" PRIx32, ((writeSuceeded) ? "/verify" : ""), retry, flashPos - IFLASH_ADDR);
					delayMs(RetryMessageDelay);
				}

				writeSuceeded = Flash::Write(flashPos, pageSize, reinterpret_cast<const uint32_t *>(readData + bytesWritten));
				if (!writeSuceeded)
				{
					MessageF("Flash write failed: pos=%08" PRIx32 " size=%" PRIu32 " err=%08" PRIx32, flashPos, pageSize, Flash::GetLastFlashError());
					++retry;
					break;
				}

#ifndef IAP_VIA_SBC
				// Verify the written data. Our data is aligned, so we can compare words.
				// In SBC mode this happens at the end of the flash process using a CRC checksum.
				const int cmp = CompareMemory(reinterpret_cast<const uint32_t*>(flashPos), (const uint32_t*)(readData + bytesWritten), pageSize >> 2);
				if (cmp == 1)
				{
					// There are some bits reading as 1 that should be zero
					MessageF("Flash compare failed, missing zeros");
					++retry;
					break;
				}

				if (cmp != 0)
				{
					// There are some bits reading as 0 that should be 1. Erase the sector and start again from the beginning of this sector.
					MessageF("Flash compare failed, missing ones");
					flashPos = FindSectorStart(flashPos);
					if (flashPos == lastEraseRetryPos)
					{
						++eraseRetryCount;
						if (eraseRetryCount == MaxEraseRetries)
						{
							MessageF("ERROR: too many erase sector retries at pos %" PRIu32, flashPos);
							Reset(false);
						}
					}
					else
					{
						eraseRetryCount = 0;
					}
					retry = 0;
					state = EraseRetry;
					break;
				}
#endif

				retry = 0;
				bytesWritten += pageSize;
				flashPos += pageSize;
				ShowProgress();
				if (bytesWritten == blockReadSize)
				{
					haveDataInBuffer = false;

					// SD and SPI mark the final block with a short read; the USB transport always receives
					// full fixed-size blocks and reports completion separately, so it can end on a full block
					bool lastBlock = bytesRead < blockReadSize;
#ifdef IAP_SBC_USB
					if (usingUsb && UsbTransferComplete())
					{
						lastBlock = true;
					}
#endif
					if (lastBlock)
					{
						endOfTransfer = true;
					}
				}
			}

			if (endOfTransfer)
			{
				haveDataInBuffer = false;
#ifdef IAP_VIA_SBC
				// Set up to receive the verification request
# ifdef IAP_SBC_USB
				if (usingUsb)
				{
					UsbSetupVerifyTransfer();
				}
				else
# endif
# ifdef IAP_SBC_SPI
				{
					SpiSetupVerifyTransfer();
				}
# endif
				state = VerifyingChecksum;
#else
				SdClose();
				state = LockingFlash;
#endif
			}
		}
		break;

#ifdef IAP_VIA_SBC
	case VerifyingChecksum:
		{
			// Check for timeout
			bool timedOut = false;
			bool complete = false;

# ifdef IAP_SBC_USB
			if (usingUsb)
			{
				complete = UsbIsVerifyDataReady();
				if (!complete && millis() - UsbGetTransferStartTime() > TransferTimeout)
				{
					timedOut = true;
				}
			}
			else
# endif
# ifdef IAP_SBC_SPI
			{
				complete = SpiIsTransferComplete();
				if (!complete && millis() - spiTransferStartTime > TransferTimeout)
				{
					timedOut = true;
				}
			}
# endif

			if (timedOut)
			{
				MessageF("Timeout while waiting for checksum");
				Reset(false);
			}
			else if (complete)
			{
				const FlashVerifyRequest *request = reinterpret_cast<const FlashVerifyRequest*>(readData);
				// Defensive: reject absurd firmware lengths to prevent CRC16 from reading past flash
				// and triggering a BusFault. This can happen if the verify request was not actually
				// sent by DSF (e.g. premature end-of-transfer detection filled readData with 0xFF).
				const uint32_t maxFirmwareLength = FirmwareFlashEnd - FirmwareFlashStart;
				if (request->firmwareLength == 0 || request->firmwareLength > maxFirmwareLength)
				{
					MessageF("ERROR: invalid firmware length %" PRIu32 " in verify request", request->firmwareLength);
					Reset(false);
				}
				uint16_t crc16 = CRC16(reinterpret_cast<const char*>(FirmwareFlashStart), request->firmwareLength);
				if (request->crc16 == crc16)
				{
					// Success!
					debugPrintf("Checksum OK!");
# ifdef IAP_SBC_USB
					if (usingUsb)
					{
						UsbSendVerifyResponse(0x0C);
					}
					else
# endif
# ifdef IAP_SBC_SPI
					{
						SpiSendVerifyResponse(0x0C);
					}
# endif
					state = SendingChecksumOK;
				}
				else
				{
					// Checksum mismatch
					MessageF("CRC mismatch");
# ifdef IAP_SBC_USB
					if (usingUsb)
					{
						UsbSendVerifyResponse(0xFF);
					}
					else
# endif
# ifdef IAP_SBC_SPI
					{
						SpiSendVerifyResponse(0xFF);
					}
# endif
					state = SendingChecksumError;
				}
				retry = 0;
			}
		}
		break;

	case SendingChecksumOK:
		{
			bool timedOut = false;
			bool complete = false;

# ifdef IAP_SBC_USB
			if (usingUsb)
			{
				// USB writes are immediate, go straight to locking
				complete = true;
			}
			else
# endif
# ifdef IAP_SBC_SPI
			{
				complete = SpiIsTransferComplete();
				if (!complete && millis() - spiTransferStartTime > TransferTimeout)
				{
					timedOut = true;
				}
			}
# endif

			if (timedOut)
			{
				// Although this is not expected, the firmware has been written successfully so just continue as normal
				MessageF("Timeout while exchanging checksum acknowledgement");
				delayMs(RetryMessageDelay);
				state = LockingFlash;
			}
			else if (complete)
			{
				state = LockingFlash;
			}
		}
		break;

	case SendingChecksumError:
		{
			bool timedOut = false;
			bool complete = false;

# ifdef IAP_SBC_USB
			if (usingUsb)
			{
				complete = true;
			}
			else
# endif
# ifdef IAP_SBC_SPI
			{
				complete = SpiIsTransferComplete();
				if (!complete && millis() - spiTransferStartTime > TransferTimeout)
				{
					timedOut = true;
				}
			}
# endif

			if (timedOut)
			{
				// Bad image has been flashed - restart to bossa
				MessageF("Timeout while reporting CRC error");
				Reset(false);
			}
			else if (complete)
			{
				++crcRetryCount;
				if (crcRetryCount >= MaxCrcRetries)
				{
					// Reflashing the whole image is not converging (persistent bad sector or corrupt source), so give
					// up rather than looping forever - the WritingUpgrade retry counter is reset on every successful
					// page write and so never catches this
					MessageF("ERROR: firmware checksum still wrong after %u attempts", crcRetryCount);
					Reset(false);
				}

				// Reset the transport state so the next read starts counting bytes from zero (otherwise the USB
				// length-based end-of-transfer detection sees the previous cycle's counter)
# ifdef IAP_SBC_USB
				if (usingUsb)
				{
					UsbResetTransferState();
				}
# endif
				flashPos = FirmwareFlashStart;

				// Erase the whole region before rewriting: Flash::Write requires pre-erased memory and NVMCTRL/EEFC
				// can only clear bits, so rewriting programmed pages cannot repair a corrupted image. SAME70 also
				// needs a full erase to avoid a boot failure after a page has been rewritten
				state = ErasingFlash;
				reportNextPercent = reportPercentIncrement;
				retry = 0;
			}
		}
		break;
#endif // IAP_VIA_SBC

	case LockingFlash:
		// Lock each single page again
		{
			// We can lock all the flash in one call. We may have to lock from before the firmware start.
			const uint32_t lockStart = FirmwareFlashStart & ~(Flash::GetLockRegionSize() - 1);
			debugPrintf("Locking 0x%08x - 0x%08x", lockStart, FirmwareFlashEnd - lockStart);
			if (Flash::Lock(lockStart, FirmwareFlashEnd - lockStart))
			{
				MessageF("Update successful! Rebooting...");
				delayMs(100);
				Reset(true);
			}
			else
			{
				++retry;
			}
		}
		break;
	}
}

[[noreturn]] void Reset(bool success) noexcept
{
	if (!success)
	{
		delayMs(4000);				// give the user a chance to read the error message on PanelDue
#if SAM4E || SAM4S || SAME70
		// Start from bootloader next time if the firmware couldn't be written entirely
		if (state >= WritingUpgrade)
		{
			Flash::ClearGpNvm(1);
		}
#elif SAME5x
		// Start from uf2 bootloader next time if the firmware couldn't be written entirely
		if (state >= WritingUpgrade)
		{
			*DBL_TAP_PTR = DBL_TAP_MAGIC;
		}
#endif
	}

#ifdef IAP_SBC_SPI
	if (!usingUsb)
	{
		SpiShutdown();
	}
#endif
#ifdef IAP_SBC_USB
	if (usingUsb)
	{
		UsbShutdown();			// give the host a clean USB disconnect before the bus reappears as the new firmware or bootloader
	}
#endif

	// No reason to lock the flash again
	digitalWrite(DiagLedPin, !LedOnPolarity);		// turn the LED off

	// Reboot
	ResetProcessor();
}

// Write message to PanelDue
// The message must not contain any characters that need JSON escaping, such as newline or " or \.
void MessageF(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	serialUart0.print("{\"message\":\"");
	serialUart0.vprintf(fmt, vargs);
	serialUart0.print("\"}\n");
	va_end(vargs);
}

// End
