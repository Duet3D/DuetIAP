/* In-Application Programming application for the RepRap Duet platform (Atmel SAM3X8E)
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

#include "iap.h"

#define DEBUG	0


FATFS fs;
FIL upgradeBinary;

uint32_t readData32[(blockReadSize + 3) / 4 + 1];	// should use aligned memory so DMA works well
char *readData = reinterpret_cast<char *>(readData32);

ProcessState state = UnlockingFlash;
uint32_t flashPos = IFLASH_ADDR;

size_t retry = 0;
size_t bytesRead, bytesToRead, bytesToWrite;


/** Arduino routines **/

void setup()
{
	debugPrintf("IAP Utility for Duet electronics\n");
	debugPrintf("Developed by Christian Hammacher (2016)\n");
	debugPrintf("Licensed under the terms of the GPLv2\n\n");

	initFilesystem();
	openBinary();
}

void loop()
{
	writeBinary();
}

void watchdogSetup()
{
	watchdogEnable(1000);
	// watchdog is kicked by the Arduino core
}


/** IAP routines **/

void initFilesystem()
{
	debugPrintf("Initialising SD card... ");

	memset(&fs, 0, sizeof(FATFS));
	sd_mmc_init();
	delay(20);

	bool abort = false;
	sd_mmc_err_t err;
	do {
		err = sd_mmc_check(0);
		if (err > SD_MMC_ERR_NO_CARD)
		{
			abort = true;
		}
		else
		{
			abort = (err == SD_MMC_ERR_NO_CARD && micros() > 5000000);
		}

		if (abort)
		{
			debugPrintf("ERROR: ");
			switch (err)
			{
				case SD_MMC_ERR_NO_CARD:
					debugPrintf("Card not found\n");
					break;
				case SD_MMC_ERR_UNUSABLE:
					debugPrintf("Card is unusable, try another one\n");
					break;
				case SD_MMC_ERR_SLOT:
					debugPrintf("Slot unknown\n");
					break;
				case SD_MMC_ERR_COMM:
					debugPrintf("General communication error\n");
					break;
				case SD_MMC_ERR_PARAM:
					debugPrintf("Illegal input parameter\n");
					break;
				case SD_MMC_ERR_WP:
					debugPrintf("Card write protected\n");
					break;
				default:
					debugPrintf("Unknown (code %d)\n", err);
					break;
			}
			Reset();
			return;
		}
	} while (err != SD_MMC_OK);

	int mounted = f_mount(0, &fs);
	if (mounted != FR_OK)
	{
		debugPrintf("Mount failed, code %d\n", mounted);
		Reset();
	}

	debugPrintf("Done\n");
}

// Open the upgrade binary file so we can use it for flashing
void openBinary()
{
	debugPrintf("Opening firmware binary... ");

	// Check if this file doesn't exceed our boundaries
	FILINFO info;
	info.lfname = nullptr;
	if (f_stat(fwFile, &info) != FR_OK)
	{
		debugPrintf("ERROR: Could not find upgrade file!\n");
		debugPrintf("Press the ERASE button and flash the new binary manually.\n");
		Reset();
	}

	const size_t maxFirmwareSize = IFLASH_SIZE - iapFirmwareSize;
	if (info.fsize > maxFirmwareSize)
	{
		debugPrintf("ERROR: The upgrade file is too big!\n");
		debugPrintf("Press the ERASE button and flash the new binary manually.\n");
		Reset();
	}

	// Try to open the file
	if (f_open(&upgradeBinary, fwFile, FA_OPEN_EXISTING | FA_READ) != FR_OK)
	{
		debugPrintf("ERROR: Could not open upgrade file!\n");
		debugPrintf("Press the ERASE button and flash the new binary manually.\n");
		Reset();
	}

	debugPrintf("Done\n");
}

// This implements the actual functionality of this program
void writeBinary()
{
	if (retry > maxRetries)
	{
		debugPrintf("ERROR: Operation failed after %d retries!\n", maxRetries);
		debugPrintf("Press the ERASE button and flash the new binary manually.\n");
		closeAndDeleteBinary();
		Reset();
	}

	switch (state)
	{
		case UnlockingFlash:
			// Unlock each single page
			debugPrintf("Unlocking 0x%08x - 0x%08x\n", flashPos, flashPos + IFLASH_PAGE_SIZE - 1);

			cpu_irq_disable();
			if (flash_unlock(flashPos, flashPos + IFLASH_PAGE_SIZE - 1, nullptr, nullptr) == FLASH_RC_OK)
			{
				flashPos += IFLASH_PAGE_SIZE;
				retry = 0;
			}
			else
			{
				retry++;
			}
			cpu_irq_enable();

			// Make sure we stay within FW Flash area
			if (flashPos >= firmwareFlashEnd)
			{
				flashPos = IFLASH_ADDR;
				state = WritingUpgrade;
			}
			break;

		case WritingUpgrade:
			// Attempt to read a chunk from the new firmware file
			if (retry == 0)
			{
				if (f_read(&upgradeBinary, readData, blockReadSize, &bytesRead) != FR_OK)
				{
					debugPrintf("ERROR: Could not read from upgrade file!\n");
					debugPrintf("Press the ERASE button and flash the new binary manually.\n");
					closeAndDeleteBinary();
					Reset();
				}

				// Have we finished the file?
				if (bytesRead != blockReadSize)
				{
					// Yes - close and delete it
					closeAndDeleteBinary();

					// Now we just need to fill up the remaining pages with zeros
					memset(readData + bytesRead, 0, blockReadSize - bytesRead);
				}
				bytesToWrite = min(firmwareFlashEnd - flashPos, blockReadSize);
			}

			// Write another chunk
			debugPrintf("Writing 0x%08x - 0x%08x\n", flashPos, flashPos + bytesToWrite - 1);

			cpu_irq_disable();
			if (flash_write(flashPos, readData, bytesToWrite, 1) != FLASH_RC_OK)
			{
				retry++;
				cpu_irq_enable();
				break;
			}
			cpu_irq_enable();

			// Verify the written data
			if (memcmp(readData, reinterpret_cast<void *>(flashPos), bytesToWrite) == 0)
			{
				flashPos += bytesToWrite;
				if (flashPos >= firmwareFlashEnd || bytesRead != bytesToWrite)
				{
					memset(readData, 0, blockReadSize);
					state = FillingZeros;
				}
				retry = 0;
			}
			else
			{
				retry++;
			}
			break;

		case FillingZeros:
			// We've finished the upgrade process, so fill up the remaining space with zeros
			bytesToWrite = min(firmwareFlashEnd - flashPos, blockReadSize);
			debugPrintf("Filling 0x%08x - 0x%08x with zeros\n", flashPos, flashPos + bytesToWrite - 1);

			cpu_irq_disable();
			if (flash_write(flashPos, readData, bytesToWrite, 1) == FLASH_RC_OK)
			{
				flashPos += bytesToWrite;
				retry = 0;
			}
			else
			{
				retry++;
				cpu_irq_enable();
				break;
			}
			cpu_irq_enable();

			// Verify the written data
			if (memcmp(readData, reinterpret_cast<void *>(flashPos), bytesToWrite) == 0)
			{
				flashPos += bytesToWrite;
				if (flashPos >= firmwareFlashEnd)
				{
					flashPos = IFLASH_ADDR;
					state = LockingFlash;
				}
				retry = 0;
			}
			else
			{
				retry++;
			}
			break;

		case LockingFlash:
			// Lock each single page again
			debugPrintf("Locking 0x%08x - 0x%08x\n", flashPos, flashPos + IFLASH_PAGE_SIZE - 1);

			cpu_irq_disable();
			if (flash_lock(flashPos, flashPos + IFLASH_PAGE_SIZE - 1, nullptr, nullptr) == FLASH_RC_OK)
			{
				flashPos += IFLASH_PAGE_SIZE;
				if (flashPos >= firmwareFlashEnd)
				{
					debugPrintf("Upgrade successful! Rebooting...\n");
					Reset();
				}
				retry = 0;
			}
			else
			{
				retry++;
			}
			cpu_irq_enable();
			break;
	}
}

// Does what it says
void closeAndDeleteBinary()
{
	// Close the FSO
	f_close(&upgradeBinary);

	// Unlink (delete) the file
	f_unlink(fwFile);
}


/** Helper functions **/

void debugPrintf(const char *fmt, ...)
{
#if DEBUG
	char msg[64];
	va_list vargs;
	va_start(vargs, fmt);
	vsnprintf(msg, 64, fmt, vargs);
	va_end(vargs);

	sendUSB(CDC_TX, msg, strlen(msg));
#endif
}

// We have to use our own USB transmit function here, because the Arduino core will
// assume that the USB line is closed its equivalent is called...
void sendUSB(uint32_t ep, const void* d, uint32_t len)
{
    uint32_t n;
	int r = len;
	const uint8_t* data = (const uint8_t*)d;

	while (len > 0)
	{
        if(ep==0) n = EP0_SIZE;
        else n =  EPX_SIZE;
		if (n > len)
			n = len;
		len -= n;

		UDD_Send(ep & 0xF, data, n);
		data += n;
    }

	// Not sure why, but this doesn't always work...
	if (UDD_FifoByteCount(ep) > 0)
	{
		UDD_ReleaseTX(ep);
	}
}

void Reset()
{
#if DEBUG
	// If debugging is enabled start from bootloader next time
	flash_clear_gpnvm(1);
#endif
	rstc_start_software_reset(RSTC);
	while(true);
}

// vim: ts=4:sw=4
