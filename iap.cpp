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

char readData[blockReadSize];
ProcessState state = UnlockingFlash;
uint32_t flashPos = IFLASH_ADDR;


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

	const size_t maxFirmwareSize = lastSectorAddress - IFLASH_ADDR;
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
	size_t bytesRead, bytesToRead, bytesToWrite;
	switch (state)
	{
		case UnlockingFlash:
			// Unlock each single page
			debugPrintf("Unlocking 0x%08x - 0x%08x\n", flashPos, flashPos + IFLASH_PAGE_SIZE - 1);

			cpu_irq_disable();
			flash_unlock(flashPos, flashPos + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			cpu_irq_enable();
			flashPos += IFLASH_PAGE_SIZE;

			// Make sure we stay within FW Flash area
			if (flashPos >= lastSectorAddress)
			{
				flashPos = IFLASH_ADDR;
				state = WritingUpgrade;
			}
			break;

		case WritingUpgrade:
			// Read a chunk from the upgrade file
			if (f_read(&upgradeBinary, readData, blockReadSize, &bytesRead) != FR_OK)
			{
				debugPrintf("ERROR: Could not read from upgrade file!\n");
				debugPrintf("Press the ERASE button and flash the new binary manually.\n");
				closeAndDeleteBinary();
				Reset();
			}

			// Have we finished the file?
			if (bytesRead == 0)
			{
				// Yes - now we just need to fill up the remaining pages with zeros
				closeAndDeleteBinary();

				memset(readData, 0, sizeof(readData));
				state = FillingZeros;
				break;
			}

			// No - write another chunk
			debugPrintf("Writing 0x%08x - 0x%08x\n", flashPos, flashPos + bytesRead);

			cpu_irq_disable();
			flash_write(flashPos, readData, bytesRead, 1);
			cpu_irq_enable();

			// Verify the data written
			if (memcmp(readData, reinterpret_cast<void *>(flashPos), bytesRead) != 0)
			{
				debugPrintf("ERROR: Verification during write failed!\n");
				debugPrintf("Press the ERASE button and flash the new binary manually.\n");
				closeAndDeleteBinary();
				Reset();
			}

			// Go to the next page
			flashPos += bytesRead;
			break;

		case FillingZeros:
			// We've finished the upgrade process, so fill up the remaining space with zeros
			bytesToWrite = min(lastSectorAddress - flashPos, blockReadSize);
			debugPrintf("Filling 0x%08x - 0x%08x with zeros\n", flashPos, flashPos + bytesToWrite);

			cpu_irq_disable();
			flash_write(flashPos, readData, sizeof(readData), 1);
			cpu_irq_enable();

			flashPos += bytesToWrite;
			if (flashPos >= lastSectorAddress)
			{
				flashPos = IFLASH_ADDR;
				state = LockingFlash;
			}
			break;

		case LockingFlash:
			// Lock each single page again
			debugPrintf("Locking 0x%08x - 0x%08x\n", flashPos, flashPos + IFLASH_PAGE_SIZE - 1);

			cpu_irq_disable();
			flash_lock(flashPos, flashPos + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			cpu_irq_enable();
			flashPos += IFLASH_PAGE_SIZE;

			// Make sure we stay within FW Flash area
			if (flashPos >= lastSectorAddress)
			{
				debugPrintf("Upgrade successful! Rebooting...\n");
				Reset();
			}
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
