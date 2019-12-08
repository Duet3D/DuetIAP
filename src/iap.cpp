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
#include "DueFlashStorage.h"
#include "ff.h"
#include "sd_mmc.h"
#include "rstc/rstc.h"
#include <General/SafeVsnprintf.h>

#include <cstdarg>

#define DEBUG	0

#if SAM4E || SAM4S || SAME70

// Later Duets have a diagnostic LED, which we flash regularly to indicate activity
const uint32_t LedOnOffMillis = 100;

uint32_t lastLedMillis;
bool ledIsOn;

#endif

FATFS fs;
FIL upgradeBinary;

uint32_t readData32[(blockReadSize + 3) / 4];	// use aligned memory so DMA works well
char * const readData = reinterpret_cast<char *>(readData32);

const char* fwFile = defaultFwFile;

ProcessState state = Initializing;
uint32_t flashPos = IFLASH_ADDR;

size_t retry = 0;
size_t bytesRead, bytesWritten;
bool haveDataInBuffer;
const size_t reportPercentIncrement = 20;
size_t reportNextPercent = reportPercentIncrement;

char formatBuffer[100];

void checkLed()
{
#if SAM4E || SAM4S || SAME70
	const uint32_t now = millis();
	if (now - lastLedMillis >= LedOnOffMillis)
	{
		ledIsOn = !ledIsOn;
		digitalWrite(DiagLedPin, ledIsOn);
		lastLedMillis = now;
	}
#endif
}

// Our own version of delay() that keeps the LED up to date
void delay_ms(uint32_t ms)
{
	const uint32_t startTime = millis();
	do
	{
		checkLed();
	} while (millis() - startTime < ms);
}

void debugPrintf(const char *fmt, ...);			// forward declaration
void MessageF(const char *fmt, ...);			// forward declaration

extern "C" void UrgentInit() { }

extern "C" void SysTick_Handler(void)
{
	CoreSysTick();
	wdt_restart(WDT);							// kick the watchdog

#if SAM4E || SAME70
	rswdt_restart(RSWDT);						// kick the secondary watchdog
#endif
}

extern "C" void SVC_Handler() { for (;;) {} }
extern "C" void PendSV_Handler() { for (;;) {} }

extern "C" void AppMain()
{
	SysTickInit();

#if SAM4E || SAM4S || SAME70
	digitalWrite(DiagLedPin, true);				// turn the LED on
	ledIsOn = true;
	lastLedMillis = millis();
#endif

	SERIAL_AUX_DEVICE.begin(57600);				// set serial port to default PanelDue baud rate
	MessageF("IAP started");

	debugPrintf("IAP Utility for Duet electronics\n");
	debugPrintf("Developed by Christian Hammacher (2016)\n");
	debugPrintf("Licensed under the terms of the GPLv2\n\n");

	initFilesystem();
	getFirmwareFileName();
	openBinary();
	for (;;)
	{
		checkLed();
		writeBinary();
	}
}

/** IAP routines **/

void initFilesystem()
{
	debugPrintf("Initialising SD card... ");

	memset(&fs, 0, sizeof(FATFS));
	sd_mmc_init(SdWriteProtectPins, SdSpiCSPins);
	delay_ms(20);

	const size_t startTime = millis();
	sd_mmc_err_t err;
	do {
		err = sd_mmc_check(0);
		if (err > SD_MMC_ERR_NO_CARD)
		{
			break;
		}
		delay_ms(1);
	} while (err != SD_MMC_OK && millis() - startTime < 5000);

	if (err == SD_MMC_OK)
	{
		MessageF("SD card initialised OK");
		debugPrintf("Done\n");
	}
	else
	{
		debugPrintf("ERROR: ");
		switch (err)
		{
			case SD_MMC_ERR_NO_CARD:
				MessageF("SD card not found");
				debugPrintf("Card not found\n");
				break;
			case SD_MMC_ERR_UNUSABLE:
				MessageF("SD card is unusable, try another one");
				debugPrintf("Card is unusable, try another one\n");
				break;
			case SD_MMC_ERR_SLOT:
				MessageF("SD slot unknown");
				debugPrintf("Slot unknown\n");
				break;
			case SD_MMC_ERR_COMM:
				MessageF("SD card communication error");
				debugPrintf("General communication error\n");
				break;
			case SD_MMC_ERR_PARAM:
				MessageF("SD interface illegal input parameter");
				debugPrintf("Illegal input parameter\n");
				break;
			case SD_MMC_ERR_WP:
				MessageF("SD card write protected");
				debugPrintf("Card write protected\n");
				break;
			default:
				MessageF("SD interface unknown error, code %d", err);
				debugPrintf("Unknown (code %d)\n", err);
				break;
		}
		Reset(false);
		return;
	}

	const int mounted = f_mount(0, &fs);
	if (mounted != FR_OK)
	{
		MessageF("SD card mount failed, code %d", mounted);
		debugPrintf("Mount failed, code %d\n", mounted);
		Reset(false);
	}
}

// Determine the name of the firmware file we need to flash
// Later releases of DuetWiFiFirmware and all releases of DuetEthernetFirmware put the initial stack pointer
// a little below the top of RAM and store the firmware filename just above the stack
void getFirmwareFileName()
{
	const uint32_t vtab = SCB->VTOR & SCB_VTOR_TBLOFF_Msk;
	const uint32_t stackTop = *reinterpret_cast<const uint32_t*>(vtab);
	const char* const fwFilePtr = reinterpret_cast<const char*>(stackTop);
	for (size_t i = 0; fwFilePrefix[i] != 0; ++i)
	{
		if (fwFilePtr[i] != fwFilePrefix[i])
		{
			return;			// we didn't find the expected prefix, so no filename was passed
		}
	}
	fwFile = fwFilePtr;		// replace default filename by the one we were passed
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
		MessageF("ERROR: Could not find file %s", fwFile);
		debugPrintf("ERROR: Could not find upgrade file!\n");
		Reset(false);
	}

	const size_t maxFirmwareSize = IFLASH_SIZE - iapFirmwareSize;
	if (info.fsize > maxFirmwareSize)
	{
		MessageF("ERROR: File %s is too big", fwFile);
		debugPrintf("ERROR: The upgrade file is too big!\n");
		Reset(false);
	}

	// Try to open the file
	if (f_open(&upgradeBinary, fwFile, FA_OPEN_EXISTING | FA_READ) != FR_OK)
	{
		MessageF("ERROR: Could not open file %s", fwFile);
		debugPrintf("ERROR: Could not open upgrade file!\n");
		Reset(false);
	}

	MessageF("File %s opened", fwFile);
	debugPrintf("Done\n");
}

void ShowProgress()
{
	const size_t percentDone = (100 * (flashPos - IFLASH_ADDR))/(firmwareFlashEnd - IFLASH_ADDR);
	if (percentDone >= reportNextPercent)
	{
		MessageF("Flashing firmware, %u%% completed", percentDone);
		reportNextPercent += reportPercentIncrement;
	}
}

// This implements the actual functionality of this program
void writeBinary()
{
	if (retry > maxRetries)
	{
		MessageF("ERROR: Operation failed after %d retries", maxRetries);
		debugPrintf("ERROR: Operation failed after %d retries!\n", maxRetries);
		Reset(false);
	}
	else if (retry > 0)
	{
		debugPrintf("WARNING: Retry %d of %d at pos %08x\n", retry, maxRetries, flashPos);
	}

	switch (state)
	{
	case Initializing:
		MessageF("Unlocking flash");	//TEMP
		state = UnlockingFlash;
		// no break
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
			cpu_irq_enable();
			break;
		}
		cpu_irq_enable();

		// Make sure we stay within FW Flash area
		if (flashPos >= firmwareFlashEnd)
		{
			flashPos = IFLASH_ADDR;
#if SAM4E || SAM4S || SAME70
			MessageF("Erasing flash");	//TEMP
			state = ErasingFlash;
#else
			bytesWritten = blockReadSize;
			state = WritingUpgrade;
#endif
		}
		break;

#if SAM4E || SAM4S || SAME70
	case ErasingFlash:
		debugPrintf("Erasing 0x%08x\n", flashPos);
		if (retry != 0)
		{
			MessageF("Erase retry #%u", retry);
		}

		{
			const uint32_t sectorSize =
# if SAM4E || SAM4S
				// Deal with varying size sectors on the SAM4E
				// There are two 8K sectors, then one 48K sector, then seven 64K sectors
				(flashPos - IFLASH_ADDR < 16 * 1024) ? 8 * 1024
					: (flashPos - IFLASH_ADDR == 16 * 1024) ? 48 * 1024
						: 64 * 1024;
#elif SAME70
				// Deal with varying size sectors on the SAME70
				// There are two 8K sectors, then one 112K sector, then the rest are 128K sectors
				(flashPos - IFLASH_ADDR < 16 * 1024) ? 8 * 1024
					: (flashPos - IFLASH_ADDR == 16 * 1024) ? 112 * 1024
						: 128 * 1024;
#endif

			if (flash_erase_sector(flashPos) == FLASH_RC_OK)
			{
				// Check that the sector really is erased
				for (uint32_t p = flashPos; p < flashPos + sectorSize; p += sizeof(uint32_t))
				{
					if (*reinterpret_cast<const uint32_t*>(p) != 0xFFFFFFFF)
					{
						++retry;
						break;
					}
				}
				retry = 0;
				flashPos += sectorSize;
			}
			else
			{
				++retry;
			}
			if (flashPos >= firmwareFlashEnd)
			{
				flashPos = IFLASH_ADDR;
				haveDataInBuffer = false;
				state = WritingUpgrade;
			}
		}
		break;
#endif

	case WritingUpgrade:
		// Attempt to read a chunk from the new firmware file
		if (!haveDataInBuffer)
		{
			debugPrintf("Reading %u bytes from the file\n", blockReadSize);
			if (retry != 0)
			{
				MessageF("Read file retry #%u", retry);
			}

			// Seek to the correct place in case we are doing retries
			FRESULT result = f_lseek(&upgradeBinary, flashPos - IFLASH_ADDR);
			if (result != FR_OK)
			{
				debugPrintf("WARNING: f_lseek returned err %d\n", result);
				delay_ms(100);
				retry++;
				break;
			}

			result = f_read(&upgradeBinary, readData, blockReadSize, &bytesRead);
			if (result != FR_OK)
			{
				debugPrintf("WARNING: f_read returned err %d\n", result);
				delay_ms(100);
				retry++;
				break;
			}

			// Have we finished the file?
			if (bytesRead < blockReadSize)
			{
				// Yes - close it
				closeBinary();

				// Now we just need to fill up the remaining part of the buffer with 0xFF
				memset(readData + bytesRead, 0xFF, blockReadSize - bytesRead);
			}

			haveDataInBuffer = true;
			retry = 0;
			bytesWritten = 0;
		}

		// Write another page
		{
			debugPrintf("Writing 0x%08x - 0x%08x\n", flashPos, flashPos + IFLASH_PAGE_SIZE - 1);
			if (retry != 0)
			{
				MessageF("Flash write retry #%u", retry);
			}

			cpu_irq_disable();
			const uint32_t rc =
#if SAM4E || SAM4S || SAME70
								flash_write(flashPos, readData + bytesWritten, IFLASH_PAGE_SIZE, 0);
#else
								flash_write(flashPos, readData + bytesWritten, IFLASH_PAGE_SIZE, 1);
#endif
			cpu_irq_enable();
			if (rc != FLASH_RC_OK)
			{
				retry++;
				break;
			}

			// Verify the written data
			if (memcmp(readData + bytesWritten, reinterpret_cast<void *>(flashPos), IFLASH_PAGE_SIZE) == 0)
			{
				retry = 0;
				bytesWritten += IFLASH_PAGE_SIZE;
				flashPos += IFLASH_PAGE_SIZE;
				ShowProgress();
				if (bytesWritten == blockReadSize)
				{
					haveDataInBuffer = false;
					if (bytesRead < blockReadSize)
					{
						state = LockingFlash;
					}
				}
			}
			else
			{
				retry++;
			}
		}
		break;

	case LockingFlash:
		// Lock each single page again
		{
			debugPrintf("Locking 0x%08x - 0x%08x\n", flashPos, flashPos + IFLASH_PAGE_SIZE - 1);

			cpu_irq_disable();
			const uint32_t rc = flash_lock(flashPos, flashPos + IFLASH_PAGE_SIZE - 1, nullptr, nullptr);
			cpu_irq_enable();
			if (rc == FLASH_RC_OK)
			{
				flashPos += IFLASH_PAGE_SIZE;
				if (flashPos >= firmwareFlashEnd)
				{
					MessageF("Update successful! Rebooting...");
					debugPrintf("Upgrade successful! Rebooting...\n");
					Reset(true);
				}
				retry = 0;
			}
			else
			{
				retry++;
			}
		}
		break;
	}
}

// Does what it says
void closeBinary()
{
	f_close(&upgradeBinary);
}

void Reset(bool success)
{
	// Only start from bootloader if the firmware couldn't be written entirely
	if (!success && state >= WritingUpgrade)
	{
		cpu_irq_disable();

		// If anything went wrong, write the last error message to Flash to the beginning
		// of the Flash memory. That may help finding out what went wrong...
		flash_unlock(IFLASH_ADDR, IFLASH_ADDR + 64, nullptr, nullptr);
		flash_write(IFLASH_ADDR, formatBuffer, strlen(formatBuffer), 1);
		// no reason to lock it again

		// Start from bootloader next time
		flash_clear_gpnvm(1);

		cpu_irq_enable();
	}

	delay_ms(500);				// allow last message to PanelDue to go

#if SAM4E || SAM4S || SAME70
	digitalWrite(DiagLedPin, false);		// turn the LED off
#endif

	// Reboot
	rstc_start_software_reset(RSTC);
	while(true);
}

/** Helper functions **/

void debugPrintf(const char *fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

#if DEBUG
	sendUSB(CDC_TX, formatBuffer, strlen(formatBuffer));
#endif
}

// Write message to aux.
// The message must not contain any characters that need JSON escaping, such as newline or " or \.
void MessageF(const char *fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

	SERIAL_AUX_DEVICE.print("{\"message\":\"");
	SERIAL_AUX_DEVICE.print(formatBuffer);
	SERIAL_AUX_DEVICE.print("\"}\n");
	delay_ms(10);
}

// The following functions are called by the startup code in CoreNG.
// We define our own versions here to make the binary smaller, because we don't use the associated functionality.
void AnalogInInit()
{
}

extern "C" void TWI0_Handler()
{
}

extern "C" void TWI1_Handler()
{
}

// Cache hooks called from the ASF. These are dummy because we run with the cache disabled.
extern "C" void CacheFlushBeforeDMAReceive(const volatile void *start, size_t length) { }
extern "C" void CacheInvalidateAfterDMAReceive(const volatile void *start, size_t length) { }
extern "C" void CacheFlushBeforeDMASend(const volatile void *start, size_t length) { }

#if DEBUG
// We have to use our own USB transmit function here, because the core will assume that the USB line is closed
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
#endif

// End
