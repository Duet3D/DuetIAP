/* SD card transport for DuetIAP
 * Reads firmware data from a file on the SD card (binary or UF2 format)
 */

#include "iap_sd.h"

#ifndef IAP_VIA_SBC

#include "ff.h"
#include "Libraries/sd_mmc/sd_mmc.h"

#include <General/StringFunctions.h>

#include <cstring>

// SD card state
static FATFS fs;
static FIL upgradeBinary;
static const char *fwFile = defaultFwFile;

uint32_t firmwareFileSize;
bool isUf2File;
unsigned int eraseRetryCount = 0;
uint32_t lastEraseRetryPos = 0xFFFFFFFF;

// UF2 file format support
struct UF2_Block
{
	// 32 byte header
	uint32_t magicStart0;
	uint32_t magicStart1;
	uint32_t flags;
	uint32_t targetAddr;
	uint32_t payloadSize;
	uint32_t blockNo;
	uint32_t numBlocks;
	uint32_t fileSize;		// or familyID
	uint8_t data[476];
	uint32_t magicEnd;

	static constexpr uint32_t MagicStart0Val = 0x0A324655;
	static constexpr uint32_t MagicStart1Val = 0x9E5D5157;
	static constexpr uint32_t MagicEndVal = 0x0AB16F30;
};

static void initFilesystem() noexcept
{
	debugPrintf("Initialising SD card");

	memset(&fs, 0, sizeof(FATFS));
	sd_mmc_init(SdWriteProtectPins, SdSpiCSPins);
	delayMs(20);

	const size_t startTime = millis();
	sd_mmc_err_t err;
	do {
		err = sd_mmc_check(0);
		if (err > SD_MMC_ERR_NO_CARD)
		{
			break;
		}
		delayMs(1);
	} while (err != SD_MMC_OK && millis() - startTime < 5000);

	if (err == SD_MMC_OK)
	{
		MessageF("SD card initialised OK");
	}
	else
	{
		switch (err)
		{
			case SD_MMC_ERR_NO_CARD:
				MessageF("SD card not found");
				break;
			case SD_MMC_ERR_UNUSABLE:
				MessageF("SD card is unusable, try another one");
				break;
			case SD_MMC_ERR_SLOT:
				MessageF("SD slot unknown");
				break;
			case SD_MMC_ERR_COMM:
				MessageF("SD card communication error");
				break;
			case SD_MMC_ERR_PARAM:
				MessageF("SD interface illegal input parameter");
				break;
			case SD_MMC_ERR_WP:
				MessageF("SD card write protected");
				break;
			default:
				MessageF("SD interface unknown error, code %d", err);
				break;
		}
		Reset(false);
		return;
	}

	const int mounted = f_mount(0, &fs);
	if (mounted != FR_OK)
	{
		MessageF("SD card mount failed, code %d", mounted);
		Reset(false);
	}
}

// Determine the name of the firmware file we need to flash.
// Later releases of DuetWiFiFirmware and all releases of DuetEthernetFirmware put the initial stack pointer
// a little below the top of RAM and store the firmware filename just above the stack
static void getFirmwareFileName(const IapInfo *iapInfo) noexcept
{
	if (iapInfo->magic == IapInfo::MagicValue)
	{
		// New-style: filename in the IapInfo struct
		if (iapInfo->firmwareFilename[0] != 0)
		{
			fwFile = iapInfo->firmwareFilename;
		}
	}
	else
	{
		// Fallback for older RRF that writes a raw filename string above the stack
		const char * const fwFilePtr = reinterpret_cast<const char*>(iapInfo);
		for (size_t i = 0; fwFilePrefix[i] != 0; ++i)
		{
			if (fwFilePtr[i] != fwFilePrefix[i])
			{
				return;			// no filename passed, use default
			}
		}
		fwFile = fwFilePtr;
	}
	isUf2File = StringEndsWithIgnoreCase(fwFile, ".uf2");
}

// Open the upgrade binary file so we can use it for flashing
static void openBinary() noexcept
{
	debugPrintf("Opening firmware binary");

	// Check if this file doesn't exceed our boundaries
	FILINFO info;
	info.lfname = nullptr;
	if (f_stat(fwFile, &info) != FR_OK)
	{
		MessageF("ERROR: Could not find file %s", fwFile);
		Reset(false);
	}

	size_t maxFirmwareFileSize = FirmwareFlashEnd - FirmwareFlashStart;
	if (isUf2File)
	{
		maxFirmwareFileSize *= 2;
	}
	if (info.fsize > maxFirmwareFileSize)
	{
		MessageF("ERROR: File %s is too big", fwFile);
		Reset(false);
	}

	firmwareFileSize = info.fsize;

	// Try to open the file
	if (f_open(&upgradeBinary, fwFile, FA_OPEN_EXISTING | FA_READ) != FR_OK)
	{
		MessageF("ERROR: Could not open file %s", fwFile);
		Reset(false);
	}

	MessageF("File %s opened", fwFile);
}

// Read a block of data when the file is a .uf2 file.
// We rely on Duet .uf2 files always being sequential and having 256 bytes of data per 512 byte block
static bool ReadBlockUf2() noexcept
{
	static UF2_Block uf2Buffer;

	// Seek to the correct place in case we are doing retries
	const uint32_t seekPos = (flashPos - FirmwareFlashStart) * 2;
	FRESULT result = f_lseek(&upgradeBinary, seekPos);
	if (result != FR_OK)
	{
		debugPrintf("WARNING: f_lseek returned err %d", result);
		delayMs(100);
		retry++;
		return false;
	}

	bytesRead = 0;
	do
	{
		if (seekPos + (bytesRead * 2) == firmwareFileSize)
		{
			// Now we just need to fill up the remaining part of the buffer with 0xFF
			memset(readData + bytesRead, 0xFF, blockReadSize - bytesRead);
			return true;
		}

		size_t locBytesRead;
		result = f_read(&upgradeBinary, &uf2Buffer, sizeof(uf2Buffer), &locBytesRead);
		if (result != FR_OK)
		{
			debugPrintf("WARNING: f_read returned err %d", result);
			delayMs(100);
			retry++;
			return false;
		}
		if (locBytesRead != sizeof(uf2Buffer))
		{
			//TODO just quit?
			debugPrintf("WARNING: UF2 block read returned only %u bytes", locBytesRead);
			delayMs(100);
			retry++;
			return false;
		}
		if (uf2Buffer.magicStart0 != UF2_Block::MagicStart0Val || uf2Buffer.magicStart1 != UF2_Block::MagicStart1Val || uf2Buffer.magicEnd != UF2_Block::MagicEndVal)
		{
			//TODO just quit?
			MessageF("ERROR: bad UF2 block at offset %" PRIu32, seekPos + bytesRead);
			Reset(false);
			return false;
		}
		if (uf2Buffer.targetAddr != flashPos + bytesRead || uf2Buffer.payloadSize != 256)
		{
			//TODO just quit?
			MessageF("ERROR: unexpected data in UF2 block at offset %" PRIu32, seekPos + bytesRead);
			Reset(false);
			return false;
		}
		memcpy(readData + bytesRead, uf2Buffer.data, 256);
		bytesRead += 256;
	} while (bytesRead < blockReadSize);

	return true;
}

// Public functions

void SdInit(const IapInfo *iapInfo) noexcept
{
	initFilesystem();
	getFirmwareFileName(iapInfo);
	openBinary();
}

bool SdReadBlock() noexcept
{
	debugPrintf("Reading %u bytes from the file", blockReadSize);
	if (retry != 0)
	{
		MessageF("Read file retry #%u", retry);
		delayMs(RetryMessageDelay);
	}

	if (isUf2File)
	{
		return ReadBlockUf2();
	}

	// Seek to the correct place in case we are doing retries
	FRESULT result = f_lseek(&upgradeBinary, flashPos - FirmwareFlashStart);
	if (result != FR_OK)
	{
		debugPrintf("WARNING: f_lseek returned err %d", result);
		delayMs(100);
		retry++;
		return false;
	}

	result = f_read(&upgradeBinary, readData, blockReadSize, &bytesRead);
	if (result != FR_OK)
	{
		debugPrintf("WARNING: f_read returned err %d", result);
		delayMs(100);
		retry++;
		return false;
	}

	// Have we finished the file?
	if (bytesRead < blockReadSize)
	{
		// Yes, now we just need to fill up the remaining part of the buffer with 0xFF
		memset(readData + bytesRead, 0xFF, blockReadSize - bytesRead);
	}

	return true;
}

void SdClose() noexcept
{
	f_close(&upgradeBinary);
}

#endif // !IAP_VIA_SBC
