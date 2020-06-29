/* In-Application Programming application for Duet3D platforms
 *
 * This application is first written by RepRapFirmware to the end of the second
 * Flash bank and then started by RepRapFirmware.
 *
 * Once this program is loaded, it performs in-application programming by
 * reading the new firmware binary from the SD card and replaces the corresponding
 * Flash content sector by sector.
 *
 * This application was written by Christian Hammacher (2016-2019) and is
 * licensed under the terms of the GPL v2.
 */

#ifndef IAP_H_INCLUDED

#include "Core.h"

#if SAM4S
# define IFLASH_ADDR		(IFLASH0_ADDR)
# define IFLASH_PAGE_SIZE	(IFLASH0_PAGE_SIZE)
#elif SAM3XA
# define IFLASH_ADDR		(IFLASH0_ADDR)
# define IFLASH_PAGE_SIZE	(IFLASH1_PAGE_SIZE)
#endif

const size_t baudRate = 115200;						// For USB diagnostics

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
# else	// Duet 06/085
#  define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {NoPin, NoPin};				// Don't use the Card Detect pin on the Duet 085
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {67};
const char * const defaultFwFile = "0:/sys/RepRapFirmware.bin";			// Which file shall be used for IAP?
const char * const fwFilePrefix = "0:/sys/RepRap";
# endif
#endif

#if SAM4E	// Duet 2 Ethernet/WiFi
# define USE_DMAC  1
# define USE_XDMAC 0
# define SERIAL_AUX_DEVICE Serial
constexpr Pin DiagLedPin = PortCPin(2);
# ifndef IAP_VIA_SPI
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {53, NoPin};
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {56};
const char * const defaultFwFile = "0:/sys/DuetWiFiFirmware.bin";		// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/Duet";

# else
// SPI interface and pins
#define SBC_SPI					SPI
#define SBC_SPI_INTERFACE_ID	ID_SPI
#define SBC_SPI_IRQn			SPI_IRQn
#define SBC_SPI_HANDLER			SPI_Handler
constexpr Pin APIN_SBC_SPI_MOSI = APIN_SPI_MOSI;
constexpr Pin APIN_SBC_SPI_MISO = APIN_SPI_MISO;
constexpr Pin APIN_SBC_SPI_SCK  = APIN_SPI_SCK;
constexpr Pin APIN_SBC_SPI_SS0  = APIN_SPI_SS0;

// Hardware IDs of the SPI transmit and receive DMA interfaces. See atsam datasheet.
const uint32_t SBC_SPI_TX_DMA_HW_ID = 1;
const uint32_t SBC_SPI_RX_DMA_HW_ID = 2;

constexpr Pin LinuxTfrReadyPin = PortDPin(31);
constexpr uint8_t DmacChanLinuxTx = 1;				// These two should be
constexpr uint8_t DmacChanLinuxRx = 2;				// kept in sync with RRF!
# endif
#endif

#if SAM4S	// Duet 2 Maestro
# define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {44, NoPin};
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {56};
const Pin DiagLedPin = 62;
const char * const defaultFwFile = "0:/sys/DuetMaestroFirmware.bin";	// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/Duet";
#endif

#if SAME70	// Duet 3
# define USE_DMAC  0
# define USE_XDMAC 1
# define SERIAL_AUX_DEVICE Serial
const size_t NumSdCards = 2;
const Pin DiagLedPin = PortCPin(20);

# if defined(IAP_VIA_SPI)

const uint32_t LINUX_XDMAC_TX_CH_NUM = 3;
const uint32_t LINUX_XDMAC_RX_CH_NUM = 4;
const uint8_t DmacChanLinuxTx = 5;				// These two should be
const uint8_t DmacChanLinuxRx = 6;				// kept in sync with RRF!

// Duet pin numbers for the Linux interface
#define SBC_SPI					SPI1
#define SBC_SPI_INTERFACE_ID	ID_SPI1
#define SBC_SPI_IRQn			SPI1_IRQn
#define SBC_SPI_HANDLER			SPI1_Handler

constexpr Pin APIN_SBC_SPI_MOSI = APIN_SPI1_MOSI;
constexpr Pin APIN_SBC_SPI_MISO = APIN_SPI1_MISO;
constexpr Pin APIN_SBC_SPI_SCK = APIN_SPI1_SCK;
constexpr Pin APIN_SBC_SPI_SS0 = APIN_SPI1_SS0;

constexpr Pin LinuxTfrReadyPin = PortEPin(2);

# else

const char * const defaultFwFile = "0:/sys/Duet3Firmware.bin";			// Which file shall we default to used for IAP?
const char * const fwFilePrefix = "0:/sys/Duet3";

const Pin SdCardDetectPins[NumSdCards] = { PortAPin(6), NoPin };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[1] = { NoPin };

# endif

# ifndef IAP_IN_RAM
const uint32_t iapFirmwareSize = 0x20000;								// 128 KiB max (SAME70 has 128kb flash sectors so we can't erase a smaller amount)
#endif

#else	// not a SAME70 variant

# ifndef IAP_IN_RAM
const uint32_t iapFirmwareSize = 0x10000;								// 64 KiB max
# endif

#endif

#ifdef IAP_IN_RAM
const uint32_t firmwareFlashEnd = IFLASH_ADDR + IFLASH_SIZE;
#else
const uint32_t firmwareFlashEnd = IFLASH_ADDR + IFLASH_SIZE - iapFirmwareSize;
#endif

#ifdef IAP_VIA_SPI

const uint32_t NvicPrioritySpi = 1;
const uint32_t TransferCompleteDelay = 400;								// DCS waits 500ms when the firmware image has been transferred
const uint32_t TransferTimeout = 2000;									// How long to wait before timing out

struct FlashVerifyRequest
{
	uint32_t firmwareLength;
	uint16_t crc16;
	uint16_t dummy;
};
#endif

// Read and write only 2 KiB of data at once (must be multiple of IFLASH_PAGE_SIZE).
// Unfortunately we cannot increase this value further, because f_read() would mess up data
const size_t blockReadSize = 2048;

const size_t maxRetries = 5;											// Allow 5 retries max if anything goes wrong

enum ProcessState
{
	Initializing,
	UnlockingFlash,
#if SAM4E || SAM4S || SAME70
	ErasingFlash,
#endif
	WritingUpgrade,
	LockingFlash,
#ifdef IAP_VIA_SPI
	VerifyingChecksum,
	SendingChecksumOK,
	SendingChecksumError
#endif
};

#ifndef IAP_VIA_SPI
void initFilesystem();
void getFirmwareFileName();
void openBinary();
void closeBinary();
#endif

void writeBinary();
void Reset(bool success);

void sendUSB(uint32_t ep, const void* d, uint32_t len);

#endif	// IAP_H_INCLUDED
