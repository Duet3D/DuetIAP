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

#include <CoreIO.h>

#if SAM4S
# define IFLASH_ADDR		(IFLASH0_ADDR)
# define IFLASH_PAGE_SIZE	(IFLASH0_PAGE_SIZE)
#else
// IFLASH_ADDR and IFLASH_PAGE_SIZE are already defined for the SAM4E and the SAME70
#endif

#if SAM4E	// Duet 2 Ethernet/WiFi
# define USE_DMAC  1
# define USE_XDMAC 0
# define USE_DMAC_MANAGER 0
constexpr Pin DiagLedPin = PortCPin(2);
constexpr bool LedOnPolarity = true;

# ifdef IAP_VIA_SPI

// SPI interface and pins
#define SBC_SPI					SPI
#define SBC_SPI_INTERFACE_ID	ID_SPI
#define SBC_SPI_IRQn			SPI_IRQn
#define SBC_SPI_HANDLER			SPI_Handler

constexpr Pin APIN_SBC_SPI_MOSI = PortAPin(13);
constexpr Pin APIN_SBC_SPI_MISO = PortAPin(12);
constexpr Pin APIN_SBC_SPI_SCK  = PortAPin(14);
constexpr Pin APIN_SBC_SPI_SS0  = PortAPin(11);
constexpr GpioPinFunction SpiPinsFunction = GpioPinFunction::A;

constexpr Pin SbcTfrReadyPin = PortDPin(31);

// Hardware IDs of the SPI transmit and receive DMA interfaces. See atsam datasheet.
const uint32_t SBC_SPI_TX_DMA_HW_ID = 1;
const uint32_t SBC_SPI_RX_DMA_HW_ID = 2;

constexpr uint8_t DmacChanSbcTx = 1;				// These two should be
constexpr uint8_t DmacChanSbcRx = 2;				// kept in sync with RRF!

#else

const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = { PortCPin(21), NoPin };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[1] = { PortCPin(24) };
const char * const defaultFwFile = "0:/firmware/Duet2Combinedirmware.bin";		// which file shall we default to used for IAP?

# endif
#endif

#if SAM4S	// Duet 2 Maestro
# define USE_DMAC_MANAGER 0
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = { PortCPin(8), NoPin };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[1] = { PortBPin(13) };
const Pin DiagLedPin = PortCPin(26);
constexpr bool LedOnPolarity = true;

const char * const defaultFwFile = "0:/firmware/DuetMaestroFirmware.bin";	// Which file shall we default to used for IAP?
#endif

#if SAME70	// Duet 3
# define USE_DMAC  0
# define USE_XDMAC 1
# define USE_DMAC_MANAGER 0
const size_t NumSdCards = 2;

# if defined(DUET3_MB6HC)
const Pin DiagLedPin = PortCPin(20);
constexpr bool LedOnPolarity = true;
# elif defined(DUET3_MB6XD)
const Pin DiagLedPin = PortBPin(6);
constexpr bool LedOnPolarity = false;
# else
#  error Unknown board
# endif

# ifdef IAP_VIA_SPI

const uint32_t SBC_SPI_TX_PERID = 3;
const uint32_t SBC_SPI_RX_PERID = 4;
const uint8_t DmacChanSbcTx = 5;				// These two should be
const uint8_t DmacChanSbcRx = 6;				// kept in sync with RRF!

// Duet pin numbers for the SBC interface
#define SBC_SPI					SPI1
#define SBC_SPI_INTERFACE_ID	ID_SPI1
#define SBC_SPI_IRQn			SPI1_IRQn
#define SBC_SPI_HANDLER			SPI1_Handler

constexpr Pin APIN_SBC_SPI_MOSI = PortCPin(27);
constexpr Pin APIN_SBC_SPI_MISO = PortCPin(26);
constexpr Pin APIN_SBC_SPI_SCK = PortCPin(24);
constexpr Pin APIN_SBC_SPI_SS0 = PortCPin(25);
constexpr GpioPinFunction SpiPinsFunction = GpioPinFunction::C;

constexpr Pin SbcTfrReadyPin = PortEPin(2);

# else

#  if defined(DUET3_MB6HC)
const char * const defaultFwFile = "0:/firmware/Duet3Firmware_MB6HC.bin";			// which file shall we default to used for IAP?
# elif defined(DUET3_MB6XD)
const char * const defaultFwFile = "0:/firmware/Duet3Firmware_MB6XD.bin";			// which file shall we default to used for IAP?
# else
#  error Unknown board
# endif

const Pin SdCardDetectPins[NumSdCards] = { PortAPin(6), NoPin };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
const Pin SdSpiCSPins[1] = { NoPin };

# endif
#endif	// SAME70

#if SAME5x	// Duet 3 Mini
# define USE_DMAC 			0
# define USE_XDMAC 			0
# define USE_DMAC_MANAGER	1

// Magic address and value to launch the uf2 bootloader on failure, see inc/uf2.h in uf2-samdx1 repository
# define DBL_TAP_PTR ((volatile uint32_t *)(HSRAM_ADDR + HSRAM_SIZE - 4))
# define DBL_TAP_MAGIC 0xf01669ef // Randomly selected, adjusted to have first and last bit set

// Serial on IO0
constexpr uint8_t Serial0SercomNumber = 2;
constexpr uint8_t Sercom0RxPad = 1;
#define SERIAL0_ISR0	SERCOM2_0_Handler
#define SERIAL0_ISR1	SERCOM2_1_Handler
#define SERIAL0_ISR2	SERCOM2_2_Handler
#define SERIAL0_ISR3	SERCOM2_3_Handler

constexpr Pin Serial0TxPin = PortBPin(25);
constexpr Pin Serial0RxPin = PortBPin(24);
constexpr GpioPinFunction Serial0PinFunction = GpioPinFunction::D;

const size_t NumSdCards = 1;
const Pin DiagLedPin = PortAPin(31);
constexpr bool LedOnPolarity = false;

# ifdef IAP_VIA_SPI

#  define SBC_SPI_HANDLER SERCOM0_3_Handler
#  define USE_32BIT_TRANSFERS 1
constexpr unsigned int SbcSpiSercomNumber = 0;
Sercom * const SbcSpiSercom = SERCOM0;
constexpr IRQn SBC_SPI_IRQn = SERCOM0_3_IRQn;			// this is the SS Low interrupt, the only one we use

//constexpr Pin SbcMosiPin = PortAPin(7);
//constexpr Pin SbcMisoPin = PortAPin(4);
//constexpr Pin SbcSclkPin = PortAPin(5);
constexpr Pin SbcSSPin = PortAPin(6);
constexpr Pin SbcTfrReadyPin = PortAPin(3);
constexpr Pin SbcSpiSercomPins[] = { PortAPin(4), PortAPin(5), PortAPin(6), PortAPin(7) };
constexpr GpioPinFunction SbcSpiSercomPinsMode = GpioPinFunction::D;

constexpr DmaChannel DmacChanSbcTx = 8;
constexpr DmaChannel DmacChanSbcRx = 9;
constexpr DmaPriority DmacPrioSbc = 3;					// high speed SPI in slave mode

# else

const char * const defaultFwFile = "0:/firmware/Duet3Firmware_Mini5plus.uf2";	// which file shall we default to used for IAP?

const Pin SdCardDetectPins[NumSdCards] = { PortBPin(16) };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin };
const Pin SdSpiCSPins[1] = { NoPin };

# endif
#endif	// SAME70

#if SAME5x
const uint32_t BootloaderSize = 0x4000;									// we have a 16K USB bootloader
const uint32_t FirmwareFlashStart = FLASH_ADDR + BootloaderSize;
const uint32_t FirmwareFlashEnd = FLASH_ADDR + FLASH_SIZE - (16 * 1024);	// allow 16K for SMART EEPROM
const uint32_t IFLASH_ADDR = FLASH_ADDR;								// define this so that error message print can print the offset
#else
const uint32_t FirmwareFlashStart = IFLASH_ADDR;
const uint32_t FirmwareFlashEnd = IFLASH_ADDR + IFLASH_SIZE;
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

#else

const char * const fwFilePrefix = "0:/";								// we expect this at the start of a firmware file name

void initFilesystem();
void getFirmwareFileName();
void openBinary();
void closeBinary();

#endif

// Read and write only 2 KiB of data at once (must be multiple of IFLASH_PAGE_SIZE).
const size_t blockReadSize = 2048;
const size_t maxRetries = 5;											// Allow 5 retries max if anything goes wrong

enum ProcessState
{
	Initializing,
	UnlockingFlash,
#if SAM4E || SAM4S || SAME70 || SAME5x
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

void writeBinary();
[[noreturn]] void Reset(bool success);

#endif	// IAP_H_INCLUDED
