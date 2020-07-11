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

#include "iap.h"

#if SAME5x
# include <Uart.h>
extern Uart serialUart0;
# define SERIAL_AUX_DEVICE serialUart0
#else
# include "rstc/rstc.h"
# include "flash_efc.h"
#endif

#include <General/SafeVsnprintf.h>

#ifndef IAP_VIA_SPI
# include "ff.h"
# include "Libraries/sd_mmc/sd_mmc.h"
#endif

#include <cstdarg>
#include <cstring>

#define DEBUG	0

#if SAM4E || SAM4S || SAME70 || SAME5x

# ifdef IAP_VIA_SPI

# if USE_DMAC
# include "dmac/dmac.h"
# include "matrix/matrix.h"
# endif

# if USE_XDMAC
# include "xdmac/xdmac.h"
# endif
#endif // IAP_VIA_SPI

// Later Duets have a diagnostic LED, which we flash regularly to indicate activity
const uint32_t LedOnOffMillis = 100;

uint32_t lastLedMillis;
bool ledIsOn;

#endif

#ifdef IAP_VIA_SPI

uint32_t writeData32[(blockReadSize + 3) / 4];
char * const writeData = reinterpret_cast<char *>(writeData32);
uint32_t transferStartTime;

#else

FATFS fs;
FIL upgradeBinary;
const char* fwFile = defaultFwFile;

#endif

uint32_t readData32[(blockReadSize + 3) / 4];	// use aligned memory so DMA works well
char * const readData = reinterpret_cast<char *>(readData32);

ProcessState state = Initializing;
uint32_t flashPos = IFLASH_ADDR;

size_t retry = 0;
size_t bytesRead, bytesWritten;
bool haveDataInBuffer;
const size_t reportPercentIncrement = 20;
size_t reportNextPercent = reportPercentIncrement;

char formatBuffer[100];

void checkLed() noexcept
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
void delay_ms(uint32_t ms) noexcept
{
	const uint32_t startTime = millis();
	do
	{
		checkLed();
	} while (millis() - startTime < ms);
}

void debugPrintf(const char *fmt, ...) noexcept;		// forward declaration
void MessageF(const char *fmt, ...) noexcept;			// forward declaration

extern "C" void UrgentInit() noexcept { }

extern "C" void SysTick_Handler(void) noexcept
{
	CoreSysTick();
#if SAME5x
	watchdogReset();
#else
	wdt_restart(WDT);							// kick the watchdog
#endif

#if SAM4E || SAME70
	rswdt_restart(RSWDT);						// kick the secondary watchdog
#endif
}

extern "C" void SVC_Handler() noexcept { for (;;) {} }
extern "C" void PendSV_Handler() noexcept { for (;;) {} }

extern "C" void AppMain() noexcept
{
#if SAME5x
	// Initialise systick (needed for delay calls)
	SysTick->LOAD = ((SystemCoreClock/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
#else
	SysTickInit();
#endif

#ifdef IAP_VIA_SPI
	pinMode(SbcTfrReadyPin, OUTPUT_LOW);

	ConfigurePin(APIN_SBC_SPI_MOSI);
	ConfigurePin(APIN_SBC_SPI_MISO);
	ConfigurePin(APIN_SBC_SPI_SCK);
	ConfigurePin(APIN_SBC_SPI_SS0);

# if USE_DMAC
	pmc_enable_periph_clk(ID_DMAC);
	NVIC_DisableIRQ(DMAC_IRQn);
# elif USE_XDMAC
	pmc_enable_periph_clk(ID_XDMAC);
	NVIC_DisableIRQ(XDMAC_IRQn);
# endif

	spi_enable_clock(SBC_SPI);
	spi_disable(SBC_SPI);
# if USE_DMAC
	dmac_init(DMAC);
	dmac_set_priority_mode(DMAC, DMAC_PRIORITY_ROUND_ROBIN);
	dmac_enable(DMAC);

	// The DMAC is master 4 and the SRAM is slave 0. Give the DMAC the highest priority.
	matrix_set_slave_default_master_type(0, MATRIX_DEFMSTR_LAST_DEFAULT_MASTER);
	matrix_set_slave_priority(0, (3 << MATRIX_PRAS0_M4PR_Pos));
	// Set the slave slot cycle limit.
	// If we leave it at the default value of 511 clock cycles, we get transmit underruns due to the HSMCI using the bus for too long.
	// A value of 8 seems to work. I haven't tried other values yet.
	matrix_set_slave_slot_cycle(0, 8);
# endif
#endif // IAP_VIA_SPI

#if SAM4E || SAM4S || SAME70
	digitalWrite(DiagLedPin, true);				// turn the LED on
	ledIsOn = true;
	lastLedMillis = millis();
#endif

	SERIAL_AUX_DEVICE.begin(57600);				// set serial port to default PanelDue baud rate
	MessageF("IAP started");

	debugPrintf("IAP Utility for Duet electronics\n");
	debugPrintf("Developed by Christian Hammacher (2016-2019)\n");
	debugPrintf("Licensed under the terms of the GPLv2\n\n");

#ifdef IAP_VIA_SPI
	memset(writeData, 0x1A, blockReadSize);
#else
	initFilesystem();
	getFirmwareFileName();
	openBinary();
#endif

	for (;;)
	{
		checkLed();
		writeBinary();
	}
}

/** IAP routines **/

#ifdef IAP_VIA_SPI

# if USE_XDMAC
static xdmac_channel_config_t xdmac_tx_cfg, xdmac_rx_cfg;
# endif

volatile bool dataReceived = false, transferPending = false;
bool transferReadyHigh = false;

void setup_spi(size_t bytesToTransfer) noexcept
{
	// Reset SPI
	spi_reset(SBC_SPI);
	spi_set_slave_mode(SBC_SPI);
	spi_disable_mode_fault_detect(SBC_SPI);
	spi_set_peripheral_chip_select_value(SBC_SPI, spi_get_pcs(0));
	spi_set_clock_polarity(SBC_SPI, 0, 0);
	spi_set_clock_phase(SBC_SPI, 0, 1);
	spi_set_bits_per_transfer(SBC_SPI, 0, SPI_CSR_BITS_8_BIT);
# if USE_DMAC
	dmac_channel_disable(DMAC, DmacChanSbcRx);
	dmac_channel_disable(DMAC, DmacChanSbcTx);

	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	// Initialize channel config for transmitter
	dmac_channel_set_source_addr(DMAC, DmacChanSbcTx, reinterpret_cast<uint32_t>(writeData));
	dmac_channel_set_destination_addr(DMAC, DmacChanSbcTx, reinterpret_cast<uint32_t>(&(SBC_SPI->SPI_TDR)));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanSbcTx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanSbcTx,
			bytesToTransfer |
			DMAC_CTRLA_SRC_WIDTH_WORD |
			DMAC_CTRLA_DST_WIDTH_BYTE);
	dmac_channel_set_ctrlB(DMAC, DmacChanSbcTx,
		DMAC_CTRLB_SRC_DSCR |
		DMAC_CTRLB_DST_DSCR |
		DMAC_CTRLB_FC_MEM2PER_DMA_FC |
		DMAC_CTRLB_SRC_INCR_INCREMENTING |
		DMAC_CTRLB_DST_INCR_FIXED);

	// Initialize channel config for receiver
	dmac_channel_set_source_addr(DMAC, DmacChanSbcRx, reinterpret_cast<uint32_t>(&(SBC_SPI->SPI_RDR)));
	dmac_channel_set_destination_addr(DMAC, DmacChanSbcRx, reinterpret_cast<uint32_t>(readData));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanSbcRx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanSbcRx,
			bytesToTransfer |
			DMAC_CTRLA_SRC_WIDTH_BYTE |
			DMAC_CTRLA_DST_WIDTH_WORD);
	dmac_channel_set_ctrlB(DMAC, DmacChanSbcRx,
		DMAC_CTRLB_SRC_DSCR |
		DMAC_CTRLB_DST_DSCR |
		DMAC_CTRLB_FC_PER2MEM_DMA_FC |
		DMAC_CTRLB_SRC_INCR_FIXED |
		DMAC_CTRLB_DST_INCR_INCREMENTING);

	dmac_channel_enable(DMAC, DmacChanSbcRx);
	dmac_channel_enable(DMAC, DmacChanSbcTx);

	// Configure DMA RX channel
	dmac_channel_set_configuration(DMAC, DmacChanSbcRx,
			DMAC_CFG_SRC_PER(SBC_SPI_RX_DMA_HW_ID) |
			DMAC_CFG_SRC_H2SEL |
			DMAC_CFG_SOD |
			DMAC_CFG_FIFOCFG_ASAP_CFG);

	// Configure DMA TX channel
	dmac_channel_set_configuration(DMAC, DmacChanSbcTx,
			DMAC_CFG_DST_PER(SBC_SPI_TX_DMA_HW_ID) |
			DMAC_CFG_DST_H2SEL |
			DMAC_CFG_SOD |
			DMAC_CFG_FIFOCFG_ASAP_CFG);
# elif USE_XDMAC
	// Initialize channel config for transmitter
	xdmac_tx_cfg.mbr_ubc = bytesToTransfer;
	xdmac_tx_cfg.mbr_sa = (uint32_t)writeData;
	xdmac_tx_cfg.mbr_da = (uint32_t)&(SBC_SPI->SPI_TDR);
	xdmac_tx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_MEM2PER |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF0 |
		XDMAC_CC_DIF_AHB_IF1 |
		XDMAC_CC_SAM_INCREMENTED_AM |
		XDMAC_CC_DAM_FIXED_AM |
		XDMAC_CC_PERID(SBC_SPI_TX_PERID);
	xdmac_tx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_tx_cfg.mbr_sus = 0;
	xdmac_tx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanSbcTx, &xdmac_tx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanSbcTx, 0);
	xdmac_channel_enable(XDMAC, DmacChanSbcTx);
	xdmac_disable_interrupt(XDMAC, DmacChanSbcTx);

	// Initialize channel config for receiver
	xdmac_rx_cfg.mbr_ubc = bytesToTransfer;
	xdmac_rx_cfg.mbr_da = (uint32_t)readData;
	xdmac_rx_cfg.mbr_sa = (uint32_t)&(SBC_SPI->SPI_RDR);
	xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE|
		XDMAC_CC_SIF_AHB_IF1 |
		XDMAC_CC_DIF_AHB_IF0 |
		XDMAC_CC_SAM_FIXED_AM |
		XDMAC_CC_DAM_INCREMENTED_AM |
		XDMAC_CC_PERID(SBC_SPI_RX_PERID);
	xdmac_rx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_rx_cfg.mbr_sus = 0;
	xdmac_rx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanSbcRx, &xdmac_rx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanSbcRx, 0);
	xdmac_channel_enable(XDMAC, DmacChanSbcRx);
	xdmac_disable_interrupt(XDMAC, DmacChanSbcRx);
# endif

	// Enable SPI and notify the RaspPi we are ready
	spi_enable(SBC_SPI);

	// Enable end-of-transfer interrupt
	(void)SBC_SPI->SPI_SR;						// clear any pending interrupt
	SBC_SPI->SPI_IER = SPI_IER_NSSR;				// enable the NSS rising interrupt
	NVIC_SetPriority(SBC_SPI_IRQn, NvicPrioritySpi);
	NVIC_EnableIRQ(SBC_SPI_IRQn);

	// Begin transfer
	dataReceived = false;
	transferPending = true;
	transferStartTime = millis();

	transferReadyHigh = !transferReadyHigh;
	digitalWrite(SbcTfrReadyPin, transferReadyHigh);
}

void disable_spi() noexcept
{
# if USE_DMAC
	dmac_channel_disable(DMAC, DmacChanSbcRx);
	dmac_channel_disable(DMAC, DmacChanSbcTx);
# endif

# if USE_XDMAC
	xdmac_channel_disable(XDMAC, DmacChanSbcRx);
	xdmac_channel_disable(XDMAC, DmacChanSbcTx);
# endif

	// Disable SPI
	spi_disable(SBC_SPI);
}

# ifndef SBC_SPI_HANDLER
#  error SBC_SPI_HANDLER undefined
# endif

extern "C" void SBC_SPI_HANDLER(void) noexcept
{
	const uint32_t status = SBC_SPI->SPI_SR;							// read status and clear interrupt
	SBC_SPI->SPI_IDR = SPI_IER_NSSR;									// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{
		// Data has been transferred, disable transfer ready pin and XDMAC channels
		disable_spi();
		dataReceived = true;
	}
}

bool is_spi_transfer_complete() noexcept
{
# if USE_DMAC
	const uint32_t status = DMAC->DMAC_CHSR;
	if (dataReceived &&
		  (((status & (DMAC_CHSR_ENA0 << DmacChanSbcRx)) == 0)	// controller is not enabled, perhaps because it finished a full buffer transfer
		|| ((status & (DMAC_CHSR_EMPT0 << DmacChanSbcRx)) != 0))	// controller is enabled, probably suspended, and the FIFO is empty
	   )
	{
		// Disable the channel.
		// We also need to set the resume bit, otherwise it remains suspended when we re-enable it.
		DMAC->DMAC_CHDR = (DMAC_CHDR_DIS0 << DmacChanSbcRx) | (DMAC_CHDR_RES0 << DmacChanSbcRx);
		transferPending = false;
		return true;
	}
	return false;
# elif USE_XDMAC
	if (dataReceived && (xdmac_channel_get_status(XDMAC) & ((1 << DmacChanSbcRx) | (1 << DmacChanSbcTx))) == 0)
	{
		transferPending = false;
		return true;
	}
	return false;
# endif
}

uint16_t CRC16(const char *buffer, size_t length) noexcept
{
	const uint16_t crc16_table[] = {
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

#else

void initFilesystem() noexcept
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
void getFirmwareFileName() noexcept
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
void openBinary() noexcept
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

#ifdef IAP_IN_RAM
	const size_t maxFirmwareSize = IFLASH_SIZE;
#else
	const size_t maxFirmwareSize = IFLASH_SIZE - iapFirmwareSize;
#endif
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

#endif

void ShowProgress() noexcept
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
		MessageF("ERROR: Operation %d failed after %d retries", (int)state, maxRetries);
		debugPrintf("ERROR: Operation %d failed after %d retries!\n", (int)state, maxRetries);
		Reset(false);
	}
	else if (retry > 0)
	{
		debugPrintf("WARNING: Retry %d of %d at pos %08x\n", retry, maxRetries, flashPos);
	}

	switch (state)
	{
	case Initializing:
		MessageF("Unlocking flash");
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
#if SAM4E || SAM4S || SAME70 || SAME5x
			MessageF("Erasing flash");
			state = ErasingFlash;
#else
			bytesWritten = blockReadSize;
			state = WritingUpgrade;
#endif
		}
		break;

#if SAM4E || SAM4S || SAME70 || SAME5x
	case ErasingFlash:
		debugPrintf("Erasing 0x%08x\n", flashPos);
		if (retry != 0)
		{
			MessageF("Erase retry #%u", retry);
		}

		{
			const uint32_t sectorSize =
# if SAME5x
				qq;
# elif SAM4E || SAM4S
				// Deal with varying size sectors on the SAM4E and SAM4S
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
#ifdef IAP_VIA_SPI
				transferPending = false;
#endif
				state = WritingUpgrade;
			}
		}
		break;
#endif

	case WritingUpgrade:
		// Attempt to read a chunk from the new firmware file or SBC
		if (!haveDataInBuffer)
		{
#ifdef IAP_VIA_SPI
			if (transferPending)
			{
				if (is_spi_transfer_complete())
				{
					// Got another flash block to write. The block size is fixed
					bytesRead = blockReadSize;
				}
				else if (flashPos != IFLASH_ADDR && millis() - transferStartTime > TransferCompleteDelay)
				{
					// If anything could be written before, check for the delay indicating the flashing process has finished
					bytesRead = 0;
					disable_spi();
				}
				else
				{
					if (millis() - transferStartTime > TransferTimeout)
					{
						// Timeout while waiting for new data
						debugPrintf("ERROR: Timeout while waiting for response\n");
						MessageF("ERROR: Timeout while waiting for response");
						Reset(false);
					}
					break;
				}
			}
			else
			{
				// The last block has been written to Flash. Start the next SPI transfer
				setup_spi(blockReadSize);
				break;
			}
#else
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
#endif

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
			if (memcmp(readData + bytesWritten, reinterpret_cast<const void *>(flashPos), IFLASH_PAGE_SIZE) == 0)
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
#ifdef IAP_VIA_SPI
						setup_spi(sizeof(FlashVerifyRequest));
						state = VerifyingChecksum;
#else
						state = LockingFlash;
#endif
					}
				}
			}
			else
			{
				retry++;
			}
		}
		break;

#ifdef IAP_VIA_SPI
	case VerifyingChecksum:
		if (millis() - transferStartTime > TransferTimeout)
		{
			debugPrintf("Timeout while waiting for checksum\n");
			MessageF("Timeout while waiting for checksum");
			Reset(false);
		}
		else if (is_spi_transfer_complete())
		{
			const FlashVerifyRequest *request = reinterpret_cast<const FlashVerifyRequest*>(readData);
			uint16_t crc16 = CRC16(reinterpret_cast<const char*>(IFLASH_ADDR), request->firmwareLength);
			if (request->crc16 == crc16)
			{
				// Success!
				debugPrintf("Checksum OK!\n");
				writeData[0] = 0x0C;
				state = SendingChecksumOK;
			}
			else
			{
				// Checksum mismatch
				debugPrintf("Checksum does not match\n");
				MessageF("CRC mismatch");
				writeData[0] = 0xFF;
				state = SendingChecksumError;
			}
			retry = 0;
			setup_spi(1);
		}
		break;

	case SendingChecksumOK:
		if (millis() - transferStartTime > TransferTimeout)
		{
			// Although this is not expected, the firmware has been written successfully so just continue as normal
			MessageF("Timeout while exchanging checksum acknowledgement");
			debugPrintf("Timeout while exchanging checksum acknowledgement\n");
			state = LockingFlash;
		}
		else if (is_spi_transfer_complete())
		{
			// Firmware checksum OK, lock the flash again and restart
			state = LockingFlash;
		}
		break;

	case SendingChecksumError:
		if (millis() - transferStartTime > TransferTimeout)
		{
			// Bad image has been flashed - restart to bossa
			debugPrintf("Timeout while exchanging checksum error\n");
			MessageF("Timeout while reporting CRC error");
			Reset(false);
		}
		else if (is_spi_transfer_complete())
		{
			// Attempt to flash the firmware again
			flashPos = IFLASH_ADDR;
			state = WritingUpgrade;
			retry = 0;
		}
		break;
#endif

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

#ifndef IAP_VIA_SPI
// Does what it says
void closeBinary() noexcept
{
	f_close(&upgradeBinary);
}
#endif

void Reset(bool success) noexcept
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

#if !SAME5x
		// Start from bootloader next time
		flash_clear_gpnvm(1);
#endif

		cpu_irq_enable();
	}

#ifdef IAP_VIA_SPI
	digitalWrite(SbcTfrReadyPin, false);
#endif

	delay_ms(500);							// allow last message to PanelDue to go

#if SAM4E || SAM4S || SAME70
	digitalWrite(DiagLedPin, false);		// turn the LED off
#endif

	// Reboot
	Reset();
	while(true) { }
}

/** Helper functions **/

void debugPrintf(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

#if DEBUG
	sendUSB(CDC_TX, formatBuffer, strlen(formatBuffer));
#endif
}

// Write message to PanelDue
// The message must not contain any characters that need JSON escaping, such as newline or " or \.
void MessageF(const char *fmt, ...) noexcept
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
void AnalogInInit() noexcept
{
}

extern "C" void TWI0_Handler() noexcept
{
}

extern "C" void TWI1_Handler() noexcept
{
}

// Cache hooks called from the ASF. These are dummy because we run with the cache disabled.
extern "C" void CacheFlushBeforeDMAReceive(const volatile void *start, size_t length) noexcept { }
extern "C" void CacheInvalidateAfterDMAReceive(const volatile void *start, size_t length) noexcept { }
extern "C" void CacheFlushBeforeDMASend(const volatile void *start, size_t length) noexcept { }

#if DEBUG
// We have to use our own USB transmit function here, because the core will assume that the USB line is closed
void sendUSB(uint32_t ep, const void* d, uint32_t len) noexcept
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
