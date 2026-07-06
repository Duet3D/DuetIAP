/* USB CDC transport for DuetIAP
 * Uses the SerialCDC class from CoreN2G for USB CDC communication
 */

#include "iap_usb.h"
#include "led.h"

#ifdef IAP_SBC_USB

#include <SerialCDC.h>
#include <cstring>

// Global SerialCDC instance, matching RRF's init pattern
#if SAME5x
static SerialCDC cdcDevice(NoPin, 2048, 256);	// RX buffer matches blockReadSize, TX only needs small buffer
#elif SAME70
static SerialCDC cdcDevice;
#endif
static SerialCDC *cdc = &cdcDevice;

// Firmware length received from DSF during handshake, used by UsbReadBlock to
// know exactly when the transfer ends instead of the old timing-based heuristic
uint32_t expectedFirmwareLength = 0;

void UsbInit() noexcept
{
	// USB clocks and D+/D- pins were already set up by DeviceInit(); just start CDC
#if SAME5x
	cdc->Start();
#elif SAME70
	cdc->Start(NoPin);
#endif
}

void UsbWaitReady() noexcept
{
	// Wait for USB host to enumerate and connect (DTR set)
	uint32_t startTime = millis();
	while (!cdc->IsConnected())
	{
		UsbPoll();
		checkLed();
		if (millis() - startTime > 20000)
		{
			MessageF("ERROR: USB connection timeout");
			Reset(false);
		}
	}
	MessageF("USB handshake started");

	// Handshake header from DSF: 4-byte "IAPR" marker + 4-byte little-endian firmware length;
	// read all 8 bytes in one go, then validate the marker and extract the length
	static const uint8_t marker[] = {'I', 'A', 'P', 'R'};
	uint8_t header[8];
	size_t rxPos = 0;

	startTime = millis();
	while (rxPos < ARRAY_SIZE(header))
	{
		const size_t n = cdc->readBytes(reinterpret_cast<char*>(header + rxPos), sizeof(header) - rxPos);
		rxPos += n;
		checkLed();
		if (millis() - startTime > 5000)
		{
			MessageF("ERROR: USB handshake timeout");
			Reset(false);
		}
	}

	if (memcmp(header, marker, sizeof(marker)) != 0)
	{
		MessageF("ERROR: invalid USB handshake marker");
		Reset(false);
	}

	expectedFirmwareLength = (uint32_t)header[4] | ((uint32_t)header[5] << 8) | ((uint32_t)header[6] << 16) | ((uint32_t)header[7] << 24);

	// Sanity check: firmware must fit in available flash
	const uint32_t maxFirmwareLength = FirmwareFlashEnd - FirmwareFlashStart;
	if (expectedFirmwareLength == 0 || expectedFirmwareLength > maxFirmwareLength)
	{
		MessageF("ERROR: invalid firmware length %" PRIu32 " from host", expectedFirmwareLength);
		Reset(false);
	}

	// Echo IAPR back to acknowledge readiness
	size_t txPos = 0;
	startTime = millis();
	while (txPos < sizeof(marker))
	{
		const size_t n = cdc->write(marker + txPos, sizeof(marker) - txPos);
		txPos += n;
		checkLed();
		if (millis() - startTime > 2000)
		{
			MessageF("ERROR: USB echo timeout");
			Reset(false);
		}
	}
	cdc->flush();

	MessageF("USB handshake complete, firmware length %" PRIu32 " bytes", expectedFirmwareLength);
}

// Declared in hpl_usb.c -- the main USB event handler normally called from ISR
extern "C" void USB_0_Handler(void);

void UsbPoll() noexcept
{
	// SerialCDC is interrupt-driven; calling available() triggers internal CheckCdc()
	(void)cdc->available();
}

// Block transfer protocol state
static size_t usbBlockBytesRead;
static uint32_t usbTransferStartTime;
static bool usbReadySent;
static size_t verifyBytesRead;
static bool verifyPending;
static uint32_t usbTotalBytesReceived = 0;	// total firmware bytes received across all blocks

void UsbResetTransferState() noexcept
{
	usbBlockBytesRead = 0;
	usbReadySent = false;
	usbTotalBytesReceived = 0;
	verifyBytesRead = 0;
	verifyPending = false;
}

static void sendReadyByte() noexcept
{
	const uint8_t ready = 0x1A;
	const uint32_t start = millis();
	while (cdc->write(ready) == 0)
	{
		checkLed();
		if (millis() - start > 2000)
		{
			MessageF("ERROR: USB write timeout");
			Reset(false);
		}
	}
	cdc->flush();
}

bool UsbReadBlock() noexcept
{
	if (!usbReadySent)
	{
		sendReadyByte();
		usbReadySent = true;
		usbBlockBytesRead = 0;
		usbTransferStartTime = millis();
	}

	{
		size_t toRead = blockReadSize - usbBlockBytesRead;
		size_t n = cdc->readBytes(readData + usbBlockBytesRead, toRead);
		if (n > 0)
		{
			usbBlockBytesRead += n;
			usbTransferStartTime = millis();
		}
	}

	// Full block received
	if (usbBlockBytesRead >= blockReadSize)
	{
		const uint32_t prevTotal = usbTotalBytesReceived;
		usbTotalBytesReceived += blockReadSize;
		usbReadySent = false;

		// The final block is the one whose running total reaches expectedFirmwareLength (sent in the handshake).
		// A short final block is flagged by bytesRead < blockReadSize; when the firmware length is an exact
		// multiple of blockReadSize the final block is full, so completion is reported through UsbTransferComplete
		if (usbTotalBytesReceived >= expectedFirmwareLength)
		{
			bytesRead = expectedFirmwareLength - prevTotal;
		}
		else
		{
			bytesRead = blockReadSize;
		}
		return true;
	}

	// Overall timeout (no data at all for TransferTimeout)
	if (millis() - usbTransferStartTime > TransferTimeout)
	{
		MessageF("ERROR: USB timeout");
		Reset(false);
	}

	return false;
}

// True once every firmware byte has been received. Lets the state machine detect an exactly-full final block,
// which bytesRead < blockReadSize cannot express
bool UsbTransferComplete() noexcept
{
	return usbTotalBytesReceived >= expectedFirmwareLength;
}

void UsbSetupVerifyTransfer() noexcept
{
	verifyBytesRead = 0;
	verifyPending = true;
	sendReadyByte();
	usbTransferStartTime = millis();
}

bool UsbIsVerifyDataReady() noexcept
{
	if (!verifyPending)
	{
		return false;
	}

	const size_t toRead = sizeof(FlashVerifyRequest) - verifyBytesRead;
	if (toRead > 0)
	{
		const size_t n = cdc->readBytes(readData + verifyBytesRead, toRead);
		if (n > 0)
		{
			verifyBytesRead += n;
			usbTransferStartTime = millis();
		}
	}
	return verifyBytesRead >= sizeof(FlashVerifyRequest);
}

void UsbSendVerifyResponse(uint8_t response) noexcept
{
	uint32_t start = millis();
	while (cdc->write(response) == 0)
	{
		checkLed();
		if (millis() - start > 2000)
		{
			MessageF("ERROR: USB write timeout");
			Reset(false);
		}
	}
	cdc->flush();
	verifyPending = false;
}

uint32_t UsbGetTransferStartTime() noexcept
{
	return usbTransferStartTime;
}

#endif // IAP_SBC_USB
