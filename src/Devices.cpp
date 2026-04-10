/*
 * Devices.cpp
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#include "Devices.h"

#if SAME5x

#include "iap.h"

#include <hal_gpio.h>

// Serial device support
static constexpr UartParameters serial0Params = { Serial0SercomNumber, Serial0RxPin, Serial0TxPin, Serial0PinFunction, Sercom0RxPad, 0, 512, 512 };
AsyncSerial serialUart0(serial0Params);

static void SdhcInit() noexcept
{
	// Set up SDHC clock
#if defined(DUET3_MINI)
	// Using SDHC 1
	hri_mclk_set_AHBMASK_SDHC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNumSdhc) | GCLK_PCHCTRL_CHEN);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN(GclkNumSdhc) | GCLK_PCHCTRL_CHEN);
#elif defined(FMDC)
	// Using SDHC 0 on v0.3 board
	hri_mclk_set_AHBMASK_SDHC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC0_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNumSdhc) | GCLK_PCHCTRL_CHEN);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC0_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN(GclkNumSdhc) | GCLK_PCHCTRL_CHEN);
#else
# error Unknown board
#endif

	// Setup SD card interface pins
	for (Pin p : SdMciPins)
	{
		SetPinFunction(p, SdMciPinsFunction);
	}
}

#else

// SystemCoreClock is required by CoreN2G but not provided by the library for SAM4E/SAM4S/SAME70.
// It is set to the correct value by the startup code.
uint32_t SystemCoreClock = CHIP_FREQ_MAINCK_RC_4MHZ;

# if SAM4E

static constexpr UartParameters serial0Params = { 0, PortAPin(9), PortAPin(10), GpioPinFunction::A, 512, 512 };
AsyncSerial serialUart0(serial0Params);

constexpr Pin HcmciMclkPin = PortAPin(29);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::C;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherPinsFunction = GpioPinFunction::C;

# elif SAM4S

static constexpr UartParameters serial0Params = { 1, 28, 29, GpioPinFunction::A, 512, 512 };
AsyncSerial serialUart0(serial0Params);

constexpr Pin HcmciMclkPin = PortAPin(29);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::C;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherPinsFunction = GpioPinFunction::C;

# elif SAME70

static constexpr UartParameters serial0Params = { 2, PortDPin(25), PortDPin(26), GpioPinFunction::C, 512, 512 };
AsyncSerial serialUart0(serial0Params);

constexpr Pin HcmciMclkPin = PortAPin(25);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::D;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherPinsFunction = GpioPinFunction::C;

# endif

void SdhcInit() noexcept
{
	SetPinFunction(HcmciMclkPin, HsmciMclkPinFunction);
	for (Pin p : HsmciOtherPins)
	{
		SetPinFunction(p, HsmciOtherPinsFunction);
	}
}

#endif

// Device initialisation
void DeviceInit() noexcept
{
	SdhcInit();
}

// End
