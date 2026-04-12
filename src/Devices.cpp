/*
 * Devices.cpp
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#include "Devices.h"
#include "iap.h"

#ifdef IAP_SBC_USB
# if SAME5x
#  include <hri_gclk_e54.h>
#  include <hri_mclk_e54.h>
# elif SAME70
#  include <asf/sam/drivers/pmc/pmc.h>
# endif
#endif

#if SAME5x

#include <hal_gpio.h>

// Serial device support
static constexpr UartParameters serial0Params = { Serial0SercomNumber, Serial0RxPin, Serial0TxPin, Serial0PinFunction, Sercom0RxPad, 0, 512, 512 };
AsyncSerial serialUart0(serial0Params);

#ifndef IAP_VIA_SBC
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
#endif

#else

#if SAM4E
// SystemCoreClock is not provided by the CoreN2G library for SAM4E.
// It is set to the correct value by the startup code.
uint32_t SystemCoreClock = CHIP_FREQ_MAINCK_RC_4MHZ;
#endif

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

#ifndef IAP_VIA_SBC
void SdhcInit() noexcept
{
	SetPinFunction(HcmciMclkPin, HsmciMclkPinFunction);
	for (Pin p : HsmciOtherPins)
	{
		SetPinFunction(p, HsmciOtherPinsFunction);
	}
}
#endif

#endif

#ifdef IAP_SBC_USB

// Bring up USB peripheral clocks and D+/D- pins. The ASF/HAL USB stacks we use
// do not do this themselves, and IAP is entered from RRF which may have left
// the USB controller with pending NVIC state, so we also scrub that
static void UsbInitClocksAndPins() noexcept
{
# if SAME5x
	NVIC_DisableIRQ(USB_0_IRQn);
	NVIC_DisableIRQ(USB_1_IRQn);
	NVIC_DisableIRQ(USB_2_IRQn);
	NVIC_DisableIRQ(USB_3_IRQn);
	NVIC_ClearPendingIRQ(USB_0_IRQn);
	NVIC_ClearPendingIRQ(USB_1_IRQn);
	NVIC_ClearPendingIRQ(USB_2_IRQn);
	NVIC_ClearPendingIRQ(USB_3_IRQn);
	NVIC_SetPriority(USB_0_IRQn, 3);
	NVIC_SetPriority(USB_1_IRQn, 3);
	NVIC_SetPriority(USB_2_IRQn, 3);
	NVIC_SetPriority(USB_3_IRQn, 3);

	// 48MHz GCLK + AHB/APB masks for USB peripheral
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum48MHz) | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);

	NVIC_ClearPendingIRQ(USB_0_IRQn);
	NVIC_ClearPendingIRQ(USB_1_IRQn);
	NVIC_ClearPendingIRQ(USB_2_IRQn);
	NVIC_ClearPendingIRQ(USB_3_IRQn);

	// USB D-/D+ pins (PA24, PA25)
	gpio_set_pin_direction(PortAPin(24), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(24), false);
	gpio_set_pin_pull_mode(PortAPin(24), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(24), PINMUX_PA24H_USB_DM);
	gpio_set_pin_direction(PortAPin(25), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(25), false);
	gpio_set_pin_pull_mode(PortAPin(25), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(25), PINMUX_PA25H_USB_DP);
# elif SAME70
	NVIC_DisableIRQ(USBHS_IRQn);
	NVIC_ClearPendingIRQ(USBHS_IRQn);
	NVIC_SetPriority(USBHS_IRQn, 3);

	// UPLL (480MHz), divided by 10 for 48MHz USB clock; also enable the USB peripheral clock
	PMC->CKGR_UCKR = CKGR_UCKR_UPLLEN | CKGR_UCKR_UPLLCOUNT(3);
	while (!(PMC->PMC_SR & PMC_SR_LOCKU)) {}
	PMC->PMC_USB = PMC_USB_USBS | PMC_USB_USBDIV(9);
	PMC->PMC_SCER = PMC_SCER_USBCLK;
	pmc_enable_periph_clk(ID_USBHS);

	NVIC_ClearPendingIRQ(USBHS_IRQn);
# endif
}

#endif

// Device initialisation
void DeviceInit() noexcept
{
#ifndef IAP_VIA_SBC
	SdhcInit();
#endif
#ifdef IAP_SBC_USB
	UsbInitClocksAndPins();
#endif
}

// IAP doesn't use pin descriptions, but CoreN2G references this function
const PinDescriptionBase *AppGetPinDescription(Pin p) noexcept
{
	return nullptr;
}

// Override CoreN2G's InitialiseExints to prevent EIC init in global constructor
// (IAP doesn't use external interrupts and the EIC clock may not be ready)
void InitialiseExints() noexcept
{
}

// Program initialisation hook required by CoreN2G
void AppInit() noexcept
{
}

#if SAME5x

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
	return 25;
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
	return 1;
}

#endif

#if SAME70

// Dummy assertion handler, called by the Cache module in CoreN2G
extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept { while (true) { } }

#endif

// syscalls.h must be included by exactly one .cpp file in the project
#include <syscalls.h>

[[noreturn]] void OutOfMemoryHandler() noexcept
{
	while (true) { }
}

extern "C" [[noreturn]] void __cxa_pure_virtual() noexcept
{
	while (true) { }
}

extern "C" [[noreturn]] void __cxa_deleted_virtual() noexcept
{
	while (true) { }
}

// End
