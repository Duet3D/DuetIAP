/*
 * Devices.cpp
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#include "Devices.h"

#if SAME5x

#include "iap.h"			// for defines of SERIAL0_ISR0 etc.

#include <hal_gpio.h>

// Serial device support
AsyncSerial serialUart0(Serial0SercomNumber, Sercom0RxPad, 512, 512, [](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });

# if !defined(SERIAL0_ISR0) || !defined(SERIAL0_ISR2) || !defined(SERIAL0_ISR3)
#  error SERIAL0_ISRn not defined
# endif

void SERIAL0_ISR0() noexcept
{
	serialUart0.Interrupt0();
}

void SERIAL0_ISR2() noexcept
{
	serialUart0.Interrupt2();
}

void SERIAL0_ISR3() noexcept
{
	serialUart0.Interrupt3();
}

static void SdhcInit() noexcept
{
	// Set up SDHC clock
	hri_mclk_set_AHBMASK_SDHC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum90MHz) | GCLK_PCHCTRL_CHEN);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN(GclkNum31KHz) | GCLK_PCHCTRL_CHEN);

	// Set up SDHC pins
#if 1
	// This is the code generated by Atmel Start. I don't know whether it is all necessary.
	gpio_set_pin_direction(PortAPin(21), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(21), false);
	gpio_set_pin_pull_mode(PortAPin(21), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(21), PINMUX_PA21I_SDHC1_SDCK);

	gpio_set_pin_direction(PortAPin(20), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(20), false);
	gpio_set_pin_pull_mode(PortAPin(20), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(20), PINMUX_PA20I_SDHC1_SDCMD);

	gpio_set_pin_direction(PortBPin(18), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(18), false);
	gpio_set_pin_pull_mode(PortBPin(18), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(18), PINMUX_PB18I_SDHC1_SDDAT0);

	gpio_set_pin_direction(PortBPin(19), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(19), false);
	gpio_set_pin_pull_mode(PortBPin(19), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(19), PINMUX_PB19I_SDHC1_SDDAT1);

	gpio_set_pin_direction(PortBPin(20), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(20), false);
	gpio_set_pin_pull_mode(PortBPin(20), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(20), PINMUX_PB20I_SDHC1_SDDAT2);

	gpio_set_pin_direction(PortBPin(21), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(21), false);
	gpio_set_pin_pull_mode(PortBPin(21), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(21), PINMUX_PB21I_SDHC1_SDDAT3);
#else
	// Setup SD card interface pins
	for (Pin p : SdMciPins)
	{
		SetPinFunction(p, SdMciPinsFunction);
	}
#endif
}

// Serial interface
static void SerialInit() noexcept
{
	SetPinFunction(Serial0TxPin, Serial0PinFunction);
	SetPinFunction(Serial0RxPin, Serial0PinFunction);
}

#else
# if SAM4E

AsyncSerial serialUart0(UART0, UART0_IRQn, ID_UART0, 512, 512, [](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });

constexpr Pin APIN_Serial0_RXD = PortAPin(9);
constexpr Pin APIN_Serial0_TXD = PortAPin(10);
constexpr auto Serial0PinFunction = GpioPinFunction::A;

constexpr Pin HcmciMclkPin = PortAPin(29);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::C;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherkPinsFunction = GpioPinFunction::C;

void UART0_Handler(void)
{
	serialUart0.IrqHandler();
}

# elif SAM4S

// Serial device support
AsyncSerial serialUart0(UART1, UART1_IRQn, ID_UART1, 512, 512, [](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });

constexpr Pin APIN_Serial0_RXD = 28;
constexpr Pin APIN_Serial0_TXD = 29;
constexpr auto Serial0PinFunction = GpioPinFunction::A;

constexpr Pin HcmciMclkPin = PortAPin(29);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::C;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherkPinsFunction = GpioPinFunction::C;

void UART1_Handler(void)
{
	serialUart0.IrqHandler();
}

# elif SAME70

AsyncSerial serialUart0(UART2, UART2_IRQn, ID_UART2, 512, 512, [](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });

constexpr Pin APIN_Serial0_RXD = PortDPin(25);
constexpr Pin APIN_Serial0_TXD = PortDPin(26);
constexpr auto Serial0PinFunction = GpioPinFunction::C;

constexpr Pin HcmciMclkPin = PortAPin(25);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::D;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherPinsFunction = GpioPinFunction::C;

void UART2_Handler(void)
{
	serialUart0.IrqHandler();
}

# endif

void SerialInit() noexcept
{
	SetPinFunction(APIN_Serial0_RXD, Serial0PinFunction);
	SetPinFunction(APIN_Serial0_TXD, Serial0PinFunction);
	SetPullup(APIN_Serial0_RXD, true);
}

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
	SerialInit();
	SdhcInit();
}

// End
