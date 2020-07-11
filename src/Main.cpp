/*
 * Main.cpp
 *  Program entry point used in SAME5x builds only
 *  Created on: 11 Jul 2020
 *      Author: David
 *  License: GNU GPL version 3
 */

#include <Core.h>

#if SAME5x

#include "iap.h"
#include <Uart.h>

extern "C" [[noreturn]] void AppMain();

// Serial device support
Uart serialUart0(Serial0SercomNumber, Sercom0RxPad, 512, 512);

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


// Program entry point
extern "C" int main()
{
	// As we have entered from RepRapFirmware, re assume that the clocks are already set up correctly
	AppMain();
}

#endif

// End
