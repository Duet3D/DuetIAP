/*
 * Main.cpp
 *  Program entry point used in SAME5x builds only
 *  Created on: 11 Jul 2020
 *      Author: David
 *  License: GNU GPL version 3
 */

#include <Core.h>

#if SAME5x

# include <peripheral_clk_config.h>
# include <hri_oscctrl_e54.h>
# include <hri_gclk_e54.h>

constexpr uint32_t Dpll1Multiplier = 72;								// this multiplies the 2.5MHz reference to get the PLL output frequency
constexpr uint32_t Dpll1Frequency = Dpll1Multiplier * 2500000;			// the PLL output frequency
constexpr uint32_t SdhcClockFreq = Dpll1Frequency/2;					// clock frequency to the SDHC peripheral
constexpr uint32_t MaxSdCardClockFreq = SdhcClockFreq/2;				// maximum SD card clock frequency
static_assert(MaxSdCardClockFreq <= 50000000, "SD clock frequency too high");

#endif

// Program initialisation
void AppInit() noexcept
{
#if SAME5x

	// Initialise FDPLL1
	hri_oscctrl_write_DPLLRATIO_reg(OSCCTRL, 1,
			  OSCCTRL_DPLLRATIO_LDRFRAC(0)
			| OSCCTRL_DPLLRATIO_LDR(Dpll1Multiplier - 1));
	hri_oscctrl_write_DPLLCTRLB_reg(OSCCTRL, 1,
			  OSCCTRL_DPLLCTRLB_DIV(4)
			| (0 << OSCCTRL_DPLLCTRLB_DCOEN_Pos)
			| OSCCTRL_DPLLCTRLB_DCOFILTER(0)
			| (0 << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
			| OSCCTRL_DPLLCTRLB_LTIME(0)
			| OSCCTRL_DPLLCTRLB_REFCLK_XOSC1
			| (0 << OSCCTRL_DPLLCTRLB_WUF_Pos)
			| OSCCTRL_DPLLCTRLB_FILTER(0));
	hri_oscctrl_write_DPLLCTRLA_reg(OSCCTRL, 1,
			  (0 << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
			| (1 << OSCCTRL_DPLLCTRLA_ENABLE_Pos));

	while (!(hri_oscctrl_get_DPLLSTATUS_LOCK_bit(OSCCTRL, 1) || hri_oscctrl_get_DPLLSTATUS_CLKRDY_bit(OSCCTRL, 1))) { }

	// Initialise the GCLKs we use that are not initialised by CoreNG

	// GCLK5: FDPLL1, 90MHz for SDHC
	hri_gclk_write_GENCTRL_reg(GCLK, 5,
			  GCLK_GENCTRL_DIV(2) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DPLL1);
#endif
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

// Return get the SDHC peripheral clock speed in Hz. This must be provided by the client project if using SDHC.
uint32_t AppGetSdhcClockSpeed() noexcept
{
	return SdhcClockFreq;
}

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
