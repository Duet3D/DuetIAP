/*
 * Main.cpp
 *  Program entry point used in SAME5x builds only
 *  Created on: 11 Jul 2020
 *      Author: David
 *  License: GNU GPL version 3
 */

#include <Core.h>

// Program initialisation
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
