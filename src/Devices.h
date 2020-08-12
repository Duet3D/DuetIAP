/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_DEVICES_H_
#define SRC_DEVICES_H_

#include <Core.h>

#if SAME5x

#include <Uart.h>

extern Uart serialUart0;

void DeviceInit() noexcept;

// GCLK numbers not defined in the core
static const unsigned int GclkNum90MHz = 5;		// for SDHC

#endif

#endif /* SRC_DEVICES_H_ */
