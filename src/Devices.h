/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_DEVICES_H_
#define SRC_DEVICES_H_

#include <Core.h>
#include <AsyncSerial.h>

extern AsyncSerial serialUart0;

void DeviceInit() noexcept;

#endif /* SRC_DEVICES_H_ */
