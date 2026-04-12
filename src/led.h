/* Diagnostic LED for DuetIAP */

#ifndef LED_H_INCLUDED
#define LED_H_INCLUDED

#include "iap.h"

#if defined(DUET3_MB6HC)
// LED pin and polarity are determined at runtime based on board version
extern Pin DiagLedPin;
extern bool LedOnPolarity;
#endif

void initLed() noexcept;
void checkLed() noexcept;

#endif // LED_H_INCLUDED
