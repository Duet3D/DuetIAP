/* Diagnostic LED for DuetIAP
 * Blinks the board's diagnostic LED to indicate activity
 */

#include "led.h"

#if SAM4E || SAM4S || SAME70 || SAME5x

static constexpr uint32_t LedOnOffMillis = 100;

static uint32_t lastLedMillis;
static bool ledIsOn;

#if defined(DUET3_MB6HC)

// LED pin and polarity depend on the board version
Pin DiagLedPin;
bool LedOnPolarity;

#endif

void initLed() noexcept
{
#if defined(DUET3_MB6HC)
	SetPinMode(VersionTestPin, INPUT_PULLUP, false);
	delayMicroseconds(100);
	if (digitalRead(VersionTestPin))
	{
		DiagLedPin = DiagPinPre102;
		LedOnPolarity = DiagOnPolarityPre102;
	}
	else
	{
		DiagLedPin = DiagPin102;
		LedOnPolarity = DiagOnPolarity102;
	}
#endif

	digitalWrite(DiagLedPin, LedOnPolarity);	// turn the LED on
	ledIsOn = true;
	lastLedMillis = millis();
}

void checkLed() noexcept
{
	const uint32_t now = millis();
	if (now - lastLedMillis >= LedOnOffMillis)
	{
		ledIsOn = !ledIsOn;
		digitalWrite(DiagLedPin, XNor(ledIsOn, LedOnPolarity));
		lastLedMillis = now;
	}
}

#endif
