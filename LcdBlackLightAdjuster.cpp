/*
 * LcdBlackLightAdjuster.cpp
 *
 *  Created on: 2018. jún. 29.
 *      Author: BT
 *
 * Eredeti ötlet:
 * @see https://github.com/sfrwmaker/fm_radio_rda5807
 *
 */

#include "LcdBlackLightAdjuster.h"

/**
 * inicializálás
 */
void LcdBlackLightAdjuster::init(void) {
	brightness = new_brightness = DEFAUL_BRIGHTNESS;
	lastSensorCheckMsec = 0;
	lastAdjustMsec = 0;
	adjust();
}

/**
 * LED háttérvilágítás PWM állítgatás
 */
void LcdBlackLightAdjuster::adjust(void) {

	//ha nem aktív, akkor nem megyünk tovább
	if (!blState) {
		return;
	}

	if (new_brightness != brightness) {
		//15 msec-enként állítgatjuk a fényerõt
		if (millis() - lastAdjustMsec > LED_ADJUST_MSEC) {
			lastAdjustMsec = millis();

			new_brightness > brightness ? ++brightness : --brightness;

			analogWrite(led_pin, blLedLevel == LOW ? brightness : 255 - brightness);
		}
	}

	//200msec-enként mérünk rá a szenzorra
	if (millis() - lastSensorCheckMsec > SENSOR_CHECK_MSEC) {
		lastSensorCheckMsec = millis();

		//szenzor érték leolvasása
		int measuredLight = analogRead(sensor_pin);

		if (measuredLight < SENSOR_VALUE_NIGHT) {
			new_brightness = NIGHTLY_BRIGHTNESS;
			return;
		}

		if (measuredLight > SENSOR_VALUE_DAY) {
			new_brightness = DAILY_BRIGHTNESS;
			return;
		}

		//A beállítandó érték kimatekozása
		new_brightness = map(measuredLight, SENSOR_VALUE_NIGHT, SENSOR_VALUE_DAY, NIGHTLY_BRIGHTNESS, DAILY_BRIGHTNESS);
	}
}
