/*
 * LcdBlackLightAdjuster.cpp
 *
 *  Created on: 2018. j�n. 29.
 *      Author: BT
 *
 * Eredeti �tlet:
 * @see https://github.com/sfrwmaker/fm_radio_rda5807
 *
 */

#include "LcdBlackLightAdjuster.h"

/**
 * inicializ�l�s
 */
void LcdBlackLightAdjuster::init(void) {
	brightness = new_brightness = DEFAUL_BRIGHTNESS;
	lastSensorCheckMsec = 0;
	lastAdjustMsec = 0;
	adjust();
}

/**
 * LED h�tt�rvil�g�t�s PWM �ll�tgat�s
 */
void LcdBlackLightAdjuster::adjust(void) {

	//ha nem akt�v, akkor nem megy�nk tov�bb
	if (!blState) {
		return;
	}

	if (new_brightness != brightness) {
		//15 msec-enk�nt �ll�tgatjuk a f�nyer�t
		if (millis() - lastAdjustMsec > LED_ADJUST_MSEC) {
			lastAdjustMsec = millis();

			new_brightness > brightness ? ++brightness : --brightness;

			analogWrite(led_pin, blLedLevel == LOW ? brightness : 255 - brightness);
		}
	}

	//200msec-enk�nt m�r�nk r� a szenzorra
	if (millis() - lastSensorCheckMsec > SENSOR_CHECK_MSEC) {
		lastSensorCheckMsec = millis();

		//szenzor �rt�k leolvas�sa
		int measuredLight = analogRead(sensor_pin);

		if (measuredLight < SENSOR_VALUE_NIGHT) {
			new_brightness = NIGHTLY_BRIGHTNESS;
			return;
		}

		if (measuredLight > SENSOR_VALUE_DAY) {
			new_brightness = DAILY_BRIGHTNESS;
			return;
		}

		//A be�ll�tand� �rt�k kimatekoz�sa
		new_brightness = map(measuredLight, SENSOR_VALUE_NIGHT, SENSOR_VALUE_DAY, NIGHTLY_BRIGHTNESS, DAILY_BRIGHTNESS);
	}
}
