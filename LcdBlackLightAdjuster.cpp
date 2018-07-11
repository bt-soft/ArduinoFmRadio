/*
 * LcdBlackLightAdjuster.cpp
 *
 *  Created on: 2018. jún. 29.
 *      Author: BT
 *
 * Eredeti ötlet:
 * @see https://github.com/sfrwmaker/fm_radio_rda5807
 */

#include "LcdBlackLightAdjuster.h"

/**
 *
 */
void LcdBlackLightAdjuster::init(void) {
	pinMode(led_pin, OUTPUT);
	pinMode(sensor_pin, INPUT);
	int light = analogRead(sensor_pin);
	brightness = new_brightness = DEFAUL_BRIGHTNESS;
	checkMS = 0;
	adjust();
}

/**
 *
 */
void LcdBlackLightAdjuster::adjust(void) {
	if (!blState) {
		return;
	}

	if (new_brightness != brightness) {
		new_brightness > brightness ? ++brightness : --brightness;
		analogWrite(led_pin, brightness);
		delay(5);
	}

	if (millis() < checkMS) {
		return;
	}
	checkMS = millis() + PERIOD;

	int light = analogRead(sensor_pin);
	if (light < B_NIGHT) {
		new_brightness = NIGHTLY_BRIGHTNESS;
		return;
	}

	if (light > B_DAY) {
		new_brightness = DAILY_BRIGHTNESS;
		return;
	}

	new_brightness = map(light, B_NIGHT, B_DAY, NIGHTLY_BRIGHTNESS, DAILY_BRIGHTNESS);
}
