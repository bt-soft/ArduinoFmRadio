/*
 * LcdBlackLightAdjuster.h
 *
 *
 *  Created on: 2018. jún. 29.
 *      Author: BT-Soft
 *
 * Eredeti ötlet:
 * @see https://github.com/sfrwmaker/fm_radio_rda5807
 *
 */
#ifndef LCDBLACKLIGHTADJUSTER_H_
#define LCDBLACKLIGHTADJUSTER_H_
#include <Arduino.h>

#define DAILY_BRIGHTNESS  	0             	/* Aappali fény esetén a LED vezérlõ értéke */
#define NIGHTLY_BRIGHTNESS  230           	/* Esti körülmények között a LED vezérlõ értéke */
#define DEFAUL_BRIGHTNESS  	((NIGHTLY_BRIGHTNESS - DAILY_BRIGHTNESS) /2)	/* Kezdõ LED vezérlõ érték */

#define SENSOR_VALUE_NIGHT	300             /* Az a mért érték, ahol azt mondjuk, hogy már sötét van */
#define SENSOR_VALUE_DAY  	700             /* Az a mért érték, ahol azt mondjuk, hogy már nappali fény van */

#define LED_ADJUST_MSEC		15				/* Ennyi msec-enként állítgatjuk a háttérvilágítás LED-jét */
#define SENSOR_CHECK_MSEC	200				/* Ennyi msec-enként mérünk rá a szenzorra */

class LcdBlackLightAdjuster {

private:
	bool blLedLevel;								//A háttérvilágítás LED milyen szintre aktív?
	byte sensor_pin;
	byte led_pin;
	uint32_t lastSensorCheckMsec;
	uint32_t lastAdjustMsec;

	byte brightness;
	byte new_brightness;


public:
	bool blState;

	/**
	 * konstruktor
	 */
	LcdBlackLightAdjuster(byte sensorPIN, byte lightPIN, byte blLedLevel = LOW) {
		sensor_pin = sensorPIN;
		led_pin = lightPIN;
		brightness = DEFAUL_BRIGHTNESS;
		lastSensorCheckMsec = 0;
		lastAdjustMsec = 0;
		new_brightness = 0;
		blState = true;
		this->blLedLevel = blLedLevel;


		pinMode(led_pin, OUTPUT);
		pinMode(sensor_pin, INPUT);
	}
	void init(void);                                // Initialize the data
	void adjust(void);                              // Automatically adjust the brightness

	/**
	 * Engedélyezés
	 */
	void on(void) {
		blState = true;
		brightness = new_brightness = DEFAUL_BRIGHTNESS;
	}

	/**
	 * Tiltás
	 */
	void off(void) {
		blState = false;
		analogWrite(led_pin, blLedLevel == LOW ? 255 : 0);
	}

};

#endif /* LCDBLACKLIGHTADJUSTER_H_ */
