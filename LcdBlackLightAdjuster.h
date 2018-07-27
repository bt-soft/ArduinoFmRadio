/*
 * LcdBlackLightAdjuster.h
 *
 *
 *  Created on: 2018. j�n. 29.
 *      Author: BT-Soft
 *
 * Eredeti �tlet:
 * @see https://github.com/sfrwmaker/fm_radio_rda5807
 *
 */
#ifndef LCDBLACKLIGHTADJUSTER_H_
#define LCDBLACKLIGHTADJUSTER_H_
#include <Arduino.h>

#define DAILY_BRIGHTNESS  	0             	/* Aappali f�ny eset�n a LED vez�rl� �rt�ke */
#define NIGHTLY_BRIGHTNESS  230           	/* Esti k�r�lm�nyek k�z�tt a LED vez�rl� �rt�ke */
#define DEFAUL_BRIGHTNESS  	((NIGHTLY_BRIGHTNESS - DAILY_BRIGHTNESS) /2)	/* Kezd� LED vez�rl� �rt�k */

#define SENSOR_VALUE_NIGHT	300             /* Az a m�rt �rt�k, ahol azt mondjuk, hogy m�r s�t�t van */
#define SENSOR_VALUE_DAY  	700             /* Az a m�rt �rt�k, ahol azt mondjuk, hogy m�r nappali f�ny van */

#define LED_ADJUST_MSEC		15				/* Ennyi msec-enk�nt �ll�tgatjuk a h�tt�rvil�g�t�s LED-j�t */
#define SENSOR_CHECK_MSEC	200				/* Ennyi msec-enk�nt m�r�nk r� a szenzorra */

class LcdBlackLightAdjuster {

private:
	bool blLedLevel;								//A h�tt�rvil�g�t�s LED milyen szintre akt�v?
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
	 * Enged�lyez�s
	 */
	void on(void) {
		blState = true;
		brightness = new_brightness = DEFAUL_BRIGHTNESS;
	}

	/**
	 * Tilt�s
	 */
	void off(void) {
		blState = false;
		analogWrite(led_pin, blLedLevel == LOW ? 255 : 0);
	}

};

#endif /* LCDBLACKLIGHTADJUSTER_H_ */
