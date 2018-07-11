/*
 * LcdBlackLightAdjuster.h
 *
 *
 *  Created on: 2018. jún. 29.
 *      Author: BT-Soft
 *
 * Eredeti ötlet:
 * @see https://github.com/sfrwmaker/fm_radio_rda5807
 */
#include <Arduino.h>
#ifndef LCDBLACKLIGHTADJUSTER_H_
#define LCDBLACKLIGHTADJUSTER_H_

#define DEFAUL_BRIGHTNESS 	128            	// Default brightness of backlight
#define DAILY_BRIGHTNESS  	0             	// Daily brightness of backlight
#define NIGHTLY_BRIGHTNESS  250           	// The display brightness when it is dark

#define B_NIGHT  			300             // light sensor value of the night
#define B_DAY  				700             // light sensor value of the day light

#define PERIOD 				200             // The period in ms to check the photeregister
#define MAX_DISPERSION  	15 				// The maximum dispersion of the sensor to change the brightness

class LcdBlackLightAdjuster {
public:

	LcdBlackLightAdjuster(byte sensorPIN, byte lightPIN) {
		sensor_pin = sensorPIN;
		led_pin = lightPIN;
		brightness = DEFAUL_BRIGHTNESS;
		checkMS = PERIOD;
		new_brightness = 0;
		blState = true;
	}
	void init(void);                                // Initialize the data
	void adjust(void);                              // Automatically adjust the brightness

	void on(void) {
		blState = true;
		brightness = new_brightness = DEFAUL_BRIGHTNESS;
	}

	void off(void) {
		blState = false;
		analogWrite(led_pin, 255);
	}

	bool isBlState(){
		return blState;
	}

private:
	boolean blState;
	byte sensor_pin;                                // Light sensor pin
	byte led_pin;                                   // Led PWM pin
	uint32_t checkMS;                               // Time in ms when the sensor was checked
	byte brightness;                                // The backlight brightness
	byte new_brightness;                            // The baclight brightness to set up
};

#endif /* LCDBLACKLIGHTADJUSTER_H_ */
