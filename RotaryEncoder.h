/*
 * RotaryEncoder.h
 *
 *  Created on: 2018. jún. 27.
 *      Author: BT-Soft
 */

#ifndef ROTARYENCODER_H_
#define ROTARYENCODER_H_

#include <TimerOne.h>
#include "ClickEncoder.h"

class RotaryEncoder: public ClickEncoder {

private:
	int8_t  oldValue = 0;
	int8_t  value = 0;


public:

	/**
	 * Irány enum
	 */
	typedef enum Direction_t {
		UP, DOWN, NONE
	} Direction;

	/**
	 * Visszatérési érték
	 */
	typedef struct t {
		Direction_t direction; //
		Button_e buttonState;
	} RotaryEncoderResult;

	/**
	 *
	 */
	RotaryEncoder(uint8_t CLK, uint8_t DT, uint8_t SW) :
			ClickEncoder(CLK, DT, SW) {

		//ClickEncoder
		ClickEncoder::setAccelerationEnabled(true);
		oldValue = ClickEncoder::getValue();
	}

	/**
	 *
	 */
	RotaryEncoderResult readRotaryEncoder() {
		RotaryEncoderResult result;

		result.buttonState = ClickEncoder::getButton();
		result.direction = NONE;

		if (result.buttonState == ClickEncoder::Open) { //Tekerni és klikkelni egyszerre nem lehet

			value += ClickEncoder::getValue();

			if (value / 2 > oldValue) {
				oldValue = value / 2;
				result.direction = UP;
				//delay(50);
			} else if (value / 2 < oldValue) {
				oldValue = value / 2;
				result.direction = DOWN;
				//delay(50);
			}
		}

		return result;
	}

};

#endif /* ROTARYENCODER_H_ */
