/*
 * BTRDA5807M.cpp
 *
 *  Created on: 2018. júl. 5.
 *      Author: BT-Soft
 *
 * Eredeti ötle:
 * @see: https://github.com/mathertel/Radio
 */
#include <Wire.h>
#include "BTRDA5807M.h"

// ----- Register Definitions -----

// this chip only supports FM mode
#define FREQ_STEPS 10

#define RADIO_REG_CHIPID  			0x00

#define RADIO_REG_CTRL    			0x02
#define RADIO_REG_CTRL_OUTPUT 		0x8000
#define RADIO_REG_CTRL_UNMUTE 		0x4000
#define RADIO_REG_CTRL_MONO   		0x2000
#define RADIO_REG_CTRL_BASS   		0x1000
#define RADIO_REG_CTRL_SEEKUP 		0x0200
#define RADIO_REG_CTRL_SEEK   		0x0100
#define RADIO_REG_CTRL_RDS    		0x0008
#define RADIO_REG_CTRL_NEW    		0x0004
#define RADIO_REG_CTRL_RESET  		0x0002
#define RADIO_REG_CTRL_ENABLE 		0x0001

#define RADIO_REG_CHAN				0x03
#define RADIO_REG_CHAN_SPACE     	0x0003
#define RADIO_REG_CHAN_SPACE_100 	0x0000
#define RADIO_REG_CHAN_BAND      	0x000C
#define RADIO_REG_CHAN_BAND_FM      0x0000
#define RADIO_REG_CHAN_BAND_FMWORLD 0x0008
#define RADIO_REG_CHAN_TUNE   		0x0010
#define RADIO_REG_CHAN_NR     		0x7FC0

#define RADIO_REG_R4    			0x04
#define RADIO_REG_R4_EM50   		0x0800
#define RADIO_REG_R4_SOFTMUTE   	0x0200

#define RADIO_REG_VOL     			0x05
#define RADIO_REG_VOL_VOL   		0x000F

#define RADIO_REG_RA      			0x0A
#define RADIO_REG_RA_RDS      		0x8000
#define RADIO_REG_RA_RDSBLOCK  		0x0800
#define RADIO_REG_RA_STEREO    		0x0400
#define RADIO_REG_RA_NR        		0x03FF

#define RADIO_REG_RB          		0x0B
#define RADIO_REG_RB_FMTRUE   		0x0100
#define RADIO_REG_RB_FMREADY  		0x0080

#define RADIO_REG_RDSA   			0x0C
#define RADIO_REG_RDSB   			0x0D
#define RADIO_REG_RDSC   			0x0E
#define RADIO_REG_RDSD   			0x0F

// I2C-Address RDA Chip for sequential  Access
#define I2C_SEQ  					0x10

// I2C-Address RDA Chip for Index  Access
#define I2C_INDX  					0x11

// The DEBUG_xxx Macros enable Information to the Serial port.
// They can be enabled by setting the _debugEnabled variable to true disabled by using the debugEnable function.
// When the code has to be minimized they can be redefined without implementation like:
// #define DEBUG_STR(txt) {}

/// Used for Debugging text information.
#define DEBUG_STR(txt)           if (_debugEnabled) { Serial.print('>'); Serial.println(txt); }

/// Used for Debugging function entries without parameters.
#define DEBUG_VAL(label, val)    if (_debugEnabled) { Serial.print('>'); Serial.print(label);  Serial.print(':');  Serial.println(val); }
#define DEBUG_VALX(label, val)    if (_debugEnabled) { Serial.print('>'); Serial.print(label);  Serial.print(':');  Serial.println(val, HEX); }

/// Used for Debugging function entries without parameters.
#define DEBUG_FUNC0(fn)          if (_debugEnabled) { Serial.print('>'); Serial.print(fn); Serial.println("()"); }

/// Used for Debugging function entries with 1 parameter.
#define DEBUG_FUNC1(fn, p1)      if (_debugEnabled) { Serial.print('>'); Serial.print(fn); Serial.print('('); Serial.print(p1); Serial.println(')'); }

/// Used for Debugging function entries with 1 parameters as hex Value.
#define DEBUG_FUNC1X(fn, p1) if (_debugEnabled) { Serial.print('>'); Serial.print(fn); Serial.print("(0x"); Serial.print(p1, HEX); Serial.println(')'); }

/// Used for Debugging function entries with 2 parameters.
#define DEBUG_FUNC2(fn, p1, p2)  if (_debugEnabled) { Serial.print('>'); Serial.print(fn); Serial.print('('); Serial.print(p1); Serial.print(", "); Serial.print(p2); Serial.println(')'); }

/// Used for Debugging function entries with 2 parameters and Hex Value.
#define DEBUG_FUNC2X(fn, p1, p2) if (_debugEnabled) { Serial.print('>'); Serial.print(fn); Serial.print("(0x"); Serial.print(p1, HEX); Serial.print(", 0x"); Serial.print(p2, HEX); Serial.println(')'); }

// ----- implement

/**
 * konstruktor
 */
BT_RDA5807M::BT_RDA5807M(void) {

	reg02 = 0L;
	reg03 = 0L;
	reg04 = 0L;
	reg05 = 0L;
	reg06 = 0L;
	reg07 = 0L;

	reg0A = 0L;
	reg0B = 0L;
	reg0C = 0L;
	reg0D = 0L;
	reg0E = 0L;
	reg0F = 0L;

	_sendRDS = NULL;
	_bassBoost = true;
	_softMute = true;
}

/**
 * initialize all internals.
 */
bool BT_RDA5807M::init() {
	bool result = false; // no chip found yet.
	DEBUG_FUNC0("init");

	Wire.begin();
	Wire.beginTransmission(I2C_INDX);
	result = Wire.endTransmission();
	if (result == 0) {
		DEBUG_STR("radio found.");
		result = true;

		// initialize all registers -> reset chip
		//reg02 - control
		reg02 = (RADIO_REG_CTRL_RESET | RADIO_REG_CTRL_ENABLE);

		//reg03 - band + channel
		setBand(RADIO_BAND_FM);

		//reg04 - decoder
		reg04 = RADIO_REG_R4_EM50;

		//reg05 - audio
		reg05 = 0x9081;

		//reg06 -
		reg06 = 0L;

		//reg07 -
		reg07 = 0L;

		_saveRegisters();

		//init
		reg02 = RADIO_REG_CTRL_ENABLE;
		_saveRegister(RADIO_REG_CTRL, reg02);
	}

	return (result);
}

/**
 * switch the power off
 */
void BT_RDA5807M::term() {
	DEBUG_FUNC0("term");
	setVolume(0);
	reg02 = 0L;
	_saveRegisters();
}

// ----- Volume control -----

/**
 *
 */
void BT_RDA5807M::setVolume(uint8_t newVolume) {
	_volume = newVolume;
	newVolume &= RADIO_REG_VOL_VOL;
	reg05 &= (~RADIO_REG_VOL_VOL);
	reg05 |= newVolume;
	_saveRegister(RADIO_REG_VOL, reg05);
}

/**
 *
 */
void BT_RDA5807M::setBassBoost(bool switchOn) {
	_bassBoost = switchOn;
	if (switchOn)
		reg02 |= RADIO_REG_CTRL_BASS;
	else
		reg02 &= (~RADIO_REG_CTRL_BASS);
	_saveRegister(RADIO_REG_CTRL, reg02);
}

/**
 * forceMono
 */
void BT_RDA5807M::setMono(bool switchOn) {
//	_forceMono = switchOn;

	reg02 &= (~RADIO_REG_CTRL_SEEK);
	if (switchOn) {
		reg02 |= RADIO_REG_CTRL_MONO;
	} else {
		reg02 &= ~RADIO_REG_CTRL_MONO;
	}
	_saveRegister(RADIO_REG_CTRL, reg02);
}

/**
 * Switch mute mode.
 */
void BT_RDA5807M::setMute(bool switchOn) {
//	_mute = switchOn;

	if (switchOn) {
		// now don't unmute
		reg02 &= (~RADIO_REG_CTRL_UNMUTE);
	} else {
		// now unmute
		reg02 |= RADIO_REG_CTRL_UNMUTE;
	}
	_saveRegister(RADIO_REG_CTRL, reg02);
}

/**
 * Switch softmute mode.
 */
void BT_RDA5807M::setSoftMute(bool switchOn) {
	_softMute = switchOn;

	if (switchOn) {
		reg04 |= (RADIO_REG_R4_SOFTMUTE);
	} else {
		reg04 &= (~RADIO_REG_R4_SOFTMUTE);
	}
	_saveRegister(RADIO_REG_R4, reg04);
}

//
// ----- Band and frequency control methods -----
//

// tune to new band.
void BT_RDA5807M::setBand(RADIO_BAND newBand) {
	_band = newBand;

	_freqLow = RDA5807M_BAND_LO[newBand];
	_freqHigh = RDA5807M_BAND_HI[newBand];
	_freqSteps = 10; // ez a chip csak ezt támogatja

	uint16_t r;
	if (newBand == RADIO_BAND_FM) {
		r = RADIO_REG_CHAN_BAND_FM;
	} else if (newBand == RADIO_BAND_FM_WORLD) {
		r = RADIO_REG_CHAN_BAND_FMWORLD;
	}
	reg03 = (r | RADIO_REG_CHAN_SPACE_100);
}

/**
 * retrieve the real frequency from the chip after automatic tuning.
 */
uint16_t BT_RDA5807M::getFrequency() {
	// check register A
	Wire.requestFrom(I2C_SEQ, 2);
	reg0A = _read16();
	Wire.endTransmission();

	uint16_t channel = reg0A & RADIO_REG_RA_NR;
	_currentFreq = _freqLow + (channel * _freqSteps);  // assume 100 kHz spacing
	return (_currentFreq);
}

/**
 *
 */
void BT_RDA5807M::setFrequency(uint16_t newF) {
	DEBUG_FUNC1("setFrequency", newF);

	newF = max(newF, _freqLow);
	newF = min(newF, _freqHigh);
	_currentFreq = newF;

	uint16_t newChannel = (newF - _freqLow) / _freqSteps;

	// enable output and unmute
	reg02 |= RADIO_REG_CTRL_OUTPUT | RADIO_REG_CTRL_UNMUTE | RADIO_REG_CTRL_RDS | RADIO_REG_CTRL_ENABLE | RADIO_REG_CTRL_NEW;
	_saveRegister(RADIO_REG_CTRL, reg02);

	uint16_t regChannel = reg03 & (RADIO_REG_CHAN_SPACE | RADIO_REG_CHAN_BAND);

	regChannel += RADIO_REG_CHAN_TUNE; // enable tuning
	regChannel |= newChannel << 6;
	reg03 = regChannel;
	_saveRegister(RADIO_REG_CHAN, reg03);

	// adjust Volume
	_saveRegister(RADIO_REG_VOL, reg05);
}

/**
 * start seek mode upwards
 */
void BT_RDA5807M::seekUp(bool toNextSender) {
	// start seek mode
	reg02 |= RADIO_REG_CTRL_SEEKUP | RADIO_REG_CTRL_SEEK;
	if (!toNextSender) {
		reg02 &= (~RADIO_REG_CTRL_SEEK);
	}
	_saveRegister(RADIO_REG_CTRL, reg02);
}

/**
 * start seek mode downwards
 */
void BT_RDA5807M::seekDown(bool toNextSender) {
	reg02 &= (~RADIO_REG_CTRL_SEEKUP);
	reg02 |= RADIO_REG_CTRL_SEEK;

	if (!toNextSender) {
		reg02 &= (~RADIO_REG_CTRL_SEEK);
	}
	_saveRegister(RADIO_REG_CTRL, reg02);
}

/**
 * Load all status registers from to the chip
 * registers 0A through 0F
 * using the sequential read access mode.
 */
void BT_RDA5807M::_readRegisters() {
	Wire.requestFrom(I2C_SEQ, (6 * 2));
	reg0A = _read16();
	reg0B = _read16();
	reg0C = _read16();
	reg0D = _read16();
	reg0E = _read16();
	reg0F = _read16();
	Wire.endTransmission();
}

/**
 * Save writable registers back to the chip
 * The registers 02 through 06, containing the configuration
 * using the sequential write access mode.
 */
void BT_RDA5807M::_saveRegisters() {
	DEBUG_FUNC0("-saveRegisters");
	Wire.beginTransmission(I2C_SEQ);
	_write16(reg02);
	_write16(reg03);
	_write16(reg04);
	_write16(reg05);
	_write16(reg06);
	_write16(reg07);
	Wire.endTransmission();
}

/**
 * Save one register back to the chip
 */
void BT_RDA5807M::_saveRegister(uint8_t regAddresss, uint16_t value) {
	DEBUG_FUNC2X("-_saveRegister", regAddresss, value);

	Wire.beginTransmission(I2C_INDX);
	Wire.write(regAddresss);
	_write16(value);
	Wire.endTransmission();
}

/**
 * write a register value using 2 bytes into the Wire.
 */
void BT_RDA5807M::_write16(uint16_t val) {
	Wire.write(val >> 8);
	Wire.write(val & 0xFF);
}

/**
 * read a register value using 2 bytes in a row
 */
uint16_t BT_RDA5807M::_read16(void) {
	uint8_t hiByte = Wire.read();
	uint8_t loByte = Wire.read();
	return (256 * hiByte + loByte);
}

/**
 * Retrieve all the information related to the current radio receiving situation.
 */
void BT_RDA5807M::getRadioInfo(RADIO_INFO *info) {

	memset(info, 0, sizeof(RADIO_INFO));

	// read data from registers A .. F of the chip into class memory
	_readRegisters();

	info->currentFreq = _currentFreq;
	info->rssi = reg0B >> 10;
	info->rds = reg0A & RADIO_REG_RA_RDS;
	info->tuned = reg0B & RADIO_REG_RB_FMTRUE;
	//--
	info->stereo = reg0A & RADIO_REG_RA_STEREO;
	info->volume = _volume;
	info->bassBoost = _bassBoost;
	info->softMute = _softMute;
}

/**
 *
 */
void BT_RDA5807M::checkRDS() {

	//csak ha van listener
	if (_sendRDS != NULL) {

		// check register A
		Wire.requestFrom(I2C_SEQ, 2);
		reg0A = _read16();
		Wire.endTransmission();

		if (reg0A & RADIO_REG_RA_RDSBLOCK) {
			DEBUG_STR("BLOCK_E found.");
		}

		//Van RDS szinkron?
		if (reg0A & RADIO_REG_RA_RDS) {
			// check for new RDS data available
			uint16_t newData;
			bool result = false;

			Wire.beginTransmission(I2C_INDX);                // Device 0x11 for random access
			Wire.write(RADIO_REG_RDSA);                   // Start at Register 0x0C
			Wire.endTransmission(0);                         // restart condition

			Wire.requestFrom(I2C_INDX, 8, 1);                  // Retransmit device address with READ, followed by 8 bytes
			newData = _read16();
//			if (newData != reg0C) {
			reg0C = newData;
//				result = true;
//			}

			newData = _read16();
//			if (newData != reg0D) {
			reg0D = newData;
//				result = true;
//			}

			newData = _read16();
//			if (newData != reg0E) {
			reg0E = newData;
//				result = true;
//			}

			newData = _read16();
//			if (newData != reg0F) {
			reg0F = newData;
			result = true;
//			}

			Wire.endTransmission();

			if (result) {
				_sendRDS(reg0C, reg0D, reg0E, reg0F);
			}
		}
	}
}
