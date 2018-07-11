/*
 * Config.h
 *
 *  Created on: 2018. júl. 8.
 *      Author: BT-Soft
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>
#include <EEPROM.h>

#include "Defaults.h"

class Config {

public:
	typedef struct config_t {
		char id[sizeof(CONFIG_CONFIGVARS_ID)];
		char version[sizeof(CONFIG_CONFIGVARS_VERSION)];
		uint16_t currentFrequency;
		uint8_t volume;
		bool bassBoost;
		bool softMute;
		bool lcdBackLight;

	} ConfigT;

	//config
	ConfigT configVars;

	void read() {
		memset(&configVars, 0, sizeof(configVars));
		EEPROM.get(0, configVars);
		if (memcmp(CONFIG_CONFIGVARS_ID, configVars.id, sizeof(configVars.id)) != 0 || memcmp(CONFIG_CONFIGVARS_VERSION, configVars.version, sizeof(CONFIG_CONFIGVARS_VERSION) != 0)) {
			memcpy(configVars.id, CONFIG_CONFIGVARS_ID, sizeof(configVars.id));
			memcpy(configVars.version, CONFIG_CONFIGVARS_VERSION, sizeof(configVars.version));
			configVars.currentFrequency = RADIO_DEFAUL_FREQEUNCY;
			configVars.volume = RADIO_DEFAUL_VOLUME;
			configVars.bassBoost = RADIO_DEFAUL_BASSBOOST;
			configVars.softMute = RADIO_DEFAUL_SOFTMUTE;
			configVars.lcdBackLight = LCD_DEFAULT_BACKLIGHT;
			save();
		}
	}

	/**
	 * Konfig mentése
	 */
	void save(void) {
		//le is mentjük
		EEPROM.put(0, configVars);
	}
};

#endif /* CONFIG_H_ */
