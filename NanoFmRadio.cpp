/*
 * NanoFmRadio.h
 *
 *  Created on: 2018. jún. 27.
 *      Author: BT-Soft
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Fonts/Org_01.h>
#include <avr/sleep.h>
#include <avr/power.h>

//#define SERIAL_DEBUG

//---- Akkumulátor töltöttség mérése
#define INT_REF_VOLTAGE						1.1f													/* 1.1V a belső referencia feszültség */
#define BATT_VOLTAGE_LOW					3.2f													/* Alacsony akkufesz */
#define BATT_VOLTAGE_HIGH					4.2f													/* Teljesen feltötltött akkufesz */
#define BATT_VOLTAGE_ATT_VALUE		 		(4.7f / (22.0f + 4.7f))									/* 4.7k és 22k elleállás feszosztó értékek az akku feszmérésre*/
#define BATT_VOLTAGE_ATT_LOW				(BATT_VOLTAGE_LOW * BATT_VOLTAGE_ATT_VALUE)				/* feszültésgosztó után a 3.2V értéke) */
#define BATT_VOLTAGE_ATT_HIGH				(BATT_VOLTAGE_HIGH * BATT_VOLTAGE_ATT_VALUE)			/* feszültésgosztó után a 4.2V értéke) */
#define BATT_ADC_MEAS_LOW					(BATT_VOLTAGE_ATT_LOW * 1023.0f / INT_REF_VOLTAGE) 		/* ADC mért érték lemerült akku esetén */
#define BATT_ADC_MEAS_HIGH					(BATT_VOLTAGE_ATT_HIGH * 1023.0f / INT_REF_VOLTAGE) 	/* ADC mért érték feltöltött akku esetén */
#define PIN_BATTERRY_MEAS					A3		/* ATmega328P PIN:26 */
int adcBatterryVoltage;	// Akku feszültsége az ADC mérés szerint

//------------------- LCD Display
#define DEGREE_SYMBOL_CODE 		247		/* Az LCD-n a '°' jel kódja */
#define PIN_LCD_SCLK			8		/* ATmega328P PIN:14 */
#define PIN_LCD_DIN				7		/* ATmega328P PIN:13 */
#define PIN_LCD_DC				6		/* ATmega328P PIN:12 */
#define PIN_LCD_CS				5		/* ATmega328P PIN:11 */
#define PIN_LCD_RST				4		/* ATmega328P PIN:6 */
Adafruit_PCD8544 lcd(PIN_LCD_SCLK, PIN_LCD_DIN, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST);
#define LCD_DEFAULT_FONT_WIDTH	6

#include "BTRDA5807M.h"
BT_RDA5807M radio;
RADIO_INFO radioInfo;
uint16_t freqStep;
uint16_t maxFreq;
uint16_t minFreq;

#include "RDSParser.h"
RDSParser rds;
char rdsServiceName[8 + 2];
char rdsText[64 + 2];
char rdsTime[6];
#define RDS_SCROLL_POSITION_BEGIN -13
int8_t rdsTextScrollPosition = RDS_SCROLL_POSITION_BEGIN;

#include "RotaryEncoder.h"
#define ROTARY_ENCODER_SW_BUTTON	A1
RotaryEncoder rotaryEncoder(A0 /* DT, ATmega328P PIN:23 */, 3 /* CLK, ATmega328P PIN:5*/, ROTARY_ENCODER_SW_BUTTON /* SW, ATmega328P PIN:24 */);
//
//A rotary encoder tekergetése esetén mik változhatnak?
#define NUMBER_OF_CHAR_ARRAY_ELEMENT(x) (sizeof(x) / sizeof(x[0]))
const char *rotaryEncoderClickOptionsStr[] = { /*0*/"Frequency", /*1*/"Seek", /*2*/"Volume", /*3*/"BassBoost", /*4*/"SoftMute", /*5 */"LCD BLight" };
#define ROTARY_ENCODER_CLICK_FREQUENCY_NDX	0
#define ROTARY_ENCODER_CLICK_SEEK_NDX		1
#define ROTARY_ENCODER_CLICK_VOLUME_NDX		2
#define ROTARY_ENCODER_CLICK_BASSBOOST_NDX	3
#define ROTARY_ENCODER_CLICK_SOFTMUTE_NDX	4
#define ROTARY_ENCODER_LCD_BACKLIGHT_NDX	5
#define ROTARY_CHANGE_DEFAULT_INDEX			ROTARY_ENCODER_CLICK_SEEK_NDX	/* seek a defaut menüpont a bekapcsolást követően */
uint8_t rotaryCurrentChangeIndex = ROTARY_CHANGE_DEFAULT_INDEX;

////Hőmérés
#define PIN_TEMP_SENSOR 		2		/* ATmega328P PIN:4, D10 a DS18B20 bemenete */
#define DS18B20_TEMP_SENSOR_NDX 0		/* Dallas DS18B20 hõmérõ szenzor indexe */
#include <OneWire.h>
#define REQUIRESALARMS 			false	/* nem kell a DallasTemperature ALARM supportja */
#include <DallasTemperature.h>
DallasTemperature ds18B20(new OneWire(PIN_TEMP_SENSOR));
float lastTemp;

//LCD Háttérvilágítás állítás
#include "LcdBlackLightAdjuster.h"
#define PIN_LCD_BLACKLIGHT_LED 	11		/* ATmega328P PIN:17 */
#define PIN_PHOTO_SENSOR		A2 		/* ATmega328P PIN:25  */
LcdBlackLightAdjuster lcdBlkAdjuster(PIN_PHOTO_SENSOR, PIN_LCD_BLACKLIGHT_LED);

#include "Config.h"
Config config;

//Üzemállapotok
typedef enum {
	STATE_NORMAL, STATE_INTERNAL, STATE_STANDBY
} State_t;
State_t state = STATE_NORMAL;

//általános buffer az LCD kiírogatásokhoz
char tmpBuff[10];

/**
 * Free Memory
 */
extern unsigned int __bss_end;
extern void *__brkval;
int getFreeMemory() {
	int free_memory;

	if ((int) __brkval == 0) {
		free_memory = ((int) &free_memory) - ((int) &__bss_end);
	} else {
		free_memory = ((int) &free_memory) - ((int) __brkval);
	}

	return free_memory;
}

/**
 * DS18B20 digitális hőmérő szenzor olvasása
 */
float readTemperature(void) {
	ds18B20.requestTemperaturesByIndex(DS18B20_TEMP_SENSOR_NDX);
	float temp = ds18B20.getTempCByIndex(DS18B20_TEMP_SENSOR_NDX);
	return temp;
}

/**
 * Display normál állapotban
 */
void loopNormalDisplay() {

	lcd.clearDisplay();

	//Mini font
	lcd.setFont(&Org_01);
	lcd.setTextSize(1);

	//térerő háromszög
#define MAX_RSSI_SIMBOL_SIZE	5
	uint8_t rssi = map(radioInfo.rssi, 0, RADIO_MAX_RSSI, 0, 10);
	lcd.fillTriangle(0, MAX_RSSI_SIMBOL_SIZE, rssi, MAX_RSSI_SIMBOL_SIZE, rssi, MAX_RSSI_SIMBOL_SIZE - (rssi / 2), BLACK);

	sprintf(tmpBuff, "%2d", radioInfo.rssi);
	lcd.setCursor(13, 5);
	lcd.print(tmpBuff);

	//Ha egy adó csatornán vagyunk..
	if (radioInfo.tuned) {
		//Sztereó szimbólum
		lcd.drawCircle(31, 3, 2, BLACK);
		lcd.drawCircle(34, 3, 2, BLACK);

		//RDS felirat
		lcd.setCursor(43, 5);
		lcd.print(F("RDS"));
	}

	//
	// Batterry
	//
#define BATT_SYMBOL_LENGTH 15
	lcd.drawRoundRect(LCDWIDTH - BATT_SYMBOL_LENGTH - 2, 0, BATT_SYMBOL_LENGTH, 6, 1, BLACK); //battery blokkja
	lcd.fillRect(LCDWIDTH - 2, 2, 1, 2, BLACK); //batterry szimbólum "pozitív" vége

	//Feszmérés - a loopCheckLowBatterry() függvényben van
	byte battery = map(min(adcBatterryVoltage, BATT_ADC_MEAS_HIGH), BATT_ADC_MEAS_LOW, BATT_ADC_MEAS_HIGH, 0, BATT_SYMBOL_LENGTH - 1);
	lcd.fillRect(LCDWIDTH - BATT_SYMBOL_LENGTH - 1, 1, battery, 4, BLACK);

	//
	// Mit változtat a rotary encoder?
	//
	lcd.setCursor(0, 11);
	lcd.print(rotaryEncoderClickOptionsStr[rotaryCurrentChangeIndex]);
	switch (rotaryCurrentChangeIndex) {
		case ROTARY_ENCODER_CLICK_VOLUME_NDX:	// Volume
			lcd.setCursor(75, 11);
			lcd.print(radioInfo.volume);
			break;

		case ROTARY_ENCODER_CLICK_BASSBOOST_NDX: //BasBoost
			lcd.setCursor(70, 11);
			lcd.print(radioInfo.bassBoost ? F("On") : F("Off"));
			break;

		case ROTARY_ENCODER_CLICK_SOFTMUTE_NDX: //SoftMute
			lcd.setCursor(70, 11);
			lcd.print(radioInfo.softMute ? F("On") : F("Off"));
			break;

		case ROTARY_ENCODER_LCD_BACKLIGHT_NDX:	//LCD BackLight
			lcd.setCursor(70, 11);
			lcd.print(lcdBlkAdjuster.isBlState() ? F("On") : F("Off"));
			break;

	}

	//
	// frekvencia kijelzése
	//

	sprintf(tmpBuff, "%3d.%1d", radioInfo.currentFreq / 100, (radioInfo.currentFreq % 100) / 10);
	lcd.setFont(NULL);
	lcd.setTextSize(2);
	lcd.setCursor(0, 14);
	lcd.print(tmpBuff);

	lcd.setTextSize(1);
	lcd.setCursor(65, 21);
	lcd.print(F("MHz"));

#define RDS_INFO_STATION_AND_TIME_LINE_Y 30
	//
	// RDS infók
	//
	//Service name
	lcd.setFont(NULL); //default font
	lcd.setTextSize(1);
	if (rdsServiceName != NULL) {
		//lcd.setCursor(LCDWIDTH / 2 - ((strlen(rdsServiceName) / 2) * LCD_DEFAULT_FONT_WIDTH), 24);
		lcd.setCursor(0, RDS_INFO_STATION_AND_TIME_LINE_Y);
		lcd.print(rdsServiceName);
	}

	//RDS time
	if (rdsTime != NULL) {
		//lcd.setCursor(LCDWIDTH / 2 - ((strlen(rdsTime) / 2) * LCD_DEFAULT_FONT_WIDTH), 32);
		lcd.setCursor(53, RDS_INFO_STATION_AND_TIME_LINE_Y);
		lcd.print(rdsTime);
	}

#define RDS_INFO_TEXT_LINE_Y 40
	//RDS text scroll
	if (rdsText != NULL) {
		lcd.setCursor(0, RDS_INFO_TEXT_LINE_Y);
		for (int8_t i = rdsTextScrollPosition; i < rdsTextScrollPosition + (-1) * RDS_SCROLL_POSITION_BEGIN + 1; i++) {
			if ((i >= strlen(rdsText)) || (i < 0)) {
				lcd.print(' ');
			} else {
				lcd.print(rdsText[i]);
			}
		}
		rdsTextScrollPosition++;
		if ((rdsTextScrollPosition >= strlen(rdsText)) && (rdsTextScrollPosition > 0)) {
			rdsTextScrollPosition = RDS_SCROLL_POSITION_BEGIN;
		}
	}

	lcd.display();
}

/**
 * Display a belső mérés állapotban
 */
void loopInternalDisplay() {

	lcd.clearDisplay();
	lcd.setFont(NULL);
	lcd.setTextSize(1);

	lcd.setCursor(0, 0);
	lcd.setTextColor(WHITE, BLACK);
	lcd.print(F("-INTERN STATE-"));

	lcd.setFont(&Org_01);
	lcd.setCursor(0, 15);
	lcd.setTextColor(BLACK, WHITE);
	lcd.print(F("Batt: "));
	float currBattVoltage = ((float) adcBatterryVoltage * (INT_REF_VOLTAGE / 1023.0)) / BATT_VOLTAGE_ATT_VALUE; //a feszmérés a loopCheckLowBatterry függvényben van
	dtostrf(currBattVoltage, 1, 2, tmpBuff);
	lcd.setCursor(35, 15);
	lcd.print(tmpBuff);
	lcd.setCursor(65, 15);
	lcd.print(F("[V]"));

	lcd.setCursor(0, 21);
	lcd.print(F("Temp: "));
	lcd.setCursor(35, 21);
	float temp = readTemperature();
	dtostrf(temp, 1, 1, tmpBuff);
	lcd.print(tmpBuff);
	lcd.setCursor(65, 21);
	lcd.print(F("[C]"));

	lcd.setCursor(0, 27);
	lcd.print(F("FMem: "));
	lcd.setCursor(35, 27);
	sprintf(tmpBuff, "%d", getFreeMemory());
	lcd.print(tmpBuff);
	lcd.setCursor(65, 27);
	lcd.print(F("[B]"));

	lcd.setCursor(0, 33);
	lcd.print(F("RSSI: "));
	lcd.setCursor(35, 33);
	radio.getRadioInfo(&radioInfo);
	lcd.print(radioInfo.rssi);
	lcd.setCursor(65, 33);
	lcd.print(F("[-]"));

	lcd.display();
}

/**
 * RDS adatok törlése
 */
void clearRdsInfo() {
	memset(rdsServiceName, 0, sizeof(rdsServiceName));
	memset(rdsText, 0, sizeof(rdsText));
	memset(rdsTime, 0, sizeof(rdsTime));
	radio.clearRDS();
}

/**
 * This function will be called by the RDS module when a rds service name was received.
 * The text be displayed on the LCD and written to the serial port
 * and will be stored for the web interface.
 */
void rdsDisplayServiceName(char *name) {
#ifdef SERIAL_DEBUG
	Serial.print(F("RDS-service: "));
	Serial.println(name);
#endif
	strncpy(rdsServiceName, name, sizeof(rdsServiceName));
}

/**
 * This function will be called by the RDS module when a rds text message was received.
 * The text will not displayed on the LCD but written to the serial port
 * and will be stored for the web interface.
 */
void rdsDisplayText(char *text) {

#ifdef SERIAL_DEBUG
	Serial.print(F("RDS-text: '"));
	Serial.print(text);
	Serial.println(F("'"));
#endif

	//üres text kiszűrése
	if (strlen(text) == 0) {
		return;
	}
	strncpy(rdsText, text, sizeof(rdsText));
}

/**
 *
 */
void rdsDisplayTime(uint8_t hour, uint8_t minute) {
#ifdef SERIAL_DEBUG
	Serial.print(F("RDS-time hour: "));
	Serial.print(hour);
	Serial.print(F(", minute: "));
	Serial.println(minute);
#endif

	uint8_t hour1 = hour / 10;
	uint8_t hour2 = hour % 10;
	uint8_t minute1 = minute / 10;
	uint8_t minute2 = minute % 10;

	//érvénytelen idők kiszűrése
	if (hour1 > 2 || hour2 > 9) {
		return;
	}
	if (hour1 == 2 && hour2 > 3) {
		return;
	}
	if (minute1 > 5 || minute2 > 9) {
		return;
	}

	sprintf(rdsTime, "%01d%01d:%01d%01d", hour1, hour2, minute1, minute2);

}

/// retrieve RDS data from the radio chip and forward to the RDS decoder library
void rdsProcess(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4) {
	rds.processData(block1, block2, block3, block4);
}

/**
 * Radio info lekérése
 */
void loopRadio() {
	radio.getRadioInfo(&radioInfo);
	//Ha van RDS, akkor zat is lekérjük
	if (radioInfo.tuned && radioInfo.rds) {
		radio.checkRDS();
	}
}

/**
 * Kikapcsolt állapotban a display kezelése
 */
void loopStandby() {

	float temp = readTemperature();

	if (temp != lastTemp) {
		lastTemp = temp;

		//előző hőmérséklet kijelzésének törlése
		lcd.fillRect(0, 34, LCDWIDTH, LCDHEIGHT, WHITE);

		lcd.setTextSize(2);

		//hőmérséklet
		lcd.setCursor(5, 34);
		dtostrf(temp, 1, 1, tmpBuff);
		lcd.print(tmpBuff);

		//felirat
		lcd.setTextSize(1);
		lcd.setCursor(60, 40);
		sprintf(tmpBuff, "%cC", DEGREE_SYMBOL_CODE);
		lcd.print(tmpBuff);

		lcd.display();
	}
}

/**
 * Rendszer bekapcsolása
 */
void systemSwithcOn() {

	radio.init();
#ifdef SERIAL_DEBUG
	radio._debugEnabled = true;
#endif
	radio.setVolume(config.configVars.volume);
	radio.setFrequency(config.configVars.currentFrequency);
	radio.setBassBoost(config.configVars.bassBoost);
	radio.setSoftMute(config.configVars.softMute);
	radio.setMute(false);

	freqStep = radio.getFrequencyStep();
	maxFreq = radio.getMaxFrequency();
	minFreq = radio.getMinFrequency();

	//Seek változtatás aktív
	rotaryCurrentChangeIndex = ROTARY_CHANGE_DEFAULT_INDEX;

	//RDS adatok törlése
	clearRdsInfo();

	// LCD LED állítgatás
	if (config.configVars.lcdBackLight) {
		lcdBlkAdjuster.on();
	}

	state = STATE_NORMAL;
}

/**
 * Rendszer kikapcsolása
 */
void systemSwithcOff() {

	//konfig mentése
	config.configVars.currentFrequency = radio.getFrequency();
	config.configVars.volume = radio.getVolume();
	config.configVars.bassBoost = radio.getBassBoost();
	config.configVars.softMute = radio.getSoftMute();

	config.configVars.lcdBackLight = lcdBlkAdjuster.isBlState();
	config.save();

	// LCD LED Off
	lcdBlkAdjuster.off();

	//rádió lecsukása
	radio.term();

	lcd.clearDisplay();
	lcd.setFont(NULL);
	lcd.setTextSize(2);
	lcd.setCursor(15, 0);
	lcd.print(F("BT-FM"));

	lcd.setTextSize(1);
	lcd.setCursor(5, 18);
	lcd.print(F("Radio"));

	lcd.setCursor(45, 18);
	sprintf(tmpBuff, "V%s", config.configVars.version);
	lcd.print(tmpBuff);

	lastTemp = -99.9;

	lcd.display();

	//beállítjuk, hogy stadby állapotban vagyunk
	state = STATE_STANDBY;
}

/**
 * Frekvencia változtatás - A Rotary Encoder tekergetésre
 */
void changeFrequency(RotaryEncoder::Direction direction) {

	switch (direction) {
		case RotaryEncoder::Direction::UP:
			if (radioInfo.currentFreq + freqStep <= maxFreq) {
				radio.setFrequency(radioInfo.currentFreq + freqStep);
			}
			break;

		case RotaryEncoder::Direction::DOWN:
			if (radioInfo.currentFreq - freqStep >= minFreq) {
				radio.setFrequency(radioInfo.currentFreq - freqStep);
			}
			break;
	}
	clearRdsInfo();
}
/**
 * Hangerő változtatás - A Rotary Encoder tekergetésre
 */
void changeVolume(RotaryEncoder::Direction direction) {

	switch (direction) {
		case RotaryEncoder::Direction::UP:
			if (radioInfo.volume + 1 <= RADIO_MAX_VOLUME) {
				radio.setVolume(radioInfo.volume + 1);
			}
			break;

		case RotaryEncoder::Direction::DOWN:
			if (radioInfo.volume - 1 >= 0) {
				radio.setVolume(radioInfo.volume - 1);
			}
			break;
	}
}

/**
 * Seek indítás - A Rotary Encoder tekergetésre
 */
void changeSeek(RotaryEncoder::Direction direction) {
	switch (direction) {
		case RotaryEncoder::Direction::UP:
			radio.seekUp(true);
			break;

		case RotaryEncoder::Direction::DOWN:
			radio.seekDown(true);
			break;
	}

	//töröljük az eddig RDS infókat
	clearRdsInfo();

	//várakozunk, hog ybefejeződjön a seek;
	delay(1000);

	//beállítjuk a radio osztályban az aktuális frekvencia értéket
	radio.getFrequency();
}

/**
 * BassBoost változtatás - A Rotary Encoder tekergetésre
 */
void changeBassBoost(RotaryEncoder::Direction direction) {

	switch (direction) {
		case RotaryEncoder::Direction::UP:
			if (!radioInfo.bassBoost) {
				radio.setBassBoost(true);
			}
			break;

		case RotaryEncoder::Direction::DOWN:
			if (radioInfo.bassBoost) {
				radio.setBassBoost(false);
			}
			break;
	}
}
/**
 * SoftMute változtatás - A Rotary Encoder tekergetésre
 */
void changeSoftMute(RotaryEncoder::Direction direction) {

	switch (direction) {
		case RotaryEncoder::Direction::UP:
			if (!radioInfo.softMute) {
				radio.setSoftMute(true);
			}
			break;

		case RotaryEncoder::Direction::DOWN:
			if (radioInfo.softMute) {
				radio.setSoftMute(false);
			}
			break;
	}
}
/**
 * BackLight változtatás - A Rotary Encoder tekergetésre
 */
void changeBackLight(RotaryEncoder::Direction direction) {

	switch (direction) {
		case RotaryEncoder::Direction::UP:
			if (!lcdBlkAdjuster.isBlState()) {
				lcdBlkAdjuster.on();
			}
			break;

		case RotaryEncoder::Direction::DOWN:
			if (lcdBlkAdjuster.isBlState()) {
				lcdBlkAdjuster.off();
			}
			break;
	}
}

/**
 * Rotary encoder
 */
void loopRotaryEncoder() {
	RotaryEncoder::RotaryEncoderResult rencResult = rotaryEncoder.readRotaryEncoder();

	if (state == STATE_NORMAL && rencResult.direction != RotaryEncoder::Direction::NONE) {

		switch (rotaryCurrentChangeIndex) {
			case ROTARY_ENCODER_CLICK_FREQUENCY_NDX: //frequency
				changeFrequency(rencResult.direction);
				break;

			case ROTARY_ENCODER_CLICK_VOLUME_NDX: //volume
				changeVolume(rencResult.direction);
				break;

			case ROTARY_ENCODER_CLICK_SEEK_NDX: //seek
				changeSeek(rencResult.direction);
				break;

			case ROTARY_ENCODER_CLICK_BASSBOOST_NDX: //bass boost
				changeBassBoost(rencResult.direction);
				break;

			case ROTARY_ENCODER_CLICK_SOFTMUTE_NDX: //softmute
				changeSoftMute(rencResult.direction);
				break;

			case ROTARY_ENCODER_LCD_BACKLIGHT_NDX: //LCD backlight
				changeBackLight(rencResult.direction);
				break;
		}

	} else {
		if (rencResult.buttonState != ClickEncoder::Button::Open) {

			switch (rencResult.buttonState) {

				//Egy klikk
				case ClickEncoder::Button::Clicked:
					//Ha ki van kapcsolva, akkor most bekapcsoljuk
					if (state == STATE_STANDBY) {
						systemSwithcOn();
					} else if (state == STATE_INTERNAL) {	//ha belső módban voltunk, akkor visszalépünk normál módba
						state = STATE_NORMAL;
					} else {
						//A rotary encoder tekergetés hatásának változtatása
						rotaryCurrentChangeIndex++;
						if (rotaryCurrentChangeIndex == NUMBER_OF_CHAR_ARRAY_ELEMENT(rotaryEncoderClickOptionsStr)) {
							rotaryCurrentChangeIndex = 0;
						}
					}
					break;

					//dupla klikk
				case ClickEncoder::Button::DoubleClicked:
					//Két klikkre belső módba lépünk
					if (state == STATE_NORMAL) {
						state = STATE_INTERNAL;
					}
					break;

					//Kikapcsolás
				case ClickEncoder::Button::Held:
					if (state != STATE_STANDBY) {
						systemSwithcOff();
						delay(2000);
					}
					break;
			}
		}
	}
}

/**
 * 1 msec-enként meghívott ISR a RotaryEncoder számára
 */
void timer1IsrHandler() {
	rotaryEncoder.service();
}

/**
 * Arduino core - setup
 */
void setup() {
#ifdef SERIAL_DEBUG
	Serial.begin(115200);
#endif

	lcd.begin();
	lcd.setTextColor(BLACK);

	//Config beolvasása
	config.read();

	//RDS
	radio.init();
	radio.setBand(RADIO_BAND_FM);
	radio.setFrequency(config.configVars.currentFrequency);
	radio.setVolume(config.configVars.volume);
	radio.setBassBoost(config.configVars.bassBoost);
	radio.setSoftMute(config.configVars.softMute);

	radio.attachReceiveRDS(rdsProcess);
	rds.init();
	rds.attachServicenNameCallback(rdsDisplayServiceName);
	rds.attachTextCallback(rdsDisplayText);
	rds.attachTimeCallback(rdsDisplayTime);

	//Timer1 a rotary encoderhez
	Timer1.initialize(1000);
	Timer1.attachInterrupt(timer1IsrHandler);

	//Akku feszmérés
	analogReference(INTERNAL); //1.1V a referencia feszültség
	pinMode(PIN_BATTERRY_MEAS, INPUT);

	//Hőmérés felhúzása
	ds18B20.begin();

	//LCD háttérvilágítás
	lcdBlkAdjuster.init();

	//Kikapcsolás
	systemSwithcOff();
}

/**
 * LowBatterry PowerDown módban a rotary encoder button-ra kötött megszakítás
 */
void powerDownWakeUpIsr() {
	sleep_disable()
	;
	detachInterrupt(ROTARY_ENCODER_SW_BUTTON);
}
/**
 * Alacsony akkufeszültés vizsgálata
 */
void loopCheckLowBatterry() {

	//Megmérjük az akku feszülségét
	adcBatterryVoltage = analogRead(PIN_BATTERRY_MEAS);

	//Voltban kifejezve
	float currBattVoltage = ((float) adcBatterryVoltage * (INT_REF_VOLTAGE / 1023.0)) / BATT_VOLTAGE_ATT_VALUE;

	//Akku feszültég még OK?
	if (currBattVoltage > BATT_VOLTAGE_LOW) {
		return; //akkufesz OK
	}

	//
	// Alacsony az akkufesz!!!
	//

	//Minden kikapcsolása
	systemSwithcOff();

	//Kiírjuk a figyelmeztető üzenetet
	lcd.clearDisplay();
	lcd.setFont(NULL);
	lcd.setTextSize(2);
	lcd.setCursor(0, 0);
	lcd.print(F("LOW BAT"));

	lcd.setCursor(0, 20);
	lcd.print(F("VOLTAGE"));

	lcd.setTextSize(1);
	lcd.setCursor(20, 38);
	dtostrf(currBattVoltage, 1, 2, tmpBuff);
	lcd.print(tmpBuff);
	lcd.setCursor(45, 38);
	lcd.print(F("[V]"));

	lcd.display();

	//
	// ---- Sleep
	//

	//--- Sleep mode
	attachInterrupt(ROTARY_ENCODER_SW_BUTTON, powerDownWakeUpIsr, FALLING);

	set_sleep_mode(SLEEP_MODE_IDLE);
	__power_all_disable();
	sleep_enable()
	;

	sleep_mode()
	; // !!!

	sleep_disable()
	;
	__power_all_enable();
}

/**
 * Arduino core - loop
 */
void loop() {

	//Alacsony feszülség esetén kiírjuk a képernyőre a figyelmeztető üzenetet és kikapcsolunk
	loopCheckLowBatterry();

	//RotaryEncoder olvasása
	loopRotaryEncoder();

	//LCD háttérvilágítás módosítása, ha aktív
	if (lcdBlkAdjuster.isBlState()) {
		lcdBlkAdjuster.adjust();
	}

	static long lastLoopMsec = 0L;
	switch (state) {
		//Normál állapot
		case STATE_NORMAL:

			static long lastLoopRadioMsec = 0L;
			if ((millis() - lastLoopRadioMsec) >= 50L) {
				loopRadio();
				lastLoopRadioMsec = millis();
			}
			if ((millis() - lastLoopMsec) >= 500L) {
				loopNormalDisplay();
				lastLoopMsec = millis();
			}

			break;

			//Belső mérés állapot
		case STATE_INTERNAL:
			if ((millis() - lastLoopMsec) >= 500L) {
				loopInternalDisplay();
				lastLoopMsec = millis();
			}
			break;

		default:
			//Standby állapotban vagyunk -> 1mp-enként frissítgetjük a hőmérsékletet
			static long lastStandbyMsec = 0L;
			if ((millis() - lastStandbyMsec) >= 1000L) {
				loopStandby();
				lastStandbyMsec = millis();
			}
			break;
	}
}

