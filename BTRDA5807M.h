/*
 * BTRDA5807M.h
 *
 *  Created on: 2018. júl. 5.
 *      Author: BT-Soft
 *
 * Eredeti ötle:
 * @see: https://github.com/mathertel/Radio
 *
 */
#ifndef BTRDA5807M_H_
#define BTRDA5807M_H_

#include <Arduino.h>

//-------------------------------------------------------------------------------------------------------------------------
// I2C-Address szekvenciális eléréshez
#define I2C_SEQ  		0x10

// I2C-Address indexelt eléréshez
#define I2C_INDX  		0x11

////
//// --- Register 0x02 - RADIO_REG_CTRL
////
//#define RADIO_REG_02    		0x02
//#define RADIO_REG_CTRL  		RADIO_REG_02
//#define RADIO_REG_CTRL_OUTPUT 	0x8000
//#define RADIO_REG_CTRL_UNMUTE 	0x4000
//#define RADIO_REG_CTRL_MONO   	0x2000
//#define RADIO_REG_CTRL_BASS   	0x1000
//#define RADIO_REG_CTRL_SEEKUP 	0x0200
//#define RADIO_REG_CTRL_SEEK   	0x0100
//#define RADIO_REG_CTRL_RDS    	0x0008
//#define RADIO_REG_CTRL_NEW    	0x0004
//#define RADIO_REG_CTRL_RESET  	0x0002
//#define RADIO_REG_CTRL_ENABLE 	0x0001
//typedef struct {
//
//	uint8_t audioHighZDisable :1;	//Audio Output High-Z Disable.
//									//0 = High impedance;
//									//1 = Normal operation
//
//	uint8_t muteDisable :1;		//Mute Disable.
//								//0 = Mute;
//								//1 = Normal operation
//
//	uint8_t forceMono :1;		//Mono Select.
//								//0 = Stereo;
//								//1 = Force mono
//
//	uint8_t bassBoost :1;		//Bass Boost.
//								//0 = Disabled;
//								//1 = Bass boost enabled
//
//	uint8_t rclkNonCalMode :1;	//0=RCLK clock is always supply
//								//1=RCLK clock is not always supply when FM work
//								//( when 1, RDA5807M can’t directly support -20 ℃ ~70 ℃ temperature. Only suppory ±20℃ temperature swing from tune point)
//
//	uint8_t rclkDirectInputMode :1;	//1=RCLK clock use the directly input mode
//
//	uint8_t seekUp :1;			//Seek Up.
//								//0 = Seek down;
//								//1 = Seek up
//
//	uint8_t seek :1;			//Seek.
//								//0 = Disable stop seek;
//								//1 = Enable Seek begins in the direction specified by SEEKUP and ends when a channel is found, or the entire band has been searched.
//								//The SEEK bit is set low and the STC bit is set high when the seek operation completes.
//
//	uint8_t seekMode :1;		//Seek Mode.
//								//0 = wrap at the upper or lower band limit and continue seeking.
//								//1 = stop seeking at the upper or lower band limit
//
//	uint8_t clkMode :3;			//000=32.768kHz,
//								//001=12Mhz,
//								//101=24Mhz,
//								//010=13Mhz,
//								//110=26Mhz,
//								//011=19.2Mhz,
//								//111=38.4Mhz
//
//	uint8_t rdsEnable :1;		//RDS/RBDS enable,
//								//If 1, rds/rbds enable
//
//	uint8_t newDemodMethod :1;	//New Demodulate Method Enable, can improve, the receive sensitivity about 1dB.
//
//	uint8_t softReset :1;		//Soft reset.
//								//If 0, not reset;
//								//If 1, reset.
//
//	uint8_t pwrEnable :1;		//Power Up Enable.
//								//0 = Disabled;
//								//1 = Enabled
//
//} RDA5807M_02_reg_type __attribute__ ((__packed__));
//
//typedef union {
//	RDA5807M_02_reg_type reg;
//	uint16_t word;
//} RDA5807M_02_type __attribute__ ((__packed__));
//
////
//// --- Register 0x03 - RADIO_REG_CHAN
////
//#define RADIO_REG_03    			0x03
//#define RADIO_REG_CHAN				RADIO_REG_03
//#define RADIO_REG_CHAN_SPACE     	0x0003
//#define RADIO_REG_CHAN_SPACE_100 	0x0000
//#define RADIO_REG_CHAN_BAND      	0x000C
//#define RADIO_REG_CHAN_BAND_FM      0x0000
//#define RADIO_REG_CHAN_BAND_FMWORLD 0x0008
//#define RADIO_REG_CHAN_TUNE   		0x0010
//typedef struct {
//	uint16_t channel :10;	//Channel Select.
//							//BAND = 0 Frequency =  Channel Spacing (kHz) x CHAN+ 87.0 MHz
//							//BAND = 1or 2 Frequency = Channel Spacing (kHz) x CHAN + 76.0 MHz
//							//BAND = 3 Frequency = Channel Spacing (kHz) x CHAN + 65.0 MHz
//							//CHAN is updated after a seek operation.
//
//	uint8_t direct :1;		//Directly Control Mode, Only used when test
//
//	uint8_t tune :1;		//Tune.
//							//0 = Disable.
//							//1 = Enable.
//							//The tune operation begins when the TUNE bit is set high.
//							//The STC bit is set high when the tune operation completes. The tune bit is reset to low automatically when the tune operation completes..
//
//	uint8_t band :2;		//Band Select.
//							//00 = 87-108 MHz (US/Europe)
//							//01 = 76-91 MHz (Japan).
//							//10 = 76-108 MHz (world wide)
//							//11 = 65-76 MHz East Europe or 50-65MHz
//
//	uint8_t space :2;		//Channel Spacing.
//							//00 = 100 kHz
//							//01 = 200 kHz.
//							//10 = 50kHz
//							//11 = 25KHz.
//} RDA5807M_03_reg_type __attribute__ ((__packed__));
//typedef union {
//	RDA5807M_03_reg_type reg;
//	uint16_t word;
//} RDA5807M_03_type __attribute__ ((__packed__));
//
////
//// --- Register 0x04
////
//#define RADIO_REG_04    			0x04
//#define RADIO_REG_R4_AFC   			0x0100
//#define RADIO_REG_R4_SOFTMUTE   	0x0200
//#define RADIO_REG_04_EM50   		0x0800
//typedef struct {
//	uint8_t reserved_3 :1;			//reserved
//
//	uint8_t stcInterruptEnable :1;  //nem dokumentált...
//
//	uint8_t reserved_2 :2;			//reserved
//
//	uint8_t deEmphasis :1; 			//De-emphasis.
//									//0 = 75 µs;
//									//1 = 50 µs
//
//	uint8_t reserved_1 :1;			//Reserved
//
//	uint8_t softMute :1;  			//If 1, softmute enable
//
//	uint8_t afcDisable :1;			//AFC disable.
//									//If 0, afc work;
//									//If 1, afc disabled.
//
//	uint8_t undoc :8;				//nem dokumentált
//
//} RDA5807M_04_reg_type __attribute__ ((__packed__));
//typedef union {
//	RDA5807M_04_reg_type reg;
//	uint16_t word;
//} RDA5807M_04_type __attribute__ ((__packed__));
//
////
//// --- Register 0x02 - RADIO_REG_VOL
////
//#define RADIO_REG_05 					0x05
//#define RADIO_REG_VOL 					RADIO_REG_05
//#define RADIO_REG_VOL_MAX_VOL_MASK 		0x000F
//typedef struct {
//	uint8_t interruptMode :1; 	//If 0, generate 5ms interrupt; If 1, interrupt last until read reg0CH action	occurs.
//
//	uint8_t reserved_2 :3;		//Reserved
//
//	uint8_t seekThreshold :4;	//Seek SNR threshold value
//
//	uint8_t undoc :2;			//nem dokumentált
//
//	uint8_t reserved_1 :2;		//Reserved
//
//	uint8_t volume :4;			//DAC Gain Control Bits (Volume).
//								//0000=min;
//								//1111=max.
//								//Volume scale is logarithmic When 0000, output mute and output impedance is very large
//} RDA5807M_05_reg_type __attribute__ ((__packed__));
//typedef union {
//	RDA5807M_05_reg_type reg;
//	uint16_t word;
//} RDA5807M_05_type __attribute__ ((__packed__));
//
////
//// --- Register 0x06
////
//#define RADIO_REG_06    0x06
//typedef struct {
//	uint8_t reserved_1 :1;  		//Reserved
//
//	uint8_t openMode :2;			//Open reserved register mode.
//									//11=open behind registers writing function
//									//others: only open behind registers reading function
//
//	uint8_t undoc :13;				//nem dokumentált
//
//} RDA5807M_06_reg_type __attribute__ ((__packed__));
//typedef union {
//	RDA5807M_06_reg_type reg;
//	uint16_t word;
//} RDA5807M_06_type __attribute__ ((__packed__));
//
////
//// --- Register 0x07
////
//#define RADIO_REG_07    0x07
//typedef struct {
//	uint8_t reserved_2 :1;				//Reserved
//
//	uint8_t softBlendThreshold :5; 		//Threshold for noise soft blend setting, unit 2dB
//
//	uint8_t mode65M_50M :1; 			//Valid when band[1:0] = 2Ă˘â‚¬â„˘b11 (0x03H_bit<3:2>).
//										//1 = 65~76 MHz;
//										//0 = 50~76 MHz.
//
//	uint8_t reserved_1 :1;	 			//Reserved
//
//	uint8_t seekThOld :6; 				//Seek threshold for old seek mode, Valid when Seek_Mode=001
//
//	uint8_t softBlendEnable :1;			//If 1, Softblend enable
//
//	uint8_t freqMode :1;				//If 1, then freq setting changed. Freq = 76000(or 87000) kHz + freq_direct (08H) kHz.
//
//} RDA5807M_07_reg_type __attribute__ ((__packed__));
//typedef union {
//	RDA5807M_07_reg_type reg;
//	uint16_t word;
//} RDA5807M_07_type __attribute__ ((__packed__));
//
////
//// --- Register 0x0A - RADIO_REG_STATUS
////
//#define RADIO_REG_0A      				0x0A
//#define RADIO_REG_STATUS  				RADIO_REG_0A
//#define RADIO_REG_0A_RDS       			0x8000
//#define RADIO_REG_0A_RDSBLOCK  			0x0800
//#define RADIO_REG_0A_STEREO    			0x0400
//#define RADIO_REG_0A_NR        			0x03FF
//#define RADIO_REG_0A_READ_CHANNEL_MASK  0x03FF
//typedef struct {
//	uint8_t rdsReady :1;			//RDS ready
//									//0 = No RDS/RBDS group ready(default).
//									//1 = New RDS/RBDS group ready
//
//	uint8_t seekTuneComplete :1; 	//Seek/Tune Complete.
//									//0 = Not complete
//									//1 = Complete
//									//The seek/tune complete flag is set when the seek or tune operation completes.
//
//	uint8_t seekFail :1;			//Seek Fail.
//									// 0 = Seek successful;
//									//1 = Seek failure
//									//The seek fail flag is set when the seek operation fails to find a channel with an RSSI level greater than SEEKTH[5:0].
//
//	uint8_t rdsSync :1; 			//RDS Synchronization
//									//0 = RDS decoder not synchronized(default).
//									//1 = RDS decoder synchronized.
//									//Available only in RDS Verbose mode
//
//	uint8_t rdsBklE :1; 			//When RDS enable:
//									//1 = Block E has been found
//									//0 = no Block E has been found
//
//	uint8_t stereo :1; 				//Stereo Indicator.
//									//0 = Mono;
//									//1 = Stereo
//
//	uint8_t readChannel :10; 		//Read Channel.
//									//BAND = 0 Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 87.0 MHz
//									//BAND = 1 or 2 Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 76.0 MHz
//									//BAND = 3 Frequency = Channel Spacing (kHz) x READCHAN[9:0]+ 65.0 MHz
//									//READCHAN[9:0] is updated after a tune or seek operation.
//} RDA5807M_0A_reg_type __attribute__ ((__packed__));
//typedef union {
//	RDA5807M_0A_reg_type reg;
//	uint16_t word;
//} RDA5807M_0A_type __attribute__ ((__packed__));
//
////
//// --- Register 0x0B - RADIO_REG_RB
////
//#define RADIO_REG_0B      0x0B
//#define RADIO_REG_RB      RADIO_REG_0B
//#define RADIO_REG_0B_ISSTATION		0x0100
//#define RADIO_REG_0B_FMREADY  		0x0080
//typedef struct {
//	uint8_t rssi :7;		//RSSI.
//							//000000 = min,
//							//111111 = max,
//							//RSSI scale is logarithmic
//
//	uint8_t isStation :1; 	//1 = the current channel is a station,
//							//0 = the current channel is not a station
//
//	uint8_t fmReady :1;	 	//1=ready,
//							//0=not ready
//
//	uint8_t reserved :2;  	//reserved
//
//	uint8_t abcd_e :1; 		//1= the block id of register 0cH,0dH,0eH,0fH is E
//							//0= the block id of register 0cH, 0dH, 0eH,0fH is A, B, C, D
//
//	uint8_t blerA :2;		//Block Errors Level of RDS_DATA_0,  and is always read as Errors Level of RDS BLOCK A (in RDS mode) or BLOCK E (in RBDS mode when ABCD_E flag is 1)
//							//00= 0 errors requiring correction
//							//01= 1~2 errors requiring correction
//							//10= 3~5 errors requiring correction
//							//11= 6+ errors or error in checkword, correction not possible.
//							//Available only in RDS Verbose mode
//
//	uint8_t blerB :2;		//Block Errors Level of RDS_DATA_1, and is always read as Errors Level of RDS BLOCK B (in RDS mode ) or E (in RBDS mode when ABCD_E flag is 1).
//							//00= 0 errors requiring correction
//							//01= 1~2 errors requiring correction
//							//10= 3~5 errors requiring correction
//
//} RDA5807M_0B_reg_type __attribute__ ((__packed__));
//typedef union {
//	RDA5807M_0B_reg_type reg;
//	uint16_t word;
//} RDA5807M_0B_type __attribute__ ((__packed__));
//
////
//// --- Register 0x0C - RADIO_REG_RDSA
////
//#define RADIO_REG_RDSA   0x0C
//typedef union {
//	uint16_t word;
//} RDA5807M_0C_type __attribute__ ((__packed__));
//
////
//// --- Register 0x0D - RADIO_REG_RDSB
////
//#define RADIO_REG_RDSB   0x0D
//typedef union {
//	uint16_t word;
//} RDA5807M_0D_type __attribute__ ((__packed__));
//
////
//// --- Register 0x0E - RADIO_REG_RDSC
////
//#define RADIO_REG_RDSC   0x0E
//typedef union {
//	uint16_t word;
//} RDA5807M_0E_type __attribute__ ((__packed__));
//
////
//// --- Register 0x0F - RADIO_REG_RDSD
////
//#define RADIO_REG_RDSD   0x0F
//typedef union {
//	uint16_t word;
//} RDA5807M_0F_type __attribute__ ((__packed__));
//
////
//// --- Státusz regiszterek
////
//typedef union {
//	RDA5807M_0A_type _reg_r0A;
//	RDA5807M_0B_type _reg_r0B;
//	RDA5807M_0C_type _reg_r0C;
//	RDA5807M_0D_type _reg_r0D;
//	RDA5807M_0E_type _reg_r0E;
//	RDA5807M_0F_type _reg_r0F;
//} RDA5807M_Status_Registers_type __attribute__ ((__packed__));

//-------------------------------------------------------------------------------------------------------------------------

#define RADIO_MAX_RSSI 		0b111111	/* , 63d, 0x3F, az RSSI logaritmikus érték! */
#define RADIO_MAX_VOLUME  	15

//Frekvencia határok
const uint16_t RDA5807M_BAND_LO[4] PROGMEM = { 8700, 7600, 7600, 6500 };
const uint16_t RDA5807M_BAND_HI[4] PROGMEM = { 10800, 9100, 10800, 7600 };
const uint8_t RDA5807M_BAND_SPACING[4] PROGMEM = { 100, 200, 50, 25 };

// ----- type definitions -----

/// Band datatype.
/// The BANDs a receiver probably can implement.
enum RADIO_BAND {
	RADIO_BAND_FM = 0, 			// 00 = 87-108 MHz (US/Europe)
	RADIO_BAND_FM_JAPAN = 1, 	// 01 = 76-91 MHz (Japan).
	RADIO_BAND_FM_WORLD = 2, 	// 10 = 76-108 MHz (world wide)
	RADIO_BAND_FM_EAST = 3, 	// 11 = 65-76 MHz East Europe or 50-65MHz
};

/// A structure that contains information about the radio features from the chip.
typedef struct RADIO_INFO {
	uint16_t currentFreq; 	// Aktuális vételi frekvencia
	uint8_t rssi;  			// Radio Station Strength Information.
	bool rds;      			// RDS information is available.
	bool tuned;    			// A stable frequency is tuned.
	//--
	bool stereo;   			// Stereo audio is available
	uint8_t volume;			// Aktuális hangerő
	bool bassBoost;			//bassBoost
	bool softMute;
};

/// callback function for passing RDS data.
extern "C" {
typedef void (*receiveRDSFunction)(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4);
}

class BT_RDA5807M {
public:

	bool _debugEnabled = false;

	BT_RDA5807M();

	bool init();
	void term();

// ----- Audio features -----

	void setVolume(uint8_t newVolume);
	uint8_t getVolume(void) {
		return _volume;
	}
	void setBassBoost(bool switchOn);bool getBassBoost(void) {
		return _bassBoost;
	}
	void setMono(bool switchOn);
	void setMute(bool switchOn);
	void setSoftMute(bool switchOn);    // Set the soft mute mode (mute on low signals) on or off.
	bool getSoftMute(void) {
		return _softMute;
	}

// ----- Receiver features -----
	void setBand(RADIO_BAND newBand);
	void setFrequency(uint16_t newF);
	void setBandFrequency(RADIO_BAND newBand, uint16_t newFreq) {
		setBand(newBand);
		setFrequency(newFreq);
	}

	uint16_t getFrequency(void);

	void seekUp(bool toNextSender = true);   // start seek mode upwards
	void seekDown(bool toNextSender = true); // start seek mode downwards

// ----- combined status functions -----

	void getRadioInfo(RADIO_INFO *info); // Retrieve some information about the current radio function of the chip.

	uint16_t getFrequencyStep(void) {
		return _freqSteps;
	}
	uint16_t getMaxFrequency(void) {
		return _freqHigh;
	}
	uint16_t getMinFrequency(void) {
		return _freqLow;
	}

	void checkRDS(void);
	void clearRDS() {
		if (_sendRDS) {
			_sendRDS(0x0000, 0x0000, 0x0000, 0x0000);
		}
	}

	// send valid and good data to the RDS processor via newFunction
	// remember the RDS function
	void attachReceiveRDS(receiveRDSFunction newFunction) {
		_sendRDS = newFunction;
	}

private:

	uint8_t _volume = 0;    			// Last set volume level.
	RADIO_BAND _band = RADIO_BAND_FM;   // Last set band.
	uint16_t _currentFreq = 10330;   	// Last set frequency.

	uint16_t _freqLow = RDA5807M_BAND_LO[RADIO_BAND_FM];    // Lowest frequency of the current selected band.
	uint16_t _freqHigh = RDA5807M_BAND_HI[RADIO_BAND_FM];   // Highest frequency of the current selected band.
	uint16_t _freqSteps = RDA5807M_BAND_SPACING[RADIO_BAND_FM];  // Resolution of the tuner.

	bool _bassBoost;bool _softMute;

// ----- local variables

	uint16_t reg02;
	uint16_t reg03;
	uint16_t reg04;
	uint16_t reg05;
	uint16_t reg06;
	uint16_t reg07;

	uint16_t reg0A;
	uint16_t reg0B;
	uint16_t reg0C;
	uint16_t reg0D;
	uint16_t reg0E;
	uint16_t reg0F;

// ----- low level communication to the chip using I2C bus

	void _readRegisters(); // read all status & data registers
	void _saveRegisters();     // Save writable registers back to the chip
	void _saveRegister(uint8_t regAddresss, uint16_t value); // Save one register back to the chip

	void _write16(uint16_t val);        // Write 16 Bit Value on I2C-Bus
	uint16_t _read16(void);

	receiveRDSFunction _sendRDS = NULL; // Registered RDS Function that is called on new available data.

};

#endif /* BTRDA5807M_H_ */
