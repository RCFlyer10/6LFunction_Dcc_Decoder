
#ifndef Output_Led_h
#define Output_Led_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <NmraDcc.h>

typedef enum {
	Off,
	On
} ON_OFF;

typedef enum {
	B,
	A
} PHASE;

typedef enum {
	NORMAL,
	AUTO_DIM,
	RANDOM,
	STROBE,
	BEACON,
	MARS,
	FLICKER,
	EOT,
	DITCH_A,
	DITCH_B
} MODES;

// Strobe mode config
#define S_DURATION 100U

// Beacon/Mars mode congig
#define BEACON_STEPS 72
#define BEACON_STEP PI / BEACON_STEPS
#define MARS_STEPS 36
#define MARS_STEP PI / MARS_STEPS
#define START_RADIAN PI
#define MAX_RADIAN PI * 3

extern DCC_DIRECTION myDirection;
extern uint8_t mySpeed;

const uint8_t PROGMEM beaconBrightTable[] = { 0, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 52 };

const uint8_t PROGMEM beaconFlashTable[] = { 0, 80, 90, 120, 110, 120, 130, 140, 150, 160, 170, 180, 190, 210, 220, 255 };

const uint8_t PROGMEM gamma8[] = {
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
	1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
	2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
	5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

/*!
 *  @brief  Class that stores state and functions for the Funtion Led
 */
class Output_Led {
public:
	// Constructor
	Output_Led(uint8_t pin);
	// Methods
	void setState(bool state);	
	void setEffect(uint8_t effect);
	void setConfig_1(uint8_t value);
	void setConfig_2(uint8_t value);
	void setProbability(uint8_t value);
	void setSampleTime(uint8_t tvalue);
	void setSpeed(uint8_t value);
	void setHoldoverTime(uint8_t value);
	void activateCrossing();
	void heartbeat();

private:	
	// Instance variables
	unsigned long _previousMillis;
	unsigned long _crossingTimer;
	unsigned long _fadeTimer;
	uint8_t _randomNumber;
	uint8_t _pin;
	uint8_t _effect;
	uint8_t _dim;	
	uint8_t _dimIndex;
	uint8_t _fadeRate;	
	uint8_t _fadeIndex;	
	uint8_t _flashRate;	
	uint8_t _bright;	
	uint8_t _brightIndex;
	uint8_t _probability;
	uint8_t _setSpeed;
	uint8_t _beaconIntensity;	;	
	uint8_t _flashIntensity;
	uint16_t _strobePeriod;
	uint16_t _ditchPeriod;
	uint16_t _holdOverTime;
	uint16_t _sampleTime;
	bool _crossingActive;
	bool _phase;
	bool _fading;
	bool _flash;
	bool _state;
	bool _ledState;	
	float _radians;	
	float _beaconPeriod;
};
#endif