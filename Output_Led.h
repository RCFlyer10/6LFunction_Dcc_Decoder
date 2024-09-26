
#ifndef Output_Led_h
#define Output_Led_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <NmraDcc.h>

#define On 1
#define Off 0

#define A 1
#define B 0


// Modes
#define NORMAL 0
#define DIMMABLE 1
#define STROBE 2
#define RANDOM 3
#define BEACON 4
#define MARS 5
#define FLICKER 6
#define DITCH_A 7
#define DITCH_B 8


// Fade mode config
#define STEP 2

// Beacon mode config
#define STEP_FACTOR TWO_PI * .008

// Strobe mode config
#define PERIOD 1128U
#define DURATION 50U

// Random mode config
#define TIME 15000U
#define PROBABILITY 50U

// Beacon mode congig
#define START_ANGLE PI * 1.5
#define MAX_ANGLE PI * 3.5

extern DCC_DIRECTION direction;

extern uint8_t speed;

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
	void activateCrossing();
	void heartbeat();

private:
	void setFadeTime();
	// Instance variables
	unsigned long _previousMillis;
	unsigned long _crossingTimer;
	unsigned long _fadeTimer;

	uint8_t _randomNumber;
	uint8_t _pin;
	uint8_t _effect;
	uint8_t _dimValue;
	uint8_t _fadeRate;
	uint8_t _flashRate;
	uint8_t _brightValue;
	uint8_t _probability;
	uint16_t _fadeTime;
	bool _crossingActive;
	bool _phase;
	bool _fading;
	bool _fadeDir;
	bool _state;
	bool _ledState;
	bool _fadeOn;
	bool _fadeOff;
	int _fade;
	float _angle;
	float _value;
	float _step;
};
#endif