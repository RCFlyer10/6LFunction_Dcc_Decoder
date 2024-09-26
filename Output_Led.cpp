#include "Output_Led.h"

uint8_t indexedValue[] = { 0, 2, 4, 8, 16, 24, 32, 56, 72, 88, 104, 120, 136, 168, 200, 255 };

Output_Led::Output_Led(uint8_t pin) {
	_pin = pin; 
	_state = Off;
	_effect = 0;
	_fade = 0;
	_fadeOn = false;
	_fadeOff = false;
	setState(Off);
	analogWrite(_pin, 255);
}

void Output_Led::setEffect(uint8_t effect) {
	_effect = effect;
	setState(Off);
}

void Output_Led::setConfig_1(uint8_t value) {	
	_brightValue = indexedValue[value & 0x0f];	
	_dimValue = indexedValue[value & 0xf0];
	setFadeTime();
}

void Output_Led::setConfig_2(uint8_t value) {
	_fadeRate = (value & 0x0f) * 17;
	_flashRate = ((value & 0xf0) >> 4) * 17;	
	_step = STEP_FACTOR * ((_flashRate * .002) + 0.74);
	setFadeTime();
}

void Output_Led::setProbability(uint8_t value) {
	_probability = value;
}

void Output_Led::setState(bool state) {
	if (_state != state) {
		_ledState = LOW;
		_fadeDir = true;
		_fading = true;
		_phase = true;
		_crossingActive = false;
		_crossingTimer = 0;
		_previousMillis = 0;
		_randomNumber = random(100);		
	}
	_state = state;
}

void Output_Led::activateCrossing() {
	if (_effect == DITCH_A || _effect == DITCH_B) {
		if (_crossingActive == false) {
			_crossingActive = true;
			_phase = A;
			_previousMillis = 0;
		}
		_crossingTimer = millis();
	}
}

void Output_Led::setFadeTime() {
	uint8_t steps = _brightValue / STEP;
	_fadeTime = (100 * _fadeRate) / steps;
}

void Output_Led::heartbeat() {
	unsigned long currentMillis = millis();
	switch (_effect) {
		case NORMAL: 
			if (_state == On) {
				if (_fade < _brightValue) {
					_fadeOn = true;
				}
			}
			else {
				if (_fade > 0) {
					_fadeOff = true;
				}
			}
			break;
		
		case DIMMABLE: 
			if (_state == On) {
				if (direction == DCC_DIR_REV) {
					analogWrite(_pin, 255 - _dimValue);
				}
				else {
					analogWrite(_pin, 255 - _brightValue);
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;
		
		case STROBE: 
			if (_state == On) {
				if (currentMillis - _previousMillis >= PERIOD - (_flashRate << 1) && _ledState == LOW) {
					_previousMillis = currentMillis;
					_ledState = HIGH;
				}
				else if (currentMillis - _previousMillis >= DURATION && _ledState == HIGH) {
					_ledState = LOW;
				}
				if (_ledState == HIGH) {
					analogWrite(_pin, 255 - _brightValue);
				}
				else {
					analogWrite(_pin, 255);
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;
		
		case RANDOM: 
			if (_state == On) {
				if (speed < 10 || direction == DCC_DIR_REV) {
					if (_fade < _brightValue) {
						_fadeOn = true;
						_fadeOff = false;
					}
					_ledState = HIGH;
					_previousMillis = currentMillis;
				}
				if (speed > 10 && direction == DCC_DIR_FWD && currentMillis - _previousMillis >= TIME) {					
					if (_probability >= _randomNumber) {
						if (_ledState == HIGH) {
							_ledState = LOW;
							if (_fade > 0) {
								_fadeOff = true;
							}
						}
						else {
							_ledState = HIGH;
							if (_fade < _brightValue) {
								_fadeOn = true;
							}
						}
					}
					_randomNumber = random(100);
					_previousMillis = currentMillis;
				}
			}
			else {
				_ledState = LOW;
				if (_fade > 0) {
					_fadeOff = true;
				}			
			}			
			break;
		
		case BEACON: 
			if (_state == On) {
				uint8_t intensity = _brightValue >> 1;
				if (currentMillis - _previousMillis >= 8) {
					_previousMillis = currentMillis;
					_angle = _angle + _step;                                	// increase angle by step
				}
				if (_angle >= MAX_ANGLE) {                                   	// keep angle in bounds
					_angle = START_ANGLE;
				}
				_value = sin(_angle) * intensity + intensity;         			// calculate fade, will follow sine wave
				analogWrite(_pin, 255 - _value);                   				// set beacon to fade value
			}
			else {
				analogWrite(_pin, 255);
			}
			break;
		
		case MARS: 
			if (_state == On) {
				static uint8_t fade = 0;
				static uint8_t count = 0;
				uint8_t intensity = _brightValue >> 3;
				if (currentMillis - _previousMillis >= 44U - (_flashRate >> 3) && _fading) {
					_previousMillis = currentMillis;
					if (_fadeDir) {
						fade++;
					}
					if (fade > intensity) {
						fade = intensity;
						_fadeDir = false;
						count++;
						if (count >= 3) {
							analogWrite(_pin, 255 - _brightValue);
							_fading = false;
							count = 0;
						}
					}
					if (!_fadeDir) {
						fade--;
					}
					if (fade == 0) {
						_fadeDir = true;
						count++;
					}
					if (_fading) {
						analogWrite(_pin, 255 - fade);
					}
				}
				else if (currentMillis - _previousMillis >= DURATION) {
					analogWrite(_pin, 250);
					_fading = true;
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;
		
		case FLICKER: 
			if (_state == On) {

				if (currentMillis - _previousMillis >= 255U - random(_flashRate)) {
					_previousMillis = currentMillis;
					uint8_t temp = random(_dimValue, _brightValue);
					analogWrite(_pin, 255 - temp);
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;
		
		case DITCH_A: 
			if (_state == On) {
				if (_crossingActive) {
					if (currentMillis - _previousMillis > 1400U - (_flashRate << 2)) {
						if (_phase == A) {
							analogWrite(_pin, 255 - _brightValue);
							_phase = B;
						}
						else {
							analogWrite(_pin, 255 - _dimValue);
							_phase = A;
						}
						_previousMillis = currentMillis;
					}
					if (currentMillis - _crossingTimer > 15000) {
						_crossingActive = false;
					}
				}
				else {
					analogWrite(_pin, 255 - _brightValue);
				}
			}
			else {
				analogWrite(_pin, 255);
			}
		
		case DITCH_B: 
			if (_state == On) {
				if (_crossingActive) {
					if (currentMillis - _previousMillis > 1400U - (_flashRate << 2)) {
						if (_phase == B) {
							analogWrite(_pin, 255 - _brightValue);
							_phase = A;
						}
						else {
							analogWrite(_pin, 255 - _dimValue);
							_phase = B;
						}
						_previousMillis = currentMillis;
					}
					if (currentMillis - _crossingTimer > 15000) {
						_crossingActive = false;
					}
				}
				else {
					analogWrite(_pin, 255 - _brightValue);
				}
			}
			else {
				analogWrite(_pin, 255);
			}
		break;		
	}
	if (_fadeOn) {
		if (currentMillis - _fadeTimer >= _fadeTime) {
			if (_fade < _brightValue) {
				_fade += STEP;    									// increase fade by step					
			}
			if (_fade >= _brightValue) {
				_fade = _brightValue;      							// keep fade in bounds
				_fadeOn = false;
			}
			analogWrite(_pin, 255 - _fade);
			_fadeTimer = currentMillis;
		}
	}
	if (_fadeOff) {
		if (currentMillis - _fadeTimer >= _fadeTime) {
			if (_fade > 0) {
				_fade -= STEP;     									// decrease fade by step
			}
			if (_fade <= 0) {
				_fade = 0;                        					// keep fade in bounds
				_fadeOff = false;
			}
			analogWrite(_pin, 255 - _fade);
			_fadeTimer = currentMillis;
		}				
	}
}