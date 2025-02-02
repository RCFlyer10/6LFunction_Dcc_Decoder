#include "Output_Led.h"


Output_Led::Output_Led(uint8_t pin) {
	_pin = pin;
	pinMode(pin, OUTPUT);
	_state = Off;
	_effect = 0;	
	_fadeIndex = 0;
	_fading = false;
	setState(Off);
	analogWrite(_pin, 255);
}

void Output_Led::setEffect(uint8_t effect) {
	_effect = effect;	
	setState(Off);
}

void Output_Led::setConfig_1(uint8_t value) {
	uint8_t temp;
	_bright = value & 0x0f;
	_brightIndex = (_bright * 16) + 15;	
	_dim = (value & 0xf0) >> 4;
	_dimIndex = (_dim * 16) + 15;
	_beaconIntensity = pgm_read_word(&beaconBrightTable[_bright]);
	_flashIntensity = pgm_read_word(&beaconFlashTable[_bright]);	
}

void Output_Led::setConfig_2(uint8_t value) {	
	_fadeRate = value & 0x0f;
	_flashRate = (value & 0xf0) >> 4;	
	_beaconPeriod = 14 - (_flashRate / 2);
	_strobePeriod = 1700 - (_flashRate * 100);
	_ditchPeriod = 1500 - (_flashRate * 78);
}

void Output_Led::setProbability(uint8_t value) {
	_probability = value;
}

void Output_Led::setSampleTime(uint8_t value) {
	_sampleTime = value * 1000;
}

void Output_Led::setSpeed(uint8_t value) {
	_setSpeed = value;
}

void Output_Led::setHoldoverTime(uint8_t value) {
	_holdOverTime = value * 1000;
}

void Output_Led::setState(bool state) {
	if (_state != state) {
		if (state == On) {
			_ledState = Off;
			_crossingActive = false;
			_previousMillis = millis();
			_fadeTimer = millis();
			_fadeIndex = 0;
			_radians = START_RADIAN;			
			_fading = true;
			_flash = false;
			_randomNumber = random(100);
		}			
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

void Output_Led::heartbeat() {
	unsigned long currentMillis = millis();
	switch (_effect) 
	{
		case NORMAL:
			if (_state == Off || _setSpeed > 0 && mySpeed > _setSpeed) {
				if (_fadeRate > 0) {
					if (_fadeIndex > 0) {
						if (currentMillis - _fadeTimer > (30 - _bright - _fadeRate)) {
							_fadeIndex--;
							_fadeTimer = currentMillis;
						}
					}
				}
				else {
					_fadeIndex = 0;
				}
			}
			else { // On or mySpeed < _setSpeed				
				if (_fadeRate > 0) {
					if (_fadeIndex < _brightIndex) {
						if (currentMillis - _fadeTimer > (30 - _bright - _fadeRate)) {
							_fadeIndex++;
							_fadeTimer = currentMillis;
							_ledState = On;
						}
					}
				}
				else {
					_fadeIndex = _brightIndex;
					_ledState = On;
				}
			}			
			analogWrite(_pin, 255 - pgm_read_word(&gamma8[_fadeIndex]));
			break;		
		
		case AUTO_DIM: 
			if (_state == On) {
				if (myDirection == DCC_DIR_REV) {
					analogWrite(_pin, 255 - pgm_read_word(&gamma8[_dimIndex]));
				}
				else { 
					analogWrite(_pin, 255 - pgm_read_word(&gamma8[_brightIndex])); 
				}
			}
			else { 
				analogWrite(_pin, 255);
			}
			break;
		
		case STROBE: 
			if (_state == On) {
				if (_ledState == Off && currentMillis - _previousMillis > _strobePeriod) {
					_previousMillis = currentMillis;
					_ledState = On;
				}
				else if (_ledState == On && currentMillis - _previousMillis > S_DURATION) {
					_ledState = Off;
				}
				if (_ledState == On) {
					analogWrite(_pin, 255 - pgm_read_word(&gamma8[_brightIndex]));
				}
				else {
					analogWrite(_pin, 255);
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;

		case EOT:
			if (_state == On) {
				if (_setSpeed == 0 || (mySpeed <= _setSpeed)) {
					_ledState = On;					
				}
				else if (currentMillis - _previousMillis > _strobePeriod) {
					_ledState = !_ledState;
					_previousMillis = currentMillis;
				}
				if (_ledState == On) {
					analogWrite(_pin, 255 - pgm_read_word(&gamma8[_brightIndex]));
				}
				else {
					analogWrite(_pin, 255);
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;		
		
		case BEACON: 
			if (_state == On) {
				if (currentMillis - _previousMillis > _beaconPeriod) {
					double value = cos(_radians);
					if (value > .90) {
						analogWrite(_pin, 255 - _flashIntensity);
					}
					else {
						analogWrite(_pin, 255 - ((value * _beaconIntensity) + _beaconIntensity));
					}
					_radians += BEACON_STEP;
					if (_radians >= MAX_RADIAN) {
						_radians = START_RADIAN;
					}
					_previousMillis = currentMillis;
				}
			}			
			else { 
				analogWrite(_pin, 255); 
			}
			break;
		
		case MARS: 
			if (_state == On) {
				if (currentMillis - _previousMillis > _beaconPeriod) {
					double value = cos(_radians);
					if (value > .90 && _flash) {
						analogWrite(_pin, 255 - _flashIntensity);
					}
					else {
						analogWrite(_pin, 255 - ((value * _beaconIntensity) + _beaconIntensity));
					}
					_radians += MARS_STEP;
					if (_radians >= MAX_RADIAN) {
						_radians = START_RADIAN;						
						_flash = !_flash;
					}					
					_previousMillis = currentMillis;
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;			
		
		case FLICKER: 
			if (_state == On) {

				if (currentMillis - _previousMillis > 120U - random(_flashRate << 3)) {
					_previousMillis = currentMillis;
					uint8_t temp = random(pgm_read_word(&gamma8[_brightIndex]));
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
					if (currentMillis - _previousMillis > _ditchPeriod) {
						if (_phase == A) {
							analogWrite(_pin, 255 - pgm_read_word(&gamma8[_brightIndex]));
							_phase = B;
						}
						else {
							analogWrite(_pin, 255 - pgm_read_word(&gamma8[_dimIndex]));
							_phase = A;
						}
						_previousMillis = currentMillis;
					}
					if (currentMillis - _crossingTimer > _holdOverTime) {
						_crossingActive = false;
					}
				}
				else {
					analogWrite(_pin, 255 - pgm_read_word(&gamma8[_brightIndex]));
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;
		
		case DITCH_B: 
			if (_state == On) {
				if (_crossingActive) {
					if (currentMillis - _previousMillis > _ditchPeriod) {
						if (_phase == B) {
							analogWrite(_pin, 255 - pgm_read_word(&gamma8[_brightIndex]));
							_phase = A;
						}
						else {
							analogWrite(_pin, 255 - pgm_read_word(&gamma8[_dimIndex]));
							_phase = B;
						}
						_previousMillis = currentMillis;
					}
					if (currentMillis - _crossingTimer > _holdOverTime) {
						_crossingActive = false;
					}
				}
				else {
					analogWrite(_pin, 255 - pgm_read_word(&gamma8[_brightIndex]));
				}
			}
			else {
				analogWrite(_pin, 255);
			}
			break;	

		case RANDOM:
			if (_state == On) {
				if (_setSpeed == 0 || mySpeed < _setSpeed || myDirection == DCC_DIR_REV) {
					if (_fadeIndex < _brightIndex) {
						if (_fadeRate > 0) {
							if (currentMillis - _fadeTimer > (30 - _bright - _fadeRate)) {
								_fadeIndex++;
								_fadeTimer = currentMillis;
							}
						}
						else {
							_fadeIndex = _brightIndex;
						}						
					}
					if (_fadeIndex > 0) {
						_ledState = On;
					}
					_fading = false;
				}				
				else if (_fadeIndex > _dimIndex) {
					if (_fadeRate > 0) {
						if (currentMillis - _fadeTimer > (30 - _dim - _fadeRate)) {
							_fadeIndex--;
							_fadeTimer = currentMillis;
						}
					}
					else {
						_fadeIndex = _dimIndex;
					}					
				}
				else if (currentMillis - _previousMillis > _sampleTime) {
					if (_probability > _randomNumber) {
						if (_fadeRate == 0) {
							if (_ledState == On) {
								_fadeIndex = 0;
							}
							else {
								_fadeIndex = _dimIndex;
							}
						}
						else {
							_fading = true;
						}
					}						
					_randomNumber = random(100);
					_previousMillis = currentMillis;
				}					
				if (_fading) {
					if (_ledState == On) {
						if (_fadeIndex > 0) {
							if (currentMillis - _fadeTimer > (30 - _bright - _fadeRate)) {
								_fadeIndex--;
								_fadeTimer = currentMillis;
							}
						}
						if (_fadeIndex == 0) {
							_ledState = Off;
							_fading = false;
						}
					}
					else {
						if (_fadeIndex < _dimIndex) {
							if (currentMillis - _fadeTimer > (30 - _bright - _fadeRate)) {
								_fadeIndex++;
								_fadeTimer = currentMillis;
							}
						}
						if (_fadeIndex == _dimIndex) {
							_ledState = On;
							_fading = false;
						}
					}
				}				
			}
			else if (_fadeIndex > 0) {
				if (_fadeRate > 0) {
					if (currentMillis - _fadeTimer > (30 - _bright - _fadeRate)) {
						_fadeIndex--;
						_fadeTimer = currentMillis;
					}
				}
				else {
					_fadeIndex = 0;
				}
				if (_fadeIndex == 0) {
					_ledState = Off;
				}
			}			
			analogWrite(_pin, 255 - pgm_read_word(&gamma8[_fadeIndex]));
			break;
	}	
}