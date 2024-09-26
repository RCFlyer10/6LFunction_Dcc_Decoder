
// NMRA Dcc Multifunction Lighting Decoder
//#include <EEPROM.h>
#include "Output_Led.h"
#include <NmraDcc.h>

#define VERSION 10 // 1.0

// This section defines the Arduino UNO Pins to use
#ifdef __AVR_ATmega328P__

// Define the Arduino input Pin number for the DCC Signal
#define DCC_PIN 2
#define ACK_PIN 8

#define FUNCTION_GROUPS 5
#define OUTPUTS 6
#define FEATURES 4
#define UNUSED_PINS 23 - 2 - OUTPUTS 


#else
#error "Unsupported CPU, you need to add another configuration section for your CPU"
#endif

// Comment this line out after first initia
//#define InitalizeCVs


// cache function map

uint8_t functionMap[FUNCTION_GROUPS][OUTPUTS];

// cache last DCC function stateb y function group
uint8_t lastDCCFuncState[FUNCTION_GROUPS];

// cache current function state
uint8_t outputState[OUTPUTS];

// cache consist direction
bool consistFwd = true;

// cache direction enable
uint8_t fwdDirEnable = 0;
uint8_t revDirEnable = 0;

// cache cv29, address and direction
uint8_t cv29Config = 0;
uint16_t myAddress = 0;
DCC_DIRECTION direction = DCC_DIR_FWD;

// cache Speed
uint8_t speed = 0;

// cache Packet Time-out and save config
uint16_t packetTimeOut;
bool outputStateSave;

// function list
Output_Led* outputList[OUTPUTS];

// Structure for CV Values Table
struct CVPair {
	uint16_t CV;
	uint8_t Value;
};

unsigned long saveStateTimer = 0;

// CV Addresses we will be using
#define CV_CONSIST_ADDR 19

#define CV_PROD_ID_1 47
#define CV_PROD_ID_2 48
#define CV_PROD_ID_3 49
#define CV_PROD_ID_4 50

#define CV_FWD_DIR_EN 51
#define CV_REV_DIR_EN 52

#define FX1_SAVE_STATE 112
#define FX2_SAVE_STATE 113
#define FX3_SAVE_STATE 114
#define FX4_SAVE_STATE 115
#define FX5_SAVE_STATE 116
#define FX6_SAVE_STATE 117
#define CV_FX_SAVE_CONFIG 119

// configuration CVs
#define CV_FX1_EFFECT 120
#define CV_FX2_EFFECT 121
#define CV_FX3_EFFECT 122
#define CV_FX4_EFFECT 123
#define CV_FX5_EFFECT 124
#define CV_FX6_EFFECT 125
#define CV_FX1_CONFIG_1 126
#define CV_FX2_CONFIG_1 127
#define CV_FX3_CONFIG_1 128
#define CV_FX4_CONFIG_1 129
#define CV_FX5_CONFIG_1 130
#define CV_FX6_CONFIG_1 131
#define CV_FX1_CONFIG_2 132
#define CV_FX2_CONFIG_2 133
#define CV_FX3_CONFIG_2 134
#define CV_FX4_CONFIG_2 135
#define CV_FX5_CONFIG_2 136
#define CV_FX6_CONFIG_2 137
#define CV_FX1_PROBABILITY 138
#define CV_FX2_PROBABILITY 139
#define CV_FX3_PROBABILITY 140
#define CV_FX4_PROBABILITY 141
#define CV_FX5_PROBABILITY 142
#define CV_FX6_PROBABILITY 143


// function map CVs
#define CV_FN_MAP_FX1_F0_F4 200
#define CV_FN_MAP_FX1_F5_F8 201
#define CV_FN_MAP_FX1_F9_F12 202 
#define CV_FN_MAP_FX1_F13_F20 203
#define CV_FN_MAP_FX1_F21_F28 204
#define CV_FN_MAP_FX2_F0_F4 205
#define CV_FN_MAP_FX2_F5_F8 206
#define CV_FN_MAP_FX2_F9_F12 207
#define CV_FN_MAP_FX2_F13_F20 208
#define CV_FN_MAP_FX2_F21_F28 209
#define CV_FN_MAP_FX3_F0_F4 210
#define CV_FN_MAP_FX3_F5_F8 211
#define CV_FN_MAP_FX3_F9_F12 212
#define CV_FN_MAP_FX3_F13_F20 213
#define CV_FN_MAP_FX3_F21_F28 214
#define CV_FN_MAP_FX4_F0_F4 215
#define CV_FN_MAP_FX4_F5_F8 216
#define CV_FN_MAP_FX4_F9_F12 217
#define CV_FN_MAP_FX4_F13_F20 218
#define CV_FN_MAP_FX4_F21_F28 219
#define CV_FN_MAP_FX5_F0_F4 220
#define CV_FN_MAP_FX5_F5_F8 221
#define CV_FN_MAP_FX5_F9_F12 222
#define CV_FN_MAP_FX5_F13_F20 223
#define CV_FN_MAP_FX5_F21_F28 224
#define CV_FN_MAP_FX6_F0_F4 225
#define CV_FN_MAP_FX6_F5_F8 226
#define CV_FN_MAP_FX6_F9_F12 227
#define CV_FN_MAP_FX6_F13_F20 228
#define CV_FN_MAP_FX6_F21_F28 229



#define CONFIG_END CV_FX1_EFFECT + (FEATURES * OUTPUTS) - 1

// Default CV Values Table
CVPair FactoryDefaultCVs[] = {
	// The CV Below defines the Short DCC Address
	{ CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_MULTIFUNCTION_DECODER_ADDRESS },

	{ FX1_SAVE_STATE, 0 },
	{ FX2_SAVE_STATE, 0 },
	{ FX3_SAVE_STATE, 0 },
	{ FX4_SAVE_STATE, 0 },
	{ FX5_SAVE_STATE, 0 },
	{ FX6_SAVE_STATE, 0 },
	{ CV_FX_SAVE_CONFIG, 60 },	

	// The CVs below define the lighting configuration
	{ CV_FX1_EFFECT, 0 },
	{ CV_FX2_EFFECT, 0 },
	{ CV_FX3_EFFECT, 0 },
	{ CV_FX4_EFFECT, 0 },
	{ CV_FX5_EFFECT, 0 },
	{ CV_FX6_EFFECT, 0 },
	{ CV_FX1_CONFIG_1, 7 },
	{ CV_FX2_CONFIG_1, 7 },
	{ CV_FX3_CONFIG_1, 7 },
	{ CV_FX4_CONFIG_1, 7 },
	{ CV_FX5_CONFIG_1, 7 },
	{ CV_FX6_CONFIG_1, 7 },
	{ CV_FX1_CONFIG_2, 112 },
	{ CV_FX2_CONFIG_2, 112 },
	{ CV_FX3_CONFIG_2, 112 },
	{ CV_FX4_CONFIG_2, 112 },
	{ CV_FX5_CONFIG_2, 112 },
	{ CV_FX6_CONFIG_2, 112 },
	{ CV_FX1_PROBABILITY, 50 },
	{ CV_FX2_PROBABILITY, 50 },
	{ CV_FX3_PROBABILITY, 50 },
	{ CV_FX4_PROBABILITY, 50 },
	{ CV_FX5_PROBABILITY, 50 },
	{ CV_FX6_PROBABILITY, 50 },

	// The CVs Below define the Function Map
	{ CV_FN_MAP_FX1_F0_F4, 16 },
	{ CV_FN_MAP_FX1_F5_F8, 0 },
	{ CV_FN_MAP_FX1_F9_F12, 2 },
	{ CV_FN_MAP_FX1_F13_F20, 0 },
	{ CV_FN_MAP_FX1_F21_F28, 0 },
	{ CV_FN_MAP_FX2_F0_F4, 1 },
	{ CV_FN_MAP_FX2_F5_F8, 0 },
	{ CV_FN_MAP_FX2_F9_F12, 2 },
	{ CV_FN_MAP_FX2_F13_F20, 0 },
	{ CV_FN_MAP_FX2_F21_F28, 0 },
	{ CV_FN_MAP_FX3_F0_F4, 4 },
	{ CV_FN_MAP_FX3_F5_F8, 0 },
	{ CV_FN_MAP_FX3_F9_F12, 2 },
	{ CV_FN_MAP_FX3_F13_F20, 0 },
	{ CV_FN_MAP_FX3_F21_F28, 0 },
	{ CV_FN_MAP_FX4_F0_F4, 8 },
	{ CV_FN_MAP_FX4_F5_F8, 0 },
	{ CV_FN_MAP_FX4_F9_F12, 2 },
	{ CV_FN_MAP_FX4_F13_F20, 0 },
	{ CV_FN_MAP_FX4_F21_F28, 0 },
	{ CV_FN_MAP_FX5_F0_F4, 0},
	{ CV_FN_MAP_FX5_F5_F8, 1},
	{ CV_FN_MAP_FX5_F9_F12, 2},
	{ CV_FN_MAP_FX5_F13_F20, 0},
	{ CV_FN_MAP_FX5_F21_F28, 0},
	{ CV_FN_MAP_FX6_F0_F4, 0},
	{ CV_FN_MAP_FX6_F5_F8, 2},
	{ CV_FN_MAP_FX6_F9_F12, 2},
	{ CV_FN_MAP_FX6_F13_F20, 0},
	{ CV_FN_MAP_FX6_F21_F28, 0},	

	{ CV_PROD_ID_1, 0},
	{ CV_PROD_ID_2, 24},
	{ CV_PROD_ID_3, 10},
	{ CV_PROD_ID_4, 20},	

	// The CV Below defines Advanced Consist Address
	{ CV_CONSIST_ADDR, 0 },	

	// The CV Below defines Forward Direction Enable
	{ CV_FWD_DIR_EN, 255 },

	// The CV Below defines Reverse Direction Enable
	{ CV_REV_DIR_EN, 255 },

	// These two CVs define the Long DCC Address
	{ CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0 },
	{ CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0 },

	// ONLY uncomment 1 CV_29_CONFIG line below as approprate
	//  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
	{ CV_29_CONFIG, CV29_F0_LOCATION },  // Short Address 28/128 Speed Steps
	//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps
};

// instansiate the DCC library
NmraDcc Dcc;

uint8_t FactoryDefaultCVIndex = 0;

// This call-back function is called when a CV Value changes so we can update CVs we're using
void notifyCVChange(uint16_t CV, uint8_t Value) {
	switch (CV) 
	{	
		case CV_FX_SAVE_CONFIG:
			setSaveConfig(Value);
			break;
		case CV_CONSIST_ADDR:
		case CV_MULTIFUNCTION_PRIMARY_ADDRESS:
		case CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB:
		case CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB:
			setAddress();
			break;	
		case CV_29_CONFIG:
			cv29Config = Value;
			break;
		case CV_FWD_DIR_EN:
			fwdDirEnable = Value;
			break;
		case CV_REV_DIR_EN:
			revDirEnable = Value;
			break;
		case CV_FX1_EFFECT ... CONFIG_END:
			updateOutputs(CV, Value);
			break;
		case CV_FN_MAP_FX1_F0_F4 ... CV_FN_MAP_FX1_F0_F4 + (FUNCTION_GROUPS * OUTPUTS) - 1:
			updateFunctionMap(CV, Value);
			break;
	}
}

void setSaveConfig(uint8_t Value) {	
	outputStateSave = (Value & 0x80) ? true : false;	
	packetTimeOut = Value & 0x7f;
	if (!outputStateSave) {
		for (uint8_t i = 0; i < OUTPUTS; i++) {
			Dcc.setCV(FX1_SAVE_STATE + i, 0);			
		}
	}
}

void updateOutputs(uint16_t CV, uint8_t Value) {
	int index = CV % OUTPUTS;
	Output_Led* output = outputList[index];
	switch (CV) 
	{
		case CV_FX1_EFFECT ... (CV_FX1_EFFECT + OUTPUTS) - 1:
			output->setEffect(Value);
			break;
		case CV_FX1_CONFIG_1 ... (CV_FX1_CONFIG_1 + OUTPUTS) - 1:
			output->setConfig_1(Value);
			break;
		case CV_FX1_CONFIG_2 ... (CV_FX1_CONFIG_2 + OUTPUTS) - 1:
			output->setConfig_2(Value);
			break;
		case CV_FX1_PROBABILITY ... (CV_FX1_PROBABILITY + OUTPUTS) - 1:
			output->setProbability(Value);
			break;
	}
}

void updateFunctionMap(uint16_t CV, uint8_t Value) {
	uint8_t x;
	uint8_t y = CV % FUNCTION_GROUPS;
	switch (CV) 
	{
		case CV_FN_MAP_FX1_F0_F4 ... CV_FN_MAP_FX1_F21_F28:
			x = 0;
			break;
		case CV_FN_MAP_FX2_F0_F4 ... CV_FN_MAP_FX2_F21_F28:
			x = 1;
			break;
		case CV_FN_MAP_FX3_F0_F4 ... CV_FN_MAP_FX3_F21_F28:
			x = 2;
			break;
		case CV_FN_MAP_FX4_F0_F4 ... CV_FN_MAP_FX4_F21_F28:
			x = 3;
			break;
		case CV_FN_MAP_FX5_F0_F4 ... CV_FN_MAP_FX5_F21_F28:
			x = 4;
			break;
		default:
			x = 5;
	}
	functionMap[y][x] = Value;
}

void setAddress() {
	uint8_t addr = Dcc.getCV(CV_CONSIST_ADDR);
	if (addr > 0) {
		myAddress = addr & 0x7f;
		consistFwd = (addr & 0x80) ? false : true;		
	}
	else {
		myAddress = Dcc.getAddr();		
	}
}


void configureUnusedPins() {
	uint8_t unusedPins[] = { 0, 1, 4, 7, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
	for (uint8_t i = 0; i < UNUSED_PINS; i++) {
		pinMode(unusedPins[i], INPUT_PULLUP);
	}
}

void createOutputs() {
	uint8_t pins[] = { 3, 5, 6 , 11 , 10, 9 };
	for (uint8_t i = 0; i < OUTPUTS; i++) {
		outputList[i] = new Output_Led(pins[i]);
	}
}

void notifyCVResetFactoryDefault() {
	// Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset
	// to flag to the loop() function that a reset to Factory Defaults needs to be done
	FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
}

// This call-back function is called whenever we receive a DCC Speed packet for our address
void notifyDccSpeed(uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps) {
	if (Addr == myAddress) {
		if (consistFwd) {
			direction = ((cv29Config & CV29_LOCO_DIR) ^ Dir) ? DCC_DIR_FWD : DCC_DIR_REV;
		}
		else {
			direction = ((cv29Config & CV29_LOCO_DIR) ^ Dir) ? DCC_DIR_REV: DCC_DIR_FWD;
		}
		speed = Speed;
	}
}

// This call-back function is called whenever we receive a DCC Function packet for our address
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState) {
	bool state = false;
	uint8_t functionBits;	

	if (Addr == myAddress) {		
		for (uint8_t i = 0; i < OUTPUTS; i++) {
			functionBits = functionMap[FuncGrp - 1][i];
			if (functionBits) {				
				if ((FuncState ^ lastDCCFuncState[FuncGrp - 1]) & functionBits) {
					if (direction == DCC_DIR_FWD) {
						state = (FuncState & functionBits) && (fwdDirEnable & 1 << i) ? true : false;
					}
					else {
						state = (FuncState & functionBits) && (revDirEnable & 1 << i) ? true : false;
					}

					if (state) {
						outputState[i] |= 1 << FuncGrp - 1;
					}
					else {
						outputState[i] &= ~(1 << FuncGrp - 1);
					}

					outputList[i]->setState((bool)outputState[i]);					
				}				
			}

			if (FuncGrp == FN_0_4 && (FuncState & FN_BIT_02)) {
				outputList[i]->activateCrossing();
			}
		}

		lastDCCFuncState[FuncGrp - 1] = FuncState;
	}
}

// This call-back function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
// So we will just turn the motor on for 8ms and then turn it off again.

void notifyCVAck(void) {
	digitalWrite(ACK_PIN, HIGH);
	delay(8);
	digitalWrite(ACK_PIN, LOW);
}

void setup() {
	configureUnusedPins();

	// Setup the Pin for the ACK  
	pinMode(ACK_PIN, OUTPUT);

	// Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
	// Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
	// Interrupt Number for the Arduino Pin number, which reduces confusion.

#ifdef digitalPinToInterrupt
	Dcc.pin(DCC_PIN, true);
#else
	Dcc.pin(0, DCC_PIN, true);
#endif

	Dcc.init(MAN_ID_DIY, VERSION, FLAGS_AUTO_FACTORY_DEFAULT, 0);

	createOutputs();

	setAddress();

	// Read the current CV values into cache 	
	fwdDirEnable = Dcc.getCV(CV_FWD_DIR_EN);
	revDirEnable = Dcc.getCV(CV_REV_DIR_EN);
	cv29Config = Dcc.getCV(CV_29_CONFIG);

	// cache Function Map
	uint16_t index = CV_FN_MAP_FX1_F0_F4;
	uint8_t functionBits;
	for (int x = 0; x < OUTPUTS; x++) {
		for (int y = 0; y < FUNCTION_GROUPS; y++) {
			functionBits = Dcc.getCV(index);
			if (functionBits > 0) {
				functionMap[y][x] = functionBits;
			}
			index++;
		}
	}

	// load packet time-out and save config
	uint8_t temp = Dcc.getCV(CV_FX_SAVE_CONFIG);
	packetTimeOut = (temp & 0x7f) * 1000;
	outputStateSave = temp & 0x80;

	// load output settings
	for (int index = CV_FX1_EFFECT; index < CONFIG_END; index++) {
		updateOutputs(index, Dcc.getCV(index));
	}

	// load output states
	for (uint8_t i = 0; i < OUTPUTS; i++) {
		outputList[i]->setState(Dcc.getCV(FX1_SAVE_STATE + i));
	}

#ifdef InitalizeCVs
	notifyCVResetFactoryDefault();
	#warning "Initializing CVs, please configure to not do so next upload!!!!!!!!!!!!!!"
#endif  
}

void loop() {
	// You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
	if (Dcc.process()) {
		// DCC active so reset saveStateTimer
		saveStateTimer = millis();
	}

	for (int i = 0; i < OUTPUTS; i++) {
		outputList[i]->heartbeat();
	}

	// Handle resetting CVs back to Factory Defaults
	if (FactoryDefaultCVIndex && Dcc.isSetCVReady()) {
		FactoryDefaultCVIndex--;  // Decrement first as initially it is the size of the array
		Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
	}

	// If power lost, save function states
	if (millis() - saveStateTimer >= packetTimeOut * 1000) {
		if (outputStateSave) {
			for (uint8_t i = 0; i < OUTPUTS; i++) {
				Dcc.setCV(FX1_SAVE_STATE + i, outputState[i]);
			}
		}
	}
}
