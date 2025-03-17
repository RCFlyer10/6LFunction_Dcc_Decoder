
// NMRA Dcc Multifunction Lighting Decoder


#include <NmraDcc.h>
#include "Output_Led.h"

#define VERSION 102 // 1.0.2

// This section defines the AVR Pins to use
#if defined __AVR_ATmega328P__ |__AVR_ATmega168P__

// Define the Arduino input Pin number for the DCC Signal
#define DCC_PIN 2
#define ACK_PIN 8

constexpr auto FUNCTION_GROUPS = FN_LAST - 1;
constexpr auto OUTPUTS = 6;
constexpr auto FEATURES = 7;
constexpr auto UNUSED_PINS = 23 - 2 - OUTPUTS;

#else
#error "Unsupported CPU, you need to add another configuration section for your CPU"
#endif

// Comment this line out after first initialization
//#define InitalizeCVs

// cache function map
uint8_t functionMap[OUTPUTS][FUNCTION_GROUPS];

// cache last DCC function state by function group
uint8_t lastDCCFunc[FUNCTION_GROUPS];
uint8_t lastConsistFunc[FUNCTION_GROUPS];

// cache current output state
uint8_t outputStates[OUTPUTS];

// cache consist data
bool inConsist = false;
uint8_t consistAddress;
uint8_t consistF1F8;
uint8_t consistFLF9F12;
uint8_t consistF13F20;
uint8_t consistF21F28;
DCC_DIRECTION consistDirection = DCC_DIR_FWD;

// cache direction enable
uint8_t fwdDirEnable = 0;
uint8_t revDirEnable = 0;

// cache cv29, address and direction
uint8_t cv29Config = 0;
uint16_t myAddress = 0;
DCC_DIRECTION myDirection = DCC_DIR_FWD;

// cache Speed
uint8_t mySpeed = 0;

// cache Packet Time-out and output state save config
uint16_t packetTimeOut;
bool outputStateSave;

// function list
Output_Led* outputList[OUTPUTS];

// Structure for CV Values Table
struct CVPair {
	uint16_t CV;
	uint8_t Value;
};

//float beaconStep;
//float marsStep;

unsigned long saveStateTimer = 0;

// CV Addresses we will be using
#define CV_CONSIST_ADDR 19

#define CV_CONSIST_F1_F8 21
#define CV_CONSIST_FL_F9_F12 22

#define CV_ERROR 30

#define CV_PROD_ID_1 47
#define CV_PROD_ID_2 48
#define CV_PROD_ID_3 49
#define CV_PROD_ID_4 50

#define CV_FWD_DIR_EN 51
#define CV_REV_DIR_EN 52

#define CV_CONSIST_F13_F20 53
#define CV_CONSIST_F21_F28 54

#define F1_SAVE_STATE 112
#define F2_SAVE_STATE 113
#define F3_SAVE_STATE 114
#define F4_SAVE_STATE 115
#define F5_SAVE_STATE 116
#define F6_SAVE_STATE 117
#define CV_F_SAVE_CONFIG 119

// configuration CVs
#define CV_F1_EFFECT 120
#define CV_F2_EFFECT 121
#define CV_F3_EFFECT 122
#define CV_F4_EFFECT 123
#define CV_F5_EFFECT 124
#define CV_F6_EFFECT 125
#define CV_F1_CONFIG_1 126
#define CV_F2_CONFIG_1 127
#define CV_F3_CONFIG_1 128
#define CV_F4_CONFIG_1 129
#define CV_F5_CONFIG_1 130
#define CV_F6_CONFIG_1 131
#define CV_F1_CONFIG_2 132
#define CV_F2_CONFIG_2 133
#define CV_F3_CONFIG_2 134
#define CV_F4_CONFIG_2 135
#define CV_F5_CONFIG_2 136
#define CV_F6_CONFIG_2 137
#define CV_F1_PROBABILITY 138
#define CV_F2_PROBABILITY 139
#define CV_F3_PROBABILITY 140
#define CV_F4_PROBABILITY 141
#define CV_F5_PROBABILITY 142
#define CV_F6_PROBABILITY 143
#define CV_F1_SAMPLE_TIME 144
#define CV_F2_SAMPLE_TIME 145
#define CV_F3_SAMPLE_TIME 146
#define CV_F4_SAMPLE_TIME 147
#define CV_F5_SAMPLE_TIME 148
#define CV_F6_SAMPLE_TIME 149
#define CV_F1_SPEED 150
#define CV_F2_SPEED 151
#define CV_F3_SPEED 152
#define CV_F4_SPEED 153
#define CV_F5_SPEED 154
#define CV_F6_SPEED 155
#define CV_F1_HOLDOVER 156
#define CV_F2_HOLDOVER 157
#define CV_F3_HOLDOVER 158
#define CV_F4_HOLDOVER 159
#define CV_F5_HOLDOVER 160
#define CV_F6_HOLDOVER 161

// function map CVs
#define CV_FN_MAP_F1_F0_F4 200
#define CV_FN_MAP_F1_F5_F8 201
#define CV_FN_MAP_F1_F9_F12 202 
#define CV_FN_MAP_F1_F13_F20 203
#define CV_FN_MAP_F1_F21_F28 204
#define CV_FN_MAP_F2_F0_F4 205
#define CV_FN_MAP_F2_F5_F8 206
#define CV_FN_MAP_F2_F9_F12 207
#define CV_FN_MAP_F2_F13_F20 208
#define CV_FN_MAP_F2_F21_F28 209
#define CV_FN_MAP_F3_F0_F4 210
#define CV_FN_MAP_F3_F5_F8 211
#define CV_FN_MAP_F3_F9_F12 212
#define CV_FN_MAP_F3_F13_F20 213
#define CV_FN_MAP_F3_F21_F28 214
#define CV_FN_MAP_F4_F0_F4 215
#define CV_FN_MAP_F4_F5_F8 216
#define CV_FN_MAP_F4_F9_F12 217
#define CV_FN_MAP_F4_F13_F20 218
#define CV_FN_MAP_F4_F21_F28 219
#define CV_FN_MAP_F5_F0_F4 220
#define CV_FN_MAP_F5_F5_F8 221
#define CV_FN_MAP_F5_F9_F12 222
#define CV_FN_MAP_F5_F13_F20 223
#define CV_FN_MAP_F5_F21_F28 224
#define CV_FN_MAP_F6_F0_F4 225
#define CV_FN_MAP_F6_F5_F8 226
#define CV_FN_MAP_F6_F9_F12 227
#define CV_FN_MAP_F6_F13_F20 228
#define CV_FN_MAP_F6_F21_F28 229

#define CONFIG_END CV_F1_EFFECT + (FEATURES * OUTPUTS) - 1

// Default CV Values Table
CVPair FactoryDefaultCVs[] = {
	// The CV Below defines the Short DCC Address
	{ CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_MULTIFUNCTION_DECODER_ADDRESS },

	{ F1_SAVE_STATE, 0 },
	{ F2_SAVE_STATE, 0 },
	{ F3_SAVE_STATE, 0 },
	{ F4_SAVE_STATE, 0 },
	{ F5_SAVE_STATE, 0 },
	{ F6_SAVE_STATE, 0 },
	{ CV_F_SAVE_CONFIG, 60 },
	{ CV_ERROR, 0},

	// The CVs below define the lighting configuration
	{ CV_F1_EFFECT, 0 },
	{ CV_F2_EFFECT, 0 },
	{ CV_F3_EFFECT, 0 },
	{ CV_F4_EFFECT, 0 },
	{ CV_F5_EFFECT, 0 },
	{ CV_F6_EFFECT, 0 },
	{ CV_F1_CONFIG_1, 7 },
	{ CV_F2_CONFIG_1, 7 },
	{ CV_F3_CONFIG_1, 7 },
	{ CV_F4_CONFIG_1, 7 },
	{ CV_F5_CONFIG_1, 7 },
	{ CV_F6_CONFIG_1, 7 },
	{ CV_F1_CONFIG_2, 112 },
	{ CV_F2_CONFIG_2, 112 },
	{ CV_F3_CONFIG_2, 112 },
	{ CV_F4_CONFIG_2, 112 },
	{ CV_F5_CONFIG_2, 112 },
	{ CV_F6_CONFIG_2, 112 },
	{ CV_F1_PROBABILITY, 50 },
	{ CV_F2_PROBABILITY, 50 },
	{ CV_F3_PROBABILITY, 50 },
	{ CV_F4_PROBABILITY, 50 },
	{ CV_F5_PROBABILITY, 50 },
	{ CV_F6_PROBABILITY, 50 },
	{ CV_F1_SAMPLE_TIME, 10 },
	{ CV_F2_SAMPLE_TIME, 10 },
	{ CV_F3_SAMPLE_TIME, 10 },
	{ CV_F4_SAMPLE_TIME, 10 },
	{ CV_F5_SAMPLE_TIME, 10 },
	{ CV_F6_SAMPLE_TIME, 10 },
	{ CV_F1_SPEED, 0},
	{ CV_F2_SPEED, 0},
	{ CV_F3_SPEED, 0},
	{ CV_F4_SPEED, 0},
	{ CV_F5_SPEED, 0},
	{ CV_F6_SPEED, 0},
	{ CV_F1_HOLDOVER, 15},
	{ CV_F2_HOLDOVER, 15},
	{ CV_F3_HOLDOVER, 15},
	{ CV_F4_HOLDOVER, 15},
	{ CV_F5_HOLDOVER, 15},
	{ CV_F6_HOLDOVER, 15},

	// The CVs Below define the Function Map
	{ CV_FN_MAP_F1_F0_F4, 16 },
	{ CV_FN_MAP_F1_F5_F8, 0 },
	{ CV_FN_MAP_F1_F9_F12, 2 },
	{ CV_FN_MAP_F1_F13_F20, 0 },
	{ CV_FN_MAP_F1_F21_F28, 0 },
	{ CV_FN_MAP_F2_F0_F4, 1 },
	{ CV_FN_MAP_F2_F5_F8, 0 },
	{ CV_FN_MAP_F2_F9_F12, 2 },
	{ CV_FN_MAP_F2_F13_F20, 0 },
	{ CV_FN_MAP_F2_F21_F28, 0 },
	{ CV_FN_MAP_F3_F0_F4, 4 },
	{ CV_FN_MAP_F3_F5_F8, 0 },
	{ CV_FN_MAP_F3_F9_F12, 2 },
	{ CV_FN_MAP_F3_F13_F20, 0 },
	{ CV_FN_MAP_F3_F21_F28, 0 },
	{ CV_FN_MAP_F4_F0_F4, 8 },
	{ CV_FN_MAP_F4_F5_F8, 0 },
	{ CV_FN_MAP_F4_F9_F12, 2 },
	{ CV_FN_MAP_F4_F13_F20, 0 },
	{ CV_FN_MAP_F4_F21_F28, 0 },
	{ CV_FN_MAP_F5_F0_F4, 0},
	{ CV_FN_MAP_F5_F5_F8, 1},
	{ CV_FN_MAP_F5_F9_F12, 2},
	{ CV_FN_MAP_F5_F13_F20, 0},
	{ CV_FN_MAP_F5_F21_F28, 0},
	{ CV_FN_MAP_F6_F0_F4, 0},
	{ CV_FN_MAP_F6_F5_F8, 2},
	{ CV_FN_MAP_F6_F9_F12, 2},
	{ CV_FN_MAP_F6_F13_F20, 0},
	{ CV_FN_MAP_F6_F21_F28, 0},	

	{ CV_PROD_ID_1, 0},
	{ CV_PROD_ID_2, 24},
	{ CV_PROD_ID_3, 10},
	{ CV_PROD_ID_4, 22},	

	// The CVs Below defines Advanced Consist Info
	{ CV_CONSIST_ADDR, 0 },
	{ CV_CONSIST_F1_F8, 0},
	{ CV_CONSIST_FL_F9_F12, 0},
	{ CV_CONSIST_F13_F20, 0},
	{ CV_CONSIST_F21_F28, 0},

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
		case CV_F_SAVE_CONFIG:
			setSaveConfig(Value);
			break;
		case CV_CONSIST_ADDR:
		case CV_MULTIFUNCTION_PRIMARY_ADDRESS:
		case CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB:
		case CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB:
			setAddress();
			break;
		case CV_CONSIST_F1_F8:
			consistF1F8 = Value;
			break;
		case CV_CONSIST_FL_F9_F12:
			consistFLF9F12 = Value;
			break;
		case CV_CONSIST_F13_F20:
			consistF13F20 = Value;
			break;
		case CV_CONSIST_F21_F28:
			consistF21F28 = Value;
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
		case CV_F1_EFFECT ... CONFIG_END:
			updateOutputs(CV, Value);
			break;
		case CV_FN_MAP_F1_F0_F4 ... CV_FN_MAP_F1_F0_F4 + (FUNCTION_GROUPS * OUTPUTS) - 1:
			updateFunctionMap(CV, Value);
			break;
	}
}

void setSaveConfig(uint8_t Value) {	
	outputStateSave = (Value & 0x80) ? true : false;	
	packetTimeOut = Value & 0x7f;
	if (!outputStateSave) {
		for (uint8_t i = 0; i < OUTPUTS; i++) {
			Dcc.setCV(F1_SAVE_STATE + i, 0);			
		}
	}
}

void updateOutputs(uint16_t CV, uint8_t Value) {
	int index = CV % OUTPUTS;
	Output_Led* output = outputList[index];
	switch (CV) 
	{
		case CV_F1_EFFECT ... (CV_F1_EFFECT + OUTPUTS) - 1:
			output->setEffect(Value);
			break;
		case CV_F1_CONFIG_1 ... (CV_F1_CONFIG_1 + OUTPUTS) - 1:
			output->setConfig_1(Value);
			break;
		case CV_F1_CONFIG_2 ... (CV_F1_CONFIG_2 + OUTPUTS) - 1:
			output->setConfig_2(Value);
			break;
		case CV_F1_PROBABILITY ... (CV_F1_PROBABILITY + OUTPUTS) - 1:
			output->setProbability(Value);
			break;
		case CV_F1_SAMPLE_TIME ... (CV_F1_SAMPLE_TIME + OUTPUTS) - 1:
			output->setSampleTime(Value);
			break;
		case CV_F1_SPEED ... (CV_F1_SPEED + OUTPUTS) - 1:
			output->setSpeed(Value);
			break;
		case CV_F1_HOLDOVER ... (CV_F1_HOLDOVER + OUTPUTS) - 1:
			output->setHoldoverTime(Value);
			break;
	}
}

void updateFunctionMap(uint16_t CV, uint8_t Value) {
	uint8_t col = CV % FUNCTION_GROUPS;
	uint8_t row;
	switch (CV) 
	{
		case CV_FN_MAP_F1_F0_F4 ... CV_FN_MAP_F1_F21_F28:
			row = 0;
			break;
		case CV_FN_MAP_F2_F0_F4 ... CV_FN_MAP_F2_F21_F28:
			row = 1;
			break;
		case CV_FN_MAP_F3_F0_F4 ... CV_FN_MAP_F3_F21_F28:
			row = 2;
			break;
		case CV_FN_MAP_F4_F0_F4 ... CV_FN_MAP_F4_F21_F28:
			row = 3;
			break;
		case CV_FN_MAP_F5_F0_F4 ... CV_FN_MAP_F5_F21_F28:
			row = 4;
			break;
		default:
			row = 5;
	}
	functionMap[row][col] = Value;
}

void setAddress() {		
	myAddress = Dcc.getAddr();

	uint8_t address = Dcc.getCV(CV_CONSIST_ADDR);
	
	consistAddress = address & 0x7f;
	consistDirection = (address & 0x80) ? DCC_DIR_REV : DCC_DIR_FWD;
	inConsist = (consistAddress) ? true : false;	

	consistF1F8 = Dcc.getCV(CV_CONSIST_F1_F8);
	consistFLF9F12 = Dcc.getCV(CV_CONSIST_FL_F9_F12);
	consistF13F20 = Dcc.getCV(CV_CONSIST_F13_F20);
	consistF21F28 = Dcc.getCV(CV_CONSIST_F21_F28);
}


void configureUnusedPins() {
	uint8_t unusedPins[] = { 0, 1, 4, 7, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
	for (auto& pin : unusedPins) {
		pinMode(pin, INPUT_PULLUP);
	}	
}

void createOutputs() {
	uint8_t pins[] = { 3, 5, 6 , 11 , 10, 9 }; // PWM pins in proper order
	uint8_t* pin = pins;
	for (auto& output : outputList) {
		output = new Output_Led(*pin++);
	}	
}

void notifyCVResetFactoryDefault() {
	// Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset
	// to flag to the loop() function that a reset to Factory Defaults needs to be done
	FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
}

// This call-back function is called whenever we receive a DCC Speed packet
void notifyDccSpeed(uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps) {

	// speed and direction changes only by normal address when not consited or by consit address when consited
	if ((Addr == myAddress && !inConsist) || (Addr == consistAddress && inConsist)) {

		bool dirBit = (cv29Config & CV29_LOCO_DIR) ^ Dir; // direction based on CV29_LOCO_DIR and throttle Dir

		if (inConsist && consistDirection == DCC_DIR_REV) {
			myDirection = dirBit ? DCC_DIR_REV : DCC_DIR_FWD; // invert the direction
		}
		else {
			myDirection = dirBit ? DCC_DIR_FWD : DCC_DIR_REV;
		}

		// correct output states for direction
		uint8_t i = 0;
		for (auto& output : outputList) {
			if (outputStates[i]) {
				if (myDirection == DCC_DIR_FWD) {
					output->setState((fwdDirEnable & (1 << i)));
				}
				else {
					output->setState((revDirEnable & (1 << i)));
				}
			} 

			i++;
		}

		mySpeed = Speed;
	}
}


// This call-back function is called whenever we receive a DCC Function packet
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState) {
	uint8_t changedFuncs = 0;
	bool enabled = false;
	uint8_t prevState;

	if (Addr == consistAddress && inConsist) {
		prevState = lastConsistFunc[FuncGrp - 1];
		changedFuncs = prevState ^ FuncState;

		if (changedFuncs) {
			enabled = checkConsistFunctions(FuncGrp, changedFuncs);
		}
		lastConsistFunc[FuncGrp - 1] = FuncState;
	}
	else if (Addr == myAddress) {
		prevState = lastDCCFunc[FuncGrp - 1];
		changedFuncs = prevState ^ FuncState;

		if (changedFuncs) {
			enabled = true;
		}
		lastDCCFunc[FuncGrp - 1] = FuncState;
	}

	if (enabled) {
		bool state = (changedFuncs & FuncState) ? On : Off;
		updateOutputs(FuncGrp, changedFuncs, state);
	}
}

// Helper function: Checks if consist functions should enable outputs
bool checkConsistFunctions(FN_GROUP FuncGrp, uint8_t changedFuncs) {
	uint8_t mask, consistByte;
	uint8_t startBit, endBit;

	switch (FuncGrp) {
	case FN_0_4:
		return ((changedFuncs & FN_BIT_00) && ((consistFLF9F12 & 0x01 && myDirection == DCC_DIR_FWD) ||
			(consistFLF9F12 & 0x02 && myDirection == DCC_DIR_REV))) ||
			((changedFuncs & FN_BIT_01) && (consistF1F8 & 0x01)) ||
			((changedFuncs & FN_BIT_02) && (consistF1F8 & 0x02)) ||
			((changedFuncs & FN_BIT_03) && (consistF1F8 & 0x04)) ||
			((changedFuncs & FN_BIT_04) && (consistF1F8 & 0x08));

	case FN_5_8:
		mask = 0x10; // Starting from bit 4 in consistF1F8
		consistByte = consistF1F8;
		startBit = 5;
		endBit = 8;
		break;

	case FN_9_12:
		mask = 0x04; // Starting from bit 2 in consistFLF9F12
		consistByte = consistFLF9F12;
		startBit = 9;
		endBit = 12;
		break;

	case FN_13_20:
		mask = 0x01;
		consistByte = consistF13F20;
		startBit = 13;
		endBit = 20;
		break;

	case FN_21_28:
		mask = 0x01;
		consistByte = consistF21F28;
		startBit = 21;
		endBit = 28;
		break;

	default:
		return false;
	}

	for (uint8_t i = startBit; i <= endBit; i++) {
		if ((changedFuncs & (1 << i)) && (consistByte & mask)) {
			return true;
		}
		mask <<= 1;
	}
	return false;
}

// Helper function: Updates outputs based on function changes
void updateOutputs(FN_GROUP FuncGrp, uint8_t changedFuncs, bool state) {
	uint8_t i = 0;
	for (auto& output : outputList) {
		if (changedFuncs & functionMap[i][FuncGrp - 1]) {
			if (myDirection == DCC_DIR_FWD) {
				output->setState(state && (fwdDirEnable & 1 << i));
			}
			else {
				output->setState(state && (revDirEnable & 1 << i));
			}
			
			outputStates[i] = state;			
		}

		// Special case for F2 (Function 2 in FN_0_4 group)
		if (state && (FuncGrp == FN_0_4) && (changedFuncs & FN_BIT_02)) {
			output->activateCrossing();
		}

		i++;
	}
}

// This call-back function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
void notifyCVAck(void) {
	digitalWrite(ACK_PIN, HIGH);
	delay(6);
	digitalWrite(ACK_PIN, LOW);
}

void setup() {
	createOutputs();
	
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

	setAddress();

	// Read the current CV values into cache 	
	fwdDirEnable = Dcc.getCV(CV_FWD_DIR_EN);
	revDirEnable = Dcc.getCV(CV_REV_DIR_EN);
	cv29Config = Dcc.getCV(CV_29_CONFIG);

	// cache Function Map
	uint16_t index = CV_FN_MAP_F1_F0_F4;	
	for (auto& row : functionMap) {
		for (auto& col : row) {
			col = Dcc.getCV(index++);
		}
	}

	// load packet time-out and save config
	uint8_t temp = Dcc.getCV(CV_F_SAVE_CONFIG);
	packetTimeOut = (temp & 0x7f) * 1000;
	outputStateSave = temp & 0x80;

	// load output settings
	for (int index = CV_F1_EFFECT; index < CONFIG_END; index++) {
		updateOutputs(index, Dcc.getCV(index));
	}

	// load output states
	for (uint8_t i = 0; i < OUTPUTS; i++) {
		uint8_t state = Dcc.getCV(F1_SAVE_STATE + i);
		outputList[i]->setState(state);
		outputStates[i] = state;
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

	for (auto& output : outputList) {
		output->heartbeat();
	}	

	// Handle resetting CVs back to Factory Defaults
	if (FactoryDefaultCVIndex && Dcc.isSetCVReady()) {
		FactoryDefaultCVIndex--;  // Decrement first as initially it is the size of the array
		Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
	}

	// If power lost, save function states
	if (millis() - saveStateTimer >= packetTimeOut * 1000) {
		if (outputStateSave) { 
			uint8_t i = 0;
			for (auto& outputState : outputStates) {
				Dcc.setCV(F1_SAVE_STATE + i++, outputState);
			}			
		}
	}
}
