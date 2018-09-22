//MASTER Brain
#include <RoomTwo.h>
#include <FastLED.h>
#include <TimerOne.h>
#include <avr/wdt.h>
#include <BufferedInputs.h>

unsigned long timer_status_led = 0;
#define status_led_pin 13
// Pins					
#define SENZOR_LASER_1 12
#define SENZOR_LASER_2 12		 // PROBLEM, isti portovi																																																																																																									
#define pinLasPOWER 7
#define pinLaserTrig 5

#define BIG_RED_LED 6 //PWM pin for BIG_RED_LED -- 2 Benzi Rosii - Alb Maro

#define LIFT_UP A1
#define LIFT_DOWN A0
#define VRATA_CLOSE A2
#define VRATA_OPEN A3

// SHIFT_IN
#define data_pin	11	// SER_OUT (serial data out)
#define shld_pin	10	//	SH/!LD (shift or active low load)
#define clk_pin	 9 // CLK (the clock that times the shifting)
#define ce_pin		8 // !CE (clock enable, active low)

// SHIFT_OUT
#define latchPin 2 // ST_CP of 74HC595
#define clockPin 3 // SH_CP of 74HC595
#define dataPin	4 // DS of 74HC595

// InputBuffer[0]
#define BCD_PIN_01 0 //PULL_UP
#define BCD_PIN_02 0 //PULL_UP
#define BCD_PIN_03 0 //PULL_UP
#define BCD_PIN_04 0 //PULL_UP
#define RESTART_BUTTON 1 //PULL_UP
#define START_BUTTON 2 //PULL_UP

#define FOUR_CARDS_DONE 4 //PULL_UP
#define pinMOTORNA_VRATA_HOME 5
#define pinNORMALNA_VRATA_ZATVORENA 6

// InputBuffer[1]
#define pinMOTORNA_VRATA_PRIMA 0
#define pinLIFT_PRIMA 1
#define pinLIFT_HOME 2
#define pinLIFT_EXT 3
#define BIG_RED_PRESSED 4 //PULL_UP - Wire 2 reds - ALb Albastru
#define pinVRATASKY 5
#define ROBOT_JOB_DONE 7 //PULL_UP

#define revs_lift_full 14
#define revs_vrata_full 20

#define PULSE_WIDTH_USEC	 5 // Width of pulse to trigger the shift register to read and latch.
uint8_t InputBuffer[] = { 0XFF, 0XFF }; // 16 Output ports

// Output RELAYs
#define RED_LED_LEFT_LEDS 		4
#define RED_LED_RIGHT_LEDS 		5
#define SMALLROOM_LIGHT 		6
#define SPOT_LIGHT 				7
#define MAIN_DOOR_LOCK 			8	//12V - Nr 6 Portocaliu/Verde + AlbP/AlbV
#define ROBOT_POWER 			9 // +/-5V - NR 3 - Portocaliu +5V/ Maro -5V / Alb GND (in releu) //Albastru senzor la masa
#define SECOND_DOOR_LOCK 		10 //12V - Banda Galbena - Portocaliu/Verde + AlbP/AlbV
#define BUTTONNO		 		12 // Dugmici na vratima za arrows

#define GREEN_LEDS 				13
#define LASER_1_POWER 			14	//3.3V - Nr 4 - Alb+Portocaliu
#define LASER_2_POWER 			15 //3.3V - Nr 4 - Alb+Albastru

// Inputs stored in InputBuffer[0]
#define HELD_TIME 3000
Input startButton = Input(&InputBuffer[0], START_BUTTON, BUTTON_PULLUP);

Input resetButton = Input(&InputBuffer[0], RESTART_BUTTON, BUTTON_PULLUP);
Input motorVrataHome = Input(&InputBuffer[0], pinMOTORNA_VRATA_HOME, BUTTON_PULLUP);

Input fourCardsDone = Input(&InputBuffer[0], FOUR_CARDS_DONE, BUTTON_PULLUP);
Input secondDoorClosed = Input(&InputBuffer[0], pinNORMALNA_VRATA_ZATVORENA, BUTTON_PULLUP);
Input circularPuzzleDone = Input(&InputBuffer[0], 3, BUTTON_PULLUP);

// Inputs stored in InputBuffer[1]
Input robotJobDone = Input(&InputBuffer[1], ROBOT_JOB_DONE, BUTTON_PULLUP);
Input motorLiftHome = Input(&InputBuffer[1], pinLIFT_HOME, BUTTON_PULLUP);
Input motorVrataPing = Input(&InputBuffer[1], pinMOTORNA_VRATA_PRIMA, BUTTON_PULLUP);
Input motorLiftPing = Input(&InputBuffer[1], pinLIFT_PRIMA, BUTTON_PULLUP);
Input motorLiftExt = Input(&InputBuffer[1], pinLIFT_EXT, BUTTON_PULLUP);
Input bigRedButton = Input(&InputBuffer[1], BIG_RED_PRESSED, BUTTON_PULLUP);
Input vratasky = Input(&InputBuffer[1], pinVRATASKY, BUTTON_PULLUP);

//Inputs Activated
bool circularPuzzleActive = false;
bool fourCardActive = false;
bool BigRedHit = false;
bool disksDone = false;

static uint8_t OutputBuffer[] = { 0xFF, 0xFF }; // 16 Output ports

// Relay State. Relay is ON if it has LOW on command pin
#define RELAY_ON false
#define RELAY_OFF true

//Second door state
#define DOOR_CLOSED 1
#define DOOR_OPEN 0
#define DOOR_LOCK 0
#define DOOR_UNLOCK 1

bool secondDoorState = DOOR_OPEN;
bool unstick_secondDoorState = false;

//BIG RED LED
bool fadeOUT = false;
bool fadeBIGRed = false;
uint8_t logaritmicLED[256] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 18, 18, 18, 19, 19, 20, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 36, 36, 37, 38, 39, 40, 41, 42, 42, 43, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 55, 57, 58, 59, 60, 62, 63, 64, 66, 67, 69, 70, 72, 74, 75, 77, 79, 80, 82, 84, 86, 88, 90, 91, 94, 96, 98, 100, 102, 104, 107, 109, 111, 114, 116, 119, 122, 124, 127, 130, 133, 136, 139, 142, 145, 148, 151, 155, 158, 161, 165, 169, 172, 176, 180, 184, 188, 192, 196, 201, 205, 210, 214, 219, 224, 229, 234, 239, 244, 250, 255};
uint8_t bright = 0;

//Slaves I2C
const uint8_t slavesAdresses[SLV_TOTAL]
	= {ADDR_CODES, ADDR_HANDLE, ADDR_ARROWS, ADDR_META};
uint8_t slavesStates[SLV_TOTAL];
uint8_t slavesCommands[SLV_TOTAL];
uint8_t command = NOTHING_YET;
uint8_t error;

//I2C trigger
void timerIsr();
uint8_t timer300ms = 0;
uint8_t address;
boolean activateI2C = false;

//Robot
bool robotAllDone = false;

//Lasers Smoke & Alarm
bool checkLightSensors = false;

//Sounds
bool soundNotStarted = false;

//monitor the activated codes
int codesSteps = 0;

//Handle logic
bool handleChangeState = false;
byte handleOldState = NOTHING_YET;

byte readIndex = 0;						 // the index of the current reading
long total = 0;								 // the running total
int average = 0;								// the average

const int sensorPins[] = {SENZOR_LASER_1, SENZOR_LASER_2};

bool alarmChangeState = false;
bool alarmOldState = true;

bool lockPlayer = false;
bool oneLaser = false;

uint8_t sensorOne;
uint8_t sensorTwo;
bool alarmOn = false;

//Game Logic
int GameSTEP = 1; //game just started
bool GameRESET = false;
bool goForward = false;
bool nextLevel = false;

//GAME STEPS
#define GAME_IDLE 0
#define GAME_START 1
#define GAME_CIRCULAR_PUZZLE 2
#define GAME_LIGHTS_STAGE 3
#define GAME_ROBOT_STAGE 4
#define GAME_CODES_STAGE 5
#define GAME_DISKS_STAGE 6
#define GAME_HANDLE_STAGE 7
#define GAME_BIGRED_STAGE 8
#define GAME_COLORS_STAGE 9
#define GAME_LASERS 10

//non-blocking timming
int timeout = 0; //non-blocking delay
int timerMP3 = 0;
int waitToSkip = 0;
byte reps = 0; //repetitions


int revs_lift = 14;
unsigned long timerlift = 0;
int revs_vrata = revs_vrata_full + 2;
long timervrata = 0;
bool gameInitialized = false;
bool spotlightison = false;
long timer1h = 0;


//Function Declaration and Function Prototypes
void writeOutput(uint8_t pin, bool on, bool writeNow = true);
void MasterCommands(uint8_t address, uint8_t command);
void soft_restart();

// banda albastra - DoorLocled Leds - Alb + Maro Door Unlocked - Alb Albastru / Senzor usa inchisa Alb-Portocaliu
//Conecate pe acelasi releu ca incuetoarea usii secundare.

/*
	#define STONE_DOOR_LOCK 7 //12V - Banda Rosie
	#define ROBOT_LIGHT 7 //12V;	+/-5V; GND	Secondary relays for Light and +/-5V
	#define TABLET_PLATE 7 //12V
*/

/*
	#define MAIN_LIGHT_1 7 //220V
	#define MAIN_LIGHT_2 7 //220V
	#define MAIN_LIGHT_3 7 //220V
	#define MAIN_LIGHT_4 7 //220V
	#define SMOKE_POWER 7 //220V
	#define SMOKE_PUFF 7 //220V
*/
//#define KEYPAD_BACKLIGHT_DONT_USE_THIS 11
/*	MAIN_DOOR_LOCK (relay Normal CLOSED)
	STONE_DOOR_LOCK (relay Normal CLOSED)
	SECOND_DOOR_LOCK (relay Normal OPEN)
	SECRET_BOX_LOCK (relay Normal CLOSED)
	ROBOT_POWER (relay Normal OPEN) */
//The initialization
void setup()
{
	// hic sunt leones - init soft reset code
	MCUSR = 0;
	wdt_disable();

	pinMode(status_led_pin, OUTPUT);
	digitalWrite(status_led_pin, HIGH);

	//set pins to output so you can control the shift register
	pinMode(latchPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	pinMode(dataPin, OUTPUT);
	digitalWrite(latchPin, HIGH);
	digitalWrite(clockPin, HIGH);
	digitalWrite(dataPin, HIGH);
	
	//Initialize OUTPUT
	for ( int i = 0; i < 16; i++) {
		writeOutput( i, RELAY_OFF);
	}

	for ( int i = 0; i < 16; i++) {
		writeOutput( i, RELAY_ON);
		delay(100);
		writeOutput( i, RELAY_OFF);
	}
		delay(100);
	
	for ( int i = 0; i < 16; i++) {
		writeOutput( i, RELAY_OFF);
	}
	delay(100);
	DPRINTLN(F("All OUTPUT ports OFF"));

	delay(500); // sanity check delay - allows reprogramming if accidently blowing power w/leds
	DSERIALSTART(9600); // start serial for output
	DPRINTLN(F("Serial Data Started!"));

	//PINS setup
	// Initialize each digital pin to either output or input
	// We are commanding the shift register with each pin with the exception of the serial
	// data we get back on the data_pin line.
	pinMode(shld_pin, OUTPUT);
	pinMode(ce_pin, OUTPUT);
	pinMode(clk_pin, OUTPUT);
	pinMode(data_pin, INPUT);
	// Required initial states of these two pins according to the datasheet timing diagram
	digitalWrite(clk_pin, HIGH);
	digitalWrite(shld_pin, HIGH);

	pinMode(LIFT_UP, OUTPUT);
	digitalWrite(LIFT_UP, LOW);
	pinMode(LIFT_DOWN, OUTPUT);
	digitalWrite(LIFT_DOWN, LOW);
	pinMode(VRATA_OPEN, OUTPUT);
	digitalWrite(VRATA_OPEN, LOW);
	pinMode(VRATA_CLOSE, OUTPUT);
	digitalWrite(VRATA_CLOSE, LOW);

	pinMode(pinLaserTrig, INPUT);
	pinMode(pinLasPOWER, OUTPUT);
	digitalWrite(pinLasPOWER, LOW);
	
	pinMode(BIG_RED_LED, OUTPUT);
	analogWrite(BIG_RED_LED, 0);

	/*pinMode(RED_LED_LEFT_LEDS, OUTPUT);
		writeOutput(RED_LED_LEFT_LEDS, RELAY_OFF);
		pinMode(RED_LED_RIGHT_LEDS, OUTPUT);
		digitalWrite(RED_LED_RIGHT_LEDS, LOW);*/



	//initialize slave states to none.
	for (int i = 0; i < SLV_TOTAL; i++)
		slavesStates[i] = NOTHING_YET;

	// join i2c bus (address optional for master)
	
	Wire.setClockReg(B11111111, B01);
	Wire.begin();
	delay(1);
	// what if START pressed
	startButton.clickHandler(startButtonClickEvents);

	//Reset Game
	resetButton.pressHandler(resetButtonPressEvents);
	resetButton.releaseHandler(resetButtonReleaseEvents);

	circularPuzzleDone.holdHandler(circularPuzzleHoldEvents, 100);

	fourCardsDone.holdHandler(fourCardsHoldEvents, 1000);

	//if Robot Done
	robotJobDone.holdHandler(robotDoneHoldEvents, 1000);

	//if Big Red pressed
	bigRedButton.pressHandler(bigRedButtonPressEvents);

	vratasky.pressHandler(vrataskyHOLD);

	//Secondary Door Closed/Open
	secondDoorClosed.pressHandler(secondDoorPressEvents);
	secondDoorClosed.releaseHandler(secondDoorReleaseEvents);

	motorVrataHome.holdHandler(motorVrataHomeHOLD, 300);
	motorVrataHome.pressHandler(motorVrataHomePRESS);

	motorLiftHome.pressHandler(motorLiftHomePRESS);

	motorVrataPing.releaseHandler(motorVrataPingRELEASE);
	motorLiftPing.releaseHandler(motorLiftPingRELEASE);
	//motorLiftExt.releaseHandler(motorLiftExtRELEASE);
	DPRINTLN(F("Buttons initialised!"));

	//Timer start
	Timer1.initialize(15000);
	Timer1.attachInterrupt( timerIsr );

	DPRINTLN(F("Timer one Up and Running!"));


	error = 0;
	for ( int i = 0; i < SLV_TOTAL; i++) {
		address = slavesAdresses[i];
		Wire.beginTransmission(address);
		delay(1);
		error = Wire.endTransmission();

#ifdef DEBUG
		if (error) {
			DPRINT(F("Errors on I2C address: "));
			DPRINT(address, DEC);
			DPRINT(F(" error "));
			DPRINTLN(error, DEC);
		}
		else {
			DPRINT(F("I2C on address "));
			DPRINT(address, DEC);
			DPRINTLN(F(" UP and RUNning"));
		}
#endif
	}
	delay(200);	
	digitalWrite(status_led_pin, LOW);
}

/*######################################
	Handle the inputs with their functions
	######################################*/

void startButtonClickEvents(Input &input) {
	// TODO save the number of players from the 9-position button
	if (!GameSTEP) {
		GameSTEP ++;
		waitToSkip = 30; //wfix 19800
		DPRINTLN(F("Start Clicked!"));
	}
	else {
		if (!waitToSkip) {
			goForward = true;
			DPRINT(F("Skip current Input at Game Step: "));
			DPRINTLN(GameSTEP);
			waitToSkip = 30; // WFIX 660
		}
		else {
			DPRINT(F("waitToSkip: "));
			DPRINTLN(waitToSkip);
		}
	}
}

void resetButtonPressEvents(Input &input) {
	DPRINTLN(F("Technical Room Door Closed!"));
}

void resetButtonReleaseEvents(Input &input) {
	//Standby - SLV_ALL MSG: IDLE (Soft_reset ALL - ONCE)
	DPRINTLN(F("Technical Room Door Opened!"));
	DPRINTLN(F("RESTARTING ALL CPUs!"));

	// WFIX

	for ( int i = 0; i < SLV_TOTAL; i++) {
		MasterCommands(slavesAdresses[i], RESTART);
	}
	delay(7000);
	soft_restart();

}

void motorVrataHomePRESS(Input &input) {
	DPRINTLN(F("Klizna vrata has touched home."));
}

void motorVrataHomeHOLD(Input &input) {
	DPRINTLN(F("Klizna vrata is home."));
	delay(300);
	digitalWrite(VRATA_CLOSE, LOW);
	digitalWrite(VRATA_OPEN, LOW);
	revs_vrata = 1;
	DPRINTLN(millis() / 100);
}

void vrataskyHOLD(Input &input) {
	DPRINTLN(F("reed"));
	if (digitalRead(VRATA_OPEN) == HIGH) {
		DPRINTLN(F("SKY"));
		digitalWrite(VRATA_OPEN, LOW);
		delay(500);
		writeOutput(SECOND_DOOR_LOCK, RELAY_OFF);
		writeOutput(SMALLROOM_LIGHT, RELAY_ON);
		revs_vrata = 7;
	}
}

void motorVrataPingRELEASE(Input &input) {
	DPRINT(F("Klizna vrata revolution complete. @"));
	if (digitalRead(VRATA_OPEN) == HIGH) {
		revs_vrata++;
		if (revs_vrata >= revs_vrata_full)
		{
			delay(300);
			digitalWrite(VRATA_OPEN, LOW);
			delay(1500);
			writeOutput(SECOND_DOOR_LOCK, RELAY_OFF);
			writeOutput(SMALLROOM_LIGHT, RELAY_ON);
		}
	}
	if (digitalRead(VRATA_CLOSE) == HIGH) {
		revs_vrata--;
		if (revs_vrata <= 1)
		{
			delay(300);
			digitalWrite(VRATA_CLOSE, LOW);
		}
	}
	DPRINTLN(revs_vrata);
}

void motorLiftHomePRESS(Input &input) {
	if (spotlightison) {
		DPRINTLN(F("Lift is home."));
		digitalWrite(LIFT_UP, LOW);
		digitalWrite(LIFT_DOWN, HIGH);
		delay(500);
		digitalWrite(LIFT_DOWN, LOW);
		revs_lift = 1;
		writeOutput(SPOT_LIGHT, RELAY_OFF);
		spotlightison = false;
	}
}

void motorLiftPingRELEASE(Input &input) {
	DPRINT(F("Lift revolution complete. @"));
	if (digitalRead(LIFT_DOWN) == HIGH) {
		revs_lift++;
		if (revs_lift >= revs_lift_full)
		{
			delay(500);
			digitalWrite(LIFT_DOWN, LOW);
		}
	}
	else if (digitalRead(LIFT_UP) == HIGH) {
		revs_lift--;
		if (revs_lift <= 1)
		{
			delay(500);
			digitalWrite(LIFT_UP, LOW);
			writeOutput(SPOT_LIGHT, RELAY_OFF);
			spotlightison = false;
		}
	}
	DPRINTLN(revs_lift);
	if (revs_lift % 2)
		if (digitalRead(LIFT_UP) == HIGH) {
			MasterCommands( ADDR_MP3, MP3_PLAY_17);
		}
}

void circularPuzzleHoldEvents(Input &input) {
	DPRINTLN(F("Circular Puzzle Active"));
	circularPuzzleActive = true;
}

void fourCardsHoldEvents(Input &input) {
	DPRINTLN(F("fourCardsHoldEvents"));
	fourCardActive = true;
}

void robotDoneHoldEvents(Input &input) {
	DPRINTLN(F("robotDoneHoldEvents"));
	robotAllDone = true;
}

void bigRedButtonPressEvents(Input &input) {
	DPRINTLN(F("bigRedButtonPressEvents"));
	BigRedHit = true;
}

void secondDoorPressEvents(Input &input) {
	DPRINTLN(F("secondDoorPressEvents"));
	unstick_secondDoorState = true;
	writeOutput( RED_LED_RIGHT_LEDS, RELAY_ON);
	secondDoorState = DOOR_CLOSED;
}
void secondDoorReleaseEvents(Input &input) {
	unstick_secondDoorState = true;
	DPRINTLN(F("secondDoorReleaseEvents"));
	writeOutput( RED_LED_RIGHT_LEDS, RELAY_OFF);
	secondDoorState = DOOR_OPEN;
}

/*######################################
	MAIN LOOP
	######################################*/

void loop() {
#ifdef DEBUG
	int order = 0;
	int state = 0;

	while (Serial.available() > 0) {
		DPRINTLN("Thank you!");
		order = Serial.parseInt();
		address = Serial.parseInt();
		command = Serial.parseInt();
		switch (order) {
			case 1:
				DPRINT("State of ");
				DPRINT(address);
				DPRINT(" is ");
				for ( int i = 0; i < SLV_TOTAL; i++) {
					if (slavesAdresses[i] == address) state = slavesStates[i];
				}
				DPRINTLN(state, DEC);
				break;
			case 0:
				MasterCommands( address, command);
				DPRINT("Command Sent ");
				DPRINT(command);			// clear the string for new input:
				DPRINT(" to ");
				DPRINTLN(address);
				break;
			case 2:
				writeOutput( address, command );
				break;
			case 3:
				goForward = true;
				break;
			case 4:
				DPRINT("Game State: ");
				DPRINTLN(GameSTEP);
				DPRINT(F("Number of players: "));
				DPRINTLN(readPlayerSelector());
				DPRINTLN(F("Input States"));
				DPRINTLN(F("01234567 01234567"));

				for (byte mask = 0x80; mask; mask >>= 1) {
					if (mask	& InputBuffer[0])
						DPRINT('1');
					else
						DPRINT('0');
				}
				DPRINT(" ");
				for (byte mask = 0x80; mask; mask >>= 1) {
					if (mask	& InputBuffer[1])
						DPRINT('1');
					else
						DPRINT('0');
				}
				break;
		}
	}
#endif

	readAllInputs(); //read all inputs and store them in InputBuffer
	readAllButtons(); //read all inputs defined as buttons with functions.
	chatting(); // Master chatting with slaves
	readAllButtons(); //read all inputs defined as buttons with functions.
	gameLogic();

	writeAllOutputs(); // WFIX
	//if (startButton.held(7500)) DPRINTLN(F("Held for 7500ms"));
	//if (startButton.isActive()) DPRINTLN(F("Start Active!"));
	if (startButton.stateChanged()) DPRINTLN(F("Start state changed!"));
	if(millis() - timer_status_led > 600){
		digitalWrite(status_led_pin, HIGH);
		timer_status_led += 600;
		delay(10);
		digitalWrite(status_led_pin, LOW);
	}
}

/*######################################
	GAME LOGIC
	######################################*/

void gameLogic() {
	// Check Things that are in constant change and independet of game main logic
	AlwaysCheck();

	//Start Game Logic
	switch (GameSTEP) {
		case GAME_IDLE:
			gameIdle();
			break;
		case GAME_START:
			gameInitialize();
			break;
		case GAME_CIRCULAR_PUZZLE:
			gameStart();
			break;
		case GAME_LIGHTS_STAGE:
			gameLightsStage();
			break;
		case GAME_ROBOT_STAGE:
			gameRobotStage();
			break;
		case GAME_CODES_STAGE:
			gameCodesStage();
			break;
		case GAME_DISKS_STAGE:
			gameDisksStage();
			break;
		case GAME_HANDLE_STAGE:
			gameHandleStage();
			break;
		case GAME_BIGRED_STAGE:
			gameBigRedStage();
			break;
		case GAME_COLORS_STAGE:
			gameColorsStage();
			break;
		case GAME_LASERS:
			gameLasers();
			break;
		case GAME_LASERS + 1:
			game_over();
			break;
	}
}

void gameIdle() {
}

void gameInitialize() {
	/*
		GAME_START 1
		######################################################
		ACT: SLV_CODES > NumberOfPlayers
		ACT: START_TIMER */
	//DPRINT(".");
	if (!gameInitialized) {
		//Read number of playesrs and initialize CODE CPU
		DPRINT("Players: ");
		DPRINTLN(readPlayerSelector());
		MasterCommands( ADDR_CODES, readPlayerSelector() );
		nextLevel = false; // WFIX true
		gameInitialized = true;
		writeOutput(SPOT_LIGHT, RELAY_ON);
		spotlightison = true;
		writeOutput(SECOND_DOOR_LOCK, RELAY_ON);
		writeOutput(SMALLROOM_LIGHT, RELAY_ON);
		writeOutput(BUTTONNO, RELAY_ON);
		writeOutput(RED_LED_LEFT_LEDS, RELAY_ON);
		writeOutput(RED_LED_RIGHT_LEDS, RELAY_ON);
		MasterCommands(ADDR_MP3, MP3_PLAY_16);
		writeOutput(GREEN_LEDS, RELAY_ON);
		DPRINTLN("GAME BOOT UP - PREPARE NOW");
		timeout = 50;
	}

	if (!timeout && (nextLevel || goForward)) {
		goForward = false;
		nextLevel = false;
		GameSTEP++;
		if (revs_vrata > 1)
			digitalWrite(VRATA_CLOSE, HIGH);
		timervrata = millis();
		checkLightSensors = false;
		digitalWrite(LIFT_UP, HIGH);
		writeOutput(SMALLROOM_LIGHT, RELAY_OFF);
		writeOutput(BUTTONNO, RELAY_OFF);
		writeOutput(RED_LED_LEFT_LEDS, RELAY_OFF);
		writeOutput(RED_LED_RIGHT_LEDS, RELAY_OFF);
		writeOutput(LASER_1_POWER, RELAY_OFF);
		writeOutput(LASER_2_POWER, RELAY_OFF);
		digitalWrite(pinLasPOWER, LOW);
		writeOutput(GREEN_LEDS, RELAY_OFF);
		writeOutput(MAIN_DOOR_LOCK, RELAY_ON);
		writeOutput(MAIN_DOOR_LOCK, RELAY_ON);
		DPRINTLN("GAME PREPARED");
	}
}

void gameStart() {
	circularPuzzleDone.isActive();

	if (goForward || circularPuzzleActive) {
#ifdef DEBUG
		if (goForward) DPRINTLN(F("Skip circularPuzzleActive"));
		else DPRINTLN(F("circularPuzzle finished"));
#endif
		goForward = false;
		MasterCommands( ADDR_MP3, MP3_PLAY_1 );
		MasterCommands( ADDR_DMD, STAND_BY);
		nextLevel = true;
		writeOutput(MAIN_DOOR_LOCK, RELAY_ON);
		DPRINTLN("GAME BEGINS");
	}

	if (nextLevel) {
		nextLevel = false;
		GameSTEP ++;
	}
}

void gameLightsStage() {
	fourCardsDone.isActive();
	if ((reps == 0) && (goForward || (fourCardActive && !nextLevel ))) {
		goForward = false; //Stop cheating
		if (goForward) DPRINTLN(F("Skip fourCardActive!"));
		else DPRINTLN(" LEVEL FINISHED");
		MasterCommands( ADDR_MP3, MP3_PLAY_2 ); //intro sound before lights
		reps = 1;
		timeout = 50;
		//nextLevel = true;
	}

	if ((reps > 0) && nextLevel) {
		if (!timeout) {
			writeOutput( ROBOT_POWER, RELAY_ON);
			timeout = 50; // WFIX 660;
			nextLevel = false;
			GameSTEP ++;
		}
	}

	if ( (reps > 0) && (!timeout)) {
		nextLevel = true;
		MasterCommands( ADDR_MP3, MP3_PLAY_3 );
		timeout = 50; // WFIX 858;
		reps = 99;
	}
}

void gameRobotStage() {
	/*ROBOT JOB 4
	*/
	robotJobDone.isActive();
	if ( goForward || (robotAllDone && !nextLevel) ) {
		if (!timeout) {
			if (goForward) DPRINTLN("Skip robotAllDone!");
			else DPRINT("Robot job Done!");
			goForward = false;
			MasterCommands( ADDR_MP3, MP3_PLAY_2 );
			writeOutput( ROBOT_POWER, RELAY_OFF);
			nextLevel = true;
			timeout = 50;
		}
	}
	if (nextLevel && !timeout) {
		writeOutput( SPOT_LIGHT, RELAY_ON);
		spotlightison = true;
		digitalWrite( LIFT_UP, LOW);
		digitalWrite( LIFT_DOWN, HIGH);
		timerlift = millis();
		delay(10);
		MasterCommands( ADDR_CODES, ACTIVATE );
		delay(10);
		MasterCommands( ADDR_CODES, ACTIVATE );
		nextLevel = false;
		GameSTEP ++;
	}
}

void gameCodesStage() {
	/*GAME_CODES_STAGE 5
		######################################################
		CHK: SLV_CODES < ALL_CODES_DONE
		ACT: MP3 > PLAY_SOUND_X */
	//DPRINTLN(slavesStates[INDEX_CODES]);
	if (slavesStates[INDEX_CODES] == WRONG_CODE) {
		slavesStates[INDEX_CODES] = RUNNING;
		MasterCommands( ADDR_MP3, MP3_PLAY_18 );
	}
	else if (slavesStates[INDEX_CODES] == DONE_FOUR) {
		slavesStates[INDEX_CODES] = RUNNING;
		MasterCommands( ADDR_MP3, MP3_PLAY_4 );
	}

	switch (codesSteps) {
		case 1:
			if (slavesStates[INDEX_CODES] == DONE_ONE) {
				DPRINTLN(F("DONE_ONE - play Correct!"));
				MasterCommands( ADDR_MP3, MP3_PLAY_5 );
				codesSteps++;
			}
			break;
		case 2:
			if (slavesStates[INDEX_CODES] == DONE_TWO) {
				DPRINTLN(F("DONE_TWO - play Correct!"));
				MasterCommands( ADDR_MP3, MP3_PLAY_5 );
				codesSteps++;
			}
			break;
		case 3:
			if (slavesStates[INDEX_CODES] == DONE_THREE) {
				DPRINTLN(F("DONE_THREE - play Correct!"));
				MasterCommands( ADDR_MP3, MP3_PLAY_5 );
				codesSteps++;
			}
			break;
		case 4:
			if (slavesStates[INDEX_CODES] == CODES_DONE) {
				nextLevel = true;
				MasterCommands( ADDR_MP3, MP3_PLAY_6 );
				DPRINTLN(F("play Access Granted!"));
			}
			break;
		default:
			if (slavesStates[INDEX_CODES] == RUNNING)
				codesSteps++;
	}

	if (goForward || nextLevel) {
#ifdef DEBUG
		DPRINTLN("Skip codesSteps!");
#endif
		//MasterCommands( ADDR_MP3, MP3_PLAY_6 );
		goForward = false;
		nextLevel = false;
		GameSTEP ++;
	}
}
void gameDisksStage() {
	if (!disksDone) {
		if (true || (slavesStates[INDEX_CODES] == ALL_DONE)) {
			timeout = 50;
			nextLevel = true;
			disksDone = true;
		}
	}
	if (goForward || (nextLevel && !timeout) ) {
		if (goForward) DPRINTLN("Skip disksDone!");
		else DPRINTLN("LEVEL FINISHED");
		goForward = false;
		MasterCommands( ADDR_MP3, MP3_PLAY_7 );delay(1);		
		MasterCommands( ADDR_HANDLE, ACTIVATE );delay(1);		
		if (revs_vrata < 3)
			digitalWrite(VRATA_OPEN, HIGH);
		nextLevel = false;
		GameSTEP ++;
		writeOutput( RED_LED_LEFT_LEDS, RELAY_ON);
	}
}

void gameHandleStage() {
	if (handleOldState != slavesStates[INDEX_HANDLE] ) {
		handleChangeState = true;
		handleOldState = slavesStates[INDEX_HANDLE];
	}

	secondDoorClosed.isActive();
	if	( handleChangeState ) {
		switch ( slavesStates[INDEX_HANDLE] ) {
			case RUNNING:
				MasterCommands( ADDR_MP3, MP3_PLAY_8 );
				DPRINTLN("handle RUNNING!");
				break;
			case STOPPED:
				MasterCommands( ADDR_MP3, MP3_PLAY_13 );
				DPRINTLN("handle STOPEED!");
				//writeOutput( RED_LED_LEFT_LEDS, RELAY_OFF);
				break;
			case ALL_DONE:
				writeOutput( RED_LED_LEFT_LEDS, RELAY_ON);
				break;
		}
		handleChangeState = false;
	}

	if (goForward || 
		(slavesStates[INDEX_HANDLE] == ALL_DONE && secondDoorState == DOOR_CLOSED)){
		if ( goForward ) {
			if (goForward) DPRINTLN("Skip handleDone!");
			goForward = false;
			writeOutput( RED_LED_LEFT_LEDS, RELAY_ON);
			writeOutput( RED_LED_RIGHT_LEDS, RELAY_ON);
		}
		MasterCommands( ADDR_HANDLE, STAND_BY );delay(1);
		writeOutput( SECOND_DOOR_LOCK, RELAY_ON);
		writeOutput(GREEN_LEDS, RELAY_ON);delay(1);
		MasterCommands( ADDR_MP3, MP3_PLAY_14 );delay(1);
		nextLevel = true;
	}
	if (nextLevel) {
		fadeBIGRed = true;
		nextLevel = false;
		GameSTEP ++;
	}
}

void gameBigRedStage() {
	bigRedButton.isActive();
	if (goForward || (BigRedHit && !nextLevel) ) {
		if (goForward) DPRINTLN(F("Skip BigRedHit!"));
		else DPRINTLN("Big RED Hit!");
		goForward = false;
		fadeBIGRed = false;
		analogWrite(BIG_RED_LED, 0);
		MasterCommands( ADDR_MP3, MP3_PLAY_10);
		writeOutput(SPOT_LIGHT, RELAY_OFF);
		spotlightison = false;
		writeOutput ( RED_LED_LEFT_LEDS, RELAY_OFF);
		writeOutput( RED_LED_RIGHT_LEDS, RELAY_OFF);
		writeOutput( LASER_1_POWER, RELAY_ON);
		writeOutput( LASER_2_POWER, RELAY_ON);
		digitalWrite(pinLasPOWER, HIGH);

		delay(500);
		MasterCommands( ADDR_ARROWS, ACTIVATE );
		delay(500);
		MasterCommands( ADDR_ARROWS, ACTIVATE );
		writeOutput(BUTTONNO, RELAY_ON);

		timeout = 50;
		nextLevel = true;
	}

	if (nextLevel && !timeout) {
		checkLightSensors = true;
		nextLevel = false;
		GameSTEP ++;
	}
}

void gameColorsStage() {
	secondDoorClosed.isActive();
	if (goForward || (slavesStates[INDEX_ARROWS] == ALL_DONE && !nextLevel) ) {
		goForward = false;
		DPRINTLN("Laser Step 1");
		MasterCommands( ADDR_MP3, MP3_PLAY_12 );
		writeOutput( SECOND_DOOR_LOCK, RELAY_OFF);
		//MasterCommands( ADDR_CODES, STAND_BY);
		//writeOutput( ROBOT_LIGHT, RELAY_OFF);
		oneLaser = true;
		lockPlayer = true;
		timeout = 1650; // WFIX 1650
		nextLevel = true;
	}
	if (nextLevel && !timeout) {
		DPRINTLN("Laser next level");
		timeout = 1;
		writeOutput( MAIN_DOOR_LOCK, RELAY_OFF);
		nextLevel = false;
		GameSTEP ++;
	}
}
void gameLasers() {
	if (!timeout) {
		digitalWrite( pinLasPOWER, LOW);
		writeOutput( LASER_2_POWER, RELAY_OFF);
		delay(100);
		writeOutput( LASER_2_POWER, RELAY_ON);
		delay(500);
		writeOutput( LASER_2_POWER, RELAY_OFF);
		delay(100);
		writeOutput( LASER_2_POWER, RELAY_ON);
		delay(300);
		for (int i = 0; i < 30; i++) {
			writeOutput( LASER_2_POWER, RELAY_OFF);
			delay((20 + random(120)));
			writeOutput( LASER_2_POWER, RELAY_ON);
			delay((7 + random(20)));
		}
		writeOutput( LASER_2_POWER, RELAY_OFF);
		delay(1000);
		writeOutput( LASER_2_POWER, RELAY_ON);
		delay(10);
		writeOutput( LASER_2_POWER, RELAY_OFF);

		lockPlayer = false;
		GameSTEP++;
		timeout = 10000;
	}
}

void game_over() {
	if (!timeout || goForward) {
		goForward = false;
		writeOutput(LASER_1_POWER, RELAY_OFF);
		writeOutput(LASER_2_POWER, RELAY_OFF);
		digitalWrite(pinLasPOWER, LOW);
		checkLightSensors = false;
		timeout = 10000;
	}
}

/* ################ */
/* Helper Functions */
/* ################ */

void AlwaysCheck() {
	//Check Laser ensors and trigger alarm
	checkLightSensors = false;
	if (checkLightSensors) {
		if ((digitalRead(pinLaserTrig) == HIGH) && !timerMP3) {
			DPRINT(" ALARM ON ");
			if (!lockPlayer) {
				DPRINT("- Play 11");
				MasterCommands( ADDR_MP3, MP3_PLAY_11 );
				MasterCommands( ADDR_DMD, WRONG_CODE);
				timerMP3 = 150;
			}
			DPRINTLN("");
		}
	}
	vratasky.isActive();
	motorVrataHome.isActive();
	motorLiftHome.isActive();
	motorVrataPing.isActive();
	motorLiftPing.isActive();

	if (digitalRead(VRATA_CLOSE) == HIGH)
		if (millis() - timervrata > 15000)
		{
			DPRINTLN(F("Door taking too long to close!"));
			digitalWrite(VRATA_CLOSE, LOW);
		}
	if (digitalRead(LIFT_DOWN) == HIGH)
		if (millis() - timerlift > 14000)
		{
			DPRINTLN(F("Lift stop"));
			digitalWrite(LIFT_DOWN, LOW);
		}
	if (timer1h > 0)
		if (millis() - timer1h > 3600000) {
			timer1h = 0;
			//MasterCommands(ADDR_MP3, MP3_PLAY_11);
		}
	writeAllOutputs();
}

int readPlayerSelector() {
	byte b1;
	byte b2;
	byte b3;
	byte b4;
	b1 = bitRead(InputBuffer[0], BCD_PIN_01);
	b2 = bitRead(InputBuffer[0], BCD_PIN_02);
	b3 = bitRead(InputBuffer[0], BCD_PIN_03);
	b4 = bitRead(InputBuffer[0], BCD_PIN_04);
	b1 = invertBit(b1);
	b2 = invertBit(b2);
	b3 = invertBit(b3);
	b4 = invertBit(b4);
	return 4;
	return	b1 + (b2 << 1) + (b3 << 2) + (b4 << 3);
}

byte invertBit(byte bit) {
	if ( bit )
		return 0;
	return 1;
}

// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr() {
	if (timer300ms)
		timer300ms--;
	else {
		timer300ms	= 20;
		activateI2C = true; //every 300 miliseconds
	}

	if (timeout) timeout --; //non-block delays
	if (timerMP3) timerMP3--; //MP3 Timer

	if (waitToSkip) waitToSkip--; //delay to prevent accidental skips

	//Fade BIG RED LED
	if (fadeBIGRed) {
		if (fadeOUT) {
			bright += 15;
			if (bright > 255)
			{
				bright = 255;
				fadeOUT = 0;
			}
		}
		else {
			bright -= 2;
			if (bright < 0)
			{
				bright = 0;
				fadeOUT = 1;
			}
		}
		analogWrite(BIG_RED_LED, logaritmicLED[bright]);
	}
}

void MasterCommands(uint8_t address, uint8_t command) {
	Wire.beginTransmission(address);
	Wire.write(command);
	error = Wire.endTransmission();
	/*if (error) {
		for ( int i = 0; i < SLV_TOTAL; i++) {
			if (slavesAdresses[i] == address) slavesCommands[i] = command;
		}
		}*/
	delay(1);
}

void chatting() {
	//Command Slaves
	if (activateI2C) {
		for ( int i = 0; i < SLV_TOTAL; i++) {
			address = slavesAdresses[i];
			// request status (1 byte) from slave device
			Wire.requestFrom(address, (uint8_t)1);
			// slave may send more than requested - only the last one is taken into account
			if (Wire.available()) {
				// receive a byte as character
				slavesStates[i] = Wire.read();
			}
			//Resend Unsent Commands to Slaves
			if (slavesCommands[i] != NOTHING_YET) {
				Wire.beginTransmission(address);
				Wire.write(slavesCommands[i]);
				error = Wire.endTransmission();
				if (!error) slavesCommands[i] = NOTHING_YET;
			}
		}

		switch (slavesStates[INDEX_META]) {
			case ALL_DONE:
				if (!waitToSkip) {
					goForward = true;
					DPRINT(F("Skip current Input at Game Step: "));
					DPRINTLN(GameSTEP);
					waitToSkip = 30; // WFIX 660
				}
				else {
					DPRINT(F("waitToSkip: "));
					DPRINTLN(waitToSkip);
				}
				break;
			case DONE_ONE:
				if (revs_vrata < 3) {
					revs_vrata = 1;
					digitalWrite(VRATA_OPEN, HIGH);
				}
				break;
			case RUNNING:
				Wire.beginTransmission(ADDR_DMD);
				delay(5);
				Wire.write(DONE_ONE);
				Wire.endTransmission();
				DPRINTLN("Timer start");
				break;
			case WRONG_CODE:
				MasterCommands( ADDR_ARROWS, STAND_BY );
				delay(1000);
				MasterCommands( ADDR_ARROWS, ACTIVATE );
				break;
			case TRIGGERED:
				MasterCommands(ADDR_MP3, MP3_PLAY_17);
				break;
			default:
				break;
		}

		activateI2C = false;
	}
}

bool getInput(byte n, byte k) {
	k = 7 - k;
	return ((n & ( 1 << k )) >> k) == 0;
}

void readAllButtons() {
	startButton.isActive();
	resetButton.isActive();

	/*
		if(getInput(InputBuffer[1], BIG_RED_PRESSED)) {
			DPRINTLN("CIRCLE DONE");
			circularPuzzleActive = true;
		}
		if(getInput(InputBuffer[1], BIG_RED_PRESSED)) BigRedHit = true;
		else BigRedHit = false;
		if(getInput(InputBuffer[0], ROBOT_JOB_DONE)) robotAllDone = true;
		if(getInput(InputBuffer[0], pinNORMALNA_VRATA_ZATVORENA))
			secondDoorState = true;
		else
			secondDoorState = false;

		if(getInput(InputBuffer[0], FOUR_CARDS_DONE)) fourCardActive = true;
		else fourCardActive = false;
	*/
}

void readAllInputs() {
	byte the_shifted = 0; // An 8 bit number to carry each bit value of A-H

	// Trigger loading the state of the A-H data lines into the shift register
	digitalWrite(shld_pin, LOW);
	delayMicroseconds(PULSE_WIDTH_USEC); // Requires a delay here according to the datasheet timing diagram
	digitalWrite(shld_pin, HIGH);
	delayMicroseconds(PULSE_WIDTH_USEC);

	// Required initial states of these two pins according to the datasheet timing diagram
	pinMode(clk_pin, OUTPUT);
	pinMode(data_pin, INPUT);
	digitalWrite(clk_pin, HIGH);
	digitalWrite(ce_pin, LOW); // Enable the clock

	// Get the A-H values
	InputBuffer[0] = shiftIn(data_pin, clk_pin, LSBFIRST);
	InputBuffer[1] = shiftIn(data_pin, clk_pin, LSBFIRST);
	digitalWrite(ce_pin, HIGH); // Disable the clock
}

// Shift Base Function
void shiftOut(int myDataPin, int myClockPin, uint8_t myDataOut) {
	// This shifts 8 bits out MSB first,
	//on the rising edge of the clock,
	//clock idles low

	//internal function setup
	int i = 0;
	int pinState;
	pinMode(myClockPin, OUTPUT);
	pinMode(myDataPin, OUTPUT);

	//clear everything out just in case to
	//prepare shift register for bit shifting
	digitalWrite(myDataPin, 0);
	digitalWrite(myClockPin, 0);

	//for each bit in the byte myDataOut�
	//NOTICE THAT WE ARE COUNTING DOWN in our for loop
	//This means that %00000001 or "1" will go through such
	//that it will be pin Q0 that lights.
	for (i = 7; i >= 0; i--)	{
		digitalWrite(myClockPin, 0);

		//if the value passed to myDataOut and a bitmask result
		// true then... so if we are at i=6 and our value is
		// %11010100 it would the code compares it to %01000000
		// and proceeds to set pinState to 1.
		if ( myDataOut & (1 << i) ) {
			pinState = 1;
		}
		else {
			pinState = 0;
		}

		//Sets the pin to HIGH or LOW depending on pinState
		digitalWrite(myDataPin, pinState);
		//register shifts bits on upstroke of clock pin
		digitalWrite(myClockPin, 1);
		//zero the data pin after shift to prevent bleed through
		digitalWrite(myDataPin, 0);
	}

	//stop shifting
	digitalWrite(myClockPin, 0);
}

void writeOutput(uint8_t pin, bool on, bool writeNow) {
	uint8_t xMask;
	uint8_t x = pin % 8;
	uint8_t y = pin / 8;
	//	DPRINT("PIN: ");
	//	DPRINT(pin);
	//	DPRINT("-");
	//	DPRINT(x);
	//	DPRINT("-");
	//	DPRINTLN(y);
	if (on) {
		xMask = 1 << x;
		OutputBuffer[y] |= xMask;
	} else {
		xMask = (1 << x) ^ 0xff;
		OutputBuffer[y] &= xMask;
	}
	if (writeNow)
		writeAllOutputs();
}

void writeAllOutputs() {
	for (int i = 0; i < 3; i++) {
		//Shift OUTPUT Bits
		//ground shiftLatchPin and hold low for as long as you are transmitting
		digitalWrite(latchPin, LOW);
		//move 'em out
		shiftOut(dataPin, clockPin, MSBFIRST, OutputBuffer[0]);
		shiftOut(dataPin, clockPin, MSBFIRST, OutputBuffer[1]);

		//return the latch pin high to signal chip that it
		//no longer needs to listen for information
		digitalWrite(latchPin, HIGH);

		delay(1);
	}
}

void soft_restart() {
	do {
		wdt_enable(WDTO_15MS);
		// hic sunt leones - code from a library. it works, no idea how or why. didn't manage to import the library
		for (;;) {
		}
	} while (0);
}
