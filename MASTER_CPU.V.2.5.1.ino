//#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DSERIALSTART(...)  Serial.begin(__VA_ARGS__) //DSERIALSTART is a macro, serial initialise.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
  #define DWRITE(...)  Serial.write(__VA_ARGS__)   //DWRITE is a macro, debug write
  
#else
  #define DSERIALSTART(...)  //now defines a blank line
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
  #define DWRITE(...)     //now defines a blank line
#endif

//MASTER Brain
#include <RoomTwo.h>
#include <Wire.h>
#include <FastLED.h>
#include <TimerOne.h>
#include <avr/wdt.h>
#include <BufferedInputs.h>

#define TRUE 1
#define FALSE 0

#define WRITE_NOW true
#define WRITE_WAIT false


//#####################
//Input DATA
//#####################

const int data_pin = 5; // Connect Pin 11 to SER_OUT (serial data out)
const int shld_pin = 4; // Connect Pin 8 to SH/!LD (shift or active low load)
const int clk_pin = 3; // Connect Pin 12 to CLK (the clock that times the shifting)
const int ce_pin = 2; // Connect Pin 9 to !CE (clock enable, active low)

#define PULSE_WIDTH_USEC   5 // Width of pulse to trigger the shift register to read and latch.

uint8_t InputBuffer[] = { 0XFF, 0XFF }; // 16 Output ports

// Input Pins
// Inputs stored in InputBuffer[0]

#define BCD_PIN_01 0 //PULL_UP
#define BCD_PIN_02 1 //PULL_UP
#define BCD_PIN_03 2 //PULL_UP
#define BCD_PIN_04 3 //PULL_UP
#define START_BUTTON 4 //PULL_UP
#define RESTART_BUTTON 5 //PULL_UP


// Inputs stored in InputBuffer[1]
#define CIRCULAR_PUZZLE_DONE 0 ////PULL_UP-Optocuplor
#define FOUR_CARDS_DONE 1 //PULL_UP
#define ROBOT_JOB_DONE 2 //PULL_UP
#define SECOND_DOOR_CLOSED 3 //PULL_UP
#define BIG_RED_PRESSED 4 //PULL_UP - Wire 2 reds - ALb Albastru

//#define AVAILABLE 6 ////PULL_UP-Optocuplor
#define SENZOR_LASER_1 A1
#define SENZOR_LASER_2 A2


/* //Main buttons */
// Inputs stored in InputBuffer[0]
Input startButton = Input(&InputBuffer[0], START_BUTTON, BUTTON_PULLUP);
#define HELD_TIME 3000

Input resetButton = Input(&InputBuffer[0], RESTART_BUTTON, BUTTON_PULLUP);
Input bigRedButton = Input(&InputBuffer[1], BIG_RED_PRESSED, BUTTON_PULLUP);

// Inputs stored in InputBuffer[1]
Input fourCardsDone = Input(&InputBuffer[1], FOUR_CARDS_DONE, BUTTON_PULLUP);
Input robotJobDone = Input(&InputBuffer[1], ROBOT_JOB_DONE, BUTTON_PULLUP);
Input secondDoorClosed = Input(&InputBuffer[1], SECOND_DOOR_CLOSED, BUTTON_PULLUP);
Input circularPuzzleDone = Input(&InputBuffer[1], CIRCULAR_PUZZLE_DONE, BUTTON_PULLUP);

//Inputs Activated
bool circularPuzzleActive  = FALSE;
bool fourCardActive  = FALSE;
bool BigRedHit = FALSE;
bool disksDone = FALSE;

//#####################
//Output DATA
//#####################
#define latchPin 7 //Pin connected to ST_CP of 74HC595
#define clockPin 8 //Pin connected to SH_CP of 74HC595
#define dataPin 9 //Pin connected to DS of 74HC595

// Output PINS
#define MAIN_DOOR_LOCK 0  //12V - Nr 6 Portocaliu/Verde + AlbP/AlbV
#define SECOND_DOOR_LOCK 1  //12V - Banda Galbena - Portocaliu/Verde + AlbP/AlbV
// banda albastra - DoorLocled Leds - Alb + Maro Door Unlocked - Alb Albastru / Senzor usa inchisa Alb-Portocaliu
//Conecate pe acelasi releu ca incuetoarea usii secundare.
#define STONE_DOOR_LOCK 2  //12V - Banda Rosie
#define ROBOT_LIGHT 3  //12V;  +/-5V; GND  Secondary relays for Light and +/-5V
#define RED_LED_LEFT_LEDS 4  //12V Benzi Rosii - Alb+Verde
#define RED_LED_RIGHT_LEDS 5  //12V 2 Benzi Rosii - Alb+Portocaliu
#define TABLET_PLATE 6 //12V 
#define ROBOT_POWER 7 // +/-5V - NR 3 - Portocaliu +5V/ Maro -5V / Alb GND (in releu)  //Albastru senzor la masa

#define MAIN_LIGHT_1 8 //220V
#define MAIN_LIGHT_2 9 //220V
#define MAIN_LIGHT_3 10 //220V
#define MAIN_LIGHT_4 11 //220V
#define SMOKE_POWER 12 //220V
#define SMOKE_PUFF 13 //220V
#define LASER_1_POWER 14  //3.3V - Nr 4 - Alb+Portocaliu
#define LASER_2_POWER 15  //3.3V - Nr 4 - Alb+Albastru

#define BIG_RED_LED  6 //PWM pin for BIG_RED_LED -- 2 Benzi Rosii - Alb Maro

/*  MAIN_DOOR_LOCK (relay Normal CLOSED)
  STONE_DOOR_LOCK (relay Normal CLOSED)
  SECOND_DOOR_LOCK (relay Normal OPEN)
  SECRET_BOX_LOCK (relay Normal CLOSED)
  ROBOT_POWER (relay Normal OPEN) */
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

//BIG RED LED
bool fadeOUT = FALSE;
bool fadeBIGRed = FALSE; 
uint8_t logaritmicLED[256] = {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,7,7,7,7,7,7,8,8,8,8,8,8,9,9,9,9,10,10,10,10,10,11,11,11,11,12,12,12,13,13,13,13,14,14,14,15,15,15,16,16,16,17,17,18,18,18,19,19,20, 20,20,21,21,22,22,23,23,24,24,25,26,26,27,27,28,29, 29,30,31,31,32,33,33,34,35,36,36,37,38,39,40,41,42, 42,43,44,45,46,47,48,50,51,52,53,54,55,57,58,59,60, 62,63,64,66,67,69,70,72,74,75,77,79,80,82,84,86,88, 90,91,94,96,98,100,102,104,107,109,111,114,116,119, 122,124,127,130,133,136,139,142,145,148,151,155,158,161,165,169,172,176,180,184,188,192,196,201,205,210,214,219,224,229,234,239,244,250,255};
uint8_t bright = 0;

//Slaves I2C
const uint8_t slavesAdresses[SLV_TOTAL] = {ADDR_MP3, ADDR_CODES, ADDR_HANDLE, ADDR_ARROWS};
uint8_t slavesStates[SLV_TOTAL];
uint8_t slavesCommands[SLV_TOTAL];
uint8_t command = NOTHING_YET;
uint8_t error;

//I2C trigger
uint8_t timer300ms = 0;
uint8_t address;
boolean activateI2C = false;

//Robot
bool robotAllDone = FALSE;

//Lasers Smoke & Alarm
#define SENZOR_ONE_LEVEL 150
#define SENZOR_TWO_LEVEL 200
bool checkLightSensors = FALSE;

//Sounds
bool soundNotStarted = false;


//monitor the  activated codes 
int codesSteps = 0;


//Handle logic
bool handleChangeState = FALSE;
byte handleOldState = NOTHING_YET;

//Lasers Sensors & Alarm
//Average the readings
const int numReadings = 35; //nr of reading to average

int readings[2][numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
long total = 0;                  // the running total
int average = 0;                // the average

const int sensorPins[] = {SENZOR_LASER_1, SENZOR_LASER_2};

bool alarmChangeState = FALSE;
bool alarmOldState = TRUE;

bool lockPlayer = FALSE;
bool oneLaser = FALSE;

uint8_t sensorOne;
uint8_t sensorTwo;
bool alarmOn = FALSE;

//Game Logic
int GameSTEP = 0; //game just started
bool GameRESET = FALSE;
bool goForward = FALSE;
bool nextLevel= FALSE;

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
int reps = 0; //repetitions


//Function Declaration and Function Prototypes 
void writeOutput(uint8_t pin, bool on, bool writeNow = TRUE);


//The initialization
void setup()
{
  // hic sunt leones - init soft reset code
  MCUSR = 0;
  wdt_disable();

  delay(2000); // sanity check delay - allows reprogramming if accidently blowing power w/leds
  DSERIALSTART(115200); // start serial for output
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

  
  //set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  //initialize slave states to none.
  for (int i = 0; i<SLV_TOTAL; i++)
    slavesStates[i] = NOTHING_YET;

  // join i2c bus (address optional for master)
  Wire.begin();
  error = 0;
  for ( int i = 0; i<SLV_TOTAL; i++) {
    address = slavesAdresses[i];
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    #ifdef DEBUG    
        if(error){
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

  
  // what if START pressed
  startButton.clickHandler(startButtonClickEvents);
  
  //Reset Game
  resetButton.pressHandler(resetButtonPressEvents);
  resetButton.releaseHandler(resetButtonReleaseEvents); 

  circularPuzzleDone.holdHandler(circularPuzzleHoldEvents, 1000);

  fourCardsDone.holdHandler(fourCardsHoldEvents, 1000);

  //if Robot Done
  robotJobDone.holdHandler(robotDoneHoldEvents, 1000); 
    
  //if Big Red pressed
  bigRedButton.pressHandler(bigRedButtonPressEvents);

  //Secondary Door Closed/Open 
  secondDoorClosed.pressHandler(secondDoorPressEvents);
  secondDoorClosed.releaseHandler(secondDoorReleaseEvents);
 
  DPRINTLN(F("Buttons initialised!"));
  
  //Timer start
  Timer1.initialize(15000);
  Timer1.attachInterrupt( timerIsr );

  DPRINTLN(F("Timer one Up and Running!"));

  //Initialize OUTPUT
  for ( int i = 0; i<16; i++) {
    writeOutput( i, RELAY_OFF);
  }
  DPRINTLN(F("All OUTPUT ports OFF"));

  //Initialize Laser Sensors
    for(int i = 0; i < 2; i++){
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[i][thisReading] = 0;
    }
  }
}

/*######################################
Handle the inputs with their functions
######################################*/

void startButtonClickEvents(Input &input){
  // TODO save the number of players from the 9-position button
  if (!GameSTEP){
    GameSTEP ++;
    waitToSkip = 19800;
    DPRINTLN(F("Start Clicked!"));
  }
  else{
      if (!waitToSkip) {
        goForward = TRUE;
        DPRINT(F("Skip current Input at Game Step: "));
        DPRINTLN(GameSTEP); 
        waitToSkip = 660;       
      }
      else{
        DPRINT(F("waitToSkip: "));
        DPRINTLN(waitToSkip);        
      }
  }
}

void resetButtonPressEvents(Input &input){
  DPRINTLN(F("Technical Room Door Closed!"));
}

void resetButtonReleaseEvents(Input &input){
  //Standby - SLV_ALL MSG: IDLE (Soft_reset ALL - ONCE)
  DPRINTLN(F("Technical Room Door Opened!"));
  DPRINTLN(F("RESTARTING ALL CPUs!"));

  for ( int i = 0; i<SLV_TOTAL; i++) {
    MasterCommands(slavesAdresses[i], RESTART);     
  }
  delay(7000);
  soft_restart();
}

void circularPuzzleHoldEvents(Input &input){
  DPRINTLN("Circular Puzzle Active");
  circularPuzzleActive = TRUE;
}

void fourCardsHoldEvents(Input &input){
  fourCardActive = TRUE;
}

void robotDoneHoldEvents(Input &input){
  if(!timeout) robotAllDone = TRUE;
}

void bigRedButtonPressEvents(Input &input){
  BigRedHit = TRUE;
}

void secondDoorPressEvents(Input &input){
 writeOutput( RED_LED_RIGHT_LEDS, RELAY_ON);
  secondDoorState = DOOR_CLOSED;
}
void secondDoorReleaseEvents(Input &input){
 writeOutput( RED_LED_RIGHT_LEDS, RELAY_OFF);
  secondDoorState = DOOR_OPEN;
}

/*######################################
MAIN LOOP
######################################*/

void loop(){
  #ifdef DEBUG    
    int order = 0;
    int state = 0;
    while (Serial.available() > 0) {
      DPRINTLN("Thank you!");
      order = Serial.parseInt();
      address = Serial.parseInt();
      command = Serial.parseInt();
      switch (order){
        case 1:
          DPRINT("State of ");
          DPRINT(address);
          DPRINT(" is ");
          for ( int i = 0; i<SLV_TOTAL; i++) {
            if (slavesAdresses[i] == address) state = slavesStates[i];
          }
          DPRINTLN(state, DEC);
        break;
        case 0:
           MasterCommands( address, command);
          DPRINT("Command Sent ");
          DPRINT(command);      // clear the string for new input:
          DPRINT(" to ");
          DPRINTLN(address);
        break;   
        case 2:
          writeOutput( address, command );
        break; 
        case 3:
          goForward = TRUE;
        break;
        case 4:
            DPRINT("Game State: ");
            DPRINTLN(GameSTEP);
            DPRINT("Number of players: ");
            DPRINTLN(readPlayerSelector());
            DPRINTLN("Input States");
            DPRINTLN("01234567 01234567");
            
            for(byte mask = 0x80; mask; mask >>= 1){
             if(mask  & InputBuffer[0])
                 DPRINT('1');
             else
                 DPRINT('0');
            }
            DPRINT(" ");
            for(byte mask = 0x80; mask; mask >>= 1){
             if(mask  & InputBuffer[1])
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
  //if (startButton.held(7500)) DPRINTLN(F("Held for 7500ms")); 
  //if (startButton.isActive()) DPRINTLN(F("Start Active!")); 
  if (startButton.stateChanged()) DPRINTLN(F("Start state changed!")); 
}

/*######################################
GAME LOGIC
######################################*/

void gameLogic() {
  // Check Things that are in constant change and independet of game main logic
  AlwaysCheck();

  //Start Game Logic
  switch (GameSTEP){
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
  }
}

void gameIdle(){
}

void gameInitialize(){
/* 
GAME_START 1
######################################################
ACT: SLV_CODES > NumberOfPlayers
ACT: START_TIMER */ 

  //Read number of playesrs and initialize CODE CPU
  DPRINT("Players: ");
  DPRINTLN(readPlayerSelector());
  MasterCommands( ADDR_CODES, readPlayerSelector() );
  nextLevel= TRUE;
  
  if(nextLevel){
    nextLevel= FALSE;
    GameSTEP ++;
  }
}

void gameStart(){
/* 
GAME_CIRCULAR_PUZZLE 2
######################################################
CHK: CIRCULAR_PUZZLE
ACT: MP3 > PLAY_SOUND_X */
  circularPuzzleDone.isActive();
  if(goForward || circularPuzzleActive){
    goForward = FALSE;
    #ifdef DEBUG 
    if(goForward) DPRINTLN("Skip circularPuzzleActive");
    #endif
    MasterCommands( ADDR_MP3, MP3_PLAY_1 );
    nextLevel= TRUE;
  }
  
  if(nextLevel){
    nextLevel= FALSE;
    GameSTEP ++;
  }
}

void gameLightsStage(){
/* GAME_LIGHTS_STAGE 3
######################################################
CHK: IO < 4CARDS_DONE
ACT: MP3 > PLAY_SOUND_X
ACT: IO > LIGHT_X_ON (X = 1 to 4)

ACT: MP3 > PLAY_SOUND_X
ACT: IO > ACTIVATE_ROBOT_POWER */

  fourCardsDone.isActive();
  if( goForward || (reps == 0 && fourCardActive )){
    goForward = FALSE; //Stop cheating
    #ifdef DEBUG 
    if(goForward) DPRINTLN("Skip fourCardActive!");
    #endif
    MasterCommands( ADDR_MP3, MP3_PLAY_2 ); //intro sound before lights
    reps = 1;
    timeout = 66;
  }
  
  if ( reps > 0 && reps < 5 ){
    //ACT: Light 4 leds above MAIN door one by one
    if (!timeout) {
      DPRINT("Turn ON Light: ");
      DPRINTLN(reps);
      switch (reps){
        case 1:
          writeOutput( MAIN_LIGHT_1, RELAY_ON);
          MasterCommands( ADDR_MP3, MP3_PLAY_2 ); 
          timeout = 66;
          break;
        case 2:
          writeOutput( MAIN_LIGHT_2, RELAY_ON);
          MasterCommands( ADDR_MP3, MP3_PLAY_2 ); 
          timeout = 66;
          break;
        case 3:
          writeOutput( MAIN_LIGHT_3, RELAY_ON);
          MasterCommands( ADDR_MP3, MP3_PLAY_2 ); 
          timeout = 66;
          break;
        case 4:
            writeOutput( MAIN_LIGHT_4, RELAY_ON);
            MasterCommands( ADDR_MP3, MP3_PLAY_2 );
            timeout = 130;
          break;
      }
      reps ++;
    }
  }

  if(reps == 5 && !timeout){
      nextLevel = TRUE;            
      MasterCommands( ADDR_MP3, MP3_PLAY_3 );
      timeout = 858;
      reps = 99;
  }  
              
  if(nextLevel){
    if (!timeout) {
      writeOutput( ROBOT_POWER, RELAY_ON);
      writeOutput( ROBOT_LIGHT, RELAY_ON);
      timeout = 660;
      nextLevel= FALSE;
      GameSTEP ++;
    }
  }
}
void gameRobotStage(){
/*ROBOT JOB 4  
*/
  robotJobDone.isActive();
  if ( goForward || (robotAllDone && !nextLevel) ){
    if(!timeout){
      goForward = FALSE;
      #ifdef DEBUG 
      if(goForward) DPRINTLN("Skip robotAllDone!"); 
      #endif
      DPRINT("Robot job Done!");
      writeOutput( TABLET_PLATE, RELAY_ON);
      MasterCommands( ADDR_MP3, MP3_PLAY_6 );
      timeout = 130;
      nextLevel= TRUE;
    }
  }
  
  if(nextLevel){
    if (!timeout) {
      writeOutput( TABLET_PLATE, RELAY_OFF);       
      writeOutput( ROBOT_POWER, RELAY_OFF);
      MasterCommands( ADDR_CODES, ACTIVATE );      
      nextLevel= FALSE;
      GameSTEP ++;
    }
  }  
}

void gameCodesStage(){
/*GAME_CODES_STAGE 5
######################################################
CHK: SLV_CODES < ALL_CODES_DONE
ACT: MP3 > PLAY_SOUND_X  */ 

  switch (codesSteps){
    case 1:
      if (slavesStates[INDEX_CODES] == DONE_ONE){
        DPRINTLN("DONE_ONE - play Correct!");
        MasterCommands( ADDR_MP3, MP3_PLAY_5 );
        codesSteps++;
      }    
    break;
    case 2:
      if (slavesStates[INDEX_CODES] == DONE_TWO){
        DPRINTLN("DONE_TWO - play Correct!");
        MasterCommands( ADDR_MP3, MP3_PLAY_5 );
        codesSteps++;
      }     break;
    case 3:
      if (slavesStates[INDEX_CODES] == DONE_THREE){
        DPRINTLN("DONE_THREE - play Correct!");
        MasterCommands( ADDR_MP3, MP3_PLAY_5 );
        codesSteps++;
      }     break;
    case 4:
      if (slavesStates[INDEX_CODES] == CODES_DONE){
        nextLevel = TRUE;
        MasterCommands( ADDR_MP3, MP3_PLAY_6 ); 
        DPRINTLN("play Access Granted!");    
      }    
    break;
    default:
      if (slavesStates[INDEX_CODES] == NOTHING_YET)
       codesSteps++;    
  } 

  if(goForward || nextLevel){
    if(goForward){
      #ifdef DEBUG
      if(goForward) DPRINTLN("Skip codesSteps!"); 
      #endif
      MasterCommands( ADDR_MP3, MP3_PLAY_6 );
      goForward = FALSE;
    }
    nextLevel= FALSE;
    GameSTEP ++;
  }
}
void gameDisksStage(){
/* :ReSTART GAME_DISKS_STAGE 6
######################################################
CHK: SLV_CODES < ALL_DISKS_DONE
ACT: MP3 > PLAY_SOUND_X
ACT: SECOND_DOOR_UNLOCK
ACT: BIG_STONE_UNLOCK */

  if (!disksDone){
    if (slavesStates[INDEX_CODES] == ALL_DONE){
      timeout = 66;
      nextLevel = TRUE;
      disksDone = TRUE;
    }
  }
  if(goForward || (nextLevel && !timeout) ){
    goForward = FALSE;
    #ifdef DEBUG
    DPRINTLN("Skip disksDone!");
    #endif
    MasterCommands( ADDR_MP3, MP3_PLAY_7 );
    MasterCommands( ADDR_HANDLE, ACTIVATE );
    writeOutput( STONE_DOOR_LOCK, RELAY_ON);
    nextLevel= FALSE;
    GameSTEP ++;
  }
}

void gameHandleStage(){
/* :ReSTART GAME_HANDLE_STAGE 7
######################################################
CHK: < SECOND_DOOR_CLOSED/SECOND_DOOR_OPEN
ACT: > RIGHT_SIDE_BIG_RED_ON/RIGHT_SIDE_BIG_RED_OFF

CHK: SLV_HANDLE < ALL_DONE/HANDLE_INACTIVE
ACT: MP3 > PLAY_SOUND_X/STOP_PLAY
ACT: SLV_IO > LEFT_SIDE_BIG_RED_ON/LEFT_SIDE_BIG_RED_OFF

CHK: (SLV_HANDLE < ALL_DONE) && SECOND_DOOR_CLOSED
ACT: SLV_IO > LEFT_SIDE_BIG_RED_ON
ACT: SLV_HANDLE > HANDLE_FREEZE 
ACT: MP3 > PLAY_SOUND_X
ACT: SLV_IO > SECOND_DOOR_LOCKED 
ACT: SLV_IO > BIG_RED_HEARTBEAT */  
  if (handleOldState != slavesStates[INDEX_HANDLE] ){
    handleChangeState = TRUE;
    handleOldState = slavesStates[INDEX_HANDLE];
  }
  
  secondDoorClosed.isActive();
  if  ( handleChangeState ){
    switch ( slavesStates[INDEX_HANDLE] ){
      case  RUNNING:
        MasterCommands( ADDR_MP3, MP3_PLAY_8 );
        DPRINTLN("handle RUNNING!");
      break;
      case  STOPPED:
        MasterCommands( ADDR_MP3, MP3_PLAY_13 );
        DPRINTLN("handle STOPEED!");
        writeOutput( RED_LED_LEFT_LEDS, RELAY_OFF);
      break;
      case  ALL_DONE:
        writeOutput( RED_LED_LEFT_LEDS, RELAY_ON);
      break;
    }
    handleChangeState = FALSE; 
  }

  if (goForward || (slavesStates[INDEX_HANDLE] == ALL_DONE  && secondDoorState == DOOR_CLOSED) ){
      if ( goForward ) {
        #ifdef DEBUG
        if(goForward) DPRINTLN("Skip handleDone!");
        #endif
        goForward = FALSE;
        writeOutput( RED_LED_LEFT_LEDS, RELAY_ON);
        writeOutput( RED_LED_RIGHT_LEDS, RELAY_ON);
      }
      MasterCommands( ADDR_HANDLE, STAND_BY );
      writeOutput( SECOND_DOOR_LOCK, RELAY_ON);
      MasterCommands( ADDR_MP3, MP3_PLAY_14 );
      nextLevel = TRUE;
  }
  
  if(nextLevel){
    fadeBIGRed = TRUE;
    nextLevel= FALSE;
    GameSTEP ++;
  }  
   
}

void gameBigRedStage(){

/* :ReSTART 8
######################################################
CHK: SLV_IO < BIG_RED_PUSHED
ACT: SLV_IO  > LASER_1_ON
ACT: SLV_IO  > LASER_2_ON
ACT: SLV_IO  > SMOKE_ON
ACT: MP3 > PLAY_SOUND_X
ACT: SLV_HANDLE > RUN_ARROWS */ 
  bigRedButton.isActive(); 
  if(goForward || (BigRedHit && !nextLevel) ){
    goForward = FALSE;
    #ifdef DEBUG
    if(goForward) DPRINTLN("Skip BigRedHit!");
    #endif
    DPRINTLN("Big RED Hit!");
    fadeBIGRed = FALSE;
    analogWrite(BIG_RED_LED, 0);
    MasterCommands( ADDR_MP3, MP3_PLAY_10 );
    writeOutput( MAIN_LIGHT_1, RELAY_OFF);
    writeOutput( MAIN_LIGHT_2, RELAY_OFF);
    writeOutput( MAIN_LIGHT_3, RELAY_OFF);
    writeOutput( MAIN_LIGHT_4, RELAY_OFF);
    writeOutput( RED_LED_LEFT_LEDS, RELAY_OFF);
    writeOutput( RED_LED_RIGHT_LEDS, RELAY_OFF);
    writeOutput( LASER_1_POWER, RELAY_ON);
    writeOutput( LASER_2_POWER, RELAY_ON);
    timeout = 330;
    nextLevel = TRUE;
  }

  if(nextLevel && !timeout){
    MasterCommands( ADDR_ARROWS, ACTIVATE );
    MasterCommands( ADDR_MP3, MP3_PLAY_13 );
    checkLightSensors  = TRUE;
    nextLevel= FALSE;
    GameSTEP ++;
  } 
}



void gameColorsStage(){
/*  
:ReSTART 9
######################################################
CHK: SLV_COLORS < ALL_DONE
ACT: MP3 > PLAY_SOUND_X
ACT: SLV_IO  > LIGHT_SENSOR_2_OFF
ACT: SLV_IO  > LASER_2_OFF
ACT: SLV_IO  > MAIN_DOOR_UNLOCK */

  if (goForward || (slavesStates[INDEX_ARROWS] == ALL_DONE && !nextLevel) ){
    goForward = FALSE;
    #ifdef DEBUG
    if(goForward) DPRINTLN("Skip arrowsDone!");
    #endif
    DPRINTLN("Laser Step 1");
    MasterCommands( ADDR_MP3, MP3_PLAY_12 );
    writeOutput( SECOND_DOOR_LOCK, RELAY_OFF);       
    writeOutput( LASER_2_POWER, RELAY_OFF);
    MasterCommands( ADDR_CODES, STAND_BY);
    writeOutput( ROBOT_LIGHT, RELAY_OFF);
    oneLaser = TRUE;
    lockPlayer = TRUE;
    timeout = 1650;
    nextLevel = TRUE;
  }
  if(nextLevel && !timeout){
    DPRINTLN("Laser next level");
    timeout = 198;
    writeOutput( MAIN_DOOR_LOCK, RELAY_ON);
    nextLevel= FALSE;
    GameSTEP ++;
  } 
}
void gameLasers(){
  // ACT: PLAY sound of sistem crush / burning down / victory
  if (!timeout)  lockPlayer = FALSE;
}

void game_over(){
}

/* ################ */
/* Helper Functions */
/* ################ */

void  AlwaysCheck(){
//Check Laser ensors and trigger alarm  
  if (checkLightSensors){
    updateSensorsReadings();
    sensorOne = averageReadings(0);
    sensorTwo = averageReadings(1);
    alarmOn = FALSE;
    if (oneLaser){
      alarmOn =  sensorTwo > SENZOR_TWO_LEVEL;
    }
    else{
      alarmOn =  sensorOne > SENZOR_ONE_LEVEL || sensorTwo > SENZOR_TWO_LEVEL;
    }

    if (alarmOldState != alarmOn ){
      alarmChangeState = TRUE;
      alarmOldState = alarmOn;
    } 
    if (alarmChangeState && !timerMP3){     
      DPRINT("Sensors Values");
      DPRINT(sensorOne);
      DPRINT(":");
      DPRINT(sensorTwo);
      DPRINT(":");
      DPRINT(" AlarmState: ");
      DPRINT(alarmOn);
      if (alarmOn){
         DPRINT(" ALARM ON ");
         if (!lockPlayer){
           DPRINT("- Play 11");
           MasterCommands( ADDR_MP3, MP3_PLAY_11 );
           timerMP3=150;
         }
      }
      alarmChangeState = FALSE; 
      DPRINTLN("");
    }
  }

}

// Reading Laser sensors and average the value.
int averageReadings(int sensor){
    total = 0;
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      total += readings[sensor][thisReading];
    }
    average = total / numReadings;
    return average;
}

void updateSensorsReadings(){
  for(int sensor = 0; sensor < 2; sensor++){  
    readings[sensor][readIndex] = analogRead(sensorPins[sensor]);
    // if we're at the end of the array...
  }
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
      readIndex = 0;
  }
}

int readPlayerSelector(){
  byte b1;
  byte b2;
  byte b3;
  byte b4;
  b1 = bitRead(InputBuffer[0], BCD_PIN_01);
  b2 = bitRead(InputBuffer[0], BCD_PIN_02);
  b3 = bitRead(InputBuffer[0], BCD_PIN_03);
  b4 = bitRead(InputBuffer[0], BCD_PIN_04);
  b1 =invertBit(b1);
  b2 =invertBit(b2);
  b3 =invertBit(b3);
  b4 =invertBit(b4);
  return  b1 + (b2 << 1) + (b3 << 2) + (b4 << 3);
}

byte invertBit(byte bit){
  if( bit )
    return 0;
  return 1;
}

// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr(){
  if (timer300ms)
    timer300ms--;
  else {
    timer300ms  = 20;
    activateI2C = true;  //every 300 miliseconds
  }

  if (timeout) timeout --; //non-block delays
  if (timerMP3) timerMP3--; //MP3 Timer
  
  if (waitToSkip) waitToSkip--; //delay to prevent accidental skips
  
  //Fade BIG RED LED
  if(fadeBIGRed){
    if (fadeOUT) {
      bright += 15;
      if(bright>255)
      {
        bright = 255;
        fadeOUT = 0;
      }
    }
    else {
      bright -= 2;
      if(bright<0)
      {
        bright = 0;
        fadeOUT = 1;
      }
    }
    analogWrite(BIG_RED_LED, logaritmicLED[bright]);
  }
}

void MasterCommands(uint8_t address, uint8_t command){
  Wire.beginTransmission(address);
  Wire.write(command);
  error = Wire.endTransmission();
   if(error){
     for ( int i = 0; i<SLV_TOTAL; i++) {
       if (slavesAdresses[i] == address) slavesCommands[i] = command;
     }
   }
}

void chatting(){
  //Command Slaves
  if (activateI2C) {
    for ( int i = 0; i<SLV_TOTAL; i++) {
      address = slavesAdresses[i];
      // request status (1 byte) from slave device
      Wire.requestFrom(address, 1);
      // slave may send more than requested - only the last one is taken into account
      if(Wire.available()) {
        // receive a byte as character
        slavesStates[i] = Wire.read();
      }
      //Resend Unsent Commands to Slaves
      if (slavesCommands[i] != NOTHING_YET){
        Wire.beginTransmission(address);
        Wire.write(slavesCommands[i]);
        error = Wire.endTransmission();
        if(!error) slavesCommands[i] = NOTHING_YET;
      }  
    }
    activateI2C = false;
  }
}

void readAllButtons(){
  startButton.isActive();
  resetButton.isActive();
}

void readAllInputs(){
  byte the_shifted = 0;  // An 8 bit number to carry each bit value of A-H

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
  InputBuffer[0] = shiftIn(data_pin, clk_pin, MSBFIRST);
  InputBuffer[1] = shiftIn(data_pin, clk_pin, MSBFIRST);
  digitalWrite(ce_pin, HIGH); // Disable the clock
}

// Shift Base Function
void shiftOut(int myDataPin, int myClockPin, uint8_t myDataOut) {
  // This shifts 8 bits out MSB first,
  //on the rising edge of the clock,
  //clock idles low

  //internal function setup
  int i=0;
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
  for (i=7; i>=0; i--)  {
    digitalWrite(myClockPin, 0);

    //if the value passed to myDataOut and a bitmask result
    // true then... so if we are at i=6 and our value is
    // %11010100 it would the code compares it to %01000000
    // and proceeds to set pinState to 1.
    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {
      pinState= 0;
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
  uint8_t x = pin%8;
  uint8_t y = pin/8;
//  DPRINT("PIN: ");
//  DPRINT(pin);
//  DPRINT("-");
//  DPRINT(x);
//  DPRINT("-");
//  DPRINTLN(y);
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



void writeAllOutputs(){  
  //Shift OUTPUT Bits
  //ground shiftLatchPin and hold low for as long as you are transmitting
    digitalWrite(latchPin, LOW);
  //move 'em out
    shiftOut(dataPin, clockPin, MSBFIRST, OutputBuffer[0]);
    shiftOut(dataPin, clockPin, MSBFIRST, OutputBuffer[1]);

  //return the latch pin high to signal chip that it
  //no longer needs to listen for information
    digitalWrite(latchPin, HIGH);
}

void soft_restart(){
  do {
    wdt_enable(WDTO_15MS);
    // hic sunt leones - code from a library. it works, no idea how or why. didn't manage to import the library
    for(;;){
    }
  } while(0);
}
