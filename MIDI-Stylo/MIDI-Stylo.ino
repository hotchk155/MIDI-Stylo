//////////////////////////////////////////////////////////////////////////
//
// MIDI STYLOPHONE
//
// (c) Jason Hotchkiss 2012
// http://hotchk155.blogspot.com/
// 
//////////////////////////////////////////////////////////////////////////
#include <EEPROM.h>


// standby flag
byte appStandBy;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//
//
// MIDI STUFF
//
//
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
// macro defs
#define MIDI_PITCHBEND_CENTRE 0x2000  // No pitch bend
#define MIDI_CC_MODWHEEL   1          // Continunous controller number for mod wheel

//////////////////////////////////////////////////////////////////////////
// Init MIDI comms
void midiInit()
{
#ifdef DEBUG  
   Serial.begin(9600);
#else   
   Serial.begin(31250);
#endif   
}

//////////////////////////////////////////////////////////////////////////
// Send MIDI note (zero velocity for note off)
void midiNote(byte note, byte velocity)
{
  note &= 0x7f;
  velocity &= 0x7f;
  Serial.write(0x90);
  Serial.write(note);
  Serial.write(velocity);
}

//////////////////////////////////////////////////////////////////////////
// Send MIDI pitch bend (14 bits)
void midiPitchBend(int value)
{
    value &= 0x3fff;
    Serial.write(0xe0);
    Serial.write(value & 0x7f);
    Serial.write((value>>7) & 0x7f);  
}

//////////////////////////////////////////////////////////////////////////
// Send MIDI continuous controller (7 bits)
void midiController(byte cc, byte value)
{
    value &= 0x7f;
    Serial.write(0xb0);
    Serial.write(cc);
    Serial.write(value);    
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//
//
// LED FUNCTIONS
// Based on a common cathode LED on PWM pins 3, 5 and 6
//
//
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// macro defs

// pins
#define P_PWM_RED    9
#define P_PWM_GREEN  11
#define P_PWM_BLUE   10

// colours (rgb PWM values stored in a 24 bits of an unsigned long)
#define RGB(r,g,b) (((unsigned long)(r)<<16)|((unsigned long)(g)<<8)|(unsigned long)(b))
#define COL_BLUE     RGB(0,0,255)
#define COL_RED      RGB(255,0,0)
#define COL_MAGENTA  RGB(255,0,255)
#define COL_GREEN    RGB(0,10,0)
#define COL_CYAN     RGB(0,255,255)
#define COL_YELLOW   RGB(255,255,0)
#define COL_WHITE    RGB(255,255,255)
#define COL_DIMRED      RGB(5,0,0)
#define COL_DIMGREEN    RGB(0,2,0)
#define COL_DIMBLUE     RGB(0,0,5)
#define COL_DIMWHITE    RGB(5,5,5)

// variables
byte ledState=0;
unsigned long ledNextToggle = 0;

//////////////////////////////////////////////////////////////////////////
// INIT LED STUFF
void ledInit()
{
   // pins
   pinMode(P_PWM_RED, OUTPUT);
   pinMode(P_PWM_GREEN, OUTPUT);
   pinMode(P_PWM_BLUE, OUTPUT);
   
   // reset pwm
   analogWrite(P_PWM_RED, 0);
   analogWrite(P_PWM_GREEN, 0);
   analogWrite(P_PWM_BLUE, 0);
   
   // init vars
   ledState=0;
   ledNextToggle = 0;
}

//////////////////////////////////////////////////////////////////////////
// SET UP LED PWM
// if using a common anode LED subtract each value from 255 
void setLed(unsigned long l)
{
  analogWrite(P_PWM_RED, (l>>16)&0xff);
  analogWrite(P_PWM_GREEN, (l>>8)&0xff);
  analogWrite(P_PWM_BLUE, l&0xff);
}

//////////////////////////////////////////////////////////////////////////
// FLASHING LED STATE MACHINE
void ledFlash(unsigned long milliseconds, unsigned long a, unsigned long b, unsigned long rate)
{
  if(milliseconds > ledNextToggle)
  {
     ledNextToggle = milliseconds + rate;
     setLed(ledState? a : b);
     ledState=!ledState;
  }
}

//////////////////////////////////////////////////////////////////////////
// TEST LEDS
void ledTest() 
{
  for(;;)
  {
    Serial.println("Red");
    setLed(COL_RED);
    delay(1000);
    Serial.println("Green");
    setLed(COL_GREEN);
    delay(1000);
    Serial.println("Blue");
    setLed(COL_BLUE);
    delay(1000);
  }
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//
// STYLOPHONE KEYPAD POLLING STUFF
//
// USES 3 74HC165 SHIFT REGISTERS TO POLL THE KEYS
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// MACRO DEFS

// pins
#define P_LATCH 8
#define P_CLK   7
#define P_DATA  6

#define KEYBOARD_DEBOUNCE_TIME 20
#define KEYBOARD_MAX_OCTAVE 4
#define KEYBOARD_INIT_OCTAVE 2
#define KEYBOARD_MIN_OCTAVE 0
#define KEYBOARD_ROOT_NOTE 21

enum {
  KEY_A        = 22,
  KEY_ASHARP   = 21,
  KEY_B        = 11,
  KEY_C        = 10,   
  KEY_CSHARP   = 9,
  KEY_D        = 23,
  KEY_DSHARP   = 8,
  KEY_E        = 15,
  KEY_F        = 14,
  KEY_FSHARP   = 13,
  KEY_G        = 20,
  KEY_GSHARP   = 12,
  KEY_A2       = 6,
  KEY_ASHARP2  = 3,
  KEY_B2       = 2,
  KEY_C2       = 1,
  KEY_CSHARP2  = 0,
  KEY_D2       = 4,
  KEY_DSHARP2  = 7,
  KEY_E2       = 5
};

// vars
byte noteHold;
byte notePressed;
byte keyMap[24];
byte noteState[127];
byte keyboardOctave;
unsigned long keyboardDebounceTime;
byte keyboardRootMidiNote;
byte keyboardLastNote;
byte keyboardHoldMode;
byte keyboardClearHold;
byte keyboardNotesHeld;

/////////////////////////////////////////////////////////////////
// INITIALISE STYLOPHONE KEYBOARD INPUT
void keyboardInit()
{
  // configure the shift register control lines
  pinMode(P_LATCH, OUTPUT);
  pinMode(P_CLK, OUTPUT);
  pinMode(P_DATA, INPUT);
  
  // pull up resistors on inputs
  digitalWrite(P_DATA,HIGH);  
  digitalWrite(P_LATCH,HIGH);
  
  // initialise the lookup table that converts the
  // bit inputs to the actual MIDI note values
  memset(keyMap,255,sizeof (keyMap));
  keyMap[KEY_A]       = 0;
  keyMap[KEY_ASHARP]  = 1;  
  keyMap[KEY_B]       = 2;  
  keyMap[KEY_C]       = 3;  
  keyMap[KEY_CSHARP]  = 4;  
  keyMap[KEY_D]       = 5;  
  keyMap[KEY_DSHARP]  = 6;  
  keyMap[KEY_E]       = 7;  
  keyMap[KEY_F]       = 8;  
  keyMap[KEY_FSHARP]  = 9;  
  keyMap[KEY_G]       = 10; 
  keyMap[KEY_GSHARP]  = 11; 
  keyMap[KEY_A2]      = 12; 
  keyMap[KEY_ASHARP2] = 13; 
  keyMap[KEY_B2]      = 14; 
  keyMap[KEY_C2]      = 15; 
  keyMap[KEY_CSHARP2] = 16; 
  keyMap[KEY_D2]      = 17; 
  keyMap[KEY_DSHARP2] = 18; 
  keyMap[KEY_E2]      = 19; 

  // init vars
  memset(noteState,0,sizeof(noteState));
  keyboardDebounceTime = 0;
  keyboardOctave = KEYBOARD_INIT_OCTAVE;
  keyboardRootMidiNote = KEYBOARD_ROOT_NOTE + 12 * KEYBOARD_INIT_OCTAVE;
  keyboardLastNote = 0;
  keyboardHoldMode = 0;
  keyboardClearHold = 0;
  keyboardNotesHeld = 0;
}

//////////////////////////////////////////////////////////////////////////
// OCTAVE SHIFT
void keyboardShift(int delta)
{
  if(delta > 0 && keyboardOctave >= KEYBOARD_MAX_OCTAVE)
    return;
  if(delta < 0 && keyboardOctave <= KEYBOARD_MIN_OCTAVE)
    return;  
  keyboardOctave+=delta;
  keyboardRootMidiNote = KEYBOARD_ROOT_NOTE + 12 * keyboardOctave;
}

//////////////////////////////////////////////////////////////////////////
// CLEAR HELD NOTES
void keyboardClearHeldNotes()
{
  for(int i=0;i<127;++i)
  {
    if(noteState[i])
    {
      midiNote(i, 0);
      noteState[i] = 0;
    }
  }
  keyboardNotesHeld = 0;
}

/////////////////////////////////////////////////////////////////
// POLL THE KEYBOARD
void keyboardRun(unsigned long ulMilliseconds)
{
  // dont bother polling the keyboard till any debounce 
  // period has expired
  if(ulMilliseconds < keyboardDebounceTime)
    return;
      
  // latch the digital inputs to the shift registers
  digitalWrite(P_LATCH,LOW);
  digitalWrite(P_LATCH,HIGH);
    
  // iterate through the 24 clock cycles of the shift
  // registers
  byte whichNote = 0;
  for(int i=0;i<24;++i)
  {
    // check this is not a "floating" bit
    if(keyMap[i] != 255)
    {
      // fetch the next input bit
      byte state = digitalRead(P_DATA);
      
      // is the stylus touching this pad?
      if(state)
      {
        whichNote = keyMap[i] + keyboardRootMidiNote;
        break;
      }
    }
    
    // pulse the clock line
    digitalWrite(P_CLK,LOW);
    digitalWrite(P_CLK,HIGH);
  }
  
  // release the latch
  digitalWrite(P_LATCH,HIGH);
  
  // is a note being pressed ?
  if(whichNote)
  {
    // are we in hold mode?
    if(keyboardHoldMode)
    {
      // make sure we have registered the current note (hold
      // button might have been pressed while the note was
      // already playing)
      noteState[whichNote] = 1;
      keyboardNotesHeld = 1;
      keyboardClearHold = 0;
    }
    
    // is a new note being played?
    if(whichNote != keyboardLastNote)
    {
      // need to kill the last note?
      if(keyboardLastNote && !noteState[keyboardLastNote])
      {
         midiNote(keyboardLastNote, 0);
      }
      
      // play the new note
      midiNote(whichNote, 0x7f);
      keyboardLastNote = whichNote;    
      keyboardDebounceTime = ulMilliseconds + KEYBOARD_DEBOUNCE_TIME;
    }
  }
  else 
  {
      // check the last note is not held before killing it
      if(keyboardLastNote && !noteState[keyboardLastNote])
      {
         midiNote(keyboardLastNote, 0);
      }
      keyboardLastNote = 0;
  }
}



/////////////////////////////////////////////////////////////////
// TEST KEYBOARD
void keyboardTest()
{
  byte states[24] = {0};
  for(;;)
  {
    // latch the digital inputs to the shift registers
    digitalWrite(P_LATCH,LOW);
    digitalWrite(P_LATCH,HIGH);
      
      
    // iterate through the 24 clock cycles of the shift
    // registers
    int pad = 0;
    int note = 0;
    for(int i=0;i<24;++i)
    {
      if(i==16||i==17||i==18||i==19)
      {
      }
      else
      {
        // fetch the next input bit
        byte state = digitalRead(P_DATA);
        if(state != states[i])
        {
          states[i] = state;
          Serial.print(i);
          Serial.print("=");
          Serial.println(state);        
        }
      }
      
      // pulse the clock line
      digitalWrite(P_CLK,LOW);
      digitalWrite(P_CLK,HIGH);
    }
    
    // release the latch
    digitalWrite(P_LATCH,HIGH);
  }
  
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//
// ACCELEROMETER/TILT SENSOR HANDLING
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// MACRO DEFS

// pins
#define P_ACC_ENABLE  17
#define P_ACC_X  A2
#define P_ACC_Y  A0
#define P_ACC_Z  A1

// timing
#define CALIBRATE_DELAY_PERIOD 3000
#define CALIBRATE_AVERAGE_PERIOD 3000
#define TILT_POLL_PERIOD 20

// scaling
#define TILT_X_FSD  100
#define TILT_Y_FSD  100
#define TILT_Z_FSD  100
#define TILT_X_DEADWINDOW  10
#define TILT_Y_DEADWINDOW  5
#define TILT_Z_DEADWINDOW  0

// send tolerances
#define TILT_PITCHBEND_SEND_TOLERANCE   4
#define TILT_AUX_SEND_TOLERANCE         2
#define TILT_MODWHEEL_SEND_TOLERANCE    2

// enum defines EEPROM storage of calibration values
enum {
   TILT_CAL_XORG,
   TILT_CAL_YORG,
   TILT_CAL_ZORG,
   TILT_CAL_COUNT
};

// vars
int tiltCalibration[TILT_CAL_COUNT];
unsigned long tiltNextPoll;
byte tiltActive;
byte tiltHold;
int tiltLastPitchBend;
int tiltLastAux;
int tiltLastModWheel;


/////////////////////////////////////////////////////////////////
// READ CALIBRATION VALUES FROM EEPROM
void tiltCalibrationRead()
{
  byte addr = 0;
  for(int i=0; i<TILT_CAL_COUNT; i++)
  {
    tiltCalibration[i] = EEPROM.read(addr++);
    tiltCalibration[i] += 256 * EEPROM.read(addr++);
  }
}

/////////////////////////////////////////////////////////////////
// WRITE CALIBRATION VALUES INTO EEPROM
void tiltCalibrationWrite()
{
  byte addr = 0;
  for(int i=0; i<TILT_CAL_COUNT; i++)
  {
    EEPROM.write(addr++, tiltCalibration[i] % 256);
    EEPROM.write(addr++, tiltCalibration[i] / 256);
  }
}

/////////////////////////////////////////////////////////////////
// CALIBRATE THE TILT SENSOR
void tiltCalibrateChannel(int value, int index)
{  
  if(!tiltCalibration[index])
  {
    tiltCalibration[index] = value;
  }
  else
  {
    tiltCalibration[index] = 0.9 * tiltCalibration[index] + 0.1 * value;
  }
}      
void tiltCalibrate()
{
  unsigned long nextTime;
  nextTime = millis() + CALIBRATE_DELAY_PERIOD;
  while(millis() < nextTime)
  {
    ledFlash(millis(), COL_MAGENTA, 0, 200);
  }
  memset(tiltCalibration, 0, sizeof(tiltCalibration));
  nextTime = millis() + CALIBRATE_AVERAGE_PERIOD;  
  while(millis() < nextTime)
  {
    ledFlash(millis(), COL_MAGENTA, 0, 50);
    tiltCalibrateChannel(analogRead(P_ACC_X), TILT_CAL_XORG);
    tiltCalibrateChannel(analogRead(P_ACC_Y), TILT_CAL_YORG);
    tiltCalibrateChannel(analogRead(P_ACC_Z), TILT_CAL_ZORG);
  }
  setLed(0);  
  tiltCalibrationWrite();
}

/////////////////////////////////////////////////////////////////
// INITIALISE THE TILT SENSING CODE
void tiltInit()
{
  // set pin modes
   pinMode(P_ACC_ENABLE, OUTPUT);
   pinMode(P_ACC_X, INPUT);
   pinMode(P_ACC_Y, INPUT);
   pinMode(P_ACC_Z, INPUT);

   // read the tilt calibration values
   tiltCalibrationRead();
   
   // enable the accelerometer (must be >1 ms after powerup)
   digitalWrite(P_ACC_ENABLE, LOW);
   delay(10);
   digitalWrite(P_ACC_ENABLE, HIGH);
   
   // init vars
   tiltActive = 0;
   tiltHold = 0;
   tiltLastPitchBend = MIDI_PITCHBEND_CENTRE;
   tiltLastAux = 0;
   tiltLastModWheel = 0;   
}

/////////////////////////////////////////////////////////////////
// RUN THE TILT SENSING CODE
int tiltScaleInput(int value, int index, int deadwindow, int fsd, int minVal, int maxVal, int sign)
{  
  int outFsd = (maxVal - minVal)/2;   
  int midScale = (minVal + maxVal)/2;
  int offset = value - tiltCalibration[index];
  if(abs(offset) < deadwindow)
  {
    return midScale;
  }
  else
  {
    int q = midScale + sign * ((float)offset * outFsd) / fsd;
    return constrain(q,minVal,maxVal);
  }  
}    
void tiltRun(unsigned long ulMilliseconds)
{
  // check if the next keyboard poll is due
  if(ulMilliseconds < tiltNextPoll)
    return;
  tiltNextPoll = ulMilliseconds + TILT_POLL_PERIOD;
  
  int reading;
  
  // Read the pitch bend sensor
  reading = tiltScaleInput(analogRead(P_ACC_X), TILT_CAL_XORG, TILT_X_DEADWINDOW, TILT_X_FSD, 0, 0x3fff, -1);
  if(abs(reading - tiltLastPitchBend) > TILT_PITCHBEND_SEND_TOLERANCE)
  {
    midiPitchBend(reading);
    tiltLastPitchBend = reading;
  }

/*  // auxilliary CC
  reading = tiltScaleInput(analogRead(P_ACC_Y), TILT_CAL_YORG, TILT_Y_DEADWINDOW, TILT_Y_FSD, 0, 127, 1);
  if(abs(reading - tiltLastAux) > TILT_AUX_SEND_TOLERANCE)
  {
    midiController(MIDI_CC_AUX, reading);
    tiltLastAux = reading;
  }*/
  
  // Read the mod wheel sensor
  reading = tiltScaleInput(analogRead(P_ACC_Z), TILT_CAL_ZORG, TILT_Z_DEADWINDOW, TILT_Z_FSD, 0, 127, -1);
  if(abs(reading - tiltLastModWheel) > TILT_MODWHEEL_SEND_TOLERANCE)
  {
    midiController(MIDI_CC_MODWHEEL, reading);
    tiltLastModWheel = reading;
  }
}  

/////////////////////////////////////////////////////////////////
// CALLED WHEN THE TILT CODE IS STOPPED
int tiltStopRunning()
{
  tiltLastAux = 0;
  tiltLastModWheel = 0;
  tiltLastPitchBend = MIDI_PITCHBEND_CENTRE;
  midiPitchBend(tiltLastPitchBend);
}

void tiltTest() {
  for(;;)
  {
    Serial.print(analogRead(A0));
    Serial.print(",");
    Serial.print(analogRead(A1));
    Serial.print(",");
    Serial.print(analogRead(A2));
    Serial.println();
    delay(100);
  }
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//
// BUTTON POLLING STUFF
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// MACRO DEFS

// pins
#define P_BUTTON1  3
#define P_BUTTON2  18
#define P_BUTTON3  19

#define P_BUTTON4  2


// bit map defs
#define M_BUTTON1  0x01
#define M_BUTTON2  0x02
#define M_BUTTON3  0x04
#define M_BUTTON4  0x08

// button combinations
#define BUTTON_HOLD          M_BUTTON1
#define BUTTON_TILT          M_BUTTON2
#define BUTTON_OCTAVE_DOWN   M_BUTTON3
#define BUTTON_OCTAVE_UP     M_BUTTON4
#define BUTTON_TILT_HOLD    (M_BUTTON1 | M_BUTTON2)
#define BUTTON_STANDBY      (M_BUTTON1 | M_BUTTON4)
#define BUTTON_CALIBRATE    (M_BUTTON2 | M_BUTTON3 | M_BUTTON4)

// timing
#define BUTTON_DEBOUCE_TIME 5

// vars
unsigned long buttonsDebounceTime = 0;
byte buttonsLastState;

/////////////////////////////////////////////////////////////////
// INIT BUTTONS HANDLING
void buttonsInit()
{
   // init pins
   pinMode(P_BUTTON1, INPUT);
   pinMode(P_BUTTON2, INPUT);
   pinMode(P_BUTTON3, INPUT);
   pinMode(P_BUTTON4, INPUT);

   // set pull ups
   digitalWrite(P_BUTTON1, HIGH);
   digitalWrite(P_BUTTON2, HIGH);
   digitalWrite(P_BUTTON3, HIGH);
   digitalWrite(P_BUTTON4, HIGH);
   
   // init vars
   buttonsLastState = 0;
}

/////////////////////////////////////////////////////////////////
// BUTTON PRESS 
// When a new button is pressed, this function gets the 
// state of all the buttons in a single byte
void onButtonDown(byte buttons)
{  
  if((buttons & BUTTON_TILT_HOLD) == BUTTON_TILT_HOLD)
  {
    tiltHold = 1;
    tiltActive = 1;
  }
  else if((buttons & BUTTON_STANDBY) == BUTTON_STANDBY)
  {
    if(!appStandBy)
    {
      appStandBy = 1;
      tiltActive = 0;
      keyboardClearHeldNotes();
      tiltStopRunning();
    }
  }
  else if((buttons & BUTTON_CALIBRATE) == BUTTON_CALIBRATE)
  {
    tiltCalibrate();
  }
  else if((buttons & BUTTON_HOLD) == BUTTON_HOLD)
  {
    if(appStandBy)
    {
      appStandBy = 0;
    }
    else
    {
      keyboardHoldMode = 1;
      keyboardClearHold = 1;
    }
  }
  else if((buttons & BUTTON_TILT) == BUTTON_TILT)
  {
    tiltActive = 1;
  }
  else if((buttons & BUTTON_OCTAVE_DOWN) == BUTTON_OCTAVE_DOWN)
  {
    keyboardShift(-1);
  }
  else if((buttons & BUTTON_OCTAVE_UP) == BUTTON_OCTAVE_UP)
  {
    keyboardShift(+1);
  }
}

/////////////////////////////////////////////////////////////////
// BUTTON RELEASE HANDLER
// When buttons are release, this function gets the set of 
// buttons that have just been released
void onButtonUp(byte buttons)
{  
  if(buttons & BUTTON_HOLD)
  {
    if(keyboardClearHold)
      keyboardClearHeldNotes();
    keyboardHoldMode = 0;
  }
  if(buttons & BUTTON_TILT)
  {
    if(!tiltHold)
    {
      tiltStopRunning();
      tiltActive = 0;
    }
    tiltHold = 0;
  }
}

/////////////////////////////////////////////////////////////////
// RUN BUTTONS HANDLING
void buttonsRun(unsigned long ulMilliseconds)
{
  // do nothing if we're debouncing
  if(ulMilliseconds < buttonsDebounceTime)
    return;
  
  // gather the button state map
  byte state = 0;
  state |= digitalRead(P_BUTTON1) ? 0 : M_BUTTON1;
  state |= digitalRead(P_BUTTON2) ? 0 : M_BUTTON2;
  state |= digitalRead(P_BUTTON3) ? 0 : M_BUTTON3;
  state |= digitalRead(P_BUTTON4) ? 0 : M_BUTTON4;

  // detect buttons that have just been pressed
  byte pressedButtons = ~buttonsLastState & state;
  if(pressedButtons)
  {
    // ensure we're not in standby
    if(!appStandBy)
    {
      // handle button normally
      onButtonDown(state);
    }
    else
    {
      // any button wakes us out of standby mode
      appStandBy = 0;
    }
    
    // ignore all button activity until debounce danger zone is passed
    buttonsDebounceTime = ulMilliseconds + BUTTON_DEBOUCE_TIME;
  }
  else
  {
    // detect buttons that have just been released
    byte releasedButtons = buttonsLastState & ~state;
    if(releasedButtons)
    {
      // ensure we're not in standby
      if(!appStandBy)
      {
        // handle button normally
        onButtonUp(releasedButtons);
      }
    }
  }
  
  // remember button state
  buttonsLastState = state;  
}

void buttonsTest() 
{
  for(;;)
  {
     Serial.print(digitalRead(P_BUTTON1));
     Serial.print(digitalRead(P_BUTTON2));
     Serial.print(digitalRead(P_BUTTON3));
     Serial.print(digitalRead(P_BUTTON4));
     Serial.println();
     delay(200);
  }
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//
// SKETCH SETUP
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void setup()
{
  ledInit();
  midiInit();
  keyboardInit();   
  buttonsInit();
  tiltInit();
  appStandBy = 1;
#ifdef DEBUG
  Serial.println("Begin...");
  keyboardTest();
//buttonsTest();
// tiltTest();
//  ledTest();
#endif
}

// LED colours to indicate octave
unsigned long appOctaveCol[5] = {
  COL_DIMGREEN, 
  COL_GREEN, 
  COL_DIMWHITE, 
  COL_BLUE, 
  COL_DIMBLUE
};


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//
// SKETCH RUN
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void loop()
{
  // poll buttons
  unsigned long ulMilliseconds = millis();  
  buttonsRun(ulMilliseconds);
  
  // standby mode?
  if(appStandBy)
  {
    // show dim red led
    setLed(COL_DIMRED);
  }
  else
  {
    // poll keyboard
    keyboardRun(ulMilliseconds);
    
    // decide the status led colour
    unsigned long col1 = appOctaveCol[keyboardOctave];
    unsigned long col2 = col1;
    if(tiltActive)
    {
      // run the tilt state machine if needed
      col1 = COL_MAGENTA;    
      tiltRun(ulMilliseconds);
    }
    
    // any notes held?
    if(keyboardNotesHeld)    
    {
      col2 = COL_YELLOW;    
    }
    
    // update the led
    ledFlash(ulMilliseconds, col1, col2, 200);
  }
}


