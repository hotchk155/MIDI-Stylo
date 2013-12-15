
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
// TEST LEDS
void ledTest() 
{
  for(;;)
  {
    Serial.println("Red");
    setLed(COL_RED); delay(1000); 
    setLed(0); delay(200);     
    setLed(COL_RED);   delay(1000); 
    setLed(0); delay(200);     
    
    Serial.println("Green");
    setLed(COL_GREEN); delay(1000);
    setLed(0); delay(200);     
    
    Serial.println("Blue");
    setLed(COL_BLUE); delay(1000);
    setLed(0); delay(200);     
  }
}

void setup()
{
  ledInit();
  ledTest();
}

void loop()
{
}


