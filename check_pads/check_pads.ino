
// pins
#define P_LATCH 8
#define P_CLK   7
#define P_DATA  6

#define P_PWM_RED    9
#define P_PWM_GREEN  11
#define P_PWM_BLUE   10

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

char *decodeKey(int i)
{
  switch(i)
  {
case KEY_A: return "A(1)"; 
case KEY_ASHARP: return "A#(1)"; 
case KEY_B: return "B(1)"; 
case KEY_C: return "C(1)"; 
case KEY_CSHARP: return "C#(1)"; 
case KEY_D: return "D(1)"; 
case KEY_DSHARP: return "D#(1)"; 
case KEY_E: return "E(1)"; 
case KEY_F: return "F(1)"; 
case KEY_FSHARP: return "F#(1)"; 
case KEY_G: return "G(1)"; 
case KEY_GSHARP: return "G#(1)"; 
case KEY_A2: return "A(2)"; 
case KEY_ASHARP2: return "A#(2)"; 
case KEY_B2: return "B(2)"; 
case KEY_C2: return "C(2)"; 
case KEY_CSHARP2: return"C#(2)"; 
case KEY_D2: return "D(2)"; 
case KEY_DSHARP2: return "D#(2)"; 
case KEY_E2: return "E(2)"; 
  }
  return "???";    
}

/////////////////////////////////////////////////////////////////
// INITIALISE STYLOPHONE KEYBOARD INPUT
void keyboardInit()
{
  // configure the shift register control lines
  pinMode(P_LATCH, OUTPUT);
  pinMode(P_CLK, OUTPUT);
  pinMode(P_DATA, INPUT);
  
   pinMode(P_PWM_RED, OUTPUT);
   pinMode(P_PWM_GREEN, OUTPUT);
   pinMode(P_PWM_BLUE, OUTPUT);
   
  // pull up resistors on inputs
  digitalWrite(P_DATA,HIGH);  
  digitalWrite(P_LATCH,HIGH);
  
}


/////////////////////////////////////////////////////////////////
// TEST KEYBOARD
void keyboardTest()
{
  int q=0;
  byte states[24] = {0};
  for(;;)
  {
    digitalWrite(P_PWM_BLUE, !(q&0x200));
    q++;
    
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
          Serial.print("/");
          Serial.print(decodeKey(i));
          Serial.print(" =");
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

void setup()
{
  Serial.begin(9600);
  Serial.println("Begin...");
  keyboardInit();   
  keyboardTest();
}

void loop()
{
}


