#define P_BUTTON1  3
#define P_BUTTON2  18
#define P_BUTTON3  19
#define P_BUTTON4  2

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
   
}

void buttonsTest() 
{
  for(;;)
  {
     Serial.print(digitalRead(P_BUTTON3)? "--- ": "-A- ");
     Serial.print(digitalRead(P_BUTTON4)? "--- ": "-B- ");
     Serial.print(digitalRead(P_BUTTON2)? "--- ": "-C- ");
     Serial.print(digitalRead(P_BUTTON1)? "--- ": "-D- ");
     Serial.println();
     delay(200);
  }
}

void setup()
{
  Serial.begin(9600);
  buttonsInit();
  buttonsTest();
}

void loop()
{
}


