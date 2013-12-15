#define P_ACC_ENABLE  17
#define P_ACC_X  A2
#define P_ACC_Y  A0
#define P_ACC_Z  A1

void tiltInit()
{
  // set pin modes
   pinMode(P_ACC_ENABLE, OUTPUT);
   pinMode(P_ACC_X, INPUT);
   pinMode(P_ACC_Y, INPUT);
   pinMode(P_ACC_Z, INPUT);

   // enable the accelerometer (must be >1 ms after powerup)
   digitalWrite(P_ACC_ENABLE, LOW);
   delay(10);
   digitalWrite(P_ACC_ENABLE, HIGH);
   
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

void setup()
{
  Serial.begin(9600);
  tiltInit();
  tiltTest();
}

void loop()
{
}


