/*
 Monster Moto shield firmware for Playseats
 2013.11.14 Joao Coutinho http://b.joaoubaldo.com

 Credits also go to RacingMat at X-Sim forum.
*/


/* Configuration starts here. */

#define RV 1 //beware it's depending on your hardware wiring
#define FW 2 //beware it's depending on your hardware wiring

#define pwmMax 255 // Max speed for both motors
#define calibrationTime 3000 // Milliseconds for calibration
const boolean do_calibrate = 0;

#define motLeft 0 // left motor index
#define motRight 1 // right motor index

#define potL A4
#define potR A5

/* Left pot range and behaviour */
boolean revL = false; // Reverse values read?
int potMiniL=361 ; // Not used if calling calibrate()
int potMaxiL=530; // Not used if calling calibrate()

/* Right pot range and behaviour */
const boolean revR = false;
int potMiniR=420; // Not used if calling calibrate()
int potMaxiR=571; // Not used if calling calibrate()

/* motor will not move if the target position is too close to the actual position */
int toleration = 10;

/* Configuration ends. */

/* VNH2SP30 pin definitions */
int inApin[2] = {
  7, 4}; // INA: Clockwise input
int inBpin[2] = {
  8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {
  5, 6}; // PWM input
int cspin[2] = {
  2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {
  0, 1}; // EN: Status of switches output (Analog pin)
int statpin = 13; //not explained by Sparkfun

int DataValueL=0;
int DataValueR=0;

void calibrate() {
  potMiniL=1025;
  potMaxiL=0;
  potMiniR=1025;
  potMaxiR=0;

  long m= millis();
  motorGo(motLeft, FW, 255);
  motorGo(motRight, FW, 255);
  while ((millis() - m) <= calibrationTime) {
    int l = analogRead(potL);
    int r = analogRead(potR);

    potMiniL = l < potMiniL ? l : potMiniL;
    potMaxiL = l > potMaxiL ? l : potMaxiL;

    potMiniR = r < potMiniR ? r : potMiniR;
    potMaxiR = r > potMaxiR ? r : potMaxiR;
    Serial.print(analogRead(potL));
    Serial.print(" R: ");
    Serial.println(analogRead(potR));
  }
  potMiniL+=1;
  potMiniR+=1;
  potMaxiL-=1;
  potMaxiR-=1;
  Serial.println(potMiniL);
  Serial.println(potMaxiL);

  Serial.println(potMiniR);
  Serial.println(potMaxiR);

}

void setup()
{
  Serial.begin(115200);

  pinMode(statpin, OUTPUT); //not explained by Sparkfun
  digitalWrite(statpin, LOW);

  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  // Initialize braked for motor
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  if (do_calibrate) calibrate();
  // Center motors
  byte l[3] = {
    'L', '3', '3'   };
  byte r[3] = {
    'R', '3', '3'   };

  DataValueL=NormalizeData(l, potMiniL, potMaxiL, revL);
  DataValueR=NormalizeData(r, potMiniR, potMaxiR, revR);

}

void loop()
{
  byte Data[3]={
    '0','0','0'   };

  if (Serial.available() > 2) {
    Data[0]=Serial.read();
    if (Data[0]=='R'){
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      DataValueR=NormalizeData(Data, potMiniR, potMaxiR, revR);
    }
    else if (Data[0]=='L') {
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      DataValueL=NormalizeData(Data, potMiniL, potMaxiL, revL);
    }
    else if (Data[0]=='T') {
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      if (Data[1]=='e' && Data[2]=='l') {
        Serial.print("Current: ");
        Serial.print(analogRead(potL));
        Serial.print(" R: ");
        Serial.println(analogRead(potR));

        Serial.print("L: ");
        Serial.print(potMiniL);
        Serial.print(" ");
        Serial.print(potMaxiL);
        Serial.print(" R: ");
        Serial.print(potMiniR);
        Serial.print(" ");
        Serial.println(potMaxiR);
      }
    }
    /*else if (Data[0]=='C' && Data[1]=='a' && Data[2]=='l') {
     calibrate();
     }*/
  }
  else if (Serial.available() > 16)
    Serial.flush();

  motorMotion(motLeft, analogRead(potL), DataValueL, potMiniL, potMaxiL, toleration);
  delay(10);
  motorMotion(motRight, analogRead(potR), DataValueR, potMiniR, potMaxiR, toleration);

  //toleration = 2 + (int)((analogRead(A0)/1013.0)*11);
}

void motorMotion(int numMot,int actualPos,int targetPos, int potMini, int potMaxi, int tol)
{
  int gap;
  int pwm;
  int brakingDistance=10;

  // security concern : targetPos has to be within the mechanically authorized range
  targetPos=constrain(targetPos,potMini+brakingDistance,potMaxi-brakingDistance);

  gap=abs(targetPos-actualPos);

  if (gap <= tol) {
    motorOff(numMot);
  }
  else {
    /* IMPORTANT section. TODO: improve by configuration */
    pwm=pwmMax;
    if (gap>50) pwm=(int)pwmMax*0.7;
    if (gap>75) pwm=(int)pwmMax*0.85;
    if (gap>100) pwm=(int)pwmMax;

    if ((actualPos<potMini) || (actualPos<targetPos)) motorGo(numMot, FW, pwm);
    else if ((actualPos>potMaxi) || (actualPos>targetPos)) motorGo(numMot, RV, pwm);

  }
}

void motorOff(int motor) { //Brake Ground : free wheel actually
  digitalWrite(inApin[motor], LOW);
  digitalWrite(inBpin[motor], LOW);
  analogWrite(pwmpin[motor], 0);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct== 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);

    }
  }
}

int NormalizeData(byte x[3], int potMini, int potMaxi, boolean rev)
{
  int result;

  if (x[2]==13) //only a LSB and Carrier Return
  {
    x[2]=x[1]; //move MSB to LSB
    x[1]='0'; //clear MSB
  }
  for (int i=1; i<3; i++)
  {
    if (x[i]>47 && x[i]<58 ){//for xA to xF
      x[i]=x[i]-48;
    }
    if ( x[i]>64 && x[i]<71 ){//for x0 to x9
      x[i]=x[i]-55;
    }
  }
  // map the range from Xsim (0 <-> 255) to the mechanically authorized range (potMini <-> potMaxi)
  if (rev) {
    // result=potMini+(potMaxi-map((x[1]*16+x[2]),0,255,potMini,potMaxi));
    result=map(255-(x[1]*16+x[2]),0,255,potMini,potMaxi);
  }
  else
    result=map((x[1]*16+x[2]),0,255,potMini,potMaxi);
  return result;
}

