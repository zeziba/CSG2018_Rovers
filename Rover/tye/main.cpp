#include <Stepper.h>
#include <Wire.h>
#include <Arduino.h>

#define SLAVEID 0x1
#define FWD 1
#define REV 0

#define STEPPERCOILA1 4
#define STEPPERCOILA2 5
#define STEPPERCOILB1 6
#define STEPPERCOILB2 7

#define ASEN A1
#define BSEN A2

char inByte = REV;

const int STEPS = 400;
const int microSteps = 2;


int revolutions = 7;
int motorRPM = 110;
int testRPM = 45;
int distance = microSteps * STEPS * revolutions;
int withoutSteps = STEPS * revolutions;
int turningDistance = STEPS / 3 ;

//Stepper stepper1(STEPS, microSteps, 4, 5, 6, 7);
//Stepper stepper2(STEPS, microSteps, 9,10,11,12);

Stepper stepper1(STEPS, STEPPERCOILA1, STEPPERCOILA2, STEPPERCOILB1, STEPPERCOILB2);

double map_(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void goForward(Stepper *stepper, const int spd, const int stp) {
  stepper->setSpeed(spd);
  stepper->step(stp);
}

void goBackwards(Stepper *stepper, const int spd, const int stp) {
  stepper->setSpeed(spd);
  stepper->step(stp);
}

void receiveEvent(int bytes){
  inByte = Wire.read(); // receive byte as a character
}

void doStuff(int distance) {
  if (inByte == (char)FWD)
    goForward(&stepper1, motorRPM, distance);
  else
    goBackwards(&stepper1, motorRPM, distance);
  Serial.print("I did stuff: ");
  Serial.print(map_(analogRead(ASEN), 0, 1023, 0, 5));
  Serial.print(" ");
  Serial.println(map_(analogRead(BSEN), 0, 1023, 0, 5));
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("Begin Init.");
  Wire.begin(SLAVEID);
  Wire.onReceive(receiveEvent);

  pinMode(ASEN, INPUT);
  pinMode(BSEN, INPUT);

  delay(1000);
  Serial.println("End Init, Success");
  //`Serial.end();
}

void loop()
{
  doStuff(400);
}
