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

char inByte;

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

void goForward(Stepper *stepper, const int speed, const int step) {
  stepper->setSpeed(speed);
  stepper->step(step);
}

void goBackwards(Stepper *stepper, const int speed, const int step) {
  stepper->setSpeed(speed);
  stepper->step(step);
}

void receiveEvent(int bytes){
  inByte = Wire.read(); // receive byte as a character
}

void doStuff(int distance) {
  if (inByte == (char)FWD)
    goForward(&stepper1, STEPS, distance);
  else
    goBackwards(&stepper1, STEPS, distance);
  Serial.print("I did stuff: ");
  Serial.println();
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("Begin Init.");
  Wire.begin(SLAVEID);
  Wire.onReceive(receiveEvent);

  delay(1000);
  Serial.println("End Init, Success");
  Serial.end();
}

void loop()
{
  doStuff(400);
}
