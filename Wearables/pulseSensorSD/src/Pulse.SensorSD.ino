#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define PROCESSING_VISUALIZER 1
#define SERIAL_PLOTTER  2
#define tempPinOut A5
#define sensorPin A6
#define debug true
#define ALLOWVISUALIZER false

//  Variables
int pulsePin = A1;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin
float tVoltage = 1.75;            // Max output voltage of the Temp sensor
float cModTempVolt = 0.53;
bool header = false;
float temps[20] = {};
int t_i = 0;
int n = 0;
char logFileName[16] = {'d', 'a', 't', 'a', '0', '.', 'c', 's', 'v'};

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

static int outputType = SERIAL_PLOTTER;

uint32_t time = millis();

File data;

// Adafruit GPS code
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  true

SoftwareSerial mySerial(3, 4);

Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
#endif
}

uint32_t timer_0 = millis();

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
// End Adafruit GPS

float avg_Temp(float tempArray[]) {
  float total = 0;
  int i = 0;
  for (i; i < 20;)
    total += tempArray[i++];

  return total / (float)i;
}

float map_(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void debugPrint(float voltage, float celsius, float fahrenheit, float BPM, float IBI, float Signal)
{
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" ");
  Serial.print("Celsius: ");
  Serial.print(celsius);
  Serial.print(" ");
  Serial.print("Fahrenheit: ");
  Serial.print(fahrenheit);
  Serial.print(" ");
  Serial.print("BPM: ");
  Serial.print(BPM);
  Serial.print(" ");
  Serial.print("IBI: ");
  Serial.print(IBI);
  Serial.print(" ");
  Serial.print("Signal: ");
  Serial.println(Signal);
}

void save_data(float BPM, float IBI, float Signal, float voltage, float celsius, double fahrenheit) {
  data = SD.open(logFileName, FILE_WRITE);

  if (!header)
  {
    data.print("Beats Per Minute");
    data.print(',');
    data.print("IBI");
    data.print(',');
    data.print("Signal");
    data.print('.');
    data.print("voltage");
    data.print(',');
    data.print("celsius");
    data.print(',');
    data.println("fahrenheit");
    header = true;
  }


  data.print(BPM);
  data.print(',');
  data.print(IBI);
  data.print(',');
  data.print(Signal);
  data.print(',');
  data.print(voltage);
  data.print(',');
  data.print(celsius);
  data.print(',');
  data.println(fahrenheit);

  data.close();
}

void setup(){
  pinMode(sensorPin, INPUT);
  pinMode(tempPinOut, OUTPUT);
  digitalWrite(tempPinOut, HIGH);

//  Serial.begin(9600);

  pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  Serial.begin(115200);             // we agree to talk fast!
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS
   // IF YOU ARE POWERING The Pulse Sensor AT VOLTAGE LESS THAN THE BOARD VOLTAGE,
   // UN-COMMENT THE NEXT LINE AND APPLY THAT VOLTAGE TO THE A-REF PIN
  //analogReference(EXTERNAL);
  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  Serial.println("Locating new file.");
  int n = 0;
  while (SD.exists(logFileName))
    sprintf(logFileName, "data%d.csv", ++n);
  data = SD.open(logFileName, FILE_WRITE);

  if (data) {
    Serial.print(logFileName);
    Serial.println(" exists and is now closed");
    data.close();
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(logFileName);
  }

  delay(2500);    // Delay to allow everything to start properly.

}


//  Where the Magic Happens
void loop(){
  long rawTemp;
  float voltage;
  float fahrenheit;
  float celsius;

  rawTemp = analogRead(sensorPin);
  voltage = rawTemp / 1024.0 * tVoltage;
  if (voltage - cModTempVolt > 0)
    celsius = map_(voltage - cModTempVolt, 0.0, tVoltage, 0.0, 125.0);
  else
    celsius = map_(voltage - cModTempVolt, -cModTempVolt, 0.0, 0.0, -40.0);
  temps[t_i++] = celsius;
  if (t_i > 20)
    t_i = 0;
  celsius = avg_Temp(temps);
  fahrenheit = (celsius * 9.0 / 5.0) + 32.0;
  delay(1000);

  if (ALLOWVISUALIZER)
    serialOutput();

  if (QS == true){     // A Heartbeat Was Found
                       // BPM and IBI have been Determined
                       // Quantified Self "QS" true when arduino finds a heartbeat
        if (ALLOWVISUALIZER)
          serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
        QS = false;
        save_data(BPM,IBI,Signal,voltage, celsius,fahrenheit);
  }

  if (debug)
    debugPrint(voltage, celsius, fahrenheit, BPM, IBI, Signal);

  delay(400);                             //  take a break


}
