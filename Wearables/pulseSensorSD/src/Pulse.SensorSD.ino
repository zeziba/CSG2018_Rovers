#include <SPI.h>
#include <SD.h>

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
float cModTempVolt = 0.55;
bool header = false;

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

static int outputType = SERIAL_PLOTTER;

uint32_t timer = millis();

File data;

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
  data = SD.open("data.csv", FILE_WRITE);

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

  data = SD.open("data.csv", FILE_WRITE);

  if (data) {
    Serial.println("data.csv exists and is now closed");
    data.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening data.csv");
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
    celsius = map_(voltage - cModTempVolt, 0.0, 1.75, 0.0, 125.0);
  else
    celsius = map_(voltage - cModTempVolt, -cModTempVolt, 0.0, 0.0, -40.0);
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
