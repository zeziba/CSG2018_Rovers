/*
  The purpose of this module is to allow the co-processor for the Rover to
  gather the necessary data for the ML algorithm to make decisions with as
  Little load on the primary CPU as possible.

  This module collects data from several sensors and the outputs it over serial
  And is collected by a python script and stored into a database for later use
  And analysis.
 */

#include <Arduino.h>
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>
#include <SparkFunMPL3115A2.h>
#include <Adafruit_Si7021.h>
#include <XBee.h>
#include <SoftwareSerial.h>

// PI constant - overrides PI constant
#define M_PI 3.14159265358979311599796346854418516159057617188
// END Pi

// Wire code
bool alive = false;
uint8_t buffer[512];
// END Wire code

//////////////////////////////////
// xBee Code
SoftwareSerial xb = SoftwareSerial(6, 5);

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
int resetRSSI = -1000;    //The value that RSSI is reset to after each pass through filter
#define samples 110
int temp, smoothData, rawData;
int timeToScan = 2000;
short currentHeading;

//Variable for i2c comms
uint8_t currHeadingI2c[2];

//Structure to contain the readings from the beacon
struct{
  float heading;
  int signalStrength;
} readings[samples];

//Union for converting between byte[4] and float
union{
  float f;
  uint8_t b[4];
} heading_converter;
// END of xBee CODE
//////////////////////////////////

// Timers to run different FUNCTIONS
uint16_t compass = 2500;
uint8_t detection = 120;
unsigned long last_check_compass;
unsigned long last_check_detection;
// END of Timers

// The following number is pushed to the serial, it is used to clean the data
// So choose wisly
int negValue = -1000;

// The below variables are used to control the serial communication speed
// the speed indicates the desired number of communications per second
double serialCommSpeed = 1000 / 10;
long serialCommTimer = 0;

// The register to store the data of the last measurements
// delete the measurements as they are taken
struct dataReg {
  double x, y, z;
  double heading, altitude;
  float temp1, temp2;
  float pressure, humidity;
  int bHeading;
} ;

dataReg reg;

// The below is used by the loop to update the current machine time.
// This is so that the millis function only has to be called once per loop
long now = 0;

// Create a timer and interval for events to be taken.
long HMC5883timer = 0;
// Take 1 second divided by the hz rate od the HMC5883 to get the min time
// between measurments
int HMC5883delay = (int)(1000.0 / 15.0);
// Declination angle and variable to hold adjusment Value
float declination = 0.22;
float HMC5883adj = 0;

/* Assign a unique ID to this sensor at the same time */
// Raw Data will be collected by microprocessor and offloaded to
// control unit to be processed.
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Function taken directly from Adafruit_HMC5883_Unified
void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

float getHeading(double x, double y, double z) {
  float heading = atan2(x, y);

  heading += HMC5883adj + declination;

  if (heading < 0)
    heading += 2 * M_PI;
  if (heading >= 2 * M_PI)
    heading -= 2 * M_PI;

  heading *= 180 / M_PI;

  return heading;
}

//Create an instance of the pressure sensor
MPL3115A2 pressure;

// Timer and delay for pressure sensor
long MPL3115A2timer = 0;
int MPL31152A2delay = 1000 / 2;

// Init the si7021 sensor
Adafruit_Si7021 Si7021 = Adafruit_Si7021();

// Si7021 timer and delay
long Si7021timer = 0;
int Si7021delay = 1000 / 1;

String regToString(dataReg *regIn) {
  String out = "";

  out += (String)regIn->x;
  regIn->x = (double)negValue;
  out += ',';
  out += (String)regIn->y;
  regIn->y = (double)negValue;
  out += ',';
  out += (String)regIn->z;
  regIn->z = (double)negValue;
  out += ',';
  out += (String)regIn->heading;
  regIn->heading = (double)negValue;
  out += ',';
  out += (String)regIn->temp1;
  regIn->temp1 = (double)negValue;
  out += ',';
  out += (String)regIn->temp2;
  regIn->temp2 = (double)negValue;
  out += ',';
  out += (String)regIn->humidity;
  regIn->humidity = (double)negValue;
  out += ',';
  out += (String)regIn->pressure;
  regIn->pressure = (double)negValue;
  out += ',';
  out += (String)regIn->altitude;
  regIn->altitude = (double)negValue;
  out += ',';
  out += (String)regIn->bHeading;
  regIn->bHeading = (double)negValue;

  return out;
}

// LOCAL FUNCTIONS
void setupWire()
{
  Wire.begin();
  // Uncomment if unable to connect to I2C network - the line overrides bit rate
  // Wire.setClock(100000L);
  for (uint8_t i =0; i < (sizeof(buffer)/sizeof(buffer[0])); i++)
    buffer[i] = 0;
  alive = true;
}

void shutDownWire()
{
  // Used to lower power consumption
  Serial.println("Shutting down wire.");
  Wire.end();
  alive = false;
  Serial.println("Finsihed shutting down wire.");
}

void setupXBEE()
{
  Serial.println("Seting up xBee device.");
  xb.begin(57600);
  xbee.setSerial(xb);
  Serial.println("Finished seting up xBee device.");
}

void Retrieve(int i){
  xbee.readPacket(10);    //Wait 50 to receive packet
  if (xbee.getResponse().isAvailable())     //Execute only if packet found
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE)
    {
      xbee.getResponse().getRx16Response(rx16);
      //Store the transmitted data and RSSI
      for(int i = 0; i<4; i++)
        heading_converter.b[i] = rx16.getData(i);
      int currentRSSI = -rx16.getRssi();

      //Write to array
      readings[i].heading = heading_converter.f;
      readings[i].signalStrength = currentRSSI;
    }
  }else{
    readings[i].heading = 0;
    readings[i].signalStrength = resetRSSI;
  }
}

int ProcessData(){
  int maxRSSI;
  unsigned long maxIndex = 0;  // THIS LINE DOES NOTHING
  maxRSSI = readings[0].signalStrength;

  //Find max RSSI value
  for (int i=1; i < samples; i++) {
    if (maxRSSI < readings[i].signalStrength) {
      maxRSSI = readings[i].signalStrength;
      maxIndex = i;
    }
  }
  //If there is no valid data
  if(maxRSSI == resetRSSI){
    return -1;
  }

  float headingx = 0;
  float headingy = 0;
  for(int i = 0; i < samples; i++)
  {
    if (readings[i].signalStrength == -1000 && readings[i].heading == 0)
    {
       Serial.println("this heading not included");
    }
    else
    {
      // Set magnitude of vector by signal strength
      headingx += readings[i].signalStrength * cos(readings[i].heading * PI / 180);
      headingy += readings[i].signalStrength * sin(readings[i].heading * PI / 180);
    }
  }

  float heading = atan2(headingy, headingx);
  if (heading < 0) heading += 2 * PI;
  heading = heading * 180 / PI;

  return (int) heading;
}

void getSamples()
{
  for(int i = 0;i<samples;i++){
    Retrieve(i);
    float propComplete = ((float)i)/(float)samples; // this line does NOTHING
    delay(timeToScan/samples);
  }
}
// END LOCAL FUNCTIONS

void setup() {
    // Start the Serial line at 115200 bits per second
    Serial.begin(115200);

    while (!Serial)
      delay(10);

    Serial.println("Init of HMC5883 Magnetometer");

    if (!mag.begin())
    {
      Serial.println("Failed to connect to the HMC5883");
      while (1);
    }

    displaySensorDetails();
    Serial.println("Finished Init of HMC5883 Magnetometer");

    Serial.println("Init of MPL3115A2");

    pressure.begin();
    pressure.setModeAltimeter();

    pressure.setOversampleRate(7);

    pressure.enableEventFlags();

    Serial.println("Finished Init of MPL3115A2");

    Serial.println("Init of Si7021");

    if (!Si7021.begin()) {
      Serial.println("Failed to start Si721");
      while (1);
    }

    Serial.println("Finished Init of Si7021");

    Serial.println("Start Init of XBee");

    setupXBEE();

    Serial.println("Finished Init of XBee");

    delay(1000);

    HMC5883timer = millis();
    MPL3115A2timer = HMC5883timer;
    Si7021timer = MPL3115A2timer;
    serialCommTimer = Si7021timer;
    last_check_compass = serialCommTimer;
    last_check_detection = last_check_compass;
}

void loop() {
    now = millis();
    if (now - HMC5883timer > HMC5883delay) {
      sensors_event_t event;
      mag.getEvent(&event);
      /* Display the results (magnetic vector values are in micro-Tesla (uT)) */

      reg.x = event.magnetic.x;
      reg.y = event.magnetic.y;
      reg.z = event.magnetic.z;
      reg.heading = getHeading(event.magnetic.x, event.magnetic.y, event.magnetic.z);

      HMC5883timer = now;
    }

    if (now - MPL3115A2timer > MPL31152A2delay) {
      reg.pressure = pressure.readTempF();
      reg.altitude = pressure.readAltitudeFt();
      reg.temp1 = pressure.readTemp();

      MPL3115A2timer = now;
    }

    if (now - Si7021timer > Si7021delay) {
      reg.temp2 = Si7021.readTemperature();
      reg.humidity = Si7021.readHumidity();

      Si7021timer = now;
    }

    if (now - last_check_compass > compass)
    {
      getSamples();
      last_check_compass = millis();
      currentHeading = ProcessData();
    }

    if (millis() - serialCommTimer > serialCommSpeed) {
      Serial.println(regToString(&reg));

      serialCommTimer = millis();
    }
}
