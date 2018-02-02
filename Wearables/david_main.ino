#include <Arduino.h>
//include <U8g2lib.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>

SoftwareSerial mySerial(0, 1);
Adafruit_GPS GPS(&mySerial);

// #ifdef U8X8_HAVE_HW_SPI
// #include <SPI.h>
// #endif
// #ifdef U8X8_HAVE_HW_I2C
// #include <Wire.h>
// #endif

#define GPSECHO  true

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* reset=*/ U8X8_PIN_NONE);
//typedef u8g2_uint_t u8g_uint_t;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#define SECONDS 10
uint8_t flip_color = 0;
uint8_t draw_color = 1;

// void show_result(const char *s) {
//   // assign default color value
//   u8g2.setColorIndex(draw_color);
//   u8g2.setFont(u8g2_font_8x13B_tf);
//   u8g2.firstPage();
//   do {
//     u8g2.drawStr(0,12, s);
//   } while( u8g2.nextPage() );
// }

uint32_t timer = millis();

File data;

void save_data(float lng, float lat, double spd, double alt, double angle, int sats) {
  data = SD.open("data.csv", FILE_WRITE);

  data.print(lng);
  data.print(',');
  data.print(lat);
  data.print(',');
  data.print(spd);
  data.print(',');
  data.print(sats);
  data.print(',');
  data.print(alt);
  data.print(',');
  data.println(angle);

  data.close();
}

void setup() {
  // put your setup code here, to run once:

  // u8g2.begin();
  // flip screen, if required
  // u8g2.setRot180();

  Serial.begin(115200);
  delay(5000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(250);
  display.clearDisplay();

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
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

  // show_result("Adafruit GPS library basic test!");

  // assign default color value
  draw_color = 1;         // pixel on

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

void loop() {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    int fix, quality;
    fix = (int)GPS.fix;
    quality = (int)GPS.fixquality;

    if (fix) {
      float lng, lat;
      lng = GPS.longitude;
      lat = GPS.latitude;

      double spd = GPS.speed;
      double alt = GPS.altitude;
      double angle = GPS.angle;
      int sats = (int)GPS.satellites;

      display.setTextSize(1);
      display.setCursor(0,0);
      display.setTextColor(WHITE);
      display.clearDisplay();

      display.print("Long: ");
      display.println(lng);

      display.print("Lat: ");
      display.println(lat);

      display.print("Speed: ");
      display.print(spd);
      display.println("m/s");

      display.print("Satelites: ");
      display.println(sats);

      display.print("Altitude");
      display.print(alt);
      display.print(" Angle: ");
      display.println(angle);

      Serial.print(lng);
      Serial.print(',');
      Serial.print(lat);
      Serial.print(',');
      Serial.print(spd);
      Serial.print(',');
      Serial.print(sats);
      Serial.print(',');
      Serial.print(alt);
      Serial.print(',');
      Serial.println(angle);

      save_data(lng, lat, spd, alt, angle, sats);
    }
  }
}
