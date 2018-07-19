#include <SD.h>

File dataFile;
int sdDataPin = 4;
char logFileName[16];
String.toCharArray("data0.csv", 16);
bool header = false;

uint32_t timer_1;

void writeSD(uint32_t time, float temp1, float temp2, float humdity, float pressure,
	float accelX, float accelZ) {
	
	dataFile = SD.open(logFileName, FILE_WRITE);
	
	if (!header) {
		dataFile.print("Time, Temp1, Temp2, Hunidity, Pressure, AccelX, AccelZ");
		header = true;
	}
	
	dataFile.print(time / 1000.0);
	dataFile.print(",");
	dataFile.print(temp1);
	dataFile.print(",");
	dataFile.print(temp2);
	dataFile.print(",");
	dataFile.print(humdity);
	dataFile.print(",");
	dataFile.print(pressure);
	dataFile.print(",");
	dataFile.print(accelX);
	dataFile.print(",");
	dataFile.println(accelZ);
	
	dataFile.close();
}

void setup() {
	// Turn on one LED here
	Serial.begin(9600);
	
	Serial.println("Attempting to initilize SD card.");
	if (!SD.begin(sdDataPin)) {
		Serial.println("Failed to initilize");
		while (1);
	}
	Serial.println("Finished Initilization.");
	
	// Turn on second LED here
	
	Serial.println("Attempting to open file on SD card.");
	
	int n = 0;
	while (SD.exists(logFileName))
		sprintf(logFileName, "data%d.csv", ++n)
	dataFile = SD.open(logFileName, FILE_WRITE);
	
	// TURN on third LED here
	
	if (data)
		Serial.println("Created file " + (String)logFileName);
	else {
		Serial.println("Failed to create a file, check SD card to ensure proper formatting.");
		while (1);
	}
	
	dataFile.close();
	
	for (int i = 0; i < 5; i++) {
		// LED on Here
		delay(250);
		// LED off Here
		delay(250);
	}
	
	timer_1 = millis();
	
	Serial.println("All system go.");
	// Turn on final two LED here
}

void loop() {
	now = millis();
	// Turn off status LED
	if (now - timer_1 > 1000) {
		// TURN on Status LED for writing/experiments
		// Conduct all experiments
		// Conduct SD write
		timer_1 = now;
	}
}

