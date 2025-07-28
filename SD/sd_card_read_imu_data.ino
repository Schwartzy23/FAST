#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define SD_CS 5

File myFile;

float Ax, Ay, Az;

void A_data() { // fn. that gets accleration data from IMU
  Wire.beginTransmission(0x68); // need this line before any commands to IMU
  Wire.write(0x1D); // Wrinting to register 1D (29) - Register used to turn on low pass filter for acceleration values
  Wire.write(0x05); // Sets low pass filter bandwidth to 10.2 Hz for accelerometer (see table)
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Write to register 3B (59 in decimal) - This and the following 5 1-byte registers store acquired acceleration data
  Wire.endTransmission();

  Wire.requestFrom(0x68,6); // Request 6 bytes from the first acceleration data register

  int16_t Ax_meas = Wire.read() << 8 | Wire.read(); // First two bytes are Ax. "<<8" shifts the first 8 bits to the left, and the second "Wire.read()" fills in the least significant byte.
  int16_t Ay_meas = Wire.read() << 8 | Wire.read(); // Same but for y
  int16_t Az_meas = Wire.read() << 8 | Wire.read(); // Same but for z

  // Convert to g's (1g = 16384 according to MPU-9250 datasheet for ±2g range)
  Ax = (float)Ax_meas / 16384.0;
  Ay = (float)Ay_meas / 16384.0;
  Az = (float)Az_meas / 16384.0;

   Serial.begin(9600); // Set serial baud rate
  Wire.setClock(400000); // Set internal IMU clock
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Write to power management register
  Wire.write(0x00); // Set to default, "power mode"
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  delay(1000);
  Serial.println("Program Setup");  
  while(!Serial); // Wait to initialize serial monitor


  Serial.println("Initializing SD Card...");

  if (!SD.begin(SD_CS)) { //Initialize SD card at CS pin 5}
    Serial.println("Initialization failed");
    return;
  }
  Serial.println("Initialization done.");
}


void loop() {
  // Get both acceleration and gyroscope data
  A_data();

  // Send acceleration data in comma-separated format
  Serial.print(Ax);
  Serial.print(",");
  Serial.print(Ay);
  Serial.print(",");
  Serial.println(Az);


  Serial.println("Writing to imuv2.txt");
  myFile = SD.open("/imuv2.txt", FILE_WRITE);
  if (myFile) {
    Serial.println("imuv2.txt:");
    myFile.print(Ax);
    myFile.print(", ");
    myFile.print(Ay);
    myFile.print(", ");
    myFile.println(Az);
    myFile.close();
    Serial.println("done.");

  } else {
    Serial.println("error opening imuv2.txt");
  }


  Serial.println("Now reading from imuv2.txt");
  // re-open file for reading
  myFile = SD.open("/imuv2.txt");
  if (myFile) {
      Serial.println("imuv2.txt:");

      while (myFile.available()) {
        Serial.write(myFile.read());
      }
      myFile.close();
  } else {
    Serial.println("error opening imuv2.txt"); 
  }

  delay(50);  // 50ms delay gives us 20 samples per second
}

