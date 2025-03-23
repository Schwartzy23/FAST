#include <SPI.h>
#include <SD.h>



#define SD_CS 5



File myFile;

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

  Serial.println("Writing to test.txt");
  myFile = SD.open("/test.txt",FILE_WRITE);
  if (myFile) {
    Serial.println("test.txt:");
    myFile.println("testing.");
    myFile.close();
    Serial.println("done.");

  } else {
    Serial.println("error opening test.txt");
  }


  Serial.println("Now reading from test.txt");
  // re-open file for reading
  myFile = SD.open("/test.txt");
  if (myFile) {
      Serial.println("test.txt:");

      while (myFile.available()) {
        Serial.write(myFile.read());
      }
      myFile.close();
  } else {
    Serial.println("error opening test.txt"); 
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("Program started"); 

}
