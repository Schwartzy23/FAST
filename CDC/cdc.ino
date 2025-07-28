

#include <Wire.h> // Library for using I2C

float C;

void C_data() { // fn. that gets data from CDC
  Wire.beginTransmission(0x2A); // need this line before any commands to cdc
  Wire.write(0x00); // Register address for DATA_CH0
  Wire.endTransmission();


  Wire.requestFrom(0x2A,4); // Request 4 bytes
  if (Wire.available() == 4) {
    uint32_t rawData = Wire.read() << 24 | Wire.read() << 16 | Wire.read() << 8 | Wire.read();
    C = (rawData);  // Convert rawData to capacitance
}



void setup() {
  Serial.begin(9600); // Set serial baud rate
  
  Wire.begin();
  delay(250);

}

void loop() {
  // Get cdc data
  C_data();

  // Send cdc data 
  Serial.print(C);
  Serial.println(C);

  delay(50);  // 50ms delay gives us 20 samples per second
}
