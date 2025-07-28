// Video this code follows:  https://www.youtube.com/watch?v=yhz3bRQLvBY&t=92s
// **Use "MPU-9250 Register Map and Descriptions Revision 1.6" for info on the IMU

#include <Wire.h> // Library for using I2C

float Ax, Ay, Az;
float Gx, Gy, Gz;

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
}

void Gyr_data() { // fn. that gets gyroscope data from IMU
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // Write to register 26 - 
  Wire.write(0x05); // Sets low pass filter bandwidth to 10.2 Hz for gyroscope (see table)
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Write to Register 27 (Gyroscope Configuration) - sets Gyroscope scale
  Wire.write(0x8); // Scale currently set to +500 degrees/s
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Write to Register 67 - First register of 6 registers that contain gyroscope data
  Wire.endTransmission();

  Wire.requestFrom(0x68,6); // Request 6 bytes (Registers 67,68,69,70,71,72) getting x,y,z gyro data

  int16_t Gx_meas = Wire.read() << 8 | Wire.read();
  int16_t Gy_meas = Wire.read() << 8 | Wire.read();
  int16_t Gz_meas = Wire.read() << 8 | Wire.read();

  // Convert to degrees/s (65.5 according to MPU-9250 datasheet for ±500°/s range)
  Gx = (float)Gx_meas / 65.5;
  Gy = (float)Gy_meas / 65.5;
  Gz = (float)Gz_meas / 65.5;
}

void setup() {
  Serial.begin(9600); // Set serial baud rate
  Wire.setClock(400000); // Set internal IMU clock
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Write to power management register
  Wire.write(0x00); // Set to default, "power mode"
  Wire.endTransmission();
}

void loop() {
  // Get both acceleration and gyroscope data
  A_data();
  Gyr_data();

  // Send acceleration data in comma-separated format
  Serial.print(Ax);
  Serial.print(",");
  Serial.print(Ay);
  Serial.print(",");
  Serial.println(Az);

  delay(50);  // 50ms delay gives us 20 samples per second
}
