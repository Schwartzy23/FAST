#include <Wire.h>
#include <MPU9250.h>  // Make sure you have the correct library for your sensor
#include <BluetoothSerial.h>


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

  Ax = (float)Ax_meas; // Changing variable type, may need some kind of scale factor here
  Ay = (float)Ay_meas;
  Az = (float)Az_meas;
}




String device_name = "ESP32-BT-Slave";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif


// Bluetooth Serial object
BluetoothSerial SerialBT;

// Create MPU6050 object
MPU9250 mpu;

// Variables to store IMU data










void setup() {



  delay(250);








  // Start Serial communication for debugging
  Serial.begin(115200);
  
  // Start Bluetooth Serial communication

  SerialBT.begin(device_name);  //Bluetooth device name
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  
  // Initialize I2C communication
  Wire.setClock(400000); // Set internal IMU clock
  Wire.begin();

  // Initialize MPU6050 sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Write to power management register
  Wire.write(0x00); // Set to default, "power mode"
  Wire.endTransmission();


}












void loop() {
  // Check if Bluetooth is available
  if (SerialBT.available()) {
    // Send a welcome message
    SerialBT.println("Bluetooth Serial connected! Receiving data...");
    A_data();
    //Serial.println(Ax);
    // Serial.println(Ay);
    //Serial.println(Az);
    delay(50);

    String data = String("Accelz: ") + String(Az);   // Format IMU data as a string
    
    SerialBT.println(data);   // Send the data over Bluetooth Serial

    Serial.println(data);  // Also print data to Serial Monitor for debugging

    delay(100);  // Delay to limit how often data is sent


  }
}

