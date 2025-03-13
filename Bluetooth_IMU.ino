/*
 * This example demonstrates:
 *  - A Bluetooth SPP bridge between Serial and Bluetooth
 *  - Initialization and periodic reading of an MPU-9265 IMU using the MPU9250_asukiaaa library.
 *
 * Make sure to install the MPU9250_asukiaaa library in your Arduino IDE.
 */

#include "BluetoothSerial.h"
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it.
#endif

BluetoothSerial SerialBT;
MPU9250_asukiaaa imu; // Create an IMU object

void setup() {
  // Initialize Serial and Bluetooth
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with Bluetooth!");

  // Initialize I2C (Wire) interface
  Wire.begin();

  // Configure the IMU with the I2C interface
  imu.setWire(&Wire);
  imu.beginAccel(); // Initialize accelerometer
  imu.beginGyro();  // Initialize gyroscope
  imu.beginMag();   // Initialize magnetometer

  Serial.println("IMU initialized.");
}

void loop() {
  // Bridge data between Serial and Bluetooth
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  // Update IMU readings
  imu.accelUpdate();
  imu.gyroUpdate();
  imu.magUpdate();

  // Read and print accelerometer data (in m/s^2)
  Serial.print("Accel (m/s^2): ");
  Serial.print(imu.accelX());
  Serial.print(", ");
  Serial.print(imu.accelY());
  Serial.print(", ");
  Serial.println(imu.accelZ());



  Serial.println("------------------------------");

  delay(500); // Adjust delay as needed
}
