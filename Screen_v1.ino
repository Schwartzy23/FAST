#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// Define the pins for your display
#define TFT_CS     14
#define TFT_RST    12
#define TFT_DC     13


// Create an instance of the ST7789 display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);

  // Initialize the TFT display with a resolution of 240x240 pixels (adjust if necessary)
  tft.init(240, 280);  // Set this based on your screen's resolution

  // Optionally, set the screen rotation (0-3)
  tft.setRotation(3);  // Adjust rotation as per your screen orientation

  // Fill the screen with a color (optional)
  tft.fillScreen(ST77XX_BLACK);

  // Set text color and size
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  // Display a message on the screen
  tft.setCursor(20, 50);
  tft.println("Hello, ST7789!");


}

void loop() {
  // Optional: Add dynamic content here if necessary
  delay(1000);  // Optional delay
}
