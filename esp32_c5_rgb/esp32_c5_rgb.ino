#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Starting RGB LED Test...");
}

void loop() {
  // Check if RGB_BUILTIN is defined (standard for boards with built-in Addressable LED)
  #ifdef RGB_BUILTIN
    static uint8_t hue = 0;
    static unsigned long lastLed = 0;
    
    // Update every 25ms for faster transition (100% speed increase)
    if (millis() - lastLed > 25) {
      lastLed = millis();
      hue++;
      
      // Basic Hue to RGB conversion (Rainbow Effect)
      uint8_t wheelPos = 255 - hue;
      uint8_t r = 0, g = 0, b = 0;

      if(wheelPos < 85) {
        // Red decreasing, Blue increasing
        r = 255 - wheelPos * 3;
        g = 0;
        b = wheelPos * 3;
      } else if(wheelPos < 170) {
        wheelPos -= 85;
        // Blue decreasing, Green increasing
        r = 0;
        g = wheelPos * 3;
        b = 255 - wheelPos * 3;
      } else {
        wheelPos -= 170;
        // Green decreasing, Red increasing
        r = wheelPos * 3;
        g = 255 - wheelPos * 3;
        b = 0;
      }

      // Apply consistent brightness dimming (Very low value for testing)
      uint8_t brightness = 128; // 0-255 range. 128 is ~50%.
      uint8_t R_out = (r * brightness) / 255;
      uint8_t G_out = (g * brightness) / 255;
      uint8_t B_out = (b * brightness) / 255;
      
      neopixelWrite(RGB_BUILTIN, R_out, G_out, B_out);
    }
  #endif
}
