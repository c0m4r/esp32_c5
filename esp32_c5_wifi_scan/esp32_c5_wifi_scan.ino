#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n--- WiFi Scanner ---");

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("Starting Scan...");

  // Async scan to allow LED animation
  WiFi.scanNetworks(true);

  // Pulse Orange while scanning
  int brightness = 0;
  int direction = 2; // Speed of pulse
  while (WiFi.scanComplete() == -1) {
    // Ping-pong brightness (0 to 60)
    brightness += direction;
    if (brightness >= 60 || brightness <= 0) direction = -direction;
    if (brightness < 0) brightness = 0;

    // Orange Color (approx R=255, G=140, B=0) scaled by brightness
    uint8_t r = brightness; 
    uint8_t g = (brightness * 140) / 255; 
    uint8_t b = 0;

    #ifdef RGB_BUILTIN
      neopixelWrite(RGB_BUILTIN, r, g, b);
    #endif
    delay(10);
  }

  // Scan done - turn on Green (Dimmed 50%)
  // Green (R=0, G=64, B=0)
  #ifdef RGB_BUILTIN
    neopixelWrite(RGB_BUILTIN, 0, 64, 0);
  #endif

  int n = WiFi.scanComplete();

  Serial.println("Scan done");
  if (n == 0) {
      Serial.println("no networks found");
  } else {
      Serial.print(n);
      Serial.println(" networks found");
      Serial.println("Nr | SSID                             | RSSI | CH  | Freq | BSSID             | Enc");
      Serial.println("---|----------------------------------|------|-----|------|-------------------|---");
      for (int i = 0; i < n; ++i) {
          // Print SSID and RSSI for each network found
          String ssid = WiFi.SSID(i);
          String bssid = WiFi.BSSIDstr(i);
          int32_t rssi = WiFi.RSSI(i);
          int32_t channel = WiFi.channel(i);
          String freq = (channel > 14) ? "5  " : "2.4";
          String encryption = "Unknown";
          
          switch(WiFi.encryptionType(i)) {
              case WIFI_AUTH_OPEN: encryption = "Open"; break;
              case WIFI_AUTH_WEP: encryption = "WEP"; break;
              case WIFI_AUTH_WPA_PSK: encryption = "WPA"; break;
              case WIFI_AUTH_WPA2_PSK: encryption = "WPA2"; break;
              case WIFI_AUTH_WPA_WPA2_PSK: encryption = "WPA+WPA2"; break;
              case WIFI_AUTH_WPA2_ENTERPRISE: encryption = "WPA2-E"; break;
              case WIFI_AUTH_WPA3_PSK: encryption = "WPA3"; break;
              case WIFI_AUTH_WPA2_WPA3_PSK: encryption = "WPA2+WPA3"; break;
              case WIFI_AUTH_WAPI_PSK: encryption = "WAPI"; break;
              default: encryption = "Unknown";
          }

          Serial.printf("%2d | %-32.32s | %4d | %3d | %s | %s | %s\n", 
                        i + 1, 
                        ssid.c_str(), 
                        rssi, 
                        channel, 
                        freq.c_str(),
                        bssid.c_str(),
                        encryption.c_str());
          delay(10);
      }
  }
  Serial.println("--------------------");
}

void loop() {
  // Do nothing
  delay(1000);
}
