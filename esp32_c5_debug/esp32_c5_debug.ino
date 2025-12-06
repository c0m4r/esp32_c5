#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include "secrets.h"

#define DNS_1 "1.1.1.1"
#define DNS_2 "1.0.0.1"

WiFiUDP udp;

// Helper to print memory sizes in MB
void printMB(const char* label, uint32_t bytes) {
  Serial.printf("%s: %u Bytes (%.2f MB)\n", label, bytes, bytes / (1024.0 * 1024.0));
}

// Helper to print WiFi Protocols
void printWifiProtocols() {
    wifi_protocols_t protocols = {0};
    String protoStr2G = "";
    String protoStr5G = "";
    
    esp_err_t err = esp_wifi_get_protocols(WIFI_IF_STA, &protocols);
    if (err == ESP_OK) {
      if (protocols.ghz_2g) {
        if (protocols.ghz_2g & WIFI_PROTOCOL_11B) protoStr2G += "b";
        if (protocols.ghz_2g & WIFI_PROTOCOL_11G) { if(protoStr2G.length()) protoStr2G += "/"; protoStr2G += "g"; }
        if (protocols.ghz_2g & WIFI_PROTOCOL_11N) { if(protoStr2G.length()) protoStr2G += "/"; protoStr2G += "n"; }
        if (protocols.ghz_2g & WIFI_PROTOCOL_11AX) { if(protoStr2G.length()) protoStr2G += "/"; protoStr2G += "ax"; }
      }
      if (protocols.ghz_5g) {
        if (protocols.ghz_5g & WIFI_PROTOCOL_11A) protoStr5G += "a";
        if (protocols.ghz_5g & WIFI_PROTOCOL_11N) { if(protoStr5G.length()) protoStr5G += "/"; protoStr5G += "n"; }
        if (protocols.ghz_5g & WIFI_PROTOCOL_11AC) { if(protoStr5G.length()) protoStr5G += "/"; protoStr5G += "ac"; }
        if (protocols.ghz_5g & WIFI_PROTOCOL_11AX) { if(protoStr5G.length()) protoStr5G += "/"; protoStr5G += "ax"; }
      }
      Serial.printf("Protocols 2.4G: %s | 5G: %s\n", 
                    protoStr2G.length() ? protoStr2G.c_str() : "N/A",
                    protoStr5G.length() ? protoStr5G.c_str() : "N/A");
    } else {
      Serial.printf("esp_wifi_get_protocols() error: %d\n", err);
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give time for Serial Monitor to connect

  Serial.println("\nESP32-C5 starting...");
  Serial.println("--------------------------");
  Serial.printf("This is %s rev %d with %d cores @ %d MHz\n",
    ESP.getChipModel(),
    ESP.getChipRevision(),
    ESP.getChipCores(),
    ESP.getCpuFreqMHz());
  Serial.println("--------------------------");
  printMB("Flash Size", ESP.getFlashChipSize());
  printMB("PSRAM Size", ESP.getPsramSize());
  printMB("Heap Size", ESP.getHeapSize());
  Serial.println("--------------------------");
  WiFi.hostname("ESP32-C5");
  Serial.printf("Hostname: %s\n", WiFi.getHostname());
  // Enable IPv6 (must be after setting mode)
  Serial.println("Enabling IPv6...");
  if(WiFi.enableIPv6()) {
      Serial.println("IPv6 Enabled");
  } else {
      Serial.println("IPv6 Enable Failed");
  }

  Serial.println("--------------------------");
  
  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(SSID_1, PASS_1);
  
  // Wait for connection with timeout
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected in %lu ms!\n", millis() - startAttempt);
    
    // Apply custom DNS AFTER connection (use DHCP IP/gateway/subnet, override only DNS)
    Serial.println("Setting Custom DNS to " DNS_1 ", " DNS_2 "...");
    IPAddress dns1(DNS_1);
    IPAddress dns2(DNS_2);
    WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(), dns1, dns2);
    
    Serial.print("IPv6 Address: ");
    Serial.println(WiFi.linkLocalIPv6());
    Serial.print("DNS Server 1: ");
    Serial.println(WiFi.dnsIP(0));
    Serial.print("DNS Server 2: ");
    Serial.println(WiFi.dnsIP(1));
    
    // Print Protocols
    printWifiProtocols();
    
    Serial.println("--------------------------");
  } else {
    Serial.println("Connection failed! Will retry in loop...");
  }
}

void loop() {
  // Check Connection Status periodically
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck > 2000) {
    lastCheck = millis();
    if (WiFi.status() == WL_CONNECTED) {
      int channel = WiFi.channel();
      int rssi = WiFi.RSSI();
      String band = (channel > 14) ? "5GHz" : "2.4GHz"; // Simple deduction for variable logic
      
      // Gather extended debug info
      String ipv4 = WiFi.localIP().toString();
      String ipv6 = WiFi.globalIPv6().toString();
      String ssid = WiFi.SSID();
      String bssid = WiFi.BSSIDstr();
      String gateway = WiFi.gatewayIP().toString();
      String subnet = WiFi.subnetMask().toString();
      String mac = WiFi.macAddress();
      
      // Memory Calcs
      uint32_t totalHeap = ESP.getHeapSize();
      uint32_t freeHeap = ESP.getFreeHeap();
      uint32_t usedHeap = totalHeap - freeHeap;
      float heapPct = (float)usedHeap / totalHeap * 100.0;
      
      uint32_t totalPsram = ESP.getPsramSize();
      uint32_t freePsram = psramFound() ? ESP.getFreePsram() : 0;
      uint32_t usedPsram = totalPsram - freePsram;
      float psramPct = (totalPsram > 0) ? ((float)usedPsram / totalPsram * 100.0) : 0;
      
      unsigned long uptimeSec = millis() / 1000;
      float tempC = temperatureRead(); // Internal temperature sensor (Celsius)
      
      // Print to Serial - Extended debug info
      Serial.println("--- WiFi Status ---");
      Serial.printf("  SSID: %s | BSSID: %s | MAC: %s\n", ssid.c_str(), bssid.c_str(), mac.c_str());
      Serial.printf("  IPv4: %s | IPv6: %s\n", ipv4.c_str(), ipv6.c_str());
      Serial.printf("  Gateway: %s | Subnet: %s\n", gateway.c_str(), subnet.c_str());
      Serial.printf("  Channel: %d (%s) | RSSI: %d dBm\n", channel, band.c_str(), rssi);
      Serial.printf("  Heap: %u / %u (%5.1f%%)", usedHeap, totalHeap, heapPct);
      if (psramFound()) {
        Serial.printf(" | PSRAM: %u / %u (%5.1f%%)", usedPsram, totalPsram, psramPct);
      }
      Serial.println();
      Serial.printf("  Uptime: %lu sec | Temp: %.1f C\n", uptimeSec, tempC);
      Serial.println("-------------------");
      
      // Send UDP Packet with IPv4 and extended info
      String packet = "IP: " + ipv4 + 
                      ", RSSI: " + String(rssi) + 
                      ", Band: " + band + 
                      ", CH: " + String(channel) +
                      ", Heap: " + String(freeHeap) +
                      ", Temp: " + String(tempC, 1) + "C" +
                      ", Uptime: " + String(uptimeSec) + "s";
      udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
      udp.print(packet);
      udp.endPacket();
    } else {
       Serial.println("WiFi Status: Disconnected - Reconnecting...");
       // Harder reset to force fresh scan (helps pick 2.4GHz if best)
       WiFi.disconnect(true);  // true = turn off WiFi
       WiFi.mode(WIFI_OFF);
       delay(200);
       WiFi.mode(WIFI_STA);
       delay(200);
       WiFi.begin(SSID_1, PASS_1);
    }
  }

  // LED Status Logic
  // Red = Disconnected
  // Orange = 2.4GHz (Brightness = Signal Strength)
  // Green = 5GHz (Brightness = Signal Strength)
  #ifdef RGB_BUILTIN
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > 250) { // Update 4 times a second
      lastLedUpdate = millis();
      
      if (WiFi.status() == WL_CONNECTED) {
        int rssi = WiFi.RSSI();
        
        // If signal is extremely weak, treat as disconnected (Visual Red)
        if (rssi <= -90) {
           neopixelWrite(RGB_BUILTIN, 50, 0, 0); // Red
        } 
        else {
          int channel = WiFi.channel();
          bool is5GHz = (channel > 14);
          
          // Map RSSI (-90 to -40) to Brightness (10 to 200)
          int brightness = 0;
          if (rssi >= -40) brightness = 200;
          else if (rssi <= -90) brightness = 10;
          else brightness = map(rssi, -90, -40, 10, 200);
          
          uint8_t r = 0, g = 0, b = 0;
          
          if (is5GHz) {
            // Green for 5GHz
            r = 0;
            g = brightness;
            b = 0;
          } else {
            // Orange for 2.4GHz (Red + ~half Green)
            r = brightness;
            g = brightness / 2; // Approx orange/amber
            b = 0;
          }
          neopixelWrite(RGB_BUILTIN, r, g, b);
        }
      } else {
        // Red for Disconnected
        neopixelWrite(RGB_BUILTIN, 50, 0, 0); 
      }
    }
  #endif

}
