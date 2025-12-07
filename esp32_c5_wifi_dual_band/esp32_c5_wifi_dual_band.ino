#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include "secrets.h"

// ============================================================================
// DUAL-BAND WIFI ROAMING CONFIGURATION
// ============================================================================

#define DNS_1 "1.1.1.1"
#define DNS_2 "1.0.0.1"

// Roaming thresholds
#define RSSI_ROAM_THRESHOLD     -70   // Start considering roaming below this RSSI
#define RSSI_MIN_ACCEPTABLE     -85   // Minimum acceptable RSSI for any AP
#define RSSI_5G_ADJUSTMENT      10    // 5GHz gets this dB "bonus" when comparing
#define ROAM_HYSTERESIS         8     // New AP must be this much better to switch
#define ROAM_CHECK_INTERVAL_MS  30000 // Background scan interval (30 seconds)
#define RSSI_CRITICAL           -90   // Signal below this is critical (LED red)

// ============================================================================
// GLOBAL STATE
// ============================================================================

WiFiUDP udp;

// Roaming state
struct {
  bool isScanning = false;
  bool isRoaming = false;           // True during controlled roaming transition
  unsigned long roamStartTime = 0;  // When roaming started (for timeout)
  unsigned long lastRoamCheck = 0;
  uint8_t currentBssid[6] = {0};
  int8_t currentRssi = -127;
  uint8_t currentChannel = 0;
  bool is5GHz = false;
} roamState;

#define ROAM_TIMEOUT_MS 10000  // Max time to wait for roam to complete

// Best candidate found during scan
struct {
  bool valid = false;
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  bool is5GHz;
  int8_t effectiveRssi;  // RSSI with 5GHz adjustment applied
} bestCandidate;

// LED State for pulsing indicators
enum LedState {
  LED_CONNECTING,     // Initial connection - pulsing yellow
  LED_CONNECTED,      // Connected - solid based on band/signal
  LED_RECONNECTING,   // Lost connection - pulsing red
  LED_SCANNING        // Scanning for networks - pulsing blue
};
LedState currentLedState = LED_CONNECTING;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void printMB(const char* label, uint32_t bytes) {
  Serial.printf("%s: %u Bytes (%.2f MB)\n", label, bytes, bytes / (1024.0 * 1024.0));
}

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

String bssidToString(const uint8_t* bssid) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
  return String(buf);
}

bool isSameBssid(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

bool isChannel5GHz(uint8_t channel) {
  return channel > 14;
}

// Calculate effective RSSI (with 5GHz bonus)
int8_t getEffectiveRssi(int8_t rssi, bool is5GHz) {
  return is5GHz ? (rssi + RSSI_5G_ADJUSTMENT) : rssi;
}

// Pulsing LED - creates smooth breathing effect
void updatePulsingLed(uint8_t r, uint8_t g, uint8_t b, bool fast = false) {
  #ifdef RGB_BUILTIN
    // Create pulsing effect using sine wave
    int cycleMs = fast ? 400 : 1500;  // Fast = 0.4s cycle, Normal = 1.5s cycle
    float phase = (millis() % cycleMs) / (float)cycleMs * 3.14159 * 2;
    float brightness = (sin(phase) + 1.0) / 2.0;  // 0.0 to 1.0
    brightness = 0.05 + brightness * 0.95;  // Keep minimum 5% brightness
    
    neopixelWrite(RGB_BUILTIN, 
                  (uint8_t)(r * brightness), 
                  (uint8_t)(g * brightness), 
                  (uint8_t)(b * brightness));
  #endif
}

// ============================================================================
// WIFI CONFIGURATION - DUAL-BAND ROAMING
// ============================================================================

void configureWifiForDualBand(bool fastConnect = false) {
  wifi_config_t wifi_config = {};
  
  // Copy credentials
  strncpy((char*)wifi_config.sta.ssid, SSID_1, sizeof(wifi_config.sta.ssid) - 1);
  strncpy((char*)wifi_config.sta.password, PASS_1, sizeof(wifi_config.sta.password) - 1);
  
  // === CONNECTION MODE ===
  if (fastConnect) {
    // FAST SCAN: Connect quickly to first matching AP found
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    Serial.println("[WiFi] Fast connect mode (quick startup)");
  } else {
    // ALL CHANNEL SCAN: Find the best AP across both bands
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    Serial.println("[WiFi] Full scan mode (best AP selection)");
  }
  
  // Minimum acceptable signal strength
  wifi_config.sta.threshold.rssi = RSSI_MIN_ACCEPTABLE;
  
  // 5GHz preference: give 5GHz APs a bonus in RSSI comparison
  wifi_config.sta.threshold.rssi_5g_adjustment = RSSI_5G_ADJUSTMENT;
  
  // Enable roaming assistance protocols (if router supports them)
  wifi_config.sta.rm_enabled = 1;   // 802.11k Radio Resource Measurement
  wifi_config.sta.btm_enabled = 1;  // 802.11v BSS Transition Management
  
  // DON'T lock to a specific BSSID - allow roaming
  wifi_config.sta.bssid_set = false;
  wifi_config.sta.channel = 0;  // Let it scan all channels
  
  // Apply the configuration
  esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_config failed: %d\n", err);
  } else {
    Serial.printf("  5GHz RSSI adjustment: +%d dB\n", RSSI_5G_ADJUSTMENT);
  }
}

// ============================================================================
// WIFI EVENT HANDLER
// ============================================================================

void onWifiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi] Connected to AP");
      break;
      
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] Disconnected - will auto-reconnect");
      roamState.isScanning = false;
      // Let the loop handle reconnection
      break;
      
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
      // Set LED state to connected
      currentLedState = LED_CONNECTED;
      // Clear roaming flag - transition complete
      if (roamState.isRoaming) {
        Serial.println("[Roaming] Roam completed successfully!");
        roamState.isRoaming = false;
      }
      // Update current AP info
      memcpy(roamState.currentBssid, WiFi.BSSID(), 6);
      roamState.currentChannel = WiFi.channel();
      roamState.currentRssi = WiFi.RSSI();
      roamState.is5GHz = isChannel5GHz(roamState.currentChannel);
      Serial.printf("[WiFi] BSSID: %s | CH: %d (%s) | RSSI: %d dBm\n",
                    bssidToString(roamState.currentBssid).c_str(),
                    roamState.currentChannel,
                    roamState.is5GHz ? "5GHz" : "2.4GHz",
                    roamState.currentRssi);
      break;
      
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      Serial.println("[WiFi] Background scan complete");
      processScanResults();
      roamState.isScanning = false;
      break;
      
    default:
      break;
  }
}

// ============================================================================
// BACKGROUND SCAN & ROAMING LOGIC
// ============================================================================

void startBackgroundScan() {
  if (roamState.isScanning) {
    return;  // Already scanning
  }
  
  Serial.println("[Roaming] Starting background scan...");
  roamState.isScanning = true;
  bestCandidate.valid = false;
  
  // Configure scan for our SSID only (faster, more targeted)
  wifi_scan_config_t scan_config = {};
  scan_config.ssid = (uint8_t*)SSID_1;
  scan_config.bssid = NULL;
  scan_config.channel = 0;  // All channels
  scan_config.show_hidden = false;
  scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
  scan_config.scan_time.active.min = 50;
  scan_config.scan_time.active.max = 150;
  
  // Non-blocking scan - will trigger WIFI_SCAN_DONE event
  esp_err_t err = esp_wifi_scan_start(&scan_config, false);
  if (err != ESP_OK) {
    Serial.printf("[Roaming] Scan start failed: %d\n", err);
    roamState.isScanning = false;
  }
}

void processScanResults() {
  uint16_t numAPs = 0;
  esp_wifi_scan_get_ap_num(&numAPs);
  
  if (numAPs == 0) {
    Serial.println("[Roaming] No APs found in scan");
    esp_wifi_clear_ap_list();
    return;
  }
  
  wifi_ap_record_t* apList = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * numAPs);
  if (!apList) {
    Serial.println("[Roaming] Failed to allocate memory for AP list");
    esp_wifi_clear_ap_list();
    return;
  }
  
  esp_wifi_scan_get_ap_records(&numAPs, apList);
  
  Serial.printf("[Roaming] Found %d APs for SSID '%s':\n", numAPs, SSID_1);
  
  // Update current AP's RSSI
  roamState.currentRssi = WiFi.RSSI();
  int8_t currentEffectiveRssi = getEffectiveRssi(roamState.currentRssi, roamState.is5GHz);
  
  bestCandidate.valid = false;
  int8_t bestEffectiveRssi = -127;
  
  for (int i = 0; i < numAPs; i++) {
    wifi_ap_record_t* ap = &apList[i];
    
    // Only consider APs with matching SSID
    if (strcmp((char*)ap->ssid, SSID_1) != 0) {
      continue;
    }
    
    bool apIs5GHz = isChannel5GHz(ap->primary);
    int8_t effectiveRssi = getEffectiveRssi(ap->rssi, apIs5GHz);
    bool isCurrentAP = isSameBssid(ap->bssid, roamState.currentBssid);
    
    Serial.printf("  %s BSSID: %s CH: %2d (%s) RSSI: %d dBm (eff: %d)\n",
                  isCurrentAP ? "*" : " ",
                  bssidToString(ap->bssid).c_str(),
                  ap->primary,
                  apIs5GHz ? "5GHz  " : "2.4GHz",
                  ap->rssi,
                  effectiveRssi);
    
    // Skip current AP for candidate evaluation
    if (isCurrentAP) {
      continue;
    }
    
    // Track best alternative
    if (effectiveRssi > bestEffectiveRssi) {
      bestEffectiveRssi = effectiveRssi;
      memcpy(bestCandidate.bssid, ap->bssid, 6);
      bestCandidate.rssi = ap->rssi;
      bestCandidate.channel = ap->primary;
      bestCandidate.is5GHz = apIs5GHz;
      bestCandidate.effectiveRssi = effectiveRssi;
      bestCandidate.valid = true;
    }
  }
  
  free(apList);
  esp_wifi_clear_ap_list();
  
  // Evaluate if we should roam
  evaluateRoaming(currentEffectiveRssi);
}

void evaluateRoaming(int8_t currentEffectiveRssi) {
  if (!bestCandidate.valid) {
    Serial.println("[Roaming] No alternative APs available");
    return;
  }
  
  int8_t improvement = bestCandidate.effectiveRssi - currentEffectiveRssi;
  
  Serial.printf("[Roaming] Current: %d dBm (eff: %d) | Best alt: %d dBm (eff: %d) | Δ: %+d dB\n",
                roamState.currentRssi, currentEffectiveRssi,
                bestCandidate.rssi, bestCandidate.effectiveRssi,
                improvement);
  
  // Roaming decision logic:
  // 1. If current signal is below threshold AND better AP exists with hysteresis margin
  // 2. OR if current signal is critical and any better AP exists
  bool shouldRoam = false;
  String reason = "";
  
  if (roamState.currentRssi < RSSI_CRITICAL && improvement > 0) {
    shouldRoam = true;
    reason = "Critical signal, any improvement helps";
  } else if (roamState.currentRssi < RSSI_ROAM_THRESHOLD && improvement >= ROAM_HYSTERESIS) {
    shouldRoam = true;
    reason = "Below threshold with significant improvement";
  } else if (improvement >= (ROAM_HYSTERESIS * 2)) {
    // Even with good signal, roam if there's a dramatically better AP
    shouldRoam = true;
    reason = "Dramatically better AP available";
  }
  
  if (shouldRoam) {
    Serial.printf("[Roaming] ROAMING! Reason: %s\n", reason.c_str());
    Serial.printf("[Roaming] Switching to BSSID: %s (CH: %d, %s)\n",
                  bssidToString(bestCandidate.bssid).c_str(),
                  bestCandidate.channel,
                  bestCandidate.is5GHz ? "5GHz" : "2.4GHz");
    
    performRoam();
  } else {
    Serial.println("[Roaming] Staying on current AP (no roam needed)");
  }
}

void performRoam() {
  // Mark that we're in a controlled roaming transition
  roamState.isRoaming = true;
  roamState.roamStartTime = millis();
  currentLedState = LED_RECONNECTING;  // Show pulsing red during roam
  
  Serial.println("[Roaming] Initiating roam...");
  
  // Disconnect from current AP
  WiFi.disconnect(false);  // Don't turn off WiFi
  delay(100);
  
  // Configure to connect to specific BSSID
  wifi_config_t wifi_config = {};
  strncpy((char*)wifi_config.sta.ssid, SSID_1, sizeof(wifi_config.sta.ssid) - 1);
  strncpy((char*)wifi_config.sta.password, PASS_1, sizeof(wifi_config.sta.password) - 1);
  
  // Lock to specific BSSID for this connection
  wifi_config.sta.bssid_set = true;
  memcpy(wifi_config.sta.bssid, bestCandidate.bssid, 6);
  wifi_config.sta.channel = bestCandidate.channel;
  
  // Keep other settings
  wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  wifi_config.sta.threshold.rssi = RSSI_MIN_ACCEPTABLE;
  wifi_config.sta.threshold.rssi_5g_adjustment = RSSI_5G_ADJUSTMENT;
  wifi_config.sta.rm_enabled = 1;
  wifi_config.sta.btm_enabled = 1;
  
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_connect();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==============================================");
  Serial.println("ESP32-C5 Dual-Band WiFi Roaming Demo");
  Serial.println("==============================================");
  
  // Turn off RGB LED at startup
  #ifdef RGB_BUILTIN
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  #endif
  
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
  
  // Register WiFi event handler
  WiFi.onEvent(onWifiEvent);
  
  // Set hostname
  WiFi.hostname("ESP32-C5");
  Serial.printf("Hostname: %s\n", WiFi.getHostname());
  
  // Enable IPv6
  Serial.println("Enabling IPv6...");
  if(WiFi.enableIPv6()) {
    Serial.println("IPv6 Enabled");
  } else {
    Serial.println("IPv6 Enable Failed");
  }

  Serial.println("--------------------------");
  
  // Initialize WiFi in STA mode
  WiFi.mode(WIFI_STA);
  
  // Apply FAST connect configuration for quick startup
  configureWifiForDualBand(true);  // true = fast connect
  
  // Start initial connection with pulsing yellow LED
  currentLedState = LED_CONNECTING;
  Serial.println("Connecting to WiFi (fast connect)...");
  unsigned long startAttempt = millis();
  
  esp_wifi_connect();
  
  // Wait for connection with timeout (show fast pulsing magenta LED)
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    updatePulsingLed(100, 0, 100, true);  // Magenta, fast
    delay(20);  // Small delay for smooth pulsing
  }

  if (WiFi.status() == WL_CONNECTED) {
    currentLedState = LED_CONNECTED;
    Serial.printf("Connected in %lu ms!\n", millis() - startAttempt);
    
    // Apply custom DNS
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
    
    printWifiProtocols();
    
    Serial.println("--------------------------");
  } else {
    Serial.println("Initial connection failed! Will retry in loop...");
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  static unsigned long lastStatusPrint = 0;
  
  // Status output every 2 seconds
  if (millis() - lastStatusPrint > 2000) {
    lastStatusPrint = millis();
    
    if (WiFi.status() == WL_CONNECTED) {
      // Update current state - but don't update channel during scanning
      // because WiFi.channel() reports the scanning channel, not the connected channel
      roamState.currentRssi = WiFi.RSSI();
      if (!roamState.isScanning) {
        roamState.currentChannel = WiFi.channel();
        roamState.is5GHz = isChannel5GHz(roamState.currentChannel);
        memcpy(roamState.currentBssid, WiFi.BSSID(), 6);
      }
      
      // Gather info
      String ipv4 = WiFi.localIP().toString();
      String ipv6 = WiFi.globalIPv6().toString();
      String ssid = WiFi.SSID();
      String bssid = bssidToString(roamState.currentBssid);  // Use cached BSSID
      String mac = WiFi.macAddress();
      String band = roamState.is5GHz ? "5GHz" : "2.4GHz";
      
      // Memory
      uint32_t totalHeap = ESP.getHeapSize();
      uint32_t freeHeap = ESP.getFreeHeap();
      uint32_t usedHeap = totalHeap - freeHeap;
      float heapPct = (float)usedHeap / totalHeap * 100.0;
      
      uint32_t totalPsram = ESP.getPsramSize();
      uint32_t freePsram = psramFound() ? ESP.getFreePsram() : 0;
      uint32_t usedPsram = totalPsram - freePsram;
      float psramPct = (totalPsram > 0) ? ((float)usedPsram / totalPsram * 100.0) : 0;
      
      unsigned long uptimeSec = millis() / 1000;
      float tempC = temperatureRead();
      
      // Print status
      Serial.println("--- WiFi Status ---");
      Serial.printf("  SSID: %s | BSSID: %s | MAC: %s\n", ssid.c_str(), bssid.c_str(), mac.c_str());
      Serial.printf("  IPv4: %s | IPv6: %s\n", ipv4.c_str(), ipv6.c_str());
      Serial.printf("  Channel: %d (%s) | RSSI: %d dBm\n", roamState.currentChannel, band.c_str(), roamState.currentRssi);
      Serial.printf("  Heap: %u / %u (%5.1f%%)", usedHeap, totalHeap, heapPct);
      if (psramFound()) {
        Serial.printf(" | PSRAM: %u / %u (%5.1f%%)", usedPsram, totalPsram, psramPct);
      }
      Serial.println();
      Serial.printf("  Uptime: %lu sec | Temp: %.1f C\n", uptimeSec, tempC);
      Serial.println("-------------------");
      
      // Send UDP packet
      String packet = "IP: " + ipv4 + 
                      ", RSSI: " + String(roamState.currentRssi) + 
                      ", Band: " + band + 
                      ", CH: " + String(roamState.currentChannel) +
                      ", Temp: " + String(tempC, 1) + "C" +
                      ", Uptime: " + String(uptimeSec) + "s";
      udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
      udp.print(packet);
      udp.endPacket();
      
    } else {
      // Check if we're in a controlled roaming transition
      if (roamState.isRoaming) {
        // Check for roaming timeout
        if (millis() - roamState.roamStartTime > ROAM_TIMEOUT_MS) {
          Serial.println("[Roaming] Roam timeout - falling back to normal reconnect");
          roamState.isRoaming = false;
        } else {
          // Still waiting for roam to complete, don't interfere
          Serial.printf("[Roaming] Waiting for roam... (%lu ms elapsed)\n", 
                        millis() - roamState.roamStartTime);
        }
      } else {
        // Normal disconnection - set LED state and do full scan reconnect
        currentLedState = LED_RECONNECTING;
        Serial.println("WiFi Status: Disconnected - Scanning for best AP...");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(200);
        WiFi.mode(WIFI_STA);
        configureWifiForDualBand(false);  // false = full scan for best AP
        esp_wifi_connect();
      }
    }
  }
  
  // Background roaming check (only when connected and not currently scanning)
  if (WiFi.status() == WL_CONNECTED && 
      !roamState.isScanning && 
      millis() - roamState.lastRoamCheck > ROAM_CHECK_INTERVAL_MS) {
    roamState.lastRoamCheck = millis();
    
    // Only scan if signal is below threshold or periodically for discovery
    if (roamState.currentRssi < RSSI_ROAM_THRESHOLD) {
      Serial.println("[Roaming] Signal below threshold, scanning for better AP...");
      startBackgroundScan();
    } else {
      // Still do occasional scans even with good signal (less frequently)
      static unsigned long lastDiscoveryScan = 0;
      if (millis() - lastDiscoveryScan > (ROAM_CHECK_INTERVAL_MS * 4)) {
        lastDiscoveryScan = millis();
        Serial.println("[Roaming] Periodic discovery scan...");
        startBackgroundScan();
      }
    }
  }

  // LED Status Logic - State Machine
  #ifdef RGB_BUILTIN
    // Update LED frequently for smooth pulsing
    switch (currentLedState) {
      case LED_CONNECTING:
        // Fast pulsing magenta during initial connection
        updatePulsingLed(100, 0, 100, true);  // Magenta, fast
        break;
        
      case LED_RECONNECTING:
        // Fast pulsing magenta during reconnection (same as connecting)
        updatePulsingLed(100, 0, 100, true);  // Magenta, fast
        break;
        
      case LED_SCANNING:
        // Pulsing blue during scanning (optional)
        updatePulsingLed(0, 0, 100);
        break;
        
      case LED_CONNECTED:
      default:
        // === RGB LED: Signal strength gradient (green → yellow → orange) ===
        {
          int rssi = roamState.currentRssi;
          uint8_t r = 0, g = 0, b = 0;
          
          // Critical signal = Red
          if (rssi <= RSSI_CRITICAL) {
            r = 80; g = 0; b = 0;
          } else {
            // Map RSSI to color gradient:
            // Excellent (-40 and above): Green
            // Good (-55): Yellow-green  
            // Mid (-70): Yellow
            // Weak (-85): Orange
            // Very weak (-90): Red-orange
            
            if (rssi >= -40) {
              // Excellent: Pure green
              r = 0; g = 100; b = 0;
            } else if (rssi >= -55) {
              // Good: Green to yellow-green (add some red)
              int t = map(rssi, -55, -40, 0, 100);
              r = 100 - t;  // 100 at -55, 0 at -40
              g = 100;
              r = r / 2;    // Keep it more green
            } else if (rssi >= -70) {
              // Mid: Yellow-green to yellow
              int t = map(rssi, -70, -55, 0, 100);
              r = 100 - t/2;  // More red as signal drops
              g = 100;
            } else if (rssi >= -85) {
              // Weak: Yellow to orange
              int t = map(rssi, -85, -70, 0, 100);
              r = 100;
              g = 30 + (70 * t / 100);  // 30 at -85, 100 at -70
            } else {
              // Very weak: Orange to red-orange
              int t = map(rssi, -90, -85, 0, 100);
              r = 100;
              g = t * 30 / 100;  // 0 at -90, 30 at -85
            }
          }
          
          // Band indication: 5GHz = solid, 2.4GHz = smooth pulsing
          if (!roamState.is5GHz) {
            // 2.4GHz: Smooth pulsing effect (gentle breathing)
            float phase = (millis() % 2000) / 2000.0 * 3.14159 * 2;  // 2 second cycle
            float brightness = (sin(phase) + 1.0) / 2.0;  // 0.0 to 1.0
            brightness = 0.3 + brightness * 0.7;  // Range 30% to 100%
            r = (uint8_t)(r * brightness);
            g = (uint8_t)(g * brightness);
            b = (uint8_t)(b * brightness);
          }
          
          neopixelWrite(RGB_BUILTIN, r, g, b);
        }
        break;
    }
  #endif
}
