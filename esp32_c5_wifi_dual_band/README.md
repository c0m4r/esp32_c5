# ESP32-C5 Dual-Band WiFi Debug Sketch

A comprehensive WiFi testing and debugging sketch for ESP32-C5 with **Android-like dual-band roaming** support, smart reconnection, and visual LED status indicators.

## Features

### 🌐 Dual-Band WiFi
- Supports both **2.4GHz and 5GHz** networks (same SSID)
- **5GHz preference**: Configurable RSSI bonus gives 5GHz priority
- **Background roaming**: Periodic scans for better access points
- **Hysteresis-based switching**: Prevents ping-pong between bands

### 📡 Smart Connection
- **Fast startup**: Quick initial connection using first available AP
- **Full-scan reconnection**: Scans all channels after disconnection to find best AP
- **802.11k/v support**: Enables AP-assisted roaming (if router supports)

### 💡 RGB LED Status

| State | Behavior |
|-------|----------|
| Connecting/Reconnecting | 🟣 Fast Pulsing Magenta |
| Connected 5GHz | Solid color (based on signal) |
| Connected 2.4GHz | Smooth pulsing color (gentle breathing) |

**Signal Strength Colors:**
| Signal | Color |
|--------|-------|
| Excellent (≥-40 dBm) | 🟢 Green |
| Good (-55 dBm) | Yellow-Green |
| Mid (-70 dBm) | 🟡 Yellow |
| Weak (-85 dBm) | 🟠 Orange |
| Critical (≤-90 dBm) | 🔴 Red |

### 📊 Debug Output
- Serial output every 2 seconds with:
  - SSID, BSSID, MAC address
  - IPv4 and IPv6 addresses
  - Channel, band, and RSSI
  - Heap and PSRAM usage
  - Uptime and temperature
- UDP packet broadcasting for remote monitoring

## Configuration

### secrets.h
Create a `secrets.h` file with your credentials:

```cpp
#define SSID_1 "YourWiFiNetwork"
#define PASS_1 "YourPassword"
#define UDP_TARGET_IP "192.168.1.2"  // IP to send UDP packets
#define UDP_TARGET_PORT 9090
```

### Tunable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `RSSI_ROAM_THRESHOLD` | -70 dBm | Start looking for better AP below this |
| `RSSI_MIN_ACCEPTABLE` | -85 dBm | Minimum acceptable signal strength |
| `RSSI_5G_ADJUSTMENT` | +10 dB | 5GHz gets this bonus when comparing |
| `ROAM_HYSTERESIS` | 8 dB | New AP must be this much better |
| `ROAM_CHECK_INTERVAL_MS` | 30000 | Background scan interval |
| `RSSI_CRITICAL` | -90 dBm | Signal below this shows red LED |

## How Roaming Works

```
┌── INITIAL CONNECTION (Fast) ────────────────────┐
│ Uses WIFI_FAST_SCAN for quick startup           │
│ Connects to first matching AP found             │
└─────────────────────────────────────────────────┘
                    ↓
┌── CONNECTED (Monitoring) ───────────────────────┐
│ Every 30s: Check current RSSI                   │
│   If < -70 dBm → Background scan                │
│   If better AP found (8dB+ margin) → ROAM       │
└─────────────────────────────────────────────────┘
                    ↓
┌── DISCONNECTION (Recovery) ─────────────────────┐
│ Uses WIFI_ALL_CHANNEL_SCAN                      │
│ Scans both bands, picks best signal             │
│ 5GHz gets +10dB preference bonus                │
└─────────────────────────────────────────────────┘
```

## Hardware

- **Board**: ESP32-C5-WROOM-1-N8R4 (or compatible)
- **LED**: Uses built-in RGB LED (`RGB_BUILTIN`)

## Dependencies

- ESP32 Arduino Core (with ESP32-C5 support)
- No external libraries required

## UDP Monitoring

Listen for UDP packets on your target IP/port:

```bash
./udp_listener.py
# or
nc -u -l 9090
```

Example packet:
```
IP: 192.168.1.2, RSSI: -35, Band: 5GHz, CH: 112, Heap: 78136, Temp: 33.3C, Uptime: 839s
```

## License

WTFPL
