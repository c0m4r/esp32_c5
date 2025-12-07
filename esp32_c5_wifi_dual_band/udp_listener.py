#!/usr/bin/env python3
"""
UDP listener for ESP32-C5 debug packets.
Listens on port 9090 and prints incoming UDP messages.
"""
import socket
from datetime import datetime

UDP_PORT = 9090

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))
    print(f"Listening for UDP packets on port {UDP_PORT}...")
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"[{timestamp}] {data.decode('utf-8', errors='replace')}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
