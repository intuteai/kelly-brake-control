# ESP32-C3 CAN-to-BLE Bridge with Brake Control

## 📌 Overview
This firmware runs on an **ESP32-C3 microcontroller**.  
It bridges **CAN bus data** from a **Kelly Controller (KLS)** to a **Bluetooth Low Energy (BLE) client**, while decoding critical signals and managing a relay-controlled brake system.

## ⚡ Features
- **CAN-to-BLE Bridging**: Forwards all CAN messages to BLE clients.
- **Signal Decoding**: Extracts speed, motor current, and throttle.
- **Brake Control**: Relay-based brake with a state machine (failsafe applied by default).
- **Digital Input Monitoring**: Reads DI1–DI15 via multiplexer and KL15 input, packed into BLE updates.
- **Reliability**:
  - CAN bus supervision and auto-recovery
  - Periodic watchdog DONE pulses
  - Wake interrupt handling

## 🛠️ Hardware Mapping
- **CAN**: TX=GPIO3, RX=GPIO4  
- **Relay (Brake)**: GPIO7  
- **Multiplexer**: S0=8, S1=9, S2=5, S3=10, IN=6  
- **KL15**: GPIO0  
- **USB Power**: GPIO19  
- **Watchdog**: DONE=18, WAKE=1  

## 🔗 BLE Service
- **Service UUID**: `7E400001-B5A3-F393-E0A9-E50E24DCCA9E`  
- **Characteristic UUID**: `7E400002-B5A3-F393-E0A9-E50E24DCCA9E`  
- Notifications:
  - `0x01`: CAN message (ID + data)  
  - `0x02`: Packed GPIO states  

## 🚦 Brake Logic
- **Applied (default at startup)**: Relay OFF  
- **Released**: Throttle ≥ 70  
- **Re-applied**: Throttle < 65 and Speed < 100 RPM  
- **Override**: DI12 HIGH → Force brake applied  

## 🧰 Setup
1. Install [Arduino-ESP32](https://github.com/espressif/arduino-esp32) framework.  
2. Connect ESP32-C3 hardware (CAN transceiver, relay, mux, etc.).  
3. Flash the firmware.  
4. Connect via a BLE client app or custom mobile/PC application.

## ✅ Safety
- Brake always applied at startup or if override triggered.  
- CAN supervision with automatic recovery from bus-off states.  
- Watchdog pulses ensure external system liveness monitoring.  

---

### 📄 License
MIT License. Use freely with attribution.

