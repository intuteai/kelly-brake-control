#include <Arduino.h>
#include <cstring>
#include "driver/twai.h"
#include "BLEDevice.h"
#include "esp_err.h"

// ======================= Pin Assignments =======================
// CAN pins
#define CAN_TX 3
#define CAN_RX 4

// BLE UUIDs
#define SERVICE_UUID        "7E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "7E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// Digital IO pins
int KL15    = 0;
int Relay   = 7;
int MuxS0   = 8;
int MuxS1   = 9;
int MuxS2   = 5;
int MuxS3   = 10;
int MuxIN   = 6;
int USBpower = 19;

#define WD_DONE 18
#define WD_WAKE 1

// ======================= Digital IO variables =======================
int DI13, DI14, DI15, DI10, DI11, DI12;
int DI7, DI8, DI9, DI4, DI5, DI6;
int DI1, DI2, DI3;

int KL15_Status;
String usbFaultStatus;

// ======================= BLE/CAN globals =======================
BLEServer*        bleServer = nullptr;
BLECharacteristic* bleCharacteristic = nullptr;
BLEAdvertising*   advertising = nullptr;

bool clientConnected = false;
bool wasConnected    = false;

unsigned long lastMessageTime = 0;
int           totalMessages   = 0;

unsigned long lastDIPrint = 0;
const unsigned long DI_PRINT_PERIOD_MS = 500; // 0.5s

// -------------------- KLS decode (speed/current/throttle) --------------------
// From KLS broadcast:
// 0x0CF11E05 (Msg 1): bytes 0..1 = speed rpm (1 rpm/bit), bytes 2..3 = motor current (0.1 A/bit)
// 0x0CF11F05 (Msg 2): byte 0 = throttle 0..255 (maps 0..5V)

int motorSpeedRPM   = 0;
int motorCurrent_dA = 0; // deci-amps (0.1 A units) — decoded for info only
int throttleValue   = 0; // 0..255

// -------------------- Brake control --------------------
const int  SPEED_THRESH_RPM  = 100; // re-apply when slower than this
const int  THROTTLE_RELEASE  = 47;  // NEW: release brake when throttle >= 47 (kept for reference; state machine uses thresholds below)
const int  THROTTLE_APPLY    = 70;  // NEW: apply brake when speed<100 AND throttle <= 70
const bool RELAY_ACTIVE_HIGH = true; // HIGH = coil energized

enum BrakeState { BRAKE_APPLIED, BRAKE_RELEASED }; // "applied" = holding, "released" = free to move
BrakeState brakeState = BRAKE_APPLIED;

void driveRelay(bool on) { // on=true → coil energized
  if (RELAY_ACTIVE_HIGH) digitalWrite(Relay, on ? HIGH : LOW);
  else                   digitalWrite(Relay, on ? LOW  : HIGH);
}

void setBrakeState(BrakeState s) {
  if (brakeState == s) return;
  brakeState = s;

  // Relay ON means brake released (per your hardware note)
  driveRelay(s == BRAKE_RELEASED);

  if (s == BRAKE_RELEASED) Serial.println("✅ Brake RELEASED (Relay ON)");
  else                     Serial.println("🛑 Brake APPLIED (Relay OFF)");
}

// -------------------- BLE callbacks --------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("✅ BLE Client Connected");
    clientConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("⚠ BLE Client Disconnected");
    clientConnected = false;
    delay(100);
    advertising->start(); // Restart advertising
  }
};

// -------------------- Helper: MUX select --------------------
void selectMuxChannel(byte channel) {
  digitalWrite(MuxS0, bitRead(channel, 0));
  digitalWrite(MuxS1, bitRead(channel, 1));
  digitalWrite(MuxS2, bitRead(channel, 2));
  digitalWrite(MuxS3, bitRead(channel, 3));
}

// -------------------- Pack Digital IO states --------------------
uint16_t packDigitalInputs() {
  KL15_Status = digitalRead(KL15);

  for (int ch = 0; ch < 16; ch++) {
    selectMuxChannel(ch);
    delay(5);
    int value = digitalRead(MuxIN);

    switch (ch) {
      case 0:  DI13 = value; break;
      case 1:  DI14 = value; break;
      case 2:  DI15 = value; break;
      case 3:  DI10 = value; break;
      case 4:  DI11 = value; break;
      case 5:  DI12 = value; break;
      case 6:  DI7  = value; break;
      case 7:  DI8  = value; break;
      case 8:  DI9  = value; break;
      case 9:  DI4  = value; break;
      case 10: DI5  = value; break;
      case 11: DI6  = value; break;
      case 12: DI1  = value; break;
      case 13: DI2  = value; break;
      case 14: DI3  = value; break;
      case 15: usbFaultStatus = (value == HIGH) ? "USB OK" : "USB FAULT"; break;
    }
  }

  uint16_t packed = 0;
  packed |= (KL15_Status & 1) << 0;
  packed |= (DI1         & 1) << 1;
  packed |= (DI2         & 1) << 2;
  packed |= (DI3         & 1) << 3;
  packed |= (DI4         & 1) << 4;
  packed |= (DI5         & 1) << 5;
  packed |= (DI6         & 1) << 6;
  packed |= (DI7         & 1) << 7;
  packed |= (DI8         & 1) << 8;
  packed |= (DI9         & 1) << 9;
  packed |= (DI10        & 1) << 10;
  packed |= (DI11        & 1) << 11;
  packed |= (DI12        & 1) << 12;
  packed |= ((usbFaultStatus == "USB OK") ? 1 : 0) << 13; // bit13 = USB OK

  return packed;
}

// -------------------- CAN ID decode (prints) --------------------
void decodeCANID(uint32_t id) {
  if      (id == 0x1038FF50) Serial.println(" -> Msg_DIU1 (Faults)");
  else if (id == 0x0CF11F05) Serial.println(" -> Msg2 (Throttle/Temps)");
  else if (id == 0x0CF11E05) Serial.println(" -> Msg1 (Speed/Current)");
  else if (id == 0x104FFF50) Serial.println(" -> Msg_DIU10");
  else if (id == 0x14234050) Serial.println(" -> Msg_DIU2 (Current)");
  else if (id == 0x14244050) Serial.println(" -> Msg_DIU3 (Cell Voltages)");
  else if (id == 0x10281050) Serial.println(" -> Msg_DIU4 (SOC)");
  else if (id == 0x1031FF50) Serial.println(" -> Msg_DIU14 (DIU14 Faults)");
  else if (id == 0x14498250) Serial.println(" -> Msg_DriveParameters");
  else                       Serial.println(" -> Unknown Message ID");
}

// -------------------- Handle KLS 0x0CF11E05 (speed/current) --------------------
void handleKLSMessage(uint32_t id, uint8_t* data, uint8_t len) {
  if (id != 0x0CF11E05 || len < 4) return;

  // Speed (rpm): LSB first
  motorSpeedRPM = (data[1] << 8) | data[0];

  // Current (0.1A/bit): LSB first
  motorCurrent_dA = (data[3] << 8) | data[2];

  Serial.printf("➡ Speed: %d rpm, Current: %.1f A\n", motorSpeedRPM, motorCurrent_dA / 10.0f);
  // NOTE: no brake state changes here anymore; state machine runs centrally.
}

// -------------------- Handle KLS 0x0CF11F05 (Msg 2: throttle/temps) ----------
void handleKLSMessage2(uint32_t id, uint8_t* data, uint8_t len) {
  if (id != 0x0CF11F05 || len < 1) return;

  // Byte 0 = throttle 0..255 (maps to 0..5V)
  throttleValue = data[0];
  Serial.printf("➡ Throttle: %d (0..255)\n", throttleValue);
}

// -------------------- Brake state machine (uses speed + throttle) ------------
void updateBrakeStateMachine() {
  // 🚨 Safety override: if DI12 (A2) is HIGH → force brake APPLIED
  if (DI12 == HIGH) {
    setBrakeState(BRAKE_APPLIED);
    return; // don’t check throttle/speed, brake must stay applied
  }

  switch (brakeState) {
    case BRAKE_APPLIED:
      // RELEASE when throttle >= 70
      if (throttleValue >= 70) {
        setBrakeState(BRAKE_RELEASED);
      }
      break;

    case BRAKE_RELEASED:
      // APPLY when throttle < 65 AND speed < 100 rpm
      if (throttleValue < 65 && motorSpeedRPM < SPEED_THRESH_RPM) {
        setBrakeState(BRAKE_APPLIED);
      }
      break;
  }
}

volatile bool wakeTriggered = false;
void IRAM_ATTR onWakeISR() { wakeTriggered = true; }

// ======================= setup =======================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // BLE Setup
  BLEDevice::init("Battery_ESP32_C3");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());

  BLEService* service = bleServer->createService(SERVICE_UUID);
  bleCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  service->start();

  advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();
  Serial.println("✅ BLE GATT Server Started");

  // CAN Setup
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("✅ TWAI (CAN) Driver Installed");
  } else {
    Serial.println("❌ TWAI Driver Installation Failed!");
    while (1) {}
  }

  if (twai_start() == ESP_OK) {
    Serial.println("✅ TWAI (CAN) Driver Started");
  } else {
    Serial.println("❌ Failed to Start TWAI (CAN) Driver");
    while (1) {}
  }

  // Digital IO Setup
  pinMode(KL15,   INPUT);
  pinMode(Relay,  OUTPUT);
  pinMode(MuxS0,  OUTPUT);
  pinMode(MuxS1,  OUTPUT);
  pinMode(MuxS2,  OUTPUT);
  pinMode(MuxS3,  OUTPUT);
  pinMode(MuxIN,  INPUT);
  pinMode(USBpower, OUTPUT);
  digitalWrite(USBpower, HIGH);

  // Start with brake APPLIED for safety
  driveRelay(false);
  brakeState = BRAKE_APPLIED;
  Serial.println("🛑 Brake APPLIED at startup (Relay OFF)");

  pinMode(WD_DONE, OUTPUT);
  digitalWrite(WD_DONE, LOW);

  pinMode(WD_WAKE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WD_WAKE), onWakeISR, FALLING);

  lastMessageTime = millis();
}

// ======================= loop =======================
void loop() {
  // CAN Receive
  twai_message_t     message;
  twai_status_info_t status;
  twai_get_status_info(&status);

  if (wasConnected && !clientConnected) {
    Serial.println("🔄 BLE disconnected, restarting advertisement...");
    delay(100);
    advertising->start();
    Serial.println("📡 BLE advertising restarted");
    wasConnected = false;
  }
  if (!wasConnected && clientConnected) wasConnected = true;

  esp_err_t res = twai_receive(&message, pdMS_TO_TICKS(100));
  if (res == ESP_OK) {
    lastMessageTime = millis();
    totalMessages++;

    Serial.printf("CAN Msg [%d] ID: 0x%08X, Data:", totalMessages, message.identifier);
    for (int i = 0; i < message.data_length_code; i++) Serial.printf(" 0x%02X", message.data[i]);
    Serial.println();

    decodeCANID(message.identifier);

    // Decode KLS messages
    handleKLSMessage (message.identifier, message.data, message.data_length_code); // Msg 1: speed/current
    handleKLSMessage2(message.identifier, message.data, message.data_length_code); // Msg 2: throttle

    // Update brake logic based on latest decoded values
    updateBrakeStateMachine();

    // Send CAN over BLE if client connected
    if (clientConnected) {
      uint8_t sendData[12];
      sendData[0] = 0x01; // Type = CAN
      sendData[1] = (message.identifier >> 24) & 0xFF;
      sendData[2] = (message.identifier >> 16) & 0xFF;
      sendData[3] = (message.identifier >> 8)  & 0xFF;
      sendData[4] =  message.identifier        & 0xFF;

      memcpy(&sendData[5], message.data, message.data_length_code);
      bleCharacteristic->setValue(sendData, 5 + message.data_length_code);
      bleCharacteristic->notify();
      Serial.println("📤 CAN Data sent via BLE");
    }

  } else if (millis() - lastMessageTime > 5000) {
    Serial.println("❗ No CAN data for 5 seconds!");
    if (status.state == TWAI_STATE_BUS_OFF) {
      Serial.println("⚠ CAN Bus OFF. Attempting recovery...");
      twai_stop();
      if (twai_start() == ESP_OK) {
        Serial.println("✅ CAN Bus Recovered");
      } else {
        Serial.println("❌ Recovery Failed");
      }
    } else {
      Serial.println("⚠ No RX activity. Check wiring/power.");
    }
    lastMessageTime = millis();
  }

  // Digital IO updates (BLE)
  if (millis() - lastDIPrint >= DI_PRINT_PERIOD_MS) {
    uint16_t packedGPIO = packDigitalInputs();
    lastDIPrint = millis();

    if (clientConnected) {
      uint8_t data[3];
      data[0] = 0x02;                 // Type = GPIO
      data[1] = (packedGPIO >> 8) & 0xFF;
      data[2] =  packedGPIO       & 0xFF;
      bleCharacteristic->setValue(data, 3);
      bleCharacteristic->notify();
    }
  }

  // Watchdog DONE pulse every 2 minutes
  const unsigned long WD_PULSE_INTERVAL_MS = 2UL * 60UL * 1000UL;
  static unsigned long lastWDPulseTime = 0;

  if (millis() - lastWDPulseTime >= WD_PULSE_INTERVAL_MS) {
    digitalWrite(WD_DONE, HIGH);
    delay(10);
    digitalWrite(WD_DONE, LOW);
    Serial.println("✅ Watchdog DONE pulse sent");
    lastWDPulseTime = millis();
  }

  // Handle WAKE interrupt
  if (wakeTriggered) {
    wakeTriggered = false;
    digitalWrite(WD_DONE, HIGH);
    delay(20);
    digitalWrite(WD_DONE, LOW);
    Serial.println("✅ Watchdog DONE pulse sent from WAKE");
  }
}
