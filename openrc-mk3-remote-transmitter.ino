/*
 * ======================================================================================
 * OpenRC Tractor MK3 - Remote Controller Firmware (Transmitter)
 * ======================================================================================
 * Hardware Components:
 * - Microcontroller: ESP32 DevKit V1
 * - Display: JESSINIE 1.8" ST7735 TFT LCD (128x160 RGB SPI)
 * - Joysticks: DIYables Dual 2-Axis Analog Joysticks with Push Buttons
 * - Battery: Meshnology 1S 3.7V 1000mAh LiPo Battery (via MT3608 5V Boost)
 * - Wireless Protocol: ESP-NOW (2.4 GHz low-latency radio link)
 * ======================================================================================
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// --------------------------------------------------------------------------------------
// Pin Definitions
// --------------------------------------------------------------------------------------
// JESSINIE 1.8" ST7735 TFT Display (SPI)
#define TFT_CS         5    // Chip Select
#define TFT_RST        4    // Reset
#define TFT_DC         2    // Data/Command (A0)
#define TFT_MOSI       23   // SPI SDA / MOSI
#define TFT_SCLK       18   // SPI SCL / SCK

// DIYables Joysticks
#define JOY_L_Y        33   // Left Joystick Y-Axis (Throttle)
#define JOY_L_X        32   // Left Joystick X-Axis (Aux / Trim)
#define JOY_L_BTN      25   // Left Joystick Push Button

#define JOY_R_X        34   // Right Joystick X-Axis (Steering)
#define JOY_R_Y        35   // Right Joystick Y-Axis (Rear Lift)
#define JOY_R_BTN      26   // Right Joystick Push Button

// Power & Battery Telemetry (Meshnology 1S 3.7V LiPo)
#define BATT_SENSE_PIN 36   // ADC1_CH0 (Connected to 10k/10k voltage divider on 1S battery)

// --------------------------------------------------------------------------------------
// Data Structures for ESP-NOW Communication
// --------------------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  int16_t throttle;      // -500 to +500 (Forward / Reverse)
  int16_t steering;      // -500 to +500 (Left / Right)
  int16_t rear_lift;     // -500 to +500 (Up / Down for linear actuators)
  bool btn_left;         // Left joystick button state
  bool btn_right;        // Right joystick button state
} ControlData;

typedef struct __attribute__((packed)) {
  float tractor_voltage; // URGENEX 2S Battery Pack Voltage (V)
  uint8_t tractor_pct;   // URGENEX 2S Battery Charge Percentage (0-100%)
  bool signal_ok;        // Telemetry link health flag
} TelemetryData;

ControlData txData;
TelemetryData rxData;

// Initialize Adafruit ST7735 Display Instance
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Target Tractor Receiver MAC Address (Replace with your Tractor ESP32 MAC Address)
uint8_t tractorMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// Connection tracking
unsigned long lastRecvTime = 0;
bool isConnected = false;
unsigned long lastTxTime = 0;
unsigned long lastDisplayUpdate = 0;

// --------------------------------------------------------------------------------------
// ESP-NOW Callbacks
// --------------------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Callback when packet transmission completes
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(TelemetryData)) {
    memcpy(&rxData, incomingData, sizeof(TelemetryData));
    lastRecvTime = millis();
    isConnected = true;
  }
}

// --------------------------------------------------------------------------------------
// Helper Functions
// --------------------------------------------------------------------------------------
uint8_t readRemoteBatteryPct() {
  // Read 1S Battery Voltage through 10k/10k divider (Factor 2.0)
  uint32_t rawADC = analogRead(BATT_SENSE_PIN);
  float voltage = (rawADC / 4095.0) * 3.3 * 2.0; 
  
  // Meshnology 1S LiPo voltage range: 3.3V (0%) to 4.2V (100%)
  if (voltage >= 4.2) return 100;
  if (voltage <= 3.3) return 0;
  return (uint8_t)((voltage - 3.3) / (4.2 - 3.3) * 100.0);
}

void drawUIBase() {
  tft.fillScreen(ST7735_BLACK);

  // Title Header Box
  tft.fillRect(0, 0, 128, 22, ST7735_BLUE);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.setCursor(8, 7);
  tft.print("OPENRC TRACTOR MK3");

  // Divider lines
  tft.drawFastHLine(0, 23, 128, ST7735_CYAN);
  tft.drawFastHLine(0, 60, 128, ST7735_DARKGREY);
  tft.drawFastHLine(0, 110, 128, ST7735_DARKGREY);

  // Connection Label
  tft.setCursor(6, 30);
  tft.setTextColor(ST7735_YELLOW);
  tft.print("ESP-NOW LINK:");

  // Remote Battery Label
  tft.setCursor(6, 68);
  tft.setTextColor(ST7735_GREEN);
  tft.print("REMOTE BATT (1S):");

  // Tractor Battery Label
  tft.setCursor(6, 118);
  tft.setTextColor(ST7735_MAGENTA);
  tft.print("TRACTOR BATT (2S):");
}

void updateUIData() {
  // Check telemetry freshness (timeout 1.0 sec)
  if (millis() - lastRecvTime > 1000) {
    isConnected = false;
  }

  // 1. Connection Status Banner
  tft.fillRect(6, 42, 116, 14, ST7735_BLACK);
  tft.setCursor(6, 42);
  tft.setTextSize(1);
  if (isConnected) {
    tft.setTextColor(ST7735_GREEN);
    tft.print("ONLINE (PAIRED)");
  } else {
    tft.setTextColor(ST7735_RED);
    tft.print("SEARCHING...");
  }

  // 2. Remote Battery Status (Meshnology 1S)
  uint8_t remotePct = readRemoteBatteryPct();
  tft.fillRect(6, 82, 116, 20, ST7735_BLACK);
  tft.setCursor(6, 82);
  tft.setTextSize(2);
  tft.setTextColor(remotePct < 20 ? ST7735_RED : ST7735_WHITE);
  tft.printf("%3d %%", remotePct);

  // 3. Tractor Battery Status (URGENEX 2S)
  tft.fillRect(6, 132, 116, 20, ST7735_BLACK);
  tft.setCursor(6, 132);
  tft.setTextSize(2);
  if (isConnected) {
    tft.setTextColor(rxData.tractor_pct < 20 ? ST7735_RED : ST7735_CYAN);
    tft.printf("%3d %%", rxData.tractor_pct);
  } else {
    tft.setTextColor(ST7735_DARKGREY);
    tft.print(" -- %");
  }
}

// --------------------------------------------------------------------------------------
// Setup Routine
// --------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Pin Configurations
  pinMode(JOY_L_BTN, INPUT_PULLUP);
  pinMode(JOY_R_BTN, INPUT_PULLUP);
  pinMode(BATT_SENSE_PIN, INPUT);

  // Initialize ST7735 TFT Screen
  tft.initR(INITR_BLACKTAB); // 1.8" TFT ST7735 standard init
  tft.setRotation(0);        // Portrait Mode (128x160)
  drawUIBase();

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register Tractor Peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, tractorMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer broadcast");
  }
}

// --------------------------------------------------------------------------------------
// Main Program Loop
// --------------------------------------------------------------------------------------
void loop() {
  // Read Analog Joysticks (0 - 4095) & Map to centered values (-500 to +500)
  int rawThrottle = analogRead(JOY_L_Y);
  int rawSteering = analogRead(JOY_R_X);
  int rawRearLift = analogRead(JOY_R_Y);

  txData.throttle  = map(rawThrottle, 0, 4095, 500, -500);  // Inverted for intuitiveness
  txData.steering  = map(rawSteering, 0, 4095, -500, 500);
  txData.rear_lift = map(rawRearLift, 0, 4095, -500, 500);

  // Deadzone filtering (+/- 30)
  if (abs(txData.throttle) < 30) txData.throttle = 0;
  if (abs(txData.steering) < 30) txData.steering = 0;
  if (abs(txData.rear_lift) < 30) txData.rear_lift = 0;

  txData.btn_left  = (digitalRead(JOY_L_BTN) == LOW);
  txData.btn_right = (digitalRead(JOY_R_BTN) == LOW);

  // Send Control Packet via ESP-NOW at 50Hz (every 20ms)
  if (millis() - lastTxTime >= 20) {
    lastTxTime = millis();
    esp_now_send(tractorMacAddress, (uint8_t *) &txData, sizeof(txData));
  }

  // Refresh Screen UI at 5Hz (every 200ms)
  if (millis() - lastDisplayUpdate >= 200) {
    lastDisplayUpdate = millis();
    updateUIData();
  }
}
