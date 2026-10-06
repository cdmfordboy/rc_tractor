/*
 * ======================================================================================
 * OpenRC Tractor MK3 - Onboard Control Unit Firmware (Receiver)
 * ======================================================================================
 * Hardware Components:
 * - Microcontroller: ESP32 DevKit V1
 * - Drive Motor Driver: TB6612FNG Dual H-Bridge Motor Driver
 * - Motors: Dual Drive Motors (Left & Right rear wheels)
 * - Steering: Standard Ackerman Steering Servo
 * - Attachments: Dual Linear Actuators for Rear Lift Arms
 * - Battery: URGENEX 2S 7.4V 2000mAh Li-ion Battery (with JST Plug)
 * - Wireless Protocol: ESP-NOW (2.4 GHz low-latency radio link)
 * ======================================================================================
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// --------------------------------------------------------------------------------------
// Pin Definitions
// --------------------------------------------------------------------------------------
// TB6612FNG Dual Motor Driver Pins (Dual Drive Motors)
#define MOTOR_A_PWM    16   // Left Motor PWM Speed
#define MOTOR_A_IN1    17   // Left Motor Dir 1
#define MOTOR_A_IN2    5    // Left Motor Dir 2

#define MOTOR_B_PWM    18   // Right Motor PWM Speed
#define MOTOR_B_IN1    19   // Right Motor Dir 1
#define MOTOR_B_IN2    21   // Right Motor Dir 2

#define MOTOR_STBY     22   // TB6612FNG Standby (HIGH = Active, LOW = Standby)

// Steering Servo (Ackerman Front Axle)
#define STEERING_SERVO_PIN 13

// Rear Lift Linear Actuator Pins
#define ACTUATOR_PWM   27   // Linear Actuator PWM Speed
#define ACTUATOR_IN1   14   // Linear Actuator Dir 1
#define ACTUATOR_IN2   12   // Linear Actuator Dir 2

// Battery Telemetry Sensing (URGENEX 2S 7.4V Li-ion Battery)
#define BATT_SENSE_PIN 39   // ADC1_CH3 (Connected to 30k/10k voltage divider)

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
  float tractor_voltage; // URGENEX 2S Voltage (V)
  uint8_t tractor_pct;   // URGENEX 2S Battery Charge Percentage (0-100%)
  bool signal_ok;        // Telemetry link health flag
} TelemetryData;

ControlData rxData;
TelemetryData txTelemetry;

Servo steeringServo;

// Timing & Failsafe Management
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 300; // 300ms watchdog threshold
unsigned long lastTelemetryTx = 0;

// Remote Controller MAC Address (Auto-captured upon receiving first control packet)
uint8_t remoteMacAddress[6] = {0};
bool remotePeerAdded = false;

// PWM Channels for Motor Control
#define PWM_FREQ       20000 // 20 kHz ultrasonic motor PWM
#define PWM_RES        8     // 8-bit resolution (0-255)
#define CHAN_MOTOR_A   0
#define CHAN_MOTOR_B   1
#define CHAN_ACTUATOR  2

// --------------------------------------------------------------------------------------
// Motor & Actuator Control Drivers
// --------------------------------------------------------------------------------------
void setMotorLeft(int speed) { // speed: -255 to +255
  if (speed > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    ledcWrite(CHAN_MOTOR_A, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    ledcWrite(CHAN_MOTOR_A, -speed);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    ledcWrite(CHAN_MOTOR_A, 0);
  }
}

void setMotorRight(int speed) { // speed: -255 to +255
  if (speed > 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    ledcWrite(CHAN_MOTOR_B, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    ledcWrite(CHAN_MOTOR_B, -speed);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    ledcWrite(CHAN_MOTOR_B, 0);
  }
}

void setRearLiftActuator(int speed) { // speed: -255 to +255
  if (speed > 0) {
    digitalWrite(ACTUATOR_IN1, HIGH);
    digitalWrite(ACTUATOR_IN2, LOW);
    ledcWrite(CHAN_ACTUATOR, speed);
  } else if (speed < 0) {
    digitalWrite(ACTUATOR_IN1, LOW);
    digitalWrite(ACTUATOR_IN2, HIGH);
    ledcWrite(CHAN_ACTUATOR, -speed);
  } else {
    digitalWrite(ACTUATOR_IN1, LOW);
    digitalWrite(ACTUATOR_IN2, LOW);
    ledcWrite(CHAN_ACTUATOR, 0);
  }
}

void stopAllOutputs() {
  setMotorLeft(0);
  setMotorRight(0);
  setRearLiftActuator(0);
  steeringServo.write(90); // Center front steering
}

// --------------------------------------------------------------------------------------
// Battery Reading Helper (URGENEX 2S 7.4V Battery)
// --------------------------------------------------------------------------------------
void readTractorBattery(float &voltage, uint8_t &percentage) {
  // Read 2S Battery Voltage through 30k/10k divider (Factor 4.0)
  uint32_t rawADC = analogRead(BATT_SENSE_PIN);
  voltage = (rawADC / 4095.0) * 3.3 * 4.0; 

  // URGENEX 2S Li-ion pack range: 6.0V (0%) to 8.4V (100%)
  if (voltage >= 8.4) percentage = 100;
  else if (voltage <= 6.0) percentage = 0;
  else percentage = (uint8_t)((voltage - 6.0) / (8.4 - 6.0) * 100.0);
}

// --------------------------------------------------------------------------------------
// ESP-NOW Receive Callback
// --------------------------------------------------------------------------------------
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlData)) {
    memcpy(&rxData, incomingData, sizeof(ControlData));
    lastPacketTime = millis();

    // Auto-register Remote peer if not already registered
    if (!remotePeerAdded) {
      memcpy(remoteMacAddress, mac, 6);
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, remoteMacAddress, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        remotePeerAdded = true;
      }
    }
  }
}

// --------------------------------------------------------------------------------------
// Setup Routine
// --------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motor Driver Direction & Standby Pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH); // Enable motor driver

  // Actuator Control Pins
  pinMode(ACTUATOR_IN1, OUTPUT);
  pinMode(ACTUATOR_IN2, OUTPUT);

  // PWM Configuration for Drives & Linear Actuators
  ledcSetup(CHAN_MOTOR_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_A_PWM, CHAN_MOTOR_A);

  ledcSetup(CHAN_MOTOR_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_B_PWM, CHAN_MOTOR_B);

  ledcSetup(CHAN_ACTUATOR, PWM_FREQ, PWM_RES);
  ledcAttachPin(ACTUATOR_PWM, CHAN_ACTUATOR);

  // Steering Servo Setup
  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(STEERING_SERVO_PIN, 500, 2500);

  stopAllOutputs();

  // ESP-NOW Initialization
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

// --------------------------------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------------------------------
void loop() {
  // 1. Safety Watchdog / Connection Check
  if (millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
    stopAllOutputs(); // Shut down drive and actuators if connection drops
  } else {
    // 2. Drive Control with Optional Differential Steering Assistance
    int throttlePwm = map(rxData.throttle, -500, 500, -255, 255);
    int steeringPwm = map(rxData.steering, -500, 500, -100, 100);

    // Mix throttle and steering for dual drive rear motors
    int leftMotorSpeed  = constrain(throttlePwm + steeringPwm, -255, 255);
    int rightMotorSpeed = constrain(throttlePwm - steeringPwm, -255, 255);

    setMotorLeft(leftMotorSpeed);
    setMotorRight(rightMotorSpeed);

    // 3. Ackerman Front Steering Servo Position (45 deg to 135 deg, centered at 90 deg)
    int servoAngle = map(rxData.steering, -500, 500, 45, 135);
    steeringServo.write(servoAngle);

    // 4. Rear Lift Arms Linear Actuators
    int actuatorSpeed = map(rxData.rear_lift, -500, 500, -255, 255);
    setRearLiftActuator(actuatorSpeed);
  }

  // 5. Transmit Telemetry Back to Remote (10 Hz / every 100ms)
  if (remotePeerAdded && (millis() - lastTelemetryTx >= 100)) {
    lastTelemetryTx = millis();
    readTractorBattery(txTelemetry.tractor_voltage, txTelemetry.tractor_pct);
    txTelemetry.signal_ok = (millis() - lastPacketTime <= FAILSAFE_TIMEOUT_MS);

    esp_now_send(remoteMacAddress, (uint8_t *) &txTelemetry, sizeof(txTelemetry));
  }
}
