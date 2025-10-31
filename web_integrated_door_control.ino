/*
 * ESP32 Smart Door Control System - Complete Web Dashboard with AI
 * Version: 4.2 - WiFi Web Server + AI Chatbot
 * Features: WiFi, Web Server, AI Chatbot, Real-time WebSocket, CSV Export
 *
 * HARDWARE FEATURES:
 * ==================
 * - HC-SR04 Ultrasonic Distance Detection with automatic relay/lighting control
 * - IR Remote Control with Simple Door Unlock
 * - Servo Motor for Physical Door Control (0° = Closed, 90° = Open)
 * - 20x4 I2C LCD Real-time Status Display with Countdown Timer
 * - Emergency Exit Button (Opens door for 10 seconds)
 * - Active Buzzer for Audio Alerts (Door open + Temp/Humidity alerts)
 * - Relay-Controlled Lighting (Auto-on when obstacle detected)
 * - Status LED with State-based Blinking Patterns
 * - DHT22 Temperature and Humidity Sensor with Alert System
 *
 * PIN DEFINITIONS:
 * ================
 * ULTRASONIC_TRIG_PIN = 19 - Ultrasonic sensor trigger pin
 * ULTRASONIC_ECHO_PIN = 18 - Ultrasonic sensor echo pin
 * IR_RECEIVER_PIN = 14     - IR remote receiver (Changed to GPIO 14)
 * SERVO_PIN = 26           - Door lock servo motor
 * EMERGENCY_EXIT_PIN = 34  - Emergency exit button (active LOW with pullup)
 * BUZZER_PIN = 15          - Buzzer for alerts (sounds while door open)
 * RELAY_PIN = 27           - Relay for lighting control (auto-on with obstacle)
 * LED_STATUS_PIN = 5       - Status indicator LED
 * DHT_PIN = 25             - DHT22 temperature/humidity sensor
 *
 * LCD I2C ADDRESS: 0x27
 * LCD SIZE: 20x4 characters
 *
 * IR REMOTE OPERATION:
 * ====================
 * - Press OK button to unlock door (opens for 10 seconds)
 * - Press * to toggle LED ON/OFF
 * - Press # to toggle FAN ON/OFF
 *
 * SYSTEM BEHAVIOR:
 * ================
 * 1. Ultrasonic Sensor Control:
 *    - Obstacle detected (< threshold) → Relay ON (light on)
 *    - No obstacle → Relay OFF (light off)
 *
 * 2. Door Control (Three methods):
 *    a) Emergency Push Button:
 *       - Press button → Door opens 90° → Stays open 10 seconds → Auto-closes
 *    b) IR Remote OK Button:
 *       - Press OK → Door opens 90° → Stays open 10 seconds → Auto-closes
 *    c) Automatic Ventilation (Temperature/Humidity Alert):
 *       - Temp ≥85°C OR Humidity ≥85% → Door auto-opens for ventilation
 *       - Door stays open until BOTH temp <85°C AND humidity <85%
 *       - No auto-close timer (controlled by sensor readings)
 *
 * 3. Buzzer Behavior:
 *    - Buzzer sounds while door is open (manual/remote)
 *    - Buzzer sounds continuously during temp/humidity alert
 *    - Different beep patterns for different alert types
 *
 * 4. LCD Display:
 *    - Line 1: System title
 *    - Line 2: Door status (VENTILATION mode if auto-opened)
 *    - Line 3: Alert messages / Distance / LED and Fan status
 *    - Line 4: Temperature, Humidity, Relay status
 *
 * Author: Marie Door Control System
 * Version: 3.2 - Simplified IR Remote Controls
 * Last Updated: 2025-10-24
 */

#include <ESP32Servo.h>
#include <IRremote.hpp>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include "secrets.h"

// LCD Library handling for ESP32 compatibility
#if defined(ARDUINO_ARCH_ESP32)
  #if defined(__has_include)
    #if __has_include(<ESP32_LiquidCrystal_I2C.h>)
      #include <ESP32_LiquidCrystal_I2C.h>
      using LCD_CLASS = ESP32_LiquidCrystal_I2C;
    #else
      #include <LiquidCrystal_I2C.h>
      using LCD_CLASS = LiquidCrystal_I2C;
    #endif
  #else
    #include <LiquidCrystal_I2C.h>
    using LCD_CLASS = LiquidCrystal_I2C;
  #endif
#else
  #include <LiquidCrystal_I2C.h>
  using LCD_CLASS = LiquidCrystal_I2C;
#endif

// Pin Definitions (Verified for ESP32-WROOM-32)
// I2C Pins (Implicit, handled by Wire.begin()):
//   SDA = GPIO 21 (LCD Data)
//   SCL = GPIO 22 (LCD Clock)
//
// Digital Pins:
#define ULTRASONIC_TRIG_PIN 19  // Ultrasonic trigger (OUTPUT)
#define ULTRASONIC_ECHO_PIN 18  // Ultrasonic echo (INPUT)
#define IR_RECEIVER_PIN 14      // IR receiver
#define SERVO_PIN 26            // Servo motor PWM (OUTPUT) - DAC pin
#define EMERGENCY_EXIT_PIN 34   // Emergency button (INPUT) - Input-only, needs external 10kΩ pull-up
#define BUZZER_PIN 15           // Active buzzer (OUTPUT)
#define RELAY_PIN 27            // Relay module (OUTPUT)
#define LED_STATUS_PIN 5        // Status LED (OUTPUT) - Strapping pin
#define DHT_PIN 25              // DHT22 data (INPUT/OUTPUT) - Needs external 4.7k-10kΩ pull-up, ADC2 pin
#define DHT_TYPE DHT22

// Ultrasonic sensor detection threshold (in centimeters)
#define ULTRASONIC_DISTANCE_THRESHOLD 10

// LCD Configuration
#define I2C_ADDR 0x27
#define LCD_COLUMNS 20
#define LCD_LINES 4

// IR Remote Button Codes (Updated for new IRremote.hpp library)
#define IR_BTN_OK 0x1C      // OK button (to unlock door)
#define IR_BTN_STAR 0x16    // * button (to toggle LED)
#define IR_BTN_HASH 0x0D    // # button (to toggle FAN)

// Servo Positions
#define DOOR_CLOSED_POS 0
#define DOOR_OPEN_POS 90

// Timing Constants
#define DHT_READ_INTERVAL 2000UL    // Read DHT every 2 seconds

// Temperature and Humidity Alert Thresholds
#define TEMP_THRESHOLD 85.0         // Temperature alert threshold (°C)
#define HUMIDITY_THRESHOLD 85.0     // Humidity alert threshold (%)

// Pin definitions
#define TOGGLE_LED_PIN 13       // Toggle LED (OUTPUT)
#define TOGGLE_BUTTON_PIN 35    // Toggle button (INPUT) - external pull-up
#define FAN_MOTOR_PIN 32        // DC Fan motor (OUTPUT) - PWM
#define FAN_BUTTON_PIN 23       // Fan button (INPUT) - external pull-up

// WiFi Configuration (loaded from secrets.h)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// API Keys (loaded from secrets.h, can be updated from web interface)
String GEMINI_API_KEY = GEMINI_API_KEY_DEFAULT;
String FORMSPREE_ENDPOINT = FORMSPREE_ENDPOINT_DEFAULT;

// Hardware Objects
Servo doorServo;
LCD_CLASS lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
DHT dht(DHT_PIN, DHT_TYPE);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

// Door State
bool doorOpen = false;
unsigned long doorOpenTime = 0;
bool autoOpenedByTempAlert = false;  // Track if door was auto-opened by temp/humidity alert

// Ultrasonic Sensor
float currentDistance = 999.0;
bool obstacleDetected = false;

// DHT Sensor
float temperature = 0.0;
float humidity = 0.0;
unsigned long lastDHTRead = 0;
bool tempHumidityAlert = false;  // Alert flag for high temp/humidity

// Emergency Button
bool emergencyButtonPressed = false;

// Variables for toggle states
bool ledState = false;  // LED initially off
bool fanState = false;  // Fan initially off
bool buzzerState = false;  // Manual buzzer control initially off

// Button debouncing variables for LED
int lastLedButtonState = HIGH;
unsigned long lastLedDebounceTime = 0;
int ledButtonState = HIGH;

// Button debouncing variables for Fan
int lastFanButtonState = HIGH;
unsigned long lastFanDebounceTime = 0;
int fanButtonState = HIGH;

const unsigned long debounceDelay = 50;

// PWM settings for fan
const int fanFreq = 25000;
const int fanResolution = 8;
const int FAN_PWM_CHANNEL = 8;  // Changed from 0 to 8 to avoid servo conflict

// Configurable thresholds (loaded from Preferences)
float tempThreshold = 30.0;
float humidityThreshold = 70.0;
float distanceThreshold = 10.0;

// Configurable servo angles
int servoClosedAngle = DOOR_CLOSED_POS;
int servoOpenAngle = DOOR_OPEN_POS;

// Configurable door timing (in seconds)
int doorOpenDuration = 10;  // Default 10 seconds

// Data logging
#define MAX_LOG_ENTRIES 100
struct LogEntry {
  unsigned long timestamp;
  float temp;
  float hum;
  float dist;
  bool doorState;
  bool ledState;
  bool fanState;
};
LogEntry dataLog[MAX_LOG_ENTRIES];
int logIndex = 0;

// Alert tracking
unsigned long lastAlertTime = 0;
const unsigned long ALERT_COOLDOWN = 300000;

// ===============================================
// SETUP FUNCTION
// ===============================================
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  delay(100);

  Serial.println("\n\n========================================");
  Serial.println("ESP32 Door Control System");
  Serial.println("Version 4.2 - WiFi Web Server + AI");
  Serial.println("========================================");

  // Load settings from Preferences
  preferences.begin("doorcontrol", false);
  tempThreshold = preferences.getFloat("tempThresh", 30.0);
  humidityThreshold = preferences.getFloat("humThresh", 70.0);
  distanceThreshold = preferences.getFloat("distThresh", 10.0);
  servoClosedAngle = preferences.getInt("servoClosed", DOOR_CLOSED_POS);
  servoOpenAngle = preferences.getInt("servoOpen", DOOR_OPEN_POS);
  doorOpenDuration = preferences.getInt("doorDuration", 10);
  GEMINI_API_KEY = preferences.getString("geminiKey", GEMINI_API_KEY_DEFAULT);
  FORMSPREE_ENDPOINT = preferences.getString("formspree", FORMSPREE_ENDPOINT_DEFAULT);
  Serial.println("Settings loaded from memory (including API keys)");

  // Initialize Pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(EMERGENCY_EXIT_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);

  // Test buzzer at startup (3 short beeps)
  Serial.println("========================================");
  Serial.print("TESTING BUZZER ON GPIO ");
  Serial.print(BUZZER_PIN);
  Serial.println("...");
  Serial.print("Buzzer Pin Number: ");
  Serial.println(BUZZER_PIN);

  for (int i = 0; i < 3; i++) {
    Serial.print("Beep ");
    Serial.print(i + 1);
    Serial.println(" - Setting GPIO HIGH");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    Serial.println("- Setting GPIO LOW");
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
  Serial.println("Buzzer test complete!");
  Serial.println("========================================");

  // Initialize I2C LCD with explicit I2C pins and multiple init attempts for stability
  Serial.println("Initializing LCD...");
  Wire.begin(21, 22); // SDA=21, SCL=22 (ESP32 default I2C pins)
  delay(100);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  delay(100);
  displayStartupMessage();
  delay(2000);
  Serial.println("LCD initialized!");

  // Initialize DHT Sensor
  Serial.println("Initializing DHT22 sensor...");
  dht.begin();
  delay(2000);
  Serial.println("DHT22 initialized!");

  // Initialize IR Receiver (Updated for new library)
  Serial.println("Initializing IR Receiver...");
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR Receiver initialized with new library!");

  // Initialize Servo Motor (Simplified for ESP32 2.0.17 compatibility)
  Serial.println("Initializing Servo Motor...");
  doorServo.attach(SERVO_PIN);
  doorServo.write(servoClosedAngle);
  delay(1000);
  Serial.print("Servo initialized at ");
  Serial.print(servoClosedAngle);
  Serial.println("° (closed position)!");

  // Initialize toggle pins
  pinMode(TOGGLE_LED_PIN, OUTPUT);
  pinMode(TOGGLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(FAN_BUTTON_PIN, INPUT_PULLUP);

  // Set initial states
  digitalWrite(TOGGLE_LED_PIN, LOW);

  // Setup PWM for fan motor (ESP32 2.x API)
  ledcSetup(FAN_PWM_CHANNEL, fanFreq, fanResolution);
  ledcAttachPin(FAN_MOTOR_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi Connection Failed!");
  }

  // Initialize Web Server (will be added in next step)
  initWebServer();

  Serial.println("\n========================================");
  Serial.println("SYSTEM READY!");
  Serial.println("Features:");
  Serial.println("- WiFi Web Dashboard");
  Serial.println("- AI Chatbot Assistant");
  Serial.println("- IR Remote OK: Unlock door");
  Serial.println("- IR Remote *: Toggle LED");
  Serial.println("- IR Remote #: Toggle FAN");
  Serial.println("- Push button for emergency exit");
  Serial.println("- Ultrasonic obstacle detection");
  Serial.println("- Temperature/Humidity monitoring");
  Serial.println("========================================\n");
}

// ===============================================
// MAIN LOOP
// ===============================================
void loop() {
  ws.cleanupClients();

  // Read all sensors and inputs
  readUltrasonicSensor();
  readDHTSensor();
  checkEmergencyButton();
  handleIRRemote();

  // Button handling for LED and Fan
  handleLedToggle();
  handleFanToggle();

  // Check for automatic door opening due to temp/humidity
  checkTempHumidityAlert();

  // Update door auto-close (if opened manually or by remote)
  updateDoorAutoClose();

  // Update outputs
  updateRelay();
  updateBuzzer();
  updateLED();
  updateDisplay();

  // Broadcast sensor data and log periodically
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast > 1000) {
    broadcastSensorData();
    logData();
    lastBroadcast = millis();
  }

  delay(50);  // Small delay for system stability
}

// ===============================================
// INPUT HANDLING FUNCTIONS
// ===============================================

void readUltrasonicSensor() {
  static unsigned long lastRead = 0;
  
  // Read every 100ms to avoid excessive triggering
  if (millis() - lastRead < 100) {
    return;
  }
  lastRead = millis();
  
  // Trigger ultrasonic pulse
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  // Read echo pulse duration
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);
  
  // Calculate distance in cm (speed of sound: 343 m/s = 0.0343 cm/µs)
  if (duration > 0) {
    currentDistance = duration * 0.0343 / 2;
    obstacleDetected = (currentDistance < ULTRASONIC_DISTANCE_THRESHOLD);
  } else {
    currentDistance = 999.0;
    obstacleDetected = false;
  }
}

void readDHTSensor() {
  // Read DHT sensor at defined interval
  if (millis() - lastDHTRead < DHT_READ_INTERVAL) {
    return;
  }
  lastDHTRead = millis();
  
  float newTemp = dht.readTemperature();
  float newHumidity = dht.readHumidity();
  
  if (!isnan(newTemp) && !isnan(newHumidity)) {
    temperature = newTemp;
    humidity = newHumidity;
  }
}

void checkEmergencyButton() {
  static unsigned long lastButtonPress = 0;
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(EMERGENCY_EXIT_PIN);
  
  // Detect button press (falling edge with debounce)
  if (currentButtonState == LOW && lastButtonState == HIGH && 
      (millis() - lastButtonPress > 500)) {
    Serial.println("========================================");
    Serial.println("EMERGENCY BUTTON PRESSED!");
    Serial.println("Opening door...");
    Serial.println("========================================");
    
    openDoorManually();
    lastButtonPress = millis();
  }
  
  lastButtonState = currentButtonState;
}

void handleIRRemote() {
  if (IrReceiver.decode()) {
    // Only process if it's not a repeat
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      uint8_t command = IrReceiver.decodedIRData.command;
      
      Serial.print("IR Command received: 0x");
      Serial.println(command, HEX);
      
      // Handle OK button - Unlock door
      if (command == IR_BTN_OK) {
        Serial.println("========================================");
        Serial.println("IR Remote: OK pressed - UNLOCKING DOOR");
        Serial.println("========================================");
        openDoorManually();
      }
      // Handle * button - Toggle LED
      else if (command == IR_BTN_STAR) {
        ledState = !ledState;
        digitalWrite(TOGGLE_LED_PIN, ledState ? HIGH : LOW);
        Serial.print("IR Remote: * pressed - LED ");
        Serial.println(ledState ? "ON" : "OFF");
      }
      // Handle # button - Toggle FAN
      else if (command == IR_BTN_HASH) {
        fanState = !fanState;
        ledcWrite(FAN_PWM_CHANNEL, fanState ? 255 : 0);
        Serial.print("IR Remote: # pressed - FAN ");
        Serial.println(fanState ? "ON" : "OFF");
      }
    }
    
    IrReceiver.resume();
  }
}

void handleLedToggle() {
  int reading = digitalRead(TOGGLE_BUTTON_PIN);
  
  if (reading != lastLedButtonState) {
    lastLedDebounceTime = millis();
  }
  
  if ((millis() - lastLedDebounceTime) > debounceDelay) {
    if (reading != ledButtonState) {
      ledButtonState = reading;
      
      if (ledButtonState == LOW) {
        ledState = !ledState;
        digitalWrite(TOGGLE_LED_PIN, ledState ? HIGH : LOW);
      }
    }
  }
  
  lastLedButtonState = reading;
}

void handleFanToggle() {
  int reading = digitalRead(FAN_BUTTON_PIN);
  
  if (reading != lastFanButtonState) {
    lastFanDebounceTime = millis();
  }
  
  if ((millis() - lastFanDebounceTime) > debounceDelay) {
    if (reading != fanButtonState) {
      fanButtonState = reading;
      
      if (fanButtonState == LOW) {
        fanState = !fanState;
        ledcWrite(FAN_PWM_CHANNEL, fanState ? 255 : 0);
        Serial.print("Fan button pressed - FAN ");
        Serial.println(fanState ? "ON" : "OFF");
      }
    }
  }
  
  lastFanButtonState = reading;
}

void checkTempHumidityAlert() {
  bool alertCondition = (temperature > tempThreshold) || (humidity > humidityThreshold);

  // Alert condition started
  if (alertCondition && !tempHumidityAlert) {
    Serial.println("========================================");
    Serial.println("⚠ TEMPERATURE/HUMIDITY ALERT!");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
    Serial.println("Opening door for ventilation...");
    Serial.println("========================================");

    tempHumidityAlert = true;
    openDoorForVentilation();

    // Auto-activate fan during alert
    if (!fanState) {
      fanState = true;
      ledcWrite(FAN_PWM_CHANNEL, 255);
    }

    // Send email alert (with cooldown)
    if (millis() - lastAlertTime > ALERT_COOLDOWN) {
      sendEmailAlert();
      lastAlertTime = millis();
    }
  }
  // Alert condition cleared
  else if (!alertCondition && tempHumidityAlert) {
    Serial.println("========================================");
    Serial.println("✓ Temperature/Humidity normalized");
    Serial.println("Closing door...");
    Serial.println("========================================");

    tempHumidityAlert = false;
    closeDoor();
  }
}

// ===============================================
// DOOR CONTROL FUNCTIONS
// ===============================================

void openDoorManually() {
  if (!doorOpen || autoOpenedByTempAlert) {
    doorOpen = true;
    autoOpenedByTempAlert = false;
    doorOpenTime = millis();
    doorServo.write(servoOpenAngle);
    Serial.print("Door opened manually to ");
    Serial.print(servoOpenAngle);
    Serial.println("° (10 second timer started)");
  }
}

void openDoorForVentilation() {
  doorOpen = true;
  autoOpenedByTempAlert = true;
  doorServo.write(servoOpenAngle);
  Serial.print("Door opened for ventilation to ");
  Serial.print(servoOpenAngle);
  Serial.println("° (no auto-close timer)");
}

void closeDoor() {
  if (doorOpen) {
    doorOpen = false;
    autoOpenedByTempAlert = false;
    doorServo.write(servoClosedAngle);
    Serial.print("Door closed to ");
    Serial.print(servoClosedAngle);
    Serial.println("°");
  }
}

void updateDoorAutoClose() {
  // Only auto-close if door was opened manually (not by temp/humidity alert)
  if (doorOpen && !autoOpenedByTempAlert) {
    unsigned long doorDurationMs = doorOpenDuration * 1000UL;  // Convert seconds to milliseconds
    if (millis() - doorOpenTime >= doorDurationMs) {
      Serial.println("========================================");
      Serial.print(doorOpenDuration);
      Serial.println(" seconds elapsed - Auto-closing door");
      Serial.println("========================================");
      closeDoor();
    }
  }
}

// ===============================================
// OUTPUT CONTROL FUNCTIONS
// ===============================================

void updateRelay() {
  // Relay turns ON when obstacle is detected
  digitalWrite(RELAY_PIN, obstacleDetected ? HIGH : LOW);
}

void updateBuzzer() {
  static bool lastDoorState = false;

  // Debug: Print when door state changes
  if (doorOpen != lastDoorState) {
    Serial.println("========================================");
    Serial.print("BUZZER UPDATE - Door state changed to: ");
    Serial.println(doorOpen ? "OPEN" : "CLOSED");
    lastDoorState = doorOpen;
    Serial.println("========================================");
  }

  // Buzzer activates if door is open OR temp/humidity alert OR manual control
  if (doorOpen || tempHumidityAlert || buzzerState) {
    // Different beep patterns for different alerts
    static unsigned long lastBeepTime = 0;
    static bool beepState = false;

    unsigned long currentTime = millis();
    unsigned long onInterval, offInterval;

    if (buzzerState && !doorOpen && !tempHumidityAlert) {
      // Manual buzzer control: continuous beep (100ms on, 100ms off)
      onInterval = 100;
      offInterval = 100;
    } else if (doorOpen && !tempHumidityAlert) {
      // Door open: 400ms on, 200ms off (faster beep)
      onInterval = 400;
      offInterval = 200;
    } else if (tempHumidityAlert && !doorOpen) {
      // Temp/Humidity alert only: 600ms on, 400ms off (slower, longer beep)
      onInterval = 600;
      offInterval = 400;
    } else {
      // Multiple alerts: 200ms on, 200ms off (rapid beep - urgent!)
      onInterval = 200;
      offInterval = 200;
    }

    unsigned long interval = beepState ? onInterval : offInterval;

    if (currentTime - lastBeepTime >= interval) {
      beepState = !beepState;
      digitalWrite(BUZZER_PIN, beepState ? HIGH : LOW);
      lastBeepTime = currentTime;
    }
  } else {
    // No alerts - buzzer off
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void updateLED() {
  // LED flashes rapidly ONLY while door is open, OFF when door is closed
  if (doorOpen) {
    static unsigned long lastBlinkTime = 0;
    static bool ledBlinkState = false;
    
    // Fast flash (250ms on/off) when door open
    if (millis() - lastBlinkTime > 250) {
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_STATUS_PIN, ledBlinkState ? HIGH : LOW);
      lastBlinkTime = millis();
    }
  } else {
    // Door closed - LED completely OFF
    digitalWrite(LED_STATUS_PIN, LOW);
  }
}

// ===============================================
// DISPLAY FUNCTIONS
// ===============================================

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastBacklightRefresh = 0;
  static String lastLine0 = "";
  static String lastLine1 = "";
  static String lastLine2 = "";
  static String lastLine3 = "";

  // Refresh backlight every 30 seconds to ensure LCD stays visible
  if (millis() - lastBacklightRefresh > 30000) {
    lcd.backlight();
    lastBacklightRefresh = millis();
  }

  // Update LCD every 500ms to reduce I2C traffic and prevent instability
  if (millis() - lastUpdate < 500) {
    return;
  }
  lastUpdate = millis();
  
  // Build display lines
  String line0 = "== Door Control ==  ";
  String line1 = "";
  String line2 = "";
  String line3 = "";
  
  // Line 2: Door Status with Countdown
  if (doorOpen) {
    if (autoOpenedByTempAlert) {
      // Door auto-opened due to temp/humidity - no countdown
      line1 = "OPEN: VENTILATION   ";
    } else {
      // Door opened manually/by remote - show countdown
      unsigned long elapsed = millis() - doorOpenTime;
      unsigned long doorDurationMs = doorOpenDuration * 1000UL;  // Convert seconds to milliseconds
      unsigned long timeLeft = 0;
      if (elapsed < doorDurationMs) {
        timeLeft = (doorDurationMs - elapsed) / 1000;
      }
      line1 = "Status: OPEN (";
      if (timeLeft < 10) line1 += " ";
      line1 += String(timeLeft);
      line1 += "s)  ";
    }
  } else {
    line1 = "Status: CLOSED      ";
  }
  
  // Line 3: Priority: Alert > Obstacle > LED/Fan status
  if (tempHumidityAlert) {
    // Temperature/Humidity alert has highest priority
    line2 = "⚠ TEMP/HUMID ALERT!";
  } else if (obstacleDetected) {
    line2 = "OBSTACLE DETECTED!  ";
  } else {
    line2 = "LED: " + String(ledState ? "ON " : "OFF") + "Fan: " + String(fanState ? "ON " : "OFF") + "  ";
  }
  
  // Line 4: Temperature, Humidity, Light Status
  line3 = "T:";
  line3 += String(temperature, 1);
  line3 += "C H:";
  line3 += String(humidity, 0);
  line3 += "% L:";
  line3 += obstacleDetected ? "ON " : "OFF";
  
  // Only update lines that have changed to minimize I2C traffic
  if (line0 != lastLine0) {
    lcd.setCursor(0, 0);
    lcd.print(line0);
    lastLine0 = line0;
  }
  
  if (line1 != lastLine1) {
    lcd.setCursor(0, 1);
    lcd.print(line1);
    lastLine1 = line1;
  }
  
  if (line2 != lastLine2) {
    lcd.setCursor(0, 2);
    lcd.print(line2);
    lastLine2 = line2;
  }
  
  if (line3 != lastLine3) {
    lcd.setCursor(0, 3);
    lcd.print(line3);
    lastLine3 = line3;
  }
}

void displayStartupMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Door Control System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  lcd.setCursor(0, 2);
  lcd.print("ESP32 Ready");
  lcd.setCursor(0, 3);
  lcd.print("OK=Door *=LED #=FAN");
}

// ===============================================
// WEB SERVER FUNCTIONS
// ===============================================

void saveSettings() {
  preferences.putFloat("tempThresh", tempThreshold);
  preferences.putFloat("humThresh", humidityThreshold);
  preferences.putFloat("distThresh", distanceThreshold);
  preferences.putInt("servoClosed", servoClosedAngle);
  preferences.putInt("servoOpen", servoOpenAngle);
  preferences.putInt("doorDuration", doorOpenDuration);
  preferences.putString("geminiKey", GEMINI_API_KEY);
  preferences.putString("formspree", FORMSPREE_ENDPOINT);
  Serial.println("Settings saved to memory (including API keys)");
}

void sendEmailAlert() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(FORMSPREE_ENDPOINT);
    http.addHeader("Content-Type", "application/json");

    String jsonData = "{\"name\":\"ESP32 System\",\"_replyto\":\"system@esp32.local\",\"message\":\"ALERT: Temperature: " +
                     String(temperature, 1) + "°C, Humidity: " + String(humidity, 1) + "%\"}";

    int httpCode = http.POST(jsonData);

    if (httpCode > 0) {
      Serial.println("Alert email sent successfully");
    } else {
      Serial.println("Failed to send alert email");
    }
    http.end();
  }
}

void broadcastSensorData() {
  float tempValue = isnan(temperature) ? 0.0 : temperature;
  float humValue = isnan(humidity) ? 0.0 : humidity;
  float distValue = (currentDistance > 400 || currentDistance < 0) ? 999.0 : currentDistance;

  String json = "{";
  json += "\"temperature\":" + String(tempValue, 1) + ",";
  json += "\"humidity\":" + String(humValue, 1) + ",";
  json += "\"distance\":" + String(distValue, 1) + ",";
  json += "\"doorOpen\":" + String(doorOpen ? "true" : "false") + ",";
  json += "\"obstacleDetected\":" + String(obstacleDetected ? "true" : "false") + ",";
  json += "\"tempAlert\":" + String(tempHumidityAlert ? "true" : "false") + ",";
  json += "\"ledState\":" + String(ledState ? "true" : "false") + ",";
  json += "\"fanState\":" + String(fanState ? "true" : "false") + ",";
  json += "\"buzzerState\":" + String(buzzerState ? "true" : "false");
  json += "}";

  ws.textAll(json);
}

void logData() {
  dataLog[logIndex].timestamp = millis();
  dataLog[logIndex].temp = temperature;
  dataLog[logIndex].hum = humidity;
  dataLog[logIndex].dist = currentDistance;
  dataLog[logIndex].doorState = doorOpen;
  dataLog[logIndex].ledState = ledState;
  dataLog[logIndex].fanState = fanState;

  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    broadcastSensorData();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

void initWebServer() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Main dashboard page with tabbed interface
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'><title>ESP32 Door Control</title>";
    html += "<style>*{margin:0;padding:0;box-sizing:border-box}body{font-family:'Segoe UI',sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;padding:20px}";
    html += ".container{max-width:1400px;margin:0 auto;background:#fff;border-radius:20px;box-shadow:0 20px 60px rgba(0,0,0,0.3);overflow:hidden}";
    html += ".header{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:#fff;padding:30px;text-align:center}.header h1{font-size:32px;margin-bottom:5px}.header p{opacity:0.9;font-size:16px}";
    html += ".tabs{display:flex;background:#f8f9fa;border-bottom:2px solid #dee2e6;overflow-x:auto}";
    html += ".tab{padding:15px 30px;cursor:pointer;border:none;background:none;font-size:16px;color:#495057;transition:all 0.3s;white-space:nowrap}";
    html += ".tab:hover{background:#e9ecef}.tab.active{background:#fff;color:#667eea;border-bottom:3px solid #667eea}";
    html += ".tab-content{display:none;padding:30px}.tab-content.active{display:block}";
    html += ".status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:20px;margin-bottom:30px}";
    html += ".status-card{color:#fff;padding:25px;border-radius:15px;box-shadow:0 5px 15px rgba(0,0,0,0.1)}";
    html += ".status-card.temp{background:linear-gradient(135deg,#f093fb 0%,#f5576c 100%)}";
    html += ".status-card.hum{background:linear-gradient(135deg,#4facfe 0%,#00f2fe 100%)}";
    html += ".status-card.dist{background:linear-gradient(135deg,#43e97b 0%,#38f9d7 100%)}";
    html += ".status-card.door{background:linear-gradient(135deg,#fa709a 0%,#fee140 100%)}";
    html += ".status-label{font-size:14px;opacity:0.9;margin-bottom:10px}.status-value{font-size:32px;font-weight:bold}";
    html += ".control-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px}";
    html += ".control-card{background:#f8f9fa;padding:25px;border-radius:15px}.control-card h3{margin-bottom:20px;color:#495057}";
    html += ".btn{padding:12px 24px;border:none;border-radius:8px;font-size:16px;cursor:pointer;transition:all 0.3s;margin:5px}";
    html += ".btn-primary{background:#667eea;color:#fff}.btn-primary:hover{background:#5568d3;transform:translateY(-2px)}";
    html += ".btn-success{background:#28a745;color:#fff}.btn-success:hover{background:#218838;transform:translateY(-2px)}";
    html += ".btn-danger{background:#dc3545;color:#fff}.btn-danger:hover{background:#c82333;transform:translateY(-2px)}";
    html += ".btn-warning{background:#ffc107;color:#000}.btn-warning:hover{background:#e0a800;transform:translateY(-2px)}";
    html += ".toast{position:fixed;bottom:20px;right:20px;background:#333;color:#fff;padding:15px 20px;border-radius:8px;box-shadow:0 4px 12px rgba(0,0,0,0.3);z-index:1000;opacity:0;transform:translateY(100px);transition:all 0.3s;max-width:300px}";
    html += ".toast.show{opacity:1;transform:translateY(0)}.toast.success{background:#28a745}.toast.error{background:#dc3545}.toast.info{background:#2196F3}";
    html += ".chat-input-wrapper{display:flex;gap:10px;padding:15px;background:#fff;border-top:1px solid #dee2e6;flex-wrap:wrap}";
    html += ".chat-input-wrapper input{flex:1;min-width:200px;padding:12px;border:1px solid #ced4da;border-radius:8px}";
    html += ".chat-input-wrapper button{padding:12px 24px;white-space:nowrap}";
    html += "@media(max-width:480px){.chat-input-wrapper{gap:8px}.chat-input-wrapper input{min-width:100%;flex:1 1 100%}.chat-input-wrapper button{flex:1 1 100%;width:100%}}";
    html += "</style><script>";
    html += "function showToast(msg,type='info'){";
    html += "const toast=document.createElement('div');toast.className='toast '+type;toast.textContent=msg;document.body.appendChild(toast);";
    html += "setTimeout(()=>toast.classList.add('show'),10);";
    html += "setTimeout(()=>{toast.classList.remove('show');setTimeout(()=>document.body.removeChild(toast),300);},3000);}";

    html += "function switchTab(t){document.querySelectorAll('.tab').forEach(tab=>tab.classList.remove('active'));";
    html += "document.querySelectorAll('.tab-content').forEach(c=>c.classList.remove('active'));";
    html += "event.target.classList.add('active');document.getElementById(t).classList.add('active');}";
    html += "function updateStatus(){fetch('/api/readings').then(r=>r.json()).then(d=>{";
    html += "document.getElementById('temp-status').textContent=(d.temperature||0).toFixed(1)+'°C';";
    html += "document.getElementById('hum-status').textContent=(d.humidity||0).toFixed(1)+'%';";
    html += "document.getElementById('dist-status').textContent=(d.distance||0).toFixed(1)+' cm';";
    html += "document.getElementById('door-status').textContent=d.doorOpen?'OPEN':'CLOSED';";
    html += "document.getElementById('led-state').textContent=d.ledState?'ON':'OFF';";
    html += "document.getElementById('fan-state').textContent=d.fanState?'ON':'OFF';";
    html += "document.getElementById('buzzer-state').textContent=d.buzzerState?'ON':'OFF';});}";
    html += "setInterval(updateStatus,2000);window.onload=updateStatus;";
    html += "function controlDoor(a){fetch('/api/door',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'action='+a}).then(r=>r.text()).then(m=>{showToast(m,'success');updateStatus()});}";
    html += "function toggleLED(){fetch('/api/led',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'state=toggle'}).then(r=>r.text()).then(m=>{showToast(m,'success');updateStatus()});}";
    html += "function toggleFan(){fetch('/api/fan',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'state=toggle'}).then(r=>r.text()).then(m=>{showToast(m,'success');updateStatus()});}";
    html += "function toggleBuzzer(){fetch('/api/buzzer',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'state=toggle'}).then(r=>r.text()).then(m=>{showToast(m,'success');updateStatus()});}";
    html += "function saveSettings(){";
    html += "const temp=document.getElementById('temp-threshold').value;";
    html += "const hum=document.getElementById('hum-threshold').value;";
    html += "const dist=document.getElementById('dist-threshold').value;";
    html += "const servoClosed=document.getElementById('servo-closed').value;";
    html += "const servoOpen=document.getElementById('servo-open').value;";
    html += "const doorDuration=document.getElementById('door-duration').value;";
    html += "const geminiKey=document.getElementById('gemini-key').value;";
    html += "const formspreeKey=document.getElementById('formspree-key').value;";
    html += "fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},";
    html += "body:'tempThreshold='+temp+'&humidityThreshold='+hum+'&distanceThreshold='+dist+'&servoClosedAngle='+servoClosed+'&servoOpenAngle='+servoOpen+'&doorOpenDuration='+doorDuration+'&geminiKey='+encodeURIComponent(geminiKey)+'&formspreeKey='+encodeURIComponent(formspreeKey)})";
    html += ".then(r=>r.text()).then(m=>showToast(m,'success'));}";
    html += "const apiKey='" + GEMINI_API_KEY + "';";
    html += "async function sendChat(){";
    html += "const input=document.getElementById('chat-input');";
    html += "const msg=input.value.trim();if(!msg)return;";
    html += "const msgDiv=document.getElementById('chat-messages');";
    html += "msgDiv.innerHTML+='<div style=\"margin:10px 0;text-align:right\"><span style=\"background:#667eea;color:#fff;padding:10px 15px;border-radius:15px;display:inline-block\">'+msg+'</span></div>';";
    html += "input.value='';";
    html += "if(msg.startsWith('/')||msg.startsWith('\\\\')){";
    html += "const cmd=msg.toLowerCase();let reply='';let endpoint='';let body='';";
    html += "if(cmd.match(/^[\\/\\\\](open|open door|open the door)$/)){endpoint='/api/door';body='action=open';reply='✅ Opening door...';}";
    html += "else if(cmd.match(/^[\\/\\\\](close|close door|close the door)$/)){endpoint='/api/door';body='action=close';reply='✅ Closing door...';}";
    html += "else if(cmd.match(/^[\\/\\\\](led|led on|led active|led activate|led activation)$/)){endpoint='/api/led';body='state=on';reply='✅ Turning LED on...';}";
    html += "else if(cmd.match(/^[\\/\\\\](led off|led disactive|led disactivate|led disactivation)$/)){endpoint='/api/led';body='state=off';reply='✅ Turning LED off...';}";
    html += "else if(cmd.match(/^[\\/\\\\](fan|activate fan|fan active|fan on)$/)){endpoint='/api/fan';body='state=on';reply='✅ Activating fan...';}";
    html += "else if(cmd.match(/^[\\/\\\\](fan off|disactive fan|fan disactivate|fan disactivate|fan off)$/)){endpoint='/api/fan';body='state=off';reply='✅ Deactivating fan...';}";
    html += "else if(cmd.match(/^[\\/\\\\](buzzer|buzzer on|buzzer active|buzzer activate|buzzer activation)$/)){endpoint='/api/buzzer';body='state=on';reply='✅ Activating buzzer...';}";
    html += "else if(cmd.match(/^[\\/\\\\](buzzer off|buzzer disactive|buzzer disactivate|buzzer disactivation)$/)){endpoint='/api/buzzer';body='state=off';reply='✅ Deactivating buzzer...';}";
    html += "else{reply='❌ Unknown command. Try: /open, /close, /led on, /led off, /fan on, /fan off, /buzzer on, /buzzer off';}";
    html += "if(endpoint){try{await fetch(endpoint,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body});updateStatus();}catch(e){reply='❌ Error executing command';}}";
    html += "msgDiv.innerHTML+='<div style=\"margin:10px 0\"><span style=\"background:#f8f9fa;color:#333;padding:10px 15px;border-radius:15px;display:inline-block\">'+reply+'</span></div>';";
    html += "msgDiv.scrollTop=msgDiv.scrollHeight;return;}";
    html += "try{";
    html += "const d=await fetch('/api/readings').then(r=>r.json());";
    html += "const prompt='You are ESP32 Door Control assistant. Current Status: Temperature '+d.temperature.toFixed(1)+'°C, Humidity '+d.humidity.toFixed(1)+'%, Door '+(d.doorOpen?'OPEN':'CLOSED')+', Distance '+d.distance.toFixed(1)+'cm. Answer user briefly (max 50 words). User question: '+msg;";
    html += "const response=await fetch('https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent?key='+apiKey,{";
    html += "method:'POST',headers:{'Content-Type':'application/json'},";
    html += "body:JSON.stringify({contents:[{parts:[{text:prompt}]}],generationConfig:{temperature:0.7,maxOutputTokens:150}})});";
    html += "if(!response.ok)throw new Error('API error');";
    html += "const data=await response.json();";
    html += "const reply=data.candidates[0].content.parts[0].text;";
    html += "msgDiv.innerHTML+='<div style=\"margin:10px 0\"><span style=\"background:#f8f9fa;color:#333;padding:10px 15px;border-radius:15px;display:inline-block\">🤖 '+reply+'</span></div>';";
    html += "}catch(e){console.error(e);";
    html += "msgDiv.innerHTML+='<div style=\"margin:10px 0\"><span style=\"background:#ffebee;color:#c62828;padding:10px 15px;border-radius:15px;display:inline-block\">❌ Error connecting to AI. Check console.</span></div>';}";
    html += "msgDiv.scrollTop=msgDiv.scrollHeight;";
    html += "}";
    html += "async function submitContact(e){";
    html += "e.preventDefault();";
    html += "const name=document.getElementById('contact-name').value;";
    html += "const email=document.getElementById('contact-email').value;";
    html += "const message=document.getElementById('contact-message').value;";
    html += "try{";
    html += "const response=await fetch('" + FORMSPREE_ENDPOINT + "',{";
    html += "method:'POST',headers:{'Content-Type':'application/json'},";
    html += "body:JSON.stringify({name:name,email:email,message:message})});";
    html += "if(response.ok){";
    html += "showToast('✓ Message sent successfully!','success');";
    html += "document.getElementById('contactForm').reset();}";
    html += "else{showToast('❌ Failed to send message','error');}}";
    html += "catch(err){showToast('❌ Error: '+err.message,'error');}}";
    html += "let recognition;let isListening=false;";
    html += "function initVoice(){if(!('webkitSpeechRecognition' in window)&&!('SpeechRecognition' in window)){showToast('Voice not supported in this browser','error');return false;}";
    html += "const SpeechRecognition=window.SpeechRecognition||window.webkitSpeechRecognition;";
    html += "recognition=new SpeechRecognition();recognition.continuous=false;recognition.interimResults=false;recognition.lang='en-US';";
    html += "recognition.onstart=()=>{isListening=true;document.getElementById('voice-btn').style.background='#dc3545';document.getElementById('voice-btn').textContent='🔴';showToast('Listening...','info');};";
    html += "recognition.onend=()=>{isListening=false;document.getElementById('voice-btn').style.background='#ffc107';document.getElementById('voice-btn').textContent='🎤';};";
    html += "recognition.onresult=(e)=>{const transcript=e.results[0][0].transcript.toLowerCase();document.getElementById('chat-input').value=transcript;sendChat();};";
    html += "recognition.onerror=(e)=>{showToast('Voice error: '+e.error,'error');isListening=false;document.getElementById('voice-btn').style.background='#ffc107';document.getElementById('voice-btn').textContent='🎤';};";
    html += "return true;}";
    html += "function toggleVoice(){if(!recognition&&!initVoice())return;if(isListening){recognition.stop();}else{recognition.start();}}";
    html += "window.onload=function(){updateStatus();initVoice();};";
    html += "</script></head><body><div class='container'>";
    html += "<div class='header'><h1>🚪 ESP32 Door Control Dashboard</h1><p>Smart IoT System with AI Assistant</p></div>";
    html += "<div class='tabs'><button class='tab active' onclick=\"switchTab('dashboard')\">📊 Dashboard</button>";
    html += "<button class='tab' onclick=\"switchTab('controls')\">🎮 Controls</button>";
    html += "<button class='tab' onclick=\"switchTab('settings')\">⚙️ Settings</button>";
    html += "<button class='tab' onclick=\"switchTab('chat')\">🤖 AI Chat</button>";
    html += "<button class='tab' onclick=\"switchTab('about')\">ℹ️ About</button>";
    html += "<button class='tab' onclick=\"switchTab('contact')\">📧 Contact</button></div>";

    // Dashboard Tab
    html += "<div id='dashboard' class='tab-content active'><div class='status-grid'>";
    html += "<div class='status-card temp'><div class='status-label'>🌡️ Temperature</div><div class='status-value' id='temp-status'>--°C</div></div>";
    html += "<div class='status-card hum'><div class='status-label'>💧 Humidity</div><div class='status-value' id='hum-status'>--%</div></div>";
    html += "<div class='status-card dist'><div class='status-label'>📏 Distance</div><div class='status-value' id='dist-status'>-- cm</div></div>";
    html += "<div class='status-card door'><div class='status-label'>🚪 Door Status</div><div class='status-value' id='door-status'>CLOSED</div></div>";
    html += "</div>";
    html += "<div class='control-card' style='margin-top:20px;text-align:center'><h3>📊 Data Export</h3>";
    html += "<p style='color:#6c757d;margin-bottom:15px'>Download sensor data log (last 100 entries)</p>";
    html += "<a href='/api/export' download='sensor_data.csv' class='btn btn-primary' style='text-decoration:none;display:inline-block'>📥 Download CSV File</a></div>";
    html += "</div>";

    // Controls Tab
    html += "<div id='controls' class='tab-content'><div class='control-grid'>";
    html += "<div class='control-card'><h3>🚪 Door Control</h3><button class='btn btn-success' onclick=\"controlDoor('open')\">Open Door</button>";
    html += "<button class='btn btn-danger' onclick=\"controlDoor('close')\">Close Door</button></div>";
    html += "<div class='control-card'><h3>💡 LED Control</h3><p>State: <span id='led-state'>OFF</span></p><button class='btn btn-warning' onclick='toggleLED()'>Toggle LED</button></div>";
    html += "<div class='control-card'><h3>🌀 Fan Control</h3><p>State: <span id='fan-state'>OFF</span></p><button class='btn btn-primary' onclick='toggleFan()'>Toggle Fan</button></div>";
    html += "<div class='control-card'><h3>🔔 Buzzer Test</h3><p>State: <span id='buzzer-state'>OFF</span></p><button class='btn btn-warning' onclick='toggleBuzzer()'>Toggle Buzzer</button></div>";
    html += "</div></div>";

    // Settings Tab
    html += "<div id='settings' class='tab-content'><div class='control-card'><h3>⚙️ System Settings</h3>";
    html += "<h4 style='color:#667eea;margin-top:20px;margin-bottom:10px'>🌡️ Sensor Thresholds</h4>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Temperature Threshold (°C)</label>";
    html += "<input type='number' id='temp-threshold' step='0.1' value='" + String(tempThreshold, 1) + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'></div>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Humidity Threshold (%)</label>";
    html += "<input type='number' id='hum-threshold' step='0.1' value='" + String(humidityThreshold, 1) + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'></div>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Distance Threshold (cm)</label>";
    html += "<input type='number' id='dist-threshold' step='0.1' value='" + String(distanceThreshold, 1) + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'></div>";
    html += "<h4 style='color:#667eea;margin-top:25px;margin-bottom:10px'>🔧 Servo Motor Angles</h4>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Closed Position (0-180°)</label>";
    html += "<input type='number' id='servo-closed' min='0' max='180' value='" + String(servoClosedAngle) + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'>";
    html += "<small style='color:#6c757d'>Current: " + String(servoClosedAngle) + "° (default: 0°)</small></div>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Open Position (0-180°)</label>";
    html += "<input type='number' id='servo-open' min='0' max='180' value='" + String(servoOpenAngle) + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'>";
    html += "<small style='color:#6c757d'>Current: " + String(servoOpenAngle) + "° (default: 90°)</small></div>";
    html += "<h4 style='color:#667eea;margin-top:25px;margin-bottom:10px'>⏱️ Door Timing</h4>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Door Open Duration (seconds)</label>";
    html += "<input type='number' id='door-duration' min='1' max='60' value='" + String(doorOpenDuration) + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'>";
    html += "<small style='color:#6c757d'>Current: " + String(doorOpenDuration) + " seconds (how long door stays open after manual/remote trigger)</small></div>";
    html += "<h4 style='color:#667eea;margin-top:25px;margin-bottom:10px'>🔑 API Keys</h4>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Gemini API Key</label>";
    html += "<input type='text' id='gemini-key' value='" + GEMINI_API_KEY + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'>";
    html += "<small style='color:#6c757d'>Get your free key from <a href='https://makersuite.google.com/app/apikey' target='_blank'>Google AI Studio</a></small></div>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px;color:#495057;font-weight:500'>Formspree Endpoint</label>";
    html += "<input type='text' id='formspree-key' value='" + FORMSPREE_ENDPOINT + "' style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'>";
    html += "<small style='color:#6c757d'>Your Formspree form endpoint URL</small></div>";
    html += "<button class='btn btn-primary' onclick='saveSettings()' style='width:100%;max-width:300px;margin-top:10px'>💾 Save All Settings</button></div></div>";

    // AI Chat Tab
    html += "<div id='chat' class='tab-content'>";
    html += "<div style='display:flex;flex-direction:column;height:500px;background:#f8f9fa;border-radius:15px;overflow:hidden'>";
    html += "<div id='chat-messages' style='flex:1;overflow-y:auto;padding:20px'>";
    html += "<div style='text-align:center;color:#6c757d;padding:40px'>👋 Ask me about temperature, humidity, door status, or how the system works.</div></div>";
    html += "<div class='chat-input-wrapper'>";
    html += "<input type='text' id='chat-input' placeholder='Ask about the system or use voice...' onkeypress=\"if(event.key=='Enter')sendChat()\">";
    html += "<button class='btn btn-warning' id='voice-btn' onclick='toggleVoice()' style='min-width:50px'>🎤</button>";
    html += "<button class='btn btn-primary' onclick='sendChat()'>Send</button></div></div></div>";

    // About Tab
    html += "<div id='about' class='tab-content'><div class='control-card' style='max-width:900px;margin:0 auto'>";
    html += "<h3 style='text-align:center;font-size:28px;margin-bottom:25px'>ℹ️ About This System</h3>";
    html += "<div style='line-height:1.8;color:#495057'><h4 style='color:#667eea;margin-top:20px;margin-bottom:10px'>🚀 Project Overview</h4>";
    html += "<p style='margin-bottom:15px'>The ESP32 Smart Door Control System is an advanced IoT solution that combines hardware automation with intelligent monitoring, real-time alerts, and AI-powered assistance.</p>";
    html += "<h4 style='color:#667eea;margin-top:25px;margin-bottom:10px'>✨ Key Features</h4><ul style='list-style:none;padding:0'>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Automated Door Control</b> - Servo motor with configurable open/close angles and timing</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Environmental Monitoring</b> - DHT22 sensor for temperature and humidity tracking</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Obstacle Detection</b> - HC-SR04 ultrasonic sensor with automatic lighting control</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>IR Remote Control</b> - Control door (OK), LED (*), and fan (#) with remote</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Emergency Exit Button</b> - Physical button for immediate door access</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Smart Buzzer System</b> - Audio alerts for door open, temp/humidity alerts, and manual testing</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>LED & Fan Control</b> - Toggle LED and PWM-controlled DC fan motor</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Relay-Controlled Lighting</b> - Automatic lights when obstacles detected</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Web Dashboard</b> - Real-time monitoring with live sensor data and WebSocket updates</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>AI Assistant</b> - Google Gemini-powered chatbot for natural language queries</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Chatbot Commands</b> - Control system with commands like /open, /close, /fan on, /buzzer on</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Alert System</b> - Automatic ventilation and email alerts for temperature/humidity thresholds</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Data Logging</b> - 100-entry circular buffer with CSV export capability</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Configurable Settings</b> - Adjust thresholds, servo angles, timing, and API keys via web interface</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>20x4 I2C LCD Display</b> - Real-time status display with countdown timer</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#28a745;font-weight:bold'>✓</span> <b>Contact Form Integration</b> - Formspree email integration for system notifications</li></ul>";
    html += "<h4 style='color:#667eea;margin-top:25px;margin-bottom:10px'>🎮 Control Methods</h4>";
    html += "<p style='margin-bottom:10px'>The system can be controlled through multiple interfaces:</p><ul style='list-style:none;padding:0'>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#2196F3;font-weight:bold'>→</span> Web Dashboard Controls Tab</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#2196F3;font-weight:bold'>→</span> AI Chatbot Commands (/ or \\)</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#2196F3;font-weight:bold'>→</span> IR Remote Control</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#2196F3;font-weight:bold'>→</span> Physical Emergency Button</li>";
    html += "<li style='padding:8px 0;padding-left:25px;position:relative'><span style='position:absolute;left:0;color:#2196F3;font-weight:bold'>→</span> Automatic Environmental Response</li></ul></div></div></div>";

    // Contact Tab
    html += "<div id='contact' class='tab-content'><div class='control-card'><h3>📧 Contact Form</h3>";
    html += "<form id='contactForm' onsubmit='submitContact(event)'>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px'>Name</label>";
    html += "<input type='text' id='contact-name' name='name' required style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'></div>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px'>Email</label>";
    html += "<input type='email' id='contact-email' name='email' required style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'></div>";
    html += "<div style='margin-bottom:15px'><label style='display:block;margin-bottom:5px'>Message</label>";
    html += "<textarea id='contact-message' name='message' rows='5' required style='width:100%;padding:10px;border:1px solid #ced4da;border-radius:8px'></textarea></div>";
    html += "<button type='submit' class='btn btn-primary'>📨 Send Message</button></form></div></div>";

    html += "</div></body></html>";
    request->send(200, "text/html", html);
  });

  // AI Chatbot page
  server.on("/chat", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>AI Assistant</title>";
    html += "<style>*{margin:0;padding:0;box-sizing:border-box}body{font-family:'Segoe UI',sans-serif;background:#f8f9fa;display:flex;flex-direction:column;height:100vh}";
    html += ".hdr{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);padding:20px;box-shadow:0 2px 4px rgba(0,0,0,0.08);display:flex;justify-content:space-between;align-items:center}";
    html += ".hdr h1{font-size:22px;color:#fff;font-weight:700}.btn{background:rgba(255,255,255,0.2);color:#fff;border:1px solid rgba(255,255,255,0.3);padding:10px 20px;border-radius:6px;cursor:pointer;text-decoration:none;font-weight:600}";
    html += ".btn:hover{background:rgba(255,255,255,0.3)}";
    html += ".chat{flex:1;display:flex;flex-direction:column;background:#fff;margin:20px;border-radius:10px;box-shadow:0 2px 8px rgba(0,0,0,0.08);overflow:hidden}";
    html += ".msgs{flex:1;overflow-y:auto;padding:25px;background:#f8f9fa}.msg{margin:15px 0;display:flex;gap:12px}";
    html += ".av{width:36px;height:36px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:18px}";
    html += ".u-av{background:#0d6efd;color:#fff}.b-av{background:#28a745;color:#fff}";
    html += ".cnt{flex:1;background:#fff;padding:15px;border-radius:8px;max-width:calc(100% - 48px)}";
    html += ".u-msg .cnt{background:#0d6efd;color:#fff}";
    html += ".inp{background:#fff;padding:20px;border-top:1px solid #dee2e6;display:flex;gap:12px}";
    html += ".inp input{flex:1;padding:12px;border:2px solid #dee2e6;border-radius:6px;font-size:15px}";
    html += ".inp button{background:#0d6efd;color:#fff;border:none;padding:12px 28px;border-radius:6px;cursor:pointer;font-weight:600}";
    html += ".inp button:hover{background:#0b5ed7}";
    html += ".inp button.voice{background:#ffc107;color:#000;min-width:50px}.inp button.voice.listening{background:#dc3545;color:#fff}";
    html += ".toast{position:fixed;bottom:20px;right:20px;background:#333;color:#fff;padding:15px 20px;border-radius:8px;box-shadow:0 4px 12px rgba(0,0,0,0.3);z-index:1000;opacity:0;transform:translateY(100px);transition:all 0.3s}";
    html += ".toast.show{opacity:1;transform:translateY(0)}.toast.info{background:#2196F3}.toast.error{background:#dc3545}";
    html += "</style></head><body>";
    html += "<div class='hdr'><h1>🤖 ESP32 Door Control AI Assistant</h1><a href='/' class='btn'>🏠 Dashboard</a></div>";
    html += "<div class='chat'><div class='msgs' id='msgs'><div style='text-align:center;color:#6c757d;padding:40px'>👋 Ask me about temperature, humidity, door status, or use voice control.</div></div>";
    html += "<div class='inp'><input id='inp' placeholder='Type or speak your command...' onkeypress='if(event.key==\"Enter\")send()'>";
    html += "<button class='voice' id='voice-btn' onclick='toggleVoice()'>🎤</button>";
    html += "<button onclick='send()'>Send</button></div></div>";
    html += "<script>";
    html += "const apiKey='" + GEMINI_API_KEY + "';";
    html += "function add(s,t){const m=document.createElement('div');m.className='msg '+(s==='u'?'u-msg':'b-msg');";
    html += "const a=document.createElement('div');a.className='av '+(s==='u'?'u-av':'b-av');a.textContent=s==='u'?'👤':'🤖';";
    html += "const c=document.createElement('div');c.className='cnt';c.textContent=t;m.appendChild(a);m.appendChild(c);";
    html += "document.getElementById('msgs').appendChild(m);document.getElementById('msgs').scrollTop=document.getElementById('msgs').scrollHeight}";
    html += "async function send(){const i=document.getElementById('inp');let msg=i.value.trim();if(!msg)return;i.value='';add('u',msg);";
    html += "if(msg.startsWith('/')||msg.startsWith('\\\\')){";
    html += "const cmd=msg.toLowerCase();let reply='';let endpoint='';let body='';";
    html += "if(cmd.match(/^[\\/\\\\](open|open door|open the door)$/)){endpoint='/api/door';body='action=open';reply='✅ Opening door...';}";
    html += "else if(cmd.match(/^[\\/\\\\](close|close door|close the door)$/)){endpoint='/api/door';body='action=close';reply='✅ Closing door...';}";
    html += "else if(cmd.match(/^[\\/\\\\](led|led on|led active|led activate|led activation)$/)){endpoint='/api/led';body='state=on';reply='✅ Turning LED on...';}";
    html += "else if(cmd.match(/^[\\/\\\\](led off|led disactive|led disactivate|led disactivation)$/)){endpoint='/api/led';body='state=off';reply='✅ Turning LED off...';}";
    html += "else if(cmd.match(/^[\\/\\\\](fan|activate fan|fan active|fan on)$/)){endpoint='/api/fan';body='state=on';reply='✅ Activating fan...';}";
    html += "else if(cmd.match(/^[\\/\\\\](fan off|disactive fan|fan disactivate|fan disactivate|fan off)$/)){endpoint='/api/fan';body='state=off';reply='✅ Deactivating fan...';}";
    html += "else if(cmd.match(/^[\\/\\\\](buzzer|buzzer on|buzzer active|buzzer activate|buzzer activation)$/)){endpoint='/api/buzzer';body='state=on';reply='✅ Activating buzzer...';}";
    html += "else if(cmd.match(/^[\\/\\\\](buzzer off|buzzer disactive|buzzer disactivate|buzzer disactivation)$/)){endpoint='/api/buzzer';body='state=off';reply='✅ Deactivating buzzer...';}";
    html += "else{reply='❌ Unknown command. Try: /open, /close, /led on, /led off, /fan on, /fan off, /buzzer on, /buzzer off';}";
    html += "if(endpoint){try{await fetch(endpoint,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body});}catch(e){reply='❌ Error executing command';}}";
    html += "add('b',reply);return;}";
    html += "let d=await fetch('/api/readings').then(r=>r.json()).catch(()=>({}));";
    html += "let prompt='You are ESP32 Door Control assistant. Status: Temp='+d.temperature+'°C, Humid='+d.humidity+'%, Door='+";
    html += "(d.doorOpen?'OPEN':'CLOSED')+', Distance='+d.distance+'cm. Answer briefly (max 50 words). User: '+msg;";
    html += "try{const r=await fetch('https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key='+apiKey,";
    html += "{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({contents:[{parts:[{text:prompt}]}],generationConfig:{temperature:0.7,maxOutputTokens:150}})});";
    html += "if(!r.ok)throw new Error('API error');const data=await r.json();";
    html += "const text=data.candidates[0].content.parts[0].text;add('b',text)}";
    html += "catch(e){console.error(e);add('b','Connection error. Please try again.')}}";
    html += "function showToast(msg,type='info'){const toast=document.createElement('div');toast.className='toast '+type;toast.textContent=msg;document.body.appendChild(toast);";
    html += "setTimeout(()=>toast.classList.add('show'),10);setTimeout(()=>{toast.classList.remove('show');setTimeout(()=>document.body.removeChild(toast),300);},3000);}";
    html += "let recognition;let isListening=false;";
    html += "function initVoice(){if(!('webkitSpeechRecognition' in window)&&!('SpeechRecognition' in window)){showToast('Voice not supported in this browser','error');return false;}";
    html += "const SpeechRecognition=window.SpeechRecognition||window.webkitSpeechRecognition;";
    html += "recognition=new SpeechRecognition();recognition.continuous=false;recognition.interimResults=false;recognition.lang='en-US';";
    html += "recognition.onstart=()=>{isListening=true;document.getElementById('voice-btn').classList.add('listening');document.getElementById('voice-btn').textContent='🔴';showToast('Listening...','info');};";
    html += "recognition.onend=()=>{isListening=false;document.getElementById('voice-btn').classList.remove('listening');document.getElementById('voice-btn').textContent='🎤';};";
    html += "recognition.onresult=(e)=>{const transcript=e.results[0][0].transcript.toLowerCase();document.getElementById('inp').value=transcript;send();};";
    html += "recognition.onerror=(e)=>{showToast('Voice error: '+e.error,'error');isListening=false;document.getElementById('voice-btn').classList.remove('listening');document.getElementById('voice-btn').textContent='🎤';};";
    html += "return true;}";
    html += "function toggleVoice(){if(!recognition&&!initVoice())return;if(isListening){recognition.stop();}else{recognition.start();}}";
    html += "window.onload=()=>initVoice();";
    html += "</script></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/api/readings", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"temperature\":" + String(temperature, 1) + ",";
    json += "\"humidity\":" + String(humidity, 1) + ",";
    json += "\"distance\":" + String(currentDistance, 1) + ",";
    json += "\"doorOpen\":" + String(doorOpen ? "true" : "false") + ",";
    json += "\"tempThreshold\":" + String(tempThreshold, 1) + ",";
    json += "\"humidityThreshold\":" + String(humidityThreshold, 1) + ",";
    json += "\"distanceThreshold\":" + String(distanceThreshold, 1) + ",";
    json += "\"servoClosedAngle\":" + String(servoClosedAngle) + ",";
    json += "\"servoOpenAngle\":" + String(servoOpenAngle) + ",";
    json += "\"doorOpenDuration\":" + String(doorOpenDuration) + ",";
    json += "\"ledState\":" + String(ledState ? "true" : "false") + ",";
    json += "\"fanState\":" + String(fanState ? "true" : "false") + ",";
    json += "\"buzzerState\":" + String(buzzerState ? "true" : "false");
    json += "}";
    request->send(200, "application/json", json);
  });

  server.on("/api/door", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("action", true)) {
      String action = request->getParam("action", true)->value();
      if (action == "open") {
        doorOpen = true;
        autoOpenedByTempAlert = false;
        doorOpenTime = millis();
        doorServo.write(servoOpenAngle);
        broadcastSensorData();
        request->send(200, "text/plain", "Door opened");
      } else if (action == "close") {
        doorOpen = false;
        autoOpenedByTempAlert = false;
        doorServo.write(servoClosedAngle);
        broadcastSensorData();
        request->send(200, "text/plain", "Door closed");
      }
    } else {
      request->send(400, "text/plain", "Missing action");
    }
  });

  server.on("/api/led", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("state", true)) {
      String state = request->getParam("state", true)->value();
      if (state == "toggle") {
        ledState = !ledState;
      } else {
        ledState = (state == "on");
      }
      digitalWrite(TOGGLE_LED_PIN, ledState ? HIGH : LOW);
      Serial.print("LED turned ");
      Serial.println(ledState ? "ON" : "OFF");
      broadcastSensorData();
      request->send(200, "text/plain", ledState ? "✓ LED turned ON" : "✓ LED turned OFF");
    }
  });

  server.on("/api/fan", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("state", true)) {
      String state = request->getParam("state", true)->value();
      if (state == "toggle") {
        fanState = !fanState;
      } else {
        fanState = (state == "on");
      }
      ledcWrite(FAN_PWM_CHANNEL, fanState ? 255 : 0);
      Serial.print("Fan turned ");
      Serial.println(fanState ? "ON" : "OFF");
      broadcastSensorData();
      request->send(200, "text/plain", fanState ? "✓ Fan turned ON" : "✓ Fan turned OFF");
    }
  });

  server.on("/api/buzzer", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("state", true)) {
      String state = request->getParam("state", true)->value();
      if (state == "toggle") {
        buzzerState = !buzzerState;
      } else {
        buzzerState = (state == "on");
      }
      Serial.print("Buzzer turned ");
      Serial.println(buzzerState ? "ON" : "OFF");
      broadcastSensorData();
      request->send(200, "text/plain", buzzerState ? "✓ Buzzer turned ON" : "✓ Buzzer turned OFF");
    }
  });

  server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("tempThreshold", true)) {
      tempThreshold = request->getParam("tempThreshold", true)->value().toFloat();
    }
    if (request->hasParam("humidityThreshold", true)) {
      humidityThreshold = request->getParam("humidityThreshold", true)->value().toFloat();
    }
    if (request->hasParam("distanceThreshold", true)) {
      distanceThreshold = request->getParam("distanceThreshold", true)->value().toFloat();
    }
    if (request->hasParam("servoClosedAngle", true)) {
      servoClosedAngle = request->getParam("servoClosedAngle", true)->value().toInt();
      servoClosedAngle = constrain(servoClosedAngle, 0, 180);
    }
    if (request->hasParam("servoOpenAngle", true)) {
      servoOpenAngle = request->getParam("servoOpenAngle", true)->value().toInt();
      servoOpenAngle = constrain(servoOpenAngle, 0, 180);
    }
    if (request->hasParam("doorOpenDuration", true)) {
      doorOpenDuration = request->getParam("doorOpenDuration", true)->value().toInt();
      doorOpenDuration = constrain(doorOpenDuration, 1, 60);  // Between 1 and 60 seconds
      Serial.print("Door open duration updated to ");
      Serial.print(doorOpenDuration);
      Serial.println(" seconds");
    }
    if (request->hasParam("geminiKey", true)) {
      GEMINI_API_KEY = request->getParam("geminiKey", true)->value();
      Serial.println("Gemini API key updated");
    }
    if (request->hasParam("formspreeKey", true)) {
      FORMSPREE_ENDPOINT = request->getParam("formspreeKey", true)->value();
      Serial.println("Formspree endpoint updated");
    }
    saveSettings();
    Serial.println("Settings updated:");
    Serial.print("  Temp threshold: "); Serial.println(tempThreshold);
    Serial.print("  Humidity threshold: "); Serial.println(humidityThreshold);
    Serial.print("  Distance threshold: "); Serial.println(distanceThreshold);
    Serial.print("  Servo closed: "); Serial.println(servoClosedAngle);
    Serial.print("  Servo open: "); Serial.println(servoOpenAngle);
    Serial.print("  Door duration: "); Serial.print(doorOpenDuration); Serial.println(" seconds");
    request->send(200, "text/plain", "✓ Settings saved successfully");
  });

  server.on("/api/export", HTTP_GET, [](AsyncWebServerRequest *request){
    String csv = "Timestamp (ms),Temperature (C),Humidity (%),Distance (cm),Door Status,LED Status,Fan Status\n";
    int entryCount = 0;
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
      int idx = (logIndex + i) % MAX_LOG_ENTRIES;
      if (dataLog[idx].timestamp > 0) {
        csv += String(dataLog[idx].timestamp) + ",";
        csv += String(dataLog[idx].temp, 1) + ",";
        csv += String(dataLog[idx].hum, 1) + ",";
        csv += String(dataLog[idx].dist, 1) + ",";
        csv += String(dataLog[idx].doorState ? "OPEN" : "CLOSED") + ",";
        csv += String(dataLog[idx].ledState ? "ON" : "OFF") + ",";
        csv += String(dataLog[idx].fanState ? "ON" : "OFF") + "\n";
        entryCount++;
      }
    }
    Serial.print("CSV Export: ");
    Serial.print(entryCount);
    Serial.println(" entries exported");

    AsyncWebServerResponse *response = request->beginResponse(200, "text/csv", csv);
    response->addHeader("Content-Disposition", "attachment; filename=sensor_data.csv");
    request->send(response);
  });

  server.begin();
  Serial.println("Web server started");
}