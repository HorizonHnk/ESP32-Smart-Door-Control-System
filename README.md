# 🚪 ESP32 Smart Door Control System

<div align="center">

![Version](https://img.shields.io/badge/version-4.2-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32-green.svg)
![License](https://img.shields.io/badge/license-MIT-yellow.svg)
![Status](https://img.shields.io/badge/status-active-success.svg)

**An Advanced IoT Door Control System with AI Assistant, Real-time Monitoring & Smart Automation**

[Features](#-features) • [Hardware](#-hardware-requirements) • [Installation](#-installation) • [Usage](#-usage) • [API](#-api-endpoints) • [Documentation](#-documentation)

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Pin Configuration](#-pin-configuration)
- [Software Requirements](#-software-requirements)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [Usage](#-usage)
- [Control Methods](#-control-methods)
- [API Endpoints](#-api-endpoints)
- [Web Dashboard](#-web-dashboard)
- [System Behavior](#-system-behavior)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)
- [Author](#-author)

---

## 🌟 Overview

The **ESP32 Smart Door Control System** is a comprehensive IoT solution that combines hardware automation with intelligent monitoring, real-time alerts, and AI-powered assistance. This system provides multiple control interfaces including a modern web dashboard, voice control, IR remote, and physical buttons, making it perfect for smart home applications, access control, and environmental monitoring.

### Key Highlights

- 🤖 **AI-Powered Assistant** - Google Gemini integration for natural language queries
- 📱 **Responsive Web Dashboard** - Real-time monitoring with WebSocket updates
- 🎙️ **Voice Control** - Browser-based speech recognition for hands-free operation
- 🌡️ **Environmental Monitoring** - Automatic temperature and humidity tracking with alerts
- 📊 **Data Logging** - 100-entry circular buffer with CSV export capability
- 🔒 **Multiple Security Layers** - IR remote, emergency button, and web authentication
- ⚡ **Real-time Alerts** - Email notifications via Formspree integration
- 🎨 **Modern UI** - Beautiful gradient design with tabbed interface

---

## ✨ Features

### 🔐 Door Control & Security
- ✅ **Automated Servo Door Lock** - Configurable open/close angles (0-180°)
- ✅ **Emergency Exit Button** - Physical button for immediate access
- ✅ **IR Remote Control** - Control door, LED, and fan with remote buttons
- ✅ **Auto-Close Timer** - Configurable duration (1-60 seconds)
- ✅ **Manual Override** - Multiple control methods for redundancy

### 🌡️ Environmental Monitoring
- ✅ **DHT22 Sensor** - Accurate temperature and humidity monitoring
- ✅ **Configurable Thresholds** - Set custom alert levels
- ✅ **Automatic Ventilation** - Door opens automatically when thresholds exceeded
- ✅ **Real-time Display** - 20x4 I2C LCD shows live status

### 🚨 Alert & Safety Systems
- ✅ **Multi-Pattern Buzzer** - Different beep patterns for various alerts
- ✅ **Email Notifications** - Formspree integration for critical alerts
- ✅ **Visual Indicators** - Status LED with state-based blinking
- ✅ **Cooldown Protection** - Prevents alert spam (5-minute cooldown)

### 🔍 Obstacle Detection
- ✅ **HC-SR04 Ultrasonic Sensor** - Accurate distance measurement
- ✅ **Automatic Lighting** - Relay-controlled lights activate on detection
- ✅ **Configurable Threshold** - Adjust detection distance

### 💻 Web Dashboard Features
- ✅ **Real-time Updates** - WebSocket-based live data streaming
- ✅ **Tabbed Interface** - Dashboard, Controls, Settings, AI Chat, About, Contact
- ✅ **Responsive Design** - Works on desktop, tablet, and mobile
- ✅ **Beautiful UI** - Modern gradient design with smooth animations
- ✅ **Toast Notifications** - Non-intrusive user feedback

### 🤖 AI Assistant
- ✅ **Google Gemini Integration** - Natural language understanding
- ✅ **Voice Input Support** - Browser-based speech recognition
- ✅ **Chatbot Commands** - Control system with `/open`, `/close`, etc.
- ✅ **Context-Aware Responses** - AI knows current system status

### 📊 Data Management
- ✅ **Circular Buffer Logging** - Stores last 100 sensor readings
- ✅ **CSV Export** - Download data logs for analysis
- ✅ **Persistent Settings** - Preferences stored in ESP32 flash memory
- ✅ **Remote Configuration** - Update settings via web interface

### 🎮 Additional Controls
- ✅ **Toggle LED** - Manual or remote-controlled LED
- ✅ **PWM Fan Control** - Variable speed DC fan motor
- ✅ **Buzzer Test** - Manual buzzer activation for testing

---

## 🔧 Hardware Requirements

### Core Components

| Component | Specification | Quantity | Purpose |
|-----------|--------------|----------|---------|
| **ESP32-WROOM-32** | WiFi/Bluetooth MCU | 1 | Main controller |
| **HC-SR04** | Ultrasonic Distance Sensor | 1 | Obstacle detection |
| **DHT22** | Temperature/Humidity Sensor | 1 | Environmental monitoring |
| **SG90/MG996R** | Servo Motor | 1 | Door lock mechanism |
| **20x4 I2C LCD** | LCD Display (0x27) | 1 | Status display |
| **IR Receiver** | VS1838B or similar | 1 | Remote control input |
| **IR Remote** | Universal remote | 1 | Wireless control |
| **Relay Module** | 5V 1-Channel | 1 | Lighting control |
| **Active Buzzer** | 5V Active Buzzer | 1 | Audio alerts |
| **Push Button** | Momentary switch | 2 | Emergency exit + toggles |
| **DC Fan Motor** | 3-5V DC Motor | 1 | Ventilation |
| **LED** | 5mm LED | 2 | Status indicators |
| **Resistors** | 220Ω, 10kΩ | Several | Pull-up/current limiting |

### Power Supply
- **ESP32**: 5V/2A USB power supply or regulated DC
- **Servo Motor**: Separate 5V/2A supply (high current draw)
- **Total System**: 5V/3A recommended for stable operation

### Optional Components
- **Breadboard** or **PCB** for prototyping
- **Jumper wires** (Male-to-Male, Male-to-Female)
- **Power distribution board**
- **Enclosure** for finished project

---

## 📍 Pin Configuration

### ESP32 Pin Mapping

```cpp
// I2C Communication (LCD)
SDA = GPIO 21  // I2C Data
SCL = GPIO 22  // I2C Clock

// Sensors
ULTRASONIC_TRIG_PIN = GPIO 19  // Ultrasonic trigger
ULTRASONIC_ECHO_PIN = GPIO 18  // Ultrasonic echo
DHT_PIN = GPIO 25              // DHT22 data (needs 4.7k-10kΩ pull-up)
IR_RECEIVER_PIN = GPIO 14      // IR receiver

// Actuators & Outputs
SERVO_PIN = GPIO 26            // Servo motor PWM
RELAY_PIN = GPIO 27            // Relay module control
BUZZER_PIN = GPIO 15           // Active buzzer
LED_STATUS_PIN = GPIO 5        // Status LED
TOGGLE_LED_PIN = GPIO 13       // Toggle LED
FAN_MOTOR_PIN = GPIO 32        // DC Fan PWM

// Input Buttons
EMERGENCY_EXIT_PIN = GPIO 34   // Emergency button (input-only, external pull-up)
TOGGLE_BUTTON_PIN = GPIO 35    // LED toggle button (external pull-up)
FAN_BUTTON_PIN = GPIO 23       // Fan toggle button (external pull-up)
```

### Important Pin Notes

> **⚠️ GPIO 34-39**: Input-only pins, require external pull-up resistors (10kΩ)
> 
> **⚠️ GPIO 25**: ADC2 pin, may conflict with WiFi if using ADC
> 
> **⚠️ GPIO 5**: Strapping pin, ensure proper startup state
> 
> **⚠️ DHT22**: Requires external pull-up resistor (4.7kΩ - 10kΩ) on data pin

### LCD I2C Address
- Default: `0x27`
- Alternative: `0x3F` (if 0x27 doesn't work)
- Use I2C scanner sketch to find your LCD address

---

## 💾 Software Requirements

### Arduino IDE Setup

1. **Arduino IDE**: Version 2.0 or later
   - Download: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)

2. **ESP32 Board Package**:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
   - Add to **File → Preferences → Additional Board Manager URLs**
   - Install via **Tools → Board → Boards Manager → ESP32**

### Required Libraries

Install via **Arduino IDE → Sketch → Include Library → Manage Libraries**:

| Library | Version | Purpose |
|---------|---------|---------|
| `ESP32Servo` | Latest | Servo motor control |
| `IRremote` | 4.0+ | IR receiver (new library) |
| `DHT sensor library` | 1.4.4+ | DHT22 sensor |
| `LiquidCrystal_I2C` | 1.1.2+ | I2C LCD display |
| `ESPAsyncWebServer` | Latest | Async web server |
| `AsyncTCP` | Latest | TCP async support |
| `Preferences` | Built-in | ESP32 flash storage |
| `HTTPClient` | Built-in | HTTP requests |
| `WiFi` | Built-in | WiFi connectivity |

### External Libraries (Manual Installation)

Some libraries need manual installation:

**ESPAsyncWebServer**:
```bash
git clone https://github.com/me-no-dev/ESPAsyncWebServer.git
```

**AsyncTCP**:
```bash
git clone https://github.com/me-no-dev/AsyncTCP.git
```

Place in `Arduino/libraries/` folder.

### API Keys Required

1. **Google Gemini API Key** (Free):
   - Get from: [https://makersuite.google.com/app/apikey](https://makersuite.google.com/app/apikey)
   - Used for AI chatbot functionality

2. **Formspree Endpoint** (Free):
   - Sign up: [https://formspree.io](https://formspree.io)
   - Create new form and get endpoint URL
   - Used for email alerts and contact form

---

## 🚀 Installation

### Step 1: Hardware Assembly

1. **Connect I2C LCD**:
   ```
   LCD VCC → ESP32 5V
   LCD GND → ESP32 GND
   LCD SDA → GPIO 21
   LCD SCL → GPIO 22
   ```

2. **Connect DHT22**:
   ```
   DHT22 VCC → ESP32 3.3V
   DHT22 GND → ESP32 GND
   DHT22 DATA → GPIO 25 (with 4.7kΩ pull-up to 3.3V)
   ```

3. **Connect HC-SR04**:
   ```
   TRIG → GPIO 19
   ECHO → GPIO 18
   VCC → 5V
   GND → GND
   ```

4. **Connect Servo Motor**:
   ```
   Signal → GPIO 26
   VCC → External 5V (recommended)
   GND → Common GND
   ```

5. **Connect remaining components** following the [Pin Configuration](#-pin-configuration) table.

### Step 2: Software Installation

1. **Clone Repository**:
   ```bash
   git clone https://github.com/yourusername/esp32-smart-door-control.git
   cd esp32-smart-door-control
   ```

2. **Open in Arduino IDE**:
   - Open `esp32_door_control.ino`

3. **Configure WiFi Credentials**:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

4. **Configure API Keys** (optional, can be set via web interface later):
   ```cpp
   String GEMINI_API_KEY = "YOUR_GEMINI_API_KEY";
   String FORMSPREE_ENDPOINT = "https://formspree.io/f/YOUR_FORM_ID";
   ```

5. **Select Board & Port**:
   - **Board**: ESP32 Dev Module
   - **Upload Speed**: 115200
   - **Port**: Select your ESP32 COM port

6. **Upload Code**:
   - Click **Upload** button
   - Monitor Serial output (115200 baud) for IP address

### Step 3: Initial Setup

1. **Find IP Address**:
   - Check Serial Monitor for: `IP Address: 192.168.x.x`
   - Or check your router's connected devices

2. **Access Web Dashboard**:
   - Open browser: `http://192.168.x.x`

3. **Configure Settings**:
   - Navigate to **Settings** tab
   - Update thresholds, servo angles, API keys
   - Click **Save All Settings**

---

## ⚙️ Configuration

### Configurable Parameters

All settings can be adjusted via the web dashboard (**Settings** tab):

#### 🌡️ Sensor Thresholds
- **Temperature Threshold**: 30.0°C (default) - Triggers ventilation alert
- **Humidity Threshold**: 70.0% (default) - Triggers ventilation alert
- **Distance Threshold**: 10.0 cm (default) - Obstacle detection range

#### 🔧 Servo Motor Angles
- **Closed Position**: 0° (default) - Adjust based on your lock mechanism
- **Open Position**: 90° (default) - Adjust for full door release

#### ⏱️ Door Timing
- **Door Open Duration**: 10 seconds (default) - Auto-close timer (1-60s range)

#### 🔑 API Keys
- **Gemini API Key**: Required for AI chatbot
- **Formspree Endpoint**: Required for email alerts

### IR Remote Button Codes

Default codes (may need adjustment for your remote):

```cpp
#define IR_BTN_OK 0x1C      // OK button - Opens door
#define IR_BTN_STAR 0x16    // * button - Toggle LED
#define IR_BTN_HASH 0x0D    // # button - Toggle Fan
```

To find your remote's codes:
1. Open Serial Monitor
2. Press buttons on your IR remote
3. Note the hex codes printed
4. Update the defines in the code

### LCD I2C Address

If LCD doesn't display, try changing address:

```cpp
#define I2C_ADDR 0x27  // Try 0x3F if 0x27 doesn't work
```

---

## 📱 Usage

### Control Methods

The system supports **5 different control methods**:

#### 1. 🌐 Web Dashboard
- Access via browser: `http://ESP32_IP_ADDRESS`
- Real-time sensor monitoring
- Click buttons to control door, LED, fan, buzzer
- Download data logs as CSV

#### 2. 🤖 AI Chatbot
- Navigate to **AI Chat** tab
- Type natural language queries: "What's the temperature?"
- Use commands: `/open`, `/close`, `/led on`, `/fan off`
- Voice input supported (click 🎤 button)

#### 3. 📡 IR Remote
- **OK Button**: Open door (10-second timer)
- **\* Button**: Toggle LED on/off
- **# Button**: Toggle fan on/off

#### 4. 🔴 Emergency Button
- Press physical emergency button
- Door opens immediately for 10 seconds
- Buzzer sounds while door is open

#### 5. 🌡️ Automatic
- System monitors temperature and humidity
- Auto-opens door when thresholds exceeded
- Auto-activates fan during alerts
- Door stays open until conditions normalize

### AI Chatbot Commands

#### Natural Language Queries
```
"What's the current temperature?"
"Is the door open?"
"Tell me about the humidity"
"What's the distance sensor reading?"
```

#### Direct Commands
```
/open           - Opens the door
/close          - Closes the door
/led on         - Turns LED on
/led off        - Turns LED off
/fan on         - Activates fan
/fan off        - Deactivates fan
/buzzer on      - Test buzzer
/buzzer off     - Stop buzzer
```

> **💡 Tip**: Commands work with both `/` and `\` prefix

### Voice Control

1. Click the **🎤 microphone** button
2. Wait for "Listening..." notification
3. Speak your command clearly
4. System executes voice command automatically

**Supported browsers**: Chrome, Edge (requires HTTPS in production)

---

## 🔌 API Endpoints

### GET Endpoints

#### Get Current Readings
```http
GET /api/readings
```

**Response**:
```json
{
  "temperature": 25.5,
  "humidity": 65.2,
  "distance": 15.3,
  "doorOpen": false,
  "tempThreshold": 30.0,
  "humidityThreshold": 70.0,
  "distanceThreshold": 10.0,
  "servoClosedAngle": 0,
  "servoOpenAngle": 90,
  "doorOpenDuration": 10,
  "ledState": false,
  "fanState": false,
  "buzzerState": false
}
```

#### Export Data Log (CSV)
```http
GET /api/export
```

**Response**: CSV file download with sensor history

### POST Endpoints

#### Control Door
```http
POST /api/door
Content-Type: application/x-www-form-urlencoded

action=open   # or action=close
```

#### Toggle LED
```http
POST /api/led
Content-Type: application/x-www-form-urlencoded

state=toggle  # or state=on or state=off
```

#### Toggle Fan
```http
POST /api/fan
Content-Type: application/x-www-form-urlencoded

state=toggle  # or state=on or state=off
```

#### Test Buzzer
```http
POST /api/buzzer
Content-Type: application/x-www-form-urlencoded

state=toggle  # or state=on or state=off
```

#### Update Settings
```http
POST /api/settings
Content-Type: application/x-www-form-urlencoded

tempThreshold=30.0&humidityThreshold=70.0&distanceThreshold=10.0
&servoClosedAngle=0&servoOpenAngle=90&doorOpenDuration=10
&geminiKey=YOUR_KEY&formspreeKey=YOUR_ENDPOINT
```

### WebSocket Endpoint

```
ws://ESP32_IP_ADDRESS/ws
```

**Real-time Data Stream**:
- Broadcasts sensor data every second
- Automatic reconnection on disconnect

---

## 🖥️ Web Dashboard

### Dashboard Tab
- 📊 **Real-time Sensor Cards**
  - Temperature (°C)
  - Humidity (%)
  - Distance (cm)
  - Door Status
- 📥 **CSV Export Button** - Download sensor data log

### Controls Tab
- 🚪 **Door Control** - Open/Close buttons
- 💡 **LED Control** - Toggle with state display
- 🌀 **Fan Control** - Toggle with state display
- 🔔 **Buzzer Test** - Manual buzzer activation

### Settings Tab
- 🌡️ **Sensor Thresholds** - Temperature, Humidity, Distance
- 🔧 **Servo Angles** - Closed and Open positions
- ⏱️ **Door Timing** - Auto-close duration
- 🔑 **API Keys** - Gemini and Formspree configuration
- 💾 **Save Button** - Persist settings to flash memory

### AI Chat Tab
- 🤖 **Chatbot Interface** - Natural language interaction
- 🎙️ **Voice Input** - Browser-based speech recognition
- 💬 **Command Support** - Slash commands for quick actions
- 📱 **Responsive Design** - Works on mobile devices

### About Tab
- ℹ️ **Project Overview** - System description
- ✨ **Feature List** - Complete capabilities
- 🎮 **Control Methods** - All available interfaces

### Contact Tab
- 📧 **Contact Form** - Send messages via Formspree
- 📨 **Direct Email** - System notifications

---

## ⚡ System Behavior

### Obstacle Detection Mode
```
Distance < Threshold → Relay ON (Lights ON)
Distance > Threshold → Relay OFF (Lights OFF)
```

### Door Control Modes

#### 1. Manual/Remote Opening
```
Trigger (Button/IR/Web) → Door opens 90°
→ Buzzer beeps (400ms on, 200ms off)
→ Status LED flashes (250ms interval)
→ Auto-close after 10 seconds (configurable)
→ Door closes to 0°
```

#### 2. Automatic Ventilation
```
Temp ≥ Threshold OR Humidity ≥ Threshold
→ Door opens 90° (VENTILATION mode)
→ Fan activates automatically
→ Buzzer beeps (600ms on, 400ms off)
→ Email alert sent (5-min cooldown)
→ Door stays open until BOTH conditions normalize
→ No auto-close timer
```

#### 3. Multiple Alerts (Urgent)
```
Door Open AND Temp/Humidity Alert
→ Rapid buzzer beeps (200ms on/off)
→ Visual indicators active
→ Continuous monitoring
```

### LCD Display Layout

```
Line 1: == Door Control ==
Line 2: Status: CLOSED/OPEN (Xs)
Line 3: Alert/Obstacle/LED+Fan Status
Line 4: T:25.5C H:65% L:OFF
```

### Buzzer Patterns

| Pattern | Timing | Condition |
|---------|--------|-----------|
| **Manual Door** | 400ms ON / 200ms OFF | Door manually opened |
| **Temp Alert** | 600ms ON / 400ms OFF | Temperature/humidity alert only |
| **Multiple Alerts** | 200ms ON / 200ms OFF | Door open + Temp alert (urgent!) |
| **Manual Test** | 100ms ON / 100ms OFF | Buzzer toggle via web/remote |

---

## 🔍 Troubleshooting

### WiFi Connection Issues

**Problem**: ESP32 not connecting to WiFi

**Solutions**:
- ✅ Verify SSID and password are correct
- ✅ Check 2.4GHz WiFi (ESP32 doesn't support 5GHz)
- ✅ Ensure router is within range
- ✅ Check Serial Monitor for connection attempts
- ✅ Try different WiFi channel on router

### LCD Display Issues

**Problem**: LCD shows garbage characters or nothing

**Solutions**:
- ✅ Verify I2C address (try 0x27 or 0x3F)
- ✅ Check wiring: SDA→GPIO21, SCL→GPIO22
- ✅ Run I2C scanner sketch to find address
- ✅ Adjust contrast potentiometer on LCD backpack
- ✅ Check 5V power supply is stable

### DHT22 Reading NaN

**Problem**: Temperature/Humidity show "nan"

**Solutions**:
- ✅ Add 4.7kΩ - 10kΩ pull-up resistor to DATA pin
- ✅ Check sensor wiring and orientation
- ✅ Wait 2-3 seconds after power-on for sensor initialization
- ✅ Verify DHT_PIN matches your wiring
- ✅ Try different DHT22 sensor (could be defective)

### IR Remote Not Working

**Problem**: IR remote commands not recognized

**Solutions**:
- ✅ Check IR receiver polarity (OUT, GND, VCC)
- ✅ Verify IR_RECEIVER_PIN (GPIO 14)
- ✅ Point remote directly at receiver (3-5 meters range)
- ✅ Check Serial Monitor for IR codes
- ✅ Update IR button codes in code to match your remote
- ✅ Replace IR receiver if damaged

### Servo Not Moving

**Problem**: Servo doesn't respond to commands

**Solutions**:
- ✅ Use external 5V/2A power for servo (high current draw)
- ✅ Common ground between ESP32 and servo power
- ✅ Check SERVO_PIN (GPIO 26) connection
- ✅ Verify servo angles (0-180° range)
- ✅ Test with simple servo sweep sketch
- ✅ Check Serial Monitor for servo commands

### Web Dashboard Not Loading

**Problem**: Cannot access web interface

**Solutions**:
- ✅ Check Serial Monitor for IP address
- ✅ Verify ESP32 and computer on same network
- ✅ Try accessing via hostname if mDNS enabled
- ✅ Disable browser extensions (AdBlock, etc.)
- ✅ Clear browser cache and cookies
- ✅ Check firewall settings

### AI Chatbot Not Responding

**Problem**: Chatbot shows error message

**Solutions**:
- ✅ Verify Gemini API key is correct and active
- ✅ Check API key quota hasn't exceeded
- ✅ Ensure internet connectivity is stable
- ✅ Check browser console for specific errors
- ✅ Try regenerating API key from Google AI Studio

### Buzzer Not Sounding

**Problem**: Buzzer doesn't beep

**Solutions**:
- ✅ Verify using **active buzzer** (has internal oscillator)
- ✅ Check polarity (active buzzers have +/-)
- ✅ Test with direct 5V connection
- ✅ Verify BUZZER_PIN (GPIO 15)
- ✅ Check Serial Monitor for buzzer activation logs
- ✅ Test with manual toggle via web interface

### Settings Not Saving

**Problem**: Settings reset after reboot

**Solutions**:
- ✅ Wait for "Settings saved" confirmation message
- ✅ Check preferences.begin() in code
- ✅ Verify flash memory isn't corrupted
- ✅ Try erasing flash and re-uploading code
- ✅ Check Serial Monitor for save errors

---

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

### How to Contribute

1. **Fork the Repository**
   ```bash
   git fork https://github.com/yourusername/esp32-smart-door-control.git
   ```

2. **Create Feature Branch**
   ```bash
   git checkout -b feature/AmazingFeature
   ```

3. **Commit Changes**
   ```bash
   git commit -m "Add: Amazing new feature"
   ```

4. **Push to Branch**
   ```bash
   git push origin feature/AmazingFeature
   ```

5. **Open Pull Request**
   - Describe your changes in detail
   - Include screenshots/videos if applicable
   - Reference any related issues

### Development Guidelines

- 📝 Comment your code clearly
- ✅ Test thoroughly before submitting
- 📚 Update documentation for new features
- 🎨 Follow existing code style
- 🔒 Never commit API keys or credentials

### Bug Reports

Please include:
- ESP32 board version
- Arduino IDE version
- Library versions
- Detailed description of issue
- Serial Monitor output
- Steps to reproduce

---

## 📄 License

This project is licensed under the **MIT License**.

```
MIT License

Copyright (c) 2025 Marie Door Control System

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 👨‍💻 Author

**Marie Door Control System**
- 📧 Email: contact@mariedoor.io
- 🐙 GitHub: [@mariedoor](https://github.com/mariedoor)
- 🌐 Website: [www.mariedoor.io](https://www.mariedoor.io)

---

## 🙏 Acknowledgments

- **Espressif Systems** - ESP32 platform
- **Google** - Gemini AI API
- **Formspree** - Email integration service
- **Arduino Community** - Libraries and support
- **Contributors** - Everyone who helped improve this project

---

## 📊 Project Stats

![GitHub stars](https://img.shields.io/github/stars/yourusername/esp32-smart-door-control?style=social)
![GitHub forks](https://img.shields.io/github/forks/yourusername/esp32-smart-door-control?style=social)
![GitHub watchers](https://img.shields.io/github/watchers/yourusername/esp32-smart-door-control?style=social)
![GitHub last commit](https://img.shields.io/github/last-commit/yourusername/esp32-smart-door-control)
![GitHub issues](https://img.shields.io/github/issues/yourusername/esp32-smart-door-control)
![GitHub pull requests](https://img.shields.io/github/issues-pr/yourusername/esp32-smart-door-control)

---

## 🔗 Quick Links

- [🐛 Report Bug](https://github.com/yourusername/esp32-smart-door-control/issues)
- [✨ Request Feature](https://github.com/yourusername/esp32-smart-door-control/issues)
- [💬 Discussions](https://github.com/yourusername/esp32-smart-door-control/discussions)
- [📖 Wiki](https://github.com/yourusername/esp32-smart-door-control/wiki)
- [📹 Video Tutorial](https://youtube.com/watch?v=...)

---

<div align="center">

### ⭐ Star this project if you find it helpful!

**Made with ❤️ and ESP32**

[Back to Top](#-esp32-smart-door-control-system)

</div>
