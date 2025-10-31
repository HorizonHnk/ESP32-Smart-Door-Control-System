# ESP32 Smart Door Control System

An advanced IoT solution combining hardware automation with intelligent monitoring, real-time alerts, and AI-powered assistance.

## Features

- **Automated Door Control** - Servo motor with configurable open/close angles and timing
- **Environmental Monitoring** - DHT22 sensor for temperature and humidity tracking
- **Obstacle Detection** - HC-SR04 ultrasonic sensor with automatic lighting control
- **IR Remote Control** - Control door (OK), LED (*), and fan (#) with remote
- **Emergency Exit Button** - Physical button for immediate door access
- **Smart Buzzer System** - Audio alerts for door open, temp/humidity alerts
- **LED & Fan Control** - Toggle LED and PWM-controlled DC fan motor
- **Web Dashboard** - Real-time monitoring with live sensor data
- **AI Assistant** - Google Gemini-powered chatbot for natural language queries
- **Alert System** - Automatic ventilation and email alerts
- **Data Logging** - 100-entry circular buffer with CSV export
- **20x4 I2C LCD Display** - Real-time status display with countdown timer

## Hardware Requirements

### Components
- ESP32-WROOM-32 Development Board
- HC-SR04 Ultrasonic Distance Sensor
- IR Receiver Module
- Servo Motor (SG90 or similar)
- DHT22 Temperature & Humidity Sensor
- 20x4 I2C LCD Display (I2C Address: 0x27)
- Active Buzzer Module
- Relay Module (5V)
- Push Button (Emergency Exit)
- Toggle Buttons (2x)
- DC Fan Motor
- LEDs and appropriate resistors
- Breadboard and jumper wires

### Pin Configuration

| Component | GPIO Pin | Notes |
|-----------|----------|-------|
| Ultrasonic Trigger | 19 | OUTPUT |
| Ultrasonic Echo | 18 | INPUT |
| IR Receiver | 14 | INPUT |
| Servo Motor | 26 | PWM OUTPUT |
| Emergency Button | 34 | INPUT_PULLUP |
| Buzzer | 15 | OUTPUT |
| Relay | 27 | OUTPUT |
| Status LED | 5 | OUTPUT |
| DHT22 Sensor | 25 | INPUT/OUTPUT |
| Toggle LED | 13 | OUTPUT |
| Toggle Button | 35 | INPUT_PULLUP |
| Fan Motor | 32 | PWM OUTPUT |
| Fan Button | 23 | INPUT_PULLUP |
| LCD SDA | 21 | I2C (Default) |
| LCD SCL | 22 | I2C (Default) |

## Software Setup

### Prerequisites

1. **Arduino IDE** (version 1.8.x or newer) or **PlatformIO**
2. **ESP32 Board Support** installed in Arduino IDE
3. Required Libraries:
   - ESP32Servo
   - IRremote (version 3.x or newer)
   - DHT sensor library
   - ESPAsyncWebServer
   - AsyncTCP
   - LiquidCrystal_I2C or ESP32_LiquidCrystal_I2C

### Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/HorizonHnk/ESP32-Smart-Door-Control-System.git
   cd ESP32-Smart-Door-Control-System
   ```

2. **Configure API Keys**

   Copy the example secrets file and configure your API keys:
   ```bash
   cp secrets_example.h secrets.h
   ```

   Edit `secrets.h` and add your credentials:
   ```cpp
   #define WIFI_SSID "Your_WiFi_Name"
   #define WIFI_PASSWORD "Your_WiFi_Password"
   #define GEMINI_API_KEY_DEFAULT "Your_Gemini_API_Key"
   #define FORMSPREE_ENDPOINT_DEFAULT "Your_Formspree_Endpoint"
   ```

3. **Get API Keys**

   - **Gemini API Key**: Get your free key from [Google AI Studio](https://makersuite.google.com/app/apikey)
   - **Formspree Endpoint**: Create a free form at [Formspree.io](https://formspree.io/)

4. **Install Libraries**

   In Arduino IDE:
   - Go to **Sketch** → **Include Library** → **Manage Libraries**
   - Search and install each required library listed above

5. **Upload to ESP32**

   - Open `web_integrated_door_control.ino` in Arduino IDE
   - Select your ESP32 board: **Tools** → **Board** → **ESP32 Arduino** → **ESP32 Dev Module**
   - Select the correct COM port: **Tools** → **Port**
   - Click **Upload**

## Usage

### Web Dashboard

1. After uploading, open the Serial Monitor (115200 baud) to see the ESP32's IP address
2. Connect to the same WiFi network as the ESP32
3. Open a web browser and navigate to the IP address shown in Serial Monitor
4. Access the dashboard with the following tabs:
   - **Dashboard** - View real-time sensor data
   - **Controls** - Manually control door, LED, fan, and buzzer
   - **Settings** - Configure thresholds and API keys
   - **AI Chat** - Interact with the AI assistant
   - **About** - System information
   - **Contact** - Send messages via email integration

### IR Remote Control

- **OK Button** - Unlock/open door for 10 seconds
- **\* Button** - Toggle LED on/off
- **# Button** - Toggle fan on/off

### AI Chatbot Commands

Use natural language or commands in the AI Chat:
- `/open` - Open the door
- `/close` - Close the door
- `/led on` - Turn LED on
- `/led off` - Turn LED off
- `/fan on` - Activate fan
- `/fan off` - Deactivate fan
- `/buzzer on` - Activate buzzer
- `/buzzer off` - Deactivate buzzer

### Voice Control

Click the microphone button (🎤) in the AI Chat tab to use voice commands.

## Configuration

### Adjustable Settings (via Web Interface)

- **Temperature Threshold** - Trigger point for automatic ventilation (default: 30°C)
- **Humidity Threshold** - Trigger point for automatic ventilation (default: 70%)
- **Distance Threshold** - Obstacle detection distance (default: 10 cm)
- **Servo Closed Angle** - Door closed position (default: 0°)
- **Servo Open Angle** - Door open position (default: 90°)
- **Door Open Duration** - Auto-close timer duration (default: 10 seconds)

## Safety Features

- **Emergency Exit Button** - Immediately opens door regardless of system state
- **Automatic Ventilation** - Opens door when temperature or humidity exceeds thresholds
- **Email Alerts** - Sends notifications for critical conditions (with 5-minute cooldown)
- **Buzzer Alerts** - Different beep patterns for different alert types
- **Auto-close Timer** - Prevents door from staying open indefinitely

## Data Export

Download sensor data logs (last 100 entries) in CSV format from the Dashboard tab.

## Troubleshooting

### ESP32 Won't Connect to WiFi
- Verify WiFi credentials in `secrets.h`
- Ensure your WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
- Check if WiFi is within range

### LCD Not Displaying
- Verify I2C address (default: 0x27) - use I2C scanner if needed
- Check SDA (GPIO 21) and SCL (GPIO 22) connections
- Ensure proper power supply to LCD

### IR Remote Not Working
- Verify IR receiver is connected to GPIO 14
- Check if IR codes match your remote (codes in sketch: OK=0x1C, *=0x16, #=0x0D)
- Upload example IR receive sketch to test and find your remote's codes

### AI Chat Not Responding
- Verify Gemini API key is valid
- Check internet connection
- Open browser console (F12) to see error messages

## Security Notes

- **Never commit `secrets.h`** to version control (it's in `.gitignore`)
- The `secrets_example.h` file is safe to commit as it contains no real credentials
- API keys can be updated through the web interface Settings tab
- Consider changing default WiFi credentials for production use

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is open source and available under the MIT License.

## Author

Marie Door Control System Team

## Version History

- **v4.2** - WiFi Web Server + AI Integration
- **v3.2** - Simplified IR Remote Controls
- **v3.0** - Added Temperature/Humidity Alerts
- **v2.0** - Web Dashboard Implementation
- **v1.0** - Initial Release

## Support

For issues, questions, or contributions, please visit the [GitHub repository](https://github.com/HorizonHnk/ESP32-Smart-Door-Control-System).
