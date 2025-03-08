# ESP8266 Sensor Data Logger with ThingSpeak Integration

## Overview
This project is a low-power ESP8266-based sensor data logger that collects temperature, humidity, pressure, and voltage readings, then uploads them to [ThingSpeak](https://thingspeak.com/) for remote monitoring. The system is optimized for low power consumption and fast Wi-Fi reconnection using RTC memory.

## Features
- **Sensor Integration**: Reads environmental data from a **BME280** sensor.
- **Voltage Measurement**: Uses the ESP8266's **ADC** to measure an external voltage.
- **Wi-Fi Optimization**: Implements **fast reconnection** using RTC memory.
- **Cloud Logging**: Sends collected data to **ThingSpeak** for remote access.
- **Power Efficiency**: Uses **deep sleep mode** to extend battery life.
- **RTC Memory Usage**: Stores **Wi-Fi credentials** (channel & BSSID) to speed up reconnections.

## Hardware Requirements
- **ESP8266** (e.g., NodeMCU, Wemos D1 Mini)
- **BME280 Sensor** (Temperature, Humidity, Pressure)
- **Voltage Divider Circuit** (for ADC voltage measurement)
- **Power Source** (e.g., Li-ion battery or USB power)

## Software Requirements
- **Arduino IDE** (or PlatformIO)
- **ESP8266 Board Package** installed in Arduino IDE
- **Required Libraries**:
  - `Wire.h`
  - `Adafruit_Sensor.h`
  - `Adafruit_BME280.h`
  - `ESP8266WiFi.h`
  - `ThingSpeak.h`

## Installation & Setup
1. **Install Arduino IDE** and add the **ESP8266 board package**.
2. **Install Required Libraries** from the Arduino Library Manager.
3. **Update Configuration in `#define` Directives**:
   - Replace `SSID` and `PASSWORD` with your Wi-Fi credentials.
   - Replace `API_KEY` with your **ThingSpeak Write API Key**.
   - Replace `CHANNEL_ID` with your **ThingSpeak Channel ID**.
4. **Upload the Code** to your ESP8266 board.

## How It Works
1. **Initialization**:
   - The ESP8266 initializes the **BME280 sensor** and reads stored **Wi-Fi credentials** from RTC memory.
2. **Wi-Fi Connection**:
   - If RTC data is valid, the ESP8266 attempts a **fast reconnection** using saved **BSSID & channel**.
   - If the fast connection fails, it retries with a regular **Wi-Fi scan**.
3. **Data Collection**:
   - Reads **temperature, humidity, and pressure** from the BME280 sensor.
   - Measures **voltage** using the ESP8266's **ADC pin (A0)**.
4. **Data Transmission**:
   - Sends collected sensor values to **ThingSpeak**.
5. **Power Management**:
   - Disconnects from Wi-Fi and enters **deep sleep mode** for 10 minutes.
   - Wakes up automatically and repeats the process.

## Wi-Fi Optimization Strategies
- **RTC Memory**: Saves the last used **Wi-Fi channel & BSSID** to avoid scanning.
- **Static IP Configuration**: Reduces DHCP negotiation time.
- **Wi-Fi Force Sleep/Wake**: Optimizes power usage and connection speed.
- **Persistent Wi-Fi Credentials Disabled**: Avoids unnecessary writes to flash memory.
- **Adaptive Retry Mechanism**: Switches between fast and regular connection attempts.

## Data Fields Sent to ThingSpeak
| Field | Description |
|-------|-------------|
| Field 1 | Temperature (°C) |
| Field 2 | Humidity (%) |
| Field 3 | Pressure (hPa) |
| Field 4 | Measured Voltage (V) |
| Field 5 | Wi-Fi Connection Time (ms) |

## Power Consumption
- **Deep Sleep Mode**: Reduces power usage significantly.
- **Wi-Fi Off When Idle**: Minimizes energy consumption between data transmissions.
- **Optimized Wi-Fi Connection**: Reduces active Wi-Fi time.

## Future Improvements
- Add support for **additional sensors** (e.g., light, air quality, etc.).
- Implement **MQTT** for real-time data streaming.
- Improve **error handling** and retry mechanisms.

## License
This project is licensed under the **MIT License**.

---
### Author
Developed by Riccardo Montaguti. Feel free to contribute and improve the project!

