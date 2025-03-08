/*
 * ESP8266 Sensor Data Logger with ThingSpeak Integration
 *
 * Features:
 * - Reads temperature, humidity, and pressure from a BME280 sensor.
 * - Measures voltage using the ESP8266's ADC.
 * - Connects to Wi-Fi with a fast reconnect method using RTC memory.
 * - Sends sensor data to ThingSpeak for remote monitoring.
 * - Implements power-saving by disabling Wi-Fi when idle.
 * - Uses deep sleep mode to reduce power consumption.
 * - Stores Wi-Fi connection details in RTC memory for quicker reconnection.
 *
 * Strategies for Faster Wi-Fi Connection:
 * - Attempts a "fast connection" using stored RTC memory data (Wi-Fi channel and BSSID).
 * - Configures static IP settings to speed up DHCP negotiation.
 * - Disables persistent Wi-Fi credentials to avoid unnecessary writes to flash memory.
 * - Uses Wi-Fi force sleep/wake to optimize power usage and connection time.
 * - Falls back to a regular connection if the fast connection fails.
 * - Uses 11N Mode

Optimization articles: https://www.bakke.online/index.php/tag/esp8266/

https://www.bakke.online/index.php/2017/06/24/esp8266-wifi-power-reduction-avoiding-network-scan/
https://johnmu.com/2022-esp8266-wifi-speed/
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
//ADC_MODE(ADC_VCC);

#define WLAN_SSID     "SSID"        // Replace with your Wi-Fi SSID
#define WLAN_PASSWD "PASSWORD"    // Replace with your Wi-Fi password
#define THINGSPEAK_API_KEY "API_KEY" // Replace with your ThingSpeak Write API Key
#define THINGSPEAK_CHANNEL_ID 123456    // Replace with your ThingSpeak Channel ID

IPAddress local_ip(192, 168, 1, 12);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

const unsigned long sleepDuration = 600000000;  // 10 minutes in microseconds
const float Altitude = 50.0; // Altitudine della stazione in metri
// V = (ADC/1024)*Vref) V / (R1 / (r1+r2))

//const float adc_conversion =0.001075269;  // 1.1V divided by 1023.0 ADC
//const float voltage_correction = 4.7872; // Voltage divider calculation: V * (177+674)/177

const float x = 0.005169795; // adc_conversion * voltage_correction
//const float adjustment = 0.909090909; // calibration on wrong value 1/1.1 if needed



// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct {
  uint32_t crc32;   // 4 bytes
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t bssid[6]; // 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
} rtcData;


Adafruit_BME280 bme;  // Create an instance of the BME280 sensor

WiFiClient client;

void setup() {

  // Could Be needed to start correctly
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  //delay(1);


  // Initialize serial communication
  Serial.begin(115200);
  
   pinMode(A0, INPUT);
  // Initialize BME280
  if (!bme.begin(0x76)) {
    //Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while(1);
  }

// Try to read WiFi settings from RTC memory
bool rtcValid = false;
if( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
  // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
  uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  if( crc == rtcData.crc32 ) {
    rtcValid = true;
  }
}

  // Read BME280 sensor data
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;  // Convert pressure to hPa
        pressure = bme.seaLevelForAltitude(Altitude, pressure);
  float voltage;
    int analogReadings[5];
      for (int i = 0; i < 5; i++) {
        analogReadings[i] = analogRead(A0);
        delay(5);
        }

    float average = 0;
      for (int i = 0; i < 5; i++) {
        average += analogReadings[i];
        }
      average /= 5;
      Serial.println("");
      Serial.println(average);
      voltage = average * x; //voltage

  /* Print readings to serial monitor
  Serial.print(F("Temperature: "));
  Serial.print(temperature);
  Serial.print(F(" °C, Humidity: "));
  Serial.print(humidity);
  Serial.print(F(" %, Pressure: "));
  Serial.print(pressure);
  Serial.println(F(" hPa"));
*/

  uint32_t startTime = millis();

  WiFi.forceSleepWake();
  delay( 1 );
  WiFi.persistent(false); // Not Use persistent Wi-Fi credentials
// WiFi.persistent(true); // TEST for increased speed?
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);  // Set 802.11n mode
  WiFi.mode(WIFI_STA); // Disable Scanning
//  WiFi.config(local_ip, gateway, subnet, dns);
  

if( rtcValid ) {
  // The RTC data was good, make a quick connection
  Serial.println("");
  Serial.println(F("Start fast connection"));
  WiFi.config(local_ip, gateway, subnet, dns);
  WiFi.begin( WLAN_SSID, WLAN_PASSWD, rtcData.channel, rtcData.bssid, true );
}
else {
  // The RTC data was not valid, so make a regular connection
  Serial.println("");
  Serial.println(F("Start slow connection"));
  WiFi.begin( WLAN_SSID, WLAN_PASSWD );
}

int retries = 0;
int wifiStatus = WiFi.status();

while( wifiStatus != WL_CONNECTED ) {
  retries++;
  if( retries == 400 ) {
    // after 4 seconds Quick connect is not working, reset WiFi and try regular connection
    Serial.println(F("Fast not working, switching to slow connection"));
    WiFi.disconnect();
    delay( 10 );
    WiFi.forceSleepBegin();
    delay( 10 );
    WiFi.forceSleepWake();
    delay( 10 );
    WiFi.begin( WLAN_SSID, WLAN_PASSWD );
  }
  if( retries == 1000 ) {
    // Giving up after 10 seconds and going back to sleep
    WiFi.disconnect( true );
    delay( 1 );
    WiFi.mode( WIFI_OFF );
    Serial.println(F("Wifi not working, go to sleep"));
    ESP.deepSleep( sleepDuration, WAKE_RF_DISABLED );
    return; // Not expecting this to be called, the previous call will never return.
  }
  delay( 10 );
  wifiStatus = WiFi.status();
}
long elapsed_time = millis()-startTime;

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Send data to ThingSpeak
  ThingSpeak.setField(1, temperature);  // Field 1: Temperature
  ThingSpeak.setField(2, humidity);     // Field 2: Humidity
  ThingSpeak.setField(3, pressure);     // Field 3: Pressure
  ThingSpeak.setField(4, voltage);     // Field 4: Pressure
  ThingSpeak.setField(5, elapsed_time);     // Field 5: elapsed time mS
  ThingSpeak.writeFields(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_KEY);
  
  WiFi.disconnect( true ); // Disable WiFi
  delay(1);

  // Write current connection info back to RTC
  rtcData.channel = WiFi.channel();
  memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );
  
  delay(1);
  ESP.deepSleep(sleepDuration, WAKE_RF_DISABLED); // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
}

void loop() {
  // Nothing to do here, all the action happens in the setup()
}



uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while( length-- ) {
    uint8_t c = *data++;
    for( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}