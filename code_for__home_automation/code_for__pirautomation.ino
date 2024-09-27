#ifdef ENABLE_DEBUG
       #define DEBUG_ESP_PORT Serial
       #define NODEBUG_WEBSOCKETS
       #define NDEBUG
#endif 

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "SinricPro.h"
#include "SinricProSwitch.h"
#include <map>

#define WIFI_SSID "Airtel_Home"    
#define WIFI_PASS "ksskoushik"
#define APP_KEY "6d54fb13-a6ab-4726-8149-b81f021a8c7f"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET "17db0152-41ed-470e-8a92-145ff910960c-e6604046-f93e-4153-ae43-6cf153760203"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"

// Enter the device IDs here
#define device_ID_1 "6637296afb874c7486cddcb5"
#define device_ID_2 "66372a28fb874c7486cddd6c"
#define device_ID_3 "SWITCH_ID_NO_3_HERE"
#define device_ID_4 "SWITCH_ID_NO_4_HERE"

// Define the GPIO connected with Relays and switches
#define RelayPin1 2  //D4
#define RelayPin2 0  //D3
#define RelayPin3 14 //D5
#define RelayPin4 12 //D6

#define SwitchPin1 10  //SD3
#define SwitchPin2 4  //D2
#define SwitchPin3 13  //D7
#define SwitchPin4 3   //RX

#define wifiLed 16  //D0

#define PIR_PIN 5  // GPIO5 (D1 on some boards)

// Uncomment the following line if you use tactile buttons instead of toggle switches
// #define TACTILE_BUTTON 1

#define BAUD_RATE 9600
#define DEBOUNCE_TIME 250

typedef struct {  // Struct for the std::map below
  int relayPIN;
  int flipSwitchPIN;
} deviceConfig_t;

// Main configuration
std::map<String, deviceConfig_t> devices = {
    // {deviceId, {relayPIN,  flipSwitchPIN}}
    {device_ID_1, { RelayPin1, SwitchPin1 }},
    {device_ID_2, { RelayPin2, SwitchPin2 }},
    {device_ID_3, { RelayPin3, SwitchPin3 }},
    {device_ID_4, { RelayPin4, SwitchPin4 }}     
};

typedef struct {  // Struct for the std::map below
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
} flipSwitchConfig_t;

std::map<int, flipSwitchConfig_t> flipSwitches;  // This map is used to map flipSwitch PINs to deviceId and handle debounce and last flipSwitch state checks

void setupRelays() { 
  for (auto &device : devices) {  // For each device (relay, flipSwitch combination)
    int relayPIN = device.second.relayPIN;  // Get the relay pin
    pinMode(relayPIN, OUTPUT);  // Set relay pin to OUTPUT
    digitalWrite(relayPIN, HIGH);  // Initialize relay as OFF (HIGH)
  }
}

void setupFlipSwitches() {
  for (auto &device : devices) {  // For each device (relay / flipSwitch combination)
    flipSwitchConfig_t flipSwitchConfig;  // Create a new flipSwitch configuration

    flipSwitchConfig.deviceId = device.first;  // Set the deviceId
    flipSwitchConfig.lastFlipSwitchChange = 0;  // Set debounce time
    flipSwitchConfig.lastFlipSwitchState = true;  // Set lastFlipSwitchState to true (HIGH)

    int flipSwitchPIN = device.second.flipSwitchPIN;  // Get the flipSwitchPIN

    flipSwitches[flipSwitchPIN] = flipSwitchConfig;  // Save the flipSwitch config to flipSwitches map
    pinMode(flipSwitchPIN, INPUT_PULLUP);  // Set the flipSwitch pin to INPUT_PULLUP
  }
}

bool onPowerState(String deviceId, bool &state) {
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
  int relayPIN = devices[deviceId].relayPIN;  // Get the relay pin for corresponding device
  digitalWrite(relayPIN, !state);  // Set the new relay state
  return true;
}

void handleFlipSwitches() {
  unsigned long actualMillis = millis();  // Get actual millis
  for (auto &flipSwitch : flipSwitches) {  // For each flipSwitch in flipSwitches map
    unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;  // Get the timestamp when flipSwitch was pressed last time (used to debounce / limit events)

    if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) {  // If time is > debounce time...
      int flipSwitchPIN = flipSwitch.first;  // Get the flipSwitch pin from configuration
      bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;  // Get the lastFlipSwitchState
      bool flipSwitchState = digitalRead(flipSwitchPIN);  // Read the current flipSwitch state
      if (flipSwitchState != lastFlipSwitchState) {  // If the flipSwitchState has changed...
#ifdef TACTILE_BUTTON
        if (flipSwitchState) {  // If the tactile button is pressed 
#endif      
          flipSwitch.second.lastFlipSwitchChange = actualMillis;  // Update lastFlipSwitchChange time
          String deviceId = flipSwitch.second.deviceId;  // Get the deviceId from config
          int relayPIN = devices[deviceId].relayPIN;  // Get the relayPIN from config
          bool newRelayState = !digitalRead(relayPIN);  // Set the new relay state
          digitalWrite(relayPIN, newRelayState);  // Set the relay to the new state

          SinricProSwitch &mySwitch = SinricPro[deviceId];  // Get Switch device from SinricPro
          mySwitch.sendPowerStateEvent(!newRelayState);  // Send the event
#ifdef TACTILE_BUTTON
        }
#endif      
        flipSwitch.second.lastFlipSwitchState = flipSwitchState;  // Update lastFlipSwitchState
      }
    }
  }
}

// Function to handle PIR sensor input
void handlePIRSensor() {
  static bool lastMotionState = false;
  int pirState = digitalRead(PIR_PIN);  // Read PIR sensor state

  if (pirState == HIGH) {
    Serial.println("Motion detected!");
    // Turn on both relays when motion is detected
    digitalWrite(RelayPin1, LOW);  // Turn on relay connected to RelayPin1
    digitalWrite(RelayPin2, LOW);  // Turn on relay connected to RelayPin2
    lastMotionState = true;
  } else {
    Serial.println("No motion detected.");
    // Turn off both relays if motion has stopped
    if (lastMotionState) {
      digitalWrite(RelayPin1, HIGH);  // Turn off relay connected to RelayPin1
      digitalWrite(RelayPin2, HIGH);  // Turn off relay connected to RelayPin2
      lastMotionState = false;
    }
  }
}

void setupWiFi() {
  Serial.printf("\r\n[WiFi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  digitalWrite(wifiLed, LOW);
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %s\r\n", WiFi.localIP().toString().c_str());
}

void setupSinricPro() {
  for (auto &device : devices) {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
  }

  SinricPro.begin(APP_KEY, APP_SECRET);
  SinricPro.restoreDeviceStates(true);
}

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, HIGH);

  setupRelays();
  setupFlipSwitches();
  setupWiFi();
  setupSinricPro();

  pinMode(PIR_PIN, INPUT);  // Initialize PIR sensor pin as input
}

void loop() {
  SinricPro.handle();
  handleFlipSwitches();
  handlePIRSensor();  // Call the PIR sensor handling function
}
