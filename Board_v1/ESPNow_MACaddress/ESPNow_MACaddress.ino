#include <WiFi.h>

void setup() {
    Serial.begin(115200);

    // Initialize WiFi in station mode and start it
    WiFi.mode(WIFI_STA);
    WiFi.begin();  // Start WiFi to ensure it's fully initialized

    delay(100);  // Wait briefly to ensure Wi-Fi hardware is ready
    
    // Get the MAC address for the WiFi station
    String macAddress = WiFi.macAddress();
    Serial.print("ESP32 MAC Address: ");
    Serial.println(macAddress);
}

void loop() {
    // Nothing to do here
}
