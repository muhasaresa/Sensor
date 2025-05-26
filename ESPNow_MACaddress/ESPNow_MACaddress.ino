#include <WiFi.h>

void setup() {
    Serial.begin(9600);

    delay(1000);

        while (!Serial) {              // wait (max 3 s) for the PC to open the port
        if (millis() > 3000) break;
    }

    // Initialize WiFi in station mode and start it
    WiFi.mode(WIFI_STA);
    WiFi.begin();  // Start WiFi to ensure it's fully initialized

    delay(1000);  // Wait briefly to ensure Wi-Fi hardware is ready
    
    // Get the MAC address for the WiFi station
    String macAddress = WiFi.macAddress();
    Serial.print("ESP32 MAC Address: ");
    Serial.println(macAddress);
}

void loop() {
    // Get the MAC address for the WiFi station
    //String macAddress = WiFi.macAddress();
    //Serial.print("ESP32 MAC Address: ");
    //delay(1000);
    //Serial.println(macAddress);
}
