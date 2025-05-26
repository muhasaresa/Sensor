#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Include this for esp_wifi_set_max_tx_power
// Callback function executed when data is received
void onDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    // Create a buffer to store the incoming message
    char message[len+1];  // Add one for null terminator
    memcpy(message, incomingData, len);
    message[len] = '\0';  // Null-terminate the character array

    // Print the received message
    //Serial.print("Received message: ");
    Serial.print(message);
}

void setup() {
    Serial.begin(115200);

    delay(1000);

    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
      // Select a WiFi channel. For example, choose channel 6.
  // The second parameter sets the secondary channel; here we use WIFI_SECOND_CHAN_NONE.
  uint8_t channel = 6;
  esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if (err == ESP_OK) {
    Serial.print("WiFi channel set to: ");
    Serial.println(channel);
  } else {
    Serial.print("Error setting WiFi channel: ");
    Serial.println(err);
  }

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Register receive callback function
    esp_now_register_recv_cb(onDataRecv);
}

void loop() {
    // The loop is empty as the data is received via callback
}
