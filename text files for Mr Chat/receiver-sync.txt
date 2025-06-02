#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Include this for esp_wifi_set_max_tx_power
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    uint32_t sync_time;  // Master time in milliseconds
} sync_packet_t;


uint8_t sensor1_mac[] = {0xA0, 0x85, 0xE3, 0xFB, 0x52, 0x70};  // MAC of S3 watch
uint8_t sensor2_mac[] = {0x88, 0x13, 0xBF, 0x25, 0x10, 0xA4};  // MAC of ECG

uint32_t last_sync_time_sent = 0;  // Renamed for clarity: when last sync was *sent*
const uint8_t WIFI_CHANNEL_RECEIVER = 6; // Define channel for consistency

// Configuration for sync packet frequency
const uint32_t INITIAL_SYNC_INTERVAL_MS = 5000;  // Send sync every 5 seconds initially
const uint32_t NORMAL_SYNC_INTERVAL_MS = 120000; // Then send sync every 2 minutes
const int NUM_INITIAL_FAST_SYNCS = 5;       // Send 5 fast sync packets
static int sync_packets_sent_count = 0;     // Counter for fast syncs

// Callback function executed when data is received from sensors
void onDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    char message[len + 1];
    memcpy(message, incomingData, len);
    message[len] = '\0';
    Serial.print(message); // Forward to Python plotter
}

// Function to send a sync packet to both sensors
void sendSyncPacketToSensors() {
    sync_packet_t syncData;
    syncData.sync_time = millis(); // Use current master time

    esp_err_t result1 = esp_now_send(sensor1_mac, (uint8_t *)&syncData, sizeof(syncData));
    esp_err_t result2 = esp_now_send(sensor2_mac, (uint8_t *)&syncData, sizeof(syncData));

    // Optional: Check results
    // if (result1 == ESP_OK) Serial.println("Sync sent to Sensor 1 (Watch)."); else Serial.println("Failed to send sync to Sensor 1.");
    // if (result2 == ESP_OK) Serial.println("Sync sent to Sensor 2 (ECG)."); else Serial.println("Failed to send sync to Sensor 2.");
    
    Serial.print("Sync packet (time: "); Serial.print(syncData.sync_time); Serial.println(") sent to sensors.");
}

// Function to send a reset packet (sync_time = 0) to both sensors
void sendResetPacketToSensors() {
    sync_packet_t syncData;
    syncData.sync_time = 0; // Special value for reset

    esp_err_t result1 = esp_now_send(sensor1_mac, (uint8_t *)&syncData, sizeof(syncData));
    esp_err_t result2 = esp_now_send(sensor2_mac, (uint8_t *)&syncData, sizeof(syncData));

    Serial.println("Reset packet (time: 0) sent to sensors.");
    // Optional: Check results
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Wait for Serial to initialize if connected immediately after boot

    Serial.println("\n--- ESP-NOW Master Receiver & Sync Broadcaster ---");

    WiFi.mode(WIFI_STA);
    delay(500);
    Serial.print("Receiver MAC Address: "); Serial.println(WiFi.macAddress());

    esp_err_t err_set_channel = esp_wifi_set_channel(WIFI_CHANNEL_RECEIVER, WIFI_SECOND_CHAN_NONE);
    if (err_set_channel == ESP_OK) {
        Serial.print("WiFi channel set to: "); Serial.println(WIFI_CHANNEL_RECEIVER);
    } else {
        Serial.print("Error setting WiFi channel: "); Serial.println(esp_err_to_name(err_set_channel));
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW. Halting.");
        while(1) {delay(1000);}
    }
    Serial.println("ESP-NOW Initialized.");
    
    esp_now_register_recv_cb(onDataRecv); // For receiving data FROM sensors

    // Add peers for SENDING sync/reset packets TO sensors
    esp_now_peer_info_t peerInfo;
    
    // Add Sensor 1 (Watch)
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, sensor1_mac, 6);
    peerInfo.channel = WIFI_CHANNEL_RECEIVER; // Use the globally set channel
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer 1 (Watch) added for sync sending.");
    } else {
        Serial.println("Failed to add Peer 1 (Watch).");
    }

    // Add Sensor 2 (ECG)
    memset(&peerInfo, 0, sizeof(peerInfo)); // Clear for next peer
    memcpy(peerInfo.peer_addr, sensor2_mac, 6);
    peerInfo.channel = WIFI_CHANNEL_RECEIVER; // Use the globally set channel
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer 2 (ECG) added for sync sending.");
    } else {
        Serial.println("Failed to add Peer 2 (ECG).");
    }
    
    delay(1000); // Give a moment after adding peers
    Serial.println("Sending initial RESET packet to sensors...");
    sendResetPacketToSensors(); // Send initial reset

    last_sync_time_sent = millis(); // Initialize for the first fast sync
    Serial.println("--- End of Setup ---");
    Serial.println("Waiting for data from sensors and sending sync packets...");
}

void loop() {
    uint32_t now = millis();
    uint32_t current_sync_interval_for_loop = NORMAL_SYNC_INTERVAL_MS;

    if (sync_packets_sent_count < NUM_INITIAL_FAST_SYNCS) {
        current_sync_interval_for_loop = INITIAL_SYNC_INTERVAL_MS;
    }

    if (now - last_sync_time_sent >= current_sync_interval_for_loop) {
        last_sync_time_sent = now;
        sendSyncPacketToSensors(); // This sends millis() as sync_time
        sync_packets_sent_count++;
    }
    // The loop can be used for other non-blocking tasks if needed
    // delay(10); // Small delay to prevent tight loop if no other tasks
}